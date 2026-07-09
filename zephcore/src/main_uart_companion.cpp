/*
 * SPDX-License-Identifier: MIT
 * ZephCore - UART Companion (C5 detection relay, Event-Driven)
 *
 * Phase 2 bring-up skeleton for the carrier-v3 board (XIAO MG24 +
 * Wio-SX1262, paired over UART with an ESP32-C5 Remote-ID scanner).
 * Headless, no BLE — modeled closely on src/main_repeater.cpp (same
 * boot sequence, USB/console serial CLI, event-driven mesh loop), with
 * RepeaterMesh swapped for UartCompanionMesh and a second UART added for
 * the C5 detection-frame link (`zephcore,c5-uart` chosen node — see the
 * paired boards/common/uart_companion.overlay).
 *
 * SCOPE: this file builds and boots. The C5 frame parse/send path is
 * stubbed — see process_c5_uart() below.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_uart_companion_main, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/reboot.h>
#include "oled_power.h"

/* BLE controller assert handler — some platform confs (e.g. mg24_common.conf)
 * enable CONFIG_BT_CTLR_ASSERT_HANDLER regardless of role; harmless no-op
 * when CONFIG_BT=n (this role's uart_companion.conf disables BT). Mirrors
 * main_repeater.cpp. */
#if IS_ENABLED(CONFIG_BT_CTLR_ASSERT_HANDLER)
extern "C" void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	LOG_ERR("!!! BLE CONTROLLER ASSERT: %s:%u !!!", file ? file : "?", line);
	k_sleep(K_MSEC(100));
	sys_reboot(SYS_REBOOT_COLD);
}
#endif

/* USB CDC ACM init + 1200-baud DFU + DTR callbacks (shared with companion/
 * repeater). Gate on the CDC-ACM class driver, not DT node presence alone —
 * see main_repeater.cpp's identical comment. On boards with no native USB
 * (e.g. XIAO MG24), this whole block compiles out and the CLI below falls
 * back to the chosen console UART. */
#define ZEPHCORE_USB_STACK \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))

#if ZEPHCORE_USB_STACK && !IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT)
#include <ZephyrUSBCDC.h>
#endif

#include <app/UartCompanionMesh.h>
#include <ZephyrDataStore.h>
#include <adapters/clock/ZephyrRTCClock.h>
#include <adapters/clock/ZephyrRTCDiscover.h>
#include <helpers/CommonCLI.h>
#include <helpers/ClientACL.h>

/* UI subsystem (display, buttons, buzzer) — headless here; see the
 * ui_headless_stubs.c pulled in by the CMake UART_COMPANION branch. */
#include "ui_task.h"

/* Radio + mesh includes (shared header selects LR1110 or SX126x) */
#include <mesh/RadioIncludes.h>

/* LED configuration */
#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

/* ========================================================================
 * Drone-base contact — pinned placeholder pubkey.
 *
 * Matches MESHCORE_BASE_KEY in nodes/nrf52_meshcore_node/platformio_local.ini
 * (the Arduino carrier firmware's build-time-pinned drone-base contact).
 * Hardcoded here for the Phase 2 skeleton; a real config path (CLI/NVS) can
 * replace this later without touching the mesh plumbing.
 * ======================================================================== */
static const uint8_t drone_base_pubkey[PUB_KEY_SIZE] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* USB CLI configuration */
#define USB_RING_BUF_SIZE 512
#define CLI_LINE_BUF_SIZE 256

/* C5 detection-frame UART configuration */
#define C5_RING_BUF_SIZE 256

/*
 * Event-driven mesh loop - mirrors main_repeater.cpp, plus a C5-UART-RX bit.
 */
#define MESH_EVENT_LORA_RX       BIT(0)  /* LoRa packet received */
#define MESH_EVENT_LORA_TX_DONE  BIT(1)  /* LoRa TX complete */
#define MESH_EVENT_CLI_RX        BIT(2)  /* CLI command received */
#define MESH_EVENT_HOUSEKEEPING  BIT(3)  /* Periodic housekeeping */
#define MESH_EVENT_TX_DRAIN      BIT(4)  /* Outbound packet delay expired, run checkSend */
#define MESH_EVENT_INIT_ADVERT   BIT(5)  /* Deferred boot advert — send on main thread */
#define MESH_EVENT_C5_RX         BIT(6)  /* C5 UART bytes ready to drain */
#define MESH_EVENT_ALL           (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE | \
	MESH_EVENT_CLI_RX | MESH_EVENT_HOUSEKEEPING | MESH_EVENT_TX_DRAIN | \
	MESH_EVENT_INIT_ADVERT | MESH_EVENT_C5_RX)

#define HOUSEKEEPING_INTERVAL_MS CONFIG_ZEPHCORE_HOUSEKEEPING_INTERVAL_MS

static struct k_event mesh_events;

/* NOTE: main_repeater.cpp/main_companion.cpp also carry a deferred
 * hardware-RTC-write path (MESH_EVENT_RTC_SAVE / request_rtc_save()) because
 * their GPS fix callback runs off-main (GNSS modem_chat worker thread) and
 * the RTC's blocking I2C write can't happen there. This role has no GPS in
 * the Phase 2 skeleton, so that scaffolding is omitted — add it back
 * alongside a real time source if one is wired in later. */

/* USB/console CDC state (bench diagnostics CLI) */
static const struct device *usb_dev;
static uint8_t usb_ring_buf_data[USB_RING_BUF_SIZE];
static struct ring_buf usb_ring_buf;
static char cli_line_buf[CLI_LINE_BUF_SIZE];
static char cli_reply_buf[256];
static uint16_t cli_line_idx;

struct cli_cmd_line { char buf[CLI_LINE_BUF_SIZE]; };
K_MSGQ_DEFINE(cli_cmd_queue, sizeof(struct cli_cmd_line), 4, 4);

/* C5 detection-frame UART state */
static const struct device *c5_uart_dev;
static uint8_t c5_ring_buf_data[C5_RING_BUF_SIZE];
static struct ring_buf c5_ring_buf;

/* Work items */
static void cli_rx_work_fn(struct k_work *work);
static void housekeeping_timer_fn(struct k_timer *timer);
static void tx_drain_work_fn(struct k_work *work);
static void initial_advert_work_fn(struct k_work *work);
K_WORK_DEFINE(cli_rx_work, cli_rx_work_fn);
K_WORK_DELAYABLE_DEFINE(tx_drain_work, tx_drain_work_fn);
K_WORK_DELAYABLE_DEFINE(initial_advert_work, initial_advert_work_fn);

K_TIMER_DEFINE(housekeeping_timer, housekeeping_timer_fn, NULL);

#ifdef ZEPHCORE_LORA
static UartCompanionMesh *uart_companion_mesh_ptr;
#endif

/* Print string to USB/console serial */
static void cli_print(const char *str)
{
	if (!usb_dev) return;
	while (*str) {
		uart_poll_out(usb_dev, *str++);
	}
}

/* USB/console CDC UART interrupt callback (bench CLI) */
static void cli_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
			if (recv_len > 0) {
				ring_buf_put(&usb_ring_buf, buf, recv_len);
				k_work_submit(&cli_rx_work);
			}
		}
	}
}

/* CLI RX work - processes line-based CLI commands (mirrors main_repeater.cpp) */
static void cli_rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t byte;

	while (ring_buf_get(&usb_ring_buf, &byte, 1) == 1) {
		if (byte == '\r' || byte == '\n') {
			if (cli_line_idx > 0) {
				cli_line_buf[cli_line_idx] = '\0';
				LOG_INF("CLI cmd len=%d: %.40s%s", cli_line_idx,
					cli_line_buf, cli_line_idx > 40 ? "..." : "");

				struct cli_cmd_line c;
				strncpy(c.buf, cli_line_buf, sizeof(c.buf) - 1);
				c.buf[sizeof(c.buf) - 1] = '\0';
				if (k_msgq_put(&cli_cmd_queue, &c, K_NO_WAIT) == 0) {
					k_event_post(&mesh_events, MESH_EVENT_CLI_RX);
				} else {
					cli_print("\r\n  -> busy\r\n");
				}
				cli_line_idx = 0;
			} else {
				cli_print("\r\n");
			}
		} else if (byte == 0x7F || byte == 0x08) {
			if (cli_line_idx > 0) {
				cli_line_idx--;
				if (usb_dev) {
					uart_poll_out(usb_dev, '\b');
					uart_poll_out(usb_dev, ' ');
					uart_poll_out(usb_dev, '\b');
				}
			}
		} else if (cli_line_idx < sizeof(cli_line_buf) - 1) {
			if (usb_dev) {
				uart_poll_out(usb_dev, byte);
			}
			cli_line_buf[cli_line_idx++] = (char)byte;
		}
	}
}

/* C5 UART interrupt callback — just fills the ring buffer and wakes main. */
static void c5_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[32];
			int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
			if (recv_len > 0) {
				ring_buf_put(&c5_ring_buf, buf, recv_len);
				k_event_post(&mesh_events, MESH_EVENT_C5_RX);
			}
		}
	}
}

/* Drain bytes from the C5 scanner UART and log them.
 *
 * TODO (Phase 2): parse CMD_RID_DETECTION / CMD_RID_FORMATTED_MSG frames
 * (see lib/meshcore_shared and the UART protocol used by
 * nodes/remote_meshcore_node) out of this byte stream, build the
 * Identity/Telemetry TXT_MSG text, and call
 * uart_companion_mesh_ptr->sendMessage(drone_base_contact, ...) — see the
 * TODO on UartCompanionMesh::sendDetectionToBase() in UartCompanionMesh.h.
 * For now this just proves the UART link is alive end to end (bench-day:
 * confirm bytes typed at the C5 side show up in this log). */
static void process_c5_uart(void)
{
	uint8_t chunk[C5_RING_BUF_SIZE];
	size_t n = 0;

	uint8_t byte;
	while (n < sizeof(chunk) && ring_buf_get(&c5_ring_buf, &byte, 1) == 1) {
		chunk[n++] = byte;
	}
	if (n > 0) {
		LOG_DBG("C5 UART: %u byte(s) (unparsed — TODO frame decode)", (unsigned)n);
	}
}

#ifdef ZEPHCORE_LORA
/* Run queued CLI commands on the MAIN thread (mirrors main_repeater.cpp). */
static void process_cli_commands(CommonCLI *cli)
{
	struct cli_cmd_line c;
	while (cli && k_msgq_get(&cli_cmd_queue, &c, K_NO_WAIT) == 0) {
		cli_reply_buf[0] = '\0';
		cli->handleCommand(0, c.buf, cli_reply_buf);
		if (cli_reply_buf[0] != '\0') {
			cli_print("\r\n  -> ");
			cli_print(cli_reply_buf);
		}
		cli_print("\r\n");
	}
}
#endif

static void housekeeping_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_event_post(&mesh_events, MESH_EVENT_HOUSEKEEPING);
}

#ifdef ZEPHCORE_LORA
static void lora_rx_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_RX);
}

static void lora_tx_done_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	k_event_post(&mesh_events, MESH_EVENT_LORA_TX_DONE);
}

static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_event_post(&mesh_events, MESH_EVENT_TX_DRAIN);
}

static void initial_advert_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	k_event_post(&mesh_events, MESH_EVENT_INIT_ADVERT);
}

static void tx_queued_callback(uint32_t delay_ms, void *user_data)
{
	ARG_UNUSED(user_data);
	k_work_reschedule(&tx_drain_work, K_MSEC(delay_ms));
}
#endif

static mesh::ZephyrRTCClock rtc_clock;
static ZephyrDataStore data_store(rtc_clock);

#ifdef ZEPHCORE_LORA
static mesh::ZephyrBoard zephyr_board;

static uint16_t get_battery_mv(void)
{
	return zephyr_board.getBattMilliVolts();
}

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110)
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::LR1110Radio lora_radio(lora_dev, zephyr_board);
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_SX127X)
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX127xRadio lora_radio(lora_dev, zephyr_board);
#else
static const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static mesh::SX126xRadio lora_radio(lora_dev, zephyr_board);
#endif

static mesh::ZephyrMillisecondClock ms_clock;
static mesh::ZephyrRNG zephyr_rng;
static mesh::SimpleMeshTables mesh_tables;
static mesh::StaticPoolPacketManager packet_mgr;

/* UartCompanionMesh requires: radio, ms_clock, rng, rtc, packet mgr, tables
 * (same shape as CompanionMesh's constructor in main_companion.cpp, minus
 * the ZephyrDataStore reference — this role doesn't sync contacts/channels
 * to flash). */
static UartCompanionMesh uart_companion_mesh(lora_radio, ms_clock, zephyr_rng, rtc_clock,
	packet_mgr, mesh_tables);

/* Add the pinned drone-base contact so a future sendDetectionToBase() has
 * somewhere to send to. out_path_len = OUT_PATH_UNKNOWN until we either
 * hear an advert from it or resolve a path some other way — the mesh
 * dispatcher floods first-tries automatically in that state. */
static void add_drone_base_contact(void)
{
	/* Value-init (not memset — ContactInfo has a non-trivial mesh::Identity
	 * member, so a raw memset triggers -Wclass-memaccess) then fill in the
	 * fields that matter. name[]/out_path[] zero-init is all we need beyond
	 * the explicit assignments below. */
	ContactInfo drone_base{};
	drone_base.id = mesh::Identity(drone_base_pubkey);
	strncpy(drone_base.name, "drone-base", sizeof(drone_base.name) - 1);
	drone_base.type = ADV_TYPE_CHAT;
	drone_base.out_path_len = OUT_PATH_UNKNOWN;
	drone_base.shared_secret_valid = false;
	if (!uart_companion_mesh.addContact(drone_base)) {
		LOG_ERR("add_drone_base_contact: contact table full?!");
	} else {
		LOG_INF("add_drone_base_contact: pinned drone-base contact added");
	}
}
#endif /* ZEPHCORE_LORA */

/* ========== Bench-diagnostics CLI (USB/console serial) ==========
 * Standalone CommonCLI instance (mirrors main_companion.cpp's companion_cli,
 * not RepeaterMesh's embedded-CLI pattern) — keeps UartCompanionMesh a slim
 * BaseChatMesh subclass with no CommonCLICallbacks baggage. */
#ifdef ZEPHCORE_LORA
class UartCompanionCLICallbacks : public CommonCLICallbacks {
public:
	void savePrefs() override {
		data_store.savePrefs(uart_companion_mesh.prefs);
	}
	const char *getFirmwareVer() override { return FIRMWARE_VERSION; }
	const char *getBuildDate() override { return FIRMWARE_BUILD_DATE; }
	const char *getRole() override { return "uart_companion"; }
	bool formatFileSystem() override { return data_store.formatFileSystem(); }

	/* Advert / timer controls — this role doesn't run periodic adverts
	 * (the drone-base contact is pinned, not discovered); stub for now. */
	void sendSelfAdvertisement(int delay_millis, bool flood) override {
		(void)delay_millis; (void)flood;
	}
	void updateAdvertTimer() override {}
	void updateFloodAdvertTimer() override {}

	void setLoggingOn(bool enable) override { (void)enable; }
	void eraseLogFile() override {}
	void dumpLogFile() override {}

	void setTxPower(int8_t power_dbm) override {
		LOG_INF("TX power %d dBm requested (reboot to apply)", power_dbm);
	}

	bool setRxBoostedGain(bool enable) override {
		return lora_radio.setRxBoost(enable);
	}

	mesh::LocalIdentity &getSelfId() override { return uart_companion_mesh.self_id; }

	void saveIdentity(const mesh::LocalIdentity &new_id) override {
		uart_companion_mesh.self_id = new_id;
		data_store.saveMainIdentity(new_id);
	}

	void clearStats() override {
		lora_radio.resetStats();
		uart_companion_mesh.resetStats();
	}

	void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr,
				  int timeout_mins) override {
		(void)freq; (void)bw; (void)sf; (void)cr; (void)timeout_mins;
	}
};

static UartCompanionCLICallbacks uart_companion_cli_cbs;
static ClientACL uart_companion_acl;  /* unused by CommonCLI but required by constructor */
static CommonCLI uart_companion_cli(zephyr_board, rtc_clock, uart_companion_acl,
	&uart_companion_mesh.prefs, &uart_companion_cli_cbs);
#endif /* ZEPHCORE_LORA */

/* UART Companion event loop (mirrors main_repeater.cpp's repeater_event_loop) */
static void uart_companion_event_loop(void)
{
	LOG_INF("starting event-driven loop");

	cli_print("\r\n=== ZephCore UART Companion ===\r\n");

	k_timer_start(&housekeeping_timer, K_MSEC(HOUSEKEEPING_INTERVAL_MS),
		      K_MSEC(HOUSEKEEPING_INTERVAL_MS));

	for (;;) {
		uint32_t events = k_event_wait(&mesh_events, MESH_EVENT_ALL, false, K_FOREVER);
		k_event_clear(&mesh_events, events);

#ifdef ZEPHCORE_LORA
		if (events & MESH_EVENT_CLI_RX) {
			process_cli_commands(&uart_companion_cli);
		}

		if (uart_companion_mesh_ptr && (events & MESH_EVENT_INIT_ADVERT)) {
			LOG_INF("Sending deferred initial advertisement (flood)");
			/* Flood advert — bootstraps pairing with the drone-base
			 * (see UartCompanionMesh::sendSelfAdvert doc comment). */
			uart_companion_mesh_ptr->sendSelfAdvert(true);
		}

		if (uart_companion_mesh_ptr &&
		    (events & (MESH_EVENT_LORA_RX | MESH_EVENT_LORA_TX_DONE |
			       MESH_EVENT_CLI_RX | MESH_EVENT_TX_DRAIN))) {
			uart_companion_mesh_ptr->loop();
		}
#endif

		/* C5 detection-frame link — drain + (eventually) parse/forward. */
		if (events & MESH_EVENT_C5_RX) {
			process_c5_uart();
		}

		if (events & MESH_EVENT_HOUSEKEEPING) {
#ifdef ZEPHCORE_LORA
			if (uart_companion_mesh_ptr) {
				uart_companion_mesh_ptr->maintenanceLoop();
				uart_companion_mesh_ptr->loop();
			}
#endif
			ui_set_clock(rtc_clock.getCurrentTime());
		}
	}
}

int main(void)
{
#ifdef ZEPHCORE_LORA
	zephyr_board.clearBootloaderMagic();
#endif

#if ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart) && \
	!IS_ENABLED(CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT) && \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))
	zephcore_usbd_init();
	zephcore_usbd_wait_dtr(2000);
#endif
	LOG_INF("=== ZephCore UART Companion starting ===");

#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
	if (gpio_is_ready_dt(&led0)) {
		gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	}
#endif

	if (!ZephyrDataStore::mount()) {
		LOG_ERR("LittleFS mount failed");
	}
	data_store.begin();

	{
		uint32_t rtc_epoch;
		if (zephcore_rtc_restore(&rtc_epoch)) {
			rtc_clock.setCurrentTime(rtc_epoch);
		}
	}

	ui_init();
#if !IS_ENABLED(CONFIG_ZEPHCORE_UI_DISPLAY)
	oled_sleep();
#endif

#ifdef ZEPHCORE_LORA
	uart_companion_mesh_ptr = &uart_companion_mesh;

	k_event_init(&mesh_events);

	lora_radio.setRxCallback(lora_rx_callback, nullptr);
	lora_radio.setTxDoneCallback(lora_tx_done_callback, nullptr);
	uart_companion_mesh.setTxQueuedCallback(tx_queued_callback, nullptr);

	mesh::LocalIdentity self_identity;
	if (!data_store.loadMainIdentity(self_identity)) {
		LOG_INF("No identity found, generating new keypair...");
		mesh::ZephyrRNG::generateFirstBootIdentity(self_identity);
		data_store.saveMainIdentity(self_identity);
		LOG_INF("New identity saved");
	}
	uart_companion_mesh.self_id = self_identity;

	LOG_INF("Node ID: %02x%02x%02x%02x%02x%02x%02x%02x...",
		self_identity.pub_key[0], self_identity.pub_key[1],
		self_identity.pub_key[2], self_identity.pub_key[3],
		self_identity.pub_key[4], self_identity.pub_key[5],
		self_identity.pub_key[6], self_identity.pub_key[7]);

	/* Load persisted prefs and bind the radio to prefs BEFORE begin() —
	 * mirrors main_repeater.cpp / main_companion.cpp. */
	data_store.loadPrefs(uart_companion_mesh.prefs);
	lora_radio.setPrefs(&uart_companion_mesh.prefs);

	uart_companion_mesh.begin();

	NodePrefs *prefs = uart_companion_mesh.getNodePrefs();
	if (strlen(prefs->node_name) == 0) {
		uint8_t dev_id[8];
		ssize_t id_len = hwinfo_get_device_id(dev_id, sizeof(dev_id));
		if (id_len >= 4) {
			snprintf(prefs->node_name, sizeof(prefs->node_name),
				 "UartComp-%02X%02X%02X%02X", dev_id[0], dev_id[1], dev_id[2], dev_id[3]);
		}
	}

	lora_radio.setRxBoost(prefs->rx_boost != 0);
	lora_radio.enableRxDutyCycle(prefs->rx_duty_cycle != 0);

	add_drone_base_contact();

	ui_set_node_name(prefs->node_name);
	ui_set_radio_params(
		lora_radio.getActiveFrequencyHz(),
		lora_radio.getActiveSpreadingFactor(),
		lora_radio.getActiveBandwidthKHzX10(),
		lora_radio.getActiveCodingRate(),
		lora_radio.getConfiguredTxPower(),
		lora_radio.getNoiseFloor());
	ui_set_battery_provider(get_battery_mv);
	ui_set_battery(zephyr_board.getBattMilliVolts(), 0);

	LOG_INF("Initial advertisement scheduled in 10s");
	k_work_schedule(&initial_advert_work, K_SECONDS(10));
#endif

	/* Bench-diagnostics CLI device: native USB CDC if present, else the
	 * chosen console UART (XIAO MG24 has no native USB — falls back to
	 * usart0). Mirrors main_repeater.cpp exactly. */
#if ZEPHCORE_USB_STACK && DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart)
	usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
#else
	usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
	if (device_is_ready(usb_dev)) {
		LOG_INF("CLI UART ready: %s", usb_dev->name);
		ring_buf_init(&usb_ring_buf, sizeof(usb_ring_buf_data), usb_ring_buf_data);
		uart_irq_callback_set(usb_dev, cli_uart_isr);
		uart_irq_rx_enable(usb_dev);
	} else {
		LOG_ERR("CLI UART device not ready");
		usb_dev = NULL;
	}

	/* C5 detection-frame UART — see boards/common/uart_companion.overlay
	 * for the `zephcore,c5-uart` chosen node (EUSART1 on XIAO MG24). */
#if DT_HAS_CHOSEN(zephcore_c5_uart)
	c5_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephcore_c5_uart));
	if (device_is_ready(c5_uart_dev)) {
		LOG_INF("C5 UART ready: %s", c5_uart_dev->name);
		ring_buf_init(&c5_ring_buf, sizeof(c5_ring_buf_data), c5_ring_buf_data);
		uart_irq_callback_set(c5_uart_dev, c5_uart_isr);
		uart_irq_rx_enable(c5_uart_dev);
	} else {
		LOG_ERR("C5 UART device not ready");
		c5_uart_dev = NULL;
	}
#else
	LOG_WRN("No zephcore,c5-uart chosen node for this board — C5 link disabled");
#endif

#ifdef ZEPHCORE_LORA
	uart_companion_event_loop();  /* Never returns */
#else
	for (;;) {
		k_sleep(K_FOREVER);
	}
#endif

	return 0;
}
