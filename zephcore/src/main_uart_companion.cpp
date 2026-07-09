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
 * SCOPE (increment 2 of 4): the C5 frame parser and CMD_RID_FORMATTED_MSG ->
 * BaseChatMesh::sendMessage() path are implemented — see process_c5_uart()
 * and c5_frame_dispatch() below, and UartCompanionMesh::sendDetectionToBase().
 * CMD_RID_DETECTION (raw ODID) handling and ACK-driven retry are later
 * increments (dispatch logs and drops CMD_RID_DETECTION for now; a single
 * send + recorded expected-ack is as far as this increment goes).
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
#include <mesh/Utils.h>  /* mesh::Utils::fromHex — used by the ridframe bench CLI hook */

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

/* ========================================================================
 * C5 UART wire protocol — mirrors the top-level repo's
 * lib/meshcore_shared/src/meshcore_shared.h (MESHCORE_FRAME_IN,
 * CMD_RID_DETECTION, CMD_RID_FORMATTED_MSG, RID_FORMATTED_MSG_MAX_TEXT) and
 * lib/meshcore_shared/src/rid_uart_protocol.h. Those headers live in the
 * Arduino/PlatformIO build tree (HardwareSerial etc.) and aren't shared with
 * this Zephyr build, so the constants and framing state machine are
 * hand-ported here — keep in sync if the wire format ever changes. Direct
 * template: nodes/nrf52_meshcore_node/src/c5_uart_rx.cpp.
 *
 * Wire layout: <marker><len_lo><len_hi><cmd><payload>
 *   marker  = 0x3C (MESHCORE_FRAME_IN, repurposed as the C5->carrier
 *             inter-chip direction marker — see the naming caveat in
 *             rid_uart_protocol.cpp)
 *   len     = uint16_t LE, counts the cmd byte + payload bytes (NOT the
 *             marker/len bytes themselves)
 *   cmd     = CMD_RID_DETECTION (0x70) or CMD_RID_FORMATTED_MSG (0x71)
 *   payload = cmd-specific; CMD_RID_FORMATTED_MSG's payload is the raw
 *             'I'/'T' TXT_MSG text, no NUL terminator on the wire
 * No CRC, no closing/trailer marker — c5_uart_rx.cpp resyncs by returning to
 * IDLE once `len` payload bytes are collected and waiting for the next 0x3C;
 * this parser mirrors that exactly.
 * ======================================================================== */
#define C5_UART_FRAME_MARKER          0x3C
#define C5_CMD_RID_DETECTION          0x70
#define C5_CMD_RID_FORMATTED_MSG      0x71
#define C5_RID_FORMATTED_MSG_MAX_TEXT 151

enum class C5FrameState : uint8_t {
	IDLE,
	GOT_START,
	GOT_LEN_LO,
	IN_PAYLOAD,
};

static C5FrameState c5_frame_state = C5FrameState::IDLE;
static uint16_t c5_frame_len = 0;
static uint16_t c5_frame_pos = 0;
/* Sized for the largest known opcode payload (CMD_RID_FORMATTED_MSG: cmd
 * byte + up to C5_RID_FORMATTED_MSG_MAX_TEXT text bytes), plus slack —
 * mirrors c5_uart_rx.cpp's s_buf sizing rationale. */
static uint8_t c5_frame_buf[C5_RID_FORMATTED_MSG_MAX_TEXT + 16];

/* Dispatch one complete frame. cmd/payload point into c5_frame_buf. */
static void c5_frame_dispatch(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	switch (cmd) {
	case C5_CMD_RID_FORMATTED_MSG: {
		uint16_t text_len = len;
		if (text_len > C5_RID_FORMATTED_MSG_MAX_TEXT) {
			text_len = C5_RID_FORMATTED_MSG_MAX_TEXT;
		}
		char text[C5_RID_FORMATTED_MSG_MAX_TEXT + 1];
		memcpy(text, payload, text_len);
		text[text_len] = '\0';

		LOG_INF("C5 frame: FORMATTED_MSG len=%d -> send", (int)text_len);
#ifdef ZEPHCORE_LORA
		if (uart_companion_mesh_ptr) {
			uart_companion_mesh_ptr->sendDetectionToBase(text);
		}
#endif
		break;
	}
	case C5_CMD_RID_DETECTION:
		/* Raw ODID relay path — later increment. Drop for now. */
		LOG_INF("C5 frame: RID_DETECTION (raw path TODO)");
		break;
	default:
		LOG_WRN("C5 frame: unknown cmd=0x%02x len=%u, dropped", cmd, (unsigned)len);
		break;
	}
}

/* Single incremental parser step. This is the ONE entry point that both real
 * C5 UART bytes (process_c5_uart(), below) and the `ridframe` bench-test CLI
 * command (c5_ridframe_cli_cmd(), further down) feed through, so the CLI
 * hook exercises the exact same state machine rather than a parallel
 * test-only shortcut. State persists across calls so frames split across
 * UART RX chunks parse correctly. */
static void c5_frame_feed_byte(uint8_t b)
{
	switch (c5_frame_state) {
	case C5FrameState::IDLE:
		if (b == C5_UART_FRAME_MARKER) {
			c5_frame_state = C5FrameState::GOT_START;
		}
		break;
	case C5FrameState::GOT_START:
		c5_frame_len = b;
		c5_frame_state = C5FrameState::GOT_LEN_LO;
		break;
	case C5FrameState::GOT_LEN_LO:
		c5_frame_len |= ((uint16_t)b) << 8;
		if (c5_frame_len == 0 || c5_frame_len > sizeof(c5_frame_buf)) {
			LOG_WRN("C5 frame: bad length=%u, resync", (unsigned)c5_frame_len);
			c5_frame_state = C5FrameState::IDLE;
			break;
		}
		c5_frame_pos = 0;
		c5_frame_state = C5FrameState::IN_PAYLOAD;
		break;
	case C5FrameState::IN_PAYLOAD:
		if (c5_frame_pos < c5_frame_len) {
			c5_frame_buf[c5_frame_pos++] = b;
		}
		if (c5_frame_pos == c5_frame_len) {
			uint8_t cmd = c5_frame_buf[0];
			c5_frame_dispatch(cmd, &c5_frame_buf[1], (uint16_t)(c5_frame_len - 1));
			c5_frame_state = C5FrameState::IDLE;
		}
		break;
	}
}

/* Drain bytes from the C5 scanner UART through the frame parser. */
static void process_c5_uart(void)
{
	uint8_t byte;
	while (ring_buf_get(&c5_ring_buf, &byte, 1) == 1) {
		c5_frame_feed_byte(byte);
	}
}

/* Bench-test hook (increment 2, STEP 4): decode a hex-encoded frame from the
 * CLI and feed it through the SAME c5_frame_feed_byte() state machine used
 * for real UART bytes above — this exercises the actual framing code, not a
 * parallel test-only shortcut. Invoked from process_cli_commands() on a
 * `ridframe <hexbytes>` line, intercepted before CommonCLI ever sees it. */
static void c5_ridframe_cli_cmd(const char *hex, char *reply, size_t reply_size)
{
	size_t hexlen = strlen(hex);
	if (hexlen == 0 || (hexlen % 2) != 0) {
		snprintf(reply, reply_size, "ridframe: bad hex length %u (must be even, nonzero)",
			 (unsigned)hexlen);
		return;
	}

	size_t nbytes = hexlen / 2;
	uint8_t bytes[3 + sizeof(c5_frame_buf)];  /* marker + len16 + max frame payload */
	if (nbytes > sizeof(bytes)) {
		snprintf(reply, reply_size, "ridframe: %u bytes exceeds max %u",
			 (unsigned)nbytes, (unsigned)sizeof(bytes));
		return;
	}

	if (!mesh::Utils::fromHex(bytes, (int)nbytes, hex)) {
		snprintf(reply, reply_size, "ridframe: invalid hex characters");
		return;
	}

	for (size_t i = 0; i < nbytes; i++) {
		c5_frame_feed_byte(bytes[i]);
	}
	snprintf(reply, reply_size, "ridframe: fed %u byte(s) to parser", (unsigned)nbytes);
}

#ifdef ZEPHCORE_LORA
/* Run queued CLI commands on the MAIN thread (mirrors main_repeater.cpp). */
static void process_cli_commands(CommonCLI *cli)
{
	struct cli_cmd_line c;
	while (cli && k_msgq_get(&cli_cmd_queue, &c, K_NO_WAIT) == 0) {
		cli_reply_buf[0] = '\0';
		if (memcmp(c.buf, "ridframe ", 9) == 0) {
			/* Bench-test hook — see c5_ridframe_cli_cmd() above. */
			c5_ridframe_cli_cmd(c.buf + 9, cli_reply_buf, sizeof(cli_reply_buf));
		} else {
			cli->handleCommand(0, c.buf, cli_reply_buf);
		}
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
		uart_companion_mesh.setDroneBaseContact(drone_base_pubkey);
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
