/*
 * SPDX-License-Identifier: MIT
 * BLE Remote-ID observer implementation — see ble_rid_observer.h.
 */

#include "ble_rid_observer.h"

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_ble_rid_observer, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>

#include "rid_odid.h"
#include <app/UartCompanionMesh.h>

/* ASTM F3411 BLE service-data identifiers — ported from
 * nodes/nrf52_ble_scanner/src/main.cpp:39-42 (the project's existing
 * Nordic/Bluefruit BLE-RID scanner; PORT THE FILTER LOGIC, not the Bluefruit
 * scan API — see that file's header comment for the wire-format reference).
 * Keep in sync if the ASTM/opendroneid wire format ever changes.
 *
 * Zephyr's own BT_DATA_SVC_DATA16 (zephyr/bluetooth/assigned_numbers.h) ==
 * 0x16, same value as AD_TYPE_SERVICE_DATA below; named locally to keep this
 * file's provenance comment attached to the constant. */
static constexpr uint8_t ASTM_AD_TYPE_SERVICE_DATA = 0x16;
static constexpr uint8_t ASTM_UUID_LO  = 0xFA; /* UUID 0xFFFA, LE */
static constexpr uint8_t ASTM_UUID_HI  = 0xFF;
static constexpr uint8_t ASTM_APP_CODE = 0x0D;
static constexpr size_t  ASTM_ODID_BODY_LEN = 25;
/* uuid(2) + app_code(1) + msg_counter(1) + odid_body(25) */
static constexpr size_t  ASTM_SVC_DATA_MIN_LEN = 2 + 1 + 1 + ASTM_ODID_BODY_LEN;

/* Per-MAC ODID accumulation slots — ported from nrf52_ble_scanner's
 * g_uav_table / find_or_alloc_slot() (main.cpp:54-80). ASTM spreads Identity
 * fields (UASID, operator ID, aircraft type...) across up to 5 message
 * types per drone; without accumulation, a Location-only advert would
 * format an all-empty Identity. No rate-limit/dedup/motion-tier logic here
 * — that's the nRF52 Arduino carrier's drone_cache.cpp, explicitly out of
 * scope for this increment (see CLAUDE.md "increment 4 of 4" scope note). */
#define MAX_TRACKED 4
static rid_id_data g_uav_table[MAX_TRACKED];

static UartCompanionMesh *s_mesh;

static uint32_t g_advs_total;
static uint32_t g_astm_catches;

/* Log an advert-count line at DBG every N adverts so a bench operator can
 * confirm the radio is actually scanning even with no ASTM broadcaster
 * present (ambient phone/laptop BLE adverts are enough to tick this). */
static constexpr uint32_t ADV_COUNT_LOG_INTERVAL = 25;

static rid_id_data *find_or_alloc_slot(const uint8_t mac[6])
{
	static const uint8_t zero_mac[6] = {0};

	for (int i = 0; i < MAX_TRACKED; i++) {
		if (memcmp(g_uav_table[i].mac, mac, 6) == 0) {
			return &g_uav_table[i];
		}
	}
	for (int i = 0; i < MAX_TRACKED; i++) {
		if (memcmp(g_uav_table[i].mac, zero_mac, 6) == 0) {
			return &g_uav_table[i];
		}
	}
	/* LRU eviction */
	uint32_t oldest = UINT32_MAX;
	int oldest_i = 0;
	for (int i = 0; i < MAX_TRACKED; i++) {
		if (g_uav_table[i].last_seen < oldest) {
			oldest = g_uav_table[i].last_seen;
			oldest_i = i;
		}
	}
	memset(&g_uav_table[oldest_i], 0, sizeof(rid_id_data));
	return &g_uav_table[oldest_i];
}

void ble_rid_observer_init(UartCompanionMesh *mesh)
{
	s_mesh = mesh;
	memset(g_uav_table, 0, sizeof(g_uav_table));
	g_advs_total = 0;
	g_astm_catches = 0;
}

void ble_rid_observer_handle_astm(const uint8_t *msg, uint16_t len, int8_t rssi,
				   const uint8_t mac6[6])
{
	if (!msg || len < ASTM_ODID_BODY_LEN) {
		LOG_WRN("handle_astm_ble: short ODID body len=%u (need >=%u)", (unsigned)len,
			(unsigned)ASTM_ODID_BODY_LEN);
		return;
	}

	char mac_str[18];
	snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac6[0], mac6[1],
		  mac6[2], mac6[3], mac6[4], mac6[5]);

	rid_id_data *uav = find_or_alloc_slot(mac6);
	memcpy(uav->mac, mac6, 6);
	uav->last_seen = (uint32_t)k_uptime_get();
	uav->rssi = rssi;
	uav->band = RID_BAND_BLE;
	uav->channel = 0;

	uint8_t msg_type = msg[0] & 0xF0;
	rid_odid_parse_ble_message(uav, msg);
	g_astm_catches++;

	if (msg_type == 0x00) {
		char text[160];
		int n = rid_odid_format_identity_msg(uav, text, sizeof(text));
		LOG_INF("ASTM parsed: BasicID mac=%s uasid='%s' rssi=%d -> Identity fmt_len=%d",
			mac_str, uav->uav_id, (int)rssi, n);
		if (n <= 0) {
			LOG_WRN("handle_astm_ble: Identity format failed, dropping");
		} else if (!s_mesh) {
			LOG_WRN("handle_astm_ble: mesh not initialized, dropping Identity");
		} else {
			s_mesh->sendDetectionToBase(text);
		}
	} else if (msg_type == 0x10) {
		char text[80];
		int n = rid_odid_format_telemetry_msg(uav, text, sizeof(text));
		LOG_INF("ASTM parsed: Location mac=%s rssi=%d -> Telemetry fmt_len=%d", mac_str,
			(int)rssi, n);
		if (n <= 0) {
			LOG_WRN("handle_astm_ble: Telemetry format failed, dropping");
		} else if (!s_mesh) {
			LOG_WRN("handle_astm_ble: mesh not initialized, dropping Telemetry");
		} else {
			s_mesh->sendDetectionToBase(text);
		}
	} else {
		LOG_DBG("ASTM parsed: msg_type=0x%02x mac=%s (accumulate only, no send)",
			msg_type, mac_str);
	}
}

namespace {

struct ble_scan_ctx {
	const bt_addr_le_t *addr;
	int8_t rssi;
};

/* bt_data_parse() AD-structure callback. Returning false stops parsing this
 * advert (we've found our one ASTM service-data record); true continues to
 * the next AD structure. */
bool ble_ad_parse_cb(struct bt_data *data, void *user_data)
{
	auto *ctx = static_cast<ble_scan_ctx *>(user_data);

	if (data->type != ASTM_AD_TYPE_SERVICE_DATA) {
		return true;
	}
	if (data->data_len < ASTM_SVC_DATA_MIN_LEN) {
		return true;
	}
	/* data->data[0..1] = UUID (LE), [2] = app code, [3] = msg counter
	 * (unused here), [4..28] = 25-byte ODID body. Same layout as the
	 * nRF52 Bluefruit reference — see ble_scan_nimble.cpp:50 and
	 * nrf52_ble_scanner/src/main.cpp:149-157. */
	if (data->data[0] != ASTM_UUID_LO || data->data[1] != ASTM_UUID_HI ||
	    data->data[2] != ASTM_APP_CODE) {
		return true;
	}

	/* Zephyr's bt_addr_t.val[] is LSB-first on the wire (see
	 * addr.h:bt_addr_to_str printing val[5]..val[0]) — same convention as
	 * Nordic's SoftDevice peer_addr.addr[]. Reverse to MSB-first canonical
	 * to match the project's wire/display convention. */
	uint8_t mac[6];
	for (int i = 0; i < 6; i++) {
		mac[i] = ctx->addr->a.val[5 - i];
	}

	ble_rid_observer_handle_astm(&data->data[4], (uint16_t)ASTM_ODID_BODY_LEN, ctx->rssi, mac);
	return false;
}

void ble_scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		  struct net_buf_simple *buf)
{
	ARG_UNUSED(adv_type);

	g_advs_total++;
	if ((g_advs_total % ADV_COUNT_LOG_INTERVAL) == 0) {
		LOG_DBG("BLE scan liveness: %u adverts seen, %u ASTM catches",
			(unsigned)g_advs_total, (unsigned)g_astm_catches);
	}

	ble_scan_ctx ctx = {addr, rssi};
	bt_data_parse(buf, ble_ad_parse_cb, &ctx);
}

} /* anonymous namespace */

bool ble_rid_observer_start(void)
{
	/* Synchronous form (cb=NULL): blocks until the BLE controller/host
	 * init completes and returns the result directly, rather than the
	 * async bt_ready()-callback form main_companion.cpp uses for the
	 * peripheral/NUS role. This role has nothing to defer past init
	 * (no advertising, no GATT server, no settings_load() for bonds —
	 * see boards/common/uart_companion.conf), so synchronous init keeps
	 * boot sequencing simple. */
	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("BLE observer: bt_enable failed (err %d)", err);
		return false;
	}

	/* Passive scan, NO duplicate filtering. BT_LE_SCAN_PASSIVE (the
	 * Zephyr helper macro) bakes in BT_LE_SCAN_OPT_FILTER_DUPLICATE,
	 * which for legacy (non-extended) advertising PDUs — what ASTM F3411
	 * BLE RID broadcasts use — dedups on advertiser address ALONE for
	 * the life of the scan session. A tracked drone's Location messages
	 * change every broadcast from the same MAC; with FILTER_DUPLICATE
	 * the host would see exactly one report ever per drone and then go
	 * silent. That's unacceptable against CLAUDE.md's design priority #1
	 * (detection capability first) — every RID scanner elsewhere in this
	 * project (WiFi NAN/beacon, nRF52 Bluefruit BLE) reports every
	 * matching packet uncached, so this role matches that. */
	static const struct bt_le_scan_param astm_scan_param = BT_LE_SCAN_PARAM_INIT(
		BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL,
		BT_GAP_SCAN_FAST_WINDOW);

	err = bt_le_scan_start(&astm_scan_param, ble_scan_cb);
	if (err) {
		LOG_ERR("BLE observer: bt_le_scan_start failed (err %d)", err);
		return false;
	}

	LOG_INF("BLE observer started");
	return true;
}
