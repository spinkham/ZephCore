/*
 * SPDX-License-Identifier: MIT
 * DroneDetectionCache implementation — see DroneDetectionCache.h.
 *
 * The cadence / motion-tier / airtime-budget logic is a near-verbatim port of
 * nodes/nrf52_meshcore_node/src/drone_cache.cpp's process_slot() +
 * tx_budget_*() (the Arduino carrier's proven throttle), restructured from a
 * 1 Hz walk-all-slots tick into a per-advert admission gate. That change is
 * behaviourally equivalent here: a live RID broadcaster re-adverts several
 * times per second — far faster than any cadence (≥5 s) — so gating on each
 * incoming advert re-emits at the same cadence a periodic walk would, without
 * a separate timer. Delivery confirmation + retry are NOT ported (they live in
 * UartCompanionMesh's ACK table); the identity-hash gate keys off last-*sent*,
 * not last-*delivered*, because the mesh already retries un-ACKed Identity.
 */

#include "DroneDetectionCache.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>   /* CONTAINER_OF, CLAMP */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_drone_cache, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* One tracked drone: the ODID accumulator plus the send-cadence bookkeeping
 * the gate needs. Kept private to this translation unit; callers only ever see
 * the embedded rid_id_data via drone_cache_slot(). */
struct DroneSlot {
	bool        in_use;
	rid_id_data uav;                    /* ODID accumulator (owns mac[6]) */
	uint32_t    first_seen_ms;
	uint32_t    last_identity_sent_ms;
	uint32_t    last_identity_hash;
	uint32_t    last_telemetry_sent_ms;
	double      last_emit_lat;          /* position at last Telemetry emit */
	double      last_emit_long;
	uint32_t    last_motion_ms;         /* when the slot last crossed MOTION_M */
};

static DroneSlot            g_slots[DDC_MAX_SLOTS];
static struct drone_cache_stats g_stats;

/* Airtime token bucket — packed array of live emit events, compacted on each
 * age pass. Ported from drone_cache.cpp (millis() -> k_uptime_get()). Sized
 * comfortably above the worst-case in-flight count at DDC_TX_BUDGET_AIRTIME_MS
 * / min-per-emit airtime. */
#define TX_BUDGET_RING_LEN 32
struct emit_event_t {
	uint32_t ts_ms;
	uint16_t airtime_ms;
};
static emit_event_t s_emit_log[TX_BUDGET_RING_LEN];
static uint8_t      s_emit_log_count;   /* live entries (also = write head) */

/* Last base-down Telemetry probe on the slot-less C5 relay path (gate_relay).
 * The BLE path tracks the probe cadence per-slot; the relay path has no slot,
 * so it gates off this single module-level timestamp instead. */
static uint32_t s_last_relay_probe_ms;

/* ----------------------------------------------------------------------------
 * Pure helpers ported from lib/meshcore_shared/src/meshcore_shared.h (they had
 * no Arduino dependency; only the id_data type changes to rid_id_data).
 * --------------------------------------------------------------------------*/

/* FNV-1a over the static Identity fields — a change here forces an Identity
 * re-emit (meshcore_identity_hash). Field-by-field with NUL delimiters, not a
 * raw struct hash, so struct padding can't cause false re-emits. */
static uint32_t identity_hash(const rid_id_data *u)
{
	uint32_t h = 2166136261u;
	size_t i;

	for (i = 0; i < ODID_ID_SIZE && u->uav_id[i] != '\0'; i++) {
		h ^= (uint8_t)u->uav_id[i]; h *= 16777619u;
	}
	h ^= 0; h *= 16777619u;
	for (i = 0; i < ODID_ID_SIZE && u->uav_id2[i] != '\0'; i++) {
		h ^= (uint8_t)u->uav_id2[i]; h *= 16777619u;
	}
	h ^= 0; h *= 16777619u;
	for (i = 0; i < ODID_ID_SIZE && u->op_id[i] != '\0'; i++) {
		h ^= (uint8_t)u->op_id[i]; h *= 16777619u;
	}
	h ^= 0; h *= 16777619u;
	for (i = 0; i < ODID_STR_SIZE && u->self_id[i] != '\0'; i++) {
		h ^= (uint8_t)u->self_id[i]; h *= 16777619u;
	}
	h ^= 0; h *= 16777619u;

	h ^= u->id_type;           h *= 16777619u;
	h ^= u->id_type2;          h *= 16777619u;
	h ^= u->ua_type;           h *= 16777619u;
	h ^= u->op_id_type;        h *= 16777619u;
	h ^= u->self_id_desc_type; h *= 16777619u;
	return h;
}

/* Equirectangular distance in metres (meshcore_approx_distance_m) — accurate
 * to <1 m over RID-relevant separations, ~3x cheaper than haversine. */
static float approx_distance_m(double lat1, double lon1, double lat2, double lon2)
{
	const double earth_r_m       = 6371000.0;
	const double deg_to_rad      = 0.017453292519943295; /* pi/180 */
	double mean_lat_rad = (lat1 + lat2) * 0.5 * deg_to_rad;
	double dlat_rad = (lat2 - lat1) * deg_to_rad;
	double dlon_rad = (lon2 - lon1) * deg_to_rad * cos(mean_lat_rad);
	double dist2 = (dlat_rad * dlat_rad + dlon_rad * dlon_rad)
		       * (earth_r_m * earth_r_m);
	return (float)sqrt(dist2);
}

/* ----------------------------------------------------------------------------
 * Airtime token bucket (ported from drone_cache.cpp).
 * --------------------------------------------------------------------------*/

/* Drop entries older than the rolling window, compacting in place. */
static void tx_budget_age(uint32_t now)
{
	uint8_t write = 0;
	for (uint8_t i = 0; i < s_emit_log_count; i++) {
		if ((now - s_emit_log[i].ts_ms) < DDC_TX_BUDGET_WINDOW_MS) {
			if (write != i) {
				s_emit_log[write] = s_emit_log[i];
			}
			write++;
		}
	}
	s_emit_log_count = write;
}

static uint32_t tx_budget_current_airtime(uint32_t now)
{
	tx_budget_age(now);
	uint32_t used = 0;
	for (uint8_t i = 0; i < s_emit_log_count; i++) {
		used += s_emit_log[i].airtime_ms;
	}
	return used;
}

/* Reserve est_airtime_ms against the rolling window if it fits. Returns false
 * (and charges nothing) when the window is saturated — the caller must NOT
 * advance its last-sent bookkeeping in that case so the next advert retries. */
static bool tx_budget_take(uint32_t now, uint32_t est_airtime_ms)
{
	uint32_t used = tx_budget_current_airtime(now);
	if (used + est_airtime_ms > DDC_TX_BUDGET_AIRTIME_MS) {
		g_stats.budget_skipped++;
		static uint32_t s_last_log;
		if (s_last_log == 0 || (now - s_last_log) > 30000u) {
			LOG_INF("tx-budget reached: %u/%u ms in %u ms window — skip %u ms emit",
				(unsigned)used, (unsigned)DDC_TX_BUDGET_AIRTIME_MS,
				(unsigned)DDC_TX_BUDGET_WINDOW_MS,
				(unsigned)est_airtime_ms);
			s_last_log = now;
		}
		return false;
	}
	if (s_emit_log_count >= TX_BUDGET_RING_LEN) {
		/* Ring is sized above the theoretical max in-flight count; if it
		 * ever fills, drop the oldest to make room. */
		for (uint8_t i = 1; i < s_emit_log_count; i++) {
			s_emit_log[i - 1] = s_emit_log[i];
		}
		s_emit_log_count--;
	}
	s_emit_log[s_emit_log_count].ts_ms      = now;
	s_emit_log[s_emit_log_count].airtime_ms =
		(uint16_t)(est_airtime_ms > 65535u ? 65535u : est_airtime_ms);
	s_emit_log_count++;
	g_stats.airtime_used_ms += est_airtime_ms;
	return true;
}

/* ----------------------------------------------------------------------------
 * Slot table.
 * --------------------------------------------------------------------------*/

static int find_slot_by_mac(const uint8_t mac[6])
{
	for (int i = 0; i < DDC_MAX_SLOTS; i++) {
		if (g_slots[i].in_use && memcmp(g_slots[i].uav.mac, mac, 6) == 0) {
			return i;
		}
	}
	return -1;
}

void drone_cache_init(void)
{
	memset(g_slots, 0, sizeof(g_slots));
	memset(&g_stats, 0, sizeof(g_stats));
	memset(s_emit_log, 0, sizeof(s_emit_log));
	s_emit_log_count = 0;
	s_last_relay_probe_ms = 0;
}

rid_id_data *drone_cache_slot(const uint8_t mac[6], uint32_t now_ms)
{
	int idx = find_slot_by_mac(mac);
	if (idx < 0) {
		/* Free slot? */
		for (int i = 0; i < DDC_MAX_SLOTS; i++) {
			if (!g_slots[i].in_use) {
				idx = i;
				break;
			}
		}
	}
	if (idx < 0) {
		/* LRU-evict the oldest last_seen. */
		uint32_t oldest = UINT32_MAX;
		idx = 0;
		for (int i = 0; i < DDC_MAX_SLOTS; i++) {
			if (g_slots[i].uav.last_seen < oldest) {
				oldest = g_slots[i].uav.last_seen;
				idx = i;
			}
		}
	}

	DroneSlot &s = g_slots[idx];
	if (!s.in_use || memcmp(s.uav.mac, mac, 6) != 0) {
		memset(&s, 0, sizeof(s));
		s.in_use = true;
		s.first_seen_ms = now_ms;
		memcpy(s.uav.mac, mac, 6);
	}
	s.uav.last_seen = now_ms;
	return &s.uav;
}

/* ----------------------------------------------------------------------------
 * Admission gate.
 * --------------------------------------------------------------------------*/

bool drone_cache_gate(rid_id_data *uav, char kind, uint32_t now_ms,
		      int free_pool, bool base_down)
{
	if (!uav) {
		return false;
	}
	DroneSlot &s = *CONTAINER_OF(uav, DroneSlot, uav);

	if (kind == 'I') {
		/* Identity: re-emit only on first sight, static-field hash change,
		 * or keepalive. Base-down does NOT suppress Identity — it is the
		 * priority stream, and the keepalive is what the operator relies on
		 * to learn the airspace once the base recovers. */
		uint32_t cur_hash = identity_hash(uav);
		bool due = (s.last_identity_sent_ms == 0) ||
			   (cur_hash != s.last_identity_hash) ||
			   ((now_ms - s.last_identity_sent_ms) >= DDC_IDENTITY_KEEPALIVE_MS);
		if (!due) {
			g_stats.identity_suppressed++;
			return false;
		}
		if (free_pool < DDC_POOL_RESERVE_CRITICAL) {
			g_stats.backpressure_skipped++;
			return false;
		}
		if (!tx_budget_take(now_ms, DDC_AIRTIME_IDENTITY_MS)) {
			return false;   /* budget_skipped counted inside */
		}
		s.last_identity_sent_ms = now_ms;
		s.last_identity_hash    = cur_hash;
		g_stats.identity_emitted++;
		return true;
	}

	if (kind == 'T') {
		/* Telemetry: motion-tiered cadence, sheds first under load. */
		float motion_m = 0.0f;
		if (s.last_telemetry_sent_ms != 0) {
			motion_m = approx_distance_m(s.last_emit_lat, s.last_emit_long,
						     uav->lat_d, uav->long_d);
		}
		bool moving = (motion_m >= DDC_MOTION_M);
		if (moving) {
			s.last_motion_ms = now_ms;
		}
		uint32_t age_ms = (s.first_seen_ms == 0) ? 0 : (now_ms - s.first_seen_ms);
		bool just_arrived = (age_ms < DDC_NEW_TRACK_WINDOW_MS);
		bool stationary = (motion_m < DDC_STATIONARY_M) &&
				  (s.last_motion_ms != 0) &&
				  ((now_ms - s.last_motion_ms) > DDC_STATIONARY_HOLD_MS);

		uint32_t interval =
			moving       ? DDC_MIN_TELEMETRY_SPACING_MS
			: just_arrived ? DDC_NEW_TRACK_INTERVAL_MS
			: stationary   ? DDC_STATIONARY_INTERVAL_MS
			:                DDC_NORMAL_INTERVAL_MS;

		/* Base-down breaker: don't fully suppress Telemetry — override the
		 * motion tier with the fixed probe cadence so a returning base gets an
		 * ACK (which clears the breaker) within one probe interval instead of
		 * waiting up to the 5-min Identity keepalive. Still ~1 send / probe
		 * interval per drone (pool-safe, Telemetry never retried). */
		if (base_down) {
			interval = DDC_BASE_DOWN_PROBE_MS;
		}

		uint32_t since_last = (s.last_telemetry_sent_ms == 0)
				      ? UINT32_MAX
				      : (now_ms - s.last_telemetry_sent_ms);
		if (since_last < interval) {
			/* Count outage-window holdbacks as breaker suppression so
			 * `brk_supp` still reflects an engaged breaker; normal-load
			 * cadence holdbacks stay telemetry_suppressed. */
			if (base_down) {
				g_stats.breaker_suppressed++;
			} else {
				g_stats.telemetry_suppressed++;
			}
			return false;
		}
		if (free_pool < DDC_POOL_RESERVE) {
			g_stats.backpressure_skipped++;
			return false;
		}
		if (!tx_budget_take(now_ms, DDC_AIRTIME_TELEMETRY_MS)) {
			return false;
		}
		s.last_telemetry_sent_ms = now_ms;
		s.last_emit_lat          = uav->lat_d;
		s.last_emit_long         = uav->long_d;
		g_stats.telemetry_emitted++;
		return true;
	}

	/* Unknown kind — drop rather than pass an unbudgeted send through. */
	LOG_WRN("drone_cache_gate: unknown kind '%c'", kind);
	return false;
}

bool drone_cache_gate_relay(char kind, uint32_t now_ms,
			    int free_pool, bool base_down)
{
	/* No per-MAC slot — the C5 owns its own per-MAC cadence upstream; this
	 * path only shares the airtime budget + backpressure + breaker so a
	 * misbehaving C5 image (or a base outage) can't flood/wedge the mesh. */
	if (kind == 'T') {
		/* Base-down: throttle relay Telemetry to the probe cadence (gated off
		 * a module-level timestamp — this path keeps no per-MAC slot) so a
		 * returning base recovers within a probe interval, not the 5-min
		 * Identity keepalive. */
		if (base_down &&
		    s_last_relay_probe_ms != 0 &&
		    (now_ms - s_last_relay_probe_ms) < DDC_BASE_DOWN_PROBE_MS) {
			g_stats.breaker_suppressed++;
			return false;
		}
		if (free_pool < DDC_POOL_RESERVE) {
			g_stats.backpressure_skipped++;
			return false;
		}
		if (!tx_budget_take(now_ms, DDC_AIRTIME_TELEMETRY_MS)) {
			return false;
		}
		if (base_down) {
			s_last_relay_probe_ms = now_ms;
		}
		g_stats.telemetry_emitted++;
		return true;
	}
	if (kind == 'I') {
		if (free_pool < DDC_POOL_RESERVE_CRITICAL) {
			g_stats.backpressure_skipped++;
			return false;
		}
		if (!tx_budget_take(now_ms, DDC_AIRTIME_IDENTITY_MS)) {
			return false;
		}
		g_stats.identity_emitted++;
		return true;
	}
	LOG_WRN("drone_cache_gate_relay: unknown kind '%c'", kind);
	return false;
}

void drone_cache_get_stats(struct drone_cache_stats *out)
{
	if (!out) {
		return;
	}
	uint8_t used = 0;
	for (int i = 0; i < DDC_MAX_SLOTS; i++) {
		if (g_slots[i].in_use) {
			used++;
		}
	}
	g_stats.slots_used     = used;
	g_stats.cur_airtime_ms = tx_budget_current_airtime((uint32_t)k_uptime_get());
	*out = g_stats;
}
