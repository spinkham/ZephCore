/*
 * SPDX-License-Identifier: MIT
 * DroneDetectionCache — detection-relay write-combining (airtime throttling)
 * for the ZephCore UART Companion role.
 *
 * The admission gate that sits in front of UartCompanionMesh::
 * sendDetectionToBase(). Every relayed detection — the MG24's own BLE ASTM
 * adverts (ble_rid_observer.cpp, the unthrottled flood source) and, as a
 * coarse backstop, the C5-UART CMD_RID_FORMATTED_MSG relay
 * (main_uart_companion.cpp) — passes through here first. It applies:
 *
 *   - a unified per-MAC cache (owns the rid_id_data ODID accumulator),
 *   - Identity hash + keepalive cadence (re-emit only on static-field change
 *     or every DDC_IDENTITY_KEEPALIVE_MS),
 *   - motion-tiered Telemetry cadence (moving / new-track / steady / stationary),
 *   - a shared airtime token bucket sized for the pinned CR8 radio config,
 *   - packet-pool low-water backpressure (Telemetry sheds first, Identity has
 *     priority), and
 *   - a base-down breaker that throttles Telemetry to a slow recovery probe
 *     (DDC_BASE_DOWN_PROBE_MS) while the base is unreachable.
 *
 * Delivery confirmation + retry stay in UartCompanionMesh's ACK table — this
 * module decides *whether new information is due to send*, the mesh decides
 * *whether it got there*. See docs/carrier-v3-write-combining.md for the full
 * design and the airtime accounting.
 *
 * PORTING NOTE: the cadence/motion/budget logic + tuning values are hand-ported
 * from nodes/nrf52_meshcore_node/src/drone_cache.cpp and the MESHCORE_* macros
 * in lib/meshcore_shared/src/meshcore_shared.h (the Arduino carrier's proven
 * throttle). Those headers pull in <Arduino.h> and aren't shared with this
 * Zephyr build, so the constants are re-declared below as literals — same
 * established pattern as lib/rid_odid/rid_odid.h and UartCompanionMesh.h's
 * UART_COMPANION_RETRY_LIMIT. KEEP THE VALUES IN SYNC with meshcore_shared.h if
 * they are ever re-tuned there.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "rid_odid.h"   /* rid_id_data — the per-MAC ODID accumulator */

/* ============================================================================
 * Tuning constants — hand-ported from lib/meshcore_shared/src/meshcore_shared.h
 * (values verbatim). #ifndef-guarded so a board .conf / build flag can override.
 * ==========================================================================*/

/* Re-emit Identity every 5 min as a keepalive even when static fields haven't
 * changed (MESHCORE_IDENTITY_KEEPALIVE_MS). */
#ifndef DDC_IDENTITY_KEEPALIVE_MS
#define DDC_IDENTITY_KEEPALIVE_MS   300000u
#endif

/* Telemetry cadence during the first 60 s of a track (MESHCORE_NEW_TRACK_*). */
#ifndef DDC_NEW_TRACK_INTERVAL_MS
#define DDC_NEW_TRACK_INTERVAL_MS    10000u
#endif
#ifndef DDC_NEW_TRACK_WINDOW_MS
#define DDC_NEW_TRACK_WINDOW_MS      60000u
#endif

/* Telemetry cadence for an established, slow-moving track (MESHCORE_NORMAL_*). */
#ifndef DDC_NORMAL_INTERVAL_MS
#define DDC_NORMAL_INTERVAL_MS       30000u
#endif

/* Keepalive cadence for a stationary drone (MESHCORE_STATIONARY_*). */
#ifndef DDC_STATIONARY_INTERVAL_MS
#define DDC_STATIONARY_INTERVAL_MS  120000u
#endif
#ifndef DDC_STATIONARY_HOLD_MS
#define DDC_STATIONARY_HOLD_MS       60000u
#endif

/* Motion thresholds, metres (MESHCORE_MOTION_M / MESHCORE_STATIONARY_M). */
#ifndef DDC_MOTION_M
#define DDC_MOTION_M                    50.0f
#endif
#ifndef DDC_STATIONARY_M
#define DDC_STATIONARY_M                10.0f
#endif

/* Hard floor on Telemetry inter-emit spacing per drone; also the "moving"
 * interval (MESHCORE_MIN_TELEMETRY_SPACING_MS). */
#ifndef DDC_MIN_TELEMETRY_SPACING_MS
#define DDC_MIN_TELEMETRY_SPACING_MS  5000u
#endif

/* Telemetry probe cadence while the base is down (isBaseDown()). Instead of
 * fully suppressing Telemetry during an outage, throttle it to one send per
 * this interval so a returning base gets an ACK — and clears the breaker —
 * within one probe interval, rather than waiting up to the 5-min Identity
 * keepalive. Still cuts >90% of the flood, and pool-safe (Telemetry is never
 * retried). This bounds post-outage detection recovery to ~this interval. */
#ifndef DDC_BASE_DOWN_PROBE_MS
#define DDC_BASE_DOWN_PROBE_MS       30000u
#endif

/* Shared airtime budget: max airtime consumed in any rolling window
 * (MESHCORE_TX_BUDGET_AIRTIME_MS / _WINDOW_MS) — ~19% duty. */
#ifndef DDC_TX_BUDGET_AIRTIME_MS
#define DDC_TX_BUDGET_AIRTIME_MS     11500u
#endif
#ifndef DDC_TX_BUDGET_WINDOW_MS
#define DDC_TX_BUDGET_WINDOW_MS      60000u
#endif

/* Fixed per-message airtime cost, CR8 (BW 62.5 kHz / SF8 / CR 4/8) — the
 * pinned bench/deployment radio config. A fixed table per docs/carrier-v3-
 * write-combining.md §3 rather than deriving from text length each send: the
 * shared approx_lora_airtime_ms() helper keys off MESHCORE_LORA_CR (default
 * CR5), which would under-throttle by ~half against the CR8 the radio actually
 * uses. Telemetry ~804 ms, typical Identity ~1000 ms (max 1492 ms). */
#ifndef DDC_AIRTIME_TELEMETRY_MS
#define DDC_AIRTIME_TELEMETRY_MS       804u
#endif
#ifndef DDC_AIRTIME_IDENTITY_MS
#define DDC_AIRTIME_IDENTITY_MS       1050u
#endif

/* Packet-pool low-water reserves (StaticPoolPacketManager POOL_SIZE = 32).
 * Below RESERVE, shed Telemetry so adverts/ACKs/retries always have room;
 * Identity keeps priority down to the tighter CRITICAL reserve. */
#ifndef DDC_POOL_RESERVE
#define DDC_POOL_RESERVE                 8
#endif
#ifndef DDC_POOL_RESERVE_CRITICAL
#define DDC_POOL_RESERVE_CRITICAL        3
#endif

/* Must match ble_rid_observer.cpp's MAX_TRACKED and the Arduino drone_cache's
 * MAX_SLOTS — the realistic concurrent-drone count within RF range. */
#ifndef DDC_MAX_SLOTS
#define DDC_MAX_SLOTS                    4
#endif

/* ============================================================================
 * Diagnostics — surfaced by the `ridstats` bench CLI (main_uart_companion.cpp).
 * ==========================================================================*/
struct drone_cache_stats {
	uint32_t identity_emitted;      /* gate opened for an Identity send      */
	uint32_t identity_suppressed;   /* Identity not due (hash/keepalive gate) */
	uint32_t telemetry_emitted;     /* gate opened for a Telemetry send      */
	uint32_t telemetry_suppressed;  /* Telemetry not due (cadence gate)      */
	uint32_t budget_skipped;        /* refused by the airtime token bucket   */
	uint32_t backpressure_skipped;  /* refused by pool low-water             */
	uint32_t breaker_suppressed;    /* Telemetry dropped while base is down  */
	uint32_t airtime_used_ms;       /* lifetime airtime charged to the budget*/
	uint32_t cur_airtime_ms;        /* airtime live in the current window    */
	uint8_t  slots_used;            /* active per-MAC slots                   */
};

/* ============================================================================
 * API — all calls are single-threaded on the main event loop (BLE observer
 * drain, C5 UART drain, and housekeeping all run there; the BT RX workqueue
 * only enqueues raw bytes and never touches this module). No locking.
 * ==========================================================================*/

/* Zero all cache + budget + counter state. Call once at boot, before the BLE
 * observer or C5 UART link can feed detections in. */
void drone_cache_init(void);

/* Find (or allocate, LRU-evicting when full) the per-MAC slot for `mac` and
 * return its rid_id_data accumulator for the caller to parse ODID messages
 * into. Bumps the slot's last_seen to `now_ms`; stamps first_seen on a fresh
 * slot. The returned pointer is stable until the slot is LRU-evicted. */
rid_id_data *drone_cache_slot(const uint8_t mac[6], uint32_t now_ms);

/* Admission gate for the BLE path. `uav` MUST be a pointer previously returned
 * by drone_cache_slot() (the slot is recovered from it). `kind` is 'I'
 * (Identity) or 'T' (Telemetry) — i.e. the first char of the formatted TXT_MSG.
 * `free_pool` is UartCompanionMesh::getFreePoolCount(); `base_down` is
 * isBaseDown(). Returns true iff this emit is due AND admitted under
 * budget/backpressure/breaker — in which case the budget has been charged and
 * the slot's send bookkeeping advanced, and the caller should proceed to
 * sendDetectionToBase(). Returns false (drop) otherwise. */
bool drone_cache_gate(rid_id_data *uav, char kind, uint32_t now_ms,
		      int free_pool, bool base_down);

/* Coarse backstop gate for the C5-UART CMD_RID_FORMATTED_MSG relay, which
 * carries pre-formatted text the C5 has already per-MAC-throttled. Shares the
 * airtime budget + backpressure + breaker with the BLE path but keeps no
 * per-MAC slot (no rid_id_data to key on). Same kind/free_pool/base_down
 * contract as drone_cache_gate(); returns true iff admitted (budget charged). */
bool drone_cache_gate_relay(char kind, uint32_t now_ms,
			    int free_pool, bool base_down);

/* Snapshot the counters (and compute live-window airtime + slots-used). */
void drone_cache_get_stats(struct drone_cache_stats *out);
