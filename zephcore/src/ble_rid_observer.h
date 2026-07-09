/*
 * SPDX-License-Identifier: MIT
 * BLE Remote-ID observer — ZephCore UART Companion, increment 4 of 4.
 *
 * The MG24's own BLE radio scans (BT_OBSERVER role — passive scan, no
 * connections) for ASTM F3411 RemoteID service-data adverts, parses them via
 * the ported ODID path (lib/rid_odid/rid_odid.h), formats an Identity or
 * Telemetry MeshCore TXT_MSG, and relays it to the drone-base contact via
 * the SAME send/retry path the C5-UART detection relay uses (increments
 * 2-3): UartCompanionMesh::sendDetectionToBase().
 *
 * See main_uart_companion.cpp for wiring: ble_rid_observer_init() is called
 * once at boot (after the mesh + drone-base contact are up), then
 * ble_rid_observer_start() enables the BT stack and kicks off the scan.
 */

#pragma once

#include <stdint.h>

class UartCompanionMesh;

/* Registers the mesh instance used to relay parsed detections, plus a notify
 * callback the scan callback fires (from the BT RX workqueue) to wake the main
 * event loop when a detection has been queued. Call once during boot, before
 * ble_rid_observer_start(). A null mesh is tolerated (handle_astm() logs +
 * drops instead of crashing) so the `bletest` CLI bench hook still works even
 * if called before boot finishes wiring the mesh up — see
 * main_uart_companion.cpp. `notify` should post the loop's BLE-RX event bit. */
void ble_rid_observer_init(UartCompanionMesh *mesh, void (*notify)(void));

/* Drain BLE ASTM detections queued by the scan callback and run each through
 * ble_rid_observer_handle_astm() ON THE CALLING THREAD. Call from the main
 * event loop when the notify-posted BLE-RX event fires. The scan callback runs
 * on the BT RX workqueue, whose stack overflows in the mesh send path, so it
 * only enqueues the raw ODID body + MAC + RSSI; the parse/format/mesh-send work
 * happens here — on the main thread's stack, and single-threaded with the rest
 * of the mesh. Returns the number of detections processed this call. */
int ble_rid_observer_process_pending(void);

/* Enables the Zephyr BT stack (bt_enable, synchronous/blocking form) and
 * starts a passive BT_OBSERVER scan filtered for ASTM F3411 RemoteID
 * service-data adverts (UUID 0xFFFA, app code 0x0D — see
 * ble_rid_observer.cpp). Logs "BLE observer started" on success. On failure
 * (bt_enable or bt_le_scan_start returning nonzero) LOG_ERR's the exact
 * error and returns false WITHOUT hanging the caller — BLE+LoRa coexistence
 * on the MG24 is the real risk this increment is testing; a failure here is
 * a finding to report, not something to retry/hack around. */
bool ble_rid_observer_start(void);

/* Decode a raw ASTM F3411 ODID message body (the 25 bytes after the
 * service-data AD structure's uuid(2)+app_code(1)+msg_counter(1) header)
 * through the SAME path a live scan_cb() match uses: accumulate into the
 * per-MAC rid_id_data slot, then format + send an Identity (BasicID
 * messages) or Telemetry (Location messages) MeshCore TXT_MSG via
 * sendDetectionToBase(). Other ASTM message types (Auth/SelfID/System/
 * OperatorID) only update the accumulation slot — no send, since neither
 * formatter needs them standalone.
 *
 * `len` should be 25 (MIN_BLE_ODID_BODY_LEN); shorter bodies are logged +
 * dropped. Exposed (not static) so the `bletest <hex>` bench CLI command in
 * main_uart_companion.cpp can exercise the full parse -> format -> send path
 * without a live RID broadcaster on the bench. */
void ble_rid_observer_handle_astm(const uint8_t *msg, uint16_t len, int8_t rssi,
				   const uint8_t mac6[6]);
