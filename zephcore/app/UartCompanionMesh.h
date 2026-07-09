/*
 * SPDX-License-Identifier: MIT
 * UartCompanionMesh - ZephCore UART Companion device application layer
 *
 * Phase 2 bring-up skeleton for the carrier-v3 board (XIAO MG24 + Wio-SX1262,
 * paired with an ESP32-C5 Remote-ID scanner over UART). A slim BaseChatMesh
 * subclass: no BLE companion protocol (NUS/phone pairing/GATT), no USB
 * companion binary protocol, no contact/channel flash persistence. (The
 * MG24's BLE radio IS used as of increment 4 — see ../src/ble_rid_observer.h
 * — but purely as a passive RID-advert scanner, with zero involvement from
 * this class's BaseChatMesh transport.) Its job is to relay detections
 * (ESP32-C5 frames over the `zephcore,c5-uart` UART, or BLE RID adverts
 * caught directly by this board) to a single pinned drone-base contact as
 * MeshCore TXT_MSGs.
 *
 * Modeled on app/CompanionMesh.{h,cpp} — see that file for the fuller
 * BLE-companion implementation of each hook this class stubs minimally.
 */

#pragma once

#include <helpers/BaseChatMesh.h>
#include <helpers/NodePrefs.h>

/* Pending-ack table for detection TXT_MSGs sent to the drone-base contact.
 * Sized well above the single-contact case for headroom; there is no
 * CONFIG_ZEPHCORE_ACK_TABLE_SIZE visible outside ZEPHCORE_ROLE_COMPANION
 * today (see zephcore/Kconfig), so this is a local constant, not a Kconfig. */
#define UART_COMPANION_ACK_TABLE_SIZE 8

/* Retry budget for a pending detection-relay send. Reuses the value of
 * MESHCORE_IDENTITY_RETRY_LIMIT (lib/meshcore_shared/src/meshcore_shared.h,
 * =3) that the Arduino carrier's drone_cache retry engine
 * (nodes/nrf52_meshcore_node/src/drone_cache.cpp) uses for Identity sends —
 * hand-ported as a literal here since that header lives in the
 * Arduino/PlatformIO build tree and isn't shared with this Zephyr build
 * (same caveat as main_uart_companion.cpp's C5 UART wire-protocol block).
 * Counts total attempts (1 initial send + up to 2 retries), matching
 * drone_cache's `pending_id_attempts < MESHCORE_IDENTITY_RETRY_LIMIT` gate. */
#define UART_COMPANION_RETRY_LIMIT 3

/* Max text length copied into a pending-send entry for retry. Mirrors
 * RID_FORMATTED_MSG_MAX_TEXT (lib/meshcore_shared/src/meshcore_shared.h,
 * =151) / C5_RID_FORMATTED_MSG_MAX_TEXT in main_uart_companion.cpp — keep in
 * sync if that constant ever changes. */
#define UART_COMPANION_TEXT_MAX 151

class UartCompanionMesh : public BaseChatMesh {
public:
	UartCompanionMesh(mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
		mesh::RTCClock &rtc, mesh::PacketManager &mgr, mesh::MeshTables &tables);

	/* Prefs (radio params, node name). Bound to the radio via setPrefs()
	 * before begin() — see main_uart_companion.cpp, mirrors main_repeater.cpp. */
	NodePrefs prefs;
	NodePrefs *getNodePrefs() { return &prefs; }

	/* Zero-hop or flood self-advert (name only, no location — this role has
	 * no GPS). Real (non-stubbed) implementation: matches the "both sides
	 * fire one flood self-advert in setup()" pairing bootstrap documented
	 * in the top-level CLAUDE.md for the Arduino carrier firmware, so the
	 * drone-base auto-adds this node as a contact via onAdvertRecv. */
	bool sendSelfAdvert(bool flood) override;

	/* Record the pinned drone-base contact's pubkey so sendDetectionToBase()
	 * can find it via lookupContactByPubKey(). Call once after addContact()
	 * has added the drone-base contact — see main_uart_companion.cpp's
	 * add_drone_base_contact(). */
	void setDroneBaseContact(const uint8_t pubkey[PUB_KEY_SIZE]);

	/* Increment 3: send a pre-formatted 'I'/'T' MeshCore TXT_MSG (already
	 * decoded from a CMD_RID_FORMATTED_MSG C5 frame — see
	 * main_uart_companion.cpp's process_c5_uart()/c5_frame_dispatch()) to the
	 * pinned drone-base contact via BaseChatMesh::sendMessage() (attempt=1),
	 * recording a pending-send entry (text copy, attempt, expected ACK,
	 * deadline) via addPendingSend() so checkTimeouts() can retry on ACK
	 * timeout and processAck() can confirm delivery. Returns false
	 * (LOG_WRN'd) if the drone-base contact isn't registered yet or
	 * sendMessage() reports MSG_SEND_FAILED; true means the packet was
	 * handed to the dispatcher (flood or direct), not that delivery was
	 * confirmed — see getAckStats()/countActivePending() for that.
	 */
	bool sendDetectionToBase(const char *text);

	/* Age pending detection-relay sends past their ACK-wait deadline.
	 * Retries (re-sends via sendMessage() with attempt+1, which yields a NEW
	 * expected_ack per BaseChatMesh::composeMsgPacket folding `attempt` into
	 * the ACK hash) up to UART_COMPANION_RETRY_LIMIT total attempts; once
	 * exhausted, marks the entry undelivered and clears it. Call once per
	 * housekeeping tick (main_uart_companion.cpp's MESH_EVENT_HOUSEKEEPING
	 * handler) — uses the mesh millisecond clock (futureMillis()/
	 * millisHasNowPassed(), inherited from mesh::Dispatcher), not RTC. */
	void checkTimeouts();

	/* Delivery-confirmation + retry counters for the `ridstats` bench CLI
	 * command (main_uart_companion.cpp). attempted counts only the initial
	 * send per detection (not retries); retries counts re-sends;
	 * undelivered counts retry-budget exhaustion; delivered counts
	 * processAck() matches. */
	struct AckStats {
		uint32_t attempted;
		uint32_t delivered;
		uint32_t undelivered;
		uint32_t retries;
	};
	void getAckStats(AckStats *out) const;
	int countActivePending() const;

	/* Pending-send tracking (model: CompanionMesh::addPendingAck/
	 * findAndRemoveAck, extended with what a retry needs). contact_idx is
	 * whatever sendMessage's caller wants to recover in processAck() — the
	 * Phase 2/3 sender passes the drone-base contact's index. */
	void addPendingSend(const char *text, uint8_t attempt, uint32_t expected,
		int contact_idx, uint32_t deadline_ms);
	int findAndRemoveAck(uint32_t ack);

protected:
	/* BaseChatMesh pure virtuals — minimal stubs for the Phase 2 skeleton.
	 * See UartCompanionMesh.cpp for what each currently does; CompanionMesh
	 * (app/CompanionMesh.cpp) is the reference for the full behavior. */
	void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path) override;
	ContactInfo *processAck(const uint8_t *data) override;
	void onContactPathUpdated(const ContactInfo &contact) override;
	void onMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) override;
	void onCommandDataRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) override;
	void onSignedMessageRecv(const ContactInfo &contact, mesh::Packet *pkt, uint32_t sender_timestamp,
		const uint8_t *sender_prefix, const char *text) override;
	uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override;
	uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override;
	void onSendTimeout() override;
	void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp, const char *text) override;
	uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp,
		const uint8_t *data, uint8_t len, uint8_t *reply) override;
	void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) override;

private:
	/* One in-flight detection-relay send. `text` is a bounded copy (not a
	 * pointer into the caller's stack buffer — c5_frame_dispatch()'s text[]
	 * doesn't outlive the dispatch call) so checkTimeouts() can recompose +
	 * resend it on retry. */
	struct PendingSend {
		bool active;
		char text[UART_COMPANION_TEXT_MAX + 1];
		uint8_t attempt;
		uint32_t expected_ack;
		int contact_idx;
		uint32_t deadline_ms;
	};
	PendingSend _ack_table[UART_COMPANION_ACK_TABLE_SIZE];
	int _ack_next_overwrite;
	AckStats _ack_stats;

	uint8_t _drone_base_pubkey[PUB_KEY_SIZE];
	bool _drone_base_pubkey_set;
};
