/*
 * SPDX-License-Identifier: MIT
 * UartCompanionMesh - ZephCore UART Companion device application layer
 *
 * Phase 2 bring-up skeleton for the carrier-v3 board (XIAO MG24 + Wio-SX1262,
 * paired with an ESP32-C5 Remote-ID scanner over UART). A slim BaseChatMesh
 * subclass: no BLE, no USB companion binary protocol, no contact/channel
 * flash persistence. Its eventual job is to relay ESP32-C5 detection frames
 * (received over the `zephcore,c5-uart` UART — see main_uart_companion.cpp)
 * to a single pinned drone-base contact as MeshCore TXT_MSGs.
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

	/* Increment 2: send a pre-formatted 'I'/'T' MeshCore TXT_MSG (already
	 * decoded from a CMD_RID_FORMATTED_MSG C5 frame — see
	 * main_uart_companion.cpp's process_c5_uart()/c5_frame_dispatch()) to the
	 * pinned drone-base contact via BaseChatMesh::sendMessage(), tracking the
	 * expected ACK with addPendingAck() so a later increment can match it in
	 * processAck(). No retry here — a single send + recorded expected-ack is
	 * the whole job of this increment. Returns false (LOG_WRN'd) if the
	 * drone-base contact isn't registered yet or sendMessage() reports
	 * MSG_SEND_FAILED; true means the packet was handed to the dispatcher
	 * (flood or direct), not that delivery was confirmed.
	 */
	bool sendDetectionToBase(const char *text);

	/* Pending-ACK tracking (model: CompanionMesh::addPendingAck/
	 * findAndRemoveAck). contact_idx is whatever sendMessage's caller wants
	 * to recover in processAck() — the Phase 2 sender will pass the
	 * drone-base contact's index. */
	void addPendingAck(uint32_t expected, int contact_idx);
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
	struct AckEntry {
		uint32_t expected_ack;
		int contact_idx;
		bool active;
	};
	AckEntry _ack_table[UART_COMPANION_ACK_TABLE_SIZE];
	int _ack_next_overwrite;

	uint8_t _drone_base_pubkey[PUB_KEY_SIZE];
	bool _drone_base_pubkey_set;
};
