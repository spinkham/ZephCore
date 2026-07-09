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

	/* TODO (Phase 2): parse a C5 detection frame (CMD_RID_DETECTION /
	 * CMD_RID_FORMATTED_MSG — see lib/meshcore_shared and the UART protocol
	 * used by nodes/remote_meshcore_node) received over the C5 UART, build
	 * the Identity/Telemetry TXT_MSG text, and send it via
	 * BaseChatMesh::sendMessage(drone_base_contact, ...), tracking the
	 * expected ACK with addPendingAck(). Left unimplemented for this
	 * build-and-boot skeleton — see main_uart_companion.cpp's
	 * process_c5_uart() for the current byte-drain stub.
	 */
	// void sendDetectionToBase(const ContactInfo &drone_base, const char *text);

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
};
