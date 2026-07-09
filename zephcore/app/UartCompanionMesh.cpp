/*
 * SPDX-License-Identifier: MIT
 * UartCompanionMesh implementation
 *
 * See UartCompanionMesh.h for the role this class plays. Each BaseChatMesh
 * hook below is implemented minimally (log + safe default) for the Phase 2
 * bring-up skeleton — app/CompanionMesh.cpp is the reference for the fuller
 * behavior each of these would eventually need.
 */

#include "UartCompanionMesh.h"
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_uart_companion, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

UartCompanionMesh::UartCompanionMesh(mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
	mesh::RTCClock &rtc, mesh::PacketManager &mgr, mesh::MeshTables &tables)
	: BaseChatMesh(radio, ms, rng, rtc, mgr, tables)
{
	initNodePrefs(&prefs);
	memset(_ack_table, 0, sizeof(_ack_table));
	_ack_next_overwrite = 0;
	memset(&_ack_stats, 0, sizeof(_ack_stats));
	memset(_drone_base_pubkey, 0, sizeof(_drone_base_pubkey));
	_drone_base_pubkey_set = false;
}

void UartCompanionMesh::setDroneBaseContact(const uint8_t pubkey[PUB_KEY_SIZE])
{
	memcpy(_drone_base_pubkey, pubkey, PUB_KEY_SIZE);
	_drone_base_pubkey_set = true;
}

bool UartCompanionMesh::sendDetectionToBase(const char *text)
{
	if (!_drone_base_pubkey_set) {
		LOG_WRN("sendDetectionToBase: drone-base contact not configured");
		return false;
	}

	ContactInfo *contact = lookupContactByPubKey(_drone_base_pubkey, PUB_KEY_SIZE);
	if (!contact) {
		LOG_WRN("sendDetectionToBase: drone-base contact not found");
		return false;
	}

	uint32_t expected_ack = 0, est_timeout = 0;
	uint32_t now = getRTCClock()->getCurrentTimeUnique();
	int result = sendMessage(*contact, now, /*attempt*/ 1, text, expected_ack, est_timeout);
	if (result == MSG_SEND_FAILED) {
		LOG_WRN("sendDetectionToBase: sendMessage failed for '%s'", contact->name);
		return false;
	}

	_ack_stats.attempted++;

	if (expected_ack) {
		/* Resolve the contact's slot index the same way CompanionMesh's
		 * CMD_SEND_TXT_MSG handler does, since sendMessage() only hands back
		 * the ContactInfo, not its index. */
		int idx = -1;
		ContactInfo ci;
		for (int k = 0; k < getNumContacts(); k++) {
			if (getContactByIdx(k, ci) && ci.id.matches(contact->id)) {
				idx = k;
				break;
			}
		}
		addPendingSend(text, /*attempt=*/1, expected_ack, idx, futureMillis((int)est_timeout));
	}

	LOG_INF("sendDetectionToBase: sent to '%s' result=%d expected_ack=0x%08x est_timeout=%u",
		contact->name, result, expected_ack, est_timeout);
	return true;
}

void UartCompanionMesh::addPendingSend(const char *text, uint8_t attempt, uint32_t expected,
	int contact_idx, uint32_t deadline_ms)
{
	int slot = -1;
	for (int i = 0; i < UART_COMPANION_ACK_TABLE_SIZE; i++) {
		if (!_ack_table[i].active) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		/* Table full — circular overwrite (matches CompanionMesh). */
		slot = _ack_next_overwrite;
		_ack_next_overwrite = (_ack_next_overwrite + 1) % UART_COMPANION_ACK_TABLE_SIZE;
	}

	PendingSend &p = _ack_table[slot];
	strncpy(p.text, text, sizeof(p.text) - 1);
	p.text[sizeof(p.text) - 1] = '\0';
	p.attempt = attempt;
	p.expected_ack = expected;
	p.contact_idx = contact_idx;
	p.deadline_ms = deadline_ms;
	p.active = true;
}

int UartCompanionMesh::findAndRemoveAck(uint32_t ack)
{
	for (int i = 0; i < UART_COMPANION_ACK_TABLE_SIZE; i++) {
		if (_ack_table[i].active && _ack_table[i].expected_ack == ack) {
			_ack_table[i].active = false;
			return _ack_table[i].contact_idx;
		}
	}
	return -1;
}

void UartCompanionMesh::checkTimeouts()
{
	for (int i = 0; i < UART_COMPANION_ACK_TABLE_SIZE; i++) {
		PendingSend &p = _ack_table[i];
		if (!p.active || !millisHasNowPassed(p.deadline_ms)) {
			continue;
		}

		if (p.attempt >= UART_COMPANION_RETRY_LIMIT) {
			LOG_WRN("checkTimeouts: retry budget exhausted (%u/%u attempts), "
				"marking undelivered: '%.32s'",
				(unsigned)p.attempt, (unsigned)UART_COMPANION_RETRY_LIMIT, p.text);
			p.active = false;
			_ack_stats.undelivered++;
			continue;
		}

		ContactInfo ci;
		if (p.contact_idx < 0 || !getContactByIdx((uint32_t)p.contact_idx, ci)) {
			LOG_WRN("checkTimeouts: contact_idx=%d no longer valid, dropping pending send",
				p.contact_idx);
			p.active = false;
			_ack_stats.undelivered++;
			continue;
		}

		uint8_t next_attempt = p.attempt + 1;
		uint32_t now_ts = getRTCClock()->getCurrentTimeUnique();
		uint32_t expected_ack = 0, est_timeout = 0;
		int result = sendMessage(ci, now_ts, next_attempt, p.text, expected_ack, est_timeout);
		if (result == MSG_SEND_FAILED) {
			LOG_WRN("checkTimeouts: retry send failed for '%s' attempt=%u, will retry next tick",
				ci.name, (unsigned)next_attempt);
			/* Leave attempt/table state as-is; push the deadline out a bit
			 * so we don't spin every event-loop iteration, and try again
			 * on the next housekeeping tick. */
			p.deadline_ms = futureMillis(1000);
			continue;
		}

		p.attempt = next_attempt;
		p.expected_ack = expected_ack;
		p.deadline_ms = futureMillis((int)est_timeout);
		_ack_stats.retries++;
		LOG_INF("checkTimeouts: retry %u/%u for '%s' expected_ack=0x%08x est_timeout=%u",
			(unsigned)p.attempt, (unsigned)UART_COMPANION_RETRY_LIMIT, ci.name,
			expected_ack, est_timeout);
	}
}

void UartCompanionMesh::getAckStats(AckStats *out) const
{
	if (out) {
		*out = _ack_stats;
	}
}

int UartCompanionMesh::countActivePending() const
{
	int n = 0;
	for (int i = 0; i < UART_COMPANION_ACK_TABLE_SIZE; i++) {
		if (_ack_table[i].active) {
			n++;
		}
	}
	return n;
}

bool UartCompanionMesh::sendSelfAdvert(bool flood)
{
	mesh::Packet *adv = createSelfAdvert(prefs.node_name);
	if (!adv) {
		return false;
	}
	if (flood) {
		sendFlood(adv);
	} else {
		sendZeroHop(adv);
	}
	return true;
}

void UartCompanionMesh::onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path)
{
	ARG_UNUSED(path_len);
	ARG_UNUSED(path);
	LOG_INF("onDiscoveredContact: '%s' is_new=%d num_contacts=%d",
		contact.name, (int)is_new, getNumContacts());
	/* TODO (Phase 2): today only the pinned drone-base contact (added at
	 * boot in main_uart_companion.cpp) is used, so adverts from other
	 * nodes are just logged. If this role ever needs to react to a
	 * fresh drone-base advert (e.g. to learn its path faster), do it
	 * here. */
}

ContactInfo *UartCompanionMesh::processAck(const uint8_t *data)
{
	uint32_t ack_crc;
	memcpy(&ack_crc, data, 4);

	int contact_idx = findAndRemoveAck(ack_crc);
	if (contact_idx < 0) {
		return nullptr;
	}
	_ack_stats.delivered++;
	ContactInfo ci;
	if (!getContactByIdx(contact_idx, ci)) {
		return nullptr;
	}
	LOG_INF("processAck: delivery confirmed for '%s'", ci.name);
	return lookupContactByPubKey(ci.id.pub_key, PUB_KEY_SIZE);
}

void UartCompanionMesh::onContactPathUpdated(const ContactInfo &contact)
{
	/* BaseChatMesh::onContactPathRecv() has already written the learned
	 * out_path/out_path_len/lastmod into the contacts[] slot before this
	 * hook fires — nothing else to persist for this skeleton (no flash
	 * store for contacts in this role). */
	LOG_INF("onContactPathUpdated: '%s' out_path_len=%d", contact.name, contact.out_path_len);
}

void UartCompanionMesh::onMessageRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const char *text)
{
	ARG_UNUSED(pkt);
	LOG_INF("onMessageRecv: from '%s' ts=%u text='%s'", contact.name, sender_timestamp, text);
}

void UartCompanionMesh::onCommandDataRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const char *text)
{
	ARG_UNUSED(pkt);
	LOG_INF("onCommandDataRecv: from '%s' ts=%u text='%s'", contact.name, sender_timestamp, text);
}

void UartCompanionMesh::onSignedMessageRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text)
{
	ARG_UNUSED(pkt);
	ARG_UNUSED(sender_prefix);
	LOG_INF("onSignedMessageRecv: from '%s' ts=%u text='%s'", contact.name, sender_timestamp, text);
}

uint32_t UartCompanionMesh::calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const
{
	/* Shared dispatcher timing formula — matches CompanionMesh/RepeaterMesh. */
	return 500 + (uint32_t)(16.0f * pkt_airtime_millis);
}

uint32_t UartCompanionMesh::calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const
{
	/* path_len is the packed hash-size-encoded byte (top 2 bits =
	 * hash_size-1, bottom 6 = hop count) — mask to the real hop count
	 * before scaling (see CompanionMesh::calcDirectTimeoutMillisFor). */
	uint8_t path_hash_count = path_len & 63;
	return 500 + (uint32_t)((pkt_airtime_millis * 6.0f + 250) * (path_hash_count + 1));
}

void UartCompanionMesh::onSendTimeout()
{
	/* BaseChatMesh's txt_send_timeout is a single scalar tracking only the
	 * MOST RECENT sendMessage() call, so it can't drive per-entry retry when
	 * multiple detection-relay sends are in flight. The actual retry/
	 * undelivered state machine lives in checkTimeouts(), driven from
	 * main_uart_companion.cpp's MESH_EVENT_HOUSEKEEPING handler on a fixed
	 * poll interval instead — same defense-in-depth-via-polling shape as the
	 * Arduino carrier's drone_cache_check_timeouts(). Nothing to do here. */
}

void UartCompanionMesh::onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt,
	uint32_t timestamp, const char *text)
{
	ARG_UNUSED(channel);
	ARG_UNUSED(pkt);
	LOG_INF("onChannelMessageRecv: ts=%u text='%s' (channels unused by this role)", timestamp, text);
}

uint8_t UartCompanionMesh::onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp,
	const uint8_t *data, uint8_t len, uint8_t *reply)
{
	ARG_UNUSED(contact);
	ARG_UNUSED(sender_timestamp);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	ARG_UNUSED(reply);
	return 0;  /* No requests (telemetry, etc.) served by this role. */
}

void UartCompanionMesh::onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len)
{
	ARG_UNUSED(data);
	LOG_INF("onContactResponse: from '%s' len=%d", contact.name, len);
}
