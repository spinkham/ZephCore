/*
 * SPDX-License-Identifier: MIT
 *
 * rid_odid_format — ported verbatim (byte layout unchanged) from
 * lib/meshcore_shared/src/meshcore_shared.cpp's meshcore_format_telemetry_msg
 * / meshcore_format_identity_msg + their local base64/little-endian helpers.
 * See rid_odid.h for why this is a hand-port rather than a #include of the
 * original (Arduino.h dependency in the shared header, not in these
 * functions themselves).
 *
 * RID_ODID_MSG_VERSION must track MESHCORE_MSG_VERSION
 * (lib/meshcore_shared/src/meshcore_shared.h) exactly — it's the wire
 * version byte every receiver (including the drone-base) hard-drops on
 * mismatch. Both are 0 today.
 */

#include "rid_odid.h"

#include <string.h>

#define RID_ODID_MSG_VERSION 0

namespace {

const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int base64_encode(const uint8_t *input, size_t len, char *output)
{
	int out = 0;
	for (size_t i = 0; i < len; i += 3) {
		uint32_t v = (uint32_t)input[i] << 16;
		if (i + 1 < len) v |= (uint32_t)input[i + 1] << 8;
		if (i + 2 < len) v |= (uint32_t)input[i + 2];
		output[out++] = b64_table[(v >> 18) & 0x3F];
		output[out++] = b64_table[(v >> 12) & 0x3F];
		output[out++] = (i + 1 < len) ? b64_table[(v >> 6) & 0x3F] : '=';
		output[out++] = (i + 2 < len) ? b64_table[v & 0x3F] : '=';
	}
	output[out] = '\0';
	return out;
}

inline void write_u16_le(uint8_t *buf, uint16_t val)
{
	buf[0] = val & 0xFF;
	buf[1] = (val >> 8) & 0xFF;
}

inline void write_i16_le(uint8_t *buf, int16_t val) { write_u16_le(buf, (uint16_t)val); }

inline void write_u32_le(uint8_t *buf, uint32_t val)
{
	buf[0] = val & 0xFF;
	buf[1] = (val >> 8) & 0xFF;
	buf[2] = (val >> 16) & 0xFF;
	buf[3] = (val >> 24) & 0xFF;
}

inline int clamp_i(int val, int lo, int hi) { return val < lo ? lo : (val > hi ? hi : val); }

} /* anonymous namespace */

int rid_odid_format_telemetry_msg(const rid_id_data *uav, char *buf, size_t bufSize)
{
	if (bufSize < 66) return 0; /* 1 prefix + 64 base64 chars + null */
	uint8_t raw[47];
	memset(raw, 0, sizeof(raw));

	raw[0] = RID_ODID_MSG_VERSION;
	memcpy(&raw[1], uav->mac, 6);
	raw[7] = (int8_t)uav->rssi;
	write_u32_le(&raw[8], (uint32_t)((uav->lat_d + 90.0) * 1e7));
	write_u32_le(&raw[12], (uint32_t)((uav->long_d + 180.0) * 1e7));
	write_i16_le(&raw[16], (int16_t)clamp_i((uav->altitude_geo + 1000) * 2, -32768, 32767));
	raw[18] = (uint8_t)clamp_i(uav->speed * 4, 0, 255);
	write_u16_le(&raw[19], (uint16_t)uav->heading);
	raw[21] = uav->status;
	raw[22] = (int8_t)clamp_i(uav->speed_vertical, -128, 127);
	write_i16_le(&raw[23], (int16_t)clamp_i((uav->height_agl + 1000) * 2, -32768, 32767));
	write_i16_le(&raw[25], (int16_t)clamp_i((uav->altitude_baro + 1000) * 2, -32768, 32767));
	raw[27] = uav->height_type;
	write_u16_le(&raw[28], (uint16_t)(uav->loc_timestamp * 10));
	raw[30] = uav->horiz_accuracy;
	raw[31] = uav->vert_accuracy;
	raw[32] = uav->baro_accuracy;
	raw[33] = uav->speed_accuracy;
	raw[34] = uav->ts_accuracy;
	write_i16_le(&raw[35], (int16_t)clamp_i((uav->operator_alt + 1000) * 2, -32768, 32767));
	write_u32_le(&raw[37], (uint32_t)((uav->base_lat_d + 90.0) * 1e7));
	write_u32_le(&raw[41], (uint32_t)((uav->base_long_d + 180.0) * 1e7));
	/* Packed band+phy / channel — see meshcore_shared.h "Wire Format" notes. */
	raw[45] = ((uint8_t)uav->band & 0x03) | ((uav->ble_phy & 0x03) << 2);
	raw[46] = uav->channel;

	buf[0] = 'T';
	int len = 1 + base64_encode(raw, sizeof(raw), &buf[1]);
	return len;
}

int rid_odid_format_identity_msg(const rid_id_data *uav, char *buf, size_t bufSize)
{
	if (bufSize < 146) return 0; /* 1 prefix + max ~144 base64 chars + null */
	uint8_t raw[130];
	memset(raw, 0, sizeof(raw));
	int pos = 0;

	raw[pos++] = RID_ODID_MSG_VERSION;
	memcpy(&raw[pos], uav->mac, 6); pos += 6;
	raw[pos++] = uav->ua_type;
	raw[pos++] = uav->id_type;
	raw[pos++] = ((uint8_t)uav->band & 0x03) | ((uav->ble_phy & 0x03) << 2);
	raw[pos++] = uav->channel;

	uint8_t len1 = (uint8_t)strlen(uav->uav_id);
	raw[pos++] = len1;
	if (pos + len1 > (int)sizeof(raw)) return 0;
	memcpy(&raw[pos], uav->uav_id, len1); pos += len1;

	raw[pos++] = uav->id_type2;
	uint8_t len2 = (uint8_t)strlen(uav->uav_id2);
	raw[pos++] = len2;
	if (pos + len2 > (int)sizeof(raw)) return 0;
	memcpy(&raw[pos], uav->uav_id2, len2); pos += len2;

	raw[pos++] = uav->op_id_type;
	uint8_t lenOp = (uint8_t)strlen(uav->op_id);
	raw[pos++] = lenOp;
	if (pos + lenOp > (int)sizeof(raw)) return 0;
	memcpy(&raw[pos], uav->op_id, lenOp); pos += lenOp;

	raw[pos++] = uav->self_id_desc_type;
	raw[pos++] = uav->op_loc_type;

	uint8_t lenSelf = (uint8_t)strlen(uav->self_id);
	raw[pos++] = lenSelf;
	if (pos + lenSelf > (int)sizeof(raw)) return 0;
	memcpy(&raw[pos], uav->self_id, lenSelf); pos += lenSelf;

	if (pos + 4 > (int)sizeof(raw)) return 0;
	write_u16_le(&raw[pos], uav->area_count); pos += 2;
	write_u16_le(&raw[pos], uav->area_radius); pos += 2;

	bool hasExtended = uav->classification_type != 0 || uav->area_ceiling != 0 ||
			   uav->area_floor != 0 || uav->system_timestamp != 0;
	int extendedStart = pos;
	if (hasExtended && pos + 9 <= (int)sizeof(raw)) {
		raw[pos++] = (uint8_t)uav->classification_type;
		write_i16_le(&raw[pos], (int16_t)clamp_i((uav->area_ceiling + 1000) * 2, -32768, 32767));
		pos += 2;
		write_i16_le(&raw[pos], (int16_t)clamp_i((uav->area_floor + 1000) * 2, -32768, 32767));
		pos += 2;
		write_u32_le(&raw[pos], uav->system_timestamp); pos += 4;
	}

	int b64Len = ((pos + 2) / 3) * 4;
	if (1 + b64Len > 151 && hasExtended) {
		pos = extendedStart;
	}

	buf[0] = 'I';
	int outLen = 1 + base64_encode(raw, pos, &buf[1]);
	return outLen;
}
