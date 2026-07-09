/*
 * SPDX-License-Identifier: MIT
 *
 * rid_odid — trimmed, Arduino-free port of the top-level repo's
 * lib/meshcore_shared/src/meshcore_shared.h `id_data` struct + the ODID->
 * id_data accumulation path (odid_parse.h's parseBLEODIDMessage()) + the two
 * MeshCore TXT_MSG binary formatters (meshcore_shared.cpp's
 * meshcore_format_identity_msg / meshcore_format_telemetry_msg).
 *
 * WHY THIS EXISTS RATHER THAN INCLUDING THE ORIGINAL HEADERS DIRECTLY:
 * lib/meshcore_shared/src/meshcore_shared.h unconditionally includes
 * <Arduino.h> + <HardwareSerial.h> (it's written for the PlatformIO/Arduino
 * build of nodes/remote_meshcore_node and nodes/nrf52_meshcore_node), which
 * doesn't exist in this Zephyr build. The `id_data` struct itself and the
 * BasicID/Location decode + binary-format logic have no Arduino dependency
 * at all (confirmed by reading meshcore_shared.cpp — the formatters only use
 * memcpy/memset/strlen), so they're hand-ported here verbatim rather than
 * hand-rolled from scratch. This mirrors the project's own established
 * pattern for this exact situation — see the C5 UART wire-protocol block
 * comment in ../../src/main_uart_companion.cpp ("hand-ported here since that
 * header lives in the Arduino/PlatformIO build tree").
 *
 * KEEP IN SYNC with lib/meshcore_shared/src/meshcore_shared.h (id_data struct
 * layout + MESHCORE_MSG_VERSION), lib/meshcore_shared/src/meshcore_shared.cpp
 * (the two formatters + their byte layout), and
 * lib/meshcore_shared/src/odid_parse.h (parseBLEODIDMessage) if any of those
 * ever change — the wire format these produce is consumed by the same
 * drone-base receiver as the Arduino carrier's Identity/Telemetry messages,
 * so a drift here is a silent wire-format bug, not just a build error.
 *
 * opendroneid.h/.c (this directory) are copied verbatim from
 * nodes/remote_meshcore_node/src/opendroneid.{h,c} — that library is
 * portable C (stdint/string/math/stdio only), so no porting was needed there.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "opendroneid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * WiFi/BLE band enum — mirrors lib/meshcore_shared/src/meshcore_shared.h
 * WiFiBand exactly (values are wire-significant: meshcore_format_*_msg packs
 * `band & 0x03` into the binary payload; a receiver-side mismatch would
 * silently mislabel the detection source).
 * ==========================================================================*/
enum RidOdidBand {
	RID_BAND_UNKNOWN = 0,
	RID_BAND_2_4GHZ  = 1,
	RID_BAND_5GHZ    = 2,
	RID_BAND_BLE     = 3
};

/* ============================================================================
 * id_data — verbatim field-for-field port of lib/meshcore_shared's id_data.
 * Only the type of the `band` field changes (WiFiBand -> RidOdidBand, same
 * underlying values) since the Arduino enum isn't available here.
 * ==========================================================================*/
typedef struct rid_id_data {
	uint8_t  mac[6];
	int      rssi;
	uint32_t last_seen;
	/* Identification (static/slow-changing, sent in Identity message) */
	char     uav_id[ODID_ID_SIZE + 1];   /* BasicID[0].UASID */
	uint8_t  id_type;                     /* BasicID[0].IDType */
	uint8_t  ua_type;                     /* BasicID[0].UAType */
	char     uav_id2[ODID_ID_SIZE + 1];  /* BasicID[1].UASID */
	uint8_t  id_type2;                    /* BasicID[1].IDType */
	char     op_id[ODID_ID_SIZE + 1];    /* OperatorID.OperatorId */
	uint8_t  op_id_type;                 /* OperatorID.OperatorIdType */
	char     self_id[ODID_STR_SIZE + 1]; /* SelfID.Desc */
	uint8_t  self_id_desc_type;          /* SelfID.DescType */
	uint8_t  op_loc_type;                /* System.OperatorLocationType */
	uint16_t area_count;                 /* System.AreaCount */
	uint16_t area_radius;                /* System.AreaRadius (meters) */
	/* Telemetry (fast-changing, sent in Telemetry message) */
	double   lat_d;
	double   long_d;
	double   base_lat_d;
	double   base_long_d;
	int      altitude_geo;
	int      altitude_baro;
	int      height_agl;
	int      speed;
	int      heading;
	uint8_t  status;
	int      speed_vertical;
	uint8_t  height_type;
	float    loc_timestamp;
	uint8_t  horiz_accuracy;
	uint8_t  vert_accuracy;
	uint8_t  baro_accuracy;
	uint8_t  speed_accuracy;
	uint8_t  ts_accuracy;
	int      operator_alt;
	int      flag;
	/* Extended System fields */
	int      classification_type;
	int      area_ceiling;
	int      area_floor;
	uint32_t system_timestamp;
	/* Authentication data */
	uint8_t  auth_type;
	uint8_t  auth_page;
	uint8_t  auth_length;
	uint32_t auth_timestamp;
	char     auth_data[ODID_AUTH_PAGE_NONZERO_DATA_SIZE + 1];
	/* Band/channel tracking */
	uint8_t  band;      /* RidOdidBand value */
	uint8_t  channel;
	uint8_t  ble_phy;
} rid_id_data;

/* ============================================================================
 * ODID -> id_data accumulation — ported from
 * lib/meshcore_shared/src/odid_parse.h parseBLEODIDMessage(). Only the BLE
 * per-message-type path is ported (this role has no WiFi/NAN scanner), and
 * the `id_data`/opendroneid types are the local ones above instead of the
 * Arduino build's. Logic is otherwise unchanged — same field-for-field
 * assignment, same "first BasicID populates uav_id, a differing UASID goes
 * to uav_id2" dedup rule.
 * ==========================================================================*/
static inline void rid_odid_parse_ble_message(rid_id_data *uav, const uint8_t *odid)
{
	switch (odid[0] & 0xF0) {
	case 0x00: {
		ODID_BasicID_data basic;
		decodeBasicIDMessage(&basic, (ODID_BasicID_encoded *)odid);
		if (uav->uav_id[0] == '\0') {
			strncpy(uav->uav_id, (char *)basic.UASID, ODID_ID_SIZE);
			uav->uav_id[ODID_ID_SIZE] = '\0';
			uav->id_type = basic.IDType;
			uav->ua_type = basic.UAType;
		} else if (strncmp(uav->uav_id, (char *)basic.UASID, ODID_ID_SIZE) != 0) {
			strncpy(uav->uav_id2, (char *)basic.UASID, ODID_ID_SIZE);
			uav->uav_id2[ODID_ID_SIZE] = '\0';
			uav->id_type2 = basic.IDType;
		}
		break;
	}
	case 0x10: {
		ODID_Location_data loc;
		decodeLocationMessage(&loc, (ODID_Location_encoded *)odid);
		uav->lat_d = loc.Latitude;
		uav->long_d = loc.Longitude;
		uav->altitude_geo = (int)loc.AltitudeGeo;
		uav->altitude_baro = (int)loc.AltitudeBaro;
		uav->height_agl = (int)loc.Height;
		uav->speed = (int)loc.SpeedHorizontal;
		uav->heading = (int)loc.Direction;
		uav->status = loc.Status;
		uav->speed_vertical = (int)loc.SpeedVertical;
		uav->height_type = loc.HeightType;
		uav->loc_timestamp = loc.TimeStamp;
		uav->horiz_accuracy = loc.HorizAccuracy;
		uav->vert_accuracy = loc.VertAccuracy;
		uav->baro_accuracy = loc.BaroAccuracy;
		uav->speed_accuracy = loc.SpeedAccuracy;
		uav->ts_accuracy = loc.TSAccuracy;
		break;
	}
	case 0x20: {
		ODID_Auth_data auth;
		decodeAuthMessage(&auth, (ODID_Auth_encoded *)odid);
		if (auth.DataPage == 0) {
			uav->auth_type = auth.AuthType;
			uav->auth_page = auth.DataPage;
			uav->auth_length = auth.Length;
			uav->auth_timestamp = auth.Timestamp;
			size_t cpy = auth.Length;
			if (cpy > sizeof(uav->auth_data) - 1) cpy = sizeof(uav->auth_data) - 1;
			memcpy(uav->auth_data, auth.AuthData, cpy);
			memset(uav->auth_data + cpy, 0, sizeof(uav->auth_data) - cpy);
		}
		break;
	}
	case 0x30: {
		ODID_SelfID_data self;
		decodeSelfIDMessage(&self, (ODID_SelfID_encoded *)odid);
		strncpy(uav->self_id, (char *)self.Desc, ODID_STR_SIZE);
		uav->self_id[ODID_STR_SIZE] = '\0';
		uav->self_id_desc_type = self.DescType;
		break;
	}
	case 0x40: {
		ODID_System_data sys;
		decodeSystemMessage(&sys, (ODID_System_encoded *)odid);
		uav->base_lat_d = sys.OperatorLatitude;
		uav->base_long_d = sys.OperatorLongitude;
		uav->op_loc_type = sys.OperatorLocationType;
		uav->classification_type = sys.ClassificationType;
		uav->area_count = sys.AreaCount;
		uav->area_radius = sys.AreaRadius;
		uav->area_ceiling = (int)sys.AreaCeiling;
		uav->area_floor = (int)sys.AreaFloor;
		uav->operator_alt = (int)sys.OperatorAltitudeGeo;
		uav->system_timestamp = sys.Timestamp;
		break;
	}
	case 0x50: {
		ODID_OperatorID_data op;
		decodeOperatorIDMessage(&op, (ODID_OperatorID_encoded *)odid);
		strncpy(uav->op_id, (char *)op.OperatorId, ODID_ID_SIZE);
		uav->op_id[ODID_ID_SIZE] = '\0';
		uav->op_id_type = op.OperatorIdType;
		break;
	}
	default:
		break;
	}
}

/* ============================================================================
 * MeshCore TXT_MSG binary formatters — implemented in rid_odid_format.cpp,
 * ported verbatim (byte layout unchanged) from
 * lib/meshcore_shared/src/meshcore_shared.cpp's meshcore_format_telemetry_msg
 * / meshcore_format_identity_msg. Returns the same 'T'/'I'-prefixed base64
 * text a real Arduino carrier or C5-offloaded formatter would produce for
 * the same id_data — a drone-base receiver cannot tell the two apart.
 * ==========================================================================*/
int rid_odid_format_telemetry_msg(const rid_id_data *uav, char *buf, size_t bufSize);
int rid_odid_format_identity_msg(const rid_id_data *uav, char *buf, size_t bufSize);

#ifdef __cplusplus
}
#endif
