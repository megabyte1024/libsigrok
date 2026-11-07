/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 sigrok contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The parts of WCH's Windows driver interface (CH375_A64, "USB CH372/CH375")
 * that need no Windows API: request headers, reply lengths, the meaning of
 * Windows error codes, and how a device found by libusb is matched to its
 * Windows device node. No libsigrok or Windows header is needed, so the
 * standalone test tests/fnirsi_dla_wch_codec_test.c builds it on any host.
 */

#ifndef LIBSIGROK_HARDWARE_FNIRSI_DLA_WCH_CODEC_H
#define LIBSIGROK_HARDWARE_FNIRSI_DLA_WCH_CODEC_H

#include <stddef.h>
#include <stdint.h>

#define DLA_WCH_HEADER_BYTES		8
#define DLA_WCH_MAX_TRANSFER_BYTES	0x400000u
#define DLA_WCH_SETTING_BYTES		72
#define DLA_WCH_MAX_PORT_DEPTH		7

enum dla_wch_result {
	DLA_WCH_OK,
	DLA_WCH_TIMEOUT,
	DLA_WCH_HALTED,
	DLA_WCH_CANCELLED,
	DLA_WCH_NO_DEVICE,
	DLA_WCH_BUSY,
	DLA_WCH_ACCESS,
	DLA_WCH_OTHER,
};

int dla_wch_read_header(uint8_t endpoint, uint32_t length,
		uint8_t header[DLA_WCH_HEADER_BYTES]);
int dla_wch_write_header(uint8_t endpoint, uint32_t length,
		uint8_t header[DLA_WCH_HEADER_BYTES]);
int dla_wch_reply_length(const uint8_t *reply, size_t reply_size,
		uint32_t limit, uint32_t *length);
void dla_wch_timeouts_request(uint32_t timeout_ms,
		uint8_t request[DLA_WCH_SETTING_BYTES]);
void dla_wch_exclusive_request(int exclusive,
		uint8_t request[DLA_WCH_SETTING_BYTES]);
enum dla_wch_result dla_wch_result_of(uint32_t windows_error);
const char *dla_wch_result_name(enum dla_wch_result result);

int dla_wch_is_wch_service(const char *service);
int dla_wch_parse_instance_id(const char *instance_id,
		uint16_t *vid, uint16_t *pid);
int dla_wch_is_root_hub(const char *instance_id);
int dla_wch_port_path(const uint32_t *addresses_device_first,
		size_t count, uint8_t *ports, size_t max_ports);

#endif
