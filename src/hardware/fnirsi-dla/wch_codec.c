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

#include <ctype.h>
#include <string.h>
#include "wch_codec.h"

#define READ_FLAG		0x10000u
#define WRITE_FLAG		0x20000u
#define DIRECTION_IN		0x80
#define ENDPOINT_NUMBER_MASK	0x0F
#define FIRST_ENDPOINT		1
#define LAST_ENDPOINT		8
#define SET_EXCLUSIVE		0x0Bu
#define SET_TIMEOUTS		0x0Fu
#define TIMEOUT_COUNT		4

#define ERROR_FILE_NOT_FOUND		2
#define ERROR_ACCESS_DENIED		5
#define ERROR_GEN_FAILURE		31
#define ERROR_SHARING_VIOLATION		32
#define ERROR_SEM_TIMEOUT		121
#define ERROR_OPERATION_ABORTED		995
#define ERROR_DEVICE_NOT_CONNECTED	1167
#define ERROR_TIMEOUT			1460

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
		(uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static int header(uint8_t endpoint, int read, uint32_t length,
		uint8_t out[DLA_WCH_HEADER_BYTES])
{
	unsigned int number = endpoint & ENDPOINT_NUMBER_MASK;
	int in = (endpoint & DIRECTION_IN) != 0;

	if (number < FIRST_ENDPOINT || number > LAST_ENDPOINT)
		return -1;
	if (read != in)
		return -1;
	if (length == 0 || length > DLA_WCH_MAX_TRANSFER_BYTES)
		return -1;
	put32(out, (number - FIRST_ENDPOINT) | (read ? READ_FLAG : WRITE_FLAG));
	put32(out + 4, length);
	return 0;
}

int dla_wch_read_header(uint8_t endpoint, uint32_t length,
		uint8_t out[DLA_WCH_HEADER_BYTES])
{
	return header(endpoint, 1, length, out);
}

int dla_wch_write_header(uint8_t endpoint, uint32_t length,
		uint8_t out[DLA_WCH_HEADER_BYTES])
{
	return header(endpoint, 0, length, out);
}

int dla_wch_reply_length(const uint8_t *reply, size_t reply_size,
		uint32_t limit, uint32_t *length)
{
	uint32_t moved;

	if (reply_size < DLA_WCH_HEADER_BYTES)
		return -1;
	moved = get32(reply + 4);
	if (moved > limit || moved > reply_size - DLA_WCH_HEADER_BYTES)
		return -1;
	*length = moved;
	return 0;
}

void dla_wch_timeouts_request(uint32_t timeout_ms,
		uint8_t request[DLA_WCH_SETTING_BYTES])
{
	int i;

	memset(request, 0, DLA_WCH_SETTING_BYTES);
	put32(request, SET_TIMEOUTS);
	put32(request + 4, TIMEOUT_COUNT * 4);
	for (i = 0; i < TIMEOUT_COUNT; i++)
		put32(request + DLA_WCH_HEADER_BYTES + 4 * i, timeout_ms);
}

void dla_wch_exclusive_request(int exclusive,
		uint8_t request[DLA_WCH_SETTING_BYTES])
{
	memset(request, 0, DLA_WCH_SETTING_BYTES);
	put32(request, SET_EXCLUSIVE);
	put32(request + 4, 1);
	request[DLA_WCH_HEADER_BYTES] = exclusive ? 1 : 0;
}

enum dla_wch_result dla_wch_result_of(uint32_t windows_error)
{
	switch (windows_error) {
	case 0:
		return DLA_WCH_OK;
	case ERROR_GEN_FAILURE:
		return DLA_WCH_HALTED;
	case ERROR_SEM_TIMEOUT:
	case ERROR_TIMEOUT:
		return DLA_WCH_TIMEOUT;
	case ERROR_OPERATION_ABORTED:
		return DLA_WCH_CANCELLED;
	case ERROR_FILE_NOT_FOUND:
	case ERROR_DEVICE_NOT_CONNECTED:
		return DLA_WCH_NO_DEVICE;
	case ERROR_SHARING_VIOLATION:
		return DLA_WCH_BUSY;
	case ERROR_ACCESS_DENIED:
		return DLA_WCH_ACCESS;
	default:
		return DLA_WCH_OTHER;
	}
}

const char *dla_wch_result_name(enum dla_wch_result result)
{
	switch (result) {
	case DLA_WCH_OK:
		return "success";
	case DLA_WCH_TIMEOUT:
		return "timed out";
	case DLA_WCH_HALTED:
		return "device halted endpoint";
	case DLA_WCH_CANCELLED:
		return "request cancelled";
	case DLA_WCH_NO_DEVICE:
		return "device gone";
	case DLA_WCH_BUSY:
		return "device open in another program";
	case DLA_WCH_ACCESS:
		return "access denied";
	default:
		return "WCH driver error";
	}
}

static int same_ignoring_case(const char *a, const char *b, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++) {
		if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i]))
			return 0;
	}
	return 1;
}

int dla_wch_is_wch_service(const char *service)
{
	static const char *const services[] = { "ch375", "ch375_a64", "ch375_m64" };
	size_t i, len;

	len = strlen(service);
	for (i = 0; i < sizeof(services) / sizeof(services[0]); i++) {
		if (len == strlen(services[i]) &&
				same_ignoring_case(service, services[i], len))
			return 1;
	}
	return 0;
}

static int hex4(const char *p, uint16_t *value)
{
	unsigned int v = 0;
	int i;

	for (i = 0; i < 4; i++) {
		int c = toupper((unsigned char)p[i]);
		if (c >= '0' && c <= '9')
			v = v * 16 + (unsigned int)(c - '0');
		else if (c >= 'A' && c <= 'F')
			v = v * 16 + (unsigned int)(c - 'A' + 10);
		else
			return -1;
	}
	*value = (uint16_t)v;
	return 0;
}

int dla_wch_parse_instance_id(const char *instance_id,
		uint16_t *vid, uint16_t *pid)
{
	const char *first, *second;

	first = strchr(instance_id, '\\');
	if (!first || first - instance_id != 3 ||
			!same_ignoring_case(instance_id, "usb", 3))
		return -1;
	second = strchr(first + 1, '\\');
	if (!second || strchr(second + 1, '\\') || second[1] == '\0')
		return -1;
	/* VID_xxxx&PID_xxxx: exactly 17 characters, no interface suffix. */
	if (second - first - 1 != 17)
		return -1;
	if (!same_ignoring_case(first + 1, "VID_", 4) ||
			!same_ignoring_case(first + 9, "&PID_", 5))
		return -1;
	if (hex4(first + 5, vid) < 0 || hex4(first + 14, pid) < 0)
		return -1;
	return 0;
}

int dla_wch_is_root_hub(const char *instance_id)
{
	static const char prefix[] = "USB\\ROOT_HUB";

	return strlen(instance_id) >= sizeof(prefix) - 1 &&
		same_ignoring_case(instance_id, prefix, sizeof(prefix) - 1);
}

int dla_wch_port_path(const uint32_t *addresses_device_first,
		size_t count, uint8_t *ports, size_t max_ports)
{
	size_t i;

	if (count == 0 || count > DLA_WCH_MAX_PORT_DEPTH || count > max_ports)
		return -1;
	for (i = 0; i < count; i++) {
		uint32_t port = addresses_device_first[count - 1 - i];
		if (port == 0 || port > 255)
			return -1;
		ports[i] = (uint8_t)port;
	}
	return (int)count;
}
