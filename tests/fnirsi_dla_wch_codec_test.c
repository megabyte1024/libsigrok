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
 * Standalone test of the fnirsi-dla driver's WCH request codec. It needs no
 * libsigrok build and no Windows; from the source root:
 *
 *   gcc -Wall -Wextra -I src/hardware/fnirsi-dla \
 *       tests/fnirsi_dla_wch_codec_test.c \
 *       src/hardware/fnirsi-dla/wch_codec.c -o wch_codec_test
 *   ./wch_codec_test
 */

#include <stdio.h>
#include <string.h>
#include "wch_codec.h"

static int failures;

#define CHECK(cond) do { \
	if (!(cond)) { \
		failures++; \
		printf("FAILED line %d: %s\n", __LINE__, #cond); \
	} \
} while (0)

static uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
		(uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void test_headers(void)
{
	uint8_t h[DLA_WCH_HEADER_BYTES];

	/* Read from 0x81: function (endpoint number - 1) | 0x10000. */
	CHECK(dla_wch_read_header(0x81, 1024 * 1024, h) == 0);
	CHECK(le32(h) == 0x10000);
	CHECK(le32(h + 4) == 1024 * 1024);
	CHECK(dla_wch_read_header(0x82, 512, h) == 0);
	CHECK(le32(h) == 0x10001);
	/* Write to 0x02: (2 - 1) | 0x20000. */
	CHECK(dla_wch_write_header(0x02, 512, h) == 0);
	CHECK(le32(h) == 0x20001);
	CHECK(le32(h + 4) == 512);

	/* Wrong direction, endpoint 0 or 9, empty or oversized: refused. */
	CHECK(dla_wch_read_header(0x02, 512, h) < 0);
	CHECK(dla_wch_write_header(0x82, 512, h) < 0);
	CHECK(dla_wch_read_header(0x80, 512, h) < 0);
	CHECK(dla_wch_read_header(0x89, 512, h) < 0);
	CHECK(dla_wch_read_header(0x81, 0, h) < 0);
	CHECK(dla_wch_read_header(0x81, DLA_WCH_MAX_TRANSFER_BYTES, h) == 0);
	CHECK(dla_wch_read_header(0x81, DLA_WCH_MAX_TRANSFER_BYTES + 1, h) < 0);
}

static void test_reply_length(void)
{
	uint8_t reply[DLA_WCH_HEADER_BYTES + 16];
	uint32_t n;

	memset(reply, 0, sizeof(reply));
	reply[4] = 12;
	CHECK(dla_wch_reply_length(reply, sizeof(reply), 16, &n) == 0);
	CHECK(n == 12);
	/* More than asked for, more than the reply holds, or no header. */
	CHECK(dla_wch_reply_length(reply, sizeof(reply), 8, &n) < 0);
	reply[4] = 17;
	CHECK(dla_wch_reply_length(reply, sizeof(reply), 32, &n) < 0);
	CHECK(dla_wch_reply_length(reply, 7, 32, &n) < 0);
}

static void test_settings(void)
{
	uint8_t r[DLA_WCH_SETTING_BYTES];
	int i;

	dla_wch_timeouts_request(300000, r);
	CHECK(le32(r) == 0x0F);
	CHECK(le32(r + 4) == 16);
	for (i = 0; i < 4; i++)
		CHECK(le32(r + 8 + 4 * i) == 300000);
	dla_wch_exclusive_request(1, r);
	CHECK(le32(r) == 0x0B);
	CHECK(le32(r + 4) == 1);
	CHECK(r[8] == 1);
}

static void test_results(void)
{
	CHECK(dla_wch_result_of(0) == DLA_WCH_OK);
	CHECK(dla_wch_result_of(31) == DLA_WCH_HALTED);
	CHECK(dla_wch_result_of(121) == DLA_WCH_TIMEOUT);
	CHECK(dla_wch_result_of(1460) == DLA_WCH_TIMEOUT);
	CHECK(dla_wch_result_of(995) == DLA_WCH_CANCELLED);
	CHECK(dla_wch_result_of(2) == DLA_WCH_NO_DEVICE);
	CHECK(dla_wch_result_of(1167) == DLA_WCH_NO_DEVICE);
	CHECK(dla_wch_result_of(32) == DLA_WCH_BUSY);
	CHECK(dla_wch_result_of(5) == DLA_WCH_ACCESS);
	CHECK(dla_wch_result_of(87) == DLA_WCH_OTHER);
	CHECK(strcmp(dla_wch_result_name(DLA_WCH_HALTED),
		"device halted endpoint") == 0);
}

static void test_matching(void)
{
	uint16_t vid, pid;
	const uint32_t device_first[] = { 15, 3 };
	const uint32_t too_deep[] = { 1, 1, 1, 1, 1, 1, 1, 1 };
	const uint32_t zero[] = { 15, 0 };
	uint8_t ports[DLA_WCH_MAX_PORT_DEPTH];

	CHECK(dla_wch_is_wch_service("CH375_A64"));
	CHECK(dla_wch_is_wch_service("ch375"));
	CHECK(dla_wch_is_wch_service("CH375_M64"));
	CHECK(!dla_wch_is_wch_service("WinUSB"));
	CHECK(!dla_wch_is_wch_service("CH375_A65"));
	CHECK(!dla_wch_is_wch_service(""));

	CHECK(dla_wch_parse_instance_id("USB\\VID_1A86&PID_5537\\0123456789",
		&vid, &pid) == 0);
	CHECK(vid == 0x1a86 && pid == 0x5537);
	CHECK(dla_wch_parse_instance_id("usb\\vid_1a86&pid_5537\\5&2&0&3",
		&vid, &pid) == 0);
	CHECK(dla_wch_parse_instance_id("USB\\VID_1A86&PID_5537&MI_00\\x",
		&vid, &pid) < 0);
	CHECK(dla_wch_parse_instance_id("HID\\VID_1A86&PID_5537\\x",
		&vid, &pid) < 0);
	CHECK(dla_wch_parse_instance_id("USB\\VID_1A86&PID_5537",
		&vid, &pid) < 0);

	CHECK(dla_wch_is_root_hub("USB\\ROOT_HUB30\\4&1&0&0"));
	CHECK(!dla_wch_is_root_hub("USB\\VID_1A86&PID_5537\\0123456789"));

	/* Windows gives the device's own port first; libusb root first. */
	CHECK(dla_wch_port_path(device_first, 2, ports, sizeof(ports)) == 2);
	CHECK(ports[0] == 3 && ports[1] == 15);
	CHECK(dla_wch_port_path(too_deep, 8, ports, sizeof(ports)) < 0);
	CHECK(dla_wch_port_path(zero, 2, ports, sizeof(ports)) < 0);
	CHECK(dla_wch_port_path(device_first, 0, ports, sizeof(ports)) < 0);
}

int main(void)
{
	test_headers();
	test_reply_length();
	test_settings();
	test_results();
	test_matching();
	if (failures) {
		printf("%d check(s) failed\n", failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
