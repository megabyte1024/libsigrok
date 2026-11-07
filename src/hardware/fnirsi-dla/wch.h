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
 * On Windows the DLA-16 is bound either to WinUSB (libusb opens it) or to
 * WCH's own driver (CH375_A64, "USB CH372/CH375", installed by the
 * manufacturer's application), which libusb cannot open. This is the path
 * through WCH's driver: its device is found by the port path libusb reports,
 * and spoken to with WCH's one IOCTL (see wch_codec.h). Everything here is
 * Windows only; elsewhere dla_wch_find_path() finds nothing.
 */

#ifndef LIBSIGROK_HARDWARE_FNIRSI_DLA_WCH_H
#define LIBSIGROK_HARDWARE_FNIRSI_DLA_WCH_H

#include <stdint.h>
#include "wch_codec.h"

struct dla_wch;
struct dla_wch_stream;

/* WCH's device path for the USB device at this place if Windows has bound
 * WCH's driver to it, else NULL. g_free() the result. */
char *dla_wch_find_path(uint16_t vid, uint16_t pid,
		const uint8_t *ports, int num_ports);

enum dla_wch_result dla_wch_open(const char *path, struct dla_wch **wch);
void dla_wch_close(struct dla_wch *wch);

/* A bulk transfer with libusb_bulk_transfer()'s meaning: *actual bytes
 * moved; a read of nothing or a short write is DLA_WCH_TIMEOUT. */
enum dla_wch_result dla_wch_bulk(struct dla_wch *wch, uint8_t endpoint,
		uint8_t *data, int length, int *actual, unsigned int timeout_ms);

/* Reads queued on one IN endpoint, handed back in the order they were
 * queued. While a stream exists, commands keep the driver's long stream
 * timeout. */
struct dla_wch_stream *dla_wch_stream_new(struct dla_wch *wch,
		uint8_t endpoint, unsigned int slots, size_t bytes);
enum dla_wch_result dla_wch_stream_submit(struct dla_wch_stream *stream,
		unsigned int slot);
/* The oldest queued read, if it is done within wait_ms: returns 1 and
 * fills slot, result, data and length. */
int dla_wch_stream_next(struct dla_wch_stream *stream,
		unsigned int wait_ms, unsigned int *slot,
		enum dla_wch_result *result, const uint8_t **data,
		size_t *length);
unsigned int dla_wch_stream_queued(const struct dla_wch_stream *stream);
void dla_wch_stream_cancel(struct dla_wch_stream *stream);
/* Frees the stream; a read the driver never gives back keeps its memory
 * for good, since the driver may still write into it. */
void dla_wch_stream_free(struct dla_wch_stream *stream);

#endif
