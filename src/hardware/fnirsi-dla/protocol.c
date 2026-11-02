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

#include <config.h>
#include <limits.h>
#include "protocol.h"

/* Rate table (see struct fnirsi_dla_rate in protocol.h). Index 0x0F is
 * invalid (falls back to ~40MHz) and is deliberately omitted here. */
SR_PRIV const struct fnirsi_dla_rate fnirsi_dla_rates[] = {
	{ SR_MHZ(1),    0x00 },
	{ SR_MHZ(2),    0x01 },
	{ SR_MHZ(4),    0x02 },
	{ SR_MHZ(5),    0x03 },
	{ SR_MHZ(10),   0x04 },
	{ SR_MHZ(20),   0x05 },
	{ SR_MHZ(25),   0x06 },
	{ SR_MHZ(40),   0x07 },
	{ SR_MHZ(50),   0x08 },
	{ SR_MHZ(100),  0x09 },
	{ SR_MHZ(125),  0x0a },
	{ SR_MHZ(200),  0x0b },
	{ SR_MHZ(250),  0x0c },
	{ SR_MHZ(500),  0x0d },
	{ SR_MHZ(1000), 0x0e },
};
SR_PRIV const unsigned int fnirsi_dla_num_rates = ARRAY_SIZE(fnirsi_dla_rates);

#define CMD_FLUSH		0x0B
#define CMD_GET_MCU_VERSION	0x81
#define CMD_GET_DEVICE_DATA	0x10
#define CMD_SET_RESET_STATE	0x87
#define CMD_STOP		0x15
#define CMD_PARAMETER_SETTING	0x11
#define CMD_SIMPLE_TRIGGER	0x12
#define CMD_EXIT		0x18

/* Custom CRC32 variant used to check framed commands: standard reflected
 * CRC-32/IEEE-802.3 polynomial, but the running CRC starts at 0 (not
 * 0xFFFFFFFF) and there is a single final XOR. Computed bitwise since only
 * three framed commands are built per acquisition - not worth a 256-entry
 * table. */
static uint32_t gcrc32(const uint8_t *data, size_t len)
{
	uint32_t crc;
	size_t i;
	unsigned int j;

	crc = 0;
	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xEDB88320u;
			else
				crc = crc >> 1;
		}
	}

	return crc ^ 0xFFFFFFFFu;
}

static void build_direct_command(uint8_t *buf, uint8_t subcmd,
		const uint8_t *payload, size_t payload_len)
{
	memset(buf, 0, USB_CMD_SIZE);
	buf[0] = 0x0A;
	buf[1] = subcmd;
	if (payload && payload_len)
		memcpy(buf + 2, payload, payload_len);
}

static size_t build_framed_command(uint8_t *buf, size_t buf_size, uint8_t code,
		const uint8_t *payload, size_t payload_len)
{
	size_t body_len, frame_len, padded_len;
	uint8_t body[MAX_FRAME_BODY];
	uint32_t crc;

	if (payload_len > MAX_FRAME_BODY - 3)
		return 0;

	body_len = payload_len + 3;

	frame_len = 8 + 1 + body_len + 1 + 4;
	padded_len = (frame_len + FRAME_ALIGN - 1) & ~(size_t)(FRAME_ALIGN - 1);
	if (padded_len > buf_size)
		return 0;

	/* The firmware reads a 16-bit LE length (payload + 2) after the
	 * command code. */
	body[0] = code;
	body[1] = (uint8_t)((payload_len + 2) & 0xFF);
	body[2] = (uint8_t)((payload_len + 2) >> 8);
	if (payload && payload_len)
		memcpy(body + 3, payload, payload_len);

	crc = gcrc32(body, body_len);

	memset(buf, 0, 8);
	frame_len = 8;
	buf[frame_len++] = 0x0A;
	memcpy(buf + frame_len, body, body_len);
	frame_len += body_len;
	buf[frame_len++] = 0x0B;
	buf[frame_len++] = (uint8_t)(crc & 0xFF);
	buf[frame_len++] = (uint8_t)((crc >> 8) & 0xFF);
	buf[frame_len++] = (uint8_t)((crc >> 16) & 0xFF);
	buf[frame_len++] = (uint8_t)((crc >> 24) & 0xFF);

	memset(buf + frame_len, 0, padded_len - frame_len);

	return padded_len;
}

/*
 * Writes buf, up to max_writes times while the device takes none of it:
 * the write then times out with 0 bytes taken, and the same packet sent
 * again usually goes through at once. Only a 512-byte direct command that
 * changes no device state may be written more than once - such a packet is
 * taken whole or not at all, so the device never runs it twice.
 */
static int bulk_out_retry(libusb_device_handle *devhdl, const uint8_t *buf,
		int len, int max_writes)
{
	int actual, ret, write;

	for (write = 1; ; write++) {
		actual = 0;
		ret = libusb_bulk_transfer(devhdl, EP_CMD_OUT,
				(unsigned char *)buf, len, &actual, USB_TIMEOUT_MS);
		if (ret != LIBUSB_ERROR_TIMEOUT || actual != 0 ||
				write >= max_writes)
			break;
		sr_warn("Command 0x%02x was not taken by the device, sending "
			"it again (write %d of %d).", buf[1], write + 1,
			max_writes);
	}
	if (ret < 0) {
		if (write > 1)
			sr_err("Bulk OUT failed: %s, after %d writes.",
				libusb_error_name(ret), write);
		else
			sr_err("Bulk OUT failed: %s.", libusb_error_name(ret));
		return SR_ERR_IO;
	}
	if (actual != len) {
		sr_err("Bulk OUT short write: %d of %d bytes.", actual, len);
		return SR_ERR_IO;
	}

	return SR_OK;
}

static int bulk_out(libusb_device_handle *devhdl, const uint8_t *buf, int len)
{
	return bulk_out_retry(devhdl, buf, len, 1);
}

/* Sends a direct command without payload and reads its reply. Only for
 * queries that change no device state, as the command may be written more
 * than once (see bulk_out_retry()). */
static int direct_command_reply(libusb_device_handle *devhdl, uint8_t subcmd,
		uint8_t *reply, int *reply_len)
{
	uint8_t buf[USB_CMD_SIZE];
	int ret;

	build_direct_command(buf, subcmd, NULL, 0);
	if ((ret = bulk_out_retry(devhdl, buf, USB_CMD_SIZE,
			QUERY_MAX_WRITES)) != SR_OK)
		return ret;

	*reply_len = 0;
	ret = libusb_bulk_transfer(devhdl, EP_CMD_REPLY_IN, reply,
			USB_CMD_SIZE, reply_len, CMD_REPLY_TIMEOUT_MS);
	if (ret != LIBUSB_SUCCESS) {
		sr_err("Reading the reply to command 0x%02x failed: %s.",
			subcmd, libusb_error_name(ret));
		return SR_ERR_IO;
	}

	return SR_OK;
}

static int check_mcu_version_reply(struct dev_context *devc,
		const uint8_t *reply, int len)
{
	if (len < 9 || reply[0] != 0x0A || reply[1] != CMD_GET_MCU_VERSION ||
			reply[2] != 0x01 ||
			(reply[3] != 0x61 && reply[3] != 0x62)) {
		sr_err("Malformed GetMCUVersion reply.");
		return SR_ERR_DATA;
	}
	/* Byte 3 only says what starts next (0x61 application, 0x62
	 * bootloader), not what runs now: the running bootloader is told
	 * apart by its firmware version 0.0. */
	if (reply[4] == 0 && reply[5] == 0) {
		devc->in_bootloader = TRUE;
		sr_err("Device is in bootloader mode, not application mode "
			"(firmware version 0.0). Complete a firmware update "
			"first.");
		return SR_ERR;
	}

	sr_info("MCU firmware %u.%u, hardware version %u.",
		reply[4], reply[5], reply[6]);

	return SR_OK;
}

static int parse_device_data_reply(struct sr_dev_inst *sdi,
		const uint8_t *reply, int len)
{
	struct dev_context *devc;
	char code[5];
	unsigned int i;

	devc = sdi->priv;
	devc->model_code[0] = '\0';

	if (len < 17 || reply[0] != 0x0A || reply[1] != 0x02 ||
			reply[2] != 0x0D) {
		sr_err("Malformed GetDeviceData reply.");
		return SR_ERR_DATA;
	}

	/* Byte 6 is a constant 1, not an FPGA health flag, and bytes 9-10
	 * hold the MCU's built-in default 1.0, not the FPGA firmware's
	 * version: neither is checked or shown. */

	memcpy(code, reply + 13, 4);
	code[4] = '\0';
	for (i = 0; code[i]; i++) {
		if (!g_ascii_isprint(code[i]))
			code[i] = '?';
	}

	g_strlcpy(devc->model_code, code, sizeof(devc->model_code));
	sr_info("Device reports model code \"%s\".", devc->model_code);

	/* Other models may share the DLA-16 Plus's USB ID, but nothing in
	 * this driver was written or verified for them. */
	if (strcmp(devc->model_code, "DL16") != 0) {
		if (strcmp(devc->model_code, "DL32") == 0)
			sr_err("The FNIRSI DLA-32 Plus is not supported yet; "
				"this driver supports the DLA-16 Plus only.");
		else
			sr_err("FNIRSI DLA with model code \"%s\" is not "
				"supported; this driver supports the DLA-16 "
				"Plus only.", devc->model_code);
		return SR_ERR_NA;
	}

	return SR_OK;
}

SR_PRIV int fnirsi_dla_connect(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	uint8_t buf[FRAME_ALIGN];
	uint8_t reply[USB_CMD_SIZE];
	uint8_t payload;
	int ret, actual;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->device_gone = FALSE;
	devc->in_bootloader = FALSE;

	if ((ret = direct_command_reply(usb->devhdl, CMD_FLUSH,
			reply, &actual)) != SR_OK)
		return ret;

	if ((ret = direct_command_reply(usb->devhdl, CMD_GET_MCU_VERSION,
			reply, &actual)) != SR_OK)
		return ret;
	if ((ret = check_mcu_version_reply(devc, reply, actual)) != SR_OK)
		return ret;

	if ((ret = direct_command_reply(usb->devhdl, CMD_GET_DEVICE_DATA,
			reply, &actual)) != SR_OK)
		return ret;
	if ((ret = parse_device_data_reply(sdi, reply, actual)) != SR_OK)
		return ret;

	payload = 0x01;
	build_direct_command(buf, CMD_SET_RESET_STATE, &payload, 1);
	if ((ret = bulk_out(usb->devhdl, buf, USB_CMD_SIZE)) != SR_OK)
		return ret;
	g_usleep(200 * 1000);

	return SR_OK;
}

static int send_stop(libusb_device_handle *devhdl)
{
	uint8_t buf[FRAME_ALIGN];
	size_t len;

	len = build_framed_command(buf, sizeof(buf), CMD_STOP, NULL, 0);

	if (len == 0) {
		sr_err("Internal error building the Stop command.");
		return SR_ERR_BUG;
	}

	return bulk_out(devhdl, buf, (int)len);
}

/* Sent last before a claimed interface is released, once no acquisition
 * runs. Not sent when claiming the interface failed: another program may
 * be capturing from the device then. Not sent to the bootloader either:
 * it is an application command. */
SR_PRIV int fnirsi_dla_send_exit(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	uint8_t buf[FRAME_ALIGN];
	size_t len;

	devc = sdi->priv;
	usb = sdi->conn;

	if (devc->device_gone || devc->in_bootloader)
		return SR_OK;

	len = build_framed_command(buf, sizeof(buf), CMD_EXIT, NULL, 0);
	if (len == 0) {
		sr_err("Internal error building the Exit command.");
		return SR_ERR_BUG;
	}

	return bulk_out(usb->devhdl, buf, (int)len);
}

SR_PRIV void fnirsi_dla_send_pending_stop(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb = sdi->conn;

	if (!devc->stop_pending)
		return;
	devc->stop_pending = FALSE;
	devc->stop_sent = TRUE;
	send_stop(usb->devhdl);
}

/*
 * Reads and throws away sample data left over from an earlier run, until a
 * read gets nothing. Fails if the device is still sending after
 * STALE_DRAIN_LIMIT_MS: that data would otherwise start the acquisition.
 */
static int drain_sample_endpoint(libusb_device_handle *devhdl)
{
	uint8_t *buf;
	int actual, ret;
	int64_t start_us, elapsed_ms;
	uint64_t thrown_away;

	buf = g_try_malloc(STALE_READ_BYTES);
	if (!buf) {
		sr_err("Drain buffer malloc failed.");
		return SR_ERR_MALLOC;
	}

	thrown_away = 0;
	start_us = g_get_monotonic_time();
	for (;;) {
		actual = 0;
		ret = libusb_bulk_transfer(devhdl, EP_SAMPLE_IN, buf,
				STALE_READ_BYTES, &actual, STALE_READ_TIMEOUT_MS);
		/* A read that timed out may still have carried data. */
		if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_TIMEOUT) {
			g_free(buf);
			if (ret == LIBUSB_ERROR_NO_DEVICE) {
				sr_err("Device disconnected while clearing "
					"stale sample data.");
				return SR_ERR_IO;
			}
			sr_warn("Clearing stale sample data ended with %s "
				"after %" PRIu64 " bytes thrown away.",
				libusb_error_name(ret), thrown_away);
			return SR_OK;
		}
		if (actual == 0)
			break;
		thrown_away += (uint64_t)actual;

		elapsed_ms = (g_get_monotonic_time() - start_us) / 1000;
		if (elapsed_ms >= STALE_DRAIN_LIMIT_MS) {
			g_free(buf);
			sr_err("Stale data from an earlier run could not be "
				"cleared: the device is still sending after "
				"%" PRId64 " ms, %" PRIu64 " bytes thrown away.",
				elapsed_ms, thrown_away);
			return SR_ERR_IO;
		}
	}
	g_free(buf);

	if (thrown_away > 0)
		sr_dbg("Threw away %" PRIu64 " bytes of stale sample data.",
			thrown_away);

	return SR_OK;
}

static int send_parameter_setting(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	uint8_t payload[28];
	uint8_t buf[FRAME_ALIGN];
	size_t len;
	/*
	 * Only used in Buffer mode, which offset 1 selects - this driver
	 * always leaves offset 1 at 0 (Streaming mode) and never uses
	 * Buffer mode, so this is a plain hardcoded literal.
	 */
	const uint32_t depth = 0;

	devc = sdi->priv;
	usb = sdi->conn;

	/* The firmware reads only offsets 1, 2, 9-11 and 16-27; offsets 0,
	 * 3-8 and 12-15 are never read. */
	memset(payload, 0, sizeof(payload));
	/* Offset 1: Buffer mode flag - 0 is Streaming mode. */
	/* Offset 2: sub-mode, 0 - written to an FPGA register. */
	/* Offset 3: never read - sent as 1, as FNIRSI's software does. */
	payload[3] = 0x01;
	/* Offsets 5-8: sample rate in Hz, u32 LE - never read, sent as
	 * FNIRSI's software does; offset 9 sets the rate. */
	payload[5] = (uint8_t)(devc->cur_samplerate & 0xFF);
	payload[6] = (uint8_t)((devc->cur_samplerate >> 8) & 0xFF);
	payload[7] = (uint8_t)((devc->cur_samplerate >> 16) & 0xFF);
	payload[8] = (uint8_t)((devc->cur_samplerate >> 24) & 0xFF);
	/* Offset 9: rate-table index. */
	payload[9] = devc->cur_rate_idx;
	/* Offsets 10-11: threshold voltage, signed 16-bit LE, hundredths of
	 * a volt. */
	payload[10] = (uint8_t)(devc->threshold_hundredths & 0xFF);
	payload[11] = (uint8_t)((devc->threshold_hundredths >> 8) & 0xFF);
	/* Offsets 16-19 and 24-27: meaning unknown, 0 works for Streaming
	 * mode. */
	/* Offsets 20-23: depth, u32 LE - only used in Buffer mode. */
	payload[20] = (uint8_t)(depth & 0xFF);
	payload[21] = (uint8_t)((depth >> 8) & 0xFF);
	payload[22] = (uint8_t)((depth >> 16) & 0xFF);
	payload[23] = (uint8_t)((depth >> 24) & 0xFF);

	sr_info("ParameterSetting: rate=%" PRIu64 "Hz idx=0x%02x threshold=%.2fV "
		"(Streaming mode).", devc->cur_samplerate, devc->cur_rate_idx,
		devc->threshold_hundredths / 100.0);

	len = build_framed_command(buf, sizeof(buf), CMD_PARAMETER_SETTING,
			payload, sizeof(payload));
	if (len == 0) {
		sr_err("Internal error building the ParameterSetting command.");
		return SR_ERR_BUG;
	}

	return bulk_out(usb->devhdl, buf, (int)len);
}

static int send_simple_trigger(const struct sr_dev_inst *sdi, uint16_t channel_mask)
{
	struct sr_usb_dev_inst *usb;
	uint8_t payload[37];
	uint8_t buf[FRAME_ALIGN];
	size_t len;

	usb = sdi->conn;

	memset(payload, 0, sizeof(payload));
	/* Offsets 1-2: channel mask, u16 LE. */
	payload[1] = (uint8_t)(channel_mask & 0xFF);
	payload[2] = (uint8_t)((channel_mask >> 8) & 0xFF);
	/* Offset 8: hardware trigger type, 0 = none. Types 1-5 wait for a
	 * condition on D3 only, so the software trigger is used instead. */

	len = build_framed_command(buf, sizeof(buf), CMD_SIMPLE_TRIGGER,
			payload, sizeof(payload));
	if (len == 0) {
		sr_err("Internal error building the SimpleTrigger command.");
		return SR_ERR_BUG;
	}

	return bulk_out(usb->devhdl, buf, (int)len);
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	/* Every transfer is back now: a device that wasn't sent Stop (the
	 * limit was reached) is still streaming. This runs inside a transfer
	 * callback, so only mark it - see stop_pending in protocol.h. */
	if (!devc->stop_sent && !devc->device_gone)
		devc->stop_pending = TRUE;

	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);
	devc->transfers = NULL;

	g_free(devc->outbuf);
	devc->outbuf = NULL;

	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;
	gboolean was_last;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	was_last = TRUE;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
		} else if (devc->transfers[i]) {
			was_last = FALSE;
		}
	}

	if (was_last)
		finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("Failed to resubmit transfer: %s.", libusb_error_name(ret));
	free_transfer(transfer);
}

/* Deinterleaves the device's channel-round-robin wire format into standard
 * sigrok SR_DF_LOGIC packets. */
/* Explicit little-endian byte stores (not a raw struct/memcpy write of the
 * uint64_t) so output byte order is correct regardless of host endianness. */
static inline void store_le64(uint8_t *o, uint64_t v)
{
	o[0] = (uint8_t)(v);
	o[1] = (uint8_t)(v >> 8);
	o[2] = (uint8_t)(v >> 16);
	o[3] = (uint8_t)(v >> 24);
	o[4] = (uint8_t)(v >> 32);
	o[5] = (uint8_t)(v >> 40);
	o[6] = (uint8_t)(v >> 48);
	o[7] = (uint8_t)(v >> 56);
}

static void deinterleave(struct dev_context *devc, const uint8_t *data,
		size_t len, uint8_t **out, size_t *out_len)
{
	size_t i;
	unsigned int c;
	uint8_t *o;
	uint64_t lo, hi;

	o = devc->outbuf;

	for (i = 0; i < len; i++) {
		devc->group_buf[devc->group_fill++] = data[i];
		if (devc->group_fill < devc->num_active)
			continue;
		devc->group_fill = 0;

		/*
		 * One complete interleave group -> 8 time-samples of
		 * devc->unitsize bytes each. Built via the per-channel lookup
		 * tables (chan_lut_lo/hi, protocol.h) instead of a branchy
		 * per-bit test loop, each active channel contributing with a
		 * single branch-free OR. With 1-byte samples, lo holds all 8
		 * times in 8-bit lanes. With 2-byte samples, lo packs times
		 * 0-3 into 16-bit lanes and hi times 4-7. This is required to
		 * sustain 16ch/250MHz in real time.
		 */
		lo = 0;
		if (devc->unitsize == 1) {
			for (c = 0; c < devc->num_active; c++)
				lo |= devc->chan_lut_lo[c][devc->group_buf[c]];
			store_le64(o, lo);
			o += 8;
		} else {
			hi = 0;
			for (c = 0; c < devc->num_active; c++) {
				lo |= devc->chan_lut_lo[c][devc->group_buf[c]];
				hi |= devc->chan_lut_hi[c][devc->group_buf[c]];
			}
			store_le64(o, lo);
			store_le64(o + 8, hi);
			o += 16;
		}
	}

	*out = devc->outbuf;
	*out_len = (size_t)(o - devc->outbuf);
}

static void send_logic_packet(const struct sr_dev_inst *sdi, uint8_t *data, uint64_t length)
{
	const struct dev_context *devc = sdi->priv;
	const struct sr_datafeed_logic logic = {
		.length = length,
		.unitsize = devc->unitsize,
		.data = data,
	};
	const struct sr_datafeed_packet packet = {
		.type = SR_DF_LOGIC,
		.payload = &logic,
	};

	sr_session_send(sdi, &packet);
}

static void cancel_transfers(struct dev_context *devc)
{
	unsigned int i;

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

/*
 * Like fnirsi_dla_abort_acquisition(), but for use inside a transfer
 * callback: Stop isn't sent here, finish_acquisition() leaves it pending
 * once every transfer is back (see stop_pending in protocol.h).
 */
static void abort_acquisition_in_callback(struct dev_context *devc)
{
	devc->acq_aborted = TRUE;
	cancel_transfers(devc);
}

/*
 * Counts a transfer's bytes and returns how far delivery has fallen behind
 * sampling: bytes the device produced since the first transfer arrived,
 * minus bytes received, relative to the least it has been behind so far.
 */
static uint64_t delivery_lag(struct dev_context *devc, size_t bytes)
{
	int64_t now_ms, behind;
	uint64_t produced;
	gboolean first;

	now_ms = g_get_monotonic_time() / 1000;
	first = !devc->lag_started;
	if (first) {
		devc->lag_started = TRUE;
		devc->lag_start_ms = now_ms;
	}
	devc->received_bytes += bytes;

	produced = (uint64_t)(now_ms - devc->lag_start_ms) *
		devc->wire_bytes_per_sec / 1000;
	behind = (int64_t)produced - (int64_t)devc->received_bytes;
	if (first || behind < devc->least_behind)
		devc->least_behind = behind;

	return (uint64_t)(behind - devc->least_behind);
}

static void drop_preroll(struct dev_context *devc, const uint8_t **data,
		size_t *len)
{
	size_t drop;

	if (devc->preroll_bytes_left == 0)
		return;

	drop = *len;
	if (drop > devc->preroll_bytes_left)
		drop = (size_t)devc->preroll_bytes_left;
	*data += drop;
	*len -= drop;
	devc->preroll_bytes_left -= drop;
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	const uint8_t *data;
	size_t len;
	uint64_t lag;
	uint8_t *out;
	size_t out_len, out_samples;
	int pre_trigger_samples, trigger_offset;

	sdi = transfer->user_data;
	devc = sdi->priv;

	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	/*
	 * None of these resolve themselves by blind resubmission - a
	 * stalled endpoint in particular just returns STALL again
	 * immediately, turning resubmission into an unbounded tight loop.
	 * Treat all of them as a hard, immediate end to the acquisition.
	 */
	if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE ||
			transfer->status == LIBUSB_TRANSFER_STALL ||
			transfer->status == LIBUSB_TRANSFER_ERROR ||
			transfer->status == LIBUSB_TRANSFER_OVERFLOW) {
		sr_err("USB transfer failed (%s), aborting acquisition.",
				libusb_error_name(transfer->status));
		if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
			devc->device_gone = TRUE;
		abort_acquisition_in_callback(devc);
		free_transfer(transfer);
		return;
	}

	if (transfer->actual_length > 0)
		devc->empty_transfer_count = 0;
	else
		devc->empty_transfer_count++;

	/* The transfer that shows the lag isn't delivered: data may already
	 * be missing before it. */
	if (transfer->actual_length > 0 && !devc->limit_reached) {
		lag = delivery_lag(devc, (size_t)transfer->actual_length);
		if (lag >= LAG_LIMIT_BYTES) {
			sr_err("Delivery fell %" PRIu64 " MiB (%" PRIu64 " ms) "
				"behind sampling, the device may drop data - "
				"aborting acquisition. Lower the samplerate or "
				"the channel count.", lag / (1024 * 1024),
				lag * 1000 / devc->wire_bytes_per_sec);
			abort_acquisition_in_callback(devc);
			free_transfer(transfer);
			return;
		}
	}

	/* Only deinterleave/send if we haven't already reached the limit in
	 * an earlier (sibling) transfer's callback - once reached, any data
	 * from transfers that were already in flight at that point (and were
	 * deliberately allowed to drain naturally rather than being
	 * cancelled, see below) is simply discarded here. */
	data = transfer->buffer;
	len = transfer->actual_length > 0 ? (size_t)transfer->actual_length : 0;
	drop_preroll(devc, &data, &len);

	if (len > 0 && !devc->limit_reached) {
		deinterleave(devc, data, len, &out, &out_len);

		/*
		 * Software trigger: the device streams unconditionally (its
		 * hardware trigger type is left at 0, see
		 * send_simple_trigger()), so withhold data until the
		 * host-side scan finds the match,
		 * then start sending from exactly that sample onward.
		 */
		if (devc->stl && !devc->trigger_fired) {
			pre_trigger_samples = 0;
			trigger_offset = soft_trigger_logic_check(devc->stl,
					out, (int)out_len, &pre_trigger_samples);
			if (trigger_offset > -1) {
				devc->trigger_fired = TRUE;
				/*
				 * soft_trigger_logic_check() already sent the
				 * buffered pre-trigger samples to the session
				 * bus directly (pre_trigger_send()) - just
				 * account for them here so acq_limit_samples
				 * still covers the whole capture, pre- and
				 * post-trigger together.
				 */
				devc->sent_samples += (uint64_t)pre_trigger_samples;
				out += (size_t)trigger_offset * devc->unitsize;
				out_len -= (size_t)trigger_offset * devc->unitsize;
			} else {
				out_len = 0;
			}
		}

		out_samples = out_len / devc->unitsize;

		if (out_samples > 0) {
			if (devc->acq_limit_samples) {
				if (devc->sent_samples >= devc->acq_limit_samples)
					out_samples = 0;
				else if (devc->sent_samples + out_samples > devc->acq_limit_samples)
					out_samples = devc->acq_limit_samples - devc->sent_samples;
			}

			if (out_samples > 0) {
				send_logic_packet(sdi, out,
					out_samples * devc->unitsize);
				devc->sent_samples += out_samples;
			}
		}
	}

	if (devc->acq_limit_samples && devc->sent_samples >= devc->acq_limit_samples)
		devc->limit_reached = TRUE;

	if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
		sr_err("Device stopped producing data (%u consecutive empty "
			"transfers), aborting acquisition.",
			devc->empty_transfer_count);
		/* What a damaged FPGA image looks like, e.g. after an
		 * interrupted firmware update. */
		if (devc->received_bytes == 0)
			sr_err("The device answers commands but sent no "
				"sample data at all: its FPGA image may be "
				"damaged. Update its firmware.");
		abort_acquisition_in_callback(devc);
	}

	/*
	 * Once the limit is reached, stop resubmitting - but do NOT
	 * force-cancel sibling transfers that are still genuinely in flight
	 * (unlike fnirsi_dla_abort_acquisition(), which is for a hard user-
	 * initiated stop or device removal). Cancelling them here would
	 * discard data the device hadn't finished sending yet, silently
	 * delivering fewer samples than requested. Instead just let this one
	 * transfer go; free_transfer() finishes the acquisition once every
	 * outstanding transfer has been freed this way.
	 */
	if (devc->acq_aborted || devc->limit_reached)
		free_transfer(transfer);
	else
		resubmit_transfer(transfer);
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct timeval tv;
	const struct sr_dev_inst *sdi;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	drvc = sdi->driver->context;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	/* Outside libusb's event handling now, so a synchronous Stop can
	 * complete. */
	fnirsi_dla_send_pending_stop(sdi);

	return TRUE;
}

SR_PRIV void fnirsi_dla_abort_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->acq_aborted = TRUE;

	if (!devc->device_gone)
		send_stop(usb->devhdl);
	devc->stop_sent = TRUE;
	devc->stop_pending = FALSE;

	cancel_transfers(devc);
}

static int configure_channels(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	const GSList *l;
	struct sr_channel *ch;
	unsigned int n, hw_n, i, idx;
	gboolean used[MAX_CHANNELS];
	uint64_t max_rate;

	devc = sdi->priv;
	n = 0;
	for (i = 0; i < devc->num_channels; i++)
		used[i] = FALSE;

	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		if (!ch->enabled)
			continue;
		if (n >= devc->num_channels)
			return SR_ERR_ARG;
		devc->active_channels[n] = (uint8_t)ch->index;
		used[ch->index] = TRUE;
		n++;
	}

	if (n == 0) {
		sr_err("No logic channels enabled.");
		return SR_ERR_ARG;
	}

	/*
	 * Only power-of-2 channel counts (1, 2, 4, 8, 16) produce valid data
	 * on this hardware - any other count (3, 5, 6, 7, 9-15) silently
	 * corrupts every channel's data, with no "reduced but usable" mode.
	 * Rather than reject those counts outright, sample the next
	 * power-of-2 count up *on the wire*, padding with extra
	 * currently-disabled channels, but keep forwarding only the
	 * originally-requested channels to the session - deinterleave()
	 * always writes a fixed 2-byte word covering all 16 possible channel
	 * bit positions regardless of how many are "active", so the padding
	 * channels' bits are simply present-but-ignored: they were never
	 * marked enabled on sdi, so PulseView/sigrok-cli never look at them.
	 */
	hw_n = 1;
	while (hw_n < n)
		hw_n *= 2;

	if (hw_n > n)
		sr_info("%u channels requested - not a valid hardware count, "
			"sampling %u channels on the wire instead (the %u extra "
			"channels' data is captured but not forwarded).",
			n, hw_n, hw_n - n);

	/*
	 * Pick the padding channels (lowest-indexed still-unused ones) and
	 * mark them used, but DON'T just append them after the requested
	 * ones - the wire's interleave round-robin always cycles through
	 * physical channels in ascending index order, regardless of which
	 * ones are selected, so active_channels[] must end up sorted
	 * ascending overall, not "requested block, then padding block".
	 * A scattered, non-contiguous request (e.g. D0,D2,D3,D7,D8,D10,
	 * padded with D1,D4) previously broke this: appending gave
	 * {0,2,3,7,8,10,1,4}, mismatching the true wire order {0,1,2,3,4,7,
	 * 8,10} and corrupting every channel's data.
	 */
	i = n;
	for (idx = 0; i < hw_n && idx < devc->num_channels; idx++) {
		if (used[idx])
			continue;
		used[idx] = TRUE;
		i++;
		sr_info("Auto-added channel D%u as a padding channel "
			"(physically sampled on the wire, not forwarded "
			"to the session).", idx);
	}
	/* Can't happen - hw_n <= num_channels and n < hw_n <= num_channels
	 * means there are always at least hw_n - n unused indices left among
	 * the device's channels to pad with. */
	if (i != hw_n) {
		sr_err("Internal error padding channel count to %u.", hw_n);
		return SR_ERR_BUG;
	}

	i = 0;
	for (idx = 0; idx < devc->num_channels; idx++) {
		if (used[idx])
			devc->active_channels[i++] = (uint8_t)idx;
	}

	/*
	 * Aggregate throughput ceiling: the samplerate configured via
	 * SR_CONF_SAMPLERATE must not push the aggregate (samplerate * hw_n,
	 * the actual wire channel count after padding above) past
	 * AGGREGATE_SAMPLERATE_MAX (2 Gsa/s - see the constant's own comment
	 * in protocol.h for how this was corrected from an earlier, wrong
	 * 4 Gsa/s assumption), or the hardware silently corrupts data rather
	 * than failing cleanly. This can only be checked here, once both the
	 * final channel selection and the samplerate are both known -
	 * config_set() for either one alone cannot see the other's current
	 * value reliably (a frontend may set the samplerate before or after
	 * enabling channels).
	 */
	max_rate = AGGREGATE_SAMPLERATE_MAX / hw_n;
	if (devc->cur_samplerate > max_rate) {
		sr_err("Samplerate %" PRIu64 "Hz with %u channels sampled on "
			"the wire (%u requested) would need %" PRIu64 "Msa/s "
			"aggregate, over the ~2Gsa/s ceiling - max is %" PRIu64
			"Hz for %u wire channels. Lower the samplerate or the "
			"channel count.",
			devc->cur_samplerate, hw_n, n,
			(devc->cur_samplerate * hw_n) / 1000000, max_rate, hw_n);
		return SR_ERR_ARG;
	}

	devc->num_active = hw_n;
	devc->group_fill = 0;

	return SR_OK;
}

/*
 * Builds the fast-path deinterleave() lookup tables for the current channel
 * selection and devc->unitsize (see chan_lut_lo/hi in protocol.h). This is
 * only O(num_active * 256) = at most 4096 table entries, done once per
 * acquisition rather than per-sample.
 */
static int build_deinterleave_luts(struct dev_context *devc)
{
	unsigned int n, dst, v, t, lane_bits;
	uint64_t lo, hi;

	lane_bits = 8 * devc->unitsize;

	for (n = 0; n < devc->num_active; n++) {
		dst = devc->active_channels[n];

		if (dst >= lane_bits) {
			sr_err("Channel index %u has no valid bit position in "
				"the %u-bit-wide sample word.", dst, lane_bits);
			return SR_ERR_BUG;
		}

		for (v = 0; v < 256; v++) {
			lo = 0;
			hi = 0;

			if (devc->unitsize == 1) {
				for (t = 0; t < 8; t++) {
					if ((v >> t) & 1)
						lo |= (uint64_t)(1u << dst) << (8 * t);
				}
			} else {
				for (t = 0; t < 4; t++) {
					if ((v >> t) & 1)
						lo |= (uint64_t)(1u << dst) << (16 * t);
				}
				for (t = 0; t < 4; t++) {
					if ((v >> (t + 4)) & 1)
						hi |= (uint64_t)(1u << dst) << (16 * t);
				}
			}
			devc->chan_lut_lo[n][v] = lo;
			devc->chan_lut_hi[n][v] = hi;
		}
	}

	return SR_OK;
}

static uint16_t channel_mask(const struct dev_context *devc)
{
	uint16_t mask;
	unsigned int i;

	mask = 0;
	for (i = 0; i < devc->num_active; i++)
		mask |= (uint16_t)1 << devc->active_channels[i];

	return mask;
}

static size_t compute_transfer_bufsize(uint64_t samplerate, unsigned int hw_n)
{
	uint64_t aggregate_bytes_per_sec, target;
	size_t bufsize;

	aggregate_bytes_per_sec = (samplerate * hw_n) / 8;
	target = aggregate_bytes_per_sec * TARGET_TRANSFER_MS / 1000;

	if (target < MIN_TRANSFER_BUFSIZE)
		target = MIN_TRANSFER_BUFSIZE;
	if (target > TRANSFER_BUFSIZE)
		target = TRANSFER_BUFSIZE;

	bufsize = (size_t)target;
	bufsize -= bufsize % USB_SS_MAX_PACKET_SIZE;
	if (bufsize < USB_SS_MAX_PACKET_SIZE)
		bufsize = USB_SS_MAX_PACKET_SIZE;

	return bufsize;
}

SR_PRIV void fnirsi_dla_drain_pending_transfers(struct dev_context *devc)
{
	struct timeval tv;
	int i, max_iterations;

	tv.tv_sec = 0;
	tv.tv_usec = DRAIN_POLL_INTERVAL_MS * 1000;

	max_iterations = DRAIN_MAX_WAIT_MS / DRAIN_POLL_INTERVAL_MS;
	for (i = 0; i < max_iterations && devc->transfers; i++)
		libusb_handle_events_timeout(devc->ctx->libusb_ctx, &tv);
}

static uint64_t preroll_bytes(uint64_t samplerate, unsigned int hw_n)
{
	uint64_t samples, groups;

	samples = (PREROLL_MS * samplerate + 999) / 1000;
	groups = (samples + 7) / 8;

	return groups * hw_n;
}

SR_PRIV int fnirsi_dla_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	struct sr_trigger *trigger;
	unsigned char *buf;
	unsigned int i;
	int ret;
	uint64_t msec_samples;
	uint64_t pre_trigger_u64;
	int pre_trigger_samples;
	gboolean any_submitted;

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	if ((ret = configure_channels(sdi)) != SR_OK)
		return ret;

	devc->transfer_bufsize = compute_transfer_bufsize(devc->cur_samplerate,
			devc->num_active);
	sr_info("Using %" G_GSIZE_FORMAT " byte USB transfer buffers "
		"(%u simultaneous) for this acquisition.",
		devc->transfer_bufsize, NUM_SIMUL_TRANSFERS);

	devc->preroll_bytes_left = preroll_bytes(devc->cur_samplerate,
			devc->num_active);
	sr_dbg("Dropping the first %" PRIu64 " bytes (%d ms) of the "
		"acquisition while the threshold settles.",
		devc->preroll_bytes_left, PREROLL_MS);

	devc->wire_bytes_per_sec = devc->cur_samplerate * devc->num_active / 8;
	devc->received_bytes = 0;
	devc->lag_started = FALSE;
	devc->lag_start_ms = 0;
	devc->least_behind = 0;

	devc->ctx = drvc->sr_ctx;
	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->limit_reached = FALSE;
	devc->stop_sent = FALSE;
	devc->stop_pending = FALSE;
	devc->empty_transfer_count = 0;
	devc->group_fill = 0;

	/* SR_CONF_LIMIT_MSEC is an alternate way to bound the acquisition -
	 * convert it to a sample count against the current samplerate. If
	 * both are set, whichever bound is smaller wins. The result goes to
	 * acq_limit_samples, never back into limit_samples (see protocol.h). */
	devc->acq_limit_samples = devc->limit_samples;
	if (devc->limit_msec) {
		msec_samples = devc->cur_samplerate * devc->limit_msec / 1000;
		if (!devc->acq_limit_samples || msec_samples < devc->acq_limit_samples)
			devc->acq_limit_samples = msec_samples;
	}

	if ((trigger = sr_session_trigger_get(sdi->session))) {
		pre_trigger_samples = 0;
		/* Pre-trigger only makes sense relative to a bounded capture -
		 * capture_ratio is "% of the sample bound that should come
		 * before the trigger", so it's meaningless without one set. */
		if (devc->acq_limit_samples > 0) {
			pre_trigger_u64 = (devc->capture_ratio * devc->acq_limit_samples) / 100;
			if (pre_trigger_u64 > INT_MAX / FNIRSI_DLA16_SAMPLE_UNITSIZE)
				pre_trigger_u64 = INT_MAX / FNIRSI_DLA16_SAMPLE_UNITSIZE;
			pre_trigger_samples = (int)pre_trigger_u64;
		}
		devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
		if (!devc->stl)
			return SR_ERR_MALLOC;
		devc->trigger_fired = FALSE;
	} else {
		devc->stl = NULL;
		devc->trigger_fired = TRUE;
	}

	/* 1-byte samples halve the data the session handles, but only fit
	 * when every wire channel is D0-D7 (padding takes the lowest unused
	 * channels, so it stays there too), and the software trigger always
	 * works on 2-byte samples - one bit per channel the device has. */
	devc->unitsize = (!devc->stl && channel_mask(devc) <= 0xFF) ?
		1 : FNIRSI_DLA16_SAMPLE_UNITSIZE;
	if ((ret = build_deinterleave_luts(devc)) != SR_OK)
		goto err_free_stl;

	/* Drain only here, while the device is stopped and before any
	 * transfer is queued: a drain after the start commands would eat the
	 * capture's own first bytes. */
	if ((ret = send_stop(usb->devhdl)) != SR_OK)
		goto err_free_stl;
	if ((ret = drain_sample_endpoint(usb->devhdl)) != SR_OK)
		goto err_free_stl;

	/* Worst case expansion is (8 * unitsize)-fold: num_active == 1, each
	 * wire byte becomes 8 samples of unitsize bytes. Sized against
	 * transfer_bufsize (the actual per-transfer buffer size for this
	 * acquisition), not the fixed TRANSFER_BUFSIZE ceiling. */
	devc->outbuf_size = devc->transfer_bufsize * 8 * devc->unitsize;
	devc->outbuf = g_try_malloc(devc->outbuf_size);
	if (!devc->outbuf) {
		sr_err("Output buffer malloc failed.");
		ret = SR_ERR_MALLOC;
		goto err_free_stl;
	}

	devc->num_transfers = NUM_SIMUL_TRANSFERS;
	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * devc->num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		ret = SR_ERR_MALLOC;
		goto err_free_outbuf;
	}

	usb_source_add(sdi->session, devc->ctx, USB_TIMEOUT_MS, receive_data,
			(void *)sdi);

	any_submitted = FALSE;
	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc(devc->transfer_bufsize);
		if (!buf) {
			sr_err("USB transfer buffer malloc failed.");
			ret = SR_ERR_MALLOC;
			break;
		}
		transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			sr_err("USB transfer allocation failed.");
			g_free(buf);
			ret = SR_ERR_MALLOC;
			break;
		}
		libusb_fill_bulk_transfer(transfer, usb->devhdl, EP_SAMPLE_IN,
				buf, (int)devc->transfer_bufsize, receive_transfer,
				(void *)sdi, USB_TIMEOUT_MS);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.", libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			ret = SR_ERR;
			break;
		}
		devc->transfers[i] = transfer;
		any_submitted = TRUE;
	}

	if (i < devc->num_transfers) {
		if (any_submitted) {
			/*
			 * At least one transfer is genuinely in flight - its
			 * (and any siblings') existing completion-callback path
			 * (free_transfer() -> finish_acquisition()) is still
			 * the only thing allowed to free devc->outbuf/transfers
			 * (freeing them here too would race that callback and
			 * double-free them), but nothing else is guaranteed to
			 * ever pump the event loop that drives it - see
			 * drain_pending_transfers().
			 */
			fnirsi_dla_abort_acquisition(sdi);
			fnirsi_dla_drain_pending_transfers(devc);
			return ret;
		}
		/*
		 * Nothing was ever submitted - finish_acquisition() will
		 * never run, so nothing else will free these; do it here.
		 */
		usb_source_remove(sdi->session, devc->ctx);
		g_free(devc->transfers);
		devc->transfers = NULL;
		devc->num_transfers = 0;
		goto err_free_outbuf;
	}

	/*
	 * Start the device only now that every transfer is queued, so its
	 * first bytes land in them instead of waiting in its own memory. The
	 * header goes first: the synchronous command writes below run
	 * libusb's event handling, so a sample transfer may complete (and
	 * send SR_DF_LOGIC) inside them. On failure, the abort below ends
	 * the acquisition through finish_acquisition(), which sends the
	 * matching SR_DF_END.
	 */
	std_session_send_df_header(sdi);

	if ((ret = send_parameter_setting(sdi)) != SR_OK ||
			(ret = send_simple_trigger(sdi, channel_mask(devc))) != SR_OK) {
		fnirsi_dla_abort_acquisition(sdi);
		fnirsi_dla_drain_pending_transfers(devc);
		return ret;
	}

	return SR_OK;

err_free_outbuf:
	g_free(devc->outbuf);
	devc->outbuf = NULL;
err_free_stl:
	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
	return ret;
}
