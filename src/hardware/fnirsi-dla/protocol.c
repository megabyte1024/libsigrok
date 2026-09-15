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

/* Custom CRC32 variant used to check framed commands: standard reflected
 * CRC-32/IEEE-802.3 table, but the running CRC starts at 0 (not 0xFFFFFFFF)
 * and there is a single final XOR. */
static uint32_t crc_table[256];

static void build_crc_table(void)
{
	unsigned int i, j;
	uint32_t c;

	for (i = 0; i < 256; i++) {
		c = i;
		for (j = 0; j < 8; j++) {
			if (c & 1)
				c = (c >> 1) ^ 0xEDB88320u;
			else
				c = (c >> 1);
		}
		crc_table[i] = c;
	}
}

SR_PRIV int fnirsi_dla_init(struct sr_dev_driver *di, struct sr_context *sr_ctx)
{
	build_crc_table();

	return std_init(di, sr_ctx);
}

static uint32_t gcrc32(const uint8_t *data, size_t len)
{
	uint32_t crc;
	size_t i;

	crc = 0;
	for (i = 0; i < len; i++)
		crc = crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);

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

static size_t build_framed_command(uint8_t *buf, uint8_t code,
		const uint8_t *payload, size_t payload_len)
{
	size_t body_len, frame_len, padded_len;
	uint8_t body[MAX_FRAME_BODY];
	uint32_t crc;

	body_len = payload_len + 2;
	body[0] = code;
	body[1] = (uint8_t)(payload_len + 1);
	if (payload && payload_len)
		memcpy(body + 2, payload, payload_len);

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

	padded_len = (frame_len + FRAME_ALIGN - 1) & ~(size_t)(FRAME_ALIGN - 1);
	memset(buf + frame_len, 0, padded_len - frame_len);

	return padded_len;
}

static int bulk_out(libusb_device_handle *devhdl, const uint8_t *buf, int len)
{
	int actual, ret;

	ret = libusb_bulk_transfer(devhdl, EP_CMD_OUT, (unsigned char *)buf,
			len, &actual, USB_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("Bulk OUT failed: %s.", libusb_error_name(ret));
		return SR_ERR_IO;
	}

	return SR_OK;
}

static int bulk_in_discard(libusb_device_handle *devhdl, int ep, int len,
		unsigned int timeout_ms)
{
	uint8_t buf[USB_CMD_SIZE];
	int actual, ret;

	if (len > (int)sizeof(buf))
		len = sizeof(buf);

	ret = libusb_bulk_transfer(devhdl, ep, buf, len, &actual, timeout_ms);
	/* Timeouts here are expected/harmless (e.g. flush reply already
	 * consumed, or reply endpoint quiet) - only report hard errors. */
	if (ret < 0 && ret != LIBUSB_ERROR_TIMEOUT)
		sr_dbg("Bulk IN (discard) on EP 0x%02x: %s.", ep,
				libusb_error_name(ret));

	return SR_OK;
}

static void parse_device_data_reply(struct sr_dev_inst *sdi,
		const uint8_t *reply, int len)
{
	struct dev_context *devc;
	char code[5];

	devc = sdi->priv;
	devc->model_code[0] = '\0';

	if (len < 17)
		return;
	memcpy(code, reply + 13, 4);
	code[4] = '\0';
	if (!g_ascii_isprint(code[0]) || !g_ascii_isprint(code[1]) ||
			!g_ascii_isprint(code[2]) || !g_ascii_isprint(code[3]))
		return;

	g_strlcpy(devc->model_code, code, sizeof(devc->model_code));
	sr_info("Device reports model code \"%s\".", devc->model_code);

	g_free(sdi->model);
	if (strcmp(devc->model_code, "DL16") == 0)
		sdi->model = g_strdup("DLA-16 Plus");
	else
		sdi->model = g_strdup_printf(
			"DLA (unrecognized model code \"%s\" - please report "
			"this)", devc->model_code);
}

SR_PRIV int fnirsi_dla_connect(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	uint8_t buf[FRAME_ALIGN];
	uint8_t reply[USB_CMD_SIZE];
	uint8_t payload;
	int ret, actual;

	usb = sdi->conn;

	build_direct_command(buf, CMD_FLUSH, NULL, 0);
	if ((ret = bulk_out(usb->devhdl, buf, USB_CMD_SIZE)) != SR_OK)
		return ret;
	bulk_in_discard(usb->devhdl, EP_CMD_REPLY_IN, USB_CMD_SIZE, 500);

	build_direct_command(buf, CMD_GET_MCU_VERSION, NULL, 0);
	if ((ret = bulk_out(usb->devhdl, buf, USB_CMD_SIZE)) != SR_OK)
		return ret;
	bulk_in_discard(usb->devhdl, EP_CMD_REPLY_IN, USB_CMD_SIZE, 500);

	build_direct_command(buf, CMD_GET_DEVICE_DATA, NULL, 0);
	if ((ret = bulk_out(usb->devhdl, buf, USB_CMD_SIZE)) != SR_OK)
		return ret;
	ret = libusb_bulk_transfer(usb->devhdl, EP_CMD_REPLY_IN, reply,
			sizeof(reply), &actual, 500);
	if (ret == LIBUSB_SUCCESS)
		parse_device_data_reply(sdi, reply, actual);
	else if (ret != LIBUSB_ERROR_TIMEOUT)
		sr_dbg("GetDeviceData reply read failed: %s.",
				libusb_error_name(ret));

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
	uint8_t payload = 0x00;
	size_t len = build_framed_command(buf, CMD_STOP, &payload, 1);

	return bulk_out(devhdl, buf, (int)len);
}

static void drain_sample_endpoint(libusb_device_handle *devhdl)
{
	uint8_t *buf;
	int actual, ret, i;

	buf = g_try_malloc(TRANSFER_BUFSIZE);
	if (!buf) {
		sr_err("Drain buffer malloc failed.");
		return;
	}
	for (i = 0; i < 64; i++) {
		ret = libusb_bulk_transfer(devhdl, EP_SAMPLE_IN, buf,
				TRANSFER_BUFSIZE, &actual, 100);
		if (ret == LIBUSB_ERROR_TIMEOUT || actual == 0)
			break;
	}
	g_free(buf);
}

static int send_parameter_setting(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	uint8_t payload[29];
	uint8_t buf[FRAME_ALIGN];
	size_t len;
	/*
	 * ALWAYS 0. This is the ONLY thing that selects Streaming vs Buffer
	 * mode - depth=0 is Streaming mode, any nonzero value is Buffer mode.
	 * This driver must never use Buffer mode, so this is a plain
	 * hardcoded literal, not derived from any config option - there is
	 * no code path anywhere in this driver that can set it to anything
	 * else.
	 */
	const uint32_t depth = 0;

	devc = sdi->priv;
	usb = sdi->conn;

	memset(payload, 0, sizeof(payload));
	/* Offsets 0-3: NOT safe to touch (hangs / non-reproducible behaviour) -
	 * always left at 0. */
	/* Offset 4: no effect across its full 0-255 range - 0 is fine. */
	/* Offsets 6-9: sample rate in Hz, u32 LE. */
	payload[6] = (uint8_t)(devc->cur_samplerate & 0xFF);
	payload[7] = (uint8_t)((devc->cur_samplerate >> 8) & 0xFF);
	payload[8] = (uint8_t)((devc->cur_samplerate >> 16) & 0xFF);
	payload[9] = (uint8_t)((devc->cur_samplerate >> 24) & 0xFF);
	/* Offset 10: rate-table index, required alongside offsets 6-9. */
	payload[10] = devc->cur_rate_idx;
	/* Offsets 11-12: threshold voltage, signed 16-bit LE, hundredths of
	 * a volt. */
	payload[11] = (uint8_t)(devc->threshold_hundredths & 0xFF);
	payload[12] = (uint8_t)((devc->threshold_hundredths >> 8) & 0xFF);
	/* Offset 13: threshold preset selector - cosmetic only, no effect on
	 * the actual comparator threshold. 0 = custom. */
	payload[13] = 0x00;
	/* Offsets 14-17: no effect - 0 is fine. */
	/* Offsets 18-20: unknown, always 0 in every capture seen. */
	/* Offsets 21-24: depth, u32 LE - Streaming mode iff 0. */
	payload[21] = (uint8_t)(depth & 0xFF);
	payload[22] = (uint8_t)((depth >> 8) & 0xFF);
	payload[23] = (uint8_t)((depth >> 16) & 0xFF);
	payload[24] = (uint8_t)((depth >> 24) & 0xFF);
	/* Offsets 25-28: unknown, always 0 in every capture seen. */

	sr_info("ParameterSetting: rate=%" PRIu64 "Hz idx=0x%02x threshold=%.2fV "
		"depth=%u (%s).", devc->cur_samplerate, devc->cur_rate_idx,
		devc->threshold_hundredths / 100.0, depth,
		depth == 0 ? "Streaming mode" : "BUG: Buffer mode - must never happen");

	len = build_framed_command(buf, CMD_PARAMETER_SETTING, payload, sizeof(payload));

	return bulk_out(usb->devhdl, buf, (int)len);
}

static int send_simple_trigger(const struct sr_dev_inst *sdi, uint16_t channel_mask)
{
	struct sr_usb_dev_inst *usb;
	uint8_t payload[38];
	uint8_t buf[FRAME_ALIGN];
	size_t len;

	usb = sdi->conn;

	memset(payload, 0, sizeof(payload));
	payload[2] = (uint8_t)(channel_mask & 0xFF);
	payload[3] = (uint8_t)((channel_mask >> 8) & 0xFF);

	len = build_framed_command(buf, CMD_SIMPLE_TRIGGER, payload, sizeof(payload));

	return bulk_out(usb->devhdl, buf, (int)len);
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

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
static void deinterleave(struct dev_context *devc, const uint8_t *data,
		size_t len, uint8_t **out, size_t *out_len)
{
	size_t i;
	unsigned int c;
	uint8_t *o;
	uint64_t lo, hi;
	uint8_t v;

	o = devc->outbuf;

	for (i = 0; i < len; i++) {
		devc->group_buf[devc->group_fill++] = data[i];
		if (devc->group_fill < devc->num_active)
			continue;
		devc->group_fill = 0;

		/*
		 * One complete interleave group -> 8 time-samples, each a
		 * 16-bit-wide (2 byte) sigrok logic sample - the DLA-16's
		 * actual wire sample format (one bit per
		 * FNIRSI_DLA16_NUM_CHANNELS possible channel, protocol.h).
		 * Built via the per-channel lookup tables (chan_lut_lo/hi,
		 * protocol.h) instead of a branchy per-bit test loop: lo
		 * packs times 0-3 into 16-bit lanes of one 64-bit word, hi
		 * packs times 4-7 the same way, and each active channel
		 * contributes with a single branch-free OR into each half.
		 * This is required to sustain 16ch/250MHz in real time.
		 */
		lo = 0;
		hi = 0;
		for (c = 0; c < devc->num_active; c++) {
			v = devc->group_buf[c];
			lo |= devc->chan_lut_lo[c][v];
			hi |= devc->chan_lut_hi[c][v];
		}
		/* Explicit little-endian byte stores (not a raw struct/memcpy
		 * write of the uint64_t) so output byte order is correct
		 * regardless of host endianness. */
		o[0]  = (uint8_t)(lo);
		o[1]  = (uint8_t)(lo >> 8);
		o[2]  = (uint8_t)(lo >> 16);
		o[3]  = (uint8_t)(lo >> 24);
		o[4]  = (uint8_t)(lo >> 32);
		o[5]  = (uint8_t)(lo >> 40);
		o[6]  = (uint8_t)(lo >> 48);
		o[7]  = (uint8_t)(lo >> 56);
		o[8]  = (uint8_t)(hi);
		o[9]  = (uint8_t)(hi >> 8);
		o[10] = (uint8_t)(hi >> 16);
		o[11] = (uint8_t)(hi >> 24);
		o[12] = (uint8_t)(hi >> 32);
		o[13] = (uint8_t)(hi >> 40);
		o[14] = (uint8_t)(hi >> 48);
		o[15] = (uint8_t)(hi >> 56);
		o += 16;
	}

	*out = devc->outbuf;
	*out_len = (size_t)(o - devc->outbuf);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
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
		fnirsi_dla_abort_acquisition(sdi);
		free_transfer(transfer);
		return;
	}

	if (transfer->actual_length > 0)
		devc->empty_transfer_count = 0;
	else
		devc->empty_transfer_count++;

	/* Only deinterleave/send if we haven't already reached the limit in
	 * an earlier (sibling) transfer's callback - once reached, any data
	 * from transfers that were already in flight at that point (and were
	 * deliberately allowed to drain naturally rather than being
	 * cancelled, see below) is simply discarded here. */
	if (transfer->actual_length > 0 && !devc->limit_reached) {
		deinterleave(devc, transfer->buffer,
				(size_t)transfer->actual_length, &out, &out_len);

		/*
		 * Software trigger: the device streams unconditionally
		 * regardless of the (non-functional) hardware trigger byte,
		 * so withhold data until the host-side scan finds the match,
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
				out += (size_t)trigger_offset * FNIRSI_DLA16_SAMPLE_UNITSIZE;
				out_len -= (size_t)trigger_offset * FNIRSI_DLA16_SAMPLE_UNITSIZE;
			} else {
				out_len = 0;
			}
		}

		out_samples = out_len / FNIRSI_DLA16_SAMPLE_UNITSIZE;

		if (out_samples > 0) {
			if (devc->acq_limit_samples) {
				if (devc->sent_samples >= devc->acq_limit_samples)
					out_samples = 0;
				else if (devc->sent_samples + out_samples > devc->acq_limit_samples)
					out_samples = devc->acq_limit_samples - devc->sent_samples;
			}

			if (out_samples > 0) {
				const struct sr_datafeed_logic logic = {
					.length = out_samples * FNIRSI_DLA16_SAMPLE_UNITSIZE,
					.unitsize = FNIRSI_DLA16_SAMPLE_UNITSIZE,
					.data = out,
				};
				const struct sr_datafeed_packet packet = {
					.type = SR_DF_LOGIC,
					.payload = &logic,
				};
				sr_session_send(sdi, &packet);
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
		fnirsi_dla_abort_acquisition(sdi);
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
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	drvc = cb_data;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	return TRUE;
}

SR_PRIV void fnirsi_dla_abort_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	unsigned int i;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->acq_aborted = TRUE;

	send_stop(usb->devhdl);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static int configure_channels(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	const GSList *l;
	struct sr_channel *ch;
	unsigned int n, hw_n, i, idx;
	unsigned int dst, v, t;
	gboolean used[MAX_CHANNELS];
	uint64_t max_rate;
	uint64_t lo, hi;

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

	/*
	 * Build the fast-path deinterleave() lookup tables for the current
	 * channel selection (see chan_lut_lo/hi in protocol.h). This is only
	 * O(num_active * 256) = at most 4096 table entries, done once here
	 * rather than per-sample.
	 */
	for (n = 0; n < devc->num_active; n++) {
		dst = devc->active_channels[n];

		for (v = 0; v < 256; v++) {
			lo = 0;
			hi = 0;

			for (t = 0; t < 4; t++) {
				if ((v >> t) & 1)
					lo |= (uint64_t)(1u << dst) << (16 * t);
			}
			for (t = 0; t < 4; t++) {
				if ((v >> (t + 4)) & 1)
					hi |= (uint64_t)(1u << dst) << (16 * t);
			}
			devc->chan_lut_lo[n][v] = lo;
			devc->chan_lut_hi[n][v] = hi;
		}
	}

	return SR_OK;
}

static uint16_t channel_mask(const struct dev_context *devc)
{
	uint16_t mask = 0;
	unsigned int i;

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

	devc->ctx = drvc->sr_ctx;
	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->limit_reached = FALSE;
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
		if (devc->acq_limit_samples > 0)
			pre_trigger_samples = (int)((devc->capture_ratio * devc->acq_limit_samples) / 100);
		devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
		if (!devc->stl)
			return SR_ERR_MALLOC;
		devc->trigger_fired = FALSE;
	} else {
		devc->stl = NULL;
		devc->trigger_fired = TRUE;
	}

	if ((ret = send_stop(usb->devhdl)) != SR_OK)
		goto err_free_stl;
	drain_sample_endpoint(usb->devhdl);
	if ((ret = send_parameter_setting(sdi)) != SR_OK)
		goto err_free_stl;
	if ((ret = send_simple_trigger(sdi, channel_mask(devc))) != SR_OK)
		goto err_free_stl;

	/* Worst case expansion is FNIRSI_DLA16_NUM_CHANNELS-fold (num_active
	 * == 1, the fixed 2-byte-wide wire sample word is emitted in full
	 * regardless), see protocol.h. Sized against transfer_bufsize (the
	 * actual per-transfer buffer size for this acquisition), not the
	 * fixed TRANSFER_BUFSIZE ceiling. */
	devc->outbuf_size = devc->transfer_bufsize * FNIRSI_DLA16_NUM_CHANNELS;
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

	usb_source_add(sdi->session, devc->ctx, USB_TIMEOUT_MS, receive_data, drvc);

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
			 * At least one transfer is genuinely in flight - let
			 * its (and any siblings') existing completion-callback
			 * path (free_transfer() -> finish_acquisition()) drive
			 * cleanup once cancellation completes, same as any
			 * other abort. Freeing devc->outbuf/transfers here too
			 * would race that callback and double-free them.
			 */
			fnirsi_dla_abort_acquisition(sdi);
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

	std_session_send_df_header(sdi);

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
