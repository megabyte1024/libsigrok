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
 * Driver for the FNIRSI DLA series USB logic analyzers. Currently only the
 * DLA-16 Plus (USB ID 1a86:5537, 16 channels) is actually supported/tested
 * against real hardware.
 *
 * This driver deliberately only ever uses the device's "Streaming" data
 * cache mode, never "Buffer" mode (a finite depth).
 */

#ifndef LIBSIGROK_HARDWARE_FNIRSI_DLA_PROTOCOL_H
#define LIBSIGROK_HARDWARE_FNIRSI_DLA_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

struct dla_wch;
struct dla_wch_stream;

#define LOG_PREFIX "fnirsi-dla"

/* USB VID/PID for the DLA-16 Plus. */
#define FNIRSI_DLA16_VID	0x1a86
#define FNIRSI_DLA16_PID	0x5537

#define USB_INTERFACE		0

#define EP_CMD_OUT			0x02
#define EP_CMD_REPLY_IN			0x82
#define EP_SAMPLE_IN			0x81

/* Size of every "direct" (unframed) command and its reply. */
#define USB_CMD_SIZE		512

/* Framed command length alignment. */
#define FRAME_ALIGN		2048

/* Scratch buffer size for build_framed()'s unpadded command body. */
#define MAX_FRAME_BODY		256

/* DLA-16's hardware channel count. */
#define FNIRSI_DLA16_NUM_CHANNELS	16

/* Bytes per sigrok logic sample holding all of the DLA-16's channels; see
 * unitsize in struct dev_context for when 1 byte is used instead. */
#define FNIRSI_DLA16_SAMPLE_UNITSIZE	2

/* Array-sizing cap for channel-indexed arrays. */
#define MAX_CHANNELS		32

/* Max aggregate samplerate across all enabled channels combined. */
#define AGGREGATE_SAMPLERATE_MAX	SR_GHZ(2)

#define USB_TIMEOUT_MS		2000
/* How long a direct command's reply may take. */
#define CMD_REPLY_TIMEOUT_MS	1000
/* How many times a query the device didn't take is written in all. */
#define QUERY_MAX_WRITES	6

#define DRAIN_POLL_INTERVAL_MS	100
#define DRAIN_MAX_WAIT_MS	(USB_TIMEOUT_MS * 2)

/* Clearing stale sample data before an acquisition: read size, how long
 * one read may wait for data, and how long the device may keep sending
 * before the acquisition is refused. */
#define STALE_READ_BYTES	(16 * 1024)
#define STALE_READ_TIMEOUT_MS	20
#define STALE_DRAIN_LIMIT_MS	1000

/* USB 3.0 SuperSpeed bulk endpoint max packet size. */
#define USB_SS_MAX_PACKET_SIZE	1024

#define NUM_SIMUL_TRANSFERS	8
#define MAX_EMPTY_TRANSFERS	(NUM_SIMUL_TRANSFERS * 2)
/* On WCH's driver: the session polls every WCH_POLL_PERIOD_MS and waits at
 * most WCH_POLL_WAIT_MS for the oldest read; nothing read for
 * WCH_SILENCE_MS ends the acquisition, as MAX_EMPTY_TRANSFERS timed-out
 * libusb transfers do. */
#define WCH_NUM_READS		16
#define WCH_POLL_PERIOD_MS	1
#define WCH_POLL_WAIT_MS	10
#define WCH_SILENCE_MS		(USB_TIMEOUT_MS * MAX_EMPTY_TRANSFERS / NUM_SIMUL_TRANSFERS)
/* Ceiling for the per-transfer USB buffer size. */
#define TRANSFER_BUFSIZE	(1024 * 1024)
/* Floor for the per-transfer USB buffer size. */
#define MIN_TRANSFER_BUFSIZE	(16 * 1024)
/* Target transfer-buffer size, in milliseconds of aggregate sample data. */
#define TARGET_TRANSFER_MS	20
/* Time dropped at the start of every acquisition, so the input threshold
 * just sent with ParameterSetting has settled. Dropped in whole interleave
 * groups (8 samples of every wire channel) to keep deinterleave() aligned. */
#define PREROLL_MS		50
/* How far delivery may fall behind sampling before the acquisition is
 * aborted. The device drops data silently, with nothing in its stream to
 * show it, once the host falls about 120 MB behind; clean runs were
 * measured at up to about 47 MB behind. */
#define LAG_LIMIT_BYTES		(80ULL * 1024 * 1024)

struct fnirsi_dla_rate {
	uint64_t rate_hz;
	uint8_t rate_idx;
};

struct dev_context {
	struct sr_dev_inst *sdi;

	uint64_t cur_samplerate;
	uint8_t cur_rate_idx;

	/* Threshold voltage, hundredths of a volt, signed 16-bit range as
	 * used on the wire (0x11 payload offsets 10-11). Vendor-declared
	 * range is +/-5.00V; any 0.01V step is accepted and sent, though
	 * the vendor software offers 0.1V steps only. */
	int16_t threshold_hundredths;

	uint64_t limit_samples;
	/* Time-based limit, milliseconds - an alternate way to bound the
	 * acquisition, converted to a sample count against cur_samplerate at
	 * acquisition start. If both this and limit_samples are set, the
	 * more restrictive (smaller sample count) of the two wins. */
	uint64_t limit_msec;
	/* Effective sample-count bound for the acquisition in progress,
	 * derived at acquisition start from limit_samples and limit_msec
	 * (whichever is more restrictive). Deliberately separate from
	 * limit_samples: deriving it in place would overwrite the user's own
	 * configured value, so a later acquisition at a different samplerate
	 * would reuse the previous run's derived bound instead of converting
	 * limit_msec afresh. */
	uint64_t acq_limit_samples;
	uint64_t sent_samples;
	/* Wire bytes still to drop at the start of the acquisition, see
	 * PREROLL_MS. */
	uint64_t preroll_bytes_left;

	/* Delivery lag tracking, see LAG_LIMIT_BYTES: the wire byte rate,
	 * the bytes received so far, when the first transfer arrived, and
	 * the least the delivery has been behind sampling since then. */
	uint64_t wire_bytes_per_sec;
	uint64_t received_bytes;
	gboolean lag_started;
	int64_t lag_start_ms;
	int64_t least_behind;

	/* Model code self-reported by the device in its GetDeviceData (0x10)
	 * reply, parsed in fnirsi_dla_connect() - "DL16" on every DLA-16 Plus
	 * unit tested. Empty until connect() runs. */
	char model_code[8];

	/* Percentage (0-100) of limit_samples that should be captured
	 * before the trigger point - only meaningful together with a
	 * trigger and a nonzero limit_samples. */
	uint64_t capture_ratio;

	/* Software trigger support. */
	struct soft_trigger_logic *stl;
	gboolean trigger_fired;

	unsigned int empty_transfer_count;

	gboolean acq_aborted;
	/* Set once limit_samples has been reached: stop resubmitting
	 * transfers, but let any still-genuinely-in-flight ones complete and
	 * drain naturally instead of force-cancelling them - force-cancelling
	 * siblings that hadn't finished receiving their share yet drops
	 * legitimate data, short by exactly one transfer's worth of samples. */
	gboolean limit_reached;
	/* Set once Stop has been sent for the acquisition in progress. */
	gboolean stop_sent;
	/* Set when the acquisition ended without Stop (e.g. limit_samples
	 * reached), so the device is still streaming. Stop is then sent by
	 * fnirsi_dla_send_pending_stop() from receive_data() or dev_close(),
	 * never from a transfer callback: a synchronous libusb transfer made
	 * inside one can't complete, as that thread already holds libusb's
	 * event lock. */
	gboolean stop_pending;
	/* Set when a transfer reported the device unplugged: no Stop or Exit
	 * is sent to it any more. */
	gboolean device_gone;
	/* Set when connect found the device in its bootloader: no Exit is
	 * sent to it. */
	gboolean in_bootloader;

	/* Number of physical logic channels the connected device actually
	 * has - detected at scan time (see scan() in api.c), always
	 * FNIRSI_DLA16_NUM_CHANNELS today since that's the only VID/PID this
	 * driver recognizes. Runtime value rather than a compile-time
	 * constant; MAX_CHANNELS only bounds array storage. */
	unsigned int num_channels;

	/* List of logic channel indices (0 to num_channels-1) actually
	 * sampled on the wire, and how many there are - always one of the
	 * hardware's only valid counts (1, 2, 4, 8, 16 for the 16-channel
	 * hardware this driver supports). The user-enabled channels come
	 * first, followed by any padding channels configure_channels() had to
	 * add to round a non-power-of-2 request up to the next valid count
	 * (see its comment); padding channels are never marked enabled on
	 * sdi, so their data is captured but never forwarded to the session. */
	uint8_t active_channels[MAX_CHANNELS];
	unsigned int num_active;

	/* Deinterleave state, carried across USB transfers so short reads
	 * can't desync the channel round-robin: partially filled interleave
	 * group. */
	uint8_t group_buf[MAX_CHANNELS];
	unsigned int group_fill;

	/* Bytes per SR_DF_LOGIC sample for the acquisition in progress:
	 * 1 when every wire channel is D0-D7 and no software trigger is set,
	 * else FNIRSI_DLA16_SAMPLE_UNITSIZE. */
	unsigned int unitsize;

	/* Per-physical-channel bit-transpose lookup tables used by the fast
	 * path in deinterleave() (protocol.c). With 1-byte samples only
	 * chan_lut_lo is used. */
	uint64_t chan_lut_lo[MAX_CHANNELS][256];
	uint64_t chan_lut_hi[MAX_CHANNELS][256];

	/* Scratch buffer for deinterleaved SR_DF_LOGIC output, sized for the
	 * worst case (num_active == 1, 8 * unitsize expansion). */
	uint8_t *outbuf;
	size_t outbuf_size;

	/* Per-transfer USB buffer size actually used for the current
	 * acquisition, chosen by compute_transfer_bufsize() from the
	 * configured aggregate rate - between MIN_TRANSFER_BUFSIZE and
	 * TRANSFER_BUFSIZE (protocol.h). */
	size_t transfer_bufsize;

	unsigned int num_transfers;
	struct libusb_transfer **transfers;
	struct sr_context *ctx;

	/* Windows only: the device is on WCH's driver instead of WinUSB, and
	 * the reads queued on it while an acquisition runs (see wch.h). */
	struct dla_wch *wch;
	struct dla_wch_stream *wch_stream;
	uint8_t *wch_chunk;
	const struct sr_dev_inst *wch_sdi;
	int64_t wch_last_data_ms;
	/* The acquisition ended on an error, not on STOP or its limit. */
	gboolean acq_failed;
	/* A capture on WCH's driver failed: only a replug recovers it. */
	gboolean wch_replug_needed;
};

SR_PRIV extern const struct fnirsi_dla_rate fnirsi_dla_rates[];
SR_PRIV extern const unsigned int fnirsi_dla_num_rates;

SR_PRIV int fnirsi_dla_connect(struct sr_dev_inst *sdi);
SR_PRIV int fnirsi_dla_start_acquisition(const struct sr_dev_inst *sdi);
SR_PRIV void fnirsi_dla_abort_acquisition(const struct sr_dev_inst *sdi);
SR_PRIV void fnirsi_dla_send_pending_stop(const struct sr_dev_inst *sdi);
SR_PRIV int fnirsi_dla_send_exit(const struct sr_dev_inst *sdi);
SR_PRIV void fnirsi_dla_drain_pending_transfers(struct dev_context *devc);

#endif
