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
#include <math.h>
#include "protocol.h"

#define DEFAULT_SAMPLERATE	SR_MHZ(10)
#define DEFAULT_THRESHOLD	1.50 /* volts, on the vendor's declared 0.1V grid */

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
	SR_CONF_PROBE_NAMES,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_NUM_LOGIC_CHANNELS | SR_CONF_GET,
	SR_CONF_LOGIC_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LOGIC_THRESHOLD_CUSTOM | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
};

/*
 * Named threshold presets, exposed via the standard SR_CONF_LOGIC_THRESHOLD
 * string config key for convenience - these are just friendly shortcuts to
 * specific SR_CONF_LOGIC_THRESHOLD_CUSTOM voltages using the exact same
 * underlying wire mechanism.
 */
static const struct {
	const char *name;
	double volts;
} threshold_presets[] = {
	{ "5V CMOS",   2.50 },
	{ "3.3V CMOS", 1.60 },
	{ "3V CMOS",   1.50 },
	{ "2.5V CMOS", 1.20 },
	{ "1.8V CMOS", 0.90 },
	{ "1.5V CMOS", 0.70 },
	{ "1.2V CMOS", 0.60 },
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

/* Sized to MAX_CHANNELS so the array is ready for a wider future model, but
 * only the first FNIRSI_DLA16_NUM_CHANNELS (16) names are ever used today -
 * see devc->num_channels in protocol.h. */
static const char *channel_names[MAX_CHANNELS] = {
	"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7",
	"D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15",
	"D16", "D17", "D18", "D19", "D20", "D21", "D22", "D23",
	"D24", "D25", "D26", "D27", "D28", "D29", "D30", "D31",
};

static GSList *fnirsi_dla_scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct sr_channel_group *cg;
	struct sr_config *src;
	GSList *l, *devices, *conn_devices;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	int device_count, i;
	const char *conn;
	const char *probe_names;
	char connection_id[64];
	unsigned int j;
	size_t ch_max;
	char **names;

	drvc = di->context;

	conn = NULL;
	probe_names = NULL;
	for (l = options; l; l = l->next) {
		src = l->data;
		if (src->key == SR_CONF_CONN)
			conn = g_variant_get_string(src->data, NULL);
		else if (src->key == SR_CONF_PROBE_NAMES)
			probe_names = g_variant_get_string(src->data, NULL);
	}
	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	else
		conn_devices = NULL;

	devices = NULL;
	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
				libusb_error_name(device_count));
		g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);
		return NULL;
	}

	for (i = 0; i < device_count; i++) {
		if (conn) {
			usb = NULL;
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i])
						&& usb->address == libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				continue;
		}

		libusb_get_device_descriptor(devlist[i], &des);
		if (des.idVendor != FNIRSI_DLA16_VID || des.idProduct != FNIRSI_DLA16_PID)
			continue;

		if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
			continue;

		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->status = SR_ST_INACTIVE;
		sdi->vendor = g_strdup("FNIRSI");
		sdi->model = g_strdup("DLA-16 Plus");
		sdi->connection_id = g_strdup(connection_id);
		sdi->inst_type = SR_INST_USB;
		sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
				libusb_get_device_address(devlist[i]), NULL);

		devc = g_malloc0(sizeof(struct dev_context));
		devc->sdi = sdi;
		/* FNIRSI_DLA16_VID/PID is the only pair this driver recognizes
		 * (see the match above), so this is always a 16-channel unit -
		 * a real second model would need its own VID/PID match (or
		 * some other runtime-detectable signal) setting a different
		 * value here; nothing else in this driver has been written or
		 * verified for any channel count but 16. */
		devc->num_channels = FNIRSI_DLA16_NUM_CHANNELS;
		devc->cur_samplerate = DEFAULT_SAMPLERATE;
		devc->cur_rate_idx = 0x04; /* matches DEFAULT_SAMPLERATE, 10MHz */
		devc->threshold_hundredths = (int16_t)(DEFAULT_THRESHOLD * 100);
		sdi->priv = devc;

		/*
		 * All 16 physical logic channels are always created - the
		 * channel index is tied directly to the physical channel
		 * number (used verbatim as the hardware channel-mask bit
		 * position), so unlike some other drivers this cannot honor
		 * a probe_names spec that asks for *fewer* channels, only
		 * custom names for the full set. Any name the spec didn't
		 * cover falls back to the default "Dn".
		 */
		ch_max = devc->num_channels;
		names = sr_parse_probe_names(probe_names,
			channel_names, devc->num_channels, devc->num_channels,
			&ch_max);

		cg = sr_channel_group_new(sdi, "Logic", NULL);
		for (j = 0; j < devc->num_channels; j++) {
			const char *name = (j < ch_max) ? names[j] : channel_names[j];
			struct sr_channel *ch = sr_channel_new(sdi, (int)j,
					SR_CHANNEL_LOGIC, TRUE, name);
			cg->channels = g_slist_append(cg->channels, ch);
		}
		sr_free_probe_names(names);

		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

static int fnirsi_dla_dev_open(struct sr_dev_inst *sdi)
{
	struct drv_context *drvc;
	struct sr_usb_dev_inst *usb;
	libusb_device **devlist;
	struct libusb_device_descriptor des;
	int device_count, i, ret;

	drvc = sdi->driver->context;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
				libusb_error_name(device_count));
		return SR_ERR;
	}

	ret = SR_ERR;
	for (i = 0; i < device_count; i++) {
		if (libusb_get_bus_number(devlist[i]) != usb->bus ||
				libusb_get_device_address(devlist[i]) != usb->address)
			continue;

		libusb_get_device_descriptor(devlist[i], &des);
		if (des.idVendor != FNIRSI_DLA16_VID || des.idProduct != FNIRSI_DLA16_PID)
			continue;

		ret = libusb_get_device_speed(devlist[i]);
		if (ret < LIBUSB_SPEED_SUPER) {
			sr_err("Device is connected at %s, not USB 3.0 SuperSpeed - "
				"refusing to open. Reconnect it via a USB 3.0 port/cable.",
				ret == LIBUSB_SPEED_HIGH ? "USB 2.0 High Speed" :
				ret == LIBUSB_SPEED_FULL ? "USB 1.1 Full Speed" :
				ret == LIBUSB_SPEED_LOW ? "USB 1.0 Low Speed" :
				"an unknown/unrecognized speed");
			ret = SR_ERR;
			break;
		}

		if ((ret = libusb_open(devlist[i], &usb->devhdl)) < 0) {
			sr_err("Failed to open device: %s.", libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER))
			libusb_set_auto_detach_kernel_driver(usb->devhdl, 1);

		if ((ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE)) != 0) {
			sr_err("Failed to claim interface: %s.", libusb_error_name(ret));
			sr_usb_close(usb);
			ret = SR_ERR;
			break;
		}

		if ((ret = fnirsi_dla_connect(sdi)) != SR_OK)
			sr_usb_close(usb);
		break;
	}
	libusb_free_device_list(devlist, 1);

	return ret;
}

static int fnirsi_dla_dev_close(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;
	if (!usb->devhdl)
		return SR_OK;

	devc = sdi->priv;
	if (devc->transfers) {
		fnirsi_dla_abort_acquisition(sdi);
		fnirsi_dla_drain_pending_transfers(devc);
	}

	libusb_release_interface(usb->devhdl, USB_INTERFACE);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return SR_OK;
}

static int fnirsi_dla_config_get(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	unsigned int i;

	if (!sdi)
		return SR_ERR_ARG;
	devc = sdi->priv;

	(void)cg;

	switch (key) {
	case SR_CONF_CONN:
		if (!sdi->conn)
			return SR_ERR_ARG;
		usb = sdi->conn;
		*data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_LIMIT_MSEC:
		*data = g_variant_new_uint64(devc->limit_msec);
		break;
	case SR_CONF_NUM_LOGIC_CHANNELS:
		*data = g_variant_new_int32(devc->num_channels);
		break;
	case SR_CONF_LOGIC_THRESHOLD_CUSTOM:
		*data = g_variant_new_double(devc->threshold_hundredths / 100.0);
		break;
	case SR_CONF_LOGIC_THRESHOLD:
		for (i = 0; i < ARRAY_SIZE(threshold_presets); i++) {
			if ((int16_t)lround(threshold_presets[i].volts * 100.0)
					== devc->threshold_hundredths) {
				*data = g_variant_new_string(threshold_presets[i].name);
				break;
			}
		}
		if (i == ARRAY_SIZE(threshold_presets))
			return SR_ERR_NA;
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int fnirsi_dla_config_set(uint32_t key, GVariant *data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t rate;
	uint64_t capture_ratio;
	double volts;
	long hundredths;
	unsigned int i;
	const char *preset_names[ARRAY_SIZE(threshold_presets)];
	int idx;

	if (!sdi)
		return SR_ERR_ARG;
	devc = sdi->priv;

	(void)cg;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		rate = g_variant_get_uint64(data);
		for (i = 0; i < fnirsi_dla_num_rates; i++) {
			if (fnirsi_dla_rates[i].rate_hz == rate) {
				devc->cur_samplerate = rate;
				devc->cur_rate_idx = fnirsi_dla_rates[i].rate_idx;
				return SR_OK;
			}
		}
		sr_err("Unsupported samplerate %" PRIu64 "Hz.", rate);
		return SR_ERR_ARG;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_MSEC:
		devc->limit_msec = g_variant_get_uint64(data);
		break;
	case SR_CONF_LOGIC_THRESHOLD_CUSTOM:
		volts = g_variant_get_double(data);
		if (volts < -5.0 || volts > 5.0) {
			sr_err("Threshold %.2fV out of the vendor-declared "
				"+/-5V range.", volts);
			return SR_ERR_ARG;
		}
		hundredths = lround(volts * 10.0) * 10;
		if (hundredths != lround(volts * 100.0)) {
			sr_info("Rounding threshold %.2fV to %.1fV "
				"(0.1V grid).", volts, hundredths / 100.0);
		}
		devc->threshold_hundredths = (int16_t)hundredths;
		break;
	case SR_CONF_LOGIC_THRESHOLD:
		for (i = 0; i < ARRAY_SIZE(threshold_presets); i++)
			preset_names[i] = threshold_presets[i].name;
		idx = std_str_idx(data, preset_names, ARRAY_SIZE(preset_names));
		if (idx < 0) {
			sr_err("Unknown logic threshold preset '%s'.",
				g_variant_get_string(data, NULL));
			return SR_ERR_ARG;
		}
		devc->threshold_hundredths =
			(int16_t)lround(threshold_presets[idx].volts * 100.0);
		break;
	case SR_CONF_CAPTURE_RATIO:
		capture_ratio = g_variant_get_uint64(data);
		if (capture_ratio > 100) {
			sr_err("Capture ratio %" PRIu64 " out of range (0-100).",
				capture_ratio);
			return SR_ERR_ARG;
		}
		devc->capture_ratio = capture_ratio;
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int fnirsi_dla_config_list(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	uint64_t *rates;
	const char *preset_names[ARRAY_SIZE(threshold_presets)];
	unsigned int i;

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		if (cg)
			return SR_ERR_NA;
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		rates = g_malloc(fnirsi_dla_num_rates * sizeof(rates[0]));
		for (i = 0; i < fnirsi_dla_num_rates; i++)
			rates[i] = fnirsi_dla_rates[i].rate_hz;
		*data = std_gvar_samplerates(rates, fnirsi_dla_num_rates);
		g_free(rates);
		break;
	case SR_CONF_LOGIC_THRESHOLD:
		for (i = 0; i < ARRAY_SIZE(threshold_presets); i++)
			preset_names[i] = threshold_presets[i].name;
		*data = std_gvar_array_str(preset_names, ARRAY_SIZE(preset_names));
		break;
	case SR_CONF_LOGIC_THRESHOLD_CUSTOM:
		*data = std_gvar_min_max_step(-5.0, 5.0, 0.1);
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int fnirsi_dla_dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	fnirsi_dla_abort_acquisition(sdi);

	return SR_OK;
}

static void clear_helper(struct dev_context *devc)
{
	g_free(devc->outbuf);
	g_free(devc->transfers);
	if (devc->stl)
		soft_trigger_logic_free(devc->stl);
}

static int fnirsi_dla_dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear_with_callback(di, (std_dev_clear_callback)clear_helper);
}

static struct sr_dev_driver fnirsi_dla_driver_info = {
	.name = "fnirsi-dla",
	.longname = "FNIRSI DLA",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = fnirsi_dla_scan,
	.dev_list = std_dev_list,
	.dev_clear = fnirsi_dla_dev_clear,
	.config_get = fnirsi_dla_config_get,
	.config_set = fnirsi_dla_config_set,
	.config_list = fnirsi_dla_config_list,
	.dev_open = fnirsi_dla_dev_open,
	.dev_close = fnirsi_dla_dev_close,
	.dev_acquisition_start = fnirsi_dla_start_acquisition,
	.dev_acquisition_stop = fnirsi_dla_dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(fnirsi_dla_driver_info);
