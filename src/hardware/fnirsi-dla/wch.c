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
#include <string.h>
#include "protocol.h"
#include "wch.h"

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <cfgmgr32.h>
#include <initguid.h>
#include <devpkey.h>

#define WCH_IOCTL			0x223CDC
/* Our own wait for a request is the driver's timeout plus this. */
#define DRIVER_MARGIN_MS		1000
/* How long a cancelled request may take to come back. */
#define CANCEL_LIMIT_MS			2000
/* The driver's timeout while a stream runs: it must never cut a read. */
#define STREAM_DRIVER_TIMEOUT_MS	300000
#define SETTING_WAIT_MS			2000

static const GUID wch_interface = { 0x5e7f6bdf, 0x1ce5, 0x4d78,
	{ 0xbb, 0xcf, 0xd2, 0x0c, 0x44, 0x32, 0x9f, 0x7d } };

struct dla_wch {
	HANDLE handle;
	uint32_t driver_timeout_ms;
	gboolean timeout_set;
	gboolean streaming;
	gboolean stranded;
};

/* One request in flight. Allocated on its own: a request the driver never
 * gives back keeps this memory for good, as the driver may still write
 * into it. */
struct dla_wch_io {
	OVERLAPPED ov;
	uint8_t *buf;
	size_t size;
	gboolean queued;
	uint64_t seq;
};

struct dla_wch_stream {
	struct dla_wch *wch;
	uint8_t header[DLA_WCH_HEADER_BYTES];
	uint32_t bytes;
	unsigned int num_slots;
	struct dla_wch_io **slots;
	uint64_t next_seq;
};

static struct dla_wch_io *io_new(size_t size)
{
	struct dla_wch_io *io;

	io = g_malloc0(sizeof(*io));
	io->size = size;
	io->buf = g_malloc0(size);
	io->ov.hEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
	if (!io->ov.hEvent) {
		g_free(io->buf);
		g_free(io);
		return NULL;
	}
	return io;
}

static void io_free(struct dla_wch_io *io)
{
	if (!io)
		return;
	CloseHandle(io->ov.hEvent);
	g_free(io->buf);
	g_free(io);
}

/*
 * Sends one request and waits for it at most wait_ms; a request still out
 * then is cancelled. The reply buffer is at least as large as the request:
 * the driver reports a write's header and data as bytes returned, and
 * Windows copies that many bytes back.
 */
static enum dla_wch_result exchange(struct dla_wch *wch, const uint8_t *req,
		size_t req_len, size_t reply_len, unsigned int wait_ms,
		struct dla_wch_io **reply, DWORD *moved)
{
	struct dla_wch_io *io;
	DWORD error;

	*reply = NULL;
	*moved = 0;
	io = io_new(reply_len > req_len ? reply_len : req_len);
	if (!io)
		return DLA_WCH_OTHER;

	if (!DeviceIoControl(wch->handle, WCH_IOCTL, (LPVOID)req,
			(DWORD)req_len, io->buf, (DWORD)io->size, NULL, &io->ov)) {
		error = GetLastError();
		if (error != ERROR_IO_PENDING) {
			io_free(io);
			return dla_wch_result_of(error);
		}
		if (WaitForSingleObject(io->ov.hEvent, wait_ms) != WAIT_OBJECT_0) {
			CancelIoEx(wch->handle, &io->ov);
			if (WaitForSingleObject(io->ov.hEvent, CANCEL_LIMIT_MS)
					!= WAIT_OBJECT_0) {
				wch->stranded = TRUE;
				sr_err("WCH driver kept a cancelled request: "
					"unplug and replug the device.");
				return DLA_WCH_TIMEOUT;
			}
		}
	}
	if (!GetOverlappedResult(wch->handle, &io->ov, moved, FALSE)) {
		error = GetLastError();
		io_free(io);
		return dla_wch_result_of(error);
	}
	*reply = io;
	return DLA_WCH_OK;
}

static enum dla_wch_result setting(struct dla_wch *wch,
		const uint8_t request[DLA_WCH_SETTING_BYTES])
{
	struct dla_wch_io *io;
	DWORD moved;
	enum dla_wch_result ret;

	ret = exchange(wch, request, DLA_WCH_SETTING_BYTES,
			DLA_WCH_SETTING_BYTES, SETTING_WAIT_MS, &io, &moved);
	io_free(io);
	return ret;
}

static enum dla_wch_result set_driver_timeout(struct dla_wch *wch,
		uint32_t timeout_ms)
{
	uint8_t request[DLA_WCH_SETTING_BYTES];
	enum dla_wch_result ret;

	if (wch->timeout_set && wch->driver_timeout_ms == timeout_ms)
		return DLA_WCH_OK;
	dla_wch_timeouts_request(timeout_ms, request);
	if ((ret = setting(wch, request)) != DLA_WCH_OK)
		return ret;
	wch->driver_timeout_ms = timeout_ms;
	wch->timeout_set = TRUE;
	return DLA_WCH_OK;
}

static char *narrow(const WCHAR *text)
{
	return g_utf16_to_utf8((const gunichar2 *)text, -1, NULL, NULL, NULL);
}

static char *instance_id_of(DEVINST node)
{
	WCHAR id[MAX_DEVICE_ID_LEN + 1];

	if (CM_Get_Device_IDW(node, id, MAX_DEVICE_ID_LEN + 1, 0) != CR_SUCCESS)
		return NULL;
	return narrow(id);
}

static gboolean uint32_property(DEVINST node, const DEVPROPKEY *key,
		uint32_t *value)
{
	DEVPROPTYPE type = 0;
	ULONG v = 0, size = sizeof(v);

	if (CM_Get_DevNode_PropertyW(node, key, &type, (PBYTE)&v, &size, 0)
			!= CR_SUCCESS || type != DEVPROP_TYPE_UINT32)
		return FALSE;
	*value = v;
	return TRUE;
}

static char *string_property(DEVINST node, const DEVPROPKEY *key)
{
	DEVPROPTYPE type = 0;
	ULONG size = 0;
	WCHAR *text;
	char *out;

	if (CM_Get_DevNode_PropertyW(node, key, &type, NULL, &size, 0)
			!= CR_BUFFER_SMALL || type != DEVPROP_TYPE_STRING ||
			size < sizeof(WCHAR))
		return NULL;
	text = g_malloc0(size + sizeof(WCHAR));
	if (CM_Get_DevNode_PropertyW(node, key, &type, (PBYTE)text, &size, 0)
			!= CR_SUCCESS) {
		g_free(text);
		return NULL;
	}
	out = narrow(text);
	g_free(text);
	return out;
}

/* The port path of a device node, root first as libusb gives it. */
static int port_path_of(DEVINST device, uint8_t *ports, size_t max_ports)
{
	uint32_t addresses[DLA_WCH_MAX_PORT_DEPTH];
	size_t count;
	DEVINST node, parent;
	char *parent_id;
	gboolean at_root;

	node = device;
	for (count = 0; count < DLA_WCH_MAX_PORT_DEPTH; count++) {
		if (!uint32_property(node, &DEVPKEY_Device_Address,
				&addresses[count]))
			return -1;
		if (CM_Get_Parent(&parent, node, 0) != CR_SUCCESS)
			return -1;
		if (!(parent_id = instance_id_of(parent)))
			return -1;
		at_root = dla_wch_is_root_hub(parent_id);
		g_free(parent_id);
		if (at_root)
			return dla_wch_port_path(addresses, count + 1,
					ports, max_ports);
		node = parent;
	}
	return -1;
}

static char *wch_interface_path(WCHAR *instance_id)
{
	GUID guid = wch_interface;
	ULONG length = 0;
	WCHAR *list;
	char *path;

	if (CM_Get_Device_Interface_List_SizeW(&length, &guid, instance_id,
			CM_GET_DEVICE_INTERFACE_LIST_PRESENT) != CR_SUCCESS ||
			length <= 1)
		return NULL;
	list = g_new0(WCHAR, length);
	if (CM_Get_Device_Interface_ListW(&guid, instance_id, list, length,
			CM_GET_DEVICE_INTERFACE_LIST_PRESENT) != CR_SUCCESS) {
		g_free(list);
		return NULL;
	}
	path = list[0] ? narrow(list) : NULL;
	g_free(list);
	return path;
}

char *dla_wch_find_path(uint16_t vid, uint16_t pid,
		const uint8_t *ports, int num_ports)
{
	const ULONG flags = CM_GETIDLIST_FILTER_ENUMERATOR |
		CM_GETIDLIST_FILTER_PRESENT;
	ULONG length = 0;
	WCHAR *ids, *id;
	char *narrowed, *service, *found;
	uint16_t id_vid, id_pid;
	uint8_t id_ports[DLA_WCH_MAX_PORT_DEPTH];
	int id_num_ports, matches;
	DEVINST node;

	if (num_ports <= 0 || CM_Get_Device_ID_List_SizeW(&length, L"USB",
			flags) != CR_SUCCESS)
		return NULL;
	ids = g_new0(WCHAR, length);
	if (CM_Get_Device_ID_ListW(L"USB", ids, length, flags) != CR_SUCCESS) {
		g_free(ids);
		return NULL;
	}

	found = NULL;
	matches = 0;
	for (id = ids; *id; id += wcslen(id) + 1) {
		if (!(narrowed = narrow(id)))
			continue;
		if (dla_wch_parse_instance_id(narrowed, &id_vid, &id_pid) < 0 ||
				id_vid != vid || id_pid != pid) {
			g_free(narrowed);
			continue;
		}
		g_free(narrowed);
		if (CM_Locate_DevNodeW(&node, id, CM_LOCATE_DEVNODE_NORMAL)
				!= CR_SUCCESS)
			continue;
		id_num_ports = port_path_of(node, id_ports, sizeof(id_ports));
		if (id_num_ports != num_ports ||
				memcmp(id_ports, ports, (size_t)num_ports) != 0)
			continue;
		matches++;
		service = string_property(node, &DEVPKEY_Device_Service);
		if (service && dla_wch_is_wch_service(service) && !found)
			found = wch_interface_path(id);
		g_free(service);
	}
	g_free(ids);

	/* Two devices on the same port path under different root hubs:
	 * which one libusb means is not known. */
	if (matches > 1) {
		sr_warn("Two USB devices %04x:%04x share this port path; not "
			"using WCH's driver for either.", vid, pid);
		g_free(found);
		return NULL;
	}
	return found;
}

enum dla_wch_result dla_wch_open(const char *path, struct dla_wch **wch)
{
	uint8_t request[DLA_WCH_SETTING_BYTES];
	struct dla_wch *w;
	HANDLE handle;
	enum dla_wch_result ret;

	*wch = NULL;
	handle = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
			OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
	if (handle == INVALID_HANDLE_VALUE)
		return dla_wch_result_of(GetLastError());

	w = g_malloc0(sizeof(*w));
	w->handle = handle;
	/* Without asking for exclusive use a second open succeeds too. */
	dla_wch_exclusive_request(1, request);
	if ((ret = setting(w, request)) != DLA_WCH_OK) {
		CloseHandle(handle);
		g_free(w);
		return ret;
	}
	*wch = w;
	return DLA_WCH_OK;
}

void dla_wch_close(struct dla_wch *wch)
{
	if (!wch)
		return;
	CloseHandle(wch->handle);
	g_free(wch);
}

enum dla_wch_result dla_wch_bulk(struct dla_wch *wch, uint8_t endpoint,
		uint8_t *data, int length, int *actual, unsigned int timeout_ms)
{
	struct dla_wch_io *io;
	uint8_t header[DLA_WCH_HEADER_BYTES];
	uint8_t *request;
	uint32_t moved_bytes;
	DWORD moved;
	enum dla_wch_result ret;

	*actual = 0;
	if (wch->stranded || length <= 0)
		return DLA_WCH_OTHER;
	if (!wch->streaming &&
			(ret = set_driver_timeout(wch, timeout_ms)) != DLA_WCH_OK)
		return ret;

	if (endpoint & 0x80) {
		if (dla_wch_read_header(endpoint, (uint32_t)length, header) < 0)
			return DLA_WCH_OTHER;
		ret = exchange(wch, header, sizeof(header),
				DLA_WCH_HEADER_BYTES + (size_t)length,
				timeout_ms + DRIVER_MARGIN_MS, &io, &moved);
	} else {
		request = g_malloc(DLA_WCH_HEADER_BYTES + (size_t)length);
		if (dla_wch_write_header(endpoint, (uint32_t)length,
				request) < 0) {
			g_free(request);
			return DLA_WCH_OTHER;
		}
		memcpy(request + DLA_WCH_HEADER_BYTES, data, (size_t)length);
		ret = exchange(wch, request, DLA_WCH_HEADER_BYTES + (size_t)length,
				DLA_WCH_HEADER_BYTES + (size_t)length,
				timeout_ms + DRIVER_MARGIN_MS, &io, &moved);
		g_free(request);
	}
	if (ret == DLA_WCH_CANCELLED)
		return DLA_WCH_TIMEOUT;
	if (ret != DLA_WCH_OK)
		return ret;

	if (dla_wch_reply_length(io->buf, moved, (uint32_t)length,
			&moved_bytes) < 0) {
		io_free(io);
		return DLA_WCH_OTHER;
	}
	if (endpoint & 0x80)
		memcpy(data, io->buf + DLA_WCH_HEADER_BYTES, moved_bytes);
	io_free(io);
	*actual = (int)moved_bytes;
	/* The driver's own timeout: a read of nothing, a write cut short. */
	if ((endpoint & 0x80) ? moved_bytes == 0 : moved_bytes < (uint32_t)length)
		return DLA_WCH_TIMEOUT;
	return DLA_WCH_OK;
}

struct dla_wch_stream *dla_wch_stream_new(struct dla_wch *wch,
		uint8_t endpoint, unsigned int slots, size_t bytes)
{
	struct dla_wch_stream *stream;
	unsigned int i;

	if (wch->stranded || slots == 0 || bytes > DLA_WCH_MAX_TRANSFER_BYTES)
		return NULL;
	stream = g_malloc0(sizeof(*stream));
	if (dla_wch_read_header(endpoint, (uint32_t)bytes, stream->header) < 0 ||
			set_driver_timeout(wch, STREAM_DRIVER_TIMEOUT_MS)
				!= DLA_WCH_OK) {
		g_free(stream);
		return NULL;
	}
	stream->wch = wch;
	stream->bytes = (uint32_t)bytes;
	stream->num_slots = slots;
	stream->slots = g_new0(struct dla_wch_io *, slots);
	for (i = 0; i < slots; i++) {
		stream->slots[i] = io_new(DLA_WCH_HEADER_BYTES + bytes);
		if (!stream->slots[i]) {
			dla_wch_stream_free(stream);
			return NULL;
		}
	}
	wch->streaming = TRUE;
	return stream;
}

enum dla_wch_result dla_wch_stream_submit(struct dla_wch_stream *stream,
		unsigned int slot)
{
	struct dla_wch_io *io;
	HANDLE event;
	DWORD error;

	io = stream->slots[slot];
	event = io->ov.hEvent;
	memset(&io->ov, 0, sizeof(io->ov));
	io->ov.hEvent = event;
	ResetEvent(event);
	io->seq = ++stream->next_seq;
	io->queued = TRUE;
	if (DeviceIoControl(stream->wch->handle, WCH_IOCTL, stream->header,
			sizeof(stream->header), io->buf, (DWORD)io->size, NULL,
			&io->ov))
		return DLA_WCH_OK;
	error = GetLastError();
	if (error == ERROR_IO_PENDING)
		return DLA_WCH_OK;
	io->queued = FALSE;
	return dla_wch_result_of(error);
}

int dla_wch_stream_next(struct dla_wch_stream *stream,
		unsigned int wait_ms, unsigned int *slot,
		enum dla_wch_result *result, const uint8_t **data,
		size_t *length)
{
	struct dla_wch_io *io, *oldest;
	unsigned int i, oldest_slot;
	uint32_t moved_bytes;
	DWORD moved;

	oldest = NULL;
	oldest_slot = 0;
	for (i = 0; i < stream->num_slots; i++) {
		io = stream->slots[i];
		if (io && io->queued && (!oldest || io->seq < oldest->seq)) {
			oldest = io;
			oldest_slot = i;
		}
	}
	if (!oldest)
		return FALSE;
	if (!HasOverlappedIoCompleted(&oldest->ov)) {
		if (wait_ms == 0 || WaitForSingleObject(oldest->ov.hEvent,
				wait_ms) != WAIT_OBJECT_0)
			return FALSE;
	}

	oldest->queued = FALSE;
	*slot = oldest_slot;
	*data = NULL;
	*length = 0;
	if (!GetOverlappedResult(stream->wch->handle, &oldest->ov, &moved,
			FALSE)) {
		*result = dla_wch_result_of(GetLastError());
		return TRUE;
	}
	if (dla_wch_reply_length(oldest->buf, moved, stream->bytes,
			&moved_bytes) < 0) {
		*result = DLA_WCH_OTHER;
		return TRUE;
	}
	*result = DLA_WCH_OK;
	*data = oldest->buf + DLA_WCH_HEADER_BYTES;
	*length = moved_bytes;
	return TRUE;
}

unsigned int dla_wch_stream_queued(const struct dla_wch_stream *stream)
{
	unsigned int i, n;

	n = 0;
	for (i = 0; i < stream->num_slots; i++) {
		if (stream->slots[i] && stream->slots[i]->queued)
			n++;
	}
	return n;
}

void dla_wch_stream_cancel(struct dla_wch_stream *stream)
{
	unsigned int i;

	for (i = 0; i < stream->num_slots; i++) {
		if (stream->slots[i] && stream->slots[i]->queued)
			CancelIoEx(stream->wch->handle, &stream->slots[i]->ov);
	}
}

void dla_wch_stream_free(struct dla_wch_stream *stream)
{
	struct dla_wch_io *io;
	unsigned int i;

	if (!stream)
		return;
	dla_wch_stream_cancel(stream);
	for (i = 0; i < stream->num_slots; i++) {
		io = stream->slots[i];
		if (!io)
			continue;
		if (io->queued && WaitForSingleObject(io->ov.hEvent,
				CANCEL_LIMIT_MS) != WAIT_OBJECT_0) {
			stream->wch->stranded = TRUE;
			sr_err("WCH driver kept a cancelled read: unplug and "
				"replug the device.");
			continue;
		}
		io_free(io);
	}
	stream->wch->streaming = FALSE;
	stream->wch->timeout_set = FALSE;
	g_free(stream->slots);
	g_free(stream);
}

#else

char *dla_wch_find_path(uint16_t vid, uint16_t pid,
		const uint8_t *ports, int num_ports)
{
	(void)vid;
	(void)pid;
	(void)ports;
	(void)num_ports;
	return NULL;
}

enum dla_wch_result dla_wch_open(const char *path, struct dla_wch **wch)
{
	(void)path;
	*wch = NULL;
	return DLA_WCH_OTHER;
}

void dla_wch_close(struct dla_wch *wch)
{
	(void)wch;
}

enum dla_wch_result dla_wch_bulk(struct dla_wch *wch, uint8_t endpoint,
		uint8_t *data, int length, int *actual, unsigned int timeout_ms)
{
	(void)wch;
	(void)endpoint;
	(void)data;
	(void)length;
	(void)timeout_ms;
	*actual = 0;
	return DLA_WCH_OTHER;
}

struct dla_wch_stream *dla_wch_stream_new(struct dla_wch *wch,
		uint8_t endpoint, unsigned int slots, size_t bytes)
{
	(void)wch;
	(void)endpoint;
	(void)slots;
	(void)bytes;
	return NULL;
}

enum dla_wch_result dla_wch_stream_submit(struct dla_wch_stream *stream,
		unsigned int slot)
{
	(void)stream;
	(void)slot;
	return DLA_WCH_OTHER;
}

int dla_wch_stream_next(struct dla_wch_stream *stream,
		unsigned int wait_ms, unsigned int *slot,
		enum dla_wch_result *result, const uint8_t **data,
		size_t *length)
{
	(void)stream;
	(void)wait_ms;
	(void)slot;
	(void)result;
	(void)data;
	(void)length;
	return FALSE;
}

unsigned int dla_wch_stream_queued(const struct dla_wch_stream *stream)
{
	(void)stream;
	return 0;
}

void dla_wch_stream_cancel(struct dla_wch_stream *stream)
{
	(void)stream;
}

void dla_wch_stream_free(struct dla_wch_stream *stream)
{
	(void)stream;
}

#endif
