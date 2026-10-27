#include "stdafx.h"
#include "PSP.h"
#include "Emu/IdManager.h"
#include "Emu/Cell/lv2/sys_usbd.h"
#include "Emu/system_config.h"
#include "Input/pad_thread.h"
#include <Utilities/Thread.h>
#include <algorithm>
#include <cstring>
#include <thread>

LOG_CHANNEL(psp_log);

usb_device_psp::usb_device_psp(const std::array<u8, 7>& location, const pspcm_descriptors* desc)
	: usb_device_emulated(location)
{
	device = UsbDescriptorNode(USB_DESCRIPTOR_DEVICE,
		UsbDeviceDescriptor {
			.bcdUSB             = 0x0200,
			.bDeviceClass       = 0x00,
			.bDeviceSubClass    = 0x00,
			.bDeviceProtocol    = 0x00,
			.bMaxPacketSize0    = 0x40,
			.idVendor           = 0x054c,
			.idProduct          = 0x01cb,
			.bcdDevice          = 0x0100,
			.iManufacturer      = 0x01,
			.iProduct           = 0x02,
			.iSerialNumber      = 0x00,
			.bNumConfigurations = 0x01});
	auto& config0 = device.add_node(UsbDescriptorNode(USB_DESCRIPTOR_CONFIG,
		UsbDeviceConfiguration {
			.wTotalLength        = 0x0027,
			.bNumInterfaces      = 0x01,
			.bConfigurationValue = 0x01,
			.iConfiguration      = 0x00,
			.bmAttributes        = 0xc0,
			.bMaxPower           = 0x01}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_INTERFACE,
		UsbDeviceInterface {
			.bInterfaceNumber   = 0x00,
			.bAlternateSetting  = 0x00,
			.bNumEndpoints      = 0x03,
			.bInterfaceClass    = 0xff,
			.bInterfaceSubClass = 0x01,
			.bInterfaceProtocol = 0xff,
			.iInterface         = 0x03}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint {
			.bEndpointAddress = 0x81,
			.bmAttributes     = 0x02,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x00}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint {
			.bEndpointAddress = 0x02,
			.bmAttributes     = 0x02,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x00}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint {
			.bEndpointAddress = 0x83,
			.bmAttributes     = 0x03,
			.wMaxPacketSize   = 0x0008,
			.bInterval        = 0x08}));

	add_string("Sony");
	add_string("PSP Type D");
	if (desc && !load_descriptors(*desc))
	{
		psp_log.error("usb_device_psp: the descriptors of the PSP are not valid, using the built-in ones");
	}
	psp_log.error("usb_device_psp::usb_device_psp()");
}

bool usb_device_psp::load_descriptors(const pspcm_descriptors& desc)
{
	const auto& d = desc.device;
	const auto& c = desc.config;
	if (d.size() != 18 || d[0] != 18 || d[1] != USB_DESCRIPTOR_DEVICE || c.size() < 9 || c[0] != 9 || c[1] != USB_DESCRIPTOR_CONFIG)
	{
		return false;
	}

	UsbDescriptorNode new_device(18, USB_DESCRIPTOR_DEVICE, d.data() + 2);
	UsbDescriptorNode new_config(9, USB_DESCRIPTOR_CONFIG, c.data() + 2);

	// The interfaces and endpoints are sub nodes of the configuration, in the order they are listed.
	for (usz pos = 9; pos < c.size();)
	{
		const u8 len = c[pos];
		if (len < 2 || pos + len > c.size())
		{
			return false;
		}
		new_config.add_node(UsbDescriptorNode(len, c[pos + 1], c.data() + pos + 2));
		pos += len;
	}

	new_device.add_node(new_config);
	device = std::move(new_device);
	strings = desc.strings;
	return true;
}

usb_device_psp::~usb_device_psp()
{
	psp_log.error("usb_device_psp::~usb_device_psp()");
}

void usb_device_psp::control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer)
{
	transfer->fake = true;
	transfer->expected_count = buf_size;
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_time = get_timestamp();

	psp_log.error("control_transfer: bmRequestType=%x, bRequest=%x, wLength=%x", bmRequestType, bRequest, wLength);

	switch (bmRequestType)
	{
	case LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE: // 0x41
	case LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE:  // 0xc1
	{
		const bool device_to_host = (bmRequestType & LIBUSB_ENDPOINT_IN) != 0;

		// For a device to host request buf is where the answer goes, there is nothing to send from it.
		add_ctrl(bmRequestType, bRequest, wValue, wIndex, wLength, device_to_host ? nullptr : buf);

		pspcm_data resp{};
		bool answered = false;
		for (u32 retry = 0; retry < PSPCM_CTRL_TIMEOUT_MS; retry++)
		{
			if (m_ctrl_resp.pop(resp))
			{
				answered = true;
				break;
			}
			std::this_thread::sleep_for(1ms);
		}

		if (!answered)
		{
			psp_log.error("   no answer from the PSP, bRequest=%x", bRequest);
			if (device_to_host)
			{
				transfer->expected_count = 0;
			}
			break;
		}

		psp_log.error("   get_response reqType=%x  bRequest=%x, wLength=%x, totalLen=%x", resp.ctrl.bmRequestType, resp.ctrl.bRequest, resp.ctrl.wLength, resp.totalLen);

		if (device_to_host)
		{
			// What the PSP sent, which can be less than the host asked for.
			const u32 sent = resp.totalLen > PSPCM_CTRL_HEADER_LEN ? resp.totalLen - PSPCM_CTRL_HEADER_LEN : 0;
			const u32 len = std::min({ sent, static_cast<u32>(resp.ctrl.wLength), PSPCM_MAX_DATA, buf_size });
			if (buf && len)
			{
				std::memcpy(buf, resp.ctrl.data, len);
			}
			transfer->expected_count = len;
		}
		break;
	}
	default:
		usb_device_emulated::control_transfer(bmRequestType, bRequest, wValue, wIndex, wLength, buf_size, buf, transfer);
		break;
	}
}

bool usb_device_psp::take_in_data(pspcm_queue& queue, u32 buf_size, u8* buf, UsbTransfer* transfer)
{
	pspcm_data pk{};
	if (!queue.pop(pk))
	{
		return false;
	}

	const u32 payload = pk.totalLen > PSPCM_HEADER_LEN ? std::min<u32>(pk.totalLen - PSPCM_HEADER_LEN, PSPCM_MAX_DATA) : 0;
	const u32 len = std::min(payload, buf_size);
	psp_log.error("interrupt_transfer: in : ep=%02x, len=%d", pk.endpoint, len);

	if (buf && len)
	{
		std::memcpy(buf, pk.bulk.data, len);
	}
	transfer->expected_count = len;

	if (len < payload)
	{
		std::memmove(pk.bulk.data, pk.bulk.data + len, payload - len);
		pk.totalLen = static_cast<u8>(PSPCM_HEADER_LEN + payload - len);
		queue.push_front(pk);
	}
	return true;
}

void usb_device_psp::request_in(u32 endpoint, u32 buf_size)
{
	// The guest polls these endpoints all the time. One request goes out, the rest are dropped until
	// the PSP answers, the timeout is for an answer that never comes.
	auto& since = m_in_request_since[in_endpoint_index(endpoint)];
	const s64 now = now_ms();
	const s64 sent = since.load();
	if (sent != 0 && now - sent < PSPCM_IN_REQUEST_TIMEOUT_MS)
	{
		return;
	}
	since = now;

	pspcm_data pk{};
	pk.magic = PSPCM_MAGIC;
	pk.totalLen = static_cast<u8>(PSPCM_HEADER_LEN + 1);
	pk.endpoint = static_cast<u8>(endpoint);
	pk.bulk.data[0] = static_cast<char>(std::min(buf_size, PSPCM_MAX_DATA));
	m_out_data.push(pk);
}

void usb_device_psp::interrupt_transfer(u32 buf_size, u8* buf, u32 endpoint, UsbTransfer* transfer)
{
	transfer->fake = true;
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_count = 0;
	transfer->expected_time = get_timestamp();

	psp_log.error("interrupt_transfer: buf_size=%x, endpoint=%x", buf_size, endpoint);
	switch (endpoint)
	{
	case PSPCM_EP_BULK_IN: // 0x81
		if (!take_in_data(m_bulk_in, buf_size, buf, transfer))
		{
			request_in(endpoint, buf_size);
		}
		break;
	case PSPCM_EP_INTERRUPT_IN: // 0x83
		if (!take_in_data(m_interrupt_in, buf_size, buf, transfer))
		{
			request_in(endpoint, buf_size);
		}
		break;
	case PSPCM_EP_BULK_OUT: // 0x02
		add_data(endpoint, buf_size, buf);
		transfer->expected_count = buf_size;
		break;
	}
}

bool usb_device_psp::get_output(pspcm_data& pk)
{
	return m_out_data.pop(pk);
}

bool usb_device_psp::add_from_psp(const pspcm_data& pk)
{
	switch (pk.endpoint)
	{
	case 0:
		m_ctrl_resp.push(pk);
		return true;
	case PSPCM_EP_BULK_IN:
		m_bulk_in.push(pk);
		m_in_request_since[0] = 0; // answered, the next poll may ask again
		return true;
	case PSPCM_EP_INTERRUPT_IN:
		m_interrupt_in.push(pk);
		m_in_request_since[1] = 0;
		return true;
	}
	return false;
}

void usb_device_psp::add_ctrl(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u8* buf)
{
	// Only a host to device request has data. The packet keeps the real wLength.
	const u32 len = buf ? std::min<u32>(wLength, PSPCM_MAX_DATA) : 0;
	if (buf && wLength > PSPCM_MAX_DATA)
	{
		psp_log.error("add_ctrl: wLength=%x, only the first %x bytes are sent", wLength, PSPCM_MAX_DATA);
	}

	pspcm_data pk{};
	pk.magic = PSPCM_MAGIC;
	pk.totalLen = static_cast<u8>(PSPCM_CTRL_HEADER_LEN + len);
	pk.endpoint = 0;

	pk.ctrl.bmRequestType = bmRequestType;
	pk.ctrl.bRequest = bRequest;
	pk.ctrl.wValue = wValue;
	pk.ctrl.wIndex = wIndex;
	pk.ctrl.wLength = wLength;
	if (len)
	{
		std::memcpy(pk.ctrl.data, buf, len);
	}
	m_out_data.push(pk);
}

void usb_device_psp::add_data(u8 endpoint, u32 buf_size, u8* buf)
{
	// A transfer is sent as packets of wMaxPacketSize. One without data is a packet too.
	u32 pos = 0;
	do
	{
		const u32 len = std::min(buf_size - pos, PSPCM_MAX_DATA);

		pspcm_data pk{};
		pk.magic = PSPCM_MAGIC;
		pk.totalLen = static_cast<u8>(PSPCM_HEADER_LEN + len);
		pk.endpoint = endpoint;
		if (buf && len)
		{
			std::memcpy(pk.bulk.data, buf + pos, len);
		}
		m_out_data.push(pk);

		pos += len;
	}
	while (pos < buf_size);
}
