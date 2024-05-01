#include "stdafx.h"
#include "PSP.h"
#include "Emu/IdManager.h"
#include "Emu/Cell/lv2/sys_usbd.h"
#include "Emu/system_config.h"
#include "Input/pad_thread.h"
#include <Utilities/Thread.h>

LOG_CHANNEL(psp_log);

usb_device_psp::usb_device_psp(const std::array<u8, 7>& location)
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
	psp_log.error("usb_device_psp::usb_device_psp()");
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
		// patch ppsspp crash
		//if (bmRequestType == 0x41 && bRequest == 0x02)
		//{
		//	psp_log.error("patch len 0x1000 > 0x0050");
		//	buf[8] = 0x50;
		//	buf[9] = 0x00;
		//}

		add_ctrl(bmRequestType, bRequest, wValue, wIndex, wLength, buf);

		int retry = 500;
		while (retry --> 0)
		{
			if (ctrl_resp.size() > 0)
			{
				struct pspcm_data d = get_ctrl_resp();
				psp_log.error("   get_response reqType=%x  bRequest=%x, wLength=%x", d.ctrl.bmRequestType, d.ctrl.bRequest, d.ctrl.wLength);
				if (wLength && buf && d.totalLen >= 12 && d.ctrl.wLength)
				{
					memcpy(buf, d.ctrl.data, d.ctrl.wLength);
					transfer->expected_count = d.ctrl.wLength;
				}
				break;
			}
			std::this_thread::sleep_for(1ms);
		}
		break;
	}
	default:
		usb_device_emulated::control_transfer(bmRequestType, bRequest, wValue, wIndex, wLength, buf_size, buf, transfer);
		break;
	}
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
		//add_data(endpoint, buf_size, buf);
		if (bulk_in.size())
		{
			struct pspcm_data d = get_bulk_in();
			psp_log.error("interrupt_transfer: bulk in : ep=%02x, len=%d", d.endpoint, d.totalLen);
			ensure(endpoint == d.endpoint);
			memcpy(buf, d.bulk.data, d.totalLen - 4);
			transfer->expected_count = buf_size;
		}
		break;
	case PSPCM_EP_INTERRUPT_IN: // 0x83
		//add_data(endpoint, buf_size, buf);
		if (interrupt_in.size())
		{
			struct pspcm_data d = get_interrupt_in();
			psp_log.error("interrupt_transfer: interrupt in : ep=%02x, len=%d", d.endpoint, d.totalLen);
			ensure(endpoint == d.endpoint);
			memcpy(buf, d.interrupt.data, d.totalLen - 4);
			transfer->expected_count = buf_size;
		}
		break;
	case PSPCM_EP_BULK_OUT: // 0x02
		add_data(endpoint, buf_size, buf);
		transfer->expected_count = buf_size;
		break;
	}
}

int usb_device_psp::get_output_size()
{
	return out_data.size();
}

struct pspcm_data usb_device_psp::get_output()
{
	struct pspcm_data request = out_data.front();
	out_data.pop();
	return request;
}

struct pspcm_data usb_device_psp::get_ctrl_resp()
{
	struct pspcm_data response = ctrl_resp.front();
	ctrl_resp.pop();
	return response;
}

void usb_device_psp::add_ctrl_resp(struct pspcm_data data)
{
	ctrl_resp.push(data);
}

struct pspcm_data usb_device_psp::get_bulk_in()
{
	struct pspcm_data response = bulk_in.front();
	bulk_in.pop();
	return response;
}

void usb_device_psp::add_bulk_in(struct pspcm_data data)
{
	bulk_in.push(data);
}

struct pspcm_data usb_device_psp::get_interrupt_in()
{
	struct pspcm_data response = interrupt_in.front();
	interrupt_in.pop();
	return response;
}

void usb_device_psp::add_interrupt_in(struct pspcm_data data)
{
	interrupt_in.push(data);
}

void usb_device_psp::add_ctrl(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u8* buf)
{
	struct pspcm_data pk{};
	pk.magic = 0x0ff0;
	pk.totalLen = 12 + wLength;
	pk.endpoint = 0;

	pk.ctrl.bmRequestType = bmRequestType;
	pk.ctrl.bRequest = bRequest;
	pk.ctrl.wValue = wValue;
	pk.ctrl.wIndex = wIndex;
	pk.ctrl.wLength = wLength;
	if (buf)
		std::memcpy(pk.ctrl.data, buf, wLength);
	out_data.emplace(pk);
}

void usb_device_psp::add_data(u8 endpoint, u32 buf_size, u8* buf)
{
	struct pspcm_data pk{};
	pk.magic = 0x0ff0;
	pk.totalLen = 4 + buf_size;
	pk.endpoint = endpoint;
	if (buf)
		std::memcpy(pk.bulk.data, buf, buf_size);
	out_data.emplace(pk);
}
