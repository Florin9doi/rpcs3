#include "stdafx.h"
#include "bodytracker.h"
#include "MouseHandler.h"
#include "Emu/IdManager.h"
#include "Emu/Io/bodytracker_config.h"
#include "Emu/Cell/lv2/sys_usbd.h"
#include "Emu/system_config.h"
#include "Input/pad_thread.h"

LOG_CHANNEL(bodytracker_log);

template <>
void fmt_class_string<bodytracker_btn>::format(std::string& out, u64 arg)
{
	format_enum(out, arg, [](bodytracker_btn value)
	{
		switch (value)
		{
		case bodytracker_btn::dpad_up: return "D-Pad Up";
		case bodytracker_btn::dpad_down: return "D-Pad Down";
		case bodytracker_btn::dpad_left: return "D-Pad Left";
		case bodytracker_btn::dpad_right: return "D-Pad Right";
		case bodytracker_btn::motion_x: return "Motion X";
		case bodytracker_btn::motion_y: return "Motion Y";
		case bodytracker_btn::motion_z: return "Motion Z";
		case bodytracker_btn::motion_g: return "Motion G";
		case bodytracker_btn::count: return "Count";
		}

		return unknown;
	});
}

#pragma pack(push, 1)
struct bodytracker_data
{
	union
	{
		u8 buf[31];
		struct
		{
			u8 part1[11];
			u8 part2[11];
			u8 part3[9];
		};
	};
};
#pragma pack(pop)

usb_device_bodytracker::usb_device_bodytracker(u32 controller_index, const std::array<u8, 7>& location)
	: usb_device_emulated(location)
	, m_controller_index(controller_index)
{
	device = UsbDescriptorNode(USB_DESCRIPTOR_DEVICE,
		UsbDeviceDescriptor {
			.bcdUSB             = 0x0110,
			.bDeviceClass       = 0x00,
			.bDeviceSubClass    = 0x00,
			.bDeviceProtocol    = 0x00,
			.bMaxPacketSize0    = 0x08,
			.idVendor           = 0x21a4,
			.idProduct          = 0xac27,
			.bcdDevice          = 0x0300,
			.iManufacturer      = 0x01,
			.iProduct           = 0x02,
			.iSerialNumber      = 0x03,
			.bNumConfigurations = 0x01});
	auto& config0 = device.add_node(UsbDescriptorNode(USB_DESCRIPTOR_CONFIG,
		UsbDeviceConfiguration {
			.wTotalLength        = 0x0022,
			.bNumInterfaces      = 0x01,
			.bConfigurationValue = 0x01,
			.iConfiguration      = 0x00,
			.bmAttributes        = 0x80,
			.bMaxPower           = 0x32}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_INTERFACE,
		UsbDeviceInterface {
			.bInterfaceNumber   = 0x00,
			.bAlternateSetting  = 0x00,
			.bNumEndpoints      = 0x01,
			.bInterfaceClass    = 0x03,
			.bInterfaceSubClass = 0x00,
			.bInterfaceProtocol = 0x00,
			.iInterface         = 0x00}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_HID,
		UsbDeviceHID {
			.bcdHID            = 0x0110,
			.bCountryCode      = 0x00,
			.bNumDescriptors   = 0x01,
			.bDescriptorType   = 0x22,
			.wDescriptorLength = 0x0064}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint {
			.bEndpointAddress = 0x81,
			.bmAttributes     = 0x03,
			.wMaxPacketSize   = 0x0010,
			.bInterval        = 0x10}));

	add_string("GuitarHero for Playstation (R) 3");
	add_string("GuitarHero for Playstation (R) 3");
}

usb_device_bodytracker::~usb_device_bodytracker()
{
}

std::shared_ptr<usb_device> usb_device_bodytracker::make_instance(u32 controller_index, const std::array<u8, 7>& location)
{
	return std::make_shared<usb_device_bodytracker>(controller_index, location);
}

u16 usb_device_bodytracker::get_num_emu_devices()
{
	return 1;
}

void usb_device_bodytracker::control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer)
{
	transfer->fake            = true;
	transfer->expected_count  = buf_size;
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_time   = get_timestamp() + 100;

	switch (bmRequestType)
	{
	case 0U /*silences warning*/ | LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE: // 0x21
		switch (bRequest)
		{
		case 0x09: // SET_REPORT
			ensure(buf_size >= 2);
			switch (buf[0])
			{
			case 0x7c:
				bodytracker_log.error("SET_REPORT : %s", ::fmt::buf_to_hexstring(buf, buf_size));
				break;
			case 0x01:
				bodytracker_log.trace("Leds: %s/%s/%s/%s",
					buf[1] & 1 ? "ON" : "OFF",
					buf[1] & 2 ? "ON" : "OFF",
					buf[1] & 4 ? "ON" : "OFF",
					buf[1] & 8 ? "ON" : "OFF");
				break;
			default:
				bodytracker_log.error("Unhandled SET_REPORT packet : %x", buf[0]);
				break;
			}
			break;
		default:
			bodytracker_log.error("Unhandled Request: 0x%02X/0x%02X", bmRequestType, bRequest);
			break;
		}
		break;
	default:
		bodytracker_log.error("control_transfer Request: 0x%02X/0x%02X", bmRequestType, bRequest);
		usb_device_emulated::control_transfer(bmRequestType, bRequest, wValue, wIndex, wLength, buf_size, buf, transfer);
		break;
	}
}

extern bool is_input_allowed();

static void prepare_data(const u8* tbt, u8* data, u8 size)
{
	std::memcpy(data, tbt, size);
	bodytracker_log.success("interrupt_transfer: %s", fmt::buf_to_hexstring(data, size));
}

void usb_device_bodytracker::interrupt_transfer(u32 buf_size, u8* buf, u32 /*endpoint*/, UsbTransfer* transfer)
{
	ensure(buf_size >= sizeof(bodytracker_data));

	transfer->fake = true;
	transfer->expected_count = buf_size;
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_time = get_timestamp() + 4000;

	static // TODO: check this
	bodytracker_data tbt{};

	if (!is_input_allowed())
	{
		//prepare_data(&tbt, buf);
		return;
	}

	if (m_controller_index >= g_cfg_bodytracker.players.size())
	{
		bodytracker_log.warning("Total Body Tracker System controllers are only supported for Player1 to Player%d", g_cfg_bodytracker.players.size());
		//prepare_data(&tbt, buf);
		return;
	}

	bool up = false, right = false, down = false, left = false;
	const auto input_callback = [&up, &down, &left, &right](const emulated_pad_config<bodytracker_btn>::input_value& value, bool& /*abort*/)
	{
		if (!value.pressed)
			return;

		switch (value.btn)
		{
		case bodytracker_btn::dpad_up: up = true; break;
		case bodytracker_btn::dpad_down: down = true; break;
		case bodytracker_btn::dpad_left: left = true; break;
		case bodytracker_btn::dpad_right: right = true; break;
		case bodytracker_btn::count: break;
		}
	};

	const auto& cfg = ::at32(g_cfg_bodytracker.players, m_controller_index);

	{
		std::lock_guard lock(pad::g_pad_mutex);
		const auto gamepad_handler = pad::get_pad_thread();
		const auto& pads = gamepad_handler->GetPads();
		const auto& pad = ::at32(pads, m_controller_index);
		if (pad->m_port_status & CELL_PAD_STATUS_CONNECTED)
		{
			cfg->handle_input(pad, true, input_callback);
		}
	}

	bool up2 = up && !m_up;
	bool down2 = down && !m_down;
	bool left2 = left && !m_left;
	bool right2 = right && !m_right;
	
	if (left2)  m_dyn_index = m_dyn_index == 0 ? 30 : m_dyn_index - 1;
	if (right2) m_dyn_index = m_dyn_index == 30 ? 0 : m_dyn_index + 1;
	if (up2)   tbt.buf[m_dyn_index] ++;
	if (down2) tbt.buf[m_dyn_index] --;

	char hexdump[120] = {};
	for (int i = 0; i < sizeof(bodytracker_data); i++)
	{
		sprintf(&hexdump[3 * i], " %02x ", tbt.buf[i]);
	}
	hexdump[3 * m_dyn_index + 0] = '[';
	hexdump[3 * m_dyn_index + 3] = ']';
	bodytracker_log.todo("TBT: pkidx=%d, dpad=%d%d%d%d dynidx=%2d buf=[%s]", /*m_pk_part*/0, up2, down2, left2, right2, m_dyn_index, hexdump);

	m_up = up;
	m_down = down;
	m_left = left;
	m_right = right;

	switch (m_pk_part)
	{
		case 0:
			prepare_data(tbt.part1, buf, 11);
			transfer->expected_count = 11;
			m_pk_part++;
			break;
		case 1:
			prepare_data(tbt.part2, buf, 11);
			transfer->expected_count = 11;
			m_pk_part++;
			break;
		case 2:
			prepare_data(tbt.part3, buf, 9);
			transfer->expected_count = 9;
			m_pk_part = 0;
			break;
	}
}
