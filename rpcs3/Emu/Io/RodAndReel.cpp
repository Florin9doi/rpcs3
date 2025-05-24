#include "stdafx.h"
#include "RodAndReel.h"
#include "MouseHandler.h"
#include "Emu/IdManager.h"
#include "Emu/Io/rodandreel_config.h"
#include "Emu/Cell/lv2/sys_usbd.h"
#include "Emu/system_config.h"
#include "Input/pad_thread.h"

LOG_CHANNEL(rodandreel_log);

template <>
void fmt_class_string<rodandreel_btn>::format(std::string& out, u64 arg)
{
	format_enum(out, arg, [](rodandreel_btn value)
	{
		switch (value)
		{
		case rodandreel_btn::square: return "Square";
		case rodandreel_btn::cross: return "Cross";
		case rodandreel_btn::circle: return "Circle";
		case rodandreel_btn::triangle: return "Triangle";
		case rodandreel_btn::reel_in: return "Reel-In";
		case rodandreel_btn::trigger: return "Trigger";
		case rodandreel_btn::l2: return "L2";
		case rodandreel_btn::r2: return "R2";
		case rodandreel_btn::select: return "Select";
		case rodandreel_btn::start: return "Start";
		case rodandreel_btn::l3: return "L3";
		case rodandreel_btn::ps: return "PS";
		case rodandreel_btn::dpad_up: return "D-Pad Up";
		case rodandreel_btn::dpad_down: return "D-Pad Down";
		case rodandreel_btn::dpad_left: return "D-Pad Left";
		case rodandreel_btn::dpad_right: return "D-Pad Right";
		case rodandreel_btn::stick_x: return "Stick X-Axis";
		case rodandreel_btn::stick_y: return "Stick Y-Axis";
		case rodandreel_btn::motion_x: return "Motion X";
		case rodandreel_btn::motion_y: return "Motion Y";
		case rodandreel_btn::motion_z: return "Motion Z";
		case rodandreel_btn::motion_g: return "Motion G";
		case rodandreel_btn::count: return "Count";
		}

		return unknown;
	});
}

#pragma pack(push, 1)
struct RodAndReel_data
{
	uint8_t btn_square : 1;
	uint8_t btn_cross : 1;
	uint8_t btn_circle : 1;
	uint8_t btn_triangle : 1;
	uint8_t : 1;
	uint8_t : 1;
	uint8_t btn_l2 : 1;
	uint8_t btn_r2 : 1;

	uint8_t btn_select : 1;
	uint8_t btn_start : 1;
	uint8_t btn_l3 : 1;
	uint8_t : 1;
	uint8_t btn_ps: 1;
	uint8_t : 3;

	uint8_t dpad;
	uint8_t stick_x;
	uint8_t stick_y;
	uint8_t : 8;
	uint8_t : 8;
	
	uint8_t : 8;
	uint8_t : 8;
	uint8_t : 8;
	uint8_t : 8;
	
	uint8_t : 8;
	uint8_t : 8;
	uint8_t : 8;
	uint8_t : 8;

	uint8_t reel_in;
	uint8_t trigger;
	uint8_t : 8;
	uint8_t : 8;

	uint16_t motion_x;
	uint16_t motion_y;
	uint16_t motion_z;
	uint16_t motion_g;
};
#pragma pack(pop)

enum
{
	Dpad_North,
	Dpad_NE,
	Dpad_East,
	Dpad_SE,
	Dpad_South,
	Dpad_SW,
	Dpad_West,
	Dpad_NW,
	Dpad_None = 0x0f
};

usb_device_rodandreel::usb_device_rodandreel(u32 controller_index, const std::array<u8, 7>& location)
	: usb_device_emulated(location)
	, m_controller_index(controller_index)
{
	device = UsbDescriptorNode(USB_DESCRIPTOR_DEVICE,
		UsbDeviceDescriptor {
			.bcdUSB             = 0x0100,
			.bDeviceClass       = 0x00,
			.bDeviceSubClass    = 0x00,
			.bDeviceProtocol    = 0x00,
			.bMaxPacketSize0    = 0x20,
			.idVendor           = 0x12ba,
			.idProduct          = 0x04b0,
			.bcdDevice          = 0x0108,
			.iManufacturer      = 0x01,
			.iProduct           = 0x02,
			.iSerialNumber      = 0x03,
			.bNumConfigurations = 0x01});
	auto& config0 = device.add_node(UsbDescriptorNode(USB_DESCRIPTOR_CONFIG,
		UsbDeviceConfiguration {
			.wTotalLength        = 0x0029,
			.bNumInterfaces      = 0x01,
			.bConfigurationValue = 0x01,
			.iConfiguration      = 0x00,
			.bmAttributes        = 0x80,
			.bMaxPower           = 0x32}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_INTERFACE,
		UsbDeviceInterface {
			.bInterfaceNumber   = 0x00,
			.bAlternateSetting  = 0x00,
			.bNumEndpoints      = 0x02,
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
			.wDescriptorLength = 0x0089}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint {
			.bEndpointAddress = 0x81,
			.bmAttributes     = 0x03,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x0a}));
	config0.add_node(UsbDescriptorNode(USB_DESCRIPTOR_ENDPOINT,
		UsbDeviceEndpoint {
			.bEndpointAddress = 0x02,
			.bmAttributes     = 0x03,
			.wMaxPacketSize   = 0x0040,
			.bInterval        = 0x0a}));

	add_string("GuitarHero for Playstation (R) 3");
	add_string("GuitarHero for Playstation (R) 3");
}

usb_device_rodandreel::~usb_device_rodandreel()
{
}

std::shared_ptr<usb_device> usb_device_rodandreel::make_instance(u32 controller_index, const std::array<u8, 7>& location)
{
	return std::make_shared<usb_device_rodandreel>(controller_index, location);
}

u16 usb_device_rodandreel::get_num_emu_devices()
{
	return 1;
}

void usb_device_rodandreel::control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer)
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
			ensure(buf_size >= 8);
			switch (buf[0])
			{
			case 0x01:
				rodandreel_log.trace("Leds: %s/%s/%s/%s",
					buf[2] & 1 ? "ON" : "OFF",
					buf[2] & 2 ? "ON" : "OFF",
					buf[2] & 4 ? "ON" : "OFF",
					buf[2] & 8 ? "ON" : "OFF");
				break;
			default:
				rodandreel_log.error("Unhandled SET_REPORT packet : %x", buf[0]);
				break;
			}
			break;
		default:
			rodandreel_log.error("Unhandled Request: 0x%02X/0x%02X", bmRequestType, bRequest);
			break;
		}
		break;
	default:
		rodandreel_log.error("control_transfer Request: 0x%02X/0x%02X", bmRequestType, bRequest);
		usb_device_emulated::control_transfer(bmRequestType, bRequest, wValue, wIndex, wLength, buf_size, buf, transfer);
		break;
	}
}

extern bool is_input_allowed();

static void prepare_data(const RodAndReel_data* rnr, u8* data)
{
	std::memcpy(data, rnr, sizeof(RodAndReel_data));
	rodandreel_log.error("interrupt_transfer: %s", fmt::buf_to_hexstring(data, sizeof(RodAndReel_data), 32));
}

void usb_device_rodandreel::interrupt_transfer(u32 buf_size, u8* buf, u32 /*endpoint*/, UsbTransfer* transfer)
{
	ensure(buf_size >= sizeof(RodAndReel_data));

	transfer->fake = true;
	transfer->expected_count = sizeof(RodAndReel_data);
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_time = get_timestamp() + 4000;

	struct RodAndReel_data rnr{};
	rnr.dpad = Dpad_None;
	rnr.stick_x = rnr.stick_y = 0x7f;
	rnr.motion_x = rnr.motion_y = rnr.motion_z = rnr.motion_g = 0x0200;

	if (!is_input_allowed())
	{
		prepare_data(&rnr, buf);
		return;
	}

	if (m_controller_index >= g_cfg_rodandreel.players.size())
	{
		rodandreel_log.warning("Rod and Reel controllers are only supported for Player1 to Player%d", g_cfg_rodandreel.players.size());
		prepare_data(&rnr, buf);
		return;
	}

	bool up = false, right = false, down = false, left = false;
	const auto input_callback = [&rnr, &up, &down, &left, &right](const emulated_pad_config<rodandreel_btn>::input_value& value, bool& /*abort*/)
	{
		if (!value.pressed)
			return;

		switch (value.btn)
		{
		case rodandreel_btn::square: rnr.btn_square |= 1; break;
		case rodandreel_btn::cross: rnr.btn_cross |= 1; break;
		case rodandreel_btn::circle: rnr.btn_circle |= 1; break;
		case rodandreel_btn::triangle: rnr.btn_triangle |= 1; break;
		case rodandreel_btn::reel_in: rnr.reel_in = static_cast<uint8_t>(value.value); break;
		case rodandreel_btn::trigger: rnr.trigger = static_cast<uint8_t>(value.value); break;
		case rodandreel_btn::l2: rnr.btn_l2 |= 1; break;
		case rodandreel_btn::r2: rnr.btn_r2 |= 1; break;
		case rodandreel_btn::select: rnr.btn_select |= 1; break;
		case rodandreel_btn::start: rnr.btn_start |= 1; break;
		case rodandreel_btn::l3: rnr.btn_l3 |= 1; break;
		case rodandreel_btn::ps: rnr.btn_ps |= 1; break;
		case rodandreel_btn::dpad_up: up = true; break;
		case rodandreel_btn::dpad_down: down = true; break;
		case rodandreel_btn::dpad_left: left = true; break;
		case rodandreel_btn::dpad_right: right = true; break;
		case rodandreel_btn::stick_x: rnr.stick_x = static_cast<uint8_t>(value.value); break;
		case rodandreel_btn::stick_y: rnr.stick_y = static_cast<uint8_t>(value.value); break;
		case rodandreel_btn::motion_x: rnr.motion_x = static_cast<uint16_t>(value.value); break;
		case rodandreel_btn::motion_y: rnr.motion_y = static_cast<uint16_t>(value.value); break;
		case rodandreel_btn::motion_z: rnr.motion_z = static_cast<uint16_t>(value.value); break;
		case rodandreel_btn::motion_g: rnr.motion_g = static_cast<uint16_t>(value.value); break;
		case rodandreel_btn::count: break;
		}
	};

	const auto& cfg = ::at32(g_cfg_rodandreel.players, m_controller_index);

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

	if (!up && !right && !down && !left)
		rnr.dpad = Dpad_None;
	else if (up && !left && !right)
		rnr.dpad = Dpad_North;
	else if (up && right)
		rnr.dpad = Dpad_NE;
	else if (right && !up && !down)
		rnr.dpad = Dpad_East;
	else if (down && right)
		rnr.dpad = Dpad_SE;
	else if (down && !left && !right)
		rnr.dpad = Dpad_South;
	else if (down && left)
		rnr.dpad = Dpad_SW;
	else if (left && !up && !down)
		rnr.dpad = Dpad_West;
	else if (up && left)
		rnr.dpad = Dpad_NW;

	prepare_data(&rnr, buf);
}
