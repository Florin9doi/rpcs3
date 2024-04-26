#include "stdafx.h"
#include "TopShotElite.h"
#include "MouseHandler.h"
#include "Emu/IdManager.h"
#include "Emu/Cell/lv2/sys_usbd.h"
#include "Emu/system_config.h"
#include "Input/pad_thread.h"

LOG_CHANNEL(topshotelite_log);

#pragma pack(push, 1)
struct TopShotElite_data
{
	uint8_t btn_square : 1;
	uint8_t btn_cross : 1;
	uint8_t btn_circle : 1;
	uint8_t btn_triangle : 1;
	uint8_t btn_reload: 1;
	uint8_t btn_trigger: 1;
	uint8_t : 2;

	uint8_t btn_select : 1;
	uint8_t btn_start : 1;
	uint8_t btn_l3 : 1;
	uint8_t btn_r3 : 1;
	uint8_t btn_ps: 1;
	uint8_t : 3;

	uint8_t dpad;
	uint8_t stick_lx;
	uint8_t stick_ly;
	uint8_t stick_rx;
	uint8_t stick_ry;

	uint8_t led_lx_hi : 8;
	uint8_t led_ly_hi : 6;
	uint8_t led_lx_lo : 2;
	uint8_t unk1 : 4;
	uint8_t led_ly_lo : 4;

	uint8_t led_rx_hi : 8;
	uint8_t led_ry_hi : 6;
	uint8_t led_rx_lo : 2;
	uint8_t unk2 : 4;
	uint8_t led_ry_lo : 4;

	uint8_t : 8;
	uint8_t : 8;
	uint8_t : 8;

	uint8_t trigger;
	uint16_t unk3[5];
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

usb_device_topshotelite::usb_device_topshotelite(u32 controller_index, const std::array<u8, 7>& location)
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
			.idProduct          = 0x04a0,
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

usb_device_topshotelite::~usb_device_topshotelite()
{
}

void usb_device_topshotelite::control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer)
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
				// TODO: trace
				topshotelite_log.trace("Leds: %s/%s/%s/%s",
					buf[2] & 1 ? "ON" : "OFF",
					buf[2] & 2 ? "ON" : "OFF",
					buf[2] & 4 ? "ON" : "OFF",
					buf[2] & 8 ? "ON" : "OFF");
				break;
			case 0x82:
				m_mode = buf[2];
				break;
			default:
				topshotelite_log.error("Unknown SET_REPORT packet : %x", buf[0]);
				break;
			}
			break;
		default:
			topshotelite_log.error("Unhandled Request: 0x%02X/0x%02X", bmRequestType, bRequest);
			break;
		}
		break;
	default:
		usb_device_emulated::control_transfer(bmRequestType, bRequest, wValue, wIndex, wLength, buf_size, buf, transfer);
		break;
	}
}

extern bool is_input_allowed();

static void set_sensor_pos(struct TopShotElite_data* ts, s32 led_lx, s32 led_ly, s32 led_rx, s32 led_ry, s32 unk)
{
	ts->led_lx_hi = led_lx >> 2;
	ts->led_lx_lo = led_lx & 0x3;
	ts->led_ly_hi = led_ly >> 4;
	ts->led_ly_lo = led_ly & 0xf;

	ts->led_rx_hi = led_rx >> 2;
	ts->led_rx_lo = led_rx & 0x3;
	ts->led_ry_hi = led_ry >> 4;
	ts->led_ry_lo = led_ry & 0xf;

	ts->unk1 = ts->unk2 = unk;
}

void usb_device_topshotelite::interrupt_transfer(u32 buf_size, u8* buf, u32 endpoint, UsbTransfer* transfer)
{
	ensure(buf_size >= sizeof(TopShotElite_data));

	transfer->fake = true;
	transfer->expected_count = sizeof(TopShotElite_data);
	transfer->expected_result = HC_CC_NOERR;
	transfer->expected_time = get_timestamp() + 500;

	struct TopShotElite_data ts{};
	ts.dpad = Dpad_None;
	ts.stick_lx = ts.stick_ly = ts.stick_rx = ts.stick_ry = 0x7f;
	if (m_mode)
	{
		set_sensor_pos(&ts, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0xf);
	}
	ts.unk3[1] = ts.unk3[2] = ts.unk3[3] = ts.unk3[4] = 0x0200;

	if (!is_input_allowed())
	{
		std::memcpy(buf, &ts, sizeof(TopShotElite_data));
		return;
	}

	if (g_cfg.io.mouse == mouse_handler::null)
	{
		topshotelite_log.warning("Top Shot Elite requires a Mouse Handler enabled");
		std::memcpy(buf, &ts, sizeof(TopShotElite_data));
		return;
	}

	bool up = false, right = false, down = false, left = false;

	{
		std::lock_guard lock(pad::g_pad_mutex);
		const auto gamepad_handler = pad::get_current_handler();
		const auto& pads = gamepad_handler->GetPads();

		const auto& pad = ::at32(pads, m_controller_index);
		if (pad->m_port_status & CELL_PAD_STATUS_CONNECTED)
		{
			for (const Button& button : pad->m_buttons)
			{
				if (!button.m_pressed)
				{
					continue;
				}

				if (button.m_offset == CELL_PAD_BTN_OFFSET_DIGITAL1)
				{
					switch (button.m_outKeyCode)
					{
					case CELL_PAD_CTRL_UP:
						up = true;
						break;
					case CELL_PAD_CTRL_RIGHT:
						right = true;
						break;
					case CELL_PAD_CTRL_DOWN:
						down = true;
						break;
					case CELL_PAD_CTRL_LEFT:
						left = true;
						break;
					case CELL_PAD_CTRL_SELECT:
						ts.btn_select |= 1;
						break;
					case CELL_PAD_CTRL_START:
						ts.btn_start |= 1;
						break;
					case CELL_PAD_CTRL_L3:
						ts.btn_l3 |= 1;
						break;
					case CELL_PAD_CTRL_R3:
						ts.btn_r3 |= 1;
						break;
					case CELL_PAD_CTRL_PS:
						ts.btn_ps |= 1;
						break;
					default:
						break;
					}
				}
				else if (button.m_offset == CELL_PAD_BTN_OFFSET_DIGITAL2)
				{
					switch (button.m_outKeyCode)
					{
					case CELL_PAD_CTRL_CROSS:
						ts.btn_cross |= 1;
						break;
					case CELL_PAD_CTRL_CIRCLE:
						ts.btn_circle |= 1;
						break;
					case CELL_PAD_CTRL_SQUARE:
						ts.btn_square |= 1;
						break;
					case CELL_PAD_CTRL_TRIANGLE:
						ts.btn_triangle |= 1;
						break;
					case CELL_PAD_CTRL_L1:
						ts.btn_reload |= 1;
						break;
					case CELL_PAD_CTRL_R1:
						ts.btn_trigger |= 1;
						ts.trigger = 0xff;
						break;
					default:
						break;
					}
				}
			}

			for (const AnalogStick& stick : pad->m_sticks)
			{
				switch (stick.m_offset)
				{
				case CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X:
					ts.stick_lx = static_cast<uint8_t>(stick.m_value);
					break;
				case CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y:
					ts.stick_ly = static_cast<uint8_t>(stick.m_value);
					break;
				case CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X:
					ts.stick_rx = static_cast<uint8_t>(stick.m_value);
					break;
				case CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y:
					ts.stick_ry = static_cast<uint8_t>(stick.m_value);
					break;
				default:
					break;
				}
			}
		}
	}

	if (!up && !right && !down && !left)
		ts.dpad = Dpad_None;
	else if (up && !left && !right)
		ts.dpad = Dpad_North;
	else if (up && right)
		ts.dpad = Dpad_NE;
	else if (right && !up && !down)
		ts.dpad = Dpad_East;
	else if (down && right)
		ts.dpad = Dpad_SE;
	else if (down && !left && !right)
		ts.dpad = Dpad_South;
	else if (down && left)
		ts.dpad = Dpad_SW;
	else if (left && !up && !down)
		ts.dpad = Dpad_West;
	else if (up && left)
		ts.dpad = Dpad_NW;

	if (m_mode)
	{
		auto& mouse_handler = g_fxo->get<MouseHandlerBase>();
		std::lock_guard mouse_lock(mouse_handler.mutex);

		mouse_handler.Init(2);

		if (m_controller_index >= mouse_handler.GetMice().size())
		{
			std::memcpy(buf, &ts, sizeof(TopShotElite_data));
			return;
		}

		const Mouse& mouse_data = ::at32(mouse_handler.GetMice(), m_controller_index);
		if (mouse_data.x_max <= 0 || mouse_data.y_max <= 0)
		{
			std::memcpy(buf, &ts, sizeof(TopShotElite_data));
			return;
		}

		// calibration
		// Expand 0..+w to (a part of) 0x3ff..0
		constexpr s32 diff = 95;
		constexpr s32 right = 200;
		constexpr s32 left = 0x3ff - right;

		// sensor on top - only half of range (0..0x2ff) is usable
		constexpr s32 top = 416;
		constexpr s32 bottom = 767;

		s32 led_lx = 0x3ff - (right + (mouse_data.x_pos * (left - right) / mouse_data.x_max) + diff);
		s32 led_rx = 0x3ff - (right + (mouse_data.x_pos * (left - right) / mouse_data.x_max) - diff);

		s32 led_ly = top + (mouse_data.y_pos * (bottom - top) / mouse_data.y_max);
		s32 led_ry = top + (mouse_data.y_pos * (bottom - top) / mouse_data.y_max);

		if (led_lx < 0 || led_lx > 0x3ff /*|| led_ly < 0 || led_ly > 0x3ff*/)
		{
			led_lx = 0x3ff;
			led_ly = 0x3ff;
		}

		if (led_rx < 0 || led_rx > 0x3ff /*|| led_ry < 0 || led_ry > 0x3ff*/)
		{
			led_rx = 0x3ff;
			led_ry = 0x3ff;
		}

		set_sensor_pos(&ts, led_lx, led_ly, led_rx, led_ry, 0x2);

		ts.btn_trigger |= !!(mouse_data.buttons & CELL_MOUSE_BUTTON_1);
		ts.trigger = ts.btn_trigger ? 0xff : 0x00;
	}

	std::memcpy(buf, &ts, sizeof(TopShotElite_data));

	/*
	topshotelite_log.error("%02x %02x %02x %02x %02x %02x - %02x %02x - %02x %02x",
	    buf[7], buf[8], buf[9], buf[10], buf[11], buf[12],
	    led_lx, led_ly, led_rx, led_ry
	);
	/*
	topshotelite_log.error("%02x %02x %02x %02x %02x %02x %02x %02x - %02x %02x %02x %02x %02x %02x %02x %02x - %02x %02x %02x %02x %02x %02x %02x %02x - %02x %02x %02x",
	    buf[ 0], buf[ 1], buf[ 2], buf[ 3], buf[ 4], buf[ 5], buf[ 6], buf[ 7],
	    buf[ 8], buf[ 9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],
	    buf[16], buf[17], buf[18], buf[19], buf[20], buf[21], buf[22], buf[23],
	    buf[24], buf[25], buf[26]);
	//*/
}
