#pragma once

#include "Emu/Io/usb_device.h"
#include <queue>

#pragma pack(push, 1)
struct pspcm_data {
	uint16_t magic;
	uint8_t totalLen;
	uint8_t endpoint;

	union {
		struct {
			uint8_t bmRequestType;
			uint8_t bRequest;
			uint16_t wValue;
			uint16_t wIndex;
			uint16_t wLength;
			char data[0x40];
		} ctrl;
		struct {
			char data[0x40];
		} bulk;
		struct {
			char data[0x40];
		} interrupt;
	};
};
#pragma pack(pop)

constexpr u8 PSPCM_EP_BULK_IN      = 0x81;
constexpr u8 PSPCM_EP_BULK_OUT     = 0x02;
constexpr u8 PSPCM_EP_INTERRUPT_IN = 0x83;

class usb_device_psp : public usb_device_emulated
{
public:
	usb_device_psp(const std::array<u8, 7>& location);
	~usb_device_psp();

	void control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer) override;
	void interrupt_transfer(u32 buf_size, u8* buf, u32 endpoint, UsbTransfer* transfer) override;

	int get_output_size();
	struct pspcm_data get_output();
	struct pspcm_data get_ctrl_resp();
	void add_ctrl_resp(struct pspcm_data);
	struct pspcm_data get_bulk_in();
	void add_bulk_in(struct pspcm_data);
	struct pspcm_data get_interrupt_in();
	void add_interrupt_in(struct pspcm_data);

private:
	std::queue<struct pspcm_data> out_data; // common for ctrl ep and bulk requests
	std::queue<struct pspcm_data> ctrl_resp;
	std::queue<struct pspcm_data> bulk_in;
	std::queue<struct pspcm_data> interrupt_in;
	void add_ctrl(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u8* buf);
	void add_data(u8 endpoint, u32 buf_size, u8* buf);
};
