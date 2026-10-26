#pragma once

#include "Emu/Io/usb_device.h"
#include <deque>
#include <mutex>

// Packets exchanged with PPSSPP (Core/HLE/sceUsb.cpp) over TCP. The stream has no packet boundaries,
// totalLen is the size of the whole packet and is how it is split again.
//  - Control (endpoint 0): a 12 bytes header, then the data of a host to device request. A reply to a
//    device to host request carries the data it has, and says how much in wLength.
//  - Other endpoints: 4 bytes of header, then the data. The endpoint is the USB address (0x81, 0x02, 0x83).
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

constexpr u16 PSPCM_MAGIC            = 0x0ff0;
constexpr u32 PSPCM_HEADER_LEN       = 4;
constexpr u32 PSPCM_CTRL_HEADER_LEN  = 12;
constexpr u32 PSPCM_MAX_DATA         = 0x40; // wMaxPacketSize
constexpr u32 PSPCM_CTRL_TIMEOUT_MS  = 2000;

constexpr u8 PSPCM_EP_BULK_IN      = 0x81;
constexpr u8 PSPCM_EP_BULK_OUT     = 0x02;
constexpr u8 PSPCM_EP_INTERRUPT_IN = 0x83;

// Packets passed between the thread talking to PPSSPP and the threads running the transfers.
class pspcm_queue
{
public:
	void push(const pspcm_data& pk)
	{
		std::lock_guard lock(m_mutex);
		m_queue.push_back(pk);
	}

	// Puts back what a transfer was too small to take.
	void push_front(const pspcm_data& pk)
	{
		std::lock_guard lock(m_mutex);
		m_queue.push_front(pk);
	}

	bool pop(pspcm_data& pk)
	{
		std::lock_guard lock(m_mutex);
		if (m_queue.empty())
		{
			return false;
		}
		pk = m_queue.front();
		m_queue.pop_front();
		return true;
	}

	void clear()
	{
		std::lock_guard lock(m_mutex);
		m_queue.clear();
	}

private:
	std::mutex m_mutex;
	std::deque<pspcm_data> m_queue;
};

class usb_device_psp : public usb_device_emulated
{
public:
	usb_device_psp(const std::array<u8, 7>& location);
	~usb_device_psp();

	void control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer) override;
	void interrupt_transfer(u32 buf_size, u8* buf, u32 endpoint, UsbTransfer* transfer) override;

	// Next packet for the PSP. False if there is none.
	bool get_output(pspcm_data& pk);
	// A packet received from the PSP. False if its endpoint isn't one of the device's.
	bool add_from_psp(const pspcm_data& pk);

private:
	pspcm_queue m_out_data; // common for ctrl ep and bulk requests
	pspcm_queue m_ctrl_resp;
	pspcm_queue m_bulk_in;
	pspcm_queue m_interrupt_in;
	void add_ctrl(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u8* buf);
	void add_data(u8 endpoint, u32 buf_size, u8* buf);
	// Takes a packet of an IN endpoint into buf, the rest of it stays for the next transfer.
	void take_in_data(pspcm_queue& queue, u32 buf_size, u8* buf, UsbTransfer* transfer);
};
