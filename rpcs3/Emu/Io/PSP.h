#pragma once

#include "Emu/Io/usb_device.h"
#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>

// Packets exchanged with PPSSPP (Core/HLE/sceUsb.cpp) over TCP. The stream has no packet boundaries,
// totalLen is the size of the whole packet and is how it is split again.
//  - Control (endpoint 0): a 12 bytes header, then the data of a host to device request. A reply to a
//    device to host request carries the data it has, and says how much in wLength.
//  - Other endpoints: 4 bytes of header, then the data. The endpoint is the USB address (0x81, 0x02, 0x83).
//  - Standard control requests are not forwarded once the device is plugged: the host reads the descriptors
//    from the PSP when the connection is made (GET_DESCRIPTOR, device / configuration / strings) and
//    answers its guest from that copy.
//  - A control OUT request is acknowledged with the 4 bytes packet (endpoint 0, no payload), whether it
//    carried data or not.
//  - An IN request (host to PSP, endpoint 0x81 or 0x83): the host wants data from that endpoint. One byte
//    of payload, the size of its buffer. The PSP answers with a data packet on the same endpoint, sent when
//    a sceUsbbdReqSend of its driver is waiting. There is one request per endpoint, the host doesn't ask
//    again until it was answered (or PSPCM_IN_REQUEST_TIMEOUT_MS passed).
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
constexpr u32 PSPCM_IN_REQUEST_TIMEOUT_MS = 2000;

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

// The descriptors of the device, as the PSP's usbd stack answered them (raw USB descriptors).
struct pspcm_descriptors
{
	std::vector<u8> device;              // the 18 bytes
	std::vector<u8> config;              // the configuration followed by its interfaces and endpoints
	std::vector<std::string> strings;    // from index 1, UTF-8
};

class usb_device_psp : public usb_device_emulated
{
public:
	// Without descriptors, or with ones that aren't valid, a built-in set is used.
	usb_device_psp(const std::array<u8, 7>& location, const pspcm_descriptors* desc = nullptr);
	~usb_device_psp();

	void control_transfer(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u32 buf_size, u8* buf, UsbTransfer* transfer) override;
	void interrupt_transfer(u32 buf_size, u8* buf, u32 endpoint, UsbTransfer* transfer) override;

	// Next packet for the PSP. False if there is none.
	bool get_output(pspcm_data& pk);
	// A packet received from the PSP. False if its endpoint isn't one of the device's.
	bool add_from_psp(const pspcm_data& pk);
	// Makes the device the PSP described. False (and nothing changed) if the descriptors don't hold together.
	bool load_descriptors(const pspcm_descriptors& desc);

private:
	pspcm_queue m_out_data; // common for ctrl ep and bulk requests
	pspcm_queue m_ctrl_resp;
	pspcm_queue m_bulk_in;
	pspcm_queue m_interrupt_in;
	// When the request for an IN endpoint (0: bulk, 1: interrupt) was sent, 0 if none is waiting.
	std::atomic<s64> m_in_request_since[2]{};
	static int in_endpoint_index(u32 endpoint) { return endpoint == PSPCM_EP_BULK_IN ? 0 : 1; }
	static s64 now_ms() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() + 1; }
	// Asks the PSP for data, unless that was already done and it hasn't answered.
	void request_in(u32 endpoint, u32 buf_size);
	void add_ctrl(u8 bmRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength, u8* buf);
	void add_data(u8 endpoint, u32 buf_size, u8* buf);
	// Takes a packet of an IN endpoint into buf, the rest of it stays for the next transfer.
	// False if there was nothing.
	bool take_in_data(pspcm_queue& queue, u32 buf_size, u8* buf, UsbTransfer* transfer);
};
