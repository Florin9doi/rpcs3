#pragma once

#include "Emu/Io/PSP.h"

#include <functional>
#include <memory>
#include <vector>

// The link to the PSP emulator: connects to it, reads the descriptors of the device it describes, and
// carries the packets between it and the emulated USB device (usb_device_psp).
//
// Plugging the device in and out of the emulated USB bus belongs to the USB code, it is done by the
// two functions the manager is given.
class pspcm_manager
{
public:
	// Creates the device from the descriptors read from the PSP (null if there are none: the device uses
	// its built-in ones), plugs it in, and returns it.
	using connect_fn = std::function<std::shared_ptr<usb_device_psp>(const pspcm_descriptors*)>;
	// Unplugs the device.
	using disconnect_fn = std::function<void(const std::shared_ptr<usb_device_psp>&)>;

	pspcm_manager(connect_fn connect, disconnect_fn disconnect);
	~pspcm_manager();

	void operator()();

	static constexpr auto thread_name = "UsbPspCm manager"sv;

private:
	int start_server();
	void close_server();
	void disconnect();
	void process_stream();
	bool take_packet(pspcm_data& pk);
	bool control_in(u8 request, u16 value, u16 index, std::vector<u8>& out);
	bool fetch_descriptors(pspcm_descriptors& desc);

	connect_fn m_connect;
	disconnect_fn m_disconnect;

	bool m_device_connected = false;
	s64 m_server_socket = -1; // SOCKET or fd, -1 when there is none
	std::vector<u8> m_stream; // received bytes not yet forming a whole packet
	std::shared_ptr<usb_device_psp> m_psp_dev;
};
