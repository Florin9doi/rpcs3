#include "stdafx.h"

#ifdef _WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#endif

#include "pspcm_manager.h"
#include "Emu/Io/usb_device.h"
#include "Utilities/Thread.h"
#include "Utilities/StrUtil.h"
#include <algorithm>
#include <cstring>
#include <thread>

LOG_CHANNEL(pspcm_log);

namespace
{
#ifdef _WIN32
	using pspcm_socket_t = SOCKET;
	constexpr pspcm_socket_t PSPCM_INVALID_SOCKET = INVALID_SOCKET;
	void pspcm_close_socket(pspcm_socket_t s) { closesocket(s); }
	bool pspcm_would_block() { return WSAGetLastError() == WSAEWOULDBLOCK; }
	constexpr int PSPCM_SEND_FLAGS = 0;
#else
	using pspcm_socket_t = int;
	constexpr pspcm_socket_t PSPCM_INVALID_SOCKET = -1;
	void pspcm_close_socket(pspcm_socket_t s) { close(s); }
	bool pspcm_would_block() { return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR; }
	constexpr int PSPCM_SEND_FLAGS = MSG_NOSIGNAL; // a closed peer must not raise SIGPIPE
#endif

	// The socket is non blocking, so wait for it when its buffer is full.
	bool pspcm_send_all(pspcm_socket_t s, const char* data, int size)
	{
		int sent = 0;
		int stalled = 0;
		while (sent < size)
		{
			const int ret = static_cast<int>(send(s, data + sent, size - sent, PSPCM_SEND_FLAGS));
			if (ret > 0)
			{
				sent += ret;
				stalled = 0;
			}
			else if (ret < 0 && pspcm_would_block() && stalled++ < 2000)
			{
				std::this_thread::sleep_for(1ms);
			}
			else
			{
				return false;
			}
		}
		return true;
	}
}

pspcm_manager::pspcm_manager(connect_fn connect, disconnect_fn disconnect)
	: m_connect(std::move(connect)), m_disconnect(std::move(disconnect))
{
	pspcm_log.notice("pspcm_manager::pspcm_manager()");
#ifdef _WIN32
	WSADATA wsa_data;
	WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
}

pspcm_manager::~pspcm_manager()
{
	pspcm_log.notice("pspcm_manager::~pspcm_manager()");
	close_server();
}

void pspcm_manager::close_server()
{
	if (m_server_socket != static_cast<s64>(PSPCM_INVALID_SOCKET))
	{
		shutdown(static_cast<pspcm_socket_t>(m_server_socket), 2 /* both directions */);
		pspcm_close_socket(static_cast<pspcm_socket_t>(m_server_socket));
		m_server_socket = static_cast<s64>(PSPCM_INVALID_SOCKET);
	}
	m_stream.clear();
}

int pspcm_manager::start_server()
{
	struct addrinfo hints{};
	struct addrinfo* info = nullptr;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	if (getaddrinfo("127.0.0.1", "27015", &hints, &info) != 0 || !info)
	{
		pspcm_log.error("pspcm_manager: getaddrinfo error");
		return -1;
	}

	const pspcm_socket_t s = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
	if (s == PSPCM_INVALID_SOCKET)
	{
		pspcm_log.error("pspcm_manager: Error creating socket");
		freeaddrinfo(info);
		return -2;
	}

	if (connect(s, info->ai_addr, static_cast<int>(info->ai_addrlen)) != 0)
	{
		// PPSSPP isn't listening (yet), this is retried
		freeaddrinfo(info);
		pspcm_close_socket(s);
		return -3;
	}
	freeaddrinfo(info);

	// The packets are small and the PSP waits for the answers
	const int one = 1;
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));

#ifdef _WIN32
	u_long mode = 1;
	ioctlsocket(s, FIONBIO, &mode);
#else
	fcntl(s, F_SETFL, fcntl(s, F_GETFL) | O_NONBLOCK);
#endif

	m_server_socket = static_cast<s64>(s);
	m_stream.clear();
	return 0;
}

void pspcm_manager::disconnect()
{
	if (!m_device_connected)
	{
		return;
	}

	pspcm_log.notice("pspcm_manager: PSP disconnected");
	m_device_connected = false;
	close_server();

	m_disconnect(m_psp_dev);
	m_psp_dev.reset();
}

// The TCP stream has no packet boundaries: takes the first complete packet, if there is one.
bool pspcm_manager::take_packet(pspcm_data& pk)
{
	while (m_stream.size() >= PSPCM_HEADER_LEN)
	{
		if (m_stream[0] != 0xf0 || m_stream[1] != 0x0f || m_stream[2] < PSPCM_HEADER_LEN)
		{
			pspcm_log.warning("pspcm_manager: lost sync, skipping a byte (%02x)", m_stream[0]);
			m_stream.erase(m_stream.begin());
			continue;
		}

		const usz len = m_stream[2];
		if (m_stream.size() < len)
		{
			return false; // the rest comes later
		}

		pk = {};
		std::memcpy(&pk, m_stream.data(), std::min<usz>(len, sizeof(pk)));
		m_stream.erase(m_stream.begin(), m_stream.begin() + len);
		return true;
	}
	return false;
}

void pspcm_manager::process_stream()
{
	pspcm_data pk{};
	while (take_packet(pk))
	{
		if (!m_psp_dev->add_from_psp(pk))
		{
			pspcm_log.warning("pspcm_manager: packet for unknown endpoint %02x dropped", pk.endpoint);
		}
	}
}

// A standard device to host control request, before the device is plugged. out = the data of the reply.
bool pspcm_manager::control_in(u8 request, u16 value, u16 index, std::vector<u8>& out)
{
	const pspcm_socket_t s = static_cast<pspcm_socket_t>(m_server_socket);

	pspcm_data pk{};
	pk.magic = PSPCM_MAGIC;
	pk.totalLen = static_cast<u8>(PSPCM_CTRL_HEADER_LEN);
	pk.endpoint = 0;
	pk.ctrl.bmRequestType = 0x80; // device to host, standard, device
	pk.ctrl.bRequest = request;
	pk.ctrl.wValue = value;
	pk.ctrl.wIndex = index;
	pk.ctrl.wLength = PSPCM_MAX_DATA;
	if (!pspcm_send_all(s, reinterpret_cast<const char*>(&pk), PSPCM_CTRL_HEADER_LEN))
	{
		return false;
	}

	for (u32 retry = 0; retry < PSPCM_CTRL_TIMEOUT_MS && thread_ctrl::state() != thread_state::aborting; retry++)
	{
		char buf[512];
		const int ret = static_cast<int>(recv(s, buf, sizeof(buf), 0));
		if (ret > 0)
		{
			m_stream.insert(m_stream.end(), buf, buf + ret);
		}
		else if (ret == 0 || !pspcm_would_block())
		{
			return false;
		}

		pspcm_data resp{};
		while (take_packet(resp))
		{
			if (resp.endpoint == 0 && resp.ctrl.bRequest == request)
			{
				const u32 len = resp.totalLen > PSPCM_CTRL_HEADER_LEN ? std::min<u32>(resp.totalLen - PSPCM_CTRL_HEADER_LEN, PSPCM_MAX_DATA) : 0;
				out.assign(reinterpret_cast<const u8*>(resp.ctrl.data), reinterpret_cast<const u8*>(resp.ctrl.data) + len);
				return len > 0;
			}
		}
		std::this_thread::sleep_for(1ms);
	}
	return false;
}

// Reads the device, the configuration and the strings it uses from the PSP.
bool pspcm_manager::fetch_descriptors(pspcm_descriptors& desc)
{
	if (!control_in(6 /* GET_DESCRIPTOR */, USB_DESCRIPTOR_DEVICE << 8, 0, desc.device) || desc.device.size() != 18)
	{
		return false;
	}

	if (!control_in(6, USB_DESCRIPTOR_CONFIG << 8, 0, desc.config) || desc.config.size() < 9)
	{
		return false;
	}
	const usz total = desc.config[2] | (desc.config[3] << 8);
	if (desc.config.size() < total)
	{
		pspcm_log.error("pspcm_manager: the configuration is %d bytes, only %d fit a packet", total, desc.config.size());
		return false;
	}
	desc.config.resize(total);

	// The strings the descriptors point to: the device's, the configuration's and the interfaces'.
	u8 last_string = std::max({ desc.device[14], desc.device[15], desc.device[16], desc.config[6] });
	for (usz pos = 9; pos + 9 <= desc.config.size() && desc.config[pos] >= 2; pos += desc.config[pos])
	{
		if (desc.config[pos + 1] == USB_DESCRIPTOR_INTERFACE)
		{
			last_string = std::max(last_string, desc.config[pos + 8]);
		}
	}

	u16 langid = 0x0409;
	std::vector<u8> str;
	if (last_string && control_in(6, USB_DESCRIPTOR_STRING << 8, 0, str) && str.size() >= 4)
	{
		langid = str[2] | (str[3] << 8);
	}

	for (u32 i = 1; i <= last_string; i++)
	{
		std::string text;
		if (control_in(6, (USB_DESCRIPTOR_STRING << 8) | i, langid, str) && str.size() >= 2)
		{
			std::u16string u16;
			for (usz p = 2; p + 1 < str.size() && p < str[0]; p += 2)
			{
				u16.push_back(static_cast<char16_t>(str[p] | (str[p + 1] << 8)));
			}
			text = utf16_to_utf8(u16);
		}
		desc.strings.push_back(std::move(text));
	}
	return true;
}

void pspcm_manager::operator()()
{
	while (thread_ctrl::state() != thread_state::aborting)
	{
		if (!m_device_connected)
		{
			if (start_server() == 0)
			{
				pspcm_log.notice("pspcm_manager: connected to PPSSPP");
				// What the device is comes from the PSP, the built-in descriptors are the fallback.
				pspcm_descriptors desc;
				const bool have_desc = fetch_descriptors(desc);
				if (!have_desc)
				{
					pspcm_log.warning("pspcm_manager: no descriptors from the PSP, using the built-in ones");
				}
				m_psp_dev = m_connect(have_desc ? &desc : nullptr);
				m_device_connected = true;
			}
			else
			{
				std::this_thread::sleep_for(200ms);
			}
			continue;
		}

		const pspcm_socket_t s = static_cast<pspcm_socket_t>(m_server_socket);

		// To the PSP
		pspcm_data pk{};
		bool failed = false;
		while (!failed && m_psp_dev->get_output(pk))
		{
			const int len = std::min<int>(pk.totalLen, sizeof(pk));
			if (!pspcm_send_all(s, reinterpret_cast<const char*>(&pk), len))
			{
				failed = true;
			}
		}

		// From the PSP. 0 is the peer closing the connection, which the sends alone don't tell.
		char buf[512];
		while (!failed)
		{
			const int ret = static_cast<int>(recv(s, buf, sizeof(buf), 0));
			if (ret > 0)
			{
				m_stream.insert(m_stream.end(), buf, buf + ret);
				process_stream();
			}
			else
			{
				failed = ret == 0 || !pspcm_would_block();
				break;
			}
		}

		if (failed)
		{
			disconnect();
			continue;
		}

		std::this_thread::sleep_for(1ms);
	}

	disconnect();
}
