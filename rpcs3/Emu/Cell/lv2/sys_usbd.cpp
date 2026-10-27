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

#include "sys_usbd.h"
#include "sys_ppu_thread.h"
#include "sys_sync.h"

#include <queue>
#include <thread>
#include <vector>
#include "Emu/System.h"
#include "Emu/system_config.h"
#include "Emu/IdManager.h"
#include "Emu/vfs_config.h"

#include "Emu/Cell/PPUThread.h"
#include "Emu/Cell/ErrorCodes.h"
#include "Emu/Cell/timers.hpp"

#include "Emu/Io/usb_device.h"
#include "Emu/Io/usb_microphone.h"
#include "Emu/Io/usb_vfs.h"
#include "Emu/Io/Skylander.h"
#include "Emu/Io/Infinity.h"
#include "Emu/Io/Dimensions.h"
#include "Emu/Io/KamenRider.h"
#include "Emu/Io/GHLtar.h"
#include "Emu/Io/ghltar_config.h"
#include "Emu/Io/guncon3_config.h"
#include "Emu/Io/topshotelite_config.h"
#include "Emu/Io/topshotfearmaster_config.h"
#include "Emu/Io/Buzz.h"
#include "Emu/Io/buzz_config.h"
#include "Emu/Io/GameTablet.h"
#include "Emu/Io/GunCon3.h"
#include "Emu/Io/PSP.h"
#include "Emu/Io/TopShotElite.h"
#include "Emu/Io/TopShotFearmaster.h"
#include "Emu/Io/Turntable.h"
#include "Emu/Io/turntable_config.h"
#include "Emu/Io/RB3MidiKeyboard.h"
#include "Emu/Io/RB3MidiGuitar.h"
#include "Emu/Io/RB3MidiDrums.h"
#include "Emu/Io/rb3drums_config.h"
#include "Emu/Io/usio.h"
#include "Emu/Io/usio_config.h"
#include "Emu/Io/midi_config_types.h"
#ifdef HAVE_SDL3
#include "Emu/Io/LogitechG27.h"
#endif

#ifdef _WIN32
#if LIBUSB_WINDOWS_HOTPLUG && LIBUSB_API_VERSION >= 0x0100010C
#define SYS_USBD_HOTPLUG_SUPPORTED 1
#endif
#elif LIBUSB_API_VERSION >= 0x01000102
#define SYS_USBD_HOTPLUG_SUPPORTED 1
#endif

LOG_CHANNEL(sys_usbd);

cfg_buzz g_cfg_buzz;
cfg_ghltars g_cfg_ghltar;
cfg_turntables g_cfg_turntable;
cfg_usios g_cfg_usio;
cfg_guncon3 g_cfg_guncon3;
cfg_topshotelite g_cfg_topshotelite;
cfg_topshotfearmaster g_cfg_topshotfearmaster;

extern atomic_t<bool> libusbd_active;

template <>
void fmt_class_string<libusb_transfer>::format(std::string& out, u64 arg)
{
	const auto& transfer = get_object(arg);
	const int data_start = transfer.type == LIBUSB_TRANSFER_TYPE_CONTROL ? LIBUSB_CONTROL_SETUP_SIZE : 0;
	fmt::append(out, "TR[r:%d][sz:%d] => %s", +transfer.status, transfer.actual_length, fmt::buf_to_hexstring(&transfer.buffer[data_start], transfer.actual_length));
}

struct UsbLdd
{
	u16 id_vendor{};
	u16 id_product_min{};
	u16 id_product_max{};
};

struct UsbPipe
{
	std::shared_ptr<usb_device> device = nullptr;

	u8 endpoint = 0;
};

struct usb_allow_list_entry
{
	u16 id_vendor;
	u16 id_product_min;
	u16 id_product_max;
	std::string_view device_name;
	u16(*max_device_count)(void);
	std::shared_ptr<usb_device>(*make_instance)(u32, const std::array<u8, 7>&);
	auto operator<(const usb_allow_list_entry& r) const
	{
		return std::tuple(id_vendor, id_product_min, id_product_max, device_name, max_device_count, make_instance) < std::tuple(r.id_vendor, r.id_product_min, r.id_product_max, device_name, max_device_count, make_instance);
	}
};
class usb_handler_thread
{
public:
	usb_handler_thread();
	~usb_handler_thread();

	SAVESTATE_INIT_POS(14);

	usb_handler_thread(utils::serial& ar) : usb_handler_thread()
	{
		is_init = !!ar.pop<u8>();
	}

	void save(utils::serial& ar)
	{
		ar(u8{is_init.load()});
	}

	// Thread loop
	void operator()();

	// Called by the libusb callback function to notify transfer completion
	void transfer_complete(libusb_transfer* transfer);

	// LDDs handling functions
	bool add_ldd(std::string_view product, u16 id_vendor, u16 id_product_min, u16 id_product_max);
	bool remove_ldd(std::string_view product);

	// Pipe functions
	u32 open_pipe(u32 device_handle, u8 endpoint);
	bool close_pipe(u32 pipe_id);
	bool is_pipe(u32 pipe_id) const;
	const UsbPipe& get_pipe(u32 pipe_id) const;

	// Events related functions
	bool get_event(vm::ptr<u64>& arg1, vm::ptr<u64>& arg2, vm::ptr<u64>& arg3);
	void add_event(u64 arg1, u64 arg2, u64 arg3);

	// Transfers related functions
	std::pair<u32, UsbTransfer&> get_free_transfer();
	std::pair<u32, u32> get_transfer_status(u32 transfer_id);
	std::pair<u32, UsbDeviceIsoRequest> get_isochronous_transfer_status(u32 transfer_id);
	void push_fake_transfer(UsbTransfer* transfer);

	const std::array<u8, 7>& get_new_location();
	void connect_usb_device(std::shared_ptr<usb_device> dev, bool update_usb_devices = false);
	void disconnect_usb_device(std::shared_ptr<usb_device> dev, bool update_usb_devices = false);
	void reconnect_usb_device(u32 assigned_number);
	void set_usb_device_attached(usb_device* raw_dev, bool attached);
	void print_dev_list();

	// Map of devices actively handled by the ps3(device_id, device)
	std::map<u32, std::pair<UsbInternalDevice, std::shared_ptr<usb_device>>> handled_devices;
	std::map<u8, std::pair<input::product_type, std::shared_ptr<usb_device>>> pad_to_usb;

	shared_mutex mutex;
	atomic_t<bool> is_init = false;

	// sys_usbd_receive_event PPU Threads
	shared_mutex mutex_sq;
	ppu_thread* sq{};

	atomic_t<u64> usb_hotplug_timeout = umax;
	atomic_t<bool> hotplug_supported = false;

	static constexpr auto thread_name = "Usb Manager Thread"sv;

private:
	// Lock free functions for internal use(ie make sure to lock before using those)
	UsbTransfer& get_transfer(u32 transfer_id);
	u32 get_free_transfer_id();

	void send_message(u32 message, u32 tr_id);
	void perform_scan();

private:
	// Counters for device IDs, transfer IDs and pipe IDs
	atomic_t<u8> dev_counter = 1;
	u32 transfer_counter     = 0;
	u32 pipe_counter         = 0x10; // Start at 0x10 only for tracing purposes

	// List of device drivers
	std::unordered_map<std::string, UsbLdd, fmt::string_hash, std::equal_to<>> ldds;

	const std::vector<usb_allow_list_entry> device_allow_list
	{
		// Portals
		{0x1430, 0x0150, 0x0150, "Skylanders Portal", &usb_device_skylander::get_num_emu_devices, &usb_device_skylander::make_instance},
		{0x0E6F, 0x0129, 0x0129, "Disney Infinity Base", &usb_device_infinity::get_num_emu_devices, &usb_device_infinity::make_instance},
		{0x0E6F, 0x0241, 0x0241, "Lego Dimensions Portal", &usb_device_dimensions::get_num_emu_devices, &usb_device_dimensions::make_instance},
		{0x0E6F, 0x200A, 0x200A, "Kamen Rider Summonride Portal", &usb_device_kamen_rider::get_num_emu_devices, &usb_device_kamen_rider::make_instance},

		// Cameras
		// {0x1415, 0x0020, 0x2000, "Sony Playstation Eye", nullptr, nullptr}, // TODO: verifiy

		// Music devices
		{0x1415, 0x0000, 0x0000, "Singstar Microphone", nullptr, nullptr},
		// {0x1415, 0x0020, 0x0020, "SingStar Microphone Wireless", nullptr, nullptr}, // TODO: verifiy
		// {0x12ba, 0x00f0, 0x00f0, "Bandfuse USB Guitar Adapter", nullptr, nullptr},
		// {0x28aa, 0x0001, 0x0001, "Bandfuse USB Microphone", nullptr, nullptr},
		// {0x046d, 0x0a03, 0x0a03, "Logitech Microphone", nullptr, nullptr},

		{0x12BA, 0x00FF, 0x00FF, "Rocksmith Guitar Adapter", nullptr, nullptr},
		{0x12BA, 0x0100, 0x0100, "Guitar Hero Guitar", nullptr, nullptr},
		{0x12BA, 0x0120, 0x0120, "Guitar Hero Drums", nullptr, nullptr},
		{0x12BA, 0x074B, 0x074B, "Guitar Hero Live Guitar", &usb_device_ghltar::get_num_emu_devices, &usb_device_ghltar::make_instance},

		{0x12BA, 0x0140, 0x0140, "DJ Hero Turntable", &usb_device_turntable::get_num_emu_devices, &usb_device_turntable::make_instance},
		{0x12BA, 0x0200, 0x020F, "Harmonix Guitar", nullptr, nullptr},
		{0x12BA, 0x0210, 0x021F, "Harmonix Drums", nullptr, nullptr},
		{0x12BA, 0x2330, 0x233F, "Harmonix Keyboard", nullptr, nullptr},
		{0x12BA, 0x2430, 0x243F, "Harmonix Button Guitar", nullptr, nullptr},
		{0x12BA, 0x2530, 0x253F, "Harmonix Real Guitar", nullptr, nullptr},

		{0x1BAD, 0x0004, 0x0004, "Harmonix RB1 Guitar - Wii", nullptr, nullptr},
		{0x1BAD, 0x0005, 0x0005, "Harmonix RB1 Drums - Wii", nullptr, nullptr},
		{0x1BAD, 0x3010, 0x301F, "Harmonix RB2 Guitar - Wii", nullptr, nullptr},
		{0x1BAD, 0x3110, 0x313F, "Harmonix RB2 Drums - Wii", nullptr, nullptr},
		{0x1BAD, 0x3330, 0x333F, "Harmonix Keyboard - Wii", nullptr, nullptr},
		{0x1BAD, 0x3430, 0x343F, "Harmonix Button Guitar - Wii", nullptr, nullptr},
		{0x1BAD, 0x3530, 0x353F, "Harmonix Real Guitar - Wii", nullptr, nullptr},

		// Top Shot Elite controllers
		{0x12BA, 0x04A0, 0x04A0, "Top Shot Elite", nullptr, nullptr},
		{0x12BA, 0x04A1, 0x04A1, "Top Shot Fearmaster", nullptr, nullptr},
		{0x12BA, 0x04B0, 0x04B0, "Rapala Fishing Rod", nullptr, nullptr},

		// Wheels
#ifdef HAVE_SDL3
		{0x046D, 0xC283, 0xC29B, "lgFF_c283_c29b", &usb_device_logitech_g27::get_num_emu_devices, &usb_device_logitech_g27::make_instance},
#else
		{0x046D, 0xC283, 0xC29B, "lgFF_c283_c29b", nullptr, nullptr},
#endif
		{0x046D, 0xCA03, 0xCA03, "lgFF_ca03_ca03", nullptr, nullptr},
		{0x044F, 0xB652, 0xB652, "Thrustmaster FGT FFB old", nullptr, nullptr},
		{0x044F, 0xB653, 0xB653, "Thrustmaster RGT FFB Pro", nullptr, nullptr},
		{0x044F, 0xB654, 0xB654, "Thrustmaster FGT FFB", nullptr, nullptr},
		{0x044F, 0xb655, 0xb655, "Thrustmaster FGT Rumble 3-in-1", nullptr, nullptr},
		{0x044F, 0xB65A, 0xB65A, "Thrustmaster F430", nullptr, nullptr},
		{0x044F, 0xB65D, 0xB65D, "Thrustmaster FFB", nullptr, nullptr},
		{0x044F, 0xB65E, 0xB65E, "Thrustmaster TRS", nullptr, nullptr},
		{0x044F, 0xB660, 0xB660, "Thrustmaster T500 RS Gear Shift", nullptr, nullptr},

		// GT6
		{0x2833, 0x0001, 0x0001, "Oculus", nullptr, nullptr},

		// Buzz controllers
		{0x054C, 0x1000, 0x1040, "buzzer0", &usb_device_buzz::get_num_emu_devices, &usb_device_buzz::make_instance},
		{0x054C, 0x0001, 0x0041, "buzzer1", nullptr, nullptr},
		{0x054C, 0x0042, 0x0042, "buzzer2", nullptr, nullptr},
		{0x046D, 0xC220, 0xC220, "buzzer9", nullptr, nullptr},

		// GCon3 Gun
		{0x0B9A, 0x0800, 0x0800, "guncon3", nullptr, nullptr},

		// uDraw GameTablet
		{0x20D6, 0xCB17, 0xCB17, "uDraw GameTablet", nullptr, nullptr},

		// TV Tuners
		{0x1415, 0x0003, 0x0003, "PlayTV SCEH-0036", nullptr, nullptr},
		// {0x054c, 0x04b2, 0x04b2, "Torne CECH-ZD1 J", nullptr, nullptr},

		// PSP Devices
		{0x054C, 0x01C8, 0x01C8, "PSP Type A", nullptr, nullptr},
		{0x054C, 0x01C9, 0x01C9, "PSP Type B", nullptr, nullptr},
		{0x054C, 0x01CA, 0x01CA, "PSP Type C", nullptr, nullptr},
		{0x054C, 0x01CB, 0x01CB, "PSP Type D", nullptr, nullptr}, // UsbPspCm
		{0x054C, 0x02D2, 0x02D2, "PSP Slim", nullptr, nullptr},

		// 0x0900: "H050 USJ(C) PCB rev00", 0x0910: "USIO PCB rev00"
		{0x0B9A, 0x0900, 0x0910, "PS3A-USJ", &usb_device_usio::get_num_emu_devices, &usb_device_usio::make_instance},

		// Densha de GO! controller
		{0x0AE4, 0x0004, 0x0004, "Densha de GO! Type 2 Controller", nullptr, nullptr},

		// EA Active 2 dongle for connecting wristbands & legband
		{0x21A4, 0xAC27, 0xAC27, "EA Active 2 Dongle", nullptr, nullptr},

		// Tony Hawk RIDE Skateboard
		{0x12BA, 0x0400, 0x0400, "Tony Hawk RIDE Skateboard Controller", nullptr, nullptr},

		// Sony Stereo Headsets
		{0x12BA, 0x0032, 0x0032, "Wireless Stereo Headset", nullptr, nullptr},
		{0x12BA, 0x0042, 0x0042, "Wireless Stereo Headset", nullptr, nullptr},
	};

	// List of pipes
	std::map<u32, UsbPipe> open_pipes;
	// Transfers infos
	shared_mutex mutex_transfers;
	std::array<UsbTransfer, MAX_SYS_USBD_TRANSFERS> transfers;
	std::vector<UsbTransfer*> fake_transfers;

	// Queue of pending usbd events
	std::queue<std::tuple<u64, u64, u64>> usbd_events;

	// List of devices "connected" to the ps3
	std::array<u8, 7> location{};
	std::vector<std::shared_ptr<usb_device>> usb_devices;
	std::unordered_map<uint64_t, std::shared_ptr<usb_device>> usb_passthrough_devices;

	libusb_context* ctx = nullptr;

#if SYS_USBD_HOTPLUG_SUPPORTED
	libusb_hotplug_callback_handle callback_handle {};
#endif
};

struct pspcm_manager
{
	void operator()();
	pspcm_manager();
	~pspcm_manager();
	int start_server();
	void close_server();
	void disconnect();
	void process_stream();
	bool take_packet(pspcm_data& pk);
	bool control_in(u8 request, u16 value, u16 index, std::vector<u8>& out);
	bool fetch_descriptors(pspcm_descriptors& desc);

	static constexpr auto thread_name = "UsbPspCm manager"sv;

private:
	bool m_device_connected = false;
	s64 m_server_socket = -1; // SOCKET or fd, -1 when there is none
	std::vector<u8> m_stream; // received bytes not yet forming a whole packet
	std::shared_ptr<usb_device_psp> m_psp_dev;
};

void LIBUSB_CALL callback_transfer(struct libusb_transfer* transfer)
{
	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	if (!usbh.is_init)
		return;

	usbh.transfer_complete(transfer);
}

#if SYS_USBD_HOTPLUG_SUPPORTED
static int LIBUSB_CALL hotplug_callback(libusb_context* /*ctx*/, libusb_device* /*dev*/, libusb_hotplug_event event, void* /*user_data*/)
{
	handle_hotplug_event(event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, true);
	return 0;
}
#endif

#if LIBUSB_API_VERSION >= 0x0100010A
static void LIBUSB_CALL log_cb(libusb_context* /*ctx*/, enum libusb_log_level level, const char* str)
{
	if (!str)
		return;

	const std::string_view msg = fmt::trim_sv(str, " \t\n");

	switch (level)
	{
	case LIBUSB_LOG_LEVEL_ERROR:
		sys_usbd.error("libusb log: %s", msg);
		break;
	case LIBUSB_LOG_LEVEL_WARNING:
		sys_usbd.warning("libusb log: %s", msg);
		break;
	case LIBUSB_LOG_LEVEL_INFO:
		sys_usbd.notice("libusb log: %s", msg);
		break;
	case LIBUSB_LOG_LEVEL_DEBUG:
		sys_usbd.trace("libusb log: %s", msg);
		break;
	default:
		break;
	}
}
#endif

void usb_handler_thread::perform_scan()
{
	// look if any device which we could be interested in is actually connected
	libusb_device** list = nullptr;
	const auto ndev = libusb_get_device_list(ctx, &list);
	std::set<uint64_t> seen_usb_devices;

	if (ndev < 0)
	{
		sys_usbd.error("Failed to get device list: %s", libusb_error_name(static_cast<s32>(ndev)));
		return;
	}

	for (auto index = 0; index < ndev; index++)
	{
		libusb_device* dev = list[index];
		libusb_device_descriptor desc;
		if (int res = libusb_get_device_descriptor(dev, &desc); res < 0)
		{
			sys_usbd.error("Failed to get device descriptor: %s", libusb_error_name(res));
			continue;
		}

		const u8 port = libusb_get_port_number(dev);
		const u8 address = libusb_get_device_address(dev);
		const u64 usb_id = (static_cast<uint64_t>(desc.idVendor) << 48) | (static_cast<uint64_t>(desc.idProduct) << 32) | (static_cast<uint64_t>(port) << 8) | address;

		seen_usb_devices.insert(usb_id);
		if (usb_passthrough_devices.contains(usb_id))
		{
			continue;
		}

		for (const auto& entry : device_allow_list)
		{
			// attach
			if (desc.idVendor == entry.id_vendor
				&& desc.idProduct >= entry.id_product_min
				&& desc.idProduct <= entry.id_product_max)
			{
#ifdef __APPLE__
				// On macOS, libusb cannot claim HID interfaces, so passing through a real
				// controller that also has an emulated implementation yields a non-functional
				// device and silently overrides the user's emulated-device setting (the emulated
				// setup below only fills slots not already passed through). Prefer the emulated
				// implementation whenever the user has enabled it. (e.g. DJ Hero Turntable)
				if (entry.make_instance && entry.max_device_count && entry.max_device_count() > 0)
				{
					continue;
				}
#endif
				sys_usbd.success("Found device: %s", std::basic_string(entry.device_name));
				libusb_ref_device(dev);
				std::shared_ptr<usb_device_passthrough> usb_dev = std::make_shared<usb_device_passthrough>(dev, desc, get_new_location());
				connect_usb_device(usb_dev, true);
				usb_passthrough_devices[usb_id] = usb_dev;
			}
		}

		if (desc.idVendor == 0x1209 && (desc.idProduct == 0x2882 || desc.idProduct == 0x2883))
		{
			sys_usbd.success("Found device: Santroller");
			// Send the device a specific control transfer so that it jumps to a RPCS3 compatible mode
			libusb_device_handle* lusb_handle;
			if (libusb_open(dev, &lusb_handle) == LIBUSB_SUCCESS)
			{
#ifdef __linux__
				libusb_set_auto_detach_kernel_driver(lusb_handle, true);
				libusb_claim_interface(lusb_handle, 2);
#endif
				libusb_control_transfer(lusb_handle, +LIBUSB_ENDPOINT_IN | +LIBUSB_REQUEST_TYPE_CLASS | +LIBUSB_RECIPIENT_INTERFACE, 0x01, 0x03f2, 2, nullptr, 0, 5000);
				libusb_close(lusb_handle);
			}
			else
			{
				sys_usbd.error("Unable to open Santroller device, make sure Santroller isn't open in the background.");
			}
		}
	}

	for (auto it = usb_passthrough_devices.begin(); it != usb_passthrough_devices.end();)
	{
		auto& dev = *it;
		// If a device is no longer visible, disconnect it
		if (seen_usb_devices.contains(dev.first))
		{
			++it;
		}
		else
		{
			disconnect_usb_device(dev.second, true);
			it = usb_passthrough_devices.erase(it);
		}

	}
	libusb_free_device_list(list, 1);
}

usb_handler_thread::usb_handler_thread()
{
#if LIBUSB_API_VERSION >= 0x0100010A
	libusb_init_option log_lv_opt{};
	log_lv_opt.option = LIBUSB_OPTION_LOG_LEVEL;
	log_lv_opt.value.ival = LIBUSB_LOG_LEVEL_WARNING;// You can also set the LIBUSB_DEBUG env variable instead

	libusb_init_option log_cb_opt{};
	log_cb_opt.option = LIBUSB_OPTION_LOG_CB;
	log_cb_opt.value.log_cbval = &log_cb;

	std::vector<libusb_init_option> options = {
		std::move(log_lv_opt),
		std::move(log_cb_opt)
	};

	if (int res = libusb_init_context(&ctx, options.data(), static_cast<int>(options.size())); res < 0)
#else
	if (int res = libusb_init(&ctx); res < 0)
#endif
	{
		sys_usbd.error("Failed to initialize sys_usbd: %s", libusb_error_name(res));
		return;
	}

#if SYS_USBD_HOTPLUG_SUPPORTED
	if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG))
	{
		if (int res = libusb_hotplug_register_callback(ctx, static_cast<libusb_hotplug_event>(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
			LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT), static_cast<libusb_hotplug_flag>(0), LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
			LIBUSB_HOTPLUG_MATCH_ANY, static_cast<libusb_hotplug_callback_fn>(hotplug_callback), nullptr,
			&callback_handle); res < 0)
		{
			sys_usbd.error("Failed to initialize sys_usbd hotplug: %s", libusb_error_name(res));
		}
		else
		{
			hotplug_supported = true;
		}
	}
#elif defined(_WIN32)
	hotplug_supported = true;
#endif

	for (u32 index = 0; index < MAX_SYS_USBD_TRANSFERS; index++)
	{
		transfers[index].transfer    = libusb_alloc_transfer(8);
		transfers[index].transfer_id = index;
	}

	if (!g_cfg_usio.load())
	{
		sys_usbd.notice("Could not load usio config. Using defaults.");
	}

	sys_usbd.notice("USIO config=\n", g_cfg_usio.to_string());

	if (g_cfg.io.ghltar == ghltar_handler::one_controller || g_cfg.io.ghltar == ghltar_handler::two_controllers)
	{
		if (!g_cfg_ghltar.load())
		{
			sys_usbd.notice("Could not load ghltar config. Using defaults.");
		}

		sys_usbd.notice("Ghltar config=\n", g_cfg_ghltar.to_string());
	}

	if (g_cfg.io.turntable == turntable_handler::one_controller || g_cfg.io.turntable == turntable_handler::two_controllers)
	{
		if (!g_cfg_turntable.load())
		{
			sys_usbd.notice("Could not load turntable config. Using defaults.");
		}

		sys_usbd.notice("Turntable config=\n", g_cfg_turntable.to_string());
	}

	if (g_cfg.io.buzz == buzz_handler::one_controller || g_cfg.io.buzz == buzz_handler::two_controllers)
	{
		if (!g_cfg_buzz.load())
		{
			sys_usbd.notice("Could not load buzz config. Using defaults.");
		}

		sys_usbd.notice("Buzz config=\n", g_cfg_buzz.to_string());
	}

	perform_scan();

	// Set up emulated devices for any devices that are not already being passed through
	std::map<usb_allow_list_entry, int> passthrough_usb_device_counts;
	for (const auto& dev : usb_devices)
	{
		for (const auto& entry : device_allow_list)
		{
			const u16 idVendor = dev->device._device.idVendor;
			const u16 idProduct = dev->device._device.idProduct;
			if (entry.max_device_count != nullptr && (idVendor == entry.id_vendor && idProduct >= entry.id_product_min && idProduct <= entry.id_product_max))
			{
				passthrough_usb_device_counts[entry]++;
			}
		}
	}

	for (const auto& entry : device_allow_list)
	{
		if (entry.max_device_count && entry.make_instance)
		{
			const int count = passthrough_usb_device_counts[entry];
			for (int i = count; i < entry.max_device_count(); i++)
			{
				sys_usbd.success("Emulating device: %s (%d)", std::basic_string(entry.device_name), i + 1);
				auto usb_dev = entry.make_instance(i, get_new_location());
				connect_usb_device(usb_dev, true);
			}
		}
	}

	switch (g_cfg.audio.microphone_type)
	{
		case microphone_handler::null:
			break;
		case microphone_handler::standard:
			usb_devices.push_back(std::make_shared<usb_device_mic>(0, get_new_location(), MicType::Logitech));
			break;
		case microphone_handler::eye_toy:
			usb_devices.push_back(std::make_shared<usb_device_mic>(0, get_new_location(), MicType::EyeToy));
			break;
		case microphone_handler::ps_eye:
			usb_devices.push_back(std::make_shared<usb_device_mic>(0, get_new_location(), MicType::PsEye));
			break;
		case microphone_handler::real_singstar:
		case microphone_handler::singstar:
			usb_devices.push_back(std::make_shared<usb_device_mic>(0, get_new_location(), MicType::SingStar));
			break;
		case microphone_handler::rocksmith:
			usb_devices.push_back(std::make_shared<usb_device_mic>(0, get_new_location(), MicType::Rocksmith));
			break;
	}

	for (int i = 0; i < 8; i++) // Add VFS USB mass storage devices (/dev_usbXXX) to the USB device list
	{
		const auto usb_info = g_cfg_vfs.get_device(g_cfg_vfs.dev_usb, fmt::format("/dev_usb%03d", i));
		if (fs::is_dir(usb_info.path))
			usb_devices.push_back(std::make_shared<usb_device_vfs>(usb_info, get_new_location()));
	}

	const std::string midi_devices = g_cfg.io.midi_devices.to_string();
	const std::vector<std::string_view> devices_list = fmt::split_sv(midi_devices, { "@@@" });
	for (usz index = 0; index < std::min(max_midi_devices, devices_list.size()); index++)
	{
		const midi_device device = midi_device::from_string(::at32(devices_list, index));
		if (device.name.empty()) continue;

		sys_usbd.notice("Adding Emulated Midi Pro Adapter (type=%s, name=%s)", device.type, device.name);

		switch (device.type)
		{
		case midi_device_type::guitar:
			usb_devices.push_back(std::make_shared<usb_device_rb3_midi_guitar>(get_new_location(), device.name, false));
			break;
		case midi_device_type::guitar_22fret:
			usb_devices.push_back(std::make_shared<usb_device_rb3_midi_guitar>(get_new_location(), device.name, true));
			break;
		case midi_device_type::keyboard:
			usb_devices.push_back(std::make_shared<usb_device_rb3_midi_keyboard>(get_new_location(), device.name));
			break;
		case midi_device_type::drums:
			if (!g_cfg_rb3drums.load())
			{
				sys_usbd.notice("Could not load rb3drums config. Using defaults.");
			}

			usb_devices.push_back(std::make_shared<usb_device_rb3_midi_drums>(get_new_location(), device.name));

			sys_usbd.notice("RB3 drums config=\n", g_cfg_rb3drums.to_string());
			break;
		}
	}
}

usb_handler_thread::~usb_handler_thread()
{
	// Ensures shared_ptr<usb_device> are all cleared before terminating libusb
	handled_devices.clear();
	open_pipes.clear();
	usb_devices.clear();
	usb_passthrough_devices.clear();

	for (u32 index = 0; index < MAX_SYS_USBD_TRANSFERS; index++)
	{
		if (transfers[index].transfer)
			libusb_free_transfer(transfers[index].transfer);
	}

#if SYS_USBD_HOTPLUG_SUPPORTED
	if (ctx && hotplug_supported)
		libusb_hotplug_deregister_callback(ctx, callback_handle);
#endif

	if (ctx)
		libusb_exit(ctx);
}

void usb_handler_thread::operator()()
{
	timeval lusb_tv{0, 0};
	if (!hotplug_supported)
	{
		usb_hotplug_timeout = get_system_time() + 4'000'000ull;
	}
	while (ctx && thread_ctrl::state() != thread_state::aborting)
	{
		const u64 now = get_system_time();
		if (now > usb_hotplug_timeout)
		{
			// If we did the hotplug scan each cycle the game performance was significantly degraded, so we only perform this scan
			// every 4 seconds.
			// On systems where hotplug is native, we wait a little bit for devices to settle before we start the scan
			perform_scan();
			usb_hotplug_timeout = hotplug_supported ? umax : (get_system_time() + 4'000'000ull);
		}

		// Process asynchronous requests that are pending
		libusb_handle_events_timeout_completed(ctx, &lusb_tv, nullptr);

		u64 delay = 1'000;

		// Process fake transfers
		if (libusbd_active && !fake_transfers.empty())
		{
			std::lock_guard lock_tf(mutex_transfers);
			u64 timestamp = get_system_time() - Emu.GetPauseTime();

			for (auto it = fake_transfers.begin(); it != fake_transfers.end();)
			{
				auto transfer = *it;

				ensure(transfer->busy && transfer->fake);

				if (transfer->expected_time > timestamp)
				{
					const u64 diff_time = transfer->expected_time - timestamp;

					if (diff_time < delay)
					{
						delay = diff_time;
					}

					++it;
					continue;
				}

				transfer->result = transfer->expected_result;
				transfer->count  = transfer->expected_count;
				transfer->fake   = false;
				transfer->busy   = false;

				send_message(SYS_USBD_TRANSFER_COMPLETE, transfer->transfer_id);
				it = fake_transfers.erase(it); // if we've processed this, then we erase this entry (replacing the iterator with the new reference)
			}
		}

		// If there is no handled devices usb thread is not actively needed
		if (handled_devices.empty())
			thread_ctrl::wait_for(500'000);
		else
			thread_ctrl::wait_for(delay);
	}
}

void usb_handler_thread::send_message(u32 message, u32 tr_id)
{
	add_event(message, tr_id, 0x00);
}

void usb_handler_thread::transfer_complete(struct libusb_transfer* transfer)
{
	std::lock_guard lock_tf(mutex_transfers);

	UsbTransfer* usbd_transfer = static_cast<UsbTransfer*>(transfer->user_data);

	if (transfer->status != 0)
	{
		sys_usbd.error("Transfer Error: %d", +transfer->status);
	}

	switch (transfer->status)
	{
	case LIBUSB_TRANSFER_COMPLETED: usbd_transfer->result = HC_CC_NOERR; break;
	case LIBUSB_TRANSFER_TIMED_OUT: usbd_transfer->result = EHCI_CC_XACT; break;
	case LIBUSB_TRANSFER_OVERFLOW: usbd_transfer->result = EHCI_CC_BABBLE; break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		usbd_transfer->result = EHCI_CC_HALTED;
		for (const auto& dev : usb_devices)
		{
			if (dev->assigned_number == usbd_transfer->assigned_number)
			{
				disconnect_usb_device(dev, true);
				break;
			}
		}
		break;
	case LIBUSB_TRANSFER_ERROR:
	case LIBUSB_TRANSFER_CANCELLED:
	case LIBUSB_TRANSFER_STALL:
	default:
		usbd_transfer->result = EHCI_CC_HALTED;
		break;
	}

	usbd_transfer->count = transfer->actual_length;

	for (s32 index = 0; index < transfer->num_iso_packets; index++)
	{
		u8 iso_status;
		switch (transfer->iso_packet_desc[index].status)
		{
		case LIBUSB_TRANSFER_COMPLETED: iso_status = USBD_HC_CC_NOERR; break;
		case LIBUSB_TRANSFER_TIMED_OUT: iso_status = USBD_HC_CC_XACT; break;
		case LIBUSB_TRANSFER_OVERFLOW: iso_status = USBD_HC_CC_BABBLE; break;
		case LIBUSB_TRANSFER_ERROR:
		case LIBUSB_TRANSFER_CANCELLED:
		case LIBUSB_TRANSFER_STALL:
		case LIBUSB_TRANSFER_NO_DEVICE:
		default: iso_status = USBD_HC_CC_MISSMF; break;
		}

		usbd_transfer->iso_request.packets[index] = ((iso_status & 0xF) << 12 | (transfer->iso_packet_desc[index].actual_length & 0xFFF));
	}

	if (transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL && usbd_transfer->control_destbuf)
	{
		memcpy(usbd_transfer->control_destbuf, transfer->buffer + LIBUSB_CONTROL_SETUP_SIZE, transfer->actual_length);
		usbd_transfer->control_destbuf = nullptr;
	}

	usbd_transfer->busy = false;

	send_message(SYS_USBD_TRANSFER_COMPLETE, usbd_transfer->transfer_id);

	sys_usbd.trace("Transfer complete(0x%x): %s", usbd_transfer->transfer_id, *transfer);
}

bool usb_handler_thread::add_ldd(std::string_view product, u16 id_vendor, u16 id_product_min, u16 id_product_max)
{
	if (ldds.try_emplace(std::string(product), UsbLdd{id_vendor, id_product_min, id_product_max}).second)
	{
		for (const auto& dev : usb_devices)
		{
			if (dev->assigned_number)
				continue;

			if (dev->device._device.idVendor == id_vendor && dev->device._device.idProduct >= id_product_min && dev->device._device.idProduct <= id_product_max)
			{
				connect_usb_device(dev);
			}
		}

		return true;
	}

	return false;
}

bool usb_handler_thread::remove_ldd(std::string_view product)
{
	if (const auto iterator = ldds.find(product); iterator != ldds.end())
	{
		for (const auto& dev : usb_devices)
		{
			if (!dev->assigned_number)
				continue;

			if (dev->device._device.idVendor == iterator->second.id_vendor && dev->device._device.idProduct >= iterator->second.id_product_min && dev->device._device.idProduct <= iterator->second.id_product_max)
			{
				disconnect_usb_device(dev);
			}
		}

		ldds.erase(iterator);
		return true;
	}

	return false;
}

u32 usb_handler_thread::open_pipe(u32 device_handle, u8 endpoint)
{
	open_pipes.emplace(pipe_counter, UsbPipe{handled_devices[device_handle].second, endpoint});
	return pipe_counter++;
}

bool usb_handler_thread::close_pipe(u32 pipe_id)
{
	return open_pipes.erase(pipe_id) != 0;
}

bool usb_handler_thread::is_pipe(u32 pipe_id) const
{
	return open_pipes.count(pipe_id) != 0;
}

const UsbPipe& usb_handler_thread::get_pipe(u32 pipe_id) const
{
	return ::at32(open_pipes, pipe_id);
}

bool usb_handler_thread::get_event(vm::ptr<u64>& arg1, vm::ptr<u64>& arg2, vm::ptr<u64>& arg3)
{
	if (!usbd_events.empty())
	{
		const auto& usb_event = usbd_events.front();
		*arg1                 = std::get<0>(usb_event);
		*arg2                 = std::get<1>(usb_event);
		*arg3                 = std::get<2>(usb_event);
		usbd_events.pop();
		sys_usbd.trace("Received event: arg1=0x%x arg2=0x%x arg3=0x%x", *arg1, *arg2, *arg3);
		return true;
	}

	return false;
}

void usb_handler_thread::add_event(u64 arg1, u64 arg2, u64 arg3)
{
	// sys_usbd events use an internal event queue with SYS_SYNC_PRIORITY protocol
	std::lock_guard lock_sq(mutex_sq);

	if (const auto cpu = lv2_obj::schedule<ppu_thread>(sq, SYS_SYNC_PRIORITY))
	{
		sys_usbd.trace("Sending event(queue): arg1=0x%x arg2=0x%x arg3=0x%x", arg1, arg2, arg3);
		cpu->gpr[4] = arg1;
		cpu->gpr[5] = arg2;
		cpu->gpr[6] = arg3;
		lv2_obj::awake(cpu);
	}
	else
	{
		sys_usbd.trace("Sending event: arg1=0x%x arg2=0x%x arg3=0x%x", arg1, arg2, arg3);
		usbd_events.emplace(arg1, arg2, arg3);
	}
}

u32 usb_handler_thread::get_free_transfer_id()
{
	u32 num_loops = 0;
	do
	{
		num_loops++;
		transfer_counter++;

		if (transfer_counter >= MAX_SYS_USBD_TRANSFERS)
		{
			transfer_counter = 0;
		}

		if (num_loops > MAX_SYS_USBD_TRANSFERS)
		{
			sys_usbd.fatal("Usb transfers are saturated!");
		}
	} while (transfers[transfer_counter].busy);

	return transfer_counter;
}

UsbTransfer& usb_handler_thread::get_transfer(u32 transfer_id)
{
	return transfers[transfer_id];
}

std::pair<u32, UsbTransfer&> usb_handler_thread::get_free_transfer()
{
	std::lock_guard lock_tf(mutex_transfers);

	u32 transfer_id = get_free_transfer_id();
	auto& transfer  = get_transfer(transfer_id);

	libusb_transfer* const transfer_buf = transfer.transfer;
	transfer = {.transfer_id = transfer_id, .transfer = transfer_buf, .busy = true};

	return {transfer_id, transfer};
}

std::pair<u32, u32> usb_handler_thread::get_transfer_status(u32 transfer_id)
{
	std::lock_guard lock_tf(mutex_transfers);

	const auto& transfer = get_transfer(transfer_id);

	return {transfer.result, transfer.count};
}

std::pair<u32, UsbDeviceIsoRequest> usb_handler_thread::get_isochronous_transfer_status(u32 transfer_id)
{
	std::lock_guard lock_tf(mutex_transfers);

	const auto& transfer = get_transfer(transfer_id);

	return {transfer.result, transfer.iso_request};
}

void usb_handler_thread::push_fake_transfer(UsbTransfer* transfer)
{
	std::lock_guard lock_tf(mutex_transfers);
	fake_transfers.push_back(transfer);
}

const std::array<u8, 7>& usb_handler_thread::get_new_location()
{
	location[0]++;
	return location;
}

void usb_handler_thread::connect_usb_device(std::shared_ptr<usb_device> dev, bool update_usb_devices)
{
	if (update_usb_devices)
		usb_devices.push_back(dev);

	if (!dev->is_attachable())
	{
		sys_usbd.trace("USB device(VID=0x%04x, PID=0x%04x) is not attachable, skipping connection", dev->device._device.idVendor, dev->device._device.idProduct);
		return;
	}

	for (const auto& [name, ldd] : ldds)
	{
		if (dev->device._device.idVendor == ldd.id_vendor && dev->device._device.idProduct >= ldd.id_product_min && dev->device._device.idProduct <= ldd.id_product_max)
		{
			if (!dev->open_device())
			{
				sys_usbd.error("Failed to open USB device(VID=0x%04x, PID=0x%04x) for LDD <%s>", dev->device._device.idVendor, dev->device._device.idProduct, name);
				disconnect_usb_device(dev);
				return;
			}

			dev->read_descriptors();
			dev->assigned_number = dev_counter++; // assign current dev_counter, and atomically increment0
			handled_devices.emplace(dev->assigned_number, std::pair(UsbInternalDevice{0x00, narrow<u8>(dev->assigned_number), 0x02, 0x40}, dev));
			send_message(SYS_USBD_ATTACH, dev->assigned_number);
			sys_usbd.success("USB device(VID=0x%04x, PID=0x%04x) matches up with LDD <%s>, assigned as handled_device=0x%x", dev->device._device.idVendor, dev->device._device.idProduct, name, dev->assigned_number);
			return;
		}
	}
}

void usb_handler_thread::disconnect_usb_device(std::shared_ptr<usb_device> dev, bool update_usb_devices)
{
	if (dev->assigned_number && handled_devices.erase(dev->assigned_number))
	{
		send_message(SYS_USBD_DETACH, dev->assigned_number);
		sys_usbd.success("USB device(VID=0x%04x, PID=0x%04x) unassigned, handled_device=0x%x", dev->device._device.idVendor, dev->device._device.idProduct, dev->assigned_number);
		dev->assigned_number = 0;
		std::erase_if(open_pipes, [&](const auto& val)
		{
			return val.second.device == dev;
		});
	}
	if (update_usb_devices)
	{
		std::erase_if(usb_devices, [&](const auto& val)
		{
			return val == dev;
		});
	}
}

void usb_handler_thread::reconnect_usb_device(u32 assigned_number)
{
	std::lock_guard lock(mutex);
	ensure(assigned_number != 0);
	for (const auto& dev : usb_devices)
	{
		if (dev->assigned_number == assigned_number)
		{
			disconnect_usb_device(dev, false);
			connect_usb_device(dev, false);
			break;
		}
	}
}

void usb_handler_thread::set_usb_device_attached(usb_device* raw_dev, bool attached)
{
	std::lock_guard lock(mutex);
	for (const auto& dev : usb_devices)
	{
		if (dev.get() != raw_dev)
			continue;

		if (attached && !dev->assigned_number)
			connect_usb_device(dev, false);
		else if (!attached && dev->assigned_number)
			disconnect_usb_device(dev, false);

		break;
	}
}

void usb_handler_thread::print_dev_list()
{
	for (u32 index = 1; index < ::size32(handled_devices); index++)
	{
		if (handled_devices[index].second)
		{
			sys_usbd.error("print_dev_list : handled_devices : UsbInternalDevice=%d/%d, VID=0x%04x, PID=0x%04x",
				handled_devices[index].first.device_high, handled_devices[index].first.device_low,
				handled_devices[index].second->device._device.idVendor,
				handled_devices[index].second->device._device.idProduct);
		}
		else
		{
			sys_usbd.error("print_dev_list : handled_devices : UsbInternalDevice=%d/%d, nullptr",
				handled_devices[index].first.device_high, handled_devices[index].first.device_low);
		}
	}

	for (const auto& dev : usb_devices)
	{
		sys_usbd.error("print_dev_list : usb_devices : nr=%d VID=0x%04x, PID=0x%04x",
			dev->assigned_number, dev->device._device.idVendor, dev->device._device.idProduct);
	}
}

void connect_usb_controller(u8 index, input::product_type type)
{
	auto usbh = g_fxo->try_get<named_thread<usb_handler_thread>>();
	if (!usbh)
	{
		return;
	}

	bool already_connected = false;

	if (const auto it = usbh->pad_to_usb.find(index); it != usbh->pad_to_usb.end())
	{
		if (it->second.first == type)
		{
			already_connected = true;
		}
		else
		{
			usbh->disconnect_usb_device(it->second.second, true);
			usbh->pad_to_usb.erase(it->first);
		}
	}

	if (!already_connected)
	{
		switch (type)
		{
		case input::product_type::guncon_3:
		{
			if (!g_cfg_guncon3.load())
			{
				sys_usbd.notice("Could not load GunCon3 config. Using defaults.");
			}

			sys_usbd.success("Adding emulated GunCon3 (controller %d)", index);
			std::shared_ptr<usb_device> dev = std::make_shared<usb_device_guncon3>(index, usbh->get_new_location());
			usbh->connect_usb_device(dev, true);
			usbh->pad_to_usb.emplace(index, std::pair(type, dev));

			sys_usbd.notice("GunCon3 config=\n", g_cfg_guncon3.to_string());
			break;
		}
		case input::product_type::top_shot_elite:
		{
			if (!g_cfg_topshotelite.load())
			{
				sys_usbd.notice("Could not load Top Shot Elite config. Using defaults.");
			}

			sys_usbd.success("Adding emulated Top Shot Elite (controller %d)", index);
			std::shared_ptr<usb_device> dev = std::make_shared<usb_device_topshotelite>(index, usbh->get_new_location());
			usbh->connect_usb_device(dev, true);
			usbh->pad_to_usb.emplace(index, std::pair(type, dev));

			sys_usbd.notice("Top shot elite config=\n", g_cfg_topshotelite.to_string());
			break;
		}
		case input::product_type::top_shot_fearmaster:
		{
			if (!g_cfg_topshotfearmaster.load())
			{
				sys_usbd.notice("Could not load Top Shot Fearmaster config. Using defaults.");
			}

			sys_usbd.success("Adding emulated Top Shot Fearmaster (controller %d)", index);
			std::shared_ptr<usb_device> dev = std::make_shared<usb_device_topshotfearmaster>(index, usbh->get_new_location());
			usbh->connect_usb_device(dev, true);
			usbh->pad_to_usb.emplace(index, std::pair(type, dev));

			sys_usbd.notice("Top shot fearmaster config=\n", g_cfg_topshotfearmaster.to_string());
			break;
		}
		case input::product_type::udraw_gametablet:
		{
			sys_usbd.success("Adding emulated uDraw GameTablet (controller %d)", index);
			std::shared_ptr<usb_device> dev = std::make_shared<usb_device_gametablet>(index, usbh->get_new_location());
			usbh->connect_usb_device(dev, true);
			usbh->pad_to_usb.emplace(index, std::pair(type, dev));
			break;
		}
		default:
			break;
		}
	}
}

void reconnect_usb(u32 assigned_number)
{
	auto usbh = g_fxo->try_get<named_thread<usb_handler_thread>>();
	if (!usbh)
	{
		return;
	}
	usbh->reconnect_usb_device(assigned_number);
}

void set_usb_device_attached(usb_device* dev, bool attached)
{
	auto usbh = g_fxo->try_get<named_thread<usb_handler_thread>>();
	if (!usbh)
	{
		return;
	}
	usbh->set_usb_device_attached(dev, attached);
}

void handle_hotplug_event(bool connected, bool source_is_libusb)
{
	if (auto usbh = g_fxo->try_get<named_thread<usb_handler_thread>>())
	{
		if (usbh->hotplug_supported && !source_is_libusb) return;

		sys_usbd.notice("handle_hotplug_event: connected=%d", connected);

		usbh->usb_hotplug_timeout = get_system_time() + (connected ? 1'000'000ull : 0);
	}
}

error_code sys_usbd_initialize(ppu_thread& ppu, vm::ptr<u32> handle)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.warning("sys_usbd_initialize(handle=*0x%x)", handle);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	{
		std::lock_guard lock(usbh.mutex);

		// Must not occur (lv2 allows multiple handles, cellUsbd does not)
		ensure(!usbh.is_init.exchange(true));
	}

	g_fxo->init<named_thread<pspcm_manager>>();

	ppu.check_state();
	*handle = 0x115B;

	// TODO
	return CELL_OK;
}

error_code sys_usbd_finalize(ppu_thread& ppu, u32 handle)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.warning("sys_usbd_finalize(handle=0x%x)", handle);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);
	usbh.is_init = false;

	// Forcefully awake all waiters
	while (auto cpu = lv2_obj::schedule<ppu_thread>(usbh.sq, SYS_SYNC_FIFO))
	{
		// Special termination signal value
		cpu->gpr[4] = 4;
		cpu->gpr[5] = 0;
		cpu->gpr[6] = 0;
		lv2_obj::awake(cpu);
	}

	// TODO
	return CELL_OK;
}

error_code sys_usbd_get_device_list(ppu_thread& ppu, u32 handle, vm::ptr<UsbInternalDevice> device_list, u32 max_devices)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.warning("sys_usbd_get_device_list(handle=0x%x, device_list=*0x%x, max_devices=0x%x)", handle, device_list, max_devices);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);
	if (!usbh.is_init)
		return CELL_EINVAL;

	// TODO: was std::min<s32>
	const u32 i_tocopy = std::min<u32>(max_devices, ::size32(usbh.handled_devices));
	u32 index = 0;

	for (const auto& [_, device] : usbh.handled_devices)
	{
		if (index == i_tocopy)
			break;
		
		device_list[index++] = device.first;
	}

	return not_an_error(i_tocopy);
}

error_code sys_usbd_register_extra_ldd(ppu_thread& ppu, u32 handle, vm::cptr<char> s_product, u16 slen_product, u16 id_vendor, u16 id_product_min, u16 id_product_max)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_register_extra_ldd(handle=0x%x, s_product=%s, slen_product=%d, id_vendor=0x%04x, id_product_min=0x%04x, id_product_max=0x%04x)", handle, s_product, slen_product, id_vendor, id_product_min, id_product_max);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);
	if (!usbh.is_init)
		return CELL_EINVAL;

	std::string_view product{s_product.get_ptr(), slen_product};

	if (usbh.add_ldd(product, id_vendor, id_product_min, id_product_max))
		return CELL_OK;

	return CELL_EEXIST;
}

error_code sys_usbd_unregister_extra_ldd(ppu_thread& ppu, u32 handle, vm::cptr<char> s_product, u16 slen_product)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_unregister_extra_ldd(handle=0x%x, s_product=%s, slen_product=%d)", handle, s_product, slen_product);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);
	if (!usbh.is_init)
		return CELL_EINVAL;

	std::string_view product{s_product.get_ptr(), slen_product};

	if (usbh.remove_ldd(product))
		return CELL_OK;

	return CELL_ESRCH;
}

error_code sys_usbd_get_descriptor_size(ppu_thread& ppu, u32 handle, u32 device_handle)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_get_descriptor_size(handle=0x%x, deviceNumber=0x%x)", handle, device_handle);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.handled_devices.count(device_handle))
	{
		return CELL_EINVAL;
	}

	return not_an_error(usbh.handled_devices[device_handle].second->device.get_size());
}

error_code sys_usbd_get_descriptor(ppu_thread& ppu, u32 handle, u32 device_handle, vm::ptr<void> descriptor, u32 desc_size)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_get_descriptor(handle=0x%x, deviceNumber=0x%x, descriptor=0x%x, desc_size=0x%x)", handle, device_handle, descriptor, desc_size);

	if (!descriptor)
	{
		return CELL_EINVAL;
	}

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.handled_devices.count(device_handle))
	{
		return CELL_EINVAL;
	}

	if (!desc_size)
	{
		return CELL_ENOMEM;
	}

	usbh.handled_devices[device_handle].second->device.write_data(reinterpret_cast<u8*>(descriptor.get_ptr()), desc_size);

	return CELL_OK;
}

// This function is used for psp(cellUsbPspcm), ps3 arcade usj io(PS3A-USJ), ps2 cam(eyetoy), generic usb camera?(sample_usb2cam)
error_code sys_usbd_register_ldd(ppu_thread& ppu, u32 handle, vm::cptr<char> s_product, u16 slen_product)
{
	ppu.state += cpu_flag::wait;

	std::string_view product{s_product.get_ptr(), slen_product};

	// slightly hacky way of getting Namco GCon3 gun to work.
	// The register_ldd appears to be a more promiscuous mode function, where all device 'inserts' would be presented to the cellUsbd for Probing.
	// Unsure how many more devices might need similar treatment (i.e. just a compare and force VID/PID add), or if it's worth adding a full promiscuous capability
	static const std::unordered_map<std::string, UsbLdd, fmt::string_hash, std::equal_to<>> predefined_ldds
	{
		{"cellUsbPspcm", {0x054C, 0x01CB, 0x01CB}},
		{"guncon3", {0x0B9A, 0x0800, 0x0800}},
		{"PS3A-USJ", {0x0B9A, 0x0900, 0x0910}}
	};

	if (const auto iterator = predefined_ldds.find(product); iterator != predefined_ldds.end())
	{
		sys_usbd.trace("sys_usbd_register_ldd(handle=0x%x, s_product=%s, slen_product=%d) -> Redirecting to sys_usbd_register_extra_ldd()", handle, s_product, slen_product);
		return sys_usbd_register_extra_ldd(ppu, handle, s_product, slen_product, iterator->second.id_vendor, iterator->second.id_product_min, iterator->second.id_product_max);
	}

	sys_usbd.todo("sys_usbd_register_ldd(handle=0x%x, s_product=%s, slen_product=%d)", handle, s_product, slen_product);
	return CELL_OK;
}

error_code sys_usbd_unregister_ldd(ppu_thread& ppu, u32 handle, vm::cptr<char> s_product, u16 slen_product)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_unregister_ldd(handle=0x%x, s_product=%s, slen_product=%d) -> Redirecting to sys_usbd_unregister_extra_ldd()", handle, s_product, slen_product);

	return sys_usbd_unregister_extra_ldd(ppu, handle, s_product, slen_product);
}

// TODO: determine what the unknown params are
// attributes (bmAttributes) : 2=Bulk, 3=Interrupt
error_code sys_usbd_open_pipe(ppu_thread& ppu, u32 handle, u32 device_handle, u32 unk1, u64 unk2, u64 unk3, u32 endpoint, u64 attributes)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.warning("sys_usbd_open_pipe(handle=0x%x, device_handle=0x%x, unk1=0x%x, unk2=0x%x, unk3=0x%x, endpoint=0x%x, attributes=0x%x)", handle, device_handle, unk1, unk2, unk3, endpoint, attributes);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.handled_devices.count(device_handle))
	{
		return CELL_EINVAL;
	}

	return not_an_error(usbh.open_pipe(device_handle, static_cast<u8>(endpoint)));
}

error_code sys_usbd_open_default_pipe(ppu_thread& ppu, u32 handle, u32 device_handle)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_open_default_pipe(handle=0x%x, device_handle=0x%x)", handle, device_handle);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.handled_devices.count(device_handle))
	{
		return CELL_EINVAL;
	}

	return not_an_error(usbh.open_pipe(device_handle, 0));
}

error_code sys_usbd_close_pipe(ppu_thread& ppu, u32 handle, u32 pipe_handle)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.todo("sys_usbd_close_pipe(handle=0x%x, pipe_handle=0x%x)", handle, pipe_handle);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.is_pipe(pipe_handle))
	{
		return CELL_EINVAL;
	}

	usbh.close_pipe(pipe_handle);

	return CELL_OK;
}

// From RE:
// In libusbd_callback_thread
// *arg1 = 4 will terminate CellUsbd libusbd_callback_thread
// *arg1 = 3 will do some extra processing right away(notification of transfer finishing)
// *arg1 < 1 || *arg1 > 4 are ignored(rewait instantly for event)
// *arg1 == 1 || *arg1 == 2 will send a sys_event to internal CellUsbd event queue with same parameters as received and loop(attach and detach event)
error_code sys_usbd_receive_event(ppu_thread& ppu, u32 handle, vm::ptr<u64> arg1, vm::ptr<u64> arg2, vm::ptr<u64> arg3)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_receive_event(handle=0x%x, arg1=*0x%x, arg2=*0x%x, arg3=*0x%x)", handle, arg1, arg2, arg3);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	{
		std::lock_guard lock_sq(usbh.mutex_sq);

		if (!usbh.is_init)
			return CELL_EINVAL;

		if (usbh.get_event(arg1, arg2, arg3))
		{
			// hack for Guitar Hero Live
			// Attaching the device too fast seems to result in a nullptr along the way
			if (*arg1 == SYS_USBD_ATTACH)
				lv2_obj::sleep(ppu), lv2_obj::wait_timeout(5000);

			return CELL_OK;
		}

		lv2_obj::sleep(ppu);
		lv2_obj::emplace(usbh.sq, &ppu);
	}

	while (auto state = +ppu.state)
	{
		if (state & cpu_flag::signal && ppu.state.test_and_reset(cpu_flag::signal))
		{
			sys_usbd.trace("Received event(queued): arg1=0x%x arg2=0x%x arg3=0x%x", ppu.gpr[4], ppu.gpr[5], ppu.gpr[6]);
			break;
		}

		if (is_stopped(state))
		{
			std::lock_guard lock(usbh.mutex_sq);

			for (auto cpu = +usbh.sq; cpu; cpu = cpu->next_cpu)
			{
				if (cpu == &ppu)
				{
					ppu.state += cpu_flag::again;
					sys_usbd.trace("sys_usbd_receive_event: aborting");
					return {};
				}
			}

			break;
		}

		ppu.state.wait(state);
	}

	ppu.check_state();
	*arg1 = ppu.gpr[4];
	*arg2 = ppu.gpr[5];
	*arg3 = ppu.gpr[6];

	if (*arg1 == SYS_USBD_ATTACH)
		lv2_obj::sleep(ppu), lv2_obj::wait_timeout(5000);

	return CELL_OK;
}

error_code sys_usbd_detect_event(ppu_thread& ppu)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.todo("sys_usbd_detect_event()");
	return CELL_OK;
}

error_code sys_usbd_attach(ppu_thread& ppu, u32 handle, u32 unk1, u32 unk2, u32 device_handle)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.todo("sys_usbd_attach(handle=0x%x, unk1=0x%x, unk2=0x%x, device_handle=0x%x)", handle, unk1, unk2, device_handle);
	return CELL_OK;
}

error_code sys_usbd_transfer_data(ppu_thread& ppu, u32 handle, u32 id_pipe, vm::ptr<u8> buf, u32 buf_size, vm::ptr<UsbDeviceRequest> request, u32 type_transfer)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_transfer_data(handle=0x%x, id_pipe=0x%x, buf=*0x%x, buf_length=0x%x, request=*0x%x, type=0x%x)", handle, id_pipe, buf, buf_size, request, type_transfer);

	if (sys_usbd.trace && request)
	{
		sys_usbd.trace("RequestType:0x%02x, Request:0x%02x, wValue:0x%04x, wIndex:0x%04x, wLength:0x%04x", request->bmRequestType, request->bRequest, request->wValue, request->wIndex, request->wLength);

		if ((request->bmRequestType & 0x80) == 0 && buf && buf_size != 0)
			sys_usbd.trace("Control sent:\n%s", fmt::buf_to_hexstring(buf.get_ptr(), buf_size));
	}

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.is_pipe(id_pipe))
	{
		return CELL_EINVAL;
	}

	const auto& pipe               = usbh.get_pipe(id_pipe);
	auto&& [transfer_id, transfer] = usbh.get_free_transfer();

	transfer.assigned_number = pipe.device->assigned_number;

	// Default endpoint is control endpoint
	if (pipe.endpoint == 0)
	{
		if (!request)
		{
			sys_usbd.error("Tried to use control pipe without proper request pointer");
			return CELL_EINVAL;
		}

		// Claiming interface
		switch (request->bmRequestType)
		{
		case 0U /*silences warning*/ | LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE:
		{
			switch (request->bRequest)
			{
			case LIBUSB_REQUEST_SET_CONFIGURATION:
			{
				pipe.device->set_configuration(static_cast<u8>(+request->wValue));
				pipe.device->set_interface(0, 0);
				break;
			}
			default: break;
			}
			break;
		}
		case 0U /*silences warning*/ | LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE:
		{
			if (!buf)
			{
				sys_usbd.error("Invalid buffer for control_transfer");
				return CELL_EFAULT;
			}
			break;
		}
		default: break;
		}

		pipe.device->control_transfer(request->bmRequestType, request->bRequest, request->wValue, request->wIndex, request->wLength, buf_size, buf.get_ptr(), &transfer);
	}
	else
	{
		// If output endpoint
		if (!(pipe.endpoint & 0x80))
			sys_usbd.trace("Write Int(s: %d):\n%s", buf_size, fmt::buf_to_hexstring(buf.get_ptr(), buf_size));

		pipe.device->interrupt_transfer(buf_size, buf.get_ptr(), pipe.endpoint, &transfer);
	}

	if (transfer.fake)
	{
		usbh.push_fake_transfer(&transfer);
	}

	// returns an identifier specific to the transfer
	return not_an_error(transfer_id);
}

error_code sys_usbd_isochronous_transfer_data(ppu_thread& ppu, u32 handle, u32 id_pipe, vm::ptr<UsbDeviceIsoRequest> iso_request)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_isochronous_transfer_data(handle=0x%x, id_pipe=0x%x, iso_request=*0x%x)", handle, id_pipe, iso_request);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.is_pipe(id_pipe))
	{
		return CELL_EINVAL;
	}

	const auto& pipe               = usbh.get_pipe(id_pipe);
	auto&& [transfer_id, transfer] = usbh.get_free_transfer();

	transfer.iso_request.buf = iso_request->buf;
	transfer.iso_request.start_frame = iso_request->start_frame;
	transfer.iso_request.num_packets = iso_request->num_packets;
	for (u32 index = 0; index < iso_request->num_packets; index++)
	{
		::at32(transfer.iso_request.packets, index) = ::at32(iso_request->packets, index);
	}

	pipe.device->isochronous_transfer(&transfer);

	if (transfer.fake)
	{
		usbh.push_fake_transfer(&transfer);
	}

	// returns an identifier specific to the transfer
	return not_an_error(transfer_id);
}

error_code sys_usbd_get_transfer_status(ppu_thread& ppu, u32 handle, u32 id_transfer, u32 unk1, vm::ptr<u32> result, vm::ptr<u32> count)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_get_transfer_status(handle=0x%x, id_transfer=0x%x, unk1=0x%x, result=*0x%x, count=*0x%x)", handle, id_transfer, unk1, result, count);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || id_transfer >= MAX_SYS_USBD_TRANSFERS)
		return CELL_EINVAL;

	const auto status = usbh.get_transfer_status(id_transfer);
	*result           = status.first;
	*count            = status.second;

	return CELL_OK;
}

error_code sys_usbd_get_isochronous_transfer_status(ppu_thread& ppu, u32 handle, u32 id_transfer, u32 unk1, vm::ptr<UsbDeviceIsoRequest> request, vm::ptr<u32> result)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.trace("sys_usbd_get_isochronous_transfer_status(handle=0x%x, id_transfer=0x%x, unk1=0x%x, request=*0x%x, result=*0x%x)", handle, id_transfer, unk1, request, result);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || id_transfer >= MAX_SYS_USBD_TRANSFERS)
		return CELL_EINVAL;

	const auto status = usbh.get_isochronous_transfer_status(id_transfer);

	*result  = status.first;
	*request = status.second;

	return CELL_OK;
}

error_code sys_usbd_get_device_location(ppu_thread& ppu, u32 handle, u32 device_handle, vm::ptr<u8> location)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.notice("sys_usbd_get_device_location(handle=0x%x, device_handle=0x%x, location=*0x%x)", handle, device_handle, location);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init || !usbh.handled_devices.count(device_handle))
		return CELL_EINVAL;

	usbh.handled_devices[device_handle].second->get_location(location.get_ptr());

	return CELL_OK;
}

error_code sys_usbd_send_event(ppu_thread& ppu)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.todo("sys_usbd_send_event()");
	return CELL_OK;
}

error_code sys_usbd_event_port_send(ppu_thread& ppu, u32 handle, u64 arg1, u64 arg2, u64 arg3)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.warning("sys_usbd_event_port_send(handle=0x%x, arg1=0x%x, arg2=0x%x, arg3=0x%x)", handle, arg1, arg2, arg3);

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();

	std::lock_guard lock(usbh.mutex);

	if (!usbh.is_init)
		return CELL_EINVAL;

	usbh.add_event(arg1, arg2, arg3);

	return CELL_OK;
}

error_code sys_usbd_allocate_memory(ppu_thread& ppu)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.todo("sys_usbd_allocate_memory()");
	return CELL_OK;
}

error_code sys_usbd_free_memory(ppu_thread& ppu)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.todo("sys_usbd_free_memory()");
	return CELL_OK;
}

error_code sys_usbd_get_device_speed(ppu_thread& ppu)
{
	ppu.state += cpu_flag::wait;

	sys_usbd.todo("sys_usbd_get_device_speed()");
	return CELL_OK;
}


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

pspcm_manager::pspcm_manager()
{
	sys_usbd.notice("pspcm_manager::pspcm_manager()");
#ifdef _WIN32
	WSADATA wsa_data;
	WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
}

pspcm_manager::~pspcm_manager()
{
	sys_usbd.notice("pspcm_manager::~pspcm_manager()");
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
		sys_usbd.error("pspcm_manager: getaddrinfo error");
		return -1;
	}

	const pspcm_socket_t s = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
	if (s == PSPCM_INVALID_SOCKET)
	{
		sys_usbd.error("pspcm_manager: Error creating socket");
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

	sys_usbd.notice("pspcm_manager: PSP disconnected");
	m_device_connected = false;
	close_server();

	auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();
	std::lock_guard lock(usbh.mutex);
	usbh.disconnect_usb_device(m_psp_dev);
	m_psp_dev.reset();
}

// The TCP stream has no packet boundaries: takes the first complete packet, if there is one.
bool pspcm_manager::take_packet(pspcm_data& pk)
{
	while (m_stream.size() >= PSPCM_HEADER_LEN)
	{
		if (m_stream[0] != 0xf0 || m_stream[1] != 0x0f || m_stream[2] < PSPCM_HEADER_LEN)
		{
			sys_usbd.warning("pspcm_manager: lost sync, skipping a byte (%02x)", m_stream[0]);
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
			sys_usbd.warning("pspcm_manager: packet for unknown endpoint %02x dropped", pk.endpoint);
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
		sys_usbd.error("pspcm_manager: the configuration is %d bytes, only %d fit a packet", total, desc.config.size());
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
				sys_usbd.notice("pspcm_manager: connected to PPSSPP");
				// What the device is comes from the PSP, the built-in descriptors are the fallback.
				pspcm_descriptors desc;
				const bool have_desc = fetch_descriptors(desc);
				if (!have_desc)
				{
					sys_usbd.warning("pspcm_manager: no descriptors from the PSP, using the built-in ones");
				}
				auto& usbh = g_fxo->get<named_thread<usb_handler_thread>>();
				std::lock_guard lock(usbh.mutex);
				m_psp_dev = std::make_shared<usb_device_psp>(usbh.get_new_location(), have_desc ? &desc : nullptr);
				usbh.connect_usb_device(m_psp_dev);
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
