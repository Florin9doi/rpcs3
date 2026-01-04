#pragma once

#include "emulated_pad_config.h"

enum class bodytracker_btn
{
	dpad_up,
	dpad_down,
	dpad_left,
	dpad_right,
	motion_x,
	motion_y,
	motion_z,
	motion_g,

	count
};

struct cfg_tbt final : public emulated_pad_config<bodytracker_btn>
{
	cfg_tbt(node* owner, const std::string& name) : emulated_pad_config(owner, name) {}

	cfg_pad_btn<bodytracker_btn> dpad_up{this, "D-Pad Up", bodytracker_btn::dpad_up, pad_button::dpad_up};
	cfg_pad_btn<bodytracker_btn> dpad_down{this, "D-Pad Down", bodytracker_btn::dpad_down, pad_button::dpad_down};
	cfg_pad_btn<bodytracker_btn> dpad_left{this, "D-Pad Left", bodytracker_btn::dpad_left, pad_button::dpad_left};
	cfg_pad_btn<bodytracker_btn> dpad_right{this, "D-Pad Right", bodytracker_btn::dpad_right, pad_button::dpad_right};
	cfg_pad_btn<bodytracker_btn> motion_x{this, "Motion X", bodytracker_btn::motion_x, pad_button::motion_x};
	cfg_pad_btn<bodytracker_btn> motion_y{this, "Motion Y", bodytracker_btn::motion_y, pad_button::motion_y};
	cfg_pad_btn<bodytracker_btn> motion_z{this, "Motion Z", bodytracker_btn::motion_z, pad_button::motion_z};
	cfg_pad_btn<bodytracker_btn> motion_g{this, "Motion G", bodytracker_btn::motion_g, pad_button::motion_g};
};

struct cfg_bodytracker final : public emulated_pads_config<cfg_tbt, 4>
{
	cfg_bodytracker() : emulated_pads_config<cfg_tbt, 4>("bodytracker") {};
};

extern cfg_bodytracker g_cfg_bodytracker;
