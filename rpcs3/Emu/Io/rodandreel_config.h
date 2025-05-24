#pragma once

#include "emulated_pad_config.h"

enum class rodandreel_btn
{
	square,
	cross,
	circle,
	triangle,
	reel_in,
	trigger,
	l2,
	r2,
	select,
	start,
	l3,
	ps,
	dpad_up,
	dpad_down,
	dpad_left,
	dpad_right,
	stick_x,
	stick_y,
	motion_x,
	motion_y,
	motion_z,
	motion_g,

	count
};

struct cfg_rnr final : public emulated_pad_config<rodandreel_btn>
{
	cfg_rnr(node* owner, const std::string& name) : emulated_pad_config(owner, name) {}
	
	cfg_pad_btn<rodandreel_btn> square{this, "Square", rodandreel_btn::square, pad_button::square};
	cfg_pad_btn<rodandreel_btn> cross{this, "Cross", rodandreel_btn::cross, pad_button::cross};
	cfg_pad_btn<rodandreel_btn> circle{this, "Circle", rodandreel_btn::circle, pad_button::circle};
	cfg_pad_btn<rodandreel_btn> triangle{this, "Triangle", rodandreel_btn::triangle, pad_button::triangle};
	cfg_pad_btn<rodandreel_btn> reel_up{this, "Reel Up", rodandreel_btn::reel_in, pad_button::L2};
	cfg_pad_btn<rodandreel_btn> trigger{this, "Trigger", rodandreel_btn::trigger, pad_button::R2};
	cfg_pad_btn<rodandreel_btn> l2{this, "L2", rodandreel_btn::l2, pad_button::L1};
	cfg_pad_btn<rodandreel_btn> r2{this, "R2", rodandreel_btn::r2, pad_button::R1};
	cfg_pad_btn<rodandreel_btn> select{this, "Select", rodandreel_btn::select, pad_button::select};
	cfg_pad_btn<rodandreel_btn> start{this, "Start", rodandreel_btn::start, pad_button::start};
	cfg_pad_btn<rodandreel_btn> l3{this, "L3", rodandreel_btn::l3, pad_button::L3};
	cfg_pad_btn<rodandreel_btn> ps{this, "PS", rodandreel_btn::ps, pad_button::ps};
	cfg_pad_btn<rodandreel_btn> dpad_up{this, "D-Pad Up", rodandreel_btn::dpad_up, pad_button::dpad_up};
	cfg_pad_btn<rodandreel_btn> dpad_down{this, "D-Pad Down", rodandreel_btn::dpad_down, pad_button::dpad_down};
	cfg_pad_btn<rodandreel_btn> dpad_left{this, "D-Pad Left", rodandreel_btn::dpad_left, pad_button::dpad_left};
	cfg_pad_btn<rodandreel_btn> dpad_right{this, "D-Pad Right", rodandreel_btn::dpad_right, pad_button::dpad_right};
	cfg_pad_btn<rodandreel_btn> stick_x{this, "Stick X-Axis", rodandreel_btn::stick_x, pad_button::ls_x};
	cfg_pad_btn<rodandreel_btn> stick_y{this, "Stick Y-Axis", rodandreel_btn::stick_y, pad_button::ls_y};
	cfg_pad_btn<rodandreel_btn> motion_x{this, "Motion X", rodandreel_btn::motion_x, pad_button::motion_x};
	cfg_pad_btn<rodandreel_btn> motion_y{this, "Motion Y", rodandreel_btn::motion_y, pad_button::motion_y};
	cfg_pad_btn<rodandreel_btn> motion_z{this, "Motion Z", rodandreel_btn::motion_z, pad_button::motion_z};
	cfg_pad_btn<rodandreel_btn> motion_g{this, "Motion G", rodandreel_btn::motion_g, pad_button::motion_g};
};

struct cfg_rodandreel final : public emulated_pads_config<cfg_rnr, 4>
{
	cfg_rodandreel() : emulated_pads_config<cfg_rnr, 4>("rodandreel") {};
};

extern cfg_rodandreel g_cfg_rodandreel;
