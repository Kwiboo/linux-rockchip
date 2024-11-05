// SPDX-License-Identifier: GPL-2.0
/*
 * sc535hai driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 * V0.0X01.0X02 add support thunder boot
 * V0.0X01.0X03 add support sleep wake-up mode
 * V0.0X01.0X04 add support hw standby for aov
 * V0.0X01.0X05 add support 2/4lane linear & hdr2 settings
 * V0.0X01.0X05 fix exposure time error
 */

// #define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <linux/pinctrl/consumer.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"
#include "cam-tb-setup.h"
#include "cam-sleep-wakeup.h"
#include "light_ctl.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x05)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SC535HAI_BITS_PER_SAMPLE	10
#define SC535HAI_LINK_FREQ_450		450000000

/* 2 lane */
#define PIXEL_RATE_WITH_450M_10BIT_2L	(SC535HAI_LINK_FREQ_450 * 2 * \
					2 / SC535HAI_BITS_PER_SAMPLE)
/* 4 lane */
#define PIXEL_RATE_WITH_450M_10BIT_4L	(SC535HAI_LINK_FREQ_450 * 2 / \
					SC535HAI_BITS_PER_SAMPLE * 4)

#define SC535HAI_XVCLK_FREQ		27000000

#define CHIP_ID				0xce78
#define SC535HAI_REG_CHIP_ID		0x3107
#define SC535HAI_REG_CHIP_FLG		0x8037

#define SC535HAI_REG_MIPI_CTRL		0x3019
#define SC535HAI_MIPI_CTRL_ON		0x00
#define SC535HAI_MIPI_CTRL_OFF		0xff

#define SC535HAI_REG_CTRL_MODE		0x0100
#define SC535HAI_MODE_SW_STANDBY	0x0
#define SC535HAI_MODE_STREAMING		BIT(0)

#define SC535HAI_REG_EXPOSURE_H		0x3e00
#define SC535HAI_REG_EXPOSURE_M		0x3e01
#define SC535HAI_REG_EXPOSURE_L		0x3e02
#define SC535HAI_REG_SEXPOSURE_H	0x3e22
#define SC535HAI_REG_SEXPOSURE_M	0x3e04
#define SC535HAI_REG_SEXPOSURE_L	0x3e05

#define	SC535HAI_EXPOSURE_MIN		1
#define	SC535HAI_EXPOSURE_STEP		1
#define SC535HAI_VTS_MAX		0x7fff

#define SC535HAI_REG_DIG_GAIN		0x3e06
#define SC535HAI_REG_DIG_FINE_GAIN	0x3e07
#define SC535HAI_REG_ANA_GAIN		0x3e08
#define SC535HAI_REG_ANA_FINE_GAIN	0x3e09
#define SC535HAI_REG_SDIG_GAIN		0x3e10
#define SC535HAI_REG_SDIG_FINE_GAIN	0x3e11
#define SC535HAI_REG_SANA_GAIN		0x3e12
#define SC535HAI_REG_SANA_FINE_GAIN	0x3e13
#define SC535HAI_REG_MAX_SEXPOSURE_H	0x3e23
#define SC535HAI_REG_MAX_SEXPOSURE_L	0x3e24
#define SC535HAI_GAIN_MIN		0x20
#define SC535HAI_GAIN_MAX		40803 //79.695*16*32	(99614)	//48.64*16*128
#define SC535HAI_GAIN_STEP		1
#define SC535HAI_GAIN_DEFAULT		0x20 //0x80 // Note that the benchmark is 0x20
#define SC535HAI_LGAIN			0
#define SC535HAI_SGAIN			1

#define SC535HAI_REG_GROUP_HOLD		0x3800//0x3812
#define SC535HAI_GROUP_HOLD_START	0x00
#define SC535HAI_GROUP_HOLD_END		0x30 // Not used

#define SC535HAI_REG_TEST_PATTERN	0x4501
#define SC535HAI_TEST_PATTERN_BIT_MASK	BIT(3)

#define SC535HAI_REG_VTS_H		0x320e
#define SC535HAI_REG_VTS_L		0x320f

#define SC535HAI_FLIP_MIRROR_REG	0x3221

#define SC535HAI_FETCH_EXP_H(VAL)	(((VAL) >> 12) & 0xF)
#define SC535HAI_FETCH_EXP_M(VAL)	(((VAL) >> 4) & 0xFF)
#define SC535HAI_FETCH_EXP_L(VAL)	(((VAL) & 0xF) << 4)

#define SC535HAI_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x06 : VAL & 0xf9)
#define SC535HAI_FETCH_FLIP(VAL, ENABLE)	(ENABLE ? VAL | 0x60 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define SC535HAI_REG_VALUE_08BIT	1
#define SC535HAI_REG_VALUE_16BIT	2
#define SC535HAI_REG_VALUE_24BIT	3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define SC535HAI_NAME			"sc535hai"

static const char *const sc535hai_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC535HAI_NUM_SUPPLIES ARRAY_SIZE(sc535hai_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc535hai_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *global_reg_list;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 mclk;
	u32 link_freq_idx;
	u32 vc[PAD_MAX];
	u8 bpp;
	u32 lanes;
};

struct sc535hai {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC535HAI_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	struct v4l2_fract	cur_fps;
	bool			streaming;
	bool			power_on;
	const struct sc535hai_mode *supported_modes;
	const struct sc535hai_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			standby_hw;
	u32			cur_vts;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	bool			is_standby;
	bool			enable_light_ctl;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	struct cam_sw_info	*cam_sw_inf;
	struct v4l2_fwnode_endpoint bus_cfg;
	struct rk_light_param	light_param;
};

#define to_sc535hai(sd) container_of(sd, struct sc535hai, subdev)


static const struct regval sc535hai_global_4lane_regs[] = {
	{REG_NULL, 0x00},
};

static const struct regval sc535hai_global_2lane_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 900Mbps, 2lane
 *
 * avdd=2.800000
 * dovdd=1.800000
 * dvdd=1.2
 * Cleaned_0x06_SC535HAI_raw_MIPI_27Minput_2Lane_10bit_900Mbps_2592x1944_30fps.ini
 */
static const struct regval sc535hai_linear_10_2592x1944_30fps_2lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x23b0, 0x00},
	{0x23b1, 0x08},
	{0x23b2, 0x00},
	{0x23b3, 0x18},
	{0x23b4, 0x00},
	{0x23b5, 0x38},
	{0x23b6, 0x04},
	{0x23b7, 0x08},
	{0x23b8, 0x04},
	{0x23b9, 0x18},
	{0x23ba, 0x04},
	{0x23bb, 0x38},
	{0x23c0, 0x04},
	{0x23c1, 0x00},
	{0x23c2, 0x04},
	{0x23c3, 0x18},
	{0x23c4, 0x04},
	{0x23c5, 0x78},
	{0x23c6, 0x04},
	{0x23c7, 0x08},
	{0x23c8, 0x04},
	{0x23c9, 0x78},
	{0x3018, 0x3b},
	{0x3019, 0x0c},
	{0x301e, 0xf0},
	{0x301f, 0x06},
	{0x302c, 0x00},
	{0x30b8, 0x44},
	{0x3200, 0x00},
	{0x3201, 0x30},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3204, 0x0a},
	{0x3205, 0x57},
	{0x3206, 0x07},
	{0x3207, 0x9f},
	{0x3208, 0x0a},
	{0x3209, 0x20},
	{0x320a, 0x07},
	{0x320b, 0x98},
	{0x320c, 0x05},
	{0x320d, 0xdc},
	{0x320e, 0x07},
	{0x320f, 0xd0},
	{0x3210, 0x00},
	{0x3211, 0x04},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0xc0},
	{0x3250, 0x40},
	{0x327f, 0x3f},
	{0x32e0, 0x00},
	{0x3301, 0x12},
	{0x3302, 0x20},
	{0x3304, 0xc0},
	{0x3306, 0xb0},
	{0x3309, 0xf0},
	{0x330a, 0x01},
	{0x330b, 0x70},
	{0x330d, 0x10},
	{0x3310, 0x18},
	{0x331e, 0xb1},
	{0x331f, 0xe1},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x3364, 0x56},
	{0x338f, 0x80},
	{0x3393, 0x1c},
	{0x3394, 0x2c},
	{0x3395, 0x3c},
	{0x3399, 0x0c},
	{0x339a, 0x10},
	{0x339b, 0x18},
	{0x339c, 0x80},
	{0x33ac, 0x10},
	{0x33ad, 0x2c},
	{0x33ae, 0xb0},
	{0x33af, 0xe0},
	{0x33b0, 0x0f},
	{0x33b2, 0x2c},
	{0x33b3, 0x02},
	{0x349f, 0x03},
	{0x34a8, 0x02},
	{0x34a9, 0x08},
	{0x34aa, 0x01},
	{0x34ab, 0x70},
	{0x34ac, 0x01},
	{0x34ad, 0x70},
	{0x34f9, 0x12},
	{0x3631, 0x0f},
	{0x3632, 0x8d},
	{0x3633, 0x4d},
	{0x363b, 0x58},
	{0x363c, 0xd8},
	{0x363d, 0x20},
	{0x3641, 0x08},
	{0x3670, 0x32},
	{0x3671, 0x34},
	{0x3672, 0x26},
	{0x3673, 0x04},
	{0x3674, 0x08},
	{0x3675, 0x04},
	{0x3676, 0x18},
	{0x367e, 0x49},
	{0x367f, 0x49},
	{0x3680, 0x49},
	{0x3681, 0x04},
	{0x3682, 0x08},
	{0x3683, 0x04},
	{0x3684, 0x38},
	{0x3685, 0xc1},
	{0x3686, 0xc2},
	{0x3687, 0xc1},
	{0x3688, 0xc1},
	{0x3689, 0xc1},
	{0x368a, 0xc1},
	{0x368b, 0xc4},
	{0x368c, 0xc1},
	{0x368d, 0x00},
	{0x368e, 0x08},
	{0x368f, 0x00},
	{0x3690, 0x18},
	{0x3691, 0x04},
	{0x3692, 0x00},
	{0x3693, 0x04},
	{0x3694, 0x08},
	{0x3695, 0x04},
	{0x3696, 0x18},
	{0x3697, 0x04},
	{0x3698, 0x38},
	{0x3699, 0x04},
	{0x369a, 0x78},
	{0x36d0, 0x0d},
	{0x36ea, 0x0a},
	{0x36eb, 0x0c},
	{0x36ec, 0x43},
	{0x36ed, 0xaa},
	{0x370f, 0x13},
	{0x3721, 0x6c},
	{0x3722, 0x8b},
	{0x3724, 0xd1},
	{0x3729, 0x34},
	{0x37b0, 0x17},
	{0x37b1, 0x17},
	{0x37b2, 0x13},
	{0x37b3, 0x04},
	{0x37b4, 0x08},
	{0x37b5, 0x04},
	{0x37b6, 0x38},
	{0x37b7, 0x1d},
	{0x37b8, 0x1f},
	{0x37b9, 0x1f},
	{0x37ba, 0x04},
	{0x37bb, 0x04},
	{0x37bc, 0x04},
	{0x37bd, 0x04},
	{0x37be, 0x08},
	{0x37bf, 0x04},
	{0x37c0, 0x38},
	{0x37c1, 0x04},
	{0x37c2, 0x08},
	{0x37c3, 0x04},
	{0x37c4, 0x38},
	{0x37fa, 0x0a},
	{0x37fb, 0x22},
	{0x37fc, 0x30},
	{0x37fd, 0x16},
	{0x3900, 0x05},
	{0x3901, 0x00},
	{0x3902, 0xc0},
	{0x3903, 0x40},
	{0x3905, 0x2d},
	{0x391a, 0x72},
	{0x391b, 0x39},
	{0x391c, 0x22},
	{0x391d, 0x00},
	{0x391f, 0x41},
	{0x3926, 0xe0},
	{0x3933, 0x80},
	{0x3934, 0x03},
	{0x3935, 0x01},
	{0x3936, 0xc0},
	{0x3937, 0x6a},
	{0x3938, 0x6b},
	{0x3939, 0x0f},
	{0x393a, 0xf6},
	{0x393d, 0x05},
	{0x393e, 0x50},
	{0x39dd, 0x00},
	{0x39de, 0x06},
	{0x39e7, 0x04},
	{0x39e8, 0x04},
	{0x39e9, 0x80},
	{0x3e00, 0x00},
	{0x3e01, 0x7c},
	{0x3e02, 0x80},
	{0x3e03, 0x0b},
	{0x3e16, 0x01},
	{0x3e17, 0x44},
	{0x3e18, 0x01},
	{0x3e19, 0x44},
	{0x440e, 0x02},
	{0x4509, 0x18},
	{0x450d, 0x07},
	{0x480f, 0x03},
	{0x5000, 0x06},
	{0x5780, 0x76},
	{0x5784, 0x10},
	{0x5785, 0x08},
	{0x5787, 0x16},
	{0x5788, 0x16},
	{0x5789, 0x15},
	{0x578a, 0x16},
	{0x578b, 0x16},
	{0x578c, 0x15},
	{0x578d, 0x41},
	{0x5790, 0x11},
	{0x5791, 0x0f},
	{0x5792, 0x0f},
	{0x5793, 0x11},
	{0x5794, 0x0f},
	{0x5795, 0x0f},
	{0x5799, 0x46},
	{0x579a, 0x77},
	{0x57a1, 0x04},
	{0x57a8, 0xd2},
	{0x57aa, 0x2a},
	{0x57ab, 0x7f},
	{0x57ac, 0x00},
	{0x57ad, 0x00},
	{0x36e9, 0x44},
	{0x37f9, 0x04},
	// {0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 15fps
 * mipi_datarate per lane 900Mbps, 2lane
 *
 * avdd=2.800000
 * dovdd=1.800000
 * dvdd=1.2
 * Cleaned_0x0e_SC535HAI_raw_MIPI_24Minput_2Lane_10bit_900Mbps_2592x1944_15fps_SHDR_VC.ini
 */
static const struct regval sc535hai_hdr2_10_2592x1944_15fps_2lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x23b0, 0x00},
	{0x23b1, 0x08},
	{0x23b2, 0x00},
	{0x23b3, 0x18},
	{0x23b4, 0x00},
	{0x23b5, 0x38},
	{0x23b6, 0x04},
	{0x23b7, 0x08},
	{0x23b8, 0x04},
	{0x23b9, 0x18},
	{0x23ba, 0x04},
	{0x23bb, 0x38},
	{0x23c0, 0x04},
	{0x23c1, 0x00},
	{0x23c2, 0x04},
	{0x23c3, 0x18},
	{0x23c4, 0x04},
	{0x23c5, 0x78},
	{0x23c6, 0x04},
	{0x23c7, 0x08},
	{0x23c8, 0x04},
	{0x23c9, 0x78},
	{0x3018, 0x3b},
	{0x3019, 0x0c},
	{0x301e, 0xf0},
	{0x301f, 0x0e},
	{0x302c, 0x00},
	{0x30b8, 0x44},
	{0x3200, 0x00},
	{0x3201, 0x30},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3204, 0x0a},
	{0x3205, 0x57},
	{0x3206, 0x07},
	{0x3207, 0x9f},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x07},
	{0x320b, 0x80},
	{0x320c, 0x05},
	{0x320d, 0xdc},
	{0x320e, 0x0f},
	{0x320f, 0xa0},
	{0x3210, 0x00},
	{0x3211, 0x04},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0xc0},
	{0x3250, 0xff},
	{0x327f, 0x3f},
	{0x3281, 0x01},
	{0x32e0, 0x00},
	{0x3301, 0x12},
	{0x3302, 0x20},
	{0x3304, 0xc0},
	{0x3306, 0xb0},
	{0x3309, 0xf0},
	{0x330a, 0x01},
	{0x330b, 0x70},
	{0x330d, 0x10},
	{0x3310, 0x18},
	{0x331e, 0xb1},
	{0x331f, 0xe1},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x3364, 0x56},
	{0x338f, 0x80},
	{0x3393, 0x1c},
	{0x3394, 0x2c},
	{0x3395, 0x3c},
	{0x3399, 0x0c},
	{0x339a, 0x10},
	{0x339b, 0x18},
	{0x339c, 0x80},
	{0x33ac, 0x10},
	{0x33ad, 0x2c},
	{0x33ae, 0xb0},
	{0x33af, 0xe0},
	{0x33b0, 0x0f},
	{0x33b2, 0x2c},
	{0x33b3, 0x02},
	{0x349f, 0x03},
	{0x34a8, 0x02},
	{0x34a9, 0x08},
	{0x34aa, 0x01},
	{0x34ab, 0x70},
	{0x34ac, 0x01},
	{0x34ad, 0x70},
	{0x34f9, 0x12},
	{0x3631, 0x0f},
	{0x3632, 0x8d},
	{0x3633, 0x4d},
	{0x363b, 0x58},
	{0x363c, 0xd8},
	{0x363d, 0x20},
	{0x3641, 0x08},
	{0x3670, 0x32},
	{0x3671, 0x34},
	{0x3672, 0x26},
	{0x3673, 0x04},
	{0x3674, 0x08},
	{0x3675, 0x04},
	{0x3676, 0x18},
	{0x367e, 0x49},
	{0x367f, 0x49},
	{0x3680, 0x49},
	{0x3681, 0x04},
	{0x3682, 0x08},
	{0x3683, 0x04},
	{0x3684, 0x38},
	{0x3685, 0xc1},
	{0x3686, 0xc2},
	{0x3687, 0xc1},
	{0x3688, 0xc1},
	{0x3689, 0xc1},
	{0x368a, 0xc1},
	{0x368b, 0xc4},
	{0x368c, 0xc1},
	{0x368d, 0x00},
	{0x368e, 0x08},
	{0x368f, 0x00},
	{0x3690, 0x18},
	{0x3691, 0x04},
	{0x3692, 0x00},
	{0x3693, 0x04},
	{0x3694, 0x08},
	{0x3695, 0x04},
	{0x3696, 0x18},
	{0x3697, 0x04},
	{0x3698, 0x38},
	{0x3699, 0x04},
	{0x369a, 0x78},
	{0x36d0, 0x0d},
	{0x36ea, 0x0a},
	{0x36eb, 0x0c},
	{0x36ec, 0x43},
	{0x36ed, 0xaa},
	{0x370f, 0x13},
	{0x3721, 0x6c},
	{0x3722, 0x8b},
	{0x3724, 0xd1},
	{0x3729, 0x34},
	{0x37b0, 0x17},
	{0x37b1, 0x17},
	{0x37b2, 0x13},
	{0x37b3, 0x04},
	{0x37b4, 0x08},
	{0x37b5, 0x04},
	{0x37b6, 0x38},
	{0x37b7, 0x1d},
	{0x37b8, 0x1f},
	{0x37b9, 0x1f},
	{0x37ba, 0x04},
	{0x37bb, 0x04},
	{0x37bc, 0x04},
	{0x37bd, 0x04},
	{0x37be, 0x08},
	{0x37bf, 0x04},
	{0x37c0, 0x38},
	{0x37c1, 0x04},
	{0x37c2, 0x08},
	{0x37c3, 0x04},
	{0x37c4, 0x38},
	{0x37fa, 0x09},
	{0x37fb, 0x22},
	{0x37fc, 0x30},
	{0x37fd, 0x26},
	{0x3900, 0x05},
	{0x3901, 0x00},
	{0x3902, 0xc0},
	{0x3903, 0x40},
	{0x3905, 0x2d},
	{0x391a, 0x72},
	{0x391b, 0x39},
	{0x391c, 0x22},
	{0x391d, 0x00},
	{0x391f, 0x41},
	{0x3926, 0xe0},
	{0x3933, 0x80},
	{0x3934, 0x03},
	{0x3935, 0x01},
	{0x3936, 0xc0},
	{0x3937, 0x6a},
	{0x3938, 0x6b},
	{0x3939, 0x0f},
	{0x393a, 0xf6},
	{0x393d, 0x05},
	{0x393e, 0x50},
	{0x39dd, 0x00},
	{0x39de, 0x06},
	{0x39e7, 0x04},
	{0x39e8, 0x04},
	{0x39e9, 0x80},
	{0x3e00, 0x00},
	{0x3e01, 0xe9},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e04, 0x0e},
	{0x3e05, 0x90},
	{0x3e16, 0x01},
	{0x3e17, 0x44},
	{0x3e18, 0x01},
	{0x3e19, 0x44},
	{0x3e23, 0x00},
	{0x3e24, 0xf2},
	{0x440e, 0x02},
	{0x4509, 0x18},
	{0x450d, 0x07},
	{0x480f, 0x03},
	{0x5000, 0x06},
	{0x5780, 0x76},
	{0x5784, 0x10},
	{0x5785, 0x08},
	{0x5787, 0x16},
	{0x5788, 0x16},
	{0x5789, 0x15},
	{0x578a, 0x16},
	{0x578b, 0x16},
	{0x578c, 0x15},
	{0x578d, 0x41},
	{0x5790, 0x11},
	{0x5791, 0x0f},
	{0x5792, 0x0f},
	{0x5793, 0x11},
	{0x5794, 0x0f},
	{0x5795, 0x0f},
	{0x5799, 0x46},
	{0x579a, 0x77},
	{0x57a1, 0x04},
	{0x57a8, 0xd2},
	{0x57aa, 0x2a},
	{0x57ab, 0x7f},
	{0x57ac, 0x00},
	{0x57ad, 0x00},
	{0x36e9, 0x53},
	{0x37f9, 0x00},
	// {0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * linear
 * mipi_datarate per lane 900Mbps, 4lane
 *
 * avdd=2.800000
 * dovdd=1.800000
 * dvdd=1.5
 * Cleaned_0x23_SC535HAI_raw_MIPI_27Minput_4Lane_10bit_900Mbps_2592x1944_30fps.ini
 */
static const struct regval sc535hai_linear_10_2592x1944_30fps_4lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x23b0, 0x00},
	{0x23b1, 0x08},
	{0x23b2, 0x00},
	{0x23b3, 0x18},
	{0x23b4, 0x00},
	{0x23b5, 0x38},
	{0x23b6, 0x04},
	{0x23b7, 0x08},
	{0x23b8, 0x04},
	{0x23b9, 0x18},
	{0x23ba, 0x04},
	{0x23bb, 0x38},
	{0x23c0, 0x04},
	{0x23c1, 0x00},
	{0x23c2, 0x04},
	{0x23c3, 0x18},
	{0x23c4, 0x04},
	{0x23c5, 0x78},
	{0x23c6, 0x04},
	{0x23c7, 0x08},
	{0x23c8, 0x04},
	{0x23c9, 0x78},
	{0x3018, 0x7b},
	{0x301e, 0xf0},
	{0x301f, 0x23},
	{0x302c, 0x00},
	{0x30b8, 0x44},
	{0x3200, 0x00},
	{0x3201, 0x30},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3204, 0x0a},
	{0x3205, 0x57},
	{0x3206, 0x07},
	{0x3207, 0x9f},
	{0x3208, 0x0a},
	{0x3209, 0x20},
	{0x320a, 0x07},
	{0x320b, 0x98},
	{0x320c, 0x05},
	{0x320d, 0xdc},
	{0x320e, 0x07},
	{0x320f, 0xd0},
	{0x3210, 0x00},
	{0x3211, 0x04},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0xc0},
	{0x3250, 0x40},
	{0x327f, 0x3f},
	{0x32e0, 0x00},
	{0x3301, 0x12},
	{0x3302, 0x20},
	{0x3304, 0xc0},
	{0x3306, 0xb0},
	{0x3309, 0xf0},
	{0x330a, 0x01},
	{0x330b, 0x70},
	{0x330d, 0x10},
	{0x3310, 0x18},
	{0x331e, 0xb1},
	{0x331f, 0xe1},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x3364, 0x56},
	{0x338f, 0x80},
	{0x3393, 0x1c},
	{0x3394, 0x2c},
	{0x3395, 0x3c},
	{0x3399, 0x0c},
	{0x339a, 0x10},
	{0x339b, 0x18},
	{0x339c, 0x80},
	{0x33ac, 0x10},
	{0x33ad, 0x2c},
	{0x33ae, 0xb0},
	{0x33af, 0xe0},
	{0x33b0, 0x0f},
	{0x33b2, 0x2c},
	{0x33b3, 0x02},
	{0x349f, 0x03},
	{0x34a8, 0x02},
	{0x34a9, 0x08},
	{0x34aa, 0x01},
	{0x34ab, 0x70},
	{0x34ac, 0x01},
	{0x34ad, 0x70},
	{0x34f9, 0x12},
	{0x3631, 0x0f},
	{0x3632, 0x8d},
	{0x3633, 0x4d},
	{0x363b, 0x58},
	{0x363c, 0xd8},
	{0x363d, 0x20},
	{0x3641, 0x08},
	{0x3670, 0x32},
	{0x3671, 0x34},
	{0x3672, 0x26},
	{0x3673, 0x04},
	{0x3674, 0x08},
	{0x3675, 0x04},
	{0x3676, 0x18},
	{0x367e, 0x49},
	{0x367f, 0x49},
	{0x3680, 0x49},
	{0x3681, 0x04},
	{0x3682, 0x08},
	{0x3683, 0x04},
	{0x3684, 0x38},
	{0x3685, 0xc1},
	{0x3686, 0xc2},
	{0x3687, 0xc1},
	{0x3688, 0xc1},
	{0x3689, 0xc1},
	{0x368a, 0xc1},
	{0x368b, 0xc4},
	{0x368c, 0xc1},
	{0x368d, 0x00},
	{0x368e, 0x08},
	{0x368f, 0x00},
	{0x3690, 0x18},
	{0x3691, 0x04},
	{0x3692, 0x00},
	{0x3693, 0x04},
	{0x3694, 0x08},
	{0x3695, 0x04},
	{0x3696, 0x18},
	{0x3697, 0x04},
	{0x3698, 0x38},
	{0x3699, 0x04},
	{0x369a, 0x78},
	{0x36d0, 0x0d},
	{0x36ea, 0x0a},
	{0x36eb, 0x0c},
	{0x36ec, 0x43},
	{0x36ed, 0xaa},
	{0x370f, 0x13},
	{0x3721, 0x6c},
	{0x3722, 0x8b},
	{0x3724, 0xd1},
	{0x3729, 0x34},
	{0x37b0, 0x17},
	{0x37b1, 0x17},
	{0x37b2, 0x13},
	{0x37b3, 0x04},
	{0x37b4, 0x08},
	{0x37b5, 0x04},
	{0x37b6, 0x38},
	{0x37b7, 0x1d},
	{0x37b8, 0x1f},
	{0x37b9, 0x1f},
	{0x37ba, 0x04},
	{0x37bb, 0x04},
	{0x37bc, 0x04},
	{0x37bd, 0x04},
	{0x37be, 0x08},
	{0x37bf, 0x04},
	{0x37c0, 0x38},
	{0x37c1, 0x04},
	{0x37c2, 0x08},
	{0x37c3, 0x04},
	{0x37c4, 0x38},
	{0x37fa, 0x08},
	{0x37fb, 0x22},
	{0x37fc, 0x30},
	{0x37fd, 0x26},
	{0x3900, 0x05},
	{0x3901, 0x00},
	{0x3902, 0xc0},
	{0x3903, 0x40},
	{0x3905, 0x2d},
	{0x391a, 0x72},
	{0x391b, 0x39},
	{0x391c, 0x22},
	{0x391d, 0x00},
	{0x391f, 0x41},
	{0x3926, 0xe0},
	{0x3933, 0x80},
	{0x3934, 0x03},
	{0x3935, 0x01},
	{0x3936, 0xc0},
	{0x3937, 0x6a},
	{0x3938, 0x6b},
	{0x3939, 0x0f},
	{0x393a, 0xf6},
	{0x393d, 0x05},
	{0x393e, 0x50},
	{0x39dd, 0x00},
	{0x39de, 0x06},
	{0x39e7, 0x04},
	{0x39e8, 0x04},
	{0x39e9, 0x80},
	{0x3e00, 0x00},
	{0x3e01, 0x7c},
	{0x3e02, 0x80},
	{0x3e03, 0x0b},
	{0x3e16, 0x01},
	{0x3e17, 0x44},
	{0x3e18, 0x01},
	{0x3e19, 0x44},
	{0x440e, 0x02},
	{0x4509, 0x18},
	{0x450d, 0x07},
	{0x480f, 0x03},
	{0x5000, 0x06},
	{0x5780, 0x76},
	{0x5784, 0x10},
	{0x5785, 0x08},
	{0x5787, 0x16},
	{0x5788, 0x16},
	{0x5789, 0x15},
	{0x578a, 0x16},
	{0x578b, 0x16},
	{0x578c, 0x15},
	{0x578d, 0x41},
	{0x5790, 0x11},
	{0x5791, 0x0f},
	{0x5792, 0x0f},
	{0x5793, 0x11},
	{0x5794, 0x0f},
	{0x5795, 0x0f},
	{0x5799, 0x46},
	{0x579a, 0x77},
	{0x57a1, 0x04},
	{0x57a8, 0xd2},
	{0x57aa, 0x2a},
	{0x57ab, 0x7f},
	{0x57ac, 0x00},
	{0x57ad, 0x00},
	{0x36e9, 0x44},
	{0x37f9, 0x24},
	// {0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 900Mbps, 4lane
 *
 * avdd=2.800000
 * dovdd=1.800000
 * dvdd=1.2
 * Cleaned_0x05_SC535HAI_raw_MIPI_27Minput_4Lane_10bit_900Mbps_2592x1944_30fps_SHDR_VC.ini
 */
static const struct regval sc535hai_hdr2_10_2592x1944_30fps_4lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x23b0, 0x00},
	{0x23b1, 0x08},
	{0x23b2, 0x00},
	{0x23b3, 0x18},
	{0x23b4, 0x00},
	{0x23b5, 0x38},
	{0x23b6, 0x04},
	{0x23b7, 0x08},
	{0x23b8, 0x04},
	{0x23b9, 0x18},
	{0x23ba, 0x04},
	{0x23bb, 0x38},
	{0x23c0, 0x04},
	{0x23c1, 0x00},
	{0x23c2, 0x04},
	{0x23c3, 0x18},
	{0x23c4, 0x04},
	{0x23c5, 0x78},
	{0x23c6, 0x04},
	{0x23c7, 0x08},
	{0x23c8, 0x04},
	{0x23c9, 0x78},
	{0x3018, 0x7b},
	{0x301e, 0xf0},
	{0x301f, 0x05},
	{0x302c, 0x00},
	{0x30b8, 0x44},
	{0x3200, 0x00},
	{0x3201, 0x30},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3204, 0x0a},
	{0x3205, 0x57},
	{0x3206, 0x07},
	{0x3207, 0x9f},
	{0x3208, 0x0a},
	{0x3209, 0x20},
	{0x320a, 0x07},
	{0x320b, 0x98},
	{0x320c, 0x05},
	{0x320d, 0xdc},
	{0x320e, 0x0f},
	{0x320f, 0xa0},
	{0x3210, 0x00},
	{0x3211, 0x04},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0xc0},
	{0x3250, 0xff},
	{0x327f, 0x3f},
	{0x3281, 0x01},
	{0x32e0, 0x00},
	{0x3301, 0x1a},
	{0x3302, 0x20},
	{0x3304, 0xc0},
	{0x3306, 0xe8},
	{0x3309, 0xf0},
	{0x330a, 0x01},
	{0x330b, 0xa0},
	{0x330d, 0x10},
	{0x3310, 0x18},
	{0x331e, 0xa9},
	{0x331f, 0xd9},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x3364, 0x56},
	{0x338f, 0x80},
	{0x3393, 0x24},
	{0x3394, 0x2c},
	{0x3395, 0x3c},
	{0x3399, 0x14},
	{0x339a, 0x1a},
	{0x339b, 0x2c},
	{0x339c, 0x50},
	{0x33ac, 0x10},
	{0x33ad, 0x2c},
	{0x33ae, 0xb0},
	{0x33af, 0xe0},
	{0x33b0, 0x0f},
	{0x33b2, 0x2c},
	{0x33b3, 0x04},
	{0x349f, 0x03},
	{0x34a8, 0x06},
	{0x34a9, 0x08},
	{0x34aa, 0x01},
	{0x34ab, 0xa0},
	{0x34ac, 0x01},
	{0x34ad, 0xa0},
	{0x34f9, 0x0a},
	{0x3631, 0x0f},
	{0x3632, 0x8d},
	{0x3633, 0x4d},
	{0x363b, 0x58},
	{0x363c, 0xb4},
	{0x363d, 0x40},
	{0x3641, 0x08},
	{0x3670, 0x22},
	{0x3671, 0x24},
	{0x3672, 0x26},
	{0x3673, 0x04},
	{0x3674, 0x08},
	{0x3675, 0x04},
	{0x3676, 0x18},
	{0x367e, 0x69},
	{0x367f, 0x69},
	{0x3680, 0x69},
	{0x3681, 0x04},
	{0x3682, 0x08},
	{0x3683, 0x04},
	{0x3684, 0x38},
	{0x3685, 0x80},
	{0x3686, 0x81},
	{0x3687, 0x83},
	{0x3688, 0x86},
	{0x3689, 0x88},
	{0x368a, 0x8e},
	{0x368b, 0xa3},
	{0x368c, 0xbb},
	{0x368d, 0x00},
	{0x368e, 0x08},
	{0x368f, 0x00},
	{0x3690, 0x18},
	{0x3691, 0x04},
	{0x3692, 0x00},
	{0x3693, 0x04},
	{0x3694, 0x08},
	{0x3695, 0x04},
	{0x3696, 0x18},
	{0x3697, 0x04},
	{0x3698, 0x38},
	{0x3699, 0x04},
	{0x369a, 0x78},
	{0x36d0, 0x0d},
	{0x36ea, 0x0a},
	{0x36eb, 0x04},
	{0x36ec, 0x43},
	{0x36ed, 0xaa},
	{0x370f, 0x13},
	{0x3721, 0x6c},
	{0x3722, 0x8b},
	{0x3724, 0xd1},
	{0x3729, 0x34},
	{0x37b0, 0x77},
	{0x37b1, 0x77},
	{0x37b2, 0x77},
	{0x37b3, 0x04},
	{0x37b4, 0x08},
	{0x37b5, 0x04},
	{0x37b6, 0x38},
	{0x37b7, 0x1d},
	{0x37b8, 0x1f},
	{0x37b9, 0x1f},
	{0x37ba, 0x04},
	{0x37bb, 0x04},
	{0x37bc, 0x04},
	{0x37bd, 0x04},
	{0x37be, 0x08},
	{0x37bf, 0x04},
	{0x37c0, 0x38},
	{0x37c1, 0x04},
	{0x37c2, 0x08},
	{0x37c3, 0x04},
	{0x37c4, 0x38},
	{0x37fa, 0x0a},
	{0x37fb, 0x02},
	{0x37fc, 0x20},
	{0x37fd, 0x16},
	{0x3900, 0x05},
	{0x3901, 0x00},
	{0x3902, 0xc0},
	{0x3903, 0x40},
	{0x3905, 0x2d},
	{0x391a, 0x72},
	{0x391b, 0x39},
	{0x391c, 0x22},
	{0x391d, 0x00},
	{0x391f, 0x41},
	{0x3926, 0xe0},
	{0x3933, 0x80},
	{0x3934, 0x03},
	{0x3935, 0x01},
	{0x3936, 0x00},
	{0x3937, 0x68},
	{0x3938, 0x6c},
	{0x3939, 0x0f},
	{0x393a, 0xf6},
	{0x393b, 0x0f},
	{0x393c, 0x70},
	{0x393d, 0x05},
	{0x393e, 0x80},
	{0x39dd, 0x00},
	{0x39de, 0x06},
	{0x39e7, 0x04},
	{0x39e8, 0x04},
	{0x39e9, 0x80},
	{0x3e00, 0x00},
	{0x3e01, 0xe9},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e04, 0x0e},
	{0x3e05, 0x90},
	{0x3e16, 0x01},
	{0x3e17, 0x44},
	{0x3e18, 0x01},
	{0x3e19, 0x44},
	{0x3e23, 0x00},
	{0x3e24, 0xf2},
	{0x440e, 0x02},
	{0x4509, 0x18},
	{0x450d, 0x07},
	{0x480f, 0x03},
	{0x5000, 0x06},
	{0x5780, 0x76},
	{0x5784, 0x10},
	{0x5785, 0x08},
	{0x5787, 0x16},
	{0x5788, 0x16},
	{0x5789, 0x15},
	{0x578a, 0x16},
	{0x578b, 0x16},
	{0x578c, 0x15},
	{0x578d, 0x41},
	{0x5790, 0x11},
	{0x5791, 0x0f},
	{0x5792, 0x0f},
	{0x5793, 0x11},
	{0x5794, 0x0f},
	{0x5795, 0x0f},
	{0x5799, 0x46},
	{0x579a, 0x77},
	{0x57a1, 0x04},
	{0x57a8, 0xd2},
	{0x57aa, 0x2a},
	{0x57ab, 0x7f},
	{0x57ac, 0x00},
	{0x57ad, 0x00},
	{0x36e9, 0x44},
	{0x37f9, 0x04},
	{REG_NULL, 0x00},
};

/*
 * The width and height must be configured to be
 * the same as the current output resolution of the sensor.
 * The input width of the isp needs to be 16 aligned.
 * The input height of the isp needs to be 8 aligned.
 * If the width or height does not meet the alignment rules,
 * you can configure the cropping parameters with the following function to
 * crop out the appropriate resolution.
 * struct v4l2_subdev_pad_ops {
 *	.get_selection
 * }
 */
static const struct sc535hai_mode supported_modes_4lane[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0040,
		.hts_def = 0x5dc * 2,
		.vts_def = 0x07d0,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.global_reg_list = sc535hai_global_4lane_regs,
		.reg_list = sc535hai_linear_10_2592x1944_30fps_4lane_regs,
		.hdr_mode = NO_HDR,
		.mclk = 27000000,
		.link_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.lanes = 4,
	},
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0040,//mark
		.hts_def = 0xbb8,
		.vts_def = 0x0fa0,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.global_reg_list = sc535hai_global_4lane_regs,
		.reg_list = sc535hai_hdr2_10_2592x1944_30fps_4lane_regs,
		.hdr_mode = HDR_X2,
		.mclk = 27000000,
		.link_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
		.lanes = 4,
	},
};

static const struct sc535hai_mode supported_modes_2lane[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0040,//mark
		.hts_def = 0x5dc * 2,
		.vts_def = 0x07d0,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.global_reg_list = sc535hai_global_2lane_regs,
		.reg_list = sc535hai_linear_10_2592x1944_30fps_2lane_regs,
		.hdr_mode = NO_HDR,
		.mclk = 27000000,
		.link_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.lanes = 2,
	},
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x0040,//mark
		.hts_def = 0x5dc,
		.vts_def = 0x0fa0,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.global_reg_list = sc535hai_global_2lane_regs,
		.reg_list = sc535hai_hdr2_10_2592x1944_15fps_2lane_regs,
		.hdr_mode = HDR_X2,
		.mclk = 27000000,
		.link_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
		.lanes = 2,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const s64 link_freq_menu_items[] = {
	SC535HAI_LINK_FREQ_450,
};

static const char *const sc535hai_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4",
};

/* Write registers up to 4 at a time */
static int sc535hai_write_reg(struct i2c_client *client, u16 reg,
			      u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;
	return 0;
}

static int sc535hai_write_array(struct i2c_client *client,
				const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = sc535hai_write_reg(client, regs[i].addr,
					 SC535HAI_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int sc535hai_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			     u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int sc535hai_set_gain_reg(struct sc535hai *sc535hai, u32 gain, int mode)
{
	struct i2c_client *client = sc535hai->client;
	u32 coarse_again = 0, coarse_dgain = 0, fine_again = 0, fine_dgain = 0;
	int ret = 0, gain_factor;

	if (gain < 32)
		gain = 32;
	else if (gain > SC535HAI_GAIN_MAX)
		gain = SC535HAI_GAIN_MAX;

	gain_factor = gain * 1000 / 32;
	if (gain_factor < 2000) {		/*Start again 1.0x - 2.0x gain */
		coarse_again = 0x00;
		coarse_dgain = 0x00;
		fine_dgain = 0x80;
		fine_again = gain_factor * 32 / 1000;
	} else if (gain_factor < 2530) {	/* 2.0x ~ 2.53x  1000 * 2.53 = 2530*/
		coarse_again = 0x01;
		coarse_dgain = 0x00;
		fine_dgain = 0x80;
		fine_again = gain_factor * 32 / 2000;
	} else if (gain_factor < 5060) {	/* 2.53x ~ 5.06x  1000 * 5.06 = 5060*/
		coarse_again = 0x80;
		coarse_dgain = 0x00;
		fine_dgain = 0x80;
		fine_again = gain_factor * 32 / 2530;
	} else if (gain_factor < 10120) {	/* 5.06x ~ 10.12x  1000 * 10.12 = 10120*/
		coarse_again = 0x81;
		coarse_dgain = 0x00;
		fine_dgain = 0x80;
		fine_again = gain_factor * 32 / 5060;
	} else if (gain_factor < 20240) {	/* 10.12x ~ 20.24x  1000 * 20.24 = 20240*/
		coarse_again = 0x83;
		coarse_dgain = 0x00;
		fine_dgain = 0x80;
		fine_again = gain_factor * 32 / 10120;
	} else if (gain_factor < 40480) {	/* 20.24x ~ 40.48x  1000 * 40.48 = 40480*/
		coarse_again = 0x87;
		coarse_dgain = 0x00;
		fine_dgain = 0x80;
		fine_again = gain_factor * 32 / 20240;
	} else if (gain_factor <= 79695) {	/* 40.48x ~ 79.695x 1000 * 79.695 = 79695*/
		/* End again 79.695x */
		coarse_again = 0x8f;
		coarse_dgain = 0x00;
		fine_dgain = 0x80;
		fine_again = gain_factor * 32 / 40480;
	} else if (gain_factor < 79695 * 2) {
		//open dgain begin  max digital gain 4X
		coarse_again = 0x8f;
		coarse_dgain = 0x00;
		fine_again = 0x3f;
		fine_dgain = gain_factor * 128 / 79695;
	} else if (gain_factor < 79695 * 4) {
		coarse_again = 0x8f;
		coarse_dgain = 0x01;
		fine_again = 0x3f;
		fine_dgain = gain_factor * 128 / 79695 / 2;
	} else if (gain_factor < 79695 * 8) {
		coarse_again = 0x8f;
		coarse_dgain = 0x03;
		fine_again = 0x3f;
		fine_dgain = gain_factor * 128 / 79695 / 4;
	} else if (gain_factor < 79695 * 16) {
		coarse_again = 0x8f;
		coarse_dgain = 0x07;
		fine_again = 0x3f;
		fine_dgain = gain_factor * 128 / 79695 / 8;
	}
	dev_dbg(&client->dev, "c_again: 0x%x, c_dgain: 0x%x, f_again: 0x%x, f_dgain: 0x%0x\n",
		coarse_again, coarse_dgain, fine_again, fine_dgain);

	if (mode == SC535HAI_LGAIN) {
		ret = sc535hai_write_reg(sc535hai->client,
					 SC535HAI_REG_DIG_GAIN,
					 SC535HAI_REG_VALUE_08BIT,
					 coarse_dgain);
		ret |= sc535hai_write_reg(sc535hai->client,
					  SC535HAI_REG_DIG_FINE_GAIN,
					  SC535HAI_REG_VALUE_08BIT,
					  fine_dgain);
		ret |= sc535hai_write_reg(sc535hai->client,
					  SC535HAI_REG_ANA_GAIN,
					  SC535HAI_REG_VALUE_08BIT,
					  coarse_again);
		ret |= sc535hai_write_reg(sc535hai->client,
					  SC535HAI_REG_ANA_FINE_GAIN,
					  SC535HAI_REG_VALUE_08BIT,
					  fine_again);
	} else {
		ret = sc535hai_write_reg(sc535hai->client,
					 SC535HAI_REG_SDIG_GAIN,
					 SC535HAI_REG_VALUE_08BIT,
					 coarse_dgain);
		ret |= sc535hai_write_reg(sc535hai->client,
					  SC535HAI_REG_SDIG_FINE_GAIN,
					  SC535HAI_REG_VALUE_08BIT,
					  fine_dgain);
		ret |= sc535hai_write_reg(sc535hai->client,
					  SC535HAI_REG_SANA_GAIN,
					  SC535HAI_REG_VALUE_08BIT,
					  coarse_again);
		ret |= sc535hai_write_reg(sc535hai->client,
					  SC535HAI_REG_SANA_FINE_GAIN,
					  SC535HAI_REG_VALUE_08BIT,
					  fine_again);
	}
	return ret;
}

static int sc535hai_set_hdrae(struct sc535hai *sc535hai,
			      struct preisp_hdrae_exp_s *ae)
{
	int ret = 0;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;
	u32 l_exp_max = 0, s_max_exp_time = 0, s_max_exp_time_h = 0, s_max_exp_time_l = 0;

	if (!sc535hai->has_init_exp && !sc535hai->streaming) {
		sc535hai->init_hdrae_exp = *ae;
		sc535hai->has_init_exp = true;
		dev_dbg(&sc535hai->client->dev, "sc535hai don't stream, record exp for hdr!\n");
		return ret;
	}
	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;

	dev_dbg(&sc535hai->client->dev,
		"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
		l_exp_time, l_a_gain, m_exp_time,
		m_a_gain, s_exp_time, s_a_gain);

	if (sc535hai->cur_mode->hdr_mode == HDR_X2) {
		//2 stagger
		l_a_gain = m_a_gain;
		l_exp_time = m_exp_time;
	}

	ret = sc535hai_read_reg(sc535hai->client,
				SC535HAI_REG_MAX_SEXPOSURE_H,
				SC535HAI_REG_VALUE_08BIT,
				&s_max_exp_time_h);
	ret |= sc535hai_read_reg(sc535hai->client,
				 SC535HAI_REG_MAX_SEXPOSURE_L,
				 SC535HAI_REG_VALUE_08BIT,
				 &s_max_exp_time_l);
	s_max_exp_time = (s_max_exp_time_h << 8) | s_max_exp_time_l;

	l_exp_max = sc535hai->cur_vts - s_max_exp_time - 11;  //{vts-(3e23~3e24)}-11
	if (l_exp_time > l_exp_max)
		l_exp_time = l_exp_max;
	if (s_exp_time > s_max_exp_time - 9)  //{3e23~3e24}-9
		s_exp_time = s_max_exp_time - 9;

	ret |= sc535hai_write_reg(sc535hai->client,
				  SC535HAI_REG_EXPOSURE_H,
				  SC535HAI_REG_VALUE_08BIT,
				  SC535HAI_FETCH_EXP_H(l_exp_time));
	ret |= sc535hai_write_reg(sc535hai->client,
				  SC535HAI_REG_EXPOSURE_M,
				  SC535HAI_REG_VALUE_08BIT,
				  SC535HAI_FETCH_EXP_M(l_exp_time));
	ret |= sc535hai_write_reg(sc535hai->client,
				  SC535HAI_REG_EXPOSURE_L,
				  SC535HAI_REG_VALUE_08BIT,
				  SC535HAI_FETCH_EXP_L(l_exp_time));
	ret |= sc535hai_write_reg(sc535hai->client,
				  SC535HAI_REG_SEXPOSURE_M,
				  SC535HAI_REG_VALUE_08BIT,
				  SC535HAI_FETCH_EXP_M(s_exp_time));
	ret |= sc535hai_write_reg(sc535hai->client,
				  SC535HAI_REG_SEXPOSURE_L,
				  SC535HAI_REG_VALUE_08BIT,
				  SC535HAI_FETCH_EXP_L(s_exp_time));

	ret |= sc535hai_set_gain_reg(sc535hai, l_a_gain, SC535HAI_LGAIN);
	ret |= sc535hai_set_gain_reg(sc535hai, s_a_gain, SC535HAI_SGAIN);

	return ret;
}

static int sc535hai_get_reso_dist(const struct sc535hai_mode *mode,
				  struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc535hai_mode *
sc535hai_find_best_fit(struct sc535hai *sc535hai, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < sc535hai->cfg_num; i++) {
		dist = sc535hai_get_reso_dist(&sc535hai->supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		} else if (dist == cur_best_fit_dist &&
			   framefmt->code == sc535hai->supported_modes[i].bus_fmt) {
			cur_best_fit = i;
			break;
		}
	}

	return &sc535hai->supported_modes[cur_best_fit];
}

static int sc535hai_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	const struct sc535hai_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes;

	mutex_lock(&sc535hai->mutex);

	mode = sc535hai_find_best_fit(sc535hai, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc535hai->mutex);
		return -ENOTTY;
#endif
	} else {
		sc535hai->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc535hai->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc535hai->vblank, vblank_def,
					 SC535HAI_VTS_MAX - mode->height,
					 1, vblank_def);
		dst_link_freq = mode->link_freq_idx;
		dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
				 mode->bpp * 2 * lanes;
		__v4l2_ctrl_s_ctrl_int64(sc535hai->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(sc535hai->link_freq,
				   dst_link_freq);
		sc535hai->cur_fps = mode->max_fps;
	}

	mutex_unlock(&sc535hai->mutex);

	return 0;
}

static int sc535hai_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	const struct sc535hai_mode *mode = sc535hai->cur_mode;

	mutex_lock(&sc535hai->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc535hai->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virtual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc535hai->mutex);

	return 0;
}

static int sc535hai_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int sc535hai_enum_frame_sizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);

	if (fse->index >= sc535hai->cfg_num)
		return -EINVAL;

	if (fse->code != sc535hai->supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = sc535hai->supported_modes[fse->index].width;
	fse->max_width  = sc535hai->supported_modes[fse->index].width;
	fse->max_height = sc535hai->supported_modes[fse->index].height;
	fse->min_height = sc535hai->supported_modes[fse->index].height;

	return 0;
}

static int sc535hai_enable_test_pattern(struct sc535hai *sc535hai, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc535hai_read_reg(sc535hai->client, SC535HAI_REG_TEST_PATTERN,
				SC535HAI_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC535HAI_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SC535HAI_TEST_PATTERN_BIT_MASK;

	ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_TEST_PATTERN,
				  SC535HAI_REG_VALUE_08BIT, val);
	return ret;
}

static int sc535hai_g_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	const struct sc535hai_mode *mode = sc535hai->cur_mode;

	if (sc535hai->streaming)
		fi->interval = sc535hai->cur_fps;
	else
		fi->interval = mode->max_fps;
	return 0;
}

static const struct sc535hai_mode *sc535hai_find_mode(struct sc535hai *sc535hai, int fps)
{
	const struct sc535hai_mode *mode = NULL;
	const struct sc535hai_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < sc535hai->cfg_num; i++) {
		mode = &sc535hai->supported_modes[i];
		if (mode->width == sc535hai->cur_mode->width &&
		    mode->height == sc535hai->cur_mode->height &&
		    mode->hdr_mode == sc535hai->cur_mode->hdr_mode &&
		    mode->bus_fmt == sc535hai->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
						    mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int sc535hai_s_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	const struct sc535hai_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	int fps;

	if (sc535hai->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = sc535hai_find_mode(sc535hai, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	sc535hai->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(sc535hai->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(sc535hai->vblank, vblank_def,
				 SC535HAI_VTS_MAX - mode->height,
				 1, vblank_def);
	pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
		     mode->bpp * 2 * mode->lanes;

	__v4l2_ctrl_s_ctrl_int64(sc535hai->pixel_rate,
				 pixel_rate);
	__v4l2_ctrl_s_ctrl(sc535hai->link_freq,
			   mode->link_freq_idx);
	sc535hai->cur_fps = mode->max_fps;

	return 0;
}

static int sc535hai_g_mbus_config(struct v4l2_subdev *sd,
				  unsigned int pad_id,
				  struct v4l2_mbus_config *config)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	const struct sc535hai_mode *mode = sc535hai->cur_mode;
	u8 lanes = sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes;

	u32 val = 1 << (lanes - 1) |
		  V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void sc535hai_get_module_inf(struct sc535hai *sc535hai,
				    struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC535HAI_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc535hai->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc535hai->len_name, sizeof(inf->base.lens));
}

static int sc535hai_set_setting(struct sc535hai *sc535hai, struct rk_sensor_setting *setting)
{
	int i = 0;
	int cur_fps = 0;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	const struct sc535hai_mode *mode = NULL;
	const struct sc535hai_mode *match = NULL;
	u8 lane = sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes;

	dev_info(&sc535hai->client->dev,
		 "sensor setting: %d x %d, fps:%d fmt:%d, mode:%d\n",
		 setting->width, setting->height,
		 setting->fps, setting->fmt, setting->mode);

	for (i = 0; i < sc535hai->cfg_num; i++) {
		mode = &sc535hai->supported_modes[i];
		if (mode->width == setting->width &&
		    mode->height == setting->height &&
		    mode->hdr_mode == setting->mode &&
		    mode->bus_fmt == setting->fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
						    mode->max_fps.numerator);
			if (cur_fps == setting->fps) {
				match = mode;
				break;
			}
		}
	}

	if (match) {
		dev_info(&sc535hai->client->dev, "-----%s: match the support mode, mode idx:%d-----\n",
			 __func__, i);
		sc535hai->cur_mode = mode;

		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc535hai->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc535hai->vblank, vblank_def,
					 SC535HAI_VTS_MAX - mode->height,
					 1, vblank_def);


		__v4l2_ctrl_s_ctrl(sc535hai->link_freq, mode->link_freq_idx);
		pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
			     mode->bpp * 2 * lane;
		__v4l2_ctrl_s_ctrl_int64(sc535hai->pixel_rate, pixel_rate);
		dev_info(&sc535hai->client->dev, "freq_idx:%d pixel_rate:%lld\n",
			 mode->link_freq_idx, pixel_rate);

		sc535hai->cur_vts = mode->vts_def;
		sc535hai->cur_fps = mode->max_fps;

		dev_info(&sc535hai->client->dev, "hts_def:%d cur_vts:%d cur_fps:%d\n",
			 mode->hts_def, mode->vts_def,
			 sc535hai->cur_fps.denominator / sc535hai->cur_fps.numerator);
	} else {
		dev_err(&sc535hai->client->dev, "couldn't match the support modes\n");
		return -EINVAL;
	}

	return 0;
}

static long sc535hai_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rk_sensor_setting *setting;
	struct rk_light_param *light_param;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes;
	const struct sc535hai_mode *mode;
	int cur_best_fit = -1;
	int cur_best_fit_dist = -1;
	int cur_dist, cur_fps, dst_fps;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc535hai_get_module_inf(sc535hai, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc535hai->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		if (hdr->hdr_mode == sc535hai->cur_mode->hdr_mode)
			return 0;
		w = sc535hai->cur_mode->width;
		h = sc535hai->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(sc535hai->cur_mode->max_fps.denominator,
					    sc535hai->cur_mode->max_fps.numerator);
		for (i = 0; i < sc535hai->cfg_num; i++) {
			if (w == sc535hai->supported_modes[i].width &&
			    h == sc535hai->supported_modes[i].height &&
			    sc535hai->supported_modes[i].hdr_mode == hdr->hdr_mode &&
			    sc535hai->supported_modes[i].bus_fmt == sc535hai->cur_mode->bus_fmt) {
				cur_fps = DIV_ROUND_CLOSEST(sc535hai->supported_modes[i].max_fps.denominator,
							    sc535hai->supported_modes[i].max_fps.numerator);
				cur_dist = abs(cur_fps - dst_fps);
				if (cur_best_fit_dist == -1 || cur_dist < cur_best_fit_dist) {
					cur_best_fit_dist = cur_dist;
					cur_best_fit = i;
				} else if (cur_dist == cur_best_fit_dist) {
					cur_best_fit = i;
					break;
				}
			}
		}
		if (cur_best_fit == -1) {
			dev_err(&sc535hai->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			sc535hai->cur_mode = &sc535hai->supported_modes[cur_best_fit];
			mode = sc535hai->cur_mode;
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(sc535hai->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc535hai->vblank, h,
						 SC535HAI_VTS_MAX - sc535hai->cur_mode->height,
						 1, h);
			sc535hai->cur_fps = sc535hai->cur_mode->max_fps;

			dst_link_freq = mode->link_freq_idx;
			dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
					 mode->bpp * 2 * lanes;
			__v4l2_ctrl_s_ctrl_int64(sc535hai->pixel_rate,
						 dst_pixel_rate);
			__v4l2_ctrl_s_ctrl(sc535hai->link_freq,
					   dst_link_freq);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		sc535hai_set_hdrae(sc535hai, arg);
		if (sc535hai->cam_sw_inf)
			memcpy(&sc535hai->cam_sw_inf->hdr_ae, (struct preisp_hdrae_exp_s *)(arg),
			       sizeof(struct preisp_hdrae_exp_s));
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (sc535hai->enable_light_ctl) {
			sc535hai->light_param.light_enable = stream;
			light_ctl_write(sc535hai->module_index, &sc535hai->light_param);
		}
		if (sc535hai->standby_hw) {	/* hardware standby */
			if (stream) {
				u32 val;

				/* pwdn gpio pull up */
				if (!IS_ERR(sc535hai->pwdn_gpio))
					gpiod_set_value_cansleep(sc535hai->pwdn_gpio, 1);
				/* mipi clk on */
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_MIPI_CTRL,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MIPI_CTRL_ON);
				/* adjust timing */
				ret |= sc535hai_read_reg(sc535hai->client, 0x36e9,
							 SC535HAI_REG_VALUE_08BIT, &val);
				val &= 0x7f;
				ret |= sc535hai_write_reg(sc535hai->client, 0x36e9,
							  SC535HAI_REG_VALUE_08BIT,
							  val);
				ret |= sc535hai_read_reg(sc535hai->client, 0x36f9,
							 SC535HAI_REG_VALUE_08BIT, &val);
				val &= 0x7f;
				ret |= sc535hai_write_reg(sc535hai->client, 0x36f9,
							  SC535HAI_REG_VALUE_08BIT,
							  val);

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
				if (__v4l2_ctrl_handler_setup(&sc535hai->ctrl_handler))
					dev_err(&sc535hai->client->dev, "__v4l2_ctrl_handler_setup fail!");
				if (sc535hai->cur_mode->hdr_mode != NO_HDR) {	// hdr mode
					if (sc535hai->cam_sw_inf) {
						ret = sc535hai_ioctl(&sc535hai->subdev,
								     PREISP_CMD_SET_HDRAE_EXP,
								     &sc535hai->cam_sw_inf->hdr_ae);
						if (ret) {
							dev_err(&sc535hai->client->dev,
								"init exp fail in hdr mode\n");
							return ret;
						}
					}
				}
#endif

				/* stream on */
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_CTRL_MODE,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MODE_STREAMING);
				dev_info(&sc535hai->client->dev,
					 "quickstream, streaming on: exit hw standby mode\n");
				sc535hai->is_standby = false;
			} else {
				u32 val;

				/* adjust timing */
				ret |= sc535hai_read_reg(sc535hai->client, 0x36e9,
							 SC535HAI_REG_VALUE_08BIT, &val);
				val |= 0x80;
				ret |= sc535hai_write_reg(sc535hai->client, 0x36e9,
							  SC535HAI_REG_VALUE_08BIT,
							  val);
				ret |= sc535hai_read_reg(sc535hai->client, 0x36f9,
							 SC535HAI_REG_VALUE_08BIT, &val);
				val |= 0x80;
				ret |= sc535hai_write_reg(sc535hai->client, 0x36f9,
							  SC535HAI_REG_VALUE_08BIT,
							  val);
				/* stream off */
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_CTRL_MODE,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MODE_SW_STANDBY);
				/* mipi clk off */
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_MIPI_CTRL,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MIPI_CTRL_OFF);
				/* pwnd gpio pull down */
				if (!IS_ERR(sc535hai->pwdn_gpio))
					gpiod_set_value_cansleep(sc535hai->pwdn_gpio, 0);
				dev_info(&sc535hai->client->dev,
					 "quickstream, streaming off: enter hw standby mode\n");
				sc535hai->is_standby = true;
			}
		} else {	/* software standby */
			if (stream) {
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_MIPI_CTRL,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MIPI_CTRL_ON);
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_CTRL_MODE,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MODE_STREAMING);
				dev_info(&sc535hai->client->dev,
					 "quickstream, streaming on: exit soft standby mode\n");
			} else {
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_CTRL_MODE,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MODE_SW_STANDBY);
				ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_REG_MIPI_CTRL,
							  SC535HAI_REG_VALUE_08BIT,
							  SC535HAI_MIPI_CTRL_OFF);
				dev_info(&sc535hai->client->dev,
					 "quickstream, streaming off: enter soft standby mode\n");
			}
		}
		break;
	case RKCIS_CMD_SELECT_SETTING:
		setting = (struct rk_sensor_setting *)arg;
		ret = sc535hai_set_setting(sc535hai, setting);
		break;
	case RKCIS_CMD_FLASH_LIGHT_CTRL:
		dev_info(&sc535hai->client->dev, "set flash light param\n");
		light_param = (struct rk_light_param *)arg;
		if (light_param->light_enable) {
			memcpy(&sc535hai->light_param, light_param, sizeof(struct rk_light_param));
			sc535hai->enable_light_ctl = true;
		} else {
			sc535hai->enable_light_ctl = false;
		}
		ret = light_ctl_write(sc535hai->module_index, light_param);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc535hai_compat_ioctl32(struct v4l2_subdev *sd,
				    unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	struct rk_sensor_setting *setting;
	struct rk_light_param *light_param;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc535hai_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf)))
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc535hai_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr)))
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = sc535hai_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = sc535hai_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = sc535hai_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	case RKCIS_CMD_SELECT_SETTING:
		setting = kzalloc(sizeof(*setting), GFP_KERNEL);
		if (!setting) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(setting, up, sizeof(*setting));
		if (!ret)
			ret = sc535hai_ioctl(sd, cmd, setting);
		else
			ret = -EFAULT;
		kfree(setting);
		break;
	case RKCIS_CMD_FLASH_LIGHT_CTRL:
		light_param = kzalloc(sizeof(*light_param), GFP_KERNEL);
		if (!light_param) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(light_param, up, sizeof(*light_param));
		if (!ret)
			ret = sc535hai_ioctl(sd, cmd, light_param);
		else
			ret = -EFAULT;
		kfree(light_param);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc535hai_start_stream(struct sc535hai *sc535hai)
{
	int ret;

	if (!sc535hai->is_thunderboot) {
		ret = sc535hai_write_array(sc535hai->client, sc535hai->cur_mode->reg_list);
		if (ret)
			return ret;
		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&sc535hai->ctrl_handler);
		if (ret)
			return ret;
		if (sc535hai->has_init_exp && sc535hai->cur_mode->hdr_mode != NO_HDR) {
			ret = sc535hai_ioctl(&sc535hai->subdev, PREISP_CMD_SET_HDRAE_EXP,
					     &sc535hai->init_hdrae_exp);
			if (ret) {
				dev_err(&sc535hai->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}
	ret = sc535hai_write_reg(sc535hai->client, SC535HAI_REG_CTRL_MODE,
				 SC535HAI_REG_VALUE_08BIT, SC535HAI_MODE_STREAMING);
	return ret;
}

static int __sc535hai_stop_stream(struct sc535hai *sc535hai)
{
	sc535hai->has_init_exp = false;
	if (sc535hai->is_thunderboot)
		sc535hai->is_first_streamoff = true;
	sc535hai->enable_light_ctl = false;
	return sc535hai_write_reg(sc535hai->client, SC535HAI_REG_CTRL_MODE,
				  SC535HAI_REG_VALUE_08BIT, SC535HAI_MODE_SW_STANDBY);
}

static int __sc535hai_power_on(struct sc535hai *sc535hai);
static int sc535hai_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	struct i2c_client *client = sc535hai->client;
	int ret = 0;

	mutex_lock(&sc535hai->mutex);
	on = !!on;
	if (on == sc535hai->streaming)
		goto unlock_and_return;
	if (on) {
		if (sc535hai->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			sc535hai->is_thunderboot = false;
			__sc535hai_power_on(sc535hai);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __sc535hai_start_stream(sc535hai);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc535hai_stop_stream(sc535hai);
		pm_runtime_put(&client->dev);
	}

	sc535hai->streaming = on;
unlock_and_return:
	mutex_unlock(&sc535hai->mutex);
	return ret;
}

static int sc535hai_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	struct i2c_client *client = sc535hai->client;
	int ret = 0;

	mutex_lock(&sc535hai->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc535hai->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!sc535hai->is_thunderboot) {
			ret = sc535hai_write_array(sc535hai->client,
						   sc535hai->cur_mode->global_reg_list);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		sc535hai->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc535hai->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc535hai->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sc535hai_cal_delay(u32 cycles, struct sc535hai *sc535hai)
{
	return DIV_ROUND_UP(cycles, sc535hai->cur_mode->mclk / 1000 / 1000);
}

static int __sc535hai_power_on(struct sc535hai *sc535hai)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sc535hai->client->dev;

	if (!IS_ERR_OR_NULL(sc535hai->pins_default)) {
		ret = pinctrl_select_state(sc535hai->pinctrl,
					   sc535hai->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc535hai->xvclk, sc535hai->cur_mode->mclk);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (%dHz)\n", sc535hai->cur_mode->mclk);
	if (clk_get_rate(sc535hai->xvclk) != sc535hai->cur_mode->mclk)
		dev_warn(dev, "xvclk mismatched, modes are based on %dHz\n",
			 sc535hai->cur_mode->mclk);
	ret = clk_prepare_enable(sc535hai->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	cam_sw_regulator_bulk_init(sc535hai->cam_sw_inf, SC535HAI_NUM_SUPPLIES, sc535hai->supplies);

	if (sc535hai->is_thunderboot)
		return 0;

	if (!IS_ERR(sc535hai->reset_gpio))
		gpiod_set_value_cansleep(sc535hai->reset_gpio, 0);

	ret = regulator_bulk_enable(SC535HAI_NUM_SUPPLIES, sc535hai->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc535hai->reset_gpio))
		gpiod_set_value_cansleep(sc535hai->reset_gpio, 1);

	usleep_range(500, 1000);

	if (!IS_ERR(sc535hai->pwdn_gpio))
		gpiod_set_value_cansleep(sc535hai->pwdn_gpio, 1);

	if (!IS_ERR(sc535hai->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sc535hai_cal_delay(8192, sc535hai);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(sc535hai->xvclk);

	return ret;
}

static void __sc535hai_power_off(struct sc535hai *sc535hai)
{
	int ret;
	struct device *dev = &sc535hai->client->dev;

	clk_disable_unprepare(sc535hai->xvclk);
	if (sc535hai->is_thunderboot) {
		if (sc535hai->is_first_streamoff) {
			sc535hai->is_thunderboot = false;
			sc535hai->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(sc535hai->pwdn_gpio))
		gpiod_set_value_cansleep(sc535hai->pwdn_gpio, 0);
	clk_disable_unprepare(sc535hai->xvclk);
	if (!IS_ERR(sc535hai->reset_gpio))
		gpiod_set_value_cansleep(sc535hai->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(sc535hai->pins_sleep)) {
		ret = pinctrl_select_state(sc535hai->pinctrl,
					   sc535hai->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC535HAI_NUM_SUPPLIES, sc535hai->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int __maybe_unused sc535hai_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hai *sc535hai = to_sc535hai(sd);

	if (sc535hai->standby_hw) {
		dev_info(dev, "resume standby!");
		return 0;
	} else {
		cam_sw_prepare_wakeup(sc535hai->cam_sw_inf, dev);

		usleep_range(4000, 5000);
		cam_sw_write_array(sc535hai->cam_sw_inf);

		if (__v4l2_ctrl_handler_setup(&sc535hai->ctrl_handler))
			dev_err(dev, "__v4l2_ctrl_handler_setup fail!");

		if (sc535hai->has_init_exp && sc535hai->cur_mode != NO_HDR) {	// hdr mode
			ret = sc535hai_ioctl(&sc535hai->subdev, PREISP_CMD_SET_HDRAE_EXP,
					     &sc535hai->cam_sw_inf->hdr_ae);
			if (ret) {
				dev_err(&sc535hai->client->dev, "set exp fail in hdr mode\n");
				return ret;
			}
		}
	}

	return 0;
}

static int __maybe_unused sc535hai_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hai *sc535hai = to_sc535hai(sd);

	if (sc535hai->standby_hw) {
		dev_info(dev, "suspend standby!");
		return 0;
	}

	cam_sw_write_array_cb_init(sc535hai->cam_sw_inf, client,
				   (void *)sc535hai->cur_mode->reg_list,
				   (sensor_write_array)sc535hai_write_array);
	cam_sw_prepare_sleep(sc535hai->cam_sw_inf);

	return 0;
}
#else
#define sc535hai_resume NULL
#define sc535hai_suspend NULL
#endif

static int __maybe_unused sc535hai_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hai *sc535hai = to_sc535hai(sd);

	return __sc535hai_power_on(sc535hai);
}

static int __maybe_unused sc535hai_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hai *sc535hai = to_sc535hai(sd);

	__sc535hai_power_off(sc535hai);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc535hai_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc535hai_mode *def_mode = &sc535hai->supported_modes[0];

	mutex_lock(&sc535hai->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc535hai->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc535hai_enum_frame_interval(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc535hai *sc535hai = to_sc535hai(sd);

	if (fie->index >= sc535hai->cfg_num)
		return -EINVAL;

	fie->code = sc535hai->supported_modes[fie->index].bus_fmt;
	fie->width = sc535hai->supported_modes[fie->index].width;
	fie->height = sc535hai->supported_modes[fie->index].height;
	fie->interval = sc535hai->supported_modes[fie->index].max_fps;
	fie->reserved[0] = sc535hai->supported_modes[fie->index].hdr_mode;
	return 0;
}

#define DST_WIDTH 2592
#define DST_HEIGHT 1944

/*
 * The resolution of the driver configuration needs to be exactly
 * the same as the current output resolution of the sensor,
 * the input width of the isp needs to be 16 aligned,
 * the input height of the isp needs to be 8 aligned.
 * Can be cropped to standard resolution by this function,
 * otherwise it will crop out strange resolution according
 * to the alignment rules.
 */
static int sc535hai_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = 0;
		sel->r.width = DST_WIDTH;
		sel->r.top = 0;
		sel->r.height = DST_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static const struct dev_pm_ops sc535hai_pm_ops = {
	SET_RUNTIME_PM_OPS(sc535hai_runtime_suspend,
			   sc535hai_runtime_resume, NULL)
#ifdef CONFIG_VIDEO_CAM_SLEEP_WAKEUP
	SET_LATE_SYSTEM_SLEEP_PM_OPS(sc535hai_suspend, sc535hai_resume)
#endif
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc535hai_internal_ops = {
	.open = sc535hai_open,
};
#endif

static const struct v4l2_subdev_core_ops sc535hai_core_ops = {
	.s_power = sc535hai_s_power,
	.ioctl = sc535hai_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc535hai_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc535hai_video_ops = {
	.s_stream = sc535hai_s_stream,
	.g_frame_interval = sc535hai_g_frame_interval,
	.s_frame_interval = sc535hai_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc535hai_pad_ops = {
	.enum_mbus_code = sc535hai_enum_mbus_code,
	.enum_frame_size = sc535hai_enum_frame_sizes,
	.enum_frame_interval = sc535hai_enum_frame_interval,
	.get_fmt = sc535hai_get_fmt,
	.set_fmt = sc535hai_set_fmt,
	.get_selection = sc535hai_get_selection,
	.get_mbus_config = sc535hai_g_mbus_config,
};

static const struct v4l2_subdev_ops sc535hai_subdev_ops = {
	.core	= &sc535hai_core_ops,
	.video	= &sc535hai_video_ops,
	.pad	= &sc535hai_pad_ops,
};

static void sc535hai_modify_fps_info(struct sc535hai *sc535hai)
{
	const struct sc535hai_mode *mode = sc535hai->cur_mode;

	sc535hai->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
					sc535hai->cur_vts;
}

static int sc535hai_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc535hai *sc535hai = container_of(ctrl->handler,
				    struct sc535hai, ctrl_handler);
	struct i2c_client *client = sc535hai->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc535hai->cur_mode->height + ctrl->val - 8;
		__v4l2_ctrl_modify_range(sc535hai->exposure,
					 sc535hai->exposure->minimum, max,
					 sc535hai->exposure->step,
					 sc535hai->exposure->default_value);
		break;
	}

	if (sc535hai->standby_hw && sc535hai->is_standby) {
		dev_dbg(&client->dev, "%s: is_standby = true, will return\n", __func__);
		return 0;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (sc535hai->cur_mode->hdr_mode == NO_HDR) {
			/* 4 least significant bits of expsoure are fractional part */
			ret = sc535hai_write_reg(sc535hai->client,
						 SC535HAI_REG_EXPOSURE_H,
						 SC535HAI_REG_VALUE_08BIT,
						 SC535HAI_FETCH_EXP_H(ctrl->val));
			ret |= sc535hai_write_reg(sc535hai->client,
						  SC535HAI_REG_EXPOSURE_M,
						  SC535HAI_REG_VALUE_08BIT,
						  SC535HAI_FETCH_EXP_M(ctrl->val));
			ret |= sc535hai_write_reg(sc535hai->client,
						  SC535HAI_REG_EXPOSURE_L,
						  SC535HAI_REG_VALUE_08BIT,
						  SC535HAI_FETCH_EXP_L(ctrl->val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (sc535hai->cur_mode->hdr_mode == NO_HDR)
			ret = sc535hai_set_gain_reg(sc535hai, ctrl->val, SC535HAI_LGAIN);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = sc535hai_write_reg(sc535hai->client,
					 SC535HAI_REG_VTS_H,
					 SC535HAI_REG_VALUE_08BIT,
					 (ctrl->val + sc535hai->cur_mode->height)
					 >> 8);
		ret |= sc535hai_write_reg(sc535hai->client,
					  SC535HAI_REG_VTS_L,
					  SC535HAI_REG_VALUE_08BIT,
					  (ctrl->val + sc535hai->cur_mode->height)
					  & 0xff);
		sc535hai->cur_vts = ctrl->val + sc535hai->cur_mode->height;
		if (sc535hai->cur_vts != sc535hai->cur_mode->vts_def)
			sc535hai_modify_fps_info(sc535hai);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc535hai_enable_test_pattern(sc535hai, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc535hai_read_reg(sc535hai->client, SC535HAI_FLIP_MIRROR_REG,
					SC535HAI_REG_VALUE_08BIT, &val);
		ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_FLIP_MIRROR_REG,
					  SC535HAI_REG_VALUE_08BIT,
					  SC535HAI_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = sc535hai_read_reg(sc535hai->client, SC535HAI_FLIP_MIRROR_REG,
					SC535HAI_REG_VALUE_08BIT, &val);
		ret |= sc535hai_write_reg(sc535hai->client, SC535HAI_FLIP_MIRROR_REG,
					  SC535HAI_REG_VALUE_08BIT,
					  SC535HAI_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc535hai_ctrl_ops = {
	.s_ctrl = sc535hai_set_ctrl,
};

static int sc535hai_initialize_controls(struct sc535hai *sc535hai)
{
	const struct sc535hai_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes;

	handler = &sc535hai->ctrl_handler;
	mode = sc535hai->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc535hai->mutex;

	sc535hai->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			      V4L2_CID_LINK_FREQ,
			      ARRAY_SIZE(link_freq_menu_items) - 1,
			      0, link_freq_menu_items);
	if (sc535hai->link_freq)
		sc535hai->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc535hai->client->dev,
			"00 Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	dst_link_freq = mode->link_freq_idx;
	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
			 mode->bpp * 2 * lanes;
	sc535hai->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
				0, PIXEL_RATE_WITH_450M_10BIT_4L, 1, dst_pixel_rate);
	__v4l2_ctrl_s_ctrl(sc535hai->link_freq, dst_link_freq);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc535hai->client->dev,
			"11 Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	h_blank = mode->hts_def - mode->width;
	sc535hai->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					     h_blank, h_blank, 1, h_blank);
	if (sc535hai->hblank)
		sc535hai->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	sc535hai->vblank = v4l2_ctrl_new_std(handler, &sc535hai_ctrl_ops,
					     V4L2_CID_VBLANK, vblank_def,
					     SC535HAI_VTS_MAX - mode->height,
					     1, vblank_def);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc535hai->client->dev,
			"22 Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	exposure_max = mode->vts_def - 8;
	sc535hai->exposure = v4l2_ctrl_new_std(handler, &sc535hai_ctrl_ops,
					       V4L2_CID_EXPOSURE, SC535HAI_EXPOSURE_MIN,
					       exposure_max, SC535HAI_EXPOSURE_STEP,
					       mode->exp_def);
	sc535hai->anal_gain = v4l2_ctrl_new_std(handler, &sc535hai_ctrl_ops,
						V4L2_CID_ANALOGUE_GAIN, SC535HAI_GAIN_MIN,
						SC535HAI_GAIN_MAX, SC535HAI_GAIN_STEP,
						SC535HAI_GAIN_DEFAULT);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc535hai->client->dev,
			"33 Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	sc535hai->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				 &sc535hai_ctrl_ops,
				 V4L2_CID_TEST_PATTERN,
				 ARRAY_SIZE(sc535hai_test_pattern_menu) - 1,
				 0, 0, sc535hai_test_pattern_menu);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc535hai->client->dev,
			"44 Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	v4l2_ctrl_new_std(handler, &sc535hai_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc535hai->client->dev,
			"55 Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	v4l2_ctrl_new_std(handler, &sc535hai_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc535hai->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc535hai->subdev.ctrl_handler = handler;
	sc535hai->has_init_exp = false;
	sc535hai->cur_fps = mode->max_fps;
	sc535hai->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc535hai_check_sensor_id(struct sc535hai *sc535hai,
				    struct i2c_client *client)
{
	struct device *dev = &sc535hai->client->dev;
	u32 id = 0, flag = 0;
	int ret;

	if (sc535hai->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = sc535hai_read_reg(client, SC535HAI_REG_CHIP_ID,
				SC535HAI_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	/* read identifier id flag 0x8037*/
	ret |= sc535hai_read_reg(client, SC535HAI_REG_CHIP_FLG,
				 SC535HAI_REG_VALUE_08BIT, &flag);
	if (flag != 0) {
		dev_err(dev, "Unexpected id flag(0x%02x), ret(%d)\n", flag, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC535HAI (0x%04x) sensor\n", CHIP_ID);

	return 0;
}

static int sc535hai_configure_regulators(struct sc535hai *sc535hai)
{
	unsigned int i;

	for (i = 0; i < SC535HAI_NUM_SUPPLIES; i++)
		sc535hai->supplies[i].supply = sc535hai_supply_names[i];

	return devm_regulator_bulk_get(&sc535hai->client->dev,
				       SC535HAI_NUM_SUPPLIES,
				       sc535hai->supplies);
}


#ifdef CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP
static void find_terminal_resolution(struct sc535hai *sc535hai)
{
	int i = 0;
	const struct sc535hai_mode *mode = NULL;
	u32 rk_cam_hdr = get_rk_cam_hdr();
	u32 rk_cam_w = get_rk_cam_w();
	u32 rk_cam_h = get_rk_cam_h();

	if (rk_cam_w == 0 || rk_cam_h == 0)
		goto err_find_res;

	for (i = 0; i < sc535hai->cfg_num; i++) {
		mode = &sc535hai->supported_modes[i];
		if (mode->width == rk_cam_w && mode->height == rk_cam_h &&
		    mode->hdr_mode == rk_cam_hdr) {
			sc535hai->cur_mode = mode;
			return;
		}
	}
err_find_res:
	dev_err(&sc535hai->client->dev, "not match %dx%d mode %d\n!",
		rk_cam_w, rk_cam_h, rk_cam_hdr);
	sc535hai->cur_mode = &sc535hai->supported_modes[0];
}
#else
static void find_terminal_resolution(struct sc535hai *sc535hai)
{
	u32 hdr_mode = 0;
	struct device_node *node = sc535hai->client->dev.of_node;
	int i = 0;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	for (i = 0; i < sc535hai->cfg_num; i++) {
		if (hdr_mode == sc535hai->supported_modes[i].hdr_mode) {
			sc535hai->cur_mode = &sc535hai->supported_modes[i];
			break;
		}
	}
	if (i == sc535hai->cfg_num)
		sc535hai->cur_mode = &sc535hai->supported_modes[0];
}
#endif

static int sc535hai_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc535hai *sc535hai;
	struct v4l2_subdev *sd;
	struct device_node *endpoint;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc535hai = devm_kzalloc(dev, sizeof(*sc535hai), GFP_KERNEL);
	if (!sc535hai)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc535hai->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc535hai->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc535hai->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc535hai->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	/* Compatible with non-standby mode if this attribute is not configured in dts*/
	of_property_read_u32(node, RKMODULE_CAMERA_STANDBY_HW,
			     &sc535hai->standby_hw);
	dev_info(dev, "sc535hai->standby_hw = %d\n", sc535hai->standby_hw);

	sc535hai->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &sc535hai->bus_cfg);
	of_node_put(endpoint);
	if (ret) {
		dev_err(dev, "Failed to get bus config\n");
		return -EINVAL;
	}

	if (sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes == 4) {
		sc535hai->supported_modes = supported_modes_4lane;
		sc535hai->cfg_num = ARRAY_SIZE(supported_modes_4lane);
		dev_info(dev, "detect sc535hai lane: %d\n",
			 sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes);
	} else {
		sc535hai->supported_modes = supported_modes_2lane;
		sc535hai->cfg_num = ARRAY_SIZE(supported_modes_2lane);
		dev_info(dev, "detect sc535hai lane: %d\n",
			 sc535hai->bus_cfg.bus.mipi_csi2.num_data_lanes);
	}

	sc535hai->client = client;
	find_terminal_resolution(sc535hai);
	sc535hai->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc535hai->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	if (!sc535hai->is_thunderboot)
		sc535hai->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	else
		sc535hai->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(sc535hai->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	if (!sc535hai->is_thunderboot)
		sc535hai->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	else
		sc535hai->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
	if (IS_ERR(sc535hai->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc535hai->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc535hai->pinctrl)) {
		sc535hai->pins_default =
			pinctrl_lookup_state(sc535hai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc535hai->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc535hai->pins_sleep =
			pinctrl_lookup_state(sc535hai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc535hai->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc535hai_configure_regulators(sc535hai);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc535hai->mutex);

	sd = &sc535hai->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc535hai_subdev_ops);
	ret = sc535hai_initialize_controls(sc535hai);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc535hai_power_on(sc535hai);
	if (ret)
		goto err_free_handler;

	ret = sc535hai_check_sensor_id(sc535hai, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc535hai_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc535hai->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc535hai->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	if (!sc535hai->cam_sw_inf) {
		sc535hai->cam_sw_inf = cam_sw_init();
		cam_sw_clk_init(sc535hai->cam_sw_inf, sc535hai->xvclk,
				sc535hai->cur_mode->mclk);
		cam_sw_reset_pin_init(sc535hai->cam_sw_inf, sc535hai->reset_gpio, 0);
		cam_sw_pwdn_pin_init(sc535hai->cam_sw_inf, sc535hai->pwdn_gpio, 1);
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc535hai->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc535hai->module_index, facing,
		 SC535HAI_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (sc535hai->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc535hai_power_off(sc535hai);
err_free_handler:
	v4l2_ctrl_handler_free(&sc535hai->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc535hai->mutex);

	return ret;
}

static int sc535hai_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hai *sc535hai = to_sc535hai(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc535hai->ctrl_handler);
	mutex_destroy(&sc535hai->mutex);

	cam_sw_deinit(sc535hai->cam_sw_inf);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc535hai_power_off(sc535hai);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc535hai_of_match[] = {
	{ .compatible = "smartsens,sc535hai" },
	{},
};
MODULE_DEVICE_TABLE(of, sc535hai_of_match);
#endif

static const struct i2c_device_id sc535hai_match_id[] = {
	{ "smartsens,sc535hai", 0 },
	{ },
};

static struct i2c_driver sc535hai_i2c_driver = {
	.driver = {
		.name = SC535HAI_NAME,
		.pm = &sc535hai_pm_ops,
		.of_match_table = of_match_ptr(sc535hai_of_match),
	},
	.probe		= &sc535hai_probe,
	.remove		= &sc535hai_remove,
	.id_table	= sc535hai_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc535hai_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc535hai_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens sc535hai sensor driver");
MODULE_LICENSE("GPL");
