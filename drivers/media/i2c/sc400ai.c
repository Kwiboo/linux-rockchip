// SPDX-License-Identifier: GPL-2.0
/*
 * sc400ai driver
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first implement, support hdr.
 * V0.0X01.0X01 add power on function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 fix gain range.
 * V0.0X01.0X04 add enum_frame_interval function.
 * V0.0X01.0X05 add quick stream on/off
 * V0.0X01.0X06 support thunder boot function.
 * V0.0X01.0X07 support HDR2 function.
 */

// #define DEBUG

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x07)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SC400AI_BITS_PER_SAMPLE		10

#define SC400AI_LINK_FREQ_315		((s64)157500000)// 315Mbps
#define SC400AI_LINK_FREQ_630		((s64)315000000)// 630Mbps
#define SC400AI_LINK_FREQ_1080		((s64)540000000)// 1080Mbps

#define PIXEL_RATE_WITH_315M_10BIT	(SC400AI_LINK_FREQ_315 * 2 * \
					4 / SC400AI_BITS_PER_SAMPLE)
#define PIXEL_RATE_WITH_630M_10BIT	(SC400AI_LINK_FREQ_630 * 2 * \
					2 / SC400AI_BITS_PER_SAMPLE)
#define PIXEL_RATE_WITH_1080M_10BIT	(SC400AI_LINK_FREQ_1080 * 2 * \
					2 / SC400AI_BITS_PER_SAMPLE)

#define PIXEL_RATE_WITH_MAX		(SC400AI_LINK_FREQ_630 * 2 * \
					2 / SC400AI_BITS_PER_SAMPLE)

#define SC400AI_XVCLK_FREQ		27000000

#define CHIP_ID				0xcd2e
#define SC400AI_REG_CHIP_ID		0x3107

#define SC400AI_REG_CTRL_MODE		0x0100
#define SC400AI_MODE_SW_STANDBY		0x0
#define SC400AI_MODE_STREAMING		BIT(0)

#define	SC400AI_EXPOSURE_MIN		3	// Half line exposure time
#define	SC400AI_EXPOSURE_STEP		1	// Half line exposure time

/* HDR long exposure time */
#define SC400AI_REG_EXPOSURE_H		0x3e00
#define SC400AI_REG_EXPOSURE_M		0x3e01
#define SC400AI_REG_EXPOSURE_L		0x3e02
/* HDR short exposure time*/
#define SC400AI_REG_SEXPOSURE_M		0x3e04
#define SC400AI_REG_SEXPOSURE_L		0x3e05

#define SC400AI_HDR_EXPOSURE_MIN	5	// Half line exposure time
#define SC400AI_HDR_EXPOSURE_STEP	4	// Half line exposure time

#define SC400AI_VTS_MAX			0x7fff

#define SC400AI_MAX_LONG_EXPOSURE	5627	// 2*{16’h320e,16’h320f} -
						// 2*{16’h3e23,16’h3e24} - ‘d13
#define SC400AI_MAX_SHORT_EXPOSURE	350	// 2*{16’h3e23,16’h3e24} - ‘d10

/* Linear / HDR long exposure */
#define SC400AI_REG_DIG_GAIN		0x3e06
#define SC400AI_REG_DIG_FINE_GAIN	0x3e07
#define SC400AI_REG_ANA_GAIN		0x3e08
#define SC400AI_REG_ANA_FINE_GAIN	0x3e09

/* HDR short exposure */
#define SC400AI_REG_SDIG_GAIN		0x3e10
#define SC400AI_REG_SDIG_FINE_GAIN	0x3e11
#define SC400AI_REG_SANA_GAIN		0x3e12
#define SC400AI_REG_SANA_FINE_GAIN	0x3e13

#define SC400AI_GAIN_MIN		0x0040
#define SC400AI_GAIN_MAX		(24 * 32 * 64)    //23.32*31.75*64
#define SC400AI_GAIN_STEP		1
#define SC400AI_GAIN_DEFAULT		0x0800

/* Linear / HDR long frame mode */
#define SC400AI_LGAIN			0
/* HDR short frame mode */
#define SC400AI_SGAIN			1

#define SC400AI_REG_GROUP_HOLD		0x3812
#define SC400AI_GROUP_HOLD_START	0x00
#define SC400AI_GROUP_HOLD_END		0x30

#define SC400AI_REG_HIGH_TEMP_H		0x3974
#define SC400AI_REG_HIGH_TEMP_L		0x3975

/* test parttern */
#define SC400AI_REG_TEST_PATTERN	0x4501
#define SC400AI_TEST_PATTERN_BIT_MASK	BIT(3)	//testpattern , bit 3 to enable

/* VTS reg */
#define SC400AI_REG_VTS_H		0x320e
#define SC400AI_REG_VTS_L		0x320f

#define SC400AI_FLIP_MIRROR_REG		0x3221

#define SC400AI_FETCH_EXP_H(VAL)	(((VAL) >> 12) & 0xF)
#define SC400AI_FETCH_EXP_M(VAL)	(((VAL) >> 4) & 0xFF)
#define SC400AI_FETCH_EXP_L(VAL)	(((VAL) & 0xF) << 4)

#define SC400AI_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define SC400AI_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define SC400AI_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x06 : VAL & 0xf9)
#define SC400AI_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x60 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define SC400AI_REG_VALUE_08BIT		1
#define SC400AI_REG_VALUE_16BIT		2
#define SC400AI_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define SC400AI_NAME			"sc400ai"

static const char * const sc400ai_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC400AI_NUM_SUPPLIES ARRAY_SIZE(sc400ai_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc400ai_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 bpp;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 mipi_freq_idx;
	u32 vc[PAD_MAX];
};

struct sc400ai {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC400AI_NUM_SUPPLIES];

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
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct mutex		mutex;
	struct v4l2_fract	cur_fps;
	bool			streaming;
	bool			power_on;
	unsigned int		lane_num;
	unsigned int		cfg_num;
	const struct sc400ai_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_sc400ai(sd) container_of(sd, struct sc400ai, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval sc400ai_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * linear
 * mipi_datarate per lane 315Mbps, 4lane
 */
static const struct regval sc400ai_linear_10_2560x1440_4lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x301c, 0x78},
	{0x301f, 0x01},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x05},
	{0x320b, 0xa0},
	{0x320e, 0x05},
	{0x320f, 0xdc},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0x80},
	{0x3250, 0x00},
	{0x3253, 0x08},
	{0x3274, 0x01},
	{0x3301, 0x20},
	{0x3302, 0x18},
	{0x3303, 0x10},
	{0x3304, 0x50},
	{0x3306, 0x38},
	{0x3308, 0x18},
	{0x3309, 0x60},
	{0x330b, 0xc0},
	{0x330d, 0x10},
	{0x330e, 0x18},
	{0x330f, 0x04},
	{0x3310, 0x02},
	{0x331c, 0x04},
	{0x331e, 0x41},
	{0x331f, 0x51},
	{0x3320, 0x09},
	{0x3333, 0x10},
	{0x334c, 0x08},
	{0x3356, 0x09},
	{0x3364, 0x17},
	{0x338e, 0xfd},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x20},
	{0x3394, 0x20},
	{0x3395, 0x20},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x20},
	{0x339a, 0x20},
	{0x339b, 0x20},
	{0x339c, 0x20},
	{0x33ac, 0x10},
	{0x33ae, 0x18},
	{0x33af, 0x19},
	{0x360f, 0x01},
	{0x3620, 0x08},
	{0x3637, 0x25},
	{0x363a, 0x12},
	{0x3670, 0x0a},
	{0x3671, 0x07},
	{0x3672, 0x57},
	{0x3673, 0x5e},
	{0x3674, 0x84},
	{0x3675, 0x88},
	{0x3676, 0x8a},
	{0x367a, 0x58},
	{0x367b, 0x78},
	{0x367c, 0x58},
	{0x367d, 0x78},
	{0x3690, 0x33},
	{0x3691, 0x43},
	{0x3692, 0x34},
	{0x369c, 0x40},
	{0x369d, 0x78},
	{0x36ea, 0x39},
	{0x36eb, 0x0d},
	{0x36ec, 0x2c},
	{0x36ed, 0x24},
	{0x36fa, 0x39},
	{0x36fb, 0x33},
	{0x36fc, 0x10},
	{0x36fd, 0x14},
	{0x3908, 0x41},
	{0x396c, 0x0e},
	{0x3e00, 0x00},
	{0x3e01, 0xb6},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e1b, 0x2a},
	{0x4509, 0x30},
	{0x57a8, 0xd0},
	{0x36e9, 0x14},
	{0x36f9, 0x14},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * linear
 * mipi_datarate per lane 630Mbps, 2lane
 */
static const struct regval sc400ai_linear_10_2560x1440_2lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3018, 0x3a},
	{0x3019, 0x0c},
	{0x301c, 0x78},
	{0x301f, 0x05},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x05},
	{0x320b, 0xa0},
	{0x320e, 0x05},	// 1500
	{0x320f, 0xdc},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0x80},
	{0x3250, 0x00},
	{0x3253, 0x08},
	{0x3274, 0x01},
	{0x3301, 0x20},
	{0x3302, 0x18},
	{0x3303, 0x10},
	{0x3304, 0x50},
	{0x3306, 0x38},
	{0x3308, 0x18},
	{0x3309, 0x60},
	{0x330b, 0xc0},
	{0x330d, 0x10},
	{0x330e, 0x18},
	{0x330f, 0x04},
	{0x3310, 0x02},
	{0x331c, 0x04},
	{0x331e, 0x41},
	{0x331f, 0x51},
	{0x3320, 0x09},
	{0x3333, 0x10},
	{0x334c, 0x08},
	{0x3356, 0x09},
	{0x3364, 0x17},
	{0x338e, 0xfd},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x20},
	{0x3394, 0x20},
	{0x3395, 0x20},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x20},
	{0x339a, 0x20},
	{0x339b, 0x20},
	{0x339c, 0x20},
	{0x33ac, 0x10},
	{0x33ae, 0x18},
	{0x33af, 0x19},
	{0x360f, 0x01},
	{0x3620, 0x08},
	{0x3637, 0x25},
	{0x363a, 0x12},
	{0x3670, 0x0a},
	{0x3671, 0x07},
	{0x3672, 0x57},
	{0x3673, 0x5e},
	{0x3674, 0x84},
	{0x3675, 0x88},
	{0x3676, 0x8a},
	{0x367a, 0x58},
	{0x367b, 0x78},
	{0x367c, 0x58},
	{0x367d, 0x78},
	{0x3690, 0x33},
	{0x3691, 0x43},
	{0x3692, 0x34},
	{0x369c, 0x40},
	{0x369d, 0x78},
	{0x36ea, 0x39},
	{0x36eb, 0x0d},
	{0x36ec, 0x1c},
	{0x36ed, 0x24},
	{0x36fa, 0x39},
	{0x36fb, 0x33},
	{0x36fc, 0x10},
	{0x36fd, 0x14},
	{0x3908, 0x41},
	{0x396c, 0x0e},
	{0x3e00, 0x00},
	{0x3e01, 0xb6},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e1b, 0x2a},
	{0x4509, 0x30},
	{0x4819, 0x08},
	{0x481b, 0x05},
	{0x481d, 0x11},
	{0x481f, 0x04},
	{0x4821, 0x09},
	{0x4823, 0x04},
	{0x4825, 0x04},
	{0x4827, 0x04},
	{0x4829, 0x07},
	{0x57a8, 0xd0},
	{0x36e9, 0x14},
	{0x36f9, 0x14},
	//{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 25fps
 * mipi_datarate per lane 1080Mbps, 2lane
 * hdrx2
 * line time  = 1 / (fps * frame_len) = 1 / (25 * 0x5dc) = 2.666us
 */
// cleaned_0x78_SC400AI_MIPI_27Minput_2lane_1080Mbps_10bit_2560x1440_25fps_SHDR.ini
static const struct regval __maybe_unused sc400ai_hdr_10_2560x1440_2lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3018, 0x3a},
	{0x3019, 0x0c},
	{0x301c, 0x78},
	{0x301f, 0x78},
	{0x3106, 0x01},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x05},
	{0x320b, 0xa0},
	{0x320c, 0x05},
	{0x320d, 0xa0},
	// {0x320e, 0x0b},	// 3000
	// {0x320f, 0xb8},
	{0x320e, 0x0c},	// 3084
	{0x320f, 0x0e},
	// {0x320e, 0x07},	// 1800
	// {0x320f, 0x08},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3220, 0x53},
	{0x3223, 0x80},
	{0x3250, 0xff},
	{0x3253, 0x08},
	{0x3274, 0x01},
	{0x3301, 0x08},
	{0x3302, 0x18},
	{0x3303, 0x10},
	{0x3304, 0x70},
	{0x3306, 0x40},
	{0x3308, 0x10},
	{0x3309, 0x70},
	{0x330b, 0xb0},
	{0x330d, 0x20},
	{0x330e, 0x20},
	{0x330f, 0x04},
	{0x3310, 0x02},
	{0x331c, 0x08},
	{0x331e, 0x61},
	{0x331f, 0x61},
	{0x3320, 0x0f},
	{0x3333, 0x10},
	{0x334c, 0x10},
	{0x3356, 0x0f},
	{0x3364, 0x17},
	{0x338e, 0xfd},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x08},
	{0x3394, 0x10},
	{0x3395, 0x20},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x08},
	{0x339a, 0x10},
	{0x339b, 0x20},
	{0x339c, 0x20},
	{0x33ac, 0x15},
	{0x33ae, 0x1f},
	{0x33af, 0x1f},
	{0x3415, 0x42},
	{0x360f, 0x01},
	{0x3620, 0x08},
	{0x3637, 0x4a},
	{0x363a, 0x12},
	{0x3651, 0x7f},	/* mipi hs driver strength, default:0x7d*/
	{0x3670, 0x0a},
	{0x3671, 0x07},
	{0x3672, 0x07},
	{0x3673, 0x57},
	{0x3674, 0x74},
	{0x3675, 0x78},
	{0x3676, 0x7a},
	{0x367a, 0x48},
	{0x367b, 0x58},
	{0x367c, 0x58},
	{0x367d, 0x78},
	{0x3690, 0x45},
	{0x3691, 0x35},
	{0x3692, 0x36},
	{0x369c, 0x40},
	{0x369d, 0x48},
	{0x36ea, 0x36},
	{0x36eb, 0x0c},
	{0x36ec, 0x0c},
	{0x36ed, 0x14},
	{0x36fa, 0x36},
	{0x36fb, 0x04},
	{0x36fc, 0x00},
	{0x36fd, 0x14},
	{0x3908, 0x41},
	{0x391f, 0x10},
	{0x396c, 0x0e},
	{0x3e00, 0x01},
	{0x3e01, 0x5e},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e04, 0x15},
	{0x3e05, 0xe0},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e10, 0x00},
	{0x3e11, 0x80},
	{0x3e12, 0x03},
	{0x3e13, 0x40},
	{0x3e1b, 0x2a},
	{0x3e23, 0x00},
	{0x3e24, 0xb4},
	{0x440e, 0x02},
	{0x4509, 0x30},
	{0x4816, 0x71},
	{0x4819, 0x0d},
	{0x481b, 0x07},
	{0x481d, 0x1d},
	{0x481f, 0x06},
	{0x4821, 0x0c},
	{0x4823, 0x07},
	{0x4825, 0x06},
	{0x4827, 0x06},
	{0x4829, 0x0b},
	{0x5001, 0x44},
	{0x5011, 0x80},
	{0x57a8, 0xd0},
	{0x36e9, 0x24},
	{0x36f9, 0x20},
	//{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 15fps
 * mipi_datarate per lane 630Mbps, 2lane
 */
static const struct regval sc400ai_hdr_15fps_10_2560x1440_2lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3018, 0x3a},
	{0x3019, 0x0c},
	{0x301c, 0x78},
	{0x301f, 0x44},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x05},
	{0x320b, 0xa0},
	{0x320e, 0x0b},
	{0x320f, 0xb8},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3220, 0x53},
	{0x3223, 0x80},
	{0x3250, 0xff},
	{0x3253, 0x08},
	{0x3274, 0x01},
	{0x3301, 0x20},
	{0x3302, 0x18},
	{0x3303, 0x10},
	{0x3304, 0x50},
	{0x3306, 0x38},
	{0x3308, 0x18},
	{0x3309, 0x60},
	{0x330b, 0xc0},
	{0x330d, 0x10},
	{0x330e, 0x18},
	{0x330f, 0x04},
	{0x3310, 0x02},
	{0x331c, 0x04},
	{0x331e, 0x41},
	{0x331f, 0x51},
	{0x3320, 0x09},
	{0x3333, 0x10},
	{0x334c, 0x08},
	{0x3356, 0x09},
	{0x3364, 0x17},
	{0x338e, 0xfd},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x20},
	{0x3394, 0x20},
	{0x3395, 0x20},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x20},
	{0x339a, 0x20},
	{0x339b, 0x20},
	{0x339c, 0x20},
	{0x33ac, 0x10},
	{0x33ae, 0x18},
	{0x33af, 0x19},
	{0x360f, 0x01},
	{0x3620, 0x08},
	{0x3637, 0x25},
	{0x363a, 0x12},
	{0x3670, 0x0a},
	{0x3671, 0x07},
	{0x3672, 0x57},
	{0x3673, 0x5e},
	{0x3674, 0x84},
	{0x3675, 0x88},
	{0x3676, 0x8a},
	{0x367a, 0x58},
	{0x367b, 0x78},
	{0x367c, 0x58},
	{0x367d, 0x78},
	{0x3690, 0x33},
	{0x3691, 0x43},
	{0x3692, 0x34},
	{0x369c, 0x40},
	{0x369d, 0x78},
	{0x36ea, 0x32},
	{0x36eb, 0x0d},
	{0x36ec, 0x1c},
	{0x36ed, 0x24},
	{0x36fa, 0x32},
	{0x36fb, 0x33},
	{0x36fc, 0x10},
	{0x36fd, 0x14},
	{0x3908, 0x41},
	{0x396c, 0x0e},
	{0x3e00, 0x01},
	{0x3e01, 0x5e},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e04, 0x15},
	{0x3e05, 0xe0},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e10, 0x00},
	{0x3e11, 0x80},
	{0x3e12, 0x03},
	{0x3e13, 0x40},
	{0x3e1b, 0x2a},
	{0x3e23, 0x00},
	{0x3e24, 0xb4},
	{0x440e, 0x02},
	{0x4509, 0x30},
	{0x4816, 0x71},
	{0x4819, 0x08},
	{0x481b, 0x05},
	{0x481d, 0x11},
	{0x481f, 0x04},
	{0x4821, 0x09},
	{0x4823, 0x04},
	{0x4825, 0x04},
	{0x4827, 0x04},
	{0x4829, 0x07},
	{0x5001, 0x44},
	{0x5011, 0x80},
	{0x57a8, 0xd0},
	{0x36e9, 0x44},
	{0x36f9, 0x44},
	// {0x0100, 0x01},
	{REG_NULL, 0x00},
};
static const struct sc400ai_mode supported_modes[] = {
	{
		.width = 2560,
		.height = 1440,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0080,
		.hts_def = 0x0578 * 2,
		.vts_def = 0x05dc,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = sc400ai_linear_10_2560x1440_4lane_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.width = 2560,
		.height = 1440,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0080,
		.hts_def = 0x0578 * 2,
		.vts_def = 0x05dc,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = sc400ai_linear_10_2560x1440_2lane_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	// 630Mbps 15fps 2lane hdr
	{
		.width = 2560,
		.height = 1440,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x0080,
		.hts_def = 0x0578 * 2,
		.vts_def = 0xbb8,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = sc400ai_hdr_15fps_10_2560x1440_2lane_regs,
		.mipi_freq_idx = 1,
		.bpp = 10,
		.hdr_mode = HDR_X2,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
	},
};

static const s64 link_freq_menu_items[] = {
	SC400AI_LINK_FREQ_315,
	SC400AI_LINK_FREQ_630,
	SC400AI_LINK_FREQ_1080,
};

static const char * const sc400ai_test_pattern_menu[] = {
	"Disabled",
	"Vertical Grey Color Bar",
};

static int __sc400ai_power_on(struct sc400ai *sc400ai);

/* Write registers up to 4 at a time */
static int sc400ai_write_reg(struct i2c_client *client, u16 reg,
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

static int sc400ai_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = sc400ai_write_reg(client, regs[i].addr,
					SC400AI_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int sc400ai_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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
	if (ret != ARRAY_SIZE(msgs)) {
		dev_info(&client->dev, "%s: ret = %d\n", __func__, ret);
		return -EIO;
	}

	*val = be32_to_cpu(data_be);

	return 0;
}

static int sc400ai_set_hightemp_dpc(struct sc400ai *sc400ai, int mode, u32 total_gain)
{
	int ret = 0;

	if (mode == SC400AI_LGAIN) {
		if (total_gain <= 23879 * 16) {
			ret = sc400ai_write_reg(sc400ai->client, 0x5000,
						SC400AI_REG_VALUE_08BIT, 0x04);
		} else if (total_gain > 23879 * 16) {
			ret = sc400ai_write_reg(sc400ai->client, 0x5000,
						SC400AI_REG_VALUE_08BIT, 0x02);
		}
	} else if (mode == SC400AI_SGAIN) {
		if (total_gain <= 23879 * 16) {
			ret = sc400ai_write_reg(sc400ai->client, 0x5002,
						SC400AI_REG_VALUE_08BIT, 0x04);
		} else if (total_gain > 23879 * 16) {
			ret = sc400ai_write_reg(sc400ai->client, 0x5002,
						SC400AI_REG_VALUE_08BIT, 0x02);
		}
	}
	return ret;
}

static int sc400ai_set_gain_reg(struct sc400ai *sc400ai, u32 gain, int mode)
{
	u8 Coarse_gain = 1, DIG_gain = 1;
	u32 Dcg_gainx100 = 1, ANA_Fine_gainx64 = 1;
	u8 Coarse_gain_reg = 0, DIG_gain_reg = 0;
	u8 ANA_Fine_gain_reg = 0x20, DIG_Fine_gain_reg = 0x80;
	int ret = 0;

	gain = gain << 4;
	if (gain <= 1024)
		gain = 1024;
	else if (gain > SC400AI_GAIN_MAX * 16)
		gain = SC400AI_GAIN_MAX * 16;

	if (gain < 1504) {	// start again,1.469 * 1024
		Dcg_gainx100 = 1000;
		Coarse_gain = 1;
		DIG_gain = 1;
		Coarse_gain_reg = 0x03;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 3008) {	// 2.938 * 1024
		Dcg_gainx100 = 1469;
		Coarse_gain = 1;
		DIG_gain = 1;
		Coarse_gain_reg = 0x23;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 6017) {	// 5.876 * 1024
		Dcg_gainx100 = 1469;
		Coarse_gain = 2;
		DIG_gain = 1;
		Coarse_gain_reg = 0x27;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 12034) {	// 11.752 * 1024
		Dcg_gainx100 = 1469;
		Coarse_gain = 4;
		DIG_gain = 1;
		Coarse_gain_reg = 0x2f;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 23879) {	// 23.320 * 1024 // end again
		Dcg_gainx100 = 1469;
		Coarse_gain = 8;
		DIG_gain = 1;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain < 23879 * 2) {         // start dgain
		Dcg_gainx100 = 1469;
		Coarse_gain = 8;
		DIG_gain = 1;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x0;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain < 23879 * 4) {
		Dcg_gainx100 = 1469;
		Coarse_gain = 8;
		DIG_gain = 2;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x1;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain < 23879 * 8) {
		Dcg_gainx100 = 1469;
		Coarse_gain = 8;
		DIG_gain = 4;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x3;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain < 23879 * 16) {
		Dcg_gainx100 = 1469;
		Coarse_gain = 8;
		DIG_gain = 8;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x7;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain <= 782466) {
		Dcg_gainx100 = 1469;
		Coarse_gain = 8;
		DIG_gain = 16;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0xF;
		ANA_Fine_gain_reg = 0x7f;
	}

	if (gain < 1504)
		ANA_Fine_gain_reg = abs(100 * gain / (Dcg_gainx100 * Coarse_gain) / 16);
	else if (gain == 1504)
		ANA_Fine_gain_reg = 0x5d;
	else if (gain < 23879)
		ANA_Fine_gain_reg = abs(100 * gain / (Dcg_gainx100 * Coarse_gain) / 16);
	else
		DIG_Fine_gain_reg = abs(800 * gain / (Dcg_gainx100 * Coarse_gain *
							DIG_gain) / ANA_Fine_gainx64);

	if (sc400ai->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
		sc400ai->is_thunderboot = false;
		__sc400ai_power_on(sc400ai);
	}

	dev_info(&sc400ai->client->dev,
		"%s: mode=%d, Coarse_gain_reg:%d ANA_Fine_gain_reg:%d DIG_gain_reg:%d DIG_Fine_gain_reg:%d\n",
		__func__, mode,
		Coarse_gain_reg, ANA_Fine_gain_reg,
		DIG_gain_reg, DIG_Fine_gain_reg);
	if (mode == SC400AI_LGAIN) {
		ret = sc400ai_write_reg(sc400ai->client,
					SC400AI_REG_DIG_GAIN,
					SC400AI_REG_VALUE_08BIT,
					DIG_gain_reg & 0xF);
		ret |= sc400ai_write_reg(sc400ai->client,
					SC400AI_REG_DIG_FINE_GAIN,
					SC400AI_REG_VALUE_08BIT,
					DIG_Fine_gain_reg);
		ret |= sc400ai_write_reg(sc400ai->client,
					SC400AI_REG_ANA_GAIN,
					SC400AI_REG_VALUE_08BIT,
					Coarse_gain_reg);
		ret |= sc400ai_write_reg(sc400ai->client,
					SC400AI_REG_ANA_FINE_GAIN,
					SC400AI_REG_VALUE_08BIT,
					ANA_Fine_gain_reg);
	} else { // hdr short gain
		ret = sc400ai_write_reg(sc400ai->client,
					SC400AI_REG_SDIG_GAIN,
					SC400AI_REG_VALUE_08BIT,
					DIG_gain_reg & 0xF);
		ret |= sc400ai_write_reg(sc400ai->client,
					 SC400AI_REG_SDIG_FINE_GAIN,
					 SC400AI_REG_VALUE_08BIT,
					 DIG_Fine_gain_reg);
		ret |= sc400ai_write_reg(sc400ai->client,
					 SC400AI_REG_SANA_GAIN,
					 SC400AI_REG_VALUE_08BIT,
					 Coarse_gain_reg);
		ret |= sc400ai_write_reg(sc400ai->client,
					 SC400AI_REG_SANA_FINE_GAIN,
					 SC400AI_REG_VALUE_08BIT,
					 ANA_Fine_gain_reg);

	}
	sc400ai_set_hightemp_dpc(sc400ai, mode, gain);

	return ret;
}

static int sc400ai_set_hdrae(struct sc400ai *sc400ai,
			    struct preisp_hdrae_exp_s *ae)
{
	int ret = 0;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;

	if (!sc400ai->has_init_exp && !sc400ai->streaming) {
		sc400ai->init_hdrae_exp = *ae;
		sc400ai->has_init_exp = true;
		dev_dbg(&sc400ai->client->dev, "sc400ai don't stream, record exp for hdr!\n");
		return ret;
	}

	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;

	dev_info(&sc400ai->client->dev,
		"L_exp: 0x%x, M_exp: 0x%x, S_exp: 0x%x, L_tg: 0x%x, M_tg: 0x%x, S_tg: 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_a_gain, m_a_gain, s_a_gain);

	if (sc400ai->cur_mode->hdr_mode == HDR_X2) {
		//2 stagger
		l_exp_time = m_exp_time;
		l_a_gain = m_a_gain;
	}

	//set exposure
	l_exp_time = l_exp_time << 1;
	s_exp_time = s_exp_time << 1;
	if (s_a_gain != l_a_gain)
		dev_err(&sc400ai->client->dev,
			"line mode: LF and SF gains must be equal, l_a_gain: 0x%x, s_a_gain: 0x%x\n",
			l_a_gain, s_a_gain);
	if (l_exp_time > SC400AI_MAX_LONG_EXPOSURE)		// (2*0xbb8 - 2*0xb4 - 13)
		l_exp_time = SC400AI_MAX_LONG_EXPOSURE;
	if (s_exp_time > SC400AI_MAX_SHORT_EXPOSURE)		// (2*0xb4 - 10)
		s_exp_time = SC400AI_MAX_LONG_EXPOSURE;

	/* LE */
	ret = sc400ai_write_reg(sc400ai->client,
				SC400AI_REG_EXPOSURE_H,
				SC400AI_REG_VALUE_08BIT,
				SC400AI_FETCH_EXP_H(l_exp_time));
	ret |= sc400ai_write_reg(sc400ai->client,
				 SC400AI_REG_EXPOSURE_M,
				 SC400AI_REG_VALUE_08BIT,
				 SC400AI_FETCH_EXP_M(l_exp_time));
	ret |= sc400ai_write_reg(sc400ai->client,
				 SC400AI_REG_EXPOSURE_L,
				 SC400AI_REG_VALUE_08BIT,
				 SC400AI_FETCH_EXP_L(l_exp_time));
	/* SE */
	ret |= sc400ai_write_reg(sc400ai->client,
				 SC400AI_REG_SEXPOSURE_M,
				 SC400AI_REG_VALUE_08BIT,
				 SC400AI_FETCH_EXP_M(s_exp_time));
	ret |= sc400ai_write_reg(sc400ai->client,
				 SC400AI_REG_SEXPOSURE_L,
				 SC400AI_REG_VALUE_08BIT,
				 SC400AI_FETCH_EXP_L(s_exp_time));

	ret |= sc400ai_set_gain_reg(sc400ai, l_a_gain, SC400AI_LGAIN);
	ret |= sc400ai_set_gain_reg(sc400ai, s_a_gain, SC400AI_SGAIN);
	return ret;
}

static int sc400ai_get_reso_dist(const struct sc400ai_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc400ai_mode *
sc400ai_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sc400ai_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sc400ai_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	const struct sc400ai_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sc400ai->mutex);

	mode = sc400ai_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc400ai->mutex);
		return -ENOTTY;
#endif
	} else {
		sc400ai->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc400ai->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc400ai->vblank, vblank_def,
					 SC400AI_VTS_MAX - mode->height,
					 1, vblank_def);
		sc400ai->cur_fps = mode->max_fps;
		sc400ai->cur_vts = mode->vts_def;
	}

	mutex_unlock(&sc400ai->mutex);

	return 0;
}

static int sc400ai_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	const struct sc400ai_mode *mode = sc400ai->cur_mode;

	mutex_lock(&sc400ai->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc400ai->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc400ai->mutex);

	return 0;
}

static int sc400ai_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sc400ai->cur_mode->bus_fmt;

	return 0;
}

static int sc400ai_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != supported_modes[1].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc400ai_enable_test_pattern(struct sc400ai *sc400ai, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc400ai_read_reg(sc400ai->client, SC400AI_REG_TEST_PATTERN,
			       SC400AI_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC400AI_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SC400AI_TEST_PATTERN_BIT_MASK;
	ret |= sc400ai_write_reg(sc400ai->client, SC400AI_REG_TEST_PATTERN,
				 SC400AI_REG_VALUE_08BIT, val);
	return ret;
}

static int sc400ai_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	const struct sc400ai_mode *mode = sc400ai->cur_mode;

	if (sc400ai->streaming)
		fi->interval = sc400ai->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int sc400ai_g_mbus_config(struct v4l2_subdev *sd,
				unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	const struct sc400ai_mode *mode = sc400ai->cur_mode;
	u32 val = 0;

	val = 1 << (sc400ai->lane_num - 1) |
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

static void sc400ai_get_module_inf(struct sc400ai *sc400ai,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC400AI_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc400ai->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc400ai->len_name, sizeof(inf->base.lens));
}

static int sc400ai_get_channel_info(struct sc400ai *sc400ai, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = sc400ai->cur_mode->vc[ch_info->index];
	ch_info->width = sc400ai->cur_mode->width;
	ch_info->height = sc400ai->cur_mode->height;
	ch_info->bus_fmt = sc400ai->cur_mode->bus_fmt;
	return 0;
}

static long sc400ai_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc400ai_get_module_inf(sc400ai, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc400ai->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = sc400ai->cur_mode->width;
		h = sc400ai->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				sc400ai->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&sc400ai->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = sc400ai->cur_mode->hts_def - sc400ai->cur_mode->width;
			h = sc400ai->cur_mode->vts_def - sc400ai->cur_mode->height;
			__v4l2_ctrl_modify_range(sc400ai->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc400ai->vblank, h,
						 SC400AI_VTS_MAX - sc400ai->cur_mode->height, 1, h);
			sc400ai->cur_fps = sc400ai->cur_mode->max_fps;
			sc400ai->cur_vts = sc400ai->cur_mode->vts_def;
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		if (sc400ai->cur_mode->hdr_mode == HDR_X2)
			ret = sc400ai_set_hdrae(sc400ai, arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = sc400ai_write_reg(sc400ai->client,
						SC400AI_REG_CTRL_MODE,
						SC400AI_REG_VALUE_08BIT,
						SC400AI_MODE_STREAMING);
		else
			ret = sc400ai_write_reg(sc400ai->client,
						SC400AI_REG_CTRL_MODE,
						SC400AI_REG_VALUE_08BIT,
						SC400AI_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = sc400ai_get_channel_info(sc400ai, ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc400ai_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc400ai_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -EFAULT;
		}

		sc400ai_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc400ai_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret)
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

		if (copy_from_user(hdr, up, sizeof(*hdr))) {
			kfree(hdr);
			return -EFAULT;
		}

		sc400ai_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdrae, up, sizeof(*hdrae))) {
			kfree(hdrae);
			return -EFAULT;
		}

		sc400ai_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		sc400ai_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc400ai_start_stream(struct sc400ai *sc400ai)
{
	int ret;
	int count = 5;

	if (!sc400ai->is_thunderboot) {
		ret = sc400ai_write_array(sc400ai->client, sc400ai->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&sc400ai->ctrl_handler);
		if (ret)
			return ret;
		if (sc400ai->has_init_exp && sc400ai->cur_mode->hdr_mode != NO_HDR) {
			ret = sc400ai_ioctl(&sc400ai->subdev, PREISP_CMD_SET_HDRAE_EXP,
				&sc400ai->init_hdrae_exp);
			if (ret) {
				dev_err(&sc400ai->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}
retry:
	ret = sc400ai_write_reg(sc400ai->client,
				SC400AI_REG_CTRL_MODE,
				SC400AI_REG_VALUE_08BIT,
				SC400AI_MODE_STREAMING);

	if (ret < 0 && count--) {
		usleep_range(1000, 2000);
		goto retry;
	}
	return ret;
}

static int __sc400ai_stop_stream(struct sc400ai *sc400ai)
{
	sc400ai->has_init_exp = false;
	if (sc400ai->is_thunderboot) {
		sc400ai->is_first_streamoff = true;
		pm_runtime_put(&sc400ai->client->dev);
	}

	return sc400ai_write_reg(sc400ai->client,
				 SC400AI_REG_CTRL_MODE,
				 SC400AI_REG_VALUE_08BIT,
				 SC400AI_MODE_SW_STANDBY);
}

static int sc400ai_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	struct i2c_client *client = sc400ai->client;
	int ret = 0;

	mutex_lock(&sc400ai->mutex);
	on = !!on;
	if (on == sc400ai->streaming)
		goto unlock_and_return;

	if (on) {
		if (sc400ai->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			sc400ai->is_thunderboot = false;
			__sc400ai_power_on(sc400ai);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc400ai_start_stream(sc400ai);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
		/* delay 10ms above after stream on */
		usleep_range(10000, 20000);
	} else {
		__sc400ai_stop_stream(sc400ai);
		pm_runtime_put(&client->dev);
	}

	sc400ai->streaming = on;

unlock_and_return:
	mutex_unlock(&sc400ai->mutex);

	return ret;
}

static int sc400ai_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	struct i2c_client *client = sc400ai->client;
	int ret = 0;

	mutex_lock(&sc400ai->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc400ai->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = sc400ai_write_array(sc400ai->client, sc400ai_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		sc400ai->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc400ai->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc400ai->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sc400ai_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SC400AI_XVCLK_FREQ / 1000 / 1000);
}

static int __sc400ai_power_on(struct sc400ai *sc400ai)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sc400ai->client->dev;

	if (!IS_ERR_OR_NULL(sc400ai->pins_default)) {
		ret = pinctrl_select_state(sc400ai->pinctrl,
					   sc400ai->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc400ai->xvclk, SC400AI_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sc400ai->xvclk) != SC400AI_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sc400ai->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (sc400ai->is_thunderboot)
		return 0;

	if (!IS_ERR(sc400ai->reset_gpio))
		gpiod_set_value_cansleep(sc400ai->reset_gpio, 0);

	ret = regulator_bulk_enable(SC400AI_NUM_SUPPLIES, sc400ai->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc400ai->reset_gpio))
		gpiod_set_value_cansleep(sc400ai->reset_gpio, 1);

	usleep_range(5000, 10000);
	if (!IS_ERR(sc400ai->pwdn_gpio))
		gpiod_set_value_cansleep(sc400ai->pwdn_gpio, 1);

	if (!IS_ERR(sc400ai->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sc400ai_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(sc400ai->xvclk);

	return ret;
}

static void __sc400ai_power_off(struct sc400ai *sc400ai)
{
	int ret;
	struct device *dev = &sc400ai->client->dev;

	if (sc400ai->is_thunderboot) {
		if (sc400ai->is_first_streamoff) {
			sc400ai->is_thunderboot = false;
			sc400ai->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(sc400ai->pwdn_gpio))
		gpiod_set_value_cansleep(sc400ai->pwdn_gpio, 0);
	clk_disable_unprepare(sc400ai->xvclk);
	if (!IS_ERR(sc400ai->reset_gpio))
		gpiod_set_value_cansleep(sc400ai->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(sc400ai->pins_sleep)) {
		ret = pinctrl_select_state(sc400ai->pinctrl,
					   sc400ai->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC400AI_NUM_SUPPLIES, sc400ai->supplies);
}

static int sc400ai_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc400ai *sc400ai = to_sc400ai(sd);

	return __sc400ai_power_on(sc400ai);
}

static int sc400ai_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc400ai *sc400ai = to_sc400ai(sd);

	__sc400ai_power_off(sc400ai);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc400ai_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc400ai *sc400ai = to_sc400ai(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc400ai_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc400ai->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc400ai->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc400ai_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops sc400ai_pm_ops = {
	SET_RUNTIME_PM_OPS(sc400ai_runtime_suspend,
			   sc400ai_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc400ai_internal_ops = {
	.open = sc400ai_open,
};
#endif

static const struct v4l2_subdev_core_ops sc400ai_core_ops = {
	.s_power = sc400ai_s_power,
	.ioctl = sc400ai_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc400ai_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc400ai_video_ops = {
	.s_stream = sc400ai_s_stream,
	.g_frame_interval = sc400ai_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc400ai_pad_ops = {
	.enum_mbus_code = sc400ai_enum_mbus_code,
	.enum_frame_size = sc400ai_enum_frame_sizes,
	.enum_frame_interval = sc400ai_enum_frame_interval,
	.get_fmt = sc400ai_get_fmt,
	.set_fmt = sc400ai_set_fmt,
	.get_mbus_config = sc400ai_g_mbus_config,
};

static const struct v4l2_subdev_ops sc400ai_subdev_ops = {
	.core	= &sc400ai_core_ops,
	.video	= &sc400ai_video_ops,
	.pad	= &sc400ai_pad_ops,
};

static void sc400ai_modify_fps_info(struct sc400ai *sc400ai)
{
	const struct sc400ai_mode *mode = sc400ai->cur_mode;

	sc400ai->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       sc400ai->cur_vts;
}

static int sc400ai_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc400ai *sc400ai = container_of(ctrl->handler,
					       struct sc400ai, ctrl_handler);
	struct i2c_client *client = sc400ai->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc400ai->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sc400ai->exposure,
					sc400ai->exposure->minimum, max,
					sc400ai->exposure->step,
					sc400ai->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		  // max expo line vts-4!
		if (sc400ai->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val * 2;
			/* 4 least significant bits of expsoure are fractional part */
			ret = sc400ai_write_reg(sc400ai->client,
						SC400AI_REG_EXPOSURE_H,
						SC400AI_REG_VALUE_08BIT,
						SC400AI_FETCH_EXP_H(val));
			ret |= sc400ai_write_reg(sc400ai->client,
						 SC400AI_REG_EXPOSURE_M,
						 SC400AI_REG_VALUE_08BIT,
						 SC400AI_FETCH_EXP_M(val));
			ret |= sc400ai_write_reg(sc400ai->client,
						 SC400AI_REG_EXPOSURE_L,
						 SC400AI_REG_VALUE_08BIT,
						 SC400AI_FETCH_EXP_L(val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain value 0x%x\n", ctrl->val);
		if (sc400ai->cur_mode->hdr_mode == NO_HDR)
			ret = sc400ai_set_gain_reg(sc400ai, ctrl->val, SC400AI_LGAIN);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set blank value 0x%x\n", ctrl->val);
		ret = sc400ai_write_reg(sc400ai->client,
					SC400AI_REG_VTS_H,
					SC400AI_REG_VALUE_08BIT,
					(ctrl->val + sc400ai->cur_mode->height)
					>> 8);
		ret |= sc400ai_write_reg(sc400ai->client,
					 SC400AI_REG_VTS_L,
					 SC400AI_REG_VALUE_08BIT,
					 (ctrl->val + sc400ai->cur_mode->height)
					 & 0xff);
		if (!ret)
			sc400ai->cur_vts = ctrl->val + sc400ai->cur_mode->height;
		sc400ai_modify_fps_info(sc400ai);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc400ai_enable_test_pattern(sc400ai, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc400ai_read_reg(sc400ai->client, SC400AI_FLIP_MIRROR_REG,
				       SC400AI_REG_VALUE_08BIT, &val);
		ret |= sc400ai_write_reg(sc400ai->client,
					SC400AI_FLIP_MIRROR_REG,
					SC400AI_REG_VALUE_08BIT,
					SC400AI_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = sc400ai_read_reg(sc400ai->client, SC400AI_FLIP_MIRROR_REG,
				      SC400AI_REG_VALUE_08BIT, &val);
		ret |= sc400ai_write_reg(sc400ai->client,
					SC400AI_FLIP_MIRROR_REG,
					SC400AI_REG_VALUE_08BIT,
					SC400AI_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc400ai_ctrl_ops = {
	.s_ctrl = sc400ai_set_ctrl,
};

static int sc400ai_parse_of(struct sc400ai *sc400ai)
{
	struct device *dev = &sc400ai->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval, hdr_mode = 0;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	of_property_read_u32(dev->of_node, OF_CAMERA_HDR_MODE, &hdr_mode);

	sc400ai->lane_num = rval;
	sc400ai->cfg_num = ARRAY_SIZE(supported_modes);

	if (sc400ai->lane_num == 2) {
		if (hdr_mode == HDR_X2) {
			sc400ai->cur_mode = &supported_modes[2];	// hdr 2x
			dev_info(dev, "lane_num(%d) HDR_X2 mode\n", sc400ai->lane_num);
		} else if (hdr_mode == NO_HDR) {
			sc400ai->cur_mode = &supported_modes[1];	// linear 2lane
			dev_info(dev, "lane_num(%d) NO_HDR mode\n", sc400ai->lane_num);
		}
	} else if (sc400ai->lane_num == 4) {
		sc400ai->cur_mode = &supported_modes[0];	// linear 4lane
		dev_info(dev, "lane_num(%d) NO_HDR mode\n", sc400ai->lane_num);
	} else {
		dev_err(dev, "unsupported lane_num(%d)\n", sc400ai->lane_num);
		return -1;
	}
	return 0;
}

static int sc400ai_initialize_controls(struct sc400ai *sc400ai)
{
	const struct sc400ai_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct device *dev = &sc400ai->client->dev;

	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_pixel_rate = 0;

	handler = &sc400ai->ctrl_handler;
	mode = sc400ai->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc400ai->mutex;

	sc400ai->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_menu_items) - 1, 0,
				link_freq_menu_items);
	if (sc400ai->link_freq)
		sc400ai->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	__v4l2_ctrl_s_ctrl(sc400ai->link_freq, mode->mipi_freq_idx);

	if (ret < 0)
		dev_err(dev, "get data num failed");

	if (mode->mipi_freq_idx == 0)
		dst_pixel_rate = PIXEL_RATE_WITH_315M_10BIT;
	else if (mode->mipi_freq_idx == 1)
		dst_pixel_rate = PIXEL_RATE_WITH_630M_10BIT;
	else if (mode->mipi_freq_idx == 2)
		dst_pixel_rate = PIXEL_RATE_WITH_1080M_10BIT;

	dev_err(dev, "dst_pixel_rate = %lld mipi_freq_idx:%d\n",
		dst_pixel_rate, mode->mipi_freq_idx);
	sc400ai->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
						V4L2_CID_PIXEL_RATE, 0,
						PIXEL_RATE_WITH_MAX,
						1, dst_pixel_rate);

	h_blank = mode->hts_def - mode->width;
	sc400ai->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (sc400ai->hblank)
		sc400ai->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	sc400ai->vblank = v4l2_ctrl_new_std(handler, &sc400ai_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   SC400AI_VTS_MAX - mode->height,
					   1, vblank_def);
	sc400ai->cur_vts = mode->vts_def;
	exposure_max = mode->vts_def - 4;
	sc400ai->exposure = v4l2_ctrl_new_std(handler, &sc400ai_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     SC400AI_EXPOSURE_MIN,
					     exposure_max,
					     SC400AI_EXPOSURE_STEP,
					     mode->exp_def);
	sc400ai->anal_gain = v4l2_ctrl_new_std(handler, &sc400ai_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      SC400AI_GAIN_MIN,
					      SC400AI_GAIN_MAX,
					      SC400AI_GAIN_STEP,
					      SC400AI_GAIN_DEFAULT);
	sc400ai->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
					&sc400ai_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(sc400ai_test_pattern_menu) - 1,
					0, 0, sc400ai_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &sc400ai_ctrl_ops,
			 V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &sc400ai_ctrl_ops,
			 V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc400ai->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc400ai->has_init_exp = false;
	sc400ai->subdev.ctrl_handler = handler;
	sc400ai->cur_fps = mode->max_fps;
	sc400ai->cur_vts = mode->vts_def;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc400ai_check_sensor_id(struct sc400ai *sc400ai,
				  struct i2c_client *client)
{
	struct device *dev = &sc400ai->client->dev;
	u32 id = 0;
	int ret;

	if (sc400ai->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = sc400ai_read_reg(client, SC400AI_REG_CHIP_ID,
			       SC400AI_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC400AI(%06x) sensor\n", CHIP_ID);

	return 0;
}

static int sc400ai_configure_regulators(struct sc400ai *sc400ai)
{
	unsigned int i;

	for (i = 0; i < SC400AI_NUM_SUPPLIES; i++)
		sc400ai->supplies[i].supply = sc400ai_supply_names[i];

	return devm_regulator_bulk_get(&sc400ai->client->dev,
				      SC400AI_NUM_SUPPLIES,
				      sc400ai->supplies);
}

static int sc400ai_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc400ai *sc400ai;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc400ai = devm_kzalloc(dev, sizeof(*sc400ai), GFP_KERNEL);
	if (!sc400ai)
		return -ENOMEM;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc400ai->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc400ai->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc400ai->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc400ai->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sc400ai->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	sc400ai->client = client;
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			sc400ai->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		sc400ai->cur_mode = &supported_modes[0];

	sc400ai->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc400ai->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	if (sc400ai->is_thunderboot) {
		sc400ai->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
		if (IS_ERR(sc400ai->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		sc400ai->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
		if (IS_ERR(sc400ai->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	} else {
		sc400ai->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(sc400ai->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		sc400ai->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
		if (IS_ERR(sc400ai->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	}

	sc400ai->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc400ai->pinctrl)) {
		sc400ai->pins_default =
			pinctrl_lookup_state(sc400ai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc400ai->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc400ai->pins_sleep =
			pinctrl_lookup_state(sc400ai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc400ai->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}
	ret = sc400ai_configure_regulators(sc400ai);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}
	ret = sc400ai_parse_of(sc400ai);
	if (ret != 0) {
		dev_err(dev, "Failed to get power regulators\n");
		return -EINVAL;
	}
	mutex_init(&sc400ai->mutex);

	sd = &sc400ai->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc400ai_subdev_ops);
	ret = sc400ai_initialize_controls(sc400ai);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc400ai_power_on(sc400ai);
	if (ret)
		goto err_free_handler;

	ret = sc400ai_check_sensor_id(sc400ai, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc400ai_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc400ai->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc400ai->pad);
	if (ret < 0)
		goto err_power_off;
#endif
	memset(facing, 0, sizeof(facing));
	if (strcmp(sc400ai->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc400ai->module_index, facing,
		 SC400AI_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (sc400ai->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc400ai_power_off(sc400ai);
err_free_handler:
	v4l2_ctrl_handler_free(&sc400ai->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc400ai->mutex);

	return ret;
}

static int sc400ai_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc400ai *sc400ai = to_sc400ai(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc400ai->ctrl_handler);
	mutex_destroy(&sc400ai->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc400ai_power_off(sc400ai);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc400ai_of_match[] = {
	{ .compatible = "smartsens,sc400ai" },
	{},
};
MODULE_DEVICE_TABLE(of, sc400ai_of_match);
#endif

static const struct i2c_device_id sc400ai_match_id[] = {
	{ "smartsens,sc400ai", 0 },
	{ },
};

static struct i2c_driver sc400ai_i2c_driver = {
	.driver = {
		.name = SC400AI_NAME,
		.pm = &sc400ai_pm_ops,
		.of_match_table = of_match_ptr(sc400ai_of_match),
	},
	.probe		= &sc400ai_probe,
	.remove		= &sc400ai_remove,
	.id_table	= sc400ai_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc400ai_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc400ai_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif

module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens sc400ai sensor driver");
MODULE_LICENSE("GPL");
