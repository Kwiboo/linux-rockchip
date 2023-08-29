// SPDX-License-Identifier: GPL-2.0
/*
 * ov08d10 driver
 *
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 *
 */

//  #define DEBUG

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
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)
#define OV08D10_I2C_7BIT_ADDR		0x36

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OV08D10_SCLK			144000000ULL
#define OV08D10_XVCLK_19_2		19200000
#define OV08D10_ROWCLK			36000

#define MIPI_FREQ_720M			720000000
#define MIPI_FREQ_360M			360000000

#define PIXEL_RATE_WITH_720M		(MIPI_FREQ_720M * 2 / 10 * 2)
#define PIXEL_RATE_WITH_360M		(MIPI_FREQ_360M * 2 / 10 * 2)
#define OV08D10_RGB_DEPTH		10

#define OV08D10_XVCLK_FREQ		24000000

// #define OV08D10_CHIP_ID		0x56084700
#define OV08D10_REG_CHIP_ID_0		0x00
#define OV08D10_REG_CHIP_ID_1		0x01
// #define OV08D10_REG_CHIP_ID_2	0x02
// #define OV08D10_REG_CHIP_ID_3	0x03
#define OV08D10_ID_MASK			GENMASK(15, 0)
#define OV08D10_CHIP_ID			0x5608

#define OV08D10_REG_ECO_VER		0x04

#define OV08D10_VTS_MAX			0x7FFF

#define OV08D10_GAIN_MIN		0x10
#define OV08D10_GAIN_MAX		0xF8
#define OV08D10_GAIN_STEP		1
#define OV08D10_GAIN_DEFAULT		0x10

#define OV08D10_REG_PAGE_SELECT		0xFD
#define OV08D10_REG_GLOBAL_EFFECTIVE	0x01

#define OV08D10_REG_EXP_H		0x02
#define OV08D10_REG_EXP_M		0x03
#define OV08D10_REG_EXP_L		0x04
#define	OV08D10_EXPOSURE_MIN		6
#define OV08D10_EXPOSURE_MAX_MARGIN	6
#define	OV08D10_EXPOSURE_STEP		1

/* Analog gain controls from sensor */
#define OV08D10_REG_AGAIN		0x24
// #define OV08D10_REG_AGAIN_H1		0x25
#define	OV08D10_ANAL_GAIN_MIN		128
#define	OV08D10_ANAL_GAIN_MAX		2047
#define	OV08D10_ANAL_GAIN_STEP		1

/* Digital gain controls from sensor */
#define OV08D10_REG_MWB_DGAIN_C		0x21
#define OV08D10_REG_MWB_DGAIN_F		0x22
#define OV08D10_DGTL_GAIN_MIN		0
#define OV08D10_DGTL_GAIN_MAX		4095
#define OV08D10_DGTL_GAIN_STEP		1
#define OV08D10_DGTL_GAIN_DEFAULT	1024

/* Test Pattern Control */
#define OV08D10_REG_TEST_PATTERN	0x12
#define OV08D10_TEST_PATTERN_ENABLE	0x01
#define OV08D10_TEST_PATTERN_DISABLE	0x00

/*read-only, unit: row-clk*/
#define OV08D10_REG_HTS_H		0x37
/*read-only*/
#define OV08D10_REG_HTS_L		0x38
/*read-only, unit: Tline*/
#define OV08D10_REG_VTS_H		0x34
/*read-only*/
#define OV08D10_REG_VTS_S		0x35
/*read-only*/
#define OV08D10_REG_VTS_L		0x36

#define OV08D10_REG_VBLANK_H		0x05
#define OV08D10_REG_VBLANK_L		0x06

#define OV08D10_REG_TEST_MOD		0x12

#define OV08D10_REG_CTRL_MODE		0xA0
#define OV08D10_MODE_SW_STANDBY		0x0
#define OV08D10_MODE_STREAMING		0x1

#define OV08D10_REG_SOFTWARE_RESET	0x20

/* Flip Mirror Controls from sensor */
#define OV08D10_REG_FLIP_OPT		0x32
#define OV08D10_REG_FLIP_MASK		0x3


/* system control page*/
#define	OV08D10_PAGE_SYS_CTL		0x0
/* CIS control page*/
#define	OV08D10_PAGE_CIS_CTL		0x1
/* ISP control page*/
#define	OV08D10_PAGE_ISP_CTL		0x2
/* OTP (one time programmable) SC page*/
#define	OV08D10_PAGE_OTP_SC_CTL		0x3
/* dynamic DPC page*/
#define	OV08D10_PAGE_DDPC_CTL		0x4
/* OTP(one time programmable) DPC page*/
#define	OV08D10_PAGE_OTP_DPC_CTL	0x5
/* pre_ISP page*/
#define	OV08D10_PAGE_PRE_ISP_CTL	0x6
/* BLC control page*/
#define	OV08D10_PAGE_BLC_CTL		0x7
/* OTP(one time programmable) SRAM page*/
#define	OV08D10_PAGE_OTP_SRAM_CTL	0x8

#define OV08D10_LANES			2
#define OV08D10_NAME			"OV08D10"

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define REG_NULL			0xFF

#define to_ov08d10(sd)		container_of(sd, struct ov08d10, subdev)

static const char *const ov08d10_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV08D10_NUM_SUPPLIES ARRAY_SIZE(ov08d10_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct ov08d10_mode {
	u32 bus_fmt;
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	struct v4l2_fract max_fps;

	/* Horizontal timining size */
	u32 hts_def;

	/* Default vertical timining size */
	u32 vts_def;
	u32 exp_def;

	/* Link frequency needed for this resolution */
	u32 link_freq_index;

	/* Sensor register settings for this resolution */
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct ov08d10 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OV08D10_NUM_SUPPLIES];

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
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov08d10_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	u8			flip;
};


/*
 * Xclk 24Mhz
 */
static const struct regval ov08d10_linear10bit_3280x2460_regs[] = {
	{0xfd, 0x00},
	{0x11, 0x2a},
	{0x14, 0x43},

	{0x1a, 0x04},
	{0x1b, 0xe1},
	{0x1e, 0x13},
	{0xb7, 0x02},

	{0xfd, 0x01},
	{0x12, 0x00},
	{0x02, 0x00},
	{0x03, 0x12},
	{0x04, 0x50},
	{0x07, 0x05},
	{0x21, 0x02},
	{0x24, 0x30},
	{0x33, 0x03},
	{0x01, 0x03},
	{0x19, 0x10},
	{0x42, 0x55},
	{0x43, 0x00},
	{0x47, 0x07},
	{0x48, 0x08},
	{0xb2, 0x7f},
	{0xb3, 0x7b},
	{0xbd, 0x08},
	{0xd2, 0x57},
	{0xd3, 0x10},
	{0xd4, 0x08},
	{0xd5, 0x08},
	{0xd6, 0x06},
	{0xb1, 0x00},
	{0xb4, 0x00},
	{0xb7, 0x0a},
	{0xbc, 0x44},
	{0xbf, 0x48},
	{0xc1, 0x10},
	{0xc3, 0x24},
	{0xc8, 0x03},
	{0xc9, 0xf8},
	{0xe1, 0x33},
	{0xe2, 0xbb},
	{0x51, 0x0c},
	{0x52, 0x0a},
	{0x57, 0x8c},
	{0x59, 0x09},
	{0x5a, 0x08},
	{0x5e, 0x10},
	{0x60, 0x02},
	{0x6d, 0x5c},
	{0x76, 0x16},
	{0x7c, 0x11},
	{0x90, 0x28},
	{0x91, 0x16},
	{0x92, 0x1c},
	{0x93, 0x24},
	{0x95, 0x48},
	{0x9c, 0x06},
	{0xca, 0x0c},
	{0xce, 0x0d},
	{0xfd, 0x01},
	{0xc0, 0x00},
	{0xdd, 0x18},
	{0xde, 0x19},
	{0xdf, 0x32},
	{0xe0, 0x70},
	{0xfd, 0x01},
	{0xc2, 0x05},
	{0xd7, 0x88},
	{0xd8, 0x77},
	{0xd9, 0x66},
	{0xfd, 0x07},
	{0x00, 0xf8},
	{0x01, 0x2b},
	{0x05, 0x40},
	{0x08, 0x06},
	{0x09, 0x11},
	{0x28, 0x6f},
	{0x2a, 0x20},
	{0x2b, 0x05},
	{0x5e, 0x10},
	{0x52, 0x00},
	{0x53, 0x80},
	{0x54, 0x00},
	{0x55, 0x80},
	{0x56, 0x00},
	{0x57, 0x80},
	{0x58, 0x00},
	{0x59, 0x80},
	{0x5c, 0x3f},
	{0xfd, 0x02},
	{0x9a, 0x30},	/* isp mode: 0 */
	{0xa8, 0x02},

	{0xfd, 0x02},
	{0xa0, 0x00},	/* height start: 0 */
	{0xa1, 0x00},
	{0xa2, 0x09},	/* height: 2460 */
	{0xa3, 0x9c},
	{0xa4, 0x00},	/* height start: 0 */
	{0xa5, 0x00},
	{0xa6, 0x0c},	/* width: 3280 */
	{0xa7, 0xd0},

	{0xfd, 0x05},
	{0x04, 0x40},
	{0x07, 0x00},
	{0x0D, 0x01},
	{0x0F, 0x01},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x0C},
	{0x13, 0xCF},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x18, 0x00},
	{0x19, 0x00},

	{0xfd, 0x00},
	{0x24, 0x01},
	{0xc0, 0x16},
	{0xc1, 0x08},
	{0xc2, 0x30},
	{0x8e, 0x0c},
	{0x8f, 0xd0},
	{0x90, 0x09},
	{0x91, 0xa0},
	{0xb7, 0x02},

	{0xfd, 0x00},
	{0x20, 0x0f},
	{0xe7, 0x03},
	{0xe7, 0x00},

	// {0xfd, 0x00},
	// {0xa0, 0x01},	/* stream on */
	{REG_NULL, 0x00},
};

static const struct regval ov08d10_lane_2_mode_3264x2448_regs[] = {
	/* 3264x2448 resolution */
	{0xfd, 0x01},
	{0x12, 0x00},
	{0x03, 0x12},
	{0x04, 0x58},
	{0x07, 0x05},
	{0x21, 0x02},
	{0x24, 0x30},
	{0x33, 0x03},
	{0x01, 0x03},
	{0x19, 0x10},
	{0x42, 0x55},
	{0x43, 0x00},
	{0x47, 0x07},
	{0x48, 0x08},
	{0xb2, 0x7f},
	{0xb3, 0x7b},
	{0xbd, 0x08},
	{0xd2, 0x57},
	{0xd3, 0x10},
	{0xd4, 0x08},
	{0xd5, 0x08},
	{0xd6, 0x06},
	{0xb1, 0x00},
	{0xb4, 0x00},
	{0xb7, 0x0a},
	{0xbc, 0x44},
	{0xbf, 0x48},
	{0xc1, 0x10},
	{0xc3, 0x24},
	{0xc8, 0x03},
	{0xc9, 0xf8},
	{0xe1, 0x33},
	{0xe2, 0xbb},
	{0x51, 0x0c},
	{0x52, 0x0a},
	{0x57, 0x8c},
	{0x59, 0x09},
	{0x5a, 0x08},
	{0x5e, 0x10},
	{0x60, 0x02},
	{0x6d, 0x5c},
	{0x76, 0x16},
	{0x7c, 0x11},
	{0x90, 0x28},
	{0x91, 0x16},
	{0x92, 0x1c},
	{0x93, 0x24},
	{0x95, 0x48},
	{0x9c, 0x06},
	{0xca, 0x0c},
	{0xce, 0x0d},
	{0xfd, 0x01},
	{0xc0, 0x00},
	{0xdd, 0x18},
	{0xde, 0x19},
	{0xdf, 0x32},
	{0xe0, 0x70},
	{0xfd, 0x01},
	{0xc2, 0x05},
	{0xd7, 0x88},
	{0xd8, 0x77},
	{0xd9, 0x00},
	{0xfd, 0x07},
	{0x00, 0xf8},
	{0x01, 0x2b},
	{0x05, 0x40},
	{0x08, 0x06},
	{0x09, 0x11},
	{0x28, 0x6f},
	{0x2a, 0x20},
	{0x2b, 0x05},
	{0x5e, 0x10},
	{0x52, 0x00},
	{0x53, 0x7c},
	{0x54, 0x00},
	{0x55, 0x7c},
	{0x56, 0x00},
	{0x57, 0x7c},
	{0x58, 0x00},
	{0x59, 0x7c},
	{0xfd, 0x02},
	{0x9a, 0x30},
	{0xa8, 0x02},
	{0xfd, 0x02},
	{0xa1, 0x09},
	{0xa2, 0x09},
	{0xa3, 0x90},
	{0xa5, 0x08},
	{0xa6, 0x0c},
	{0xa7, 0xc0},
	{0xfd, 0x00},
	{0x24, 0x01},
	{0xc0, 0x16},
	{0xc1, 0x08},
	{0xc2, 0x30},
	{0x8e, 0x0c},
	{0x8f, 0xc0},
	{0x90, 0x09},
	{0x91, 0x90},
	{0xfd, 0x05},
	{0x04, 0x40},
	{0x07, 0x00},
	{0x0d, 0x01},
	{0x0f, 0x01},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x0c},
	{0x13, 0xcf},
	{0x14, 0x00},
	{0x15, 0x00},
	{0xfd, 0x00},
	{0x20, 0x0f},
	{0xe7, 0x03},
	{0xe7, 0x00},
	{REG_NULL, 0x00},
};

static const struct regval ov08d10_lane_2_mode_1632x1224_regs[] = {
	/* 1640x1232 resolution */
	{0xfd, 0x01},
	{0x1a, 0x0a},
	{0x1b, 0x08},
	{0x2a, 0x01},
	{0x2b, 0x9a},
	{0xfd, 0x01},
	{0x12, 0x00},
	{0x03, 0x05},
	{0x04, 0xe2},
	{0x07, 0x05},
	{0x21, 0x02},
	{0x24, 0x30},
	{0x33, 0x03},
	{0x31, 0x06},
	{0x33, 0x03},
	{0x01, 0x03},
	{0x19, 0x10},
	{0x42, 0x55},
	{0x43, 0x00},
	{0x47, 0x07},
	{0x48, 0x08},
	{0xb2, 0x7f},
	{0xb3, 0x7b},
	{0xbd, 0x08},
	{0xd2, 0x57},
	{0xd3, 0x10},
	{0xd4, 0x08},
	{0xd5, 0x08},
	{0xd6, 0x06},
	{0xb1, 0x00},
	{0xb4, 0x00},
	{0xb7, 0x0a},
	{0xbc, 0x44},
	{0xbf, 0x48},
	{0xc1, 0x10},
	{0xc3, 0x24},
	{0xc8, 0x03},
	{0xc9, 0xf8},
	{0xe1, 0x33},
	{0xe2, 0xbb},
	{0x51, 0x0c},
	{0x52, 0x0a},
	{0x57, 0x8c},
	{0x59, 0x09},
	{0x5a, 0x08},
	{0x5e, 0x10},
	{0x60, 0x02},
	{0x6d, 0x5c},
	{0x76, 0x16},
	{0x7c, 0x1a},
	{0x90, 0x28},
	{0x91, 0x16},
	{0x92, 0x1c},
	{0x93, 0x24},
	{0x95, 0x48},
	{0x9c, 0x06},
	{0xca, 0x0c},
	{0xce, 0x0d},
	{0xfd, 0x01},
	{0xc0, 0x00},
	{0xdd, 0x18},
	{0xde, 0x19},
	{0xdf, 0x32},
	{0xe0, 0x70},
	{0xfd, 0x01},
	{0xc2, 0x05},
	{0xd7, 0x88},
	{0xd8, 0x77},
	{0xd9, 0x00},
	{0xfd, 0x07},
	{0x00, 0xf8},
	{0x01, 0x2b},
	{0x05, 0x40},
	{0x08, 0x03},
	{0x09, 0x08},
	{0x28, 0x6f},
	{0x2a, 0x20},
	{0x2b, 0x05},
	{0x2c, 0x01},
	{0x50, 0x02},
	{0x51, 0x03},
	{0x5e, 0x00},
	{0x52, 0x00},
	{0x53, 0x7c},
	{0x54, 0x00},
	{0x55, 0x7c},
	{0x56, 0x00},
	{0x57, 0x7c},
	{0x58, 0x00},
	{0x59, 0x7c},
	{0xfd, 0x02},
	{0x9a, 0x30},
	{0xa8, 0x02},
	{0xfd, 0x02},
	{0xa9, 0x04},
	{0xaa, 0xd0},
	{0xab, 0x06},
	{0xac, 0x68},
	{0xa1, 0x09},
	{0xa2, 0x04},
	{0xa3, 0xc8},
	{0xa5, 0x04},
	{0xa6, 0x06},
	{0xa7, 0x60},
	{0xfd, 0x05},
	{0x06, 0x80},
	{0x18, 0x06},
	{0x19, 0x68},
	{0xfd, 0x00},
	{0x24, 0x01},
	{0xc0, 0x16},
	{0xc1, 0x08},
	{0xc2, 0x30},
	{0x8e, 0x06},
	{0x8f, 0x60},
	{0x90, 0x04},
	{0x91, 0xc8},
	{0x93, 0x0e},
	{0x94, 0x77},
	{0x95, 0x77},
	{0x96, 0x10},
	{0x98, 0x88},
	{0x9c, 0x1a},
	{0xfd, 0x05},
	{0x04, 0x40},
	{0x07, 0x99},
	{0x0d, 0x03},
	{0x0f, 0x03},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x0c},
	{0x13, 0xcf},
	{0x14, 0x00},
	{0x15, 0x00},
	{0xfd, 0x00},
	{0x20, 0x0f},
	{0xe7, 0x03},
	{0xe7, 0x00},
};

static const char *const __maybe_unused ov08d10_test_pattern_menu[] = {
	"Disabled",
	"Standard Color Bar",
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
static const struct ov08d10_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 3280,
		.height = 2460,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x02ea,
		.hts_def = 0x01cc,
		.vts_def = 0x0a30,
		.link_freq_index = 0,
		.reg_list = ov08d10_linear10bit_3280x2460_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x02ea,
		.hts_def = 1840,
		.vts_def = 2504,
		.link_freq_index = 0,
		.reg_list = ov08d10_lane_2_mode_3264x2448_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},

	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1632,
		.height = 1224,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x02ea,
		.hts_def = 1912,
		.vts_def = 3736,
		.link_freq_index = 1,
		.reg_list = ov08d10_lane_2_mode_1632x1224_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_720M,
	MIPI_FREQ_360M,
};

static int __ov08d10_power_on(struct ov08d10 *ov08d10);

static int ov08d10_check_sensor_id(struct ov08d10 *ov08d10,
				  struct i2c_client *client);


static u64 __maybe_unused to_rate(const s64 *link_freq_menu,
				 u32 f_index, u8 nlanes)
{
	u64 pixel_rate = link_freq_menu[f_index] * 2 * nlanes;

	do_div(pixel_rate, OV08D10_RGB_DEPTH);

	return pixel_rate;
}

static u64 __maybe_unused to_pixels_per_line(const s64 *link_freq_menu, u32 hts,
					    u32 f_index, u8 nlanes)
{
	u64 ppl = hts * to_rate(link_freq_menu, f_index, nlanes);

	do_div(ppl, OV08D10_SCLK);

	return ppl;
}

/* sensor register read */
static int __maybe_unused ov08d10_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		dev_err(&client->dev, "read reg(%02x), ret(%02x)\n", reg, *val);
		return 0;
	}

	dev_err(&client->dev,
		"ov08d10 read reg(0x%x val:0x%x) failed !\n", reg, *val);

	return ret;
}

/* sensor register write */
static int ov08d10_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"ov08d10 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int ov08d10_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		ret = ov08d10_write_reg(client, regs[i].addr, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}

static int ov08d10_set_exposure(struct ov08d10 *ov08d10, u32 exposure)
{
	struct i2c_client *client = ov08d10->client;
	u8 val;
	u8 hts_h, hts_l;
	u32 hts, cur_vts, exp_cal;
	int ret;

	cur_vts = ov08d10->cur_mode->vts_def;
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	hts_h = i2c_smbus_read_byte_data(client, 0x37);
	hts_l = i2c_smbus_read_byte_data(client, 0x38);
	hts = ((hts_h << 8) | (hts_l));
	exp_cal = 66 * OV08D10_ROWCLK / hts;
	exposure = exposure * exp_cal / (cur_vts - OV08D10_EXPOSURE_MAX_MARGIN);

	/* CIS control registers */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	/* update exposure */
	val = ((exposure >> 16) & 0xFF);
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_EXP_H, val);
	if (ret < 0)
		return ret;

	val = ((exposure >> 8) & 0xFF);
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_EXP_M, val);
	if (ret < 0)
		return ret;

	val = exposure & 0xFF;
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_EXP_L, val);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client,
					OV08D10_REG_GLOBAL_EFFECTIVE, 0x01);
}

static int ov08d10_update_analog_gain(struct ov08d10 *ov08d10, u32 a_gain)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov08d10->subdev);
	u8 val;
	int ret;

	val = ((a_gain >> 3) & 0xFF);
	/* CIS control registers */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	/* update AGAIN */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_AGAIN, val);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client,
					OV08D10_REG_GLOBAL_EFFECTIVE, 0x01);
}

static int ov08d10_update_digital_gain(struct ov08d10 *ov08d10, u32 d_gain)
{
	struct i2c_client *client = ov08d10->client;
	u8 val;
	int ret;

	d_gain = (d_gain >> 1);
	/* CIS control registers */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	val = ((d_gain >> 8) & 0x3F);
	/* update DGAIN */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_MWB_DGAIN_C, val);
	if (ret < 0)
		return ret;

	val = d_gain & 0xFF;
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_MWB_DGAIN_F, val);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client,
					OV08D10_REG_GLOBAL_EFFECTIVE, 0x01);
}

static int ov08d10_set_ctrl_flip(struct ov08d10 *ov08d10, u32 ctrl_val)
{
	struct i2c_client *client = ov08d10->client;
	u8 val;
	int ret;

	/* System control registers */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte_data(client, OV08D10_REG_FLIP_OPT);
	if (ret < 0)
		return ret;

	val = ret | (ctrl_val & OV08D10_REG_FLIP_MASK);

	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_FLIP_OPT, val);

	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client,
					OV08D10_REG_GLOBAL_EFFECTIVE, 0x01);
}

static int ov08d10_set_vblank(struct ov08d10 *ov08d10, u32 vblank)
{
	struct i2c_client *client = ov08d10->client;
	u8 val;
	int ret;

	/* CIS control registers */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	val = ((vblank >> 8) & 0xFF);
	/* update vblank */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_VBLANK_H, val);
	if (ret < 0)
		return ret;

	val = vblank & 0xFF;
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_VBLANK_L, val);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client,
					OV08D10_REG_GLOBAL_EFFECTIVE, 0x01);
}

static int ov08d10_test_pattern(struct ov08d10 *ov08d10, u32 pattern)
{
	struct i2c_client *client = ov08d10->client;
	u8 val;
	int ret;

	if (pattern)
		val = OV08D10_TEST_PATTERN_ENABLE;
	else
		val = OV08D10_TEST_PATTERN_DISABLE;

	/* CIS control registers */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(client,
					OV08D10_REG_TEST_PATTERN, val);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client,
					OV08D10_REG_GLOBAL_EFFECTIVE, 0x01);
}

static int ov08d10_get_reso_dist(const struct ov08d10_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct ov08d10_mode *
ov08d10_find_best_fit(struct ov08d10 *ov08d10, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ov08d10->cfg_num; i++) {
		dist = ov08d10_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
			(supported_modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov08d10_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	const struct ov08d10_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	mutex_lock(&ov08d10->mutex);

	mode = ov08d10_find_best_fit(ov08d10, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov08d10->mutex);
		return -ENOTTY;
#endif
	} else {
		ov08d10->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov08d10->hblank, h_blank,
					h_blank, 1, h_blank);

		/* Update limits and set FPS to default */
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov08d10->vblank, vblank_def,
					OV08D10_VTS_MAX - mode->height,
					1, vblank_def);
		if (mode->hdr_mode == NO_HDR) {
			if (mode->bus_fmt == MEDIA_BUS_FMT_SBGGR10_1X10) {
				dst_link_freq = 0;
				dst_pixel_rate = PIXEL_RATE_WITH_720M;
			}
		}
		__v4l2_ctrl_s_ctrl_int64(ov08d10->pixel_rate,
					dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(ov08d10->link_freq,
				  dst_link_freq);
	}

	mutex_unlock(&ov08d10->mutex);

	return 0;
}

static int ov08d10_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	const struct ov08d10_mode *mode = ov08d10->cur_mode;

	mutex_lock(&ov08d10->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov08d10->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&ov08d10->mutex);

	return 0;
}

static int ov08d10_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);

	if (code->index != 0)
		return -EINVAL;

	mutex_lock(&ov08d10->mutex);
	code->code = ov08d10->cur_mode->bus_fmt;
	mutex_unlock(&ov08d10->mutex);

	return 0;
}

static int ov08d10_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);

	if (fse->index >= ov08d10->cfg_num)
		return -EINVAL;

	mutex_lock(&ov08d10->mutex);
	if (fse->code != supported_modes[fse->index].bus_fmt) {
		mutex_unlock(&ov08d10->mutex);
		return -EINVAL;
	}
	mutex_unlock(&ov08d10->mutex);

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ov08d10_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	const struct ov08d10_mode *mode = ov08d10->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int ov08d10_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	const struct ov08d10_mode *mode = ov08d10->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (OV08D10_LANES - 1) |
			  V4L2_MBUS_CSI2_CHANNEL_0 |
			  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode == HDR_X2)
		val = 1 << (OV08D10_LANES - 1) |
			  V4L2_MBUS_CSI2_CHANNEL_0 |
			  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
			  V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void ov08d10_get_module_inf(struct ov08d10 *ov08d10,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OV08D10_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, ov08d10->module_name,
			sizeof(inf->base.module));
	strscpy(inf->base.lens, ov08d10->len_name, sizeof(inf->base.lens));
}

static long ov08d10_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		ret = -1;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode != 0)
			ret = -1;
		break;
	case RKMODULE_GET_MODULE_INFO:
		ov08d10_get_module_inf(ov08d10, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = ov08d10->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		ret = ov08d10_write_reg(ov08d10->client, OV08D10_REG_PAGE_SELECT,
					OV08D10_PAGE_SYS_CTL);
		if (stream)
			ret |= ov08d10_write_reg(ov08d10->client, OV08D10_REG_CTRL_MODE,
						 OV08D10_MODE_STREAMING);
		else
			ret |= ov08d10_write_reg(ov08d10->client, OV08D10_REG_CTRL_MODE,
						 OV08D10_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov08d10_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov08d10_ioctl(sd, cmd, inf);
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

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = ov08d10_ioctl(sd, cmd, cfg);
		else
			ret = -EFAULT;
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov08d10_ioctl(sd, cmd, hdr);
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

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = ov08d10_ioctl(sd, cmd, hdr);
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
			ret = ov08d10_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = ov08d10_ioctl(sd, cmd, &cg);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ov08d10_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ov08d10_start_stream(struct ov08d10 *ov08d10)
{
	int ret;
	struct i2c_client *client = ov08d10->client;

	/* soft reset */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "failed to reset sensor");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, 0x20, 0x0e);
	if (ret < 0) {
		dev_err(&client->dev, "failed to reset sensor");
		return ret;
	}
	usleep_range(3000, 4000);
	ret = i2c_smbus_write_byte_data(client, 0x20, 0x0b);
	if (ret < 0) {
		dev_err(&client->dev, "failed to reset sensor");
		return ret;
	}

	/* update sensor setting */
	ret = ov08d10_write_array(ov08d10->client, ov08d10->cur_mode->reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set mode");
		return ret;
	}

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&ov08d10->ctrl_handler);
	if (ret)
		return ret;
	if (ov08d10->has_init_exp && ov08d10->cur_mode->hdr_mode != NO_HDR) {
		ret = ov08d10_ioctl(&ov08d10->subdev, PREISP_CMD_SET_HDRAE_EXP,
				   &ov08d10->init_hdrae_exp);
		if (ret) {
			dev_err(&ov08d10->client->dev,
					"init exp fail in hdr mode\n");
			return ret;
		}
	}

	ret = ov08d10_write_reg(ov08d10->client, OV08D10_REG_PAGE_SELECT, OV08D10_PAGE_SYS_CTL);
	if (ret < 0)
		return ret;
	ret = ov08d10_write_reg(ov08d10->client, OV08D10_REG_CTRL_MODE, OV08D10_MODE_STREAMING);
	if (ret < 0) {
		dev_err(&ov08d10->client->dev, "failed to start streaming");
		return ret;
	}

	ret = ov08d10_write_reg(ov08d10->client, OV08D10_REG_PAGE_SELECT, 0x01);
	return ret;
}

static int __ov08d10_stop_stream(struct ov08d10 *ov08d10)
{
	int ret;

	ov08d10->has_init_exp = false;
	ret = ov08d10_write_reg(ov08d10->client, OV08D10_REG_PAGE_SELECT, OV08D10_PAGE_SYS_CTL);
	if (ret < 0) {
		dev_err(&ov08d10->client->dev, "failed to stop streaming");
		return ret;
	}

	ret = ov08d10_write_reg(ov08d10->client, OV08D10_REG_CTRL_MODE, OV08D10_MODE_SW_STANDBY);
	if (ret < 0) {
		dev_err(&ov08d10->client->dev, "failed to stop streaming");
		return ret;
	}

	ret = ov08d10_write_reg(ov08d10->client, OV08D10_REG_PAGE_SELECT, 0x01);
	if (ret < 0) {
		dev_err(&ov08d10->client->dev, "failed to stop streaming");
		return ret;
	}

	return ret;
}

static int ov08d10_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	struct i2c_client *client = ov08d10->client;
	int ret = 0;

	mutex_lock(&ov08d10->mutex);
	on = !!on;
	if (on == ov08d10->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov08d10_start_stream(ov08d10);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov08d10_stop_stream(ov08d10);
		pm_runtime_put(&client->dev);
	}

	ov08d10->streaming = on;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(ov08d10->v_flip, on);
	__v4l2_ctrl_grab(ov08d10->h_flip, on);

unlock_and_return:
	mutex_unlock(&ov08d10->mutex);

	return ret;
}

static int ov08d10_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	struct i2c_client *client = ov08d10->client;
	int ret = 0;

	mutex_lock(&ov08d10->mutex);

	/* If the power state is not modified - no work to do. */
	if (ov08d10->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret |= ov08d10_write_reg(ov08d10->client,
					OV08D10_REG_PAGE_SELECT,
					OV08D10_PAGE_SYS_CTL);
		ret |= ov08d10_write_reg(ov08d10->client,
					OV08D10_REG_SOFTWARE_RESET,
					0x0e);
		usleep_range(1000, 2000);
		ret |= ov08d10_write_reg(ov08d10->client,
					OV08D10_REG_SOFTWARE_RESET,
					0x0b);
		usleep_range(1000, 2000);

		ov08d10->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov08d10->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov08d10->mutex);

	return ret;
}

static int ov08d10_enable_regulators(struct ov08d10 *ov08d10,
				    struct regulator_bulk_data *consumers)
{
	int i, j;
	int ret = 0;
	struct device *dev = &ov08d10->client->dev;
	int num_consumers = OV08D10_NUM_SUPPLIES;

	for (i = 0; i < num_consumers; i++) {

		ret = regulator_enable(consumers[i].consumer);
		if (ret < 0) {
			dev_err(dev, "Failed to enable regulator: %s\n",
				consumers[i].supply);
			goto err;
		}
	}
	return 0;
err:
	for (j = 0; j < i; j++)
		regulator_disable(consumers[j].consumer);

	return ret;
}

static int __ov08d10_power_on(struct ov08d10 *ov08d10)
{
	int ret;
	struct device *dev = &ov08d10->client->dev;

	if (!IS_ERR_OR_NULL(ov08d10->pins_default)) {
		ret = pinctrl_select_state(ov08d10->pinctrl,
					  ov08d10->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(ov08d10->xvclk, OV08D10_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov08d10->xvclk) != OV08D10_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

	if (!IS_ERR(ov08d10->pwdn_gpio))
		gpiod_direction_output(ov08d10->pwdn_gpio, 0);

	ret = ov08d10_enable_regulators(ov08d10, ov08d10->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	usleep_range(100, 110);
	ret = clk_prepare_enable(ov08d10->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	/* From spec: delay from power stable to pwdn off: 5ms */
	usleep_range(5000, 6000);
	if (!IS_ERR(ov08d10->pwdn_gpio))
		gpiod_direction_output(ov08d10->pwdn_gpio, 1);

	/* From spec: 5ms for SCCB initialization */
	usleep_range(5000, 6000);
	return 0;

disable_clk:
	clk_disable_unprepare(ov08d10->xvclk);

	return ret;
}

static void __ov08d10_power_off(struct ov08d10 *ov08d10)
{
	int ret;
	struct device *dev = &ov08d10->client->dev;

	if (!IS_ERR(ov08d10->reset_gpio))
		gpiod_direction_output(ov08d10->reset_gpio, 1);

	clk_disable_unprepare(ov08d10->xvclk);

	if (!IS_ERR(ov08d10->pwdn_gpio))
		gpiod_direction_output(ov08d10->pwdn_gpio, 1);

	if (!IS_ERR_OR_NULL(ov08d10->pins_sleep)) {
		ret = pinctrl_select_state(ov08d10->pinctrl,
					  ov08d10->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(OV08D10_NUM_SUPPLIES, ov08d10->supplies);
}

static int __maybe_unused ov08d10_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov08d10 *ov08d10 = to_ov08d10(sd);

	return __ov08d10_power_on(ov08d10);
}

static int __maybe_unused ov08d10_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov08d10 *ov08d10 = to_ov08d10(sd);

	__ov08d10_power_off(ov08d10);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov08d10_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov08d10_mode *def_mode = &supported_modes[0];

	mutex_lock(&ov08d10->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov08d10->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ov08d10_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov08d10 *ov08d10 = to_ov08d10(sd);

	if (fie->index >= ov08d10->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops ov08d10_pm_ops = {
	SET_RUNTIME_PM_OPS(ov08d10_runtime_suspend,
			  ov08d10_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov08d10_internal_ops = {
	.open = ov08d10_open,
};
#endif

static const struct v4l2_subdev_core_ops ov08d10_core_ops = {
	.s_power = ov08d10_s_power,
	.ioctl = ov08d10_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov08d10_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov08d10_video_ops = {
	.s_stream = ov08d10_s_stream,
	.g_frame_interval = ov08d10_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov08d10_pad_ops = {
	.enum_mbus_code = ov08d10_enum_mbus_code,
	.enum_frame_size = ov08d10_enum_frame_sizes,
	.enum_frame_interval = ov08d10_enum_frame_interval,
	.get_fmt = ov08d10_get_fmt,
	.set_fmt = ov08d10_set_fmt,
	.get_mbus_config = ov08d10_g_mbus_config,
};

static const struct v4l2_subdev_ops ov08d10_subdev_ops = {
	.core	= &ov08d10_core_ops,
	.video	= &ov08d10_video_ops,
	.pad	= &ov08d10_pad_ops,
};

static int ov08d10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov08d10 *ov08d10 = container_of(ctrl->handler,
					      struct ov08d10, ctrl_handler);
	struct i2c_client *client = ov08d10->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ov08d10->cur_mode->height + ctrl->val - 7;
		__v4l2_ctrl_modify_range(ov08d10->exposure,
					ov08d10->exposure->minimum, max,
					ov08d10->exposure->step,
					ov08d10->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov08d10_set_exposure(ov08d10, ctrl->val);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov08d10_update_analog_gain(ov08d10, ctrl->val);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		ret = ov08d10_update_digital_gain(ov08d10, ctrl->val);
		break;

	case V4L2_CID_VBLANK:
		ret = ov08d10_set_vblank(ov08d10, ctrl->val);
		break;

	case V4L2_CID_TEST_PATTERN:
		ret = ov08d10_test_pattern(ov08d10, ctrl->val);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = ov08d10_set_ctrl_flip(ov08d10,
					   ov08d10->h_flip->val |
					   ov08d10->v_flip->val << 1);
		break;

	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			__func__, ctrl->id, ctrl->val);
		break;
	}
	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov08d10_ctrl_ops = {
	.s_ctrl = ov08d10_set_ctrl,
};

static int ov08d10_initialize_controls(struct ov08d10 *ov08d10)
{
	const struct ov08d10_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	handler = &ov08d10->ctrl_handler;
	mode = ov08d10->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &ov08d10->mutex;

	ov08d10->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						   V4L2_CID_LINK_FREQ,
						   1, 0, link_freq_menu_items);

	if (ov08d10->cur_mode->bus_fmt == MEDIA_BUS_FMT_SBGGR10_1X10) {
		dst_link_freq = 0;
		dst_pixel_rate = PIXEL_RATE_WITH_720M;
	}
	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	ov08d10->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
						V4L2_CID_PIXEL_RATE,
						0, PIXEL_RATE_WITH_720M,
						1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(ov08d10->link_freq,
					   dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	ov08d10->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (ov08d10->hblank)
		ov08d10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* From spec: vstart is 0xc by default */
	vblank_def = mode->vts_def - mode->height - 0xc;
	ov08d10->vblank = v4l2_ctrl_new_std(handler, &ov08d10_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   OV08D10_VTS_MAX - mode->height,
					   1, vblank_def);

	exposure_max = mode->vts_def - OV08D10_EXPOSURE_MAX_MARGIN;
	ov08d10->exposure = v4l2_ctrl_new_std(handler, &ov08d10_ctrl_ops,
					     V4L2_CID_EXPOSURE, OV08D10_EXPOSURE_MIN,
					     exposure_max, OV08D10_EXPOSURE_STEP,
					     exposure_max);

	ov08d10->anal_gain = v4l2_ctrl_new_std(handler, &ov08d10_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, OV08D10_ANAL_GAIN_MIN,
					      OV08D10_ANAL_GAIN_MAX, OV08D10_ANAL_GAIN_STEP,
					      OV08D10_ANAL_GAIN_MIN);

	ov08d10->digi_gain = v4l2_ctrl_new_std(handler, &ov08d10_ctrl_ops,
					      V4L2_CID_DIGITAL_GAIN,
					      OV08D10_DGTL_GAIN_MIN, OV08D10_DGTL_GAIN_MAX,
					      OV08D10_DGTL_GAIN_STEP, OV08D10_DGTL_GAIN_DEFAULT);

	ov08d10->h_flip = v4l2_ctrl_new_std(handler, &ov08d10_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	ov08d10->v_flip = v4l2_ctrl_new_std(handler, &ov08d10_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	ov08d10->flip = 0;
	if (handler->error) {
		ret = handler->error;
		dev_err(&ov08d10->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov08d10->subdev.ctrl_handler = handler;
	ov08d10->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov08d10_check_sensor_id(struct ov08d10 *ov08d10,
				  struct i2c_client *client)
{
	struct device *dev = &ov08d10->client->dev;

	u32 val;
	u16 chip_id;
	int ret;

	dev_err(dev, "dev i2c_addr:%02x\n", client->addr);

	/* System control registers */
	ret = i2c_smbus_write_byte_data(client, OV08D10_REG_PAGE_SELECT, 0x00);
	if (ret < 0) {
		dev_err(dev, "write reg page select fail\n");
		return ret;
	}

	/* Validate the chip ID */
	ret = i2c_smbus_read_byte_data(client, OV08D10_REG_CHIP_ID_0);
	if (ret < 0) {
		dev_err(dev, "read chip id 0 fail\n");
		return ret;
	}

	val = ret << 8;

	ret = i2c_smbus_read_byte_data(client, OV08D10_REG_CHIP_ID_1);
	if (ret < 0) {

		dev_err(dev, "read chip id 1 fail\n");
		return ret;
	}

	chip_id = val | ret;

	if ((chip_id & OV08D10_ID_MASK) != OV08D10_CHIP_ID) {
		dev_err(dev, "unexpected sensor id(0x%04x)\n", chip_id);
		return -EINVAL;
	}

	dev_info(dev, "Detected OV08d10 chip id: 0x%04x: sensor\n", chip_id);

	return 0;
}

static int ov08d10_configure_regulators(struct ov08d10 *ov08d10)
{
	unsigned int i;

	for (i = 0; i < OV08D10_NUM_SUPPLIES; i++)
		ov08d10->supplies[i].supply = ov08d10_supply_names[i];

	return devm_regulator_bulk_get(&ov08d10->client->dev,
				      OV08D10_NUM_SUPPLIES,
				      ov08d10->supplies);
}

static int ov08d10_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov08d10 *ov08d10;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
			 DRIVER_VERSION >> 16,
			 (DRIVER_VERSION & 0xff00) >> 8,
			 DRIVER_VERSION & 0x00ff);

	ov08d10 = devm_kzalloc(dev, sizeof(*ov08d10), GFP_KERNEL);
	if (!ov08d10)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				  &ov08d10->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				      &ov08d10->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				      &ov08d10->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				      &ov08d10->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE,
				  &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}
	ov08d10->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < ov08d10->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			ov08d10->cur_mode = &supported_modes[i];
			break;
		}
	}
	ov08d10->client = client;

	ov08d10->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov08d10->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ov08d10->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ov08d10->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov08d10->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
	if (IS_ERR(ov08d10->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ov08d10->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov08d10->pinctrl)) {
		ov08d10->pins_default =
			pinctrl_lookup_state(ov08d10->pinctrl,
					    OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov08d10->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov08d10->pins_sleep =
			pinctrl_lookup_state(ov08d10->pinctrl,
					    OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov08d10->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = ov08d10_configure_regulators(ov08d10);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ov08d10->mutex);

	sd = &ov08d10->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov08d10_subdev_ops);
	ret = ov08d10_initialize_controls(ov08d10);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov08d10_power_on(ov08d10);
	if (ret)
		goto err_free_handler;

	ret = ov08d10_check_sensor_id(ov08d10, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov08d10_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov08d10->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov08d10->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov08d10->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
			 ov08d10->module_index, facing,
			 OV08D10_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ov08d10_power_off(ov08d10);
err_free_handler:
	v4l2_ctrl_handler_free(&ov08d10->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov08d10->mutex);

	return ret;
}

static int ov08d10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov08d10 *ov08d10 = to_ov08d10(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov08d10->ctrl_handler);
	mutex_destroy(&ov08d10->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov08d10_power_off(ov08d10);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov08d10_of_match[] = {
	{ .compatible = "ovti,ov08d10" },
	{},
};
MODULE_DEVICE_TABLE(of, ov08d10_of_match);
#endif

static const struct i2c_device_id ov08d10_match_id[] = {
	{ "ovti,ov08d10", 0 },
	{ },
};

static struct i2c_driver ov08d10_i2c_driver = {
	.driver = {
		.name = OV08D10_NAME,
		.pm = &ov08d10_pm_ops,
		.of_match_table = of_match_ptr(ov08d10_of_match),
	},
	.probe		= ov08d10_probe,
	.remove		= ov08d10_remove,
	.id_table	= ov08d10_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(ov08d10_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov08d10_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov08d10_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("OmniVision ov08d10 sensor driver");
MODULE_LICENSE("GPL");
