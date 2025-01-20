// SPDX-License-Identifier: GPL-2.0
/*
 * mis8001 driver
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
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
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif
#define MIPI_FREQ_720M			720000000
#define MIPI_FREQ_320M			320000000
#define MIPI_FREQ_223M			223000000
#define MIPI_FREQ_371M			371500000
#define MIPI_FREQ_143M			143500000

#define MIS8001_LANES			4
#define MIS8001_BITS_PER_SAMPLE		10
#define MIS8001_LINK_FREQ		MIPI_FREQ_720M  //mipiclk

#define PIXEL_RATE_WITH_315M_10BIT	(MIPI_FREQ_720M / MIS8001_BITS_PER_SAMPLE * \
					2 * MIS8001_LANES)
#define MIS8001_XVCLK_FREQ		27000000

#define CHIP_ID				0x0000
#define MIS8001_REG_CHIP_ID		0x0215

#define MIS8001_REG_CTRL_MODE		0x0000
#define MIS8001_MODE_SW_STANDBY		0x10
#define MIS8001_MODE_STREAMING		0x1a

#define MIS8001_REG_EXPOSURE_H		0x000b
#define MIS8001_REG_EXPOSURE_M		0x000a
#define MIS8001_REG_EXPOSURE_L		0x0009
#define	MIS8001_EXPOSURE_MIN		2
#define	MIS8001_EXPOSURE_STEP		1
#define MIS8001_FETCH_EXP_H_2H(VAL)	(((VAL) >> 16) & 0x03)
#define MIS8001_FETCH_EXP_H(VAL)	(((VAL) >> 8) & 0xFF)
#define MIS8001_FETCH_EXP_L(VAL)	((VAL) & 0xFF)

#define MIS8001_REG_ANA_GAIN		0x0212

#define MIS8001_ONCE_GAIN_STEP		0x20
#define MIS8001_GAIN_MIN		MIS8001_ONCE_GAIN_STEP
#define MIS8001_GAIN_MAX		(MIS8001_ONCE_GAIN_STEP * 32)	//31.62x32 = 1011
#define MIS8001_GAIN_STEP		1
#define MIS8001_GAIN_DEFAULT		MIS8001_GAIN_MIN

#define MIS8001_VTS_MAX			0xffff
#define MIS8001_REG_VTS_H		0x0008
#define MIS8001_REG_VTS_M		0x0007
#define MIS8001_REG_VTS_L		0x0006
#define MIS8001_FETCH_VTS_H_2H(VAL)	(((VAL) >> 16) & 0x03)
#define MIS8001_FETCH_VTS_H(VAL)	(((VAL) >> 8) & 0xFF)
#define MIS8001_FETCH_VTS_L(VAL)	((VAL) & 0xFF)

#define MIS8001_REG_TEST_PATTERN	0x3500
#define MIS8001_TEST_PATTERN_BIT_MASK	BIT(0)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define MIS8001_REG_VALUE_08BIT		1
#define MIS8001_REG_VALUE_16BIT		2
#define MIS8001_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define MIS8001_NAME			"mis8001"

static const char *const mis8001_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define MIS8001_NUM_SUPPLIES ARRAY_SIZE(mis8001_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct mis8001_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct mis8001 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[MIS8001_NUM_SUPPLIES];

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
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct mis8001_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
};

#define to_mis8001(sd) container_of(sd, struct mis8001, subdev)

/*
 * Xclk 27Mhz
 */
static const struct regval mis8001_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane Mbps, 4lane
 * MIS8001_NO4_Mclk27M_Pclk37P125M_3840x2160_FH2250FW550_30fps_MIPI4lane_RAW10
 */
static const struct regval mis8001_linear_10_3840x2160_regs[] = {
	{0x0000, 0x10},
	{0x0001, 0x00},
	{0x0002, 0x00},
	{0x0003, 0x00},
	{0x0004, 0x20},
	{0x0005, 0x08},
	{0x0006, 0xCa},
	{0x0007, 0x08},
	{0x0008, 0x00},
	{0x0009, 0x63},
	{0x000A, 0x04},
	{0x000B, 0x00},
	{0x000C, 0x0F},
	{0x000D, 0xB0},
	{0x000E, 0x03},
	{0x000F, 0x0F},
	{0x0010, 0xB0},
	{0x0011, 0x03},
	{0x0012, 0x0F},
	{0x0013, 0xB0},
	{0x0014, 0x03},
	{0x0015, 0x00},
	{0x0016, 0x00},
	{0x0017, 0x3C},
	{0x0018, 0x02},
	{0x0019, 0x3C},
	{0x001A, 0x02},
	{0x001B, 0x3C},
	{0x001C, 0x02},
	{0x001D, 0x16},
	{0x001E, 0x2D},
	{0x001F, 0x16},
	{0x0020, 0x16},
	{0x0021, 0x00},
	{0x0022, 0x00},
	{0x0023, 0x00},
	{0x0024, 0x00},
	{0x0025, 0x00},
	{0x0026, 0x00},
	{0x0027, 0x00},
	{0x0028, 0x00},
	{0x0029, 0x00},
	{0x002A, 0x61},
	{0x002B, 0x00},
	{0x002C, 0x03},
	{0x002D, 0x03},
	{0x002E, 0x65},
	{0x002F, 0x50},
	{0x0030, 0x09},
	{0x0031, 0x01},
	{0x0032, 0x58},
	{0x0033, 0x82},
	{0x0034, 0x00},
	{0x0035, 0x67},
	{0x0036, 0x4A},
	{0x0037, 0x0A},
	{0x0038, 0x01},
	{0x0039, 0x58},
	{0x003A, 0x82},
	{0x003B, 0x00},
	{0x003C, 0x03},
	{0x003D, 0x16},
	{0x003E, 0x00},
	{0x003F, 0x00},
	{0x0040, 0x28},
	{0x0041, 0x05},
	{0x0042, 0x0C},
	{0x0043, 0x01},
	{0x0044, 0x00},
	{0x0045, 0xD0},
	{0x0046, 0x00},
	{0x0047, 0x00},
	{0x0048, 0x00},
	{0x0049, 0x00},
	{0x004A, 0x00},
	{0x004B, 0x00},
	{0x004C, 0x00},
	{0x004D, 0x00},
	{0x004E, 0x06},
	{0x004F, 0xB8},
	{0x0050, 0x0E},
	{0x0051, 0x00},
	{0x0052, 0x06},
	{0x0053, 0x10},
	{0x0054, 0xF0},
	{0x0055, 0x08},
	{0x0056, 0x22},
	{0x0057, 0xD0},
	{0x0058, 0x09},
	{0x0059, 0xFF},
	{0x005A, 0xFF},
	{0x005B, 0xFF},
	{0x005C, 0xFF},
	{0x005D, 0xFF},
	{0x005E, 0xFF},
	{0x005F, 0xFF},
	{0x0060, 0xFF},
	{0x0061, 0xFF},
	{0x0062, 0xFF},
	{0x0063, 0xFF},
	{0x0064, 0xFF},
	{0x0065, 0xFF},
	{0x0066, 0xFF},
	{0x0067, 0xFF},
	{0x0068, 0xFF},
	{0x0069, 0xFF},
	{0x006A, 0xFF},
	{0x006B, 0xFF},
	{0x006C, 0xFF},
	{0x006D, 0xFF},
	{0x006E, 0x16},
	{0x006F, 0x18},
	{0x0070, 0x10},
	{0x0071, 0x13},
	/* 2560 * 1440 */
	//{0x0072,0xc0},
	//{0x0073,0x00},
	//{0x0074,0x87},
	//{0x0075,0xFb},
	//{0x0084,0x29},
	//{0x0085,0xc9},
	//{0x01BE,0x00},
	//{0x01BF,0x0A},
	//{0x01C0,0xA0},
	//{0x01C1,0x05},
	/* 3840 * 2160 */
	{0x0072, 0x0C},
	{0x0073, 0x00},
	{0x0074, 0x3B},
	{0x0075, 0xFC},
	{0x0084, 0x01},
	{0x0085, 0xF1},
	{0x01BE, 0x00},
	{0x01BF, 0x0F},
	{0x01C0, 0x70},
	{0x01C1, 0x08},

	{0x0076, 0x26},
	{0x0077, 0x02},
	{0x0078, 0x05},
	{0x0079, 0x00},
	{0x007A, 0x07},
	{0x007B, 0x00},
	{0x007C, 0x09},
	{0x007D, 0x00},
	{0x007E, 0x12},
	{0x007F, 0x00},
	{0x0080, 0x99},
	{0x0081, 0x00},
	{0x0082, 0x16},
	{0x0083, 0x50},
	{0x0086, 0x00},
	{0x0087, 0x00},
	{0x0088, 0x00},
	{0x0089, 0x00},
	{0x008A, 0x00},
	{0x008B, 0x0A},
	{0x008C, 0x00},
	{0x008D, 0x00},
	{0x008E, 0x00},
	{0x008F, 0x00},
	{0x0090, 0x00},
	{0x0091, 0x00},
	{0x0092, 0x00},
	{0x0093, 0x00},
	{0x0094, 0x40},
	{0x0095, 0x18},
	{0x0096, 0x00},
	{0x0097, 0x00},
	{0x0098, 0x00},
	{0x0099, 0x00},
	{0x009A, 0x44},
	{0x009B, 0xB0},
	{0x009C, 0x07},
	{0x009D, 0x07},
	{0x009E, 0x80},
	{0x009F, 0x07},
	{0x00A0, 0x45},
	{0x00A1, 0x90},
	{0x00A2, 0x04},
	{0x00A3, 0x08},
	{0x00A4, 0xC0},
	{0x00A5, 0x00},
	{0x00A6, 0x1B},
	{0x00A7, 0x40},
	{0x00A8, 0x08},
	{0x00A9, 0xA6},
	{0x00AA, 0x10},
	{0x00AB, 0x00},
	{0x00AC, 0x57},
	{0x00AD, 0xA0},
	{0x00AE, 0x07},
	{0x00AF, 0x37},
	{0x00B0, 0x70},
	{0x00B1, 0x07},
	{0x00B2, 0x00},
	{0x00B3, 0x00},
	{0x00B4, 0x00},
	{0x00B5, 0x00},
	{0x00B6, 0x00},
	{0x00B7, 0x00},
	{0x00B8, 0x1B},
	{0x00B9, 0x40},
	{0x00BA, 0x08},
	{0x00BB, 0xA6},
	{0x00BC, 0x10},
	{0x00BD, 0x00},
	{0x00BE, 0x8C},
	{0x00BF, 0x70},
	{0x00C0, 0x0A},
	{0x00C1, 0xFF},
	{0x00C2, 0xFF},
	{0x00C3, 0xFF},
	{0x00C4, 0x8C},
	{0x00C5, 0x70},
	{0x00C6, 0x0A},
	{0x00C7, 0xFF},
	{0x00C8, 0xFF},
	{0x00C9, 0xFF},
	{0x00CA, 0x8C},
	{0x00CB, 0x70},
	{0x00CC, 0x0A},
	{0x00CD, 0xFF},
	{0x00CE, 0xFF},
	{0x00CF, 0xFF},
	{0x00D0, 0x0C},
	{0x00D1, 0xE0},
	{0x00D2, 0x01},
	{0x00D3, 0xFF},
	{0x00D4, 0xFF},
	{0x00D5, 0xFF},
	{0x00D6, 0x00},
	{0x00D7, 0xF0},
	{0x00D8, 0xFF},
	{0x00D9, 0xFF},
	{0x00DA, 0xFF},
	{0x00DB, 0xFF},
	{0x00DC, 0x00},
	{0x00DD, 0x00},
	{0x00DE, 0x01},
	{0x00DF, 0x60},
	{0x00E0, 0x09},
	{0x00E1, 0x8C},
	{0x00E2, 0xB0},
	{0x00E3, 0x09},
	{0x00E4, 0x7B},
	{0x00E5, 0xA0},
	{0x00E6, 0x07},
	{0x00E7, 0x58},
	{0x00E8, 0x90},
	{0x00E9, 0x07},
	{0x00EA, 0x38},
	{0x00EB, 0x60},
	{0x00EC, 0x07},
	{0x00ED, 0xFF},
	{0x00EE, 0xFF},
	{0x00EF, 0xFF},
	{0x00F0, 0xFF},
	{0x00F1, 0xFF},
	{0x00F2, 0xFF},
	{0x00F3, 0xFF},
	{0x00F4, 0xFF},
	{0x00F5, 0xFF},
	{0x00F6, 0xFF},
	{0x00F7, 0xFF},
	{0x00F8, 0xFF},
	{0x00F9, 0x00},
	{0x00FA, 0x00},
	{0x00FB, 0x01},
	{0x00FC, 0x10},
	{0x00FD, 0x00},
	{0x00FE, 0xFF},
	{0x00FF, 0x1F},
	{0x0100, 0x00},
	{0x0101, 0x00},
	{0x0102, 0xF0},
	{0x0103, 0xFF},
	{0x0104, 0xFF},
	{0x0105, 0x3F},
	{0x0106, 0x00},
	{0x0107, 0x85},
	{0x0108, 0x60},
	{0x0109, 0x08},
	{0x010A, 0x02},
	{0x010B, 0xF0},
	{0x010C, 0x03},
	{0x010D, 0x08},
	{0x010E, 0x00},
	{0x010F, 0x00},
	{0x0110, 0x00},
	{0x0111, 0x80},
	{0x0112, 0x00},
	{0x0113, 0x40},
	{0x0114, 0x00},
	{0x0115, 0x00},
	{0x0116, 0x00},
	{0x0117, 0x80},
	{0x0118, 0x00},
	{0x0119, 0x49},
	{0x011A, 0x00},
	{0x011B, 0x00},
	{0x011C, 0x00},
	{0x011D, 0xA0},
	{0x011E, 0x04},
	{0x011F, 0x08},
	{0x0120, 0x00},
	{0x0121, 0x00},
	{0x0122, 0x00},
	{0x0123, 0x00},
	{0x0124, 0x00},
	{0x0125, 0x00},
	{0x0126, 0x00},
	{0x0127, 0x00},
	{0x0128, 0x00},
	{0x0129, 0x80},
	{0x012A, 0x05},
	{0x012B, 0x7A},
	{0x012C, 0x80},
	{0x012D, 0x03},
	{0x012E, 0x77},
	{0x012F, 0x70},
	{0x0130, 0x05},
	{0x0131, 0x7B},
	{0x0132, 0x70},
	{0x0133, 0x03},
	{0x0134, 0x78},
	{0x0135, 0x00},
	{0x0136, 0x00},
	{0x0137, 0x00},
	{0x0138, 0x00},
	{0x0139, 0x00},
	{0x013A, 0x00},
	{0x013B, 0x00},
	{0x013C, 0x00},
	{0x013D, 0x00},
	{0x013E, 0xF0},
	{0x013F, 0xFF},
	{0x0140, 0xFF},
	{0x0141, 0x0F},
	{0x0142, 0x00},
	{0x0143, 0x00},
	{0x0144, 0xF0},
	{0x0145, 0xFF},
	{0x0146, 0xFF},
	{0x0147, 0x0F},
	{0x0148, 0x02},
	{0x0149, 0xFF},
	{0x014A, 0x0F},
	{0x014B, 0x00},
	{0x014C, 0x00},
	{0x014D, 0xF0},
	{0x014E, 0xFF},
	{0x014F, 0x00},
	{0x0150, 0x00},
	{0x0151, 0x00},
	{0x0152, 0x00},
	{0x0153, 0x00},
	{0x0154, 0x00},
	{0x0155, 0x00},
	{0x0156, 0x00},
	{0x0157, 0x00},
	{0x0158, 0x00},
	{0x0159, 0x00},
	{0x015A, 0x00},
	{0x015B, 0x00},
	{0x015C, 0x00},
	{0x015D, 0x00},
	{0x015E, 0x00},
	{0x015F, 0x00},
	{0x0160, 0x00},
	{0x0161, 0x08},
	{0x0162, 0x40},
	{0x0163, 0x08},
	{0x0164, 0xA6},
	{0x0165, 0x10},
	{0x0166, 0x00},
	{0x0167, 0x08},
	{0x0168, 0x40},
	{0x0169, 0x08},
	{0x016A, 0xA6},
	{0x016B, 0x10},
	{0x016C, 0x00},
	{0x016D, 0x51},
	{0x016E, 0x30},
	{0x016F, 0x05},
	{0x0170, 0x31},
	{0x0171, 0x30},
	{0x0172, 0x03},
	{0x0173, 0x55},
	{0x0174, 0xA0},
	{0x0175, 0x07},
	{0x0176, 0x35},
	{0x0177, 0x70},
	{0x0178, 0x07},
	{0x0179, 0x00},
	{0x017A, 0x00},
	{0x017B, 0x00},
	{0x017C, 0x2B},
	{0x017D, 0x15},
	{0x017E, 0x6B},
	{0x017F, 0xAB},
	{0x0180, 0x00},
	{0x0181, 0x00},
	{0x0182, 0x00},
	{0x0183, 0x00},
	{0x0184, 0x00},
	{0x0185, 0x00},
	{0x0186, 0x00},
	{0x0187, 0x00},
	{0x0188, 0x00},
	{0x0189, 0x00},
	{0x018A, 0x00},
	{0x018B, 0x01},
	{0x018C, 0x00},
	{0x018D, 0x00},
	{0x018E, 0x08},
	{0x018F, 0x06},
	{0x0190, 0x00},
	{0x0191, 0x05},
	{0x0192, 0x02},
	{0x0193, 0x0B},
	{0x0194, 0x00},
	{0x0195, 0x00},
	{0x0196, 0xFF},
	{0x0197, 0x03},
	{0x0198, 0xFF},
	{0x0199, 0xFF},
	{0x019A, 0xFF},
	{0x019B, 0xFF},
	{0x019C, 0x00},
	{0x019D, 0x00},
	{0x019E, 0x00},
	{0x019F, 0x00},
	{0x01A0, 0x00},
	{0x01A1, 0x00},
	{0x01A2, 0x00},
	{0x01A3, 0x00},
	{0x01A4, 0x00},
	{0x01A5, 0x00},
	{0x01A6, 0x00},
	{0x01A7, 0x00},
	{0x01A8, 0x00},
	{0x01A9, 0x00},
	{0x01AA, 0x00},
	{0x01AB, 0x00},
	{0x01AC, 0x00},
	{0x01AD, 0x00},
	{0x01AE, 0x00},
	{0x01AF, 0x40},
	{0x01B0, 0x00},
	{0x01B1, 0x00},
	{0x01B2, 0x00},
	{0x01B3, 0x00},
	{0x01B4, 0x00},
	{0x01B5, 0x00},
	{0x01B6, 0x00},
	{0x01B7, 0x00},
	{0x01B8, 0x00},
	{0x01B9, 0x00},
	{0x01BA, 0x08},
	{0x01BB, 0x00},
	{0x01BC, 0x00},
	{0x01BD, 0x00},
	{0x01C2, 0x00},
	{0x01C3, 0x0F},
	{0x01C4, 0x70},
	{0x01C5, 0x08},
	{0x01C6, 0x00},
	{0x01C7, 0x0F},
	{0x01C8, 0x70},
	{0x01C9, 0x08},
	{0x01CA, 0x80},
	{0x01CB, 0x02},
	{0x01CC, 0xE0},
	{0x01CD, 0x01},
	{0x01CE, 0x0F},
	{0x01CF, 0x02},
	{0x01D0, 0x00},
	{0x01D1, 0xF0},
	{0x01D2, 0xFF},
	{0x01D3, 0x70},
	{0x01D4, 0x10},
	{0x01D5, 0x00},
	{0x01D6, 0x02},
	{0x01D7, 0x00},
	{0x01D8, 0x50},
	{0x01D9, 0x00},
	{0x01DA, 0xDC},
	{0x01DB, 0x00},
	{0x01DC, 0x3C},
	{0x01DD, 0x00},
	{0x01DE, 0x00},
	{0x01DF, 0x08},
	{0x01E0, 0x3C},
	{0x01E1, 0x34},
	{0x01E2, 0x28},
	{0x01E3, 0x04},
	{0x01E4, 0x69},
	{0x01E5, 0x06},
	{0x01E6, 0x3C},
	{0x01E7, 0x04},
	{0x01E8, 0x64},
	{0x01E9, 0x00},
	{0x01EA, 0x32},
	{0x01EB, 0x00},
	{0x01EC, 0xFF},
	{0x01ED, 0xFF},
	{0x01EE, 0xFF},
	{0x01EF, 0xFF},
	{0x01F0, 0xFF},
	{0x01F1, 0xFF},
	{0x01F2, 0xFF},
	{0x01F3, 0xFF},
	{0x01F4, 0x8C},
	{0x01F5, 0x00},
	{0x01F6, 0xFF},
	{0x01F7, 0x00},
	{0x01F8, 0x00},
	{0x01F9, 0x00},
	{0x01FA, 0x00},
	{0x01FB, 0x1E},
	{0x01FC, 0x07},
	{0x01FD, 0x00},
	{0x01FE, 0x00},
	{0x01FF, 0xFF},
	{0x0200, 0x80},
	{0x0201, 0x02},
	{0x0202, 0xE0},
	{0x0203, 0x01},
	{0x0204, 0x03},
	{0x0205, 0x05},
	{0x0206, 0x99},
	{0x0207, 0x50},
	{0x0208, 0x00},
	{0x0209, 0xFF},
	{0x020A, 0x00},
	{0x020B, 0x00},
	{0x020C, 0x00},
	{0x020D, 0x00},
	{0x020E, 0x00},
	{0x020F, 0x0F},
	{0x0210, 0x00},
	{0x0211, 0x00},
	{0x0212, 0x00},
	{0x0213, 0x00},
	{0x0214, 0x00},
	//{0x0000,0x1A},
	{REG_NULL, 0x00},
};

static const struct mis8001_mode supported_modes[] = {
	{
		// .width = 2560,
		// .height = 1440,
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0052,
		.hts_def = 550,
		.vts_def = 0x8ca,
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		.reg_list = mis8001_linear_10_3840x2160_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	MIS8001_LINK_FREQ
};

static const char *const mis8001_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int mis8001_write_reg(struct i2c_client *client, u16 reg,
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

static int mis8001_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY)
			mdelay(regs[i].val);
		else {
			ret = mis8001_write_reg(client, regs[i].addr,
						MIS8001_REG_VALUE_08BIT, regs[i].val);
			if (ret) {
				dev_err(&(client->dev), "mis8001_write_reg ret:%d, addre-value %x--%x\n",
					ret, regs[i].addr, regs[i].val);
			}
		}
	}

	return ret;
}

/* Read registers up to 4 at a time */
static int mis8001_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

struct gain_reg_map {
	int gain;
	unsigned char reg_value;
};

static const struct gain_reg_map gain_reg_table[] = {
	{1, 0},
	{2, 20},
	{3, 32},
	{4, 40},
	{5, 47},
	{6, 52},
	{7, 56},
	{8, 60},
	{9, 64},
	{10, 67},
	{11, 69},
	{12, 72},
	{13, 74},
	{14, 76},
	{15, 78},
	{16, 80},
	{17, 82},
	{18, 84},
	{19, 85},
	{20, 87},
	{21, 88},
	{22, 89},
	{23, 91},
	{24, 92},
	{25, 93},
	{26, 94},
	{27, 95},
	{28, 96},
	{29, 97},
	{30, 98},
	{31, 99},
	{32, 100}
};

static int mis8001_set_gain_reg(struct mis8001 *mis8001, u32 gain)
{
	u32 adc_again = 0;
	int ret = 0;
	int i = 0;
	struct device *dev = &mis8001->client->dev;

	gain = gain >> 5;

	if (gain < MIS8001_ONCE_GAIN_STEP)
		adc_again = MIS8001_ONCE_GAIN_STEP;
	else if (gain > MIS8001_GAIN_MAX - 1)
		adc_again = MIS8001_GAIN_MAX - 1;

	if (gain < MIS8001_GAIN_MAX)
		adc_again = gain;

	for (i = 0; i < ARRAY_SIZE(gain_reg_table); i++) {
		if (gain_reg_table[i].gain <= adc_again) {
			ret = mis8001_write_reg(mis8001->client,
						MIS8001_REG_ANA_GAIN,
						MIS8001_REG_VALUE_08BIT,
						gain_reg_table[i].reg_value);
		}
	}

	// adc_again = 10 / 3 * adc_again / MIS8001_ONCE_GAIN_STEP;
	dev_dbg(dev, "gain - adc_again --- %d - %d\n", gain, adc_again);

	return ret;
}

static int mis8001_get_reso_dist(const struct mis8001_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct mis8001_mode *
mis8001_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = mis8001_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int mis8001_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	const struct mis8001_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&mis8001->mutex);

	mode = mis8001_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&mis8001->mutex);
		return -ENOTTY;
#endif
	} else {
		mis8001->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(mis8001->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(mis8001->vblank, vblank_def,
					 MIS8001_VTS_MAX - mode->height,
					 1, vblank_def);
		mis8001->cur_fps = mode->max_fps;
	}

	mutex_unlock(&mis8001->mutex);

	return 0;
}

static int mis8001_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	const struct mis8001_mode *mode = mis8001->cur_mode;

	mutex_lock(&mis8001->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&mis8001->mutex);
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
	mutex_unlock(&mis8001->mutex);

	return 0;
}

static int mis8001_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mis8001 *mis8001 = to_mis8001(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = mis8001->cur_mode->bus_fmt;

	return 0;
}

static int mis8001_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}



static int mis8001_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	const struct mis8001_mode *mode = mis8001->cur_mode;

	if (mis8001->streaming)
		fi->interval = mis8001->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int mis8001_g_mbus_config(struct v4l2_subdev *sd,
				 unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	const struct mis8001_mode *mode = mis8001->cur_mode;
	u32 val = 1 << (MIS8001_LANES - 1) |
		  V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;

	config->type = V4L2_MBUS_CSI2_DPHY;
	//config->type = V4L2_MBUS_CSI2; TODO silingjie
	config->flags = val;

	return 0;
}

static void mis8001_get_module_inf(struct mis8001 *mis8001,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, MIS8001_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, mis8001->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, mis8001->len_name, sizeof(inf->base.lens));
}

static long mis8001_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_GET_MODULE_INFO:
		mis8001_get_module_inf(mis8001, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = mis8001->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = mis8001->cur_mode->width;
		h = mis8001->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				mis8001->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&mis8001->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = mis8001->cur_mode->hts_def - mis8001->cur_mode->width;
			h = mis8001->cur_mode->vts_def - mis8001->cur_mode->height;
			__v4l2_ctrl_modify_range(mis8001->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(mis8001->vblank, h,
						 MIS8001_VTS_MAX - mis8001->cur_mode->height, 1, h);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = mis8001_write_reg(mis8001->client, MIS8001_REG_CTRL_MODE,
						MIS8001_REG_VALUE_08BIT, MIS8001_MODE_STREAMING);
		else
			ret = mis8001_write_reg(mis8001->client, MIS8001_REG_CTRL_MODE,
						MIS8001_REG_VALUE_08BIT, MIS8001_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long mis8001_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = mis8001_ioctl(sd, cmd, inf);
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

		ret = mis8001_ioctl(sd, cmd, hdr);
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
			ret = mis8001_ioctl(sd, cmd, hdr);
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
			ret = mis8001_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = mis8001_ioctl(sd, cmd, &stream);
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

static int __mis8001_start_stream(struct mis8001 *mis8001)
{
	int ret;

	ret = mis8001_write_array(mis8001->client, mis8001->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&mis8001->ctrl_handler);
	if (ret)
		return ret;

	return mis8001_write_reg(mis8001->client, MIS8001_REG_CTRL_MODE,
				 MIS8001_REG_VALUE_08BIT, MIS8001_MODE_STREAMING);
}

static int __mis8001_stop_stream(struct mis8001 *mis8001)
{
	return mis8001_write_reg(mis8001->client, MIS8001_REG_CTRL_MODE,
				 MIS8001_REG_VALUE_08BIT, MIS8001_MODE_SW_STANDBY);
}

static int mis8001_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	struct i2c_client *client = mis8001->client;
	int ret = 0;

	mutex_lock(&mis8001->mutex);
	on = !!on;
	if (on == mis8001->streaming)
		goto unlock_and_return;

	if (on) {

		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __mis8001_start_stream(mis8001);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__mis8001_stop_stream(mis8001);
		pm_runtime_put(&client->dev);
	}

	mis8001->streaming = on;

unlock_and_return:
	mutex_unlock(&mis8001->mutex);

	return ret;
}

static int mis8001_s_power(struct v4l2_subdev *sd, int on)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	struct i2c_client *client = mis8001->client;
	int ret = 0;

	mutex_lock(&mis8001->mutex);

	/* If the power state is not modified - no work to do. */
	if (mis8001->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = mis8001_write_array(mis8001->client, mis8001_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		mis8001->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		mis8001->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&mis8001->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 mis8001_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, MIS8001_XVCLK_FREQ / 1000 / 1000);
}

static int __mis8001_power_on(struct mis8001 *mis8001)
{
	int ret;
	u32 delay_us;
	struct device *dev = &mis8001->client->dev;

	if (!IS_ERR_OR_NULL(mis8001->pins_default)) {
		ret = pinctrl_select_state(mis8001->pinctrl,
					   mis8001->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(mis8001->xvclk, MIS8001_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(mis8001->xvclk) != MIS8001_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(mis8001->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(mis8001->reset_gpio))
		gpiod_set_value(mis8001->reset_gpio, 0);

	ret = regulator_bulk_enable(MIS8001_NUM_SUPPLIES, mis8001->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	usleep_range(5000, 6000);
	if (!IS_ERR(mis8001->reset_gpio))
		gpiod_set_value(mis8001->reset_gpio, 1);

	// usleep_range(5000, 6000);
	//if (!IS_ERR(mis8001->pwdn_gpio))
	//	gpiod_set_value_cansleep(mis8001->pwdn_gpio, 1);

	if (!IS_ERR(mis8001->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = mis8001_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

disable_clk:
	clk_disable_unprepare(mis8001->xvclk);

	return ret;
}

static void __mis8001_power_off(struct mis8001 *mis8001)
{
	int ret;
	struct device *dev = &mis8001->client->dev;

	if (!IS_ERR(mis8001->pwdn_gpio))
		gpiod_set_value_cansleep(mis8001->pwdn_gpio, 0);
	clk_disable_unprepare(mis8001->xvclk);
	if (!IS_ERR(mis8001->reset_gpio))
		gpiod_set_value_cansleep(mis8001->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(mis8001->pins_sleep)) {
		ret = pinctrl_select_state(mis8001->pinctrl,
					   mis8001->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(MIS8001_NUM_SUPPLIES, mis8001->supplies);
}

static int __maybe_unused mis8001_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis8001 *mis8001 = to_mis8001(sd);

	return __mis8001_power_on(mis8001);
}

static int __maybe_unused mis8001_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis8001 *mis8001 = to_mis8001(sd);

	__mis8001_power_off(mis8001);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int mis8001_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mis8001 *mis8001 = to_mis8001(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct mis8001_mode *def_mode = &supported_modes[0];

	mutex_lock(&mis8001->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&mis8001->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int mis8001_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops mis8001_pm_ops = {
	SET_RUNTIME_PM_OPS(mis8001_runtime_suspend,
	mis8001_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops mis8001_internal_ops = {
	.open = mis8001_open,
};
#endif

static const struct v4l2_subdev_core_ops mis8001_core_ops = {
	.s_power = mis8001_s_power,
	.ioctl = mis8001_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = mis8001_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops mis8001_video_ops = {
	.s_stream = mis8001_s_stream,
	.g_frame_interval = mis8001_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops mis8001_pad_ops = {
	.enum_mbus_code = mis8001_enum_mbus_code,
	.enum_frame_size = mis8001_enum_frame_sizes,
	.enum_frame_interval = mis8001_enum_frame_interval,
	.get_fmt = mis8001_get_fmt,
	.set_fmt = mis8001_set_fmt,
	.get_mbus_config = mis8001_g_mbus_config,
};

static const struct v4l2_subdev_ops mis8001_subdev_ops = {
	.core = &mis8001_core_ops,
	.video = &mis8001_video_ops,
	.pad = &mis8001_pad_ops,
};

static void mis8001_modify_fps_info(struct mis8001 *mis8001)
{
	const struct mis8001_mode *mode = mis8001->cur_mode;

	mis8001->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       mis8001->cur_vts;
}

static int mis8001_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mis8001 *mis8001 = container_of(ctrl->handler,
					       struct mis8001, ctrl_handler);
	struct i2c_client *client = mis8001->client;
	s64 max;
	u32 vts;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = mis8001->cur_mode->height + ctrl->val - 10;
		__v4l2_ctrl_modify_range(mis8001->exposure,
					 mis8001->exposure->minimum, max,
					 mis8001->exposure->step,
					 mis8001->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_info(&client->dev, "vts 0x%x, set exposure 0x%x\n",
			 mis8001->cur_vts, ctrl->val);
		if (mis8001->cur_mode->hdr_mode == NO_HDR) {
			val = mis8001->cur_vts - ctrl->val;
			/* 4 least significant bits of expsoure are fractional part */
			ret = mis8001_write_reg(mis8001->client,
						MIS8001_REG_EXPOSURE_H,
						MIS8001_REG_VALUE_08BIT,
						MIS8001_FETCH_EXP_H_2H(val));
			ret |= mis8001_write_reg(mis8001->client,
						 MIS8001_REG_EXPOSURE_M,
						 MIS8001_REG_VALUE_08BIT,
						 MIS8001_FETCH_EXP_H(val));
			ret |= mis8001_write_reg(mis8001->client,
						 MIS8001_REG_EXPOSURE_L,
						 MIS8001_REG_VALUE_08BIT,
						 MIS8001_FETCH_EXP_L(val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (mis8001->cur_mode->hdr_mode == NO_HDR)
			ret |= mis8001_set_gain_reg(mis8001, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_info(&client->dev, "set vblank 0x%x mis8001->cur_vts 0x%x\n",
			 ctrl->val, mis8001->cur_vts);
		vts = ctrl->val + mis8001->cur_mode->height;
		ret |= mis8001_write_reg(mis8001->client, MIS8001_REG_VTS_H,
					 MIS8001_REG_VALUE_08BIT,
					 MIS8001_FETCH_VTS_H_2H(vts));
		ret |= mis8001_write_reg(mis8001->client,
					 MIS8001_REG_VTS_M,
					 MIS8001_REG_VALUE_08BIT,
					 MIS8001_FETCH_VTS_H(vts));
		ret |= mis8001_write_reg(mis8001->client,
					 MIS8001_REG_VTS_L,
					 MIS8001_REG_VALUE_08BIT,
					 MIS8001_FETCH_VTS_L(vts));
		mis8001->cur_vts = vts;
		if (mis8001->cur_vts != mis8001->cur_mode->vts_def)
			mis8001_modify_fps_info(mis8001);
		break;
	case V4L2_CID_TEST_PATTERN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops mis8001_ctrl_ops = {
	.s_ctrl = mis8001_set_ctrl,
};

static int mis8001_initialize_controls(struct mis8001 *mis8001)
{
	const struct mis8001_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &mis8001->ctrl_handler;
	mode = mis8001->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &mis8001->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_315M_10BIT, 1, PIXEL_RATE_WITH_315M_10BIT);

	h_blank = mode->hts_def - mode->width;
	mis8001->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (mis8001->hblank)
		mis8001->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	mis8001->vblank = v4l2_ctrl_new_std(handler, &mis8001_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    MIS8001_VTS_MAX - mode->height,
					    1, vblank_def);
	mis8001->cur_fps = mode->max_fps;
	exposure_max = mode->vts_def - 8;
	mis8001->exposure = v4l2_ctrl_new_std(handler, &mis8001_ctrl_ops,
					      V4L2_CID_EXPOSURE, MIS8001_EXPOSURE_MIN,
					      exposure_max, MIS8001_EXPOSURE_STEP,
					      mode->exp_def);
	mis8001->anal_gain = v4l2_ctrl_new_std(handler, &mis8001_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, MIS8001_GAIN_MIN,
					       MIS8001_GAIN_MAX, MIS8001_GAIN_STEP,
					       MIS8001_GAIN_DEFAULT);
	mis8001->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&mis8001_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(mis8001_test_pattern_menu) - 1,
				0, 0, mis8001_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &mis8001_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &mis8001_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&mis8001->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	mis8001->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int mis8001_check_sensor_id(struct mis8001 *mis8001,
				   struct i2c_client *client)
{
	struct device *dev = &mis8001->client->dev;
	u32 id = 0;
	int ret;

	ret = mis8001_read_reg(client, MIS8001_REG_CHIP_ID,
			       MIS8001_REG_VALUE_16BIT, &id);
	if (id == 0x0) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected mis8001 sensor, version is %d\n", id);

	return 0;
}

static int mis8001_configure_regulators(struct mis8001 *mis8001)
{
	unsigned int i;

	for (i = 0; i < MIS8001_NUM_SUPPLIES; i++)
		mis8001->supplies[i].supply = mis8001_supply_names[i];

	return devm_regulator_bulk_get(&mis8001->client->dev,
				       MIS8001_NUM_SUPPLIES,
				       mis8001->supplies);
}

static int mis8001_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct mis8001 *mis8001;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	mis8001 = devm_kzalloc(dev, sizeof(*mis8001), GFP_KERNEL);
	if (!mis8001)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &mis8001->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &mis8001->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &mis8001->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &mis8001->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	mis8001->client = client;
	mis8001->cur_mode = &supported_modes[0];

	mis8001->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(mis8001->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	mis8001->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(mis8001->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	mis8001->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(mis8001->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	mis8001->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(mis8001->pinctrl)) {
		mis8001->pins_default =
			pinctrl_lookup_state(mis8001->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(mis8001->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		mis8001->pins_sleep =
			pinctrl_lookup_state(mis8001->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(mis8001->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = mis8001_configure_regulators(mis8001);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&mis8001->mutex);

	sd = &mis8001->subdev;
	v4l2_i2c_subdev_init(sd, client, &mis8001_subdev_ops);
	ret = mis8001_initialize_controls(mis8001);
	if (ret)
		goto err_destroy_mutex;

	ret = __mis8001_power_on(mis8001);
	if (ret)
		goto err_free_handler;

	ret = mis8001_check_sensor_id(mis8001, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &mis8001_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	mis8001->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &mis8001->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(mis8001->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 mis8001->module_index, facing,
		 MIS8001_NAME, dev_name(sd->dev));
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
	__mis8001_power_off(mis8001);
err_free_handler:
	v4l2_ctrl_handler_free(&mis8001->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&mis8001->mutex);

	return ret;
}

static int mis8001_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis8001 *mis8001 = to_mis8001(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&mis8001->ctrl_handler);
	mutex_destroy(&mis8001->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__mis8001_power_off(mis8001);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mis8001_of_match[] = {
	{ .compatible = "imagedesign,mis8001" },
	{},
};
MODULE_DEVICE_TABLE(of, mis8001_of_match);
#endif

static const struct i2c_device_id mis8001_match_id[] = {
	{ "imagedesign,mis8001", 0 },
	{ },
};

static struct i2c_driver mis8001_i2c_driver = {
	.driver = {
		.name = MIS8001_NAME,
		.pm = &mis8001_pm_ops,
		.of_match_table = of_match_ptr(mis8001_of_match),
	},
	.probe = &mis8001_probe,
	.remove = &mis8001_remove,
	.id_table = mis8001_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&mis8001_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&mis8001_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("chengdu image design mis8001 sensor driver");
MODULE_LICENSE("GPL");
