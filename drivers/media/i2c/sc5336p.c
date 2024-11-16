// SPDX-License-Identifier: GPL-2.0
/*
 * sc5336p driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 * V0.0X01.0X02 add support thunder boot
 * V0.0X01.0X03 add support sleep wake-up mode
 * V0.0X01.0X04 add support hw standby for aov
 * V0.0X01.0X05 modify hw standby resume new way
 * V0.0X01.0X06 add support for light control
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x06)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SC5336P_BITS_PER_SAMPLE		10
#define SC5336P_LINK_FREQ_432		432000000

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

/* 2 lane */
#define PIXEL_RATE_WITH_432M_10BIT_2L	(SC5336P_LINK_FREQ_432 * 2 * \
					2 / SC5336P_BITS_PER_SAMPLE)
/* 4 lane */
#define PIXEL_RATE_WITH_432M_10BIT_4L	(SC5336P_LINK_FREQ_432 * 2 / \
					SC5336P_BITS_PER_SAMPLE * 4)

#define SC5336P_XVCLK_FREQ		27000000

#define CHIP_ID				0xce50
#define SC5336P_REG_CHIP_ID		0x3107

#define SC5336P_REG_MIPI_CTRL		0x3019
#define SC5336P_MIPI_CTRL_ON		0x00
#define SC5336P_MIPI_CTRL_OFF		0xff

#define SC5336P_REG_CTRL_MODE		0x0100
#define SC5336P_MODE_SW_STANDBY		0x0
#define SC5336P_MODE_STREAMING		BIT(0)

#define SC5336P_REG_EXPOSURE_H		0x3e00
#define SC5336P_REG_EXPOSURE_M		0x3e01
#define SC5336P_REG_EXPOSURE_L		0x3e02
#define SC5336P_REG_SEXPOSURE_H		0x3e22
#define SC5336P_REG_SEXPOSURE_M		0x3e04
#define SC5336P_REG_SEXPOSURE_L		0x3e05

#define	SC5336P_EXPOSURE_MIN		1
#define	SC5336P_EXPOSURE_STEP		1
#define SC5336P_VTS_MAX			0x7fff
/* linear / hdr long exposure reg */
#define SC5336P_REG_DIG_GAIN		0x3e06
#define SC5336P_REG_DIG_FINE_GAIN	0x3e07
#define SC5336P_REG_ANA_GAIN		0x3e09
/* hdr short exposure reg */
#define SC5336P_REG_SDIG_GAIN		0x3e10
#define SC5336P_REG_SDIG_FINE_GAIN	0x3e11
#define SC5336P_REG_SANA_GAIN		0x3e13

#define SC5336P_GAIN_MIN		0x0020
#define SC5336P_GAIN_MAX		15360 //32 * 15 * 32
#define SC5336P_GAIN_STEP		1
#define SC5336P_GAIN_DEFAULT		0x40
#define SC5336P_LGAIN			0
#define SC5336P_SGAIN			1

#define SC5336P_REG_GROUP_HOLD		0x3812
#define SC5336P_GROUP_HOLD_START	0x00
#define SC5336P_GROUP_HOLD_END		0x30 // Not used

#define SC5336P_REG_TEST_PATTERN	0x4501
#define SC5336P_TEST_PATTERN_BIT_MASK	BIT(3)

#define SC5336P_REG_VTS_H		0x320e
#define SC5336P_REG_VTS_L		0x320f

#define SC5336P_FLIP_MIRROR_REG		0x3221

#define SC5336P_FETCH_EXP_H(VAL)	(((VAL) >> 12) & 0xF)
#define SC5336P_FETCH_EXP_M(VAL)	(((VAL) >> 4) & 0xFF)
#define SC5336P_FETCH_EXP_L(VAL)	(((VAL) & 0xF) << 4)

#define SC5336_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define SC5336_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define SC5336P_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x06 : VAL & 0xf9)
#define SC5336P_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x60 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define SC5336P_REG_VALUE_08BIT		1
#define SC5336P_REG_VALUE_16BIT		2
#define SC5336P_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define SC5336P_NAME			"sc5336p"

static const char *const sc5336p_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC5336P_NUM_SUPPLIES ARRAY_SIZE(sc5336p_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc5336p_mode {
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

struct sc5336p {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC5336P_NUM_SUPPLIES];

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
	const struct sc5336p_mode *supported_modes;
	const struct sc5336p_mode *cur_mode;
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

#define to_sc5336p(sd) container_of(sd, struct sc5336p, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval sc5336p_global_4lane_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 864Mbps, 4lane
 */
static const struct regval sc5336p_linear_10_2880x1620_30fps_4lane_regs[] = {	// TODO: check

	{REG_NULL, 0x00},
};

static const struct regval sc5336p_global_regs_2lane[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * linear, 10bit
 * mipi_datarate per lane 864Mbps, 2lane
 * Cleaned_0x06_FT_SC5336P_MIPI_27Minput_2Lane_10bit_864Mbps_2880x1620_30fps_ECO_外供1.5V.ini
 */
static const struct regval sc5336p_linear_10_2880x1620_30fps_2lane_regs[] = {
	{0x0103, 0x01},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x301f, 0x06},
	{0x320e, 0x07},
	{0x320f, 0x08},
	{0x3213, 0x04},
	{0x3241, 0x00},
	{0x3243, 0x01},
	{0x3248, 0x02},
	{0x3249, 0x07},
	{0x3253, 0x10},
	{0x3258, 0x08},
	{0x3301, 0x06},
	{0x3305, 0x00},
	{0x3306, 0x58},
	{0x3308, 0x08},
	{0x3309, 0xf0},
	{0x330a, 0x00},
	{0x330b, 0xc8},
	{0x3314, 0x14},
	{0x331f, 0xe1},
	{0x3321, 0x10},
	{0x3327, 0x14},
	{0x3328, 0x0b},
	{0x3329, 0x0e},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x3356, 0x10},
	{0x3364, 0x5e},
	{0x338f, 0x80},
	{0x3390, 0x09},
	{0x3391, 0x0b},
	{0x3392, 0x0f},
	{0x3393, 0x10},
	{0x3394, 0x16},
	{0x3395, 0x98},
	{0x3396, 0x08},
	{0x3397, 0x09},
	{0x3398, 0x0f},
	{0x3399, 0x0a},
	{0x339a, 0x18},
	{0x339b, 0x60},
	{0x339c, 0xff},
	{0x33ad, 0x0c},
	{0x33ae, 0xa0},
	{0x33af, 0xd0},
	{0x33b1, 0xa0},
	{0x33b2, 0x38},
	{0x33b3, 0x18},
	{0x33f8, 0x00},
	{0x33f9, 0x68},
	{0x33fa, 0x00},
	{0x33fb, 0x78},
	{0x33fc, 0x0b},
	{0x33fd, 0x1f},
	{0x349f, 0x03},
	{0x34a6, 0x0b},
	{0x34a7, 0x1f},
	{0x34a8, 0x08},
	{0x34a9, 0x08},
	{0x34aa, 0x00},
	{0x34ab, 0xd8},
	{0x34ac, 0x00},
	{0x34ad, 0xe8},
	{0x34f8, 0x3f},
	{0x34f9, 0x08},
	{0x3630, 0xc0},
	{0x3631, 0x83},
	{0x3632, 0x54},
	{0x3633, 0x33},
	{0x3638, 0xcf},
	{0x363f, 0xc0},
	{0x3641, 0x08},
	{0x3670, 0x56},
	{0x3674, 0xd0},
	{0x3675, 0xa0},
	{0x3676, 0xa0},
	{0x3677, 0x83},
	{0x3678, 0x86},
	{0x3679, 0x8a},
	{0x367c, 0x08},
	{0x367d, 0x0f},
	{0x367e, 0x08},
	{0x367f, 0x0f},
	{0x3696, 0x22},
	{0x3697, 0x33},
	{0x3698, 0x24},
	{0x36a0, 0x09},
	{0x36a1, 0x0f},
	{0x36b0, 0x85},
	{0x36b1, 0x8a},
	{0x36b2, 0x95},
	{0x36b3, 0xa6},
	{0x36b4, 0x09},
	{0x36b5, 0x0b},
	{0x36b6, 0x0f},
	{0x36ea, 0x0c},
	{0x370f, 0x01},
	{0x3721, 0x6c},
	{0x3722, 0x89},
	{0x3724, 0x21},
	{0x3725, 0xb4},
	{0x3727, 0x14},
	{0x3771, 0x89},
	{0x3772, 0x89},
	{0x3773, 0xc9},
	{0x377a, 0x0b},
	{0x377b, 0x1f},
	{0x37fa, 0x0c},
	{0x3900, 0x0d},
	{0x3901, 0x00},
	{0x3904, 0x04},
	{0x3905, 0x8c},
	{0x391d, 0x04},
	{0x391e, 0x01},
	{0x391f, 0x49},
	{0x3926, 0x21},
	{0x3933, 0x80},
	{0x3934, 0x05},
	{0x3935, 0x00},
	{0x3936, 0x73},
	{0x3937, 0x79},
	{0x3938, 0x78},
	{0x3939, 0x00},
	{0x393a, 0x00},
	{0x393b, 0x00},
	{0x393c, 0x10},
	{0x39dc, 0x02},
	{0x3e00, 0x00},
	{0x3e01, 0x70},
	{0x3e02, 0x00},
	{0x3e09, 0x00},
	{0x440d, 0x10},
	{0x440e, 0x02},
	{0x450d, 0x18},
	{0x4819, 0x0b},
	{0x481b, 0x06},
	{0x481d, 0x17},
	{0x481f, 0x05},
	{0x4821, 0x0b},
	{0x4823, 0x06},
	{0x4825, 0x05},
	{0x4827, 0x05},
	{0x4829, 0x09},
	{0x5780, 0x76},
	{0x5784, 0x08},
	{0x5785, 0x04},
	{0x5787, 0x0a},
	{0x5788, 0x0a},
	{0x5789, 0x08},
	{0x578a, 0x0a},
	{0x578b, 0x0a},
	{0x578c, 0x08},
	{0x578d, 0x40},
	{0x5790, 0x08},
	{0x5791, 0x04},
	{0x5792, 0x04},
	{0x5793, 0x08},
	{0x5794, 0x04},
	{0x5795, 0x04},
	{0x5799, 0x46},
	{0x579a, 0x77},
	{0x57a1, 0x04},
	{0x57a8, 0xd2},
	{0x57aa, 0x2a},
	{0x57ab, 0x7f},
	{0x57ac, 0x00},
	{0x57ad, 0x00},
	{0x5ae0, 0xfe},
	{0x5ae1, 0x40},
	{0x5ae2, 0x38},
	{0x5ae3, 0x30},
	{0x5ae4, 0x0c},
	{0x5ae5, 0x38},
	{0x5ae6, 0x30},
	{0x5ae7, 0x28},
	{0x5ae8, 0x3f},
	{0x5ae9, 0x34},
	{0x5aea, 0x2c},
	{0x5aeb, 0x3f},
	{0x5aec, 0x34},
	{0x5aed, 0x2c},
	{0x36e9, 0x44},
	{0x37f9, 0x44},
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

static const struct sc5336p_mode supported_modes_4lane[] = {	// TODO: check
	{
		// TODO: check
		.width = 2880,
		.height = 1620,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0080,//mark
		.hts_def = 0x2ee * 2,
		.vts_def = 0x0640,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.global_reg_list = sc5336p_global_4lane_regs,
		.reg_list = sc5336p_linear_10_2880x1620_30fps_4lane_regs,
		.hdr_mode = NO_HDR,
		.mclk = 27000000,
		.link_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.lanes = 4,
	},
};

static const struct sc5336p_mode supported_modes_2lane[] = {
	{
		.width = 2880,
		.height = 1620,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0080 * 4,
		.hts_def = 0x0654 * 2,
		.vts_def = 0x0708,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.global_reg_list = sc5336p_global_regs_2lane,
		.reg_list = sc5336p_linear_10_2880x1620_30fps_2lane_regs,
		.hdr_mode = NO_HDR,
		.mclk = 27000000,
		.link_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.lanes = 2,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const s64 link_freq_menu_items[] = {
	SC5336P_LINK_FREQ_432,
};

static const char *const sc5336p_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4",
};

/* Write registers up to 4 at a time */
static int sc5336p_write_reg(struct i2c_client *client, u16 reg,
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

static int sc5336p_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = sc5336p_write_reg(client, regs[i].addr,
					SC5336P_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int sc5336p_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

// TODO: check
static int sc5336p_set_gain_reg(struct sc5336p *sc5336p, u32 gain, int mode)
{
	struct i2c_client *client = sc5336p->client;
	u32 coarse_again = 0, coarse_dgain = 0, fine_dgain = 0;
	int ret = 0, gain_factor;

	if (gain < 32)
		gain = 32;
	else if (gain > SC5336P_GAIN_MAX)
		gain = SC5336P_GAIN_MAX;

	gain_factor = gain * 1000 / 32;
	if (gain_factor < 2000) {		/*Start again 1.0x - 2.0x gain */
		coarse_again = 0x00;
		coarse_dgain = 0x00;
		fine_dgain = gain_factor * 128 / 1000;
	} else if (gain_factor < 4000) {	// 2.x - 4.x
		coarse_again = 0x08;
		coarse_dgain = 0x00;
		fine_dgain = gain_factor * 128 / 2000;
	} else if (gain_factor < 8000) {	// 4.x - 8.x
		coarse_again = 0x09;
		coarse_dgain = 0x00;
		fine_dgain = gain_factor * 128 / 4000;
	} else if (gain_factor < 16000) {	// 8.x - 16.x
		coarse_again = 0x0b;
		coarse_dgain = 0x00;
		fine_dgain = gain_factor * 128 / 8000;
	} else if (gain_factor < 32000) {	// 16.x - 32.x
		coarse_again = 0x0f;
		coarse_dgain = 0x00;
		fine_dgain = gain_factor * 128 / 16000;
	} else if (gain_factor <= 32000 * 2) {	// 16.x - 32.x
		coarse_again = 0x1f;
		coarse_dgain = 0x00;
		fine_dgain = gain_factor * 128 / 32000;
	} else if (gain_factor < 32000 * 4) {
		//open dgain begin  max digital gain 4X
		coarse_again = 0x1f;
		coarse_dgain = 0x01;
		fine_dgain = gain_factor * 128 / 32000 / 2;
	} else if (gain_factor < 32000 * 8) {
		coarse_again = 0x1f;
		coarse_dgain = 0x03;
		fine_dgain = gain_factor * 128 / 32000 / 4;
	} else if (gain_factor < 32000 * 15) {
		coarse_again = 0x1f;
		coarse_dgain = 0x07;
		fine_dgain = gain_factor * 128 / 32000 / 8;
	} else {
		coarse_again = 0x1f;
		coarse_dgain = 0x07;
		fine_dgain = 0xf0;
	}
	dev_dbg(&client->dev, "c_again: 0x%x, c_dgain: 0x%x, f_dgain: 0x%0x\n",
		coarse_again, coarse_dgain, fine_dgain);

	if (mode == SC5336P_LGAIN) {
		ret = sc5336p_write_reg(sc5336p->client,
					SC5336P_REG_DIG_GAIN,
					SC5336P_REG_VALUE_08BIT,
					coarse_dgain);
		ret |= sc5336p_write_reg(sc5336p->client,
					 SC5336P_REG_DIG_FINE_GAIN,
					 SC5336P_REG_VALUE_08BIT,
					 fine_dgain);
		ret |= sc5336p_write_reg(sc5336p->client,
					 SC5336P_REG_ANA_GAIN,
					 SC5336P_REG_VALUE_08BIT,
					 coarse_again);
	} else {
		ret = sc5336p_write_reg(sc5336p->client,
					SC5336P_REG_SDIG_GAIN,
					SC5336P_REG_VALUE_08BIT,
					coarse_dgain);
		ret |= sc5336p_write_reg(sc5336p->client,
					 SC5336P_REG_SDIG_FINE_GAIN,
					 SC5336P_REG_VALUE_08BIT,
					 fine_dgain);
		ret |= sc5336p_write_reg(sc5336p->client,
					 SC5336P_REG_SANA_GAIN,
					 SC5336P_REG_VALUE_08BIT,
					 coarse_again);
	}
	return ret;
}

static int sc5336p_set_hdrae(struct sc5336p *sc5336p,
			     struct preisp_hdrae_exp_s *ae)
{
	int ret = 0;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;
	u32 l_exp_max = 0;

	if (!sc5336p->has_init_exp && !sc5336p->streaming) {
		sc5336p->init_hdrae_exp = *ae;
		sc5336p->has_init_exp = true;
		dev_dbg(&sc5336p->client->dev, "sc5336p don't stream, record exp for hdr!\n");
		return ret;
	}
	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;

	dev_dbg(&sc5336p->client->dev,
		"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_a_gain, m_a_gain, s_a_gain);

	if (sc5336p->cur_mode->hdr_mode == HDR_X2) {
		//2 stagger
		l_a_gain = m_a_gain;
		l_exp_time = m_exp_time;
	}

	l_exp_max = sc5336p->cur_vts - 318 - 16; // vts-(3e23~3e24)-16
	//set exposure
	l_exp_time = l_exp_time * 2;
	s_exp_time = s_exp_time * 2;
	if (l_exp_time > l_exp_max)
		l_exp_time = l_exp_max;
	if (s_exp_time > 302)  // (3e23~3e24)-16
		s_exp_time = 302;

	ret = sc5336p_write_reg(sc5336p->client,
				SC5336P_REG_EXPOSURE_H,
				SC5336P_REG_VALUE_08BIT,
				SC5336P_FETCH_EXP_H(l_exp_time));
	ret |= sc5336p_write_reg(sc5336p->client,
				 SC5336P_REG_EXPOSURE_M,
				 SC5336P_REG_VALUE_08BIT,
				 SC5336P_FETCH_EXP_M(l_exp_time));
	ret |= sc5336p_write_reg(sc5336p->client,
				 SC5336P_REG_EXPOSURE_L,
				 SC5336P_REG_VALUE_08BIT,
				 SC5336P_FETCH_EXP_L(l_exp_time));
	ret |= sc5336p_write_reg(sc5336p->client,
				 SC5336P_REG_SEXPOSURE_M,
				 SC5336P_REG_VALUE_08BIT,
				 SC5336P_FETCH_EXP_M(s_exp_time));
	ret |= sc5336p_write_reg(sc5336p->client,
				 SC5336P_REG_SEXPOSURE_L,
				 SC5336P_REG_VALUE_08BIT,
				 SC5336P_FETCH_EXP_L(s_exp_time));

	ret |= sc5336p_set_gain_reg(sc5336p, l_a_gain, SC5336P_LGAIN);
	ret |= sc5336p_set_gain_reg(sc5336p, s_a_gain, SC5336P_SGAIN);
	return ret;
}

static int sc5336p_get_reso_dist(const struct sc5336p_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc5336p_mode *
sc5336p_find_best_fit(struct sc5336p *sc5336p, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < sc5336p->cfg_num; i++) {
		dist = sc5336p_get_reso_dist(&sc5336p->supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		} else if (dist == cur_best_fit_dist &&
			   framefmt->code == sc5336p->supported_modes[i].bus_fmt) {
			cur_best_fit = i;
			break;
		}
	}

	return &sc5336p->supported_modes[cur_best_fit];
}

static int sc5336p_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	const struct sc5336p_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = sc5336p->bus_cfg.bus.mipi_csi2.num_data_lanes;

	mutex_lock(&sc5336p->mutex);

	mode = sc5336p_find_best_fit(sc5336p, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc5336p->mutex);
		return -ENOTTY;
#endif
	} else {
		sc5336p->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc5336p->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc5336p->vblank, vblank_def,
					 SC5336P_VTS_MAX - mode->height,
					 1, vblank_def);
		dst_link_freq = mode->link_freq_idx;
		dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
				 mode->bpp * 2 * lanes;
		__v4l2_ctrl_s_ctrl_int64(sc5336p->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(sc5336p->link_freq,
				   dst_link_freq);
		sc5336p->cur_fps = mode->max_fps;
	}

	mutex_unlock(&sc5336p->mutex);

	return 0;
}

static int sc5336p_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	const struct sc5336p_mode *mode = sc5336p->cur_mode;

	mutex_lock(&sc5336p->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc5336p->mutex);
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
	mutex_unlock(&sc5336p->mutex);

	return 0;
}

static int sc5336p_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int sc5336p_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);

	if (fse->index >= sc5336p->cfg_num)
		return -EINVAL;

	if (fse->code != sc5336p->supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = sc5336p->supported_modes[fse->index].width;
	fse->max_width  = sc5336p->supported_modes[fse->index].width;
	fse->max_height = sc5336p->supported_modes[fse->index].height;
	fse->min_height = sc5336p->supported_modes[fse->index].height;

	return 0;
}

static int sc5336p_enable_test_pattern(struct sc5336p *sc5336p, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc5336p_read_reg(sc5336p->client, SC5336P_REG_TEST_PATTERN,
			       SC5336P_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC5336P_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SC5336P_TEST_PATTERN_BIT_MASK;

	ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_TEST_PATTERN,
				 SC5336P_REG_VALUE_08BIT, val);
	return ret;
}

static int sc5336p_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	const struct sc5336p_mode *mode = sc5336p->cur_mode;

	if (sc5336p->streaming)
		fi->interval = sc5336p->cur_fps;
	else
		fi->interval = mode->max_fps;
	return 0;
}

static const struct sc5336p_mode *sc5336p_find_mode(struct sc5336p *sc5336p, int fps)
{
	const struct sc5336p_mode *mode = NULL;
	const struct sc5336p_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < sc5336p->cfg_num; i++) {
		mode = &sc5336p->supported_modes[i];
		if (mode->width == sc5336p->cur_mode->width &&
		    mode->height == sc5336p->cur_mode->height &&
		    mode->hdr_mode == sc5336p->cur_mode->hdr_mode &&
		    mode->bus_fmt == sc5336p->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int sc5336p_s_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	const struct sc5336p_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	int fps;

	if (sc5336p->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = sc5336p_find_mode(sc5336p, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	sc5336p->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(sc5336p->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(sc5336p->vblank, vblank_def,
				 SC5336P_VTS_MAX - mode->height,
				 1, vblank_def);
	pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
		     mode->bpp * 2 * mode->lanes;

	__v4l2_ctrl_s_ctrl_int64(sc5336p->pixel_rate,
				 pixel_rate);
	__v4l2_ctrl_s_ctrl(sc5336p->link_freq,
			   mode->link_freq_idx);
	sc5336p->cur_fps = mode->max_fps;

	return 0;
}

static int sc5336p_g_mbus_config(struct v4l2_subdev *sd,
				 unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	const struct sc5336p_mode *mode = sc5336p->cur_mode;
	u8 lanes = sc5336p->bus_cfg.bus.mipi_csi2.num_data_lanes;

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

static void sc5336p_get_module_inf(struct sc5336p *sc5336p,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC5336P_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc5336p->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc5336p->len_name, sizeof(inf->base.lens));
}

static int sc5336p_set_setting(struct sc5336p *sc5336p, struct rk_sensor_setting *setting)
{
	int i = 0;
	int cur_fps = 0;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	const struct sc5336p_mode *mode = NULL;
	const struct sc5336p_mode *match = NULL;
	u8 lane = sc5336p->bus_cfg.bus.mipi_csi2.num_data_lanes;

	dev_info(&sc5336p->client->dev,
		 "sensor setting: %d x %d, fps:%d fmt:%d, mode:%d\n",
		 setting->width, setting->height,
		 setting->fps, setting->fmt, setting->mode);

	for (i = 0; i < sc5336p->cfg_num; i++) {
		mode = &sc5336p->supported_modes[i];
		if (mode->width == setting->width &&
		    mode->height == setting->height &&
		    mode->hdr_mode == setting->mode &&
		    mode->bus_fmt == setting->fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == setting->fps) {
				match = mode;
				break;
			}
		}
	}

	if (match) {
		dev_info(&sc5336p->client->dev, "-----%s: match the support mode, mode idx:%d-----\n",
			 __func__, i);
		sc5336p->cur_mode = mode;

		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc5336p->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc5336p->vblank, vblank_def,
					 SC5336P_VTS_MAX - mode->height,
					 1, vblank_def);


		__v4l2_ctrl_s_ctrl(sc5336p->link_freq, mode->link_freq_idx);
		pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
			     mode->bpp * 2 * lane;
		__v4l2_ctrl_s_ctrl_int64(sc5336p->pixel_rate, pixel_rate);
		dev_info(&sc5336p->client->dev, "freq_idx:%d pixel_rate:%lld\n",
			 mode->link_freq_idx, pixel_rate);

		sc5336p->cur_vts = mode->vts_def;
		sc5336p->cur_fps = mode->max_fps;

		dev_info(&sc5336p->client->dev, "hts_def:%d cur_vts:%d cur_fps:%d\n",
			 mode->hts_def, mode->vts_def,
			 sc5336p->cur_fps.denominator / sc5336p->cur_fps.numerator);
	} else {
		dev_err(&sc5336p->client->dev, "couldn't match the support modes\n");
		return -EINVAL;
	}

	return 0;
}

static long sc5336p_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rk_sensor_setting *setting;
	struct rk_light_param *light_param;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = sc5336p->bus_cfg.bus.mipi_csi2.num_data_lanes;
	const struct sc5336p_mode *mode;
	int cur_best_fit = -1;
	int cur_best_fit_dist = -1;
	int cur_dist, cur_fps, dst_fps;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc5336p_get_module_inf(sc5336p, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc5336p->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		if (hdr->hdr_mode == sc5336p->cur_mode->hdr_mode)
			return 0;
		w = sc5336p->cur_mode->width;
		h = sc5336p->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(sc5336p->cur_mode->max_fps.denominator,
					    sc5336p->cur_mode->max_fps.numerator);
		for (i = 0; i < sc5336p->cfg_num; i++) {
			if (w == sc5336p->supported_modes[i].width &&
			    h == sc5336p->supported_modes[i].height &&
			    sc5336p->supported_modes[i].hdr_mode == hdr->hdr_mode &&
			    sc5336p->supported_modes[i].bus_fmt == sc5336p->cur_mode->bus_fmt) {
				cur_fps = DIV_ROUND_CLOSEST(sc5336p->supported_modes[i].max_fps.denominator,
							    sc5336p->supported_modes[i].max_fps.numerator);
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
			dev_err(&sc5336p->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			sc5336p->cur_mode = &sc5336p->supported_modes[cur_best_fit];
			mode = sc5336p->cur_mode;
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(sc5336p->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc5336p->vblank, h,
						 SC5336P_VTS_MAX - sc5336p->cur_mode->height, 1, h);
			sc5336p->cur_fps = sc5336p->cur_mode->max_fps;

			dst_link_freq = mode->link_freq_idx;
			dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
					 mode->bpp * 2 * lanes;
			__v4l2_ctrl_s_ctrl_int64(sc5336p->pixel_rate,
						 dst_pixel_rate);
			__v4l2_ctrl_s_ctrl(sc5336p->link_freq,
					   dst_link_freq);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		sc5336p_set_hdrae(sc5336p, arg);
		if (sc5336p->cam_sw_inf)
			memcpy(&sc5336p->cam_sw_inf->hdr_ae, (struct preisp_hdrae_exp_s *)(arg),
			       sizeof(struct preisp_hdrae_exp_s));
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (sc5336p->enable_light_ctl) {
			sc5336p->light_param.light_enable = stream;
			light_ctl_write(sc5336p->module_index, &sc5336p->light_param);
		}

		if (sc5336p->standby_hw) {	/* hardware standby */
			if (stream) {
				/* pwdn gpio pull up */
				if (!IS_ERR(sc5336p->pwdn_gpio))
					gpiod_set_value_cansleep(sc5336p->pwdn_gpio, 1);
				sc5336p->is_standby = false;
				/* mipi clk on */
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_MIPI_CTRL,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MIPI_CTRL_ON);
#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
				if (__v4l2_ctrl_handler_setup(&sc5336p->ctrl_handler))
					dev_err(&sc5336p->client->dev, "__v4l2_ctrl_handler_setup fail!");
				if (sc5336p->cur_mode->hdr_mode != NO_HDR) {	// hdr mode
					if (sc5336p->cam_sw_inf) {
						ret = sc5336p_ioctl(&sc5336p->subdev,
								    PREISP_CMD_SET_HDRAE_EXP,
								    &sc5336p->cam_sw_inf->hdr_ae);
						if (ret) {
							dev_err(&sc5336p->client->dev,
								"init exp fail in hdr mode\n");
							return ret;
						}
					}
				}
#endif

				/* stream on */
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_CTRL_MODE,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MODE_STREAMING);
				dev_info(&sc5336p->client->dev,
					 "quickstream, streaming on: exit hw standby mode\n");
			} else {
				/* stream off */
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_CTRL_MODE,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MODE_SW_STANDBY);
				/* mipi clk off */
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_MIPI_CTRL,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MIPI_CTRL_OFF);
				/* pwnd gpio pull down */
				if (!IS_ERR(sc5336p->pwdn_gpio))
					gpiod_set_value_cansleep(sc5336p->pwdn_gpio, 0);
				dev_info(&sc5336p->client->dev,
					 "quickstream, streaming off: enter hw standby mode\n");
				sc5336p->is_standby = true;
			}
		} else {	/* software standby */
			if (stream) {
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_MIPI_CTRL,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MIPI_CTRL_ON);
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_CTRL_MODE,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MODE_STREAMING);
				dev_info(&sc5336p->client->dev,
					 "quickstream, streaming on: exit soft standby mode\n");
			} else {
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_CTRL_MODE,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MODE_SW_STANDBY);
				ret |= sc5336p_write_reg(sc5336p->client, SC5336P_REG_MIPI_CTRL,
							 SC5336P_REG_VALUE_08BIT,
							 SC5336P_MIPI_CTRL_OFF);
				dev_info(&sc5336p->client->dev,
					 "quickstream, streaming off: enter soft standby mode\n");
			}
		}
		break;
	case RKCIS_CMD_SELECT_SETTING:
		setting = (struct rk_sensor_setting *)arg;
		ret = sc5336p_set_setting(sc5336p, setting);
		break;
	case RKCIS_CMD_FLASH_LIGHT_CTRL:
		dev_info(&sc5336p->client->dev, "set flash light param\n");
		light_param = (struct rk_light_param *)arg;
		if (light_param->light_enable) {
			memcpy(&sc5336p->light_param, light_param, sizeof(struct rk_light_param));
			sc5336p->enable_light_ctl = true;
		} else {
			sc5336p->enable_light_ctl = false;
		}
		ret = light_ctl_write(sc5336p->module_index, light_param);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc5336p_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = sc5336p_ioctl(sd, cmd, inf);
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

		ret = sc5336p_ioctl(sd, cmd, hdr);
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
			ret = sc5336p_ioctl(sd, cmd, hdr);
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
			ret = sc5336p_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = sc5336p_ioctl(sd, cmd, &stream);
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
			ret = sc5336p_ioctl(sd, cmd, setting);
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
			ret = sc5336p_ioctl(sd, cmd, light_param);
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

static int __sc5336p_start_stream(struct sc5336p *sc5336p)
{
	int ret;

	if (!sc5336p->is_thunderboot) {
		ret = sc5336p_write_array(sc5336p->client, sc5336p->cur_mode->reg_list);
		if (ret)
			return ret;
		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&sc5336p->ctrl_handler);
		if (ret)
			return ret;
		if (sc5336p->has_init_exp && sc5336p->cur_mode->hdr_mode != NO_HDR) {
			ret = sc5336p_ioctl(&sc5336p->subdev, PREISP_CMD_SET_HDRAE_EXP,
					    &sc5336p->init_hdrae_exp);
			if (ret) {
				dev_err(&sc5336p->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}
	ret = sc5336p_write_reg(sc5336p->client, SC5336P_REG_CTRL_MODE,
				SC5336P_REG_VALUE_08BIT, SC5336P_MODE_STREAMING);
	return ret;
}

static int __sc5336p_stop_stream(struct sc5336p *sc5336p)
{
	sc5336p->has_init_exp = false;
	if (sc5336p->is_thunderboot)
		sc5336p->is_first_streamoff = true;
	sc5336p->enable_light_ctl = false;
	return sc5336p_write_reg(sc5336p->client, SC5336P_REG_CTRL_MODE,
				 SC5336P_REG_VALUE_08BIT, SC5336P_MODE_SW_STANDBY);
}

static int __sc5336p_power_on(struct sc5336p *sc5336p);
static int sc5336p_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	struct i2c_client *client = sc5336p->client;
	int ret = 0;

	mutex_lock(&sc5336p->mutex);
	on = !!on;
	if (on == sc5336p->streaming)
		goto unlock_and_return;
	if (on) {
		if (sc5336p->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			sc5336p->is_thunderboot = false;
			__sc5336p_power_on(sc5336p);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __sc5336p_start_stream(sc5336p);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc5336p_stop_stream(sc5336p);
		pm_runtime_put(&client->dev);
	}

	sc5336p->streaming = on;
unlock_and_return:
	mutex_unlock(&sc5336p->mutex);
	return ret;
}

static int sc5336p_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	struct i2c_client *client = sc5336p->client;
	int ret = 0;

	mutex_lock(&sc5336p->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc5336p->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!sc5336p->is_thunderboot) {
			ret = sc5336p_write_array(sc5336p->client,
						  sc5336p->cur_mode->global_reg_list);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		sc5336p->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc5336p->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc5336p->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sc5336p_cal_delay(u32 cycles, struct sc5336p *sc5336p)
{
	return DIV_ROUND_UP(cycles, sc5336p->cur_mode->mclk / 1000 / 1000);
}

static int __sc5336p_power_on(struct sc5336p *sc5336p)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sc5336p->client->dev;

	if (!IS_ERR_OR_NULL(sc5336p->pins_default)) {
		ret = pinctrl_select_state(sc5336p->pinctrl,
					   sc5336p->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc5336p->xvclk, sc5336p->cur_mode->mclk);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (%dHz)\n", sc5336p->cur_mode->mclk);
	if (clk_get_rate(sc5336p->xvclk) != sc5336p->cur_mode->mclk)
		dev_warn(dev, "xvclk mismatched, modes are based on %dHz\n",
			 sc5336p->cur_mode->mclk);
	ret = clk_prepare_enable(sc5336p->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	cam_sw_regulator_bulk_init(sc5336p->cam_sw_inf, SC5336P_NUM_SUPPLIES, sc5336p->supplies);

	if (sc5336p->is_thunderboot)
		return 0;

	if (!IS_ERR(sc5336p->reset_gpio))
		gpiod_set_value_cansleep(sc5336p->reset_gpio, 0);

	ret = regulator_bulk_enable(SC5336P_NUM_SUPPLIES, sc5336p->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc5336p->reset_gpio))
		gpiod_set_value_cansleep(sc5336p->reset_gpio, 1);

	usleep_range(500, 1000);

	if (!IS_ERR(sc5336p->pwdn_gpio))
		gpiod_set_value_cansleep(sc5336p->pwdn_gpio, 1);

	if (!IS_ERR(sc5336p->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sc5336p_cal_delay(8192, sc5336p);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(sc5336p->xvclk);

	return ret;
}

static void __sc5336p_power_off(struct sc5336p *sc5336p)
{
	int ret;
	struct device *dev = &sc5336p->client->dev;

	clk_disable_unprepare(sc5336p->xvclk);
	if (sc5336p->is_thunderboot) {
		if (sc5336p->is_first_streamoff) {
			sc5336p->is_thunderboot = false;
			sc5336p->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(sc5336p->pwdn_gpio))
		gpiod_set_value_cansleep(sc5336p->pwdn_gpio, 0);
	clk_disable_unprepare(sc5336p->xvclk);
	if (!IS_ERR(sc5336p->reset_gpio))
		gpiod_set_value_cansleep(sc5336p->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(sc5336p->pins_sleep)) {
		ret = pinctrl_select_state(sc5336p->pinctrl,
					   sc5336p->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC5336P_NUM_SUPPLIES, sc5336p->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int __maybe_unused sc5336p_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5336p *sc5336p = to_sc5336p(sd);

	if (sc5336p->standby_hw) {
		dev_info(dev, "resume standby!");
		return 0;
	}

	cam_sw_prepare_wakeup(sc5336p->cam_sw_inf, dev);

	usleep_range(4000, 5000);
	cam_sw_write_array(sc5336p->cam_sw_inf);

	if (__v4l2_ctrl_handler_setup(&sc5336p->ctrl_handler))
		dev_err(dev, "__v4l2_ctrl_handler_setup fail!");

	if (sc5336p->has_init_exp && sc5336p->cur_mode != NO_HDR) {	// hdr mode
		ret = sc5336p_ioctl(&sc5336p->subdev, PREISP_CMD_SET_HDRAE_EXP,
					&sc5336p->cam_sw_inf->hdr_ae);
		if (ret) {
			dev_err(&sc5336p->client->dev, "set exp fail in hdr mode\n");
			return ret;
		}
	}

	return 0;
}

static int __maybe_unused sc5336p_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5336p *sc5336p = to_sc5336p(sd);

	if (sc5336p->standby_hw) {
		dev_info(dev, "suspend standby!");
		return 0;
	}

	cam_sw_write_array_cb_init(sc5336p->cam_sw_inf, client,
				   (void *)sc5336p->cur_mode->reg_list,
				   (sensor_write_array)sc5336p_write_array);
	cam_sw_prepare_sleep(sc5336p->cam_sw_inf);

	return 0;
}
#else
#define sc5336p_resume NULL
#define sc5336p_suspend NULL
#endif

static int __maybe_unused sc5336p_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5336p *sc5336p = to_sc5336p(sd);

	return __sc5336p_power_on(sc5336p);
}

static int __maybe_unused sc5336p_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5336p *sc5336p = to_sc5336p(sd);

	__sc5336p_power_off(sc5336p);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc5336p_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc5336p_mode *def_mode = &sc5336p->supported_modes[0];

	mutex_lock(&sc5336p->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc5336p->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc5336p_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc5336p *sc5336p = to_sc5336p(sd);

	if (fie->index >= sc5336p->cfg_num)
		return -EINVAL;

	fie->code = sc5336p->supported_modes[fie->index].bus_fmt;
	fie->width = sc5336p->supported_modes[fie->index].width;
	fie->height = sc5336p->supported_modes[fie->index].height;
	fie->interval = sc5336p->supported_modes[fie->index].max_fps;
	fie->reserved[0] = sc5336p->supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops sc5336p_pm_ops = {
	SET_RUNTIME_PM_OPS(sc5336p_runtime_suspend,
			   sc5336p_runtime_resume, NULL)
#ifdef CONFIG_VIDEO_CAM_SLEEP_WAKEUP
	SET_LATE_SYSTEM_SLEEP_PM_OPS(sc5336p_suspend, sc5336p_resume)
#endif
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc5336p_internal_ops = {
	.open = sc5336p_open,
};
#endif

static const struct v4l2_subdev_core_ops sc5336p_core_ops = {
	.s_power = sc5336p_s_power,
	.ioctl = sc5336p_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc5336p_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc5336p_video_ops = {
	.s_stream = sc5336p_s_stream,
	.g_frame_interval = sc5336p_g_frame_interval,
	.s_frame_interval = sc5336p_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc5336p_pad_ops = {
	.enum_mbus_code = sc5336p_enum_mbus_code,
	.enum_frame_size = sc5336p_enum_frame_sizes,
	.enum_frame_interval = sc5336p_enum_frame_interval,
	.get_fmt = sc5336p_get_fmt,
	.set_fmt = sc5336p_set_fmt,
	.get_mbus_config = sc5336p_g_mbus_config,
};

static const struct v4l2_subdev_ops sc5336p_subdev_ops = {
	.core	= &sc5336p_core_ops,
	.video	= &sc5336p_video_ops,
	.pad	= &sc5336p_pad_ops,
};

static void sc5336p_modify_fps_info(struct sc5336p *sc5336p)
{
	const struct sc5336p_mode *mode = sc5336p->cur_mode;

	sc5336p->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       sc5336p->cur_vts;
}

static int sc5336p_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc5336p *sc5336p = container_of(ctrl->handler,
					       struct sc5336p, ctrl_handler);
	struct i2c_client *client = sc5336p->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc5336p->cur_mode->height + ctrl->val - 5;
		__v4l2_ctrl_modify_range(sc5336p->exposure,
					 sc5336p->exposure->minimum, max,
					 sc5336p->exposure->step,
					 sc5336p->exposure->default_value);
		break;
	}

	if (sc5336p->standby_hw && sc5336p->is_standby) {
		dev_dbg(&client->dev, "%s: is_standby = true, will return\n", __func__);
		return 0;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (sc5336p->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val << 1;
			/* 4 least significant bits of expsoure are fractional part */
			ret = sc5336p_write_reg(sc5336p->client,
						SC5336P_REG_EXPOSURE_H,
						SC5336P_REG_VALUE_08BIT,
						SC5336P_FETCH_EXP_H(val));
			ret |= sc5336p_write_reg(sc5336p->client,
						 SC5336P_REG_EXPOSURE_M,
						 SC5336P_REG_VALUE_08BIT,
						 SC5336P_FETCH_EXP_M(val));
			ret |= sc5336p_write_reg(sc5336p->client,
						 SC5336P_REG_EXPOSURE_L,
						 SC5336P_REG_VALUE_08BIT,
						 SC5336P_FETCH_EXP_L(val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (sc5336p->cur_mode->hdr_mode == NO_HDR)
			ret = sc5336p_set_gain_reg(sc5336p, ctrl->val, SC5336P_LGAIN);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = sc5336p_write_reg(sc5336p->client,
					SC5336P_REG_VTS_H,
					SC5336P_REG_VALUE_08BIT,
					(ctrl->val + sc5336p->cur_mode->height)
					>> 8);
		ret |= sc5336p_write_reg(sc5336p->client,
					 SC5336P_REG_VTS_L,
					 SC5336P_REG_VALUE_08BIT,
					 (ctrl->val + sc5336p->cur_mode->height)
					 & 0xff);
		sc5336p->cur_vts = ctrl->val + sc5336p->cur_mode->height;
		if (sc5336p->cur_vts != sc5336p->cur_mode->vts_def)
			sc5336p_modify_fps_info(sc5336p);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc5336p_enable_test_pattern(sc5336p, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc5336p_read_reg(sc5336p->client, SC5336P_FLIP_MIRROR_REG,
				       SC5336P_REG_VALUE_08BIT, &val);
		ret |= sc5336p_write_reg(sc5336p->client, SC5336P_FLIP_MIRROR_REG,
					 SC5336P_REG_VALUE_08BIT,
					 SC5336P_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = sc5336p_read_reg(sc5336p->client, SC5336P_FLIP_MIRROR_REG,
				       SC5336P_REG_VALUE_08BIT, &val);
		ret |= sc5336p_write_reg(sc5336p->client, SC5336P_FLIP_MIRROR_REG,
					 SC5336P_REG_VALUE_08BIT,
					 SC5336P_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc5336p_ctrl_ops = {
	.s_ctrl = sc5336p_set_ctrl,
};

static int sc5336p_initialize_controls(struct sc5336p *sc5336p)
{
	const struct sc5336p_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = sc5336p->bus_cfg.bus.mipi_csi2.num_data_lanes;

	handler = &sc5336p->ctrl_handler;
	mode = sc5336p->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc5336p->mutex;

	sc5336p->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			     V4L2_CID_LINK_FREQ,
			     ARRAY_SIZE(link_freq_menu_items) - 1, 0, link_freq_menu_items);
	if (sc5336p->link_freq)
		sc5336p->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	dst_link_freq = mode->link_freq_idx;
	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
			 mode->bpp * 2 * lanes;
	sc5336p->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
						0, PIXEL_RATE_WITH_432M_10BIT_4L, 1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(sc5336p->link_freq, dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	sc5336p->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (sc5336p->hblank)
		sc5336p->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	sc5336p->vblank = v4l2_ctrl_new_std(handler, &sc5336p_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    SC5336P_VTS_MAX - mode->height,
					    1, vblank_def);
	exposure_max = mode->vts_def - 8;
	sc5336p->exposure = v4l2_ctrl_new_std(handler, &sc5336p_ctrl_ops,
					      V4L2_CID_EXPOSURE, SC5336P_EXPOSURE_MIN,
					      exposure_max, SC5336P_EXPOSURE_STEP,
					      mode->exp_def);
	sc5336p->anal_gain = v4l2_ctrl_new_std(handler, &sc5336p_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, SC5336P_GAIN_MIN,
					       SC5336P_GAIN_MAX, SC5336P_GAIN_STEP,
					       SC5336P_GAIN_DEFAULT);
	sc5336p->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&sc5336p_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(sc5336p_test_pattern_menu) - 1,
				0, 0, sc5336p_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &sc5336p_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &sc5336p_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc5336p->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc5336p->subdev.ctrl_handler = handler;
	sc5336p->has_init_exp = false;
	sc5336p->cur_fps = mode->max_fps;
	sc5336p->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc5336p_check_sensor_id(struct sc5336p *sc5336p,
				   struct i2c_client *client)
{
	struct device *dev = &sc5336p->client->dev;
	u32 id = 0;
	int ret;

	if (sc5336p->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = sc5336p_read_reg(client, SC5336P_REG_CHIP_ID,
			       SC5336P_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(0x%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC5336P (0x%04x) sensor\n", CHIP_ID);
	return 0;
}

static int sc5336p_configure_regulators(struct sc5336p *sc5336p)
{
	unsigned int i;

	for (i = 0; i < SC5336P_NUM_SUPPLIES; i++)
		sc5336p->supplies[i].supply = sc5336p_supply_names[i];

	return devm_regulator_bulk_get(&sc5336p->client->dev,
				       SC5336P_NUM_SUPPLIES,
				       sc5336p->supplies);
}

static int sc5336p_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc5336p *sc5336p;
	struct v4l2_subdev *sd;
	struct device_node *endpoint;
	char facing[2];
	int ret;
	int i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc5336p = devm_kzalloc(dev, sizeof(*sc5336p), GFP_KERNEL);
	if (!sc5336p)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc5336p->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc5336p->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc5336p->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc5336p->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	/* Compatible with non-standby mode if this attribute is not configured in dts*/
	of_property_read_u32(node, RKMODULE_CAMERA_STANDBY_HW,
			     &sc5336p->standby_hw);
	dev_info(dev, "sc5336p->standby_hw is %s\n",
		 (sc5336p->standby_hw > 0) ? "configured" : "not configured");

	sc5336p->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, "Get hdr mode failed! no hdr default\n");
	}
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint), &sc5336p->bus_cfg);
	of_node_put(endpoint);
	if (ret) {
		dev_err(dev, "Failed to get bus config\n");
		return -EINVAL;
	}

	if (sc5336p->bus_cfg.bus.mipi_csi2.num_data_lanes == 4) {
		sc5336p->supported_modes = supported_modes_4lane;
		sc5336p->cfg_num = ARRAY_SIZE(supported_modes_4lane);
	} else {
		sc5336p->supported_modes = supported_modes_2lane;
		sc5336p->cfg_num = ARRAY_SIZE(supported_modes_2lane);
	}
	dev_info(dev, "detect sc5336p lane: %d\n",
		 sc5336p->bus_cfg.bus.mipi_csi2.num_data_lanes);

	sc5336p->client = client;
	for (i = 0; i < sc5336p->cfg_num; i++) {
		if (hdr_mode == sc5336p->supported_modes[i].hdr_mode) {
			sc5336p->cur_mode = &sc5336p->supported_modes[i];
			break;
		}
	}

	if (i == sc5336p->cfg_num)
		sc5336p->cur_mode = &sc5336p->supported_modes[0];

	sc5336p->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc5336p->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc5336p->reset_gpio = devm_gpiod_get(dev, "reset",
					     sc5336p->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sc5336p->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc5336p->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
					    sc5336p->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sc5336p->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc5336p->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc5336p->pinctrl)) {
		sc5336p->pins_default =
			pinctrl_lookup_state(sc5336p->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc5336p->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc5336p->pins_sleep =
			pinctrl_lookup_state(sc5336p->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc5336p->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc5336p_configure_regulators(sc5336p);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc5336p->mutex);

	sd = &sc5336p->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc5336p_subdev_ops);
	ret = sc5336p_initialize_controls(sc5336p);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc5336p_power_on(sc5336p);
	if (ret)
		goto err_free_handler;

	ret = sc5336p_check_sensor_id(sc5336p, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc5336p_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc5336p->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc5336p->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	if (!sc5336p->cam_sw_inf) {
		sc5336p->cam_sw_inf = cam_sw_init();
		cam_sw_clk_init(sc5336p->cam_sw_inf, sc5336p->xvclk,
				sc5336p->cur_mode->mclk);
		cam_sw_reset_pin_init(sc5336p->cam_sw_inf, sc5336p->reset_gpio, 0);
		cam_sw_pwdn_pin_init(sc5336p->cam_sw_inf, sc5336p->pwdn_gpio, 1);
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc5336p->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc5336p->module_index, facing,
		 SC5336P_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (sc5336p->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc5336p_power_off(sc5336p);
err_free_handler:
	v4l2_ctrl_handler_free(&sc5336p->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc5336p->mutex);

	return ret;
}

static int sc5336p_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5336p *sc5336p = to_sc5336p(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc5336p->ctrl_handler);
	mutex_destroy(&sc5336p->mutex);

	cam_sw_deinit(sc5336p->cam_sw_inf);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc5336p_power_off(sc5336p);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc5336p_of_match[] = {
	{ .compatible = "smartsens,sc5336p" },
	{},
};
MODULE_DEVICE_TABLE(of, sc5336p_of_match);
#endif

static const struct i2c_device_id sc5336p_match_id[] = {
	{ "smartsens,sc5336p", 0 },
	{ },
};

static struct i2c_driver sc5336p_i2c_driver = {
	.driver = {
		.name = SC5336P_NAME,
		.pm = &sc5336p_pm_ops,
		.of_match_table = of_match_ptr(sc5336p_of_match),
	},
	.probe		= &sc5336p_probe,
	.remove		= &sc5336p_remove,
	.id_table	= sc5336p_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc5336p_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc5336p_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens sc5336p sensor driver");
MODULE_LICENSE("GPL");
