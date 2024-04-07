// SPDX-License-Identifier: GPL-2.0
/*
 * mis40c1 driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 * V0.0X01.0X02 support fastboot
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define MIS40C1_LANES			2
#define MIS40C1_BITS_PER_SAMPLE		10
#define MIS40C1_LINK_FREQ_317		317250000 //mipi rate per line 317.25M

#define PIXEL_RATE_WITH_317M_10BIT	(MIS40C1_LINK_FREQ_317 * 2 * \
					MIS40C1_LANES / MIS40C1_BITS_PER_SAMPLE)
#define MIS40C1_XVCLK_FREQ		27000000

#define CHIP_ID				0x0004
#define MIS40C1_REG_CHIP_ID		0x3005

#define MIS40C1_REG_CTRL_MODE		0x302d
#define MIS40C1_MODE_SW_STANDBY		BIT(0)
#define MIS40C1_MODE_STREAMING		0x0

#define MIS40C1_REG_CTRL_PWDN		0x3006
#define MIS40C1_SW_PWDN_ON		0x0
#define MIS40C1_SW_PWDN_OFF		0x2
#define MIS40C1_REG_EXPOSURE_H		0x3100
#define MIS40C1_REG_EXPOSURE_L		0x3101
#define MIS40C1_EXPOSURE_MIN		1
#define MIS40C1_EXPOSURE_STEP		1
#define MIS40C1_VTS_MAX			0xffff

#define MIS40C1_REG_DIG_GAIN_R		0x3709
#define MIS40C1_REG_DIG_GAIN_GR		0x370a
#define MIS40C1_REG_DIG_GAIN_GB		0x370b
#define MIS40C1_REG_DIG_GAIN_B		0x370c

#define MIS40C1_REG_ANA_GAIN_H		0x3103
#define MIS40C1_REG_ANA_GAIN_L		0x3104
#define MIS40C1_GAIN_MIN		0x20
#define MIS40C1_GAIN_MAX		(64 * 2 * 32)
#define MIS40C1_GAIN_STEP		1

#define MIS40C1_GAIN_DEFAULT		0x20

#define MIS40C1_REG_GAIN_EXP_VALID	0x300c
#define MIS40C1_REG_GAIN_EXP_VALID_VAL	BIT(0)

#define MIS40C1_REG_TEST_PATTERN	0x3500
#define MIS40C1_TEST_PATTERN_BIT_MASK	BIT(0)

#define MIS40C1_REG_VTS_H		0x3105
#define MIS40C1_REG_VTS_L		0x3106

#define MIS40C1_FLIP_MIRROR_REG		0x3007

#define MIS40C1_FETCH_EXP_H(VAL)	(((VAL) >> 8) & 0xFF)
//#define MIS40C1_FETCH_EXP_M(VAL)	(((VAL) >> 4) & 0xFF)
#define MIS40C1_FETCH_EXP_L(VAL)	((VAL) & 0xFF)

#define MIS40C1_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define MIS40C1_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define MIS40C1_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x01 : VAL & 0xfe)
#define MIS40C1_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x10 : VAL & 0xfd)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define MIS40C1_REG_VALUE_08BIT		1
#define MIS40C1_REG_VALUE_16BIT		2
#define MIS40C1_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define MIS40C1_NAME			"mis40c1"

static const char * const mis40c1_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define MIS40C1_NUM_SUPPLIES ARRAY_SIZE(mis40c1_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct mis40c1_mode {
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

struct mis40c1 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[MIS40C1_NUM_SUPPLIES];

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
	const struct mis40c1_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			is_thunderboot;
	bool			is_first_streamoff;
};

#define to_mis40c1(sd) container_of(sd, struct mis40c1, subdev)

/*
 * Xclk 27Mhz
 */
static const struct regval mis40c1_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 630Mbps, 2lane
 */
static const struct regval mis40c1_linear_10_2560x1440_regs[] = {
	{0x302d, 0x01},
	{REG_DELAY, 0x50},
	{0x3006, 0x02},
	{0x3014, 0x00},
	{0x301f, 0x01},
	{0x3c1d, 0x0a},
	{0x3c1e, 0x00},
	{0x3c1f, 0x05},
	{0x3c20, 0xa0},

	{0x3106, 0xdc},
	{0x3105, 0x05},
	{0x3108, 0xe4},
	{0x3107, 0x0c},
	{0x310a, 0x04},
	{0x3109, 0x00},
	{0x310c, 0xa3},
	{0x310b, 0x05},
	{0x310e, 0x04},
	{0x310d, 0x00},
	{0x3110, 0x03},
	{0x310f, 0x0a},
	{0x3112, 0x0c},
	{0x61a8, 0x00},
	{0x4201, 0x01},
	{0x4200, 0x6e},
	{0x4203, 0x04},
	{0x4210, 0x00},
	{0x420a, 0x01},
	{0x4202, 0x01},
	{0x4208, 0x01},
	{0x4204, 0x01},
	{0x6101, 0x3c},
	{0x6100, 0x00},
	{0x6105, 0xff},
	{0x6104, 0x1f},
	{0x6103, 0xc3},
	{0x6102, 0x0c},
	{0x6107, 0xff},
	{0x6106, 0x1f},
	{0x6109, 0x7c},
	{0x6108, 0x04},
	{0x610d, 0xff},
	{0x610c, 0x1f},
	{0x610b, 0xa5},
	{0x610a, 0x05},
	{0x610f, 0xff},
	{0x610e, 0x1f},
	{0x6111, 0x00},
	{0x6110, 0x00},
	{0x6113, 0x35},
	{0x6112, 0x02},
	{0x6115, 0x6d},
	{0x6114, 0x04},
	{0x6117, 0xa8},
	{0x6116, 0x04},
	{0x6119, 0x7c},
	{0x6118, 0x04},
	{0x611b, 0xb7},
	{0x611a, 0x04},
	{0x611d, 0x1e},
	{0x611c, 0x00},
	{0x611f, 0x99},
	{0x611e, 0x04},
	{0x6121, 0x00},
	{0x6120, 0x00},
	{0x6123, 0xd2},
	{0x6122, 0x0c},
	{0x6125, 0x00},
	{0x6124, 0x00},
	{0x6127, 0xd2},
	{0x6126, 0x0c},
	{0x6129, 0x1e},
	{0x6128, 0x00},
	{0x612b, 0xab},
	{0x612a, 0x0c},
	{0x612d, 0x00},
	{0x612c, 0x00},
	{0x612f, 0x84},
	{0x612e, 0x04},
	{0x6131, 0x3c},
	{0x6130, 0x00},
	{0x6133, 0xbe},
	{0x6132, 0x01},
	{0x6135, 0x3c},
	{0x6134, 0x00},
	{0x6137, 0xaf},
	{0x6136, 0x01},
	{0x61ad, 0x17},
	{0x61ac, 0x02},
	{0x61b1, 0x5e},
	{0x61b0, 0x04},
	{0x61af, 0x35},
	{0x61ae, 0x02},
	{0x61b3, 0x39},
	{0x61b2, 0x06},
	{0x6139, 0x70},
	{0x6138, 0x02},
	{0x613d, 0x74},
	{0x613c, 0x06},
	{0x613b, 0x40},
	{0x613a, 0x04},
	{0x613f, 0xb4},
	{0x613e, 0x0c},
	{0x6141, 0x61},
	{0x6140, 0x02},
	{0x6143, 0x4f},
	{0x6142, 0x04},
	{0x6145, 0x66},
	{0x6144, 0x06},
	{0x6147, 0xc3},
	{0x6146, 0x0c},
	{0x6149, 0x52},
	{0x6148, 0x02},
	{0x614d, 0x57},
	{0x614c, 0x06},
	{0x614b, 0x3c},
	{0x614a, 0x04},
	{0x614f, 0xb0},
	{0x614e, 0x0c},
	{0x6151, 0x00},
	{0x6150, 0x00},
	{0x6155, 0x52},
	{0x6154, 0x02},
	{0x6159, 0x57},
	{0x6158, 0x06},
	{0x6153, 0x77},
	{0x6152, 0x00},
	{0x6157, 0x49},
	{0x6156, 0x04},
	{0x615b, 0xbd},
	{0x615a, 0x0c},
	{0x615d, 0x38},
	{0x615c, 0x03},
	{0x6161, 0x3c},
	{0x6160, 0x07},
	{0x615f, 0x40},
	{0x615e, 0x04},
	{0x6163, 0xb4},
	{0x6162, 0x0c},
	{0x6165, 0x00},
	{0x6164, 0x00},
	{0x6169, 0x7c},
	{0x6168, 0x04},
	{0x6167, 0x70},
	{0x6166, 0x02},
	{0x616b, 0x74},
	{0x616a, 0x06},
	{0x616d, 0xf9},
	{0x616c, 0x01},
	{0x6171, 0x5e},
	{0x6170, 0x04},
	{0x616f, 0x49},
	{0x616e, 0x04},
	{0x6173, 0xbd},
	{0x6172, 0x0c},
	{0x6175, 0x00},
	{0x6174, 0x00},
	{0x6177, 0x0f},
	{0x6176, 0x00},
	{0x6179, 0x01},
	{0x6178, 0x00},
	{0x617b, 0xbd},
	{0x617a, 0x0c},
	{0x617d, 0x00},
	{0x617c, 0x00},
	{0x617f, 0x29},
	{0x617e, 0x01},
	{0x6181, 0x00},
	{0x6180, 0x00},
	{0x6183, 0x29},
	{0x6182, 0x01},
	{0x6185, 0x00},
	{0x6184, 0x00},
	{0x6187, 0x29},
	{0x6186, 0x01},
	{0x61b5, 0x5e},
	{0x61b4, 0x04},
	{0x61b7, 0x6d},
	{0x61b6, 0x04},
	{0x6189, 0xd2},
	{0x6188, 0x0c},
	{0x618b, 0xe1},
	{0x618a, 0x0c},
	{0x618d, 0xd0},
	{0x618c, 0x00},
	{0x6191, 0x7c},
	{0x6190, 0x04},
	{0x618f, 0xee},
	{0x618e, 0x00},
	{0x6193, 0x99},
	{0x6192, 0x04},
	{0x6195, 0x0d},
	{0x6194, 0x0d},
	{0x61a3, 0xd0},
	{0x61a2, 0x01},
	{0x61a5, 0xcc},
	{0x61a4, 0x05},
	//BLC
	{0x5400, 0x2B},
	{0x5403, 0x08},
	{0x5406, 0x00},
	{0x5407, 0x40},
	{0x5408, 0x3e},
	{0x3902, 0x02},
	{0x3a04, 0x10},
	{0x3a17, 0x00},
	{0x3a18, 0x00},
	{0x3048, 0x01},
	{0x6207, 0x00},
	{0x6208, 0x00},
	//{0x3103, 0x03},
	//{0x3104, 0xe0},
	{0x3118, 0x01},
	{0x3700, 0x01},
	{0x3701, 0x00},
	{0x3702, 0x20},
	{0x3703, 0x00},
	{0x3704, 0x20},
	{0x3705, 0x00},
	{0x3706, 0x20},
	{0x3707, 0x00},
	{0x3708, 0x20},
	//{0x300c, 0x01},
	{0x610b, 0x40},
	{0x613d, 0x70},
	{0x61a1, 0x90},
	{0x6202, 0x0c},
	{0x3a03, 0x39},
	{0x3a02, 0xfc},
	{0x3a08, 0x0c},
	{0x3048, 0x01},
	{0x61ae, 0x02},
	{0x61af, 0x50},
	{0x61b2, 0x06},
	{0x61b3, 0x50},
	{0x3040, 0x01},
	{0x6200, 0x09},
	{0x6201, 0x09},
	{0x3a02, 0x2d},
	{0x3a03, 0x19},
	{0x6200, 0x00},
	{0x6201, 0x09},
	{0x6124, 0x1f},
	{0x6125, 0xff},
	{0x6126, 0x00},
	{0x6127, 0x00},
	{0x3a13, 0x39},
	{0x3a15, 0x08},
	{0x3a02, 0x2d},
	{0x5400, 0x23},
	{0x5407, 0x10},
	{0x3a0c, 0x07},
	{0x3040, 0x01},
	{0x6199, 0x90},
	{0x619b, 0x90},
	{0x619d, 0x90},
	{0x619f, 0x90},
	{0x61a1, 0x90},
	{0x420c, 0x00},
	{0x420d, 0x0a},
	{0x3a01, 0x01},
	{0x6209, 0x12},
	{0x3a02, 0xad},
	{0x610b, 0x08},
	{0x3a14, 0x0c},
	{0x3a0d, 0x30},
	{0x3a0e, 0xe0},
	{0x3118, 0x00},
	{0x3014, 0x00},
	//{0x3006, 0x00},   //pwdn on
	//{0x302d,0x00},//streaming  bit0 value 0/ON  value1/off

	{REG_NULL, 0x00},
};

static const struct mis40c1_mode supported_modes[] = {
	{
		.width = 2560,
		.height = 1440,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0020,
		.hts_def = 0x0ce4,
		.vts_def = 0x05dc,
		.bus_fmt = MEDIA_BUS_FMT_SGRBG10_1X10,
		.reg_list = mis40c1_linear_10_2560x1440_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	MIS40C1_LINK_FREQ_317
};

static const char * const mis40c1_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int mis40c1_write_reg(struct i2c_client *client, u16 reg,
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

static int mis40c1_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
			mdelay(regs[i].val);
		} else {
			ret = mis40c1_write_reg(client, regs[i].addr,
						MIS40C1_REG_VALUE_08BIT, regs[i].val);
		}
		if (ret) {
			dev_err(&client->dev, "write reg failed reg=%x value= %x\n",
				regs[i].addr, regs[i].val);
		}

	}

	return ret;
}

/* Read registers up to 4 at a time */
static int mis40c1_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static int mis40c1_set_gain_reg(struct mis40c1 *mis40c1, u32 gain)
{
	u32 again = 0, dgain = 0;
	int ret = 0;

	if (gain < 32)
		gain = 32;
	else if (gain > MIS40C1_GAIN_MAX)
		gain = MIS40C1_GAIN_MAX;

	if (gain < 2048) {
		again = 1024 - (1024 * 32 / gain);
		dgain = 0x80;
	} else {
		again = 1008;
		dgain = gain / 16;
		if (dgain > 0xff)
			dgain = 0xff;
	}

	//printk("set gain = %d again = %d dgain = %d\n",gain,coarse_again,dgain);
	ret = mis40c1_write_reg(mis40c1->client,
				MIS40C1_REG_DIG_GAIN_R,
				MIS40C1_REG_VALUE_08BIT,
				dgain);
	ret |= mis40c1_write_reg(mis40c1->client,
				MIS40C1_REG_DIG_GAIN_GR,
				MIS40C1_REG_VALUE_08BIT,
				dgain);
	ret |= mis40c1_write_reg(mis40c1->client,
				MIS40C1_REG_DIG_GAIN_GB,
				MIS40C1_REG_VALUE_08BIT,
				dgain);
	ret |= mis40c1_write_reg(mis40c1->client,
				MIS40C1_REG_DIG_GAIN_B,
				MIS40C1_REG_VALUE_08BIT,
				dgain);
	ret |= mis40c1_write_reg(mis40c1->client,
				 MIS40C1_REG_ANA_GAIN_H,
				 MIS40C1_REG_VALUE_08BIT,
				 MIS40C1_FETCH_AGAIN_H(again));
	ret |= mis40c1_write_reg(mis40c1->client,
				 MIS40C1_REG_ANA_GAIN_L,
				 MIS40C1_REG_VALUE_08BIT,
				 MIS40C1_FETCH_AGAIN_L(again));
	ret |= mis40c1_write_reg(mis40c1->client,
				 MIS40C1_REG_GAIN_EXP_VALID,
				 MIS40C1_REG_VALUE_08BIT,
				 MIS40C1_REG_GAIN_EXP_VALID_VAL);
	return ret;
}

static int mis40c1_get_reso_dist(const struct mis40c1_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct mis40c1_mode *
mis40c1_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = mis40c1_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int mis40c1_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	const struct mis40c1_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&mis40c1->mutex);

	mode = mis40c1_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&mis40c1->mutex);
		return -ENOTTY;
#endif
	} else {
		mis40c1->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(mis40c1->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(mis40c1->vblank, vblank_def,
					 MIS40C1_VTS_MAX - mode->height,
					 1, vblank_def);
		mis40c1->cur_fps = mode->max_fps;
	}

	mutex_unlock(&mis40c1->mutex);

	return 0;
}

static int mis40c1_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	const struct mis40c1_mode *mode = mis40c1->cur_mode;

	mutex_lock(&mis40c1->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&mis40c1->mutex);
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
	mutex_unlock(&mis40c1->mutex);

	return 0;
}

static int mis40c1_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = mis40c1->cur_mode->bus_fmt;

	return 0;
}

static int mis40c1_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int mis40c1_enable_test_pattern(struct mis40c1 *mis40c1, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = mis40c1_read_reg(mis40c1->client, MIS40C1_REG_TEST_PATTERN,
			       MIS40C1_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= MIS40C1_TEST_PATTERN_BIT_MASK;
	else
		val &= ~MIS40C1_TEST_PATTERN_BIT_MASK;

	ret |= mis40c1_write_reg(mis40c1->client, MIS40C1_REG_TEST_PATTERN,
				 MIS40C1_REG_VALUE_08BIT, val);
	return ret;
}

static int mis40c1_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	const struct mis40c1_mode *mode = mis40c1->cur_mode;

	if (mis40c1->streaming)
		fi->interval = mis40c1->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int mis40c1_g_mbus_config(struct v4l2_subdev *sd,
				 unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	const struct mis40c1_mode *mode = mis40c1->cur_mode;
	u32 val = 1 << (MIS40C1_LANES - 1) |
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

static void mis40c1_get_module_inf(struct mis40c1 *mis40c1,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, MIS40C1_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, mis40c1->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, mis40c1->len_name, sizeof(inf->base.lens));
}

static long mis40c1_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		mis40c1_get_module_inf(mis40c1, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = mis40c1->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = mis40c1->cur_mode->width;
		h = mis40c1->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				mis40c1->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&mis40c1->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = mis40c1->cur_mode->hts_def - mis40c1->cur_mode->width;
			h = mis40c1->cur_mode->vts_def - mis40c1->cur_mode->height;
			__v4l2_ctrl_modify_range(mis40c1->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(mis40c1->vblank, h,
						 MIS40C1_VTS_MAX - mis40c1->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);
		dev_err(&mis40c1->client->dev, "stream is %d\n", stream);
		if (stream)
			ret = mis40c1_write_reg(mis40c1->client, MIS40C1_REG_CTRL_MODE,
				 MIS40C1_REG_VALUE_08BIT, MIS40C1_MODE_STREAMING);
		else
			ret = mis40c1_write_reg(mis40c1->client, MIS40C1_REG_CTRL_MODE,
				 MIS40C1_REG_VALUE_08BIT, MIS40C1_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long mis40c1_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = mis40c1_ioctl(sd, cmd, inf);
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

		ret = mis40c1_ioctl(sd, cmd, hdr);
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
			ret = mis40c1_ioctl(sd, cmd, hdr);
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
			ret = mis40c1_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = mis40c1_ioctl(sd, cmd, &stream);
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

static int __mis40c1_start_stream(struct mis40c1 *mis40c1)
{
	int ret;

	if (!mis40c1->is_thunderboot) {
		ret = mis40c1_write_array(mis40c1->client, mis40c1->cur_mode->reg_list);
		if (ret)
			return ret;
		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&mis40c1->ctrl_handler);
		if (ret)
			return ret;
		ret = mis40c1_write_reg(mis40c1->client,
					MIS40C1_REG_CTRL_PWDN,
					MIS40C1_REG_VALUE_08BIT,
					MIS40C1_SW_PWDN_ON);
		if (ret)
			return ret;
	}

	ret = mis40c1_write_reg(mis40c1->client, MIS40C1_REG_CTRL_MODE,
				MIS40C1_REG_VALUE_08BIT, MIS40C1_MODE_STREAMING);

	return ret;
}

static int __mis40c1_stop_stream(struct mis40c1 *mis40c1)
{
	if (mis40c1->is_thunderboot) {
		mis40c1->is_first_streamoff = true;
		pm_runtime_put(&mis40c1->client->dev);
	}
	return mis40c1_write_reg(mis40c1->client, MIS40C1_REG_CTRL_MODE,
				 MIS40C1_REG_VALUE_08BIT, MIS40C1_MODE_SW_STANDBY);
}

static int __mis40c1_power_on(struct mis40c1 *mis40c1);
static int mis40c1_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	struct i2c_client *client = mis40c1->client;
	int ret = 0;

	mutex_lock(&mis40c1->mutex);
	on = !!on;
	if (on == mis40c1->streaming)
		goto unlock_and_return;

	if (on) {
		if (mis40c1->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			mis40c1->is_thunderboot = false;
			__mis40c1_power_on(mis40c1);
		}

		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __mis40c1_start_stream(mis40c1);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__mis40c1_stop_stream(mis40c1);
		pm_runtime_put(&client->dev);
	}

	mis40c1->streaming = on;

unlock_and_return:
	mutex_unlock(&mis40c1->mutex);

	return ret;
}

static int mis40c1_s_power(struct v4l2_subdev *sd, int on)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	struct i2c_client *client = mis40c1->client;
	int ret = 0;

	mutex_lock(&mis40c1->mutex);

	/* If the power state is not modified - no work to do. */
	if (mis40c1->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!mis40c1->is_thunderboot) {
			ret = mis40c1_write_array(mis40c1->client, mis40c1_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		mis40c1->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		mis40c1->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&mis40c1->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 mis40c1_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, MIS40C1_XVCLK_FREQ / 1000 / 1000);
}

static int __mis40c1_power_on(struct mis40c1 *mis40c1)
{
	int ret;
	u32 delay_us;
	struct device *dev = &mis40c1->client->dev;

	if (!IS_ERR_OR_NULL(mis40c1->pins_default)) {
		ret = pinctrl_select_state(mis40c1->pinctrl,
					   mis40c1->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(mis40c1->xvclk, MIS40C1_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(mis40c1->xvclk) != MIS40C1_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(mis40c1->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (mis40c1->is_thunderboot)
		return 0;

	if (!IS_ERR(mis40c1->reset_gpio))
		gpiod_set_value_cansleep(mis40c1->reset_gpio, 0);

	ret = regulator_bulk_enable(MIS40C1_NUM_SUPPLIES, mis40c1->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(mis40c1->reset_gpio))
		gpiod_set_value_cansleep(mis40c1->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(mis40c1->pwdn_gpio))
		gpiod_set_value_cansleep(mis40c1->pwdn_gpio, 1);

	if (!IS_ERR(mis40c1->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = mis40c1_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

disable_clk:
	clk_disable_unprepare(mis40c1->xvclk);

	return ret;
}

static void __mis40c1_power_off(struct mis40c1 *mis40c1)
{
	int ret;
	struct device *dev = &mis40c1->client->dev;

	clk_disable_unprepare(mis40c1->xvclk);
	if (mis40c1->is_thunderboot) {
		if (mis40c1->is_first_streamoff) {
			mis40c1->is_thunderboot = false;
			mis40c1->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(mis40c1->pwdn_gpio))
		gpiod_set_value_cansleep(mis40c1->pwdn_gpio, 0);
	if (!IS_ERR(mis40c1->reset_gpio))
		gpiod_set_value_cansleep(mis40c1->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(mis40c1->pins_sleep)) {
		ret = pinctrl_select_state(mis40c1->pinctrl,
					   mis40c1->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(MIS40C1_NUM_SUPPLIES, mis40c1->supplies);
}

static int __maybe_unused mis40c1_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis40c1 *mis40c1 = to_mis40c1(sd);

	return __mis40c1_power_on(mis40c1);
}

static int __maybe_unused mis40c1_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis40c1 *mis40c1 = to_mis40c1(sd);

	__mis40c1_power_off(mis40c1);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int mis40c1_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mis40c1 *mis40c1 = to_mis40c1(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct mis40c1_mode *def_mode = &supported_modes[0];

	mutex_lock(&mis40c1->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&mis40c1->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int mis40c1_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops mis40c1_pm_ops = {
	SET_RUNTIME_PM_OPS(mis40c1_runtime_suspend,
			   mis40c1_runtime_resume, NULL)

};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops mis40c1_internal_ops = {
	.open = mis40c1_open,
};
#endif

static const struct v4l2_subdev_core_ops mis40c1_core_ops = {
	.s_power = mis40c1_s_power,
	.ioctl = mis40c1_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = mis40c1_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops mis40c1_video_ops = {
	.s_stream = mis40c1_s_stream,
	.g_frame_interval = mis40c1_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops mis40c1_pad_ops = {
	.enum_mbus_code = mis40c1_enum_mbus_code,
	.enum_frame_size = mis40c1_enum_frame_sizes,
	.enum_frame_interval = mis40c1_enum_frame_interval,
	.get_fmt = mis40c1_get_fmt,
	.set_fmt = mis40c1_set_fmt,
	.get_mbus_config = mis40c1_g_mbus_config,
};

static const struct v4l2_subdev_ops mis40c1_subdev_ops = {
	.core	= &mis40c1_core_ops,
	.video	= &mis40c1_video_ops,
	.pad	= &mis40c1_pad_ops,
};

static void mis40c1_modify_fps_info(struct mis40c1 *mis40c1)
{
	const struct mis40c1_mode *mode = mis40c1->cur_mode;

	mis40c1->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				      mis40c1->cur_vts;
}

static int mis40c1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mis40c1 *mis40c1 = container_of(ctrl->handler,
					       struct mis40c1, ctrl_handler);
	struct i2c_client *client = mis40c1->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = mis40c1->cur_mode->height + ctrl->val - 2;
		__v4l2_ctrl_modify_range(mis40c1->exposure,
					 mis40c1->exposure->minimum, max,
					 mis40c1->exposure->step,
					 mis40c1->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		//printk("set exposure 0x%x\n", ctrl->val);
		if (mis40c1->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val;
			/* 4 least significant bits of expsoure are fractional part */
			ret = mis40c1_write_reg(mis40c1->client,
						MIS40C1_REG_EXPOSURE_H,
						MIS40C1_REG_VALUE_08BIT,
						MIS40C1_FETCH_EXP_H(val));
			ret |= mis40c1_write_reg(mis40c1->client,
						 MIS40C1_REG_EXPOSURE_L,
						 MIS40C1_REG_VALUE_08BIT,
						 MIS40C1_FETCH_EXP_L(val));
			ret |= mis40c1_write_reg(mis40c1->client,
						 MIS40C1_REG_GAIN_EXP_VALID,
						 MIS40C1_REG_VALUE_08BIT,
						 MIS40C1_REG_GAIN_EXP_VALID_VAL);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (mis40c1->cur_mode->hdr_mode == NO_HDR)
			ret = mis40c1_set_gain_reg(mis40c1, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = mis40c1_write_reg(mis40c1->client,
					MIS40C1_REG_VTS_H,
					MIS40C1_REG_VALUE_08BIT,
					(ctrl->val + mis40c1->cur_mode->height)
					>> 8);
		ret |= mis40c1_write_reg(mis40c1->client,
					 MIS40C1_REG_VTS_L,
					 MIS40C1_REG_VALUE_08BIT,
					 (ctrl->val + mis40c1->cur_mode->height)
					 & 0xff);
		mis40c1->cur_vts = ctrl->val + mis40c1->cur_mode->height;
		if (mis40c1->cur_vts != mis40c1->cur_mode->vts_def)
			mis40c1_modify_fps_info(mis40c1);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = mis40c1_enable_test_pattern(mis40c1, ctrl->val);
		break;

	case V4L2_CID_HFLIP://mirror
	case V4L2_CID_VFLIP://flip
		{
			static int en_flip_mir;
			int en = ctrl->val;

			if (ctrl->id == V4L2_CID_VFLIP) { //flip
				en_flip_mir = en ? (en_flip_mir | 0x03) : (en_flip_mir & 0x01);
			} else { //mirror
				en_flip_mir = en ? (en_flip_mir | 0x01) : (en_flip_mir & 0x02);
			}

			switch (en_flip_mir) {
			case 0:
				ret |= mis40c1_write_reg(mis40c1->client,
							 MIS40C1_FLIP_MIRROR_REG,
							 MIS40C1_REG_VALUE_08BIT, 0x00);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3205,
							 MIS40C1_REG_VALUE_08BIT, 0x00);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3207,
							 MIS40C1_REG_VALUE_08BIT, 0x3b);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3209,
							 MIS40C1_REG_VALUE_08BIT, 0x08);
				ret |= mis40c1_write_reg(mis40c1->client, 0x320b,
							 MIS40C1_REG_VALUE_08BIT, 0x87);
				break;

			case 1:
				ret |= mis40c1_write_reg(mis40c1->client,
							 MIS40C1_FLIP_MIRROR_REG,
							 MIS40C1_REG_VALUE_08BIT, 0x02);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3205,
							 MIS40C1_REG_VALUE_08BIT, 0x01);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3207,
							 MIS40C1_REG_VALUE_08BIT, 0x3c);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3209,
							 MIS40C1_REG_VALUE_08BIT, 0x08);
				ret |= mis40c1_write_reg(mis40c1->client, 0x320b,
							 MIS40C1_REG_VALUE_08BIT, 0x87);
				break;

			case 2:
				ret |= mis40c1_write_reg(mis40c1->client,
							 MIS40C1_FLIP_MIRROR_REG,
							 MIS40C1_REG_VALUE_08BIT, 0x01);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3205,
							 MIS40C1_REG_VALUE_08BIT, 0x00);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3207,
							 MIS40C1_REG_VALUE_08BIT, 0x3b);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3209,
							 MIS40C1_REG_VALUE_08BIT, 0x09);
				ret |= mis40c1_write_reg(mis40c1->client, 0x320b,
							 MIS40C1_REG_VALUE_08BIT, 0x88);
				break;

			case 3:
				ret |= mis40c1_write_reg(mis40c1->client,
							 MIS40C1_FLIP_MIRROR_REG,
							 MIS40C1_REG_VALUE_08BIT, 0x03);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3205,
							 MIS40C1_REG_VALUE_08BIT, 0x01);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3207,
							 MIS40C1_REG_VALUE_08BIT, 0x3c);
				ret |= mis40c1_write_reg(mis40c1->client, 0x3209,
							 MIS40C1_REG_VALUE_08BIT, 0x09);
				ret |= mis40c1_write_reg(mis40c1->client, 0x320b,
							 MIS40C1_REG_VALUE_08BIT, 0x88);
				break;

			default:
				break;
			}
		}
		break;

	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops mis40c1_ctrl_ops = {
	.s_ctrl = mis40c1_set_ctrl,
};

static int mis40c1_initialize_controls(struct mis40c1 *mis40c1)
{
	const struct mis40c1_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &mis40c1->ctrl_handler;
	mode = mis40c1->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &mis40c1->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_317M_10BIT, 1, PIXEL_RATE_WITH_317M_10BIT);

	h_blank = mode->hts_def - mode->width;
	mis40c1->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (mis40c1->hblank)
		mis40c1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	mis40c1->vblank = v4l2_ctrl_new_std(handler, &mis40c1_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    MIS40C1_VTS_MAX - mode->height,
					    1, vblank_def);
	mis40c1->cur_fps = mode->max_fps;
	exposure_max = mode->vts_def - 2;
	mis40c1->exposure = v4l2_ctrl_new_std(handler, &mis40c1_ctrl_ops,
					      V4L2_CID_EXPOSURE, MIS40C1_EXPOSURE_MIN,
					      exposure_max, MIS40C1_EXPOSURE_STEP,
					      mode->exp_def);
	mis40c1->anal_gain = v4l2_ctrl_new_std(handler, &mis40c1_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, MIS40C1_GAIN_MIN,
					       MIS40C1_GAIN_MAX, MIS40C1_GAIN_STEP,
					       MIS40C1_GAIN_DEFAULT);
	mis40c1->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &mis40c1_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(mis40c1_test_pattern_menu) - 1,
					0, 0, mis40c1_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &mis40c1_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &mis40c1_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&mis40c1->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	mis40c1->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int mis40c1_check_sensor_id(struct mis40c1 *mis40c1,
				   struct i2c_client *client)
{
	struct device *dev = &mis40c1->client->dev;
	u32 id = 0;
	int ret;

	if (mis40c1->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = mis40c1_read_reg(client, MIS40C1_REG_CHIP_ID,
			       MIS40C1_REG_VALUE_08BIT, &id);

	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected mis%06x sensor\n", CHIP_ID);

	return 0;
}

static int mis40c1_configure_regulators(struct mis40c1 *mis40c1)
{
	unsigned int i;

	for (i = 0; i < MIS40C1_NUM_SUPPLIES; i++)
		mis40c1->supplies[i].supply = mis40c1_supply_names[i];

	return devm_regulator_bulk_get(&mis40c1->client->dev,
				       MIS40C1_NUM_SUPPLIES,
				       mis40c1->supplies);
}

static int mis40c1_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct mis40c1 *mis40c1;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	mis40c1 = devm_kzalloc(dev, sizeof(*mis40c1), GFP_KERNEL);
	if (!mis40c1)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &mis40c1->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &mis40c1->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &mis40c1->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &mis40c1->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	mis40c1->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	mis40c1->client = client;
	mis40c1->cur_mode = &supported_modes[0];

	mis40c1->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(mis40c1->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	if (mis40c1->is_thunderboot) {
		mis40c1->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
		if (IS_ERR(mis40c1->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		mis40c1->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
		if (IS_ERR(mis40c1->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	} else {
		mis40c1->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(mis40c1->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		mis40c1->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
		if (IS_ERR(mis40c1->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	}
	mis40c1->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(mis40c1->pinctrl)) {
		mis40c1->pins_default =
			pinctrl_lookup_state(mis40c1->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(mis40c1->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		mis40c1->pins_sleep =
			pinctrl_lookup_state(mis40c1->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(mis40c1->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = mis40c1_configure_regulators(mis40c1);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&mis40c1->mutex);

	sd = &mis40c1->subdev;
	v4l2_i2c_subdev_init(sd, client, &mis40c1_subdev_ops);
	ret = mis40c1_initialize_controls(mis40c1);
	if (ret)
		goto err_destroy_mutex;

	ret = __mis40c1_power_on(mis40c1);
	if (ret)
		goto err_free_handler;

	ret = mis40c1_check_sensor_id(mis40c1, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &mis40c1_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	mis40c1->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &mis40c1->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(mis40c1->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 mis40c1->module_index, facing,
		 MIS40C1_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (mis40c1->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__mis40c1_power_off(mis40c1);
err_free_handler:
	v4l2_ctrl_handler_free(&mis40c1->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&mis40c1->mutex);

	return ret;
}

static int mis40c1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis40c1 *mis40c1 = to_mis40c1(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&mis40c1->ctrl_handler);
	mutex_destroy(&mis40c1->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__mis40c1_power_off(mis40c1);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mis40c1_of_match[] = {
	{ .compatible = "mis,mis40c1" },
	{},
};
MODULE_DEVICE_TABLE(of, mis40c1_of_match);
#endif

static const struct i2c_device_id mis40c1_match_id[] = {
	{ "mis,mis40c1", 0 },
	{ },
};

static struct i2c_driver mis40c1_i2c_driver = {
	.driver = {
		.name = MIS40C1_NAME,
		.pm = &mis40c1_pm_ops,
		.of_match_table = of_match_ptr(mis40c1_of_match),
	},
	.probe		= &mis40c1_probe,
	.remove		= &mis40c1_remove,
	.id_table	= mis40c1_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&mis40c1_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&mis40c1_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("imagedesign mis40c1 sensor driver");
MODULE_LICENSE("GPL");
