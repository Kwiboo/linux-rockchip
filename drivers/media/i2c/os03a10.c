// SPDX-License-Identifier: GPL-2.0
/*
 * os03a10 driver
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 * V0.0X01.0X02 support fastboot
 * V0.0X01.0X03 support sleep & wake_up function
 */

//#define DEBUG

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
#include "cam-tb-setup.h"
#include "cam-sleep-wakeup.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OS03A10_LANES			2
#define OS03A10_BITS_PER_SAMPLE		10
#define OS03A10_LINK_FREQ_720		720000000

#define PIXEL_RATE_WITH_720M_10BIT	(OS03A10_LINK_FREQ_720  * 2 / OS03A10_BITS_PER_SAMPLE \
					* OS03A10_LANES)

#define OS03A10_XVCLK_FREQ		24000000

#define CHIP_ID				0x5303
#define OS03A10_REG_CHIP_ID		0x300a

#define OS03A10_REG_CTRL_MODE		0x0100
#define OS03A10_MODE_SW_STANDBY		0x0
#define OS03A10_MODE_STREAMING		BIT(0)

#define OS03A10_REG_EXPOSURE_H		0x3501
#define OS03A10_REG_EXPOSURE_L		0x3502
#define OS03A10_EXPOSURE_MIN		2
#define OS03A10_EXPOSURE_STEP		1
#define OS03A10_VTS_MAX			0x7fff

#define OS03A10_REG_DIG_GAIN		0x350A	//low 4bit
#define OS03A10_REG_DIG_FINE_GAIN	0x350B	//8bit
#define OS03A10_REG_DIG_FINE_GAIN_1	0x350C	//high 2bit
#define OS03A10_REG_ANA_GAIN		0x3508	//low 5bit
#define OS03A10_REG_ANA_FINE_GAIN	0x3509	//high 4bit	 fractional
#define OS03A10_GAIN_MIN		0x0020
#define OS03A10_GAIN_MAX		8186//(16 * 16 * 32)	//16*15.99*32  8192 8186.88
#define OS03A10_GAIN_STEP		1
#define OS03A10_GAIN_DEFAULT		0x20

#define OS03A10_REG_TEST_PATTERN	0x5080
#define OS03A10_TEST_PATTERN_BIT_MASK	BIT(7)

#define OS03A10_REG_VTS_H		0x380e
#define OS03A10_REG_VTS_L		0x380f

#define OS03A10_FLIP_MIRROR_REG		0x3820

#define OS03A10_FETCH_EXP_H(VAL)	(((VAL) >> 8) & 0xFF)
#define OS03A10_FETCH_EXP_L(VAL)	((VAL) & 0xFF)

#define OS03A10_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x02 : VAL & 0x04)
#define OS03A10_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x04 : VAL & 0x02)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define OS03A10_REG_VALUE_08BIT		1
#define OS03A10_REG_VALUE_16BIT		2
#define OS03A10_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OS03A10_NAME			"os03a10"

static const char * const os03a10_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OS03A10_NUM_SUPPLIES ARRAY_SIZE(os03a10_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct os03a10_mode {
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

struct os03a10 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OS03A10_NUM_SUPPLIES];

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
	const struct os03a10_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	bool			is_standby;
	struct cam_sw_info	*cam_sw_inf;
};

#define to_os03a10(sd) container_of(sd, struct os03a10, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval os03a10_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 60fps
 * mipi_datarate per lane 1440Mbps, 2lane
 */
static const struct regval os03a10_linear_10_2560x1440_regs[] = {
	{0x0103, 0x01},
	{0x0109, 0x01},
	{0x0104, 0x02},
	{0x0102, 0x00},
	{0x0305, 0x78},
	{0x0306, 0x00},
	{0x0308, 0x04},
	{0x030a, 0x01},
	{0x0317, 0x09},
	{0x0322, 0x01},
	{0x0323, 0x02},
	{0x0324, 0x00},
	{0x0325, 0x90},
	{0x0327, 0x05},
	{0x0329, 0x01},
	{0x032c, 0x02},
	{0x300f, 0x11},
	{0x3012, 0x21},
	{0x3026, 0x10},
	{0x3027, 0x08},
	{0x302d, 0x24},
	{0x3106, 0x10},
	{0x3400, 0x00},
	{0x3408, 0x05},
	{0x340c, 0x0c},
	{0x340d, 0xb0},
	{0x3425, 0x51},
	{0x3426, 0x10},
	{0x3427, 0x14},
	{0x3428, 0x10},
	{0x3429, 0x10},
	{0x342a, 0x10},
	{0x342b, 0x04},
	{0x3501, 0x02},
	{0x3504, 0x08},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x350a, 0x01},
	{0x3542, 0x10},
	{0x3544, 0x08},
	{0x3548, 0x01},
	{0x3549, 0x00},
	{0x3582, 0x02},
	{0x3584, 0x08},
	{0x3588, 0x01},
	{0x3589, 0x00},
	{0x3601, 0x70},
	{0x3604, 0xe3},
	{0x3603, 0xe5},
	{0x3605, 0x7f},
	{0x3606, 0x00},
	{0x3608, 0xa8},
	{0x360a, 0xd0},
	{0x360b, 0x18},
	{0x360e, 0xc8},
	{0x360f, 0x66},
	{0x3610, 0x89},
	{0x3611, 0x8a},
	{0x3612, 0x4e},
	{0x3613, 0xbd},
	{0x3614, 0x9b},
	{0x3623, 0x08},
	{0x362a, 0x0e},
	{0x362b, 0x0e},
	{0x362c, 0x0e},
	{0x362d, 0x0e},
	{0x362e, 0x07},
	{0x362f, 0x0f},
	{0x3630, 0x1f},
	{0x3631, 0x40},
	{0x3638, 0x00},
	{0x3643, 0x00},
	{0x3644, 0x00},
	{0x3645, 0x00},
	{0x3646, 0x00},
	{0x3647, 0x00},
	{0x3648, 0x00},
	{0x3649, 0x00},
	{0x364a, 0x04},
	{0x364c, 0x0e},
	{0x364d, 0x0e},
	{0x364e, 0x0e},
	{0x364f, 0x0e},
	{0x3650, 0xff},
	{0x3651, 0xff},
	{0x365a, 0x00},
	{0x365b, 0x00},
	{0x365c, 0x00},
	{0x365d, 0x00},
	{0x3661, 0x07},
	{0x3662, 0x02},
	{0x3663, 0x20},
	{0x3665, 0x12},
	{0x3667, 0xd4},
	{0x3668, 0x80},
	{0x366c, 0x00},
	{0x366d, 0x00},
	{0x366e, 0x00},
	{0x366f, 0x00},
	{0x3671, 0x08},
	{0x3673, 0x2a},
	{0x3681, 0x80},
	{0x3700, 0x2d},
	{0x3701, 0x22},
	{0x3702, 0x25},
	{0x3703, 0x20},
	{0x3705, 0x00},
	{0x3706, 0x60},
	{0x3707, 0x0a},
	{0x3708, 0x36},
	{0x3709, 0x57},
	{0x370a, 0x00},
	{0x370b, 0xd0},
	{0x3714, 0x01},
	{0x371b, 0x16},
	{0x371c, 0x00},
	{0x371d, 0x08},
	{0x373f, 0x4c},
	{0x3740, 0x4c},
	{0x3741, 0x4c},
	{0x3742, 0x63},
	{0x3756, 0x95},
	{0x3757, 0x95},
	{0x3762, 0x1d},
	{0x376c, 0x00},
	{0x3776, 0x05},
	{0x3777, 0x22},
	{0x3779, 0x60},
	{0x377c, 0x48},
	{0x3793, 0x04},
	{0x3794, 0x07},
	{0x379c, 0x4d},
	{0x3784, 0x06},
	{0x3785, 0x0a},
	{0x37c4, 0x72},
	{0x37c5, 0x72},
	{0x37c6, 0x72},
	{0x37d0, 0x00},
	{0x37d1, 0x60},
	{0x37d2, 0x00},
	{0x37d3, 0xd0},
	{0x37d4, 0x00},
	{0x37d5, 0x6c},
	{0x37d6, 0x00},
	{0x37d7, 0xf7},
	{0x37d8, 0x01},
	{0x37da, 0x00},
	{0x37db, 0x00},
	{0x37dc, 0x00},
	{0x37dd, 0x00},
	{0x3790, 0x10},
	{0x3793, 0x04},
	{0x3794, 0x07},
	{0x3796, 0x00},
	{0x3797, 0x02},
	{0x37a1, 0x80},
	{0x37bb, 0x88},
	{0x37bd, 0x01},
	{0x37be, 0x48},
	{0x37bf, 0x01},
	{0x37c0, 0x01},
	{0x37ca, 0x21},
	{0x37cc, 0x13},
	{0x37cd, 0x90},
	{0x37cf, 0x02},
	{0x37ec, 0x00},
	{0x37ed, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x0f},
	{0x3806, 0x05},
	{0x3807, 0xaf},
	{0x3808, 0x0a},
	{0x3809, 0x00},
	{0x380a, 0x05},
	{0x380b, 0xa0},
	{0x380c, 0x02},
	{0x380d, 0xe2},
	{0x380e, 0x06},
	{0x380f, 0x58},
	{0x3811, 0x08},
	{0x3813, 0x08},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3816, 0x01},
	{0x3817, 0x01},
	{0x381c, 0x00},
	{0x3820, 0x02},
	{0x3821, 0x00},
	{0x3822, 0x14},
	{0x3823, 0x18},
	{0x3826, 0x00},
	{0x3827, 0x00},
	{0x3833, 0x40},
	{0x384c, 0x02},
	{0x384d, 0xdc},
	{0x3858, 0x3c},
	{0x3865, 0x02},
	{0x3866, 0x00},
	{0x3867, 0x00},
	{0x3868, 0x02},
	{0x3900, 0x13},
	{0x3940, 0x13},
	{0x3980, 0x13},
	{0x3c01, 0x11},
	{0x3c05, 0x00},
	{0x3c0f, 0x1c},
	{0x3c12, 0x0d},
	{0x3c19, 0x00},
	{0x3c21, 0x00},
	{0x3c3a, 0x50},
	{0x3c3b, 0x18},
	{0x3c3d, 0xc6},
	{0x3c55, 0xcb},
	{0x3c5d, 0xcf},
	{0x3c5e, 0xcf},
	{0x3ce0, 0x00},
	{0x3ce1, 0x00},
	{0x3ce2, 0x00},
	{0x3ce3, 0x00},
	{0x3d8c, 0x70},
	{0x3d8d, 0x10},
	{0x4001, 0x2f},
	{0x4004, 0x00},
	{0x4005, 0x40},
	{0x4008, 0x02},
	{0x4009, 0x11},
	{0x400a, 0x07},
	{0x400b, 0x00},
	{0x400e, 0x40},
	{0x4011, 0xbb},
	{0x402e, 0x00},
	{0x402f, 0x40},
	{0x4030, 0x00},
	{0x4031, 0x40},
	{0x4032, 0x0f},
	{0x4033, 0x80},
	{0x4050, 0x00},
	{0x4051, 0x07},
	{0x405e, 0x20},
	{0x410f, 0x01},
	{0x4288, 0xcf},
	{0x4289, 0x00},
	{0x428a, 0x46},
	{0x430b, 0x0f},
	{0x430c, 0xfc},
	{0x430d, 0x00},
	{0x430e, 0x00},
	{0x4314, 0x04},
	{0x4500, 0x18},
	{0x4501, 0x18},
	{0x4503, 0x10},
	{0x4504, 0x00},
	{0x4506, 0x32},
	{0x4507, 0x02},
	{0x4601, 0x30},
	{0x4603, 0x00},
	{0x460a, 0x50},
	{0x460c, 0x60},
	{0x4640, 0x62},
	{0x4646, 0xaa},
	{0x4647, 0x55},
	{0x4648, 0x99},
	{0x4649, 0x66},
	{0x464d, 0x00},
	{0x4654, 0x11},
	{0x4655, 0x22},
	{0x4800, 0x04},
	{0x480e, 0x00},
	{0x4810, 0xff},
	{0x4811, 0xff},
	{0x4813, 0x00},
	{0x481f, 0x30},
	{0x4837, 0x0b},
	{0x484b, 0x27},
	{0x4d00, 0x4d},
	{0x4d01, 0x9d},
	{0x4d02, 0xb9},
	{0x4d03, 0x2e},
	{0x4d04, 0x4a},
	{0x4d05, 0x3d},
	{0x4d09, 0x4f},
	{0x5000, 0x1f},
	{0x5001, 0x0d},
	{0x5080, 0x00},
	{0x50c0, 0x00},
	{0x5100, 0x00},
	{0x5200, 0x00},
	{0x5201, 0x00},
	{0x5202, 0x03},
	{0x5203, 0xff},
	{0x5780, 0x53},
	{0x5782, 0x18},
	{0x5783, 0x3c},
	{0x5786, 0x01},
	{0x5788, 0x18},
	{0x5789, 0x3c},
	{0x5792, 0x11},
	{0x5793, 0x33},
	{0x5857, 0x00},
	{0x5858, 0x00},
	{0x5859, 0x00},
	{0x58d7, 0x00},
	{0x58d8, 0x00},
	{0x58d9, 0x00},
	{REG_NULL, 0x00},
};

static const struct os03a10_mode supported_modes[] = {
	{
		.width = 2560,
		.height = 1440,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0240,
		.hts_def = 0x02e2 * 4,
		.vts_def = 0x0658,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = os03a10_linear_10_2560x1440_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	OS03A10_LINK_FREQ_720
};

static const char * const os03a10_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int os03a10_write_reg(struct i2c_client *client, u16 reg,
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

static int os03a10_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	pr_info("write array\n");
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = os03a10_write_reg(client, regs[i].addr,
					OS03A10_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int os03a10_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static int os03a10_set_gain_reg(struct os03a10 *os03a10, u32 gain)
{
	u32 coarse_again = 0, fine_agian = 0, coarse_dgian = 0, fine_dgian = 0;
	u32 gain_factor;
	int ret = 0;

	if (gain < 32)
		gain = 32;
	else if (gain > OS03A10_GAIN_MAX)
		gain = OS03A10_GAIN_MAX;

	gain_factor = gain * 1000 / 32;
	if (gain_factor <= 16000) {	//again 0-16
		coarse_again = gain/32;
		fine_agian = (gain - (coarse_again * 32))/2;	// *32/2
		fine_agian = fine_agian<<4;	//high 4bit
		coarse_dgian = 0x01;
		fine_dgian = 0x00;
	} else if (gain_factor <= 255840) {//dgain 16*15.99
		coarse_again = 0x10;
		fine_agian = 0x00;
		coarse_dgian = gain/32/16;
		fine_dgian = (gain - (coarse_dgian * 16 * 32))/2;	// *32*16*2/4
		//fine_dgian_2 = gain_factor * 128 / 2000;
	}

	ret = os03a10_write_reg(os03a10->client,
				OS03A10_REG_DIG_GAIN,
				OS03A10_REG_VALUE_08BIT,
				coarse_dgian);
	ret |= os03a10_write_reg(os03a10->client,
				 OS03A10_REG_DIG_FINE_GAIN,
				 OS03A10_REG_VALUE_08BIT,
				 fine_dgian);
	ret |= os03a10_write_reg(os03a10->client,
				 OS03A10_REG_ANA_GAIN,
				 OS03A10_REG_VALUE_08BIT,
				 coarse_again);
	ret |= os03a10_write_reg(os03a10->client,
				 OS03A10_REG_ANA_FINE_GAIN,
				 OS03A10_REG_VALUE_08BIT,
				 fine_agian);
	return ret;
}

static int os03a10_get_reso_dist(const struct os03a10_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct os03a10_mode *
os03a10_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = os03a10_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int os03a10_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	const struct os03a10_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&os03a10->mutex);

	mode = os03a10_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&os03a10->mutex);
		return -ENOTTY;
#endif
	} else {
		os03a10->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(os03a10->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(os03a10->vblank, vblank_def,
					 OS03A10_VTS_MAX - mode->height,
					 1, vblank_def);
		os03a10->cur_fps = mode->max_fps;
	}

	mutex_unlock(&os03a10->mutex);

	return 0;
}

static int os03a10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	const struct os03a10_mode *mode = os03a10->cur_mode;

	mutex_lock(&os03a10->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&os03a10->mutex);
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
	mutex_unlock(&os03a10->mutex);

	return 0;
}

static int os03a10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct os03a10 *os03a10 = to_os03a10(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = os03a10->cur_mode->bus_fmt;

	return 0;
}

static int os03a10_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int os03a10_enable_test_pattern(struct os03a10 *os03a10, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = os03a10_read_reg(os03a10->client, OS03A10_REG_TEST_PATTERN,
			       OS03A10_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= OS03A10_TEST_PATTERN_BIT_MASK;
	else
		val &= ~OS03A10_TEST_PATTERN_BIT_MASK;

	ret |= os03a10_write_reg(os03a10->client, OS03A10_REG_TEST_PATTERN,
				 OS03A10_REG_VALUE_08BIT, val);
	return ret;
}

static int os03a10_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	const struct os03a10_mode *mode = os03a10->cur_mode;

	if (os03a10->streaming)
		fi->interval = os03a10->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int os03a10_g_mbus_config(struct v4l2_subdev *sd,
				 unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	const struct os03a10_mode *mode = os03a10->cur_mode;
	u32 val = 1 << (OS03A10_LANES - 1) |
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

static void os03a10_get_module_inf(struct os03a10 *os03a10,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OS03A10_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, os03a10->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, os03a10->len_name, sizeof(inf->base.lens));
}

static long os03a10_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		os03a10_get_module_inf(os03a10, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = os03a10->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = os03a10->cur_mode->width;
		h = os03a10->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				os03a10->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&os03a10->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = os03a10->cur_mode->hts_def - os03a10->cur_mode->width;
			h = os03a10->cur_mode->vts_def - os03a10->cur_mode->height;
			__v4l2_ctrl_modify_range(os03a10->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(os03a10->vblank, h,
						 OS03A10_VTS_MAX - os03a10->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream) {
			ret = os03a10_write_reg(os03a10->client, OS03A10_REG_CTRL_MODE,
						OS03A10_REG_VALUE_08BIT, OS03A10_MODE_STREAMING);
			dev_info(&os03a10->client->dev, "quickstream, streaming: is_standby = false\n");
			os03a10->is_standby = false;
		} else {
			ret = os03a10_write_reg(os03a10->client, OS03A10_REG_CTRL_MODE,
						OS03A10_REG_VALUE_08BIT, OS03A10_MODE_SW_STANDBY);

			dev_info(&os03a10->client->dev, "quickstream, not streaming: is_standby = true\n");
			os03a10->is_standby = true;
		}
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long os03a10_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = os03a10_ioctl(sd, cmd, inf);
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

		ret = os03a10_ioctl(sd, cmd, hdr);
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
			ret = os03a10_ioctl(sd, cmd, hdr);
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
			ret = os03a10_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = os03a10_ioctl(sd, cmd, &stream);
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

static int __os03a10_start_stream(struct os03a10 *os03a10)
{
	int ret;

	if (!os03a10->is_thunderboot) {
		ret = os03a10_write_array(os03a10->client, os03a10->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&os03a10->ctrl_handler);
		if (ret)
			return ret;
	}

	return os03a10_write_reg(os03a10->client, OS03A10_REG_CTRL_MODE,
				 OS03A10_REG_VALUE_08BIT, OS03A10_MODE_STREAMING);
}

static int __os03a10_stop_stream(struct os03a10 *os03a10)
{
	if (os03a10->is_thunderboot) {
		os03a10->is_first_streamoff = true;
		pm_runtime_put(&os03a10->client->dev);
	}
	return os03a10_write_reg(os03a10->client, OS03A10_REG_CTRL_MODE,
				 OS03A10_REG_VALUE_08BIT, OS03A10_MODE_SW_STANDBY);
}

static int __os03a10_power_on(struct os03a10 *os03a10);
static int os03a10_s_stream(struct v4l2_subdev *sd, int on)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	struct i2c_client *client = os03a10->client;
	int ret = 0;

	mutex_lock(&os03a10->mutex);
	on = !!on;
	if (on == os03a10->streaming)
		goto unlock_and_return;

	if (on) {
		if (os03a10->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			os03a10->is_thunderboot = false;
			__os03a10_power_on(os03a10);
		}

		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __os03a10_start_stream(os03a10);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__os03a10_stop_stream(os03a10);
		pm_runtime_put(&client->dev);
	}

	os03a10->streaming = on;

unlock_and_return:
	mutex_unlock(&os03a10->mutex);

	return ret;
}

static int os03a10_s_power(struct v4l2_subdev *sd, int on)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	struct i2c_client *client = os03a10->client;
	int ret = 0;

	mutex_lock(&os03a10->mutex);

	/* If the power state is not modified - no work to do. */
	if (os03a10->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!os03a10->is_thunderboot) {
			ret = os03a10_write_array(os03a10->client, os03a10_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		os03a10->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		os03a10->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&os03a10->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 os03a10_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OS03A10_XVCLK_FREQ / 1000 / 1000);
}

static int __os03a10_power_on(struct os03a10 *os03a10)
{
	int ret;
	u32 delay_us;
	struct device *dev = &os03a10->client->dev;

	if (!IS_ERR_OR_NULL(os03a10->pins_default)) {
		ret = pinctrl_select_state(os03a10->pinctrl,
					   os03a10->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(os03a10->xvclk, OS03A10_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(os03a10->xvclk) != OS03A10_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(os03a10->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	cam_sw_regulator_bulk_init(os03a10->cam_sw_inf,
				   OS03A10_NUM_SUPPLIES, os03a10->supplies);
	if (os03a10->is_thunderboot)
		return 0;

	if (!IS_ERR(os03a10->reset_gpio))
		gpiod_set_value_cansleep(os03a10->reset_gpio, 0);

	ret = regulator_bulk_enable(OS03A10_NUM_SUPPLIES, os03a10->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(os03a10->reset_gpio))
		gpiod_set_value_cansleep(os03a10->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(os03a10->pwdn_gpio))
		gpiod_set_value_cansleep(os03a10->pwdn_gpio, 1);

	if (!IS_ERR(os03a10->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = os03a10_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(os03a10->xvclk);

	return ret;
}

static void __os03a10_power_off(struct os03a10 *os03a10)
{
	int ret;
	struct device *dev = &os03a10->client->dev;

	clk_disable_unprepare(os03a10->xvclk);
	if (os03a10->is_thunderboot) {
		if (os03a10->is_first_streamoff) {
			os03a10->is_thunderboot = false;
			os03a10->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(os03a10->pwdn_gpio))
		gpiod_set_value_cansleep(os03a10->pwdn_gpio, 0);
	if (!IS_ERR(os03a10->reset_gpio))
		gpiod_set_value_cansleep(os03a10->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(os03a10->pins_sleep)) {
		ret = pinctrl_select_state(os03a10->pinctrl,
					   os03a10->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(OS03A10_NUM_SUPPLIES, os03a10->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int __maybe_unused os03a10_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os03a10 *os03a10 = to_os03a10(sd);
	int val = 0;
	long ret = 0;

	cam_sw_prepare_wakeup(os03a10->cam_sw_inf, dev);

	usleep_range(4000, 5000);
	cam_sw_write_array(os03a10->cam_sw_inf);

	ret = os03a10_read_reg(os03a10->client, OS03A10_REG_CTRL_MODE,
			   OS03A10_REG_VALUE_08BIT, &val);
	if (ret)
		v4l2_err(sd, "resume read os03a10_REG_CTRL_MODE failed\n");

	if (__v4l2_ctrl_handler_setup(&os03a10->ctrl_handler))
		dev_err(dev, "__v4l2_ctrl_handler_setup fail!");

	return 0;
}

static int __maybe_unused os03a10_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os03a10 *os03a10 = to_os03a10(sd);

	cam_sw_write_array_cb_init(os03a10->cam_sw_inf, client,
				   (void *)os03a10->cur_mode->reg_list,
				   (sensor_write_array)os03a10_write_array);
	cam_sw_prepare_sleep(os03a10->cam_sw_inf);

	return 0;
}
#else
#define os03a10_resume NULL
#define os03a10_suspend NULL
#endif

static int __maybe_unused os03a10_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os03a10 *os03a10 = to_os03a10(sd);

	return __os03a10_power_on(os03a10);
}

static int __maybe_unused os03a10_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os03a10 *os03a10 = to_os03a10(sd);

	__os03a10_power_off(os03a10);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int os03a10_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct os03a10 *os03a10 = to_os03a10(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct os03a10_mode *def_mode = &supported_modes[0];

	mutex_lock(&os03a10->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&os03a10->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int os03a10_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops os03a10_pm_ops = {
	SET_RUNTIME_PM_OPS(os03a10_runtime_suspend,
			   os03a10_runtime_resume, NULL)
#ifdef CONFIG_VIDEO_CAM_SLEEP_WAKEUP
	SET_LATE_SYSTEM_SLEEP_PM_OPS(os03a10_suspend, os03a10_resume)
#endif
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops os03a10_internal_ops = {
	.open = os03a10_open,
};
#endif

static const struct v4l2_subdev_core_ops os03a10_core_ops = {
	.s_power = os03a10_s_power,
	.ioctl = os03a10_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = os03a10_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops os03a10_video_ops = {
	.s_stream = os03a10_s_stream,
	.g_frame_interval = os03a10_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops os03a10_pad_ops = {
	.enum_mbus_code = os03a10_enum_mbus_code,
	.enum_frame_size = os03a10_enum_frame_sizes,
	.enum_frame_interval = os03a10_enum_frame_interval,
	.get_fmt = os03a10_get_fmt,
	.set_fmt = os03a10_set_fmt,
	.get_mbus_config = os03a10_g_mbus_config,
};

static const struct v4l2_subdev_ops os03a10_subdev_ops = {
	.core	= &os03a10_core_ops,
	.video	= &os03a10_video_ops,
	.pad	= &os03a10_pad_ops,
};

static void os03a10_modify_fps_info(struct os03a10 *os03a10)
{
	const struct os03a10_mode *mode = os03a10->cur_mode;

	os03a10->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       os03a10->cur_vts;
}

static int os03a10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct os03a10 *os03a10 = container_of(ctrl->handler,
					       struct os03a10, ctrl_handler);
	struct i2c_client *client = os03a10->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = os03a10->cur_mode->height + ctrl->val - 8;
		__v4l2_ctrl_modify_range(os03a10->exposure,
					 os03a10->exposure->minimum, max,
					 os03a10->exposure->step,
					 os03a10->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	if (os03a10->is_standby) {
		dev_dbg(&client->dev, "%s: is_standby = true, will return\n", __func__);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (os03a10->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val;
			/* 4 least significant bits of expsoure are fractional part */
			ret = os03a10_write_reg(os03a10->client,
						OS03A10_REG_EXPOSURE_H,
						OS03A10_REG_VALUE_08BIT,
						OS03A10_FETCH_EXP_H(val));
			ret |= os03a10_write_reg(os03a10->client,
						 OS03A10_REG_EXPOSURE_L,
						 OS03A10_REG_VALUE_08BIT,
						 OS03A10_FETCH_EXP_L(val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (os03a10->cur_mode->hdr_mode == NO_HDR)
			ret = os03a10_set_gain_reg(os03a10, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = os03a10_write_reg(os03a10->client,
					OS03A10_REG_VTS_H,
					OS03A10_REG_VALUE_08BIT,
					(ctrl->val + os03a10->cur_mode->height)
					>> 8);
		ret |= os03a10_write_reg(os03a10->client,
					 OS03A10_REG_VTS_L,
					 OS03A10_REG_VALUE_08BIT,
					 (ctrl->val + os03a10->cur_mode->height)
					 & 0xff);
		os03a10->cur_vts = ctrl->val + os03a10->cur_mode->height;
		os03a10_modify_fps_info(os03a10);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = os03a10_enable_test_pattern(os03a10, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = os03a10_read_reg(os03a10->client, OS03A10_FLIP_MIRROR_REG,
				       OS03A10_REG_VALUE_08BIT, &val);

		ret |= os03a10_write_reg(os03a10->client, OS03A10_FLIP_MIRROR_REG,
					 OS03A10_REG_VALUE_08BIT,
					 OS03A10_FETCH_MIRROR(val, ctrl->val));
		os03a10_read_reg(os03a10->client, OS03A10_FLIP_MIRROR_REG,
				 OS03A10_REG_VALUE_08BIT, &val);
		dev_dbg(&client->dev, "set hflip %d", val);
		break;
	case V4L2_CID_VFLIP:
		ret = os03a10_read_reg(os03a10->client, OS03A10_FLIP_MIRROR_REG,
				       OS03A10_REG_VALUE_08BIT, &val);
		ret |= os03a10_write_reg(os03a10->client, OS03A10_FLIP_MIRROR_REG,
					 OS03A10_REG_VALUE_08BIT,
					 OS03A10_FETCH_FLIP(val, ctrl->val));
		os03a10_read_reg(os03a10->client, OS03A10_FLIP_MIRROR_REG,
				 OS03A10_REG_VALUE_08BIT, &val);
		dev_dbg(&client->dev, "set vflip %d", val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops os03a10_ctrl_ops = {
	.s_ctrl = os03a10_set_ctrl,
};

static int os03a10_initialize_controls(struct os03a10 *os03a10)
{
	const struct os03a10_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &os03a10->ctrl_handler;
	mode = os03a10->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &os03a10->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_720M_10BIT, 1, PIXEL_RATE_WITH_720M_10BIT);

	h_blank = mode->hts_def - mode->width;
	os03a10->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (os03a10->hblank)
		os03a10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	os03a10->vblank = v4l2_ctrl_new_std(handler, &os03a10_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    OS03A10_VTS_MAX - mode->height,
					    1, vblank_def);
	os03a10->cur_fps = mode->max_fps;
	exposure_max = mode->vts_def - 8;
	os03a10->exposure = v4l2_ctrl_new_std(handler, &os03a10_ctrl_ops,
					      V4L2_CID_EXPOSURE, OS03A10_EXPOSURE_MIN,
					      exposure_max, OS03A10_EXPOSURE_STEP,
					      mode->exp_def);
	os03a10->anal_gain = v4l2_ctrl_new_std(handler, &os03a10_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, OS03A10_GAIN_MIN,
					       OS03A10_GAIN_MAX, OS03A10_GAIN_STEP,
					       OS03A10_GAIN_DEFAULT);
	os03a10->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
						&os03a10_ctrl_ops,
						V4L2_CID_TEST_PATTERN,
						ARRAY_SIZE(os03a10_test_pattern_menu) - 1,
						0, 0, os03a10_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &os03a10_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &os03a10_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&os03a10->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	os03a10->subdev.ctrl_handler = handler;
	os03a10->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int os03a10_check_sensor_id(struct os03a10 *os03a10,
				   struct i2c_client *client)
{
	struct device *dev = &os03a10->client->dev;
	u32 id = 0;
	int ret;

	if (os03a10->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = os03a10_read_reg(client, OS03A10_REG_CHIP_ID,
			       OS03A10_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int os03a10_configure_regulators(struct os03a10 *os03a10)
{
	unsigned int i;

	for (i = 0; i < OS03A10_NUM_SUPPLIES; i++)
		os03a10->supplies[i].supply = os03a10_supply_names[i];

	return devm_regulator_bulk_get(&os03a10->client->dev,
				       OS03A10_NUM_SUPPLIES,
				       os03a10->supplies);
}

static int os03a10_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct os03a10 *os03a10;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	os03a10 = devm_kzalloc(dev, sizeof(*os03a10), GFP_KERNEL);
	if (!os03a10)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &os03a10->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &os03a10->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &os03a10->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &os03a10->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	os03a10->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	os03a10->client = client;
	os03a10->cur_mode = &supported_modes[0];

	os03a10->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(os03a10->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	if (os03a10->is_thunderboot) {
		os03a10->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
		if (IS_ERR(os03a10->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		os03a10->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
		if (IS_ERR(os03a10->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	} else {
		os03a10->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(os03a10->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		os03a10->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
		if (IS_ERR(os03a10->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	}
	os03a10->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(os03a10->pinctrl)) {
		os03a10->pins_default =
			pinctrl_lookup_state(os03a10->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(os03a10->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		os03a10->pins_sleep =
			pinctrl_lookup_state(os03a10->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(os03a10->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = os03a10_configure_regulators(os03a10);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&os03a10->mutex);

	sd = &os03a10->subdev;
	v4l2_i2c_subdev_init(sd, client, &os03a10_subdev_ops);
	ret = os03a10_initialize_controls(os03a10);
	if (ret)
		goto err_destroy_mutex;

	ret = __os03a10_power_on(os03a10);
	if (ret)
		goto err_free_handler;

	ret = os03a10_check_sensor_id(os03a10, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &os03a10_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	os03a10->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &os03a10->pad);
	if (ret < 0)
		goto err_power_off;
#endif
	if (!os03a10->cam_sw_inf) {
		os03a10->cam_sw_inf = cam_sw_init();
		cam_sw_clk_init(os03a10->cam_sw_inf, os03a10->xvclk, OS03A10_XVCLK_FREQ);
		//cam_sw_reset_pin_init(os03a10->cam_sw_inf, os03a10->reset_gpio, 1);
		cam_sw_pwdn_pin_init(os03a10->cam_sw_inf, os03a10->reset_gpio, 1);
		cam_sw_pwdn_pin_init(os03a10->cam_sw_inf, os03a10->pwdn_gpio, 1);
	}
	memset(facing, 0, sizeof(facing));
	if (strcmp(os03a10->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 os03a10->module_index, facing,
		 OS03A10_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (os03a10->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__os03a10_power_off(os03a10);
err_free_handler:
	v4l2_ctrl_handler_free(&os03a10->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&os03a10->mutex);

	return ret;
}

static int os03a10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os03a10 *os03a10 = to_os03a10(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&os03a10->ctrl_handler);
	mutex_destroy(&os03a10->mutex);
	cam_sw_deinit(os03a10->cam_sw_inf);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__os03a10_power_off(os03a10);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id os03a10_of_match[] = {
	{ .compatible = "ovti,os03a10" },
	{},
};
MODULE_DEVICE_TABLE(of, os03a10_of_match);
#endif

static const struct i2c_device_id os03a10_match_id[] = {
	{ "ovti,os03a10", 0 },
	{ },
};

static struct i2c_driver os03a10_i2c_driver = {
	.driver = {
		.name = OS03A10_NAME,
		.pm = &os03a10_pm_ops,
		.of_match_table = of_match_ptr(os03a10_of_match),
	},
	.probe		= &os03a10_probe,
	.remove		= &os03a10_remove,
	.id_table	= os03a10_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&os03a10_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&os03a10_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision os03a10 sensor driver");
MODULE_LICENSE("GPL");
