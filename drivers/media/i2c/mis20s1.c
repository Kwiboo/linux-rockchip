// SPDX-License-Identifier: GPL-2.0
/*
 * mis20s1 driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 * V0.0X01.0X02 add support thunder boot
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

#define MIS20S1_LANES			2
#define MIS20S1_BITS_PER_SAMPLE		12
#define MIS20S1_LINK_FREQ_37125		222750000    //371250000  //37.125mbps

#define PIXEL_RATE_WITH_315M_12BIT	(MIS20S1_LINK_FREQ_37125 * 2 * \
					MIS20S1_LANES / MIS20S1_BITS_PER_SAMPLE)
#define MIS20S1_XVCLK_FREQ		27000000

#define CHIP_ID				0x20e1
#define MIS20S1_REG_CHIP_ID		0x3000

#define MIS20S1_REG_CTRL_MODE		0x3006
#define MIS20S1_MODE_SW_STANDBY		BIT(1)
#define MIS20S1_MODE_STREAMING		0x0

#define MIS20S1_REG_IMAGE_EN_MODE	0x300c
#define MIS20S1_REG_IMAGE_ABLE		BIT(0)
#define MIS20S1_REG_IMAGE_DISABLE	0x0

#define MIS20S1_REG_EXPOSURE_H		0x3100
#define MIS20S1_REG_EXPOSURE_L		0x3101
#define	MIS20S1_EXPOSURE_MIN		1
#define	MIS20S1_EXPOSURE_STEP		1
#define MIS20S1_FETCH_EXP_H(VAL)	(((VAL) >> 8) & 0xFF)
#define MIS20S1_FETCH_EXP_L(VAL)	((VAL) & 0xFF)

#define MIS20S1_REG_DIG_GAIN_H		0x420b
#define MIS20S1_REG_DIG_GAIN_L		0x420c
#define MIS20S1_REG_ANA_GAIN_H		0x3106
#define MIS20S1_REG_ANA_GAIN_L		0x3107
#define MIS20S1_ONCE_GAIN_STEP		0x32
#define MIS20S1_LCG_TO_HCG		0x5f
#define MIS20S1_GAIN_MIN		MIS20S1_ONCE_GAIN_STEP
#define MIS20S1_AGAIN_MAX		(MIS20S1_ONCE_GAIN_STEP * 32) /* just again */
#define MIS20S1_GAIN_MAX		(MIS20S1_AGAIN_MAX * 16)
#define MIS20S1_GAIN_STEP		1
#define MIS20S1_GAIN_DEFAULT		MIS20S1_ONCE_GAIN_STEP
#define MIS20S1_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define MIS20S1_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define MIS20S1_REG_GAIN_EXP_VALID	0x3008
#define MIS20S1_REG_GAIN_EXP_VALID_VAL	BIT(0)

#define MIS20S1_VTS_MAX			0xffff
#define MIS20S1_REG_VTS_H		0x310e
#define MIS20S1_REG_VTS_L		0x310f

#define MIS20S1_FLIP_MIRROR_REG		0x3007
#define MIRROR_BIT_MASK			BIT(0)
#define FLIP_BIT_MASK			BIT(1)
#define MIS20S1_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x01 : VAL & 0xfe)
#define MIS20S1_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x10 : VAL & 0xfd)

#define MIS20S1_REG_TEST_PATTERN	0x4002
#define MIS20S1_TEST_PATTERN_BIT_MASK	BIT(0)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define MIS20S1_REG_VALUE_08BIT		1
#define MIS20S1_REG_VALUE_16BIT		2
#define MIS20S1_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define MIS20S1_NAME			"mis20s1"

static const char *const mis20s1_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define MIS20S1_NUM_SUPPLIES ARRAY_SIZE(mis20s1_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct mis20s1_mode {
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

struct mis20s1 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[MIS20S1_NUM_SUPPLIES];

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
	const struct mis20s1_mode *cur_mode;
	struct v4l2_fract	cur_fps;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			is_thunderboot;
	bool			is_first_streamoff;
};

#define to_mis20s1(sd) container_of(sd, struct mis20s1, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval mis20s1_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * Input clock frequency:27M
 * Image output size:1920x1080
 * MIPI 2Lane raw12  445.5Mbps
 * HTS = 3107/3108 =0x898
 * VTS = 3105/3106 =0x465
 * Tline = 29.6us
 * NO1_PIS2308_1928x1088_VTS1125_HTS2200_VCO1782_PCLK74P25_BITCLK445p5_ACLK297_2LANE_raw12_202411111.ini
 */
static const struct regval mis20s1_linear_12_1920x1080_regs[] = {
	{0x3006, 0x01},
	{REG_DELAY, 0x05},  //delay 1ms
	{0x3006, 0x02},
	{0x3018, 0x50},
	{0x3113, 0x04},
	{0x3115, 0x3b},
	{0x3117, 0x04},
	{0x3119, 0x83},
	{0x311a, 0x01},
	{0x311b, 0xd6},
	{0x311c, 0x00},
	{0x311d, 0x19},
	{0x311e, 0x00},
	{0x311f, 0x00},
	{0x3120, 0xf8},
	{0x3121, 0x01},
	{0x3122, 0x2c},
	{0x3123, 0x12},
	{0x3205, 0xb2},
	{0x3306, 0x2f},
	{0x3307, 0x04},
	{0x3308, 0xb8},
	{0x3309, 0xb1},
	{0x330a, 0x07},
	{0x330b, 0x06},
	{0x3314, 0x25},
	{0x3502, 0x0d},
	{0x3604, 0x05},
	{0x360c, 0x05},
	{0x3612, 0x02},
	{0x3613, 0x40},
	{0x3615, 0xa0},
	{0x361c, 0x05},
	{0x3620, 0x05},
	{0x3624, 0x02},
	{0x3625, 0x4f},
	{0x3628, 0x05},
	{0x362c, 0x05},
	{0x3630, 0x02},
	{0x3631, 0x4f},
	{0x363d, 0x30},
	{0x363f, 0x40},
	{0x3641, 0xf0},
	{0x365c, 0x02},
	{0x365d, 0x40},
	{0x365e, 0x03},
	{0x365f, 0x00},
	{0x3660, 0x05},
	{0x367c, 0x02},
	{0x367d, 0x3c},
	{0x367f, 0xd5},
	{0x3680, 0x05},
	{0x3681, 0xec},
	{0x3688, 0x02},
	{0x3689, 0x4f},
	{0x368b, 0xd5},
	{0x368c, 0x05},
	{0x368f, 0xc0},
	{0x3690, 0x02},
	{0x3691, 0x51},
	{0x3692, 0x03},
	{0x3693, 0x64},
	{0x3694, 0x05},
	{0x36b3, 0x00},
	{0x36b5, 0x02},
	{0x36b6, 0x40},
	{0x36b7, 0x02},
	{0x36b8, 0xf0},
	{0x36b9, 0xe4},
	{0x36c2, 0x02},
	{0x36c3, 0x4f},
	{0x36c4, 0x02},
	{0x36c5, 0x5e},
	{0x3702, 0x05},
	{0x3704, 0x05},
	{0x3712, 0x06},
	{0x371a, 0x00},
	{0x371c, 0x00},
	{0x371d, 0x02},
	{0x371e, 0x6d},
	{0x3720, 0xa8},
	{0x3723, 0x02},
	{0x3724, 0x7c},
	{0x3725, 0x02},
	{0x3726, 0x3e},
	{0x3728, 0x9a},
	{0x3729, 0x02},
	{0x372a, 0x3b},
	{0x372b, 0x02},
	{0x372c, 0x3f},
	{0x372f, 0x05},
	{0x3741, 0x02},
	{0x3742, 0x3e},
	{0x3744, 0x9a},
	{0x375d, 0x02},
	{0x375e, 0x4f},
	{0x3760, 0xa0},
	{0x3903, 0x1b},
	{0x3905, 0x0f},
	{0x390a, 0x1f},
	{0x390b, 0x1f},
	{0x3a05, 0xfd},
	{0x3a06, 0x52},
	{0x3a08, 0x37},
	{0x3a1b, 0x7f},
	{0x3a1c, 0x3f},
	{0x3a1d, 0x0f},
	{0x3a21, 0x0e},
	{0x3b05, 0x3f},
	{0x3b07, 0x01},
	{0x4102, 0x13},
	{0x410e, 0x10},
	{0x410f, 0x39},
	{0x4303, 0x13},
	{0x432E, 0x00},
	{0x432F, 0x80},
	{0x4330, 0x00},
	{0x4331, 0x80},
	{0x4332, 0x00},
	{0x4333, 0x80},
	{0x4334, 0x00},
	{0x4335, 0x80},
	{0x4336, 0x13},
	{0x4361, 0x00},
	{0x4362, 0x80},
	{0x4363, 0x00},
	{0x4364, 0x80},
	{0x4365, 0x00},
	{0x4366, 0x80},
	{0x4367, 0x00},
	{0x4368, 0x80},
	{0x4402, 0x3f},
	{0x4403, 0x12},
	{0x3c1a, 0x00},
	{0x3c03, 0x06},
	{0x3c1a, 0x01},
	{0x3031, 0x0c},
	{0x3008, 0x01},
	{0x4607, 0x14}, //en color pattern
	{0x3006, 0x00},
	//{REG_DELAY, 0x01},  //delay 1ms
	//{0x300c, 0x01},
	{REG_NULL, 0x00},
};

static const struct mis20s1_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0052,
		.hts_def = 0x0898,
		.vts_def = 0x0465,
		.bus_fmt = MEDIA_BUS_FMT_SGRBG12_1X12,
		.reg_list = mis20s1_linear_12_1920x1080_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	MIS20S1_LINK_FREQ_37125
};

static const char *const mis20s1_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int mis20s1_write_reg(struct i2c_client *client, u16 reg,
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

static int mis20s1_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
			usleep_range(regs[i].val * 1000, regs[i].val * 1000 + 100);
			continue;
		}

		ret = mis20s1_write_reg(client, regs[i].addr,
					MIS20S1_REG_VALUE_08BIT, regs[i].val);
	}

	return ret;
}

/* Read registers up to 4 at a time */
static int mis20s1_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static void mis20s1_set_orientation_reg(struct mis20s1 *mis20s1, u32 en_flip_mir)
{
	switch (en_flip_mir) {
	case  0:
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 55000);
		mis20s1_write_reg(mis20s1->client, 0x3007, MIS20S1_REG_VALUE_08BIT, 0x00);
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x01);
		break;
	case  1:
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 55000);
		mis20s1_write_reg(mis20s1->client, 0x3007, MIS20S1_REG_VALUE_08BIT, 0x01);
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x01);
		break;
	case  2:
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 55000);
		mis20s1_write_reg(mis20s1->client, 0x3007, MIS20S1_REG_VALUE_08BIT, 0x02);
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x01);
		break;
	case  3:
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 55000);
		mis20s1_write_reg(mis20s1->client, 0x3007, MIS20S1_REG_VALUE_08BIT, 0x03);
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x01);
		break;
	default:
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 55000);
		mis20s1_write_reg(mis20s1->client, 0x3007, MIS20S1_REG_VALUE_08BIT, 0x00);
		mis20s1_write_reg(mis20s1->client, 0x300c, MIS20S1_REG_VALUE_08BIT, 0x01);
		break;
	}
}

static int mis20s1_set_gain_reg(struct mis20s1 *mis20s1, u32 gain)
{
	u32 coarse_again = 0;
	u32 dgain = 0x100;
	int ret = 0;

	//struct device *dev = &mis20s1->client->dev;
	if (gain > MIS20S1_GAIN_MAX - 1)
		gain = MIS20S1_GAIN_MAX - 1;

	if (gain < MIS20S1_LCG_TO_HCG * MIS20S1_ONCE_GAIN_STEP / 10) {
		coarse_again = 1024 - (1024 * MIS20S1_ONCE_GAIN_STEP / gain);
		mis20s1_write_reg(mis20s1->client, 0x310c, MIS20S1_REG_VALUE_08BIT, 0x00);
	} else if (gain <= MIS20S1_AGAIN_MAX) {
		coarse_again = 1024 - (1024 * MIS20S1_ONCE_GAIN_STEP * MIS20S1_LCG_TO_HCG) / gain / 10;
		mis20s1_write_reg(mis20s1->client, 0x310c, MIS20S1_REG_VALUE_08BIT, 0x01);
	} else {
		coarse_again = MIS20S1_AGAIN_MAX;
		dgain = (gain * 256  / MIS20S1_AGAIN_MAX);
	}
	dev_info(&(mis20s1->client->dev), "coarse_again = %d dgain = %d\n ",
		 coarse_again, dgain);

	ret |= mis20s1_write_reg(mis20s1->client,
				 MIS20S1_REG_DIG_GAIN_H,
				 MIS20S1_REG_VALUE_08BIT,
				 dgain >> 8 & 0xff);
	ret |= mis20s1_write_reg(mis20s1->client,
				 MIS20S1_REG_DIG_GAIN_L,
				 MIS20S1_REG_VALUE_08BIT,
				 dgain & 0xff);

	ret |= mis20s1_write_reg(mis20s1->client,
				 MIS20S1_REG_ANA_GAIN_H,
				 MIS20S1_REG_VALUE_08BIT,
				 MIS20S1_FETCH_AGAIN_H(coarse_again));
	ret |= mis20s1_write_reg(mis20s1->client,
				 MIS20S1_REG_ANA_GAIN_L,
				 MIS20S1_REG_VALUE_08BIT,
				 MIS20S1_FETCH_AGAIN_L(coarse_again));
	ret |= mis20s1_write_reg(mis20s1->client,
				 MIS20S1_REG_GAIN_EXP_VALID,
				 MIS20S1_REG_VALUE_08BIT,
				 MIS20S1_REG_GAIN_EXP_VALID_VAL);

	return ret;
}

static int mis20s1_get_reso_dist(const struct mis20s1_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct mis20s1_mode *
mis20s1_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = mis20s1_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int mis20s1_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	const struct mis20s1_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&mis20s1->mutex);

	mode = mis20s1_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&mis20s1->mutex);
		return -ENOTTY;
#endif
	} else {
		mis20s1->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(mis20s1->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(mis20s1->vblank, vblank_def,
					 MIS20S1_VTS_MAX - mode->height,
					 1, vblank_def);
		mis20s1->cur_fps = mode->max_fps;
	}

	mutex_unlock(&mis20s1->mutex);

	return 0;
}

static int mis20s1_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	const struct mis20s1_mode *mode = mis20s1->cur_mode;

	mutex_lock(&mis20s1->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&mis20s1->mutex);
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
	mutex_unlock(&mis20s1->mutex);

	return 0;
}

static int mis20s1_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = mis20s1->cur_mode->bus_fmt;

	return 0;
}

static int mis20s1_enum_frame_sizes(struct v4l2_subdev *sd,
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


static int mis20s1_enable_test_pattern(struct mis20s1 *mis20s1, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = mis20s1_read_reg(mis20s1->client, MIS20S1_REG_TEST_PATTERN,
			       MIS20S1_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= MIS20S1_TEST_PATTERN_BIT_MASK;
	else
		val &= ~MIS20S1_TEST_PATTERN_BIT_MASK;

	ret |= mis20s1_write_reg(mis20s1->client, MIS20S1_REG_TEST_PATTERN,
				 MIS20S1_REG_VALUE_08BIT, val);

	return ret;
}

static int mis20s1_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	const struct mis20s1_mode *mode = mis20s1->cur_mode;

	if (mis20s1->streaming)
		fi->interval = mis20s1->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int mis20s1_g_mbus_config(struct v4l2_subdev *sd,
				 unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	const struct mis20s1_mode *mode = mis20s1->cur_mode;
	u32 val = 1 << (MIS20S1_LANES - 1) |
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

static void mis20s1_get_module_inf(struct mis20s1 *mis20s1,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, MIS20S1_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, mis20s1->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, mis20s1->len_name, sizeof(inf->base.lens));
}

static long mis20s1_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		mis20s1_get_module_inf(mis20s1, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = mis20s1->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = mis20s1->cur_mode->width;
		h = mis20s1->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				mis20s1->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&mis20s1->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = mis20s1->cur_mode->hts_def - mis20s1->cur_mode->width;
			h = mis20s1->cur_mode->vts_def - mis20s1->cur_mode->height;
			__v4l2_ctrl_modify_range(mis20s1->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(mis20s1->vblank, h,
						 MIS20S1_VTS_MAX - mis20s1->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = mis20s1_write_reg(mis20s1->client, MIS20S1_REG_IMAGE_EN_MODE,
						MIS20S1_REG_VALUE_08BIT, MIS20S1_REG_IMAGE_ABLE);
		else
			ret = mis20s1_write_reg(mis20s1->client, MIS20S1_REG_IMAGE_EN_MODE,
						MIS20S1_REG_VALUE_08BIT, MIS20S1_REG_IMAGE_DISABLE);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long mis20s1_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = mis20s1_ioctl(sd, cmd, inf);
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

		ret = mis20s1_ioctl(sd, cmd, hdr);
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
			ret = mis20s1_ioctl(sd, cmd, hdr);
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
			ret = mis20s1_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = mis20s1_ioctl(sd, cmd, &stream);
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

static int __mis20s1_start_stream(struct mis20s1 *mis20s1)
{
	int ret;

	if (!mis20s1->is_thunderboot) {
		ret = mis20s1_write_array(mis20s1->client, mis20s1->cur_mode->reg_list);
		if (ret)
			return ret;
		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&mis20s1->ctrl_handler);
		if (ret)
			return ret;

		ret = mis20s1_write_reg(mis20s1->client, MIS20S1_REG_CTRL_MODE,
				 MIS20S1_REG_VALUE_08BIT, MIS20S1_MODE_STREAMING);
		if (ret)
			return ret;
		usleep_range(1000, 2000);
	}
	ret = mis20s1_write_reg(mis20s1->client, MIS20S1_REG_IMAGE_EN_MODE,
				 MIS20S1_REG_VALUE_08BIT, MIS20S1_REG_IMAGE_ABLE);
	return ret;
}

static int __mis20s1_stop_stream(struct mis20s1 *mis20s1)
{
	int ret;

	if (mis20s1->is_thunderboot) {
		mis20s1->is_first_streamoff = true;
		pm_runtime_put(&mis20s1->client->dev);
	}
	ret = mis20s1_write_reg(mis20s1->client, MIS20S1_REG_CTRL_MODE,
				 MIS20S1_REG_VALUE_08BIT, MIS20S1_MODE_SW_STANDBY);
	ret |= mis20s1_write_reg(mis20s1->client, MIS20S1_REG_IMAGE_EN_MODE,
				 MIS20S1_REG_VALUE_08BIT, MIS20S1_REG_IMAGE_DISABLE);

	return ret;
}

static int __mis20s1_power_on(struct mis20s1 *mis20s1);
static int mis20s1_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	struct i2c_client *client = mis20s1->client;
	int ret = 0;

	mutex_lock(&mis20s1->mutex);
	on = !!on;
	if (on == mis20s1->streaming)
		goto unlock_and_return;

	if (on) {
		if (mis20s1->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			mis20s1->is_thunderboot = false;
			__mis20s1_power_on(mis20s1);
		}

		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __mis20s1_start_stream(mis20s1);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			ret = -EIO;
			goto unlock_and_return;
		}
	} else {
		__mis20s1_stop_stream(mis20s1);
		pm_runtime_put(&client->dev);
	}

	mis20s1->streaming = on;

unlock_and_return:
	mutex_unlock(&mis20s1->mutex);

	return ret;
}

static int mis20s1_s_power(struct v4l2_subdev *sd, int on)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	struct i2c_client *client = mis20s1->client;
	int ret = 0;

	mutex_lock(&mis20s1->mutex);

	/* If the power state is not modified - no work to do. */
	if (mis20s1->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!mis20s1->is_thunderboot) {
			ret = mis20s1_write_array(mis20s1->client, mis20s1_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		mis20s1->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		mis20s1->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&mis20s1->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 mis20s1_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, MIS20S1_XVCLK_FREQ / 1000 / 1000);
}

static int __mis20s1_power_on(struct mis20s1 *mis20s1)
{
	int ret;
	u32 delay_us;
	struct device *dev = &mis20s1->client->dev;

	if (!IS_ERR_OR_NULL(mis20s1->pins_default)) {
		ret = pinctrl_select_state(mis20s1->pinctrl,
					   mis20s1->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(mis20s1->xvclk, MIS20S1_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (27MHz)\n");
	if (clk_get_rate(mis20s1->xvclk) != MIS20S1_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 27MHz\n");
	ret = clk_prepare_enable(mis20s1->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (mis20s1->is_thunderboot)
		return 0;

	if (!IS_ERR(mis20s1->reset_gpio))
		gpiod_set_value_cansleep(mis20s1->reset_gpio, 0);

	ret = regulator_bulk_enable(MIS20S1_NUM_SUPPLIES, mis20s1->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(mis20s1->reset_gpio))
		gpiod_set_value_cansleep(mis20s1->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(mis20s1->pwdn_gpio))
		gpiod_set_value_cansleep(mis20s1->pwdn_gpio, 1);

	if (!IS_ERR(mis20s1->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = mis20s1_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

disable_clk:
	clk_disable_unprepare(mis20s1->xvclk);

	return ret;
}

static void __mis20s1_power_off(struct mis20s1 *mis20s1)
{
	int ret;
	struct device *dev = &mis20s1->client->dev;

	clk_disable_unprepare(mis20s1->xvclk);
	if (mis20s1->is_thunderboot) {
		if (mis20s1->is_first_streamoff) {
			mis20s1->is_thunderboot = false;
			mis20s1->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(mis20s1->pwdn_gpio))
		gpiod_set_value_cansleep(mis20s1->pwdn_gpio, 0);
	if (!IS_ERR(mis20s1->reset_gpio))
		gpiod_set_value_cansleep(mis20s1->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(mis20s1->pins_sleep)) {
		ret = pinctrl_select_state(mis20s1->pinctrl,
					   mis20s1->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(MIS20S1_NUM_SUPPLIES, mis20s1->supplies);
}

static int __maybe_unused mis20s1_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis20s1 *mis20s1 = to_mis20s1(sd);

	return __mis20s1_power_on(mis20s1);
}

static int __maybe_unused mis20s1_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis20s1 *mis20s1 = to_mis20s1(sd);

	__mis20s1_power_off(mis20s1);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int mis20s1_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mis20s1 *mis20s1 = to_mis20s1(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct mis20s1_mode *def_mode = &supported_modes[0];

	mutex_lock(&mis20s1->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&mis20s1->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int mis20s1_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops mis20s1_pm_ops = {
	SET_RUNTIME_PM_OPS(mis20s1_runtime_suspend,
	mis20s1_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops mis20s1_internal_ops = {
	.open = mis20s1_open,
};
#endif

static const struct v4l2_subdev_core_ops mis20s1_core_ops = {
	.s_power = mis20s1_s_power,
	.ioctl = mis20s1_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = mis20s1_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops mis20s1_video_ops = {
	.s_stream = mis20s1_s_stream,
	.g_frame_interval = mis20s1_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops mis20s1_pad_ops = {
	.enum_mbus_code = mis20s1_enum_mbus_code,
	.enum_frame_size = mis20s1_enum_frame_sizes,
	.enum_frame_interval = mis20s1_enum_frame_interval,
	.get_fmt = mis20s1_get_fmt,
	.set_fmt = mis20s1_set_fmt,
	.get_mbus_config = mis20s1_g_mbus_config,
};

static const struct v4l2_subdev_ops mis20s1_subdev_ops = {
	.core = &mis20s1_core_ops,
	.video = &mis20s1_video_ops,
	.pad = &mis20s1_pad_ops,
};

static void mis20s1_modify_fps_info(struct mis20s1 *mis20s1)
{
	const struct mis20s1_mode *mode = mis20s1->cur_mode;

	mis20s1->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       mis20s1->cur_vts;
}

static int mis20s1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mis20s1 *mis20s1 = container_of(ctrl->handler,
					       struct mis20s1, ctrl_handler);
	struct i2c_client *client = mis20s1->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = mis20s1->cur_mode->height + ctrl->val - 1;
		__v4l2_ctrl_modify_range(mis20s1->exposure,
					 mis20s1->exposure->minimum, max,
					 mis20s1->exposure->step,
					 mis20s1->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_info(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (mis20s1->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val;

			/* 4 least significant bits of expsoure are fractional part */
			ret = mis20s1_write_reg(mis20s1->client,
						MIS20S1_REG_EXPOSURE_H,
						MIS20S1_REG_VALUE_08BIT,
						MIS20S1_FETCH_EXP_H(val));
			ret |= mis20s1_write_reg(mis20s1->client,
						 MIS20S1_REG_EXPOSURE_L,
						 MIS20S1_REG_VALUE_08BIT,
						 MIS20S1_FETCH_EXP_L(val));
			ret |= mis20s1_write_reg(mis20s1->client,
						 MIS20S1_REG_GAIN_EXP_VALID,
						 MIS20S1_REG_VALUE_08BIT,
						 MIS20S1_REG_GAIN_EXP_VALID_VAL);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (mis20s1->cur_mode->hdr_mode == NO_HDR)
			ret = mis20s1_set_gain_reg(mis20s1, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = mis20s1_write_reg(mis20s1->client,
					MIS20S1_REG_VTS_H,
					MIS20S1_REG_VALUE_08BIT,
					(ctrl->val + mis20s1->cur_mode->height)
					>> 8);
		ret |= mis20s1_write_reg(mis20s1->client,
					 MIS20S1_REG_VTS_L,
					 MIS20S1_REG_VALUE_08BIT,
					 (ctrl->val + mis20s1->cur_mode->height)
					 & 0xff);
		mis20s1->cur_vts = ctrl->val + mis20s1->cur_mode->height;
		if (mis20s1->cur_vts != mis20s1->cur_mode->vts_def)
			mis20s1_modify_fps_info(mis20s1);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = mis20s1_enable_test_pattern(mis20s1, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = mis20s1_read_reg(mis20s1->client, MIS20S1_FLIP_MIRROR_REG,
				       MIS20S1_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val |= MIRROR_BIT_MASK;
		else
			val &= ~MIRROR_BIT_MASK;
		mis20s1_set_orientation_reg(mis20s1, val);
		break;
	case V4L2_CID_VFLIP:
		ret = mis20s1_read_reg(mis20s1->client, MIS20S1_FLIP_MIRROR_REG,
				       MIS20S1_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val |= FLIP_BIT_MASK;
		else
			val &= ~FLIP_BIT_MASK;
		mis20s1_set_orientation_reg(mis20s1, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops mis20s1_ctrl_ops = {
	.s_ctrl = mis20s1_set_ctrl,
};

static int mis20s1_initialize_controls(struct mis20s1 *mis20s1)
{
	const struct mis20s1_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &mis20s1->ctrl_handler;
	mode = mis20s1->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &mis20s1->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_315M_12BIT, 1, PIXEL_RATE_WITH_315M_12BIT);

	h_blank = mode->hts_def - mode->width;
	mis20s1->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (mis20s1->hblank)
		mis20s1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	mis20s1->vblank = v4l2_ctrl_new_std(handler, &mis20s1_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    MIS20S1_VTS_MAX - mode->height,
					    1, vblank_def);
	mis20s1->cur_fps = mode->max_fps;
	exposure_max = mode->vts_def - 1;
	mis20s1->exposure = v4l2_ctrl_new_std(handler, &mis20s1_ctrl_ops,
					      V4L2_CID_EXPOSURE, MIS20S1_EXPOSURE_MIN,
					      exposure_max, MIS20S1_EXPOSURE_STEP,
					      mode->exp_def);
	mis20s1->anal_gain = v4l2_ctrl_new_std(handler, &mis20s1_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, MIS20S1_GAIN_MIN,
					       MIS20S1_GAIN_MAX, MIS20S1_GAIN_STEP,
					       MIS20S1_GAIN_DEFAULT);
	mis20s1->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&mis20s1_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(mis20s1_test_pattern_menu) - 1,
				0, 0, mis20s1_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &mis20s1_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &mis20s1_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&mis20s1->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	mis20s1->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int mis20s1_check_sensor_id(struct mis20s1 *mis20s1,
				   struct i2c_client *client)
{
	struct device *dev = &mis20s1->client->dev;
	u32 id = 0;
	int ret;

	if (mis20s1->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = mis20s1_read_reg(client, MIS20S1_REG_CHIP_ID,
			       MIS20S1_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected mis%04x sensor\n", CHIP_ID);

	return 0;
}

static int mis20s1_configure_regulators(struct mis20s1 *mis20s1)
{
	unsigned int i;

	for (i = 0; i < MIS20S1_NUM_SUPPLIES; i++)
		mis20s1->supplies[i].supply = mis20s1_supply_names[i];

	return devm_regulator_bulk_get(&mis20s1->client->dev,
				       MIS20S1_NUM_SUPPLIES,
				       mis20s1->supplies);
}

static int mis20s1_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct mis20s1 *mis20s1;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	mis20s1 = devm_kzalloc(dev, sizeof(*mis20s1), GFP_KERNEL);
	if (!mis20s1)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &mis20s1->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &mis20s1->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &mis20s1->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &mis20s1->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	mis20s1->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	mis20s1->client = client;
	mis20s1->cur_mode = &supported_modes[0];

	mis20s1->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(mis20s1->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	if (mis20s1->is_thunderboot) {
		mis20s1->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
		if (IS_ERR(mis20s1->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		mis20s1->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
		if (IS_ERR(mis20s1->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	} else {
		mis20s1->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(mis20s1->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		mis20s1->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
		if (IS_ERR(mis20s1->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	}
	mis20s1->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(mis20s1->pinctrl)) {
		mis20s1->pins_default =
			pinctrl_lookup_state(mis20s1->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(mis20s1->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		mis20s1->pins_sleep =
			pinctrl_lookup_state(mis20s1->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(mis20s1->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = mis20s1_configure_regulators(mis20s1);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&mis20s1->mutex);

	sd = &mis20s1->subdev;
	v4l2_i2c_subdev_init(sd, client, &mis20s1_subdev_ops);
	ret = mis20s1_initialize_controls(mis20s1);
	if (ret)
		goto err_destroy_mutex;

	ret = __mis20s1_power_on(mis20s1);
	if (ret)
		goto err_free_handler;

	ret = mis20s1_check_sensor_id(mis20s1, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &mis20s1_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	mis20s1->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &mis20s1->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(mis20s1->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 mis20s1->module_index, facing,
		 MIS20S1_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (mis20s1->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__mis20s1_power_off(mis20s1);
err_free_handler:
	v4l2_ctrl_handler_free(&mis20s1->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&mis20s1->mutex);

	return ret;
}

static int mis20s1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis20s1 *mis20s1 = to_mis20s1(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&mis20s1->ctrl_handler);
	mutex_destroy(&mis20s1->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__mis20s1_power_off(mis20s1);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mis20s1_of_match[] = {
	{ .compatible = "imagedesign,mis20s1" },
	{},
};
MODULE_DEVICE_TABLE(of, mis20s1_of_match);
#endif

static const struct i2c_device_id mis20s1_match_id[] = {
	{ "imagedesign,mis20s1", 0 },
	{ },
};

static struct i2c_driver mis20s1_i2c_driver = {
	.driver = {
		.name = MIS20S1_NAME,
		.pm = &mis20s1_pm_ops,
		.of_match_table = of_match_ptr(mis20s1_of_match),
	},
	.probe = &mis20s1_probe,
	.remove = &mis20s1_remove,
	.id_table = mis20s1_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&mis20s1_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&mis20s1_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("imagedesign mis20s1 sensor driver");
MODULE_LICENSE("GPL");
