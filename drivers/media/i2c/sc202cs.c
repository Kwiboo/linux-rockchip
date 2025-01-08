// SPDX-License-Identifier: GPL-2.0
/*
 * sc202cs driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add first implement.
 * V0.0X02.0X02 add support thunder boot
 * V0.0X03.0X03 add support wake up/sleep mode
 *
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
#include "cam-tb-setup.h"
#include "cam-sleep-wakeup.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SC202CS_LANES			1
#define SC202CS_BITS_PER_SAMPLE		10
#define SC202CS_LINK_FREQ		360000000// 720Mbps

#define PIXEL_RATE_WITH_360M_10BIT	(SC202CS_LINK_FREQ * 2 * \
					SC202CS_LANES / SC202CS_BITS_PER_SAMPLE)

#define SC202CS_XVCLK_FREQ		24000000

#define CHIP_ID				0xeb52
#define SC202CS_REG_CHIP_ID		0x3107

#define SC202CS_REG_CTRL_MODE		0x0100
#define SC202CS_MODE_SW_STANDBY		0x0
#define SC202CS_MODE_STREAMING		BIT(0)

#define SC202CS_REG_MIPI_CTRL		0x3019
#define SC202CS_MIPI_CTRL_ON		0x0c
#define SC202CS_MIPI_CTRL_OFF		0x0f

#define SC202CS_REG_EXPOSURE_H		0x3e00
#define SC202CS_REG_EXPOSURE_M		0x3e01
#define SC202CS_REG_EXPOSURE_L		0x3e02
#define	SC202CS_EXPOSURE_MIN		1
#define	SC202CS_EXPOSURE_STEP		1
#define SC202CS_VTS_MAX			0x7fff

#define SC202CS_REG_ENABLE_AGC		0X3e03	//Need set 16'h3e03 Bit[3:0] to 4'hb for AGC control
#define SC202CS_REG_DIG_GAIN		0x3e06
#define SC202CS_REG_DIG_FINE_GAIN	0x3e07
#define SC202CS_REG_ANA_GAIN		0x3e09

#define SC202CS_GAIN_MIN		0x0040
#define SC202CS_GAIN_MAX		(16 * 4 * 128)	//TotalGain*128=16*4*128
#define SC202CS_GAIN_STEP		1
#define SC202CS_GAIN_DEFAULT		0x80	// Total_gain = 1x

#define SC202CS_REG_HIGH_TEMP_H		0x3974
#define SC202CS_REG_HIGH_TEMP_L		0x3975

#define SC202CS_REG_TEST_PATTERN	0x4501
#define SC202CS_TEST_PATTERN_BIT_MASK	BIT(3)

#define SC202CS_REG_VTS_H		0x320e
#define SC202CS_REG_VTS_L		0x320f

#define SC202CS_FLIP_MIRROR_REG		0x3221

#define SC202CS_FETCH_EXP_H(VAL)	(((VAL) >> 12) & 0xF)
#define SC202CS_FETCH_EXP_M(VAL)	(((VAL) >> 4) & 0xFF)
#define SC202CS_FETCH_EXP_L(VAL)	(((VAL) & 0xF) << 4)

#define SC202CS_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define SC202CS_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define SC202CS_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x06 : VAL & 0xf9)
#define SC202CS_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x60 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define SC202CS_REG_VALUE_08BIT		1
#define SC202CS_REG_VALUE_16BIT		2
#define SC202CS_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define SC202CS_NAME			"sc202cs"

static const char *const sc202cs_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC202CS_NUM_SUPPLIES ARRAY_SIZE(sc202cs_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc202cs_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 mipi_freq_idx;
	u32 bpp;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct sc202cs {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC202CS_NUM_SUPPLIES];

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
	const struct sc202cs_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			old_gain;
	u32			standby_hw;
	u32			cur_vts;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	bool			is_standby;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	struct cam_sw_info	*cam_sw_inf;
};

#define to_sc202cs(sd) container_of(sd, struct sc202cs, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval sc202cs_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 720Mbps, 1lane
 * cleaned_0x01_FT_SC202CS_24Minput_720Mbps_1lane_10bit_1600x1200_30fps.ini
 */
static const struct regval sc202cs_linear_10_1600x1200_30fps_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36e9, 0x24},
	{0x301f, 0x01},
	{0x3301, 0xff},
	{0x3304, 0x68},
	{0x3306, 0x40},
	{0x3308, 0x08},
	{0x3309, 0xa8},
	{0x330b, 0xb0},
	{0x330c, 0x18},
	{0x330d, 0xff},
	{0x330e, 0x20},
	{0x331e, 0x59},
	{0x331f, 0x99},
	{0x3333, 0x10},
	{0x335e, 0x06},
	{0x335f, 0x08},
	{0x3364, 0x1f},
	{0x337c, 0x02},
	{0x337d, 0x0a},
	{0x338f, 0xa0},
	{0x3390, 0x01},
	{0x3391, 0x03},
	{0x3392, 0x1f},
	{0x3393, 0xff},
	{0x3394, 0xff},
	{0x3395, 0xff},
	{0x33a2, 0x04},
	{0x33ad, 0x0c},
	{0x33b1, 0x20},
	{0x33b3, 0x38},
	{0x33f9, 0x40},
	{0x33fb, 0x48},
	{0x33fc, 0x0f},
	{0x33fd, 0x1f},
	{0x349f, 0x03},
	{0x34a6, 0x03},
	{0x34a7, 0x1f},
	{0x34a8, 0x38},
	{0x34a9, 0x30},
	{0x34ab, 0xb0},
	{0x34ad, 0xb0},
	{0x34f8, 0x1f},
	{0x34f9, 0x20},
	{0x3630, 0xa0},
	{0x3631, 0x92},
	{0x3632, 0x64},
	{0x3633, 0x43},
	{0x3637, 0x49},
	{0x363a, 0x85},
	{0x363c, 0x0f},
	{0x3650, 0x31},
	{0x3670, 0x0d},
	{0x3674, 0xc0},
	{0x3675, 0xa0},
	{0x3676, 0xa0},
	{0x3677, 0x92},
	{0x3678, 0x96},
	{0x3679, 0x9a},
	{0x367c, 0x03},
	{0x367d, 0x0f},
	{0x367e, 0x01},
	{0x367f, 0x0f},
	{0x3698, 0x83},
	{0x3699, 0x86},
	{0x369a, 0x8c},
	{0x369b, 0x94},
	{0x36a2, 0x01},
	{0x36a3, 0x03},
	{0x36a4, 0x07},
	{0x36ae, 0x0f},
	{0x36af, 0x1f},
	{0x36bd, 0x22},
	{0x36be, 0x22},
	{0x36bf, 0x22},
	{0x36d0, 0x01},
	{0x370f, 0x02},
	{0x3721, 0x6c},
	{0x3722, 0x8d},
	{0x3725, 0xc5},
	{0x3727, 0x14},
	{0x3728, 0x04},
	{0x37b7, 0x04},
	{0x37b8, 0x04},
	{0x37b9, 0x06},
	{0x37bd, 0x07},
	{0x37be, 0x0f},
	{0x3901, 0x02},
	{0x3903, 0x40},
	{0x3905, 0x8d},
	{0x3907, 0x00},
	{0x3908, 0x41},
	{0x391f, 0x41},
	{0x3933, 0x80},
	{0x3934, 0x02},
	{0x3937, 0x6f},
	{0x393a, 0x01},
	{0x393d, 0x01},
	{0x393e, 0xc0},
	{0x39dd, 0x41},
	{0x3e00, 0x00},
	{0x3e01, 0x4d},
	{0x3e02, 0xc0},
	{0x3e09, 0x00},
	{0x4509, 0x28},
	{0x450d, 0x61},
	{REG_DELAY, 0x0a},  //delay 10 ms
	{REG_NULL, 0x00},
};

static const struct sc202cs_mode supported_modes[] = {
	{
		.width = 1600,
		.height = 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x4dc,
		.hts_def = 0x0672,  //1650
		.vts_def = 0x04e2,  //1250
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = sc202cs_linear_10_1600x1200_30fps_regs,
		.hdr_mode = NO_HDR,
		.bpp = 10,
		.mipi_freq_idx = 0,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const s64 link_freq_menu_items[] = {
	SC202CS_LINK_FREQ,
};

static const char *const sc202cs_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int sc202cs_write_reg(struct i2c_client *client, u16 reg,
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

static int sc202cs_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
			dev_info(&client->dev, "delay %d ms\n", regs[i].val);
			usleep_range(regs[i].val * 1000, regs[i].val * 1000 + 1000);
			continue;
		}
		ret = sc202cs_write_reg(client, regs[i].addr,
					SC202CS_REG_VALUE_08BIT, regs[i].val);
	}

	return ret;
}

/* Read registers up to 4 at a time */
static int sc202cs_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

/* mode: 0 = lgain  1 = sgain */
static int sc202cs_set_gain_reg(struct sc202cs *sc202cs, u32 gain)
{
	int total_gain = 0;
	u32 coarse_again_reg, dig_fine_reg, dgain_reg;
	struct device *dev = &sc202cs->client->dev;
	int ret = 0;

	if (gain == sc202cs->old_gain)
		return 0;

	dev_info(dev, "=>set gain[%d]\n", gain);

	//How to convert a_gain to reg  value
	// Total_gain = again * dgain
	// isp converted reg value = Total_gain * 128

	total_gain = gain >> 7;   //div 128

	if (total_gain <= 1)
		total_gain = 1;
	else if (total_gain > 64)	// 64 = 16 * 4 = 64
		total_gain = 64;

	if (total_gain < 2) {		/* start again 1.0x --2.0x */
		coarse_again_reg = 0x00;
		dgain_reg = 0x00;
		dig_fine_reg =  gain & 0xFF;
	} else if (gain < 4) {		/* 2.0x ~ 4x*/
		coarse_again_reg = 0x01;
		dgain_reg = 0x00;
		dig_fine_reg = (gain >> 1) & 0xFF;
	} else if (gain < 8) {		/* 4x ~ 8x */
		coarse_again_reg = 0x03;
		dgain_reg = 0x00;
		dig_fine_reg = (gain >> 2) & 0xFF;
	} else if (gain < 16) {		/* 8x ~ 16x */
		coarse_again_reg = 0x07;
		dgain_reg = 0x00;
		dig_fine_reg = (gain >> 3) & 0xFF;
	} else if (gain < 32) {		/* 16x ~ 32x */
		coarse_again_reg = 0x0f;
		dgain_reg = 0x00;
		dig_fine_reg = (gain >> 4) & 0xFF;
	} else {			/* 32x ~ 64x */
		coarse_again_reg = 0x0f;
		dgain_reg = 0x01;
		dig_fine_reg = (gain >> 5) & 0xFF;
	}

	dev_info(dev, "=>set Totalgain[%dx],again[0x%02x],dfine_gain[0x%02x],dgain[0x%02x]\n",
		 total_gain, coarse_again_reg, dig_fine_reg, dgain_reg);

	ret = sc202cs_write_reg(sc202cs->client,
				SC202CS_REG_DIG_GAIN,
				SC202CS_REG_VALUE_08BIT,
				dgain_reg);
	ret |= sc202cs_write_reg(sc202cs->client,
				 SC202CS_REG_DIG_FINE_GAIN,
				 SC202CS_REG_VALUE_08BIT,
				 dig_fine_reg);
	ret |= sc202cs_write_reg(sc202cs->client,
				 SC202CS_REG_ANA_GAIN,
				 SC202CS_REG_VALUE_08BIT,
				 coarse_again_reg);

	sc202cs->old_gain = gain;
	return ret;
}

static int sc202cs_get_reso_dist(const struct sc202cs_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc202cs_mode *
sc202cs_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sc202cs_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		} else if (dist == cur_best_fit_dist &&
			   framefmt->code == supported_modes[i].bus_fmt) {
			cur_best_fit = i;
			break;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sc202cs_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	const struct sc202cs_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&sc202cs->mutex);

	mode = sc202cs_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc202cs->mutex);
		return -ENOTTY;
#endif
	} else {
		sc202cs->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc202cs->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc202cs->vblank, vblank_def,
					 SC202CS_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(sc202cs->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_menu_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * SC202CS_LANES;
		__v4l2_ctrl_s_ctrl_int64(sc202cs->pixel_rate, pixel_rate);
		sc202cs->cur_fps = mode->max_fps;
		sc202cs->cur_vts = mode->vts_def;
	}

	mutex_unlock(&sc202cs->mutex);

	return 0;
}

static int sc202cs_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	const struct sc202cs_mode *mode = sc202cs->cur_mode;

	mutex_lock(&sc202cs->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc202cs->mutex);
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
	mutex_unlock(&sc202cs->mutex);

	return 0;
}

static int sc202cs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int sc202cs_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int sc202cs_enable_test_pattern(struct sc202cs *sc202cs, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc202cs_read_reg(sc202cs->client, SC202CS_REG_TEST_PATTERN,
			       SC202CS_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC202CS_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SC202CS_TEST_PATTERN_BIT_MASK;

	ret |= sc202cs_write_reg(sc202cs->client, SC202CS_REG_TEST_PATTERN,
				 SC202CS_REG_VALUE_08BIT, val);
	return ret;
}

static int sc202cs_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	const struct sc202cs_mode *mode = sc202cs->cur_mode;

	if (sc202cs->streaming)
		fi->interval = sc202cs->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct sc202cs_mode *sc202cs_find_mode(struct sc202cs *sc202cs, int fps)
{
	const struct sc202cs_mode *mode = NULL;
	const struct sc202cs_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == sc202cs->cur_mode->width &&
		    mode->height == sc202cs->cur_mode->height &&
		    mode->hdr_mode == sc202cs->cur_mode->hdr_mode &&
		    mode->bus_fmt == sc202cs->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int sc202cs_s_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	const struct sc202cs_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	int fps;

	if (sc202cs->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = sc202cs_find_mode(sc202cs, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	sc202cs->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(sc202cs->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(sc202cs->vblank, vblank_def,
				 SC202CS_VTS_MAX - mode->height,
				 1, vblank_def);
	sc202cs->cur_fps = mode->max_fps;
	return 0;
}

static int sc202cs_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	u32 val = 1 << (SC202CS_LANES - 1) |
		  V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void sc202cs_get_module_inf(struct sc202cs *sc202cs,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC202CS_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc202cs->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc202cs->len_name, sizeof(inf->base.lens));
}

static int sc202cs_get_channel_info(struct sc202cs *sc202cs, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = sc202cs->cur_mode->vc[ch_info->index];
	ch_info->width = sc202cs->cur_mode->width;
	ch_info->height = sc202cs->cur_mode->height;
	ch_info->bus_fmt = sc202cs->cur_mode->bus_fmt;
	return 0;
}

static long sc202cs_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc202cs_get_module_inf(sc202cs, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc202cs->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		if (hdr->hdr_mode == sc202cs->cur_mode->hdr_mode)
			return 0;
		if (hdr->hdr_mode != NO_HDR) {
			dev_err(&sc202cs->client->dev, "NOT hdr supported\n");
			ret = -EINVAL;
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (sc202cs->standby_hw) { // hardware standby
			if (stream) {
				if (!IS_ERR(sc202cs->pwdn_gpio))
					gpiod_set_value_cansleep(sc202cs->pwdn_gpio, 1);
				// Make sure __v4l2_ctrl_handler_setup can be called correctly
				sc202cs->is_standby = false;

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
				if (__v4l2_ctrl_handler_setup(&sc202cs->ctrl_handler))
					dev_err(&sc202cs->client->dev, "__v4l2_ctrl_handler_setup fail!");
				if (sc202cs->cur_mode->hdr_mode != NO_HDR) {
					if (sc202cs->cam_sw_inf) {
						ret = sc202cs_ioctl(&sc202cs->subdev,
								    PREISP_CMD_SET_HDRAE_EXP,
								    &sc202cs->cam_sw_inf->hdr_ae);
						if (ret) {
							dev_err(&sc202cs->client->dev,
								"init exp fail in hdr mode\n");
							return ret;
						}
					}
				}
#endif

				ret |= sc202cs_write_reg(sc202cs->client, SC202CS_REG_CTRL_MODE,
							 SC202CS_REG_VALUE_08BIT,
							 SC202CS_MODE_STREAMING);

				dev_info(&sc202cs->client->dev, "quickstream, streaming on: exit hw standby mode\n");
			} else {
				ret = sc202cs_write_reg(sc202cs->client, SC202CS_REG_CTRL_MODE,
							SC202CS_REG_VALUE_08BIT,
							SC202CS_MODE_SW_STANDBY);

				if (!IS_ERR(sc202cs->pwdn_gpio))
					gpiod_set_value_cansleep(sc202cs->pwdn_gpio, 0);

				dev_info(&sc202cs->client->dev, "quickstream, streaming off: enter hw standby mode\n");
				sc202cs->is_standby = true;
			}
		} else {	// software standby
			if (stream) {
				ret |= sc202cs_write_reg(sc202cs->client, SC202CS_REG_CTRL_MODE,
							 SC202CS_REG_VALUE_08BIT,
							 SC202CS_MODE_STREAMING);
				dev_info(&sc202cs->client->dev, "quickstream, streaming on: exit soft standby mode\n");
			} else {
				ret = sc202cs_write_reg(sc202cs->client, SC202CS_REG_CTRL_MODE,
							SC202CS_REG_VALUE_08BIT,
							SC202CS_MODE_SW_STANDBY);
				dev_info(&sc202cs->client->dev, "quickstream, streaming off: enter soft standby mode\n");
			}
		}

		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = sc202cs_get_channel_info(sc202cs, ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc202cs_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	struct rkmodule_channel_info *ch_info;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc202cs_ioctl(sd, cmd, inf);
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
			ret = sc202cs_ioctl(sd, cmd, cfg);
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

		ret = sc202cs_ioctl(sd, cmd, hdr);
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
			ret = sc202cs_ioctl(sd, cmd, hdr);
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
			ret = sc202cs_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = sc202cs_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc202cs_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc202cs_start_stream(struct sc202cs *sc202cs)
{
	int ret = 0;

	dev_info(&sc202cs->client->dev,
		 "%dx%d@%d, mode %d, vts 0x%x\n",
		 sc202cs->cur_mode->width,
		 sc202cs->cur_mode->height,
		 sc202cs->cur_fps.denominator / sc202cs->cur_fps.numerator,
		 sc202cs->cur_mode->hdr_mode,
		 sc202cs->cur_vts);

	if (!sc202cs->is_thunderboot) {
		ret = sc202cs_write_array(sc202cs->client, sc202cs->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&sc202cs->ctrl_handler);
		if (ret)
			return ret;
		if (sc202cs->has_init_exp && sc202cs->cur_mode->hdr_mode != NO_HDR) {
			ret = sc202cs_ioctl(&sc202cs->subdev, PREISP_CMD_SET_HDRAE_EXP,
					    &sc202cs->init_hdrae_exp);
			if (ret) {
				dev_err(&sc202cs->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}

	return sc202cs_write_reg(sc202cs->client, SC202CS_REG_CTRL_MODE,
				 SC202CS_REG_VALUE_08BIT, SC202CS_MODE_STREAMING);
}

static int __sc202cs_stop_stream(struct sc202cs *sc202cs)
{
	int ret = 0;

	sc202cs->has_init_exp = false;
	if (sc202cs->is_thunderboot) {
		sc202cs->is_first_streamoff = true;
		pm_runtime_put(&sc202cs->client->dev);
	}

	ret |= sc202cs_write_reg(sc202cs->client, SC202CS_REG_CTRL_MODE,
				 SC202CS_REG_VALUE_08BIT, SC202CS_MODE_SW_STANDBY);
	return ret;
}

static int __sc202cs_power_on(struct sc202cs *sc202cs);
static int sc202cs_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	struct i2c_client *client = sc202cs->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
		 sc202cs->cur_mode->width,
		 sc202cs->cur_mode->height,
		 DIV_ROUND_CLOSEST(sc202cs->cur_mode->max_fps.denominator,
				   sc202cs->cur_mode->max_fps.numerator));

	mutex_lock(&sc202cs->mutex);
	on = !!on;
	if (on == sc202cs->streaming)
		goto unlock_and_return;

	if (on) {
		if (sc202cs->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			sc202cs->is_thunderboot = false;
			__sc202cs_power_on(sc202cs);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc202cs_start_stream(sc202cs);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc202cs_stop_stream(sc202cs);
		pm_runtime_put(&client->dev);
	}

	sc202cs->streaming = on;

unlock_and_return:
	mutex_unlock(&sc202cs->mutex);

	return ret;
}

static int sc202cs_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	struct i2c_client *client = sc202cs->client;
	int ret = 0;

	mutex_lock(&sc202cs->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc202cs->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!sc202cs->is_thunderboot) {
			ret = sc202cs_write_array(sc202cs->client, sc202cs_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		sc202cs->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc202cs->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc202cs->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sc202cs_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SC202CS_XVCLK_FREQ / 1000 / 1000);
}

static int __sc202cs_power_on(struct sc202cs *sc202cs)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sc202cs->client->dev;

	if (!IS_ERR_OR_NULL(sc202cs->pins_default)) {
		ret = pinctrl_select_state(sc202cs->pinctrl,
					   sc202cs->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc202cs->xvclk, SC202CS_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sc202cs->xvclk) != SC202CS_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sc202cs->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	cam_sw_regulator_bulk_init(sc202cs->cam_sw_inf, SC202CS_NUM_SUPPLIES, sc202cs->supplies);

	dev_info(dev, "is_thunderboot : %d\n", sc202cs->is_thunderboot);
	if (sc202cs->is_thunderboot)
		return 0;

	if (!IS_ERR(sc202cs->reset_gpio))
		gpiod_set_value_cansleep(sc202cs->reset_gpio, 0);

	ret = regulator_bulk_enable(SC202CS_NUM_SUPPLIES, sc202cs->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc202cs->reset_gpio))
		gpiod_set_value_cansleep(sc202cs->reset_gpio, 1);

	dev_info(dev, "reset gpio pull up\n");

	usleep_range(500, 1000);
	if (!IS_ERR(sc202cs->pwdn_gpio))
		gpiod_set_value_cansleep(sc202cs->pwdn_gpio, 1);

	if (!IS_ERR(sc202cs->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sc202cs_cal_delay(8192);  //about 4ms delay
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(sc202cs->xvclk);

	return ret;
}

static void __sc202cs_power_off(struct sc202cs *sc202cs)
{
	int ret;
	struct device *dev = &sc202cs->client->dev;

	clk_disable_unprepare(sc202cs->xvclk);
	if (sc202cs->is_thunderboot) {
		if (sc202cs->is_first_streamoff) {
			sc202cs->is_thunderboot = false;
			sc202cs->is_first_streamoff = false;
		} else {
			return;
		}
	}
	if (!IS_ERR(sc202cs->pwdn_gpio))
		gpiod_set_value_cansleep(sc202cs->pwdn_gpio, 0);
	clk_disable_unprepare(sc202cs->xvclk);
	if (!IS_ERR(sc202cs->reset_gpio))
		gpiod_set_value_cansleep(sc202cs->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(sc202cs->pins_sleep)) {
		ret = pinctrl_select_state(sc202cs->pinctrl,
					   sc202cs->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC202CS_NUM_SUPPLIES, sc202cs->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int __maybe_unused sc202cs_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	if (sc202cs->standby_hw) {
		dev_info(dev, "resume standby!");
		return 0;
	}

	cam_sw_prepare_wakeup(sc202cs->cam_sw_inf, dev);

	usleep_range(4000, 5000);
	cam_sw_write_array(sc202cs->cam_sw_inf);

	if (__v4l2_ctrl_handler_setup(&sc202cs->ctrl_handler))
		dev_err(dev, "__v4l2_ctrl_handler_setup fail!");

	return 0;
}

static int __maybe_unused sc202cs_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	if (sc202cs->standby_hw) {
		dev_info(dev, "suspend standby!");
		return 0;
	}

	cam_sw_write_array_cb_init(sc202cs->cam_sw_inf, client,
				   (void *)sc202cs->cur_mode->reg_list,
				   (sensor_write_array)sc202cs_write_array);
	cam_sw_prepare_sleep(sc202cs->cam_sw_inf);

	return 0;
}
#else
#define sc202cs_resume NULL
#define sc202cs_suspend NULL
#endif

static int sc202cs_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	return __sc202cs_power_on(sc202cs);
}

static int sc202cs_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	__sc202cs_power_off(sc202cs);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc202cs_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc202cs_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc202cs->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc202cs->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc202cs_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops sc202cs_pm_ops = {
	SET_RUNTIME_PM_OPS(sc202cs_runtime_suspend,
			   sc202cs_runtime_resume, NULL)
#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(sc202cs_suspend, sc202cs_resume)
#endif
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc202cs_internal_ops = {
	.open = sc202cs_open,
};
#endif

static const struct v4l2_subdev_core_ops sc202cs_core_ops = {
	.s_power = sc202cs_s_power,
	.ioctl = sc202cs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc202cs_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc202cs_video_ops = {
	.s_stream = sc202cs_s_stream,
	.g_frame_interval = sc202cs_g_frame_interval,
	.s_frame_interval = sc202cs_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc202cs_pad_ops = {
	.enum_mbus_code = sc202cs_enum_mbus_code,
	.enum_frame_size = sc202cs_enum_frame_sizes,
	.enum_frame_interval = sc202cs_enum_frame_interval,
	.get_fmt = sc202cs_get_fmt,
	.set_fmt = sc202cs_set_fmt,
	.get_mbus_config = sc202cs_g_mbus_config,
};

static const struct v4l2_subdev_ops sc202cs_subdev_ops = {
	.core	= &sc202cs_core_ops,
	.video	= &sc202cs_video_ops,
	.pad	= &sc202cs_pad_ops,
};

static void sc202cs_modify_fps_info(struct sc202cs *sc202cs)
{
	const struct sc202cs_mode *mode = sc202cs->cur_mode;

	sc202cs->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       sc202cs->cur_vts;
}

static int sc202cs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc202cs *sc202cs = container_of(ctrl->handler,
					       struct sc202cs, ctrl_handler);
	struct i2c_client *client = sc202cs->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc202cs->cur_mode->height + ctrl->val - 6;
		__v4l2_ctrl_modify_range(sc202cs->exposure,
					 sc202cs->exposure->minimum, max,
					 sc202cs->exposure->step,
					 sc202cs->exposure->default_value);
		break;
	}

	if (sc202cs->standby_hw && sc202cs->is_standby) {
		dev_dbg(&client->dev, "%s: is_standby = true, will return\n", __func__);
		return 0;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure value 0x%x\n", ctrl->val);
		/* 4 least significant bits of expsoure are fractional part */
		ret = sc202cs_write_reg(sc202cs->client, SC202CS_REG_EXPOSURE_H,
					SC202CS_REG_VALUE_08BIT, SC202CS_FETCH_EXP_H(ctrl->val));
		ret |= sc202cs_write_reg(sc202cs->client, SC202CS_REG_EXPOSURE_M,
					 SC202CS_REG_VALUE_08BIT, SC202CS_FETCH_EXP_M(ctrl->val));
		ret |= sc202cs_write_reg(sc202cs->client, SC202CS_REG_EXPOSURE_L,
					 SC202CS_REG_VALUE_08BIT, SC202CS_FETCH_EXP_L(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain value 0x%x\n", ctrl->val);
		if (sc202cs->cur_mode->hdr_mode == NO_HDR)
			ret = sc202cs_set_gain_reg(sc202cs, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set blank value 0x%x\n", ctrl->val);
		ret = sc202cs_write_reg(sc202cs->client, SC202CS_REG_VTS_H,
					SC202CS_REG_VALUE_08BIT,
					(ctrl->val + sc202cs->cur_mode->height) >> 8);
		ret |= sc202cs_write_reg(sc202cs->client, SC202CS_REG_VTS_L,
					 SC202CS_REG_VALUE_08BIT,
					 (ctrl->val + sc202cs->cur_mode->height) & 0xff);
		if (!ret)
			sc202cs->cur_vts = ctrl->val + sc202cs->cur_mode->height;
		sc202cs_modify_fps_info(sc202cs);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc202cs_enable_test_pattern(sc202cs, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc202cs_read_reg(sc202cs->client, SC202CS_FLIP_MIRROR_REG,
				       SC202CS_REG_VALUE_08BIT, &val);
		ret |= sc202cs_write_reg(sc202cs->client, SC202CS_FLIP_MIRROR_REG,
					 SC202CS_REG_VALUE_08BIT,
					 SC202CS_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = sc202cs_read_reg(sc202cs->client, SC202CS_FLIP_MIRROR_REG,
				       SC202CS_REG_VALUE_08BIT, &val);
		ret |= sc202cs_write_reg(sc202cs->client, SC202CS_FLIP_MIRROR_REG,
					 SC202CS_REG_VALUE_08BIT,
					 SC202CS_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc202cs_ctrl_ops = {
	.s_ctrl = sc202cs_set_ctrl,
};

static int sc202cs_initialize_controls(struct sc202cs *sc202cs)
{
	const struct sc202cs_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &sc202cs->ctrl_handler;
	mode = sc202cs->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc202cs->mutex;

	sc202cs->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
			     ARRAY_SIZE(link_freq_menu_items) - 1, 0,
			     link_freq_menu_items);
	if (sc202cs->link_freq)
		sc202cs->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	__v4l2_ctrl_s_ctrl(sc202cs->link_freq, mode->mipi_freq_idx);

	sc202cs->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
						V4L2_CID_PIXEL_RATE, 0,
						PIXEL_RATE_WITH_360M_10BIT, 1,
						PIXEL_RATE_WITH_360M_10BIT);

	h_blank = mode->hts_def - mode->width;
	sc202cs->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (sc202cs->hblank)
		sc202cs->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sc202cs->cur_fps = mode->max_fps;
	sc202cs->cur_vts = mode->vts_def;

	vblank_def = mode->vts_def - mode->height;
	sc202cs->vblank = v4l2_ctrl_new_std(handler, &sc202cs_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    SC202CS_VTS_MAX - mode->height,
					    1, vblank_def);
	exposure_max = 2 * mode->vts_def - 6;
	sc202cs->exposure = v4l2_ctrl_new_std(handler, &sc202cs_ctrl_ops,
					      V4L2_CID_EXPOSURE, SC202CS_EXPOSURE_MIN,
					      exposure_max, SC202CS_EXPOSURE_STEP,
					      mode->exp_def);
	sc202cs->anal_gain = v4l2_ctrl_new_std(handler, &sc202cs_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, SC202CS_GAIN_MIN,
					       SC202CS_GAIN_MAX, SC202CS_GAIN_STEP,
					       SC202CS_GAIN_DEFAULT);
	sc202cs->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&sc202cs_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(sc202cs_test_pattern_menu) - 1,
				0, 0, sc202cs_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &sc202cs_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &sc202cs_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc202cs->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc202cs->subdev.ctrl_handler = handler;
	sc202cs->has_init_exp = false;
	sc202cs->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc202cs_check_sensor_id(struct sc202cs *sc202cs,
				   struct i2c_client *client)
{
	struct device *dev = &sc202cs->client->dev;
	u32 id = 0;
	int ret;

	if (sc202cs->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = sc202cs_read_reg(client, SC202CS_REG_CHIP_ID,
			       SC202CS_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC202CS CHIP ID = (0x%04x) sensor\n", CHIP_ID);

	return 0;
}

static int sc202cs_configure_regulators(struct sc202cs *sc202cs)
{
	unsigned int i;

	for (i = 0; i < SC202CS_NUM_SUPPLIES; i++)
		sc202cs->supplies[i].supply = sc202cs_supply_names[i];

	return devm_regulator_bulk_get(&sc202cs->client->dev,
				       SC202CS_NUM_SUPPLIES,
				       sc202cs->supplies);
}

#ifdef CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP
static void find_terminal_resolution(struct sc202cs *sc202cs)
{
	int i = 0;
	const struct sc202cs_mode *mode = NULL;
	u32 rk_cam_hdr = get_rk_cam_hdr();
	u32 rk_cam_w = get_rk_cam_w();
	u32 rk_cam_h = get_rk_cam_h();

	if (rk_cam_w == 0 || rk_cam_h == 0)
		goto err_find_res;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == rk_cam_w && mode->height == rk_cam_h &&
		    mode->hdr_mode == rk_cam_hdr) {
			sc202cs->cur_mode = mode;
			return;
		}
	}
err_find_res:
	dev_err(&sc202cs->client->dev, "not match %dx%d mode %d\n!",
		rk_cam_w, rk_cam_h, rk_cam_hdr);
	sc202cs->cur_mode = &supported_modes[0];
}
#else
static void find_terminal_resolution(struct sc202cs *sc202cs)
{
	u32 hdr_mode = 0;
	struct device_node *node = sc202cs->client->dev.of_node;
	int i = 0;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			sc202cs->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		sc202cs->cur_mode = &supported_modes[0];
}
#endif

static int sc202cs_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc202cs *sc202cs;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc202cs = devm_kzalloc(dev, sizeof(*sc202cs), GFP_KERNEL);
	if (!sc202cs)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc202cs->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc202cs->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc202cs->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc202cs->len_name);
	/* Compatible with non-standby mode if this attribute is not configured in dts*/
	of_property_read_u32(node, RKMODULE_CAMERA_STANDBY_HW,
			     &sc202cs->standby_hw);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	dev_info(dev, "sc202cs->standby_hw = %d\n", sc202cs->standby_hw);

	sc202cs->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	sc202cs->client = client;

	find_terminal_resolution(sc202cs);

	sc202cs->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc202cs->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc202cs->reset_gpio = devm_gpiod_get(dev, "reset",
					     sc202cs->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sc202cs->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc202cs->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
					    sc202cs->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sc202cs->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc202cs->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc202cs->pinctrl)) {
		sc202cs->pins_default =
			pinctrl_lookup_state(sc202cs->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc202cs->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc202cs->pins_sleep =
			pinctrl_lookup_state(sc202cs->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc202cs->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc202cs_configure_regulators(sc202cs);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc202cs->mutex);

	sd = &sc202cs->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc202cs_subdev_ops);
	ret = sc202cs_initialize_controls(sc202cs);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc202cs_power_on(sc202cs);
	if (ret)
		goto err_free_handler;

	ret = sc202cs_check_sensor_id(sc202cs, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc202cs_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc202cs->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc202cs->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	if (!sc202cs->cam_sw_inf) {
		sc202cs->cam_sw_inf = cam_sw_init();
		cam_sw_clk_init(sc202cs->cam_sw_inf, sc202cs->xvclk, SC202CS_XVCLK_FREQ);
		cam_sw_reset_pin_init(sc202cs->cam_sw_inf, sc202cs->reset_gpio, 0);
		cam_sw_pwdn_pin_init(sc202cs->cam_sw_inf, sc202cs->pwdn_gpio, 1);
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc202cs->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc202cs->module_index, facing,
		 SC202CS_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (sc202cs->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc202cs_power_off(sc202cs);
err_free_handler:
	v4l2_ctrl_handler_free(&sc202cs->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc202cs->mutex);

	return ret;
}

static int sc202cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc202cs->ctrl_handler);
	mutex_destroy(&sc202cs->mutex);

	cam_sw_deinit(sc202cs->cam_sw_inf);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc202cs_power_off(sc202cs);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc202cs_of_match[] = {
	{ .compatible = "smartsens,sc202cs" },
	{},
};
MODULE_DEVICE_TABLE(of, sc202cs_of_match);
#endif

static const struct i2c_device_id sc202cs_match_id[] = {
	{ "smartsens,sc202cs", 0 },
	{ },
};

static struct i2c_driver sc202cs_i2c_driver = {
	.driver = {
		.name = SC202CS_NAME,
		.pm = &sc202cs_pm_ops,
		.of_match_table = of_match_ptr(sc202cs_of_match),
	},
	.probe		= &sc202cs_probe,
	.remove		= &sc202cs_remove,
	.id_table	= sc202cs_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc202cs_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc202cs_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens sc202cs sensor driver");
MODULE_LICENSE("GPL");
