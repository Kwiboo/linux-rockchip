// SPDX-License-Identifier: GPL-2.0
/*
 * sc835hai driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 * V0.0X01.0X02 change power_gpio to pwdn_gpio
 * V0.0X01.0X03 support thunder boot
 * V0.0X01.0X04 support sleep/wake up
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
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"
#include "cam-sleep-wakeup.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x04)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define MIPI_FREQ_468M			468000000

#define SC835HAI_MAX_PIXEL_RATE	(MIPI_FREQ_468M / 10 * 2 * SC835HAI_4LANES)
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define SC835HAI_XVCLK_FREQ_24M		24000000

/* TODO: Get the real chip id from reg */
#define CHIP_ID				0xC170
#define SC835HAI_REG_CHIP_ID		0x3107

#define SC835HAI_REG_CTRL_MODE		0x0100
#define SC835HAI_MODE_SW_STANDBY	0x0
#define SC835HAI_MODE_STREAMING		BIT(0)

/*expo*/
#define	SC835HAI_EXPOSURE_MIN		2    /*okay*/
#define	SC835HAI_EXPOSURE_STEP		1    /*okay*/
#define SC835HAI_VTS_MAX		0xffff   /*okay*/

//long exposure
#define SC835HAI_REG_EXP_LONG_H		0x3e00    //[3:0]
#define SC835HAI_REG_EXP_LONG_M		0x3e01    //[7:0]
#define SC835HAI_REG_EXP_LONG_L		0x3e02    //[7:4]

//short exposure  //for hdr
#define SC835HAI_REG_EXP_SF_H		0x3e22
#define SC835HAI_REG_EXP_SF_M		0x3e04    //[7:0]
#define SC835HAI_REG_EXP_SF_L		0x3e05    //[7:4]

#define SC835HAI_FETCH_EXP_H(VAL)	(((VAL) >> 12) & 0xF)
#define SC835HAI_FETCH_EXP_M(VAL)	(((VAL) >> 4) & 0xFF)
#define SC835HAI_FETCH_EXP_L(VAL)	(((VAL) & 0xF) << 4)

/*gain*/
//long frame and normal gain reg
#define SC835HAI_REG_DGAIN		0x3e06
#define SC835HAI_REG_AGAIN		0x3e08
#define SC835HAI_REG_AGAIN_FINE		0x3e09
#define SC835HAI_REG_DGAIN_FINE		0x3e07

//short fram gain reg
#define SC835HAI_SF_REG_AGAIN		0x3e12
#define SC835HAI_SF_REG_AGAIN_FINE	0x3e13
#define SC835HAI_SF_REG_DGAIN		0x3e10

#define SC835HAI_GAIN_MIN		0x20	//1.000 = 32 * 1/32
#define SC835HAI_GAIN_MAX		(48 * 4 * 32)   /*need_view   47.25*3.938*32=5954  */
#define SC835HAI_GAIN_STEP		1
#define SC835HAI_GAIN_DEFAULT		0x40

#define SC835HAI_REG_VTS		0x320e

//group hold
#define SC835HAI_GROUP_UPDATE_ADDRESS		0x3800
#define SC835HAI_GROUP_UPDATE_START_DATA	0x00
#define SC835HAI_GROUP_UPDATE_LAUNCH		0x30

#define SC835HAI_SOFTWARE_RESET_REG	0x0103
#define SC835HAI_REG_TEST_PATTERN	0x4501
#define SC835HAI_TEST_PATTERN_ENABLE	0x08

#define SC835HAI_FLIP_REG		0x3221
#define SC835HAI_FLIP_MASK		0x60
#define SC835HAI_MIRROR_MASK		0x06

#define REG_NULL			0xFFFF

#define SC835HAI_REG_VALUE_08BIT	1
#define SC835HAI_REG_VALUE_16BIT	2
#define SC835HAI_REG_VALUE_24BIT	3

#define SC835HAI_4LANES			4

#define ENABLE_NR			1
#define SC835HAI_LGAIN			0
#define SC835HAI_SGAIN			1

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define SC835HAI_NAME			"sc835hai"

static const char *const sc835hai_supply_names[] = {
	"dvdd",		// Digital core power
	"dovdd",	// Digital I/O power
	"avdd",		// Analog power
};

#define SC835HAI_NUM_SUPPLIES ARRAY_SIZE(sc835hai_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc835hai_mode {
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

struct sc835hai {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC835HAI_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_a_gain;
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
	const struct sc835hai_mode *cur_mode;
	u32			module_index;
	u32			cfg_num;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	struct cam_sw_info	*cam_sw_info;
};

#define to_sc835hai(sd) container_of(sd, struct sc835hai, subdev)

//Cleaned_0x1f_SC835HAI_MIPI_24Minput_4lane_10bit_936Mbps_3840x2160_40fps.ini
static __maybe_unused const struct regval sc835hai_linear10bit_3840x2160_regs[] = {
	{0x0103, 0x01},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x301f, 0x1f},
	{0x30b8, 0x44},
	{0x3200, 0x00},
	{0x3201, 0x00},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3204, 0x0f},
	{0x3205, 0x07},
	{0x3206, 0x08},
	{0x3207, 0x77},
	{0x3208, 0x0f},
	{0x3209, 0x00},
	{0x320a, 0x08},
	{0x320b, 0x72},
	{0x320c, 0x07},
	{0x320d, 0xe9},
	{0x320e, 0x08},
	{0x320f, 0xca},
	{0x3210, 0x00},
	{0x3211, 0x04},
	{0x3212, 0x00},
	{0x3213, 0x02},
	{0x3301, 0x0e},
	{0x3302, 0x18},
	{0x3303, 0x10},
	{0x3304, 0x58},
	{0x3305, 0x00},
	{0x3306, 0x70},
	{0x3307, 0x04},
	{0x3308, 0x14},
	{0x3309, 0x98},
	{0x330a, 0x00},
	{0x330b, 0xf8},
	{0x330c, 0x10},
	{0x330d, 0x08},
	{0x330e, 0x4a},
	{0x331e, 0x31},
	{0x331f, 0x71},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x334c, 0x10},
	{0x335d, 0x60},
	{0x3364, 0x5e},
	{0x3366, 0x01},
	{0x3367, 0x04},
	{0x338f, 0x80},
	{0x3390, 0x01},
	{0x3391, 0x03},
	{0x3392, 0x07},
	{0x3393, 0x28},
	{0x3394, 0x4c},
	{0x3395, 0x4c},
	{0x3396, 0x01},
	{0x3397, 0x03},
	{0x3398, 0x07},
	{0x3399, 0x1a},
	{0x339a, 0x1a},
	{0x339b, 0x2c},
	{0x339c, 0x2c},
	{0x33ad, 0x24},
	{0x33ae, 0x40},
	{0x33af, 0x80},
	{0x33b2, 0x50},
	{0x33b3, 0x20},
	{0x33f8, 0x00},
	{0x33f9, 0x88},
	{0x33fa, 0x00},
	{0x33fb, 0xa0},
	{0x33fc, 0x43},
	{0x33fd, 0x47},
	{0x349f, 0x03},
	{0x34a6, 0x43},
	{0x34a7, 0x47},
	{0x34a8, 0x20},
	{0x34a9, 0x20},
	{0x34aa, 0x01},
	{0x34ab, 0x10},
	{0x34ac, 0x01},
	{0x34ad, 0x28},
	{0x34f8, 0x43},
	{0x34f9, 0x0c},
	{0x3632, 0x64},
	{0x363b, 0x16},
	{0x363c, 0x0e},
	{0x363d, 0x8e},
	{0x363e, 0x6c},
	{0x3654, 0x00},
	{0x3674, 0x94},
	{0x3675, 0x94},
	{0x3676, 0x68},
	{0x367c, 0x41},
	{0x367d, 0x43},
	{0x3690, 0x35},
	{0x3691, 0x35},
	{0x3692, 0x55},
	{0x3693, 0x40},
	{0x3694, 0x41},
	{0x3696, 0x81},
	{0x3697, 0x80},
	{0x3698, 0x80},
	{0x3699, 0x83},
	{0x369a, 0x81},
	{0x369b, 0xff},
	{0x369c, 0xff},
	{0x369d, 0xff},
	{0x36a2, 0x40},
	{0x36a3, 0x41},
	{0x36a4, 0x43},
	{0x36a5, 0x47},
	{0x36a6, 0x4f},
	{0x36a7, 0x4f},
	{0x36a8, 0x4f},
	{0x36d0, 0x15},
	{0x36ea, 0xcd},
	{0x36eb, 0x04},
	{0x36ec, 0x43},
	{0x36ed, 0x0a},
	{0x370f, 0x01},
	{0x3721, 0x6c},
	{0x3724, 0xe5},
	{0x3725, 0xa8},
	{0x3727, 0x14},
	{0x37b0, 0x17},
	{0x37b1, 0x9b},
	{0x37b2, 0xfb},
	{0x37b3, 0x41},
	{0x37b4, 0x43},
	{0x37fa, 0xc9},
	{0x37fb, 0x31},
	{0x37fc, 0x00},
	{0x37fd, 0x36},
	{0x3905, 0x0f},
	{0x391f, 0x41},
	{0x3933, 0x80},
	{0x3934, 0xd4},
	{0x3935, 0x00},
	{0x3936, 0x40},
	{0x3937, 0x69},
	{0x3938, 0x70},
	{0x3939, 0xff},
	{0x393a, 0xf6},
	{0x393b, 0xff},
	{0x393c, 0xd5},
	{0x3e00, 0x01},
	{0x3e01, 0x18},
	{0x3e02, 0x70},
	{0x3e16, 0x00},
	{0x3e17, 0xbc},
	{0x3e18, 0x00},
	{0x3e19, 0xbc},
	{0x4424, 0x02},
	{0x4509, 0x1a},
	{0x450d, 0x0b},
	{0x4800, 0x24},
	{0x5000, 0x0e},
	{0x550e, 0x02},
	{0x550f, 0x1c},
	{0x5510, 0x28},
	{0x575c, 0x10},
	{0x575d, 0x08},
	{0x5780, 0x76},
	{0x5784, 0x10},
	{0x5785, 0x08},
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
	{0x57a8, 0xd2},
	{0x57aa, 0x2a},
	{0x57ab, 0x7f},
	{0x57ac, 0x00},
	{0x57ad, 0x00},
	{0x36e9, 0x24},
	{0x37f9, 0x53},
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
static const struct sc835hai_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 400000,
		},
		.exp_def = 0x08c0,
		// .hts_def = 0x0226 * 5 - 0x180,
		.hts_def = 0x07e9,
		.vts_def = 0x08ca,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = sc835hai_linear10bit_3840x2160_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char *const sc835hai_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_468M,
};

/* Write registers up to 4 at a time */
static int sc835hai_write_reg(struct i2c_client *client, u16 reg,
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

static int sc835hai_write_array(struct i2c_client *client,
				const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret = sc835hai_write_reg(client, regs[i].addr,
					 SC835HAI_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int sc835hai_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static int sc835hai_get_reso_dist(const struct sc835hai_mode *mode,
				  struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc835hai_mode *
sc835hai_find_best_fit(struct sc835hai *sc835hai, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sc835hai_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		} else if (dist == cur_best_fit_dist &&
			   framefmt->code == supported_modes[i].bus_fmt) {
			cur_best_fit = i;
			break;
		}
	}
	dev_info(&sc835hai->client->dev, "%s: cur_best_fit(%d)",
		 __func__, cur_best_fit);

	return &supported_modes[cur_best_fit];
}

static void sc835hai_change_mode(struct sc835hai *sc835hai, const struct sc835hai_mode *mode)
{
	sc835hai->cur_mode = mode;
	sc835hai->cur_vts = sc835hai->cur_mode->vts_def;
	dev_info(&sc835hai->client->dev, "set fmt: cur_mode: %dx%d, hdr: %d\n",
		 mode->width, mode->height, mode->hdr_mode);
}

static int sc835hai_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	const struct sc835hai_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&sc835hai->mutex);

	mode = sc835hai_find_best_fit(sc835hai, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc835hai->mutex);
		return -ENOTTY;
#endif
	} else {
		sc835hai_change_mode(sc835hai, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc835hai->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc835hai->vblank, vblank_def,
					 SC835HAI_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(sc835hai->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * SC835HAI_4LANES;
		__v4l2_ctrl_s_ctrl_int64(sc835hai->pixel_rate, pixel_rate);
		sc835hai->cur_fps = mode->max_fps;
		sc835hai->cur_vts = mode->vts_def;
	}

	mutex_unlock(&sc835hai->mutex);

	return 0;
}

static int sc835hai_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	const struct sc835hai_mode *mode = sc835hai->cur_mode;

	mutex_lock(&sc835hai->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc835hai->mutex);
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
	mutex_unlock(&sc835hai->mutex);

	return 0;
}

static int sc835hai_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int sc835hai_enum_frame_sizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);

	if (fse->index >= sc835hai->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc835hai_enable_test_pattern(struct sc835hai *sc835hai, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc835hai_read_reg(sc835hai->client, SC835HAI_REG_TEST_PATTERN,
				SC835HAI_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC835HAI_TEST_PATTERN_ENABLE;
	else
		val &= ~SC835HAI_TEST_PATTERN_ENABLE;
	ret |= sc835hai_write_reg(sc835hai->client, SC835HAI_REG_TEST_PATTERN,
				  SC835HAI_REG_VALUE_08BIT, val);
	return ret;
}

static int sc835hai_g_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	const struct sc835hai_mode *mode = sc835hai->cur_mode;

	if (sc835hai->streaming)
		fi->interval = sc835hai->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct sc835hai_mode *sc835hai_find_mode(struct sc835hai *sc835hai, int fps)
{
	const struct sc835hai_mode *mode = NULL;
	const struct sc835hai_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < sc835hai->cfg_num; i++) {
		mode = &supported_modes[i];
		if (mode->width == sc835hai->cur_mode->width &&
		    mode->height == sc835hai->cur_mode->height &&
		    mode->hdr_mode == sc835hai->cur_mode->hdr_mode &&
		    mode->bus_fmt == sc835hai->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int sc835hai_s_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	const struct sc835hai_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	int fps;

	if (sc835hai->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = sc835hai_find_mode(sc835hai, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	sc835hai->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(sc835hai->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(sc835hai->vblank, vblank_def,
				 SC835HAI_VTS_MAX - mode->height,
				 1, vblank_def);
	__v4l2_ctrl_s_ctrl(sc835hai->link_freq, mode->mipi_freq_idx);
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
		     mode->bpp * 2 * SC835HAI_4LANES;
	__v4l2_ctrl_s_ctrl_int64(sc835hai->pixel_rate, pixel_rate);
	sc835hai->cur_fps = mode->max_fps;
	sc835hai->cur_vts = mode->vts_def;

	return 0;
}

static int sc835hai_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				  struct v4l2_mbus_config *config)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	const struct sc835hai_mode *mode = sc835hai->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (SC835HAI_4LANES - 1) |
		      V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode == HDR_X2)
		val = 1 << (SC835HAI_4LANES - 1) |
		      V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		      V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void sc835hai_get_module_inf(struct sc835hai *sc835hai,
				    struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC835HAI_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc835hai->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc835hai->len_name, sizeof(inf->base.lens));
}

/* mode: 0 = lgain  1 = sgain */
static int sc835hai_set_gain_reg(struct sc835hai *sc835hai, u32 gain, int mode)
{
	u8 Coarse_gain = 1, DIG_gain = 1;
	u32 Dcg_gainx1000 = 1, ANA_Fine_gainx32 = 1;
	u8 ANA_Fine_gain_reg = 0x20, DIG_Fine_gain_reg = 0;
	u8 Coarse_gain_reg = 0, DIG_gain_reg = 0;
	int ret = 0;
	u64 val = 0;

	gain = gain * 32;
	if (gain <= 1024)
		gain = 1024;
	else if (gain > SC835HAI_GAIN_MAX * 32)
		gain = SC835HAI_GAIN_MAX * 32;

	//again
	if (gain < 1536) {		/* start again 1.0x --1.5x  1.5*1024 = */
		Dcg_gainx1000 = 1000;
		Coarse_gain = 1;
		DIG_gain = 1;
		Coarse_gain_reg = 0x00;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain < 3072) {	/* 1.5x ~ 3.0x  1024 * 3.0 = 3481*/
		Dcg_gainx1000 = 1500;
		Coarse_gain = 1;
		DIG_gain = 1;
		Coarse_gain_reg = 0x80;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain < 6144) {	/* 3.0x ~ 6.0x  1024 * 6.0 = 6144*/
		Dcg_gainx1000 = 1500;
		Coarse_gain = 2;
		DIG_gain = 1;
		Coarse_gain_reg = 0x81;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain < 12288) {	/* 6.0x ~ 12.0x  1024 * 12 = 12288*/
		Dcg_gainx1000 = 1500;
		Coarse_gain = 4;
		DIG_gain = 1;
		Coarse_gain_reg = 0x83;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain < 24576) {	/* 12.0x ~ 24.0x  1024 * 24 = 24576*/
		Dcg_gainx1000 = 1500;
		Coarse_gain = 8;
		DIG_gain = 1;
		Coarse_gain_reg = 0x87;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 48384) {	/* 24.0x ~ 47.25x  1024 * 47.25 = 48384*/
		// End again
		Dcg_gainx1000 = 1500;
		Coarse_gain = 16;
		DIG_gain = 1;
		Coarse_gain_reg = 0x8f;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain < 48384 * 2) {         // start dgain
		Dcg_gainx1000 = 1500;
		Coarse_gain = 16;
		DIG_gain = 1;
		ANA_Fine_gainx32 = 0x3f;
		Coarse_gain_reg = 0x8f;
		DIG_gain_reg = 0x0;
		ANA_Fine_gain_reg = 0x3f;
	} else if (gain <= 48384 * 4) {
		Dcg_gainx1000 = 1500;
		Coarse_gain = 16;
		DIG_gain = 2;
		ANA_Fine_gainx32 = 0x3f;
		Coarse_gain_reg = 0x8f;
		DIG_gain_reg = 0x1;
		ANA_Fine_gain_reg = 0x3f;
	}

	if (gain < 1536) {
		val = div_u64(1000ULL * gain, (Dcg_gainx1000 * Coarse_gain));
		ANA_Fine_gain_reg = div_u64(val, 32);
	} else if (gain == 1536) {
		ANA_Fine_gain_reg = 0x20;
	} else if (gain < 48384) {
		val = div_u64(1000ULL * gain, (Dcg_gainx1000 * Coarse_gain));
		ANA_Fine_gain_reg = div_u64(val, 32);
	} else if (gain < 48384 * 4) {
		val = div_u64(8000ULL * gain, (Dcg_gainx1000 * Coarse_gain * DIG_gain));
		DIG_Fine_gain_reg = div_u64(val, ANA_Fine_gainx32);
	} else {
		DIG_Fine_gain_reg = 0x80;
	}

	if (mode == SC835HAI_LGAIN) {
		ret = sc835hai_write_reg(sc835hai->client,
					 SC835HAI_REG_DGAIN,
					 SC835HAI_REG_VALUE_08BIT,
					 DIG_gain_reg & 0xF);
		ret |= sc835hai_write_reg(sc835hai->client,
					  SC835HAI_REG_DGAIN_FINE,
					  SC835HAI_REG_VALUE_08BIT,
					  DIG_Fine_gain_reg);
		ret |= sc835hai_write_reg(sc835hai->client,
					  SC835HAI_REG_AGAIN,
					  SC835HAI_REG_VALUE_08BIT,
					  Coarse_gain_reg);
		ret |= sc835hai_write_reg(sc835hai->client,
					  SC835HAI_REG_AGAIN_FINE,
					  SC835HAI_REG_VALUE_08BIT,
					  ANA_Fine_gain_reg);
	}
#if ENABLE_NR
	if (gain < 1536)
		ret |= sc835hai_write_reg(sc835hai->client,
					  0x363c,
					  SC835HAI_REG_VALUE_08BIT,
					  0x05);
	else
		ret |= sc835hai_write_reg(sc835hai->client,
					  0x363c,
					  SC835HAI_REG_VALUE_08BIT,
					  0x07);
#endif
	dev_dbg(&sc835hai->client->dev,
		"recv_gain:%d set again 0x%x, again_fine 0x%x, set dgain 0x%x, dgain_fine 0x%x\n",
		gain / 16, Coarse_gain_reg, ANA_Fine_gain_reg, DIG_gain_reg, DIG_Fine_gain_reg);
	return ret;
}

static int sc835hai_get_channel_info(struct sc835hai *sc835hai,
				     struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = sc835hai->cur_mode->vc[ch_info->index];
	ch_info->width = sc835hai->cur_mode->width;
	ch_info->height = sc835hai->cur_mode->height;
	ch_info->bus_fmt = sc835hai->cur_mode->bus_fmt;
	return 0;
}

static long sc835hai_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	const struct sc835hai_mode *mode;
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	u64 pixel_rate = 0;
	u32 i, h, w, stream;
	int cur_best_fit = -1;
	int cur_best_fit_dist = -1;
	int cur_dist, cur_fps, dst_fps;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		/*
		 * ret = sc835hai_set_hdrae(sc835hai, arg);
		 */
		if (sc835hai->cam_sw_info)
			memcpy(&sc835hai->cam_sw_info->hdr_ae, (struct preisp_hdrae_exp_s *)(arg),
			       sizeof(struct preisp_hdrae_exp_s));
		break;

	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode == sc835hai->cur_mode->hdr_mode)
			return 0;
		w = sc835hai->cur_mode->width;
		h = sc835hai->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(sc835hai->cur_mode->max_fps.denominator,
					    sc835hai->cur_mode->max_fps.numerator);
		for (i = 0; i < sc835hai->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr_cfg->hdr_mode &&
			    supported_modes[i].bus_fmt == sc835hai->cur_mode->bus_fmt) {
				cur_fps = DIV_ROUND_CLOSEST(supported_modes[i].max_fps.denominator,
							    supported_modes[i].max_fps.numerator);
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
			dev_err(&sc835hai->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			sc835hai_change_mode(sc835hai, &supported_modes[cur_best_fit]);
			mode = sc835hai->cur_mode;
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(sc835hai->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc835hai->vblank, h,
						 SC835HAI_VTS_MAX - mode->height,
						 1, h);
			__v4l2_ctrl_s_ctrl(sc835hai->link_freq, mode->mipi_freq_idx);
			pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
				     mode->bpp * 2 * SC835HAI_4LANES;
			__v4l2_ctrl_s_ctrl_int64(sc835hai->pixel_rate,
						 pixel_rate);
			sc835hai->cur_fps = mode->max_fps;
			sc835hai->cur_vts = mode->vts_def;
			dev_info(&sc835hai->client->dev,
				 "sensor mode: %d\n", mode->hdr_mode);
		}
		break;
	case RKMODULE_GET_MODULE_INFO:
		sc835hai_get_module_inf(sc835hai, (struct rkmodule_inf *)arg);
		break;

	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sc835hai->cur_mode->hdr_mode;
		break;

	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream) {
			ret = sc835hai_write_reg(sc835hai->client, 0x3019,
						 SC835HAI_REG_VALUE_08BIT,
						 0xf0);
			ret |= sc835hai_write_reg(sc835hai->client, 0x3018,
						  SC835HAI_REG_VALUE_08BIT,
						  0x7a);
			ret |= sc835hai_write_reg(sc835hai->client, SC835HAI_REG_CTRL_MODE,
						  SC835HAI_REG_VALUE_08BIT,
						  SC835HAI_MODE_STREAMING);
		} else {
			ret = sc835hai_write_reg(sc835hai->client, 0x3018,
						 SC835HAI_REG_VALUE_08BIT,
						 0x7f);
			ret |= sc835hai_write_reg(sc835hai->client, 0x3019,
						  SC835HAI_REG_VALUE_08BIT,
						  0xff);
			ret |= sc835hai_write_reg(sc835hai->client, SC835HAI_REG_CTRL_MODE,
						  SC835HAI_REG_VALUE_08BIT,
						  SC835HAI_MODE_SW_STANDBY);
		}
		break;

	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = sc835hai_get_channel_info(sc835hai, ch_info);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc835hai_compat_ioctl32(struct v4l2_subdev *sd,
				    unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	struct rkmodule_channel_info *ch_info;
	long ret;
	u32  stream;
	u32 brl = 0;
	struct rkmodule_csi_dphy_param *dphy_param;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc835hai_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree(inf);
				return -EFAULT;
			}
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
		ret = sc835hai_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc835hai_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr))) {
				kfree(hdr);
				return -EFAULT;
			}
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
		ret = sc835hai_ioctl(sd, cmd, hdr);
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
		ret = sc835hai_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = sc835hai_ioctl(sd, cmd, &stream);
		break;
	case RKMODULE_GET_SONY_BRL:
		ret = sc835hai_ioctl(sd, cmd, &brl);
		if (!ret) {
			if (copy_to_user(up, &brl, sizeof(u32)))
				return -EFAULT;
		}
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc835hai_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
		dphy_param = kzalloc(sizeof(*dphy_param), GFP_KERNEL);
		if (!dphy_param) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc835hai_ioctl(sd, cmd, dphy_param);
		if (!ret) {
			ret = copy_to_user(up, dphy_param, sizeof(*dphy_param));
			if (ret)
				ret = -EFAULT;
		}
		kfree(dphy_param);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc835hai_start_stream(struct sc835hai *sc835hai)
{
	int ret;

	dev_info(&sc835hai->client->dev,
		 "%dx%d@%d, mode %d, vts 0x%x\n",
		 sc835hai->cur_mode->width,
		 sc835hai->cur_mode->height,
		 sc835hai->cur_fps.denominator / sc835hai->cur_fps.numerator,
		 sc835hai->cur_mode->hdr_mode,
		 sc835hai->cur_vts);

	if (!sc835hai->is_thunderboot) {
		ret = sc835hai_write_array(sc835hai->client, sc835hai->cur_mode->reg_list);
		if (ret)
			return ret;

		ret = __v4l2_ctrl_handler_setup(&sc835hai->ctrl_handler);
		if (ret)
			return ret;
		/* In case these controls are set before streaming */
		if (sc835hai->has_init_exp && sc835hai->cur_mode->hdr_mode != NO_HDR) {
			ret = sc835hai_ioctl(&sc835hai->subdev, PREISP_CMD_SET_HDRAE_EXP,
					     &sc835hai->init_hdrae_exp);
			if (ret) {
				dev_err(&sc835hai->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}
	return sc835hai_write_reg(sc835hai->client, SC835HAI_REG_CTRL_MODE,
				  SC835HAI_REG_VALUE_08BIT, SC835HAI_MODE_STREAMING);
}

static int __sc835hai_stop_stream(struct sc835hai *sc835hai)
{
	sc835hai->has_init_exp = false;
	if (sc835hai->is_thunderboot) {
		sc835hai->is_first_streamoff = true;
		pm_runtime_put(&sc835hai->client->dev);
	}
	return sc835hai_write_reg(sc835hai->client, SC835HAI_REG_CTRL_MODE,
				  SC835HAI_REG_VALUE_08BIT, SC835HAI_MODE_SW_STANDBY);
}

static int __sc835hai_power_on(struct sc835hai *sc835hai);
static int sc835hai_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	struct i2c_client *client = sc835hai->client;
	int ret = 0;

	dev_info(&sc835hai->client->dev, "s_stream: %d. %dx%d, hdr: %d, bpp: %d\n",
		 on, sc835hai->cur_mode->width, sc835hai->cur_mode->height,
		 sc835hai->cur_mode->hdr_mode, sc835hai->cur_mode->bpp);

	mutex_lock(&sc835hai->mutex);
	on = !!on;
	if (on == sc835hai->streaming)
		goto unlock_and_return;

	if (on) {
		if (sc835hai->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			sc835hai->is_thunderboot = false;
			__sc835hai_power_on(sc835hai);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __sc835hai_start_stream(sc835hai);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc835hai_stop_stream(sc835hai);
		pm_runtime_put(&client->dev);
	}

	sc835hai->streaming = on;

unlock_and_return:
	mutex_unlock(&sc835hai->mutex);
	return ret;
}

static int sc835hai_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	struct i2c_client *client = sc835hai->client;
	int ret = 0;

	mutex_lock(&sc835hai->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc835hai->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		if (!sc835hai->is_thunderboot) {
			ret |= sc835hai_write_reg(sc835hai->client,
						  SC835HAI_SOFTWARE_RESET_REG,
						  SC835HAI_REG_VALUE_08BIT,
						  0x01);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
			udelay(100);
		}

		sc835hai->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc835hai->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc835hai->mutex);

	return ret;
}

static int __sc835hai_power_on(struct sc835hai *sc835hai)
{
	int ret;
	struct device *dev = &sc835hai->client->dev;

	if (!IS_ERR_OR_NULL(sc835hai->pins_default)) {
		ret = pinctrl_select_state(sc835hai->pinctrl, sc835hai->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc835hai->xvclk, SC835HAI_XVCLK_FREQ_24M);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate 24MHz\n");
	if (clk_get_rate(sc835hai->xvclk) != SC835HAI_XVCLK_FREQ_24M)
		dev_warn(dev, "xvclk mismatched\n");
	ret = clk_prepare_enable(sc835hai->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		goto err_clk;
	}

	cam_sw_regulator_bulk_init(sc835hai->cam_sw_info,
				   SC835HAI_NUM_SUPPLIES, sc835hai->supplies);

	if (sc835hai->is_thunderboot)
		return 0;

	if (!IS_ERR(sc835hai->pwdn_gpio))
		gpiod_set_value_cansleep(sc835hai->pwdn_gpio, 1);

	usleep_range(4000, 6000);
	if (!IS_ERR(sc835hai->reset_gpio))
		gpiod_set_value_cansleep(sc835hai->reset_gpio, 0);

	usleep_range(4000, 6000);

	ret = regulator_bulk_enable(SC835HAI_NUM_SUPPLIES, sc835hai->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc835hai->reset_gpio))
		gpiod_set_value_cansleep(sc835hai->reset_gpio, 1);

	usleep_range(4000, 6000);

	if (!IS_ERR(sc835hai->pwdn_gpio))
		gpiod_set_value_cansleep(sc835hai->pwdn_gpio, 1);
	return 0;
err_clk:
	if (!IS_ERR(sc835hai->reset_gpio))
		gpiod_direction_output(sc835hai->reset_gpio, 0);
disable_clk:
	clk_disable_unprepare(sc835hai->xvclk);

	return ret;
}

static void __sc835hai_power_off(struct sc835hai *sc835hai)
{
	int ret;
	struct device *dev = &sc835hai->client->dev;

	clk_disable_unprepare(sc835hai->xvclk);
	if (sc835hai->is_thunderboot) {
		if (sc835hai->is_first_streamoff) {
			sc835hai->is_thunderboot = false;
			sc835hai->is_first_streamoff = false;
		} else {
			return;
		}
	}
	if (!IS_ERR(sc835hai->reset_gpio))
		gpiod_direction_output(sc835hai->reset_gpio, 0);

	if (!IS_ERR_OR_NULL(sc835hai->pins_sleep)) {
		ret = pinctrl_select_state(sc835hai->pinctrl,
					   sc835hai->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(sc835hai->pwdn_gpio))
		gpiod_direction_output(sc835hai->pwdn_gpio, 0);
	regulator_bulk_disable(SC835HAI_NUM_SUPPLIES, sc835hai->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int sc835hai_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc835hai *sc835hai = to_sc835hai(sd);

	cam_sw_prepare_wakeup(sc835hai->cam_sw_info, dev);

	usleep_range(4000, 5000);
	cam_sw_write_array(sc835hai->cam_sw_info);

	if (__v4l2_ctrl_handler_setup(&sc835hai->ctrl_handler))
		dev_err(dev, "__v4l2_ctrl_handler_setup fail!");

	if (sc835hai->has_init_exp && sc835hai->cur_mode != NO_HDR) {	// hdr mode
		ret = sc835hai_ioctl(&sc835hai->subdev, PREISP_CMD_SET_HDRAE_EXP,
				     &sc835hai->cam_sw_info->hdr_ae);
		if (ret) {
			dev_err(&sc835hai->client->dev, "set exp fail in hdr mode\n");
			return ret;
		}
	}
	return 0;
}

static int sc835hai_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc835hai *sc835hai = to_sc835hai(sd);

	cam_sw_write_array_cb_init(sc835hai->cam_sw_info, client,
				   (void *)sc835hai->cur_mode->reg_list,
				   (sensor_write_array)sc835hai_write_array);
	cam_sw_prepare_sleep(sc835hai->cam_sw_info);

	return 0;
}
#else
#define sc835hai_resume NULL
#define sc835hai_suspend NULL
#endif

static int sc835hai_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc835hai *sc835hai = to_sc835hai(sd);

	return __sc835hai_power_on(sc835hai);
}

static int sc835hai_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc835hai *sc835hai = to_sc835hai(sd);

	__sc835hai_power_off(sc835hai);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc835hai_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc835hai_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc835hai->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc835hai->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc835hai_enum_frame_interval(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);

	if (fie->index >= sc835hai->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH_3840 3840
#define DST_HEIGHT_2160 2160
#define DST_WIDTH_1920 1920
#define DST_HEIGHT_1080 1080

static int sc835hai_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_selection *sel)
{
	struct sc835hai *sc835hai = to_sc835hai(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		if (sc835hai->cur_mode->width == 3856) {
			sel->r.left = CROP_START(sc835hai->cur_mode->width, DST_WIDTH_3840);
			sel->r.width = DST_WIDTH_3840;
			sel->r.top = CROP_START(sc835hai->cur_mode->height, DST_HEIGHT_2160);
			sel->r.height = DST_HEIGHT_2160;
		} else if (sc835hai->cur_mode->width == 1944) {
			sel->r.left = CROP_START(sc835hai->cur_mode->width, DST_WIDTH_1920);
			sel->r.width = DST_WIDTH_1920;
			sel->r.top = CROP_START(sc835hai->cur_mode->height, DST_HEIGHT_1080);
			sel->r.height = DST_HEIGHT_1080;
		} else {
			sel->r.left = CROP_START(sc835hai->cur_mode->width,
						 sc835hai->cur_mode->width);
			sel->r.width = sc835hai->cur_mode->width;
			sel->r.top = CROP_START(sc835hai->cur_mode->height,
						sc835hai->cur_mode->height);
			sel->r.height = sc835hai->cur_mode->height;
		}
		return 0;
	}
	return -EINVAL;
}

static const struct dev_pm_ops sc835hai_pm_ops = {
	SET_RUNTIME_PM_OPS(sc835hai_runtime_suspend,
	sc835hai_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(sc835hai_suspend, sc835hai_resume)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc835hai_internal_ops = {
	.open = sc835hai_open,
};
#endif

static const struct v4l2_subdev_core_ops sc835hai_core_ops = {
	.s_power = sc835hai_s_power,
	.ioctl = sc835hai_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc835hai_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc835hai_video_ops = {
	.s_stream = sc835hai_s_stream,
	.g_frame_interval = sc835hai_g_frame_interval,
	.s_frame_interval = sc835hai_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc835hai_pad_ops = {
	.enum_mbus_code = sc835hai_enum_mbus_code,
	.enum_frame_size = sc835hai_enum_frame_sizes,
	.enum_frame_interval = sc835hai_enum_frame_interval,
	.get_fmt = sc835hai_get_fmt,
	.set_fmt = sc835hai_set_fmt,
	.get_selection = sc835hai_get_selection,
	.get_mbus_config = sc835hai_g_mbus_config,
};

static const struct v4l2_subdev_ops sc835hai_subdev_ops = {
	.core	= &sc835hai_core_ops,
	.video	= &sc835hai_video_ops,
	.pad	= &sc835hai_pad_ops,
};

static void sc835hai_modify_fps_info(struct sc835hai *sc835hai)
{
	const struct sc835hai_mode *mode = sc835hai->cur_mode;

	sc835hai->cur_fps.denominator = mode->max_fps.denominator * sc835hai->cur_vts /
					mode->vts_def;
}

static int sc835hai_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc835hai *sc835hai = container_of(ctrl->handler,
				    struct sc835hai, ctrl_handler);
	struct i2c_client *client = sc835hai->client;
	s64 max;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc835hai->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sc835hai->exposure,
					 sc835hai->exposure->minimum, max,
					 sc835hai->exposure->step,
					 sc835hai->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (sc835hai->cur_mode->hdr_mode != NO_HDR)
			goto out_ctrl;
		ret = sc835hai_write_reg(sc835hai->client,
					 SC835HAI_REG_EXP_LONG_H,
					 SC835HAI_REG_VALUE_08BIT,
					 SC835HAI_FETCH_EXP_H(ctrl->val));
		ret |= sc835hai_write_reg(sc835hai->client,
					  SC835HAI_REG_EXP_LONG_M,
					  SC835HAI_REG_VALUE_08BIT,
					  SC835HAI_FETCH_EXP_M(ctrl->val));
		ret |= sc835hai_write_reg(sc835hai->client,
					  SC835HAI_REG_EXP_LONG_L,
					  SC835HAI_REG_VALUE_08BIT,
					  SC835HAI_FETCH_EXP_L(ctrl->val));

		dev_dbg(&client->dev, "set exposure 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (sc835hai->cur_mode->hdr_mode != NO_HDR)
			goto out_ctrl;
		ret = sc835hai_set_gain_reg(sc835hai, ctrl->val, SC835HAI_LGAIN);
		break;
	case V4L2_CID_VBLANK:
		ret = sc835hai_write_reg(sc835hai->client, SC835HAI_REG_VTS,
					 SC835HAI_REG_VALUE_16BIT,
					 ctrl->val + sc835hai->cur_mode->height);
		if (!ret)
			sc835hai->cur_vts = ctrl->val + sc835hai->cur_mode->height;
		sc835hai_modify_fps_info(sc835hai);
		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc835hai_enable_test_pattern(sc835hai, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc835hai_read_reg(sc835hai->client, SC835HAI_FLIP_REG,
					SC835HAI_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC835HAI_MIRROR_MASK;
		else
			val &= ~SC835HAI_MIRROR_MASK;
		ret |= sc835hai_write_reg(sc835hai->client, SC835HAI_FLIP_REG,
					  SC835HAI_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc835hai_read_reg(sc835hai->client, SC835HAI_FLIP_REG,
					SC835HAI_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC835HAI_FLIP_MASK;
		else
			val &= ~SC835HAI_FLIP_MASK;
		ret |= sc835hai_write_reg(sc835hai->client, SC835HAI_FLIP_REG,
					  SC835HAI_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

out_ctrl:
	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc835hai_ctrl_ops = {
	.s_ctrl = sc835hai_set_ctrl,
};

static int sc835hai_initialize_controls(struct sc835hai *sc835hai)
{
	const struct sc835hai_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u64 pixel_rate = 0;
	u32 h_blank;
	int ret;

	handler = &sc835hai->ctrl_handler;
	mode = sc835hai->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc835hai->mutex;

	sc835hai->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			      V4L2_CID_LINK_FREQ, 0, 0, link_freq_items);
	v4l2_ctrl_s_ctrl(sc835hai->link_freq, mode->mipi_freq_idx);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * SC835HAI_4LANES;
	sc835hai->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
			       V4L2_CID_PIXEL_RATE, 0, SC835HAI_MAX_PIXEL_RATE,
			       1, pixel_rate);

	h_blank = mode->hts_def - mode->width;
	sc835hai->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					     h_blank, h_blank, 1, h_blank);
	if (sc835hai->hblank)
		sc835hai->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	sc835hai->vblank = v4l2_ctrl_new_std(handler, &sc835hai_ctrl_ops,
					     V4L2_CID_VBLANK, vblank_def,
					     SC835HAI_VTS_MAX - mode->height,
					     1, vblank_def);

	exposure_max = mode->vts_def - 4;	/*vts_def  0x08ca=2250*/
	sc835hai->exposure = v4l2_ctrl_new_std(handler, &sc835hai_ctrl_ops,
					       V4L2_CID_EXPOSURE, SC835HAI_EXPOSURE_MIN,
					       exposure_max, SC835HAI_EXPOSURE_STEP,
					       mode->exp_def);	/*exp_def 0x08c0=2240*/

	sc835hai->anal_a_gain = v4l2_ctrl_new_std(handler, &sc835hai_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, SC835HAI_GAIN_MIN,
				SC835HAI_GAIN_MAX, SC835HAI_GAIN_STEP,
				SC835HAI_GAIN_DEFAULT);

	sc835hai->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				 &sc835hai_ctrl_ops, V4L2_CID_TEST_PATTERN,
				 ARRAY_SIZE(sc835hai_test_pattern_menu) - 1,
				 0, 0, sc835hai_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &sc835hai_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &sc835hai_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc835hai->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc835hai->subdev.ctrl_handler = handler;
	sc835hai->has_init_exp = false;
	sc835hai->cur_fps = mode->max_fps;
	sc835hai->cur_vts = mode->vts_def;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc835hai_check_sensor_id(struct sc835hai *sc835hai,
				    struct i2c_client *client)
{
	struct device *dev = &sc835hai->client->dev;
	u32 id = 0;
	int ret;

	if (sc835hai->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}
	ret = sc835hai_read_reg(client, SC835HAI_REG_CHIP_ID,
				SC835HAI_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected sc835hai id(%06x)\n", CHIP_ID);

	return 0;
}

static int sc835hai_configure_regulators(struct sc835hai *sc835hai)
{
	unsigned int i;

	for (i = 0; i < SC835HAI_NUM_SUPPLIES; i++)
		sc835hai->supplies[i].supply = sc835hai_supply_names[i];

	return devm_regulator_bulk_get(&sc835hai->client->dev,
				       SC835HAI_NUM_SUPPLIES,
				       sc835hai->supplies);
}

static int sc835hai_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc835hai *sc835hai;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc835hai = devm_kzalloc(dev, sizeof(*sc835hai), GFP_KERNEL);
	if (!sc835hai)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc835hai->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc835hai->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc835hai->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc835hai->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}

	sc835hai->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	sc835hai->client = client;
	sc835hai->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < sc835hai->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			sc835hai->cur_mode = &supported_modes[i];
			break;
		}
	}

	sc835hai->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc835hai->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc835hai->reset_gpio = devm_gpiod_get(dev, "reset",
					      sc835hai->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sc835hai->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc835hai->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
					     sc835hai->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sc835hai->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn_gpio\n");

	sc835hai->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc835hai->pinctrl)) {
		sc835hai->pins_default =
			pinctrl_lookup_state(sc835hai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc835hai->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		sc835hai->pins_sleep =
			pinctrl_lookup_state(sc835hai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc835hai->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}

	ret = sc835hai_configure_regulators(sc835hai);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc835hai->mutex);

	sd = &sc835hai->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc835hai_subdev_ops);
	ret = sc835hai_initialize_controls(sc835hai);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc835hai_power_on(sc835hai);
	if (ret)
		goto err_free_handler;

	ret = sc835hai_check_sensor_id(sc835hai, client);
	if (ret)
		goto err_power_off;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc835hai_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc835hai->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc835hai->pad);
	if (ret < 0)
		goto err_power_off;
#endif
	if (!sc835hai->cam_sw_info) {
		sc835hai->cam_sw_info = cam_sw_init();
		cam_sw_clk_init(sc835hai->cam_sw_info, sc835hai->xvclk, SC835HAI_XVCLK_FREQ_24M);
		cam_sw_reset_pin_init(sc835hai->cam_sw_info, sc835hai->reset_gpio, 0);
		cam_sw_pwdn_pin_init(sc835hai->cam_sw_info, sc835hai->pwdn_gpio, 1);
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc835hai->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc835hai->module_index, facing,
		 SC835HAI_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (sc835hai->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc835hai_power_off(sc835hai);
err_free_handler:
	v4l2_ctrl_handler_free(&sc835hai->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc835hai->mutex);

	return ret;
}

static int sc835hai_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc835hai *sc835hai = to_sc835hai(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc835hai->ctrl_handler);
	mutex_destroy(&sc835hai->mutex);

	cam_sw_deinit(sc835hai->cam_sw_info);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc835hai_power_off(sc835hai);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc835hai_of_match[] = {
	{ .compatible = "smartsens,sc835hai" },
	{},
};
MODULE_DEVICE_TABLE(of, sc835hai_of_match);
#endif

static const struct i2c_device_id sc835hai_match_id[] = {
	{ "smartsens,sc835hai", 0 },
	{ },
};

static struct i2c_driver sc835hai_i2c_driver = {
	.driver = {
		.name = SC835HAI_NAME,
		.pm = &sc835hai_pm_ops,
		.of_match_table = of_match_ptr(sc835hai_of_match),
	},
	.probe		= sc835hai_probe,
	.remove		= sc835hai_remove,
	.id_table	= sc835hai_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc835hai_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc835hai_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens,sc835hai sensor driver");
MODULE_LICENSE("GPL");
