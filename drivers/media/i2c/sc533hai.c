// SPDX-License-Identifier: GPL-2.0
/*
 * sc533hai driver
 *
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
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
#include <linux/soc/rockchip/rockchip_thunderboot_service.h>

#define DRIVER_VERSION				KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN			V4L2_CID_GAIN
#endif

#define SC533HAI_LANES				2

#define SC533HAI_LINK_FREQ_972M			486000000 // 972Mbps

#define SC533HAI_PIXEL_RATE_486M_10BIT		(SC533HAI_LINK_FREQ_972M * 2 * SC533HAI_LANES / 10)

#define SC533HAI_XVCLK_FREQ			24000000

#define SC533HAI_CHIP_ID			0xce7c
#define SC533HAI_REG_CHIP_ID			0x3107

#define SC533HAI_REG_CTRL_MODE			0x0100
#define SC533HAI_MODE_SW_STANDBY		0x0
#define SC533HAI_MODE_STREAMING			BIT(0)

#define SC533HAI_REG_EXPOSURE_H			0x3e00
#define SC533HAI_REG_EXPOSURE_M			0x3e01
#define SC533HAI_REG_EXPOSURE_L			0x3e02

#define	SC533HAI_EXPOSURE_MIN			2
#define	SC533HAI_EXPOSURE_STEP			1

#define SC533HAI_REG_DIG_GAIN			0x3e06
#define SC533HAI_REG_DIG_FINE_GAIN		0x3e07
#define SC533HAI_REG_ANA_GAIN			0x3e08
#define SC533HAI_REG_ANA_FINE_GAIN		0x3e09

#define SC533HAI_GAIN_MIN			0x20
#define SC533HAI_GAIN_MAX			(42230)  //83.79 * 15.75 * 32
#define SC533HAI_GAIN_STEP			1
#define SC533HAI_GAIN_DEFAULT			0x80

#define SC533HAI_REG_VTS_H			0x320e
#define SC533HAI_REG_VTS_L			0x320f
#define SC533HAI_VTS_MAX			0x7fff

#define SC533HAI_SOFTWARE_RESET_REG		0x0103

//group hold
#define SC533HAI_GROUP_UPDATE_ADDRESS		0x3812
#define SC533HAI_GROUP_UPDATE_START_DATA	0x00
#define SC533HAI_GROUP_UPDATE_LAUNCH		0x30

#define SC533HAI_FLIP_MIRROR_REG		0x3221
#define SC533HAI_FLIP_MASK			0x60
#define SC533HAI_MIRROR_MASK			0x06

#define REG_NULL				0xFFFF

#define SC533HAI_REG_VALUE_08BIT		1
#define SC533HAI_REG_VALUE_16BIT		2
#define SC533HAI_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT		"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP		"rockchip,camera_sleep"

#define SC533HAI_NAME				"sc533hai"

#define SC533HAI_FETCH_EXP_H(VAL)		(((VAL) >> 12) & 0xF)
#define SC533HAI_FETCH_EXP_M(VAL)		(((VAL) >> 4) & 0xFF)
#define SC533HAI_FETCH_EXP_L(VAL)		(((VAL) & 0xF) << 4)

static const char * const sc533hai_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define sc533hai_NUM_SUPPLIES ARRAY_SIZE(sc533hai_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc533hai_mode {
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
	u32 vc[PAD_MAX];
};

struct sc533hai {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[sc533hai_NUM_SUPPLIES];

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
	struct v4l2_ctrl	*hflip;
	struct v4l2_ctrl	*vflip;
	struct v4l2_fract	cur_fps;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct sc533hai_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	u32			cur_vts;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	struct rk_tb_client     tb_cl;
};

#define to_sc533hai(sd) container_of(sd, struct sc533hai, subdev)

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 972Mbps, 2lane
 * Cleaned_0x0f_SC533HAI_raw_MIPI_24Minput_2Lane_10bit_972Mbps_2880x1616_30fps.ini
 */
static const struct regval sc533hai_linear_10_2880x1616_regs[] = {
	{0x3105, 0x32},
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x302c, 0x0c},
	{0x302c, 0x00},
	{0x3105, 0x12},
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
	{0x23bc, 0x04},
	{0x23bd, 0x08},
	{0x23be, 0x04},
	{0x23bf, 0x78},
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
	{0x301f, 0x0f},
	{0x302c, 0x00},
	{0x30b0, 0x01},
	{0x30b8, 0x44},
	{0x3200, 0x00},
	{0x3201, 0xa0},
	{0x3202, 0x00},
	{0x3203, 0x96},
	{0x3204, 0x0b},
	{0x3205, 0xe7},
	{0x3206, 0x06},
	{0x3207, 0xf1},
	{0x3208, 0x0b},
	{0x3209, 0x40},
	{0x320a, 0x06},
	{0x320b, 0x50},
	{0x320c, 0x03},
	{0x320d, 0xc0},
	{0x320e, 0x07},
	{0x320f, 0x53},
	{0x3210, 0x00},
	{0x3211, 0x04},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0xc0},
	{0x3250, 0x00},
	{0x3271, 0x10},
	{0x327f, 0x3f},
	{0x32e0, 0x00},
	{0x3301, 0x12},
	{0x3304, 0x50},
	{0x3305, 0x00},
	{0x3306, 0x70},
	{0x3308, 0x18},
	{0x3309, 0xb0},
	{0x330a, 0x01},
	{0x330b, 0x20},
	{0x331e, 0x39},
	{0x331f, 0x99},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x3364, 0x5e},
	{0x338f, 0xa0},
	{0x3393, 0x18},
	{0x3394, 0x2c},
	{0x3395, 0x3c},
	{0x3399, 0x12},
	{0x339a, 0x16},
	{0x339b, 0x1e},
	{0x339c, 0x3e},
	{0x33ac, 0x0c},
	{0x33ad, 0x2c},
	{0x33ae, 0x30},
	{0x33af, 0x90},
	{0x33b0, 0x0f},
	{0x33b2, 0x24},
	{0x33b3, 0x10},
	{0x33f8, 0x00},
	{0x33f9, 0x70},
	{0x33fa, 0x00},
	{0x33fb, 0x70},
	{0x349f, 0x03},
	{0x34a8, 0x10},
	{0x34a9, 0x10},
	{0x34aa, 0x01},
	{0x34ab, 0x20},
	{0x34ac, 0x01},
	{0x34ad, 0x20},
	{0x34f9, 0x12},
	{0x3632, 0x6d},
	{0x3633, 0x4d},
	{0x363a, 0x80},
	{0x363b, 0x57},
	{0x363c, 0xd8},
	{0x363d, 0x40},
	{0x3670, 0x41},
	{0x3671, 0x31},
	{0x3672, 0x31},
	{0x3673, 0x04},
	{0x3674, 0x08},
	{0x3675, 0x04},
	{0x3676, 0x18},
	{0x367e, 0x69},
	{0x367f, 0x6d},
	{0x3680, 0x8d},
	{0x3681, 0x04},
	{0x3682, 0x08},
	{0x3683, 0x04},
	{0x3684, 0x78},
	{0x3685, 0x80},
	{0x3686, 0x80},
	{0x3687, 0x83},
	{0x3688, 0x82},
	{0x3689, 0x85},
	{0x368a, 0x8b},
	{0x368b, 0x97},
	{0x368c, 0xbf},
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
	{0x36ea, 0x1b},
	{0x36eb, 0x45},
	{0x36ec, 0x4b},
	{0x36ed, 0x08},
	{0x370f, 0x13},
	{0x3721, 0x6c},
	{0x3722, 0x8b},
	{0x3724, 0xc1},
	{0x3726, 0x01},
	{0x3727, 0x24},
	{0x3729, 0xb4},
	{0x37b0, 0x7f},
	{0x37b1, 0x7f},
	{0x37b2, 0x73},
	{0x37b3, 0x04},
	{0x37b4, 0x08},
	{0x37b5, 0x04},
	{0x37b6, 0x38},
	{0x37b7, 0x11},
	{0x37b8, 0x11},
	{0x37b9, 0x00},
	{0x37ba, 0x94},
	{0x37bb, 0xd4},
	{0x37bc, 0x84},
	{0x37bd, 0x00},
	{0x37be, 0x08},
	{0x37bf, 0x04},
	{0x37c0, 0x00},
	{0x37c1, 0x04},
	{0x37c2, 0x18},
	{0x37c3, 0x04},
	{0x37c4, 0x3c},
	{0x37fa, 0x1b},
	{0x37fb, 0x55},
	{0x37fc, 0x19},
	{0x37fd, 0x0a},
	{0x3900, 0x05},
	{0x3903, 0x60},
	{0x3905, 0x0d},
	{0x391a, 0x60},
	{0x391b, 0x40},
	{0x391c, 0x26},
	{0x391d, 0x00},
	{0x3926, 0xe0},
	{0x3933, 0x80},
	{0x3934, 0x06},
	{0x3935, 0x00},
	{0x3936, 0x72},
	{0x3937, 0x71},
	{0x3938, 0x75},
	{0x3939, 0x0f},
	{0x393a, 0xf3},
	{0x393b, 0x0f},
	{0x393c, 0xd8},
	{0x393f, 0x80},
	{0x3940, 0x0b},
	{0x3941, 0x00},
	{0x3942, 0x0b},
	{0x3943, 0x7e},
	{0x3944, 0x7f},
	{0x3945, 0x7f},
	{0x3946, 0x7e},
	{0x39dd, 0x00},
	{0x39de, 0x08},
	{0x39e7, 0x04},
	{0x39e8, 0x04},
	{0x39e9, 0x80},
	{0x3e00, 0x00},
	{0x3e01, 0x74},
	{0x3e02, 0xb0},
	{0x3e03, 0x0b},
	{0x3e08, 0x00},
	{0x3e16, 0x01},
	{0x3e17, 0x54},
	{0x3e18, 0x01},
	{0x3e19, 0x54},
	{0x3e1b, 0x29},
	{0x4402, 0x11},
	{0x450a, 0x80},
	{0x450d, 0x0a},
	{0x4800, 0x24},
	{0x480f, 0x03},
	{0x4837, 0x20},
	{0x5000, 0x26},
	{0x5780, 0x76},
	{0x5784, 0x10},
	{0x5785, 0x08},
	{0x5787, 0x0a},
	{0x5788, 0x0a},
	{0x5789, 0x08},
	{0x578a, 0x0a},
	{0x578b, 0x0a},
	{0x578c, 0x08},
	{0x578d, 0x41},
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
	{0x58c0, 0x30},
	{0x58c1, 0x28},
	{0x58c2, 0x20},
	{0x58c3, 0x30},
	{0x58c4, 0x28},
	{0x58c5, 0x20},
	{0x58c6, 0x3c},
	{0x58c7, 0x30},
	{0x58c8, 0x28},
	{0x58c9, 0x3c},
	{0x58ca, 0x30},
	{0x58cb, 0x28},
	{0x36e9, 0x23},
	{0x37f9, 0x24},
	{REG_NULL, 0x00},
};

static const struct sc533hai_mode supported_modes[] = {
	{
		.width = 2880,
		.height = 1616,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0080,
		.hts_def = 0x03c0 * 4,
		.vts_def = 0x0753,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = sc533hai_linear_10_2880x1616_regs,
		.mipi_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	SC533HAI_LINK_FREQ_972M
};

/* Write registers up to 4 at a time */
static int sc533hai_write_reg(struct i2c_client *client, u16 reg,
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

static int sc533hai_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = sc533hai_write_reg(client, regs[i].addr,
					SC533HAI_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int sc533hai_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static int sc533hai_get_reso_dist(const struct sc533hai_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc533hai_mode *
sc533hai_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sc533hai_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sc533hai_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);
	const struct sc533hai_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sc533hai->mutex);

	mode = sc533hai_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc533hai->mutex);
		return -ENOTTY;
#endif
	} else {
		sc533hai->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc533hai->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc533hai->vblank, vblank_def,
					 SC533HAI_VTS_MAX - mode->height,
					 1, vblank_def);
		sc533hai->cur_fps = mode->max_fps;
	}

	mutex_unlock(&sc533hai->mutex);

	return 0;
}

static int sc533hai_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);
	const struct sc533hai_mode *mode = sc533hai->cur_mode;

	mutex_lock(&sc533hai->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc533hai->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc533hai->mutex);

	return 0;
}

static int sc533hai_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sc533hai->cur_mode->bus_fmt;

	return 0;
}

static int sc533hai_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int sc533hai_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);
	const struct sc533hai_mode *mode = sc533hai->cur_mode;

	if (sc533hai->streaming)
		fi->interval = sc533hai->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int sc533hai_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_config *config)
{
	u32 val = 1 << (SC533HAI_LANES - 1) |
		  V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void sc533hai_get_module_inf(struct sc533hai *sc533hai,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC533HAI_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc533hai->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc533hai->len_name, sizeof(inf->base.lens));
}

static int sc533hai_get_gain_reg(u32 total_gain, u32 *again, u32 *again_fine,
				u32 *dgain, u32 *dgain_fine)
{
	int ret = 0;
	uint32_t gain_factor = 0;

	if (total_gain < SC533HAI_GAIN_MIN)
		total_gain = SC533HAI_GAIN_MIN;
	else if (total_gain > SC533HAI_GAIN_MAX)
		total_gain = SC533HAI_GAIN_MAX;

	gain_factor = total_gain * 1000 / 32;
	if (gain_factor < 2000) {                    /* 1 ~ 2 gain*/
		*again = 0x00;
		*again_fine = gain_factor * 32 / 1000;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (gain_factor < 2660) {             /* 2 ~ 2.660 gain*/
		*again = 0x01;
		*again_fine = gain_factor * 32 / 2000;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (gain_factor < 2660 * 2) {         /* 2.660 ~ 5.320 gain*/
		*again = 0x80;
		*again_fine = gain_factor * 32 / 2660;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (gain_factor < 2660 * 4) {         /* 2.660 ~ 10.640 gain*/
		*again = 0x81;
		*again_fine = gain_factor * 32 / 2660 / 2;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (gain_factor < 2660 * 8) {         /* 10.640 ~ 21.280 gain*/
		*again = 0x83;
		*again_fine = gain_factor * 32 / 2660 / 4;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (gain_factor < 2390 * 16) {        /* 21.280 ~ 42.560 gain*/
		*again = 0x87;
		*again_fine = gain_factor * 32 / 2660 / 8;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (gain_factor < 2660 * 32) {        /* 42.560 ~ 83.790 gain*/
		*again = 0x8f;
		*again_fine = gain_factor * 32 / 2660 / 16;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (gain_factor < 2660 * 64) {        /* 83.790 ~ 167.580 gain*/
		//open dgain begin  max digital gain 4X
		*again = 0x8f;
		*again_fine = 0x3f;
		*dgain = 0x00;
		*dgain_fine = gain_factor * 128 / 2660 / 32;
	} else if (gain_factor < 2660 * 128) {       /* 167.580 ~ 335.160 gain*/
		*again = 0x8f;
		*again_fine = 0x3f;
		*dgain = 0x01;
		*dgain_fine = gain_factor * 128 / 2660 / 64;
	} else if (gain_factor < 2660 * 256) {       /* 335.160 ~ 670.320 gain*/
		*again = 0x8f;
		*again_fine = 0x3f;
		*dgain = 0x03;
		*dgain_fine = gain_factor * 128 / 2660 / 128;
	} else if (gain_factor < 2660 * 512) {       /* 670.320 ~ 1319.6925 gain*/
		*again = 0x8f;
		*again_fine = 0x3f;
		*dgain = 0x07;
		*dgain_fine = gain_factor * 128 / 2660 / 256;
	} else {
		*again = 0x8f;
		*again_fine = 0x3f;
		*dgain = 0x07;
		*dgain_fine = 0xfc;
	}
	*dgain_fine = *dgain_fine / 4 * 4;

	return ret;
}

static long sc533hai_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);

	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc533hai_get_module_inf(sc533hai, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = sc533hai_write_reg(sc533hai->client, SC533HAI_REG_CTRL_MODE,
						SC533HAI_REG_VALUE_08BIT, SC533HAI_MODE_STREAMING);
		else
			ret = sc533hai_write_reg(sc533hai->client, SC533HAI_REG_CTRL_MODE,
						SC533HAI_REG_VALUE_08BIT, SC533HAI_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc533hai_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc533hai_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		ret = sc533hai_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc533hai_start_stream(struct sc533hai *sc533hai)
{
	int ret;

	if (!sc533hai->is_thunderboot) {
		ret = sc533hai_write_array(sc533hai->client, sc533hai->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&sc533hai->ctrl_handler);
		if (ret)
			return ret;
	}

	return sc533hai_write_reg(sc533hai->client, SC533HAI_REG_CTRL_MODE,
				 SC533HAI_REG_VALUE_08BIT, SC533HAI_MODE_STREAMING);
}

static int __sc533hai_stop_stream(struct sc533hai *sc533hai)
{
	sc533hai->has_init_exp = false;
	if (sc533hai->is_thunderboot) {
		sc533hai->is_first_streamoff = true;
		pm_runtime_put(&sc533hai->client->dev);
	}
	return sc533hai_write_reg(sc533hai->client, SC533HAI_REG_CTRL_MODE,
				 SC533HAI_REG_VALUE_08BIT, SC533HAI_MODE_SW_STANDBY);
}

static int __sc533hai_power_on(struct sc533hai *sc533hai);
static int sc533hai_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);
	struct i2c_client *client = sc533hai->client;
	int ret = 0;

	mutex_lock(&sc533hai->mutex);
	on = !!on;
	if (on == sc533hai->streaming)
		goto unlock_and_return;

	if (on) {
		if (sc533hai->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			sc533hai->is_thunderboot = false;
			__sc533hai_power_on(sc533hai);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc533hai_start_stream(sc533hai);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc533hai_stop_stream(sc533hai);
		pm_runtime_put(&client->dev);
	}

	sc533hai->streaming = on;

unlock_and_return:
	mutex_unlock(&sc533hai->mutex);

	return ret;
}

static int sc533hai_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);
	struct i2c_client *client = sc533hai->client;
	int ret = 0;

	mutex_lock(&sc533hai->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc533hai->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!sc533hai->is_thunderboot) {
			ret |= sc533hai_write_reg(sc533hai->client,
						 SC533HAI_SOFTWARE_RESET_REG,
						 SC533HAI_REG_VALUE_08BIT,
						 0x01);
			if (ret) {
				v4l2_err(sd, "could not set soft rst registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
			usleep_range(100, 200);
		}

		sc533hai->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc533hai->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc533hai->mutex);

	return ret;
}

static int __sc533hai_power_on(struct sc533hai *sc533hai)
{
	int ret;
	struct device *dev = &sc533hai->client->dev;

	if (!IS_ERR_OR_NULL(sc533hai->pins_default)) {
		ret = pinctrl_select_state(sc533hai->pinctrl,
					   sc533hai->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc533hai->xvclk, SC533HAI_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (27MHz)\n");
	if (clk_get_rate(sc533hai->xvclk) != SC533HAI_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sc533hai->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (sc533hai->is_thunderboot)
		return 0;

	if (!IS_ERR(sc533hai->reset_gpio))
		gpiod_set_value_cansleep(sc533hai->reset_gpio, 0);

	ret = regulator_bulk_enable(sc533hai_NUM_SUPPLIES, sc533hai->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc533hai->reset_gpio))
		gpiod_set_value_cansleep(sc533hai->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(sc533hai->pwdn_gpio))
		gpiod_set_value_cansleep(sc533hai->pwdn_gpio, 1);

	usleep_range(4000, 5000);
	return 0;

disable_clk:
	clk_disable_unprepare(sc533hai->xvclk);

	return ret;
}

static void __sc533hai_power_off(struct sc533hai *sc533hai)
{
	int ret;
	struct device *dev = &sc533hai->client->dev;

	clk_disable_unprepare(sc533hai->xvclk);
	if (sc533hai->is_thunderboot) {
		if (sc533hai->is_first_streamoff) {
			sc533hai->is_thunderboot = false;
			sc533hai->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(sc533hai->pwdn_gpio))
		gpiod_set_value_cansleep(sc533hai->pwdn_gpio, 0);
	clk_disable_unprepare(sc533hai->xvclk);
	if (!IS_ERR(sc533hai->reset_gpio))
		gpiod_set_value_cansleep(sc533hai->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(sc533hai->pins_sleep)) {
		ret = pinctrl_select_state(sc533hai->pinctrl,
					   sc533hai->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(sc533hai_NUM_SUPPLIES, sc533hai->supplies);
}

static int sc533hai_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc533hai *sc533hai = to_sc533hai(sd);

	return __sc533hai_power_on(sc533hai);
}

static int sc533hai_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc533hai *sc533hai = to_sc533hai(sd);

	__sc533hai_power_off(sc533hai);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc533hai_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc533hai *sc533hai = to_sc533hai(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc533hai_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc533hai->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc533hai->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc533hai_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = NO_HDR;
	return 0;
}

static const struct dev_pm_ops sc533hai_pm_ops = {
	SET_RUNTIME_PM_OPS(sc533hai_runtime_suspend,
	sc533hai_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc533hai_internal_ops = {
	.open = sc533hai_open,
};
#endif

static const struct v4l2_subdev_core_ops sc533hai_core_ops = {
	.s_power = sc533hai_s_power,
	.ioctl = sc533hai_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc533hai_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc533hai_video_ops = {
	.s_stream = sc533hai_s_stream,
	.g_frame_interval = sc533hai_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc533hai_pad_ops = {
	.enum_mbus_code = sc533hai_enum_mbus_code,
	.enum_frame_size = sc533hai_enum_frame_sizes,
	.enum_frame_interval = sc533hai_enum_frame_interval,
	.get_fmt = sc533hai_get_fmt,
	.set_fmt = sc533hai_set_fmt,
	.get_mbus_config = sc533hai_g_mbus_config,
};

static const struct v4l2_subdev_ops sc533hai_subdev_ops = {
	.core	= &sc533hai_core_ops,
	.video	= &sc533hai_video_ops,
	.pad	= &sc533hai_pad_ops,
};

static void sc533hai_modify_fps_info(struct sc533hai *sc533hai)
{
	const struct sc533hai_mode *mode = sc533hai->cur_mode;

	sc533hai->cur_fps.denominator = mode->max_fps.denominator * sc533hai->cur_vts /
				       mode->vts_def;
}
static int sc533hai_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc533hai *sc533hai = container_of(ctrl->handler,
					       struct sc533hai, ctrl_handler);
	struct i2c_client *client = sc533hai->client;
	s64 max;
	u32 again = 0, again_fine = 0, dgain = 0, dgain_fine = 0;
	int ret = 0;
	u32 vts_l = 0, vts_h = 0;
	u32 val = 0, vts = 0;
	u64 delay_time = 0;
	u32 cur_fps = 0;
	u32 def_fps = 0;
	u32 denominator = 0;
	u32 numerator = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc533hai->cur_mode->height + ctrl->val - 8;
		__v4l2_ctrl_modify_range(sc533hai->exposure,
					 sc533hai->exposure->minimum, max,
					 sc533hai->exposure->step,
					 sc533hai->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		val = ctrl->val;
		ret = sc533hai_write_reg(sc533hai->client,
					SC533HAI_REG_EXPOSURE_H,
					SC533HAI_REG_VALUE_08BIT,
					SC533HAI_FETCH_EXP_H(val));
		ret |= sc533hai_write_reg(sc533hai->client,
					SC533HAI_REG_EXPOSURE_M,
					SC533HAI_REG_VALUE_08BIT,
					SC533HAI_FETCH_EXP_M(val));
		ret |= sc533hai_write_reg(sc533hai->client,
					 SC533HAI_REG_EXPOSURE_L,
					 SC533HAI_REG_VALUE_08BIT,
					 SC533HAI_FETCH_EXP_L(val));

		dev_dbg(&client->dev, "set exposure 0x%x\n", val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		sc533hai_get_gain_reg(ctrl->val, &again, &again_fine, &dgain, &dgain_fine);
		ret = sc533hai_write_reg(sc533hai->client,
					 SC533HAI_REG_DIG_GAIN,
					 SC533HAI_REG_VALUE_08BIT,
					 dgain);
		ret |= sc533hai_write_reg(sc533hai->client,
					 SC533HAI_REG_DIG_FINE_GAIN,
					 SC533HAI_REG_VALUE_08BIT,
					 dgain_fine);
		ret |= sc533hai_write_reg(sc533hai->client,
					 SC533HAI_REG_ANA_GAIN,
					 SC533HAI_REG_VALUE_08BIT,
					 again);
		ret |= sc533hai_write_reg(sc533hai->client,
					 SC533HAI_REG_ANA_FINE_GAIN,
					 SC533HAI_REG_VALUE_08BIT,
					 again_fine);

		dev_dbg(&client->dev,
			"total_gain:%x again 0x%x, again_fine 0x%x, dgain 0x%x, dgain_fine 0x%x\n",
			ctrl->val, again, again_fine, dgain, dgain_fine);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + sc533hai->cur_mode->height;
		ret = sc533hai_write_reg(sc533hai->client,
					SC533HAI_REG_VTS_H,
					SC533HAI_REG_VALUE_08BIT,
					(vts >> 8) & 0x7f);
		ret |= sc533hai_write_reg(sc533hai->client,
					 SC533HAI_REG_VTS_L,
					 SC533HAI_REG_VALUE_08BIT,
					 vts & 0xff);
		sc533hai->cur_vts = vts;
		if (sc533hai->cur_vts != sc533hai->cur_mode->vts_def)
			sc533hai_modify_fps_info(sc533hai);
		break;
	case V4L2_CID_HFLIP:
		ret = sc533hai_read_reg(sc533hai->client, SC533HAI_FLIP_MIRROR_REG,
				       SC533HAI_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC533HAI_MIRROR_MASK;
		else
			val &= ~SC533HAI_MIRROR_MASK;
		ret |= sc533hai_write_reg(sc533hai->client, SC533HAI_FLIP_MIRROR_REG,
					 SC533HAI_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc533hai_read_reg(sc533hai->client,
				       SC533HAI_FLIP_MIRROR_REG,
				       SC533HAI_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		ret = sc533hai_read_reg(sc533hai->client, SC533HAI_REG_VTS_H,
				       SC533HAI_REG_VALUE_08BIT, &vts_h);
		ret |= sc533hai_read_reg(sc533hai->client, SC533HAI_REG_VTS_L,
				       SC533HAI_REG_VALUE_08BIT, &vts_l);
		sc533hai->cur_vts = (vts_h << 8) | vts_l;
		denominator = sc533hai->cur_mode->max_fps.denominator;
		numerator = sc533hai->cur_mode->max_fps.numerator;
		def_fps = denominator / numerator;
		if (sc533hai->cur_vts == 0) {
			dev_err(&client->dev, "cur vts is zero\n");
			return -EINVAL;
		}
		cur_fps = def_fps * sc533hai->cur_mode->vts_def / sc533hai->cur_vts;
		if (cur_fps > 25) {
			vts = def_fps * sc533hai->cur_mode->vts_def / 25;
			ret = sc533hai_write_reg(sc533hai->client,
						SC533HAI_REG_VTS_H,
						SC533HAI_REG_VALUE_08BIT,
						(vts >> 8) & 0x7f);
			ret |= sc533hai_write_reg(sc533hai->client,
						SC533HAI_REG_VTS_L,
						SC533HAI_REG_VALUE_08BIT,
						vts & 0xff);
			delay_time = 1000000 / 25;//one frame interval
			delay_time *= 2;
			usleep_range(delay_time, delay_time + 1000);
		}

		if (ctrl->val)
			val |= SC533HAI_FLIP_MASK;
		else
			val &= ~SC533HAI_FLIP_MASK;

		ret |= sc533hai_write_reg(sc533hai->client,
					 SC533HAI_FLIP_MIRROR_REG,
					 SC533HAI_REG_VALUE_08BIT,
					 val);
		if (cur_fps > 25) {
			usleep_range(delay_time, delay_time + 1000);
			vts = sc533hai->cur_vts;
			ret = sc533hai_write_reg(sc533hai->client,
						SC533HAI_REG_VTS_H,
						SC533HAI_REG_VALUE_08BIT,
						(vts >> 8) & 0x7f);
			ret |= sc533hai_write_reg(sc533hai->client,
						SC533HAI_REG_VTS_L,
						SC533HAI_REG_VALUE_08BIT,
						vts & 0xff);
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

static const struct v4l2_ctrl_ops sc533hai_ctrl_ops = {
	.s_ctrl = sc533hai_set_ctrl,
};

static int sc533hai_initialize_controls(struct sc533hai *sc533hai)
{
	const struct sc533hai_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &sc533hai->ctrl_handler;
	mode = sc533hai->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &sc533hai->mutex;
	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, SC533HAI_PIXEL_RATE_486M_10BIT, 1, SC533HAI_PIXEL_RATE_486M_10BIT);
	h_blank = mode->hts_def - mode->width;

	sc533hai->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (sc533hai->hblank)
		sc533hai->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	sc533hai->cur_vts = mode->vts_def;

	sc533hai->vblank = v4l2_ctrl_new_std(handler, &sc533hai_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    SC533HAI_VTS_MAX - mode->height,
					    1, vblank_def);
	exposure_max = mode->vts_def - 8;
	sc533hai->exposure = v4l2_ctrl_new_std(handler, &sc533hai_ctrl_ops,
					      V4L2_CID_EXPOSURE, SC533HAI_EXPOSURE_MIN,
					      exposure_max, SC533HAI_EXPOSURE_STEP,
					      mode->exp_def);

	sc533hai->anal_gain = v4l2_ctrl_new_std(handler, &sc533hai_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, SC533HAI_GAIN_MIN,
					       SC533HAI_GAIN_MAX, SC533HAI_GAIN_STEP,
					       SC533HAI_GAIN_DEFAULT);

	sc533hai->hflip = v4l2_ctrl_new_std(handler, &sc533hai_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	sc533hai->vflip = v4l2_ctrl_new_std(handler, &sc533hai_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc533hai->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	sc533hai->subdev.ctrl_handler = handler;
	sc533hai->has_init_exp = false;
	sc533hai->cur_fps = mode->max_fps;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

static int sc533hai_check_sensor_id(struct sc533hai *sc533hai,
				   struct i2c_client *client)
{
	struct device *dev = &sc533hai->client->dev;
	u32 id = 0;
	int ret;

	if (sc533hai->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = sc533hai_read_reg(client, SC533HAI_REG_CHIP_ID,
			       SC533HAI_REG_VALUE_16BIT, &id);
	if (id != SC533HAI_CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC%06x sensor\n", SC533HAI_CHIP_ID);

	return 0;
}

static int sc533hai_configure_regulators(struct sc533hai *sc533hai)
{
	unsigned int i;

	for (i = 0; i < sc533hai_NUM_SUPPLIES; i++)
		sc533hai->supplies[i].supply = sc533hai_supply_names[i];

	return devm_regulator_bulk_get(&sc533hai->client->dev,
				       sc533hai_NUM_SUPPLIES,
				       sc533hai->supplies);
}

static void sc533hai_get_real_mirror_flip(void *data)
{
	int val = 0, ret = 0;
	bool hflip_def = 0, vflip_def = 0;
	struct sc533hai *sc533hai = (struct sc533hai *)data;
	struct device *dev = &sc533hai->client->dev;

	ret = sc533hai_read_reg(sc533hai->client, SC533HAI_FLIP_MIRROR_REG,
			       SC533HAI_REG_VALUE_08BIT, &val);
	if (ret)
		dev_err(dev, "read mirror flip failed ret: 0x%x\n", ret);
	dev_info(dev, "get mirror flip val: 0x%x\n", val);

	hflip_def = ((val & SC533HAI_MIRROR_MASK) == SC533HAI_MIRROR_MASK) ? true : false;
	vflip_def = ((val & SC533HAI_FLIP_MASK) == SC533HAI_FLIP_MASK) ? true : false;
	v4l2_ctrl_s_ctrl(sc533hai->hflip, hflip_def);
	v4l2_ctrl_s_ctrl(sc533hai->vflip, vflip_def);
}

static int sc533hai_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc533hai *sc533hai;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc533hai = devm_kzalloc(dev, sizeof(*sc533hai), GFP_KERNEL);
	if (!sc533hai)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc533hai->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc533hai->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc533hai->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc533hai->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sc533hai->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	sc533hai->client = client;
	sc533hai->cur_mode = &supported_modes[0];

	sc533hai->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc533hai->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	if (sc533hai->is_thunderboot) {
		sc533hai->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
		if (IS_ERR(sc533hai->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		sc533hai->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
		if (IS_ERR(sc533hai->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	} else {
		sc533hai->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(sc533hai->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		sc533hai->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
		if (IS_ERR(sc533hai->pwdn_gpio))
			dev_warn(dev, "Failed to get pwdn-gpios\n");
	}

	sc533hai->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc533hai->pinctrl)) {
		sc533hai->pins_default =
			pinctrl_lookup_state(sc533hai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc533hai->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc533hai->pins_sleep =
			pinctrl_lookup_state(sc533hai->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc533hai->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc533hai_configure_regulators(sc533hai);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc533hai->mutex);

	sd = &sc533hai->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc533hai_subdev_ops);
	ret = sc533hai_initialize_controls(sc533hai);
	if (ret)
		goto err_destroy_mutex;

	if (sc533hai->is_thunderboot) {
		sc533hai->tb_cl.data = sc533hai;
		sc533hai->tb_cl.cb = sc533hai_get_real_mirror_flip;
	}
	if (sc533hai->is_thunderboot && sc533hai->tb_cl.cb)
		rk_tb_client_register_cb(&sc533hai->tb_cl);

	ret = __sc533hai_power_on(sc533hai);
	if (ret)
		goto err_free_handler;

	ret = sc533hai_check_sensor_id(sc533hai, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc533hai_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc533hai->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc533hai->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc533hai->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc533hai->module_index, facing,
		 SC533HAI_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (sc533hai->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc533hai_power_off(sc533hai);
err_free_handler:
	v4l2_ctrl_handler_free(&sc533hai->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc533hai->mutex);

	return ret;
}

static int sc533hai_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc533hai *sc533hai = to_sc533hai(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc533hai->ctrl_handler);
	mutex_destroy(&sc533hai->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc533hai_power_off(sc533hai);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc533hai_of_match[] = {
	{ .compatible = "smartsens,sc533hai" },
	{},
};
MODULE_DEVICE_TABLE(of, sc533hai_of_match);
#endif

static const struct i2c_device_id sc533hai_match_id[] = {
	{ "smartsens,sc533hai", 0 },
	{ },
};

static struct i2c_driver sc533hai_i2c_driver = {
	.driver = {
		.name = SC533HAI_NAME,
		.pm = &sc533hai_pm_ops,
		.of_match_table = of_match_ptr(sc533hai_of_match),
	},
	.probe		= sc533hai_probe,
	.remove		= sc533hai_remove,
	.id_table	= sc533hai_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc533hai_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc533hai_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens sc533hai sensor driver");
MODULE_LICENSE("GPL");
