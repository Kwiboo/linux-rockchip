// SPDX-License-Identifier: GPL-2.0
/*
 * jx_h63p driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first implementation
 */

// #define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/version.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define JX_H63P_LANES			1
#define JX_H63P_LINK_FREQ		216000000

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define JX_H63P_PIXEL_RATE		(JX_H63P_LINK_FREQ * 2 * JX_H63P_LANES / 10)

#define JX_H63P_XVCLK_FREQ		24000000

#define CHIP_ID_H			0x08
#define CHIP_ID_L			0x48
#define JX_H63P_PIDH_ADDR		0x0a
#define JX_H63P_PIDL_ADDR		0x0b

#define JX_H63P_REG_CTRL_MODE		0x12
#define JX_H63P_MODE_SW_STANDBY		0x40
#define JX_H63P_MODE_STREAMING		0x00

#define JX_H63P_LONG_EXPO_HIGH_REG	0x02	/* Exposure Bits 8-15 */
#define JX_H63P_LONG_EXPO_LOW_REG	0x01	/* Exposure Bits 0-7 */
#define JX_H63P_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)	/* 8-15 Bits */
#define JX_H63P_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)	/* 0-7 Bits */
#define	JX_H63P_EXPOSURE_MIN		4
#define	JX_H63P_EXPOSURE_STEP		1
#define JX_H63P_VTS_MAX			0xffff

#define JX_H63P_AEC_PK_LONG_GAIN_REG	0x00	/* Bits 0 -7 */
#define	ANALOG_GAIN_MIN			0x00
#define	ANALOG_GAIN_MAX			0x3f
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x1f

// #define JX_H63P_DIGI_GAIN_L_MASK	0x3f
// #define JX_H63P_DIGI_GAIN_H_SHIFT	6
// #define JX_H63P_DIGI_GAIN_MIN	0
// #define JX_H63P_DIGI_GAIN_MAX	(0x4000 - 1)
// #define JX_H63P_DIGI_GAIN_STE	1
// #define JX_H63P_DIGI_GAIN_DEFAULT	1024

#define JX_H63P_REG_TEST_PATTERN	0x0c
#define JX_H63P_TEST_PATTERN_ENABLE	0x01
#define JX_H63P_TEST_PATTERN_DISABLE	0x0

#define JX_H63P_REG_HIGH_VTS		0x23
#define JX_H63P_REG_LOW_VTS		0X22
#define JX_H63P_FETCH_HIGH_BYTE_VTS(VAL) (((VAL) >> 8) & 0xFF)	/* 8-15 Bits */
#define JX_H63P_FETCH_LOW_BYTE_VTS(VAL) ((VAL) & 0xFF)	/* 0-7 Bits */

#define JX_H63P_FLIP_MIRROR_REG		0x12

#define REG_NULL			0xFF
#define REG_DELAY			0xFE

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define JX_H63P_NAME			"jx_h63p"

static const char * const jx_h63p_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define JX_H63P_NUM_SUPPLIES ARRAY_SIZE(jx_h63p_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct jx_h63p_mode {
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

struct jx_h63p {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[JX_H63P_NUM_SUPPLIES];
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
	struct v4l2_fract	cur_fps;
	bool			streaming;
	bool			power_on;
	const struct jx_h63p_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
};

#define to_jx_h63p(sd) container_of(sd, struct jx_h63p, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval jx_h63p_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * lane 2
 * linelength 750(0x370)
 * framelength 1920(0x5dc)
 * grabwindow_width 1280
 * grabwindow_height 750
 * max_framerate 60fps
 * mipi_datarate per lane 432Mbps
 */

static const struct regval jx_h63p_1280x720_2lane_regs[] = {
	{0x12, 0x40},
	{0x48, 0x85},
	{0x48, 0x05},
	{0x0E, 0x11},
	{0x0F, 0xA4},
	{0x10, 0x48},
	{0x11, 0x80},
	{0x57, 0x60},
	{0x58, 0x18},
	{0x61, 0x10},
	{0x46, 0x08},
	{0x0D, 0x50},
	{0x20, 0xC0},
	{0x21, 0x03},
	{0x22, 0xEE},
	{0x23, 0x02},
	{0x24, 0x80},
	{0x25, 0xD0},
	{0x26, 0x22},
	{0x27, 0x9B},
	{0x28, 0x15},
	{0x29, 0x03},
	{0x2A, 0x90},
	{0x2B, 0x13},
	{0x2C, 0x00},
	{0x2D, 0x00},
	{0x2E, 0xBA},
	{0x2F, 0x60},
	{0x41, 0x84},
	{0x42, 0x02},
	{0x47, 0x46},
	{0x76, 0x40},
	{0x77, 0x06},
	{0x80, 0x01},
	{0xAF, 0x22},
	{0x8A, 0x00},
	{0xA6, 0x00},
	{0xAB, 0x00},
	{0x8D, 0x49},
	{0x1D, 0x00},
	{0x1E, 0x04},
	{0x6C, 0x50},
	{0x9E, 0xF8},
	{0x6E, 0x2C},
	{0x70, 0xD8},
	{0x71, 0xDB},
	{0x72, 0xD4},
	{0x73, 0x59},
	{0x74, 0x02},
	{0x78, 0x99},
	{0x89, 0x01},
	{0x6B, 0x20},
	{0x86, 0x40},
	{0x9C, 0xE1},
	{0x3A, 0xAC},
	{0x3B, 0x24},
	{0x3C, 0xA6},
	{0x3D, 0xE0},
	{0x3E, 0xD0},
	{0x31, 0x0E},
	{0x32, 0x28},
	{0x33, 0x20},
	{0x34, 0x38},
	{0x35, 0x38},
	{0x56, 0x12},
	{0x59, 0x40},
	{0x85, 0x30},
	{0x64, 0xD2},
	{0x8F, 0x90},
	{0xA4, 0x87},
	{0xA7, 0x80},
	{0xA9, 0x48},
	{0x45, 0x01},
	{0x5B, 0xA0},
	{0x5C, 0x6C},
	{0x5D, 0x44},
	{0x5E, 0x81},
	{0x63, 0x0F},
	{0x65, 0x12},
	{0x66, 0x43},
	{0x67, 0x79},
	{0x68, 0x00},
	{0x69, 0x78},
	{0x6A, 0x28},
	{0x7A, 0x66},
	{0xA5, 0x03},
	{0x94, 0xC0},
	{0x13, 0x81},
	{0x96, 0x84},
	{0xB7, 0x4A},
	{0x4A, 0x01},
	{0xB5, 0x0C},
	{0xA1, 0x0F},
	{0xA3, 0x40},
	{0xB1, 0x00},
	{0x93, 0x00},
	{0x7E, 0x4C},
	{0x50, 0x02},
	{0x49, 0x10},
	{0x8E, 0x40},
	{0x7F, 0x56},
	{0x0C, 0x00},
	{0xBC, 0x11},
	{0x82, 0x00},
	{0x19, 0x20},
	{0x1F, 0x10},
	{0x1B, 0x4F},
	// {0x12, 0x00},
	{REG_NULL, 0x00},
};

static const struct jx_h63p_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x001f,
		.hts_def = 0x0780,
		.vts_def = 0x02ee,
		.reg_list = jx_h63p_1280x720_2lane_regs,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	JX_H63P_LINK_FREQ
};

static const char * const jx_h63p_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 jx_h63p_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, JX_H63P_XVCLK_FREQ / 1000 / 1000);
}

static int jx_h63p_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr =  client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"jx_h63p write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int jx_h63p_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i, delay_us;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
			delay_us = jx_h63p_cal_delay(500 * 1000);
			usleep_range(delay_us, delay_us * 2);
		} else {
			ret = jx_h63p_write_reg(client, regs[i].addr, regs[i].val);
		}
	}

	return ret;
}

static int jx_h63p_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		return 0;
	}

	dev_err(&client->dev,
		"jx_h63p read reg:0x%x failed !\n", reg);

	return ret;
}

static int jx_h63p_get_reso_dist(const struct jx_h63p_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct jx_h63p_mode *
jx_h63p_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = jx_h63p_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int jx_h63p_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	const struct jx_h63p_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&jx_h63p->mutex);

	mode = jx_h63p_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&jx_h63p->mutex);
		return -ENOTTY;
#endif
	} else {
		jx_h63p->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(jx_h63p->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(jx_h63p->vblank, vblank_def,
					 JX_H63P_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&jx_h63p->mutex);

	return 0;
}

static int jx_h63p_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	const struct jx_h63p_mode *mode = jx_h63p->cur_mode;

	mutex_lock(&jx_h63p->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&jx_h63p->mutex);
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
	mutex_unlock(&jx_h63p->mutex);

	return 0;
}

static int jx_h63p_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = jx_h63p->cur_mode->bus_fmt;

	return 0;
}

static int jx_h63p_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int jx_h63p_enable_test_pattern(struct jx_h63p *jx_h63p, u32 pattern)
{
	u8 val = 0;

	jx_h63p_read_reg(jx_h63p->client, JX_H63P_REG_TEST_PATTERN, &val);
	if (pattern)
		val |= (pattern - 1) | JX_H63P_TEST_PATTERN_ENABLE;
	else
		val &= ~JX_H63P_TEST_PATTERN_DISABLE;

	return jx_h63p_write_reg(jx_h63p->client, JX_H63P_REG_TEST_PATTERN, val);
}

static int jx_h63p_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	const struct jx_h63p_mode *mode = jx_h63p->cur_mode;

	mutex_lock(&jx_h63p->mutex);
	if (jx_h63p->streaming)
		fi->interval = jx_h63p->cur_fps;
	else
		fi->interval = mode->max_fps;
	mutex_unlock(&jx_h63p->mutex);

	return 0;
}

static int jx_h63p_g_mbus_config(struct v4l2_subdev *sd,
				 unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	const struct jx_h63p_mode *mode = jx_h63p->cur_mode;
	u32 val;

	val = 1 << (JX_H63P_LANES - 1) |
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

static void jx_h63p_get_module_inf(struct jx_h63p *jx_h63p,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, JX_H63P_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, jx_h63p->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, jx_h63p->len_name, sizeof(inf->base.lens));
}

static long jx_h63p_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		jx_h63p_get_module_inf(jx_h63p, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = jx_h63p->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = jx_h63p->cur_mode->width;
		h = jx_h63p->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				jx_h63p->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&jx_h63p->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = jx_h63p->cur_mode->hts_def - jx_h63p->cur_mode->width;
			h = jx_h63p->cur_mode->vts_def - jx_h63p->cur_mode->height;
			__v4l2_ctrl_modify_range(jx_h63p->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(jx_h63p->vblank, h,
						 JX_H63P_VTS_MAX - jx_h63p->cur_mode->height, 1, h);
			jx_h63p->cur_fps = jx_h63p->cur_mode->max_fps;
			jx_h63p->cur_vts = jx_h63p->cur_mode->vts_def;
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = jx_h63p_write_reg(jx_h63p->client, JX_H63P_REG_CTRL_MODE,
						JX_H63P_MODE_STREAMING);
		else
			ret = jx_h63p_write_reg(jx_h63p->client, JX_H63P_REG_CTRL_MODE,
						JX_H63P_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long jx_h63p_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = jx_h63p_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
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

		ret = jx_h63p_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret) {
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

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = jx_h63p_ioctl(sd, cmd, hdr);
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
			ret = jx_h63p_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (ret)
			return -EFAULT;
		ret = jx_h63p_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}
#endif

static int __jx_h63p_start_stream(struct jx_h63p *jx_h63p)
{
	int ret;

	dev_info(&jx_h63p->client->dev,
		 "%dx%d@%d, mode %d, vts 0x%x\n",
		 jx_h63p->cur_mode->width,
		 jx_h63p->cur_mode->height,
		 jx_h63p->cur_fps.denominator / jx_h63p->cur_fps.numerator,
		 jx_h63p->cur_mode->hdr_mode,
		 jx_h63p->cur_vts);

	ret = jx_h63p_write_array(jx_h63p->client, jx_h63p->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&jx_h63p->ctrl_handler);
	if (ret)
		return ret;

	ret = jx_h63p_write_reg(jx_h63p->client, JX_H63P_REG_CTRL_MODE,
				JX_H63P_MODE_STREAMING);
	return ret;
}

static int __jx_h63p_stop_stream(struct jx_h63p *jx_h63p)
{
	return jx_h63p_write_reg(jx_h63p->client, JX_H63P_REG_CTRL_MODE,
				 JX_H63P_MODE_SW_STANDBY);
}

static int jx_h63p_s_stream(struct v4l2_subdev *sd, int on)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	struct i2c_client *client = jx_h63p->client;
	int ret = 0;

	mutex_lock(&jx_h63p->mutex);
	on = !!on;
	if (on == jx_h63p->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __jx_h63p_start_stream(jx_h63p);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__jx_h63p_stop_stream(jx_h63p);
		pm_runtime_put(&client->dev);
	}

	jx_h63p->streaming = on;

unlock_and_return:
	mutex_unlock(&jx_h63p->mutex);

	return ret;
}

static int jx_h63p_s_power(struct v4l2_subdev *sd, int on)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	struct i2c_client *client = jx_h63p->client;
	int ret = 0;

	mutex_lock(&jx_h63p->mutex);

	/* If the power state is not modified - no work to do. */
	if (jx_h63p->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = jx_h63p_write_array(jx_h63p->client, jx_h63p_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		jx_h63p->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		jx_h63p->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&jx_h63p->mutex);

	return ret;
}

static int __jx_h63p_power_on(struct jx_h63p *jx_h63p)
{
	int ret;
	u32 delay_us;
	struct device *dev = &jx_h63p->client->dev;

	if (!IS_ERR_OR_NULL(jx_h63p->pins_default)) {
		ret = pinctrl_select_state(jx_h63p->pinctrl,
					   jx_h63p->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(jx_h63p->xvclk, JX_H63P_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(jx_h63p->xvclk) != JX_H63P_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(jx_h63p->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(jx_h63p->pwdn_gpio))
		gpiod_set_value_cansleep(jx_h63p->pwdn_gpio, 0);
	if (!IS_ERR(jx_h63p->reset_gpio))
		gpiod_set_value_cansleep(jx_h63p->reset_gpio, 1);

	usleep_range(2 * 1000, 3 * 1000);
	if (!IS_ERR(jx_h63p->reset_gpio))
		gpiod_set_value_cansleep(jx_h63p->reset_gpio, 0);

	ret = regulator_bulk_enable(JX_H63P_NUM_SUPPLIES, jx_h63p->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(2 * 1000, 3 * 1000);
	if (!IS_ERR(jx_h63p->reset_gpio))
		gpiod_set_value_cansleep(jx_h63p->reset_gpio, 1);

	/* According to datasheet, at least 10ms for reset duration */
	usleep_range(10 * 1000, 15 * 1000);

	if (!IS_ERR(jx_h63p->reset_gpio))
		gpiod_set_value_cansleep(jx_h63p->reset_gpio, 0);

	usleep_range(2000, 3000);
	if (!IS_ERR(jx_h63p->pwdn_gpio))
		gpiod_set_value_cansleep(jx_h63p->pwdn_gpio, 1);

	if (!IS_ERR(jx_h63p->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = jx_h63p_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(jx_h63p->xvclk);

	return ret;
}

static void __jx_h63p_power_off(struct jx_h63p *jx_h63p)
{
	int ret;
	struct device *dev = &jx_h63p->client->dev;

	if (!IS_ERR(jx_h63p->pwdn_gpio))
		gpiod_set_value_cansleep(jx_h63p->pwdn_gpio, 0);
	clk_disable_unprepare(jx_h63p->xvclk);
	if (!IS_ERR(jx_h63p->reset_gpio))
		gpiod_set_value_cansleep(jx_h63p->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(jx_h63p->pins_sleep)) {
		ret = pinctrl_select_state(jx_h63p->pinctrl,
					   jx_h63p->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(JX_H63P_NUM_SUPPLIES, jx_h63p->supplies);
}

static int jx_h63p_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);

	return __jx_h63p_power_on(jx_h63p);
}

static int jx_h63p_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);

	__jx_h63p_power_off(jx_h63p);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int jx_h63p_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct jx_h63p_mode *def_mode = &supported_modes[0];

	mutex_lock(&jx_h63p->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&jx_h63p->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int jx_h63p_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops jx_h63p_pm_ops = {
	SET_RUNTIME_PM_OPS(jx_h63p_runtime_suspend,
			   jx_h63p_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops jx_h63p_internal_ops = {
	.open = jx_h63p_open,
};
#endif

static const struct v4l2_subdev_core_ops jx_h63p_core_ops = {
	.s_power = jx_h63p_s_power,
	.ioctl = jx_h63p_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = jx_h63p_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops jx_h63p_video_ops = {
	.s_stream = jx_h63p_s_stream,
	.g_frame_interval = jx_h63p_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops jx_h63p_pad_ops = {
	.enum_mbus_code = jx_h63p_enum_mbus_code,
	.enum_frame_size = jx_h63p_enum_frame_sizes,
	.enum_frame_interval = jx_h63p_enum_frame_interval,
	.get_fmt = jx_h63p_get_fmt,
	.set_fmt = jx_h63p_set_fmt,
	.get_mbus_config = jx_h63p_g_mbus_config,
};

static const struct v4l2_subdev_ops jx_h63p_subdev_ops = {
	.core	= &jx_h63p_core_ops,
	.video	= &jx_h63p_video_ops,
	.pad	= &jx_h63p_pad_ops,
};

static void jx_h63p_modify_fps_info(struct jx_h63p *jx_h63p)
{
	const struct jx_h63p_mode *mode = jx_h63p->cur_mode;

	jx_h63p->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def/
				       jx_h63p->cur_vts;
}

static int jx_h63p_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct jx_h63p *jx_h63p = container_of(ctrl->handler,
					       struct jx_h63p, ctrl_handler);
	struct i2c_client *client = jx_h63p->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = jx_h63p->cur_mode->height + ctrl->val - 9;
		__v4l2_ctrl_modify_range(jx_h63p->exposure,
					 jx_h63p->exposure->minimum, max,
					 jx_h63p->exposure->step,
					 jx_h63p->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set expo: val: %d\n", ctrl->val);
		/* 4 least significant bits of expsoure are fractional part */
		ret = jx_h63p_write_reg(jx_h63p->client,
					JX_H63P_LONG_EXPO_HIGH_REG,
					JX_H63P_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= jx_h63p_write_reg(jx_h63p->client,
					 JX_H63P_LONG_EXPO_LOW_REG,
					 JX_H63P_FETCH_LOW_BYTE_EXP(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set a-gain: val: %d\n", ctrl->val);
		ret |= jx_h63p_write_reg(jx_h63p->client,
					 JX_H63P_AEC_PK_LONG_GAIN_REG, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank: val: %d\n", ctrl->val);
		ret |= jx_h63p_write_reg(jx_h63p->client, JX_H63P_REG_HIGH_VTS,
			JX_H63P_FETCH_HIGH_BYTE_VTS((ctrl->val + jx_h63p->cur_mode->height)));
		ret |= jx_h63p_write_reg(jx_h63p->client, JX_H63P_REG_LOW_VTS,
			JX_H63P_FETCH_LOW_BYTE_VTS((ctrl->val + jx_h63p->cur_mode->height)));
		if (!ret)
			jx_h63p->cur_vts =
				ctrl->val + jx_h63p->cur_mode->height;
		jx_h63p_modify_fps_info(jx_h63p);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = jx_h63p_enable_test_pattern(jx_h63p, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops jx_h63p_ctrl_ops = {
	.s_ctrl = jx_h63p_set_ctrl,
};

static int jx_h63p_initialize_controls(struct jx_h63p *jx_h63p)
{
	const struct jx_h63p_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &jx_h63p->ctrl_handler;
	mode = jx_h63p->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;
	handler->lock = &jx_h63p->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, JX_H63P_PIXEL_RATE, 1, JX_H63P_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	jx_h63p->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (jx_h63p->hblank)
		jx_h63p->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	jx_h63p->cur_fps = mode->max_fps;
	vblank_def = mode->vts_def - mode->height;
	jx_h63p->cur_vts = mode->vts_def;
	jx_h63p->vblank =
		v4l2_ctrl_new_std(handler, &jx_h63p_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_def, JX_H63P_VTS_MAX - mode->height, 1,
				  vblank_def);

	exposure_max = mode->vts_def - 9;
	jx_h63p->exposure = v4l2_ctrl_new_std(handler, &jx_h63p_ctrl_ops,
				V4L2_CID_EXPOSURE, JX_H63P_EXPOSURE_MIN,
				exposure_max, JX_H63P_EXPOSURE_STEP,
				mode->exp_def);

	jx_h63p->anal_gain = v4l2_ctrl_new_std(handler, &jx_h63p_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	jx_h63p->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&jx_h63p_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(jx_h63p_test_pattern_menu) - 1,
				0, 0, jx_h63p_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&jx_h63p->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	jx_h63p->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int jx_h63p_check_sensor_id(struct jx_h63p *jx_h63p,
				  struct i2c_client *client)
{
	struct device *dev = &jx_h63p->client->dev;
	u8 id_h = 0;
	u8 id_l = 0;
	int ret;

	ret = jx_h63p_read_reg(client, JX_H63P_PIDH_ADDR, &id_h);
	ret |= jx_h63p_read_reg(client, JX_H63P_PIDL_ADDR, &id_l);
	if (id_h != CHIP_ID_H && id_l != CHIP_ID_L) {
		dev_err(dev, "Wrong camera sensor id(0x%02x%02x)\n",
			id_h, id_l);
		return -EINVAL;
	}

	dev_info(dev, "Detected jx_h63p (0x%02x%02x) sensor\n",
		id_h, id_l);

	return ret;
}

static int jx_h63p_configure_regulators(struct jx_h63p *jx_h63p)
{
	unsigned int i;

	for (i = 0; i < JX_H63P_NUM_SUPPLIES; i++)
		jx_h63p->supplies[i].supply = jx_h63p_supply_names[i];

	return devm_regulator_bulk_get(&jx_h63p->client->dev,
				       JX_H63P_NUM_SUPPLIES,
				       jx_h63p->supplies);
}

static int jx_h63p_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct jx_h63p *jx_h63p;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	jx_h63p = devm_kzalloc(dev, sizeof(*jx_h63p), GFP_KERNEL);
	if (!jx_h63p)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &jx_h63p->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &jx_h63p->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &jx_h63p->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &jx_h63p->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	jx_h63p->client = client;
	jx_h63p->cur_mode = &supported_modes[0];

	jx_h63p->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(jx_h63p->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	jx_h63p->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(jx_h63p->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	jx_h63p->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(jx_h63p->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	jx_h63p->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(jx_h63p->pinctrl)) {
		jx_h63p->pins_default =
			pinctrl_lookup_state(jx_h63p->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(jx_h63p->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		jx_h63p->pins_sleep =
			pinctrl_lookup_state(jx_h63p->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(jx_h63p->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = jx_h63p_configure_regulators(jx_h63p);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&jx_h63p->mutex);

	sd = &jx_h63p->subdev;
	v4l2_i2c_subdev_init(sd, client, &jx_h63p_subdev_ops);
	ret = jx_h63p_initialize_controls(jx_h63p);
	if (ret)
		goto err_destroy_mutex;

	ret = __jx_h63p_power_on(jx_h63p);
	if (ret)
		goto err_free_handler;

	ret = jx_h63p_check_sensor_id(jx_h63p, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &jx_h63p_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	jx_h63p->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &jx_h63p->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(jx_h63p->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 jx_h63p->module_index, facing,
		 JX_H63P_NAME, dev_name(sd->dev));

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
	__jx_h63p_power_off(jx_h63p);
err_free_handler:
	v4l2_ctrl_handler_free(&jx_h63p->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&jx_h63p->mutex);

	return ret;
}

static int jx_h63p_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct jx_h63p *jx_h63p = to_jx_h63p(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&jx_h63p->ctrl_handler);
	mutex_destroy(&jx_h63p->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__jx_h63p_power_off(jx_h63p);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id jx_h63p_of_match[] = {
	{ .compatible = "soi,jx_h63p" },
	{},
};
MODULE_DEVICE_TABLE(of, jx_h63p_of_match);
#endif

static const struct i2c_device_id jx_h63p_match_id[] = {
	{ "soi,jx_h63p", 0 },
	{ },
};

static struct i2c_driver jx_h63p_i2c_driver = {
	.driver = {
		.name = JX_H63P_NAME,
		.pm = &jx_h63p_pm_ops,
		.of_match_table = of_match_ptr(jx_h63p_of_match),
	},
	.probe		= &jx_h63p_probe,
	.remove		= &jx_h63p_remove,
	.id_table	= jx_h63p_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&jx_h63p_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&jx_h63p_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("SOI jx_h63p sensor driver");
MODULE_LICENSE("GPL");
