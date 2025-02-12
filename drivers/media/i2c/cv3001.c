// SPDX-License-Identifier: GPL-2.0
/*
 * CV3001 sensor driver
 *
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 Initial version
 * V0.0X01.0X02 support thunder boot
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

#define DRIVER_VERSION			KERNEL_VERSION(0x0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define CV3001_LANES			2
#define CV3001_BITS_PER_SAMPLE		10
#define CV3001_LINK_FREQ_1080		540000000

#define PIXEL_RATE_WITH_1080M_10BIT	(CV3001_LINK_FREQ_1080 / 10 * 2 * CV3001_LANES)

#define CHIP_ID				0x3001
#define CV3001_REG_CHIP_ID_L		0x3002
#define CV3001_REG_CHIP_ID_H		0x3003

#define CV3001_REG_CTRL_MODE		0x3000
#define CV3001_MODE_SW_STANDBY		0x1
#define CV3001_MODE_STREAMING		0x0

#define CV3001_REG_EXPOSURE_H		0x305A
#define CV3001_REG_EXPOSURE_M		0x3059
#define CV3001_REG_EXPOSURE_L		0x3058
#define	CV3001_EXPOSURE_MIN		8
#define	CV3001_EXPOSURE_STEP		2
#define CV3001_VTS_MAX			0x7fff

#define CV3001_REG_DIG_GAIN_H		0x316D
#define CV3001_REG_DIG_GAIN_L		0x316C
#define CV3001_REG_ANA_GAIN		0x3174
#define CV3001_GAIN_MIN			0x0040
#define CV3001_GAIN_MAX			(32080)	//16*(2005/64)*64 = 32080
#define CV3001_GAIN_STEP		1
#define CV3001_GAIN_DEFAULT		0x40


#define CV3001_REG_GROUP_HOLD		0x3333
#define CV3001_GROUP_HOLD_START		0x01
#define CV3001_GROUP_HOLD_END		0x00

#define CV3001_REG_TEST_PATTERN		0x4501
#define CV3001_TEST_PATTERN_BIT_MASK	BIT(3)

#define CV3001_REG_VTS_H		0x3025
#define CV3001_REG_VTS_L		0x3024

#define CV3001_REG_HTS_H		0x3029
#define CV3001_REG_HTS_L		0x3028

#define CV3001_FLIP_MIRROR_REG		0x3030

#define CV3001_FETCH_EXP_H(VAL)		(((VAL) >> 16) & 0x0F)
#define CV3001_FETCH_EXP_M(VAL)		(((VAL) >> 8) & 0xFF)
#define CV3001_FETCH_EXP_L(VAL)		((VAL) & 0xFF)

#define CV3001_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define CV3001_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define CV3001_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x01 : VAL & 0xfe)
#define CV3001_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x02 : VAL & 0xfd)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define CV3001_REG_VALUE_08BIT		1
#define CV3001_REG_VALUE_16BIT		2
#define CV3001_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define CV3001_NAME			"cv3001"

static const char *const cv3001_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define CV3001_NUM_SUPPLIES ARRAY_SIZE(cv3001_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct cv3001_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 xvclk_freq;
	u32 link_freq_idx;
	u32 vc[PAD_MAX];
};

struct cv3001 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct regulator_bulk_data supplies[CV3001_NUM_SUPPLIES];

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
	bool			streaming;
	bool			power_on;
	const struct cv3001_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_cv3001(sd) container_of(sd, struct cv3001, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval cv3001_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 1080Mbs, 2lane
 */
static const struct regval cv3001_linear_10_2304x1296_30fps_regs[] = {
	{0x301D, 0x00},
	{0x3024, 0xDC},//vts 0xADC = 2780
	{0x3025, 0x0A},//
	{0x3026, 0x00},//
	{0x3028, 0x88},//hts 0x288 = 648
	{0x3029, 0x02},//
	{0x3032, 0x00},
	{0x3033, 0x00},
	{0x3300, 0x00},
	{0x3401, 0x01},
	{0x343E, 0x00},
	{0x3806, 0x01},
	{0x3908, 0x2D},
	{0x3909, 0x00},
	{0x390A, 0x02},
	{0x3034, 0x01},
	{0x3038, 0x04},
	{0x3039, 0x00},
	{0x303A, 0x10},
	{0x303B, 0x05},
	{0x303C, 0x04},
	{0x303D, 0x00},
	{0x303E, 0x00},
	{0x303F, 0x09},
	{0x3338, 0x00},
	{0x35B3, 0x0A},
	{0x3168, 0x64},
	{0x3871, 0x00},
	{REG_NULL, 0x00},
};

// i2c addr 0x35
static const struct cv3001_mode supported_modes[] = {
	{
		.width = 2304,
		.height = 1296,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0xad4,
		.hts_def = 0x288 * 4,
		.vts_def = 0xadc,
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		.reg_list = cv3001_linear_10_2304x1296_30fps_regs,
		.hdr_mode = NO_HDR,
		.xvclk_freq = 24000000,
		.link_freq_idx = 0,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	CV3001_LINK_FREQ_1080
};

static const char *const cv3001_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4",
};

/* Write registers up to 4 at a time */
/* sensor i2c addr: 0x35 */
static int cv3001_write_reg(struct i2c_client *client, u16 reg,
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

static int cv3001_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = cv3001_write_reg(client, regs[i].addr,
				       CV3001_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int cv3001_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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
	dev_err(&client->dev, "cv3001 read reg ret %d\n", ret);
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;
	dev_err(&client->dev, "cv3001 data_be: 0x%x\n", data_be);
	*val = be32_to_cpu(data_be);

	return 0;
}

static int cv3001_set_gain_reg(struct cv3001 *cv3001, u32 gain)
{
	u32 again, dgainH, dgainL;
	int ret = 0;

	if (gain < CV3001_GAIN_MIN) {
		again  = 0x00;
		dgainH = 0x00;
		dgainL = 0x40;
	} else if (gain > CV3001_GAIN_MAX) {
		again = 0xF0;
		dgainH = 0x07;
		dgainL = 0xD5;
	} else if (gain < 16 * 64) {
		again = gain / 64;
		dgainH = 0x00;
		dgainL = 0x40;
	} else {
		again = 0xf0;
		if (gain < 16 * 64 * 4) {
			dgainH = 0x00;
			dgainL = 0xff;
		} else {
			dgainH = 0x07;
			dgainL = (gain - 16 * 64 * 4) / 64 + 0xff;
		}
	}

	dev_dbg(&cv3001->client->dev,
		"total_gain: 0x%x, d_gain_H: 0x%x, d_gain_L: 0x%x, again: 0x%x\n",
		gain, dgainH, dgainL, again);

	ret = cv3001_write_reg(cv3001->client,
			       CV3001_REG_DIG_GAIN_H,
			       CV3001_REG_VALUE_08BIT,
			       dgainH);
	ret |= cv3001_write_reg(cv3001->client,
				CV3001_REG_DIG_GAIN_L,
				CV3001_REG_VALUE_08BIT,
				dgainL);
	ret |= cv3001_write_reg(cv3001->client,
				CV3001_REG_ANA_GAIN,
				CV3001_REG_VALUE_08BIT,
				again);

	return ret;
}

static int cv3001_set_exposure_reg(struct cv3001 *cv3001, u32 exp)
{
	// u32 again = 0, dgainH = 0, dgainL = 0;
	// u32 gain_factor;
	int ret = 0;
	int shutter0 = 0;

	shutter0 = cv3001->cur_vts - exp;
	shutter0 = shutter0 / 2 * 2;

	shutter0 = (shutter0 < 8) ? 8 :
		   (shutter0 > (cv3001->cur_vts - 4)) ? cv3001->cur_vts - 4 : shutter0;

	/* 4 least significant bits of expsoure are fractional part */
	ret = cv3001_write_reg(cv3001->client,
			       CV3001_REG_EXPOSURE_H,
			       CV3001_REG_VALUE_08BIT,
			       CV3001_FETCH_EXP_H(shutter0));
	ret |= cv3001_write_reg(cv3001->client,
				CV3001_REG_EXPOSURE_M,
				CV3001_REG_VALUE_08BIT,
				CV3001_FETCH_EXP_M(shutter0));
	ret |= cv3001_write_reg(cv3001->client,
				CV3001_REG_EXPOSURE_L,
				CV3001_REG_VALUE_08BIT,
				CV3001_FETCH_EXP_L(shutter0));

	return ret;
}

static int cv3001_get_reso_dist(const struct cv3001_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct cv3001_mode *
cv3001_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = cv3001_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int cv3001_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	const struct cv3001_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	mutex_lock(&cv3001->mutex);

	mode = cv3001_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&cv3001->mutex);
		return -ENOTTY;
#endif
	} else {
		cv3001->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(cv3001->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(cv3001->vblank, vblank_def,
					 CV3001_VTS_MAX - mode->height,
					 1, vblank_def);
		dst_link_freq = mode->link_freq_idx;
		dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
				 CV3001_BITS_PER_SAMPLE * 2 * CV3001_LANES;
		__v4l2_ctrl_s_ctrl_int64(cv3001->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(cv3001->link_freq,
				   dst_link_freq);
	}

	mutex_unlock(&cv3001->mutex);

	return 0;
}

static int cv3001_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	const struct cv3001_mode *mode = cv3001->cur_mode;

	mutex_lock(&cv3001->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&cv3001->mutex);
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
	mutex_unlock(&cv3001->mutex);

	return 0;
}

static int cv3001_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct cv3001 *cv3001 = to_cv3001(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = cv3001->cur_mode->bus_fmt;

	return 0;
}

static int cv3001_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int cv3001_enable_test_pattern(struct cv3001 *cv3001, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = cv3001_read_reg(cv3001->client, CV3001_REG_TEST_PATTERN,
			      CV3001_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= CV3001_TEST_PATTERN_BIT_MASK;
	else
		val &= ~CV3001_TEST_PATTERN_BIT_MASK;

	ret |= cv3001_write_reg(cv3001->client, CV3001_REG_TEST_PATTERN,
				CV3001_REG_VALUE_08BIT, val);
	return ret;
}

static int cv3001_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	const struct cv3001_mode *mode = cv3001->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int cv3001_g_mbus_config(struct v4l2_subdev *sd,
				unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	const struct cv3001_mode *mode = cv3001->cur_mode;

	u32 val = 1 << (CV3001_LANES - 1) |
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

static void cv3001_get_module_inf(struct cv3001 *cv3001,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, CV3001_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, cv3001->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, cv3001->len_name, sizeof(inf->base.lens));
}

static long cv3001_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		cv3001_get_module_inf(cv3001, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = cv3001->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = cv3001->cur_mode->width;
		h = cv3001->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				cv3001->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&cv3001->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = cv3001->cur_mode->hts_def - cv3001->cur_mode->width;
			h = cv3001->cur_mode->vts_def - cv3001->cur_mode->height;
			__v4l2_ctrl_modify_range(cv3001->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(cv3001->vblank, h,
						 CV3001_VTS_MAX - cv3001->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = cv3001_write_reg(cv3001->client, CV3001_REG_CTRL_MODE,
					       CV3001_REG_VALUE_08BIT, CV3001_MODE_STREAMING);
		else
			ret = cv3001_write_reg(cv3001->client, CV3001_REG_CTRL_MODE,
					       CV3001_REG_VALUE_08BIT, CV3001_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long cv3001_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = cv3001_ioctl(sd, cmd, inf);
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

		ret = cv3001_ioctl(sd, cmd, hdr);
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
			ret = cv3001_ioctl(sd, cmd, hdr);
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
			ret = cv3001_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = cv3001_ioctl(sd, cmd, &stream);
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

static int __cv3001_start_stream(struct cv3001 *cv3001)
{
	int ret;

	if (!cv3001->is_thunderboot) {
		ret = cv3001_write_array(cv3001->client, cv3001->cur_mode->reg_list);
		if (ret)
			return ret;
		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&cv3001->ctrl_handler);
		if (ret)
			return ret;
		if (cv3001->has_init_exp && cv3001->cur_mode->hdr_mode != NO_HDR) {
			ret = cv3001_ioctl(&cv3001->subdev, PREISP_CMD_SET_HDRAE_EXP,
					   &cv3001->init_hdrae_exp);
			if (ret) {
				dev_err(&cv3001->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}
	ret = cv3001_write_reg(cv3001->client, CV3001_REG_CTRL_MODE,
			       CV3001_REG_VALUE_08BIT, CV3001_MODE_STREAMING);
	return ret;
}

static int __cv3001_stop_stream(struct cv3001 *cv3001)
{
	cv3001->has_init_exp = false;
	if (cv3001->is_thunderboot) {
		cv3001->is_first_streamoff = true;
		pm_runtime_put(&cv3001->client->dev);
	}
	return cv3001_write_reg(cv3001->client, CV3001_REG_CTRL_MODE,
				CV3001_REG_VALUE_08BIT, CV3001_MODE_SW_STANDBY);
}

static int __cv3001_power_on(struct cv3001 *cv3001);
static int cv3001_s_stream(struct v4l2_subdev *sd, int on)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	struct i2c_client *client = cv3001->client;
	int ret = 0;

	mutex_lock(&cv3001->mutex);
	on = !!on;
	if (on == cv3001->streaming)
		goto unlock_and_return;
	if (on) {
		if (cv3001->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			cv3001->is_thunderboot = false;
			__cv3001_power_on(cv3001);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __cv3001_start_stream(cv3001);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__cv3001_stop_stream(cv3001);
		pm_runtime_put(&client->dev);
	}

	cv3001->streaming = on;
unlock_and_return:
	mutex_unlock(&cv3001->mutex);
	return ret;
}

static int cv3001_s_power(struct v4l2_subdev *sd, int on)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	struct i2c_client *client = cv3001->client;
	int ret = 0;

	mutex_lock(&cv3001->mutex);

	/* If the power state is not modified - no work to do. */
	if (cv3001->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!cv3001->is_thunderboot) {
			ret = cv3001_write_array(cv3001->client, cv3001_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		cv3001->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		cv3001->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&cv3001->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 cv3001_cal_delay(u32 cycles, struct cv3001 *cv3001)
{
	return DIV_ROUND_UP(cycles, cv3001->cur_mode->xvclk_freq / 1000 / 1000);
}

static int __cv3001_power_on(struct cv3001 *cv3001)
{
	int ret;
	u32 delay_us;
	struct device *dev = &cv3001->client->dev;

	if (!IS_ERR_OR_NULL(cv3001->pins_default)) {
		ret = pinctrl_select_state(cv3001->pinctrl,
					   cv3001->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(cv3001->xvclk, cv3001->cur_mode->xvclk_freq);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (%dHz)\n", cv3001->cur_mode->xvclk_freq);
	if (clk_get_rate(cv3001->xvclk) != cv3001->cur_mode->xvclk_freq)
		dev_warn(dev, "xvclk mismatched, modes are based on %dHz\n",
			 cv3001->cur_mode->xvclk_freq);
	ret = clk_prepare_enable(cv3001->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (cv3001->is_thunderboot)
		return 0;

	if (!IS_ERR(cv3001->reset_gpio))
		gpiod_set_value_cansleep(cv3001->reset_gpio, 0);

	ret = regulator_bulk_enable(CV3001_NUM_SUPPLIES, cv3001->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(cv3001->reset_gpio))
		gpiod_set_value_cansleep(cv3001->reset_gpio, 1);

	usleep_range(500, 1000);

	if (!IS_ERR(cv3001->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = cv3001_cal_delay(8192, cv3001);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(cv3001->xvclk);

	return ret;
}

static void __cv3001_power_off(struct cv3001 *cv3001)
{
	int ret;
	struct device *dev = &cv3001->client->dev;

	clk_disable_unprepare(cv3001->xvclk);
	if (cv3001->is_thunderboot) {
		if (cv3001->is_first_streamoff) {
			cv3001->is_thunderboot = false;
			cv3001->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(cv3001->reset_gpio))
		gpiod_set_value_cansleep(cv3001->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(cv3001->pins_sleep)) {
		ret = pinctrl_select_state(cv3001->pinctrl,
					   cv3001->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(CV3001_NUM_SUPPLIES, cv3001->supplies);
}

static int cv3001_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cv3001 *cv3001 = to_cv3001(sd);

	return __cv3001_power_on(cv3001);
}

static int cv3001_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cv3001 *cv3001 = to_cv3001(sd);

	__cv3001_power_off(cv3001);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int cv3001_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct cv3001 *cv3001 = to_cv3001(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct cv3001_mode *def_mode = &supported_modes[0];

	mutex_lock(&cv3001->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&cv3001->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int cv3001_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops cv3001_pm_ops = {
	SET_RUNTIME_PM_OPS(cv3001_runtime_suspend,
	cv3001_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops cv3001_internal_ops = {
	.open = cv3001_open,
};
#endif

static const struct v4l2_subdev_core_ops cv3001_core_ops = {
	.s_power = cv3001_s_power,
	.ioctl = cv3001_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cv3001_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops cv3001_video_ops = {
	.s_stream = cv3001_s_stream,
	.g_frame_interval = cv3001_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops cv3001_pad_ops = {
	.enum_mbus_code = cv3001_enum_mbus_code,
	.enum_frame_size = cv3001_enum_frame_sizes,
	.enum_frame_interval = cv3001_enum_frame_interval,
	.get_fmt = cv3001_get_fmt,
	.set_fmt = cv3001_set_fmt,
	.get_mbus_config = cv3001_g_mbus_config,
};

static const struct v4l2_subdev_ops cv3001_subdev_ops = {
	.core	= &cv3001_core_ops,
	.video	= &cv3001_video_ops,
	.pad	= &cv3001_pad_ops,
};

static int cv3001_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cv3001 *cv3001 = container_of(ctrl->handler,
					     struct cv3001, ctrl_handler);
	struct i2c_client *client = cv3001->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = cv3001->cur_mode->height + ctrl->val - 6;
		__v4l2_ctrl_modify_range(cv3001->exposure,
					 cv3001->exposure->minimum, max,
					 cv3001->exposure->step,
					 cv3001->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (cv3001->cur_mode->hdr_mode == NO_HDR)
			ret = cv3001_set_exposure_reg(cv3001, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (cv3001->cur_mode->hdr_mode == NO_HDR)
			ret = cv3001_set_gain_reg(cv3001, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = cv3001_write_reg(cv3001->client,
				       CV3001_REG_VTS_H,
				       CV3001_REG_VALUE_08BIT,
				       (ctrl->val + cv3001->cur_mode->height)
				       >> 8);
		ret |= cv3001_write_reg(cv3001->client,
					CV3001_REG_VTS_L,
					CV3001_REG_VALUE_08BIT,
					(ctrl->val + cv3001->cur_mode->height)
					& 0xff);
		cv3001->cur_vts = ctrl->val + cv3001->cur_mode->height;
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cv3001_enable_test_pattern(cv3001, ctrl->val);
		break;
	case V4L2_CID_HFLIP:	// TODO check
		ret = cv3001_read_reg(cv3001->client, CV3001_FLIP_MIRROR_REG,
				      CV3001_REG_VALUE_08BIT, &val);
		ret |= cv3001_write_reg(cv3001->client, CV3001_FLIP_MIRROR_REG,
					CV3001_REG_VALUE_08BIT,
					CV3001_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:	// TODO check
		ret = cv3001_read_reg(cv3001->client, CV3001_FLIP_MIRROR_REG,
				      CV3001_REG_VALUE_08BIT, &val);
		ret |= cv3001_write_reg(cv3001->client, CV3001_FLIP_MIRROR_REG,
					CV3001_REG_VALUE_08BIT,
					CV3001_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops cv3001_ctrl_ops = {
	.s_ctrl = cv3001_set_ctrl,
};

static int cv3001_initialize_controls(struct cv3001 *cv3001)
{
	const struct cv3001_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	handler = &cv3001->ctrl_handler;
	mode = cv3001->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &cv3001->mutex;

	cv3001->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			    V4L2_CID_LINK_FREQ,
			    ARRAY_SIZE(link_freq_menu_items) - 1, 0, link_freq_menu_items);
	if (cv3001->link_freq)
		cv3001->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	dst_link_freq = mode->link_freq_idx;
	dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
			 CV3001_BITS_PER_SAMPLE * 2 * CV3001_LANES;
	cv3001->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
					       0, PIXEL_RATE_WITH_1080M_10BIT,
					       1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(cv3001->link_freq, dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	cv3001->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (cv3001->hblank)
		cv3001->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	cv3001->vblank = v4l2_ctrl_new_std(handler, &cv3001_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   CV3001_VTS_MAX - mode->height,
					   1, vblank_def);
	exposure_max = mode->vts_def - 4;
	cv3001->exposure = v4l2_ctrl_new_std(handler, &cv3001_ctrl_ops,
					     V4L2_CID_EXPOSURE, CV3001_EXPOSURE_MIN,
					     exposure_max, CV3001_EXPOSURE_STEP,
					     mode->exp_def);
	cv3001->anal_gain = v4l2_ctrl_new_std(handler, &cv3001_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, CV3001_GAIN_MIN,
					      CV3001_GAIN_MAX, CV3001_GAIN_STEP,
					      CV3001_GAIN_DEFAULT);
	cv3001->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
			       &cv3001_ctrl_ops,
			       V4L2_CID_TEST_PATTERN,
			       ARRAY_SIZE(cv3001_test_pattern_menu) - 1,
			       0, 0, cv3001_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &cv3001_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &cv3001_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&cv3001->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	cv3001->subdev.ctrl_handler = handler;
	cv3001->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int cv3001_check_sensor_id(struct cv3001 *cv3001,
				  struct i2c_client *client)
{
	struct device *dev = &cv3001->client->dev;
	u32 idl = 0, idh = 0, id = 0;
	int ret;

	if (cv3001->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = cv3001_read_reg(client, CV3001_REG_CHIP_ID_H,
			      CV3001_REG_VALUE_08BIT, &idh);
	ret |= cv3001_read_reg(client, CV3001_REG_CHIP_ID_L,
			       CV3001_REG_VALUE_08BIT, &idl);
	id = ((idh & 0xff) << 8) | (idl & 0xff);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(0x%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected CVSENS CV3001 (0x%04x) sensor\n", CHIP_ID);

	return 0;
}

static int cv3001_configure_regulators(struct cv3001 *cv3001)
{
	unsigned int i;

	for (i = 0; i < CV3001_NUM_SUPPLIES; i++)
		cv3001->supplies[i].supply = cv3001_supply_names[i];

	return devm_regulator_bulk_get(&cv3001->client->dev,
				       CV3001_NUM_SUPPLIES,
				       cv3001->supplies);
}

static int cv3001_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct cv3001 *cv3001;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	int i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	cv3001 = devm_kzalloc(dev, sizeof(*cv3001), GFP_KERNEL);
	if (!cv3001)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &cv3001->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &cv3001->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &cv3001->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &cv3001->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	cv3001->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	cv3001->client = client;
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			cv3001->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		cv3001->cur_mode = &supported_modes[0];

	cv3001->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(cv3001->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	if (cv3001->is_thunderboot) {
		cv3001->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
		if (IS_ERR(cv3001->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");
	} else {
		cv3001->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(cv3001->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");
	}

	cv3001->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(cv3001->pinctrl)) {
		cv3001->pins_default =
			pinctrl_lookup_state(cv3001->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(cv3001->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		cv3001->pins_sleep =
			pinctrl_lookup_state(cv3001->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(cv3001->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = cv3001_configure_regulators(cv3001);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&cv3001->mutex);

	sd = &cv3001->subdev;
	v4l2_i2c_subdev_init(sd, client, &cv3001_subdev_ops);
	ret = cv3001_initialize_controls(cv3001);
	if (ret)
		goto err_destroy_mutex;

	ret = __cv3001_power_on(cv3001);
	if (ret)
		goto err_free_handler;

	ret = cv3001_check_sensor_id(cv3001, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &cv3001_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	cv3001->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &cv3001->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(cv3001->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 cv3001->module_index, facing,
		 CV3001_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (cv3001->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__cv3001_power_off(cv3001);
err_free_handler:
	v4l2_ctrl_handler_free(&cv3001->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&cv3001->mutex);

	return ret;
}

static int cv3001_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cv3001 *cv3001 = to_cv3001(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&cv3001->ctrl_handler);
	mutex_destroy(&cv3001->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__cv3001_power_off(cv3001);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id cv3001_of_match[] = {
	{ .compatible = "cvsens,cv3001" },
	{},
};
MODULE_DEVICE_TABLE(of, cv3001_of_match);
#endif

static const struct i2c_device_id cv3001_match_id[] = {
	{ "cvsens,cv3001", 0 },
	{ },
};

static struct i2c_driver cv3001_i2c_driver = {
	.driver = {
		.name = CV3001_NAME,
		.pm = &cv3001_pm_ops,
		.of_match_table = of_match_ptr(cv3001_of_match),
	},
	.probe		= cv3001_probe,
	.remove		= cv3001_remove,
	.id_table	= cv3001_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&cv3001_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&cv3001_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("cvsens cv3001 sensor driver");
MODULE_LICENSE("GPL");
