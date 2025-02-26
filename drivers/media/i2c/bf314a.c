// SPDX-License-Identifier: GPL-2.0
/*
 * bf314a driver
 *
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first implement
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
#include <linux/of.h>
#include <linux/of_graph.h>
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define BF314A_LANES			1
#define BF314A_BITS_PER_SAMPLE		10
#define BF314A_LINK_FREQ_180M		180000000  //360Mbps
#define BF314A_LINK_FREQ_360M		360000000  //360Mbps

#define PIXEL_RATE_WITH_180M_10BIT	(BF314A_LINK_FREQ_180M * 2 * \
					BF314A_LANES / BF314A_BITS_PER_SAMPLE)
#define PIXEL_RATE_WITH_360M_10BIT	(BF314A_LINK_FREQ_360M * 2 * \
					BF314A_LANES / BF314A_BITS_PER_SAMPLE)

#define BF314A_XVCLK_FREQ		24000000

#define CHIP_ID				0x314a
#define BF314A_REG_CHIP_ID_H		0xfc
#define BF314A_REG_CHIP_ID_L		0xfd
#define SENSOR_ID(_msb, _lsb)	((_msb) << 8 | (_lsb))

#define BF314A_REG_CTRL_MODE		0xf3
#define BF314A_MODE_SW_STANDBY		0x01
#define BF314A_MODE_STREAMING		0x00

#define BF314A_REG_MIPI_CTRL		0x7D
#define BF314A_MIPI_CTRL_ON		0x0f
#define BF314A_MIPI_CTRL_OFF		0x07

#define BF314A_REG_EXPOSURE_H		0x6b
#define BF314A_REG_EXPOSURE_L		0x6c
#define BF314A_EXPOSURE_MIN		1
#define BF314A_EXPOSURE_STEP		1
#define BF314A_VTS_MAX			0x7fff

#define BF314A_ANALOG_GAIN_REG		0x6a
#define BF314A_GAIN_MIN			0x40
#define BF314A_GAIN_MAX			0x400
#define BF314A_GAIN_STEP		4
#define BF314A_GAIN_DEFAULT		0x40
#define BF314A_LGAIN			0
#define BF314A_SGAIN			1

#define BF314A_REG_TEST_PATTERN		0x96
#define BF314A_TEST_PATTERN_BIT_MASK	BIT(3)

#define BF314A_REG_HTS_H		0x0c
#define BF314A_REG_HTS_L		0x0b
#define BF314A_REG_VTS_H		0x07	// dummy line, use for adjust vts(0x22 0x23)
#define BF314A_REG_VTS_L		0x06

#define BF314A_MIRROR_FLIP_REG		0x00

#define BF314A_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x08 : VAL & 0xf7)
#define BF314A_FETCH_FLIP(VAL, ENABLE)	(ENABLE ? VAL | 0x04 : VAL & 0xfb)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFF //0xFFFF

#define BF314A_REG_VALUE_08BIT		1
#define BF314A_REG_VALUE_16BIT		2
#define BF314A_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define BF314A_NAME			"bf314a"

static const char *const bf314a_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define BF314A_NUM_SUPPLIES ARRAY_SIZE(bf314a_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct bf314a_mode {
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

struct bf314a {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[BF314A_NUM_SUPPLIES];

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
	const struct bf314a_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	enum rkmodule_sync_mode	sync_mode;
	u32			standby_hw;
	u32			cur_vts;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	bool			is_standby;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	struct cam_sw_info	*cam_sw_inf;
};

#define to_bf314a(sd) container_of(sd, struct bf314a, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval bf314a_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 60fps
 * mipi_datarate per lane 720Mbps, 1lane
 */
static const struct regval bf314a_linear_10_1280x720_60fps_regs[] = {
	{0xf2, 0x01},
	{0xf3, 0x00},
	{0xf3, 0x00},//power up
	{0x00, 0x21},//mirror and vertical flip
	{0x7d, 0x0f},//MIPI:0f,DVP:07
	{0xeb, 0x05},//MIPI:05,DVP:04
	{0x0b, 0x20},//line length 1600
	{0x0c, 0x03},
	{0x03, 0xcf},
	{0x15, 0xca},
	{0x16, 0xd4},
	{0x19, 0xca},
	{0x1b, 0x3f},
	{0x1d, 0x3f},
	{0x24, 0x6c},
	{0x25, 0xc8},
	{0x26, 0xff},
	{0x27, 0x58},
	{0x29, 0x41},
	{0x2b, 0xaa},
	{0xe0, 0x16},//MIPICLK:720M
	{0xe1, 0xb4},
	{0xe2, 0x36},
	{0xe3, 0x4c},
	{0xe4, 0x62},
	{0xe5, 0x86},
	{0xe8, 0x60},
	{0x5e, 0x32},//black Lock
	{0x59, 0x10},//black target B
	{0x5a, 0x10},//black target Gb
	{0x5b, 0x10},//black target R
	{0x5c, 0x10},//black target Gr
	{0x6a, 0x1f},//again
	{0x6b, 0x01},//integration time :2step
	{0x6c, 0xc2},//integration time
	{0x6d, 0x14},//
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 360Mbps, 1lane
 */
static const struct regval bf314a_linear_10_1280x720_30fps_regs[] = {
	{0xf2, 0x01},
	{0xf3, 0x00},
	{0xf3, 0x00},//power up
	{0x00, 0x21},//mirror and vertical flip
	{0x7d, 0x0f},//MIPI:0f,DVP:07
	{0xeb, 0x05},//MIPI:05,DVP:04
	{0x0b, 0x20},//line length:1600
	{0x0c, 0x03},
	{0x03, 0xcf},
	{0x15, 0xca},
	{0x16, 0xd4},
	{0x19, 0xca},
	{0x1b, 0x3f},
	{0x1d, 0x3f},
	{0x24, 0x6c},
	{0x25, 0xc8},
	{0x26, 0xff},
	{0x27, 0x58},
	{0x29, 0x41},
	{0x2b, 0xaa},
	{0xe0, 0x16},//MIPICLK:360M
	{0xe1, 0xb4},
	{0xe2, 0x3a},
	{0xe3, 0x4a},
	{0xe4, 0x32},
	{0xe5, 0x86},
	{0xe8, 0x60},
	{0x5e, 0x32},//black Lock
	{0x59, 0x10},//black target B
	{0x5a, 0x10},//black target Gb
	{0x5b, 0x10},//black target R
	{0x5c, 0x10},//black target Gr
	{0x6a, 0x1f},//again
	{0x6b, 0x01},//integration time:2step
	{0x6c, 0xc2},//integration time
	{0x6d, 0x14},//the min value of global gain.
	{REG_NULL, 0x00},
};

/* sync mode regs */
static __maybe_unused const struct regval bf314a_interal_sync_master_start_regs[] = {
	{REG_NULL, 0x00},
};

static __maybe_unused const struct regval bf314a_interal_sync_master_stop_regs[] = {
	{REG_NULL, 0x00},
};

static __maybe_unused const struct regval bf314a_interal_sync_slaver_start_regs[] = {
	{REG_NULL, 0x00},
};

static __maybe_unused const struct regval bf314a_interal_sync_slaver_stop_regs[] = {
	{REG_NULL, 0x00},
};

static const struct bf314a_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x01c2,
		.hts_def = 0x320 * 2,
		.vts_def = 0x2ee,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = bf314a_linear_10_1280x720_30fps_regs,
		.hdr_mode = NO_HDR,
		.bpp = 10,
		.mipi_freq_idx = 0,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x01c2,
		.hts_def = 0x320 * 2,
		.vts_def = 0x2ee,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = bf314a_linear_10_1280x720_60fps_regs,
		.hdr_mode = NO_HDR,
		.bpp = 10,
		.mipi_freq_idx = 1,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const s64 link_freq_menu_items[] = {
	BF314A_LINK_FREQ_180M,
	BF314A_LINK_FREQ_360M,
};

static const char *const bf314a_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int bf314a_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"bf314a write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int bf314a_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i = 0;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = bf314a_write_reg(client, regs[i].addr, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int bf314a_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		"bf314a read reg:0x%x failed !\n", reg);

	return ret;
}

static int bf314a_set_gain_reg(struct bf314a *bf314a, u32 total_gain)
{
	struct device *dev = &bf314a->client->dev;
	int ret = 0;
	u32 temp_gain = 0;

	dev_dbg(dev, "total_gain = 0x%04x!\n", total_gain);
	if (total_gain < BF314A_GAIN_MIN)
		total_gain = BF314A_GAIN_MIN;

	if (total_gain <= BF314A_GAIN_MAX)
		temp_gain = ((total_gain << 4) / 0x40) - 1;
	else
		temp_gain = ((total_gain << 4) / 0x40) - 2;

	ret = bf314a_write_reg(bf314a->client,
			       BF314A_ANALOG_GAIN_REG, temp_gain);
	return ret;
}

static int bf314a_set_hdrae(struct bf314a *bf314a,
			    struct preisp_hdrae_exp_s *ae)
{
	int ret = 0;


	return ret;
}

static int bf314a_get_reso_dist(const struct bf314a_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct bf314a_mode *
bf314a_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = bf314a_get_reso_dist(&supported_modes[i], framefmt);
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

static int bf314a_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct bf314a *bf314a = to_bf314a(sd);
	const struct bf314a_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&bf314a->mutex);

	mode = bf314a_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&bf314a->mutex);
		return -ENOTTY;
#endif
	} else {
		bf314a->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(bf314a->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(bf314a->vblank, vblank_def,
					 BF314A_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(bf314a->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_menu_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * BF314A_LANES;
		__v4l2_ctrl_s_ctrl_int64(bf314a->pixel_rate, pixel_rate);
		bf314a->cur_fps = mode->max_fps;
		bf314a->cur_vts = mode->vts_def;
	}

	mutex_unlock(&bf314a->mutex);

	return 0;
}

static int bf314a_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct bf314a *bf314a = to_bf314a(sd);
	const struct bf314a_mode *mode = bf314a->cur_mode;

	mutex_lock(&bf314a->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&bf314a->mutex);
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
	mutex_unlock(&bf314a->mutex);

	return 0;
}

static int bf314a_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int bf314a_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int bf314a_enable_test_pattern(struct bf314a *bf314a, u32 pattern)
{
	u8 val = 0;
	int ret = 0;

	//  Bit[7:0]: 8'h00: output normal image;
	//  8'hxx: output control by bit[6:5];
	//  Bit[6:5]: 2'h00: output bar pattern;
	//	2'h01: output gradual pattern;
	//	2'h10: output manual pattern;
	//	2'h11: output normal image;
	//  Bit[4]:
	//	1'b0: vertica pattern;
	//	1'b1: horizontal pattern;
	//  Bit[1:0]: gradual gray pattern mode control.
	ret = bf314a_read_reg(bf314a->client, BF314A_REG_TEST_PATTERN, &val);
	if (pattern)
		val |= BF314A_TEST_PATTERN_BIT_MASK;
	else
		val &= ~BF314A_TEST_PATTERN_BIT_MASK;

	ret |= bf314a_write_reg(bf314a->client, BF314A_REG_TEST_PATTERN, val);
	return ret;
}

static int bf314a_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct bf314a *bf314a = to_bf314a(sd);
	const struct bf314a_mode *mode = bf314a->cur_mode;

	if (bf314a->streaming)
		fi->interval = bf314a->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct bf314a_mode *bf314a_find_mode(struct bf314a *bf314a, int fps)
{
	const struct bf314a_mode *mode = NULL;
	const struct bf314a_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == bf314a->cur_mode->width &&
		    mode->height == bf314a->cur_mode->height &&
		    mode->hdr_mode == bf314a->cur_mode->hdr_mode &&
		    mode->bus_fmt == bf314a->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int bf314a_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct bf314a *bf314a = to_bf314a(sd);
	const struct bf314a_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	int fps;

	if (bf314a->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = bf314a_find_mode(bf314a, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	bf314a->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(bf314a->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(bf314a->vblank, vblank_def,
				 BF314A_VTS_MAX - mode->height,
				 1, vblank_def);
	bf314a->cur_fps = mode->max_fps;
	return 0;
}

static int bf314a_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct bf314a *bf314a = to_bf314a(sd);
	const struct bf314a_mode *mode = bf314a->cur_mode;
	u32 val = 1 << (BF314A_LANES - 1) |
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

static void bf314a_get_module_inf(struct bf314a *bf314a,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, BF314A_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, bf314a->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, bf314a->len_name, sizeof(inf->base.lens));
}

static int bf314a_get_channel_info(struct bf314a *bf314a, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = bf314a->cur_mode->vc[ch_info->index];
	ch_info->width = bf314a->cur_mode->width;
	ch_info->height = bf314a->cur_mode->height;
	ch_info->bus_fmt = bf314a->cur_mode->bus_fmt;
	return 0;
}

static int bf314a_set_setting(struct bf314a *bf314a, struct rk_sensor_setting *setting)
{
	int i = 0;
	int cur_fps = 0;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	const struct bf314a_mode *mode = NULL;
	const struct bf314a_mode *match = NULL;

	dev_info(&bf314a->client->dev,
		 "sensor setting: %d x %d, fps:%d fmt:%d, mode:%d\n",
		 setting->width, setting->height,
		 setting->fps, setting->fmt, setting->mode);

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
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
		dev_info(&bf314a->client->dev, "-----%s: match the support mode, mode idx:%d-----\n",
			 __func__, i);
		bf314a->cur_mode = mode;

		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(bf314a->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(bf314a->vblank, vblank_def,
					 BF314A_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(bf314a->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_menu_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * BF314A_LANES;
		__v4l2_ctrl_s_ctrl_int64(bf314a->pixel_rate, pixel_rate);
		dev_info(&bf314a->client->dev, "freq_idx:%d pixel_rate:%lld\n",
			 mode->mipi_freq_idx, pixel_rate);

		bf314a->cur_vts = mode->vts_def;
		bf314a->cur_fps = mode->max_fps;

		dev_info(&bf314a->client->dev, "hts_def:%d cur_vts:%d cur_fps:%d\n",
			 mode->hts_def, mode->vts_def,
			 bf314a->cur_fps.denominator / bf314a->cur_fps.numerator);
	} else {
		dev_err(&bf314a->client->dev, "couldn't match the support modes\n");
		return -EINVAL;
	}

	return 0;
}

static long bf314a_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct bf314a *bf314a = to_bf314a(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	struct rk_sensor_setting *setting;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;
	u32 *sync_mode = NULL;
	int cur_best_fit = -1;
	int cur_best_fit_dist = -1;
	int cur_dist, cur_fps, dst_fps;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		bf314a_get_module_inf(bf314a, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = bf314a->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		if (hdr->hdr_mode == bf314a->cur_mode->hdr_mode)
			return 0;
		w = bf314a->cur_mode->width;
		h = bf314a->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(bf314a->cur_mode->max_fps.denominator,
					    bf314a->cur_mode->max_fps.numerator);
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode &&
			    supported_modes[i].bus_fmt == bf314a->cur_mode->bus_fmt) {
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
			dev_err(&bf314a->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			bf314a->cur_mode = &supported_modes[cur_best_fit];
			w = bf314a->cur_mode->hts_def - bf314a->cur_mode->width;
			h = bf314a->cur_mode->vts_def - bf314a->cur_mode->height;
			__v4l2_ctrl_modify_range(bf314a->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(bf314a->vblank, h,
						 BF314A_VTS_MAX - bf314a->cur_mode->height, 1, h);
			bf314a->cur_fps = bf314a->cur_mode->max_fps;
			bf314a->cur_vts = bf314a->cur_mode->vts_def;
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		bf314a_set_hdrae(bf314a, arg);
		if (bf314a->cam_sw_inf)
			memcpy(&bf314a->cam_sw_inf->hdr_ae, (struct preisp_hdrae_exp_s *)(arg),
			       sizeof(struct preisp_hdrae_exp_s));
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (bf314a->standby_hw) { // hardware standby
			if (stream) {
				if (!IS_ERR(bf314a->pwdn_gpio))
					gpiod_set_value_cansleep(bf314a->pwdn_gpio, 1);
				ret = bf314a_write_reg(bf314a->client, BF314A_REG_MIPI_CTRL,
						BF314A_MIPI_CTRL_ON);


				ret |= bf314a_write_reg(bf314a->client, BF314A_REG_CTRL_MODE,
						BF314A_MODE_STREAMING);

				dev_info(&bf314a->client->dev, "quickstream, streaming on: exit hw standby mode\n");
				bf314a->is_standby = false;
			} else {
				ret = bf314a_write_reg(bf314a->client, BF314A_REG_CTRL_MODE,
						BF314A_MODE_SW_STANDBY);

				ret |= bf314a_write_reg(bf314a->client, BF314A_REG_MIPI_CTRL,
							BF314A_MIPI_CTRL_OFF);

				if (!IS_ERR(bf314a->pwdn_gpio))
					gpiod_set_value_cansleep(bf314a->pwdn_gpio, 0);

				dev_info(&bf314a->client->dev, "quickstream, streaming off: enter hw standby mode\n");
				bf314a->is_standby = true;
			}
		} else {	// software standby
			if (stream) {
				ret = bf314a_write_reg(bf314a->client, BF314A_REG_MIPI_CTRL,
						BF314A_MIPI_CTRL_ON);

				ret |= bf314a_write_reg(bf314a->client, BF314A_REG_CTRL_MODE,
						BF314A_MODE_STREAMING);
				dev_info(&bf314a->client->dev, "quickstream, streaming on: exit soft standby mode\n");
			} else {
				ret = bf314a_write_reg(bf314a->client, BF314A_REG_CTRL_MODE,
						BF314A_MODE_SW_STANDBY);

				ret |= bf314a_write_reg(bf314a->client, BF314A_REG_MIPI_CTRL,
						BF314A_MIPI_CTRL_OFF);
				dev_info(&bf314a->client->dev, "quickstream, streaming off: enter soft standby mode\n");
			}
		}

		break;
	case RKMODULE_GET_SYNC_MODE:
		sync_mode = (u32 *)arg;
		*sync_mode = bf314a->sync_mode;
		break;
	case RKMODULE_SET_SYNC_MODE:
		sync_mode = (u32 *)arg;
		if (sync_mode) {
			bf314a->sync_mode = *sync_mode;
			dev_info(&bf314a->client->dev, "set sync mode is: %s\n",
				 ((*sync_mode == EXTERNAL_MASTER_MODE) ||
				  (*sync_mode == SLAVE_MODE)) ? "secondary" : "primary");
		} else {
			dev_info(&bf314a->client->dev, "set sync mode is: NO_SYNC_MODE\n");
		}
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = bf314a_get_channel_info(bf314a, ch_info);
		break;
	case RKCIS_CMD_SELECT_SETTING:
		setting = (struct rk_sensor_setting *)arg;
		ret = bf314a_set_setting(bf314a, setting);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long bf314a_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	struct rkmodule_channel_info *ch_info;
	struct rk_sensor_setting *setting;
	long ret;
	u32 stream = 0;
	u32 sync_mode;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = bf314a_ioctl(sd, cmd, inf);
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
			ret = bf314a_ioctl(sd, cmd, cfg);
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

		ret = bf314a_ioctl(sd, cmd, hdr);
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
			ret = bf314a_ioctl(sd, cmd, hdr);
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
			ret = bf314a_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = bf314a_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_SYNC_MODE:
		ret = bf314a_ioctl(sd, cmd, &sync_mode);
		if (!ret) {
			ret = copy_to_user(up, &sync_mode, sizeof(u32));
			if (ret)
				ret = -EFAULT;
		}
		break;
	case RKMODULE_SET_SYNC_MODE:
		ret = copy_from_user(&sync_mode, up, sizeof(u32));
		if (!ret)
			ret = bf314a_ioctl(sd, cmd, &sync_mode);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = bf314a_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	case RKCIS_CMD_SELECT_SETTING:
		setting = kzalloc(sizeof(*setting), GFP_KERNEL);
		if (!setting) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(setting, up, sizeof(*setting));
		if (!ret)
			ret = bf314a_ioctl(sd, cmd, setting);
		else
			ret = -EFAULT;
		kfree(setting);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __bf314a_start_stream(struct bf314a *bf314a)
{
	int ret = 0;

	dev_info(&bf314a->client->dev,
		 "%dx%d@%d, mode %d, vts 0x%x\n",
		 bf314a->cur_mode->width,
		 bf314a->cur_mode->height,
		 bf314a->cur_fps.denominator / bf314a->cur_fps.numerator,
		 bf314a->cur_mode->hdr_mode,
		 bf314a->cur_vts);
	if (!bf314a->is_thunderboot) {
		ret = bf314a_write_array(bf314a->client, bf314a->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&bf314a->ctrl_handler);
		if (ret)
			return ret;
		if (bf314a->has_init_exp && bf314a->cur_mode->hdr_mode != NO_HDR) {
			ret = bf314a_ioctl(&bf314a->subdev, PREISP_CMD_SET_HDRAE_EXP,
					   &bf314a->init_hdrae_exp);
			if (ret) {
				dev_err(&bf314a->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}

		if (bf314a->sync_mode == INTERNAL_MASTER_MODE ||
		    bf314a->sync_mode == SOFT_SYNC_MODE) {
			ret |= bf314a_write_array(bf314a->client,
						  bf314a_interal_sync_master_start_regs);
		} else if (bf314a->sync_mode == EXTERNAL_MASTER_MODE) {
			ret |= bf314a_write_array(bf314a->client,
						  bf314a_interal_sync_slaver_start_regs);
		} else if (bf314a->sync_mode == SLAVE_MODE) {
			ret |= bf314a_write_array(bf314a->client,
						  bf314a_interal_sync_slaver_start_regs);
		}
		if (ret) {
			dev_err(&bf314a->client->dev,
				"write sync regs failed\n");
			return ret;
		}
	}

	if (bf314a->sync_mode == NO_SYNC_MODE)
		ret |= bf314a_write_reg(bf314a->client, BF314A_REG_CTRL_MODE,
					BF314A_MODE_STREAMING);
	return ret;
}

static int __bf314a_stop_stream(struct bf314a *bf314a)
{
	int ret = 0;

	bf314a->has_init_exp = false;

	if (bf314a->is_thunderboot) {
		bf314a->is_first_streamoff = true;
		pm_runtime_put(&bf314a->client->dev);
	} else {
		if (bf314a->sync_mode == INTERNAL_MASTER_MODE)
			ret |= bf314a_write_array(bf314a->client,
						  bf314a_interal_sync_master_stop_regs);
		else if (bf314a->sync_mode == EXTERNAL_MASTER_MODE)
			ret |= bf314a_write_array(bf314a->client,
						  bf314a_interal_sync_slaver_stop_regs);
		else if (bf314a->sync_mode == SLAVE_MODE)
			ret |= bf314a_write_array(bf314a->client,
						  bf314a_interal_sync_slaver_stop_regs);
	}

	ret |= bf314a_write_reg(bf314a->client, BF314A_REG_CTRL_MODE,
				BF314A_MODE_SW_STANDBY);
	return ret;
}

static int __bf314a_power_on(struct bf314a *bf314a);
static int bf314a_s_stream(struct v4l2_subdev *sd, int on)
{
	struct bf314a *bf314a = to_bf314a(sd);
	struct i2c_client *client = bf314a->client;
	int ret = 0;

	mutex_lock(&bf314a->mutex);
	on = !!on;
	if (on == bf314a->streaming)
		goto unlock_and_return;

	if (on) {
		if (bf314a->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			bf314a->is_thunderboot = false;
			__bf314a_power_on(bf314a);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __bf314a_start_stream(bf314a);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__bf314a_stop_stream(bf314a);
		pm_runtime_put(&client->dev);
	}

	bf314a->streaming = on;

unlock_and_return:
	mutex_unlock(&bf314a->mutex);

	return ret;
}

static int bf314a_s_power(struct v4l2_subdev *sd, int on)
{
	struct bf314a *bf314a = to_bf314a(sd);
	struct i2c_client *client = bf314a->client;
	int ret = 0;

	mutex_lock(&bf314a->mutex);

	/* If the power state is not modified - no work to do. */
	if (bf314a->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!bf314a->is_thunderboot) {
			ret = bf314a_write_array(bf314a->client, bf314a_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		bf314a->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		bf314a->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&bf314a->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 bf314a_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, BF314A_XVCLK_FREQ / 1000 / 1000);
}

static int __bf314a_power_on(struct bf314a *bf314a)
{
	int ret;
	u32 delay_us;
	struct device *dev = &bf314a->client->dev;

	if (!IS_ERR_OR_NULL(bf314a->pins_default)) {
		ret = pinctrl_select_state(bf314a->pinctrl,
					   bf314a->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(bf314a->xvclk, BF314A_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(bf314a->xvclk) != BF314A_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(bf314a->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	cam_sw_regulator_bulk_init(bf314a->cam_sw_inf, BF314A_NUM_SUPPLIES, bf314a->supplies);

	if (bf314a->is_thunderboot)
		return 0;

	if (!IS_ERR(bf314a->reset_gpio))
		gpiod_set_value_cansleep(bf314a->reset_gpio, 0);

	ret = regulator_bulk_enable(BF314A_NUM_SUPPLIES, bf314a->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(bf314a->reset_gpio))
		gpiod_set_value_cansleep(bf314a->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(bf314a->pwdn_gpio))
		gpiod_set_value_cansleep(bf314a->pwdn_gpio, 1);

	if (!IS_ERR(bf314a->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = bf314a_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(bf314a->xvclk);

	return ret;
}

static void __bf314a_power_off(struct bf314a *bf314a)
{
	int ret;
	struct device *dev = &bf314a->client->dev;

	clk_disable_unprepare(bf314a->xvclk);
	if (bf314a->is_thunderboot) {
		if (bf314a->is_first_streamoff) {
			bf314a->is_thunderboot = false;
			bf314a->is_first_streamoff = false;
		} else {
			return;
		}
	}
	if (!IS_ERR(bf314a->pwdn_gpio))
		gpiod_set_value_cansleep(bf314a->pwdn_gpio, 0);
	clk_disable_unprepare(bf314a->xvclk);
	if (!IS_ERR(bf314a->reset_gpio))
		gpiod_set_value_cansleep(bf314a->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(bf314a->pins_sleep)) {
		ret = pinctrl_select_state(bf314a->pinctrl,
					   bf314a->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(BF314A_NUM_SUPPLIES, bf314a->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int __maybe_unused bf314a_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf314a *bf314a = to_bf314a(sd);

	if (bf314a->standby_hw) {
		dev_info(dev, "resume standby!");
		return 0;
	}

	cam_sw_prepare_wakeup(bf314a->cam_sw_inf, dev);

	usleep_range(4000, 5000);
	cam_sw_write_array(bf314a->cam_sw_inf);

	if (__v4l2_ctrl_handler_setup(&bf314a->ctrl_handler))
		dev_err(dev, "__v4l2_ctrl_handler_setup fail!");

	if (bf314a->has_init_exp && bf314a->cur_mode != NO_HDR) {// hdr mode
		ret = bf314a_ioctl(&bf314a->subdev, PREISP_CMD_SET_HDRAE_EXP,
					&bf314a->cam_sw_inf->hdr_ae);
		if (ret) {
			dev_err(&bf314a->client->dev, "set exp fail in hdr mode\n");
			return ret;
		}
	}

	return 0;
}

static int __maybe_unused bf314a_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf314a *bf314a = to_bf314a(sd);

	if (bf314a->standby_hw) {
		dev_info(dev, "suspend standby!");
		return 0;
	}

	cam_sw_write_array_cb_init(bf314a->cam_sw_inf, client,
				   (void *)bf314a->cur_mode->reg_list,
				   (sensor_write_array)bf314a_write_array);
	cam_sw_prepare_sleep(bf314a->cam_sw_inf);

	return 0;
}
#else
#define bf314a_resume NULL
#define bf314a_suspend NULL
#endif

static int bf314a_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf314a *bf314a = to_bf314a(sd);

	return __bf314a_power_on(bf314a);
}

static int bf314a_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf314a *bf314a = to_bf314a(sd);

	__bf314a_power_off(bf314a);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int bf314a_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct bf314a *bf314a = to_bf314a(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct bf314a_mode *def_mode = &supported_modes[0];

	mutex_lock(&bf314a->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&bf314a->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int bf314a_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops bf314a_pm_ops = {
	SET_RUNTIME_PM_OPS(bf314a_runtime_suspend,
	bf314a_runtime_resume, NULL)
#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(bf314a_suspend, bf314a_resume)
#endif
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops bf314a_internal_ops = {
	.open = bf314a_open,
};
#endif

static const struct v4l2_subdev_core_ops bf314a_core_ops = {
	.s_power = bf314a_s_power,
	.ioctl = bf314a_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = bf314a_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops bf314a_video_ops = {
	.s_stream = bf314a_s_stream,
	.g_frame_interval = bf314a_g_frame_interval,
	.s_frame_interval = bf314a_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops bf314a_pad_ops = {
	.enum_mbus_code = bf314a_enum_mbus_code,
	.enum_frame_size = bf314a_enum_frame_sizes,
	.enum_frame_interval = bf314a_enum_frame_interval,
	.get_fmt = bf314a_get_fmt,
	.set_fmt = bf314a_set_fmt,
	.get_mbus_config = bf314a_g_mbus_config,
};

static const struct v4l2_subdev_ops bf314a_subdev_ops = {
	.core	= &bf314a_core_ops,
	.video	= &bf314a_video_ops,
	.pad	= &bf314a_pad_ops,
};


static int bf314a_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct bf314a *bf314a = container_of(ctrl->handler,
					     struct bf314a, ctrl_handler);
	struct i2c_client *client = bf314a->client;
	s64 max;
	int ret = 0;
	u32 vts = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = bf314a->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(bf314a->exposure,
					 bf314a->exposure->minimum, max,
					 bf314a->exposure->step,
					 bf314a->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure value 0x%x\n", ctrl->val);
		ret = bf314a_write_reg(bf314a->client,
				       BF314A_REG_EXPOSURE_H,
				       (ctrl->val >> 8) & 0xff);
		ret |= bf314a_write_reg(bf314a->client,
					BF314A_REG_EXPOSURE_L,
					ctrl->val & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain value 0x%x\n", ctrl->val);
		ret = bf314a_set_gain_reg(bf314a, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set blank value 0x%x\n", ctrl->val);
		// here through adjust dummy line value to meet expected vblanking
		// dummy line default value is 0x0c, should add based this value
		vts = ctrl->val + bf314a->cur_mode->height - bf314a->cur_vts + 0x0c;
		ret = bf314a_write_reg(bf314a->client, BF314A_REG_VTS_H,
				       (vts >> 8) & 0xff);
		ret |= bf314a_write_reg(bf314a->client, BF314A_REG_VTS_L,
					vts & 0xff);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = bf314a_enable_test_pattern(bf314a, ctrl->val);
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

static const struct v4l2_ctrl_ops bf314a_ctrl_ops = {
	.s_ctrl = bf314a_set_ctrl,
};

static int bf314a_initialize_controls(struct bf314a *bf314a)
{
	const struct bf314a_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u64 dst_pixel_rate = 0;
	u32 h_blank;
	int ret;

	handler = &bf314a->ctrl_handler;
	mode = bf314a->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &bf314a->mutex;

	bf314a->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
			    ARRAY_SIZE(link_freq_menu_items) - 1, 0,
			    link_freq_menu_items);
	if (bf314a->link_freq)
		bf314a->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	__v4l2_ctrl_s_ctrl(bf314a->link_freq, mode->mipi_freq_idx);

	if (mode->mipi_freq_idx == 0)
		dst_pixel_rate = PIXEL_RATE_WITH_180M_10BIT;
	else if (mode->mipi_freq_idx == 1)
		dst_pixel_rate = PIXEL_RATE_WITH_180M_10BIT;

	bf314a->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE, 0,
					       PIXEL_RATE_WITH_180M_10BIT, 1,
					       dst_pixel_rate);

	h_blank = mode->hts_def - mode->width;
	bf314a->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (bf314a->hblank)
		bf314a->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	bf314a->cur_fps = mode->max_fps;
	vblank_def = mode->vts_def - mode->height;
	bf314a->cur_vts = mode->vts_def;
	bf314a->vblank = v4l2_ctrl_new_std(handler, &bf314a_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   BF314A_VTS_MAX - mode->height,
					   1, vblank_def);
	exposure_max = 2 * mode->vts_def - 8;
	bf314a->exposure = v4l2_ctrl_new_std(handler, &bf314a_ctrl_ops,
					     V4L2_CID_EXPOSURE, BF314A_EXPOSURE_MIN,
					     exposure_max, BF314A_EXPOSURE_STEP,
					     mode->exp_def);
	bf314a->anal_gain = v4l2_ctrl_new_std(handler, &bf314a_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, BF314A_GAIN_MIN,
					      BF314A_GAIN_MAX, BF314A_GAIN_STEP,
					      BF314A_GAIN_DEFAULT);
	bf314a->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
			       &bf314a_ctrl_ops,
			       V4L2_CID_TEST_PATTERN,
			       ARRAY_SIZE(bf314a_test_pattern_menu) - 1,
			       0, 0, bf314a_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &bf314a_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &bf314a_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&bf314a->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	bf314a->subdev.ctrl_handler = handler;
	bf314a->has_init_exp = false;
	bf314a->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int bf314a_check_sensor_id(struct bf314a *bf314a,
				  struct i2c_client *client)
{
	struct device *dev = &bf314a->client->dev;
	u8 pid, ver = 0x00;
	int ret;
	unsigned short id;

	if (bf314a->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = bf314a_read_reg(client, BF314A_REG_CHIP_ID_H, &pid);
	if (ret) {
		dev_err(dev, "Read chip ID H register error\n");
		return -EIO;
	}

	ret = bf314a_read_reg(client, BF314A_REG_CHIP_ID_L, &ver);
	if (ret) {
		dev_err(dev, "Read chip ID L register error\n");
		return -EIO;
	}

	id = SENSOR_ID(pid, ver);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -EIO;
	}

	dev_info(dev, "Detect BF314A(0x%04x) sensor\n", id);

	return 0;
}

static int bf314a_configure_regulators(struct bf314a *bf314a)
{
	unsigned int i;

	for (i = 0; i < BF314A_NUM_SUPPLIES; i++)
		bf314a->supplies[i].supply = bf314a_supply_names[i];

	return devm_regulator_bulk_get(&bf314a->client->dev,
				       BF314A_NUM_SUPPLIES,
				       bf314a->supplies);
}

#ifdef CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP
static void find_terminal_resolution(struct bf314a *bf314a)
{
	int i = 0;
	const struct bf314a_mode *mode = NULL;
	u32 rk_cam_hdr = get_rk_cam_hdr();
	u32 rk_cam_w = get_rk_cam_w();
	u32 rk_cam_h = get_rk_cam_h();

	if (rk_cam_w == 0 || rk_cam_h == 0)
		goto err_find_res;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == rk_cam_w && mode->height == rk_cam_h &&
		    mode->hdr_mode == rk_cam_hdr) {
			bf314a->cur_mode = mode;
			return;
		}
	}
err_find_res:
	dev_err(&bf314a->client->dev, "not match %dx%d mode %d\n!",
		rk_cam_w, rk_cam_h, rk_cam_hdr);
	bf314a->cur_mode = &supported_modes[0];
}
#else
static void find_terminal_resolution(struct bf314a *bf314a)
{
	u32 hdr_mode = 0;
	struct device_node *node = bf314a->client->dev.of_node;
	int i = 0;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			bf314a->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		bf314a->cur_mode = &supported_modes[0];
}
#endif

static int bf314a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct bf314a *bf314a;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	const char *sync_mode_name = NULL;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	bf314a = devm_kzalloc(dev, sizeof(*bf314a), GFP_KERNEL);
	if (!bf314a)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &bf314a->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &bf314a->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &bf314a->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &bf314a->len_name);
	/* Compatible with non-standby mode if this attribute is not configured in dts*/
	of_property_read_u32(node, RKMODULE_CAMERA_STANDBY_HW,
			     &bf314a->standby_hw);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	dev_info(dev, "bf314a->standby_hw = %d\n", bf314a->standby_hw);

	ret = of_property_read_string(node, RKMODULE_CAMERA_SYNC_MODE,
				      &sync_mode_name);
	if (ret) {
		bf314a->sync_mode = NO_SYNC_MODE;
		dev_err(dev, "could not get sync mode!\n");
	} else {
		if (strcmp(sync_mode_name, RKMODULE_EXTERNAL_MASTER_MODE) == 0) {
			bf314a->sync_mode = EXTERNAL_MASTER_MODE;
			dev_info(dev, "sync_mode = [EXTERNAL_MASTER_MODE]\n");
		} else if (strcmp(sync_mode_name, RKMODULE_INTERNAL_MASTER_MODE) == 0) {
			bf314a->sync_mode = INTERNAL_MASTER_MODE;
			dev_info(dev, "sync_mode = [INTERNAL_MASTER_MODE]\n");
		} else if (strcmp(sync_mode_name, RKMODULE_SLAVE_MODE) == 0) {
			bf314a->sync_mode = SLAVE_MODE;
			dev_info(dev, "sync_mode = [SLAVE_MODE]\n");
		} else if (strcmp(sync_mode_name, RKMODULE_SOFT_SYNC_MODE) == 0) {
			bf314a->sync_mode = SOFT_SYNC_MODE;
			dev_info(dev, "sync_mode = [SOFT_SYNC_MODE]\n");
		} else {
			dev_info(dev, "sync_mode = [NO_SYNC_MODE]\n");
		}
	}

	bf314a->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	bf314a->client = client;

	find_terminal_resolution(bf314a);

	bf314a->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(bf314a->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	bf314a->reset_gpio = devm_gpiod_get(dev, "reset",
					    bf314a->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(bf314a->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	bf314a->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
					   bf314a->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(bf314a->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	bf314a->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(bf314a->pinctrl)) {
		bf314a->pins_default =
			pinctrl_lookup_state(bf314a->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(bf314a->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		bf314a->pins_sleep =
			pinctrl_lookup_state(bf314a->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(bf314a->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = bf314a_configure_regulators(bf314a);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&bf314a->mutex);

	sd = &bf314a->subdev;
	v4l2_i2c_subdev_init(sd, client, &bf314a_subdev_ops);
	ret = bf314a_initialize_controls(bf314a);
	if (ret)
		goto err_destroy_mutex;

	ret = __bf314a_power_on(bf314a);
	if (ret)
		goto err_free_handler;

	ret = bf314a_check_sensor_id(bf314a, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &bf314a_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	bf314a->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &bf314a->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	if (!bf314a->cam_sw_inf) {
		bf314a->cam_sw_inf = cam_sw_init();
		cam_sw_clk_init(bf314a->cam_sw_inf, bf314a->xvclk, BF314A_XVCLK_FREQ);
		cam_sw_reset_pin_init(bf314a->cam_sw_inf, bf314a->reset_gpio, 0);
		cam_sw_pwdn_pin_init(bf314a->cam_sw_inf, bf314a->pwdn_gpio, 1);
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(bf314a->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 bf314a->module_index, facing,
		 BF314A_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (bf314a->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__bf314a_power_off(bf314a);
err_free_handler:
	v4l2_ctrl_handler_free(&bf314a->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&bf314a->mutex);

	return ret;
}

static int bf314a_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf314a *bf314a = to_bf314a(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&bf314a->ctrl_handler);
	mutex_destroy(&bf314a->mutex);

	cam_sw_deinit(bf314a->cam_sw_inf);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__bf314a_power_off(bf314a);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id bf314a_of_match[] = {
	{ .compatible = "byd,bf314a" },
	{},
};
MODULE_DEVICE_TABLE(of, bf314a_of_match);
#endif

static const struct i2c_device_id bf314a_match_id[] = {
	{ "byd,bf314a", 0 },
	{ },
};

static struct i2c_driver bf314a_i2c_driver = {
	.driver = {
		.name = BF314A_NAME,
		.pm = &bf314a_pm_ops,
		.of_match_table = of_match_ptr(bf314a_of_match),
	},
	.probe		= bf314a_probe,
	.remove		= bf314a_remove,
	.id_table	= bf314a_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&bf314a_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&bf314a_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens bf314a sensor driver");
MODULE_LICENSE("GPL");
