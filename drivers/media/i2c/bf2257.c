// SPDX-License-Identifier: GPL-2.0
/*
 * bf2257 driver
 *
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 fix gain range.
 * V0.0X01.0X04 add enum_frame_interval function.
 * V0.0X01.0X05 add quick stream on/off.
 * V0.0X01.0X06 fix set vflip/hflip failed bug.
 * V0.0X01.0X07
 *	1. fix set double times exposue value failed issue.
 *	2. add some debug info.
 * V0.0X01.0X08
 *	1. add support wakeup & sleep for aov function
 *	2. using 60fps output default
 * V0.0X01.0X09 add support hw standby mode in aov
 * V0.0X01.0X0a modify hw standby resume new way
 * V0.0X01.0X0b add support sync mode
 * V0.0X01.0X0c fix pm_runtime issue in aov
 * V0.0X01.0X0d add support select sensor setting
 * V0.0X01.0X0e add 120fps 960*540 sensor setting
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x02, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define BF2257_LANES			1
#define BF2257_BITS_PER_SAMPLE		10
#define BF2257_LINK_FREQ_336M		336000000  //742.5Mbps

#define PIXEL_RATE_WITH_336M_10BIT	(BF2257_LINK_FREQ_336M * 2 * \
					BF2257_LANES / BF2257_BITS_PER_SAMPLE)


#define BF2257_XVCLK_FREQ		24000000

#define CHIP_ID				0x2257
#define BF2257_REG_CHIP_ID_H		0xfc
#define BF2257_REG_CHIP_ID_L		0xfd
#define SENSOR_ID(_msb, _lsb)	((_msb) << 8 | (_lsb))

#define BF2257_REG_CTRL_MODE		0xf3
#define BF2257_MODE_SW_STANDBY		0x01
#define BF2257_MODE_STREAMING		0x00

#define BF2257_REG_MIPI_CTRL		0x7D
#define BF2257_MIPI_CTRL_ON		0x0f
#define BF2257_MIPI_CTRL_OFF		0x07

#define BF2257_REG_EXPOSURE_H		0x6b
#define BF2257_REG_EXPOSURE_L		0x6c
#define	BF2257_EXPOSURE_MIN		1
#define	BF2257_EXPOSURE_STEP		1
#define BF2257_VTS_MAX			0x7fff

#define BF2257_ANALOG_GAIN_REG		0x6a
#define BF2257_GAIN_MIN			0x0040
#define BF2257_GAIN_MAX			0x200
#define BF2257_GAIN_STEP		4
#define BF2257_GAIN_DEFAULT		0x80
#define BF2257_LGAIN			0
#define BF2257_SGAIN			1


#define BF2257_REG_TEST_PATTERN		0x80
#define BF2257_TEST_PATTERN_BIT_MASK	BIT(3)

#define BF2257_REG_VTS_H		0x07
#define BF2257_REG_VTS_L		0x06

#define BF2257_MIRROR_FLIP_REG		0x00

#define BF2257_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x08 : VAL & 0xf7)
#define BF2257_FETCH_FLIP(VAL, ENABLE)	(ENABLE ? VAL | 0x04 : VAL & 0xfb)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFF //0xFFFF

#define BF2257_REG_VALUE_08BIT		1
#define BF2257_REG_VALUE_16BIT		2
#define BF2257_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define BF2257_NAME			"bf2257_rgb"

static const char *const bf2257_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define BF2257_NUM_SUPPLIES ARRAY_SIZE(bf2257_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct bf2257_mode {
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

struct bf2257 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[BF2257_NUM_SUPPLIES];

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
	const struct bf2257_mode *cur_mode;
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

#define to_bf2257(sd) container_of(sd, struct bf2257, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval bf2257_global_regs[] = {
	{0xf2, 0x01},
	{0x00, 0x0d},//Bit[3]: Mirror Bit[2]: vFlip  /// 0000 0001 -> 0000 1101
	{0x02, 0xb7},
	{0x1e, 0x04},
	{0x24, 0x60},//add 20221020
	{0xe0, 0x00},
	{0xe1, 0x01},
	{0xe2, 0x08},
	{0xe5, 0xe3},
	{0xe6, 0x60},
	{0xe7, 0x33},
	{0xe8, 0x12},
	{0xe9, 0x89},
	{0xea, 0x87},
	{0xeb, 0x80},
	{0xec, 0x91},
	{0xed, 0x60},
	{0xe3, 0x78},
	{0xe4, 0xe0},
	{0x06, 0x10},
	{0x07, 0x00},
	{0x0b, 0x80},
	{0x0c, 0x03},
	{0x59, 0x40},
	{0x5a, 0x40},
	{0x5b, 0x40},
	{0x5c, 0x40},
	{0x70, 0x08},
	{0x71, 0x07},
	{0x72, 0x12},
	{0x73, 0x09},
	{0x74, 0x08},
	{0x75, 0x06},
	{0x76, 0x20},
	{0x77, 0x02},
	{0x78, 0x10},
	{0x79, 0x09},
	{0x7a, 0x00},
	{0x7b, 0x00},
	{0x7c, 0x00},
	{0x7d, 0x0f},
	{0xca, 0x60},
	{0xcb, 0x40},
	{0xcc, 0x04},
	{0xcd, 0x44},
	{0xce, 0x04},
	{0xcf, 0xb4},
	{0x6a, 0x0f},//Global gain
	{0x6b, 0x04},
	{0x6c, 0xe1},//{0x6b,0x6c}:int_time
	{0x6d, 0x0f},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 60fps
 * mipi_datarate per lane 1008Mbps, 4lane
 */
static const struct regval __maybe_unused bf2257_linear_10_1920x1080_60fps_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 371.25Mbps, 2lane
 */
static const struct regval __maybe_unused bf2257_linear_10_1920x1080_30fps_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 742.5Mbps, HDR 2lane
 */
static const struct regval __maybe_unused bf2257_hdr_10_1920x1080_regs[] = {
	{REG_NULL, 0x00},
};

/* sync mode regs */
static __maybe_unused const struct regval __maybe_unused bf2257_interal_sync_master_start_regs[] = {
	{REG_NULL, 0x00},
};

static __maybe_unused const struct regval __maybe_unused bf2257_interal_sync_master_stop_regs[] = {
	{REG_NULL, 0x00},
};

static __maybe_unused const struct regval __maybe_unused bf2257_interal_sync_slaver_start_regs[] = {
	{REG_NULL, 0x00},
};

static __maybe_unused const struct regval __maybe_unused bf2257_interal_sync_slaver_stop_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 120fps
 * mipi_datarate per lane 371.25Mbps, 2lane liear
 */
static const struct regval __maybe_unused bf2257_linear_10_960x540_120fps_regs[] = {
	{REG_NULL, 0x00},
};

static const struct bf2257_mode supported_modes[] = {
//#if defined CONFIG_VIDEO_CAM_SLEEP_WAKEUP || defined CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP
	{
		.width = 1600,
		.height = 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0475,
		.hts_def = 0x0380 * 2,
		.vts_def = 0x04e2,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = bf2257_global_regs,
		.hdr_mode = NO_HDR,
		.bpp = 10,
		.mipi_freq_idx = 1,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
//#endif
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const s64 link_freq_menu_items[] = {
	BF2257_LINK_FREQ_336M,
};

static const char *const bf2257_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int bf2257_write_reg(struct i2c_client *client, u8 reg, u8 val)
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
		"bf2257 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int bf2257_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i = 0;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = bf2257_write_reg(client, regs[i].addr, regs[i].val);
	return ret;
}

/* Read registers up to 4 at a time */
static int bf2257_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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

	/* mode: 0 = lgain  1 = sgain */
	dev_err(&client->dev,
		"bf2257 read reg:0x%x failed !\n", reg);

	return ret;
}
static int bf2257_set_gain_reg(struct bf2257 *bf2257, u32 total_gain)
{
	struct device *dev = &bf2257->client->dev;
	int ret = 0;
	u32 temp_gain = 0;

	dev_dbg(dev, "total_gain = 0x%04x!\n", total_gain);
	if (total_gain < 0x40)
		total_gain = 0x40;

	if (total_gain <= 8 * 0x40)
		temp_gain = ((total_gain << 4) / 0x40) - 1;
	else
		temp_gain = ((total_gain << 4) / 0x40) - 2;

	ret = bf2257_write_reg(bf2257->client,
			       BF2257_ANALOG_GAIN_REG, temp_gain);
	return ret;
}

static int bf2257_set_hdrae(struct bf2257 *bf2257,
			    struct preisp_hdrae_exp_s *ae)
{
	int ret = 0;


	return ret;
}

static int bf2257_get_reso_dist(const struct bf2257_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct bf2257_mode *
bf2257_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = bf2257_get_reso_dist(&supported_modes[i], framefmt);
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

static int bf2257_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	const struct bf2257_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&bf2257->mutex);

	mode = bf2257_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&bf2257->mutex);
		return -ENOTTY;
#endif
	} else {
		bf2257->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(bf2257->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(bf2257->vblank, vblank_def,
					 BF2257_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(bf2257->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_menu_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * BF2257_LANES;
		__v4l2_ctrl_s_ctrl_int64(bf2257->pixel_rate, pixel_rate);
		bf2257->cur_fps = mode->max_fps;
		bf2257->cur_vts = mode->vts_def;
	}

	mutex_unlock(&bf2257->mutex);

	return 0;
}

static int bf2257_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	const struct bf2257_mode *mode = bf2257->cur_mode;

	mutex_lock(&bf2257->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&bf2257->mutex);
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
	mutex_unlock(&bf2257->mutex);

	return 0;
}

static int bf2257_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int bf2257_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int bf2257_enable_test_pattern(struct bf2257 *bf2257, u32 pattern)
{
	u8 val = 0;
	int ret = 0;

	ret = bf2257_read_reg(bf2257->client, BF2257_REG_TEST_PATTERN, &val);
	if (pattern)
		val |= BF2257_TEST_PATTERN_BIT_MASK;
	else
		val &= ~BF2257_TEST_PATTERN_BIT_MASK;

	ret |= bf2257_write_reg(bf2257->client, BF2257_REG_TEST_PATTERN, val);
	return ret;
}

static int bf2257_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	const struct bf2257_mode *mode = bf2257->cur_mode;

	if (bf2257->streaming)
		fi->interval = bf2257->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct bf2257_mode *bf2257_find_mode(struct bf2257 *bf2257, int fps)
{
	const struct bf2257_mode *mode = NULL;
	const struct bf2257_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == bf2257->cur_mode->width &&
		    mode->height == bf2257->cur_mode->height &&
		    mode->hdr_mode == bf2257->cur_mode->hdr_mode &&
		    mode->bus_fmt == bf2257->cur_mode->bus_fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
						    mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int bf2257_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	const struct bf2257_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	int fps;

	if (bf2257->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = bf2257_find_mode(bf2257, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	bf2257->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(bf2257->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(bf2257->vblank, vblank_def,
				 BF2257_VTS_MAX - mode->height,
				 1, vblank_def);
	bf2257->cur_fps = mode->max_fps;
	return 0;
}

static int bf2257_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	const struct bf2257_mode *mode = bf2257->cur_mode;
	u32 val = 1 << (BF2257_LANES - 1) |
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

static void bf2257_get_module_inf(struct bf2257 *bf2257,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, BF2257_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, bf2257->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, bf2257->len_name, sizeof(inf->base.lens));
}

static int bf2257_get_channel_info(struct bf2257 *bf2257, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = bf2257->cur_mode->vc[ch_info->index];
	ch_info->width = bf2257->cur_mode->width;
	ch_info->height = bf2257->cur_mode->height;
	ch_info->bus_fmt = bf2257->cur_mode->bus_fmt;
	return 0;
}

static int bf2257_set_setting(struct bf2257 *bf2257, struct rk_sensor_setting *setting)
{
	int i = 0;
	int cur_fps = 0;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	const struct bf2257_mode *mode = NULL;
	const struct bf2257_mode *match = NULL;

	dev_info(&bf2257->client->dev,
		 "sensor setting: %d x %d, fps:%d fmt:%d, mode:%d\n",
		 setting->width, setting->height,
		 setting->fps, setting->fmt, setting->mode);

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == setting->width &&
		    mode->height == setting->height &&
		    mode->hdr_mode == setting->mode &&
		    mode->bus_fmt == setting->fmt) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
						    mode->max_fps.numerator);
			if (cur_fps == setting->fps) {
				match = mode;
				break;
			}
		}
	}

	if (match) {
		dev_info(&bf2257->client->dev, "-----%s: match the support mode, mode idx:%d-----\n",
			 __func__, i);
		bf2257->cur_mode = mode;

		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(bf2257->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(bf2257->vblank, vblank_def,
					 BF2257_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(bf2257->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_menu_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * BF2257_LANES;
		__v4l2_ctrl_s_ctrl_int64(bf2257->pixel_rate, pixel_rate);
		dev_info(&bf2257->client->dev, "freq_idx:%d pixel_rate:%lld\n",
			 mode->mipi_freq_idx, pixel_rate);

		bf2257->cur_vts = mode->vts_def;
		bf2257->cur_fps = mode->max_fps;

		dev_info(&bf2257->client->dev, "hts_def:%d cur_vts:%d cur_fps:%d\n",
			 mode->hts_def, mode->vts_def,
			 bf2257->cur_fps.denominator / bf2257->cur_fps.numerator);
	} else {
		dev_err(&bf2257->client->dev, "couldn't match the support modes\n");
		return -EINVAL;
	}

	return 0;
}

static long bf2257_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
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
		bf2257_get_module_inf(bf2257, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = bf2257->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		if (hdr->hdr_mode == bf2257->cur_mode->hdr_mode)
			return 0;
		w = bf2257->cur_mode->width;
		h = bf2257->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(bf2257->cur_mode->max_fps.denominator,
					    bf2257->cur_mode->max_fps.numerator);
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode &&
			    supported_modes[i].bus_fmt == bf2257->cur_mode->bus_fmt) {
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
			dev_err(&bf2257->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			bf2257->cur_mode = &supported_modes[cur_best_fit];
			w = bf2257->cur_mode->hts_def - bf2257->cur_mode->width;
			h = bf2257->cur_mode->vts_def - bf2257->cur_mode->height;
			__v4l2_ctrl_modify_range(bf2257->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(bf2257->vblank, h,
						 BF2257_VTS_MAX - bf2257->cur_mode->height, 1, h);
			bf2257->cur_fps = bf2257->cur_mode->max_fps;
			bf2257->cur_vts = bf2257->cur_mode->vts_def;
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		bf2257_set_hdrae(bf2257, arg);
		if (bf2257->cam_sw_inf)
			memcpy(&bf2257->cam_sw_inf->hdr_ae, (struct preisp_hdrae_exp_s *)(arg),
			       sizeof(struct preisp_hdrae_exp_s));
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (bf2257->standby_hw) { // hardware standby
			if (stream) {
				if (!IS_ERR(bf2257->pwdn_gpio))
					gpiod_set_value_cansleep(bf2257->pwdn_gpio, 1);
				ret = bf2257_write_reg(bf2257->client, BF2257_REG_MIPI_CTRL,
						       BF2257_MIPI_CTRL_ON);


				ret |= bf2257_write_reg(bf2257->client, BF2257_REG_CTRL_MODE,
							BF2257_MODE_STREAMING);

				dev_info(&bf2257->client->dev, "quickstream, streaming on: exit hw standby mode\n");
				bf2257->is_standby = false;
			} else {
				ret = bf2257_write_reg(bf2257->client, BF2257_REG_CTRL_MODE,
						       BF2257_MODE_SW_STANDBY);

				ret |= bf2257_write_reg(bf2257->client, BF2257_REG_MIPI_CTRL,
							BF2257_MIPI_CTRL_OFF);

				if (!IS_ERR(bf2257->pwdn_gpio))
					gpiod_set_value_cansleep(bf2257->pwdn_gpio, 0);

				dev_info(&bf2257->client->dev, "quickstream, streaming off: enter hw standby mode\n");
				bf2257->is_standby = true;
			}
		} else {	// software standby
			if (stream) {
				ret = bf2257_write_reg(bf2257->client, BF2257_REG_MIPI_CTRL,
						       BF2257_MIPI_CTRL_ON);

				ret |= bf2257_write_reg(bf2257->client, BF2257_REG_CTRL_MODE,
							BF2257_MODE_STREAMING);
				dev_info(&bf2257->client->dev, "quickstream, streaming on: exit soft standby mode\n");
			} else {
				ret = bf2257_write_reg(bf2257->client, BF2257_REG_CTRL_MODE,
						       BF2257_MODE_SW_STANDBY);

				ret |= bf2257_write_reg(bf2257->client, BF2257_REG_MIPI_CTRL,
							BF2257_MIPI_CTRL_OFF);
				dev_info(&bf2257->client->dev, "quickstream, streaming off: enter soft standby mode\n");
			}
		}

		break;
	case RKMODULE_GET_SYNC_MODE:
		sync_mode = (u32 *)arg;
		*sync_mode = bf2257->sync_mode;
		break;
	case RKMODULE_SET_SYNC_MODE:
		sync_mode = (u32 *)arg;
		if (sync_mode) {
			bf2257->sync_mode = *sync_mode;
			dev_info(&bf2257->client->dev, "set sync mode is: %s\n",
				 ((*sync_mode == EXTERNAL_MASTER_MODE) ||
				  (*sync_mode == SLAVE_MODE)) ? "secondary" : "primary");
		} else {
			dev_info(&bf2257->client->dev, "set sync mode is: NO_SYNC_MODE\n");
		}
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = bf2257_get_channel_info(bf2257, ch_info);
		break;
	case RKCIS_CMD_SELECT_SETTING:
		setting = (struct rk_sensor_setting *)arg;
		ret = bf2257_set_setting(bf2257, setting);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long bf2257_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = bf2257_ioctl(sd, cmd, inf);
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
			ret = bf2257_ioctl(sd, cmd, cfg);
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

		ret = bf2257_ioctl(sd, cmd, hdr);
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
			ret = bf2257_ioctl(sd, cmd, hdr);
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
			ret = bf2257_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = bf2257_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_SYNC_MODE:
		ret = bf2257_ioctl(sd, cmd, &sync_mode);
		if (!ret) {
			ret = copy_to_user(up, &sync_mode, sizeof(u32));
			if (ret)
				ret = -EFAULT;
		}
		break;
	case RKMODULE_SET_SYNC_MODE:
		ret = copy_from_user(&sync_mode, up, sizeof(u32));
		if (!ret)
			ret = bf2257_ioctl(sd, cmd, &sync_mode);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = bf2257_ioctl(sd, cmd, ch_info);
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
			ret = bf2257_ioctl(sd, cmd, setting);
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

static int __bf2257_start_stream(struct bf2257 *bf2257)
{
	int ret = 0;

	dev_info(&bf2257->client->dev,
		 "%dx%d@%d, mode %d, vts 0x%x\n",
		 bf2257->cur_mode->width,
		 bf2257->cur_mode->height,
		 bf2257->cur_fps.denominator / bf2257->cur_fps.numerator,
		 bf2257->cur_mode->hdr_mode,
		 bf2257->cur_vts);
	if (!bf2257->is_thunderboot) {
		ret = bf2257_write_array(bf2257->client, bf2257->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&bf2257->ctrl_handler);
		if (ret)
			return ret;
		if (bf2257->has_init_exp && bf2257->cur_mode->hdr_mode != NO_HDR) {
			ret = bf2257_ioctl(&bf2257->subdev, PREISP_CMD_SET_HDRAE_EXP,
					   &bf2257->init_hdrae_exp);
			if (ret) {
				dev_err(&bf2257->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}

		if (bf2257->sync_mode == INTERNAL_MASTER_MODE ||
		    bf2257->sync_mode == SOFT_SYNC_MODE) {
			ret |= bf2257_write_array(bf2257->client,
						  bf2257_interal_sync_master_start_regs);
		} else if (bf2257->sync_mode == EXTERNAL_MASTER_MODE) {
			ret |= bf2257_write_array(bf2257->client,
						  bf2257_interal_sync_slaver_start_regs);
		} else if (bf2257->sync_mode == SLAVE_MODE) {
			ret |= bf2257_write_array(bf2257->client,
						  bf2257_interal_sync_slaver_start_regs);
		}
		if (ret) {
			dev_err(&bf2257->client->dev,
				"write sync regs failed\n");
			return ret;
		}
	}

	if (bf2257->sync_mode == NO_SYNC_MODE)
		ret |= bf2257_write_reg(bf2257->client, BF2257_REG_CTRL_MODE,
					BF2257_MODE_STREAMING);
	return ret;
}

static int __bf2257_stop_stream(struct bf2257 *bf2257)
{
	int ret = 0;

	bf2257->has_init_exp = false;

	if (bf2257->is_thunderboot) {
		bf2257->is_first_streamoff = true;
		pm_runtime_put(&bf2257->client->dev);
	} else {
		if (bf2257->sync_mode == INTERNAL_MASTER_MODE)
			ret |= bf2257_write_array(bf2257->client,
						  bf2257_interal_sync_master_stop_regs);
		else if (bf2257->sync_mode == EXTERNAL_MASTER_MODE)
			ret |= bf2257_write_array(bf2257->client,
						  bf2257_interal_sync_slaver_stop_regs);
		else if (bf2257->sync_mode == SLAVE_MODE)
			ret |= bf2257_write_array(bf2257->client,
						  bf2257_interal_sync_slaver_stop_regs);
	}

	ret |= bf2257_write_reg(bf2257->client, BF2257_REG_CTRL_MODE,
				BF2257_MODE_SW_STANDBY);
	return ret;
}

static int __bf2257_power_on(struct bf2257 *bf2257);
static int bf2257_s_stream(struct v4l2_subdev *sd, int on)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	struct i2c_client *client = bf2257->client;
	int ret = 0;

	mutex_lock(&bf2257->mutex);
	on = !!on;
	if (on == bf2257->streaming)
		goto unlock_and_return;

	if (on) {
		if (bf2257->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			bf2257->is_thunderboot = false;
			__bf2257_power_on(bf2257);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __bf2257_start_stream(bf2257);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__bf2257_stop_stream(bf2257);
		pm_runtime_put(&client->dev);
	}

	bf2257->streaming = on;

unlock_and_return:
	mutex_unlock(&bf2257->mutex);

	return ret;
}

static int bf2257_s_power(struct v4l2_subdev *sd, int on)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	struct i2c_client *client = bf2257->client;
	int ret = 0;

	mutex_lock(&bf2257->mutex);

	/* If the power state is not modified - no work to do. */
	if (bf2257->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!bf2257->is_thunderboot) {
			ret = bf2257_write_array(bf2257->client, bf2257_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		bf2257->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		bf2257->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&bf2257->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 bf2257_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, BF2257_XVCLK_FREQ / 1000 / 1000);
}

static int __bf2257_power_on(struct bf2257 *bf2257)
{
	int ret;
	u32 delay_us;
	struct device *dev = &bf2257->client->dev;

	if (!IS_ERR_OR_NULL(bf2257->pins_default)) {
		ret = pinctrl_select_state(bf2257->pinctrl,
					   bf2257->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(bf2257->xvclk, BF2257_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(bf2257->xvclk) != BF2257_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(bf2257->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	cam_sw_regulator_bulk_init(bf2257->cam_sw_inf, BF2257_NUM_SUPPLIES, bf2257->supplies);

	if (bf2257->is_thunderboot)
		return 0;

	if (!IS_ERR(bf2257->reset_gpio))
		gpiod_set_value_cansleep(bf2257->reset_gpio, 0);

	ret = regulator_bulk_enable(BF2257_NUM_SUPPLIES, bf2257->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(bf2257->reset_gpio))
		gpiod_set_value_cansleep(bf2257->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(bf2257->pwdn_gpio))
		gpiod_set_value_cansleep(bf2257->pwdn_gpio, 1);

	if (!IS_ERR(bf2257->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = bf2257_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(bf2257->xvclk);

	return ret;
}

static void __bf2257_power_off(struct bf2257 *bf2257)
{
	int ret;
	struct device *dev = &bf2257->client->dev;

	clk_disable_unprepare(bf2257->xvclk);
	if (bf2257->is_thunderboot) {
		if (bf2257->is_first_streamoff) {
			bf2257->is_thunderboot = false;
			bf2257->is_first_streamoff = false;
		} else {
			return;
		}
	}
	if (!IS_ERR(bf2257->pwdn_gpio))
		gpiod_set_value_cansleep(bf2257->pwdn_gpio, 0);
	clk_disable_unprepare(bf2257->xvclk);
	if (!IS_ERR(bf2257->reset_gpio))
		gpiod_set_value_cansleep(bf2257->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(bf2257->pins_sleep)) {
		ret = pinctrl_select_state(bf2257->pinctrl,
					   bf2257->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(BF2257_NUM_SUPPLIES, bf2257->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int __maybe_unused bf2257_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf2257 *bf2257 = to_bf2257(sd);

	if (bf2257->standby_hw) {
		dev_info(dev, "resume standby!");
		return 0;
	}

	cam_sw_prepare_wakeup(bf2257->cam_sw_inf, dev);
	usleep_range(4000, 5000);
	cam_sw_write_array(bf2257->cam_sw_inf);

	if (__v4l2_ctrl_handler_setup(&bf2257->ctrl_handler))
		dev_err(dev, "__v4l2_ctrl_handler_setup fail!");

	if (bf2257->has_init_exp && bf2257->cur_mode != NO_HDR) {	// hdr mode
		ret = bf2257_ioctl(&bf2257->subdev, PREISP_CMD_SET_HDRAE_EXP,
					&bf2257->cam_sw_inf->hdr_ae);
		if (ret) {
			dev_err(&bf2257->client->dev, "set exp fail in hdr mode\n");
			return ret;
		}
	}

	return 0;
}

static int __maybe_unused bf2257_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf2257 *bf2257 = to_bf2257(sd);

	if (bf2257->standby_hw) {
		dev_info(dev, "suspend standby!");
		return 0;
	}

	cam_sw_write_array_cb_init(bf2257->cam_sw_inf, client,
				   (void *)bf2257->cur_mode->reg_list,
				   (sensor_write_array)bf2257_write_array);
	cam_sw_prepare_sleep(bf2257->cam_sw_inf);

	return 0;
}
#else
#define bf2257_resume NULL
#define bf2257_suspend NULL
#endif

static int bf2257_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf2257 *bf2257 = to_bf2257(sd);

	return __bf2257_power_on(bf2257);
}

static int bf2257_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf2257 *bf2257 = to_bf2257(sd);

	__bf2257_power_off(bf2257);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int bf2257_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct bf2257 *bf2257 = to_bf2257(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct bf2257_mode *def_mode = &supported_modes[0];

	mutex_lock(&bf2257->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&bf2257->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int bf2257_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops bf2257_pm_ops = {
	SET_RUNTIME_PM_OPS(bf2257_runtime_suspend,
	bf2257_runtime_resume, NULL)
#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(bf2257_suspend, bf2257_resume)
#endif
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops bf2257_internal_ops = {
	.open = bf2257_open,
};
#endif

static const struct v4l2_subdev_core_ops bf2257_core_ops = {
	.s_power = bf2257_s_power,
	.ioctl = bf2257_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = bf2257_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops bf2257_video_ops = {
	.s_stream = bf2257_s_stream,
	.g_frame_interval = bf2257_g_frame_interval,
	.s_frame_interval = bf2257_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops bf2257_pad_ops = {
	.enum_mbus_code = bf2257_enum_mbus_code,
	.enum_frame_size = bf2257_enum_frame_sizes,
	.enum_frame_interval = bf2257_enum_frame_interval,
	.get_fmt = bf2257_get_fmt,
	.set_fmt = bf2257_set_fmt,
	.get_mbus_config = bf2257_g_mbus_config,
};

static const struct v4l2_subdev_ops bf2257_subdev_ops = {
	.core	= &bf2257_core_ops,
	.video	= &bf2257_video_ops,
	.pad	= &bf2257_pad_ops,
};

/*
 *static void bf2257_modify_fps_info(struct bf2257 *bf2257)
 *{
 *	const struct bf2257_mode *mode = bf2257->cur_mode;
 *
 *	bf2257->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def/
 *				       bf2257->cur_vts;
 *}
 */

static int bf2257_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct bf2257 *bf2257 = container_of(ctrl->handler,
					     struct bf2257, ctrl_handler);
	struct i2c_client *client = bf2257->client;
	s64 max;
	int ret = 0;
	u32 vts = 0;
	//u8 val = 0;
	u32 vb = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = bf2257->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(bf2257->exposure,
					 bf2257->exposure->minimum, max,
					 bf2257->exposure->step,
					 bf2257->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */

		ret = bf2257_write_reg(bf2257->client,
				       BF2257_REG_EXPOSURE_H,
				       (ctrl->val >> 8) & 0xff);
		ret |= bf2257_write_reg(bf2257->client,
					BF2257_REG_EXPOSURE_L,
					ctrl->val & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain value 0x%x\n", ctrl->val);
		ret = bf2257_set_gain_reg(bf2257, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + bf2257->cur_mode->height;
		vb = vts - 1250;
		ret = bf2257_write_reg(bf2257->client, BF2257_REG_VTS_H,
				       (vb >> 8) & 0xff);
		ret |= bf2257_write_reg(bf2257->client, BF2257_REG_VTS_L,
					vb & 0xff);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = bf2257_enable_test_pattern(bf2257, ctrl->val);
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

static const struct v4l2_ctrl_ops bf2257_ctrl_ops = {
	.s_ctrl = bf2257_set_ctrl,
};

static int bf2257_initialize_controls(struct bf2257 *bf2257)
{
	const struct bf2257_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u64 dst_pixel_rate = 0;
	u32 h_blank;
	int ret;

	handler = &bf2257->ctrl_handler;
	mode = bf2257->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &bf2257->mutex;

	bf2257->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
			    ARRAY_SIZE(link_freq_menu_items) - 1, 0,
			    link_freq_menu_items);
	if (bf2257->link_freq)
		bf2257->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	__v4l2_ctrl_s_ctrl(bf2257->link_freq, mode->mipi_freq_idx);

	if (mode->mipi_freq_idx == 0)
		dst_pixel_rate = PIXEL_RATE_WITH_336M_10BIT;
	else if (mode->mipi_freq_idx == 1)
		dst_pixel_rate = PIXEL_RATE_WITH_336M_10BIT;

	bf2257->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE, 0,
					       PIXEL_RATE_WITH_336M_10BIT, 1,
					       dst_pixel_rate);

	h_blank = mode->hts_def - mode->width;
	bf2257->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (bf2257->hblank)
		bf2257->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	bf2257->cur_fps = mode->max_fps;
	vblank_def = mode->vts_def - mode->height;
	bf2257->cur_vts = mode->vts_def;
	bf2257->vblank = v4l2_ctrl_new_std(handler, &bf2257_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   BF2257_VTS_MAX - mode->height,
					   1, vblank_def);
	exposure_max = 2 * mode->vts_def - 8;
	bf2257->exposure = v4l2_ctrl_new_std(handler, &bf2257_ctrl_ops,
					     V4L2_CID_EXPOSURE, BF2257_EXPOSURE_MIN,
					     exposure_max, BF2257_EXPOSURE_STEP,
					     mode->exp_def);
	bf2257->anal_gain = v4l2_ctrl_new_std(handler, &bf2257_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, BF2257_GAIN_MIN,
					      BF2257_GAIN_MAX, BF2257_GAIN_STEP,
					      BF2257_GAIN_DEFAULT);
	bf2257->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
			       &bf2257_ctrl_ops,
			       V4L2_CID_TEST_PATTERN,
			       ARRAY_SIZE(bf2257_test_pattern_menu) - 1,
			       0, 0, bf2257_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &bf2257_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &bf2257_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&bf2257->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	bf2257->subdev.ctrl_handler = handler;
	bf2257->has_init_exp = false;
	bf2257->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int bf2257_check_sensor_id(struct bf2257 *bf2257,
				  struct i2c_client *client)
{
	struct device *dev = &bf2257->client->dev;
	u8 pid, ver = 0x00;
	int ret;
	unsigned short id;

	ret = bf2257_read_reg(client, BF2257_REG_CHIP_ID_H, &pid);
	if (ret) {
		dev_err(dev, "Read chip ID H register error\n");
		return -EIO;
	}

	ret = bf2257_read_reg(client, BF2257_REG_CHIP_ID_L, &ver);
	if (ret) {
		dev_err(dev, "Read chip ID L register error\n");
		return -EIO;
	}

	id = SENSOR_ID(pid, ver);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -EIO;
	}

	dev_info(dev, "detected gc%04x sensor\n", id);

	return 0;
}

static int bf2257_configure_regulators(struct bf2257 *bf2257)
{
	unsigned int i;

	for (i = 0; i < BF2257_NUM_SUPPLIES; i++)
		bf2257->supplies[i].supply = bf2257_supply_names[i];

	return devm_regulator_bulk_get(&bf2257->client->dev,
				       BF2257_NUM_SUPPLIES,
				       bf2257->supplies);
}

#ifdef CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP
static void find_terminal_resolution(struct bf2257 *bf2257)
{
	int i = 0;
	const struct bf2257_mode *mode = NULL;
	u32 rk_cam_hdr = get_rk_cam_hdr();
	u32 rk_cam_w = get_rk_cam_w();
	u32 rk_cam_h = get_rk_cam_h();

	if (rk_cam_w == 0 || rk_cam_h == 0)
		goto err_find_res;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == rk_cam_w && mode->height == rk_cam_h &&
		    mode->hdr_mode == rk_cam_hdr) {
			bf2257->cur_mode = mode;
			return;
		}
	}
err_find_res:
	dev_err(&bf2257->client->dev, "not match %dx%d mode %d\n!",
		rk_cam_w, rk_cam_h, rk_cam_hdr);
	bf2257->cur_mode = &supported_modes[0];
}
#else
static void find_terminal_resolution(struct bf2257 *bf2257)
{
	u32 hdr_mode = 0;
	struct device_node *node = bf2257->client->dev.of_node;
	int i = 0;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			bf2257->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		bf2257->cur_mode = &supported_modes[0];
}
#endif

static int bf2257_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct bf2257 *bf2257;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	const char *sync_mode_name = NULL;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	bf2257 = devm_kzalloc(dev, sizeof(*bf2257), GFP_KERNEL);
	if (!bf2257)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &bf2257->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &bf2257->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &bf2257->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &bf2257->len_name);
	/* Compatible with non-standby mode if this attribute is not configured in dts*/
	of_property_read_u32(node, RKMODULE_CAMERA_STANDBY_HW,
			     &bf2257->standby_hw);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	dev_info(dev, "bf2257->standby_hw = %d\n", bf2257->standby_hw);

	ret = of_property_read_string(node, RKMODULE_CAMERA_SYNC_MODE,
				      &sync_mode_name);
	if (ret) {
		bf2257->sync_mode = NO_SYNC_MODE;
		dev_err(dev, "could not get sync mode!\n");
	} else {
		if (strcmp(sync_mode_name, RKMODULE_EXTERNAL_MASTER_MODE) == 0) {
			bf2257->sync_mode = EXTERNAL_MASTER_MODE;
			dev_info(dev, "sync_mode = [EXTERNAL_MASTER_MODE]\n");
		} else if (strcmp(sync_mode_name, RKMODULE_INTERNAL_MASTER_MODE) == 0) {
			bf2257->sync_mode = INTERNAL_MASTER_MODE;
			dev_info(dev, "sync_mode = [INTERNAL_MASTER_MODE]\n");
		} else if (strcmp(sync_mode_name, RKMODULE_SLAVE_MODE) == 0) {
			bf2257->sync_mode = SLAVE_MODE;
			dev_info(dev, "sync_mode = [SLAVE_MODE]\n");
		} else if (strcmp(sync_mode_name, RKMODULE_SOFT_SYNC_MODE) == 0) {
			bf2257->sync_mode = SOFT_SYNC_MODE;
			dev_info(dev, "sync_mode = [SOFT_SYNC_MODE]\n");
		} else {
			dev_info(dev, "sync_mode = [NO_SYNC_MODE]\n");
		}
	}

	bf2257->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	bf2257->client = client;

	find_terminal_resolution(bf2257);

	bf2257->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(bf2257->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	bf2257->reset_gpio = devm_gpiod_get(dev, "reset",
					    bf2257->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(bf2257->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	bf2257->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
					   bf2257->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(bf2257->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	bf2257->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(bf2257->pinctrl)) {
		bf2257->pins_default =
			pinctrl_lookup_state(bf2257->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(bf2257->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		bf2257->pins_sleep =
			pinctrl_lookup_state(bf2257->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(bf2257->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = bf2257_configure_regulators(bf2257);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&bf2257->mutex);

	sd = &bf2257->subdev;
	v4l2_i2c_subdev_init(sd, client, &bf2257_subdev_ops);
	ret = bf2257_initialize_controls(bf2257);
	if (ret)
		goto err_destroy_mutex;

	ret = __bf2257_power_on(bf2257);
	if (ret)
		goto err_free_handler;

	ret = bf2257_check_sensor_id(bf2257, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &bf2257_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	bf2257->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &bf2257->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	if (!bf2257->cam_sw_inf) {
		bf2257->cam_sw_inf = cam_sw_init();
		cam_sw_clk_init(bf2257->cam_sw_inf, bf2257->xvclk, BF2257_XVCLK_FREQ);
		cam_sw_reset_pin_init(bf2257->cam_sw_inf, bf2257->reset_gpio, 0);
		cam_sw_pwdn_pin_init(bf2257->cam_sw_inf, bf2257->pwdn_gpio, 1);
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(bf2257->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 bf2257->module_index, facing,
		 BF2257_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (bf2257->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__bf2257_power_off(bf2257);
err_free_handler:
	v4l2_ctrl_handler_free(&bf2257->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&bf2257->mutex);

	return ret;
}

static int bf2257_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf2257 *bf2257 = to_bf2257(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&bf2257->ctrl_handler);
	mutex_destroy(&bf2257->mutex);

	cam_sw_deinit(bf2257->cam_sw_inf);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__bf2257_power_off(bf2257);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id bf2257_of_match[] = {
	{ .compatible = "byd,rgb-bf2257" },
	{},
};
MODULE_DEVICE_TABLE(of, bf2257_of_match);
#endif

static const struct i2c_device_id bf2257_match_id[] = {
	{ "byd,rgb-bf2257", 0 },
	{ },
};

static struct i2c_driver bf2257_i2c_driver = {
	.driver = {
		.name = BF2257_NAME,
		.pm = &bf2257_pm_ops,
		.of_match_table = of_match_ptr(bf2257_of_match),
	},
	.probe		= bf2257_probe,
	.remove		= bf2257_remove,
	.id_table	= bf2257_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&bf2257_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&bf2257_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens bf2257 sensor driver");
MODULE_LICENSE("GPL");
