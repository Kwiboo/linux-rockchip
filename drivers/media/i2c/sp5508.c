// SPDX-License-Identifier: GPL-2.0
/*
 * sp5508 driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
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
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SP5508_CHIP_ID			0x5508
#define SP5508_REG_CHIP_ID_H		0x02
#define SP5508_REG_CHIP_ID_L		0x03

#define SP5508_XVCLK_FREQ		24000000
#define BITS_PER_SAMPLE			10
#define MIPI_LINK_FREQ_360M		360000000
#define MIPI_LINK_FREQ_468M		468000000	//936Mbps
#define SP5508_LANES			2

/*pclk 94.5Mhz */
#define PIXEL_RATE_WITH_468M		(MIPI_LINK_FREQ_468M * SP5508_LANES * 2 / BITS_PER_SAMPLE)

#define SP5508_REG_PAGE_SELECT		0xfd

#define SP5508_REG_EXP_H		0x03
#define SP5508_REG_EXP_L		0x04
#define SP5508_EXPOSURE_MIN		4
#define SP5508_EXPOSURE_STEP		1

// again & dgain :P1
#define SP5508_REG_AGAIN_L		0x24	// Global analog gain(1X~31X)  [7:0]
#define SP5508_REG_AGAIN_H		0x38	// Global analog gain(1X~31X)  [8]
#define SP5508_REG_DGAIN		0x39	// Global digital gain(1X~32X)
#define SP5508_REG_DGAIN_R_CH		0x40	// Digital gain,red channel(1~2X)
#define SP5508_REG_DGAIN_GR_CH		0x41	// Digital gain,gr channel(1~2X)
#define SP5508_REG_DGAIN_GB_CH		0x42	// Digital gain,gb channel(1~2X)
#define SP5508_REG_DGAIN_B_CH		0x43	// Digital gain,blue channel(1~2X)

#define SP5508_GAIN_MIN			16	// 1X * 16
#define SP5508_GAIN_MAX			15872	// 31X * 32X * 16 = 15872
#define SP5508_GAIN_STEP		1
#define SP5508_GAIN_DEFAULT		512

// Frame_length & row_length :P1
#define SP5508_REG_HTS_H		0x41
#define SP5508_REG_HTS_L		0x42
#define SP5508_REG_VTS_H		0x0e	// Frame length for manual frame length setting,
// used with P1:0x0d[4] enabled
#define SP5508_REG_VTS_L		0x0f
#define SP5508_REG_VTS_H_RO		0x4e	// READ ONLY
#define SP5508_REG_VTS_L_RO		0x4f	// READ ONLY

#define SP5508_REG_VBLANK_H		0x05
#define SP5508_REG_VBLANK_L		0x06
#define SP5508_REG_HBLANK_H		0x09
#define SP5508_REG_HBLANK_L		0x0a

#define SP5508_REG_FRM_EXP_EN		0x0d	// frame_exp_seperate_en
#define SP5508_VTS_MAX			0x7fff

// Soft reset register: P0
#define SP5508_REG_SOFTRESET		0x20

#define SP5508_REG_RESTART		0x01

#define SP5508_REG_EXTER_SYNC_CTL	0x40

#define SP5508_REG_CTRL_MODE		0xa0
#define SP5508_MODE_SW_STANDBY		0x00
#define SP5508_MODE_STREAMING		0x01

#define SP5508_REG_TEST_PATTERN		0xb6
#define SP5508_TEST_PATTERN_BIT_MASK	0x01

#define SP5508_FLIP_REG			0x3f
#define MIRROR_BIT_MASK			BIT(0)
#define FLIP_BIT_MASK			BIT(1)

#define SP5508_REG_BAYER_ORDER		0x5d
#define SP5508_REG_BR_FIRST		0x5e

#define SP5508_FETCH_EXP_H(VAL)		(((VAL) >> 8) & 0xFF)
#define SP5508_FETCH_EXP_L(VAL)		((VAL) & 0xFF)

#define SP5508_FETCH_AGAIN(VAL)		((VAL) & 0xFF)
#define SP5508_FETCH_DGAIN_H(VAL)	(((VAL) >> 8) & 0xFF)
#define SP5508_FETCH_DGAIN_L(VAL)	((VAL) & 0xFF)

#define SP5508_NAME			"sp5508"

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define REG_DELAY			0xFE
#define REG_NULL			0xFF

#define SENSOR_ID(_msb, _lsb)		((_msb) << 8 | (_lsb))

static const char *const SP5508_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SP5508_NUM_SUPPLIES ARRAY_SIZE(SP5508_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct sp5508_mode {
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

struct sp5508 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SP5508_NUM_SUPPLIES];
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
	const struct sp5508_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct otp_info		*otp;
	struct rkmodule_inf	module_inf;
	struct rkmodule_awb_cfg	awb_cfg;
	u32			cur_vts;
	u8			flip;
	bool			is_thunderboot;
	bool			is_first_streamoff;
};

#define to_sp5508(sd) container_of(sd, struct sp5508, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval sp5508_global_regs[] = {
	{REG_NULL, 0x00},
};

/**
 * xclk 24Mhz
 * MIPI 2Lane, 936Mbps
 * 15fps
 *
 */
static const struct regval sp5508_linear10bit_2592x1944_15fps_regs[] = {
	{0xfd, 0x00},
	{0x36, 0x01},
	{0xfd, 0x00},
	{0x36, 0x00},
	{0xfd, 0x00},
	{0x20, 0x00},

	{REG_DELAY, 0x05},
	{0xfd, 0x00},
	{0x2e, 0x24},    /*pll clk 468M,dac clk 234M,row clk 58.5M, col clk 93.6M */
	{0x2f, 0x01},
	{0xfd, 0x01},
	{0x03, 0x06},    /* 30ms */
	{0x04, 0x36},
	{0x06, 0x01},
	{0x24, 0xf8},    /* analog gain [7~0] */
	{0x39, 0x12},
	{0x31, 0x00},    /* bit[4]=1 bypassdsp */
	{0x05, 0x07},    //dummy line
	{0x06, 0xca},
	{0x01, 0x01},
	{0x11, 0x60},    /* rst_num1 */
	{0x33, 0xb0},    /* rst_num2 */
	{0x12, 0x03},
	{0x13, 0xd0},    /*scn_num_8lsb */
	{0x45, 0x1e},    /*scnt_dds_8lsb */
	{0x16, 0xea},
	{0x19, 0xf7},
	{0x1a, 0x5f},
	{0x1c, 0x0c},
	{0x1d, 0x06},
	{0x1e, 0x09},    /*pcp rst on 3.35v,sig_pbias_boost off */
	{0x20, 0x07},    /*ncp -0.95v */
	{0x2a, 0x0f},    /*adc range 0.7v */
	{0x2c, 0x10},    /*bit[4]=1 output high 8bit */
	{0x25, 0x0d},    /*sunspot off */
	{0x26, 0x0d},    /*pcp tx off,pcp rowsel on 3.35v */
	{0x27, 0x08},    /*even/odd mode */
	{0x29, 0x01},    /*pldo off */
	{0x2d, 0x06},
	{0x55, 0x14},
	{0x56, 0x00},
	{0x57, 0x17},
	{0x59, 0x00},
	{0x5a, 0x04},
	{0x50, 0x10},
	{0x53, 0x0e},
	{0x6b, 0x10},
	{0x5c, 0x20},
	{0x5d, 0x00},    /*p12 */
	{0x5e, 0x06},    /*p13   */
	{0x66, 0x38},    /*sc1 */
	{0x68, 0x30},    /*sc2 */
	{0x71, 0x3f},    /*rst d0 */
	{0x72, 0x05},    /*rst d1  */
	{0x73, 0x3e},    /*sig d0 */
	{0x81, 0x22},
	{0x8a, 0x66},
	{0x8b, 0x66},
	{0xc0, 0x02},
	{0xc1, 0x02},
	{0xc2, 0x02},
	{0xc3, 0x02},
	{0xc4, 0x82},   /*blc_exp   */
	{0xc5, 0x82},
	{0xc6, 0x82},
	{0xc7, 0x82},
	{0xfb, 0x4b},   /*blc off   */
	{0xf0, 0x1c},   /* offset */
	{0xf1, 0x1c},
	{0xf2, 0x1c},
	{0xf3, 0x1c},
	{0xb1, 0xad},    /*mipi_12_sw en,shutdowna=1 */
	{0xb6, 0x42},    /*remove hot spot right side     */
	{0xa1, 0x04},    /*tx_speed  */
	{0xfd, 0x02},
	{0x14, 0x04},
	{0x15, 0x60},
	{0x16, 0x04},
	{0x17, 0x60},
	{0x34, 0xc8},
	{0x60, 0x99},
	{0x93, 0x03},
	{0x18, 0xe0},
	{0x19, 0xa0},
	{0xfd, 0x04},
	{0x31, 0x4b},
	{0x32, 0x4b},
	{0xfd, 0x03},
	{0xc0, 0x00},
	{0xfd, 0x02},
	{0xa0, 0x00},
	{0xa1, 0x08},
	{0xa2, 0x07},
	{0xa3, 0x98},
	{0xa4, 0x00},
	{0xa5, 0x08},
	{0xa6, 0x0a},
	{0xa7, 0x20},
	{0xfd, 0x01},
	{0x8e, 0x0a},    /*mipi output size  */
	{0x8f, 0x20},
	{0x90, 0x07},
	{0x91, 0x98},
	{0xfd, 0x01},
	// {0xa0, 0x01},	 /*mipi buf en */
	// {0xfd, 0x01},
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
static const struct sp5508_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 3976,
		.hts_def = 1562,
		.vts_def = 3984,
		.reg_list = sp5508_linear10bit_2592x1944_15fps_regs,
		.hdr_mode = NO_HDR,
		.bpp = 10,
		.mipi_freq_idx = 0,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	MIPI_LINK_FREQ_468M,
};

static const char *const sp5508_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* sensor register write */
static int sp5508_write_reg(struct i2c_client *client, u8 reg, u8 val)
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

	/* workaround reset no ack lead to write regs list error */
	if (reg == 0x20 && ret < 0)
		return 0;

	dev_err(&client->dev, "write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int sp5508_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	int i = 0, ret = 0;

	while (regs[i].addr != REG_NULL) {
		if (regs[i].addr == REG_DELAY) {
			dev_warn(&client->dev, "%s will delay %d ms\n", __func__, regs[i].val);
			usleep_range(regs[i].val * 1000, (regs[i].val + 1)  * 1000);
		} else {
			ret = sp5508_write_reg(client, regs[i].addr, regs[i].val);
			if (ret) {
				dev_err(&client->dev, "%s : %d failed !\n", __func__, i);
				break;
			}
		}
		i++;
	}

	dev_info(&client->dev, "%s: write regs array success!\n", __func__);
	return ret;
}

/* sensor register read */
static int sp5508_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		"sp5508 read reg(0x%x) failed !\n", reg);

	return ret;
}

static int sp5508_get_reso_dist(const struct sp5508_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sp5508_mode *
sp5508_find_best_fit(struct sp5508 *sp5508, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < sp5508->cfg_num; i++) {
		dist = sp5508_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
		    (supported_modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sp5508_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	const struct sp5508_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	mutex_lock(&sp5508->mutex);

	mode = sp5508_find_best_fit(sp5508, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sp5508->mutex);
		return -ENOTTY;
#endif
	} else {
		sp5508->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sp5508->hblank, h_blank,
					 h_blank, 1, h_blank);

		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sp5508->vblank, vblank_def,
					 SP5508_VTS_MAX - mode->height,
					 1, vblank_def);
		if (mode->hdr_mode == NO_HDR) {
			dst_link_freq = 0;
			dst_pixel_rate = PIXEL_RATE_WITH_468M;
		}
		__v4l2_ctrl_s_ctrl_int64(sp5508->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(sp5508->link_freq,
				   dst_link_freq);
	}

	mutex_unlock(&sp5508->mutex);

	return 0;
}

static int sp5508_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	const struct sp5508_mode *mode = sp5508->cur_mode;

	mutex_lock(&sp5508->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sp5508->mutex);
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
	mutex_unlock(&sp5508->mutex);

	return 0;
}

static int sp5508_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct sp5508 *sp5508 = to_sp5508(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sp5508->cur_mode->bus_fmt;

	return 0;
}

static int sp5508_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sp5508 *sp5508 = to_sp5508(sd);

	if (fse->index >= sp5508->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sp5508_enable_test_pattern(struct sp5508 *sp5508, u32 pattern)
{
	u8 val = 0;
	int ret = 0;

	ret = sp5508_write_reg(sp5508->client, SP5508_REG_PAGE_SELECT, 0x1);
	ret |= sp5508_read_reg(sp5508->client, SP5508_REG_TEST_PATTERN, &val);
	if (pattern)
		val |= SP5508_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SP5508_TEST_PATTERN_BIT_MASK;

	ret |= sp5508_write_reg(sp5508->client, SP5508_REG_TEST_PATTERN, val);
	return ret;
}

static int sp5508_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	const struct sp5508_mode *mode = sp5508->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int sp5508_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *config)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	const struct sp5508_mode *mode = sp5508->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (SP5508_LANES - 1) |
		      V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void sp5508_get_module_inf(struct sp5508 *sp5508,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SP5508_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sp5508->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sp5508->len_name, sizeof(inf->base.lens));
}

static long sp5508_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sp5508_get_module_inf(sp5508, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode != 0)
			ret = -1;
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		ret = -1;
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sp5508->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		/* setting page 0 select before mipi en */
		ret = sp5508_write_reg(sp5508->client, SP5508_REG_PAGE_SELECT, 0x01);
		if (stream)
			ret |= sp5508_write_reg(sp5508->client, SP5508_REG_CTRL_MODE,
					       SP5508_MODE_STREAMING);
		else
			ret |= sp5508_write_reg(sp5508->client, SP5508_REG_CTRL_MODE,
					       SP5508_MODE_SW_STANDBY);

		v4l2_info(sd,
			  "RKMODULE_SET_QUICK_STREAM, stream %s\n", stream ? "On" : "Off");
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sp5508_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sp5508_ioctl(sd, cmd, inf);
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

		ret = sp5508_ioctl(sd, cmd, hdr);
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
			ret = sp5508_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		ret = sp5508_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif


static int __sp5508_start_stream(struct sp5508 *sp5508)
{
	int ret = 0;

	if (!sp5508->is_thunderboot) {
		ret |= sp5508_write_array(sp5508->client, sp5508->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&sp5508->ctrl_handler);
		if (ret)
			return ret;
	}
	dev_err(&sp5508->client->dev,
		"sp5508->is_thunderboot = %d\n", sp5508->is_thunderboot);
	ret |= sp5508_write_reg(sp5508->client, SP5508_REG_PAGE_SELECT, 0x01);
	ret |= sp5508_write_reg(sp5508->client, SP5508_REG_CTRL_MODE,
				SP5508_MODE_STREAMING);

	return ret;
}

static int __sp5508_stop_stream(struct sp5508 *sp5508)
{
	if (sp5508->is_thunderboot) {
		sp5508->is_first_streamoff = true;
		pm_runtime_put(&sp5508->client->dev);
	}

	sp5508_write_reg(sp5508->client, SP5508_REG_PAGE_SELECT, 0x01);
	return sp5508_write_reg(sp5508->client, SP5508_REG_CTRL_MODE,
				SP5508_MODE_SW_STANDBY);
}

static int __sp5508_power_on(struct sp5508 *sp5508);
static int sp5508_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	struct i2c_client *client = sp5508->client;
	int ret = 0;

	mutex_lock(&sp5508->mutex);
	on = !!on;
	if (on == sp5508->streaming)
		goto unlock_and_return;

	if (on) {
		if (sp5508->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			sp5508->is_thunderboot = false;
			__sp5508_power_on(sp5508);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sp5508_start_stream(sp5508);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		} else {
			v4l2_info(sd, "start stream success!\n");
		}
	} else {
		__sp5508_stop_stream(sp5508);
		pm_runtime_put(&client->dev);
	}

	sp5508->streaming = on;

unlock_and_return:
	mutex_unlock(&sp5508->mutex);

	return ret;
}

static int sp5508_s_power(struct v4l2_subdev *sd, int on)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	struct i2c_client *client = sp5508->client;
	int ret = 0;

	mutex_lock(&sp5508->mutex);

	/* If the power state is not modified - no work to do. */
	if (sp5508->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!sp5508->is_thunderboot) {
			ret = sp5508_write_array(sp5508->client, sp5508_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		sp5508->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sp5508->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sp5508->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sp5508_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SP5508_XVCLK_FREQ / 1000 / 1000);
}

static int __sp5508_power_on(struct sp5508 *sp5508)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sp5508->client->dev;

	if (!IS_ERR_OR_NULL(sp5508->pins_default)) {
		ret = pinctrl_select_state(sp5508->pinctrl,
					   sp5508->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sp5508->xvclk, SP5508_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sp5508->xvclk) != SP5508_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sp5508->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}


	if (!IS_ERR(sp5508->reset_gpio))
		gpiod_set_value_cansleep(sp5508->reset_gpio, 0);

	ret = regulator_bulk_enable(SP5508_NUM_SUPPLIES, sp5508->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sp5508->reset_gpio))
		gpiod_set_value_cansleep(sp5508->reset_gpio, 1);

	/* From spec: delay from power stable to pwdn off: 5ms */
	usleep_range(5000, 6000);
	if (!IS_ERR(sp5508->pwdn_gpio))
		gpiod_set_value_cansleep(sp5508->pwdn_gpio, 1);

	/* From spec: delay from pwdn off to reset off */
	usleep_range(4000, 5000);
	if (!IS_ERR(sp5508->reset_gpio))
		gpiod_direction_output(sp5508->reset_gpio, 1);

	/* From spec: 5ms for SCCB initialization */
	if (!IS_ERR(sp5508->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sp5508_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

disable_clk:
	clk_disable_unprepare(sp5508->xvclk);

	return ret;
}

static void __sp5508_power_off(struct sp5508 *sp5508)
{
	int ret;
	struct device *dev = &sp5508->client->dev;

	clk_disable_unprepare(sp5508->xvclk);
	if (sp5508->is_thunderboot) {
		if (sp5508->is_first_streamoff) {
			sp5508->is_thunderboot = false;
			sp5508->is_first_streamoff = false;
		} else {
			return;
		}
	}
	if (!IS_ERR(sp5508->pwdn_gpio))
		gpiod_set_value_cansleep(sp5508->pwdn_gpio, 0);
	if (!IS_ERR(sp5508->reset_gpio))
		gpiod_set_value_cansleep(sp5508->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(sp5508->pins_sleep)) {
		ret = pinctrl_select_state(sp5508->pinctrl,
					   sp5508->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SP5508_NUM_SUPPLIES, sp5508->supplies);
}

static int sp5508_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sp5508 *sp5508 = to_sp5508(sd);

	return __sp5508_power_on(sp5508);
}

static int sp5508_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sp5508 *sp5508 = to_sp5508(sd);

	__sp5508_power_off(sp5508);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sp5508_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sp5508 *sp5508 = to_sp5508(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sp5508_mode *def_mode = &supported_modes[0];

	mutex_lock(&sp5508->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sp5508->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sp5508_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sp5508 *sp5508 = to_sp5508(sd);

	if (fie->index >= sp5508->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops sp5508_pm_ops = {
	SET_RUNTIME_PM_OPS(sp5508_runtime_suspend,
	sp5508_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sp5508_internal_ops = {
	.open = sp5508_open,
};
#endif

static const struct v4l2_subdev_core_ops sp5508_core_ops = {
	.s_power = sp5508_s_power,
	.ioctl = sp5508_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sp5508_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sp5508_video_ops = {
	.s_stream = sp5508_s_stream,
	.g_frame_interval = sp5508_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sp5508_pad_ops = {
	.enum_mbus_code = sp5508_enum_mbus_code,
	.enum_frame_size = sp5508_enum_frame_sizes,
	.enum_frame_interval = sp5508_enum_frame_interval,
	.get_fmt = sp5508_get_fmt,
	.set_fmt = sp5508_set_fmt,
	.get_mbus_config = sp5508_g_mbus_config,
};

static const struct v4l2_subdev_ops sp5508_subdev_ops = {
	.core	= &sp5508_core_ops,
	.video	= &sp5508_video_ops,
	.pad	= &sp5508_pad_ops,
};

static int sp5508_set_gain_reg(struct i2c_client *client, u32 total_gain)
{
	u32 again = 0, dgain = 0;
	int ret = 0;

	if (total_gain < SP5508_GAIN_MIN) {
		total_gain = SP5508_GAIN_MIN;
		again = 0x10;
		dgain = 0x08;
	} else if (total_gain > SP5508_GAIN_MAX) {
		total_gain = SP5508_GAIN_MAX;
		again = 0x1F0;
		dgain = 0xff;
	}

	if (total_gain < 496) {
		again = total_gain;
		dgain = 0x08;
	} else if (total_gain < SP5508_GAIN_MIN) {
		again = 0xF0;
		dgain = total_gain * 16 / 496;
	}

	ret |= sp5508_write_reg(client,
				SP5508_REG_PAGE_SELECT, 0x01);
	ret |= sp5508_write_reg(client,
				SP5508_REG_AGAIN_L, (again & 0xff));
	ret |= sp5508_write_reg(client,
				SP5508_REG_AGAIN_H, ((again >> 8) & 0x01));   /*0x23 */
	ret |= sp5508_write_reg(client,
				SP5508_REG_DGAIN, dgain);
	ret |= sp5508_write_reg(client,
				SP5508_REG_RESTART, 0x01);

	dev_dbg(&client->dev, "set gain 0x%x, again:0x%x, dgain:0x%x ret 0x%x\n",
		total_gain,  again, dgain, ret);
	return ret;
}

static int sp5508_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sp5508 *sp5508 = container_of(ctrl->handler,
					     struct sp5508, ctrl_handler);
	struct i2c_client *client = sp5508->client;
	s64 max;
	int ret = 0;
	u8 val_h = 0, val_l = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sp5508->cur_mode->height + ctrl->val - 8;
		__v4l2_ctrl_modify_range(sp5508->exposure,
					 sp5508->exposure->minimum, max,
					 sp5508->exposure->step,
					 sp5508->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		// if shutter bigger than frame_length, should extend frame length first
		if (ctrl->val > sp5508->cur_mode->vts_def - 8)
			sp5508->cur_vts = ctrl->val + 8;
		else
			sp5508->cur_vts = sp5508->cur_mode->vts_def;

		if (sp5508->cur_vts > SP5508_VTS_MAX)
			sp5508->cur_vts = SP5508_VTS_MAX;
		else
			sp5508->cur_vts = (sp5508->cur_vts >> 1) << 1;

		ctrl->val = (ctrl->val < SP5508_EXPOSURE_MIN) ?  SP5508_EXPOSURE_MIN : ctrl->val;
		ctrl->val = (ctrl->val > (SP5508_VTS_MAX - 8)) ? (SP5508_VTS_MAX - 8) : ctrl->val;
		ctrl->val = (ctrl->val >> 1) << 1;

		sp5508_write_reg(sp5508->client,
				 SP5508_REG_VTS_H,
				 (sp5508->cur_vts >> 8) & 0xff);
		sp5508_write_reg(sp5508->client,
				 SP5508_REG_VTS_L,
				 sp5508->cur_vts & 0xff);

		dev_dbg(&client->dev, "set exposure  === %d === 0x%x\n", ctrl->val, ctrl->val);
		// update shutter
		ret = sp5508_write_reg(sp5508->client,
				       SP5508_REG_PAGE_SELECT, 0x1);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_EXP_H,
					SP5508_FETCH_EXP_H(ctrl->val));
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_EXP_L,
					SP5508_FETCH_EXP_L(ctrl->val));
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_RESTART, 0x01);

		ret |= sp5508_read_reg(sp5508->client, SP5508_REG_EXP_H, &val_h),
		ret |= sp5508_read_reg(sp5508->client, SP5508_REG_EXP_L, &val_l),
		dev_dbg(&client->dev, "read 0x03 = 0x%x, 0x04 = 0x%x\n",
			val_h, val_l);

		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sp5508_set_gain_reg(sp5508->client, ctrl->val);

		dev_dbg(&client->dev, "set gain 0x%x, ret 0x%x\n",
			ctrl->val, ret);
		break;
	case V4L2_CID_VBLANK:
		ret = sp5508_write_reg(sp5508->client,
				       SP5508_REG_PAGE_SELECT, 0x01);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_VBLANK_H, (ctrl->val >> 8) & 0xFF);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_VBLANK_L, ctrl->val & 0xFF);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_RESTART, 0x01);
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sp5508_enable_test_pattern(sp5508, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			sp5508->flip |= MIRROR_BIT_MASK;
		else
			sp5508->flip &= ~MIRROR_BIT_MASK;

		ret = sp5508_write_reg(sp5508->client,
				       SP5508_REG_PAGE_SELECT, 0x01);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_FLIP_REG, sp5508->flip);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_RESTART, 0x01);
		dev_dbg(&client->dev, "set hflip 0x%x\n", sp5508->flip);
		break;
	case V4L2_CID_VFLIP: //no effect according to sensor FAE.
		if (ctrl->val)
			sp5508->flip |= FLIP_BIT_MASK;
		else
			sp5508->flip &= ~FLIP_BIT_MASK;

		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_PAGE_SELECT, 0x01);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_FLIP_REG, sp5508->flip);
		ret |= sp5508_write_reg(sp5508->client,
					SP5508_REG_RESTART, 0x01);
		dev_dbg(&client->dev, "set vflip 0x%x\n", sp5508->flip);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}
	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sp5508_ctrl_ops = {
	.s_ctrl = sp5508_set_ctrl,
};

static int sp5508_initialize_controls(struct sp5508 *sp5508)
{
	const struct sp5508_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;

	handler = &sp5508->ctrl_handler;
	mode = sp5508->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sp5508->mutex;

	sp5508->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			    V4L2_CID_LINK_FREQ,
			    ARRAY_SIZE(link_freq_menu_items) - 1, 0,
			    link_freq_menu_items);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	sp5508->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE,
					       0, PIXEL_RATE_WITH_468M,
					       1, PIXEL_RATE_WITH_468M);

	__v4l2_ctrl_s_ctrl(sp5508->link_freq, dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	sp5508->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (sp5508->hblank)
		sp5508->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	sp5508->vblank = v4l2_ctrl_new_std(handler, &sp5508_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   SP5508_VTS_MAX - mode->height,
					   1, vblank_def);

	exposure_max = mode->vts_def - 8;
	sp5508->exposure = v4l2_ctrl_new_std(handler, &sp5508_ctrl_ops,
					     V4L2_CID_EXPOSURE, SP5508_EXPOSURE_MIN,
					     exposure_max, SP5508_EXPOSURE_STEP,
					     mode->exp_def);

	sp5508->anal_gain = v4l2_ctrl_new_std(handler, &sp5508_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, SP5508_GAIN_MIN,
					      SP5508_GAIN_MAX, SP5508_GAIN_STEP,
					      SP5508_GAIN_DEFAULT);
	sp5508->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
			       &sp5508_ctrl_ops,
			       V4L2_CID_TEST_PATTERN,
			       ARRAY_SIZE(sp5508_test_pattern_menu) - 1,
			       0, 0, sp5508_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &sp5508_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &sp5508_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	sp5508->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&sp5508->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sp5508->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sp5508_check_sensor_id(struct sp5508 *sp5508,
				  struct i2c_client *client)
{
	struct device *dev = &sp5508->client->dev;
	u8 id_h = 0, id_l = 0;
	u32 id = 0;
	int ret;

	ret = sp5508_write_reg(client, SP5508_REG_PAGE_SELECT, 0x00);
	ret |= sp5508_read_reg(client, SP5508_REG_CHIP_ID_H, &id_h);
	ret |= sp5508_read_reg(client, SP5508_REG_CHIP_ID_L, &id_l);

	id = SENSOR_ID(id_h, id_l);
	if (id != SP5508_CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SP-sp5508(%04x) sensor\n", SP5508_CHIP_ID);
	return 0;
}

static int sp5508_configure_regulators(struct sp5508 *sp5508)
{
	unsigned int i;

	for (i = 0; i < SP5508_NUM_SUPPLIES; i++)
		sp5508->supplies[i].supply = SP5508_supply_names[i];

	return devm_regulator_bulk_get(&sp5508->client->dev,
				       SP5508_NUM_SUPPLIES,
				       sp5508->supplies);
}

static int sp5508_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sp5508 *sp5508;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sp5508 = devm_kzalloc(dev, sizeof(*sp5508), GFP_KERNEL);
	if (!sp5508)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sp5508->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sp5508->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sp5508->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sp5508->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sp5508->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < sp5508->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			sp5508->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		sp5508->cur_mode = &supported_modes[0];

	sp5508->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	dev_info(dev, "is_thunderboot: %d\n", sp5508->is_thunderboot);
	sp5508->client = client;

	sp5508->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sp5508->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sp5508->reset_gpio = devm_gpiod_get(dev, "reset",
					    sp5508->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sp5508->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sp5508->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
					   sp5508->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(sp5508->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sp5508->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sp5508->pinctrl)) {
		sp5508->pins_default =
			pinctrl_lookup_state(sp5508->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sp5508->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sp5508->pins_sleep =
			pinctrl_lookup_state(sp5508->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sp5508->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sp5508_configure_regulators(sp5508);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sp5508->mutex);

	sd = &sp5508->subdev;
	v4l2_i2c_subdev_init(sd, client, &sp5508_subdev_ops);
	ret = sp5508_initialize_controls(sp5508);
	if (ret)
		goto err_destroy_mutex;

	ret = __sp5508_power_on(sp5508);
	if (ret)
		goto err_free_handler;

	ret = sp5508_check_sensor_id(sp5508, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sp5508_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sp5508->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sp5508->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sp5508->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sp5508->module_index, facing,
		 SP5508_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (sp5508->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sp5508_power_off(sp5508);
err_free_handler:
	v4l2_ctrl_handler_free(&sp5508->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sp5508->mutex);

	return ret;
}

static int sp5508_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sp5508 *sp5508 = to_sp5508(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sp5508->ctrl_handler);
	mutex_destroy(&sp5508->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sp5508_power_off(sp5508);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sp5508_of_match[] = {
	{ .compatible = "superpix,sp5508" },
	{},
};
MODULE_DEVICE_TABLE(of, sp5508_of_match);
#endif

static const struct i2c_device_id sp5508_match_id[] = {
	{ "superpix,sp5508", 0 },
	{ },
};

static struct i2c_driver sp5508_i2c_driver = {
	.driver = {
		.name = SP5508_NAME,
		.pm = &sp5508_pm_ops,
		.of_match_table = of_match_ptr(sp5508_of_match),
	},
	.probe		= &sp5508_probe,
	.remove		= &sp5508_remove,
	.id_table	= sp5508_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sp5508_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sp5508_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Superpix sp5508 sensor driver");
MODULE_LICENSE("GPL");
