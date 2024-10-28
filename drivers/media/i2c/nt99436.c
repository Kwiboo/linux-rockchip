// SPDX-License-Identifier: GPL-2.0
/*
 * nt99436 driver
 *
 * Copyright (C) 2024 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
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

#define DRIVER_VERSION				KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN			V4L2_CID_GAIN
#endif

#define NT99436_LANES				2
#define NT99436_BITS_PER_SAMPLE			10
#define NT99436_LINK_FREQ_360			360000000

#define PIXEL_RATE_WITH_384M_10BIT		(NT99436_LINK_FREQ_360  * 2 * NT99436_LANES \
						 /NT99436_BITS_PER_SAMPLE)
#define NT99436_XVCLK_FREQ			24000000

#define CHIP_ID					0x9436
#define NT99436_REG_CHIP_ID			0x0000

#define NT99436_REG_CTRL_MODE			0x0100
#define NT99436_MODE_SW_STANDBY			0x00
#define NT99436_MODE_STREAMING			0x01

#define NT99436_REG_EXPOSURE_L			0x0203 //[7:0]
#define NT99436_REG_EXPOSURE_H			0x0202 //[15:8]
#define	NT99436_EXPOSURE_MIN			1
#define	NT99436_EXPOSURE_MAX			0x063e
#define	NT99436_EXPOSURE_STEP			1
#define NT99436_VTS_MAX				0x7fff

#define NT99436_REG_GAIN_IDX			0x0207
#define NT99436_GAIN_MIN			0
#define NT99436_GAIN_MAX			63
#define NT99436_GAIN_STEP			1
#define NT99436_GAIN_DEFAULT			0

#define NT99436_REG_GROUP_HOLD			0x0104
#define NT99436_GROUP_HOLD_START		0x01
#define NT99436_GROUP_HOLD_END			0x00

#define NT99436_REG_TEST_PATTERN		0x040a
#define NT99436_TEST_PATTERN_BIT_MASK		BIT(3)

#define NT99436_REG_VTS_H			0x0340
#define NT99436_REG_VTS_L			0x0341

#define NT99436_FLIP_MIRROR_REG			0x0101
#define NT99436_PIXEL_ORDER			0x0006

#define NT99436_FETCH_EXP_H(VAL)		(((VAL) >> 8) & 0xFF)
#define NT99436_FETCH_EXP_L(VAL)		((VAL) & 0xFF)

#define NT99436_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x01 : VAL & 0xfe)
#define NT99436_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x02 : VAL & 0xfd)

#define REG_DELAY				0xFFFE
#define REG_NULL				0xFFFF

#define NT99436_REG_VALUE_08BIT			1
#define NT99436_REG_VALUE_16BIT			2
#define NT99436_REG_VALUE_24BIT			3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT		"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP		"rockchip,camera_sleep"
#define NT99436_NAME				"nt99436"

static const char * const nt99436_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define NT99436_NUM_SUPPLIES ARRAY_SIZE(nt99436_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct nt99436_mode {
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

struct nt99436 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct regulator_bulk_data supplies[NT99436_NUM_SUPPLIES];

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
	const struct nt99436_mode *cur_mode;
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

#define to_nt99436(sd) container_of(sd, struct nt99436, subdev)

/*
 * Xclk 27Mhz
 */
static const struct regval nt99436_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 768Mbps, 2lane
 */
static const struct regval nt99436_linear_10_2560x1440_30fps_regs[] = {
	{0x3644, 0x1F},
	{0x3398, 0x60},
	{0x327C, 0x37},
	{0x3252, 0x21},
	{0x3253, 0x8B},
	{0x325E, 0x14},
	{0x32E1, 0x03},
	{0x3326, 0x30},
	{0x3328, 0x8C},
	{0x335D, 0x8C},
	{0x3399, 0x0A},
	{0x360A, 0x5F},
	{0x3620, 0x18},
	{0x3621, 0x04},
	{0x3332, 0x1C},
	{0x3325, 0x1C},
	{0x3342, 0x1C},
	{0x3343, 0x18},
	{0x335A, 0x1C},
	{0x335B, 0x26},
	{0x336A, 0x1C},
	{0x336B, 0x18},
	{0x32CF, 0x30},
	{0x804C, 0x01},
	{0x804D, 0x00},
	{0x3A0A, 0x07},
	{0x803F, 0x27},
	{0x0B04, 0x01},
	{0x0138, 0x01},
	{0x833F, 0x00},
	{0x0B06, 0x01},
	{0x0350, 0x01},
	{0x020C, 0x03},
	{0x324C, 0x03},
	{0x324D, 0x01},
	{0x8047, 0x3E},
	{0x8049, 0x3F},
	{0x0100, 0x00},
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0114, 0x01},
	{0x0305, 0x06},
	{0x0307, 0x96},
	{0x0309, 0x68},
	{0x030A, 0x06},
	{0x030B, 0x01},
	{0x030C, 0x01},
	{0x030F, 0xB4},
	{0x3131, 0x06},
	{0x3132, 0x96},
	{0x302D, 0x00},
	{0x3028, 0x02},
	{0x3029, 0x02},
	{0x0342, 0x06},
	{0x0343, 0x1A},
	{0x0340, 0x06},
	{0x0341, 0x40},
	{0x0344, 0x00},
	{0x0345, 0x40},
	{0x0346, 0x00},
	{0x0347, 0x30},
	{0x0348, 0x0A},
	{0x0349, 0x3F},
	{0x034A, 0x05},
	{0x034B, 0xCF},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0A},
	{0x040D, 0x00},
	{0x040E, 0x05},
	{0x040F, 0xA0},
	{0x034C, 0x0A},
	{0x034D, 0x00},
	{0x034E, 0x05},
	{0x034F, 0xA0},
	{0x0389, 0x00},
	{0x038A, 0x07},
	{0x0280, 0x02},
	{REG_NULL, 0x00},
};

static const struct nt99436_mode supported_modes[] = {
	{
		.width = 2560,
		.height = 1440,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0040,
		.hts_def = 0x061a * 2,
		.vts_def = 0x0640,
		.bus_fmt = MEDIA_BUS_FMT_SGRBG10_1X10,
		.reg_list = nt99436_linear_10_2560x1440_30fps_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	NT99436_LINK_FREQ_360
};

static const char * const nt99436_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int nt99436_write_reg(struct i2c_client *client, u16 reg,
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

static int nt99436_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = nt99436_write_reg(client, regs[i].addr,
					NT99436_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int nt99436_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static int nt99436_get_reso_dist(const struct nt99436_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct nt99436_mode *
nt99436_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = nt99436_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int nt99436_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	const struct nt99436_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&nt99436->mutex);

	mode = nt99436_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&nt99436->mutex);
		return -ENOTTY;
#endif
	} else {
		nt99436->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(nt99436->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(nt99436->vblank, vblank_def,
					 NT99436_VTS_MAX - mode->height,
					 1, vblank_def);
		nt99436->cur_fps = mode->max_fps;
	}

	mutex_unlock(&nt99436->mutex);

	return 0;
}

static int nt99436_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	const struct nt99436_mode *mode = nt99436->cur_mode;

	mutex_lock(&nt99436->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&nt99436->mutex);
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
	mutex_unlock(&nt99436->mutex);

	return 0;
}

static int nt99436_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct nt99436 *nt99436 = to_nt99436(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = nt99436->cur_mode->bus_fmt;

	return 0;
}

static int nt99436_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int nt99436_enable_test_pattern(struct nt99436 *nt99436, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = nt99436_read_reg(nt99436->client, NT99436_REG_TEST_PATTERN,
			       NT99436_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= NT99436_TEST_PATTERN_BIT_MASK;
	else
		val &= ~NT99436_TEST_PATTERN_BIT_MASK;

	ret |= nt99436_write_reg(nt99436->client, NT99436_REG_TEST_PATTERN,
				 NT99436_REG_VALUE_08BIT, val);
	return ret;
}

static int nt99436_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	const struct nt99436_mode *mode = nt99436->cur_mode;

	if (nt99436->streaming)
		fi->interval = nt99436->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int nt99436_g_mbus_config(struct v4l2_subdev *sd,
				unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	const struct nt99436_mode *mode = nt99436->cur_mode;

	u32 val = 1 << (NT99436_LANES - 1) |
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

static void nt99436_get_module_inf(struct nt99436 *nt99436,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, NT99436_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, nt99436->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, nt99436->len_name, sizeof(inf->base.lens));
}

static long nt99436_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	long ret = 0;
	u32 stream = 0;
	struct rkmodule_hdr_cfg *hdr;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		nt99436_get_module_inf(nt99436, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = nt99436->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream)
			ret = nt99436_write_reg(nt99436->client, NT99436_REG_CTRL_MODE,
				 NT99436_REG_VALUE_08BIT, NT99436_MODE_STREAMING);
		else
			ret = nt99436_write_reg(nt99436->client, NT99436_REG_CTRL_MODE,
				 NT99436_REG_VALUE_08BIT, NT99436_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long nt99436_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = nt99436_ioctl(sd, cmd, inf);
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

		ret = nt99436_ioctl(sd, cmd, hdr);
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
			ret = nt99436_ioctl(sd, cmd, hdr);
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
			ret = nt99436_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = nt99436_ioctl(sd, cmd, &stream);
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

static int __nt99436_start_stream(struct nt99436 *nt99436)
{
	int ret;

	if (!nt99436->is_thunderboot) {
		ret = nt99436_write_array(nt99436->client, nt99436->cur_mode->reg_list);
		if (ret)
			return ret;
		/* In case these controls are set before streaming */
		ret = __v4l2_ctrl_handler_setup(&nt99436->ctrl_handler);
		if (ret)
			return ret;
		if (nt99436->has_init_exp && nt99436->cur_mode->hdr_mode != NO_HDR) {
			ret = nt99436_ioctl(&nt99436->subdev, PREISP_CMD_SET_HDRAE_EXP,
				&nt99436->init_hdrae_exp);
			if (ret) {
				dev_err(&nt99436->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}

	ret = nt99436_write_reg(nt99436->client, NT99436_REG_CTRL_MODE,
				NT99436_REG_VALUE_08BIT, NT99436_MODE_STREAMING);
	return ret;
}

static int __nt99436_stop_stream(struct nt99436 *nt99436)
{
	nt99436->has_init_exp = false;
	if (nt99436->is_thunderboot) {
		nt99436->is_first_streamoff = true;
		pm_runtime_put(&nt99436->client->dev);
	}
	return nt99436_write_reg(nt99436->client, NT99436_REG_CTRL_MODE,
				 NT99436_REG_VALUE_08BIT, NT99436_MODE_SW_STANDBY);
}

static int __nt99436_power_on(struct nt99436 *nt99436);
static int nt99436_s_stream(struct v4l2_subdev *sd, int on)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	struct i2c_client *client = nt99436->client;
	int ret = 0;

	mutex_lock(&nt99436->mutex);
	on = !!on;
	if (on == nt99436->streaming)
		goto unlock_and_return;
	if (on) {
		if (nt99436->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			nt99436->is_thunderboot = false;
			__nt99436_power_on(nt99436);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __nt99436_start_stream(nt99436);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__nt99436_stop_stream(nt99436);
		pm_runtime_put(&client->dev);
	}

	nt99436->streaming = on;
unlock_and_return:
	mutex_unlock(&nt99436->mutex);
	return ret;
}

static int nt99436_s_power(struct v4l2_subdev *sd, int on)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	struct i2c_client *client = nt99436->client;
	int ret = 0;

	mutex_lock(&nt99436->mutex);

	/* If the power state is not modified - no work to do. */
	if (nt99436->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!nt99436->is_thunderboot) {
			ret = nt99436_write_array(nt99436->client, nt99436_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		nt99436->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		nt99436->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&nt99436->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 nt99436_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, NT99436_XVCLK_FREQ / 1000 / 1000);
}

static int __nt99436_power_on(struct nt99436 *nt99436)
{
	int ret;
	struct device *dev = &nt99436->client->dev;

	if (!IS_ERR_OR_NULL(nt99436->pins_default)) {
		ret = pinctrl_select_state(nt99436->pinctrl,
					   nt99436->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(nt99436->xvclk, NT99436_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(nt99436->xvclk) != NT99436_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(nt99436->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (nt99436->is_thunderboot)
		return 0;

	if (!IS_ERR(nt99436->reset_gpio))
		gpiod_set_value_cansleep(nt99436->reset_gpio, 0);

	ret = regulator_bulk_enable(NT99436_NUM_SUPPLIES, nt99436->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(1000, 2000);

	if (!IS_ERR(nt99436->reset_gpio))
		gpiod_set_value_cansleep(nt99436->reset_gpio, 1);

	usleep_range(12000, 13000);

	return 0;

disable_clk:
	clk_disable_unprepare(nt99436->xvclk);

	return ret;
}

static void __nt99436_power_off(struct nt99436 *nt99436)
{
	int ret;
	struct device *dev = &nt99436->client->dev;

	clk_disable_unprepare(nt99436->xvclk);
	if (nt99436->is_thunderboot) {
		if (nt99436->is_first_streamoff) {
			nt99436->is_thunderboot = false;
			nt99436->is_first_streamoff = false;
		} else {
			return;
		}
	}

	clk_disable_unprepare(nt99436->xvclk);
	if (!IS_ERR(nt99436->reset_gpio))
		gpiod_set_value_cansleep(nt99436->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(nt99436->pins_sleep)) {
		ret = pinctrl_select_state(nt99436->pinctrl,
					   nt99436->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(NT99436_NUM_SUPPLIES, nt99436->supplies);
}

static int nt99436_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nt99436 *nt99436 = to_nt99436(sd);

	return __nt99436_power_on(nt99436);
}

static int nt99436_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nt99436 *nt99436 = to_nt99436(sd);

	__nt99436_power_off(nt99436);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int nt99436_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct nt99436 *nt99436 = to_nt99436(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct nt99436_mode *def_mode = &supported_modes[0];

	mutex_lock(&nt99436->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&nt99436->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int nt99436_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops nt99436_pm_ops = {
	SET_RUNTIME_PM_OPS(nt99436_runtime_suspend, nt99436_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops nt99436_internal_ops = {
	.open = nt99436_open,
};
#endif

static const struct v4l2_subdev_core_ops nt99436_core_ops = {
	.s_power = nt99436_s_power,
	.ioctl = nt99436_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = nt99436_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops nt99436_video_ops = {
	.s_stream = nt99436_s_stream,
	.g_frame_interval = nt99436_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops nt99436_pad_ops = {
	.enum_mbus_code = nt99436_enum_mbus_code,
	.enum_frame_size = nt99436_enum_frame_sizes,
	.enum_frame_interval = nt99436_enum_frame_interval,
	.get_fmt = nt99436_get_fmt,
	.set_fmt = nt99436_set_fmt,
	.get_mbus_config = nt99436_g_mbus_config,
};

static const struct v4l2_subdev_ops nt99436_subdev_ops = {
	.core	= &nt99436_core_ops,
	.video	= &nt99436_video_ops,
	.pad	= &nt99436_pad_ops,
};

static void nt99436_modify_fps_info(struct nt99436 *nt99436)
{
	const struct nt99436_mode *mode = nt99436->cur_mode;

	nt99436->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				      nt99436->cur_vts;
}

static int nt99436_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct nt99436 *nt99436 = container_of(ctrl->handler,
					       struct nt99436, ctrl_handler);
	struct i2c_client *client = nt99436->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = nt99436->cur_mode->height + ctrl->val - 5;
		__v4l2_ctrl_modify_range(nt99436->exposure,
					 nt99436->exposure->minimum, max,
					 nt99436->exposure->step,
					 nt99436->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (nt99436->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val;
			/* 4 least significant bits of expsoure are fractional part */
			ret = nt99436_write_reg(nt99436->client,
						NT99436_REG_EXPOSURE_H,
						NT99436_REG_VALUE_08BIT,
						NT99436_FETCH_EXP_H(val));
			ret |= nt99436_write_reg(nt99436->client,
						 NT99436_REG_EXPOSURE_L,
						 NT99436_REG_VALUE_08BIT,
						 NT99436_FETCH_EXP_L(val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		ret = nt99436_write_reg(nt99436->client,
					NT99436_REG_GAIN_IDX,
					NT99436_REG_VALUE_08BIT,
					ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = nt99436_write_reg(nt99436->client,
					NT99436_REG_VTS_H,
					NT99436_REG_VALUE_08BIT,
					(ctrl->val + nt99436->cur_mode->height)
					>> 8);
		ret |= nt99436_write_reg(nt99436->client,
					 NT99436_REG_VTS_L,
					 NT99436_REG_VALUE_08BIT,
					 (ctrl->val + nt99436->cur_mode->height)
					 & 0xff);
		nt99436->cur_vts = ctrl->val + nt99436->cur_mode->height;
		nt99436_modify_fps_info(nt99436);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = nt99436_enable_test_pattern(nt99436, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = nt99436_write_reg(nt99436->client, NT99436_REG_GROUP_HOLD,
					NT99436_REG_VALUE_08BIT, NT99436_GROUP_HOLD_START);
		ret |= nt99436_read_reg(nt99436->client, NT99436_FLIP_MIRROR_REG,
					NT99436_REG_VALUE_08BIT, &val);
		ret |= nt99436_write_reg(nt99436->client, NT99436_FLIP_MIRROR_REG,
					 NT99436_REG_VALUE_08BIT,
					 NT99436_FETCH_MIRROR(val, ctrl->val));
		ret |= nt99436_write_reg(nt99436->client, NT99436_REG_GROUP_HOLD,
					 NT99436_REG_VALUE_08BIT, NT99436_GROUP_HOLD_END);
		break;
	case V4L2_CID_VFLIP:
		ret = nt99436_write_reg(nt99436->client, NT99436_REG_GROUP_HOLD,
					NT99436_REG_VALUE_08BIT, NT99436_GROUP_HOLD_START);
		ret |= nt99436_read_reg(nt99436->client, NT99436_FLIP_MIRROR_REG,
					NT99436_REG_VALUE_08BIT, &val);
		ret |= nt99436_write_reg(nt99436->client, NT99436_FLIP_MIRROR_REG,
					 NT99436_REG_VALUE_08BIT,
					 NT99436_FETCH_FLIP(val, ctrl->val));
		ret |= nt99436_write_reg(nt99436->client, NT99436_REG_GROUP_HOLD,
					 NT99436_REG_VALUE_08BIT, NT99436_GROUP_HOLD_END);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops nt99436_ctrl_ops = {
	.s_ctrl = nt99436_set_ctrl,
};

static int nt99436_initialize_controls(struct nt99436 *nt99436)
{
	const struct nt99436_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &nt99436->ctrl_handler;
	mode = nt99436->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &nt99436->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_384M_10BIT, 1, PIXEL_RATE_WITH_384M_10BIT);

	h_blank = mode->hts_def - mode->width;
	nt99436->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (nt99436->hblank)
		nt99436->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	nt99436->vblank = v4l2_ctrl_new_std(handler, &nt99436_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    NT99436_VTS_MAX - mode->height,
					    1, vblank_def);
	exposure_max = mode->vts_def - 5;
	nt99436->exposure = v4l2_ctrl_new_std(handler, &nt99436_ctrl_ops,
					      V4L2_CID_EXPOSURE, NT99436_EXPOSURE_MIN,
					      exposure_max, NT99436_EXPOSURE_STEP,
					      mode->exp_def);
	nt99436->anal_gain = v4l2_ctrl_new_std(handler, &nt99436_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, NT99436_GAIN_MIN,
					       NT99436_GAIN_MAX, NT99436_GAIN_STEP,
					       NT99436_GAIN_DEFAULT);
	nt99436->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &nt99436_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(nt99436_test_pattern_menu) - 1,
					0, 0, nt99436_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &nt99436_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &nt99436_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&nt99436->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	nt99436->subdev.ctrl_handler = handler;
	nt99436->has_init_exp = false;
	nt99436->cur_fps = mode->max_fps;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int nt99436_check_sensor_id(struct nt99436 *nt99436,
				   struct i2c_client *client)
{
	struct device *dev = &nt99436->client->dev;
	u32 id = 0;
	int ret;

	if (nt99436->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = nt99436_read_reg(client, NT99436_REG_CHIP_ID,
			       NT99436_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int nt99436_configure_regulators(struct nt99436 *nt99436)
{
	unsigned int i;

	for (i = 0; i < NT99436_NUM_SUPPLIES; i++)
		nt99436->supplies[i].supply = nt99436_supply_names[i];

	return devm_regulator_bulk_get(&nt99436->client->dev,
				       NT99436_NUM_SUPPLIES,
				       nt99436->supplies);
}

static int nt99436_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct nt99436 *nt99436;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	int i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	nt99436 = devm_kzalloc(dev, sizeof(*nt99436), GFP_KERNEL);
	if (!nt99436)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &nt99436->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &nt99436->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &nt99436->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &nt99436->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	nt99436->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	nt99436->client = client;
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			nt99436->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		nt99436->cur_mode = &supported_modes[0];

	nt99436->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(nt99436->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	nt99436->reset_gpio = devm_gpiod_get(dev, "reset",
					    nt99436->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(nt99436->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	nt99436->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(nt99436->pinctrl)) {
		nt99436->pins_default =
			pinctrl_lookup_state(nt99436->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(nt99436->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		nt99436->pins_sleep =
			pinctrl_lookup_state(nt99436->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(nt99436->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = nt99436_configure_regulators(nt99436);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&nt99436->mutex);

	sd = &nt99436->subdev;
	v4l2_i2c_subdev_init(sd, client, &nt99436_subdev_ops);
	ret = nt99436_initialize_controls(nt99436);
	if (ret)
		goto err_destroy_mutex;

	ret = __nt99436_power_on(nt99436);
	if (ret)
		goto err_free_handler;

	ret = nt99436_check_sensor_id(nt99436, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &nt99436_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	nt99436->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &nt99436->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(nt99436->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 nt99436->module_index, facing,
		 NT99436_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (nt99436->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__nt99436_power_off(nt99436);
err_free_handler:
	v4l2_ctrl_handler_free(&nt99436->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&nt99436->mutex);

	return ret;
}

static int nt99436_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nt99436 *nt99436 = to_nt99436(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&nt99436->ctrl_handler);
	mutex_destroy(&nt99436->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__nt99436_power_off(nt99436);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id nt99436_of_match[] = {
	{ .compatible = "novatek,nt99436" },
	{},
};
MODULE_DEVICE_TABLE(of, nt99436_of_match);
#endif

static const struct i2c_device_id nt99436_match_id[] = {
	{ "novatek,nt99436", 0 },
	{ },
};

static struct i2c_driver nt99436_i2c_driver = {
	.driver = {
		.name = NT99436_NAME,
		.pm = &nt99436_pm_ops,
		.of_match_table = of_match_ptr(nt99436_of_match),
	},
	.probe		= &nt99436_probe,
	.remove		= &nt99436_remove,
	.id_table	= nt99436_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&nt99436_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&nt99436_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("novatek nt99436 sensor driver");
MODULE_LICENSE("GPL");
