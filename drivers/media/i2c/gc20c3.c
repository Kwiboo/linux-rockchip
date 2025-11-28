// SPDX-License-Identifier: GPL-2.0
/*
 * gc20c3 sensor driver
 *
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version.
 */

 //#define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define GC20C3_LANES			2
#define GC20C3_BITS_PER_SAMPLE		10
#define GC20C3_LINK_FREQ_270M		270000000

#define GC20C3_PIXEL_RATE_LINEAR	(GC20C3_LINK_FREQ_270M * 2 * \
					GC20C3_LANES / GC20C3_BITS_PER_SAMPLE)

#define GC20C3_XVCLK_FREQ		24000000

#define GC20C3_CHIP_ID			0x20C3
#define GC20C3_REG_CHIP_ID_H		0x03f0
#define GC20C3_REG_CHIP_ID_L		0x03f1

#define GC20C3_REG_CTRL_MODE		0x0100
#define GC20C3_MODE_SW_STANDBY		0x00
#define GC20C3_MODE_STREAMING		0x03

#define GC20C3_REG_EXP_H		0x0202
#define GC20C3_REG_EXP_L		0x0203
#define GC20C3_EXPOSURE_MIN		4
#define GC20C3_EXPOSURE_STEP		1

#define GC20C3_VTS_MAX			0x3FFF

#define GC20C3_GAIN_MIN			0x40
#define GC20C3_GAIN_MAX			0x1000
#define GC20C3_GAIN_STEP		1
#define GC20C3_GAIN_DEFAULT		64

#define GC20C3_REG_VTS_H		0x0340
#define GC20C3_REG_VTS_L		0x0341

#define GC20C3_REG_HTS_H		0x0342
#define GC20C3_REG_HTS_L		0x0343

#define GC20C3_FLIP_MIRROR_REG		0x022c
#define GC20C3_MIRROR_BIT_MASK		BIT(0)
#define GC20C3_FLIP_BIT_MASK		BIT(1)

#define REG_DELAY			0x0000
#define REG_NULL			0xFFFF

#define GC20C3_REG_VALUE_08BIT		1
#define GC20C3_REG_VALUE_16BIT		2
#define GC20C3_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP   "rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE              "rockchip,camera-hdr-mode"
#define GC20C3_NAME                     "gc20c3"

#define SENSOR_ID(_msb, _lsb)           ((_msb) << 8 | (_lsb))

static const char * const gc20c3_supply_names[] = {
	"dovdd",    /* Digital I/O power */
	"avdd",     /* Analog power */
	"dvdd",     /* Digital core power */
};

#define GC20C3_NUM_SUPPLIES ARRAY_SIZE(gc20c3_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct gc20c3_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 link_freq_idx;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct gc20c3 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct gpio_desc	*power_gpio;
	struct regulator_bulk_data supplies[GC20C3_NUM_SUPPLIES];

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
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_fract	cur_fps;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct gc20c3_mode *cur_mode;
	unsigned int		cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	u8			flip;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	bool			is_standby;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_gc20c3(sd) container_of(sd, struct gc20c3, subdev)


/*
 * window_size=1920*1080 mipi@2lane
 * mclk=24mhz,mipi_clk=540Mbps
 * pixel_line_total=2000,line_frame_total=1125
 * row_time=29.137us,frame_rate=30fps
 * release_v1.1.0_00_2lane_raw10_1920x1080_24mhz_30fps_GC20C3.txt
 */
static const struct regval gc20c3_1920x1080_regs_2lane[] = {
	{0x03fe, 0xff},
	{0x03fe, 0x00},
	{0x03fe, 0x10},
	{0x0190, 0x03},
	{0x0b4d, 0x02},
	{0x0d40, 0x01},
	{0x0087, 0x50},
	{0x0209, 0x00},
	{0x03b5, 0x11},
	{0x031c, 0x18},
	{0x03b2, 0x03},
	{0x03bb, 0xff},
	{0x03be, 0xff},
	{0x0d10, 0x06},
	{0x0d11, 0x0b},
	{0x0d10, 0x07},
	{0x0d1a, 0x02},
	{0x0d15, 0x03},
	{0x0d12, 0x05},
	{0x0d16, 0x00},
	{0x0d17, 0x87},
	{0x0d18, 0x01},
	{0x0d19, 0x48},
	{0x0145, 0x0d},
	{0x0144, 0x02},
	{0x0142, 0x04},
	{0x0143, 0x14},
	{0x0146, 0x05},
	{0x0141, 0x05},
	{0x0149, 0x05},
	{0x014a, 0x07},
	{0x014b, 0x06},
	{0x0b4e, 0x88},
	{0x0e0c, 0x00},
	{0x0e0f, 0x00},
	{0x0e3a, 0x98},
	{0x0b45, 0x06},
	{0x0b47, 0xf0},
	{0x0d30, 0x06},
	{0x0d2f, 0x05},
	{0x0b40, 0x57},
	{0x0b43, 0x00},
	{0x0b41, 0x0c},
	{0x0d31, 0x03},
	{0x0c23, 0x40},
	{0x0e23, 0x1a},
	{0x0e2a, 0x09},
	{0x0e2b, 0xc9},
	{0x0e41, 0x87},
	{0x0e3b, 0xb5},
	{0x0e3a, 0x15},
	{0x0e37, 0x1f},
	{0x0217, 0x02},
	{0x0213, 0x04},
	{0x0219, 0xc2},
	{0x0259, 0x04},
	{0x025a, 0x5e},
	{0x0211, 0x01},
	{0x0340, 0x04},
	{0x0341, 0x65},
	{0x0342, 0x03},
	{0x0343, 0xe8},
	{0x0212, 0x14},
	{0x0350, 0x06},
	{0x0348, 0x07},
	{0x0349, 0x88},
	{0x034a, 0x04},
	{0x034b, 0x40},
	{0x0347, 0x00},
	{0x0b0c, 0x00},
	{0x0b0d, 0x02},
	{0x0b0e, 0x07},
	{0x0b0f, 0x8a},
	{0x034e, 0x07},
	{0x034f, 0xa8},
	{0x0004, 0x0f},
	{0x0444, 0x00},
	{0x0038, 0x20},
	{0x0039, 0x20},
	{0x003a, 0x20},
	{0x003b, 0x20},
	{0x0492, 0x00},
	{0x0493, 0x00},
	{0x0070, 0x00},
	{0x0094, 0x07},
	{0x0095, 0x80},
	{0x0096, 0x04},
	{0x0097, 0x38},
	{0x0099, 0x04},
	{0x009b, 0x04},
	{0x0438, 0x0f},
	{0x0439, 0xf0},
	{0x021a, 0x10},
	{0x0476, 0x01},
	{0x0430, 0x23},
	{0x0443, 0x02},
	{0x0038, 0x00},
	{0x0039, 0x00},
	{0x003a, 0x00},
	{0x003b, 0x00},
	{0x0070, 0x80},
	{0x0448, 0x0d},
	{0x0449, 0x0d},
	{0x044a, 0x0d},
	{0x044b, 0x0d},
	{0x044c, 0x74},
	{0x044d, 0x74},
	{0x044e, 0x74},
	{0x044f, 0x74},
	{0x0485, 0x68},
	{0x0d38, 0x07},
	{0x0d39, 0x57},
	{0x0e4e, 0xa9},
	{0x0072, 0x09},
	{0x0073, 0x05},
	{0x0c20, 0x01},
	{0x0c1d, 0x02},
	{0x0c1e, 0x2c},
	{0x0c1f, 0xe6},
	{0x0c19, 0x00},
	{0x0c1a, 0x11},
	{0x0c1b, 0x00},
	{0x0c1c, 0x80},
	{0x0261, 0x13},
	{0x0004, 0x0f},
	{0x0052, 0x00},
	{0x0053, 0x20},
	{0x0055, 0x20},
	{0x0152, 0x14},
	{0x0100, 0x03},
	{0x031c, 0x1f},
	{0x0336, 0x01},
	{0x0336, 0x00},
	{0x03fe, 0x00},
	{REG_NULL, 0x00},
};

static const struct gc20c3_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SGRBG10_1X10,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x460,
		.hts_def = 0x3e8 * 2,
		.vts_def = 0x465,
		.reg_list = gc20c3_1920x1080_regs_2lane,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
};

static const s64 link_freq_menu_items[] = {
	GC20C3_LINK_FREQ_270M
};

/* Write registers up to 4 at a time */
static int gc20c3_write_reg(struct i2c_client *client, u16 reg,
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

static int gc20c3_write_array(struct i2c_client *client,
				  const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
			usleep_range(regs[i].val * 1000, regs[i].val * 1000);
			continue;
		}

		ret = gc20c3_write_reg(client, regs[i].addr,
				       GC20C3_REG_VALUE_08BIT, regs[i].val);
	}

	return ret;
}

/* sensor register read */
static int gc20c3_read_reg(struct i2c_client *client, u16 reg,
			   unsigned int len, u32 *val)
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

static int gc20c3_get_reso_dist(const struct gc20c3_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct gc20c3_mode *
gc20c3_find_best_fit(struct gc20c3 *gc20c3, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < gc20c3->cfg_num; i++) {
		dist = gc20c3_get_reso_dist(&supported_modes[i], framefmt);
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

static const uint8_t gain_reg_val_table[13][7] = {
	//0d04  0d05  0e36  0e39   04a8   04a9   0052        |  实际倍数   | Again dB|
	{ 0x00, 0x01, 0x15, 0x15,  0x01,  0x00,  0x64},    //|  X1        | 0.00    |
	{ 0x00, 0x02, 0x15, 0x15,  0x01,  0x1b,  0x64},    //|  X1.43     | 3.09    |
	{ 0x00, 0x03, 0x16, 0x16,  0x02,  0x00,  0x64},    //|  X2.01     | 6.05    |
	{ 0x00, 0x04, 0x17, 0x17,  0x02,  0x37,  0x64},    //|  X2.87     | 9.17    |
	{ 0x00, 0x05, 0x17, 0x17,  0x04,  0x02,  0x84},    //|  X4.03     | 12.11   |
	{ 0x00, 0x06, 0x18, 0x18,  0x05,  0x32,  0x84},    //|  X5.79     | 15.25   |
	{ 0x00, 0x07, 0x19, 0x19,  0x08,  0x05,  0x84},    //|  X8.09     | 18.16   |
	{ 0x04, 0x97, 0x1a, 0x1a,  0x0b,  0x10,  0x84},    //|  X11.25    | 21.03   |
	{ 0x08, 0x07, 0x1b, 0x1b,  0x10,  0x04,  0x84},    //|  X16.07    | 24.12   |
	{ 0x0a, 0x4f, 0x1c, 0x1c,  0x16,  0x22,  0x88},    //|  X22.54    | 27.06   |
	{ 0x0c, 0x07, 0x1d, 0x1d,  0x20,  0x06,  0x88},    //|  X32.10    | 30.13   |
	{ 0x0d, 0x2f, 0x1e, 0x1e,  0x2d,  0x10,  0x88},    //|  X45.26    | 33.11   |
	{ 0x0e, 0x07, 0x20, 0x20,  0x3f,  0x23,  0x72},    //|  X63.56    | 36.06   |
};

static const uint32_t gain_level_table[14] = {
	64,
	91,
	128,
	183,
	257,
	370,
	517,
	720,
	1028,
	1442,
	2054,
	2896,
	4067,
	0xffffffff
};

static int gc20c3_set_gain(struct gc20c3 *gc20c3, u32 gain)
{
	int ret;
	uint8_t i = 0;
	uint8_t total = 0;
	u32 tol_dig_gain = 0; // digital gain

	total = sizeof(gain_level_table) / sizeof(u32) - 1;
	for (i = 0; i < total; i++) {
		if ((gain_level_table[i] <= gain) && (gain < gain_level_table[i+1]))
			break;
	}

	if (i > total)
		i = total;
	tol_dig_gain = gain * 1024 / gain_level_table[i];

	// again
	if (i >= 0 && i < ARRAY_SIZE(gain_reg_val_table)) {
		ret = gc20c3_write_reg(gc20c3->client, 0x0d04,
					GC20C3_REG_VALUE_08BIT, gain_reg_val_table[i][0]);
		ret |= gc20c3_write_reg(gc20c3->client, 0x0d05,
					GC20C3_REG_VALUE_08BIT, gain_reg_val_table[i][1]);
		ret |= gc20c3_write_reg(gc20c3->client, 0x0e36,
					GC20C3_REG_VALUE_08BIT, gain_reg_val_table[i][2]);
		ret |= gc20c3_write_reg(gc20c3->client, 0x0e39,
					GC20C3_REG_VALUE_08BIT, gain_reg_val_table[i][3]);
		ret |= gc20c3_write_reg(gc20c3->client, 0x04a8,
					GC20C3_REG_VALUE_08BIT, gain_reg_val_table[i][4]);
		ret |= gc20c3_write_reg(gc20c3->client, 0x04a9,
					GC20C3_REG_VALUE_08BIT, gain_reg_val_table[i][5]);
		ret |= gc20c3_write_reg(gc20c3->client, 0x0052,
					GC20C3_REG_VALUE_08BIT, gain_reg_val_table[i][6]);
	} else {
		dev_err(&gc20c3->client->dev, "Invalid gain table index: %d\n", i);
		return -EINVAL;
	}

	// digital gain
	gc20c3_write_reg(gc20c3->client, 0x0474,
			 GC20C3_REG_VALUE_08BIT, (tol_dig_gain >> 8) & 0xff);
	gc20c3_write_reg(gc20c3->client, 0x0475,
			 GC20C3_REG_VALUE_08BIT, (tol_dig_gain & 0xff));

	return ret;
}

static void gc20c3_modify_fps_info(struct gc20c3 *gc20c3)
{
	const struct gc20c3_mode *mode = gc20c3->cur_mode;

	gc20c3->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				      gc20c3->cur_vts;
}

static int gc20c3_set_flip(struct gc20c3 *gc20c3, u8 mode);

static int gc20c3_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc20c3 *gc20c3 = container_of(ctrl->handler,
					     struct gc20c3, ctrl_handler);
	struct i2c_client *client = gc20c3->client;
	s64 max;
	int ret = 0;
	u32 vts = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc20c3->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(gc20c3->exposure,
					 gc20c3->exposure->minimum, max,
					 gc20c3->exposure->step,
					 gc20c3->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		ret = gc20c3_write_reg(gc20c3->client, GC20C3_REG_EXP_H,
					GC20C3_REG_VALUE_08BIT,
					(ctrl->val >> 8) & 0xff);
		ret |= gc20c3_write_reg(gc20c3->client, GC20C3_REG_EXP_L,
					GC20C3_REG_VALUE_08BIT,
					ctrl->val & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		gc20c3_set_gain(gc20c3, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		vts = ctrl->val + gc20c3->cur_mode->height;
		ret = gc20c3_write_reg(gc20c3->client, GC20C3_REG_VTS_H,
					GC20C3_REG_VALUE_08BIT, (vts >> 8) & 0xff);
		ret |= gc20c3_write_reg(gc20c3->client, GC20C3_REG_VTS_L,
					GC20C3_REG_VALUE_08BIT, vts & 0xff);
		/* TBD: master and slave not sync to streaming, but except sleep 20ms below */
		usleep_range(20000, 50000);
		gc20c3->cur_vts = vts;
		if (gc20c3->cur_vts != gc20c3->cur_mode->vts_def)
			gc20c3_modify_fps_info(gc20c3);
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			gc20c3->flip |= GC20C3_MIRROR_BIT_MASK;
		else
			gc20c3->flip &= ~GC20C3_MIRROR_BIT_MASK;

		gc20c3_set_flip(gc20c3, gc20c3->flip);
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			gc20c3->flip |= GC20C3_FLIP_BIT_MASK;
		else
			gc20c3->flip &= ~GC20C3_FLIP_BIT_MASK;

		gc20c3_set_flip(gc20c3, gc20c3->flip);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc20c3_ctrl_ops = {
	.s_ctrl = gc20c3_set_ctrl,
};

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc20c3_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC20C3_XVCLK_FREQ / 1000 / 1000);
}

static int __gc20c3_power_on(struct gc20c3 *gc20c3)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc20c3->client->dev;

	if (!IS_ERR_OR_NULL(gc20c3->pins_default)) {
		ret = pinctrl_select_state(gc20c3->pinctrl,
					   gc20c3->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(gc20c3->xvclk, GC20C3_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc20c3->xvclk) != GC20C3_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc20c3->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		goto err_clk;
	}

	if (gc20c3->is_thunderboot)
		return 0;

	if (!IS_ERR(gc20c3->reset_gpio))
		gpiod_set_value_cansleep(gc20c3->reset_gpio, 0);

	if (!IS_ERR(gc20c3->pwdn_gpio))
		gpiod_set_value_cansleep(gc20c3->pwdn_gpio, 0);

	usleep_range(500, 1000);
	ret = regulator_bulk_enable(GC20C3_NUM_SUPPLIES, gc20c3->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(gc20c3->power_gpio)) {
		gpiod_set_value_cansleep(gc20c3->power_gpio, 1);
		usleep_range(100, 200);
	}
	if (!IS_ERR(gc20c3->reset_gpio)) {
		gpiod_set_value_cansleep(gc20c3->reset_gpio, 1);
		usleep_range(100, 200);
	}
	usleep_range(3000, 6000);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc20c3_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

err_clk:
	if (!IS_ERR(gc20c3->reset_gpio))
		gpiod_direction_output(gc20c3->reset_gpio, 0);
disable_clk:
	clk_disable_unprepare(gc20c3->xvclk);
	return ret;
}

static void __gc20c3_power_off(struct gc20c3 *gc20c3)
{
	int ret;
	struct device *dev = &gc20c3->client->dev;

	clk_disable_unprepare(gc20c3->xvclk);
	if (gc20c3->is_thunderboot) {
		if (gc20c3->is_first_streamoff) {
			gc20c3->is_thunderboot = false;
			gc20c3->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(gc20c3->pwdn_gpio))
		gpiod_set_value_cansleep(gc20c3->pwdn_gpio, 0);

	if (!IS_ERR(gc20c3->reset_gpio))
		gpiod_set_value_cansleep(gc20c3->reset_gpio, 0);

	if (!IS_ERR_OR_NULL(gc20c3->pins_sleep)) {
		ret = pinctrl_select_state(gc20c3->pinctrl,
					   gc20c3->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(GC20C3_NUM_SUPPLIES, gc20c3->supplies);
	if (!IS_ERR(gc20c3->power_gpio))
		gpiod_set_value_cansleep(gc20c3->power_gpio, 0);
}

static int gc20c3_set_flip(struct gc20c3 *gc20c3, u8 mode)
{
	u32 match_reg = 0;

	gc20c3_read_reg(gc20c3->client, GC20C3_FLIP_MIRROR_REG,
			GC20C3_REG_VALUE_08BIT, &match_reg);

	if (mode == GC20C3_FLIP_BIT_MASK) {
		match_reg |= GC20C3_FLIP_BIT_MASK;
		match_reg &= ~GC20C3_MIRROR_BIT_MASK;
	} else if (mode == GC20C3_MIRROR_BIT_MASK) {
		match_reg |= GC20C3_MIRROR_BIT_MASK;
		match_reg &= ~GC20C3_FLIP_BIT_MASK;
	} else if (mode == (GC20C3_MIRROR_BIT_MASK |
		GC20C3_FLIP_BIT_MASK)) {
		match_reg |= GC20C3_FLIP_BIT_MASK;
		match_reg |= GC20C3_MIRROR_BIT_MASK;
	} else {
		match_reg &= ~GC20C3_FLIP_BIT_MASK;
		match_reg &= ~GC20C3_MIRROR_BIT_MASK;
	}
	return gc20c3_write_reg(gc20c3->client, GC20C3_FLIP_MIRROR_REG,
				GC20C3_REG_VALUE_08BIT, match_reg);
}

static void gc20c3_get_module_inf(struct gc20c3 *gc20c3,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, GC20C3_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, gc20c3->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, gc20c3->len_name, sizeof(inf->base.lens));
}

static int gc20c3_get_channel_info(struct gc20c3 *gc20c3, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = gc20c3->cur_mode->vc[ch_info->index];
	ch_info->width = gc20c3->cur_mode->width;
	ch_info->height = gc20c3->cur_mode->height;
	ch_info->bus_fmt = gc20c3->cur_mode->bus_fmt;
	return 0;
}

static long gc20c3_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	long ret = 0;
	struct rkmodule_hdr_cfg *hdr_cfg;
	u32 stream = 0;
	struct rkmodule_channel_info *ch_info;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		gc20c3_get_module_inf(gc20c3, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = gc20c3->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = gc20c3_write_reg(gc20c3->client, GC20C3_REG_CTRL_MODE,
					       GC20C3_REG_VALUE_08BIT, GC20C3_MODE_STREAMING);
		else
			ret = gc20c3_write_reg(gc20c3->client, GC20C3_REG_CTRL_MODE,
					       GC20C3_REG_VALUE_08BIT, GC20C3_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = gc20c3_get_channel_info(gc20c3, ch_info);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long gc20c3_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *awb_cfg;
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 stream = 0;
	struct rkmodule_channel_info *ch_info;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc20c3_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		awb_cfg = kzalloc(sizeof(*awb_cfg), GFP_KERNEL);
		if (!awb_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(awb_cfg, up, sizeof(*awb_cfg))) {
			kfree(awb_cfg);
			return -EFAULT;
		}

		ret = gc20c3_ioctl(sd, cmd, awb_cfg);
		kfree(awb_cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc20c3_ioctl(sd, cmd, hdr);
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

		if (copy_from_user(hdr, up, sizeof(*hdr))) {
			kfree(hdr);
			return -EFAULT;
		}

		ret = gc20c3_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		ret = gc20c3_ioctl(sd, cmd, &stream);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc20c3_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}
#endif

static int __gc20c3_start_stream(struct gc20c3 *gc20c3)
{
	struct i2c_client *client = gc20c3->client;
	int ret;

	if (!gc20c3->is_thunderboot) {
		ret = gc20c3_write_array(client, gc20c3->cur_mode->reg_list);
		if (ret)
			return ret;

		/* In case these controls are set before streaming */
		mutex_unlock(&gc20c3->mutex);
		ret = v4l2_ctrl_handler_setup(&gc20c3->ctrl_handler);
		mutex_lock(&gc20c3->mutex);

		if (ret)
			return ret;
		if (gc20c3->has_init_exp && gc20c3->cur_mode->hdr_mode != NO_HDR) {
			ret = gc20c3_ioctl(&gc20c3->subdev, PREISP_CMD_SET_HDRAE_EXP,
					&gc20c3->init_hdrae_exp);
			if (ret) {
				dev_err(&gc20c3->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
		ret |= gc20c3_set_flip(gc20c3, gc20c3->flip);
		if (ret)
			return ret;
	} else {
		dev_info(&gc20c3->client->dev, "thunderboot mode, just streaming\n");
	}

	ret = gc20c3_write_reg(gc20c3->client, GC20C3_REG_CTRL_MODE,
				GC20C3_REG_VALUE_08BIT, GC20C3_MODE_STREAMING);
	dev_info(&gc20c3->client->dev, "write stream done, streaming ......, ret: %d\n", ret);

	return ret;
}

static int __gc20c3_stop_stream(struct gc20c3 *gc20c3)
{
	return gc20c3_write_reg(gc20c3->client, GC20C3_REG_CTRL_MODE,
				GC20C3_REG_VALUE_08BIT,
				GC20C3_MODE_SW_STANDBY);
}

static int gc20c3_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	struct i2c_client *client = gc20c3->client;
	int ret = 0;

	mutex_lock(&gc20c3->mutex);
	on = !!on;
	if (on == gc20c3->streaming)
		goto unlock_and_return;

	if (on) {
		if (gc20c3->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			gc20c3->is_thunderboot = false;
			__gc20c3_power_on(gc20c3);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc20c3_start_stream(gc20c3);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc20c3_stop_stream(gc20c3);
		pm_runtime_put(&client->dev);
	}

	gc20c3->streaming = on;

unlock_and_return:
	mutex_unlock(&gc20c3->mutex);
	return 0;
}

static int gc20c3_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	const struct gc20c3_mode *mode = gc20c3->cur_mode;

	if (gc20c3->streaming)
		fi->interval = gc20c3->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct gc20c3_mode *gc20c3_find_mode(struct gc20c3 *gc20c3, int fps)
{
	const struct gc20c3_mode *mode = NULL;
	const struct gc20c3_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode->width == gc20c3->cur_mode->width &&
		    mode->height == gc20c3->cur_mode->height &&
		    mode->bus_fmt == gc20c3->cur_mode->bus_fmt &&
		    mode->hdr_mode == gc20c3->cur_mode->hdr_mode) {
			cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
			if (cur_fps == fps) {
				match = mode;
				break;
			}
		}
	}
	return match;
}

static int gc20c3_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	const struct gc20c3_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	int fps;

	if (gc20c3->streaming)
		return -EBUSY;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}
	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = gc20c3_find_mode(gc20c3, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		return -EINVAL;
	}

	gc20c3->cur_mode = mode;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(gc20c3->hblank, h_blank,
				 h_blank, 1, h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(gc20c3->vblank, vblank_def,
				 GC20C3_VTS_MAX - mode->height,
				 1, vblank_def);

	dst_link_freq = mode->link_freq_idx;
	dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
		     GC20C3_BITS_PER_SAMPLE * 2 * GC20C3_LANES;
	__v4l2_ctrl_s_ctrl_int64(gc20c3->pixel_rate,
				 dst_pixel_rate);
	__v4l2_ctrl_s_ctrl(gc20c3->link_freq,
			   dst_link_freq);
	gc20c3->cur_vts = mode->vts_def;
	gc20c3->cur_fps = mode->max_fps;

	return 0;
}

static int gc20c3_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	const struct gc20c3_mode *mode = gc20c3->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (GC20C3_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;
	return 0;
}
static int gc20c3_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];
	return 0;
}

static int gc20c3_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);

	if (fse->index >= gc20c3->cfg_num)
		return -EINVAL;

	if (fse->code != gc20c3->cur_mode->bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int gc20c3_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);

	if (fie->index >= gc20c3->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int gc20c3_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	const struct gc20c3_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	mutex_lock(&gc20c3->mutex);

	mode = gc20c3_find_best_fit(gc20c3, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc20c3->mutex);
		return -ENOTTY;
#endif
	} else {
		gc20c3->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc20c3->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc20c3->vblank, vblank_def,
					 GC20C3_VTS_MAX - mode->height,
					 1, vblank_def);

		dst_link_freq = mode->link_freq_idx;
		dst_pixel_rate = GC20C3_PIXEL_RATE_LINEAR;
		__v4l2_ctrl_s_ctrl_int64(gc20c3->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(gc20c3->link_freq,
				   dst_link_freq);
		gc20c3->cur_vts = mode->vts_def;
		gc20c3->cur_fps = mode->max_fps;
	}

	mutex_unlock(&gc20c3->mutex);

	return 0;
}

static int gc20c3_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	const struct gc20c3_mode *mode = gc20c3->cur_mode;

	mutex_lock(&gc20c3->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc20c3->mutex);
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
	mutex_unlock(&gc20c3->mutex);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc20c3_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc20c3_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc20c3->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc20c3->mutex);
	/* No crop or compose */
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc20c3_internal_ops = {
	.open = gc20c3_open,
};
#endif

static int gc20c3_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc20c3 *gc20c3 = to_gc20c3(sd);
	struct i2c_client *client = gc20c3->client;
	int ret = 0;

	mutex_lock(&gc20c3->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc20c3->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc20c3->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc20c3->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc20c3->mutex);

	return ret;
}

static const struct v4l2_subdev_core_ops gc20c3_core_ops = {
	.s_power = gc20c3_s_power,
	.ioctl = gc20c3_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc20c3_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc20c3_video_ops = {
	.s_stream = gc20c3_s_stream,
	.g_frame_interval = gc20c3_g_frame_interval,
	.s_frame_interval = gc20c3_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc20c3_pad_ops = {
	.enum_mbus_code = gc20c3_enum_mbus_code,
	.enum_frame_size = gc20c3_enum_frame_sizes,
	.enum_frame_interval = gc20c3_enum_frame_interval,
	.get_fmt = gc20c3_get_fmt,
	.set_fmt = gc20c3_set_fmt,
	.get_mbus_config = gc20c3_g_mbus_config,
};

static const struct v4l2_subdev_ops gc20c3_subdev_ops = {
	.core   = &gc20c3_core_ops,
	.video  = &gc20c3_video_ops,
	.pad    = &gc20c3_pad_ops,
};

static int __maybe_unused gc20c3_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc20c3 *gc20c3 = to_gc20c3(sd);

	__gc20c3_power_on(gc20c3);
	return 0;
}

static int __maybe_unused gc20c3_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc20c3 *gc20c3 = to_gc20c3(sd);

	__gc20c3_power_off(gc20c3);
	return 0;
}

static const struct dev_pm_ops gc20c3_pm_ops = {
	SET_RUNTIME_PM_OPS(gc20c3_runtime_suspend,
			   gc20c3_runtime_resume, NULL)
};


static int gc20c3_initialize_controls(struct gc20c3 *gc20c3)
{
	const struct gc20c3_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	handler = &gc20c3->ctrl_handler;
	mode = gc20c3->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc20c3->mutex;

	gc20c3->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
					  0, 0, link_freq_menu_items);
	if (gc20c3->link_freq)
		gc20c3->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	dst_link_freq = mode->link_freq_idx;
	dst_pixel_rate = (u32)link_freq_menu_items[dst_link_freq] /
				GC20C3_BITS_PER_SAMPLE * 2 * GC20C3_LANES;
	gc20c3->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, dst_pixel_rate, 1, dst_pixel_rate);
	__v4l2_ctrl_s_ctrl(gc20c3->link_freq, dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	gc20c3->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (gc20c3->hblank)
		gc20c3->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc20c3->vblank = v4l2_ctrl_new_std(handler, &gc20c3_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC20C3_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	gc20c3->exposure = v4l2_ctrl_new_std(handler, &gc20c3_ctrl_ops,
				V4L2_CID_EXPOSURE, GC20C3_EXPOSURE_MIN,
				exposure_max, GC20C3_EXPOSURE_STEP,
				mode->exp_def);

	gc20c3->anal_gain = v4l2_ctrl_new_std(handler, &gc20c3_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, GC20C3_GAIN_MIN,
				GC20C3_GAIN_MAX, GC20C3_GAIN_STEP,
				GC20C3_GAIN_DEFAULT);

	gc20c3->h_flip = v4l2_ctrl_new_std(handler, &gc20c3_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	gc20c3->v_flip = v4l2_ctrl_new_std(handler, &gc20c3_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	gc20c3->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc20c3->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc20c3->subdev.ctrl_handler = handler;
	gc20c3->has_init_exp = false;
	gc20c3->cur_fps = mode->max_fps;
	gc20c3->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

static int gc20c3_check_sensor_id(struct gc20c3 *gc20c3,
				   struct i2c_client *client)
{
	struct device *dev = &gc20c3->client->dev;
	u32 id = 0;
	int ret = 0;

	if (gc20c3->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = gc20c3_read_reg(client, GC20C3_REG_CHIP_ID_H,
				GC20C3_REG_VALUE_16BIT, &id);
	if (ret) {
		dev_err(&client->dev, "gc20c3_read_reg failed (%d)\n", ret);
		return ret;
	}

	if (id != GC20C3_CHIP_ID) {
		dev_err(&client->dev,
				"Sensor detection failed (%04X,%d)\n",
				id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected GC%04x sensor success\n", id);
	return 0;
}

static int gc20c3_configure_regulators(struct gc20c3 *gc20c3)
{
	unsigned int i;

	for (i = 0; i < GC20C3_NUM_SUPPLIES; i++)
		gc20c3->supplies[i].supply = gc20c3_supply_names[i];

	return devm_regulator_bulk_get(&gc20c3->client->dev,
					   GC20C3_NUM_SUPPLIES,
					   gc20c3->supplies);
}


static int gc20c3_parse_of(struct gc20c3 *gc20c3)
{
	struct device *dev = &gc20c3->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;
	u64 pixel_rate;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}

	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	if (rval == 2) {
		gc20c3->cur_mode = &supported_modes[0];
		gc20c3->cfg_num = ARRAY_SIZE(supported_modes);

		/*pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		pixel_rate = GC20C3_LINK_FREQ_270M * 2 * GC20C3_LANES / 10U;
		dev_info(dev, "lane_num(%d)  pixel_rate(%llu)\n",
			 rval, pixel_rate);
	} else {
		dev_err(dev, "Unsupported mipi lane num(%d)\n", rval);
		return -1;
	}
	return 0;
}
static int gc20c3_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc20c3 *gc20c3;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc20c3 = devm_kzalloc(dev, sizeof(*gc20c3), GFP_KERNEL);
	if (!gc20c3)
		return -ENOMEM;

	gc20c3->client = client;
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc20c3->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
					   &gc20c3->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
					   &gc20c3->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
					   &gc20c3->len_name);
	if (ret) {
		dev_err(dev,
			"could not get module information!\n");
		return -EINVAL;
	}

	gc20c3->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	gc20c3->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(gc20c3->xvclk)) {
		dev_err(&client->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	gc20c3->reset_gpio = devm_gpiod_get(dev, "reset",
				gc20c3->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(gc20c3->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc20c3->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
				gc20c3->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(gc20c3->pwdn_gpio))
		dev_info(dev, "Failed to get pwdn-gpios, maybe no used\n");

	gc20c3->power_gpio = devm_gpiod_get(dev, "power",
				gc20c3->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(gc20c3->power_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");

	ret = gc20c3_parse_of(gc20c3);
	if (ret != 0)
		return -EINVAL;

	gc20c3->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gc20c3->pinctrl)) {
		gc20c3->pins_default =
			pinctrl_lookup_state(gc20c3->pinctrl,
						 OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gc20c3->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		gc20c3->pins_sleep =
			pinctrl_lookup_state(gc20c3->pinctrl,
						 OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gc20c3->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = gc20c3_configure_regulators(gc20c3);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&gc20c3->mutex);

	sd = &gc20c3->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc20c3_subdev_ops);
	ret = gc20c3_initialize_controls(gc20c3);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc20c3_power_on(gc20c3);
	if (ret)
		goto err_free_handler;

	ret = gc20c3_check_sensor_id(gc20c3, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc20c3_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc20c3->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc20c3->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc20c3->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc20c3->module_index, facing,
		 GC20C3_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (gc20c3->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif

err_power_off:
	__gc20c3_power_off(gc20c3);
err_free_handler:
	v4l2_ctrl_handler_free(&gc20c3->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc20c3->mutex);

	return ret;
}

static int gc20c3_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc20c3 *gc20c3 = to_gc20c3(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc20c3->ctrl_handler);
	mutex_destroy(&gc20c3->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc20c3_power_off(gc20c3);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct i2c_device_id gc20c3_match_id[] = {
	{ "galaxycore,gc20c3", 0 },
	{ },
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc20c3_of_match[] = {
	{ .compatible = "galaxycore,gc20c3" },
	{},
};
MODULE_DEVICE_TABLE(of, gc20c3_of_match);
#endif

static struct i2c_driver gc20c3_i2c_driver = {
	.driver = {
		.name = GC20C3_NAME,
		.pm = &gc20c3_pm_ops,
		.of_match_table = of_match_ptr(gc20c3_of_match),
	},
	.probe      = gc20c3_probe,
	.remove     = gc20c3_remove,
	.id_table   = gc20c3_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc20c3_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc20c3_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("galaxycore GC20C3 Sensor driver");
MODULE_LICENSE("GPL");
