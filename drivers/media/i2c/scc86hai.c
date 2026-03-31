// SPDX-License-Identifier: GPL-2.0
/*
 * scc86hai driver
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
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

#define DRIVER_VERSION KERNEL_VERSION(0, 0x01, 0x05)

#define MIPI_FREQ_720M 720000000
#define SCC86HAI_MAX_LINK_FREQ MIPI_FREQ_720M
#define SCC86HAI_4LANES 4
#define SCC86HAI_BITS_PER_SAMPLE 10

/* pixel_rate = link_freq * 2 * lanes / bpp */
#define SCC86HAI_MAX_PIXEL_RATE                                       \
	(SCC86HAI_MAX_LINK_FREQ * 2ULL * SCC86HAI_4LANES /            \
	 SCC86HAI_BITS_PER_SAMPLE)
#define OF_CAMERA_HDR_MODE "rockchip,camera-hdr-mode"

#define CHIP_ID 0xB99A
#define SCC86HAI_REG_CHIP_ID 0x3107

#define SCC86HAI_REG_CTRL_MODE 0x0100
#define SCC86HAI_MODE_SW_STANDBY 0x0
#define SCC86HAI_MODE_STREAMING BIT(0)

/* 0x3018/0x3019: MIPI data/clk output; off drives low + Hi-Z.
 * 0x3018 bit3 disables the unused 2nd MIPI group (chip is 2x4lane);
 * 4lane-only uses 0x7a to cut that group's power.
 */
#define SCC86HAI_REG_MIPI_DATA    0x3018
#define SCC86HAI_REG_MIPI_CLK     0x3019
#define SCC86HAI_MIPI_DATA_NORMAL 0x7a
#define SCC86HAI_MIPI_CLK_NORMAL  0xf0
#define SCC86HAI_MIPI_DATA_HIZ    0x7f
#define SCC86HAI_MIPI_CLK_HIZ     0xff
/* 0x302c: sensor low-power state to reduce consumption */
#define SCC86HAI_REG_LOW_POWER    0x302c
#define SCC86HAI_LOW_POWER_NORMAL 0x00
#define SCC86HAI_LOW_POWER_ENABLE 0x0f

/* Exposure */
#define SCC86HAI_EXPOSURE_MIN 3
#define SCC86HAI_EXPOSURE_STEP 1
#define SCC86HAI_VTS_MAX 0x3fffff /* 22-bit: 6+8+8 */

/* Gain */
#define SCC86HAI_REG_AGAIN_H 0x3e08
#define SCC86HAI_REG_AGAIN_L 0x3e09

#define SCC86HAI_FETCH_AGAIN_H(VAL) (((VAL) >> 8) & 0xFF)
#define SCC86HAI_FETCH_AGAIN_L(VAL) ((VAL) & 0xFF)

#define SCC86HAI_GAIN_MIN 0x10 /* real gain 1x */
#define SCC86HAI_GAIN_MAX (16 * 893) /* real gain 893x */
#define SCC86HAI_GAIN_STEP 1
#define SCC86HAI_GAIN_DEFAULT 0x40

/* VTS (Vertical Total Size) registers */
#define SCC86HAI_REG_VTS_H 0x326d /* [21:16] */
#define SCC86HAI_REG_VTS_M 0x320e /* [15:8] */
#define SCC86HAI_REG_VTS_L 0x320f /* [7:0] */

#define SCC86HAI_FETCH_VTS_H(VAL) (((VAL) >> 16) & 0x3F)
#define SCC86HAI_FETCH_VTS_M(VAL) (((VAL) >> 8) & 0xFF)
#define SCC86HAI_FETCH_VTS_L(VAL) ((VAL) & 0xFF)

/* Exposure registers (24-bit value across 4 registers) */
#define SCC86HAI_REG_EXP_0 0x3e20 /* [23:20] */
#define SCC86HAI_REG_EXP_1 0x3e00 /* [19:12] */
#define SCC86HAI_REG_EXP_2 0x3e01 /* [11:4] */
#define SCC86HAI_REG_EXP_3 0x3e02 /* [3:0] */

#define SCC86HAI_FETCH_EXP_0(VAL) (((VAL) >> 20) & 0x0F)
#define SCC86HAI_FETCH_EXP_1(VAL) (((VAL) >> 12) & 0xFF)
#define SCC86HAI_FETCH_EXP_2(VAL) (((VAL) >> 4) & 0xFF)
#define SCC86HAI_FETCH_EXP_3(VAL) (((VAL) & 0x0F) << 4)

#define SCC86HAI_SOFTWARE_RESET_REG 0x0103
#define SCC86HAI_REG_TEST_PATTERN 0x4501
#define SCC86HAI_TEST_PATTERN_ENABLE 0x08

#define SCC86HAI_FLIP_REG 0x3221
#define SCC86HAI_FLIP_MASK 0x60
#define SCC86HAI_MIRROR_MASK 0x06

#define REG_NULL 0xFFFF

#define SCC86HAI_REG_VALUE_08BIT 1
#define SCC86HAI_REG_VALUE_16BIT 2

#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP "rockchip,camera_sleep"
#define SCC86HAI_NAME "scc86hai"

static const char *const scc86hai_supply_names[] = {
	"dvdd",  /* Digital core power */
	"dovdd", /* Digital I/O power */
	"avdd",  /* Analog power */
};

#define SCC86HAI_NUM_SUPPLIES ARRAY_SIZE(scc86hai_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct scc86hai_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 mipi_freq_idx;
	u32 xvclk_freq;
	u32 bpp;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 bayer_mode;
	u32 vc[PAD_MAX];
};

struct scc86hai {
	struct i2c_client *client;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct regulator_bulk_data supplies[SCC86HAI_NUM_SUPPLIES];

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;

	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *anal_a_gain;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct mutex mutex;
	struct v4l2_fract cur_fps;
	bool streaming;
	bool power_on;
	const struct scc86hai_mode *cur_mode;
	u32 module_index;
	u32 cfg_num;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	u32 cur_vts;
	bool is_thunderboot;
	bool is_first_streamoff;
	struct cam_sw_info *cam_sw_info;
};

#define to_scc86hai(sd) container_of(sd, struct scc86hai, subdev)

static const struct regval
	scc86hai_linear10bit_5120x3072_20fps_regs[] = {
		{ 0x0103, 0x01 },
		{ 0x0100, 0x00 },
		{ 0x36e9, 0x80 },
		{ 0x37f9, 0x80 },
		{ 0x2317, 0x00 },
		{ 0x2386, 0x88 },
		{ 0x23b0, 0x00 },
		{ 0x23b1, 0x08 },
		{ 0x23b2, 0x00 },
		{ 0x23b3, 0x18 },
		{ 0x23b4, 0x00 },
		{ 0x23b5, 0x38 },
		{ 0x23b6, 0x04 },
		{ 0x23b7, 0x08 },
		{ 0x23b8, 0x04 },
		{ 0x23b9, 0x18 },
		{ 0x23ba, 0x04 },
		{ 0x23bb, 0x78 },
		{ 0x23bc, 0x04 },
		{ 0x23bd, 0x08 },
		{ 0x23be, 0x04 },
		{ 0x23bf, 0x78 },
		{ 0x23c0, 0x04 },
		{ 0x23c1, 0x00 },
		{ 0x23c2, 0x04 },
		{ 0x23c3, 0x38 },
		{ 0x23c4, 0x04 },
		{ 0x23c5, 0x78 },
		{ 0x23c6, 0x04 },
		{ 0x23c7, 0x08 },
		{ 0x23c8, 0x04 },
		{ 0x23c9, 0x78 },
		{ 0x3018, 0x7a },
		{ 0x3019, 0xf0 },
		{ 0x301a, 0xf0 },
		{ 0x301c, 0x40 },
		{ 0x301e, 0x3c },
		{ 0x301f, 0x05 },
		{ 0x3023, 0x06 },
		{ 0x3029, 0x05 },
		{ 0x302b, 0x2f },
		{ 0x3037, 0x60 },
		{ 0x3106, 0x03 },
		{ 0x3107, 0xb9 },
		{ 0x3108, 0x9a },
		{ 0x3204, 0x14 },
		{ 0x3205, 0x1f },
		{ 0x3206, 0x0c },
		{ 0x3207, 0x1f },
		{ 0x3208, 0x14 },
		{ 0x3209, 0x00 },
		{ 0x320a, 0x0c },
		{ 0x320b, 0x00 },
		{ 0x320c, 0x04 },
		{ 0x320d, 0x65 },
		{ 0x320e, 0x0c },
		{ 0x320f, 0x80 },
		{ 0x326d, 0x00 },
		{ 0x3211, 0x10 },
		{ 0x3213, 0x10 },
		{ 0x3214, 0x11 },
		{ 0x3215, 0x11 },
		{ 0x3220, 0x00 },
		{ 0x3250, 0x00 },
		{ 0x3253, 0x0c },
		{ 0x3259, 0x04 },
		{ 0x325f, 0x88 },
		{ 0x3280, 0x17 },
		{ 0x32d1, 0x78 },
		{ 0x32dd, 0x00 },
		{ 0x32e0, 0x00 },
		{ 0x32f7, 0x70 },
		{ 0x3301, 0x08 },
		{ 0x3302, 0x10 },
		{ 0x3304, 0x68 },
		{ 0x3305, 0x00 },
		{ 0x3306, 0x68 },
		{ 0x3308, 0x10 },
		{ 0x3309, 0x80 },
		{ 0x330a, 0x00 },
		{ 0x330b, 0xc8 },
		{ 0x330d, 0x18 },
		{ 0x330e, 0x18 },
		{ 0x331c, 0x01 },
		{ 0x331e, 0x51 },
		{ 0x331f, 0x69 },
		{ 0x3320, 0x0c },
		{ 0x3326, 0x14 },
		{ 0x3333, 0x10 },
		{ 0x3334, 0x40 },
		{ 0x3364, 0x5e },
		{ 0x336c, 0xcc },
		{ 0x3393, 0x0c },
		{ 0x3394, 0x22 },
		{ 0x3395, 0x22 },
		{ 0x3399, 0x08 },
		{ 0x339a, 0x08 },
		{ 0x339b, 0x10 },
		{ 0x339c, 0x1e },
		{ 0x33ad, 0x1c },
		{ 0x33ae, 0x30 },
		{ 0x33af, 0x48 },
		{ 0x33b0, 0x0f },
		{ 0x33b2, 0x50 },
		{ 0x33b3, 0x18 },
		{ 0x33bf, 0x01 },
		{ 0x33f8, 0x00 },
		{ 0x33f9, 0x70 },
		{ 0x33fa, 0x00 },
		{ 0x33fb, 0x78 },
		{ 0x3428, 0x28 },
		{ 0x3429, 0x04 },
		{ 0x342a, 0x20 },
		{ 0x349f, 0x03 },
		{ 0x34a8, 0x10 },
		{ 0x34a9, 0x08 },
		{ 0x34aa, 0x00 },
		{ 0x34ab, 0xd0 },
		{ 0x34ac, 0x00 },
		{ 0x34ad, 0xd8 },
		{ 0x34ba, 0x06 },
		{ 0x34bb, 0x04 },
		{ 0x34f9, 0x08 },
		{ 0x3604, 0x80 },
		{ 0x3605, 0x80 },
		{ 0x3606, 0x04 },
		{ 0x3607, 0x08 },
		{ 0x3608, 0x04 },
		{ 0x3609, 0x18 },
		{ 0x360a, 0x04 },
		{ 0x360b, 0x78 },
		{ 0x360d, 0x04 },
		{ 0x360e, 0x08 },
		{ 0x360f, 0x04 },
		{ 0x3610, 0x78 },
		{ 0x3620, 0x04 },
		{ 0x3621, 0x18 },
		{ 0x3624, 0x04 },
		{ 0x3625, 0x78 },
		{ 0x3626, 0x04 },
		{ 0x3627, 0x78 },
		{ 0x362f, 0x04 },
		{ 0x3633, 0x44 },
		{ 0x3637, 0x14 },
		{ 0x363c, 0x40 },
		{ 0x363d, 0x40 },
		{ 0x363e, 0x20 },
		{ 0x364a, 0x10 },
		{ 0x3657, 0x00 },
		{ 0x3658, 0x00 },
		{ 0x3660, 0x00 },
		{ 0x3661, 0x00 },
		{ 0x3662, 0x80 },
		{ 0x3663, 0x80 },
		{ 0x366c, 0x04 },
		{ 0x366d, 0x00 },
		{ 0x366e, 0x04 },
		{ 0x366f, 0x18 },
		{ 0x3670, 0x81 },
		{ 0x3671, 0x81 },
		{ 0x3672, 0x82 },
		{ 0x3673, 0x04 },
		{ 0x3674, 0x08 },
		{ 0x3675, 0x04 },
		{ 0x3676, 0x18 },
		{ 0x3679, 0x04 },
		{ 0x367a, 0x78 },
		{ 0x3685, 0x00 },
		{ 0x3686, 0x00 },
		{ 0x3687, 0x00 },
		{ 0x3688, 0x00 },
		{ 0x3689, 0x00 },
		{ 0x368a, 0x00 },
		{ 0x368b, 0x00 },
		{ 0x368c, 0x00 },
		{ 0x368d, 0x00 },
		{ 0x368e, 0x08 },
		{ 0x368f, 0x00 },
		{ 0x3690, 0x18 },
		{ 0x3691, 0x04 },
		{ 0x3692, 0x00 },
		{ 0x3693, 0x04 },
		{ 0x3694, 0x08 },
		{ 0x3695, 0x04 },
		{ 0x3696, 0x18 },
		{ 0x3697, 0x04 },
		{ 0x3698, 0x38 },
		{ 0x3699, 0x04 },
		{ 0x369a, 0x78 },
		{ 0x36d0, 0x0f },
		{ 0x36d1, 0x10 },
		{ 0x36d2, 0x44 },
		{ 0x36d3, 0x44 },
		{ 0x36d4, 0x54 },
		{ 0x36d5, 0x04 },
		{ 0x36d6, 0x00 },
		{ 0x36d7, 0x04 },
		{ 0x36d8, 0x38 },
		{ 0x36e0, 0x84 },
		{ 0x36e1, 0x74 },
		{ 0x36ea, 0x10 },
		{ 0x36eb, 0x0d },
		{ 0x36ec, 0x5a },
		{ 0x36ed, 0xac },
		{ 0x36ef, 0x10 },
		{ 0x36fd, 0x0e },
		{ 0x36fe, 0x0f },
		{ 0x36ff, 0x0f },
		{ 0x370f, 0xf3 },
		{ 0x3710, 0x01 },
		{ 0x3722, 0x3f },
		{ 0x3724, 0xb0 },
		{ 0x3728, 0x00 },
		{ 0x372a, 0x84 },
		{ 0x372c, 0x03 },
		{ 0x3784, 0x1f },
		{ 0x3790, 0x00 },
		{ 0x3791, 0x80 },
		{ 0x3792, 0x80 },
		{ 0x3793, 0x84 },
		{ 0x3794, 0x84 },
		{ 0x3795, 0x84 },
		{ 0x3796, 0x04 },
		{ 0x3797, 0x08 },
		{ 0x3798, 0x04 },
		{ 0x3799, 0x78 },
		{ 0x379a, 0x04 },
		{ 0x379b, 0x08 },
		{ 0x379c, 0x04 },
		{ 0x379d, 0x18 },
		{ 0x379e, 0x04 },
		{ 0x379f, 0x08 },
		{ 0x37a0, 0x04 },
		{ 0x37a1, 0x78 },
		{ 0x37a2, 0x07 },
		{ 0x37a3, 0x07 },
		{ 0x37a4, 0x0f },
		{ 0x37b0, 0x3f },
		{ 0x37b1, 0x7f },
		{ 0x37b2, 0xff },
		{ 0x37b3, 0x04 },
		{ 0x37b4, 0x08 },
		{ 0x37b5, 0x04 },
		{ 0x37b6, 0x78 },
		{ 0x37b7, 0x03 },
		{ 0x37b8, 0x03 },
		{ 0x37b9, 0x03 },
		{ 0x37ba, 0x1e },
		{ 0x37bb, 0x3c },
		{ 0x37bc, 0x32 },
		{ 0x37bd, 0x04 },
		{ 0x37be, 0x08 },
		{ 0x37bf, 0x04 },
		{ 0x37c0, 0x38 },
		{ 0x37c1, 0x00 },
		{ 0x37c2, 0x08 },
		{ 0x37c3, 0x00 },
		{ 0x37c4, 0x18 },
		{ 0x37c5, 0x00 },
		{ 0x37c6, 0x08 },
		{ 0x37c7, 0x00 },
		{ 0x37c8, 0x18 },
		{ 0x37c9, 0x1e },
		{ 0x37ca, 0x3c },
		{ 0x37cb, 0x32 },
		{ 0x37cc, 0x1d },
		{ 0x37cd, 0x3a },
		{ 0x37ce, 0x34 },
		{ 0x37cf, 0x32 },
		{ 0x37d6, 0x32 },
		{ 0x37d7, 0x04 },
		{ 0x37d8, 0x00 },
		{ 0x37d9, 0x04 },
		{ 0x37da, 0x08 },
		{ 0x37db, 0x04 },
		{ 0x37dc, 0x18 },
		{ 0x37dd, 0x04 },
		{ 0x37de, 0x38 },
		{ 0x37df, 0x04 },
		{ 0x37e0, 0x78 },
		{ 0x37e1, 0x1d },
		{ 0x37e2, 0x3a },
		{ 0x37e3, 0x34 },
		{ 0x37e4, 0x32 },
		{ 0x37e5, 0x32 },
		{ 0x37e6, 0x04 },
		{ 0x37e7, 0x00 },
		{ 0x37e8, 0x04 },
		{ 0x37e9, 0x08 },
		{ 0x37ea, 0x04 },
		{ 0x37eb, 0x18 },
		{ 0x37ec, 0x04 },
		{ 0x37ed, 0x38 },
		{ 0x37ee, 0x04 },
		{ 0x37ef, 0x78 },
		{ 0x37fa, 0x08 },
		{ 0x37fb, 0x65 },
		{ 0x37fc, 0x01 },
		{ 0x37fd, 0x14 },
		{ 0x3900, 0x05 },
		{ 0x3903, 0x60 },
		{ 0x3907, 0x01 },
		{ 0x3908, 0x00 },
		{ 0x391a, 0x30 },
		{ 0x391b, 0x1a },
		{ 0x391c, 0x0c },
		{ 0x391d, 0x00 },
		{ 0x391f, 0x61 },
		{ 0x3926, 0xf2 },
		{ 0x393f, 0x80 },
		{ 0x3940, 0x04 },
		{ 0x3941, 0x00 },
		{ 0x3942, 0x04 },
		{ 0x3943, 0x7b },
		{ 0x3944, 0x7a },
		{ 0x3945, 0x7b },
		{ 0x3946, 0x7b },
		{ 0x39c9, 0x80 },
		{ 0x39dd, 0x00 },
		{ 0x39de, 0x10 },
		{ 0x39e7, 0x08 },
		{ 0x39e8, 0x08 },
		{ 0x39e9, 0x80 },
		{ 0x39f0, 0x00 },
		{ 0x39f1, 0x00 },
		{ 0x39f2, 0x00 },
		{ 0x39f3, 0x04 },
		{ 0x39f4, 0x00 },
		{ 0x39f5, 0x00 },
		{ 0x39f6, 0x04 },
		{ 0x39f7, 0x00 },
		{ 0x39f8, 0x00 },
		{ 0x39f9, 0x04 },
		{ 0x39fa, 0x00 },
		{ 0x3e00, 0x00 },
		{ 0x3e01, 0xc7 },
		{ 0x3e02, 0x80 },
		{ 0x3e03, 0x03 },
		{ 0x3e06, 0x01 },
		{ 0x3e07, 0x00 },
		{ 0x3e08, 0x00 },
		{ 0x3e09, 0x10 },
		{ 0x3e16, 0x07 },
		{ 0x3e17, 0x15 },
		{ 0x3e18, 0x01 },
		{ 0x3e19, 0xc6 },
		{ 0x3e1b, 0x72 },
		{ 0x3e1d, 0x0f },
		{ 0x3e23, 0x01 },
		{ 0x3e24, 0x00 },
		{ 0x41f9, 0xd0 },
		{ 0x41fa, 0x5a },
		{ 0x41fb, 0x69 },
		{ 0x41fc, 0x10 },
		{ 0x4402, 0x02 },
		{ 0x4403, 0x0a },
		{ 0x4404, 0x1e },
		{ 0x4405, 0x27 },
		{ 0x4406, 0x00 },
		{ 0x4407, 0xe8 },
		{ 0x440c, 0x32 },
		{ 0x440d, 0x32 },
		{ 0x440e, 0x26 },
		{ 0x440f, 0x3e },
		{ 0x4412, 0x01 },
		{ 0x4424, 0x01 },
		{ 0x4501, 0xa4 },
		{ 0x4502, 0x2c },
		{ 0x4506, 0x00 },
		{ 0x4507, 0x18 },
		{ 0x450d, 0x0c },
		{ 0x4590, 0x00 },
		{ 0x45c4, 0xb4 },
		{ 0x45e5, 0x00 },
		{ 0x4800, 0x24 },
		{ 0x480f, 0x03 },
		{ 0x4837, 0x17 },
		{ 0x4900, 0x24 },
		{ 0x4937, 0x17 },
		{ 0x5000, 0x2e },
		{ 0x5002, 0x06 },
		{ 0x5400, 0x00 },
		{ 0x5780, 0x46 },
		{ 0x5784, 0x0a },
		{ 0x5787, 0x0a },
		{ 0x5788, 0x0a },
		{ 0x5789, 0x08 },
		{ 0x578a, 0x0a },
		{ 0x578b, 0x0a },
		{ 0x578c, 0x08 },
		{ 0x5792, 0x04 },
		{ 0x5795, 0x04 },
		{ 0x57ac, 0x00 },
		{ 0x57ad, 0x00 },
		{ 0x36e9, 0x44 },
		{ 0x37f9, 0x24 },

		{ REG_NULL, 0x00 },
	};

static const struct regval
	scc86hai_linear10bit_2560x1536_60fps_regs[] = {
		{ 0x0103, 0x01 },
		{ 0x0100, 0x00 },
		{ 0x36e9, 0x80 },
		{ 0x37f9, 0x80 },
		{ 0x2317, 0x00 },
		{ 0x2386, 0x88 },
		{ 0x23b0, 0x00 },
		{ 0x23b1, 0x08 },
		{ 0x23b2, 0x00 },
		{ 0x23b3, 0x18 },
		{ 0x23b4, 0x00 },
		{ 0x23b5, 0x38 },
		{ 0x23b6, 0x04 },
		{ 0x23b7, 0x08 },
		{ 0x23b8, 0x04 },
		{ 0x23b9, 0x18 },
		{ 0x23ba, 0x04 },
		{ 0x23bb, 0x78 },
		{ 0x23bc, 0x04 },
		{ 0x23bd, 0x08 },
		{ 0x23be, 0x04 },
		{ 0x23bf, 0x78 },
		{ 0x23c0, 0x04 },
		{ 0x23c1, 0x00 },
		{ 0x23c2, 0x04 },
		{ 0x23c3, 0x38 },
		{ 0x23c4, 0x04 },
		{ 0x23c5, 0x78 },
		{ 0x23c6, 0x04 },
		{ 0x23c7, 0x08 },
		{ 0x23c8, 0x04 },
		{ 0x23c9, 0x78 },
		{ 0x3018, 0x7a },
		{ 0x3019, 0xf0 },
		{ 0x301a, 0xf0 },
		{ 0x301c, 0x40 },
		{ 0x301e, 0x3c },
		{ 0x301f, 0x06 },
		{ 0x3023, 0x06 },
		{ 0x3029, 0x05 },
		{ 0x302b, 0x2f },
		{ 0x3037, 0x60 },
		{ 0x30b0, 0x01 },
		{ 0x3106, 0x03 },
		{ 0x3107, 0xb9 },
		{ 0x3108, 0x9a },
		{ 0x3204, 0x14 },
		{ 0x3205, 0x1f },
		{ 0x3206, 0x0c },
		{ 0x3207, 0x1f },
		{ 0x3208, 0x0a },
		{ 0x3209, 0x00 },
		{ 0x320a, 0x06 },
		{ 0x320b, 0x00 },
		{ 0x320c, 0x02 },
		{ 0x320d, 0xee },
		{ 0x320e, 0x06 },
		{ 0x320f, 0x40 },
		{ 0x326d, 0x00 },
		{ 0x3211, 0x08 },
		{ 0x3213, 0x08 },
		{ 0x3214, 0x22 },
		{ 0x3215, 0x22 },
		{ 0x321a, 0x22 },
		{ 0x3220, 0x22 },
		{ 0x3250, 0x00 },
		{ 0x3253, 0x0c },
		{ 0x3259, 0x04 },
		{ 0x325f, 0x88 },
		{ 0x3280, 0x16 },
		{ 0x32d1, 0x78 },
		{ 0x32dd, 0x00 },
		{ 0x32e0, 0x00 },
		{ 0x32f7, 0x70 },
		{ 0x3301, 0x08 },
		{ 0x3302, 0x10 },
		{ 0x3304, 0x68 },
		{ 0x3305, 0x00 },
		{ 0x3306, 0x68 },
		{ 0x3308, 0x10 },
		{ 0x3309, 0x80 },
		{ 0x330a, 0x00 },
		{ 0x330b, 0xc8 },
		{ 0x330d, 0x18 },
		{ 0x330e, 0x18 },
		{ 0x331c, 0x01 },
		{ 0x331e, 0x51 },
		{ 0x331f, 0x69 },
		{ 0x3320, 0x0c },
		{ 0x3326, 0x14 },
		{ 0x3333, 0x10 },
		{ 0x3334, 0x40 },
		{ 0x3364, 0x5e },
		{ 0x336c, 0xcc },
		{ 0x3393, 0x0c },
		{ 0x3394, 0x22 },
		{ 0x3395, 0x22 },
		{ 0x3399, 0x08 },
		{ 0x339a, 0x08 },
		{ 0x339b, 0x10 },
		{ 0x339c, 0x1e },
		{ 0x33ad, 0x1c },
		{ 0x33ae, 0x30 },
		{ 0x33af, 0x48 },
		{ 0x33b0, 0x0f },
		{ 0x33b2, 0x50 },
		{ 0x33b3, 0x18 },
		{ 0x33bf, 0x01 },
		{ 0x33f8, 0x00 },
		{ 0x33f9, 0x70 },
		{ 0x33fa, 0x00 },
		{ 0x33fb, 0x78 },
		{ 0x3428, 0x28 },
		{ 0x3429, 0x04 },
		{ 0x342a, 0x20 },
		{ 0x349f, 0x03 },
		{ 0x34a8, 0x10 },
		{ 0x34a9, 0x08 },
		{ 0x34aa, 0x00 },
		{ 0x34ab, 0xd0 },
		{ 0x34ac, 0x00 },
		{ 0x34ad, 0xd8 },
		{ 0x34ba, 0x06 },
		{ 0x34bb, 0x04 },
		{ 0x34f9, 0x08 },
		{ 0x3604, 0x80 },
		{ 0x3605, 0x80 },
		{ 0x3606, 0x04 },
		{ 0x3607, 0x08 },
		{ 0x3608, 0x04 },
		{ 0x3609, 0x18 },
		{ 0x360a, 0x04 },
		{ 0x360b, 0x78 },
		{ 0x360d, 0x04 },
		{ 0x360e, 0x08 },
		{ 0x360f, 0x04 },
		{ 0x3610, 0x78 },
		{ 0x3620, 0x04 },
		{ 0x3621, 0x18 },
		{ 0x3624, 0x04 },
		{ 0x3625, 0x78 },
		{ 0x3626, 0x04 },
		{ 0x3627, 0x78 },
		{ 0x362f, 0x04 },
		{ 0x3633, 0x44 },
		{ 0x3637, 0x1b },
		{ 0x363c, 0x40 },
		{ 0x363d, 0x40 },
		{ 0x363e, 0x20 },
		{ 0x364a, 0x10 },
		{ 0x3654, 0x30 },
		{ 0x3657, 0x00 },
		{ 0x3658, 0x00 },
		{ 0x365d, 0x30 },
		{ 0x3660, 0x00 },
		{ 0x3661, 0x00 },
		{ 0x3662, 0x80 },
		{ 0x3663, 0x80 },
		{ 0x366c, 0x04 },
		{ 0x366d, 0x00 },
		{ 0x366e, 0x04 },
		{ 0x366f, 0x18 },
		{ 0x3670, 0x81 },
		{ 0x3671, 0x81 },
		{ 0x3672, 0x82 },
		{ 0x3673, 0x04 },
		{ 0x3674, 0x08 },
		{ 0x3675, 0x04 },
		{ 0x3676, 0x18 },
		{ 0x3679, 0x04 },
		{ 0x367a, 0x78 },
		{ 0x3685, 0x00 },
		{ 0x3686, 0x00 },
		{ 0x3687, 0x00 },
		{ 0x3688, 0x00 },
		{ 0x3689, 0x00 },
		{ 0x368a, 0x00 },
		{ 0x368b, 0x00 },
		{ 0x368c, 0x00 },
		{ 0x368d, 0x00 },
		{ 0x368e, 0x08 },
		{ 0x368f, 0x00 },
		{ 0x3690, 0x18 },
		{ 0x3691, 0x04 },
		{ 0x3692, 0x00 },
		{ 0x3693, 0x04 },
		{ 0x3694, 0x08 },
		{ 0x3695, 0x04 },
		{ 0x3696, 0x18 },
		{ 0x3697, 0x04 },
		{ 0x3698, 0x38 },
		{ 0x3699, 0x04 },
		{ 0x369a, 0x78 },
		{ 0x36d0, 0x0f },
		{ 0x36d1, 0x10 },
		{ 0x36d2, 0x44 },
		{ 0x36d3, 0x44 },
		{ 0x36d4, 0x54 },
		{ 0x36d5, 0x04 },
		{ 0x36d6, 0x00 },
		{ 0x36d7, 0x04 },
		{ 0x36d8, 0x38 },
		{ 0x36e0, 0x84 },
		{ 0x36e1, 0x74 },
		{ 0x36ea, 0x10 },
		{ 0x36eb, 0x0d },
		{ 0x36ec, 0x5a },
		{ 0x36ed, 0xac },
		{ 0x36ef, 0x10 },
		{ 0x36fd, 0x0e },
		{ 0x36fe, 0x0f },
		{ 0x36ff, 0x0f },
		{ 0x370f, 0xf3 },
		{ 0x3710, 0x01 },
		{ 0x3722, 0x3f },
		{ 0x3724, 0xb0 },
		{ 0x3728, 0x00 },
		{ 0x372a, 0x84 },
		{ 0x372c, 0x03 },
		{ 0x3784, 0x1f },
		{ 0x3790, 0x00 },
		{ 0x3791, 0x00 },
		{ 0x3792, 0x80 },
		{ 0x3793, 0x84 },
		{ 0x3794, 0x84 },
		{ 0x3795, 0x84 },
		{ 0x3796, 0x04 },
		{ 0x3797, 0x08 },
		{ 0x3798, 0x04 },
		{ 0x3799, 0x78 },
		{ 0x379a, 0x04 },
		{ 0x379b, 0x08 },
		{ 0x379c, 0x04 },
		{ 0x379d, 0x18 },
		{ 0x379e, 0x04 },
		{ 0x379f, 0x08 },
		{ 0x37a0, 0x04 },
		{ 0x37a1, 0x78 },
		{ 0x37a2, 0x07 },
		{ 0x37a3, 0x07 },
		{ 0x37a4, 0x0f },
		{ 0x37b0, 0x3f },
		{ 0x37b1, 0x7f },
		{ 0x37b2, 0xff },
		{ 0x37b3, 0x04 },
		{ 0x37b4, 0x08 },
		{ 0x37b5, 0x04 },
		{ 0x37b6, 0x78 },
		{ 0x37b7, 0x03 },
		{ 0x37b8, 0x03 },
		{ 0x37b9, 0x03 },
		{ 0x37ba, 0x1e },
		{ 0x37bb, 0x3c },
		{ 0x37bc, 0x32 },
		{ 0x37bd, 0x04 },
		{ 0x37be, 0x08 },
		{ 0x37bf, 0x04 },
		{ 0x37c0, 0x38 },
		{ 0x37c1, 0x00 },
		{ 0x37c2, 0x08 },
		{ 0x37c3, 0x00 },
		{ 0x37c4, 0x18 },
		{ 0x37c5, 0x00 },
		{ 0x37c6, 0x08 },
		{ 0x37c7, 0x00 },
		{ 0x37c8, 0x18 },
		{ 0x37c9, 0x1e },
		{ 0x37ca, 0x3c },
		{ 0x37cb, 0x32 },
		{ 0x37cc, 0x1d },
		{ 0x37cd, 0x3a },
		{ 0x37ce, 0x34 },
		{ 0x37cf, 0x32 },
		{ 0x37d6, 0x32 },
		{ 0x37d7, 0x04 },
		{ 0x37d8, 0x00 },
		{ 0x37d9, 0x04 },
		{ 0x37da, 0x08 },
		{ 0x37db, 0x04 },
		{ 0x37dc, 0x18 },
		{ 0x37dd, 0x04 },
		{ 0x37de, 0x38 },
		{ 0x37df, 0x04 },
		{ 0x37e0, 0x78 },
		{ 0x37e1, 0x1d },
		{ 0x37e2, 0x3a },
		{ 0x37e3, 0x34 },
		{ 0x37e4, 0x32 },
		{ 0x37e5, 0x32 },
		{ 0x37e6, 0x04 },
		{ 0x37e7, 0x00 },
		{ 0x37e8, 0x04 },
		{ 0x37e9, 0x08 },
		{ 0x37ea, 0x04 },
		{ 0x37eb, 0x18 },
		{ 0x37ec, 0x04 },
		{ 0x37ed, 0x38 },
		{ 0x37ee, 0x04 },
		{ 0x37ef, 0x78 },
		{ 0x37fa, 0x08 },
		{ 0x37fb, 0x65 },
		{ 0x37fc, 0x01 },
		{ 0x37fd, 0x14 },
		{ 0x3900, 0x05 },
		{ 0x3903, 0x60 },
		{ 0x3907, 0x01 },
		{ 0x3908, 0x00 },
		{ 0x391a, 0x30 },
		{ 0x391b, 0x1a },
		{ 0x391c, 0x0c },
		{ 0x391d, 0x00 },
		{ 0x391f, 0x61 },
		{ 0x3926, 0xf2 },
		{ 0x393f, 0x80 },
		{ 0x3940, 0x04 },
		{ 0x3941, 0x00 },
		{ 0x3942, 0x04 },
		{ 0x3943, 0x7b },
		{ 0x3944, 0x7a },
		{ 0x3945, 0x7b },
		{ 0x3946, 0x7b },
		{ 0x39c9, 0x80 },
		{ 0x39dd, 0x00 },
		{ 0x39de, 0x10 },
		{ 0x39e7, 0x08 },
		{ 0x39e8, 0x08 },
		{ 0x39e9, 0x80 },
		{ 0x39f0, 0x00 },
		{ 0x39f1, 0x00 },
		{ 0x39f2, 0x00 },
		{ 0x39f3, 0x04 },
		{ 0x39f4, 0x00 },
		{ 0x39f5, 0x00 },
		{ 0x39f6, 0x04 },
		{ 0x39f7, 0x00 },
		{ 0x39f8, 0x00 },
		{ 0x39f9, 0x04 },
		{ 0x39fa, 0x00 },
		{ 0x3e00, 0x00 },
		{ 0x3e01, 0x63 },
		{ 0x3e02, 0x80 },
		{ 0x3e03, 0x03 },
		{ 0x3e06, 0x01 },
		{ 0x3e07, 0x00 },
		{ 0x3e08, 0x00 },
		{ 0x3e09, 0x10 },
		{ 0x3e16, 0x07 },
		{ 0x3e17, 0x15 },
		{ 0x3e18, 0x01 },
		{ 0x3e19, 0xc6 },
		{ 0x3e1b, 0x72 },
		{ 0x3e1d, 0x0f },
		{ 0x3e23, 0x01 },
		{ 0x3e24, 0x00 },
		{ 0x41f9, 0xd0 },
		{ 0x41fa, 0x5a },
		{ 0x41fb, 0x69 },
		{ 0x41fc, 0x10 },
		{ 0x4402, 0x02 },
		{ 0x4403, 0x0a },
		{ 0x4404, 0x1e },
		{ 0x4405, 0x27 },
		{ 0x4406, 0x00 },
		{ 0x4407, 0xe8 },
		{ 0x440c, 0x32 },
		{ 0x440d, 0x32 },
		{ 0x440e, 0x26 },
		{ 0x440f, 0x3e },
		{ 0x4412, 0x01 },
		{ 0x4424, 0x01 },
		{ 0x4501, 0xa4 },
		{ 0x4502, 0x2c },
		{ 0x4506, 0x00 },
		{ 0x4507, 0x18 },
		{ 0x450d, 0x0c },
		{ 0x4590, 0x00 },
		{ 0x45c4, 0xb4 },
		{ 0x45e5, 0x00 },
		{ 0x4800, 0x24 },
		{ 0x480f, 0x03 },
		{ 0x4837, 0x17 },
		{ 0x4900, 0x24 },
		{ 0x4937, 0x17 },
		{ 0x5000, 0x2e },
		{ 0x5002, 0x06 },
		{ 0x5400, 0x00 },
		{ 0x5780, 0x46 },
		{ 0x5784, 0x0a },
		{ 0x5787, 0x0a },
		{ 0x5788, 0x0a },
		{ 0x5789, 0x08 },
		{ 0x578a, 0x0a },
		{ 0x578b, 0x0a },
		{ 0x578c, 0x08 },
		{ 0x5792, 0x04 },
		{ 0x5795, 0x04 },
		{ 0x57ac, 0x00 },
		{ 0x57ad, 0x00 },
		{ 0x36e9, 0x44 },
		{ 0x37f9, 0x24 },

		{ REG_NULL, 0x00 },
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
static const struct scc86hai_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 5120,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 200000,
		},
		.exp_def = 0x0c80 - 8,
		.hts_def = 0x0465 * 5 - 0x180,
		.vts_def = 0x0c80,
		.reg_list = scc86hai_linear10bit_5120x3072_20fps_regs,
		.hdr_mode = NO_HDR,
		.bayer_mode = RKMODULE_QUARD_BAYER,
		.xvclk_freq = 27000000,
		.mipi_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = 0,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 2560,
		.height = 1536,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0640 - 8,
		.hts_def = 0x02ee * 5 - 0x180,
		.vts_def = 0x0640,
		.reg_list = scc86hai_linear10bit_2560x1536_60fps_regs,
		.hdr_mode = NO_HDR,
		.bayer_mode = RKMODULE_NORMAL_BAYER,
		.xvclk_freq = 27000000,
		.mipi_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = 0,
	},
};

static const u32 bus_code[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char *const scc86hai_test_pattern_menu[] = {
	"Disabled", "Vertical Color Bar"
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_720M,
};

/* Write registers up to 4 at a time */
static int scc86hai_write_reg(struct i2c_client *client, u16 reg, u32 len,
			      u32 val)
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

static int scc86hai_write_array(struct i2c_client *client,
				const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; regs[i].addr != REG_NULL; i++) {
		ret = scc86hai_write_reg(client, regs[i].addr,
					 SCC86HAI_REG_VALUE_08BIT, regs[i].val);
		if (ret) {
			dev_err(&client->dev,
				"write array failed at reg 0x%04x val 0x%02x, ret %d\n",
				regs[i].addr, regs[i].val, ret);
			break;
		}
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int scc86hai_read_reg(struct i2c_client *client, u16 reg,
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

static int scc86hai_get_reso_dist(const struct scc86hai_mode *mode,
				  struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct scc86hai_mode *
scc86hai_find_best_fit(struct scc86hai *scc86hai,
		       struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = scc86hai_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		} else if (dist == cur_best_fit_dist &&
			   framefmt->code == supported_modes[i].bus_fmt) {
			cur_best_fit = i;
			break;
		}
	}
	dev_dbg(&scc86hai->client->dev, "%s: cur_best_fit(%d)", __func__,
		 cur_best_fit);

	return &supported_modes[cur_best_fit];
}

static void scc86hai_change_mode(struct scc86hai *scc86hai,
				 const struct scc86hai_mode *mode)
{
	scc86hai->cur_mode = mode;
	scc86hai->cur_vts = scc86hai->cur_mode->vts_def;
	dev_dbg(&scc86hai->client->dev, "set fmt: cur_mode: %dx%d, hdr: %d\n",
		 mode->width, mode->height, mode->hdr_mode);
}

static u64 scc86hai_calc_pixel_rate(const struct scc86hai_mode *mode)
{
	return (u64)link_freq_items[mode->mipi_freq_idx] * 2 *
	       SCC86HAI_4LANES / mode->bpp;
}

static void scc86hai_update_mode_ctrls(struct scc86hai *scc86hai,
				       const struct scc86hai_mode *mode)
{
	s64 h_blank, vblank_def, max;

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(scc86hai->hblank, h_blank, h_blank, 1,
				 h_blank);
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(scc86hai->vblank, vblank_def,
				 SCC86HAI_VTS_MAX - mode->height, 1,
				 vblank_def);
	__v4l2_ctrl_s_ctrl(scc86hai->link_freq, mode->mipi_freq_idx);
	__v4l2_ctrl_s_ctrl_int64(scc86hai->pixel_rate,
				 scc86hai_calc_pixel_rate(mode));
	scc86hai->cur_fps = mode->max_fps;
	scc86hai->cur_vts = mode->vts_def;

	max = mode->vts_def - 8;
	if (scc86hai->exposure)
		__v4l2_ctrl_modify_range(scc86hai->exposure,
					 scc86hai->exposure->minimum, max,
					 scc86hai->exposure->step,
					 scc86hai->exposure->default_value);
}

static int scc86hai_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *fmt)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	const struct scc86hai_mode *mode;

	mutex_lock(&scc86hai->mutex);

	mode = scc86hai_find_best_fit(scc86hai, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) =
			fmt->format;
#else
		mutex_unlock(&scc86hai->mutex);
		return -ENOTTY;
#endif
	} else {
		if (scc86hai->streaming) {
			mutex_unlock(&scc86hai->mutex);
			return -EBUSY;
		}
		scc86hai_change_mode(scc86hai, mode);
		scc86hai_update_mode_ctrls(scc86hai, mode);
	}

	mutex_unlock(&scc86hai->mutex);

	return 0;
}

static int scc86hai_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *fmt)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	const struct scc86hai_mode *mode = scc86hai->cur_mode;

	mutex_lock(&scc86hai->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format =
			*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
#else
		mutex_unlock(&scc86hai->mutex);
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
	mutex_unlock(&scc86hai->mutex);

	return 0;
}

static int scc86hai_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(bus_code))
		return -EINVAL;
	code->code = bus_code[code->index];

	return 0;
}

static int scc86hai_enum_frame_sizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);

	if (fse->index >= scc86hai->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int scc86hai_enable_test_pattern(struct scc86hai *scc86hai, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = scc86hai_read_reg(scc86hai->client, SCC86HAI_REG_TEST_PATTERN,
				SCC86HAI_REG_VALUE_08BIT, &val);
	if (ret)
		return ret;
	if (pattern)
		val |= SCC86HAI_TEST_PATTERN_ENABLE;
	else
		val &= ~SCC86HAI_TEST_PATTERN_ENABLE;
	return scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_TEST_PATTERN,
				  SCC86HAI_REG_VALUE_08BIT, val);
}

static int scc86hai_g_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	const struct scc86hai_mode *mode = scc86hai->cur_mode;

	if (scc86hai->streaming)
		fi->interval = scc86hai->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static const struct scc86hai_mode *scc86hai_find_mode(struct scc86hai *scc86hai,
						      int fps)
{
	const struct scc86hai_mode *mode = NULL;
	const struct scc86hai_mode *match = NULL;
	int cur_fps = 0;
	int i = 0;

	for (i = 0; i < scc86hai->cfg_num; i++) {
		mode = &supported_modes[i];
		if (mode->width == scc86hai->cur_mode->width &&
		    mode->height == scc86hai->cur_mode->height &&
		    mode->hdr_mode == scc86hai->cur_mode->hdr_mode &&
		    mode->bus_fmt == scc86hai->cur_mode->bus_fmt) {
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

static int scc86hai_s_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	const struct scc86hai_mode *mode = NULL;
	struct v4l2_fract *fract = &fi->interval;
	int fps, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	if (fract->numerator == 0) {
		v4l2_err(sd, "error param, check interval param\n");
		return -EINVAL;
	}

	mutex_lock(&scc86hai->mutex);

	if (scc86hai->streaming) {
		ret = -EBUSY;
		goto unlock;
	}

	fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
	mode = scc86hai_find_mode(scc86hai, fps);
	if (mode == NULL) {
		v4l2_err(sd, "couldn't match fi\n");
		ret = -EINVAL;
		goto unlock;
	}

	scc86hai->cur_mode = mode;
	scc86hai_update_mode_ctrls(scc86hai, mode);

unlock:
	mutex_unlock(&scc86hai->mutex);
	return ret;
}

static int scc86hai_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				  struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->bus.mipi_csi2.num_data_lanes = SCC86HAI_4LANES;

	return 0;
}

static void scc86hai_get_module_inf(struct scc86hai *scc86hai,
				    struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SCC86HAI_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, scc86hai->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, scc86hai->len_name, sizeof(inf->base.lens));
}

static int scc86hai_get_channel_info(struct scc86hai *scc86hai,
				     struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = scc86hai->cur_mode->vc[ch_info->index];
	ch_info->width = scc86hai->cur_mode->width;
	ch_info->height = scc86hai->cur_mode->height;
	ch_info->bus_fmt = scc86hai->cur_mode->bus_fmt;
	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
/*
 * Quad-bayer 5120x3072 full array is center-cropped to 5120x2880 (16:9)
 * for ISP input. Width/height must stay 16/8 aligned (see CROP_START).
 */
#define SCC86HAI_QUAD_BAYER_CROP_WIDTH 5120
#define SCC86HAI_QUAD_BAYER_CROP_HEIGHT 2880
static int scc86hai_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		if (scc86hai->cur_mode->bayer_mode == RKMODULE_QUARD_BAYER) {
			sel->r.left = CROP_START(scc86hai->cur_mode->width,
						 SCC86HAI_QUAD_BAYER_CROP_WIDTH);
			sel->r.width = SCC86HAI_QUAD_BAYER_CROP_WIDTH;
			sel->r.top = CROP_START(scc86hai->cur_mode->height,
						SCC86HAI_QUAD_BAYER_CROP_HEIGHT);
			sel->r.height = SCC86HAI_QUAD_BAYER_CROP_HEIGHT;
		} else {
			sel->r.left = 0;
			sel->r.width = scc86hai->cur_mode->width;
			sel->r.top = 0;
			sel->r.height = scc86hai->cur_mode->height;
		}
		return 0;
	}
	return -EINVAL;
}

static long scc86hai_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	u32 i, h, w, stream;
	int cur_best_fit = -1;
	int cur_best_fit_dist = -1;
	int cur_dist, cur_fps, dst_fps;
	u32 *bayer_mode;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		/* HDR not supported yet */
		break;

	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		mutex_lock(&scc86hai->mutex);
		if (hdr_cfg->hdr_mode == scc86hai->cur_mode->hdr_mode) {
			mutex_unlock(&scc86hai->mutex);
			return 0;
		}
		if (scc86hai->streaming) {
			mutex_unlock(&scc86hai->mutex);
			return -EBUSY;
		}
		w = scc86hai->cur_mode->width;
		h = scc86hai->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(
			scc86hai->cur_mode->max_fps.denominator,
			scc86hai->cur_mode->max_fps.numerator);
		for (i = 0; i < scc86hai->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr_cfg->hdr_mode &&
			    supported_modes[i].bus_fmt ==
				    scc86hai->cur_mode->bus_fmt) {
				cur_fps = DIV_ROUND_CLOSEST(
					supported_modes[i].max_fps.denominator,
					supported_modes[i].max_fps.numerator);
				cur_dist = abs(cur_fps - dst_fps);
				if (cur_best_fit_dist == -1 ||
				    cur_dist < cur_best_fit_dist) {
					cur_best_fit_dist = cur_dist;
					cur_best_fit = i;
				} else if (cur_dist == cur_best_fit_dist) {
					cur_best_fit = i;
					break;
				}
			}
		}
		if (cur_best_fit == -1) {
			dev_err(&scc86hai->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			scc86hai_change_mode(scc86hai,
					     &supported_modes[cur_best_fit]);
			scc86hai_update_mode_ctrls(
				scc86hai, &supported_modes[cur_best_fit]);
			dev_dbg(&scc86hai->client->dev, "sensor mode: %d\n",
				 supported_modes[cur_best_fit].hdr_mode);
		}
		mutex_unlock(&scc86hai->mutex);
		break;

	case RKMODULE_GET_MODULE_INFO:
		scc86hai_get_module_inf(scc86hai, (struct rkmodule_inf *)arg);
		break;

	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = scc86hai->cur_mode->hdr_mode;
		break;

	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		dev_dbg(&scc86hai->client->dev, "stream: %d\n", stream);

		if (stream) {
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_MIPI_CLK,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_MIPI_CLK_NORMAL);
			if (ret)
				break;
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_MIPI_DATA,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_MIPI_DATA_NORMAL);
			if (ret)
				break;
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_LOW_POWER,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_LOW_POWER_NORMAL);
			if (ret)
				break;
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_CTRL_MODE,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_MODE_STREAMING);
		} else {
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_MIPI_DATA,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_MIPI_DATA_HIZ);
			if (ret)
				break;
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_MIPI_CLK,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_MIPI_CLK_HIZ);
			if (ret)
				break;
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_CTRL_MODE,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_MODE_SW_STANDBY);
			if (ret)
				break;
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_REG_LOW_POWER,
						 SCC86HAI_REG_VALUE_08BIT,
						 SCC86HAI_LOW_POWER_ENABLE);
		}
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = scc86hai_get_channel_info(scc86hai, ch_info);
		break;
	case RKMODULE_GET_BAYER_MODE:
		bayer_mode = (u32 *)arg;
		*bayer_mode = scc86hai->cur_mode->bayer_mode;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long scc86hai_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd,
				    unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	struct rkmodule_channel_info *ch_info;
	long ret;
	u32 stream;
	u32 bayer_mode = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = scc86hai_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree(inf);
				return -EFAULT;
			}
		}
		kfree(inf);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = scc86hai_ioctl(sd, cmd, hdr);
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
		ret = scc86hai_ioctl(sd, cmd, hdr);
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
		ret = scc86hai_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = scc86hai_ioctl(sd, cmd, &stream);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(ch_info, up, sizeof(*ch_info))) {
			kfree(ch_info);
			return -EFAULT;
		}

		ret = scc86hai_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	case RKMODULE_GET_BAYER_MODE:
		ret = scc86hai_ioctl(sd, cmd, &bayer_mode);
		if (!ret) {
			ret = copy_to_user(up, &bayer_mode, sizeof(bayer_mode));
			if (ret)
				ret = -EFAULT;
		}
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __scc86hai_start_stream(struct scc86hai *scc86hai)
{
	int ret;

	dev_dbg(&scc86hai->client->dev, "%dx%d@%d, mode %d, vts 0x%x\n",
		 scc86hai->cur_mode->width, scc86hai->cur_mode->height,
		 scc86hai->cur_fps.denominator / scc86hai->cur_fps.numerator,
		 scc86hai->cur_mode->hdr_mode, scc86hai->cur_vts);

	if (!scc86hai->is_thunderboot) {
		ret = scc86hai_write_array(scc86hai->client,
					   scc86hai->cur_mode->reg_list);
		if (ret)
			return ret;

		ret = __v4l2_ctrl_handler_setup(&scc86hai->ctrl_handler);
		if (ret)
			return ret;
	}
	return scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_CTRL_MODE,
				  SCC86HAI_REG_VALUE_08BIT,
				  SCC86HAI_MODE_STREAMING);
}

static int __scc86hai_stop_stream(struct scc86hai *scc86hai)
{
	if (scc86hai->is_thunderboot)
		scc86hai->is_first_streamoff = true;
	return scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_CTRL_MODE,
				  SCC86HAI_REG_VALUE_08BIT,
				  SCC86HAI_MODE_SW_STANDBY);
}

static int scc86hai_s_stream(struct v4l2_subdev *sd, int on)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	struct i2c_client *client = scc86hai->client;
	int ret = 0;

	dev_info(&scc86hai->client->dev,
		 "s_stream: %d. %dx%d, hdr: %d, bpp: %d\n", on,
		 scc86hai->cur_mode->width, scc86hai->cur_mode->height,
		 scc86hai->cur_mode->hdr_mode, scc86hai->cur_mode->bpp);

	mutex_lock(&scc86hai->mutex);
	on = !!on;
	if (on == scc86hai->streaming)
		goto unlock_and_return;

	if (on) {
		if (scc86hai->is_thunderboot &&
		    rkisp_tb_get_state() == RKISP_TB_NG) {
			scc86hai->is_thunderboot = false;
			/*
			 * Probe held usage_count=1 with thunderboot partial
			 * power-on. Drop via runtime PM so suspend/resume does
			 * a balanced full power cycle; do not call __power_on
			 * or pm_runtime_set_active here.
			 */
			pm_runtime_put_sync(&client->dev);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __scc86hai_start_stream(scc86hai);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		ret = __scc86hai_stop_stream(scc86hai);
		if (ret) {
			v4l2_err(sd, "stop stream failed\n");
			goto unlock_and_return;
		}
		pm_runtime_put(&client->dev);
	}

	scc86hai->streaming = on;

unlock_and_return:
	mutex_unlock(&scc86hai->mutex);
	return ret;
}

static int scc86hai_s_power(struct v4l2_subdev *sd, int on)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	struct i2c_client *client = scc86hai->client;
	int ret = 0;

	mutex_lock(&scc86hai->mutex);

	/* If the power state is not modified - no work to do. */
	if (scc86hai->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!scc86hai->is_thunderboot) {
			ret = scc86hai_write_reg(scc86hai->client,
						 SCC86HAI_SOFTWARE_RESET_REG,
						 SCC86HAI_REG_VALUE_08BIT,
						 0x01);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
			udelay(100);
		}

		scc86hai->power_on = true;
		/* pm_runtime_get_sync() may return 1 if already active */
		ret = 0;
	} else {
		pm_runtime_put(&client->dev);
		scc86hai->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&scc86hai->mutex);

	return ret;
}

static int __scc86hai_power_on(struct scc86hai *scc86hai)
{
	int ret;
	struct device *dev = &scc86hai->client->dev;

	if (!IS_ERR_OR_NULL(scc86hai->pins_default)) {
		ret = pinctrl_select_state(scc86hai->pinctrl,
					   scc86hai->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(scc86hai->xvclk, scc86hai->cur_mode->xvclk_freq);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate 27MHz\n");
	if (clk_get_rate(scc86hai->xvclk) != scc86hai->cur_mode->xvclk_freq)
		dev_warn(dev, "xvclk mismatched\n");
	ret = clk_prepare_enable(scc86hai->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (scc86hai->cam_sw_info)
		cam_sw_regulator_bulk_init(scc86hai->cam_sw_info,
					   SCC86HAI_NUM_SUPPLIES, scc86hai->supplies);

	if (scc86hai->is_thunderboot)
		return 0;

	if (!IS_ERR(scc86hai->pwdn_gpio))
		gpiod_set_value_cansleep(scc86hai->pwdn_gpio, 1);

	usleep_range(4000, 6000);
	if (!IS_ERR(scc86hai->reset_gpio))
		gpiod_set_value_cansleep(scc86hai->reset_gpio, 1);

	usleep_range(4000, 6000);

	ret = regulator_bulk_enable(SCC86HAI_NUM_SUPPLIES, scc86hai->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto err_power;
	}

	if (!IS_ERR(scc86hai->reset_gpio))
		gpiod_set_value_cansleep(scc86hai->reset_gpio, 0);

	usleep_range(24000, 26000);

	return 0;

err_power:
	if (!IS_ERR(scc86hai->reset_gpio))
		gpiod_direction_output(scc86hai->reset_gpio, 1);
	if (!IS_ERR(scc86hai->pwdn_gpio))
		gpiod_direction_output(scc86hai->pwdn_gpio, 0);
	clk_disable_unprepare(scc86hai->xvclk);

	return ret;
}

static void __scc86hai_power_off(struct scc86hai *scc86hai)
{
	int ret;
	struct device *dev = &scc86hai->client->dev;

	clk_disable_unprepare(scc86hai->xvclk);
	if (scc86hai->is_thunderboot) {
		if (scc86hai->is_first_streamoff) {
			scc86hai->is_thunderboot = false;
			scc86hai->is_first_streamoff = false;
		} else {
			return;
		}
	}
	if (!IS_ERR(scc86hai->reset_gpio))
		gpiod_direction_output(scc86hai->reset_gpio, 1);

	if (!IS_ERR_OR_NULL(scc86hai->pins_sleep)) {
		ret = pinctrl_select_state(scc86hai->pinctrl,
					   scc86hai->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(scc86hai->pwdn_gpio))
		gpiod_direction_output(scc86hai->pwdn_gpio, 0);
	regulator_bulk_disable(SCC86HAI_NUM_SUPPLIES, scc86hai->supplies);
}

#if IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP)
static int scc86hai_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct scc86hai *scc86hai = to_scc86hai(sd);

	if (pm_runtime_suspended(dev))
		return 0;

	cam_sw_prepare_wakeup(scc86hai->cam_sw_info, dev);

	usleep_range(4000, 5000);
	cam_sw_write_array(scc86hai->cam_sw_info);

	if (__v4l2_ctrl_handler_setup(&scc86hai->ctrl_handler))
		dev_err(dev, "__v4l2_ctrl_handler_setup fail!\n");

	return 0;
}

static int scc86hai_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct scc86hai *scc86hai = to_scc86hai(sd);

	if (pm_runtime_suspended(dev))
		return 0;

	cam_sw_write_array_cb_init(scc86hai->cam_sw_info, client,
				   (void *)scc86hai->cur_mode->reg_list,
				   (sensor_write_array)scc86hai_write_array);
	cam_sw_prepare_sleep(scc86hai->cam_sw_info);

	return 0;
}
#else
#define scc86hai_resume NULL
#define scc86hai_suspend NULL
#endif

static int scc86hai_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct scc86hai *scc86hai = to_scc86hai(sd);

	return __scc86hai_power_on(scc86hai);
}

static int scc86hai_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct scc86hai *scc86hai = to_scc86hai(sd);

	__scc86hai_power_off(scc86hai);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int scc86hai_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);
	const struct scc86hai_mode *def_mode = &supported_modes[0];

	mutex_lock(&scc86hai->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&scc86hai->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int
scc86hai_enum_frame_interval(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_frame_interval_enum *fie)
{
	struct scc86hai *scc86hai = to_scc86hai(sd);

	if (fie->index >= scc86hai->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops scc86hai_pm_ops = { SET_RUNTIME_PM_OPS(
	scc86hai_runtime_suspend, scc86hai_runtime_resume,
	NULL) SET_LATE_SYSTEM_SLEEP_PM_OPS(scc86hai_suspend, scc86hai_resume) };

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops scc86hai_internal_ops = {
	.open = scc86hai_open,
};
#endif

static const struct v4l2_subdev_core_ops scc86hai_core_ops = {
	.s_power = scc86hai_s_power,
	.ioctl = scc86hai_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = scc86hai_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops scc86hai_video_ops = {
	.s_stream = scc86hai_s_stream,
	.g_frame_interval = scc86hai_g_frame_interval,
	.s_frame_interval = scc86hai_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops scc86hai_pad_ops = {
	.enum_mbus_code = scc86hai_enum_mbus_code,
	.enum_frame_size = scc86hai_enum_frame_sizes,
	.enum_frame_interval = scc86hai_enum_frame_interval,
	.get_fmt = scc86hai_get_fmt,
	.set_fmt = scc86hai_set_fmt,
	.get_mbus_config = scc86hai_g_mbus_config,
	.get_selection = scc86hai_get_selection,
};

static const struct v4l2_subdev_ops scc86hai_subdev_ops = {
	.core = &scc86hai_core_ops,
	.video = &scc86hai_video_ops,
	.pad = &scc86hai_pad_ops,
};

static void scc86hai_modify_fps_info(struct scc86hai *scc86hai)
{
	const struct scc86hai_mode *mode = scc86hai->cur_mode;
	u64 new_denominator = (u64)mode->max_fps.denominator * mode->vts_def;

	/* fps = denominator / numerator; larger VTS -> lower fps */
	scc86hai->cur_fps.denominator = div_u64(new_denominator, scc86hai->cur_vts);
}

static int scc86hai_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct scc86hai *scc86hai =
		container_of(ctrl->handler, struct scc86hai, ctrl_handler);
	struct i2c_client *client = scc86hai->client;
	s64 max;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = scc86hai->cur_mode->height + ctrl->val - 8;
		__v4l2_ctrl_modify_range(scc86hai->exposure,
					 scc86hai->exposure->minimum, max,
					 scc86hai->exposure->step,
					 scc86hai->exposure->default_value);
		break;
	}

	if (pm_runtime_get_if_in_use(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* Set exposure: 24-bit value split across 4 registers */
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_EXP_0,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_EXP_0(ctrl->val));
		if (ret)
			break;
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_EXP_1,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_EXP_1(ctrl->val));
		if (ret)
			break;
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_EXP_2,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_EXP_2(ctrl->val));
		if (ret)
			break;
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_EXP_3,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_EXP_3(ctrl->val));

		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		/* Set analogue gain: 16-bit value split across 2 registers */
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_AGAIN_H,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_AGAIN_H(ctrl->val));
		if (ret)
			break;
		ret = scc86hai_write_reg(scc86hai->client,
					 SCC86HAI_REG_AGAIN_L,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_AGAIN_L(ctrl->val));

		dev_dbg(&client->dev, "set analogue gain 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		/* Set VTS: 22-bit value split across 3 registers */
		val = ctrl->val + scc86hai->cur_mode->height;
		/* High 6 bits [21:16] */
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_VTS_H,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_VTS_H(val));
		if (ret)
			break;
		/* Middle 8 bits [15:8] */
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_VTS_M,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_VTS_M(val));
		if (ret)
			break;
		/* Low 8 bits [7:0] */
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_REG_VTS_L,
					 SCC86HAI_REG_VALUE_08BIT,
					 SCC86HAI_FETCH_VTS_L(val));
		if (ret)
			break;
		scc86hai->cur_vts = val;
		scc86hai_modify_fps_info(scc86hai);
		dev_dbg(&client->dev, "set vblank 0x%x, vts 0x%x\n", ctrl->val,
			val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = scc86hai_enable_test_pattern(scc86hai, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = scc86hai_read_reg(scc86hai->client, SCC86HAI_FLIP_REG,
					SCC86HAI_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SCC86HAI_MIRROR_MASK;
		else
			val &= ~SCC86HAI_MIRROR_MASK;
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_FLIP_REG,
					 SCC86HAI_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = scc86hai_read_reg(scc86hai->client, SCC86HAI_FLIP_REG,
					SCC86HAI_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SCC86HAI_FLIP_MASK;
		else
			val &= ~SCC86HAI_FLIP_MASK;
		ret = scc86hai_write_reg(scc86hai->client, SCC86HAI_FLIP_REG,
					 SCC86HAI_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops scc86hai_ctrl_ops = {
	.s_ctrl = scc86hai_set_ctrl,
};

static int scc86hai_initialize_controls(struct scc86hai *scc86hai)
{
	const struct scc86hai_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u64 pixel_rate = 0;
	s64 h_blank;
	int ret;

	handler = &scc86hai->ctrl_handler;
	mode = scc86hai->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &scc86hai->mutex;

	scc86hai->link_freq = v4l2_ctrl_new_int_menu(
		handler, NULL, V4L2_CID_LINK_FREQ, 0, 0, link_freq_items);
	if (scc86hai->link_freq)
		v4l2_ctrl_s_ctrl(scc86hai->link_freq, mode->mipi_freq_idx);

	/* pixel_rate = link_freq * 2 * lanes / bpp */
	pixel_rate = scc86hai_calc_pixel_rate(mode);
	scc86hai->pixel_rate =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0,
				  SCC86HAI_MAX_PIXEL_RATE, 1, pixel_rate);

	h_blank = mode->hts_def - mode->width;
	scc86hai->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					     h_blank, h_blank, 1, h_blank);
	if (scc86hai->hblank)
		scc86hai->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	scc86hai->vblank = v4l2_ctrl_new_std(handler, &scc86hai_ctrl_ops,
					     V4L2_CID_VBLANK, vblank_def,
					     SCC86HAI_VTS_MAX - mode->height, 1,
					     vblank_def);

	exposure_max = mode->vts_def - 8;
	scc86hai->exposure = v4l2_ctrl_new_std(
		handler, &scc86hai_ctrl_ops, V4L2_CID_EXPOSURE,
		SCC86HAI_EXPOSURE_MIN, exposure_max, SCC86HAI_EXPOSURE_STEP,
		mode->exp_def);

	scc86hai->anal_a_gain = v4l2_ctrl_new_std(
		handler, &scc86hai_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
		SCC86HAI_GAIN_MIN, SCC86HAI_GAIN_MAX, SCC86HAI_GAIN_STEP,
		SCC86HAI_GAIN_DEFAULT);

	scc86hai->test_pattern = v4l2_ctrl_new_std_menu_items(
		handler, &scc86hai_ctrl_ops, V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(scc86hai_test_pattern_menu) - 1, 0, 0,
		scc86hai_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &scc86hai_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1,
			  0);
	v4l2_ctrl_new_std(handler, &scc86hai_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1,
			  0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&scc86hai->client->dev, "Failed to init controls(%d)\n",
			ret);
		goto err_free_handler;
	}

	scc86hai->subdev.ctrl_handler = handler;
	scc86hai->cur_fps = mode->max_fps;
	scc86hai->cur_vts = mode->vts_def;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int scc86hai_check_sensor_id(struct scc86hai *scc86hai,
				    struct i2c_client *client)
{
	struct device *dev = &scc86hai->client->dev;
	u32 id = 0;
	int ret;

	if (scc86hai->is_thunderboot) {
		dev_info(dev,
			 "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}
	ret = scc86hai_read_reg(client, SCC86HAI_REG_CHIP_ID,
				SCC86HAI_REG_VALUE_16BIT, &id);
	if (ret) {
		dev_err(dev, "Failed to read sensor id, ret(%d)\n", ret);
		return ret;
	}
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(0x%04x)\n", id);
		return -ENODEV;
	}

	dev_info(dev, "Detected scc86hai id(0x%04x)\n", CHIP_ID);

	return 0;
}

static int scc86hai_configure_regulators(struct scc86hai *scc86hai)
{
	unsigned int i;

	for (i = 0; i < SCC86HAI_NUM_SUPPLIES; i++)
		scc86hai->supplies[i].supply = scc86hai_supply_names[i];

	return devm_regulator_bulk_get(&scc86hai->client->dev,
				       SCC86HAI_NUM_SUPPLIES,
				       scc86hai->supplies);
}

static int scc86hai_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct scc86hai *scc86hai;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x", DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8, DRIVER_VERSION & 0x00ff);

	scc86hai = devm_kzalloc(dev, sizeof(*scc86hai), GFP_KERNEL);
	if (!scc86hai)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &scc86hai->module_index);
	if (ret) {
		dev_err(dev, "could not get module index!\n");
		return ret;
	}
	ret = of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				      &scc86hai->module_facing);
	if (ret) {
		dev_err(dev, "could not get module facing!\n");
		return ret;
	}
	ret = of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				      &scc86hai->module_name);
	if (ret) {
		dev_err(dev, "could not get module name!\n");
		return ret;
	}
	ret = of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				      &scc86hai->len_name);
	if (ret) {
		dev_err(dev, "could not get lens name!\n");
		return ret;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, "Get hdr mode failed! no hdr default\n");
	}

	scc86hai->is_thunderboot =
		IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	scc86hai->client = client;
	scc86hai->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < scc86hai->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			scc86hai->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (!scc86hai->cur_mode) {
		dev_err(dev, "not find mode for hdr_mode %d\n", hdr_mode);
		return -EINVAL;
	}

	scc86hai->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(scc86hai->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	scc86hai->reset_gpio = devm_gpiod_get(
		dev, "reset",
		scc86hai->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(scc86hai->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	scc86hai->pwdn_gpio = devm_gpiod_get(
		dev, "pwdn",
		scc86hai->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(scc86hai->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn_gpio\n");

	scc86hai->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(scc86hai->pinctrl)) {
		scc86hai->pins_default = pinctrl_lookup_state(
			scc86hai->pinctrl, OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(scc86hai->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		scc86hai->pins_sleep = pinctrl_lookup_state(
			scc86hai->pinctrl, OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(scc86hai->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}

	ret = scc86hai_configure_regulators(scc86hai);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&scc86hai->mutex);

	sd = &scc86hai->subdev;
	v4l2_i2c_subdev_init(sd, client, &scc86hai_subdev_ops);
	ret = scc86hai_initialize_controls(scc86hai);
	if (ret)
		goto err_destroy_mutex;

	ret = __scc86hai_power_on(scc86hai);
	if (ret)
		goto err_free_handler;

	ret = scc86hai_check_sensor_id(scc86hai, client);
	if (ret)
		goto err_power_off;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &scc86hai_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	scc86hai->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &scc86hai->pad);
	if (ret < 0)
		goto err_power_off;
#endif
	if (!scc86hai->cam_sw_info) {
		scc86hai->cam_sw_info = cam_sw_init();
		if (IS_REACHABLE(CONFIG_VIDEO_CAM_SLEEP_WAKEUP) &&
		    !scc86hai->cam_sw_info) {
			dev_err(dev, "Failed to alloc cam_sw_info\n");
			ret = -ENOMEM;
#if defined(CONFIG_MEDIA_CONTROLLER)
			media_entity_cleanup(&sd->entity);
#endif
			goto err_power_off;
		}
		cam_sw_clk_init(scc86hai->cam_sw_info, scc86hai->xvclk,
				scc86hai->cur_mode->xvclk_freq);
		/* reset is active-high: run=0, hold-reset=1 */
		cam_sw_reset_pin_init(scc86hai->cam_sw_info,
				      scc86hai->reset_gpio, 1);
		cam_sw_pwdn_pin_init(scc86hai->cam_sw_info, scc86hai->pwdn_gpio,
				     1);
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(scc86hai->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 scc86hai->module_index, facing, SCC86HAI_NAME,
		 dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (scc86hai->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
	cam_sw_deinit(scc86hai->cam_sw_info);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__scc86hai_power_off(scc86hai);
err_free_handler:
	v4l2_ctrl_handler_free(&scc86hai->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&scc86hai->mutex);

	return ret;
}

static void scc86hai_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct scc86hai *scc86hai = to_scc86hai(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&scc86hai->ctrl_handler);
	mutex_destroy(&scc86hai->mutex);

	cam_sw_deinit(scc86hai->cam_sw_info);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__scc86hai_power_off(scc86hai);
	pm_runtime_set_suspended(&client->dev);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id scc86hai_of_match[] = {
	{ .compatible = "smartsens,scc86hai" },
	{},
};
MODULE_DEVICE_TABLE(of, scc86hai_of_match);
#endif

static const struct i2c_device_id scc86hai_match_id[] = {
	{ "smartsens,scc86hai", 0 },
	{},
};

static struct i2c_driver scc86hai_i2c_driver = {
	.driver = {
		.name = SCC86HAI_NAME,
		.pm = &scc86hai_pm_ops,
		.of_match_table = of_match_ptr(scc86hai_of_match),
	},
	.probe		= scc86hai_probe,
	.remove		= scc86hai_remove,
	.id_table	= scc86hai_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&scc86hai_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&scc86hai_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens,scc86hai sensor driver");
MODULE_AUTHOR("Rockchip Electronics Co., Ltd.");
MODULE_LICENSE("GPL");
