// SPDX-License-Identifier: GPL-2.0
/*
 * mis40h1 driver
 *
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 * V0.0X01.0X02 add support dcg mode
 */

// #define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
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
#include <media/v4l2-fwnode.h>
#include <linux/pinctrl/consumer.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

#define MIS40H1_BITS_PER_SAMPLE		12
#define MIS40H1_LINK_FREQ_476M		476000000    //Mhz
#define MIS40H1_LINK_FREQ_474M		474000000    //Mhz

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define PIXEL_RATE_WITH_476M_12BIT_2L	(MIS40H1_LINK_FREQ_476M / MIS40H1_BITS_PER_SAMPLE * 2 * 2)
#define PIXEL_RATE_WITH_474M_12BIT_4L	(MIS40H1_LINK_FREQ_474M / MIS40H1_BITS_PER_SAMPLE * 2 * 4)

#define MIS40H1_XVCLK_FREQ		24000000

#define CHIP_ID				0x40d1
#define MIS40H1_REG_CHIP_ID		0x3000

#define MIS40H1_REG_CTRL_MODE		0x3006
#define MIS40H1_MODE_SW_STANDBY		BIT(1)
#define MIS40H1_MODE_STREAMING		0x0

#define MIS40H1_REG_STREAM_CTRL		0x300c
#define MIS40H1_STREAM_OFF		0x00
#define MIS40H1_STREAM_ON		0x81

#define MIS40H1_REG_EXPOSURE_H		0x3100
#define MIS40H1_REG_EXPOSURE_L		0x3101
#define	MIS40H1_EXPOSURE_MIN		1
#define	MIS40H1_EXPOSURE_STEP		1
#define MIS40H1_FETCH_EXP_H(VAL)	(((VAL) >> 8) & 0xFF)
#define MIS40H1_FETCH_EXP_L(VAL)	((VAL) & 0xFF)

#define MIS40H1_REG_DIG_GAIN_H		0x420b
#define MIS40H1_REG_DIG_GAIN_L		0x420c
#define MIS40H1_REG_ANA_GAIN_H		0x3106
#define MIS40H1_REG_ANA_GAIN_L		0x3107
#define MIS40H1_ONCE_GAIN_STEP		0x80
#define MIS40H1_LCG_TO_HCG		0x5a
#define MIS40H1_GAIN_MIN		MIS40H1_ONCE_GAIN_STEP
#define MIS40H1_AGAIN_MAX		(MIS40H1_ONCE_GAIN_STEP * MIS40H1_LCG_TO_HCG * 32 / 10) /* just again */
#define MIS40H1_GAIN_MAX		(MIS40H1_AGAIN_MAX * 2)
#define MIS40H1_GAIN_STEP		1
#define MIS40H1_GAIN_DEFAULT		MIS40H1_ONCE_GAIN_STEP
#define MIS40H1_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define MIS40H1_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define MIS40H1_REG_GAIN_EXP_VALID	0x3008
#define MIS40H1_REG_GAIN_EXP_VALID_VAL	BIT(0)

#define MIS40H1_VTS_MAX			0xffff
#define MIS40H1_REG_VTS_H		0x310e
#define MIS40H1_REG_VTS_L		0x310f
#define MIS40H1_REG_HTS_H		0x3110
#define MIS40H1_REG_HTS_L		0x3111

#define MIS40H1_FLIP_MIRROR_REG		0x3007
#define MIRROR_BIT_MASK			BIT(0)
#define FLIP_BIT_MASK			BIT(1)
#define MIS40H1_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x01 : VAL & 0xfe)
#define MIS40H1_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x10 : VAL & 0xfd)

#define MIS40H1_REG_TEST_PATTERN	0x4002
#define MIS40H1_TEST_PATTERN_BIT_MASK	BIT(0)
#define MIS40H1_TEST_PATTERN_GRAY_DIR	BIT(2) //ladder_mode, 1:Up-down; 0:Left-Right
#define MIS40H1_TEST_PATTERN_MODE	BIT(1) //TPG mode, 1:Gray;0:Color bar
#define MIS40H1_TEST_PATTERN_ROLLEN	BIT(3) //Rolling en,1:Enable; 0:Disable

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define MIS40H1_REG_VALUE_08BIT		1
#define MIS40H1_REG_VALUE_16BIT		2
#define MIS40H1_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define MIS40H1_NAME			"mis40h1"

static const char *const mis40h1_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define MIS40H1_NUM_SUPPLIES ARRAY_SIZE(mis40h1_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct mis40h1_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *global_reg_list;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 mclk;
	u32 link_freq_idx;
	u32 vc[PAD_MAX];
	u8 bpp;
	u32 lanes;
};

struct mis40h1 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[MIS40H1_NUM_SUPPLIES];

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
	struct v4l2_fract	cur_fps;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct mis40h1_mode *supported_modes;
	const struct mis40h1_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			standby_hw;
	u32			cur_vts;
	u8			dcgData;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	bool			is_standby;
	struct v4l2_fwnode_endpoint bus_cfg;
};

#define to_mis40h1(sd) container_of(sd, struct mis40h1, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval mis40h1_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 445.5Mbps, 2lane
 * 2688x1520, 12bits
 */
static const struct regval mis40h1_linear_12_2688x1520_30fps_2lane_regs[] = {
	//NO16_2307_2688X1520_AD11_RAW12_30FPS_LINEAR_4M_2lane_V1_1_20250114_xxxx .ini
	//Sensir revision:Mis40H1
	//Input clock frequency:24M
	//Image output size:2688x1520
	//Frame timing and frame rate: LINEAR 30Fps
	//BAYER PATTERN：GRBG
	//System clock frequency:158.67M
	//Output interface and data rate:MIPI 2Lane raw12 952Mbps
	//HTS = 3110/3111 =0xc80
	//VTS = 310e/310f =0x672
	//Tline = 20.32us
	{0x3006, 0x01},
	{REG_DELAY, 0x01},
	{0x3006, 0x02},
	//2688x1520
	{0x3113, 0x04},
	{0x3115, 0xf3},
	{0x3117, 0x04},
	{0x3119, 0x83},

	{0x3120, 0x08},
	{0x3123, 0x12},
	{0x3306, 0x25},
	{0x3605, 0xc5},
	{0x360d, 0xc5},
	{0x3615, 0x58},
	{0x361d, 0xc5},
	{0x3621, 0xc5},
	{0x3629, 0xc5},
	{0x362d, 0xb9},
	{0x363d, 0xF7},
	{0x363f, 0x7d},
	{0x3641, 0xB0},
	{0x365d, 0x08},
	{0x365f, 0xb9},
	{0x3661, 0xa2},
	{0x367f, 0xad},
	{0x3681, 0xa0},
	{0x368b, 0xad},
	{0x368d, 0xc5},
	{0x3692, 0x03},
	{0x3693, 0x39},
	{0x3695, 0xbf},
	{0x36b9, 0xb9},
	{0x3703, 0xbf},
	{0x3705, 0xc5},
	{0x3713, 0xdd},
	{0x371e, 0x66},
	{0x3720, 0x95},
	{0x3724, 0x72},
	{0x3728, 0x89},
	{0x3730, 0x42},
	{0x3740, 0x89},
	{0x375c, 0xA8},
	{0x3c03, 0x03},
	{0x3c39, 0x03},
	{0x4102, 0x13},
	{0x410e, 0x02},
	{0x4303, 0x13},
	{0x432e, 0x00},
	{0x432f, 0x80},
	{0x4330, 0x00},
	{0x4331, 0x80},
	{0x4332, 0x00},
	{0x4333, 0x80},
	{0x4334, 0x00},
	{0x4335, 0x80},
	{0x4402, 0x57},
	{0x390c, 0x1b},
	{0x390d, 0x1b},
	{0x390e, 0x1b},
	{0x390f, 0x1b},
	{0x3910, 0x1b},
	{0x3911, 0x1b},
	{0x3a0e, 0x0f},
	{0x4102, 0x13},
	{0x410e, 0x02},
	{0x410f, 0x39},
	{0x4303, 0x13},
	{0x4304, 0x39},
	{0x4309, 0x00},
	{0x367C, 0x01},
	{0x367D, 0xE8},
	{0x3a1e, 0x7f},
	{0x3a1f, 0x00},
	{0x3a20, 0xff},
	{0x3915, 0x00},
	{0x5309, 0x64}, //en color pattern
	{0x3031, 0x0c},
	{0x3008, 0x01},
	{0x3006, 0x00},
	{0x3c1a, 0x01},
	{REG_DELAY, 0x01},
	//{0x300c, 0x81},
	{REG_NULL, 0x00},
};

static const struct regval mis40h1_linear_12_2688x1520_60fps_4lane_regs[] = {
	//NO19_2307_2676X1528_AD11_RAW12_60FPS_LINEAR_4M_4lane_V1_1_20241223 - xx.ini
	//Sensir revision:Mis40H1
	//Input clock frequency:24M
	//Image output size:2676x1528
	//Frame timing and frame rate: DOL2 30Fps
	//BAYER PATTERN：GRBG
	//System clock frequency:316M
	//Output interface and data rate:MIPI 2Lane raw12 948Mbps
	//HTS = 3110/3111 =0xc80
	//VTS = 310e/310f =0x672
	//Tline = 20.24us

	{0x3006, 0x01},
	{REG_DELAY, 0x01},
	{0x3006, 0x02},	//soft power down
	{0x3b05, 0x3f},
	{0x310f, 0x72},	//configuration of image vts & hts
	{0x310e, 0x06},
	{0x3111, 0x80},
	{0x3110, 0x0c},

	{0x3113, 0x00},	// vts_st
	{0x3112, 0x00},
	{0x3115, 0xf7},	// vts_end
	{0x3114, 0x05},
	{0x3117, 0x04},	// hts_st
	{0x3116, 0x00},
	{0x3119, 0x83},	// hts_end
	{0x3118, 0x0a},

	//2688x1520
	{0x3113, 0x04},
	{0x3115, 0xf3},
	{0x3117, 0x04},
	{0x3119, 0x83},

	{0x3017, 0x00},	// binning mode
	{0x3017, 0x00},
	{0x3017, 0x00},
	{0x3017, 0x00},
	{0x3120, 0x08},
	{0x311b, 0xdc},	//RD0_ST_PNT[7:0]
	{0x311a, 0x03},	//RD0_ST_PNT[15:8]
	{0x311d, 0x19},	//RD1_ST_PNT[7:0]
	{0x311c, 0x00},	//RD1_ST_PNT[15:8]
	{0x311f, 0x32},	//RD2_ST_PNT[7:0]
	{0x311e, 0x00},	//RD0_ST_PNT[15:8]
	{0x3018, 0x40},	//AD 11BIT
	{0x3304, 0xed},	//PLL configuration
	{0x3305, 0xd2},
	{0x3305, 0x72},
	{0x3305, 0x72},
	{0x3306, 0x23},
	{0x3306, 0x21},
	{0x3307, 0x08},
	{0x3306, 0x21},
	{0x3308, 0xed},
	{0x3309, 0xb2},
	{0x3309, 0x72},
	{0x3309, 0x72},
	{0x330a, 0x04},
	{0x330a, 0x04},
	{0x330b, 0x08},
	{0x330a, 0x04},
	{0x330f, 0x18},
	{0x330e, 0x00},
	{0x3023, 0x2c},	//CH0_DT[7:0]
	{0x3c39, 0x0f},	// mipi physical channel
	{0x371c, 0x00},	// analog timing configuration
	{0x371a, 0x00},
	{0x370f, 0x01},
	{0x370e, 0x00},
	{0x3711, 0x0a},
	{0x3710, 0x00},
	{0x3627, 0x00},
	{0x3626, 0x00},
	{0x3629, 0xd1},
	{0x3628, 0x06},
	{0x3603, 0x72},
	{0x3602, 0x00},
	{0x3605, 0xd1},
	{0x3604, 0x06},
	{0x3607, 0xff},
	{0x3606, 0x1f},
	{0x3609, 0xff},
	{0x3608, 0x1f},
	{0x360b, 0x69},
	{0x360a, 0x00},
	{0x360d, 0xd1},
	{0x360c, 0x06},
	{0x360f, 0xff},
	{0x360e, 0x1f},
	{0x3611, 0xff},
	{0x3610, 0x1f},
	{0x3613, 0xa2},
	{0x3612, 0x02},
	{0x3615, 0x60},
	{0x3614, 0x03},
	{0x3617, 0xff},
	{0x3616, 0x1f},
	{0x3619, 0xff},
	{0x3618, 0x1f},
	{0x361b, 0x01},
	{0x361a, 0x00},
	{0x361d, 0xd1},
	{0x361c, 0x06},
	{0x361f, 0x01},
	{0x361e, 0x00},
	{0x3621, 0xd1},
	{0x3620, 0x06},
	{0x362b, 0x13},
	{0x362a, 0x00},
	{0x362d, 0xbe},
	{0x362c, 0x06},
	{0x3633, 0x4c},
	{0x3632, 0x00},
	{0x3635, 0x1d},
	{0x3634, 0x01},
	{0x3637, 0x4c},
	{0x3636, 0x00},
	{0x3639, 0x13},
	{0x3638, 0x01},
	{0x36bb, 0x0a},
	{0x36ba, 0x00},
	{0x36bd, 0x4c},
	{0x36bc, 0x01},
	{0x36bf, 0x4c},
	{0x36be, 0x01},
	{0x36c1, 0x5f},
	{0x36c0, 0x01},
	{0x36c3, 0xab},
	{0x36c2, 0x02},
	{0x36c5, 0xbe},
	{0x36c4, 0x02},
	{0x36c7, 0xff},
	{0x36c6, 0x1f},
	{0x36c9, 0xff},
	{0x36c8, 0x1f},
	{0x36cb, 0xff},
	{0x36ca, 0x1f},
	{0x36cd, 0xff},
	{0x36cc, 0x1f},
	{0x36cf, 0xff},
	{0x36ce, 0x1f},
	{0x36d1, 0xff},
	{0x36d0, 0x1f},
	{0x36d3, 0xff},
	{0x36d2, 0x1f},
	{0x36d5, 0xff},
	{0x36d4, 0x1f},
	{0x36d7, 0xff},
	{0x36d6, 0x1f},
	{0x36d9, 0xff},
	{0x36d8, 0x1f},
	{0x36db, 0xff},
	{0x36da, 0x1f},
	{0x36dd, 0xff},
	{0x36dc, 0x1f},
	{0x365b, 0x98},
	{0x365a, 0x01},
	{0x365d, 0x98},
	{0x365c, 0x02},
	{0x365f, 0xbe},
	{0x365e, 0x03},
	{0x3661, 0xbe},
	{0x3660, 0x06},
	{0x3663, 0xff},
	{0x3662, 0x1f},
	{0x3665, 0xff},
	{0x3664, 0x1f},
	{0x3667, 0xff},
	{0x3666, 0x1f},
	{0x3669, 0xff},
	{0x3668, 0x1f},
	{0x366b, 0xff},
	{0x366a, 0x1f},
	{0x366d, 0xff},
	{0x366c, 0x1f},
	{0x366f, 0xff},
	{0x366e, 0x1f},
	{0x3671, 0xff},
	{0x3670, 0x1f},
	{0x3673, 0xff},
	{0x3672, 0x1f},
	{0x3675, 0xff},
	{0x3674, 0x1f},
	{0x3677, 0xff},
	{0x3676, 0x1f},
	{0x3679, 0xff},
	{0x3678, 0x1f},
	{0x3703, 0xc8},
	{0x3702, 0x06},
	{0x3705, 0xd1},
	{0x3704, 0x06},
	{0x3707, 0x00},
	{0x3706, 0x00},
	{0x3709, 0xab},
	{0x3708, 0x00},
	{0x370b, 0x00},
	{0x370a, 0x00},
	{0x370d, 0xab},
	{0x370c, 0x00},
	{0x36e7, 0x26},
	{0x36e6, 0x00},
	{0x36e9, 0x72},
	{0x36e8, 0x00},
	{0x3683, 0x5f},
	{0x3682, 0x00},
	{0x3685, 0x85},
	{0x3684, 0x00},
	{0x3687, 0x85},
	{0x3686, 0x01},
	{0x3689, 0xab},
	{0x3688, 0x02},
	{0x368b, 0xab},
	{0x368a, 0x03},
	{0x368d, 0xd1},
	{0x368c, 0x06},
	{0x368f, 0x18},
	{0x368e, 0x02},
	{0x3691, 0xa2},
	{0x3690, 0x02},
	{0x3693, 0x3e},
	{0x3692, 0x04},
	{0x3695, 0xc8},
	{0x3694, 0x06},
	{0x3697, 0xff},
	{0x3696, 0x1f},
	{0x3699, 0xff},
	{0x3698, 0x1f},
	{0x369b, 0xff},
	{0x369a, 0x1f},
	{0x369d, 0xff},
	{0x369c, 0x1f},
	{0x369f, 0xff},
	{0x369e, 0x1f},
	{0x36a1, 0xff},
	{0x36a0, 0x1f},
	{0x36a3, 0xff},
	{0x36a2, 0x1f},
	{0x36a5, 0xff},
	{0x36a4, 0x1f},
	{0x36a7, 0xff},
	{0x36a6, 0x1f},
	{0x36a9, 0xff},
	{0x36a8, 0x1f},
	{0x36ab, 0xff},
	{0x36aa, 0x1f},
	{0x36ad, 0xff},
	{0x36ac, 0x1f},
	{0x36af, 0x39},
	{0x36ae, 0x00},
	{0x36b1, 0x85},
	{0x36b0, 0x01},
	{0x36b3, 0x43},
	{0x36b2, 0x00},
	{0x36b5, 0x98},
	{0x36b4, 0x01},
	{0x36b7, 0xab},
	{0x36b6, 0x02},
	{0x36b9, 0xbe},
	{0x36b8, 0x03},
	{0x362f, 0x0a},
	{0x362e, 0x00},
	{0x3631, 0xab},
	{0x3630, 0x02},
	{0x363b, 0x39},
	{0x363a, 0x01},
	{0x363d, 0x5f},
	{0x363c, 0x01},
	{0x363f, 0x60},
	{0x363e, 0x03},
	{0x3641, 0x85},
	{0x3640, 0x03},
	{0x3643, 0xff},
	{0x3642, 0x1f},
	{0x3645, 0xff},
	{0x3644, 0x1f},
	{0x3647, 0xff},
	{0x3646, 0x1f},
	{0x3649, 0xff},
	{0x3648, 0x1f},
	{0x364b, 0xff},
	{0x364a, 0x1f},
	{0x364d, 0xff},
	{0x364c, 0x1f},
	{0x364f, 0xff},
	{0x364e, 0x1f},
	{0x3651, 0xff},
	{0x3650, 0x1f},
	{0x3653, 0xff},
	{0x3652, 0x1f},
	{0x3655, 0xff},
	{0x3654, 0x1f},
	{0x3657, 0xff},
	{0x3656, 0x1f},
	{0x3659, 0xff},
	{0x3658, 0x1f},
	{0x367b, 0x85},
	{0x367a, 0x01},
	{0x367d, 0xa2},
	{0x367c, 0x02},
	{0x367f, 0xab},
	{0x367e, 0x03},
	{0x3681, 0xc8},
	{0x3680, 0x06},
	{0x36df, 0x00},
	{0x36de, 0x00},
	{0x36e1, 0xab},
	{0x36e0, 0x00},
	{0x36e3, 0x00},
	{0x36e2, 0x00},
	{0x36e5, 0xab},
	{0x36e4, 0x00},
	{0x3623, 0x00},
	{0x3622, 0x00},
	{0x3625, 0xab},
	{0x3624, 0x02},
	{0x3713, 0xf7},
	{0x3712, 0x06},
	{0x371e, 0x3a},
	{0x371d, 0x03},
	{0x3720, 0x85},
	{0x371f, 0x03},
	{0x3722, 0x13},
	{0x3721, 0x00},
	{0x3724, 0x4d},
	{0x3723, 0x03},
	{0x3726, 0x95},
	{0x3725, 0x02},
	{0x3728, 0x73},
	{0x3727, 0x03},
	{0x372a, 0x93},
	{0x3729, 0x02},
	{0x372c, 0x97},
	{0x372b, 0x02},
	{0x372e, 0x26},
	{0x372d, 0x00},
	{0x3730, 0x01},
	{0x372f, 0x06},
	{0x3732, 0x00},
	{0x3731, 0x00},
	{0x3734, 0x98},
	{0x3733, 0x01},
	{0x3736, 0x00},
	{0x3735, 0x00},
	{0x3738, 0xab},
	{0x3737, 0x00},
	{0x373a, 0x00},
	{0x3739, 0x00},
	{0x373c, 0xab},
	{0x373b, 0x00},
	{0x373e, 0x95},
	{0x373d, 0x02},
	{0x3740, 0x73},
	{0x373f, 0x03},
	{0x3742, 0xff},
	{0x3741, 0x1f},
	{0x3744, 0xff},
	{0x3743, 0x1f},
	{0x3746, 0xff},
	{0x3745, 0x1f},
	{0x3748, 0xff},
	{0x3747, 0x1f},
	{0x374a, 0xff},
	{0x3749, 0x1f},
	{0x374c, 0xff},
	{0x374b, 0x1f},
	{0x374e, 0xff},
	{0x374d, 0x1f},
	{0x3750, 0xff},
	{0x374f, 0x1f},
	{0x3752, 0xff},
	{0x3751, 0x1f},
	{0x3754, 0xff},
	{0x3753, 0x1f},
	{0x3756, 0xff},
	{0x3755, 0x1f},
	{0x3758, 0xff},
	{0x3757, 0x1f},
	{0x375a, 0xa2},
	{0x3759, 0x02},
	{0x375c, 0x73},
	{0x375b, 0x03},
	{0x3c03, 0x03},	//MIPI_CLK_PERIOD
	{0x3c02, 0x00},	//MIPI_CLK_PERIOD
	{0x3c04, 0x46},	//MIPI_CLK_PREPARE_CONST_MIN
	{0x3c1a, 0x00},	//MIPI_MANUAL_PARAM_CAL_EN
	{0x3310, 0x18},	//clock check
	{0x3312, 0xbe},	//clock check
	{0x3311, 0x00},	//clock check
	{0x3314, 0x9e},	//clock check
	{0x3313, 0x00},	//clock check
	{0x3316, 0x77},	//clock check
	{0x3315, 0x00},	//clock check

	//OFFSET LEF
	{0x432E, 0x00},
	{0x432F, 0x80},
	{0x4330, 0x00},
	{0x4331, 0x80},
	{0x4332, 0x00},
	{0x4333, 0x80},
	{0x4334, 0x00},
	{0x4335, 0x80},
	//OFFSET MEF
	{0x4361, 0x00},
	{0x4362, 0x80},
	{0x4363, 0x00},
	{0x4364, 0x80},
	{0x4365, 0x00},
	{0x4366, 0x80},
	{0x4367, 0x00},
	{0x4368, 0x80},
	//OFFSET SEF
	{0x4394, 0x00},
	{0x4395, 0x80},
	{0x4396, 0x00},
	{0x4397, 0x80},
	{0x4398, 0x00},
	{0x4399, 0x80},
	{0x439A, 0x00},
	{0x439B, 0x80},

	{0x4402, 0x57},	// ‌LEF(Long Exposure Frame) DPC enabled
	{0x3614, 0x03},
	{0x3615, 0x08},

	{0x3507, 0xcb},
	{0x3502, 0x1D},
	{0x390c, 0x1b},
	{0x390d, 0x1b},
	{0x390e, 0x1b},
	{0x390f, 0x1b},
	{0x3910, 0x1b},
	{0x3911, 0x1b},
	{0x3a0e, 0x0f},
	{0x4102, 0x13},
	{0x411e, 0x13},
	{0x410e, 0x02},
	{0x410f, 0x39},
	{0x412a, 0x02},
	{0x412b, 0x39},
	{0x4303, 0x13},
	{0x4304, 0x39},
	{0x4309, 0x00},
	{0x4336, 0x13},
	{0x4337, 0x39},
	{0x433c, 0x00},
	{0x3c37, 0x8c},
	{0x5309, 0x64},
	{0x3913, 0x1c},
	{0x3914, 0x1c},
	{0x3031, 0x0c},
	{0x3008, 0x01},
	{0x3006, 0x00},
	{0x3c16, 0x22},
	{0x3c1a, 0x01},
	{REG_DELAY, 0x01},
	//{0x300c, 0x81},
	{REG_NULL, 0x00},
};

static const struct mis40h1_mode supported_modes_2lane[] = {
	{
		.width = 2688,
		.height = 1520,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0052,
		.hts_def = 0x0c80,
		.vts_def = 0x0672,
		.bus_fmt = MEDIA_BUS_FMT_SGRBG12_1X12,
		.global_reg_list = mis40h1_global_regs,
		.reg_list = mis40h1_linear_12_2688x1520_30fps_2lane_regs,
		.hdr_mode = NO_HDR,
		.mclk = 24000000,
		.link_freq_idx = 0,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.lanes = 2,
	}
};

static const struct mis40h1_mode supported_modes_4lane[] = {
	{
		.width = 2688,
		.height = 1520,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0052,
		.hts_def = 0x0c80,
		.vts_def = 0x0672,
		.bus_fmt = MEDIA_BUS_FMT_SGRBG12_1X12,
		.global_reg_list = mis40h1_global_regs,
		.reg_list = mis40h1_linear_12_2688x1520_60fps_4lane_regs,
		.hdr_mode = NO_HDR,
		.mclk = 24000000,
		.link_freq_idx = 1,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
		.lanes = 4,
	}
};

static const s64 link_freq_menu_items[] = {
	MIS40H1_LINK_FREQ_476M,
	MIS40H1_LINK_FREQ_474M,
};

static const char *const mis40h1_test_pattern_menu[] = {
	"Disabled",
	"Vertical Gradient Gray Type 1",
	"Horizontal Gradient Gray Type 2",
	"Vertical Color Bar Type 3",
};

/* Write registers up to 4 at a time */
static int mis40h1_write_reg(struct i2c_client *client, u16 reg,
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

static int mis40h1_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY)
			usleep_range(regs[i].val * 1000, regs[i].val * 1000 + 1000);
		else
			ret = mis40h1_write_reg(client, regs[i].addr,
						MIS40H1_REG_VALUE_08BIT, regs[i].val);
	}

	return ret;
}

/* Read registers up to 4 at a time */
static int mis40h1_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static void mis40h1_set_orientation_reg(struct mis40h1 *mis40h1, u32 en_flip_mir)
{
	int ret = 0;

	switch (en_flip_mir) {
	case  0:
		ret = mis40h1_write_reg(mis40h1->client, 0x300c,
					MIS40H1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 60000);
		ret |= mis40h1_write_reg(mis40h1->client, 0x3007,
					 MIS40H1_REG_VALUE_08BIT, 0x00);
		ret |= mis40h1_write_reg(mis40h1->client, 0x300c,
					 MIS40H1_REG_VALUE_08BIT, 0x01);
		break;
	case  1:
		ret = mis40h1_write_reg(mis40h1->client, 0x300c,
					MIS40H1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 60000);
		ret |= mis40h1_write_reg(mis40h1->client, 0x3007,
					 MIS40H1_REG_VALUE_08BIT, 0x01);
		ret |= mis40h1_write_reg(mis40h1->client, 0x300c,
					 MIS40H1_REG_VALUE_08BIT, 0x01);
		break;
	case  2:
		ret = mis40h1_write_reg(mis40h1->client, 0x300c,
					MIS40H1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 60000);
		ret |= mis40h1_write_reg(mis40h1->client, 0x3007,
					 MIS40H1_REG_VALUE_08BIT, 0x02);
		ret |= mis40h1_write_reg(mis40h1->client, 0x300c,
					 MIS40H1_REG_VALUE_08BIT, 0x01);
		break;
	case  3:
		ret = mis40h1_write_reg(mis40h1->client, 0x300c,
					MIS40H1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 60000);
		ret |= mis40h1_write_reg(mis40h1->client, 0x3007,
					 MIS40H1_REG_VALUE_08BIT, 0x03);
		ret |= mis40h1_write_reg(mis40h1->client, 0x300c,
					 MIS40H1_REG_VALUE_08BIT, 0x01);
		break;
	default:
		ret = mis40h1_write_reg(mis40h1->client, 0x300c,
					MIS40H1_REG_VALUE_08BIT, 0x00);
		usleep_range(50000, 60000);
		ret |= mis40h1_write_reg(mis40h1->client, 0x3007,
					 MIS40H1_REG_VALUE_08BIT, 0x00);
		ret |= mis40h1_write_reg(mis40h1->client, 0x300c,
					 MIS40H1_REG_VALUE_08BIT, 0x01);
		break;
	}
}

static int tempDetect(struct mis40h1 *mis40h1, u32 *tmpHigh_en)
{
	u32 tempH = 0, tempL = 0;

	mis40h1_read_reg(mis40h1->client, 0x3504,
			 MIS40H1_REG_VALUE_08BIT, &tempH);
	mis40h1_read_reg(mis40h1->client, 0x3505,
			 MIS40H1_REG_VALUE_08BIT, &tempL);
	if ((tempH << 8 | tempL) > 0xc80)
		*tmpHigh_en = 1;
	else if ((tempH << 8 | tempL) < 0xc00)
		*tmpHigh_en = 0;

	//printk("[%s] temp %d, *tmpHigh_en %d\n", __FUNCTION__, tempH << 8 | tempL, *tmpHigh_en);
	return 0;
}

static int pCus_dcgCheck(struct i2c_client *client, u8 *dcgData)
{
	u32 dcg_data = 0x0000;
	int ret = 0;

	ret = mis40h1_read_reg(client, 0x2003, MIS40H1_REG_VALUE_08BIT, &dcg_data);
	if (dcg_data == 0 || dcg_data == 0xff) {
		*dcgData = MIS40H1_LCG_TO_HCG;
		dev_info(&(client->dev), "------dcgData define data:%d------\n", *dcgData);
		return 0;
	}
	*dcgData = (dcg_data & 0x0f) * 10;
	ret = mis40h1_read_reg(client, 0x2004, MIS40H1_REG_VALUE_08BIT, &dcg_data);
	*dcgData = *dcgData + ((dcg_data & 0xfc) >> 2);
	dev_info(&(client->dev), "-----dcgData = %d------\n", *dcgData);

	return 0;
}

static int mis40h1_set_gain_reg(struct mis40h1 *mis40h1, u32 gain)
{
	u32 coarse_again = 0,  dgain = 0x100;
	u32 againMax = 32 * mis40h1->dcgData * MIS40H1_ONCE_GAIN_STEP / 10;
	static u32 tmpHigh_en;// static variable to keep lcg-Hcg having a buffer zone
	int ret = 0;

	if (gain > MIS40H1_GAIN_MAX - 1)
		gain = MIS40H1_GAIN_MAX - 1;

	tempDetect(mis40h1, &tmpHigh_en);
	if (tmpHigh_en == 0) {
		if (gain < mis40h1->dcgData * MIS40H1_ONCE_GAIN_STEP / 10) {
			mis40h1_write_reg(mis40h1->client, 0x310c,
					  MIS40H1_REG_VALUE_08BIT, 0x00);
			coarse_again = 1024 - (1024 * MIS40H1_ONCE_GAIN_STEP / gain);
		}  else if (gain < againMax) {
			coarse_again = 1024 - (1024 * MIS40H1_ONCE_GAIN_STEP * mis40h1->dcgData) / gain / 10;
			mis40h1_write_reg(mis40h1->client, 0x310c, MIS40H1_REG_VALUE_08BIT, 0x01);

		} else {
			dgain = gain * 256 / (againMax);
			coarse_again = 992; // 32x hcg  //
			mis40h1_write_reg(mis40h1->client, 0x310c, MIS40H1_REG_VALUE_08BIT, 0x01);
		}
	} else { // high temp
		if (gain > mis40h1->dcgData * MIS40H1_ONCE_GAIN_STEP * 16 / 10) {
			//dgain = gain * 256 / (1024 * 32);// high temp doesn't use dgain
			coarse_again = 960; // 16x hcg
		} else if (gain < mis40h1->dcgData * MIS40H1_ONCE_GAIN_STEP / 10) {
			coarse_again = 0;
		} else {
			coarse_again = 1024 - (1024 * MIS40H1_ONCE_GAIN_STEP * mis40h1->dcgData) / gain / 10;
		}
		mis40h1_write_reg(mis40h1->client, 0x310c,
				  MIS40H1_REG_VALUE_08BIT, 0x01);
	}

	dev_info(&(mis40h1->client->dev), "---mis40h1->dcgData = %d---,coarse_again:%d dgain:%d\n",
		 mis40h1->dcgData, coarse_again, dgain);

	ret |= mis40h1_write_reg(mis40h1->client,
				 MIS40H1_REG_DIG_GAIN_H,
				 MIS40H1_REG_VALUE_08BIT,
				 dgain >> 8 & 0xff);
	ret |= mis40h1_write_reg(mis40h1->client,
				 MIS40H1_REG_DIG_GAIN_L,
				 MIS40H1_REG_VALUE_08BIT,
				 dgain & 0xff);

	ret |= mis40h1_write_reg(mis40h1->client,
				 MIS40H1_REG_ANA_GAIN_H,
				 MIS40H1_REG_VALUE_08BIT,
				 MIS40H1_FETCH_AGAIN_H(coarse_again));
	ret |= mis40h1_write_reg(mis40h1->client,
				 MIS40H1_REG_ANA_GAIN_L,
				 MIS40H1_REG_VALUE_08BIT,
				 MIS40H1_FETCH_AGAIN_L(coarse_again));
	ret |= mis40h1_write_reg(mis40h1->client,
				 MIS40H1_REG_GAIN_EXP_VALID,
				 MIS40H1_REG_VALUE_08BIT,
				 MIS40H1_REG_GAIN_EXP_VALID_VAL);

	return ret;
}

static int mis40h1_get_reso_dist(const struct mis40h1_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct mis40h1_mode *
mis40h1_find_best_fit(struct mis40h1 *mis40h1, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < mis40h1->cfg_num; i++) {
		dist = mis40h1_get_reso_dist(&mis40h1->supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		} else if (dist == cur_best_fit_dist &&
			   framefmt->code == mis40h1->supported_modes[i].bus_fmt) {
			cur_best_fit = i;
			break;
		}
	}

	return &mis40h1->supported_modes[cur_best_fit];
}

static int mis40h1_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	const struct mis40h1_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&mis40h1->mutex);

	mode = mis40h1_find_best_fit(mis40h1, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&mis40h1->mutex);
		return -ENOTTY;
#endif
	} else {
		mis40h1->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(mis40h1->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(mis40h1->vblank, vblank_def,
					 MIS40H1_VTS_MAX - mode->height,
					 1, vblank_def);
		mis40h1->cur_fps = mode->max_fps;
	}

	mutex_unlock(&mis40h1->mutex);

	return 0;
}

static int mis40h1_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	const struct mis40h1_mode *mode = mis40h1->cur_mode;

	mutex_lock(&mis40h1->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&mis40h1->mutex);
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
	mutex_unlock(&mis40h1->mutex);

	return 0;
}

static int mis40h1_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = mis40h1->cur_mode->bus_fmt;

	return 0;
}

static int mis40h1_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);

	if (fse->index >= mis40h1->cfg_num)
		return -EINVAL;

	if (fse->code != mis40h1->supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width  = mis40h1->supported_modes[fse->index].width;
	fse->max_width  = mis40h1->supported_modes[fse->index].width;
	fse->max_height = mis40h1->supported_modes[fse->index].height;
	fse->min_height = mis40h1->supported_modes[fse->index].height;

	return 0;
}

static int mis40h1_enable_test_pattern(struct mis40h1 *mis40h1, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = mis40h1_read_reg(mis40h1->client, MIS40H1_REG_TEST_PATTERN,
			       MIS40H1_REG_VALUE_08BIT, &val);
	if (pattern == 1) { // Gradient gray normal left-right
		val |= MIS40H1_TEST_PATTERN_BIT_MASK;
		val &= ~MIS40H1_TEST_PATTERN_GRAY_DIR;
		val |= MIS40H1_TEST_PATTERN_MODE;
	} else if (pattern == 2) { // Gradient gray normal up-down
		val |= MIS40H1_TEST_PATTERN_BIT_MASK;
		val |= MIS40H1_TEST_PATTERN_GRAY_DIR;
		val |= MIS40H1_TEST_PATTERN_MODE;
	} else if (pattern == 3) { // color bar
		val |= MIS40H1_TEST_PATTERN_BIT_MASK;
		val &= ~MIS40H1_TEST_PATTERN_MODE;
	} else if (pattern == 0) {
		val = 0;
	}
	ret |= mis40h1_write_reg(mis40h1->client, MIS40H1_REG_TEST_PATTERN,
				 MIS40H1_REG_VALUE_08BIT, val);

	return ret;
}

static int mis40h1_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	const struct mis40h1_mode *mode = mis40h1->cur_mode;

	if (mis40h1->streaming)
		fi->interval = mis40h1->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int mis40h1_g_mbus_config(struct v4l2_subdev *sd,
				 unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	const struct mis40h1_mode *mode = mis40h1->cur_mode;
	u32 val = 1 << (mode->lanes - 1) |
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

static void mis40h1_get_module_inf(struct mis40h1 *mis40h1,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, MIS40H1_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, mis40h1->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, mis40h1->len_name, sizeof(inf->base.lens));
}

static long mis40h1_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = mis40h1->bus_cfg.bus.mipi_csi2.num_data_lanes;
	const struct mis40h1_mode *mode;
	int cur_best_fit = -1;
	int cur_best_fit_dist = -1;
	int cur_dist, cur_fps, dst_fps;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		mis40h1_get_module_inf(mis40h1, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = mis40h1->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		if (hdr->hdr_mode == mis40h1->cur_mode->hdr_mode)
			return 0;
		w = mis40h1->cur_mode->width;
		h = mis40h1->cur_mode->height;
		dst_fps = DIV_ROUND_CLOSEST(mis40h1->cur_mode->max_fps.denominator,
					    mis40h1->cur_mode->max_fps.numerator);

		for (i = 0; i < mis40h1->cfg_num; i++) {
			if (w == mis40h1->supported_modes[i].width &&
			    h == mis40h1->supported_modes[i].height &&
			    mis40h1->supported_modes[i].hdr_mode == hdr->hdr_mode) {
				cur_fps = DIV_ROUND_CLOSEST(mis40h1->supported_modes[i].max_fps.denominator,
							    mis40h1->supported_modes[i].max_fps.numerator);
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
			dev_err(&mis40h1->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mis40h1->cur_mode = &mis40h1->supported_modes[cur_best_fit];
			mode = mis40h1->cur_mode;
			w = mis40h1->cur_mode->hts_def - mis40h1->cur_mode->width;
			h = mis40h1->cur_mode->vts_def - mis40h1->cur_mode->height;
			__v4l2_ctrl_modify_range(mis40h1->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(mis40h1->vblank, h,
						 MIS40H1_VTS_MAX - mis40h1->cur_mode->height, 1, h);
			mis40h1->cur_fps = mis40h1->cur_mode->max_fps;

			dst_link_freq = mode->link_freq_idx;
			dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
					 mode->bpp * 2 * lanes;
			__v4l2_ctrl_s_ctrl_int64(mis40h1->pixel_rate,
						 dst_pixel_rate);
			__v4l2_ctrl_s_ctrl(mis40h1->link_freq,
					   dst_link_freq);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream)
			ret = mis40h1_write_reg(mis40h1->client, MIS40H1_REG_STREAM_CTRL,
						MIS40H1_REG_VALUE_08BIT, MIS40H1_STREAM_ON);
		else
			ret = mis40h1_write_reg(mis40h1->client, MIS40H1_REG_STREAM_CTRL,
						MIS40H1_REG_VALUE_08BIT, MIS40H1_STREAM_OFF);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long mis40h1_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = mis40h1_ioctl(sd, cmd, inf);
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

		ret = mis40h1_ioctl(sd, cmd, hdr);
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
			ret = mis40h1_ioctl(sd, cmd, hdr);
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
			ret = mis40h1_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = mis40h1_ioctl(sd, cmd, &stream);
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

static int __mis40h1_start_stream(struct mis40h1 *mis40h1)
{
	int ret = 0;

	if (!mis40h1->is_thunderboot) {
		ret = mis40h1_write_array(mis40h1->client, mis40h1->cur_mode->reg_list);
		if (ret)
			return ret;
		ret = mis40h1_write_reg(mis40h1->client, MIS40H1_REG_CTRL_MODE,
					MIS40H1_REG_VALUE_08BIT, MIS40H1_MODE_STREAMING);
		/* In case these controls are set before streaming */
		ret |= __v4l2_ctrl_handler_setup(&mis40h1->ctrl_handler);
		if (ret)
			return ret;
	}

	ret |= mis40h1_write_reg(mis40h1->client, MIS40H1_REG_STREAM_CTRL,
				 MIS40H1_REG_VALUE_08BIT, MIS40H1_STREAM_ON);

	return ret;
}

static int __mis40h1_stop_stream(struct mis40h1 *mis40h1)
{
	if (mis40h1->is_thunderboot) {
		mis40h1->is_first_streamoff = true;
		pm_runtime_put(&mis40h1->client->dev);
	}
	return mis40h1_write_reg(mis40h1->client, MIS40H1_REG_STREAM_CTRL,
				 MIS40H1_REG_VALUE_08BIT, MIS40H1_STREAM_OFF);
}

static int __mis40h1_power_on(struct mis40h1 *mis40h1);
static int mis40h1_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	struct i2c_client *client = mis40h1->client;
	int ret = 0;

	mutex_lock(&mis40h1->mutex);
	on = !!on;
	if (on == mis40h1->streaming)
		goto unlock_and_return;

	if (on) {
		if (mis40h1->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			mis40h1->is_thunderboot = false;
			__mis40h1_power_on(mis40h1);
		}

		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __mis40h1_start_stream(mis40h1);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__mis40h1_stop_stream(mis40h1);
		pm_runtime_put(&client->dev);
	}

	mis40h1->streaming = on;

unlock_and_return:
	mutex_unlock(&mis40h1->mutex);

	return ret;
}

static int mis40h1_s_power(struct v4l2_subdev *sd, int on)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	struct i2c_client *client = mis40h1->client;
	int ret = 0;

	mutex_lock(&mis40h1->mutex);

	/* If the power state is not modified - no work to do. */
	if (mis40h1->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!mis40h1->is_thunderboot) {
			ret = mis40h1_write_array(mis40h1->client, mis40h1_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		mis40h1->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		mis40h1->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&mis40h1->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 mis40h1_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, MIS40H1_XVCLK_FREQ / 1000 / 1000);
}

static int __mis40h1_power_on(struct mis40h1 *mis40h1)
{
	int ret;
	u32 delay_us;
	struct device *dev = &mis40h1->client->dev;

	if (!IS_ERR_OR_NULL(mis40h1->pins_default)) {
		ret = pinctrl_select_state(mis40h1->pinctrl,
					   mis40h1->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(mis40h1->xvclk, mis40h1->cur_mode->mclk);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate  (%dHz)\n", mis40h1->cur_mode->mclk);
	if (clk_get_rate(mis40h1->xvclk) != mis40h1->cur_mode->mclk)
		dev_warn(dev, "xvclk mismatched, modes are based on %dHz\n",
			 mis40h1->cur_mode->mclk);
	ret = clk_prepare_enable(mis40h1->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (mis40h1->is_thunderboot)
		return 0;

	if (!IS_ERR(mis40h1->reset_gpio))
		gpiod_set_value_cansleep(mis40h1->reset_gpio, 0);

	ret = regulator_bulk_enable(MIS40H1_NUM_SUPPLIES, mis40h1->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(mis40h1->reset_gpio))
		gpiod_set_value_cansleep(mis40h1->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(mis40h1->pwdn_gpio))
		gpiod_set_value_cansleep(mis40h1->pwdn_gpio, 1);

	if (!IS_ERR(mis40h1->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = mis40h1_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

disable_clk:
	clk_disable_unprepare(mis40h1->xvclk);

	return ret;
}

static void __mis40h1_power_off(struct mis40h1 *mis40h1)
{
	int ret;
	struct device *dev = &mis40h1->client->dev;

	clk_disable_unprepare(mis40h1->xvclk);
	if (mis40h1->is_thunderboot) {
		if (mis40h1->is_first_streamoff) {
			mis40h1->is_thunderboot = false;
			mis40h1->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(mis40h1->pwdn_gpio))
		gpiod_set_value_cansleep(mis40h1->pwdn_gpio, 0);
	if (!IS_ERR(mis40h1->reset_gpio))
		gpiod_set_value_cansleep(mis40h1->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(mis40h1->pins_sleep)) {
		ret = pinctrl_select_state(mis40h1->pinctrl,
					   mis40h1->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(MIS40H1_NUM_SUPPLIES, mis40h1->supplies);
}

static int __maybe_unused mis40h1_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis40h1 *mis40h1 = to_mis40h1(sd);

	return __mis40h1_power_on(mis40h1);
}

static int __maybe_unused mis40h1_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis40h1 *mis40h1 = to_mis40h1(sd);

	__mis40h1_power_off(mis40h1);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int mis40h1_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct mis40h1_mode *def_mode = &mis40h1->supported_modes[0];

	mutex_lock(&mis40h1->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&mis40h1->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int mis40h1_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct mis40h1 *mis40h1 = to_mis40h1(sd);

	if (fie->index >= mis40h1->cfg_num)
		return -EINVAL;

	fie->code = mis40h1->supported_modes[fie->index].bus_fmt;
	fie->width = mis40h1->supported_modes[fie->index].width;
	fie->height = mis40h1->supported_modes[fie->index].height;
	fie->interval = mis40h1->supported_modes[fie->index].max_fps;
	fie->reserved[0] = mis40h1->supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops mis40h1_pm_ops = {
	SET_RUNTIME_PM_OPS(mis40h1_runtime_suspend,
	mis40h1_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops mis40h1_internal_ops = {
	.open = mis40h1_open,
};
#endif

static const struct v4l2_subdev_core_ops mis40h1_core_ops = {
	.s_power = mis40h1_s_power,
	.ioctl = mis40h1_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = mis40h1_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops mis40h1_video_ops = {
	.s_stream = mis40h1_s_stream,
	.g_frame_interval = mis40h1_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops mis40h1_pad_ops = {
	.enum_mbus_code = mis40h1_enum_mbus_code,
	.enum_frame_size = mis40h1_enum_frame_sizes,
	.enum_frame_interval = mis40h1_enum_frame_interval,
	.get_fmt = mis40h1_get_fmt,
	.set_fmt = mis40h1_set_fmt,
	.get_mbus_config = mis40h1_g_mbus_config,
};

static const struct v4l2_subdev_ops mis40h1_subdev_ops = {
	.core = &mis40h1_core_ops,
	.video = &mis40h1_video_ops,
	.pad = &mis40h1_pad_ops,
};

static void mis40h1_modify_fps_info(struct mis40h1 *mis40h1)
{
	const struct mis40h1_mode *mode = mis40h1->cur_mode;

	mis40h1->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       mis40h1->cur_vts;
}

static int mis40h1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mis40h1 *mis40h1 = container_of(ctrl->handler,
					       struct mis40h1, ctrl_handler);
	struct i2c_client *client = mis40h1->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = mis40h1->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(mis40h1->exposure,
					 mis40h1->exposure->minimum, max,
					 mis40h1->exposure->step,
					 mis40h1->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_info(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (mis40h1->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val;

			/* 4 least significant bits of expsoure are fractional part */
			ret = mis40h1_write_reg(mis40h1->client,
						MIS40H1_REG_EXPOSURE_H,
						MIS40H1_REG_VALUE_08BIT,
						MIS40H1_FETCH_EXP_H(val));
			ret |= mis40h1_write_reg(mis40h1->client,
						 MIS40H1_REG_EXPOSURE_L,
						 MIS40H1_REG_VALUE_08BIT,
						 MIS40H1_FETCH_EXP_L(val));
			ret |= mis40h1_write_reg(mis40h1->client,
						 MIS40H1_REG_GAIN_EXP_VALID,
						 MIS40H1_REG_VALUE_08BIT,
						 MIS40H1_REG_GAIN_EXP_VALID_VAL);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (mis40h1->cur_mode->hdr_mode == NO_HDR)
			ret = mis40h1_set_gain_reg(mis40h1, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = mis40h1_write_reg(mis40h1->client,
					MIS40H1_REG_VTS_H,
					MIS40H1_REG_VALUE_08BIT,
					(ctrl->val + mis40h1->cur_mode->height)
					>> 8);
		ret |= mis40h1_write_reg(mis40h1->client,
					 MIS40H1_REG_VTS_L,
					 MIS40H1_REG_VALUE_08BIT,
					 (ctrl->val + mis40h1->cur_mode->height)
					 & 0xff);

		mis40h1->cur_vts = ctrl->val + mis40h1->cur_mode->height;
		if (mis40h1->cur_vts != mis40h1->cur_mode->vts_def)
			mis40h1_modify_fps_info(mis40h1);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = mis40h1_enable_test_pattern(mis40h1, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = mis40h1_read_reg(mis40h1->client, MIS40H1_FLIP_MIRROR_REG,
				       MIS40H1_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val |= MIRROR_BIT_MASK;
		else
			val &= ~MIRROR_BIT_MASK;
		mis40h1_set_orientation_reg(mis40h1, val);
		break;
	case V4L2_CID_VFLIP:
		ret = mis40h1_read_reg(mis40h1->client, MIS40H1_FLIP_MIRROR_REG,
				       MIS40H1_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val |= FLIP_BIT_MASK;
		else
			val &= ~FLIP_BIT_MASK;
		mis40h1_set_orientation_reg(mis40h1, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops mis40h1_ctrl_ops = {
	.s_ctrl = mis40h1_set_ctrl,
};

static int mis40h1_initialize_controls(struct mis40h1 *mis40h1)
{
	const struct mis40h1_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;
	u8 lanes = mis40h1->bus_cfg.bus.mipi_csi2.num_data_lanes;

	handler = &mis40h1->ctrl_handler;
	mode = mis40h1->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &mis40h1->mutex;

	mis40h1->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
			     ARRAY_SIZE(link_freq_menu_items) - 1,
			     0, link_freq_menu_items);
	if (mis40h1->link_freq)
		mis40h1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	dst_link_freq = mode->link_freq_idx;
	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
			 mode->bpp * 2 * lanes;

	if (lanes == 2)
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
				  0, PIXEL_RATE_WITH_476M_12BIT_2L,
				  1, dst_pixel_rate);
	else
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
				  0, PIXEL_RATE_WITH_474M_12BIT_4L,
				  1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(mis40h1->link_freq, dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	mis40h1->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (mis40h1->hblank)
		mis40h1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	mis40h1->vblank = v4l2_ctrl_new_std(handler, &mis40h1_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    MIS40H1_VTS_MAX - mode->height,
					    1, vblank_def);
	mis40h1->cur_fps = mode->max_fps;
	exposure_max = mode->vts_def - 1;
	mis40h1->exposure = v4l2_ctrl_new_std(handler, &mis40h1_ctrl_ops,
					      V4L2_CID_EXPOSURE, MIS40H1_EXPOSURE_MIN,
					      exposure_max, MIS40H1_EXPOSURE_STEP,
					      mode->exp_def);
	mis40h1->anal_gain = v4l2_ctrl_new_std(handler, &mis40h1_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, MIS40H1_GAIN_MIN,
					       MIS40H1_GAIN_MAX, MIS40H1_GAIN_STEP,
					       MIS40H1_GAIN_DEFAULT);
	mis40h1->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&mis40h1_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(mis40h1_test_pattern_menu) - 1,
				0, 0, mis40h1_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &mis40h1_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &mis40h1_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&mis40h1->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	mis40h1->subdev.ctrl_handler = handler;
	mis40h1->cur_fps = mode->max_fps;
	mis40h1->is_standby = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int mis40h1_check_sensor_id(struct mis40h1 *mis40h1,
				   struct i2c_client *client)
{
	struct device *dev = &mis40h1->client->dev;
	u32 id = 0;
	int ret;

	if (mis40h1->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = mis40h1_read_reg(client, MIS40H1_REG_CHIP_ID,
			       MIS40H1_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected MIS40H1 (id:%04x) sensor\n", CHIP_ID);

	return 0;
}

static int mis40h1_configure_regulators(struct mis40h1 *mis40h1)
{
	unsigned int i;

	for (i = 0; i < MIS40H1_NUM_SUPPLIES; i++)
		mis40h1->supplies[i].supply = mis40h1_supply_names[i];

	return devm_regulator_bulk_get(&mis40h1->client->dev,
				       MIS40H1_NUM_SUPPLIES,
				       mis40h1->supplies);
}

static int mis40h1_read_module_info(struct mis40h1 *mis40h1)
{
	int ret;
	struct device *dev = &mis40h1->client->dev;
	struct device_node *node = dev->of_node;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &mis40h1->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &mis40h1->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &mis40h1->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &mis40h1->len_name);
	if (ret)
		dev_err(dev, "could not get module information!\n");

	/* Compatible with non-standby mode if this attribute is not configured in dts*/
	of_property_read_u32(node, RKMODULE_CAMERA_STANDBY_HW,
			     &mis40h1->standby_hw);
	dev_info(dev, "mis40h1->standby_hw = %d\n", mis40h1->standby_hw);

	return ret;
}

static int mis40h1_find_modes(struct mis40h1 *mis40h1)
{
	int i, ret;
	u32 hdr_mode = 0;
	struct device_node *endpoint;
	struct device *dev = &mis40h1->client->dev;
	struct device_node *node = mis40h1->client->dev.of_node;

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, "Get hdr mode failed! no hdr default\n");
	} else
		dev_warn(dev, "Get hdr mode OK! hdr_mode = %d\n", hdr_mode);

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &mis40h1->bus_cfg);
	of_node_put(endpoint);
	if (ret) {
		dev_err(dev, "Failed to get bus config\n");
		return -EINVAL;
	}

	if (mis40h1->bus_cfg.bus.mipi_csi2.num_data_lanes == 4) {
		mis40h1->supported_modes = supported_modes_4lane;
		mis40h1->cfg_num = ARRAY_SIZE(supported_modes_4lane);
		dev_info(dev, "Detect mis40h1 lane: %d\n",
			 mis40h1->bus_cfg.bus.mipi_csi2.num_data_lanes);
	} else {
		mis40h1->supported_modes = supported_modes_2lane;
		mis40h1->cfg_num = ARRAY_SIZE(supported_modes_2lane);
		dev_info(dev, "Detect mis40h1 lane: %d\n",
			 mis40h1->bus_cfg.bus.mipi_csi2.num_data_lanes);
	}

	for (i = 0; i < mis40h1->cfg_num; i++) {
		if (hdr_mode == mis40h1->supported_modes[i].hdr_mode) {
			mis40h1->cur_mode = &mis40h1->supported_modes[i];
			break;
		}
	}

	if (i == mis40h1->cfg_num)
		mis40h1->cur_mode = &mis40h1->supported_modes[0];

	return 0;
}

static int mis40h1_setup_clocks_and_gpios(struct mis40h1 *mis40h1)
{
	struct device *dev = &mis40h1->client->dev;

	mis40h1->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(mis40h1->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	mis40h1->reset_gpio = devm_gpiod_get(dev, "reset",
					     mis40h1->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(mis40h1->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	mis40h1->pwdn_gpio = devm_gpiod_get(dev, "pwdn",
					    mis40h1->is_thunderboot ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(mis40h1->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	mis40h1->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(mis40h1->pinctrl)) {
		mis40h1->pins_default =
			pinctrl_lookup_state(mis40h1->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(mis40h1->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		mis40h1->pins_sleep =
			pinctrl_lookup_state(mis40h1->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(mis40h1->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	return 0;
}

static int mis40h1_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct mis40h1 *mis40h1;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	mis40h1 = devm_kzalloc(dev, sizeof(*mis40h1), GFP_KERNEL);
	if (!mis40h1)
		return -ENOMEM;

	mis40h1->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);
	mis40h1->client = client;

	ret = mis40h1_read_module_info(mis40h1);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	/* Set current mode based on HDR mode */
	ret = mis40h1_find_modes(mis40h1);
	if (ret) {
		dev_err(dev, "Failed to get modes!\n");
		return -EINVAL;
	}

	/* setup mis40h1 clock and gpios*/
	ret = mis40h1_setup_clocks_and_gpios(mis40h1);
	if (ret) {
		dev_err(dev, "Failed to set up clocks and GPIOs\n");
		return ret;
	}

	ret = mis40h1_configure_regulators(mis40h1);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&mis40h1->mutex);

	sd = &mis40h1->subdev;
	v4l2_i2c_subdev_init(sd, client, &mis40h1_subdev_ops);
	ret = mis40h1_initialize_controls(mis40h1);
	if (ret)
		goto err_destroy_mutex;

	ret = __mis40h1_power_on(mis40h1);
	if (ret)
		goto err_free_handler;

	ret = mis40h1_check_sensor_id(mis40h1, client);
	if (ret)
		goto err_power_off;

	pCus_dcgCheck(mis40h1->client, &mis40h1->dcgData);

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &mis40h1_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	mis40h1->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &mis40h1->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(mis40h1->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 mis40h1->module_index, facing,
		 MIS40H1_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (mis40h1->is_thunderboot)
		pm_runtime_get_sync(dev);
	else
		pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__mis40h1_power_off(mis40h1);
err_free_handler:
	v4l2_ctrl_handler_free(&mis40h1->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&mis40h1->mutex);

	return ret;
}

static int mis40h1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mis40h1 *mis40h1 = to_mis40h1(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&mis40h1->ctrl_handler);
	mutex_destroy(&mis40h1->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__mis40h1_power_off(mis40h1);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mis40h1_of_match[] = {
	{ .compatible = "imagedesign,mis40h1" },
	{},
};
MODULE_DEVICE_TABLE(of, mis40h1_of_match);
#endif

static const struct i2c_device_id mis40h1_match_id[] = {
	{ "imagedesign,mis40h1", 0 },
	{ },
};

static struct i2c_driver mis40h1_i2c_driver = {
	.driver = {
		.name = MIS40H1_NAME,
		.pm = &mis40h1_pm_ops,
		.of_match_table = of_match_ptr(mis40h1_of_match),
	},
	.probe = &mis40h1_probe,
	.remove = &mis40h1_remove,
	.id_table = mis40h1_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&mis40h1_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&mis40h1_i2c_driver);
}

#if defined(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP) && !defined(CONFIG_INITCALL_ASYNC)
subsys_initcall(sensor_mod_init);
#else
device_initcall_sync(sensor_mod_init);
#endif
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("imagedesign mis40h1 sensor driver");
MODULE_LICENSE("GPL");
