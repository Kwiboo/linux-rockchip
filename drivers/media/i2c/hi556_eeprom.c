// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020 Rockchip Electronics Co., Ltd.
/*
 * otp_eeprom driver
 *
 * V0.0X01.0X01
 * 1. fix table_size.
 * 2. fix ioctl return value.
 * 3. add version control.
 * V0.0X01.0X02
 * 1. fix otp info null issue.
 */

//#define DEBUG

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/rk-camera-module.h>
#include <linux/sem.h>
#include <linux/seq_file.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/version.h>
#include "otp_eeprom.h"
#include "hi556_eeprom_head.h"

#define DRIVER_VERSION		KERNEL_VERSION(0, 0x01, 0x02)
#define DEVICE_NAME			"otp_eeprom"

#define OTP_READ_REG		0x0108

static inline struct eeprom_device
*sd_to_eeprom(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct eeprom_device, sd);
}

static int write_reg_otp(struct i2c_client *client, u16 reg,
			 unsigned int len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	client->addr = 0x20;

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

	if (i2c_master_send(client, buf, len + 2) != len + 2) {
		dev_err(&client->dev,
			"write reg(0x%x val:0x%x)failed !\n", reg, val);
		return -EIO;
	}
	return 0;
}

/* Read registers up to 4 at a time */
static int read_reg_otp(struct i2c_client *client, u16 reg,
			unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	client->addr = 0x20;

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

static int enable_otp_read(struct i2c_client *client, u16 addr)
{
	int ret = 0;

	ret |= write_reg_otp(client, 0x010a, 1, (addr >> 8) & 0xff);
	ret |= write_reg_otp(client, 0x010b, 1, addr & 0xff);
	ret |= write_reg_otp(client, 0x0102, 1, 0x01);

	return ret;
}

static u8 get_vendor_flag(struct i2c_client *client)
{
	u8 vendor_flag = 0;
	u8 vendor[9];
	int i = 0;
	u32 temp = 0;

	enable_otp_read(client, INFO_FLAG_REG);

	for (i = 0; i < 8; i++) {
		read_reg_otp(client, OTP_READ_REG, 1, &temp);
		dev_info(&client->dev, "read vendor value: %x", temp);
		vendor[i] = (u8)temp;
	}
	vendor[8] = 0;
	if (strcmp(vendor, "ROCKCHIP") == 0)
		vendor_flag |= 0x40;
	else
		vendor_flag |= 0x80;
	return vendor_flag;
}

static int otp_read_data(struct eeprom_device *eeprom_dev)
{
	struct i2c_client *client = eeprom_dev->client;
	int i;
	struct otp_info *otp_ptr;
	struct device *dev = &eeprom_dev->client->dev;
	int ret = 0;
	u32 temp = 0;

	otp_ptr = kzalloc(sizeof(*otp_ptr), GFP_KERNEL);
	if (!otp_ptr)
		return -ENOMEM;
	/* OTP base information*/
	ret = read_reg_otp(client, INFO_FLAG_REG,
			   1, &otp_ptr->basic_data.flag);
	if (otp_ptr->basic_data.flag == 0x01) {
		ret |= read_reg_otp(client, INFO_ID_REG,
				    1, &otp_ptr->basic_data.id.supplier_id);
		ret |= read_reg_otp(client, INFO_ID_REG + 1,
				    1, &otp_ptr->basic_data.id.year);
		ret |= read_reg_otp(client, INFO_ID_REG + 2,
				    1, &otp_ptr->basic_data.id.month);
		ret |= read_reg_otp(client, INFO_ID_REG + 3,
				    1, &otp_ptr->basic_data.id.day);
		ret |= read_reg_otp(client, INFO_ID_REG + 4,
				    1, &otp_ptr->basic_data.id.sensor_id);
		ret |= read_reg_otp(client, INFO_ID_REG + 5,
				    1, &otp_ptr->basic_data.id.lens_id);
		ret |= read_reg_otp(client, INFO_ID_REG + 6,
				    1, &otp_ptr->basic_data.id.vcm_id);
		ret |= read_reg_otp(client, INFO_ID_REG + 7,
				    1, &otp_ptr->basic_data.id.driver_ic_id);
		ret |= read_reg_otp(client, INFO_ID_REG + 8,
				    1, &otp_ptr->basic_data.id.color_temperature_id);
		for (i = 0; i < SMARTISAN_PN_SIZE; i++) {
			ret |= read_reg_otp(client, SMARTISAN_PN_REG + i,
					    1, &temp);
			otp_ptr->basic_data.smartisan_pn[i] = temp;
		}
		for (i = 0; i < MOUDLE_ID_SIZE; i++) {
			ret |= read_reg_otp(client, MOUDLE_ID_REG + i,
					    1, &temp);
			otp_ptr->basic_data.modul_id[i] = temp;
		}
		ret |= read_reg_otp(client, MIRROR_FLIP_REG,
				    1, &otp_ptr->basic_data.mirror_flip);
		ret |= read_reg_otp(client, FULL_SIZE_WIGHT_REG,
				    2, &temp);
		otp_ptr->basic_data.size.width = temp;
		ret |= read_reg_otp(client, FULL_SIZE_HEIGHT_REG,
				    2, &temp);
		otp_ptr->basic_data.size.height = temp;
		ret |= read_reg_otp(client, INFO_CHECKSUM_REG,
				    1, &otp_ptr->basic_data.checksum);

		dev_dbg(dev, "fasic info: supplier_id(0x%x) lens(0x%x) time(%d_%d_%d)\n",
			otp_ptr->basic_data.id.supplier_id,
			otp_ptr->basic_data.id.lens_id,
			otp_ptr->basic_data.id.year,
			otp_ptr->basic_data.id.month,
			otp_ptr->basic_data.id.day);
		if (ret)
			goto err;
	}

	/* OTP WB calibration data */
	ret = read_reg_otp(client, AWB_FLAG_REG,
			   1, &otp_ptr->awb_data.flag);
	if (otp_ptr->awb_data.flag == 0x01) {
		ret |= read_reg_otp(client, AWB_VERSION_REG,
				    1, &otp_ptr->awb_data.version);
		ret |= read_reg_otp(client, CUR_R_REG,
				    2, &otp_ptr->awb_data.r_ratio);
		ret |= read_reg_otp(client, CUR_B_REG,
				    2, &otp_ptr->awb_data.b_ratio);
		ret |= read_reg_otp(client, CUR_G_REG,
				    2, &otp_ptr->awb_data.g_ratio);
		ret |= read_reg_otp(client, GOLDEN_R_REG,
				    2, &otp_ptr->awb_data.r_golden);
		ret |= read_reg_otp(client, GOLDEN_B_REG,
				    2, &otp_ptr->awb_data.b_golden);
		ret |= read_reg_otp(client, GOLDEN_G_REG,
				    2, &otp_ptr->awb_data.g_golden);
		ret |= read_reg_otp(client, AWB_CHECKSUM_REG,
				    1, &otp_ptr->awb_data.checksum);

		dev_dbg(dev, "awb version:0x%x\n",
			otp_ptr->awb_data.version);
		if (ret)
			goto err;
	}

	/* OTP LSC calibration data */
	ret = read_reg_otp(client, LSC_FLAG_REG,
			   1, &otp_ptr->lsc_data.flag);
	if (otp_ptr->lsc_data.flag == 0x01) {
		ret |= read_reg_otp(client, LSC_VERSION_REG,
				    1, &otp_ptr->lsc_data.version);
		ret |= read_reg_otp(client, LSC_TABLE_SIZE_REG,
				    2, &temp);
		otp_ptr->lsc_data.table_size = temp;
		for (i = 0; i < LSC_DATA_SIZE; i++) {
			ret |= read_reg_otp(client, LSC_DATA_START_REG + i,
					    1, &temp);
			otp_ptr->lsc_data.data[i] = temp;
		}
		ret |= read_reg_otp(client, LSC_CHECKSUM_REG,
				    1, &otp_ptr->lsc_data.checksum);
		dev_dbg(dev, "lsc cur:(version 0x%x, table_size 0x%x checksum 0x%x)\n",
			otp_ptr->lsc_data.version,
			otp_ptr->lsc_data.table_size,
			otp_ptr->lsc_data.checksum);
		if (ret)
			goto err;
	}

	/* OTP sfr calibration data */
	ret = read_reg_otp(client, LSC_FLAG_REG,
			   1, &otp_ptr->sfr_otp_data.flag);
	if (otp_ptr->sfr_otp_data.flag == 0x01) {
		ret |= read_reg_otp(client, SFR_EQUIQ_NUM_REG,
				    1, &otp_ptr->sfr_otp_data.equip_num);
		ret |= read_reg_otp(client, SFR_C_HOR_REG,
				    2, &otp_ptr->sfr_otp_data.center_horizontal);
		ret |= read_reg_otp(client, SFR_C_VER_REG,
				    2, &otp_ptr->sfr_otp_data.center_vertical);
		for (i = 0; i < 3; i++) {
			ret |= read_reg_otp(client, SFR_TOP_L_HOR_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].top_l_horizontal);
			ret |= read_reg_otp(client, SFR_TOP_L_VER_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].top_l_vertical);
			ret |= read_reg_otp(client, SFR_TOP_R_HOR_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].top_r_horizontal);
			ret |= read_reg_otp(client, SFR_TOP_R_VER_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].top_r_vertical);
			ret |= read_reg_otp(client, SFR_BOTTOM_L_HOR_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].bottom_l_horizontal);
			ret |= read_reg_otp(client, SFR_BOTTOM_L_VER_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].bottom_l_vertical);
			ret |= read_reg_otp(client, SFR_BOTTOM_R_HOR_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].bottom_r_horizontal);
			ret |= read_reg_otp(client, SFR_BOTTOM_R_VER_REG + 16 * i,
					    2, &otp_ptr->sfr_otp_data.data[i].bottom_r_vertical);
		}

		ret |= read_reg_otp(client, SFR_CHECKSUM_REG,
				    1, &otp_ptr->sfr_otp_data.checksum);
		if (ret)
			goto err;
	}

	ret = read_reg_otp(client, TOTAL_CHECKSUM_REG,
			   1, &otp_ptr->total_checksum);
	if (ret)
		goto err;

	if (otp_ptr->total_checksum) {
		eeprom_dev->otp = otp_ptr;
		dev_info(dev, "get otp successful\n");
	} else {
		eeprom_dev->otp = NULL;
		kfree(otp_ptr);
		dev_warn(&client->dev, "otp is NULL!\n");
	}

	return 0;
err:
	eeprom_dev->otp = NULL;
	kfree(otp_ptr);
	dev_warn(&client->dev, "@%s read otp err!\n", __func__);
	return -EINVAL;
}

static void rkotp_read_module_info(struct eeprom_device *eeprom_dev,
				   struct otp_info *otp_ptr,
				   u32 base_addr)
{
	struct i2c_client *client = eeprom_dev->client;
	struct device *dev = &eeprom_dev->client->dev;
	int i = 0;
	u8 module_size[4];
	u8 version[2];
	u8 width[2];
	u8 height[2];
	u32 temp = 0;
	u32 checksum = 0;
	int ret = 0;

	enable_otp_read(client, INFO_FLAG_REG);

	for (i = 0; i < 8; i++) {
		read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(&client->dev, "read vendor value: %x", temp);
		checksum += temp;
	}

	for (i = 0; i < 4; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		module_size[i] = temp;
		base_addr += 1;
	}
	otp_ptr->basic_data.module_size = (module_size[0] << 24) | (module_size[1] << 16) |
					  (module_size[2] << 8) | module_size[3];
	checksum += otp_ptr->basic_data.module_size;
	dev_info(dev, "otp_ptr->basic_data.module_size = %x\n", otp_ptr->basic_data.module_size);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		version[i] = temp;
		base_addr += 1;
	}

	otp_ptr->basic_data.version = (version[0] << 8) | version[1];
	checksum += otp_ptr->basic_data.version;
	dev_info(dev, "otp_ptr->basic_data.version = %x\n", otp_ptr->basic_data.version);

	enable_otp_read(client, base_addr);
	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.supplier_id);
	//dev_info(dev, "read reg %x = %x\n", base_addr, otp_ptr->basic_data.id.supplier_id);
	checksum += otp_ptr->basic_data.id.supplier_id;
	base_addr += 1;
	enable_otp_read(client, base_addr);

	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.year);
	//dev_info(dev, "read reg %x = %x\n", base_addr, otp_ptr->basic_data.id.year);
	checksum += otp_ptr->basic_data.id.year;
	base_addr += 1;
	enable_otp_read(client, base_addr);

	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.month);
	//dev_info(dev, "read reg %x = %x\n", base_addr, otp_ptr->basic_data.id.month);
	checksum += otp_ptr->basic_data.id.month;
	base_addr += 1;
	enable_otp_read(client, base_addr);

	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.day);
	//dev_info(dev, "read reg %x = %x\n", base_addr, otp_ptr->basic_data.id.day);
	checksum += otp_ptr->basic_data.id.day;
	base_addr += 1;
	enable_otp_read(client, base_addr);

	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.sensor_id);
	checksum += otp_ptr->basic_data.id.sensor_id;
	//dev_info(dev, "read reg %x = %x\n", base_addr, otp_ptr->basic_data.id.sensor_id);
	base_addr += 1;
	enable_otp_read(client, base_addr);

	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.lens_id);
	checksum += otp_ptr->basic_data.id.lens_id;
	base_addr += 1;
	enable_otp_read(client, base_addr);

	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.vcm_id);
	checksum += otp_ptr->basic_data.id.vcm_id;
	base_addr += 1;
	enable_otp_read(client, base_addr);

	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.id.driver_ic_id);
	checksum += otp_ptr->basic_data.id.driver_ic_id;
	base_addr += 1;
	for (i = 0; i < RKMOUDLE_ID_SIZE; i++) {
		enable_otp_read(client, base_addr);

		ret |= read_reg_otp(client, OTP_READ_REG,
				    1, &temp);
		otp_ptr->basic_data.modul_id[i] = temp;
		//dev_info(dev, "read reg %x = %x\n", base_addr, otp_ptr->basic_data.modul_id[i]);
		checksum += temp;
		base_addr += 1;
	}
	enable_otp_read(client, base_addr);
	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.mirror_flip);
	checksum += otp_ptr->basic_data.mirror_flip;
	base_addr += 1;

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		width[i] = temp;
		base_addr += 1;
	}
	otp_ptr->basic_data.size.width = (width[0] << 8) | width[1];
	checksum += otp_ptr->basic_data.size.width;
	dev_info(dev, "width = %x\n",  otp_ptr->basic_data.size.width);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		height[i] = temp;
		base_addr += 1;
	}
	otp_ptr->basic_data.size.height = (height[0] << 8) | height[1];
	checksum += otp_ptr->basic_data.size.height;
	dev_info(dev, "height = %x\n", otp_ptr->basic_data.size.height);

	for (i = 0; i < RK_INFO_RESERVED_SIZE; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG,
				    1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		checksum += temp;
		base_addr += 1;
	}
	enable_otp_read(client, base_addr);
	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->basic_data.checksum);
	//dev_info(dev, "read reg %x = %x\n", base_addr, otp_ptr->basic_data.checksum);
	if ((checksum % 255 + 1) == otp_ptr->basic_data.checksum && (!ret)) {
		otp_ptr->basic_data.flag = 0x01;
		otp_ptr->flag++;
		dev_info(dev, "fasic info: supplier_id(0x%x) lens(0x%x) time(%d_%d_%d) module id %x\n",
			 otp_ptr->basic_data.id.supplier_id,
			 otp_ptr->basic_data.id.lens_id,
			 otp_ptr->basic_data.id.year,
			 otp_ptr->basic_data.id.month,
			 otp_ptr->basic_data.id.day,
			 (u32)(*otp_ptr->basic_data.modul_id));
	} else {
		otp_ptr->basic_data.flag = 0;
		dev_info(dev, "fasic info: checksum err, checksum %d, reg_checksum %d\n",
			 (int)(checksum % 255 + 1),
			 (int)otp_ptr->basic_data.checksum);
		dev_info(dev, "fasic info: supplier_id(0x%x) lens(0x%x) time(%d_%d_%d)\n",
			 otp_ptr->basic_data.id.supplier_id,
			 otp_ptr->basic_data.id.lens_id,
			 otp_ptr->basic_data.id.year,
			 otp_ptr->basic_data.id.month,
			 otp_ptr->basic_data.id.day);
		dev_info(dev, "fasic info: full size, width(%d) height(%d) flip(0x%x)\n",
			 otp_ptr->basic_data.size.width,
			 otp_ptr->basic_data.size.height,
			 otp_ptr->basic_data.mirror_flip);
	}
}

static void rkotp_read_awb(struct eeprom_device *eeprom_dev,
			   struct otp_info *otp_ptr,
			   u32 base_addr)
{
	struct i2c_client *client = eeprom_dev->client;
	struct device *dev = &eeprom_dev->client->dev;
	u32 checksum = 0;
	u32 temp = 0;
	u8 size[4];
	u8 version[2];
	u8 r_ratio[2];
	u8 g_ratio[2];
	u8 b_ratio[2];
	u8 r_golden[2];
	u8 g_golden[2];
	u8 b_golden[2];
	int i = 0;
	int ret = 0;

	for (i = 0; i < 4; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		size[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.size = (size[0] << 24) | (size[1] << 16) |
				 (size[2] << 8) | size[3];
	checksum += otp_ptr->awb_data.size;
	dev_info(dev, "otp_ptr->awb_data.size = %x\n", otp_ptr->awb_data.size);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		version[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.version = (version[0] << 8) | version[1];
	checksum += otp_ptr->awb_data.version;
	dev_info(dev, "awb_data.version = %x\n",  otp_ptr->awb_data.version);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		r_ratio[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.r_ratio = (r_ratio[0] << 8) | r_ratio[1];
	checksum += otp_ptr->awb_data.r_ratio;
	dev_info(dev, "awb_data.r_ratio = %x\n",  otp_ptr->awb_data.r_ratio);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		b_ratio[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.b_ratio = (b_ratio[0] << 8) | b_ratio[1];
	checksum += otp_ptr->awb_data.b_ratio;
	dev_info(dev, "awb_data.b_ratio = %x\n",  otp_ptr->awb_data.b_ratio);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		g_ratio[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.g_ratio = (g_ratio[0] << 8) | g_ratio[1];
	checksum += otp_ptr->awb_data.g_ratio;
	dev_info(dev, "awb_data.g_ratio = %x\n",  otp_ptr->awb_data.g_ratio);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		r_golden[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.r_golden = (r_golden[0] << 8) | r_golden[1];
	checksum += otp_ptr->awb_data.r_golden;
	dev_info(dev, "awb_data.r_golden = %x\n",  otp_ptr->awb_data.r_golden);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		b_golden[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.b_golden = (b_golden[0] << 8) | b_golden[1];
	checksum += otp_ptr->awb_data.b_golden;
	dev_info(dev, "awb_data.b_golden = %x\n",  otp_ptr->awb_data.b_golden);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		g_golden[i] = temp;
		base_addr += 1;
	}
	otp_ptr->awb_data.g_golden = (g_golden[0] << 8) | g_golden[1];
	checksum += otp_ptr->awb_data.g_golden;
	dev_info(dev, "awb_data.g_golden = %x\n",  otp_ptr->awb_data.g_golden);

	for (i = 0; i < RK_AWB_RESERVED_SIZE; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG,
				    1, &temp);
		//dev_info(dev, "read otp reg %x = %x\n", base_addr, temp);
		checksum += temp;
		base_addr += 1;
	}

	checksum += 1;

	enable_otp_read(client, base_addr);
	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->awb_data.checksum);
	dev_info(dev, "read otp reg %x = %x\n", base_addr, otp_ptr->awb_data.checksum);

	if ((checksum % 255 + 1) == otp_ptr->awb_data.checksum && (!ret)) {
		otp_ptr->awb_data.flag = 0x01;
		otp_ptr->flag++;
		dev_info(dev, "awb version:0x%x\n",
			 otp_ptr->awb_data.version);
		dev_info(dev, "awb cur:(r 0x%x, b 0x%x, g 0x%x)\n",
			 otp_ptr->awb_data.r_ratio,
			 otp_ptr->awb_data.b_ratio,
			 otp_ptr->awb_data.g_ratio);
		dev_info(dev, "awb gol:(r 0x%x, b 0x%x, g 0x%x),\n",
			 otp_ptr->awb_data.r_golden,
			 otp_ptr->awb_data.b_golden,
			 otp_ptr->awb_data.g_golden);
	} else {
		otp_ptr->awb_data.flag = 0;
		dev_info(dev, "awb info: checksum err, checksum %d, reg_checksum %d\n",
			 (int)(checksum % 255 + 1),
			 (int) otp_ptr->awb_data.checksum);
	}
}

static void rkotp_read_lsc(struct eeprom_device *eeprom_dev,
			   struct otp_info *otp_ptr,
			   u32 base_addr)
{
	struct i2c_client *client = eeprom_dev->client;
	struct device *dev = &eeprom_dev->client->dev;
	u32 checksum = 0;
	u32 temp = 0;
	u8 size[4];
	u8 version[2];
	int i = 0;
	int ret = 0;
#ifdef DEBUG
	int w, h, j;
#endif

	for (i = 0; i < 4; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		size[i] = temp;
		base_addr += 1;
	}
	otp_ptr->lsc_data.size = (size[0] << 24) | (size[1] << 16) |
				 (size[2] << 8) | size[3];
	checksum += otp_ptr->lsc_data.size;
	dev_info(dev, "otp_ptr->lsc_data.size = %x\n", otp_ptr->lsc_data.size);

	for (i = 0; i < 2; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG, 1, &temp);
		//dev_info(dev, "read reg %x = %x\n", base_addr, temp);
		version[i] = temp;
		base_addr += 1;
	}
	otp_ptr->lsc_data.version = (version[0] << 8) | version[1];
	checksum += otp_ptr->lsc_data.version;
	dev_info(dev, "lsc_data.version = %x\n",  otp_ptr->lsc_data.version);

	for (i = 0; i < LSC_DATA_SIZE; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG,
				    1, &temp);
		otp_ptr->lsc_data.data[i] = temp;
		checksum += temp;
		base_addr += 1;
	}
	otp_ptr->lsc_data.table_size = LSC_DATA_SIZE;
#ifdef DEBUG
	w = 17 * 2;
	h = 17 * 4;
	dev_info(dev, "show lsc table\n");
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++)
			dev_info(dev, "%d ", otp_ptr->lsc_data.data[i * w + j]);
		if (i < h)
			dev_info(dev, "\n");
	}
#endif
	for (i = 0; i < RK_LSC_RESERVED_SIZE; i++) {
		enable_otp_read(client, base_addr);
		ret |= read_reg_otp(client, OTP_READ_REG,
				    1, &temp);
		checksum += temp;
		base_addr += 1;
	}
	checksum += 2;
	enable_otp_read(client, base_addr);
	ret |= read_reg_otp(client, OTP_READ_REG,
			    1, &otp_ptr->lsc_data.checksum);
	if ((checksum % 255 + 1) == otp_ptr->lsc_data.checksum && (!ret)) {
		otp_ptr->lsc_data.flag = 0x01;
		otp_ptr->flag++;
		dev_info(dev, "lsc info:(version 0x%x, checksum 0x%x)\n",
			 otp_ptr->lsc_data.version,
			 (int)otp_ptr->lsc_data.checksum);
	} else {
		otp_ptr->lsc_data.flag = 0x00;
		dev_info(dev, "lsc info: checksum err, checksum %d, reg_checksum %d\n",
			 (int)(checksum % 255 + 1),
			 (int)otp_ptr->lsc_data.checksum);
	}
}

static void rkotp_read_pdaf(struct eeprom_device *eeprom_dev,
			    struct otp_info *otp_ptr,
			    u32 base_addr)
{
	struct i2c_client *client = eeprom_dev->client;
	struct device *dev = &eeprom_dev->client->dev;
	u32 checksum = 0;
	u32 temp = 0;
	int i = 0;
	int ret = 0;
#ifdef DEBUG
	int w, h, j;
#endif

	ret = read_reg_otp(client, base_addr,
			   4, &otp_ptr->pdaf_data.size);
	checksum += otp_ptr->pdaf_data.size;
	base_addr += 4;
	ret |= read_reg_otp(client, base_addr,
			    2, &otp_ptr->pdaf_data.version);
	checksum += otp_ptr->pdaf_data.version;
	base_addr += 2;
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.gainmap_width);
	checksum += otp_ptr->pdaf_data.gainmap_width;
	base_addr += 1;
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.gainmap_height);
	checksum += otp_ptr->pdaf_data.gainmap_height;
	base_addr += 1;
	for (i = 0; i < RK_GAINMAP_SIZE; i++) {
		ret |= read_reg_otp(client, base_addr,
				    1, &otp_ptr->pdaf_data.gainmap[i]);
		checksum += otp_ptr->pdaf_data.gainmap[i];
		base_addr += 1;
	}
#ifdef DEBUG
	w = 64;
	h = 32;
	dev_info(dev, "show pdaf gainmap table\n");
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++)
			dev_info(dev, "%d ", otp_ptr->pdaf_data.gainmap[i * w + j]);
		if (i < h)
			dev_info(dev, "\n");
	}
#endif
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.gainmap_checksum);
	checksum += otp_ptr->pdaf_data.gainmap_checksum;
	base_addr += 1;
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.dcc_mode);
	checksum += otp_ptr->pdaf_data.dcc_mode;
	base_addr += 1;
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.dcc_dir);
	checksum += otp_ptr->pdaf_data.dcc_dir;
	base_addr += 1;
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.dccmap_width);
	checksum += otp_ptr->pdaf_data.dccmap_width;
	base_addr += 1;
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.dccmap_height);
	checksum += otp_ptr->pdaf_data.dccmap_height;
	base_addr += 1;
	for (i = 0; i < RK_DCCMAP_SIZE; i++) {
		ret |= read_reg_otp(client, base_addr,
				    1, &otp_ptr->pdaf_data.dccmap[i]);
		checksum += otp_ptr->pdaf_data.dccmap[i];
		base_addr += 1;
	}
#ifdef DEBUG
	w = 32;
	h = 16;
	dev_info(dev, "show pdaf dccmap table\n");
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++)
			dev_info(dev, "%d ", otp_ptr->pdaf_data.dccmap[i * w + j]);
		if (i < h)
			dev_info(dev, "\n");
	}
#endif
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.dccmap_checksum);
	checksum += otp_ptr->pdaf_data.dccmap_checksum;
	base_addr += 1;
	for (i = 0; i < RK_PDAF_RESERVED_SIZE; i++) {
		ret |= read_reg_otp(client, base_addr,
				    1, &temp);
		checksum += temp;
		base_addr += 1;
	}
	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->pdaf_data.checksum);
	if ((checksum % 255 + 1) == otp_ptr->pdaf_data.checksum && (!ret)) {
		otp_ptr->pdaf_data.flag = 0x01;
		otp_ptr->flag++;
		dev_info(dev, "pdaf info:(version 0x%x, checksum 0x%x)\n",
			 otp_ptr->pdaf_data.version,
			 (int)otp_ptr->pdaf_data.checksum);
	} else {
		otp_ptr->pdaf_data.flag = 0x00;
		dev_info(dev, "pdaf info: checksum err, checksum %d, reg_checksum %d\n",
			 (int)(checksum % 255 + 1),
			 (int)otp_ptr->pdaf_data.checksum);
	}
}

static void rkotp_read_af(struct eeprom_device *eeprom_dev,
			  struct otp_info *otp_ptr,
			  u32 base_addr)
{
	struct i2c_client *client = eeprom_dev->client;
	struct device *dev = &eeprom_dev->client->dev;
	u32 checksum = 0;
	u32 temp = 0;
	int i = 0;
	int ret = 0;

	ret = read_reg_otp(client, base_addr,
			   4, &otp_ptr->af_data.size);
	checksum += otp_ptr->af_data.size;
	base_addr += 4;
	ret |= read_reg_otp(client, base_addr,
			    2, &otp_ptr->af_data.version);
	checksum += otp_ptr->af_data.version;
	base_addr += 2;
	ret |= read_reg_otp(client, base_addr,
			    2, &otp_ptr->af_data.af_inf);
	checksum += otp_ptr->af_data.af_inf;
	base_addr += 2;
	ret |= read_reg_otp(client, base_addr,
			    2, &otp_ptr->af_data.af_macro);
	checksum += otp_ptr->af_data.af_macro;
	base_addr += 2;
	for (i = 0; i < RK_AF_RESERVED_SIZE; i++) {
		ret |= read_reg_otp(client, base_addr,
				    1, &temp);
		checksum += temp;
		base_addr += 1;
	}

	ret |= read_reg_otp(client, base_addr,
			    1, &otp_ptr->af_data.checksum);
	if ((checksum % 255 + 1) == otp_ptr->af_data.checksum && (!ret)) {
		otp_ptr->af_data.flag = 0x01;
		otp_ptr->flag++;
		dev_info(dev, "af info:(version 0x%x, checksum 0x%x)\n",
			 otp_ptr->af_data.version,
			 (int)otp_ptr->af_data.checksum);
	} else {
		otp_ptr->af_data.flag = 0x00;
		dev_info(dev, "af info: checksum err, checksum %d, reg_checksum %d\n",
			 (int)(checksum % 255 + 1),
			 (int)otp_ptr->af_data.checksum);
	}

}

static int rkotp_read_data(struct eeprom_device *eeprom_dev)
{
	struct i2c_client *client = eeprom_dev->client;
	struct otp_info *otp_ptr;
	struct device *dev = &eeprom_dev->client->dev;
	u32 id = 0;
	u32 base_addr = 0;
	int i = 0;
	int ret = 0;

	otp_ptr = kzalloc(sizeof(*otp_ptr), GFP_KERNEL);
	if (!otp_ptr)
		return -ENOMEM;
	base_addr = RKOTP_REG_START;
	otp_ptr->flag = 0;
	for (i = 0; i < RKOTP_MAX_MODULE; i++) {
		enable_otp_read(client, base_addr);
		read_reg_otp(client, OTP_READ_REG, 1, &id);
		dev_info(dev, "show block id %d, addr 0x%x\n", id, base_addr);
		switch (id) {
		case RKOTP_INFO_ID:
			rkotp_read_module_info(eeprom_dev,
					       otp_ptr,
					       base_addr + 1);
			base_addr += 0x28;//v1 0x30 v2 0x28;
			break;
		case RKOTP_AWB_ID:
			rkotp_read_awb(eeprom_dev,
				       otp_ptr,
				       base_addr + 1);
			base_addr += 0x30;
			break;
		case RKOTP_LSC_ID:
			rkotp_read_lsc(eeprom_dev,
				       otp_ptr,
				       base_addr + 1);
			base_addr += 0x930;
			break;
		case RKOTP_PDAF_ID:
			rkotp_read_pdaf(eeprom_dev,
					otp_ptr,
					base_addr + 1);
			base_addr += 0xA30;
			break;
		case RKOTP_AF_ID:
			rkotp_read_af(eeprom_dev,
				      otp_ptr,
				      base_addr + 1);
			base_addr += 0x20;
			break;
		default:
			id = -1;
			break;
		}
		if (id == -1)
			break;
	}
	if (otp_ptr->flag) {
		eeprom_dev->otp = otp_ptr;
		dev_info(dev, "rkotp read successful!\n");
	} else {
		eeprom_dev->otp = NULL;
		kfree(otp_ptr);
		dev_warn(&client->dev, "otp is NULL!\n");
		ret = -1;
	}
	return ret;
}

static int otp_read(struct eeprom_device *eeprom_dev)
{
	u8 vendor_flag = 0;
	struct i2c_client *client = eeprom_dev->client;

	vendor_flag = get_vendor_flag(client);
	dev_info(&client->dev, "get_vendor_flag %x\n", vendor_flag);
	if (vendor_flag == 0x80)
		otp_read_data(eeprom_dev);
	else if (vendor_flag == 0x40)
		rkotp_read_data(eeprom_dev);
	else {
		dev_warn(&client->dev, "no vendor flag infos!\n");
		return -1;
	}
	return 0;
}

static long eeprom_ioctl(struct v4l2_subdev *sd,
			 unsigned int cmd, void *arg)
{
	struct eeprom_device *eeprom_dev =
		sd_to_eeprom(sd);
	long ret = 0;

	if (!eeprom_dev->otp) {
		if (otp_read(eeprom_dev))
			ret = -EFAULT;
	}
	if (arg && eeprom_dev->otp)
		memcpy(arg, eeprom_dev->otp,
		       sizeof(struct otp_info));
	else
		ret = -EFAULT;
	return ret;
}

#ifdef CONFIG_PROC_FS
static int otp_eeprom_show(struct seq_file *p, void *v)
{
	struct eeprom_device *dev = p->private;
	int i = 0;
	int j = 0;
	u32 gainmap_w, gainmap_h;
	u32 dccmap_w, dccmap_h;

	if (dev && dev->otp) {
		seq_puts(p, "[Header]\n");
		seq_puts(p, "version=1.0;\n\n");

		if (dev->otp->awb_data.flag) {
			seq_puts(p, "[RKAWBOTPParam]\n");
			seq_printf(p, "flag=%d;\n", dev->otp->awb_data.flag);
			seq_printf(p, "r_value=%d;\n", dev->otp->awb_data.r_ratio);
			seq_printf(p, "b_value=%d;\n", dev->otp->awb_data.b_ratio);
			seq_printf(p, "gr_value=%d;\n", dev->otp->awb_data.g_ratio);
			seq_puts(p, "gb_value=-1;\n");
			seq_printf(p, "golden_r_value=%d;\n", dev->otp->awb_data.r_golden);
			seq_printf(p, "golden_b_value=%d;\n", dev->otp->awb_data.b_golden);
			seq_printf(p, "golden_gr_value=%d;\n", dev->otp->awb_data.g_golden);
			seq_puts(p, "golden_gb_value=-1;\n\n");

			seq_puts(p, "[RKLSCOTPParam]\n");
			seq_printf(p, "flag=%d;\n", dev->otp->lsc_data.flag);
			seq_printf(p, "width=%d;\n", dev->otp->basic_data.size.width);
			seq_printf(p, "height=%d;\n", dev->otp->basic_data.size.height);
			seq_printf(p, "tablesize=%d;\n\n", dev->otp->lsc_data.table_size);

			seq_puts(p, "lsc_r_table=\n");
			for (i = 0; i < 17; i++) {
				for (j = 0; j < 17; j++) {
					seq_printf(p, "%d",
						  (dev->otp->lsc_data.data[(i * 17 + j) * 2] << 8) |
						   dev->otp->lsc_data.data[(i * 17 + j) * 2 + 1]);
					if (j < 16)
						seq_puts(p, " ");
				}
				if (i < 16)
					seq_puts(p, "\n");
			}
			seq_puts(p, "\n\n");
		}

		if (dev->otp->lsc_data.flag) {
			seq_puts(p, "lsc_b_table=\n");
			for (i = 0; i < 17; i++) {
				for (j = 0; j < 17; j++) {
					seq_printf(p, "%d",
						   (dev->otp->lsc_data.data[(i * 17 + j) * 2 + 1734]
						    << 8) | dev->otp->lsc_data.data[(i * 17 + j) *
						   2 + 1735]);
					if (j < 16)
						seq_puts(p, " ");
				}
				if (i < 16)
					seq_puts(p, "\n");
			}
			seq_puts(p, "\n\n");

			seq_puts(p, "lsc_gr_table=\n");
			for (i = 0; i < 17; i++) {
				for (j = 0; j < 17; j++) {
					seq_printf(p, "%d",
						   (dev->otp->lsc_data.data[(i * 17 + j) * 2 + 578]
						   << 8) | dev->otp->lsc_data.data[(i * 17 + j) *
						   2 + 579]);
					if (j < 16)
						seq_puts(p, " ");
				}
				if (i < 16)
					seq_puts(p, "\n");
			}
			seq_puts(p, "\n\n");

			seq_puts(p, "lsc_gb_table=\n");
			for (i = 0; i < 17; i++) {
				for (j = 0; j < 17; j++) {
					seq_printf(p, "%d",
						   (dev->otp->lsc_data.data[(i * 17 + j) * 2 + 1156]
						   << 8) | dev->otp->lsc_data.data[(i * 17 + j) *
						   2 + 1157]);
					if (j < 16)
						seq_puts(p, " ");
				}
				if (i < 16)
					seq_puts(p, "\n");
			}
			seq_puts(p, "\n\n");
		}
		if (dev->otp->pdaf_data.flag) {
			gainmap_w = dev->otp->pdaf_data.gainmap_width;
			gainmap_h = dev->otp->pdaf_data.gainmap_height;
			dccmap_w = dev->otp->pdaf_data.dccmap_width;
			dccmap_h = dev->otp->pdaf_data.dccmap_height;
			seq_puts(p, "[RKPDAFOTPParam]\n");
			seq_printf(p, "flag=%d;\n", dev->otp->pdaf_data.flag);
			seq_printf(p, "gainmap_width=%d;\n", gainmap_w);
			seq_printf(p, "gainmap_height=%d;\n", gainmap_h);

			seq_puts(p, "gainmap_table=\n");
			for (i = 0; i < gainmap_h; i++) {
				for (j = 0; j < gainmap_w; j++) {
					u32 val =
						dev->otp->pdaf_data.gainmap[(i * gainmap_w + j) * 2];
					u32 val1 =
						dev->otp->pdaf_data.gainmap[(i * gainmap_w + j) * 2 + 1];

					seq_printf(p, "%d", (val << 8) | val1);
					if (j < gainmap_w)
						seq_puts(p, " ");
				}
				if (i < gainmap_h)
					seq_puts(p, "\n");
			}
			seq_puts(p, "\n");
			seq_printf(p, "dcc_mode=%d\n", dev->otp->pdaf_data.dcc_mode);
			seq_printf(p, "dcc_dir=%d\n", dev->otp->pdaf_data.dcc_dir);
			seq_printf(p, "dccmap_width=%d\n", dev->otp->pdaf_data.dccmap_width);
			seq_printf(p, "dccmap_height=%d\n", dev->otp->pdaf_data.dccmap_height);
			for (i = 0; i < dccmap_h; i++) {
				for (j = 0; j < dccmap_w; j++) {
					u32 val =
						dev->otp->pdaf_data.dccmap[(i * dccmap_w + j) * 2];
					u32 val1 =
						dev->otp->pdaf_data.dccmap[(i * dccmap_w + j) * 2 + 1];

					seq_printf(p, "%d", (val << 8) | val1);
					if (j < dccmap_w)
						seq_puts(p, " ");
				}
				if (i < dccmap_h)
					seq_puts(p, "\n");
			}
			seq_puts(p, "\n");
		}

		if (dev->otp->af_data.flag) {
			seq_puts(p, "[RKAFOTPParam]\n");
			seq_printf(p, "flag=%d;\n", dev->otp->af_data.flag);
			seq_printf(p, "af_inf=%d;\n", dev->otp->af_data.af_inf);
			seq_printf(p, "af_macro=%d;\n", dev->otp->af_data.af_macro);
		}
	} else {
		seq_puts(p, "otp is null!\n");
	}
	return 0;
}

static int eeprom_open(struct inode *inode, struct file *file)
{
	struct eeprom_device *data = PDE_DATA(inode);

	return single_open(file, otp_eeprom_show, data);
}

static const struct proc_ops ops = {
	.proc_open	= eeprom_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int eeprom_proc_init(struct eeprom_device *dev)
{
	dev->procfs = proc_create_data(dev->name, 0, NULL, &ops, dev);
	if (!dev->procfs)
		return -EINVAL;
	return 0;
}

static void eeprom_proc_cleanup(struct eeprom_device *dev)
{
	if (dev->procfs)
		remove_proc_entry(dev->name, NULL);
	dev->procfs = NULL;
}

#endif

static const struct v4l2_subdev_core_ops eeprom_core_ops = {
	.ioctl = eeprom_ioctl,
};

static const struct v4l2_subdev_ops eeprom_ops = {
	.core = &eeprom_core_ops,
};

static void eeprom_subdev_cleanup(struct eeprom_device *dev)
{
	v4l2_device_unregister_subdev(&dev->sd);
	media_entity_cleanup(&dev->sd.entity);
}

static int eeprom_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct eeprom_device *eeprom_dev;

	dev_info(&client->dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	eeprom_dev = devm_kzalloc(&client->dev,
				  sizeof(*eeprom_dev),
				  GFP_KERNEL);

	if (eeprom_dev == NULL) {
		dev_err(&client->dev, "Probe failed\n");
		return -ENOMEM;
	}
	v4l2_i2c_subdev_init(&eeprom_dev->sd,
			     client, &eeprom_ops);
	eeprom_dev->client = client;
	snprintf(eeprom_dev->name, sizeof(eeprom_dev->name), "%s-%d-%02x",
		 DEVICE_NAME, i2c_adapter_id(client->adapter), client->addr);
	eeprom_proc_init(eeprom_dev);
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	dev_info(&client->dev, "probing successful\n");

	return 0;
}

static int eeprom_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct eeprom_device *eeprom_dev =
		sd_to_eeprom(sd);
	kfree(eeprom_dev->otp);
	pm_runtime_disable(&client->dev);
	eeprom_subdev_cleanup(eeprom_dev);
	eeprom_proc_cleanup(eeprom_dev);

	return 0;
}

static int __maybe_unused eeprom_suspend(struct device *dev)
{
	return 0;
}

static int  __maybe_unused eeprom_resume(struct device *dev)
{
	return 0;
}

static const struct i2c_device_id eeprom_id_table[] = {
	{ DEVICE_NAME, 0 },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(i2c, eeprom_id_table);

static const struct of_device_id eeprom_of_table[] = {
	{ .compatible = "hynix,hi556_eeprom" },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(of, eeprom_of_table);

static const struct dev_pm_ops eeprom_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(eeprom_suspend, eeprom_resume)
	SET_RUNTIME_PM_OPS(eeprom_suspend, eeprom_resume, NULL)
};

static struct i2c_driver eeprom_i2c_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.pm = &eeprom_pm_ops,
		.of_match_table = eeprom_of_table,
	},
	.probe = &eeprom_probe,
	.remove = &eeprom_remove,
	.id_table = eeprom_id_table,
};

module_i2c_driver(eeprom_i2c_driver);

MODULE_DESCRIPTION("OTP driver");
MODULE_LICENSE("GPL");
