/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 */

#ifndef RK_AMP_IMU_H_
#define RK_AMP_IMU_H_

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "../../../soc/rockchip/rockchip_amp_rpmsg_sharedmem.h"

#define RK_AMP_IMU_OUTPUT_DATA_SIZE 24 // align 8, last 8 for timestamp
#define RK_AMP_IMU_OUTPUT_DATA_SIZE_PULS_ONE 25
#define RK_AMP_IMU_DATA_BUFF_SIZE 960

#define IIO_TRIGGER 1
#define RK_AMP_IMU_RESET_FLAG 1
#define RK_AMP_IMU_DEBUG_FLAG 0

// fifo
#define RK_AMP_IMU_ACCEL_GYRO_SIZE 12
#define RK_AMP_IMU_FIFO_DATUM 16
#define RK_AMP_IMU_BYTES_PER_3AXIS_SENSOR 6
#define RK_AMP_IMU_FIFO_COUNT_BYTE 2
#define RK_AMP_IMU_BYTE_FIFO_TEMP 1
#define RK_AMP_IMU_FIFO_SIZE 1024

struct rk_amp_imu_chip_config {
	unsigned int fsr:2;
	unsigned int lpf:3;
	unsigned int accl_fs:2;
	int gyro_odr:10;
	int accel_odr:10;
	unsigned int accl_fifo_enable:1;
	unsigned int gyro_fifo_enable:1;
	unsigned int temp_fifo_enable:1;
	unsigned int time_fifo_enable:1;
	u8 divider;
	u8 user_ctrl;
};

#define MAILBOX_V2_B2A_INTEN           0x10
#define MAILBOX_V2_B2A_STATUS          0x14
#define MAILBOX_V2_B2A_CMD             0x18
#define MAILBOX_V2_B2A_DAT             0x1c

#define MAILBOX_V2_TRIGGER_SHIFT       8
#define MAILBOX_V2_TRIGGER_MASK        BIT(8)
#define MAILBOX_V2_INT_MASK            BIT(0)
#define MAILBOX_V2_INT_CLR             BIT(0)

#define MAILBOX_POLLING_MS             5 /* default polling interval 5ms */
#define BIT_WRITEABLE_SHIFT            16

// struct rk_amp_imu_data *
struct rkamp_3d_data {
	uint32_t X;
	uint32_t Y;
	uint32_t Z;
} __packed;

struct rkamp_imu_data {
	uint32_t temp;
	struct rkamp_3d_data accel;
	struct rkamp_3d_data gyro;
} __packed;

struct rkamp_imu_shared_data {
	uint32_t tag;
	uint32_t index;
	uint64_t timestamps_ns;
	struct rkamp_imu_data imu_data;
} __packed;

#define IMU_DATA_SIZE sizeof(struct rkamp_imu_shared_data)

struct rk_amp_imu_data {
	struct mutex lock;
	struct regmap *regmap;
	struct iio_trigger  *trig;
	struct device_node	*node;
	struct gpio_desc *int1_gpiod;
	struct regulator *vdd_supply;
	struct regulator *vddio_supply;
	u16  accel_frequency;
	u16  gyro_frequency;
	u16  accel_frequency_buff;
	u16  gyro_frequency_buff;
	u16  accel_lpf_bw;
	u16  gyro_lpf_bw;
	u16  accel_lpf_bw_buff;
	u16  gyro_lpf_bw_buff;
	int irq;
	u8 irq_mask;
	int chip_type; // not used
	unsigned int powerup_count;
	struct rk_amp_imu_chip_config chip_config;
	int skip_samples;
	s64 it_timestamp;		// Timestamp of when the data was read
	s64 data_timestamp;		 // Timestamp of when the data was generated
	s64 standard_period;	 // Standard interrupt period in nanoseconds
	s64 interrupt_period;	// Actual interrupt period in nanoseconds
	s64 period_min;		// Minimum interrupt period deviation in nanoseconds
	s64 period_max;		 // Maximum interrupt period deviation in nanoseconds
	int period_divider;
	int interrupt_regval;
	bool enable_fifo;
	u8  data_buff[RK_AMP_IMU_DATA_BUFF_SIZE];

	bool is_reserved_irq;
	void *imu_handle;
	struct device *rdev;
	struct device *dev;
	void __iomem *mbox_base;
	void *imu_virt_addr;
	bool enable;
};

/* scan indexes follow DATA register order */
enum rk_amp_imu_scan_axis {
	RK_AMP_IMU_SCAN_ACCEL_X = 0,
	RK_AMP_IMU_SCAN_ACCEL_Y,
	RK_AMP_IMU_SCAN_ACCEL_Z,
	RK_AMP_IMU_SCAN_GYRO_X,
	RK_AMP_IMU_SCAN_GYRO_Y,
	RK_AMP_IMU_SCAN_GYRO_Z,
	RK_AMP_IMU_SCAN_TEMP,
	RK_AMP_IMU_SCAN_TIMESTAMP,
};

enum rk_amp_imu_sensor_type {
	RK_AMP_IMU_ACCEL = 0,
	RK_AMP_IMU_GYRO,
	RK_AMP_IMU_TEMP,
	RK_AMP_IMU_TIMESTAMP,
	RK_AMP_IMU_NUM_SENSORS /* must be last */
};

typedef int (*rk_amp_imu_bus_setup)(struct rk_amp_imu_data *);

int rk_amp_imu_probe_trigger(struct iio_dev *indio_dev);
int rk_amp_imu_set_enable(struct iio_dev *indio_dev, bool enable);

#endif
