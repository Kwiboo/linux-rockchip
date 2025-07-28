// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 */

#include <linux/math64.h>

#include "rk_amp_imu.h"

/**
 *  rk_amp_imu_set_enable() - enable chip functions.
 *  @indio_dev:	Device driver instance.
 *  @enable: enable/disable
 */
int rk_amp_imu_set_enable(struct iio_dev *indio_dev, bool enable)
{
	struct rk_amp_imu_data *data = iio_priv(indio_dev);

	data->enable = enable;
	return 0;
}

/**
 * inv_mpu_data_rdy_trigger_set_state() - set data ready interrupt state
 * @trig: Trigger instance
 * @state: Desired trigger state
 */
static int inv_mpu_data_rdy_trigger_set_state(struct iio_trigger *trig,
						  bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct rk_amp_imu_data *data = iio_priv(indio_dev);
	int result;

	mutex_lock(&data->lock);
	dev_info(regmap_get_device(data->regmap), "in data_rdy_trigger_set_state, %d\n", state);
	result = rk_amp_imu_set_enable(indio_dev, state);
	mutex_unlock(&data->lock);

	return result;
}

static const struct iio_trigger_ops inv_mpu_trigger_ops = {
	.set_trigger_state = &inv_mpu_data_rdy_trigger_set_state,
};

static irqreturn_t rkamp_imu_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct rk_amp_imu_data *st = iio_priv(indio_dev);
	// const struct device *dev = regmap_get_device(st->regmap);
	struct rkamp_imu_shared_data *imu_shared = st->imu_virt_addr;
	u8 data[RK_AMP_IMU_OUTPUT_DATA_SIZE_PULS_ONE];

	if (!st->enable) {
		writel_relaxed(MAILBOX_V2_INT_CLR, st->mbox_base + MAILBOX_V2_B2A_STATUS);
		return IRQ_HANDLED;
	}

	rkamp_sharedmem_invalidate(st->imu_handle, 0, IMU_DATA_SIZE);
	//pr_info("gyro xyz %8d %8d %8d, accel xyz %8d %8d %8d| time = %lld\n",
	//	imu_shared->imu_data.gyro.X, imu_shared->imu_data.gyro.Y,
	//	imu_shared->imu_data.gyro.Z, imu_shared->imu_data.accel.X,
	//	imu_shared->imu_data.accel.Y, imu_shared->imu_data.accel.Z,
	//	imu_shared->timestamps_ns);

	// accel xyz
	data[0] = (uint8_t)(imu_shared->imu_data.accel.X & 0xFF);
	data[1] = (uint8_t)((imu_shared->imu_data.accel.X >> 8) & 0xFF);
	data[2] = (uint8_t)(imu_shared->imu_data.accel.Y & 0xFF);
	data[3] = (uint8_t)((imu_shared->imu_data.accel.Y >> 8) & 0xFF);
	data[4] = (uint8_t)(imu_shared->imu_data.accel.Z & 0xFF);
	data[5] = (uint8_t)((imu_shared->imu_data.accel.Z >> 8) & 0xFF);
	// gyro xyz
	data[6] = (uint8_t)(imu_shared->imu_data.gyro.X & 0xFF);
	data[7] = (uint8_t)((imu_shared->imu_data.gyro.X >> 8) & 0xFF);
	data[8] = (uint8_t)(imu_shared->imu_data.gyro.Y & 0xFF);
	data[9] = (uint8_t)((imu_shared->imu_data.gyro.Y >> 8) & 0xFF);
	data[10] = (uint8_t)(imu_shared->imu_data.gyro.Z & 0xFF);
	data[11] = (uint8_t)((imu_shared->imu_data.gyro.Z >> 8) & 0xFF);
	// temp
	data[12] = (uint8_t)(imu_shared->imu_data.temp & 0xFF);
	data[13] = (uint8_t)((imu_shared->imu_data.temp >> 8) & 0xFF);

	st->data_timestamp = imu_shared->timestamps_ns; // actual is us
	// imu_shared->reg[14] to data
	iio_push_to_buffers_with_timestamp(indio_dev, &(data[0]), st->data_timestamp);
	if (st->trig)
		iio_trigger_notify_done(indio_dev->trig);

	writel_relaxed(MAILBOX_V2_INT_CLR, st->mbox_base + MAILBOX_V2_B2A_STATUS);

	return IRQ_HANDLED;
}

int rk_amp_imu_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct rk_amp_imu_data *data = iio_priv(indio_dev);

	data->trig = devm_iio_trigger_alloc(&indio_dev->dev,
						"%s-dev%d",
						indio_dev->name,
						indio_dev->id);
	if (!data->trig)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, data->irq, &rkamp_imu_irq_handler,
			0, "rk_amp_imu", indio_dev);
	if (ret)
		return ret;

	data->trig->dev.parent = regmap_get_device(data->regmap);
	data->trig->ops = &inv_mpu_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, data->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(data->trig);

	return 0;
}
