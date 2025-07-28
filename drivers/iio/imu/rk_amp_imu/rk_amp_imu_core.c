// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Rockchip Electronics Co., Ltd.
 */

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include "rk_amp_imu.h"

static int rk_amp_imu_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask);

static int rk_amp_imu_write_raw_get_fmt(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					long mask);

static int rk_amp_imu_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask);

static int rk_amp_imu_temp_read_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int *val, int *val2, long mask);

#define RK_AMP_IMU_CHANNEL(_type, _axis, _index) {		\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = _axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_BE,				\
	},							\
}

#define RK_AMP_IMU_TEMP_CHANNEL_16BIT(_type, _index) {			\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = 1,					\
	.info_mask_separate =					\
		BIT(IIO_CHAN_INFO_RAW) |			\
		BIT(IIO_CHAN_INFO_OFFSET) |			\
		BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_BE,				\
	},							\
}

static const struct iio_chan_spec rk_amp_imu_channels[] = {
	RK_AMP_IMU_CHANNEL(IIO_ACCEL, IIO_MOD_X, RK_AMP_IMU_SCAN_ACCEL_X),
	RK_AMP_IMU_CHANNEL(IIO_ACCEL, IIO_MOD_Y, RK_AMP_IMU_SCAN_ACCEL_Y),
	RK_AMP_IMU_CHANNEL(IIO_ACCEL, IIO_MOD_Z, RK_AMP_IMU_SCAN_ACCEL_Z),
	RK_AMP_IMU_CHANNEL(IIO_ANGL_VEL, IIO_MOD_X, RK_AMP_IMU_SCAN_GYRO_X),
	RK_AMP_IMU_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Y, RK_AMP_IMU_SCAN_GYRO_Y),
	RK_AMP_IMU_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Z, RK_AMP_IMU_SCAN_GYRO_Z),
	RK_AMP_IMU_TEMP_CHANNEL_16BIT(IIO_TEMP, RK_AMP_IMU_SCAN_TEMP),
	IIO_CHAN_SOFT_TIMESTAMP(RK_AMP_IMU_SCAN_TIMESTAMP),
};

static const unsigned long rk_amp_imu_scan_masks[] = {
	/* 6-axis accel + gyro + temp */
	BIT(RK_AMP_IMU_SCAN_ACCEL_X)
		| BIT(RK_AMP_IMU_SCAN_ACCEL_Y)
		| BIT(RK_AMP_IMU_SCAN_ACCEL_Z)
		| BIT(RK_AMP_IMU_SCAN_GYRO_X)
		| BIT(RK_AMP_IMU_SCAN_GYRO_Y)
		| BIT(RK_AMP_IMU_SCAN_GYRO_Z)
		| BIT(RK_AMP_IMU_SCAN_TEMP),
	0,
};

static struct attribute *rk_amp_imu_attrs[] = {
	NULL,
};

static const struct attribute_group rk_amp_imu_attrs_group = {
	.attrs = rk_amp_imu_attrs,
};

static const struct iio_info rk_amp_imu_info = {
	.read_raw = rk_amp_imu_read_raw,
	.write_raw = rk_amp_imu_write_raw,
	.write_raw_get_fmt = &rk_amp_imu_write_raw_get_fmt,
	.attrs = &rk_amp_imu_attrs_group,
};

struct rk_amp_imu_odr {
	u8 bits;
	int odr;
	int divider;		//us
};

struct rk_amp_imu_odr_item {
	const struct rk_amp_imu_odr *tbl;
	int num;
};

struct rk_amp_imu_scale {
	u8 bits;
	int scale;
	int uscale;
};

struct rk_amp_imu_scale_item {
	const struct rk_amp_imu_scale *tbl;
	int num;
	int format;
};

static const struct regmap_config rk_amp_imu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int rk_amp_imu_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	int ret = -EINVAL;

	switch (chan->type) {
	case IIO_ANGL_VEL:
	case IIO_ACCEL:
		break;
	case IIO_TEMP:
		return rk_amp_imu_temp_read_raw(indio_dev, chan, val, val2, mask);
	default:
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
		// Fixed range, set by MCU
		if (chan->type == IIO_ANGL_VEL) {
			*val = 0;
			*val2 = 1064225;
			ret = IIO_VAL_INT_PLUS_NANO;
		} else if (chan->type == IIO_ACCEL) {
			*val = 0;
			*val2 = 4785;
			ret = IIO_VAL_INT_PLUS_MICRO;
		}
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return -EINVAL;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rk_amp_imu_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	return 0;
}

static int rk_amp_imu_write_raw_get_fmt(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return IIO_VAL_INT_PLUS_MICRO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int rk_amp_imu_temp_read_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int *val, int *val2, long mask)
{
	struct rk_amp_imu_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = data->enable_fifo ? 500000 : 7812500;
		return data->enable_fifo ? IIO_VAL_INT_PLUS_MICRO : IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_OFFSET:
		*val = data->enable_fifo ? 50 : 3200;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const char *rk_amp_imu_match_acpi_device(struct device *dev)
{
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return NULL;

	return dev_name(dev);
}

static void rk_amp_imu_chip_uninit(struct rk_amp_imu_data *data)
{
	// TODO, send signal to mcu for uninit.
}

static void *rkamp_sharedmem_get_va(struct rkamp_sharedmem_node *node)
{
	if (!node)
		return ERR_PTR(-EINVAL);

	return node->dma_va;
}

static int rk_amp_imu_chip_init(struct rk_amp_imu_data *data, rk_amp_imu_bus_setup bus_setup)
{
	struct device *dev = regmap_get_device(data->regmap);
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct resource *res;

	dev_info(dev, "is_reserved_irq, skip chip init, use amp\n");
	data->rdev = rkamp_sharedmem_get_device("rpmsg-sharedmem-mcu");
	if (!data->rdev) {
		pr_info("%s %d: failed to get sharedmem device\n", __func__, __LINE__);
		return -EPROBE_DEFER;
	}

	data->dev = dev;

	data->imu_handle = rkamp_sharedmem_register(data->rdev, IMU_DATA_SIZE,
						"rk-amp-imu", sizeof("rk-amp-imu"), NULL, NULL);
	if (IS_ERR(data->imu_handle)) {
		pr_info("%s %d: failed to connect to mcu\n", __func__, __LINE__);
		return -ENODEV;
	}
	if (!data->imu_handle) {
		pr_info("%s %d: failed to connect to mcu\n", __func__, __LINE__);
		return -ENODEV;
	}

	data->imu_virt_addr = rkamp_sharedmem_get_va(data->imu_handle);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	data->mbox_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->mbox_base))
		return PTR_ERR(data->mbox_base);

	writel_relaxed((1U << BIT_WRITEABLE_SHIFT | MAILBOX_V2_INT_MASK),
		data->mbox_base + MAILBOX_V2_B2A_INTEN);

	return 0;
}

static irqreturn_t rk_amp_imu_do_nothing(int irq, void *private)
{
	return IRQ_HANDLED;
}

static int rk_amp_imu_core_probe(struct regmap *regmap,
			  int irq, const char *name,
			  int chip_type, rk_amp_imu_bus_setup bus_setup)
{
	struct iio_dev *indio_dev;
	struct rk_amp_imu_data *data;
	struct device *dev = regmap_get_device(regmap);
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	memset(data, 0, sizeof(struct rk_amp_imu_data));
	mutex_init(&data->lock);
	data->chip_type = chip_type;
	data->powerup_count = 0;
	data->irq = irq;
	data->regmap = regmap;

	/* get the node */
	data->node = of_find_node_by_name(NULL, "rk-amp-imu");
	if (data->node == NULL)
		dev_err(dev, "rk-amp-imu node not find!\n");

	data->is_reserved_irq = of_property_read_bool(data->node, "use-reserved-irq");
	if (data->is_reserved_irq)
		dev_info(dev, "is_reserved_irq!\n");

	ret = rk_amp_imu_chip_init(data, bus_setup);
	if (ret < 0) {
		dev_err(dev, "rk_amp_imu_chip_init fail\n");
		return ret;
	}

	dev_set_drvdata(dev, indio_dev);
	if (!name && ACPI_HANDLE(dev))
		name = rk_amp_imu_match_acpi_device(dev);

	indio_dev->dev.parent = dev;
	indio_dev->channels = rk_amp_imu_channels;
	indio_dev->num_channels = ARRAY_SIZE(rk_amp_imu_channels);
	indio_dev->available_scan_masks = rk_amp_imu_scan_masks;
	indio_dev->name = name;
	indio_dev->info = &rk_amp_imu_info;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
			rk_amp_imu_do_nothing, NULL, NULL);
	if (ret < 0)
		goto uninit;

	ret = rk_amp_imu_probe_trigger(indio_dev);
	if (ret) {
		dev_err(dev, "trigger probe fail %d\n", ret);
		goto uninit;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0) {
		dev_err(dev, "devm_iio_device_register fail\n");
		goto uninit;
	}

	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;

	return 0;

uninit:
	rk_amp_imu_chip_uninit(data);
	return ret;
}

/*
 * System resume gets the system back on and restores the sensors state.
 * Manually put runtime power management in system active state.
 */
static int __maybe_unused sleep_rk_amp_imu_resume(struct device *dev)
{
	int ret = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	usleep_range(3000, 4000);

	ret = rk_amp_imu_set_enable(indio_dev, 1);

	return ret;
}

/*
 * Suspend saves sensors state and turns everything off.
 * Check first if runtime suspend has not already done the job.
 */
static int __maybe_unused sleep_rk_amp_imu_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int ret = 0;

	ret = rk_amp_imu_set_enable(indio_dev, 0);

	return ret;
}

static int rk_amp_imu_platform_bus_setup(struct rk_amp_imu_data *st)
{
	return 0;
}

static int rk_amp_imu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk_amp_imu_data *data;
	struct regmap *regmap;
	int irq;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	dev_info(dev, "irq = %d\n", irq);

	regmap = devm_regmap_init(dev, NULL, data, &rk_amp_imu_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return rk_amp_imu_core_probe(regmap, irq, "rk-amp-imu", 0, rk_amp_imu_platform_bus_setup);
}

static const struct of_device_id rk_amp_imu_of_match[] = {
	{ .compatible = "rockchip,rk-amp-imu", },
	{},
};
MODULE_DEVICE_TABLE(of, rk_amp_imu_of_match);

static struct platform_driver rk_amp_imu_core_driver = {
	.probe = rk_amp_imu_probe,
	.driver = {
		.name = "rk_amp_imu_core_driver",
		.of_match_table = rk_amp_imu_of_match,
	},
};

module_platform_driver(rk_amp_imu_core_driver);

MODULE_AUTHOR("Fenrir Lin <fenrir.lin@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip AMP IMU driver");
MODULE_LICENSE("GPL");
