// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2024-2025 Tomeu Vizoso <tomeu@tomeuvizoso.net> */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "rocket_device.h"
#include "rocket_core.h"
#include "rocket_job.h"

static const char * const rocket_opp_regulators[] = { "npu" };

static int rocket_core_init_opp(struct rocket_core *core)
{
	struct device *dev = core->dev;
	struct rocket_device *rdev = core->rdev;
	struct dev_pm_opp *opp;
	unsigned long freq = ULONG_MAX;
	int err;

	core->max_freq = 0;
	core->suspend_freq = 0;

	/*
	 * On RK3588 all three NPU cores share the same SCMI NPU clock and the
	 * same npu-supply rail. The OPP framework can only have one device
	 * configure the (shared) OPP table; configuring it from a second core
	 * after the first has populated the OPPs returns -EBUSY. Let only the
	 * first-probed core set up OPP, and let sibling cores ride along on the
	 * clock/voltage state programmed by that lead core.
	 */
	if (rdev->opp_core != -1)
		return 0;

	err = devm_pm_opp_set_clkname(dev, "npu");
	if (err)
		return dev_err_probe(dev, err, "failed to set OPP clock for core %d\n",
				     core->index);

	err = devm_pm_opp_set_regulators(dev, rocket_opp_regulators);
	if (err && err != -ENOTSUPP)
		return dev_err_probe(dev, err, "failed to set OPP regulators for core %d\n",
				     core->index);

	err = devm_pm_opp_of_add_table(dev);
	if (err == -ENODEV) {
		/* No operating-points-v2 in DT: leave DVFS disabled. */
		return 0;
	}
	if (err)
		return dev_err_probe(dev, err, "failed to add OPP table for core %d\n",
				     core->index);

	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		return dev_err_probe(dev, PTR_ERR(opp),
				     "failed to find highest OPP for core %d\n", core->index);
	dev_pm_opp_put(opp);
	core->max_freq = freq;

	/*
	 * On RK3588 the nputop power domain refuses to ack a power-on
	 * transition while the SCMI NPU clock is on the PVTPLL path (any
	 * rate >= 300 MHz, per BL31's rk3588_clk.c). The OPP marked with
	 * "opp-suspend" in DT is the rate that bypasses the PVTPLL
	 * (200 MHz on RK3588) and must be programmed before letting the
	 * power domain power down.
	 */
	core->suspend_freq = dev_pm_opp_get_suspend_opp_freq(dev);
	if (!core->suspend_freq) {
		dev_warn(dev, "no opp-suspend in OPP table; runtime suspend may fail to power-cycle nputop\n");
		core->suspend_freq = core->max_freq;
	}

	rdev->opp_core = core->index;
	dev_info(dev, "OPP enabled, max %lu MHz, suspend %lu MHz\n",
		 core->max_freq / 1000000, core->suspend_freq / 1000000);

	return 0;
}

int rocket_core_init(struct rocket_core *core)
{
	struct device *dev = core->dev;
	struct platform_device *pdev = to_platform_device(dev);
	u32 version;
	int err = 0;

	core->resets[0].id = "srst_a";
	core->resets[1].id = "srst_h";
	err = devm_reset_control_bulk_get_exclusive(&pdev->dev, ARRAY_SIZE(core->resets),
						    core->resets);
	if (err)
		return dev_err_probe(dev, err, "failed to get resets for core %d\n", core->index);

	err = devm_clk_bulk_get(dev, ARRAY_SIZE(core->clks), core->clks);
	if (err)
		return dev_err_probe(dev, err, "failed to get clocks for core %d\n", core->index);

	err = rocket_core_init_opp(core);
	if (err)
		return err;

	core->pc_iomem = devm_platform_ioremap_resource_byname(pdev, "pc");
	if (IS_ERR(core->pc_iomem)) {
		dev_err(dev, "couldn't find PC registers %ld\n", PTR_ERR(core->pc_iomem));
		return PTR_ERR(core->pc_iomem);
	}

	core->cna_iomem = devm_platform_ioremap_resource_byname(pdev, "cna");
	if (IS_ERR(core->cna_iomem)) {
		dev_err(dev, "couldn't find CNA registers %ld\n", PTR_ERR(core->cna_iomem));
		return PTR_ERR(core->cna_iomem);
	}

	core->core_iomem = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(core->core_iomem)) {
		dev_err(dev, "couldn't find CORE registers %ld\n", PTR_ERR(core->core_iomem));
		return PTR_ERR(core->core_iomem);
	}

	dma_set_max_seg_size(dev, UINT_MAX);

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (err)
		return err;

	core->iommu_group = iommu_group_get(dev);

	err = rocket_job_init(core);
	if (err) {
		iommu_group_put(core->iommu_group);
		core->iommu_group = NULL;
		return err;
	}

	pm_runtime_use_autosuspend(dev);

	/*
	 * As this NPU will be most often used as part of a media pipeline that
	 * ends presenting in a display, choose 50 ms (~3 frames at 60Hz) as an
	 * autosuspend delay as that will keep the device powered up while the
	 * pipeline is running.
	 */
	pm_runtime_set_autosuspend_delay(dev, 50);

	pm_runtime_enable(dev);

	err = pm_runtime_resume_and_get(dev);
	if (err) {
		rocket_core_fini(core);
		return err;
	}

	version = rocket_pc_readl(core, VERSION);
	version += rocket_pc_readl(core, VERSION_NUM) & 0xffff;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	dev_info(dev, "Rockchip NPU core %d version: %d\n", core->index, version);

	return 0;
}

void rocket_core_fini(struct rocket_core *core)
{
	pm_runtime_dont_use_autosuspend(core->dev);
	pm_runtime_disable(core->dev);
	iommu_group_put(core->iommu_group);
	core->iommu_group = NULL;
	rocket_job_fini(core);
}

void rocket_core_reset(struct rocket_core *core)
{
	reset_control_bulk_assert(ARRAY_SIZE(core->resets), core->resets);

	udelay(10);

	reset_control_bulk_deassert(ARRAY_SIZE(core->resets), core->resets);
}
