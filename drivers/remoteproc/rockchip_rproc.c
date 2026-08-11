// SPDX-License-Identifier: GPL-2.0-only
/*
 * Rockchip Remote Processor Driver
 *
 * Supports RK3562 (BUS MCU), RK3568 (MCU),
 * RK3588 AP (Cortex-A55 AMP via PSCI/SMC),
 * and RK3588 MCU (PMU Cortex-M0 via CRU/SMC).
 */

#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/rockchip/rockchip_sip.h>
#include <asm/barrier.h>

#include "remoteproc_internal.h"

/* GRF register offsets (platform-specific) */
#define RK3562_SYS_GRF_SOC_CON5		0x0414
#define RK3562_SYS_GRF_SOC_CON6		0x0418

#define RK3568_SYS_GRF_SOC_CON4		0x0510

/* Max extra carveouts (in addition to primary firmware region) */
#define RK_PROC_MAX_CARVEOUTS		2	/* firmware + 2 extras = 3 total */

/** @brief Encodes MCU code address for SMC parameter */
#define RK3588_MCU_CODE_ADDR(load) \
	(0xffff0000 | ((unsigned long long)(load) >> 16))

struct rockchip_rproc;

/**
 * struct rockchip_rproc_funcs - Chip/core-specific callbacks
 * @rproc_start:    Platform-specific start (SMC/GRF/CRU config before reset deassert)
 * @rproc_stop:     Platform-specific stop (SMC CPU_OFF for AP, NULL for others)
 * @rproc_da_to_va: Device address to offset conversion
 */
struct rockchip_rproc_funcs {
	int (*rproc_start)(struct rockchip_rproc *drv);
	int (*rproc_stop)(struct rockchip_rproc *drv);
	u64 (*rproc_da_to_va)(struct rockchip_rproc *drv, u64 da,
			      size_t len, bool *is_iomem);
};

/**
 * struct rockchip_rproc_data - Static per-compatible data
 * @funcs: Callback table bound at compile time via of_device_id.data
 */
struct rockchip_rproc_data {
	const struct rockchip_rproc_funcs *funcs;
};

/**
 * struct rockchip_rproc - Rockchip remoteproc private data
 * @grf:        GRF syscon regmap (via "rockchip,grf" phandle), all platforms
 * @core_reset: Reset control handle (via "resets" property); may be NULL for AP
 * @rproc:      Back-reference to remoteproc core handle
 * @mem_start:  Primary carveout physical start (firmware code/data)
 * @mem_region: Primary carveout ioremap virtual address
 * @mem_size:   Primary carveout size
 * @num_carveouts: Number of carveout regions (1-3)
 * @carveout:     Array of extra carveout regions (share, rpmsg)
 * @funcs:      Chip-specific callback table
 * @cpu_id:     AP core MPIDR (via "rockchip,cpu-id"), e.g. 0x300 for A55 core3
 * @pe_state:   AP core PE state config (via "rockchip,pe-state"), MCU unused
 */
struct rockchip_rproc {
	struct regmap *grf;
	struct reset_control *core_reset;
	struct rproc *rproc;
	phys_addr_t mem_start;
	void __iomem *mem_region;
	size_t mem_size;
	int num_carveouts;
	struct {
		phys_addr_t start;
		void __iomem *region;
		size_t size;
	} carveout[RK_PROC_MAX_CARVEOUTS];
	const struct rockchip_rproc_funcs *funcs;
	u32 cpu_id;
	u32 pe_state;
};

static int rockchip_rproc_start(struct rproc *rproc)
{
	struct rockchip_rproc *drv = rproc->priv;
	int ret;

	if (drv->funcs && drv->funcs->rproc_start) {
		ret = drv->funcs->rproc_start(drv);
		if (ret) {
			dev_err(&rproc->dev, "Platform start failed: %d\n", ret);
			return ret;
		}
	}

	dev_info(&rproc->dev, "Firmware loaded at 0x%llx\n",
		 (unsigned long long)drv->mem_start);

	if (drv->core_reset) {
		ret = reset_control_deassert(drv->core_reset);
		if (ret) {
			dev_err(&rproc->dev, "Reset release failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int rockchip_rproc_stop(struct rproc *rproc)
{
	struct rockchip_rproc *drv = rproc->priv;
	int ret = 0;

	if (drv->funcs && drv->funcs->rproc_stop) {
		ret = drv->funcs->rproc_stop(drv);
		if (ret) {
			dev_err(&rproc->dev, "Platform stop failed: %d\n", ret);
			return ret;
		}
	}

	if (drv->core_reset)
		ret = reset_control_assert(drv->core_reset);

	return ret;
}

static void *rockchip_da_to_va(struct rproc *rproc, u64 da, size_t len,
			       bool *is_iomem)
{
	struct rockchip_rproc *drv = rproc->priv;
	u64 offset;
	void __iomem *region;
	size_t region_size;
	int i;

	if (is_iomem)
		*is_iomem = true;

	/* Check extra carveouts first */
	for (i = 0; i < drv->num_carveouts; i++) {
		if (da >= drv->carveout[i].start &&
		    da - drv->carveout[i].start < drv->carveout[i].size) {
			offset = da - drv->carveout[i].start;
			region = drv->carveout[i].region;
			region_size = drv->carveout[i].size;

			if (len > region_size || offset > region_size - len) {
				dev_err(&rproc->dev,
					"Invalid carveout address: da=0x%llx, size=0x%zx\n",
					da, region_size);
				return NULL;
			}
			return (void *)(region + offset);
		}
	}

	/* DA not in carveout: use primary region */
	region = drv->mem_region;
	region_size = drv->mem_size;

	if (drv->funcs && drv->funcs->rproc_da_to_va)
		offset = drv->funcs->rproc_da_to_va(drv, da, len, is_iomem);
	else
		offset = da;

	if (len > region_size || offset > region_size - len) {
		dev_err(&rproc->dev, "Invalid address: da=0x%llx, size=0x%zx\n",
			da, region_size);
		return NULL;
	}

	return (void *)(region + offset);
}

static const struct rproc_ops rockchip_rproc_ops = {
	.start		= rockchip_rproc_start,
	.stop		= rockchip_rproc_stop,
	.load		= rproc_elf_load_segments,
	.da_to_va	= rockchip_da_to_va,
};

static int rockchip_rproc_parse_memory_regions(struct device *dev,
						struct rockchip_rproc *drv)
{
	struct device_node *np;
	struct resource r;
	int i;
	int ret;

	for (i = 0; i < RK_PROC_MAX_CARVEOUTS + 1; i++) {
		np = of_parse_phandle(dev->of_node, "memory-region", i);
		if (!np)
			break;

		ret = of_address_to_resource(np, 0, &r);
		of_node_put(np);
		if (ret)
			return dev_err_probe(dev, ret,
					     "no memory-region specified\n");

		if (i == 0) {
			drv->mem_start = r.start;
			drv->mem_size = resource_size(&r);
			drv->mem_region = devm_ioremap_wc(dev, drv->mem_start,
							 drv->mem_size);
			if (!drv->mem_region)
				return dev_err_probe(dev, -ENOMEM,
						     "unable to map mem region\n");
		} else {
			int idx = i - 1;

			drv->carveout[idx].start = r.start;
			drv->carveout[idx].size = resource_size(&r);
			drv->carveout[idx].region = devm_ioremap_wc(dev,
							drv->carveout[idx].start,
							drv->carveout[idx].size);
			if (!drv->carveout[idx].region)
				return dev_err_probe(dev, -ENOMEM,
						     "unable to map carveout %d\n",
						     i);
			drv->num_carveouts++;
		}
	}

	if (i == 0)
		return dev_err_probe(dev, -ENODEV,
				     "no memory-region specified\n");

	return 0;
}

static int rockchip_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_rproc *drv;
	struct rproc *rproc;
	const struct rockchip_rproc_data *rproc_data;
	int ret;

	rproc = devm_rproc_alloc(dev, dev_name(dev),
			&rockchip_rproc_ops, NULL, sizeof(*drv));
	if (!rproc)
		return -ENOMEM;

	drv = rproc->priv;
	drv->rproc = rproc;
	platform_set_drvdata(pdev, rproc);

	rproc_data = of_device_get_match_data(dev);
	if (rproc_data)
		drv->funcs = rproc_data->funcs;

	/* Primary region is firmware, extras are share/rpmsg carveouts. */
	ret = rockchip_rproc_parse_memory_regions(dev, drv);
	if (ret)
		return ret;

	/* Get reset control (optional for AP core) */
	drv->core_reset = devm_reset_control_array_get(dev, false, false);
	if (IS_ERR(drv->core_reset)) {
		ret = PTR_ERR(drv->core_reset);
		if (ret == -EPROBE_DEFER)
			return ret;
		/* Optional for AP scenarios without resets in DTS */
		dev_dbg(dev, "No reset control found, continuing without\n");
		drv->core_reset = NULL;
	}

	/* Get GRF regmap (optional for AP core that doesn't use GRF) */
	drv->grf = syscon_regmap_lookup_by_phandle(dev->of_node, "rockchip,grf");
	if (IS_ERR(drv->grf)) {
		ret = PTR_ERR(drv->grf);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_dbg(dev, "No rockchip,grf phandle, continuing without\n");
		drv->grf = NULL;
	}

	/* Parse AP-specific DTS properties */
	of_property_read_u32(dev->of_node, "rockchip,cpu-id", &drv->cpu_id);
	of_property_read_u32(dev->of_node, "rockchip,pe-state", &drv->pe_state);

	rproc->auto_boot = false;

	/* Validate required properties based on compatible string */
	if (of_device_is_compatible(dev->of_node, "rockchip,rk3588-rproc-ap")) {
		if (!of_property_read_bool(dev->of_node, "rockchip,cpu-id")) {
			ret = dev_err_probe(dev, -EINVAL,
				"Missing required 'rockchip,cpu-id' property for AP core\n");
			return ret;
		}
	} else if (of_device_is_compatible(dev->of_node, "rockchip,rk3588-rproc-mcu")) {
		if (!drv->core_reset) {
			ret = dev_err_probe(dev, -ENODEV,
				"Missing required 'resets' property for MCU core\n");
			return ret;
		}
	}

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;

	dev_dbg(dev, "Rockchip remote proc probe success\n");

	return 0;
}

/* ==================== rk3562 platform callbacks ==================== */

static int rk3562_rproc_start(struct rockchip_rproc *drv)
{
	int ret = 0;

	/*
	 * mcu_cache_peripheral_addr
	 * The uncache area ranges from 0x7c00000 to 0xffb400000
	 * and contains rpmsg shared memory
	 */
	if (!drv->grf)
		return -ENODEV;

	regmap_write(drv->grf, RK3562_SYS_GRF_SOC_CON5, 0x07c00000);
	regmap_write(drv->grf, RK3562_SYS_GRF_SOC_CON6, 0xffb40000);

	ret = sip_smc_mcu_config(RK_SIP_CFG_BUSMCU_0_ID,
			  CONFIG_MCU_CODE_START_ADDR,
			  0xffff0000 | ((unsigned long long)drv->mem_start >> 16));
	return ret;
}

static u64 rk3562_da_to_va(struct rockchip_rproc *drv, u64 da,
			    size_t len, bool *is_iomem)
{
	return da;
}

static const struct rockchip_rproc_funcs rk3562_rproc_funcs = {
	.rproc_start          = rk3562_rproc_start,
	.rproc_da_to_va       = rk3562_da_to_va,
};

static const struct rockchip_rproc_data rk3562_rproc = {
	.funcs = &rk3562_rproc_funcs,
};

/* ==================== rk3568 platform callbacks ==================== */

static int rk3568_rproc_start(struct rockchip_rproc *drv)
{
	if (!drv->grf)
		return -ENODEV;

	regmap_write(drv->grf, RK3568_SYS_GRF_SOC_CON4,
		     0xffff0000 | ((unsigned long long)drv->mem_start >> 16));
	return 0;
}

static u64 rk3568_da_to_va(struct rockchip_rproc *drv, u64 da,
			    size_t len, bool *is_iomem)
{
	return da - drv->mem_start;
}

static const struct rockchip_rproc_funcs rk3568_rproc_funcs = {
	.rproc_start          = rk3568_rproc_start,
	.rproc_da_to_va       = rk3568_da_to_va,
};

static const struct rockchip_rproc_data rk3568_rproc = {
	.funcs = &rk3568_rproc_funcs,
};

/* ==================== rk3588 AP platform callbacks ==================== */

/**
 * rk3588_ap_rproc_start - RK3588 AP core (A55) start sequence
 * @drv: rockchip_rproc private data
 *
 * Configures CPU mode via SMC and brings up the AMP core using PSCI CPU_ON.
 *
 * Return: 0 on success, negative errno on failure
 */
static int rk3588_ap_rproc_start(struct rockchip_rproc *drv)
{
	int ret;

	/* Configure CPU mode */
	ret = sip_smc_amp_config(RK_AMP_SUB_FUNC_CFG_MODE,
				 drv->cpu_id, drv->pe_state, 0);
	if (ret) {
		dev_err(&drv->rproc->dev,
			"CFG_MODE failed, ret=%d\n", ret);
		return ret;
	}

	/* Set boot args */
	ret = sip_smc_amp_config(RK_AMP_SUB_FUNC_BOOT_ARG01,
				 drv->cpu_id, 0, 0);
	if (ret)
		dev_warn(&drv->rproc->dev,
			 "BOOT_ARG01 failed: %d\n", ret);
	ret = sip_smc_amp_config(RK_AMP_SUB_FUNC_BOOT_ARG23,
				 drv->cpu_id, 0, 0);
	if (ret)
		dev_warn(&drv->rproc->dev,
			 "BOOT_ARG23 failed: %d\n", ret);

	/*
	 * Firmware is written via memcpy_toio to WC-mapped memory, so it
	 * never enters the D-cache. A write barrier suffices to ensure the
	 * data is visible to the remote core before CPU_ON.
	 */
	wmb();

	/*
	 * Bring up remote CPU via RK AMP SMC CPU_ON (bypasses OP-TEE).
	 * Standard PSCI is intercepted by OP-TEE which may not route
	 * the entry address correctly for AMP cores.
	 */
	ret = sip_smc_amp_config(RK_AMP_SUB_FUNC_CPU_ON,
				 drv->cpu_id, drv->mem_start, 0);
	if (ret) {
		dev_err(&drv->rproc->dev,
			"AMP CPU_ON failed for cpu 0x%x, ret=%d\n",
			drv->cpu_id, ret);
		return ret;
	}

	dev_info(&drv->rproc->dev,
		 "AP core (cpu_id=0x%x) started at 0x%llx\n",
		 drv->cpu_id, (unsigned long long)drv->mem_start);

	return 0;
}

/**
 * rk3588_ap_rproc_stop - RK3588 AP core (A55) stop sequence
 * @drv: rockchip_rproc private data
 *
 * Requests CPU off via SMC. Best-effort: failure does not block shutdown.
 *
 * Return: 0 on success, negative errno on failure (logged as warning)
 */
static int rk3588_ap_rproc_stop(struct rockchip_rproc *drv)
{
	int ret;

	ret = sip_smc_amp_config(RK_AMP_SUB_FUNC_REQ_CPU_OFF,
				 drv->cpu_id, 0, 0);
	if (ret)
		dev_warn(&drv->rproc->dev,
			 "request cpu off for 0x%x failed, ret=%d\n",
			 drv->cpu_id, ret);

	return ret;
}

static u64 rk3588_ap_da_to_va(struct rockchip_rproc *drv, u64 da,
			       size_t len, bool *is_iomem)
{
	return da - drv->mem_start;
}

static const struct rockchip_rproc_funcs rk3588_ap_rproc_funcs = {
	.rproc_start    = rk3588_ap_rproc_start,
	.rproc_stop     = rk3588_ap_rproc_stop,
	.rproc_da_to_va = rk3588_ap_da_to_va,
};

static const struct rockchip_rproc_data rk3588_ap_rproc = {
	.funcs = &rk3588_ap_rproc_funcs,
};

/* ==================== rk3588 MCU platform callbacks ==================== */

/**
 * rk3588_mcu_rproc_start - RK3588 MCU core (PMU M0) start sequence
 * @drv: rockchip_rproc private data
 *
 * Sequence:
 *   Step 0: assert PMU M0 core reset so the MCU is halted.
 *   Step 1: set the MCU code start address via SMC.
 *
 * The reset deassert (releasing the core) is performed by the framework
 * in rockchip_rproc_start() after this callback returns.
 *
 * Return: 0 on success, negative errno on failure
 */
static int rk3588_mcu_rproc_start(struct rockchip_rproc *drv)
{
	int ret;

	if (!drv->core_reset) {
		dev_err(&drv->rproc->dev, "Reset control not available\n");
		return -ENODEV;
	}

	/* Assert PMU M0 reset so the MCU is halted while we
	 * configure the code start address.
	 */
	ret = reset_control_assert(drv->core_reset);
	if (ret) {
		dev_err(&drv->rproc->dev, "Reset assert failed: %d\n", ret);
		return ret;
	}

	/* Set MCU code start address */
	ret = sip_smc_mcu_config(RK_SIP_CFG_PMUMCU_0_ID,
				 CONFIG_MCU_CODE_START_ADDR,
				 RK3588_MCU_CODE_ADDR(drv->mem_start));
	if (ret) {
		dev_err(&drv->rproc->dev,
			"SMC set code addr failed: %d\n", ret);
		return ret;
	}

	return 0;
}

/**
 * rk3588_mcu_rproc_stop - RK3588 MCU core (PMU M0) stop sequence
 * @drv: rockchip_rproc private data
 *
 * The PMU M0 core reset assert is performed by the framework in
 * rockchip_rproc_stop() after this callback returns, so there is
 * nothing platform-specific left to do here.
 *
 * Return: 0 on success, negative errno on failure
 */
static int rk3588_mcu_rproc_stop(struct rockchip_rproc *drv)
{
	return 0;
}

static u64 rk3588_mcu_da_to_va(struct rockchip_rproc *drv, u64 da,
				size_t len, bool *is_iomem)
{
	return da;
}

static const struct rockchip_rproc_funcs rk3588_mcu_rproc_funcs = {
	.rproc_start    = rk3588_mcu_rproc_start,
	.rproc_stop     = rk3588_mcu_rproc_stop,
	.rproc_da_to_va = rk3588_mcu_da_to_va,
};

static const struct rockchip_rproc_data rk3588_mcu_rproc = {
	.funcs = &rk3588_mcu_rproc_funcs,
};

/* ==================== of_device_id table ==================== */
static const struct of_device_id rockchip_rproc_of_match[] = {
	{ .compatible = "rockchip,rk3562-rproc",     .data = &rk3562_rproc },
	{ .compatible = "rockchip,rk3568-rproc",     .data = &rk3568_rproc },
	{ .compatible = "rockchip,rk3588-rproc-ap",  .data = &rk3588_ap_rproc },
	{ .compatible = "rockchip,rk3588-rproc-mcu", .data = &rk3588_mcu_rproc },
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_rproc_of_match);

static struct platform_driver rockchip_rproc_driver = {
	.probe = rockchip_rproc_probe,
	.driver = {
		.name = "rockchip-rproc",
		.of_match_table = rockchip_rproc_of_match,
	},
};

module_platform_driver(rockchip_rproc_driver);

MODULE_AUTHOR("Jing Wu <jing.wu@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip Remote Processor Driver");
MODULE_LICENSE("GPL");
