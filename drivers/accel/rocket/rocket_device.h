/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2024-2025 Tomeu Vizoso <tomeu@tomeuvizoso.net> */

#ifndef __ROCKET_DEVICE_H__
#define __ROCKET_DEVICE_H__

#include <drm/drm_device.h>
#include <linux/clk.h>
#include <linux/container_of.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>

#include "rocket_core.h"
#include "rocket_devfreq.h"

struct rocket_device {
	struct drm_device ddev;

	struct mutex sched_lock;

	struct rocket_core *cores;
	unsigned int num_cores;

	/*
	 * All NPU cores share the same SCMI clock and the same npu-supply on
	 * the SoCs we currently support (RK3588). Only the first-probed core
	 * sets up the OPP table and drives the shared rail; sibling cores rely
	 * on that shared state. opp_core is the index of the core that owns the
	 * OPP, or -1 if no core has OPP configured.
	 */
	int opp_core;

	/* Shared devfreq state, hosted by the lead opp_core's device. */
	struct rocket_devfreq devfreq;
};

struct rocket_device *rocket_device_init(struct platform_device *pdev,
					 const struct drm_driver *rocket_drm_driver);
void rocket_device_fini(struct rocket_device *rdev);
#define to_rocket_device(drm_dev) \
	((struct rocket_device *)(container_of((drm_dev), struct rocket_device, ddev)))

#endif /* __ROCKET_DEVICE_H__ */
