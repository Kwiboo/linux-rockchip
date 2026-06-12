/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2025 Collabora Ltd. */

#ifndef __ROCKET_DEVFREQ_H__
#define __ROCKET_DEVFREQ_H__

#include <linux/devfreq.h>
#include <linux/ktime.h>
#include <linux/spinlock.h>

struct devfreq;
struct rocket_core;
struct rocket_device;
struct thermal_cooling_device;

/*
 * Per-device dynamic frequency scaling state. The three NPU cores in RK3588
 * share a single SCMI clock and npu-supply rail, so we expose a single
 * devfreq instance (attached to the lead opp_core's device) and aggregate
 * job-utilization tracking across all cores.
 */
struct rocket_devfreq {
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	struct devfreq_simple_ondemand_data gov_data;

	unsigned long current_frequency;

	/*
	 * Number of runtime-resumed cores. The devfreq instance is suspended
	 * (which also clamps the shared clock to the opp-suspend rate) only
	 * when the last core runtime-suspends, and resumed when the first
	 * core comes back up. Per-core suspends must not disturb the shared
	 * clock while sibling cores are still executing jobs.
	 */
	atomic_t power_count;

	/* Protects busy_time, idle_time, time_last_update and busy_count. */
	spinlock_t lock;
	ktime_t busy_time;
	ktime_t idle_time;
	ktime_t time_last_update;
	int busy_count;
};

int rocket_devfreq_init(struct rocket_core *core);
void rocket_devfreq_fini(struct rocket_device *rdev);

void rocket_devfreq_resume(struct rocket_core *core);
void rocket_devfreq_suspend(struct rocket_core *core);

void rocket_devfreq_record_busy(struct rocket_device *rdev);
void rocket_devfreq_record_idle(struct rocket_device *rdev);

#endif /* __ROCKET_DEVFREQ_H__ */
