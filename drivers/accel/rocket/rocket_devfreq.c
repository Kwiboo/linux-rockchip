// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2025 Collabora Ltd. */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>

#include "rocket_core.h"
#include "rocket_device.h"
#include "rocket_devfreq.h"

static void rocket_devfreq_update_utilization(struct rocket_devfreq *rdevfreq)
{
	ktime_t now, last;

	now = ktime_get();
	last = rdevfreq->time_last_update;

	if (rdevfreq->busy_count > 0)
		rdevfreq->busy_time += ktime_sub(now, last);
	else
		rdevfreq->idle_time += ktime_sub(now, last);

	rdevfreq->time_last_update = now;
}

static int rocket_devfreq_target(struct device *dev, unsigned long *freq,
				 u32 flags)
{
	struct rocket_device *rdev = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	err = dev_pm_opp_set_opp(dev, opp);
	if (!err)
		rdev->devfreq.current_frequency = *freq;

	dev_pm_opp_put(opp);

	return err;
}

static void rocket_devfreq_reset(struct rocket_devfreq *rdevfreq)
{
	rdevfreq->busy_time = 0;
	rdevfreq->idle_time = 0;
	rdevfreq->time_last_update = ktime_get();
}

static int rocket_devfreq_get_dev_status(struct device *dev,
					 struct devfreq_dev_status *status)
{
	struct rocket_device *rdev = dev_get_drvdata(dev);
	struct rocket_devfreq *rdevfreq = &rdev->devfreq;
	unsigned long irqflags;

	status->current_frequency = rdevfreq->current_frequency;

	spin_lock_irqsave(&rdevfreq->lock, irqflags);

	rocket_devfreq_update_utilization(rdevfreq);

	status->total_time = ktime_to_ns(ktime_add(rdevfreq->busy_time,
						   rdevfreq->idle_time));
	status->busy_time = ktime_to_ns(rdevfreq->busy_time);

	rocket_devfreq_reset(rdevfreq);

	spin_unlock_irqrestore(&rdevfreq->lock, irqflags);

	dev_info(dev, "busy %lu total %lu %lu %% freq %lu MHz\n",
		status->busy_time, status->total_time,
		status->total_time ?
			status->busy_time / (status->total_time / 100) : 0,
		status->current_frequency / 1000 / 1000);

	return 0;
}

static struct devfreq_dev_profile rocket_devfreq_profile = {
	.timer = DEVFREQ_TIMER_DELAYED,
	/*
	 * Average utilization over several inference jobs. Typical NPU jobs
	 * are 10-40 ms long and arrive at frame rate (30-60 Hz); with a
	 * window in the same order as the job cadence, sampling aliases
	 * against job boundaries and the measured busy fraction oscillates
	 * wildly (e.g. 18 % vs 55 % on alternating windows for a perfectly
	 * steady pipeline), making simple_ondemand bounce between the
	 * lowest and highest OPPs. A 200 ms window covers 5+ frames and
	 * yields a stable utilization figure.
	 */
	.polling_ms = 200,
	.target = rocket_devfreq_target,
	.get_dev_status = rocket_devfreq_get_dev_status,
};

static int rocket_devfreq_config_clks(struct device *dev,
				   struct opp_table *opp_table,
				   struct dev_pm_opp *opp,
				   void *data, bool scaling_down)
{
	struct rocket_device *rdev = dev_get_drvdata(dev);
	struct devfreq *devfreq = rdev->devfreq.devfreq;
	int err = 0;

	pm_runtime_get_noresume(dev);

	if (!pm_runtime_suspended(dev) || !devfreq || !devfreq->suspend_freq)
		err = dev_pm_opp_config_clks_simple(dev, opp_table, opp,
						    data, scaling_down);

	pm_runtime_put_noidle(dev);

	return err;
}

static int rocket_devfreq_init_opp(struct rocket_core *core)
{
	static const char * const regulator_names[] = { "npu", NULL };
	static const char * const clk_names[] = { "npu", NULL };
	struct dev_pm_opp_config config = {
		.clk_names = clk_names,
		.config_clks = rocket_devfreq_config_clks,
		.regulator_names = regulator_names,
	};
	struct device *dev = core->dev;
	struct rocket_device *rdev = core->rdev;
	int err;

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

	err = devm_pm_opp_set_config(dev, &config);
	if (err && err != -EOPNOTSUPP)
		return dev_err_probe(dev, err, "failed to set OPP config for core %d\n",
				     core->index);

	err = devm_pm_opp_of_add_table(dev);
	if (err == -ENODEV) {
		/* No operating-points-v2 in DT: leave DVFS disabled. */
		return 0;
	}
	if (err)
		return dev_err_probe(dev, err, "failed to add OPP table for core %d\n",
				     core->index);

	rdev->opp_core = core->index;
	dev_info(dev, "OPP enabled, clock rate %lu MHz\n",
		 clk_get_rate(core->clks[0].clk) / 1000 / 1000);

	return 0;
}

int rocket_devfreq_init(struct rocket_core *core)
{
	struct device *dev = core->dev;
	struct rocket_device *rdev = core->rdev;
	struct rocket_devfreq *rdevfreq = &rdev->devfreq;
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	unsigned long cur_freq;
	int err;

	err = rocket_devfreq_init_opp(core);
	if (err)
		return err;

	/*
	 * Only the lead opp_core hosts a devfreq instance, since the OPP
	 * table, the SCMI NPU clock and the npu-supply rail are all shared
	 * between the three cores on RK3588.
	 */
	if (rdev->opp_core != (int)core->index)
		return 0;

	spin_lock_init(&rdevfreq->lock);
	rocket_devfreq_reset(rdevfreq);

	cur_freq = clk_get_rate(core->clks[0].clk);
	rdevfreq->current_frequency = cur_freq;
	rocket_devfreq_profile.initial_freq = cur_freq;

	/*
	 * Thresholds for simple_ondemand, tuned for a latency-oriented
	 * accelerator running frame-paced workloads: jobs arrive at frame
	 * rate, so even a fully-loaded real-time pipeline shows a modest
	 * busy fraction (about 20 % at max frequency for a 30 fps SSD
	 * pipeline). Scale to max as soon as the busy fraction exceeds
	 * 30 %, hold the frequency down to 10 %, and only scale down
	 * proportionally below that. Higher thresholds (e.g. the 90/5
	 * governor defaults or panfrost-like 45/5) trap the NPU at a low
	 * OPP: at 200 MHz the same pipeline plateaus around 44 % busy,
	 * just below such an upthreshold, and never scales up.
	 */
	rdevfreq->gov_data.upthreshold = 30;
	rdevfreq->gov_data.downdifferential = 20;

	devfreq = devm_devfreq_add_device(dev, &rocket_devfreq_profile,
					  DEVFREQ_GOV_SIMPLE_ONDEMAND,
					  &rdevfreq->gov_data);
	if (IS_ERR(devfreq)) {
		dev_err(dev, "Couldn't initialize NPU devfreq: %ld\n",
			PTR_ERR(devfreq));
		return PTR_ERR(devfreq);
	}
	rdevfreq->devfreq = devfreq;

	cooling = devfreq_cooling_em_register(devfreq, NULL);
	if (IS_ERR(cooling))
		dev_info(dev, "Failed to register cooling device\n");
	else
		rdevfreq->cooling = cooling;

	return 0;
}

void rocket_devfreq_fini(struct rocket_device *rdev)
{
	struct rocket_devfreq *rdevfreq = &rdev->devfreq;

	if (rdevfreq->cooling) {
		devfreq_cooling_unregister(rdevfreq->cooling);
		rdevfreq->cooling = NULL;
	}
}

void rocket_devfreq_resume(struct rocket_core *core)
{
	struct rocket_devfreq *rdevfreq = &core->rdev->devfreq;

	if (!rdevfreq->devfreq)
		return;

	/*
	 * The SCMI NPU clock is shared by all three cores, and each core
	 * runtime-suspends independently after its autosuspend delay.
	 * Only resume the devfreq instance when the first core powers up,
	 * and (in rocket_devfreq_suspend) only suspend it when the last
	 * core goes down, so a single idle core cannot clamp the shared
	 * clock to the opp-suspend rate while its siblings are busy.
	 */
	if (atomic_inc_return(&rdevfreq->power_count) > 1)
		return;

	rocket_devfreq_reset(rdevfreq);
	devfreq_resume_device(rdevfreq->devfreq);
}

void rocket_devfreq_suspend(struct rocket_core *core)
{
	struct rocket_devfreq *rdevfreq = &core->rdev->devfreq;

	if (!rdevfreq->devfreq)
		return;

	if (atomic_dec_return(&rdevfreq->power_count) > 0)
		return;

	devfreq_suspend_device(rdevfreq->devfreq);
}

void rocket_devfreq_record_busy(struct rocket_device *rdev)
{
	struct rocket_devfreq *rdevfreq = &rdev->devfreq;
	unsigned long irqflags;

	if (!rdevfreq->devfreq)
		return;

	spin_lock_irqsave(&rdevfreq->lock, irqflags);

	rocket_devfreq_update_utilization(rdevfreq);
	rdevfreq->busy_count++;

	spin_unlock_irqrestore(&rdevfreq->lock, irqflags);
}

void rocket_devfreq_record_idle(struct rocket_device *rdev)
{
	struct rocket_devfreq *rdevfreq = &rdev->devfreq;
	unsigned long irqflags;

	if (!rdevfreq->devfreq)
		return;

	spin_lock_irqsave(&rdevfreq->lock, irqflags);

	rocket_devfreq_update_utilization(rdevfreq);
	WARN_ON(--rdevfreq->busy_count < 0);

	spin_unlock_irqrestore(&rdevfreq->lock, irqflags);
}
