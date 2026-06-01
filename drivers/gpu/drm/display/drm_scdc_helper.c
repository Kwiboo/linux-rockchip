/*
 * Copyright (c) 2015 NVIDIA Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/delay.h>
#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_print.h>

#include <drm/display/drm_scdc_helper.h>

/**
 * DOC: scdc helpers
 *
 * Status and Control Data Channel (SCDC) is a mechanism introduced by the
 * HDMI 2.0 specification. It is a point-to-point protocol that allows the
 * HDMI source and HDMI sink to exchange data. The same I2C interface that
 * is used to access EDID serves as the transport mechanism for SCDC.
 *
 * Note: The SCDC status is going to be lost when the display is
 * disconnected. This can happen physically when the user disconnects
 * the cable, but also when a display is switched on (such as waking up
 * a TV).
 *
 * This is further complicated by the fact that, upon a disconnection /
 * reconnection, KMS won't change the mode on its own. This means that
 * one can't just rely on setting the SCDC status on enable, but also
 * has to track the connector status changes using interrupts and
 * restore the SCDC status. The typical solution for this is to trigger an
 * empty modeset in drm_connector_helper_funcs.detect_ctx(), like what vc4 does
 * in vc4_hdmi_reset_link(). Alternatively, use the HDMI connector framework
 * which ensures drm_scdc_sync_status() is called in the context of
 * drm_atomic_helper_connector_hdmi_hotplug_ctx().
 */

#define SCDC_I2C_SLAVE_ADDRESS		0x54
#define SCDC_MAX_SOURCE_VERSION		0x1
#define SCDC_STATUS_POLL_DELAY_MS	1000

#define drm_scdc_dbg(connector, fmt, ...)					\
	drm_dbg_kms((connector)->dev, "[CONNECTOR:%d:%s] " fmt,			\
		    (connector)->base.id, (connector)->name, ##__VA_ARGS__)

/**
 * drm_scdc_read - read a block of data from SCDC
 * @adapter: I2C controller
 * @offset: start offset of block to read
 * @buffer: return location for the block to read
 * @size: size of the block to read
 *
 * Reads a block of data from SCDC, starting at a given offset.
 *
 * Returns:
 * 0 on success, negative error code on failure.
 */
ssize_t drm_scdc_read(struct i2c_adapter *adapter, u8 offset, void *buffer,
		      size_t size)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = SCDC_I2C_SLAVE_ADDRESS,
			.flags = 0,
			.len = 1,
			.buf = &offset,
		}, {
			.addr = SCDC_I2C_SLAVE_ADDRESS,
			.flags = I2C_M_RD,
			.len = size,
			.buf = buffer,
		}
	};

	ret = i2c_transfer(adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EPROTO;

	return 0;
}
EXPORT_SYMBOL(drm_scdc_read);

/**
 * drm_scdc_write - write a block of data to SCDC
 * @adapter: I2C controller
 * @offset: start offset of block to write
 * @buffer: block of data to write
 * @size: size of the block to write
 *
 * Writes a block of data to SCDC, starting at a given offset.
 *
 * Returns:
 * 0 on success, negative error code on failure.
 */
ssize_t drm_scdc_write(struct i2c_adapter *adapter, u8 offset,
		       const void *buffer, size_t size)
{
	struct i2c_msg msg = {
		.addr = SCDC_I2C_SLAVE_ADDRESS,
		.flags = 0,
		.len = 1 + size,
		.buf = NULL,
	};
	void *data;
	int err;

	data = kmalloc(1 + size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	msg.buf = data;

	memcpy(data, &offset, sizeof(offset));
	memcpy(data + 1, buffer, size);

	err = i2c_transfer(adapter, &msg, 1);

	kfree(data);

	if (err < 0)
		return err;
	if (err != 1)
		return -EPROTO;

	return 0;
}
EXPORT_SYMBOL(drm_scdc_write);

/**
 * drm_scdc_get_scrambling_status - what is status of scrambling?
 * @connector: connector
 *
 * Reads the scrambler status over SCDC, and checks the
 * scrambling status.
 *
 * Returns:
 * True if the scrambling is enabled, false otherwise.
 */
bool drm_scdc_get_scrambling_status(struct drm_connector *connector)
{
	u8 status;
	int ret;

	ret = drm_scdc_readb(connector->ddc, SCDC_SCRAMBLER_STATUS, &status);
	if (ret < 0) {
		drm_scdc_dbg(connector, "Failed to read scrambling status: %d\n", ret);
		return false;
	}

	return status & SCDC_SCRAMBLING_STATUS;
}
EXPORT_SYMBOL(drm_scdc_get_scrambling_status);

/**
 * drm_scdc_set_scrambling - enable scrambling
 * @connector: connector
 * @enable: bool to indicate if scrambling is to be enabled/disabled
 *
 * Writes the TMDS config register over SCDC channel, and:
 * enables scrambling when enable = 1
 * disables scrambling when enable = 0
 *
 * Returns:
 * True if scrambling is set/reset successfully, false otherwise.
 */
bool drm_scdc_set_scrambling(struct drm_connector *connector,
			     bool enable)
{
	u8 config;
	int ret;

	ret = drm_scdc_readb(connector->ddc, SCDC_TMDS_CONFIG, &config);
	if (ret < 0) {
		drm_scdc_dbg(connector, "Failed to read TMDS config: %d\n", ret);
		return false;
	}

	if (enable)
		config |= SCDC_SCRAMBLING_ENABLE;
	else
		config &= ~SCDC_SCRAMBLING_ENABLE;

	ret = drm_scdc_writeb(connector->ddc, SCDC_TMDS_CONFIG, config);
	if (ret < 0) {
		drm_scdc_dbg(connector, "Failed to enable scrambling: %d\n", ret);
		return false;
	}

	return true;
}
EXPORT_SYMBOL(drm_scdc_set_scrambling);

/**
 * drm_scdc_set_high_tmds_clock_ratio - set TMDS clock ratio
 * @connector: connector
 * @set: ret or reset the high clock ratio
 *
 *
 *	TMDS clock ratio calculations go like this:
 *		TMDS character = 10 bit TMDS encoded value
 *
 *		TMDS character rate = The rate at which TMDS characters are
 *		transmitted (Mcsc)
 *
 *		TMDS bit rate = 10x TMDS character rate
 *
 *	As per the spec:
 *		TMDS clock rate for pixel clock < 340 MHz = 1x the character
 *		rate = 1/10 pixel clock rate
 *
 *		TMDS clock rate for pixel clock > 340 MHz = 0.25x the character
 *		rate = 1/40 pixel clock rate
 *
 *	Writes to the TMDS config register over SCDC channel, and:
 *		sets TMDS clock ratio to 1/40 when set = 1
 *
 *		sets TMDS clock ratio to 1/10 when set = 0
 *
 * Returns:
 * True if write is successful, false otherwise.
 */
bool drm_scdc_set_high_tmds_clock_ratio(struct drm_connector *connector,
					bool set)
{
	u8 config;
	int ret;

	ret = drm_scdc_readb(connector->ddc, SCDC_TMDS_CONFIG, &config);
	if (ret < 0) {
		drm_scdc_dbg(connector, "Failed to read TMDS config: %d\n", ret);
		return false;
	}

	if (set)
		config |= SCDC_TMDS_BIT_CLOCK_RATIO_BY_40;
	else
		config &= ~SCDC_TMDS_BIT_CLOCK_RATIO_BY_40;

	ret = drm_scdc_writeb(connector->ddc, SCDC_TMDS_CONFIG, config);
	if (ret < 0) {
		drm_scdc_dbg(connector, "Failed to set TMDS clock ratio: %d\n", ret);
		return false;
	}

	/*
	 * The spec says that a source should wait minimum 1ms and maximum
	 * 100ms after writing the TMDS config for clock ratio. Lets allow a
	 * wait of up to 2ms here.
	 */
	usleep_range(1000, 2000);
	return true;
}
EXPORT_SYMBOL(drm_scdc_set_high_tmds_clock_ratio);

static int drm_scdc_try_scrambling_setup(struct drm_connector *connector)
{
	bool done;

	done = drm_scdc_set_high_tmds_clock_ratio(connector, true);
	if (!done)
		return -EIO;

	done = drm_scdc_set_scrambling(connector, true);
	if (!done)
		return -EIO;

	if (READ_ONCE(connector->hdmi.scrambler_enabled))
		schedule_delayed_work(&connector->hdmi.scdc_work,
				      msecs_to_jiffies(SCDC_STATUS_POLL_DELAY_MS));

	return 0;
}

static void drm_scdc_monitor_scrambler(struct drm_connector *connector)
{
	if (READ_ONCE(connector->hdmi.scrambler_enabled) &&
	    !drm_scdc_get_scrambling_status(connector))
		drm_scdc_try_scrambling_setup(connector);
}

static int drm_scdc_reset_crtc(struct drm_connector *connector,
			       struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_device *dev = connector->dev;
	struct drm_crtc *crtc;
	u8 config;
	int ret;

	if (!ctx)
		return 0;

	/*
	 * This is normally part of .detect_ctx() call path, which already holds
	 * connection_mutex through @ctx.  However, re-acquiring it with the
	 * same context is a no-op and makes the helper safe under any caller.
	 */
	ret = drm_modeset_lock(&dev->mode_config.connection_mutex, ctx);
	if (ret)
		return ret;

	if (!connector->state)
		return 0;

	crtc = connector->state->crtc;
	if (!crtc)
		return 0;

	ret = drm_scdc_readb(connector->ddc, SCDC_TMDS_CONFIG, &config);
	if (ret) {
		drm_scdc_dbg(connector, "Failed to read TMDS config: %d\n", ret);
		return ret;
	}

	if ((config & SCDC_SCRAMBLING_ENABLE) &&
	    (config & SCDC_TMDS_BIT_CLOCK_RATIO_BY_40))
		return 0;

	/*
	 * Reset the CRTC to suspend TMDS transmission, conforming to HDMI 2.0
	 * spec which requires scrambled data not to be sent before the sink is
	 * configured, and TMDS clock to be suspended while changing the clock
	 * ratio.  The disable/re-enable cycle triggered by the reset should
	 * call drm_scdc_start_scrambling() during re-enable, properly
	 * configuring the sink before data transmission resumes.
	 */

	drm_scdc_dbg(connector, "Resetting CRTC to restore SCDC status\n");

	ret = drm_atomic_helper_reset_crtc(crtc, ctx);
	if (ret && ret != -EDEADLK)
		drm_scdc_dbg(connector, "Failed to reset CRTC: %d\n", ret);

	return ret;
}

/**
 * drm_scdc_start_scrambling - activate scrambling and monitor SCDC status
 * @connector: connector
 *
 * Enables scrambling and high TMDS clock ratio on both source and sink sides.
 * Additionally, use a delayed work item to monitor the scrambling status on
 * the sink side and retry the operation, as some displays refuse to set the
 * scrambling bit right away.
 *
 * Returns:
 * Zero if scrambling is set successfully, an error code otherwise.
 */
int drm_scdc_start_scrambling(struct drm_connector *connector)
{
	struct drm_display_info *info = &connector->display_info;
	struct drm_connector_hdmi *hdmi = &connector->hdmi;
	int ret;
	u8 ver;

	if (!hdmi->scrambler_supported) {
		drm_scdc_dbg(connector, "Scrambler not supported, bailing.\n");
		return -EINVAL;
	}

	if (!info->is_hdmi ||
	    !info->hdmi.scdc.supported ||
	    !info->hdmi.scdc.scrambling.supported) {
		drm_scdc_dbg(connector, "Sink doesn't support scrambling.\n");
		return -EINVAL;
	}

	drm_scdc_dbg(connector, "Enabling scrambling\n");

	ret = drm_scdc_readb(connector->ddc, SCDC_SINK_VERSION, &ver);
	if (ret) {
		drm_scdc_dbg(connector, "Failed to read SCDC_SINK_VERSION: %d\n", ret);
		return ret;
	}

	ret = drm_scdc_writeb(connector->ddc, SCDC_SOURCE_VERSION,
			      min_t(u8, ver, SCDC_MAX_SOURCE_VERSION));
	if (ret) {
		drm_scdc_dbg(connector, "Failed to write SCDC_SOURCE_VERSION: %d\n", ret);
		return ret;
	}

	hdmi->scdc_cb = drm_scdc_monitor_scrambler;
	WRITE_ONCE(hdmi->scrambler_enabled, true);

	ret = drm_scdc_try_scrambling_setup(connector);
	if (!ret)
		ret = hdmi->funcs->scrambler_enable(connector);

	if (ret) {
		WRITE_ONCE(hdmi->scrambler_enabled, false);
		cancel_delayed_work_sync(&hdmi->scdc_work);
		hdmi->scdc_cb = NULL;

		drm_scdc_set_scrambling(connector, false);
		drm_scdc_set_high_tmds_clock_ratio(connector, false);
	}

	return ret;
}
EXPORT_SYMBOL(drm_scdc_start_scrambling);

/**
 * drm_scdc_stop_scrambling - deactivate scrambling and SCDC status monitor
 * @connector: connector
 *
 * Disables scrambling and high TMDS clock ratio on both source and sink sides.
 * Also cancels the SCDC status monitoring work item, if it is still pending.
 *
 * Returns:
 * Zero if scrambling is reset successfully, an error code otherwise.
 */
int drm_scdc_stop_scrambling(struct drm_connector *connector)
{
	struct drm_display_info *info = &connector->display_info;
	struct drm_connector_hdmi *hdmi = &connector->hdmi;

	if (!hdmi->scrambler_supported) {
		drm_scdc_dbg(connector, "Scrambler not supported, bailing.\n");
		return -EINVAL;
	}

	if (!READ_ONCE(hdmi->scrambler_enabled))
		return 0;

	drm_scdc_dbg(connector, "Disabling scrambling\n");

	WRITE_ONCE(hdmi->scrambler_enabled, false);
	cancel_delayed_work_sync(&hdmi->scdc_work);
	hdmi->scdc_cb = NULL;

	if (connector->status == connector_status_connected &&
	    info->is_hdmi && info->hdmi.scdc.supported &&
	    info->hdmi.scdc.scrambling.supported) {
		drm_scdc_set_scrambling(connector, false);
		drm_scdc_set_high_tmds_clock_ratio(connector, false);
	}

	return hdmi->funcs->scrambler_disable(connector);
}
EXPORT_SYMBOL(drm_scdc_stop_scrambling);

/**
 * drm_scdc_sync_status - resync the sink-side SCDC upon reconnect
 * @connector: connector
 * @plugged: connector plugged status event
 * @ctx: initialized lock acquisition context
 *
 * When receiving hotplug disconnect/reconnect event, while the display is
 * still active (CRTC enabled), the SCDC status on the sink side is reset
 * and must be explicitly restored.
 *
 * The typical solution for this is to trigger an empty modeset in
 * drm_connector_helper_funcs.detect_ctx(), which is what this helper does
 * by triggering a CRTC reset on reconnection.
 *
 * When making use of the HDMI connector framework, this is automatically
 * triggered via drm_atomic_helper_connector_hdmi_hotplug_ctx().
 *
 * Returns:
 * Zero on success, an error code otherwise, including -EDEADLK.
 */
int drm_scdc_sync_status(struct drm_connector *connector, bool plugged,
			 struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_connector_hdmi *hdmi = &connector->hdmi;

	if (hdmi->scrambler_supported && plugged &&
	    READ_ONCE(hdmi->scrambler_enabled))
		return drm_scdc_reset_crtc(connector, ctx);

	/* TODO: Also handle HDMI 2.1 FRL link training */

	return 0;
}
EXPORT_SYMBOL(drm_scdc_sync_status);
