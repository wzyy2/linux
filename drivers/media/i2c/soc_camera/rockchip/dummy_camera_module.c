/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Jacob Chen <jacob-chen@iotwrt.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include <linux/gcd.h>
#include <media/v4l2-subdev.h>

#include "dummy_camera_module.h"

static struct dummy_camera_module dummy_priv;

static struct dummy_module_config dummy_configs[] = {
	{
		.name = "1920x1080_60fps",
		.frm_fmt = {
			.width = 1920,
			.height = 1080,
			.code = MEDIA_BUS_FMT_RGB888_1X24
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 60
			}
		},
	}
};

static struct dummy_camera_module *to_dummy_camera_module(struct v4l2_subdev *sd)
{
	return container_of(sd, struct dummy_camera_module, sd);
}

int dummy_camera_module_g_ctrl(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return 0;
}

int dummy_camera_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return 0;
}

int dummy_camera_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	return 0;
}

int dummy_camera_module_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

long dummy_camera_module_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	int ret = 0;

	// ret = pltfrm_camera_module_ioctl(sd, cmd, arg);

	return ret;
}

int dummy_camera_module_s_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *interval)
{
	struct dummy_camera_module *cam_mod =  to_dummy_camera_module(sd);
	unsigned long gcdiv;
	struct v4l2_subdev_frame_interval norm_interval;
	int ret = 0;

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "%d/%d (%dfps)\n",
		interval->interval.numerator, interval->interval.denominator,
		(interval->interval.denominator +
		(interval->interval.numerator >> 1)) /
		interval->interval.numerator);

	/* normalize interval */
	gcdiv = gcd(interval->interval.numerator,
		interval->interval.denominator);
	norm_interval.interval.numerator =
		interval->interval.numerator / gcdiv;
	norm_interval.interval.denominator =
		interval->interval.denominator / gcdiv;

	cam_mod->frm_intrvl = norm_interval;

	return ret;
}

int dummy_camera_module_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	return ret;
}

int dummy_camera_module_enum_frameintervals(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= 1)
		return -EINVAL;

	fie->code = dummy_configs[fie->index].frm_fmt.code;
	fie->width = dummy_configs[fie->index].frm_fmt.width;
	fie->height = dummy_configs[fie->index].frm_fmt.height;
	fie->interval.numerator = dummy_configs[fie->index].frm_intrvl.interval.numerator;
	fie->interval.denominator = dummy_configs[fie->index].frm_intrvl.interval.denominator;

	return 0;
}

int dummy_camera_module_g_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	struct dummy_camera_module *cam_mod =  to_dummy_camera_module(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;

	*fmt = cam_mod->frm_fmt;

	return 0;
}

int dummy_camera_module_s_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	struct dummy_camera_module *cam_mod =  to_dummy_camera_module(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;

	cam_mod->frm_fmt = *fmt;

	return 0;
}

static struct v4l2_subdev_core_ops dummy_camera_module_core_ops = {
	.g_ctrl = dummy_camera_module_g_ctrl,
	.s_ctrl = dummy_camera_module_s_ctrl,
	.s_ext_ctrls = dummy_camera_module_s_ext_ctrls,
	.s_power = dummy_camera_module_s_power,
	.ioctl = dummy_camera_module_ioctl
};

static struct v4l2_subdev_video_ops dummy_camera_module_video_ops = {
	.s_frame_interval = dummy_camera_module_s_frame_interval,
	.s_stream = dummy_camera_module_s_stream
};

static struct v4l2_subdev_pad_ops dummy_camera_module_pad_ops = {
	.enum_frame_interval = dummy_camera_module_enum_frameintervals,
	.get_fmt = dummy_camera_module_g_fmt,
	.set_fmt = dummy_camera_module_s_fmt,
};

static struct v4l2_subdev_ops dummy_camera_module_ops = {
	.core = &dummy_camera_module_core_ops,
	.video = &dummy_camera_module_video_ops,
	.pad = &dummy_camera_module_pad_ops
};

static int dummy_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");
	v4l2_i2c_subdev_init(&dummy_priv.sd, client, &dummy_camera_module_ops);

	dev_info(&client->dev, "probing successful\n");

	return 0;
}

static int dummy_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	dev_info(&client->dev, "removed\n");

	return 0;
}

static const struct i2c_device_id dummy_rockchip_soc_camera_id[] = {
	{ "dummy camera", 0 },
	{},
};

static const struct of_device_id dummy_rockchip_soc_camera_of_match[] = {
	{.compatible = "rockchip,dummy-soc-camera"},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov8858_id);

static struct i2c_driver dummy_rockchip_soc_camera_i2c_driver = {
	.driver = {
		.name = "dummy camera",
		.owner = THIS_MODULE,
		.of_match_table = dummy_rockchip_soc_camera_of_match
	},
	.probe = dummy_probe,
	.remove = dummy_remove,
	.id_table = dummy_rockchip_soc_camera_id,
};

module_i2c_driver(dummy_rockchip_soc_camera_i2c_driver);

MODULE_DESCRIPTION("Dummy Rockchip SOC Camera driver");
MODULE_AUTHOR("jacob2.chen@rock-chips.com");
MODULE_LICENSE("GPL");
