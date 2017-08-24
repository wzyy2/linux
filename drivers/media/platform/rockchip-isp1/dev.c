/*
 * Rockchip isp1 driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include "mipi_dphy_sy.h"
#include "regs.h"
#include "rkisp1.h"

struct isp_clk_data {
	const char * const *clks;
	int size;
};

static int rkisp1_create_links(struct rkisp1_device *dev)
{
	int ret, i;
	unsigned int flags = 0;
	struct media_entity *source, *sink;

	/* note: entities source pad link should be
	 * established prior to sink pad link.
	 * isp sink links shoud be established first.
	 */
	for (i = 0; i < dev->num_sensors; i++) {
		if (i == 0)
			flags = MEDIA_LNK_FL_DYNAMIC | MEDIA_LNK_FL_ENABLED;
		else
			flags = MEDIA_LNK_FL_DYNAMIC;
		source = &dev->sensors[i].sd->entity;
		//TODO: find out the sensor source pad instead of hardcode 0
		sink = &dev->subdevs[RKISP1_SD_PHY_CSI]->entity;
		ret = media_entity_create_link(source, 0, sink,
					       MIPI_DPHY_SY_PAD_SINK, flags);
		if (ret < 0)
			return ret;
	}
	/* Make the first sensor enable as default */
	dev->subdevs[RKISP1_SD_SENSOR] = dev->sensors[0].sd;

	flags = MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE;
	source = &dev->subdevs[RKISP1_SD_PHY_CSI]->entity;
	sink = &dev->isp_sdev.sd.entity;
	ret = media_entity_create_link(source, MIPI_DPHY_SY_PAD_SOURCE,
				       sink, RKISP1_ISP_PAD_SINK, flags);
	if (ret < 0)
		return ret;

	ret = media_entity_call(sink, link_setup,
				&sink->pads[RKISP1_ISP_PAD_SINK],
				&source->pads[MIPI_DPHY_SY_PAD_SOURCE], flags);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	/* params links */
	source = &dev->params_vdev.vnode.vdev.entity;
	sink = &dev->isp_sdev.sd.entity;
	ret = media_entity_create_link(source, 0, sink,
				       RKISP1_ISP_PAD_SINK_PARAMS, flags);

	if (ret < 0)
		return ret;
	/* create isp internal links */
	/* SP links */
	source = &dev->isp_sdev.sd.entity;
	sink = &dev->sp_stream.vnode.vdev.entity;
	ret = media_entity_create_link(source, RKISP1_ISP_PAD_SOURCE_PATH,
					sink, 0, flags);
	if (ret < 0)
		return ret;

	/* MP links */
	source = &dev->isp_sdev.sd.entity;
	sink = &dev->mp_stream.vnode.vdev.entity;
	ret = media_entity_create_link(source, RKISP1_ISP_PAD_SOURCE_PATH,
				       sink, 0, flags);
	if (ret < 0)
		return ret;

	/* 3A stats links */
	source = &dev->isp_sdev.sd.entity;
	sink = &dev->stats_vdev.vnode.vdev.entity;
	return media_entity_create_link(source, RKISP1_ISP_PAD_SOURCE_STATS,
					sink, 0, flags);
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct rkisp1_device *dev;
	int ret;

	dev = container_of(notifier, struct rkisp1_device, notifier);

	mutex_lock(&dev->media_dev.graph_mutex);
	ret = rkisp1_create_links(dev);
	if (ret < 0)
		goto unlock;
	ret = v4l2_device_register_subdev_nodes(&dev->v4l2_dev);
	if (ret < 0)
		goto unlock;

	v4l2_info(&dev->v4l2_dev, "Async subdev notifier completed\n");

unlock:
	mutex_unlock(&dev->media_dev.graph_mutex);
	return ret;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct rkisp1_sensor_info *sensor_sd;

	sensor_sd = container_of(asd, struct rkisp1_sensor_info, asd);
	sensor_sd->sd = subdev;
	subdev->grp_id = GRP_ID_SENSOR;

	v4l2_info(subdev, "Async registered subdev\n");

	return 0;
}

static int isp_subdev_notifier(struct rkisp1_device *isp_dev,
			       struct device_node *parent)
{
	struct v4l2_async_notifier *ntf = &isp_dev->notifier;
	struct v4l2_async_subdev *asd;
	struct device *dev = isp_dev->dev;
	struct device_node *node, *pre_node = NULL, *remote = NULL;
	int i;

	ntf->subdevs = devm_kzalloc(dev,
				    RKISP1_MAX_SENSOR * sizeof(*ntf->subdevs),
				    GFP_KERNEL);
	if (!ntf->subdevs)
		return -ENOMEM;

	i = 0;
	while ((node = of_graph_get_next_endpoint(parent, pre_node)) != NULL) {
		struct rkisp1_sensor_info *sensor = &isp_dev->sensors[i];

		of_node_put(pre_node);
		pre_node = node;

		asd = &sensor->asd;
		asd->match_type = V4L2_ASYNC_MATCH_FWNODE;
		remote = of_graph_get_remote_port_parent(node);
		asd->match.fwnode.fwnode = of_fwnode_handle(remote);
		of_node_put(remote);
		if (!asd->match.fwnode.fwnode) {
			dev_err(dev, "Can't get remote endpoint parent\n");
			goto put_node;
		}

		if (v4l2_of_parse_endpoint(node, &sensor->ep)) {
			dev_err(dev, "Can't parse endpoint\n");
			goto put_node;
		}
		ntf->subdevs[i] = asd;
		i++;
	}
	if (i == 0) {
		dev_err(dev, "Can't find the sensor endpoint\n");
		goto put_node;
	}

	isp_dev->num_sensors = i;
	ntf->num_subdevs = i;
	ntf->bound = subdev_notifier_bound;
	ntf->complete = subdev_notifier_complete;

	return v4l2_async_notifier_register(&isp_dev->v4l2_dev, ntf);

put_node:
	of_node_put(pre_node);
	return -EINVAL;
}

static int register_mipidphy_subdev(struct rkisp1_device *isp_dev)
{
	struct media_entity *me;
	struct v4l2_subdev *sd;
	struct device *dev = isp_dev->dev;
	struct platform_device *mipi_pdev;
	struct device_node *of_mipi;
	int ret;

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret < 0) {
		dev_err(dev, "Failed to populate child mipidphy(%d)\n", ret);
		return ret;
	}

	of_mipi = of_get_next_available_child(dev->of_node, NULL);
	if (!of_mipi) {
		dev_err(dev, "Failed to get mipidphy node\n");
		return -EINVAL;
	}
	mipi_pdev = of_find_device_by_node(of_mipi);
	if (!mipi_pdev) {
		dev_err(dev, "Failed to get mipidphy device\n");
		ret = -EINVAL;
		goto err_put_node;
	}

	me = platform_get_drvdata(mipi_pdev);
	if (!me) {
		dev_err(dev, "Deferred for mipidphy device is not probed\n");
		ret = -EPROBE_DEFER;
		goto err_put_node;
	}

	sd = media_entity_to_v4l2_subdev(me);
	ret = v4l2_device_register_subdev(&isp_dev->v4l2_dev, sd);
	if (ret < 0) {
		v4l2_err(&isp_dev->v4l2_dev, "Failed to register dphy sd\n");
		goto err_put_node;
	}

	isp_dev->subdevs[RKISP1_SD_PHY_CSI] = sd;
	sd->grp_id = GRP_ID_MIPIPHY;

	ret = isp_subdev_notifier(isp_dev, of_mipi);
	if (ret < 0) {
		v4l2_err(&isp_dev->v4l2_dev,
			 "Failed to register subdev notifier(%d)\n", ret);
		goto err_unreg_v4l2_subdev;
	}
	of_node_put(of_mipi);

	return 0;
err_unreg_v4l2_subdev:
	v4l2_device_unregister_subdev(sd);
err_put_node:
	of_node_put(of_mipi);
	isp_dev->subdevs[RKISP1_SD_PHY_CSI] = NULL;
	return ret;
}

/*TODO: maybe renamed*/
static int rkisp1_register_platform_subdevs(struct rkisp1_device *dev)
{
	int ret;

	/* register isp v4l2 subdev */
	ret = rkisp1_register_isp_subdev(dev, &dev->v4l2_dev);
	if (ret < 0)
		return ret;
	ret = rkisp1_register_stream_vdevs(dev);
	if (ret < 0)
		goto err_unreg_isp_subdev;
	/* register 3A stats vdev */
	ret = rkisp1_register_stats_vdev(&dev->stats_vdev, &dev->v4l2_dev, dev);
	if (ret < 0)
		goto err_unreg_stream_vdev;

	/* register ISP params vdev */
	ret = rkisp1_register_params_vdev(&dev->params_vdev, &dev->v4l2_dev,
					  dev);
	if (ret < 0)
		goto err_unreg_stats_vdev;

	/* register mipi dev */
	ret = register_mipidphy_subdev(dev);
	if (ret < 0)
		goto err_unreg_params_vdev;

	return 0;
err_unreg_params_vdev:
	rkisp1_unregister_params_vdev(&dev->params_vdev);
err_unreg_stats_vdev:
	rkisp1_unregister_stats_vdev(&dev->stats_vdev);
err_unreg_stream_vdev:
	rkisp1_unregister_stream_vdevs(dev);
err_unreg_isp_subdev:
	rkisp1_unregister_isp_subdev(dev);
	return ret;
}

static const char * const rk3399_isp_clks[] = {
	"clk-isp",
	"aclk-isp-noc",
	"aclk-isp-wrapper",
	"hclk-isp-noc",
	"hclk-isp-wrapper",
};

static const char * const rk3288_isp_clks[] = {
	"clk-isp",
	"hclk_isp",
	"sclk_isp",
	"sclk_isp_jpe",
	"mipi_csi",
	"pclk_isp_in",
	"sclk_mipidsi_24m",
};

const struct isp_clk_data rk3288_isp_clk_data = {
	.clks = rk3288_isp_clks,
	.size = ARRAY_SIZE(rk3288_isp_clks),
};

const struct isp_clk_data rk3399_isp_clk_data = {
	.clks = rk3399_isp_clks,
	.size = ARRAY_SIZE(rk3399_isp_clks),
};

static const struct of_device_id rkisp1_plat_of_match[] = {
	{
		.compatible = "rockchip,rk3288-cif-isp",
		.data = &rk3288_isp_clk_data,
	}, {
		.compatible = "rockchip,rk3399-cif-isp",
		.data = &rk3399_isp_clk_data,
	},
	{},
};

static irqreturn_t rkisp1_irq_handler(int irq, void *cntxt)
{
	struct device *dev = cntxt;
	struct rkisp1_device *rkisp1_dev = dev_get_drvdata(dev);
	void __iomem *base = rkisp1_dev->config.base_addr;
	unsigned int mis_val;

	if (irq != rkisp1_dev->irq)
		return IRQ_NONE;

	mis_val = readl(rkisp1_dev->base_addr + CIF_ISP_MIS);
	if (mis_val) {
		rkisp1_isp_isr(mis_val, rkisp1_dev);
		return IRQ_HANDLED;
	}

	mis_val = readl(rkisp1_dev->base_addr + CIF_MIPI_MIS);
	if (mis_val) {
		rkisp1_mipi_isr(mis_val, rkisp1_dev);
		return IRQ_HANDLED;
	}

	mis_val = rkisp1_dev->sp_stream.ops->is_frame_end_int_masked(base);
	if (mis_val) {
		rkisp1_mi_isr(&rkisp1_dev->sp_stream);
		return IRQ_HANDLED;
	}
	mis_val = rkisp1_dev->mp_stream.ops->is_frame_end_int_masked(base);
	if (mis_val) {
		rkisp1_mi_isr(&rkisp1_dev->mp_stream);
		return IRQ_HANDLED;
	}
	/* TODO: update crop & resize */
	clr_all_int( base);
	return IRQ_NONE;
}

void rkisp1_disable_sys_clk(struct rkisp1_device *rkisp1_dev)
{
	struct clk *clk;
	int i;

	for (i = RKISP1_MAX_BUS_CLK - 1; i >= 0; i--) {
		clk = rkisp1_dev->clks[i];

		if (IS_ERR(clk))
			break;
		clk_disable_unprepare(clk);
	}
}

int rkisp1_enable_sys_clk(struct rkisp1_device *rkisp1_dev)
{
	struct clk *clk;
	int i, ret = -EINVAL;

	for (i = 0; i < RKISP1_MAX_BUS_CLK; i++) {
		clk = rkisp1_dev->clks[i];
		if (IS_ERR(clk))
			return 0;

		ret = clk_prepare_enable(clk);
		if (ret < 0)
			goto err;

	}
	return 0;
err:
	while (i--)
		clk_disable_unprepare(rkisp1_dev->clks[i]);
	return ret;
}

void rkisp1_v4l2_event(struct rkisp1_device *dev, __u32 frame_sequence)
{
	struct v4l2_event ev = {
		.type = V4L2_EVENT_FRAME_SYNC,
		.u.frame_sync.frame_sequence = frame_sequence,
	};

	v4l2_event_queue(&dev->sp_stream.vnode.vdev, &ev);
}

static int rkisp1_plat_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct rkisp1_device *isp_dev;
	struct isp_clk_data *clk_data;
	struct resource *res;
	int i, ret, irq;

	match = of_match_node(rkisp1_plat_of_match, node);
	isp_dev = devm_kzalloc(dev, sizeof(*isp_dev), GFP_KERNEL);
	if (!isp_dev)
		return -ENOMEM;
	isp_dev->sof_event = rkisp1_v4l2_event;
	dev_set_drvdata(dev, isp_dev);
	isp_dev->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	isp_dev->base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(isp_dev->base_addr))
		return PTR_ERR(isp_dev->base_addr);
	isp_dev->config.base_addr = isp_dev->base_addr;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(dev, irq, rkisp1_irq_handler,
					NULL, 0, dev_driver_string(dev), dev);
	if (ret < 0) {
		dev_err(dev, "request irq failed: %d\n", ret);
		return ret;
	}
	isp_dev->irq = irq;
	clk_data = (struct isp_clk_data *)match->data;
	for (i = 0; i < clk_data->size; i++) {
		struct clk *clk = devm_clk_get(dev, clk_data->clks[i]);

		if (IS_ERR(clk)) {
			dev_err(dev, "failed to get %s\n", clk_data->clks[i]);
			return PTR_ERR(clk);
		}
		isp_dev->clks[i] = clk;
	}

	isp_dev->sp_stream.state = RKISP1_STATE_DISABLED;
	isp_dev->sp_stream.id = RKISP1_STREAM_SP;
	isp_dev->sp_stream.ispdev = isp_dev;
	isp_dev->mp_stream.state = RKISP1_STATE_DISABLED;
	isp_dev->mp_stream.id = RKISP1_STREAM_MP;
	isp_dev->mp_stream.ispdev = isp_dev;
	init_waitqueue_head(&isp_dev->sp_stream.done);
	init_waitqueue_head(&isp_dev->mp_stream.done);
	rkisp1_stream_init(&isp_dev->sp_stream, RKISP1_STREAM_SP);
	rkisp1_stream_init(&isp_dev->mp_stream, RKISP1_STREAM_MP);
	spin_lock_init(&isp_dev->sp_stream.vbq_lock);
	spin_lock_init(&isp_dev->mp_stream.vbq_lock);

	strlcpy(isp_dev->media_dev.model, "rkisp1",
		sizeof(isp_dev->media_dev.model));
	isp_dev->media_dev.dev = &pdev->dev;
	v4l2_dev = &isp_dev->v4l2_dev;
	v4l2_dev->mdev = &isp_dev->media_dev;
	strlcpy(v4l2_dev->name, "rkisp1", sizeof(v4l2_dev->name));

	/* 2.  register v4l2 dev, register media dev */
	ret = v4l2_device_register(isp_dev->dev, &isp_dev->v4l2_dev);
	if (ret < 0)
		return ret;
	ret = media_device_register(&isp_dev->media_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register media device: %d\n",
			 ret);
		goto err_unreg_v4l2_dev;
	}

	/* 3.  create & register platefom subdev (from of_node) */
	ret = rkisp1_register_platform_subdevs(isp_dev);
	if (ret < 0)
		goto err_unreg_media_dev;

	pm_runtime_enable(&pdev->dev);
	return 0;

err_unreg_media_dev:
	media_device_unregister(&isp_dev->media_dev);
err_unreg_v4l2_dev:
	v4l2_device_unregister(&isp_dev->v4l2_dev);
	return ret;
}

static int rkisp1_plat_remove(struct platform_device *pdev)
{
	struct rkisp1_device *isp_dev = platform_get_drvdata(pdev);

	rkisp1_stream_release(&isp_dev->sp_stream);
	rkisp1_stream_release(&isp_dev->mp_stream);
	rkisp1_unregister_params_vdev(&isp_dev->params_vdev);
	rkisp1_unregister_stats_vdev(&isp_dev->stats_vdev);
	rkisp1_unregister_stream_vdevs(isp_dev);
	rkisp1_unregister_isp_subdev(isp_dev);
	media_device_unregister(&isp_dev->media_dev);
	v4l2_device_unregister(&isp_dev->v4l2_dev);
	return 0;
}

static int rkisp1_pm_suspend(struct device *dev)
{
	struct rkisp1_device *isp_dev = dev_get_drvdata(dev);

	if (pm_runtime_suspended(isp_dev->dev))
		return 0;

	return pinctrl_pm_select_sleep_state(dev);
}

static int rkisp1_pm_resume(struct device *dev)
{
	struct rkisp1_device *isp_dev = dev_get_drvdata(dev);

	if (isp_dev->num_sensors == 0) {
		v4l2_err(&isp_dev->v4l2_dev, "No sensor\n");
		return -EINVAL;
	}

	return pinctrl_pm_select_default_state(dev);
}

static int rkisp1_runtime_suspend(struct device *dev)
{
	struct rkisp1_device *isp_dev = dev_get_drvdata(dev);

	rkisp1_disable_sys_clk(isp_dev);
	return 0;
}

static int rkisp1_runtime_resume(struct device *dev)
{
	struct rkisp1_device *isp_dev = dev_get_drvdata(dev);

	return rkisp1_enable_sys_clk(isp_dev);
}

static const struct dev_pm_ops rkisp1_plat_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rkisp1_pm_suspend, rkisp1_pm_resume)
	SET_RUNTIME_PM_OPS(rkisp1_runtime_suspend, rkisp1_runtime_resume, NULL)
};

static struct platform_driver rkisp1_plat_drv = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = of_match_ptr(rkisp1_plat_of_match),
		   .pm = &rkisp1_plat_pm_ops,
	},
	.probe = rkisp1_plat_probe,
	.remove = rkisp1_plat_remove,
};

module_platform_driver(rkisp1_plat_drv);
MODULE_AUTHOR("Rockchip Camera/ISP team");
MODULE_DESCRIPTION("Rockchip ISP1 platform driver");
MODULE_LICENSE("Dual BSD/GPL");
