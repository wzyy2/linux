/*
 * Rockchip driver for rkisp1
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/platform_data/rkisp1-platform.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include "mipiphy.h"
#include "regs.h"
#include "rkisp1.h"

struct rkisp1_pltfrm_data {
	void __iomem *base_addr;
	int irq;
};

struct isp_clk_data {
	const char * const *clks;
	int size;
};

/*
 * pipeline management
 */

static int __isp_pipeline_prepare(struct cif_isp10_pipeline *p,
				  struct media_entity *me)
{
	struct v4l2_subdev *sd;
	struct v4l2_subdev *sensor = NULL;
	int i;

	memset(p->subdevs, 0, sizeof(p->subdevs));

	while (1) {
		struct media_pad *pad = NULL;

		/* Find remote source pad */
		for (i = 0; i < me->num_pads; i++) {
			struct media_pad *spad = &me->pads[i];

			if (!(spad->flags & MEDIA_PAD_FL_SINK))
				continue;
			pad = media_entity_remote_pad(spad);
			if (pad)
				break;
		}

		if (pad == NULL ||
		    media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;
		sd = media_entity_to_v4l2_subdev(pad->entity);
		switch (sd->grp_id) {
		case GRP_ID_SENSOR:
			sensor = sd;
			p->subdevs[IDX_SENSOR] = sd;
			break;
		case GRP_ID_MIPIPHY:
			p->subdevs[IDX_MIPIPHY] = sd;
			break;
		case GRP_ID_ISP:
			p->subdevs[IDX_ISP] = sd;
			break;
		case GRP_ID_ISP_MP:
		case GRP_ID_ISP_SP:
			p->subdevs[IDX_ISP_PATH] = sd;
			break;
		default:
			break;
		}
		me = &sd->entity;
		if (me->num_pads == 1)
			break;
	}
	return 0;
}

static int __subdev_set_power(struct v4l2_subdev *sd, int on)
{
	int *use_count;
	int ret;

	v4l2_info(sd, "%d: name %s,on %d\n", __LINE__, sd->name, on);

	if (sd == NULL)
		return -ENXIO;

	use_count = &sd->entity.use_count;
	if (on && (*use_count)++ > 0)
		return 0;
	else if (!on && (*use_count == 0 || --(*use_count) > 0))
		return 0;
	ret = v4l2_subdev_call(sd, core, s_power, on);

	return ret != -ENOIOCTLCMD ? ret : 0;
}

static int isp_pipeline_s_power(struct cif_isp10_pipeline *p, bool on)
{
	static const u8 seq[2][IDX_MAX] = {
		{ IDX_ISP_PATH, IDX_ISP, IDX_SENSOR, IDX_MIPIPHY },
		{ IDX_ISP_PATH, IDX_ISP, IDX_SENSOR, IDX_MIPIPHY },
	};
	int i, ret = 0;

	if (p->subdevs[IDX_SENSOR] == NULL)
		return -ENXIO;

	for (i = 0; i < IDX_MAX; i++) {
		unsigned int idx = seq[on][i];

		ret = __subdev_set_power(p->subdevs[idx], on);
		if (ret < 0 && ret != -ENXIO)
			goto error;
	}

	return 0;
error:
	for (; i >= 0; i--) {
		unsigned int idx = seq[on][i];

		__subdev_set_power(p->subdevs[idx], !on);
	}
	return ret;
}

int isp_pipeline_open(struct cif_isp10_pipeline *p, struct media_entity *me,
		      bool prepare)
{
	int ret;
	struct v4l2_subdev *sd;

	if (WARN_ON(p == NULL || me == NULL))
		return -EINVAL;

	if (prepare)
		__isp_pipeline_prepare(p, me);

	sd = p->subdevs[IDX_SENSOR];
	if (sd == NULL)
		return -EINVAL;

	ret = isp_pipeline_s_power(p, 1);
	if (ret < 0)
		return ret;

	return 0;
}

int isp_pipeline_close(struct cif_isp10_pipeline *p)
{
	int ret;

	ret = isp_pipeline_s_power(p, 0);

	return ret == -ENXIO ? 0 : ret;
}

int isp_pipeline_set_stream(struct cif_isp10_pipeline *p, bool on)
{
	static const u8 seq[2][IDX_MAX] = {
		{ IDX_MIPIPHY, IDX_SENSOR, IDX_ISP, IDX_ISP_PATH },
		{ IDX_ISP, IDX_ISP_PATH, IDX_MIPIPHY, IDX_SENSOR },
	};
	int i, ret = 0;

	if (p->subdevs[IDX_SENSOR] == NULL)
		return -ENODEV;

	for (i = 0; i < IDX_MAX; i++) {
		unsigned int idx = seq[on][i];

		ret = v4l2_subdev_call(p->subdevs[idx], video, s_stream, on);

		if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
			goto error;
	}

	return 0;
error:
	for (; i >= 0; i--) {
		unsigned int idx = seq[on][i];

		v4l2_subdev_call(p->subdevs[idx], video, s_stream, !on);
	}
	return ret;
}

static int rkisp1_create_links(struct rkisp1_device *dev)
{
	int ret;
	unsigned int flags = 0;
	struct media_entity *source, *sink;

	/*
	* note: entities source pad link should be
	* established prior to sink pad link.
	* isp sink links shoud be established first.
	*/
	flags = MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE;
	source = &dev->subdevs[RKISP1_SD_SENSOR]->entity;
	sink = &dev->subdevs[RKISP1_SD_PHY_CSI]->entity;
	ret = media_entity_create_link(source, 0,
				       sink, MIPIPHY_PAD_SINK, flags);
	if (ret < 0)
		return ret;

	source = &dev->subdevs[RKISP1_SD_PHY_CSI]->entity;
	sink = &dev->isp_sdev.isp_sub_dev.entity;
	ret = media_entity_create_link(source, MIPIPHY_PAD_SOURCE,
				       sink, RKISP1_ISP_PAD_SINK, flags);
	if (ret < 0)
		return ret;

	ret = media_entity_call(sink, link_setup,
				&sink->pads[RKISP1_ISP_PAD_SINK],
				&source->pads[MIPIPHY_PAD_SOURCE], flags);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	/* create isp internal links */
	/* SP links */
	source = &dev->isp_sdev.isp_sub_dev.entity;
	sink = &dev->path_sdevs.sp_sdev.subdev.entity;
	ret = media_entity_create_link(source, RKISP1_ISP_PAD_SOURCE_PATH,
				       sink, 0, flags);
	if (ret < 0)
		return ret;

	source = &dev->path_sdevs.sp_sdev.subdev.entity;
	sink = &dev->strm_vdevs.sp_vdev.vnode.vdev.entity;
	ret = media_entity_create_link(source, 1, sink, 0, flags);
	if (ret < 0)
		return ret;

	/* MP links */
	source = &dev->isp_sdev.isp_sub_dev.entity;
	sink = &dev->path_sdevs.mp_sdev.subdev.entity;
	ret = media_entity_create_link(source, RKISP1_ISP_PAD_SOURCE_PATH,
				       sink, 0, flags);
	if (ret < 0)
		return ret;

	source = &dev->path_sdevs.mp_sdev.subdev.entity;
	sink = &dev->strm_vdevs.mp_vdev.vnode.vdev.entity;
	ret = media_entity_create_link(source, 1, sink, 0, flags);
	if (ret < 0)
		return ret;

	/* 3A stats links */
	source = &dev->isp_sdev.isp_sub_dev.entity;
	sink = &dev->isp_sdev.isp_dev.vnode.vdev.entity;
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
	struct rkisp1_device *isp_dev;

	isp_dev = container_of(asd, struct rkisp1_device, asd);

	if (isp_dev->subdevs[RKISP1_SD_SENSOR]) {
		v4l2_err(subdev, "Only one sensor is supported now\n");
		return -EINVAL;
	}
	isp_dev->subdevs[RKISP1_SD_SENSOR] = subdev;
	subdev->grp_id = GRP_ID_SENSOR;
	isp_dev->num_sensors++;
	v4l2_info(subdev, "Async registered subdev\n");

	return 0;
}

static int isp_subdev_notifier(struct rkisp1_device *isp_dev,
			       struct device_node *parent)
{
	struct v4l2_async_notifier *notifier = &isp_dev->notifier;
	struct v4l2_async_subdev *asd = &isp_dev->asd;
	struct device *dev = isp_dev->dev;
	struct device_node *node, *remote = NULL;

	node = of_graph_get_next_endpoint(parent, NULL);
	if (!node) {
		dev_err(dev, "Can't find the sensor endpoint\n");
		return -EINVAL;
	}

	notifier->subdevs = devm_kzalloc(dev, sizeof(*notifier->subdevs),
					 GFP_KERNEL);
	if (!notifier->subdevs)
		return -ENOMEM;

	asd->match_type = V4L2_ASYNC_MATCH_FWNODE;
	remote = of_graph_get_remote_port_parent(node);
	asd->match.fwnode.fwnode = of_fwnode_handle(remote);
	of_node_put(node);
	if (!asd->match.fwnode.fwnode) {
		dev_err(dev, "Can't get remote endpoint parent\n");
		return -EINVAL;
	}
	notifier->subdevs[0] = asd;
	notifier->num_subdevs = 1;
	notifier->bound = subdev_notifier_bound;
	notifier->complete = subdev_notifier_complete;

	return v4l2_async_notifier_register(&isp_dev->v4l2_dev, notifier);
}

static int register_mipiphy_subdev(struct rkisp1_device *isp_dev)
{
	struct media_entity *me;
	struct v4l2_subdev *sd;
	struct device *dev = isp_dev->dev;
	struct platform_device *mipi_pdev;
	struct device_node *of_mipi;
	int ret;

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret < 0) {
		dev_err(dev, "Failed to populate child mipiphy(%d)\n", ret);
		return ret;
	}

	of_mipi = of_get_next_available_child(dev->of_node, NULL);
	if (!of_mipi) {
		dev_err(dev, "Failed to get mipiphy node\n");
		return -EINVAL;
	}
	mipi_pdev = of_find_device_by_node(of_mipi);
	if (!mipi_pdev) {
		dev_err(dev, "Failed to get mipiphy device\n");
		ret = -EINVAL;
		goto err_put_node;
	}

	me = platform_get_drvdata(mipi_pdev);
	if (!me) {
		dev_err(dev, "Defer for mipiphy device is not probed\n");
		ret = -EPROBE_DEFER;
		goto err_put_node;
	}

	sd = media_entity_to_v4l2_subdev(me);
	ret = v4l2_device_register_subdev(&isp_dev->v4l2_dev, sd);
	if (ret < 0) {
		v4l2_err(&isp_dev->v4l2_dev, "Failed to register phy subdev\n");
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

static int register_stream_subdevs(struct rkisp1_device *dev)
{
	int ret;

	ret = register_stream_subdev(&dev->path_sdevs.mp_sdev.subdev,
				     &dev->v4l2_dev);
	if (ret < 0)
		return ret;
	ret = cif_isp10_register_videodev(dev, RKISP1_STREAM_MP);
	if (ret < 0)
		goto err_unreg_mp_subdev;

	ret = register_stream_subdev(&dev->path_sdevs.sp_sdev.subdev,
				     &dev->v4l2_dev);
	if (ret < 0)
		goto err_unreg_mp_vdev;

	ret = cif_isp10_register_videodev(dev, RKISP1_STREAM_SP);
	if (ret < 0)
		goto err_unreg_sp_subdev;

	return 0;
err_unreg_sp_subdev:
	unregister_stream_subdev(&dev->path_sdevs.sp_sdev.subdev);
err_unreg_mp_vdev:
	cif_isp10_unregister_videodev(dev, RKISP1_STREAM_MP);
err_unreg_mp_subdev:
	unregister_stream_subdev(&dev->path_sdevs.mp_sdev.subdev);
	return ret;
}

static void unregister_stream_subdevs(struct rkisp1_device *dev)
{
	unregister_stream_subdev(&dev->path_sdevs.mp_sdev.subdev);
	cif_isp10_unregister_videodev(dev, RKISP1_STREAM_MP);
	unregister_stream_subdev(&dev->path_sdevs.sp_sdev.subdev);
	cif_isp10_unregister_videodev(dev, RKISP1_STREAM_SP);
}

static int register_platform_subdevs(struct rkisp1_device *dev)
{
	int ret;
	struct cif_isp10_isp_dev *isp_dev = &dev->isp_sdev.isp_dev;

	/*/ register isp v4l2 subdev */
	ret = register_cifisp_isp_subdev(dev, &dev->v4l2_dev);
	if (ret < 0)
		return ret;

	ret = register_stream_subdevs(dev);
	if (ret < 0)
		goto err_unreg_isp_subdev;

	/* register  ISP(3A stats)dev */
	ret = register_cifisp_device(isp_dev,
				     &isp_dev->vnode.vdev,
				     &dev->v4l2_dev, dev->config.base_addr);
	if (ret < 0)
		goto err_unreg_stream_subdev;

	/* register mipi dev */
	ret = register_mipiphy_subdev(dev);
	if (ret < 0)
		goto err_unreg_isp_dev;

	return 0;
err_unreg_isp_dev:
	unregister_cifisp_device(&isp_dev->vnode.vdev);
err_unreg_stream_subdev:
	unregister_stream_subdevs(dev);
err_unreg_isp_subdev:
	unregister_cifisp_isp_subdev(dev);
	return ret;
}

static void unregister_sensor_subdevs(struct rkisp1_device *dev)
{

}

static void unregister_mipiphy_subdev(struct rkisp1_device *dev)
{

}

static void unregister_platform_subdevs(struct rkisp1_device *dev)
{
	struct video_device *vdev_cifisp = &dev->isp_sdev.isp_dev.vnode.vdev;

	unregister_mipiphy_subdev(dev);
	if (vdev_cifisp)
		unregister_cifisp_device(vdev_cifisp);
	unregister_stream_subdevs(dev);
	unregister_cifisp_isp_subdev(dev);
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
	struct rkisp1_pltfrm_data *pdata = dev_get_platdata(dev);
	struct rkisp1_device *rkisp1_dev = dev_get_drvdata(dev);
	unsigned int mis_val;

	if (irq != pdata->irq)
		return IRQ_NONE;

	mis_val = ioread32(pdata->base_addr + CIF_ISP_MIS);
	if (mis_val) {
		cif_isp10_isp_isr(mis_val, (void *)rkisp1_dev);
		return IRQ_HANDLED;
	}

	mis_val = ioread32(pdata->base_addr + CIF_MIPI_MIS);
	if (mis_val) {
		cif_isp10_mipi_isr(mis_val, (void *)rkisp1_dev);
		return IRQ_HANDLED;
	}

	mis_val = ioread32(pdata->base_addr + CIF_MI_MIS);
	if (mis_val) {
		cif_isp10_mi_isr(mis_val, (void *)rkisp1_dev);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

int rkisp1_disable_sys_clk(struct rkisp1_device *rkisp1_dev)
{
	struct clk *clk;
	int i;

	for (i = RKISP1_MAX_BUS_CLK - 1; i >= 0; i--) {
		clk = rkisp1_dev->clks[i];

		if (IS_ERR(clk))
			continue;
		clk_disable_unprepare(clk);
		rkisp1_dev->clks[i] = 0;
	}

	return 0;
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
			while (i--)
				clk_disable_unprepare(rkisp1_dev->clks[i]);
	}

	return ret;
}

static int rkisp1_plat_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct rkisp1_pltfrm_data *pdata;
	struct rkisp1_device *isp_dev;
	struct isp_clk_data *clk_data;
	struct resource *res;
	int i, ret, irq;

	pr_info("rkisp1 probing...\n");
	match = of_match_node(rkisp1_plat_of_match, node);
	isp_dev = devm_kzalloc(dev, sizeof(*isp_dev), GFP_KERNEL);
	if (!isp_dev)
		return -ENOMEM;
	isp_dev->sof_event = cif_isp10_v4l2_event;
	dev_set_drvdata(dev, isp_dev);
	isp_dev->dev = dev;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata->base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdata->base_addr))
		return PTR_ERR(pdata->base_addr);

	isp_dev->config.base_addr = pdata->base_addr;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(dev, irq, rkisp1_irq_handler,
					NULL, 0, dev_driver_string(dev), dev);
	if (ret < 0) {
		dev_err(dev, "request irq failed: %d\n", ret);
		return ret;
	}
	pdata->irq = irq;
	dev->platform_data = pdata;

	clk_data = (struct isp_clk_data *)match->data;
	if (clk_data) {
		for (i = 0; i < clk_data->size; i++) {
			struct clk *clk = devm_clk_get(dev, clk_data->clks[i]);

			if (IS_ERR(clk)) {
				dev_err(dev, "failed to get %s\n",
					clk_data->clks[i]);
				while (i--)
					devm_clk_put(dev, isp_dev->clks[i]);
				return PTR_ERR(clk);
			}
			isp_dev->clks[i] = clk;
		}
	}

	isp_dev->strm_vdevs.sp_vdev.state = RKISP1_STATE_DISABLED;
	isp_dev->strm_vdevs.sp_vdev.id = RKISP1_STREAM_SP;
	isp_dev->strm_vdevs.mp_vdev.state = RKISP1_STATE_DISABLED;
	isp_dev->strm_vdevs.mp_vdev.id = RKISP1_STREAM_MP;
	isp_dev->strm_vdevs.mi_config.async_updt = 0;
	init_waitqueue_head(&isp_dev->strm_vdevs.sp_vdev.done);
	init_waitqueue_head(&isp_dev->strm_vdevs.mp_vdev.done);

	cif_isp10_stream_init(isp_dev, RKISP1_ALL_STREAMS);
	spin_lock_init(&isp_dev->strm_vdevs.vbq_lock);
	spin_lock_init(&iowrite32_verify_lock);
	strlcpy(isp_dev->media_dev.model, "ROCKCHIP ISP",
		sizeof(isp_dev->media_dev.model));
	isp_dev->media_dev.dev = &pdev->dev;
	v4l2_dev = &isp_dev->v4l2_dev;
	v4l2_dev->mdev = &isp_dev->media_dev;
	v4l2_dev->notify = NULL;
	strlcpy(v4l2_dev->name, "rk-isp10", sizeof(v4l2_dev->name));

	/* 2.  register v4l2 dev, register media dev */
	ret = v4l2_device_register(isp_dev->dev, &isp_dev->v4l2_dev);
	if (ret < 0) {
		pr_err("Failed to register v4l2 device\n");
		return ret;
	}

	ret = media_device_register(&isp_dev->media_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register media device: %d\n",
			 ret);
		goto err_unreg_v4l2_dev;
	}

	isp_dev->num_sensors = 0;

	/* 3.  create & register platefom subdev (from of_node) */
	ret = register_platform_subdevs(isp_dev);
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

	cif_isp10_stream_release(isp_dev, RKISP1_ALL_STREAMS);
	unregister_sensor_subdevs(isp_dev);
	unregister_platform_subdevs(isp_dev);
	media_device_unregister(&isp_dev->media_dev);
	v4l2_device_unregister(&isp_dev->v4l2_dev);
	return 0;
}

static int rkisp1_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct rkisp1_device *isp_dev = dev_get_drvdata(dev);

	if (pm_runtime_suspended(isp_dev->dev))
		return 0;

	isp_dev->strm_vdevs.sp_vdev.saved_state =
	isp_dev->strm_vdevs.sp_vdev.state;
	isp_dev->strm_vdevs.mp_vdev.saved_state =
	isp_dev->strm_vdevs.mp_vdev.state;

	ret = cif_isp10_streamoff(isp_dev, RKISP1_ALL_STREAMS);
	if (ret < 0)
		return ret;

	return pinctrl_pm_select_sleep_state(dev);
}

static int rkisp1_pm_resume(struct device *dev)
{
	struct rkisp1_device *isp_dev = dev_get_drvdata(dev);
	struct cif_isp10_stream *mp_vdev = &isp_dev->strm_vdevs.mp_vdev;
	struct cif_isp10_stream *sp_vdev = &isp_dev->strm_vdevs.sp_vdev;
	int ret = 0, stream_ids = 0;

	if (isp_dev->num_sensors == 0) {
		v4l2_err(&isp_dev->v4l2_dev, "no sensor\n");
		return -EINVAL;
	}

	ret = pinctrl_pm_select_default_state(dev);
	if (ret < 0)
		return ret;

	if ((sp_vdev->saved_state == RKISP1_STATE_READY) ||
	    (sp_vdev->saved_state == RKISP1_STATE_STREAMING)) {
		sp_vdev->updt_cfg = true;
		sp_vdev->state = RKISP1_STATE_READY;
		if (sp_vdev->saved_state == RKISP1_STATE_STREAMING)
			stream_ids |= RKISP1_STREAM_SP;
	}
	if ((mp_vdev->saved_state == RKISP1_STATE_READY) ||
	    (mp_vdev->saved_state == RKISP1_STATE_STREAMING)) {
		mp_vdev->updt_cfg = true;
		mp_vdev->state = RKISP1_STATE_READY;
		if (mp_vdev->saved_state == RKISP1_STATE_STREAMING)
			stream_ids |= RKISP1_STREAM_MP;
	}

	return cif_isp10_streamon(isp_dev, stream_ids);
}

static int rkisp1_runtime_suspend(struct device *dev)
{
	struct rkisp1_device *isp_dev = dev_get_drvdata(dev);

	return rkisp1_disable_sys_clk(isp_dev);
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

MODULE_DESCRIPTION("RKISP1 platform driver");
MODULE_LICENSE("GPL");
