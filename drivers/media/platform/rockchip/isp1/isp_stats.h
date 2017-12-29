// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip isp1 driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 */

#ifndef _RKISP1_ISP_STATS_H
#define _RKISP1_ISP_STATS_H

#include <linux/rkisp1-config.h>
#include "common.h"

struct rkisp1_isp_stats_vdev;

enum rkisp1_isp_readout_cmd {
	RKISP1_ISP_READOUT_MEAS,
	RKISP1_ISP_READOUT_META,
};

struct rkisp1_isp_readout_work {
	struct work_struct work;
	struct rkisp1_isp_stats_vdev *stats_vdev;

	unsigned int frame_id;
	unsigned int isp_ris;
	enum rkisp1_isp_readout_cmd readout;
	struct vb2_buffer *vb;
};

/*
 * struct rkisp1_isp_stats_vdev - ISP Statistics device
 *
 * @irq_lock: buffer queue lock
 * @stat: stats buffer list
 * @readout_wq: workqueue for statistics information read
 */
struct rkisp1_isp_stats_vdev {
	struct rkisp1_vdev_node vnode;
	struct rkisp1_device *dev;

	spinlock_t irq_lock;
	struct list_head stat;
	struct v4l2_format vdev_fmt;
	bool streamon;

	struct workqueue_struct *readout_wq;
};

int rkisp1_stats_isr(struct rkisp1_isp_stats_vdev *stats_vdev, u32 isp_ris);

int rkisp1_register_stats_vdev(struct rkisp1_isp_stats_vdev *stats_vdev,
			       struct v4l2_device *v4l2_dev,
			       struct rkisp1_device *dev);

void rkisp1_unregister_stats_vdev(struct rkisp1_isp_stats_vdev *stats_vdev);

#endif /* _RKISP1_ISP_STATS_H */
