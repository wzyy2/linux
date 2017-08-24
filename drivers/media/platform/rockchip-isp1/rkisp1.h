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

#ifndef _RKISP1_H
#define _RKISP1_H

#include <linux/platform_device.h>
#include <media/v4l2-of.h>
#include "common.h"
#include "isp_stats.h"
#include "isp_params.h"
#include "capture.h"

struct rkisp1_ie_config {
	/* TODO: bit field ? */
	unsigned int effect;
};

struct rkisp1_config {
	void __iomem *base_addr;
	bool out_of_buffer_stall;
};

enum rkisp1_isp_pad {
	RKISP1_ISP_PAD_SINK,
	RKISP1_ISP_PAD_SINK_PARAMS,
	RKISP1_ISP_PAD_SOURCE_PATH,
	RKISP1_ISP_PAD_SOURCE_STATS,
	/* TODO: meta data pad ? */
	RKISP1_ISP_PAD_MAX
};

struct rkisp1_isp_subdev {
	struct v4l2_subdev		sd;
	struct media_pad		pads[RKISP1_ISP_PAD_MAX];
	struct v4l2_ctrl_handler	ctrl_handler;

	struct rkisp1_fmt		in_fmt;
	struct v4l2_rect		in_acqui;
	struct rkisp1_fmt		out_fmt;
	struct v4l2_rect		out_win;
};

struct rkisp1_sensor_info {
	struct v4l2_subdev *sd;
	struct v4l2_async_subdev asd;
	struct v4l2_of_endpoint ep;
};

#define RKISP1_MAX_BUS_CLK	8
#define RKISP1_MAX_SENSOR	2
struct rkisp1_device {
	void __iomem	*base_addr;
	int		irq;
	struct		device *dev;
	struct		clk *clks[RKISP1_MAX_BUS_CLK];

	struct v4l2_device		v4l2_dev;
	struct media_device		media_dev;
	struct v4l2_async_notifier	notifier;
	struct v4l2_subdev		*subdevs[RKISP1_SD_MAX];
	struct rkisp1_sensor_info	sensors[RKISP1_MAX_SENSOR];
	int num_sensors;

	spinlock_t writel_verify_lock;
	struct rkisp1_isp_subdev isp_sdev;
	struct rkisp1_stream mp_stream;
	struct rkisp1_stream sp_stream;
	struct rkisp1_isp_stats_vdev stats_vdev;
	struct rkisp1_isp_params_vdev params_vdev;

	struct rkisp1_config config;
	struct vb2_alloc_ctx *alloc_ctx;

	int cif_streamon_cnt;
	void (*sof_event)(struct rkisp1_device *dev, __u32 frame_sequence);
};

/* all ISP submodules are always working in sync mode, that is, the shadow registers of
 * submodules are  updated automatically  along with the frame end signal.
 */
#define RKISP1_ALWAYS_ASYNC 0
/* ISM sub module can be used to implement digital zoom functionality with resizer sub module,
 * and ISM parameters are changed frequently and accidentally,
 * in this case ISP will be changed to async mode to keep ISP pipeline stable.
 * and RKISP1_ASYNC_ISM tell the reason why ISP mode was changed.
 */
#define	RKISP1_ASYNC_ISM 1

/* define whether the MI sub module stop processing frames when there is no
 * queued buffer. if defined as 0, MI will hold one buffer when out of buffer occurs.
 */
#define RKISP1_ALWAYS_STALL_ON_NO_BUFS (0)

/* Clean code starts from here */

int rkisp1_register_isp_subdev(struct rkisp1_device *isp_dev,
			       struct v4l2_device *v4l2_dev);

void rkisp1_unregister_isp_subdev(struct rkisp1_device *isp_dev);

void rkisp1_mipi_isr(unsigned int mipi_mis, struct rkisp1_device *dev);

void rkisp1_isp_isr(unsigned int isp_mis, struct rkisp1_device *dev);

static inline struct rkisp1_device *sd_to_isp_dev(struct v4l2_subdev *sd)
{
	return container_of(sd->v4l2_dev, struct rkisp1_device, v4l2_dev);
}

#endif /* _RKISP1_H */
