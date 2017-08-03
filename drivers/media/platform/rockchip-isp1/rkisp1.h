/*
 * Rockchip driver for CIF ISP 1.0
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _RKISP1_H
#define _RKISP1_H

#include <linux/platform_device.h>
#include <linux/platform_data/rkisp1-platform.h>
#include "common.h"
#include "isp.h"
#include "path.h"
#include "path_video.h"

enum rkisp1_inp {
	RKISP1_INP_CSI = 0x10000000,
	RKISP1_INP_CPI = 0x20000000,

	RKISP1_INP_DMA = 0x30000000,	/* DMA -> ISP */
	RKISP1_INP_DMA_IE = 0x31000000,	/* DMA -> IE */
	RKISP1_INP_DMA_SP = 0x32000000,	/* DMA -> SP */
	RKISP1_INP_DMA_MAX = 0x33000000,

	RKISP1_INP_MAX = 0x7fffffff
};

#define RKISP1_INP_IS_DMA(inp) \
	(((inp) & 0xf0000000) == RKISP1_INP_DMA)
#define RKISP1_INP_IS_MIPI(inp) \
	(((inp) & 0xf0000000) == RKISP1_INP_CSI)
#define RKISP1_INP_IS_DVP(inp) \
	(((inp) & 0xf0000000) == RKISP1_INP_CPI)
#define RKISP1_INP_NEED_ISP(inp) \
	((inp) <  RKISP1_INP_DMA_IE)
#define RKISP1_INP_DMA_CNT() \
	((RKISP1_INP_DMA_MAX - RKISP1_INP_DMA) >> 24)

/* correspond to bit field values */
enum rkisp1_image_effect {
	RKISP1_IE_BW = 0,
	RKISP1_IE_NEGATIVE = 1,
	RKISP1_IE_SEPIA = 2,
	RKISP1_IE_C_SEL = 3,
	RKISP1_IE_EMBOSS = 4,
	RKISP1_IE_SKETCH = 5,
	RKISP1_IE_NONE		/* not a bit field value */
};

struct rkisp1_csi_config {
	u32 vc;
	u32 nb_lanes;
	u32 bit_rate;
};

struct rkisp1_ie_config {
	enum rkisp1_image_effect effect;
};

struct rkisp1_config {
	void __iomem *base_addr;
	enum rkisp1_inp input_sel;
	struct pltfrm_cam_itf cam_itf;
	bool out_of_buffer_stall;
};

enum rkisp1_isp_pad {
	RKISP1_ISP_PAD_SINK,
	RKISP1_ISP_PAD_SOURCE_PATH,
	RKISP1_ISP_PAD_SOURCE_STATS,
	/* TODO: meta data pad ? */
	RKISP1_ISP_PAD_MAX
};

struct rkisp1_isp_subdev {
	struct v4l2_subdev		sd;
	struct media_pad		pads[RKISP1_ISP_PAD_MAX];
	struct v4l2_ctrl_handler	ctrl_handler;

	struct cif_fmt_info		in_fmt;
	struct v4l2_rect		in_acqui;
	struct cif_fmt_info		out_fmt;
	struct v4l2_rect		out_win;

	struct cif_isp10_isp_dev	isp_dev;
};

struct rkisp1_img_src_data {
	unsigned int v_frame_id;
	struct isp_supplemental_sensor_mode_data data;
};

struct rkisp1_img_src_exps {
	spinlock_t lock;	/* protect list */
	struct list_head list;

	struct mutex mutex;	/* protect frm_exp */
	struct rkisp1_img_src_data data[2];
	unsigned char exp_valid_frms;
};

#define RKISP1_MAX_BUS_CLK 8
struct rkisp1_device {
	struct device *dev;
	struct clk *clks[RKISP1_MAX_BUS_CLK];
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct v4l2_subdev *subdevs[RKISP1_SD_MAX];
	void __iomem *base_addr;
	int irq;
	int num_sensors;
	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev asd;
	spinlock_t writel_verify_lock;
	struct cif_isp10_path_subdevs path_sdevs;
	struct rkisp1_stream_vdevs strm_vdevs;
	struct rkisp1_isp_subdev isp_sdev;

	struct rkisp1_config config;

	struct vb2_alloc_ctx *alloc_ctx;

	struct rkisp1_img_src_exps img_src_exps;

	int cif_streamon_cnt;
	void (*sof_event)(struct rkisp1_device *dev, __u32 frame_sequence);
	bool b_mi_frame_end;
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

/* when ISP are working in async mode, ISP registers should only be updated when
  * sub module mi is idle
  */
#define RKISP1_MI_IS_BUSY(dev) \
	(dev->strm_vdevs.mp_vdev.path_cfg.busy || \
	 dev->strm_vdevs.sp_vdev.path_cfg.busy)
/* this define means whether the MI sub module stop processing frames when there is no 
 * queued buffer. if defined as 0, MI will hold one buffer when out of buffer occurs.
 */
#define RKISP1_ALWAYS_STALL_ON_NO_BUFS (0)

/* Clean code starts from here */

int register_cifisp_isp_subdev(struct rkisp1_device *isp_dev,
			       struct v4l2_device *v4l2_dev);

void unregister_cifisp_isp_subdev(struct rkisp1_device *isp_dev);

void rkisp1_mipi_isr(unsigned int mipi_mis, struct rkisp1_device *dev);

void rkisp1_isp_isr(unsigned int isp_mis, struct rkisp1_device *dev);

static inline struct rkisp1_device *sd_to_isp_dev(struct v4l2_subdev *sd)
{
	return container_of(sd->v4l2_dev, struct rkisp1_device, v4l2_dev);
}

static inline
struct rkisp1_stream *to_stream_by_id(struct rkisp1_device *dev,
					     enum rkisp1_stream_id id)
{
	if (id == RKISP1_STREAM_MP)
		return &dev->strm_vdevs.mp_vdev;
	else
		return &dev->strm_vdevs.sp_vdev;
}

static inline
struct rkisp1_device *stream_to_dev(struct rkisp1_stream *stream)
{
	struct rkisp1_stream_vdevs *stream_vdevs = NULL;

	if (stream->id == RKISP1_STREAM_SP)
		stream_vdevs = container_of(stream,
					    struct rkisp1_stream_vdevs,
					    sp_vdev);
	else
		stream_vdevs = container_of(stream,
					    struct rkisp1_stream_vdevs,
					    mp_vdev);

	return container_of(stream_vdevs, struct rkisp1_device, strm_vdevs);
}

#endif /* _RKISP1_H */
