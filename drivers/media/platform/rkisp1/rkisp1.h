/*
 * Rockchip driver for CIF ISP 1.0
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

#ifndef _RKISP1_H
#define _RKISP1_H

#include <linux/platform_device.h>
#include <linux/platform_data/rkisp1-platform.h>
#include "common.h"
#include "isp.h"
#include "path.h"
#include "path_video.h"
#include "platform.h"

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
	((RKISP1_INP_DMA_MAX -\
	RKISP1_INP_DMA) >> 24)

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

#define RKISP1_PIX_FMT_MASK                  0xf0000000
#define RKISP1_PIX_FMT_MASK_BPP              0x0003f000
#define RKISP1_PIX_FMT_YUV_MASK_CPLANES      0x00000003
#define RKISP1_PIX_FMT_YUV_MASK_UVSWAP       0x00000004
#define RKISP1_PIX_FMT_YUV_MASK_YCSWAP       0x00000008
#define RKISP1_PIX_FMT_YUV_MASK_X            0x00000f00
#define RKISP1_PIX_FMT_YUV_MASK_Y            0x000000f0
#define RKISP1_PIX_FMT_RGB_MASK_PAT          0x000000f0
#define RKISP1_PIX_FMT_BAYER_MASK_PAT        0x000000f0
#define RKISP1_PIX_FMT_GET_BPP(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_MASK_BPP) >> 12)
#define cif_isp10_pix_fmt_set_bpp(pix_fmt, bpp) \
	{ \
		pix_fmt = (((pix_fmt) & ~RKISP1_PIX_FMT_MASK_BPP) | \
			(((bpp) << 12) & RKISP1_PIX_FMT_MASK_BPP)); \
	}

#define RKISP1_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt) \
	((pix_fmt) & RKISP1_PIX_FMT_YUV_MASK_CPLANES)
#define RKISP1_PIX_FMT_YUV_IS_YC_SWAPPED(pix_fmt) \
	((pix_fmt) & RKISP1_PIX_FMT_YUV_MASK_YCSWAP)
#define RKISP1_PIX_FMT_YUV_IS_UV_SWAPPED(pix_fmt) \
		((pix_fmt) & RKISP1_PIX_FMT_YUV_MASK_UVSWAP)
#define RKISP1_PIX_FMT_YUV_GET_X_SUBS(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_YUV_MASK_X) >> 8)
#define RKISP1_PIX_FMT_YUV_GET_Y_SUBS(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_YUV_MASK_Y) >> 4)
#define cif_isp10_pix_fmt_set_y_subs(pix_fmt, y_subs) \
	{ \
		pix_fmt = (((pix_fmt) & ~RKISP1_PIX_FMT_YUV_MASK_Y) | \
			((y_subs << 4) & RKISP1_PIX_FMT_YUV_MASK_Y)); \
	}
#define cif_isp10_pix_fmt_set_x_subs(pix_fmt, x_subs) \
	{ \
		pix_fmt = (((pix_fmt) & ~RKISP1_PIX_FMT_YUV_MASK_X) | \
			(((x_subs) << 8) & RKISP1_PIX_FMT_YUV_MASK_X)); \
	}
#define cif_isp10_pix_fmt_set_yc_swapped(pix_fmt, yc_swapped) \
	{ \
		pix_fmt = (((pix_fmt) & ~RKISP1_PIX_FMT_YUV_MASK_YCSWAP) | \
			(((yc_swapped) << 3) & \
			RKISP1_PIX_FMT_YUV_MASK_YCSWAP)); \
	}

#define RKISP1_PIX_FMT_BAYER_PAT_IS_BGGR(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_BAYER_MASK_PAT) == 0x0)
#define RKISP1_PIX_FMT_BAYER_PAT_IS_GBRG(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_BAYER_MASK_PAT) == 0x10)
#define RKISP1_PIX_FMT_BAYER_PAT_IS_GRBG(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_BAYER_MASK_PAT) == 0x20)
#define RKISP1_PIX_FMT_BAYER_PAT_IS_RGGB(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_BAYER_MASK_PAT) == 0x30)

#define RKISP1_PIX_FMT_IS_YUV(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_MASK) == 0x10000000)
#define RKISP1_PIX_FMT_IS_RGB(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_MASK) == 0x20000000)
#define RKISP1_PIX_FMT_IS_RAW_BAYER(pix_fmt) \
	(((pix_fmt) & RKISP1_PIX_FMT_MASK) == 0x30000000)

#define RKISP1_PIX_FMT_IS_INTERLEAVED(pix_fmt) \
	(!RKISP1_PIX_FMT_IS_YUV(pix_fmt) ||\
	!RKISP1_PIX_FMT_YUV_GET_NUM_CPLANES(pix_fmt))

struct rkisp1_csi_config {
	u32 vc;
	u32 nb_lanes;
	u32 bit_rate;
};

struct rkisp1_ie_config {
	enum rkisp1_image_effect effect;
};

/* IS */
struct rkisp1_ism_params {
	unsigned int ctrl;
	unsigned int recenter;
	unsigned int h_offs;
	unsigned int v_offs;
	unsigned int h_size;
	unsigned int v_size;
	unsigned int max_dx;
	unsigned int max_dy;
	unsigned int displace;
};

struct rkisp1_ism_config {
	bool ism_en;
	struct rkisp1_ism_params ism_params;
	bool ism_update_needed;
};

struct rkisp_isp_config {
	bool si_enable;
	struct rkisp1_ie_config ie_config;
	struct rkisp1_ism_config ism_config;
	struct cif_isp10_frm_fmt input;
	struct cif_isp10_frm_fmt output;
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
	struct v4l2_subdev isp_sub_dev;
	struct media_pad pads[RKISP1_ISP_PAD_MAX];
	struct v4l2_ctrl_handler ctrl_handler;
	struct rkisp_isp_config isp_config;
	struct cif_isp10_isp_dev isp_dev;
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
	int num_sensors;
	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev asd;

	struct cif_isp10_path_subdevs path_sdevs;
	struct cif_isp10_stream_vdevs strm_vdevs;
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

void cif_isp10_mipi_isr(unsigned int mipi_mis, void *cntxt);

void cif_isp10_isp_isr(unsigned int isp_mis, void *cntxt);

void cif_isp10_sensor_mode_data_sync(struct rkisp1_device *dev,
				     unsigned int frame_id,
				     struct isp_supplemental_sensor_mode_data
				     *data);

static inline struct rkisp1_device *sd_to_isp_dev(struct v4l2_subdev *sd)
{
	return container_of(sd->v4l2_dev, struct rkisp1_device, v4l2_dev);
}

static inline
struct cif_isp10_stream *to_stream_by_id(struct rkisp1_device *dev,
					     enum cif_isp10_stream_id id)
{
	if (id == RKISP1_STREAM_MP)
		return &dev->strm_vdevs.mp_vdev;
	else
		return &dev->strm_vdevs.sp_vdev;
}

static inline
struct rkisp1_device *stream_to_dev(struct cif_isp10_stream *stream)
{
	struct cif_isp10_stream_vdevs *stream_vdevs = NULL;

	if (stream->id == RKISP1_STREAM_SP)
		stream_vdevs = container_of(stream,
					    struct cif_isp10_stream_vdevs,
					    sp_vdev);
	else
		stream_vdevs = container_of(stream,
					    struct cif_isp10_stream_vdevs,
					    mp_vdev);

	return container_of(stream_vdevs, struct rkisp1_device, strm_vdevs);
}

#endif /* _RKISP1_H */
