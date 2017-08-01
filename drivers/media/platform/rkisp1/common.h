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

#ifndef _RKISP1_COMMON_H
#define _RKISP1_COMMON_H

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>

/*
 * as linux/media-bus-format.h said, these value can't be changed,
 * so it's safe to define following macros.
 */
#define CIFISP_BUSCODE_FMT_IS_YUV(code) \
	(code >= MEDIA_BUS_FMT_Y8_1X8 && code < MEDIA_BUS_FMT_SBGGR8_1X8)

#define CIFISP_BUSCODE_FMT_IS_RAW(code) \
	(code >= MEDIA_BUS_FMT_SBGGR8_1X8 && code < MEDIA_BUS_FMT_JPEG_1X8)


#define RKISP1_V4L2_SP_DEV_MAJOR 0
#define RKISP1_V4L2_ISP_DEV_MAJOR 1
#define RKISP1_V4L2_MP_DEV_MAJOR 2

#define GRP_ID_SENSOR           (1 << 0)
#define GRP_ID_MIPIPHY          (1 << 1)
#define GRP_ID_ISP              (1 << 2)
#define GRP_ID_ISP_MP		(1 << 3)
#define GRP_ID_ISP_SP		(1 << 4)

#define STREAM_PATH_DEFAULT_WIDTH  800
#define STREAM_PATH_DEFAULT_HEIGHT 600

enum isp_subdev_index {
	IDX_SENSOR,
	IDX_MIPIPHY,
	IDX_ISP,
	IDX_ISP_PATH,
	IDX_MAX,
};

enum cif_isp10_sd_type {
	RKISP1_SD_SENSOR,
	RKISP1_SD_PHY_CSI,
	RKISP1_SD_VCM,
	RKISP1_SD_FLASH,
	RKISP1_SD_MAX,
};

enum cif_isp10_stream_id {
	RKISP1_STREAM_SP = 1 << 0,
	RKISP1_STREAM_MP = 1 << 1,
};

#define RKISP1_ALL_STREAMS \
	(RKISP1_STREAM_SP | RKISP1_STREAM_MP)

struct cif_isp10_pipeline;

typedef int (*cif_isp10_pipeline_open) (struct cif_isp10_pipeline *p,
					struct media_entity *me, bool prepare);

typedef int (*cif_isp10_pipeline_close) (struct cif_isp10_pipeline *p);

typedef int (*cif_isp10_pipeline_set_stream) (struct cif_isp10_pipeline *p,
					      bool on);

/*
 * struct cif_isp10_pipeline - An ISP hardware pipeline
 * @entities: Bitmask of entities in the pipeline (indexed by entity ID)
 */
struct cif_isp10_pipeline {
	struct media_pipeline pipe;
	struct v4l2_subdev *subdevs[IDX_MAX];
	cif_isp10_pipeline_open open;
	cif_isp10_pipeline_close close;
	cif_isp10_pipeline_set_stream set_stream;
};

/* One structure per video node */
struct cif_isp10_vdev_node {
	struct vb2_queue buf_queue;
	struct mutex qlock;
	struct video_device vdev;
	struct media_pad pad;
	struct cif_isp10_pipeline pipe;
};

enum cif_isp10_pix_fmt {
	/* YUV */
	CIF_YUV400 = 0x10008000,

	CIF_YUV420SP = 0x1000c221,	/* NV12 */
	CIF_YUV420P = 0x1000c222,
	CIF_YVU420SP = 0x1000c225,	/* NV21 */
	CIF_YVU420P = 0x1000c226,	/* YV12 */

	CIF_YUV422I = 0x10010240,
	CIF_YUV422SP = 0x10010241,
	CIF_YUV422P = 0x10010242,

	CIF_YUV444SP = 0x10018441,
	CIF_YUV444P = 0x10018442,

	CIF_UYV422I = 0x10010248,

	/* RGB */
	CIF_RGB565 = 0x20010000,
	CIF_RGB666 = 0x20012000,
	CIF_RGB888 = 0x20018000,

	/* RAW Bayer */
	CIF_BAYER_SRGGB10 = 0x3000a030,
	CIF_BAYER_SRGGB12 = 0x3000c030,

	CIF_UNKNOWN_FORMAT = 0x80000000
};

enum cif_fmt_pix_type {
	CIF_FMT_PIX_TYPE_YUV,
	CIF_FMT_PIX_TYPE_RGB,
	CIF_FMT_PIX_TYPE_BAYER,
	CIF_FMT_PIX_TYPE_JPEG,
	CIF_FMT_PIX_TYPE_MAX
};

enum cif_fmt_raw_pat_type {
	CIF_FMT_RAW_PAT_TYPE_BGGR,
	CIF_FMT_RAW_PAT_TYPE_GBBG,
	CIF_FMT_RAW_PAT_TYPE_GRBG,
	CIF_FMT_RAW_PAT_TYPE_RGGB,
	CIF_FMT_RAW_PAT_TYPE_MAX,
};

/*
  * @fourcc: pixel format
  * @mbus_code: pixel format over bus
  * @colorspace
  * @fmt_type: helper filed for pixel format
  * @depth : bits over bus
  * @bpp: bits per pixel
  * @xsubs: horizon subsamps, for yuv
  * @ysubs: vertical subsamps, for yuv
  * @cplanes: number of colour planes
  * @mplanes: number of stored memory planes
  * @uv_swap: for yuv formats
  */
struct cif_fmt_info {
	u32 fourcc;
	u32 mbus_code;
	u8 depth;
	u8 colorspace;
	u8 fmt_type;
	u8 bpp;
	u8 xsubs;
	u8 ysubs;
	u8 uv_swap;
	u8 yc_swap;
	u8 bayer_pat;
	u8 cplanes;
	u8 mplanes;
};

/* help macros */
#define CIF_PIX_FMT_BAYER_PAT_IS_BGGR(pix_fmt) \
	(pix_fmt->bayer_pat == CIF_FMT_RAW_PAT_TYPE_BGGR)
#define CIF_PIX_FMT_BAYER_PAT_IS_GBRG(pix_fmt) \
	(pix_fmt->fbayer_pat == CIF_FMT_RAW_PAT_TYPE_GBBG)
#define CIF_PIX_FMT_BAYER_PAT_IS_GRBG(pix_fmt) \
	(pix_fmt->bayer_pat == CIF_FMT_RAW_PAT_TYPE_GRBG)
#define CIF_PIX_FMT_BAYER_PAT_IS_RGGB(pix_fmt) \
	(pix_fmt->bayer_pat == CIF_FMT_RAW_PAT_TYPE_RGGB)
#define CIF_PIX_FMT_IS_YUV(pix_fmt) (pix_fmt->fmt_type == CIF_FMT_PIX_TYPE_YUV)
#define CIF_PIX_FMT_IS_RGB(pix_fmt) (pix_fmt->fmt_type == CIF_FMT_PIX_TYPE_RGB)
#define CIF_PIX_FMT_IS_RAW_BAYER(pix_fmt) \
	(pix_fmt->fmt_type == CIF_FMT_PIX_TYPE_BAYER)

struct cif_frm_fmt {
	struct cif_fmt_info frm_fmt;
	u32 width;
	u32 height;
	enum v4l2_quantization quantization;
};

enum cif_isp10_pix_fmt_quantization {
	RKISP1_QUANTIZATION_DEFAULT,
	RKISP1_QUANTIZATION_FULL_RANGE,
	RKISP1_QUANTIZATION_LIM_RANGE
};

struct cif_isp10_frm_fmt {
	u32 width;
	u32 height;
	enum cif_isp10_pix_fmt pix_fmt;
	enum cif_isp10_pix_fmt_quantization quantization;
	struct v4l2_rect defrect;
};

struct cif_isp10_fmt {
	char *name;
	u32 fourcc;
	u32 mbus_code;
	u32 cif_fmt;
	int flags;
	int depth;
	unsigned char rotation;
	unsigned char overlay;
};
/* FORMAT */
#define MAX_NB_FORMATS          30

enum cif_isp10_state {
	/* path not yet opened: */
	RKISP1_STATE_DISABLED,
	/* path opened but not yet configured: */
	RKISP1_STATE_INACTIVE,
	/* path opened and configured, ready for streaming: */
	RKISP1_STATE_READY,
	/* path is streaming: */
	RKISP1_STATE_STREAMING
};

static inline
struct cif_isp10_vdev_node *vdev_to_node(struct video_device *vdev)
{
	return container_of(vdev, struct cif_isp10_vdev_node, vdev);
}

static inline struct cif_isp10_vdev_node *queue_to_node(struct vb2_queue *q)
{
	return container_of(q, struct cif_isp10_vdev_node, buf_queue);
}

#endif /* _RKISP1_COMMON_H */
