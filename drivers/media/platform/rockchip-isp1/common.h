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

#ifndef _RKISP1_COMMON_H
#define _RKISP1_COMMON_H

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>

#define DRIVER_NAME "rkisp1"
#define ISP_VDEV_NAME DRIVER_NAME  "_ispdev"
#define SP_VDEV_NAME DRIVER_NAME   "_selfpath"
#define MP_VDEV_NAME DRIVER_NAME   "_mainpath"
#define DMA_VDEV_NAME DRIVER_NAME  "_dmapath"

#define GRP_ID_SENSOR			BIT( 0)
#define GRP_ID_MIPIPHY			BIT( 1)
#define GRP_ID_ISP				BIT( 2)
#define GRP_ID_ISP_MP			BIT( 3)
#define GRP_ID_ISP_SP			BIT( 4)

#define RKISP1_DEFAULT_WIDTH 800
#define RKISP1_DEFAULT_HEIGHT 600

#define RKISP1_STREAM_SP	BIT(0)
#define RKISP1_STREAM_MP	BIT(1)
#define RKISP1_STREAM_ALL	(RKISP1_STREAM_SP | RKISP1_STREAM_MP)

enum isp_subdev_index {
	IDX_SENSOR,
	IDX_MIPIPHY,
	IDX_ISP,
	IDX_MAX,
};

enum cif_isp10_sd_type {
	RKISP1_SD_SENSOR,
	RKISP1_SD_PHY_CSI,
	RKISP1_SD_VCM,
	RKISP1_SD_FLASH,
	RKISP1_SD_MAX,
};

struct rkisp1_pipeline;

typedef int (*rkisp1_pipeline_open) (struct rkisp1_pipeline *p,
					struct media_entity *me, bool prepare);

typedef int (*rkisp1_pipeline_close) (struct rkisp1_pipeline *p);

typedef int (*rkisp1_pipeline_set_stream) (struct rkisp1_pipeline *p,
					      bool on);

/*
 * struct rkisp1_pipeline - An ISP hardware pipeline
 * @entities: Bitmask of entities in the pipeline (indexed by entity ID)
 */
struct rkisp1_pipeline {
	struct media_pipeline pipe;
	struct v4l2_subdev *subdevs[IDX_MAX];
	rkisp1_pipeline_open open;
	rkisp1_pipeline_close close;
	rkisp1_pipeline_set_stream set_stream;
};

/* One structure per video node */
struct rkisp1_vdev_node {
	struct vb2_queue buf_queue;
	struct mutex vlock;
	struct video_device vdev;
	struct media_pad pad;
	struct rkisp1_pipeline pipe;
};

enum cif_fmt_pix_type {
	FMT_YUV,
	FMT_RGB,
	FMT_BAYER,
	FMT_JPEG,
	FMT_MAX
};

enum cif_fmt_raw_pat_type {
	RAW_BGGR,
	RAW_GBRG,
	RAW_GRBG,
	RAW_RGGB,
	RAW_MAX,
};

/*
  * @fourcc: pixel format
  * @mbus_code: pixel format over bus
  * @colorspace: enum of v4l2_colorspace
  * @fmt_type: helper filed for pixel format
  * @bpp: bits per pixel
  * @xsubs: horizon color samples in a 4*4 matrix, for yuv
  * @ysubs: vertical color samples in a 4*4 matrix, for yuv
  * @bayer_pat: bayer patten type
  * @cplanes: number of colour planes
  * @mplanes: number of stored memory planes
  * @uv_swap: if cb cr swaped, for yuv
  * @yc_swap: if cb/cr comes before y, for yuv
  * @write_format: defines how YCbCr self picture data is written to memory
  * @input_format: defines sp input format
  * @output_format: defines sp output format
  */
struct rkisp1_fmt {
	u32 fourcc;
	u32 mbus_code;
	u8 colorspace;
	u8 fmt_type;
	u8 bpp;
	u8 xsubs;
	u8 ysubs;
	u8 bayer_pat;
	u8 cplanes;
	u8 mplanes;
	u8 uv_swap;
	u8 yc_swap;
	u32 write_format;
	u32 input_format;
	u32 output_format;
	u32 mipi_data_type;
};

struct cif_frm_fmt {
	const struct rkisp1_fmt *fmt;
	struct v4l2_mbus_framefmt mbus;
	enum v4l2_quantization quantization;
};

enum rkisp1_state {
	/* path not yet opened: */
	RKISP1_STATE_DISABLED,
	/* path opened and configured, ready for streaming: */
	RKISP1_STATE_READY,
	/* path is streaming: */
	RKISP1_STATE_STREAMING
};

struct rkisp1_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head queue;
};

struct rkisp1_win {
	int w;
	int h;
};

static inline
struct rkisp1_vdev_node *vdev_to_node(struct video_device *vdev)
{
	return container_of(vdev, struct rkisp1_vdev_node, vdev);
}

static inline struct rkisp1_vdev_node *queue_to_node(struct vb2_queue *q)
{
	return container_of(q, struct rkisp1_vdev_node, buf_queue);
}

static inline struct rkisp1_buffer *to_rkisp1_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct rkisp1_buffer, vb);
}

static inline struct vb2_queue *to_vb2_queue(struct file *file)
{
	struct rkisp1_vdev_node *vnode = video_drvdata(file);

	return &vnode->buf_queue;
}

#endif /* _RKISP1_COMMON_H */
