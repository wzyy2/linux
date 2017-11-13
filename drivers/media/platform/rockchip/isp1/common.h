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
#include <media/videobuf2-v4l2.h>

#define DRIVER_NAME "rkisp1"
#define ISP_VDEV_NAME DRIVER_NAME  "_ispdev"
#define SP_VDEV_NAME DRIVER_NAME   "_selfpath"
#define MP_VDEV_NAME DRIVER_NAME   "_mainpath"
#define DMA_VDEV_NAME DRIVER_NAME  "_dmapath"

#define GRP_ID_SENSOR			BIT(0)
#define GRP_ID_MIPIPHY			BIT(1)
#define GRP_ID_ISP			BIT(2)
#define GRP_ID_ISP_MP			BIT(3)
#define GRP_ID_ISP_SP			BIT(4)

#define RKISP1_DEFAULT_WIDTH		800
#define RKISP1_DEFAULT_HEIGHT		600

#define RKISP1_MAX_STREAM		2
#define RKISP1_STREAM_SP		0
#define RKISP1_STREAM_MP		1

#define RKISP1_PLANE_Y			0
#define RKISP1_PLANE_CB			1
#define RKISP1_PLANE_CR			2

#define RKISP1_PIPELINE_MAX		4

enum rkisp1_sd_type {
	RKISP1_SD_SENSOR,
	RKISP1_SD_PHY_CSI,
	RKISP1_SD_VCM,
	RKISP1_SD_FLASH,
	RKISP1_SD_MAX,
};

struct rkisp1_pipeline;

/*
 * struct rkisp1_pipeline - An ISP hardware pipeline
 * @entities: Bitmask of entities in the pipeline (indexed by entity ID)
 */
struct rkisp1_pipeline {
	struct media_pipeline pipe;
	int num_subdevs;
	struct v4l2_subdev *subdevs[RKISP1_PIPELINE_MAX];
	int (*open)(struct rkisp1_pipeline *p,
		    struct media_entity *me, bool prepare);
	int (*close)(struct rkisp1_pipeline *p);
	int (*set_stream)(struct rkisp1_pipeline *p, bool on);
};

/* One structure per video node */
struct rkisp1_vdev_node {
	struct vb2_queue buf_queue;
	/* vfd lock */
	struct mutex vlock;
	struct video_device vdev;
	struct media_pad pad;
};

enum rkisp1_fmt_pix_type {
	FMT_YUV,
	FMT_RGB,
	FMT_BAYER,
	FMT_JPEG,
	FMT_MAX
};

enum rkisp1_fmt_raw_pat_type {
	RAW_RGGB = 0,
	RAW_GRBG,
	RAW_GBRG,
	RAW_BGGR,
};

struct ispsd_in_fmt {
	u32 mbus_code;
	u8 fmt_type;
	u32 mipi_dt;
	u32 yuv_seq;
	enum rkisp1_fmt_raw_pat_type bayer_pat;
	u8 bus_width;
};

struct ispsd_out_fmt {
	u32 mbus_code;
	u8 fmt_type;
};

/*
 * @fourcc: pixel format
 * @mbus_code: pixel format over bus
 * @fmt_type: helper filed for pixel format
 * @bpp: bits per pixel
 * @bayer_pat: bayer patten type
 * @cplanes: number of colour planes
 * @mplanes: number of stored memory planes
 * @uv_swap: if cb cr swaped, for yuv
 * @write_format: defines how YCbCr self picture data is written to memory
 * @input_format: defines sp input format
 * @output_format: defines sp output format
 */
struct capture_fmt {
	u32 fourcc;
	u32 mbus_code;
	u8 fmt_type;
	u8 cplanes;
	u8 mplanes;
	u8 uv_swap;
	u32 write_format;
	u32 output_format;
	u8 bpp[VIDEO_MAX_PLANES];
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
	union {
		u32 buff_addr[VIDEO_MAX_PLANES];
		void *vaddr[VIDEO_MAX_PLANES];
	};
};

struct rkisp1_dummy_buffer {
	void *vaddr;
	dma_addr_t dma_addr;
	u32 size;
};

enum rkisp1_sp_inp {
	RKISP1_SP_INP_ISP,
	RKISP1_SP_INP_DMA_SP,
	RKISP1_SP_INP_MAX
};

struct rkisp1_device;
struct rkisp1_stream;

struct rkisp1_stream_sp {
	int y_stride;
	enum rkisp1_sp_inp input_sel;
};

struct rkisp1_stream_mp {
	bool raw_enable;
};

struct stream_config {
	const struct capture_fmt *fmts;
	int fmt_size;
	/* constrains */
	const int max_rsz_width;
	const int max_rsz_height;
	const int min_rsz_width;
	const int min_rsz_height;
	/* registers */
	struct {
		u32 ctrl;
		u32 ctrl_shd;
		u32 scale_hy;
		u32 scale_hcr;
		u32 scale_hcb;
		u32 scale_vy;
		u32 scale_vc;
		u32 scale_lut;
		u32 scale_lut_addr;
		u32 scale_hy_shd;
		u32 scale_hcr_shd;
		u32 scale_hcb_shd;
		u32 scale_vy_shd;
		u32 scale_vc_shd;
		u32 phase_hy;
		u32 phase_hc;
		u32 phase_vy;
		u32 phase_vc;
		u32 phase_hy_shd;
		u32 phase_hc_shd;
		u32 phase_vy_shd;
		u32 phase_vc_shd;
	} rsz;
	struct {
		u32 ctrl;
		u32 yuvmode_mask;
		u32 rawmode_mask;
		u32 h_offset;
		u32 v_offset;
		u32 h_size;
		u32 v_size;
	} dual_crop;
	struct {
		u32 y_size_init;
		u32 cb_size_init;
		u32 cr_size_init;
		u32 y_base_ad_init;
		u32 cb_base_ad_init;
		u32 cr_base_ad_init;
		u32 y_offs_cnt_init;
		u32 cb_offs_cnt_init;
		u32 cr_offs_cnt_init;
	} mi;
};

struct streams_ops {
	int (*check_against)(struct rkisp1_stream *stream);
	int (*config_mi)(struct rkisp1_stream *stream);
	void (*stop_mi)(struct rkisp1_stream *stream);
	void (*enable_mi)(struct rkisp1_stream *stream);
	void (*disable_mi)(struct rkisp1_stream *stream);
	void (*set_data_path)(void __iomem *base);
	void (*clr_frame_end_int)(void __iomem *base);
	u32 (*is_frame_end_int_masked)(void __iomem *base);
};

struct rkisp1_stream {
	u32 id;
	struct rkisp1_device *ispdev;
	struct rkisp1_vdev_node vnode;
	enum rkisp1_state state;
	enum rkisp1_state saved_state;
	struct capture_fmt out_isp_fmt;
	struct v4l2_pix_format_mplane out_fmt;
	struct v4l2_rect dcrop;
	struct streams_ops *ops;
	struct stream_config *config;
	/* spinlock for videobuf queues */
	spinlock_t vbq_lock;
	/* mi config */
	struct list_head buf_queue;
	struct rkisp1_dummy_buffer dummy_buf;
	struct rkisp1_buffer *curr_buf;
	struct rkisp1_buffer *next_buf;
	bool stop;
	wait_queue_head_t done;
	union {
		struct rkisp1_stream_sp sp;
		struct rkisp1_stream_mp mp;
	} u;
};

extern int rkisp1_debug;

static inline u32 rkisp1_get_colorspace(u32 fmt_type)
{
	switch (fmt_type) {
	case FMT_BAYER:
		return V4L2_COLORSPACE_SRGB;
	case FMT_YUV:
	case FMT_JPEG:
		return V4L2_COLORSPACE_JPEG;
	case FMT_RGB:
		return V4L2_COLORSPACE_RAW;
	default:
		return V4L2_COLORSPACE_DEFAULT;
	}
}

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
