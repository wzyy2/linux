// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip isp1 driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 */

#ifndef _RKISP1_COMMON_H
#define _RKISP1_COMMON_H

#include <linux/mutex.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>

#define RKISP1_DEFAULT_WIDTH		800
#define RKISP1_DEFAULT_HEIGHT		600

#define RKISP1_MAX_STREAM		2
#define RKISP1_STREAM_SP		0
#define RKISP1_STREAM_MP		1

#define RKISP1_PLANE_Y			0
#define RKISP1_PLANE_CB			1
#define RKISP1_PLANE_CR			2

enum rkisp1_sd_type {
	RKISP1_SD_SENSOR,
	RKISP1_SD_PHY_CSI,
	RKISP1_SD_VCM,
	RKISP1_SD_FLASH,
	RKISP1_SD_MAX,
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

extern int rkisp1_debug;

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
