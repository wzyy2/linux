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

#ifndef _RKISP1_PATH_VIDEO_H
#define _RKISP1_PATH_VIDEO_H

#include "common.h"

struct rkisp1_device;

struct rkisp1_mi_path_config {
	struct cif_frm_fmt output;
	u32 llength;
	u32 curr_buff_addr;
	u32 curr_buff_addr_cb;
	u32 curr_buff_addr_cr;
	u32 next_buff_addr;
	u32 next_buff_addr_cb;
	u32 next_buff_addr_cr;
	u32 y_size;
	u32 cb_size;
	u32 cr_size;
	bool busy;
};

struct rkisp1_mi_config {
	bool raw_enable;
	u32 async_updt;
};

struct rkisp1_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head queue;
};

enum rkisp1_buff_fmt {
	/* values correspond to bitfield values */
	RKISP1_BUFF_FMT_PLANAR = 0,
	RKISP1_BUFF_FMT_SEMIPLANAR = 1,
	RKISP1_BUFF_FMT_INTERLEAVED = 2,

	RKISP1_BUFF_FMT_RAW8 = 0,
	RKISP1_BUFF_FMT_RAW12 = 2
};


struct rkisp1_stream {
	struct rkisp1_vdev_node vnode;
	struct rkisp1_mi_path_config path_cfg;
	enum rkisp1_stream_id id;
	enum rkisp1_state state;
	enum rkisp1_state saved_state;
	struct list_head buf_queue;
	struct rkisp1_buffer *curr_buf;
	struct rkisp1_buffer *next_buf;
	bool updt_cfg;
	bool stall;
	bool stop;
	bool restart;
	wait_queue_head_t done;
};

struct rkisp1_stream_vdevs {
	struct rkisp1_stream mp_vdev;
	struct rkisp1_stream sp_vdev;
	struct rkisp1_mi_config mi_config;
	spinlock_t vbq_lock;	/* spinlock for videobuf queues */
};

int rkisp1_register_videodev(struct rkisp1_device *dev,
				enum rkisp1_stream_id stream_id);

void rkisp1_unregister_video_device(struct rkisp1_stream *stream);

void rkisp1_mi_isr(unsigned int mi_mis, struct rkisp1_device *dev);

int rkisp1_stream_init(struct rkisp1_device *dev, u32 stream_ids);
int rkisp1_stream_release(struct rkisp1_device *dev, int stream_ids);

int rkisp1_streamon(struct rkisp1_device *dev, u32 stream_ids);
int rkisp1_streamoff(struct rkisp1_device *dev, u32 stream_ids);

int isp_pipeline_open(struct cif_isp10_pipeline *p,
		      struct media_entity *me, bool prepare);
int isp_pipeline_close(struct cif_isp10_pipeline *p);
int isp_pipeline_set_stream(struct cif_isp10_pipeline *p, bool on);

#endif /* _RKISP1_PATH_VIDEO_H */
