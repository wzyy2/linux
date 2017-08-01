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
#include "platform.h"

struct rkisp1_device;

struct cif_isp10_mi_path_config {
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

struct cif_isp10_mi_config {
	bool raw_enable;
	u32 async_updt;
};

struct cif_isp10_buffer {
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


struct cif_isp10_stream {
	struct cif_isp10_vdev_node vnode;
	struct cif_isp10_mi_path_config path_cfg;
	enum cif_isp10_stream_id id;
	enum cif_isp10_state state;
	enum cif_isp10_state saved_state;
	struct list_head buf_queue;
	struct cif_isp10_buffer *curr_buf;
	struct cif_isp10_buffer *next_buf;
	bool updt_cfg;
	bool stall;
	bool stop;
	bool restart;
	wait_queue_head_t done;
};

struct cif_isp10_stream_vdevs {
	struct cif_isp10_stream mp_vdev;
	struct cif_isp10_stream sp_vdev;
	struct cif_isp10_mi_config mi_config;
	spinlock_t vbq_lock;	/* spinlock for videobuf queues */
};

int cif_isp10_register_videodev(struct rkisp1_device *dev,
				enum cif_isp10_stream_id stream_id);

void cif_isp10_unregister_videodev(struct rkisp1_device *dev,
				   enum cif_isp10_stream_id stream_id);

void cif_isp10_v4l2_event(struct rkisp1_device *dev, __u32 frame_sequence);

void cif_isp10_mi_isr(unsigned int mi_mis, void *cntxt);

int cif_isp10_stream_init(struct rkisp1_device *dev, u32 stream_ids);
int cif_isp10_stream_release(struct rkisp1_device *dev, int stream_ids);

int cif_isp10_streamon(struct rkisp1_device *dev, u32 stream_ids);
int cif_isp10_streamoff(struct rkisp1_device *dev, u32 stream_ids);

/* from cif_isp10_dev.c */
int isp_pipeline_open(struct cif_isp10_pipeline *p,
		      struct media_entity *me, bool prepare);
int isp_pipeline_close(struct cif_isp10_pipeline *p);
int isp_pipeline_set_stream(struct cif_isp10_pipeline *p, bool on);

#endif /* _RKISP1_PATH_VIDEO_H */
