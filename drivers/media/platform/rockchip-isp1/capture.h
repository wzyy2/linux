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

#ifndef _RKISP1_PATH_VIDEO_H
#define _RKISP1_PATH_VIDEO_H

#include "common.h"

enum rkisp1_sp_inp {
	RKISP1_SP_INP_ISP,
	RKISP1_SP_INP_DMA_SP,
	RKISP1_SP_INP_MAX
};

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
};

struct rkisp1_stream;
struct streams_ops {
	void (*stream_init)(struct rkisp1_stream *stream);
	int (*check_against)(struct rkisp1_stream *stream);
	int (*config_mi)(struct rkisp1_stream *stream);
	int (*update_mi)(struct rkisp1_stream *stream);
	int (*stop_mi)(struct rkisp1_stream *stream);
	int (*set_data_path)(struct rkisp1_stream *stream);
	int (*calc_size)(struct rkisp1_stream *stream);
	void (*config_dcrop)(void __iomem *base, struct v4l2_rect *rect, bool async);
	void (*disable_dcrop)(void __iomem *base, bool async);
	void (*disable_rsz)(void __iomem *base, bool async);
	void (*set_phase)(void __iomem *base);
	void (*set_lut)(void __iomem *base);
	void (*set_scale)(void __iomem *base, struct rkisp1_win *in_y, struct rkisp1_win *in_c, struct rkisp1_win *out_y, struct rkisp1_win *out_c);
	void (*update_shadow_reg)(void __iomem *base);
	void (*dump_rsz_regs)(void __iomem *base);
	void (*clr_frame_end_int)(void __iomem *base);
	u32 (*is_frame_end_int_masked)(void __iomem *base);
	u32 (*get_y_offset_counter_shd)(void __iomem *base);
	struct rkisp1_stream *(*get_other_stream)(struct rkisp1_stream *stream);
};

struct rkisp1_stream {
	struct rkisp1_vdev_node vnode;
	struct rkisp1_mi_path_config path_cfg;
	u32 id;
	enum rkisp1_state state;
	enum rkisp1_state saved_state;
	struct list_head buf_queue;
	struct rkisp1_buffer *curr_buf;
	struct rkisp1_buffer *next_buf;
	bool stall;
	bool stop;
	wait_queue_head_t done;
	struct rkisp1_device *ispdev;
	const struct rkisp1_fmt *fmts;
	int fmt_size;
	struct v4l2_rect dcrop;
	/* spinlock for videobuf queues */
	spinlock_t vbq_lock;
	/* SP Only */
	enum rkisp1_sp_inp input_sel;
	bool raw_enable;
	struct streams_ops *ops;
};

void rkisp1_unregister_stream_vdevs(struct rkisp1_device *dev);
int rkisp1_register_stream_vdevs(struct rkisp1_device *dev);
void rkisp1_mi_isr(struct rkisp1_stream *stream);
int rkisp1_stream_init(struct rkisp1_stream *stream, u32 id);
int rkisp1_stream_release(struct rkisp1_stream *stream);

#endif /* _RKISP1_PATH_VIDEO_H */
