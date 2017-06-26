/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Jacob Chen <jacob-chen@iotwrt.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __RGA_H__
#define __RGA_H__

#include <linux/platform_device.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define RGA_NAME "rockchip-rga"

struct rga_fmt {
	char *name;
	u32 fourcc;
	int depth;
	u8 uv_factor;
	u8 y_div;
	u8 x_div;
	u8 color_swap;
	u8 hw_format;
};

struct rga_frame {
	/* Original dimensions */
	u32 width;
	u32 height;

	/* Crop */
	struct v4l2_rect crop;

	/* Image format */
	struct rga_fmt *fmt;

	/* Variables that can calculated once and reused */
	u32 stride;
	u32 size;
};

struct rockchip_rga_version {
	u32 major;
	u32 minor;
};

struct rga_buf {
	u32 index;
	u32 type;
	void *mmu_pages;
	struct list_head entry;
};

struct rga_ctx {
	struct v4l2_fh fh;
	struct rockchip_rga *rga;
	struct rga_frame in;
	struct rga_frame out;
	struct v4l2_ctrl_handler ctrl_handler;

	/* Control values */
	u32 op;
	u32 hflip;
	u32 vflip;
	u32 rotate;
	u32 fill_color;
	u32 alpha0;
	u32 alpha1;

	/* CMD Buffers for RGA reading */
	dma_addr_t cmdbuf_phy;
	void *cmdbuf_virt;

	/* Buffers queued for RGA */
	struct list_head buffers_list;
};

struct rockchip_rga {
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device *vfd;

	struct device *dev;
	struct regmap *grf;
	void __iomem *regs;
	struct clk *sclk;
	struct clk *aclk;
	struct clk *hclk;
	struct rockchip_rga_version version;

	struct mutex mutex;
	spinlock_t ctrl_lock;

	wait_queue_head_t irq_queue;

	struct rga_ctx *curr;
	void *alloc_ctx;
};

/* Controls */

#define V4L2_CID_RGA_OP (V4L2_CID_USER_BASE | 0x1001)
#define V4L2_CID_RGA_ALHPA_REG0 (V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_RGA_ALHPA_REG1 (V4L2_CID_USER_BASE | 0x1003)

/* Operation values */
#define OP_COPY 0
#define OP_SOLID_FILL 1
#define OP_ALPHA_BLEND 2

struct rga_frame *rga_get_frame(struct rga_ctx *ctx, enum v4l2_buf_type type);

/* RGA Buffers Manage Part */
extern const struct vb2_ops rga_qops;
void *rga_buf_find_page(struct vb2_buffer *vb);
void rga_buf_clean(struct rga_ctx *ctx);

/* RGA Hardware Part */
void rga_write(struct rockchip_rga *rga, u32 reg, u32 value);
u32 rga_read(struct rockchip_rga *rga, u32 reg);
void rga_mod(struct rockchip_rga *rga, u32 reg, u32 val, u32 mask);
void rga_start(struct rockchip_rga *rga);
void rga_cmd_set(struct rga_ctx *ctx, void *src_mmu_pages, void *dst_mmu_pages);

#endif
