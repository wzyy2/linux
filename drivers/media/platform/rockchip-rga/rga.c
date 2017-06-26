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

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-v4l2.h>

#include "rga-hw.h"
#include "rga.h"

static void job_abort(void *prv)
{
	struct rga_ctx *ctx = prv;
	struct rockchip_rga *rga = ctx->rga;

	if (rga->curr == NULL)	/* No job currently running */
		return;

	wait_event_timeout(rga->irq_queue,
	                   rga->curr == NULL, msecs_to_jiffies(RGA_TIMEOUT));
}

static void device_run(void *prv)
{
	struct rga_ctx *ctx = prv;
	struct rockchip_rga *rga = ctx->rga;
	struct vb2_buffer *src, *dst;
	unsigned long flags;

	spin_lock_irqsave(&rga->ctrl_lock, flags);

	rga->curr = ctx;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	rga_cmd_set(ctx, rga_buf_find_page(src), rga_buf_find_page(dst));

	rga_start(rga);

	spin_unlock_irqrestore(&rga->ctrl_lock, flags);
}

static irqreturn_t rga_isr(int irq, void *prv)
{
	struct rockchip_rga *rga = prv;
	int intr;

	intr = rga_read(rga, RGA_INT) & 0xf;

	rga_mod(rga, RGA_INT, intr << 4, 0xf << 4);

	if (intr & 0x04) {
		struct vb2_v4l2_buffer *src, *dst;
		struct rga_ctx *ctx = rga->curr;

		BUG_ON(ctx == NULL);

		rga->curr = NULL;

		src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		BUG_ON(src == NULL);
		BUG_ON(dst == NULL);

		dst->timecode = src->timecode;
		dst->timestamp = src->timestamp;
		dst->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
		dst->flags |= src->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

		v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
		v4l2_m2m_job_finish(rga->m2m_dev, ctx->fh.m2m_ctx);

		wake_up(&rga->irq_queue);
	}

	return IRQ_HANDLED;
}

static struct v4l2_m2m_ops rga_m2m_ops = {
	.device_run = device_run,
	.job_abort = job_abort,
};

static int
queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct rga_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &rga_qops;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->rga->mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &rga_qops;
	dst_vq->mem_ops = &vb2_dma_sg_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->rga->mutex;

	return vb2_queue_init(dst_vq);
}

static int rga_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rga_ctx *ctx = container_of(ctrl->handler, struct rga_ctx,
	                                   ctrl_handler);
	unsigned long flags;

	spin_lock_irqsave(&ctx->rga->ctrl_lock, flags);
	switch (ctrl->id) {
	case V4L2_CID_RGA_OP:
		ctx->op = ctrl->val;
		break;
	case V4L2_CID_HFLIP:
		ctx->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		ctx->vflip = ctrl->val;
		break;
	case V4L2_CID_ROTATE:
		ctx->rotate = ctrl->val;
		break;
	case V4L2_CID_BG_COLOR:
		ctx->fill_color = ctrl->val;
		break;
	case V4L2_CID_RGA_ALHPA_REG0:
		ctx->alpha0 = ctrl->val;
		break;
	case V4L2_CID_RGA_ALHPA_REG1:
		ctx->alpha1 = ctrl->val;
		break;
	}
	spin_unlock_irqrestore(&ctx->rga->ctrl_lock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops rga_ctrl_ops = {
	.s_ctrl = rga_s_ctrl,
};

static const struct v4l2_ctrl_config op_control = {
	.ops = &rga_ctrl_ops,
	.id = V4L2_CID_RGA_OP,
	.name = "Transform operation",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0,
	.max = 0xf,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config alpha0_control = {
	.ops = &rga_ctrl_ops,
	.id = V4L2_CID_RGA_ALHPA_REG0,
	.name = "Exposed alpha blending register 0",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0,
	.max = 0xffffffff,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config alpha1_control = {
	.ops = &rga_ctrl_ops,
	.id = V4L2_CID_RGA_ALHPA_REG1,
	.name = "Exposed alpha blending register 1",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0,
	.max = 0xffffffff,
	.step = 1,
	.def = 0,
};

static int rga_setup_ctrls(struct rga_ctx *ctx)
{
	struct rockchip_rga *rga = ctx->rga;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 7);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &rga_ctrl_ops,
	                  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &rga_ctrl_ops,
	                  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &rga_ctrl_ops,
	                  V4L2_CID_ROTATE, 0, 270, 90, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &rga_ctrl_ops,
	                  V4L2_CID_BG_COLOR, 0, 0xffffffff, 1, 0);

	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &op_control, NULL);
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &alpha0_control, NULL);
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &alpha1_control, NULL);

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;
		v4l2_err(&rga->v4l2_dev, "%s failed\n", __func__);
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	return 0;
}

struct rga_fmt formats[] = {
	{
		.name = "ARGB_8888",
		.fourcc = V4L2_PIX_FMT_ARGB32,
		.color_swap = RGA_COLOR_RB_SWAP,
		.hw_format = RGA_COLOR_FMT_ABGR8888,
		.depth = 32,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "XRGB_8888",
		.fourcc = V4L2_PIX_FMT_XRGB32,
		.color_swap = RGA_COLOR_RB_SWAP,
		.hw_format = RGA_COLOR_FMT_XBGR8888,
		.depth = 32,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "BGRA_8888",
		.fourcc = V4L2_PIX_FMT_ABGR32,
		.color_swap = RGA_COLOR_ALPHA_SWAP,
		.hw_format = RGA_COLOR_FMT_ABGR8888,
		.depth = 32,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "BGRX_8888",
		.fourcc = V4L2_PIX_FMT_XBGR32,
		.color_swap = RGA_COLOR_ALPHA_SWAP,
		.hw_format = RGA_COLOR_FMT_XBGR8888,
		.depth = 32,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "RGB_888",
		.fourcc = V4L2_PIX_FMT_RGB24,
		.color_swap = RGA_COLOR_RB_SWAP,
		.hw_format = RGA_COLOR_FMT_BGR888,
		.depth = 24,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "ARGB_444",
		.fourcc = V4L2_PIX_FMT_ARGB444,
		.color_swap = RGA_COLOR_RB_SWAP,
		.hw_format = RGA_COLOR_FMT_ABGR4444,
		.depth = 16,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "ARGB_1555",
		.fourcc = V4L2_PIX_FMT_ARGB555,
		.color_swap = RGA_COLOR_RB_SWAP,
		.hw_format = RGA_COLOR_FMT_ABGR1555,
		.depth = 16,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "RGB_565",
		.fourcc = V4L2_PIX_FMT_RGB565,
		.color_swap = RGA_COLOR_RB_SWAP,
		.hw_format = RGA_COLOR_FMT_BGR565,
		.depth = 16,
		.uv_factor = 1,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "NV_21",
		.fourcc = V4L2_PIX_FMT_NV21,
		.color_swap = RGA_COLOR_UV_SWAP,
		.hw_format = RGA_COLOR_FMT_YUV420SP,
		.depth = 12,
		.uv_factor = 4,
		.y_div = 2,
		.x_div = 1,
	},
	{
		.name = "NV_61",
		.fourcc = V4L2_PIX_FMT_NV61,
		.color_swap = RGA_COLOR_UV_SWAP,
		.hw_format = RGA_COLOR_FMT_YUV422SP,
		.depth = 16,
		.uv_factor = 2,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "NV_12",
		.fourcc = V4L2_PIX_FMT_NV12,
		.color_swap = RGA_COLOR_NONE_SWAP,
		.hw_format = RGA_COLOR_FMT_YUV420SP,
		.depth = 12,
		.uv_factor = 4,
		.y_div = 2,
		.x_div = 1,
	},
	{
		.name = "NV_16",
		.fourcc = V4L2_PIX_FMT_NV16,
		.color_swap = RGA_COLOR_NONE_SWAP,
		.hw_format = RGA_COLOR_FMT_YUV422SP,
		.depth = 16,
		.uv_factor = 2,
		.y_div = 1,
		.x_div = 1,
	},
	{
		.name = "YUV_420",
		.fourcc = V4L2_PIX_FMT_YUV420,
		.color_swap = RGA_COLOR_NONE_SWAP,
		.hw_format = RGA_COLOR_FMT_YUV420P,
		.depth = 12,
		.uv_factor = 4,
		.y_div = 2,
		.x_div = 2,
	},
	{
		.name = "YUV_422",
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.color_swap = RGA_COLOR_NONE_SWAP,
		.hw_format = RGA_COLOR_FMT_YUV422P,
		.depth = 16,
		.uv_factor = 2,
		.y_div = 1,
		.x_div = 2,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

struct rga_fmt *rga_fmt_find(struct v4l2_format *f)
{
	unsigned int i;
	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix.pixelformat)
			return &formats[i];
	}
	return NULL;
}

static struct rga_frame def_frame = {
	.width = DEFAULT_WIDTH,
	.height = DEFAULT_HEIGHT,
	.crop.left = 0,
	.crop.top = 0,
	.crop.width = DEFAULT_WIDTH,
	.crop.height = DEFAULT_HEIGHT,
	.fmt = &formats[0],
};

struct rga_frame *rga_get_frame(struct rga_ctx *ctx, enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->in;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->out;
	default:
		return ERR_PTR(-EINVAL);
	}
}

static int rga_open(struct file *file)
{
	struct dma_attrs cmdlist_dma_attrs;
	struct rockchip_rga *rga = video_drvdata(file);
	struct rga_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->rga = rga;
	/* Set default formats */
	ctx->in = def_frame;
	ctx->out = def_frame;

	if (mutex_lock_interruptible(&rga->mutex)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(rga->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		mutex_unlock(&rga->mutex);
		kfree(ctx);
		return ret;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	rga_setup_ctrls(ctx);

	/* Write the default values to the ctx struct */
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	mutex_unlock(&rga->mutex);

	/* Create CMD buffer */
	init_dma_attrs(&cmdlist_dma_attrs);
	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &cmdlist_dma_attrs);
	ctx->cmdbuf_virt = dma_alloc_attrs(rga->dev, RGA_CMDBUF_SIZE,
	                                   &ctx->cmdbuf_phy, GFP_KERNEL,
	                                   &cmdlist_dma_attrs);

	/* Init RGA private buffer list */
	INIT_LIST_HEAD(&ctx->buffers_list);

	pm_runtime_get_sync(rga->dev);

	v4l2_info(&rga->v4l2_dev, "instance opened\n");
	return 0;
}

static int rga_release(struct file *file)
{
	struct rockchip_rga *rga = video_drvdata(file);
	struct rga_ctx *ctx =
	    container_of(file->private_data, struct rga_ctx, fh);

	pm_runtime_put(rga->dev);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	if (!list_empty(&ctx->buffers_list)) {
		v4l2_info(&rga->v4l2_dev, "close with unreleased buffers\n");
		rga_buf_clean(ctx);
	}
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	v4l2_info(&rga->v4l2_dev, "instance closed\n");
	return 0;
}

static const struct v4l2_file_operations rga_fops = {
	.owner = THIS_MODULE,
	.open = rga_open,
	.release = rga_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int
vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strncpy(cap->driver, RGA_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, RGA_NAME, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_enum_fmt(struct file *file, void *prv, struct v4l2_fmtdesc *f)
{
	struct rga_fmt *fmt;
	if (f->index >= NUM_FORMATS)
		return -EINVAL;
	fmt = &formats[f->index];
	f->pixelformat = fmt->fourcc;
	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	return 0;
}

static int vidioc_g_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct rga_ctx *ctx = prv;
	struct vb2_queue *vq;
	struct rga_frame *frm;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;
	frm = rga_get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);

	f->fmt.pix.width = frm->width;
	f->fmt.pix.height = frm->height;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat = frm->fmt->fourcc;
	f->fmt.pix.bytesperline = frm->stride;
	f->fmt.pix.sizeimage = frm->size;

	return 0;
}

static int vidioc_try_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct rga_fmt *fmt;
	enum v4l2_field *field;

	fmt = rga_fmt_find(f);
	if (!fmt)
		return -EINVAL;

	field = &f->fmt.pix.field;
	if (*field == V4L2_FIELD_ANY)
		*field = V4L2_FIELD_NONE;
	else if (*field != V4L2_FIELD_NONE)
		return -EINVAL;

	if (f->fmt.pix.width > MAX_WIDTH)
		f->fmt.pix.width = MAX_WIDTH;
	if (f->fmt.pix.height > MAX_HEIGHT)
		f->fmt.pix.height = MAX_HEIGHT;

	if (f->fmt.pix.width < MIN_WIDTH)
		f->fmt.pix.width = MIN_WIDTH;
	if (f->fmt.pix.height < MIN_HEIGHT)
		f->fmt.pix.height = MIN_HEIGHT;

	if (fmt->hw_format >= RGA_COLOR_FMT_YUV422SP)
		f->fmt.pix.bytesperline = f->fmt.pix.width;
	else
		f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;

	f->fmt.pix.sizeimage =
	    f->fmt.pix.height * (f->fmt.pix.width * fmt->depth) >> 3;

	return 0;
}

static int vidioc_s_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct rga_ctx *ctx = prv;
	struct rockchip_rga *rga = ctx->rga;
	struct vb2_queue *vq;
	struct rga_frame *frm;
	struct rga_fmt *fmt;
	int ret = 0;

	/* Adjust all values accordingly to the hardware capabilities
	 * and chosen format. */
	ret = vidioc_try_fmt(file, prv, f);
	if (ret)
		return ret;
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&rga->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}
	frm = rga_get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);
	fmt = rga_fmt_find(f);
	if (!fmt)
		return -EINVAL;
	frm->width = f->fmt.pix.width;
	frm->height = f->fmt.pix.height;
	frm->size = f->fmt.pix.sizeimage;
	frm->fmt = fmt;
	frm->stride = f->fmt.pix.bytesperline;

	/* Reset crop settings */
	frm->crop.left = 0;
	frm->crop.top = 0;
	frm->crop.width = frm->width;
	frm->crop.height = frm->height;
	return 0;
}

static int
vidioc_cropcap(struct file *file, void *priv, struct v4l2_cropcap *cr)
{
	struct rga_ctx *ctx = priv;
	struct rga_frame *f;

	f = rga_get_frame(ctx, cr->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	cr->bounds.left = 0;
	cr->bounds.top = 0;
	cr->bounds.width = f->width;
	cr->bounds.height = f->height;
	cr->defrect = cr->bounds;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *prv, struct v4l2_crop *cr)
{
	struct rga_ctx *ctx = prv;
	struct rga_frame *f;

	f = rga_get_frame(ctx, cr->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	cr->c = f->crop;

	return 0;
}

static int
vidioc_try_crop(struct file *file, void *prv, const struct v4l2_crop *cr)
{
	struct rga_ctx *ctx = prv;
	struct rockchip_rga *rga = ctx->rga;
	struct rga_frame *f;

	f = rga_get_frame(ctx, cr->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	if (cr->c.top < 0 || cr->c.left < 0) {
		v4l2_err(&rga->v4l2_dev,
		         "doesn't support negative values for top & left. \n");
		return -EINVAL;
	}

	if (cr->c.left + cr->c.width > f->width ||
	    cr->c.top + cr->c.height > f->height ||
	    cr->c.width < MIN_WIDTH || cr->c.height < MIN_HEIGHT) {
		v4l2_err(&rga->v4l2_dev, "unsupport crop value. \n");
		return -EINVAL;
	}

	return 0;
}

static int
vidioc_s_crop(struct file *file, void *prv, const struct v4l2_crop *cr)
{
	struct rga_ctx *ctx = prv;
	struct rga_frame *f;
	int ret;

	ret = vidioc_try_crop(file, prv, cr);
	if (ret)
		return ret;
	f = rga_get_frame(ctx, cr->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	f->crop = cr->c;

	return 0;
}

static const struct v4l2_ioctl_ops rga_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap = vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap = vidioc_s_fmt,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_out = vidioc_g_fmt,
	.vidioc_try_fmt_vid_out = vidioc_try_fmt,
	.vidioc_s_fmt_vid_out = vidioc_s_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_g_crop = vidioc_g_crop,
	.vidioc_s_crop = vidioc_s_crop,
	.vidioc_cropcap = vidioc_cropcap,
};

static struct video_device rga_videodev = {
	.name = "rockchip-rga",
	.fops = &rga_fops,
	.ioctl_ops = &rga_ioctl_ops,
	.minor = -1,
	.release = video_device_release,
	.vfl_dir = VFL_DIR_M2M,
};

static int rga_enable_clocks(struct rockchip_rga *rga)
{
	int ret;

	ret = clk_prepare_enable(rga->sclk);
	if (ret) {
		dev_err(rga->dev, "Cannot enable rga sclk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(rga->aclk);
	if (ret) {
		dev_err(rga->dev, "Cannot enable rga aclk: %d\n", ret);
		goto err_disable_sclk;
	}

	ret = clk_prepare_enable(rga->hclk);
	if (ret) {
		dev_err(rga->dev, "Cannot enable rga hclk: %d\n", ret);
		goto err_disable_aclk;
	}

	return 0;

err_disable_sclk:
	clk_disable_unprepare(rga->sclk);
err_disable_aclk:
	clk_disable_unprepare(rga->aclk);

	return ret;
}

static int rga_parse_dt(struct rockchip_rga *rga)
{
	struct reset_control *core_rst, *axi_rst, *ahb_rst;

	core_rst = devm_reset_control_get(rga->dev, "core");
	if (IS_ERR(core_rst)) {
		dev_err(rga->dev, "failed to get core reset controller\n");
		return PTR_ERR(core_rst);
	}

	axi_rst = devm_reset_control_get(rga->dev, "axi");
	if (IS_ERR(axi_rst)) {
		dev_err(rga->dev, "failed to get axi reset controller\n");
		return PTR_ERR(axi_rst);
	}

	ahb_rst = devm_reset_control_get(rga->dev, "ahb");
	if (IS_ERR(ahb_rst)) {
		dev_err(rga->dev, "failed to get ahb reset controller\n");
		return PTR_ERR(ahb_rst);
	}

	reset_control_assert(core_rst);
	udelay(1);
	reset_control_deassert(core_rst);

	reset_control_assert(axi_rst);
	udelay(1);
	reset_control_deassert(axi_rst);

	reset_control_assert(ahb_rst);
	udelay(1);
	reset_control_deassert(ahb_rst);

	rga->sclk = devm_clk_get(rga->dev, "sclk");
	if (IS_ERR(rga->sclk)) {
		dev_err(rga->dev, "failed to get sclk clock\n");
		return PTR_ERR(rga->sclk);
	}

	rga->aclk = devm_clk_get(rga->dev, "aclk");
	if (IS_ERR(rga->aclk)) {
		dev_err(rga->dev, "failed to get aclk clock\n");
		return PTR_ERR(rga->aclk);
	}

	rga->hclk = devm_clk_get(rga->dev, "hclk");
	if (IS_ERR(rga->hclk)) {
		dev_err(rga->dev, "failed to get hclk clock\n");
		return PTR_ERR(rga->hclk);
	}

	return rga_enable_clocks(rga);
}

static int rga_probe(struct platform_device *pdev)
{
	struct rockchip_rga *rga;
	struct video_device *vfd;
	struct resource *res;
	int ret = 0;
	int irq;

	if (!pdev->dev.of_node)
		return -ENODEV;

	rga = devm_kzalloc(&pdev->dev, sizeof(*rga), GFP_KERNEL);
	if (!rga)
		return -ENOMEM;

	rga->dev = &pdev->dev;
	spin_lock_init(&rga->ctrl_lock);
	mutex_init(&rga->mutex);

	init_waitqueue_head(&rga->irq_queue);

	ret = rga_parse_dt(rga);
	if (ret) {
		dev_err(&pdev->dev, "Unable to parse OF data\n");
	}

	pm_runtime_enable(rga->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	rga->regs = devm_ioremap_resource(rga->dev, res);
	if (IS_ERR(rga->regs)) {
		ret = PTR_ERR(rga->regs);
		goto err_put_clk;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(rga->dev, "failed to get irq\n");
		ret = irq;
		goto err_put_clk;
	}

	ret = devm_request_irq(rga->dev, irq, rga_isr, 0,
	                       dev_name(rga->dev), rga);
	if (ret < 0) {
		dev_err(rga->dev, "failed to request irq\n");
		goto err_put_clk;
	}

	pm_runtime_get_sync(rga->dev);

	rga->version.major = (rga_read(rga, RGA_VERSION_INFO) >> 24) & 0xFF;
	rga->version.minor = (rga_read(rga, RGA_VERSION_INFO) >> 20) & 0x0F;

	pm_runtime_put(rga->dev);

	rga->alloc_ctx = vb2_dma_sg_init_ctx(&pdev->dev);

	ret = v4l2_device_register(&pdev->dev, &rga->v4l2_dev);
	if (ret)
		goto err_put_clk;
	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&rga->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_v4l2_dev;
	}
	*vfd = rga_videodev;
	vfd->lock = &rga->mutex;
	vfd->v4l2_dev = &rga->v4l2_dev;
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&rga->v4l2_dev, "Failed to register video device\n");
		goto rel_vdev;
	}

	video_set_drvdata(vfd, rga);
	snprintf(vfd->name, sizeof(vfd->name), "%s", rga_videodev.name);
	rga->vfd = vfd;
	v4l2_info(&rga->v4l2_dev, "device registered as /dev/video%d\n",
	          vfd->num);
	platform_set_drvdata(pdev, rga);
	rga->m2m_dev = v4l2_m2m_init(&rga_m2m_ops);
	if (IS_ERR(rga->m2m_dev)) {
		v4l2_err(&rga->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(rga->m2m_dev);
		goto unreg_video_dev;
	}

	def_frame.stride = (def_frame.width * def_frame.fmt->depth) >> 3;

	return 0;

unreg_video_dev:
	video_unregister_device(rga->vfd);
rel_vdev:
	video_device_release(vfd);
unreg_v4l2_dev:
	v4l2_device_unregister(&rga->v4l2_dev);
err_put_clk:
	pm_runtime_disable(rga->dev);

	return ret;
}

static int rga_remove(struct platform_device *pdev)
{
	struct rockchip_rga *rga = platform_get_drvdata(pdev);

	v4l2_info(&rga->v4l2_dev, "Removing \n");
	v4l2_m2m_release(rga->m2m_dev);
	video_unregister_device(rga->vfd);
	v4l2_device_unregister(&rga->v4l2_dev);
	vb2_dma_sg_cleanup_ctx(rga->alloc_ctx);

	pm_runtime_disable(rga->dev);

	return 0;
}

static const struct of_device_id rockchip_rga_match[] = {
	{
		.compatible = "rockchip,rk3288-rga",
	},
	{
		.compatible = "rockchip,rk3228-rga",
	},
	{
		.compatible = "rockchip,rk3328-rga",
	},
	{
		.compatible = "rockchip,rk3399-rga",
	},
	{},
};

MODULE_DEVICE_TABLE(of, rockchip_rga_match);

static struct platform_driver rga_pdrv = {
	.probe = rga_probe,
	.remove = rga_remove,
	.driver = {
		.name = "rockchip-rga",
		.of_match_table = rockchip_rga_match,
	},
};

module_platform_driver(rga_pdrv);

MODULE_AUTHOR("Jacob Chen <jacob-chen@iotwrt.com>");
MODULE_DESCRIPTION("Rockchip Raster 2d Grapphic Acceleration Unit");
MODULE_LICENSE("GPL");
