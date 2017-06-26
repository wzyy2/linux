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

#include <linux/pm_runtime.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-v4l2.h>

#include "rga-hw.h"
#include "rga.h"

static int
rga_queue_setup(struct vb2_queue *vq, const void *parg,
                unsigned int *nbuffers, unsigned int *nplanes,
                unsigned int sizes[], void *alloc_devs[])
{
	struct rga_ctx *ctx = vb2_get_drv_priv(vq);
	struct rockchip_rga *rga = ctx->rga;
	struct rga_frame *f = rga_get_frame(ctx, vq->type);

	if (IS_ERR(f))
		return PTR_ERR(f);

	sizes[0] = f->size;
	*nplanes = 1;
	alloc_devs[0] = rga->alloc_ctx;

	if (*nbuffers == 0)
		*nbuffers = 1;

	return 0;
}

static int rga_buf_prepare(struct vb2_buffer *vb)
{
	struct rga_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct rga_frame *f = rga_get_frame(ctx, vb->vb2_queue->type);

	if (IS_ERR(f))
		return PTR_ERR(f);

	vb2_set_plane_payload(vb, 0, f->size);

	return 0;
}

static void rga_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rga_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int rga_buf_init(struct vb2_buffer *vb)
{
	struct rga_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct rockchip_rga *rga = ctx->rga;
	struct sg_table *sgt;
	struct scatterlist *sgl;
	unsigned int *pages;
	struct rga_buf *buf;
	unsigned int address, len, i, p;
	unsigned int mapped_size = 0;

	/* Create local MMU table for RGA */
	sgt = vb2_plane_cookie(vb, 0);

	/*
	 * Alloc (2^3 * 4K) = 32K byte for storing pages, those space could
	 * cover 32K * 4K = 128M ram address.
	 */
	pages = (unsigned int *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 3);

	for_each_sg(sgt->sgl, sgl, sgt->nents, i) {
		len = sg_dma_len(sgl) >> PAGE_SHIFT;
		address = sg_phys(sgl);

		for (p = 0; p < len; p++) {
			dma_addr_t phys = address + (p << PAGE_SHIFT);
			pages[mapped_size + p] = phys;
		}

		mapped_size += len;
	}

	/* sync local MMU table for RGA */
	dma_sync_single_for_device(rga->dev, virt_to_phys(pages),
	                           8 * PAGE_SIZE, DMA_BIDIRECTIONAL);

	/* Store the buffer to the RGA private buffers list */
	buf = kmalloc(sizeof(struct rga_buf), GFP_KERNEL);
	buf->index = vb->index;
	buf->type = vb->type;
	buf->mmu_pages = pages;

	list_add_tail(&buf->entry, &ctx->buffers_list);

	return 0;
}

static void rga_buf_cleanup(struct vb2_buffer *vb)
{
	struct rga_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct rga_buf *buf, *tmp;

	/* Release the RGA private buffers */
	list_for_each_entry_safe(buf, tmp, &ctx->buffers_list, entry) {
		if (buf->index == vb->index && buf->type == vb->type) {
			free_pages((unsigned long)buf->mmu_pages, 3);
			list_del(&buf->entry);
			kfree(buf);
		}
	}
}

static void rga_buf_stop_streaming(struct vb2_queue *q)
{
	struct rga_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (vbuf == NULL)
			return;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

const struct vb2_ops rga_qops = {
	.queue_setup = rga_queue_setup,
	.buf_prepare = rga_buf_prepare,
	.buf_queue = rga_buf_queue,
	.buf_init = rga_buf_init,
	.buf_cleanup = rga_buf_cleanup,
	.stop_streaming = rga_buf_stop_streaming,
};

void *rga_buf_find_page(struct vb2_buffer *vb)
{
	struct rga_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct rga_buf *buf;

	list_for_each_entry(buf, &ctx->buffers_list, entry) {
		if (buf->index == vb->index && buf->type == vb->type) {
			return buf->mmu_pages;
		}
	}

	return NULL;
}

void rga_buf_clean(struct rga_ctx *ctx)
{
	struct rga_buf *buf, *tmp;

	list_for_each_entry_safe(buf, tmp, &ctx->buffers_list, entry) {
		free_pages((unsigned long)buf->mmu_pages, 3);
		list_del(&buf->entry);
		kfree(buf);
	}
}
