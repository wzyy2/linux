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

#include <linux/delay.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include "path_video.h"
#include "regs.h"
#include "rkisp1.h"


#define writel_verify(d, a, mask, lock) \
	{ \
		unsigned int i = 0; \
		unsigned long flags = 0; \
		spin_lock_irqsave(lock, flags); \
		do { \
			writel(d, a); \
			udelay(1); \
			if (i++ == 50) { \
				pr_err("Error in writing %x@0x%p, read %x\n", \
					(d) & (mask), a, readl(a)); \
					WARN_ON(1); \
			} \
		} while ((readl(a) & mask) != ((d) & mask)); \
		spin_unlock_irqrestore(lock, flags);\
	}
#define RKISP1_INVALID_BUFF_ADDR ((u32)~0)
#define CIF_ISP_REQ_BUFS_MIN 2
#define CIF_ISP_REQ_BUFS_MAX 8

#define CIF_FMT_RAW(_fo, _mb, _de, _bp) \
{ \
	.fourcc = _fo, \
	.mbus_code = _mb, \
	.depth = _de, \
	.colorspace = V4L2_COLORSPACE_SRGB, \
	.fmt_type = CIF_FMT_PIX_TYPE_BAYER, \
	.bpp = _bp, \
	.mplanes = 1, \
}

#define CIF_FMT_RGB(_fo, _bp) \
{ \
	.fourcc = _fo, \
	.colorspace = V4L2_COLORSPACE_SRGB, \
	.fmt_type = CIF_FMT_PIX_TYPE_RGB, \
	.bpp = _bp, \
	.mplanes = 1, \
}

#define CIF_FMT_YUV(_fo, _mb, _bp, _cp, _mp, _uv) \
{ \
	.fourcc = _fo, \
	.mbus_code = _mb, \
	.depth = 8, \
	.colorspace = V4L2_COLORSPACE_JPEG, \
	.fmt_type = CIF_FMT_PIX_TYPE_YUV, \
	.bpp = _bp, \
	.cplanes = _cp, \
	.mplanes = _mp, \
	.uv_swap = _uv, \
}

static const struct cif_fmt_info rkisp1_output_formats[] = {
	/* yuv422 */
	CIF_FMT_YUV(V4L2_PIX_FMT_YUYV, MEDIA_BUS_FMT_YUYV8_2X8, 16, 0, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_YVYU, MEDIA_BUS_FMT_YVYU8_2X8, 16, 0, 1, 1),
	CIF_FMT_YUV(V4L2_PIX_FMT_UYVY, MEDIA_BUS_FMT_UYVY8_2X8, 16, 0, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_VYUY, MEDIA_BUS_FMT_VYUY8_2X8, 16, 0, 1, 1),
	CIF_FMT_YUV(V4L2_PIX_FMT_YUV422P, MEDIA_BUS_FMT_YUYV8_2X8, 16, 2, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV16, MEDIA_BUS_FMT_YUYV8_2X8, 16, 1, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV61, MEDIA_BUS_FMT_YUYV8_2X8, 16, 1, 1, 0),

	/* yuv420 */
	CIF_FMT_YUV(V4L2_PIX_FMT_NV21, MEDIA_BUS_FMT_YUYV8_1_5X8, 12, 1, 1, 1),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV12, MEDIA_BUS_FMT_YVYU8_1_5X8, 12, 1, 1, 0),

	CIF_FMT_YUV(V4L2_PIX_FMT_YVU420, MEDIA_BUS_FMT_YVYU8_1_5X8, 12, 2, 1, 1),
	/* yuv444 */

	/* yuv400 */
	CIF_FMT_YUV(V4L2_PIX_FMT_GREY, MEDIA_BUS_FMT_Y8_1X8, 8, 1, 1, 0),
	/* raw */
	CIF_FMT_RAW(V4L2_PIX_FMT_SRGGB8, MEDIA_BUS_FMT_SRGGB8_1X8, 8, 8),
	CIF_FMT_RAW(V4L2_PIX_FMT_SGRBG8, MEDIA_BUS_FMT_SGRBG8_1X8, 8, 8),
	CIF_FMT_RAW(V4L2_PIX_FMT_SGBRG8, MEDIA_BUS_FMT_SGBRG8_1X8, 8, 8),
	CIF_FMT_RAW(V4L2_PIX_FMT_SBGGR8, MEDIA_BUS_FMT_SBGGR8_1X8, 8, 8),
	CIF_FMT_RAW(V4L2_PIX_FMT_SRGGB8, MEDIA_BUS_FMT_SRGGB10_1X10, 10, 16),
	CIF_FMT_RAW(V4L2_PIX_FMT_SGRBG8, MEDIA_BUS_FMT_SGRBG10_1X10, 10, 16),
	CIF_FMT_RAW(V4L2_PIX_FMT_SGBRG8, MEDIA_BUS_FMT_SGBRG10_1X10, 10, 16),
	CIF_FMT_RAW(V4L2_PIX_FMT_SBGGR8, MEDIA_BUS_FMT_SBGGR10_1X10, 10, 16),
	CIF_FMT_RAW(V4L2_PIX_FMT_SRGGB8, MEDIA_BUS_FMT_SRGGB12_1X12, 12, 16),
	CIF_FMT_RAW(V4L2_PIX_FMT_SGRBG8, MEDIA_BUS_FMT_SGRBG12_1X12, 12, 16),
	CIF_FMT_RAW(V4L2_PIX_FMT_SGBRG8, MEDIA_BUS_FMT_SGBRG12_1X12, 12, 16),
	CIF_FMT_RAW(V4L2_PIX_FMT_SBGGR8, MEDIA_BUS_FMT_SBGGR12_1X12, 12, 16),
	/* rgb */
	CIF_FMT_RGB(V4L2_PIX_FMT_RGB24, 24),
	CIF_FMT_RGB(V4L2_PIX_FMT_RGB565, 16),
};

static const struct cif_fmt_info *find_fmt(enum
						     rkisp1_stream_id
						     id,
						     const u32 *
						     pixelfmt, int index)
{
	unsigned int i, array_size;
	const struct cif_fmt_info *fmt, *def_fmt = &rkisp1_output_formats[0];

	if ((id != RKISP1_STREAM_SP) && (id != RKISP1_STREAM_MP))
		return NULL;

	array_size = ARRAY_SIZE(rkisp1_output_formats);
	if (index >= (int)array_size)
		return NULL;

	for (i = 0; i < array_size; i++) {
		fmt = &rkisp1_output_formats[i];
		if (pixelfmt && fmt->fourcc == *pixelfmt) {
			/* do some checks on sp and mp */
			if (id == RKISP1_STREAM_SP) {
				/* only support yuv and rgb formats */
				if (!CIF_PIX_FMT_IS_YUV(fmt) &&
				    !CIF_PIX_FMT_IS_RGB(fmt))
					continue;
			} else {
				/* only support yuv and raw formats */
				if (!CIF_PIX_FMT_IS_YUV(fmt) &&
				    !CIF_PIX_FMT_IS_RAW_BAYER(fmt))
					continue;
			}

			return fmt;
		}

		if (index == i)
			def_fmt = fmt;
	}

	return def_fmt;
}

static struct rkisp1_buffer *_to_vb(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct rkisp1_buffer, vb);
}

static struct vb2_queue *to_vb2_queue(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_vdev_node *node = vdev_to_node(vdev);

	WARN_ON(!vdev);

	return &node->buf_queue;
}

static inline
struct rkisp1_stream *node_to_stream(struct rkisp1_vdev_node *node)
{
	return container_of(node, struct rkisp1_stream, vnode);
}

static enum rkisp1_stream_id file_to_stream_id(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_vdev_node *node = vdev_to_node(vdev);
	struct rkisp1_stream *stream =
	    container_of(node, struct rkisp1_stream, vnode);

	return stream->id;
}

static enum rkisp1_stream_id queue_to_stream_id(struct vb2_queue *queue)
{
	struct rkisp1_vdev_node *node = queue_to_node(queue);

	return node_to_stream(node)->id;
}

static int rkisp1_s_fmt_mp(struct rkisp1_device *dev,
			      struct cif_frm_fmt *strm_fmt)
{
	const struct cif_fmt_info *cif_fmt = find_fmt(RKISP1_STREAM_MP,
								&strm_fmt->frm_fmt.fourcc, -1);
	strm_fmt->frm_fmt = *cif_fmt;

	if (CIF_PIX_FMT_IS_RAW_BAYER(cif_fmt)) {
		if ((dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_READY) ||
		    (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING))
			v4l2_warn(&dev->v4l2_dev,
				  "cannot output RAW data "
				  "when SP is active, you "
				  "will not be able to "
				  "(re-)start streaming\n");
		dev->strm_vdevs.mi_config.raw_enable = true;
	}

	dev->strm_vdevs.mp_vdev.path_cfg.output = *strm_fmt;
	dev->strm_vdevs.mp_vdev.path_cfg.llength = strm_fmt->width;
	dev->strm_vdevs.mp_vdev.updt_cfg = true;
	dev->strm_vdevs.mp_vdev.state = RKISP1_STATE_READY;

	return 0;
}

static int rkisp1_s_fmt_sp(struct rkisp1_device *dev,
			      struct cif_frm_fmt *strm_fmt)
{
	const struct cif_fmt_info *cif_fmt = find_fmt(RKISP1_STREAM_SP,
							&strm_fmt->frm_fmt.fourcc, -1);
	strm_fmt->frm_fmt = *cif_fmt;

	if (dev->strm_vdevs.mi_config.raw_enable)
		v4l2_warn(&dev->v4l2_dev,
			  "cannot activate SP when MP is set to RAW data output, "
			  "you will not be able to (re-)start streaming\n");

	/* TBD: more detailed check whether format is a valid format for SP */
	/* TBD: remove the mode stuff */
	if (!CIF_PIX_FMT_IS_YUV(cif_fmt) && !CIF_PIX_FMT_IS_RGB(cif_fmt)) {
		v4l2_warn(&dev->v4l2_dev,
			  "format %c%c%c%c %dx%d, "
			  "not supported on SP\n",
			  (strm_fmt->frm_fmt.fourcc & 0xff),
			  (strm_fmt->frm_fmt.fourcc >> 8) & 0xff,
			  (strm_fmt->frm_fmt.fourcc >> 16) & 0xff,
			  (strm_fmt->frm_fmt.fourcc >> 24) & 0xff,
			  strm_fmt->width, strm_fmt->height);
		return -EINVAL;
	}

	dev->strm_vdevs.sp_vdev.path_cfg.output = *strm_fmt;
	dev->strm_vdevs.sp_vdev.path_cfg.llength = strm_fmt->width;
	dev->strm_vdevs.sp_vdev.updt_cfg = true;
	dev->strm_vdevs.sp_vdev.state = RKISP1_STATE_READY;

	return 0;
}

static int rkisp1_s_fmt(struct rkisp1_device *dev,
			   enum rkisp1_stream_id stream_id,
			   struct cif_frm_fmt *strm_fmt)
{
	switch (stream_id) {
	case RKISP1_STREAM_SP:
		return rkisp1_s_fmt_sp(dev, strm_fmt);
	case RKISP1_STREAM_MP:
		return rkisp1_s_fmt_mp(dev, strm_fmt);
	default:
		v4l2_err(&dev->v4l2_dev, "unknown/unsupported stream ID %d\n",
			 stream_id);
		return -EINVAL;
	}

	return 0;
}

static int rkisp1_config_stream_path(struct rkisp1_device *dev, u32 stream_ids)
{
	u32 dpcl = readl(dev->config.base_addr + CIF_VI_DPCL);

	/* chan_mode */
	if (stream_ids & RKISP1_STREAM_SP)
		dpcl |= CIF_VI_DPCL_CHAN_MODE_SP;

	if (stream_ids & RKISP1_STREAM_MP) {
		dpcl |= CIF_VI_DPCL_CHAN_MODE_MP;
		dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_MI;
	}
	writel(dpcl, dev->config.base_addr + CIF_VI_DPCL);

	return 0;
}

static int rkisp1_config_mi_mp(struct rkisp1_device *dev)
{
	struct cif_frm_fmt *out_frm_fmt =
	    &dev->strm_vdevs.mp_vdev.path_cfg.output;
	struct cif_fmt_info *cif_fmt = &out_frm_fmt->frm_fmt;
	u32 llength = dev->strm_vdevs.mp_vdev.path_cfg.llength;
	u32 width = out_frm_fmt->width;
	u32 height = out_frm_fmt->height;
	u32 writeformat = RKISP1_BUFF_FMT_PLANAR;
	spinlock_t *lock = &dev->writel_verify_lock;
	u32 swap_cb_cr = 0;
	u32 bpp = cif_fmt->bpp;
	u32 size = llength * height * bpp / 8;
	u32 mi_ctrl, val;
	void *addr;

	v4l2_info(&dev->v4l2_dev,
		  "%c%c%c%c %dx%d, llength = %d\n",
		  (cif_fmt->fourcc & 0xff),
		  (cif_fmt->fourcc >> 8) & 0xff,
		  (cif_fmt->fourcc >> 16) & 0xff,
		  (cif_fmt->fourcc >> 24) & 0xff, width, height, llength);

	dev->strm_vdevs.mp_vdev.path_cfg.y_size = size;
	dev->strm_vdevs.mp_vdev.path_cfg.cb_size = 0;
	dev->strm_vdevs.mp_vdev.path_cfg.cr_size = 0;
	if (CIF_PIX_FMT_IS_YUV(cif_fmt)) {
		u32 num_cplanes = cif_fmt->cplanes;

		if (num_cplanes == 0) {
			writeformat = RKISP1_BUFF_FMT_INTERLEAVED;
		} else {
			dev->strm_vdevs.mp_vdev.path_cfg.y_size =
			    (dev->strm_vdevs.mp_vdev.path_cfg.y_size * 4) /
			    (4 + (cif_fmt->xsubs * cif_fmt->ysubs / 2));

			dev->strm_vdevs.mp_vdev.path_cfg.cb_size =
			    size - dev->strm_vdevs.mp_vdev.path_cfg.y_size;

			if (num_cplanes == 1) {
				writeformat = RKISP1_BUFF_FMT_SEMIPLANAR;
				if (cif_fmt->uv_swap)
					swap_cb_cr =
					    CIF_MI_XTD_FMT_CTRL_MP_CB_CR_SWAP;
			} else if (num_cplanes == 2) {
				writeformat = RKISP1_BUFF_FMT_PLANAR;
				dev->strm_vdevs.mp_vdev.path_cfg.cb_size /= 2;
			}
			/* for U<->V swapping: */
			dev->strm_vdevs.mp_vdev.path_cfg.cr_size =
			    dev->strm_vdevs.mp_vdev.path_cfg.cb_size;
		}

	} else if (CIF_PIX_FMT_IS_RAW_BAYER(cif_fmt)) {
		if (cif_fmt->bpp > 8) {
			writeformat = RKISP1_BUFF_FMT_RAW12;
			dev->strm_vdevs.mp_vdev.path_cfg.y_size = width *
			    height * 2;
		} else {
			writeformat = RKISP1_BUFF_FMT_RAW8;
			dev->strm_vdevs.mp_vdev.path_cfg.y_size = width *
			    height;
		}
		dev->strm_vdevs.mp_vdev.path_cfg.cb_size = 0;
		dev->strm_vdevs.mp_vdev.path_cfg.cr_size = 0;
	}

	writel_verify(dev->strm_vdevs.mp_vdev.path_cfg.y_size,
			     dev->config.base_addr + CIF_MI_MP_Y_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	writel_verify(dev->strm_vdevs.mp_vdev.path_cfg.cb_size,
			     dev->config.base_addr + CIF_MI_MP_CB_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	writel_verify(dev->strm_vdevs.mp_vdev.path_cfg.cr_size,
			     dev->config.base_addr + CIF_MI_MP_CR_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	val = CIF_MI_MP_FRAME;
	addr = dev->config.base_addr + CIF_MI_IMSC;
	writel_verify(val | readl(addr), addr, ~0, lock);

	if (swap_cb_cr) {
		val = swap_cb_cr;
		addr = dev->config.base_addr + CIF_MI_XTD_FORMAT_CTRL;
		writel(readl(addr) | val, addr);
	}

	mi_ctrl = readl(dev->config.base_addr + CIF_MI_CTRL) |
	    CIF_MI_CTRL_MP_WRITE_FMT(writeformat) |
	    CIF_MI_CTRL_BURST_LEN_LUM_64 |
	    CIF_MI_CTRL_BURST_LEN_CHROM_64 |
	    CIF_MI_CTRL_INIT_BASE_EN |
	    CIF_MI_CTRL_INIT_OFFSET_EN | CIF_MI_MP_AUTOUPDATE_ENABLE;

	writel_verify(mi_ctrl, dev->config.base_addr + CIF_MI_CTRL, ~0, lock);

	v4l2_info(&dev->v4l2_dev,
		  "\n  MI_CTRL 0x%08x\n"
		  "  MI_STATUS 0x%08x\n"
		  "  MI_MP_Y_SIZE %d\n"
		  "  MI_MP_CB_SIZE %d\n"
		  "  MI_MP_CR_SIZE %d\n",
		  readl(dev->config.base_addr + CIF_MI_CTRL),
		  readl(dev->config.base_addr + CIF_MI_STATUS),
		  readl(dev->config.base_addr + CIF_MI_MP_Y_SIZE_INIT),
		  readl(dev->config.base_addr + CIF_MI_MP_CB_SIZE_INIT),
		  readl(dev->config.base_addr + CIF_MI_MP_CR_SIZE_INIT));

	return 0;
}

static struct v4l2_subdev *rkisp1_remote_subdev(struct rkisp1_vdev_node
						   *video, u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_pad(&video->pad);

	if (remote == NULL ||
	    media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int cif_mbus_to_cifmt(const struct v4l2_mbus_framefmt *mbus,
			     struct cif_frm_fmt *cifmt)
{
	int i;

	cifmt->width = mbus->width;
	cifmt->height = mbus->height;

	for (i = 0; i < ARRAY_SIZE(rkisp1_output_formats); ++i) {
		/* TODO: should do more checks */
		if (rkisp1_output_formats[i].mbus_code == mbus->code)
			break;
	}

	if (WARN_ON(i == ARRAY_SIZE(rkisp1_output_formats)))
		return 0;

	cifmt->frm_fmt = rkisp1_output_formats[i];

	return 0;
}

static int
__get_format(struct rkisp1_vdev_node *video,
		       struct cif_frm_fmt *cifmt)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = rkisp1_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);

	if (ret < 0)
		return ret;
	return cif_mbus_to_cifmt(&fmt.format, cifmt);
}

static int
_check_format(struct rkisp1_vdev_node *video)
{
	struct rkisp1_stream *strm = node_to_stream(video);
	struct cif_frm_fmt *video_fmt = &strm->path_cfg.output;
	struct cif_frm_fmt remote_fmt;
	int ret;

	ret = __get_format(video, &remote_fmt);
	if (ret < 0)
		return ret;

	if (video_fmt->frm_fmt.fourcc != remote_fmt.frm_fmt.fourcc ||
	    video_fmt->height != remote_fmt.height ||
	    video_fmt->width != remote_fmt.width)
		return -EINVAL;

	return 0;
}

static int rkisp1_config_mi_sp(struct rkisp1_device *dev)
{
	struct cif_frm_fmt *out_frm_fmt =
	    &dev->strm_vdevs.mp_vdev.path_cfg.output;
	struct cif_fmt_info *out_cif_fmt = &out_frm_fmt->frm_fmt;
	struct cif_frm_fmt in_frm_fmt;
	struct cif_fmt_info *in_cif_fmt;
	u32 llength = dev->strm_vdevs.sp_vdev.path_cfg.llength;
	u32 width = dev->strm_vdevs.sp_vdev.path_cfg.output.width;
	u32 height = dev->strm_vdevs.sp_vdev.path_cfg.output.height;
	u32 writeformat = RKISP1_BUFF_FMT_PLANAR;
	u32 swap_cb_cr = 0;
	u32 bpp = out_cif_fmt->bpp;
	spinlock_t *lock = &dev->writel_verify_lock;
	u32 size = llength * height * bpp / 8;
	u32 input_format = 0;
	u32 output_format;
	u32 mi_ctrl, val;
	void *addr;

	v4l2_info(&dev->v4l2_dev,
		  "%c%c%c%c %dx%d, llength = %d\n",
		  (out_cif_fmt->fourcc & 0xff),
		  (out_cif_fmt->fourcc >> 8) & 0xff,
		  (out_cif_fmt->fourcc >> 16) & 0xff,
		  (out_cif_fmt->fourcc >> 24) & 0xff, width, height, llength);

	if (__get_format(&dev->strm_vdevs.sp_vdev.vnode, &in_frm_fmt))
		return -EINVAL;
	in_cif_fmt = &in_frm_fmt.frm_fmt;
	if (!CIF_PIX_FMT_IS_YUV(in_cif_fmt)) {
		v4l2_err(&dev->v4l2_dev,
			 "unsupported format %c%c%c%c "
			 "(must be YUV)\n",
			 (out_cif_fmt->fourcc & 0xff),
			 (out_cif_fmt->fourcc >> 8) & 0xff,
			 (out_cif_fmt->fourcc >> 16) & 0xff,
			 (out_cif_fmt->fourcc >> 24) & 0xff);
		return -EINVAL;
	}

	dev->strm_vdevs.sp_vdev.path_cfg.y_size = size;
	dev->strm_vdevs.sp_vdev.path_cfg.cb_size = 0;
	dev->strm_vdevs.sp_vdev.path_cfg.cr_size = 0;
	if (CIF_PIX_FMT_IS_YUV(out_cif_fmt)) {
		u32 num_cplanes = out_cif_fmt->cplanes;

		if (num_cplanes == 0) {
			writeformat = RKISP1_BUFF_FMT_INTERLEAVED;
		} else {
			dev->strm_vdevs.sp_vdev.path_cfg.y_size =
			    (dev->strm_vdevs.sp_vdev.path_cfg.y_size * 4) /
			    (4 + out_cif_fmt->xsubs * out_cif_fmt->ysubs / 2);
			dev->strm_vdevs.sp_vdev.path_cfg.cb_size =
			    size - dev->strm_vdevs.sp_vdev.path_cfg.y_size;
			if (num_cplanes == 1) {
				writeformat = RKISP1_BUFF_FMT_SEMIPLANAR;
			} else if (num_cplanes == 2) {
				writeformat = RKISP1_BUFF_FMT_PLANAR;
				dev->strm_vdevs.sp_vdev.path_cfg.cb_size /= 2;
				if (out_cif_fmt->uv_swap)
					swap_cb_cr =
					    CIF_MI_XTD_FMT_CTRL_SP_CB_CR_SWAP;
			}
			/* for U<->V swapping: */
			dev->strm_vdevs.sp_vdev.path_cfg.cr_size =
			    dev->strm_vdevs.sp_vdev.path_cfg.cb_size;
		}
		if (out_cif_fmt->uv_swap)
			swap_cb_cr = CIF_MI_XTD_FMT_CTRL_SP_CB_CR_SWAP;

		if ((out_cif_fmt->xsubs == 0) && (out_cif_fmt->ysubs == 0))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV400;
		else if ((out_cif_fmt->xsubs == 2) && (out_cif_fmt->ysubs == 2))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV420;
		else if ((out_cif_fmt->xsubs == 2) && (out_cif_fmt->ysubs == 4))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV422;
		else if ((out_cif_fmt->xsubs == 4) && (out_cif_fmt->ysubs == 4))
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_YUV444;
		else {
			v4l2_err(&dev->v4l2_dev,
				 "unsupported yuv output format "
				 "%c%c%c%c (must be YUV)\n",
				 (out_cif_fmt->fourcc & 0xff),
				 (out_cif_fmt->fourcc >> 8) & 0xff,
				 (out_cif_fmt->fourcc >> 16) & 0xff,
				 (out_cif_fmt->fourcc >> 24) & 0xff);
			return -EINVAL;
		}
	} else if (CIF_PIX_FMT_IS_RGB(out_cif_fmt)) {
		if (out_cif_fmt->fourcc == V4L2_PIX_FMT_RGB565) {
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_RGB565;
		} else if (out_cif_fmt->fourcc == V4L2_PIX_FMT_BGR666) {
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_RGB666;
		} else if (out_cif_fmt->fourcc == V4L2_PIX_FMT_RGB24) {
			output_format = CIF_MI_CTRL_SP_OUTPUT_FMT_RGB888;
		} else {
			v4l2_err(&dev->v4l2_dev,
				 "unsupported RGB output format "
				 "%c%c%c%c\n",
				 (out_cif_fmt->fourcc & 0xff),
				 (out_cif_fmt->fourcc >> 8) & 0xff,
				 (out_cif_fmt->fourcc >> 16) & 0xff,
				 (out_cif_fmt->fourcc >> 24) & 0xff);
			return -EINVAL;
		}
	} else {
		v4l2_err(&dev->v4l2_dev,
			 "unsupported yuv output format "
			 "%c%c%c%c\n",
			 (out_cif_fmt->fourcc & 0xff),
			 (out_cif_fmt->fourcc >> 8) & 0xff,
			 (out_cif_fmt->fourcc >> 16) & 0xff,
			 (out_cif_fmt->fourcc >> 24) & 0xff);
		return -EINVAL;
	}

	if ((in_cif_fmt->xsubs == 0) && (in_cif_fmt->ysubs == 0))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV400;
	else if ((in_cif_fmt->xsubs == 2) && (in_cif_fmt->ysubs == 2))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV420;
	else if ((in_cif_fmt->xsubs == 2) && (in_cif_fmt->ysubs == 4))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV422;
	else if ((in_cif_fmt->xsubs == 4) && (in_cif_fmt->ysubs == 4))
		input_format = CIF_MI_CTRL_SP_INPUT_FMT_YUV444;
	else {
		v4l2_err(&dev->v4l2_dev,
			 "unsupported yuv input format "
			 "%c%c%c%c\n",
			 (out_cif_fmt->fourcc & 0xff),
			 (out_cif_fmt->fourcc >> 8) & 0xff,
			 (out_cif_fmt->fourcc >> 16) & 0xff,
			 (out_cif_fmt->fourcc >> 24) & 0xff);
		return -EINVAL;
	}

	writel_verify(dev->strm_vdevs.sp_vdev.path_cfg.y_size,
			     dev->config.base_addr + CIF_MI_SP_Y_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);

	writel_verify(dev->strm_vdevs.sp_vdev.path_cfg.y_size,
			     dev->config.base_addr + CIF_MI_SP_Y_PIC_SIZE,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	writel_verify(dev->strm_vdevs.sp_vdev.path_cfg.cb_size,
			     dev->config.base_addr + CIF_MI_SP_CB_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	writel_verify(dev->strm_vdevs.sp_vdev.path_cfg.cr_size,
			     dev->config.base_addr + CIF_MI_SP_CR_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	writel_verify(width,
			     dev->config.base_addr + CIF_MI_SP_Y_PIC_WIDTH,
			     ~0x3, lock);
	writel_verify(height,
			     dev->config.base_addr + CIF_MI_SP_Y_PIC_HEIGHT,
			     ~0x3, lock);
	writel_verify(llength,
			     dev->config.base_addr + CIF_MI_SP_Y_LLENGTH,
				 ~0x3, lock);
	val = CIF_MI_SP_FRAME;
	addr = dev->config.base_addr + CIF_MI_IMSC;
	writel_verify(val | readl(addr), addr, ~0, lock);

	if (swap_cb_cr) {
		val = swap_cb_cr;
		addr = dev->config.base_addr + CIF_MI_XTD_FORMAT_CTRL;
		writel(readl(addr) | val, addr);
	}

	mi_ctrl = readl(dev->config.base_addr + CIF_MI_CTRL) |
	    CIF_MI_CTRL_SP_WRITE_FMT(writeformat) |
	    input_format |
	    output_format |
	    CIF_MI_CTRL_BURST_LEN_LUM_64 |
	    CIF_MI_CTRL_BURST_LEN_CHROM_64 |
	    CIF_MI_CTRL_INIT_BASE_EN |
	    CIF_MI_CTRL_INIT_OFFSET_EN | CIF_MI_SP_AUTOUPDATE_ENABLE;
	writel_verify(mi_ctrl, dev->config.base_addr + CIF_MI_CTRL, ~0, lock);

	v4l2_info(&dev->v4l2_dev,
		  "\n  MI_CTRL 0x%08x\n"
		  "  MI_STATUS 0x%08x\n"
		  "  MI_SP_Y_SIZE %d\n"
		  "  MI_SP_CB_SIZE %d\n"
		  "  MI_SP_CR_SIZE %d\n"
		  "  MI_SP_PIC_WIDTH %d\n"
		  "  MI_SP_PIC_HEIGHT %d\n"
		  "  MI_SP_PIC_LLENGTH %d\n"
		  "  MI_SP_PIC_SIZE %d\n",
		  readl(dev->config.base_addr + CIF_MI_CTRL),
		  readl(dev->config.base_addr + CIF_MI_STATUS),
		  readl(dev->config.base_addr + CIF_MI_SP_Y_SIZE_INIT),
		  readl(dev->config.base_addr + CIF_MI_SP_CB_SIZE_INIT),
		  readl(dev->config.base_addr + CIF_MI_SP_CR_SIZE_INIT),
		  readl(dev->config.base_addr + CIF_MI_SP_Y_PIC_WIDTH),
		  readl(dev->config.base_addr + CIF_MI_SP_Y_PIC_HEIGHT),
		  readl(dev->config.base_addr + CIF_MI_SP_Y_LLENGTH),
		  readl(dev->config.base_addr + CIF_MI_SP_Y_PIC_SIZE));

	return 0;
}

static void rkisp1_stop_mi(struct rkisp1_device *dev,
			      bool stop_mi_sp, bool stop_mi_mp)
{
	spinlock_t *lock = &dev->writel_verify_lock;
	u32 val;
	void *addr;

	if (stop_mi_sp &&
	    (dev->strm_vdevs.sp_vdev.state != RKISP1_STATE_STREAMING))
		stop_mi_sp = false;
	if (stop_mi_mp &&
	    (dev->strm_vdevs.mp_vdev.state != RKISP1_STATE_STREAMING))
		stop_mi_mp = false;

	if (!stop_mi_sp && !stop_mi_mp)
		return;

	if (stop_mi_sp && stop_mi_mp) {
		val = (u32)~(CIF_MI_SP_FRAME | CIF_MI_MP_FRAME);
		addr = dev->config.base_addr + CIF_MI_IMSC;
		writel_verify(val & readl(addr), addr, ~0, lock);
		val = CIF_MI_SP_FRAME | CIF_MI_MP_FRAME;
		addr = dev->config.base_addr + CIF_MI_ICR;
		writel(val, addr);
		val = (u32)~CIF_MI_CTRL_SP_ENABLE;
		addr = dev->config.base_addr + CIF_MI_CTRL;
		writel_verify(val & readl(addr), addr, ~0, lock);
		val = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN |
			CIF_MI_CTRL_SP_ENABLE | CIF_MI_CTRL_RAW_ENABLE);
		addr = dev->config.base_addr + CIF_MI_CTRL;
		writel_verify(val & readl(addr), addr, ~0, lock);
		writel(CIF_MI_INIT_SOFT_UPD,
			  dev->config.base_addr + CIF_MI_INIT);
	} else if (stop_mi_sp) {
		writel(CIF_MI_SP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		val = (u32)~CIF_MI_CTRL_SP_ENABLE;
		addr = dev->config.base_addr + CIF_MI_CTRL;
		writel_verify(val & readl(addr), addr, ~0, lock);
	} else if (stop_mi_mp) {
		writel(CIF_MI_MP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		val = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN | CIF_MI_CTRL_RAW_ENABLE);
		addr = dev->config.base_addr + CIF_MI_CTRL;
		writel_verify(val & readl(addr), addr,	~0, lock);
	}
}

static int rkisp1_stream_stop_sp(struct rkisp1_device *dev)
{
	int ret = 0;
	unsigned long flags = 0;

	if (!dev->strm_vdevs.mi_config.async_updt) {
		local_irq_save(flags);
		rkisp1_stop_mi(dev, true, false);
		local_irq_restore(flags);
	}

	if (dev->strm_vdevs.sp_vdev.state != RKISP1_STATE_STREAMING)
		return 0;

	dev->strm_vdevs.sp_vdev.stop = true;
	ret = wait_event_interruptible_timeout(
			dev->strm_vdevs.sp_vdev.done,
			dev->strm_vdevs.sp_vdev.state != RKISP1_STATE_STREAMING,
			1 * HZ);
	if (ret < 0) {
		v4l2_warn(&dev->v4l2_dev,
			  "waiting on event returned with error %d\n", ret);
		return ret;
	}
	dev->strm_vdevs.sp_vdev.stop = false;
	if (dev->strm_vdevs.sp_vdev.path_cfg.busy)
		v4l2_warn(&dev->v4l2_dev,
			  "SP path still active while stopping it\n");

	return 0;
}

static int rkisp1_stream_stop_mp(struct rkisp1_device *dev)
{
	int ret = 0;
	unsigned long flags = 0;

	if (!dev->strm_vdevs.mi_config.async_updt) {
		local_irq_save(flags);
		rkisp1_stop_mi(dev, false, true);
		local_irq_restore(flags);
	}

	if (dev->strm_vdevs.mp_vdev.state != RKISP1_STATE_STREAMING)
		return 0;

	dev->strm_vdevs.mp_vdev.stop = true;

	ret = wait_event_interruptible_timeout(
			dev->strm_vdevs.mp_vdev.done,
			dev->strm_vdevs.mp_vdev.state != RKISP1_STATE_STREAMING,
			1 * HZ);
	if (ret < 0) {
		v4l2_warn(&dev->v4l2_dev, "waiting on event error %d\n", ret);
		return 0;
	}
	dev->strm_vdevs.mp_vdev.stop = false;
	if (dev->strm_vdevs.mp_vdev.path_cfg.busy)
		v4l2_warn(&dev->v4l2_dev,
			  "MP path still active while stopping it\n");

	return 0;
}

int rkisp1_streamon(struct rkisp1_device *dev, u32 stream_ids)
{
	int ret = 0;
	bool streamon_sp = stream_ids & RKISP1_STREAM_SP;
	bool streamon_mp = stream_ids & RKISP1_STREAM_MP;
	struct rkisp1_vdev_node *node;

	if (!((streamon_sp &&
	       (dev->strm_vdevs.sp_vdev.state != RKISP1_STATE_STREAMING)) ||
	      (streamon_mp &&
	       (dev->strm_vdevs.mp_vdev.state != RKISP1_STATE_STREAMING))))
		return 0;

	if (streamon_sp &&
	    (dev->strm_vdevs.sp_vdev.state != RKISP1_STATE_READY)) {
		v4l2_err(&dev->v4l2_dev, "cannot start streaming on SP path, "
			 "path not yet enabled\n");
		return -EBUSY;
	}

	if (streamon_mp &&
	    (dev->strm_vdevs.mp_vdev.state != RKISP1_STATE_READY)) {
		v4l2_err(&dev->v4l2_dev,
			 "cannot start streaming on MP path, path not yet enabled\n");
		return -EBUSY;
	}

	if (streamon_sp && dev->strm_vdevs.mi_config.raw_enable &&
	    (streamon_mp ||
	     (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING))) {
		v4l2_err(&dev->v4l2_dev,
			 "cannot start streaming on SP path when MP is active "
			 "and set to RAW output\n");
		return -EBUSY;
	}

	if (streamon_mp &&
	    (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING))
		dev->strm_vdevs.mp_vdev.updt_cfg = true;
	if (streamon_sp &&
	    (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING))
		dev->strm_vdevs.sp_vdev.updt_cfg = true;

	if (streamon_sp && dev->strm_vdevs.sp_vdev.updt_cfg &&
	    (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING)) {
		rkisp1_stream_stop_sp(dev);
		/* pipeline stream off */
		node = &dev->strm_vdevs.sp_vdev.vnode;
		ret = (node->pipe.set_stream) (&node->pipe, false);
		if (ret < 0)
			return ret;

		streamon_mp = true;
		dev->strm_vdevs.mp_vdev.updt_cfg = true;
		dev->strm_vdevs.mp_vdev.restart = true;
	}
	if (streamon_mp && dev->strm_vdevs.mp_vdev.updt_cfg &&
	    (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING)) {
		rkisp1_stream_stop_mp(dev);
		/* pipeline stream off */
		node = &dev->strm_vdevs.mp_vdev.vnode;
		ret = (node->pipe.set_stream) (&node->pipe, false);
		if (ret < 0)
			return ret;

		streamon_sp = true;
		dev->strm_vdevs.sp_vdev.updt_cfg = true;
		dev->strm_vdevs.sp_vdev.restart = true;
	}

	return 0;
}

int rkisp1_streamoff(struct rkisp1_device *dev, u32 stream_ids)
{
	int ret = 0;
	bool streamoff_sp = stream_ids & RKISP1_STREAM_SP;
	bool streamoff_mp = stream_ids & RKISP1_STREAM_MP;
	struct rkisp1_vdev_node *node;

	v4l2_info(&dev->v4l2_dev,
		  "state: SP(%d) MP(%d), streamoff: SP(%d) MP(%d)\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state, streamoff_sp, streamoff_mp);

	/* TODO power off flash. Flash subdev is not ready yet */

	/* pipeline stream off  */
	if (streamoff_sp) {
		rkisp1_stream_stop_sp(dev);
		node = &dev->strm_vdevs.sp_vdev.vnode;
		ret = (node->pipe.set_stream) (&node->pipe, false);
		if (ret < 0)
			return ret;
	}

	if (streamoff_mp) {
		rkisp1_stream_stop_mp(dev);
		node = &dev->strm_vdevs.mp_vdev.vnode;
		ret = (node->pipe.set_stream) (&node->pipe, false);
		if (ret < 0)
			return ret;
	}

	if ((streamoff_sp) &&
	    (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_READY))
		dev->strm_vdevs.sp_vdev.state = RKISP1_STATE_INACTIVE;
	if (streamoff_mp) {
		dev->strm_vdevs.mi_config.raw_enable = false;
		dev->strm_vdevs.mp_vdev.path_cfg.output.width = 0;
		dev->strm_vdevs.mp_vdev.path_cfg.output.height = 0;
		if (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_READY)
			dev->strm_vdevs.mp_vdev.state = RKISP1_STATE_INACTIVE;
	}
	if ((dev->strm_vdevs.mp_vdev.state <= RKISP1_STATE_INACTIVE) &&
	    (dev->strm_vdevs.sp_vdev.state <= RKISP1_STATE_INACTIVE)) {
		dev->isp_sdev.isp_dev.input_width = 0;
		dev->isp_sdev.isp_dev.input_height = 0;
	}

	return 0;
}

static int rkisp1_v4l2_streamon(struct file *file,
				   void *priv, enum v4l2_buf_type buf_type)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_vdev_node *node = vdev_to_node(vdev);
	struct rkisp1_device *dev = video_get_drvdata(vdev);
	struct vb2_queue *queue = to_vb2_queue(file);
	u32 stream_ids = file_to_stream_id(file);

	ret = media_entity_pipeline_start(&node->vdev.entity, &node->pipe.pipe);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "start pipeline  failed %d\n", ret);
		return ret;
	}

	ret = _check_format(node);
	if (ret < 0) {
		/* TODO: just warning now */
		v4l2_warn(&dev->v4l2_dev, "check video format failed\n");
	}

	ret = node->pipe.open(&node->pipe, &node->vdev.entity, true);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "open cif pipeline failed %d\n", ret);
		return ret;
	}

	ret = rkisp1_streamon(dev, stream_ids);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "stream on failed %d\n", ret);
		return ret;
	}

	ret = vb2_streamon(queue, buf_type);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "videobuf_streamon failed\n");
		return ret;
	}

	return 0;
}

static int rkisp1_v4l2_streamoff(struct file *file,
				    void *priv, enum v4l2_buf_type buf_type)
{
	int ret = 0;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_vdev_node *node = vdev_to_node(vdev);

	ret = vb2_streamoff(queue, queue->type);
	if (ret < 0) {
		pr_err("videobuf_streamoff failed with error %d\n", ret);
		return ret;
	}

	ret = node->pipe.close(&node->pipe);
	if (ret < 0) {
		pr_err("pipeline close failed with error %d\n", ret);
		return ret;
	}

	media_entity_pipeline_stop(&node->vdev.entity);
	return 0;
}

static int fmt_calc_plane_size(const struct cif_fmt_info *fmt,
				   u32 width, u32 height, int plane_idx,
				   u32 *sizeimage, u32 *bpl)
{
	int mplanes = fmt->mplanes;
	int cplanes = fmt->cplanes;
	u32 image_size = sizeimage ? *sizeimage : 0;
	u32 bytesperline = bpl ? *bpl : 0;

	if (plane_idx >= mplanes)
		return -EINVAL;

	if (mplanes > 1) {
		if (plane_idx == 0) {
			bytesperline = width;
			image_size = width * height;
		} else if (mplanes == 2) {
			bytesperline = (fmt->xsubs / 4 * 2) * width;
			image_size = max(image_size,
					 height * (bytesperline * fmt->ysubs /
						   4));
		} else {
			bytesperline = (fmt->xsubs / 4) * width;
			image_size = max(image_size,
					 height * (bytesperline * fmt->ysubs /
						   4));
		}

	} else {
		if (cplanes > 0)
			bytesperline = width;
		if ((cplanes == 0) &&
		    (bytesperline == 0
		     || (bytesperline * 8) / fmt->bpp < width))
			bytesperline = (width * fmt->bpp) / 8;

		image_size = max(image_size, height * width * fmt->bpp / 8);

	}

	if (bpl)
		*bpl = bytesperline;
	if (sizeimage)
		*sizeimage = image_size;

	return 0;
}

static void fmt_to_v4l2_mplane_fmt(struct cif_frm_fmt *strm_fmt,
				       struct v4l2_pix_format_mplane *pix)
{
	struct v4l2_plane_pix_format *plane_fmt;
	int i;

	pix->colorspace = strm_fmt->frm_fmt.colorspace;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = strm_fmt->frm_fmt.mplanes;
	pix->pixelformat = strm_fmt->frm_fmt.fourcc;
	pix->height = strm_fmt->height;
	pix->width = strm_fmt->width;

	for (i = 0; i < pix->num_planes; i++) {
		plane_fmt = &pix->plane_fmt[i];
		fmt_calc_plane_size(&strm_fmt->frm_fmt, pix->width,
					pix->height, i, &plane_fmt->sizeimage,
					&plane_fmt->bytesperline);
	}

}

static int rkisp1_s_fmt_vid_cap_mplane(struct file *file,
				       void *priv, struct v4l2_format *f)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_device *dev = video_get_drvdata(vdev);
	struct cif_frm_fmt strm_fmt;

	strm_fmt.frm_fmt.fourcc = f->fmt.pix_mp.pixelformat;
	strm_fmt.width = f->fmt.pix_mp.width;
	strm_fmt.height = f->fmt.pix_mp.height;
	/* TODO: quantization */
	strm_fmt.quantization = 0;
	ret = rkisp1_s_fmt(dev, file_to_stream_id(file), &strm_fmt);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	fmt_to_v4l2_mplane_fmt(&strm_fmt, &f->fmt.pix_mp);

	return 0;
}

static int rkisp1_g_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_vdev_node *node = vdev_to_node(vdev);
	struct rkisp1_stream *strm = node_to_stream(node);
	struct cif_frm_fmt *strm_fmt = &strm->path_cfg.output;

	fmt_to_v4l2_mplane_fmt(strm_fmt, &f->fmt.pix_mp);

	return 0;
}

static int rkisp1_try_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct cif_frm_fmt strm_fmt;
	const struct cif_fmt_info *cif_fmt = find_fmt(file_to_stream_id(file),
									&f->fmt.pix_mp.pixelformat, -1);

	strm_fmt.frm_fmt = *cif_fmt;
	/* TODO: do more checks on resolution */
	strm_fmt.width = f->fmt.pix_mp.width;
	strm_fmt.height = f->fmt.pix_mp.height;
	/* TODO: quantization */
	strm_fmt.quantization = 0;
	fmt_to_v4l2_mplane_fmt(&strm_fmt, &f->fmt.pix_mp);

	return 0;
}


static int rkisp1_queue_setup(struct vb2_queue *queue,
					  const void *parg,
					  unsigned int *num_buffers,
					  unsigned int *num_planes,
					  unsigned int sizes[],
					  void *alloc_ctxs[])
{
	int i;
	const struct v4l2_format *pfmt = parg;
	const struct v4l2_pix_format_mplane *pixm = NULL;
	struct rkisp1_device *dev = queue->drv_priv;
	enum rkisp1_stream_id id = queue_to_stream_id(queue);
	const struct cif_fmt_info *cif_fmt = NULL;
	struct rkisp1_stream *stream = to_stream_by_id(dev, id);
	u32 imagsize = 0;

	if (pfmt) {
		pixm = &pfmt->fmt.pix_mp;
		cif_fmt = find_fmt(id, &pixm->pixelformat, -1);
	} else {
		cif_fmt = &stream->path_cfg.output.frm_fmt;
	}

	*num_buffers = clamp_t(u32, *num_buffers, CIF_ISP_REQ_BUFS_MIN,
			       CIF_ISP_REQ_BUFS_MAX);
	*num_planes = cif_fmt->mplanes;

	for (i = 0; i < cif_fmt->mplanes; i++) {
		if (pixm) {
			const struct v4l2_plane_pix_format *plane_fmt =
			    &pixm->plane_fmt[i];

			imagsize = plane_fmt->sizeimage;
			fmt_calc_plane_size(cif_fmt, pixm->width,
						pixm->height, i,
						&imagsize, NULL);
		} else {
			struct cif_frm_fmt *strm_fmt = &stream->path_cfg.output;

			fmt_calc_plane_size(cif_fmt, strm_fmt->width,
						strm_fmt->height, i,
						&imagsize, NULL);
		}

		sizes[i] = imagsize;
		alloc_ctxs[0] = dev->alloc_ctx;
	}

	v4l2_info(&dev->v4l2_dev, "%s count %d, size %d\n",
		  v4l2_type_names[queue->type], *num_buffers, sizes[0]);

	return 0;
}

static void rkisp1_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rkisp1_buffer *ispbuf = _to_vb(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct rkisp1_device *dev = queue->drv_priv;
	enum rkisp1_stream_id id = queue_to_stream_id(queue);
	struct rkisp1_stream *stream = to_stream_by_id(dev, id);
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&dev->strm_vdevs.vbq_lock, lock_flags);
	list_add_tail(&ispbuf->queue, &stream->buf_queue);
	spin_unlock_irqrestore(&dev->strm_vdevs.vbq_lock, lock_flags);
}

static void rkisp1_stop_streaming(struct vb2_queue *queue)
{
	struct rkisp1_vdev_node *node;
	enum rkisp1_stream_id id = queue_to_stream_id(queue);
	struct rkisp1_stream *stream = NULL;
	struct rkisp1_device *dev;
	struct rkisp1_buffer *buf;
	unsigned long lock_flags = 0;
	int i = 0;

	node = queue_to_node(queue);

	dev = video_get_drvdata(&node->vdev);

	stream = to_stream_by_id(dev, id);

	if (rkisp1_streamoff(dev, id))
		return;

	spin_lock_irqsave(&dev->strm_vdevs.vbq_lock, lock_flags);
	buf = stream->curr_buf;
	stream->curr_buf = NULL;
	spin_unlock_irqrestore(&dev->strm_vdevs.vbq_lock, lock_flags);
	if (buf)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	spin_lock_irqsave(&dev->strm_vdevs.vbq_lock, lock_flags);
	buf = stream->next_buf;
	stream->next_buf = NULL;
	spin_unlock_irqrestore(&dev->strm_vdevs.vbq_lock, lock_flags);
	if (buf)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	buf = NULL;

	for (i = 0; i < CIF_ISP_REQ_BUFS_MAX; i++) {
		spin_lock_irqsave(&dev->strm_vdevs.vbq_lock, lock_flags);
		if (!list_empty(&stream->buf_queue)) {
			buf = list_first_entry(&stream->buf_queue,
					       struct rkisp1_buffer, queue);
			list_del(&buf->queue);
			spin_unlock_irqrestore(&dev->strm_vdevs.vbq_lock,
					       lock_flags);
		} else {
			spin_unlock_irqrestore(&dev->strm_vdevs.vbq_lock,
					       lock_flags);
			break;
		}

		if (buf)
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		buf = NULL;
	}
}

static void mi_update_buff_addr(struct rkisp1_device *dev,
					  enum rkisp1_stream_id strm_id)
{
	spinlock_t *lock = &dev->writel_verify_lock;
	if (strm_id == RKISP1_STREAM_SP) {
		writel_verify(dev->strm_vdevs.sp_vdev.path_cfg.
				     next_buff_addr,
				     dev->config.base_addr +
				     CIF_MI_SP_Y_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(dev->strm_vdevs.sp_vdev.path_cfg.
				     next_buff_addr_cb,
				     dev->config.base_addr +
				     CIF_MI_SP_CB_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(dev->strm_vdevs.sp_vdev.path_cfg.
				     next_buff_addr_cr,
				     dev->config.base_addr +
				     CIF_MI_SP_CR_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		/*
		 * There have bee repeatedly issues with
		 * the offset registers, it is safer to write
		 * them each time, even though it is always
		 * 0 and even though that is the
		 * register's default value
		 */
		writel_verify(0,
				     dev->config.base_addr +
				     CIF_MI_SP_Y_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(0,
				     dev->config.base_addr +
				     CIF_MI_SP_CB_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(0,
				     dev->config.base_addr +
				     CIF_MI_SP_CR_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	} else if (strm_id == RKISP1_STREAM_MP) {
		writel_verify(dev->strm_vdevs.mp_vdev.path_cfg.
				     next_buff_addr,
				     dev->config.base_addr +
				     CIF_MI_MP_Y_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(dev->strm_vdevs.mp_vdev.path_cfg.
				     next_buff_addr_cb,
				     dev->config.base_addr +
				     CIF_MI_MP_CB_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(dev->strm_vdevs.mp_vdev.path_cfg.
				     next_buff_addr_cr,
				     dev->config.base_addr +
				     CIF_MI_MP_CR_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		/*
		 * There have bee repeatedly issues with
		 * the offset registers, it is safer to write
		 * them each time, even though it is always
		 * 0 and even though that is the
		 * register's default value
		 */
		writel_verify(0,
				     dev->config.base_addr +
				     CIF_MI_MP_Y_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(0,
				     dev->config.base_addr +
				     CIF_MI_MP_CB_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
		writel_verify(0,
				     dev->config.base_addr +
				     CIF_MI_MP_CR_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK, lock);
	}
}

static int mi_update_mp(struct rkisp1_device *dev)
{
	struct cif_frm_fmt *out_frm_fmt =
	    &dev->strm_vdevs.mp_vdev.path_cfg.output;
	struct cif_fmt_info *cif_fmt = &out_frm_fmt->frm_fmt;
	spinlock_t *lock = &dev->writel_verify_lock;
	u32 val;
	void *addr;

	if (dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr !=
	    dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr) {
		if (dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr ==
		    RKISP1_INVALID_BUFF_ADDR) {
			/* disable MI MP */
			v4l2_info(&dev->v4l2_dev, "disabling MP MI\n");
			val = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN | CIF_MI_CTRL_RAW_ENABLE);
			addr = dev->config.base_addr + CIF_MI_CTRL;
			writel_verify(val & readl(addr), addr, ~0, lock);
		} else if (dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr ==
			   RKISP1_INVALID_BUFF_ADDR) {
			/* re-enable MI MP */
			v4l2_info(&dev->v4l2_dev, "enabling MP MI\n");
			writel(CIF_MI_MP_FRAME,
				  dev->config.base_addr + CIF_MI_ICR);
			val = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN | CIF_MI_CTRL_RAW_ENABLE);
			addr = dev->config.base_addr + CIF_MI_CTRL;
			writel_verify(val & readl(addr), addr, ~0, lock);
			if (CIF_PIX_FMT_IS_RAW_BAYER(cif_fmt)) {
				val = CIF_MI_CTRL_RAW_ENABLE;
				addr = dev->config.base_addr + CIF_MI_CTRL;
				writel_verify(val | readl(addr), addr, ~0, lock);
			} else if (CIF_PIX_FMT_IS_YUV(cif_fmt)) {
				val = CIF_MI_CTRL_MP_ENABLE_IN;
				addr = dev->config.base_addr + CIF_MI_CTRL;
				writel_verify(val | readl(addr), addr, ~0, lock);
			}
		}
		mi_update_buff_addr(dev, RKISP1_STREAM_MP);
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr =
		    dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr;
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr_cb =
		    dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr_cb;
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr_cr =
		    dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr_cr;
	}

	return 0;
}

static int mi_update_sp(struct rkisp1_device *dev)
{
	spinlock_t *lock = &dev->writel_verify_lock;
	u32 val;
	void *addr;

	if (dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr !=
	    dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr) {
		if (dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr ==
		    RKISP1_INVALID_BUFF_ADDR) {
			/* disable MI SP */
			val = (u32)~CIF_MI_CTRL_SP_ENABLE;
			addr = dev->config.base_addr + CIF_MI_CTRL;
			writel_verify(val & readl(addr), addr, ~0, lock);
		} else if (dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr ==
			   RKISP1_INVALID_BUFF_ADDR) {
			/* re-enable MI SP */
			writel(CIF_MI_SP_FRAME,
				  dev->config.base_addr + CIF_MI_ICR);
			val = CIF_MI_CTRL_SP_ENABLE;
			addr = dev->config.base_addr + CIF_MI_CTRL;
			writel_verify(val | readl(addr), addr, ~0, lock);
		}
		mi_update_buff_addr(dev, RKISP1_STREAM_SP);
		dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr =
		    dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr;
		dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr_cb =
		    dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cb;
		dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr_cr =
		    dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cr;
	}

	return 0;
}

static int mi_frame_end(struct rkisp1_device *dev,
				  enum rkisp1_stream_id stream_id)
{
	struct rkisp1_stream *stream = NULL;
	u32 *next_buff_addr = NULL, *next_buff_addr_cb, *next_buff_addr_cr;
	int i = 0;
	struct cif_frm_fmt *fmt;
	struct cif_fmt_info *cif_fmt;
	void __iomem *y_base_addr;
	int (*update_mi)(struct rkisp1_device *dev);

	if (stream_id == RKISP1_STREAM_MP) {
		stream = &dev->strm_vdevs.mp_vdev;
		y_base_addr = dev->config.base_addr + CIF_MI_MP_Y_BASE_AD_SHD;
		next_buff_addr =
		    &dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr;
		next_buff_addr_cb =
		    &dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr_cb;
		next_buff_addr_cr =
		    &dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr_cr;
		update_mi = mi_update_mp;
	} else {
		stream = &dev->strm_vdevs.sp_vdev;
		y_base_addr = dev->config.base_addr + CIF_MI_SP_Y_BASE_AD_SHD;
		next_buff_addr =
		    &dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr;
		next_buff_addr_cb =
		    &dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cb;
		next_buff_addr_cr =
		    &dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cr;
		update_mi = mi_update_sp;
	}

	fmt = &stream->path_cfg.output;
	cif_fmt = &fmt->frm_fmt;

	if (!stream->next_buf && stream_id == RKISP1_STREAM_MP) {
		stream->stall = dev->config.out_of_buffer_stall;
	} else if ((stream->next_buf) &&
		   (vb2_dma_contig_plane_dma_addr
		    (&stream->next_buf->vb.vb2_buf, 0)
		    != readl(y_base_addr))) {
		v4l2_warn(&dev->v4l2_dev,
			  "stream id: %d buffer queue is not advancing "
			  "(0x%08x/0x%08x)\n",
			  stream_id, (stream_id & RKISP1_STREAM_MP) ?
			  readl(dev->config.base_addr +
				   CIF_MI_MP_Y_BASE_AD_INIT)
			  : readl(dev->config.base_addr +
				     CIF_MI_SP_Y_BASE_AD_INIT),
			  readl(y_base_addr));
		stream->stall = true;
	}

	if (!stream->stall) {
		/*
		 * If mi restart after switch off for buffer is empty,
		 * mi may be restart failed. So mi write data to last
		 * buffer, the last buffer isn't been release to user
		 * until new buffer queue;
		 */
		if (stream->curr_buf && stream->next_buf) {
			/* stream->curr_buf->field_count = dev->isp_dev.frame_id; */
			u32 payload_size = 0;

			for (i = 0; i < cif_fmt->mplanes; i++) {
				fmt_calc_plane_size(cif_fmt, fmt->width,
							fmt->height, i,
							&payload_size, NULL);
				vb2_set_plane_payload(&stream->curr_buf->
						      vb.vb2_buf, i,
						      payload_size);
				payload_size = 0;
			}

			vb2_buffer_done(&stream->curr_buf->vb.vb2_buf,
					VB2_BUF_STATE_DONE);
			wake_up(&stream->curr_buf->vb.vb2_buf.
				vb2_queue->done_wq);
			stream->curr_buf = NULL;
		}

		if (!stream->curr_buf) {
			stream->curr_buf = stream->next_buf;
			stream->next_buf = NULL;
		}
	}

	if (!stream->next_buf) {
		struct vb2_buffer *vb2;

		if (!list_empty(&stream->buf_queue)) {
			stream->next_buf =
			    list_first_entry(&stream->buf_queue,
					     struct rkisp1_buffer, queue);
			list_del(&stream->next_buf->queue);

			vb2 = &stream->next_buf->vb.vb2_buf;
		} else if (!dev->config.out_of_buffer_stall) {
			/*
			 * If mi restart after switch off for buffer is empty,
			 * mi may be restart failed. So mi write data to last
			 * buffer, the last buffer isn't been release to user
			 * until new buffer queue;
			 *
			 * if
			 * *next_buff_addr = RKISP1_INVALID_BUFF_ADDR;
			 * mi will stop;
			 */
			vb2 = &stream->curr_buf->vb.vb2_buf;
		}

		*next_buff_addr = vb2_dma_contig_plane_dma_addr(vb2, 0);
		if (cif_fmt->mplanes > 1)
			*next_buff_addr_cb =
			    vb2_dma_contig_plane_dma_addr(vb2, 1);
		if (cif_fmt->mplanes > 2) {
			/* swap uv */
			if (cif_fmt->uv_swap) {
				*next_buff_addr_cr = *next_buff_addr_cb;
				*next_buff_addr_cb =
				    vb2_dma_contig_plane_dma_addr(vb2, 2);
			} else {
				*next_buff_addr_cr =
				    vb2_dma_contig_plane_dma_addr(vb2, 2);
			}
		}
	}
	(void)update_mi(dev);

	stream->stall = false;
	return 0;
}

static void mi_start(struct rkisp1_device *dev,
			       bool start_mi_sp, bool start_mi_mp)
{
	u32 val;
	void *addr;

	if (start_mi_sp &&
	    (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING))
		start_mi_sp = false;
	if (start_mi_mp &&
	    (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING))
		start_mi_mp = false;
	if (!start_mi_sp && !start_mi_mp)
		return;

	if ((start_mi_sp &&
	     (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING)) ||
	    (start_mi_mp &&
	     (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING)))
		WARN_ON(1);

	if (start_mi_sp) {
		dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr =
		    RKISP1_INVALID_BUFF_ADDR;
		dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr =
		    RKISP1_INVALID_BUFF_ADDR;
		spin_lock(&dev->strm_vdevs.vbq_lock);
		mi_frame_end(dev, RKISP1_STREAM_SP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		dev->strm_vdevs.sp_vdev.stall = false;
	}

	if (start_mi_mp) {
		dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr =
		    RKISP1_INVALID_BUFF_ADDR;
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr =
		    RKISP1_INVALID_BUFF_ADDR;
		spin_lock(&dev->strm_vdevs.vbq_lock);
		mi_frame_end(dev, RKISP1_STREAM_MP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		dev->strm_vdevs.mp_vdev.stall = false;
	}

	addr = dev->config.base_addr + CIF_MI_INIT;
	val = readl(addr) | CIF_MI_INIT_SOFT_UPD;
	writel(val, addr);

	if (start_mi_sp) {
		spin_lock(&dev->strm_vdevs.vbq_lock);
		mi_frame_end(dev, RKISP1_STREAM_SP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		if (dev->strm_vdevs.sp_vdev.curr_buf)
			dev->strm_vdevs.sp_vdev.path_cfg.busy = true;
	}

	if (start_mi_mp) {
		spin_lock(&dev->strm_vdevs.vbq_lock);
		mi_frame_end(dev, RKISP1_STREAM_MP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		if (dev->strm_vdevs.mp_vdev.curr_buf)
			dev->strm_vdevs.mp_vdev.path_cfg.busy = true;
	}

	if (!dev->strm_vdevs.mi_config.async_updt){
		addr = dev->config.base_addr + CIF_ISP_CTRL;
		val = CIF_ISP_CTRL_ISP_GEN_CFG_UPD;
		writel(readl(addr) | val, addr);
	}
}

static int
rkisp1_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	int ret = 0;
	struct rkisp1_vdev_node *node;
	enum rkisp1_stream_id id = queue_to_stream_id(queue);
	struct rkisp1_stream *stream = NULL;
	struct rkisp1_device *dev;
	bool start_sp = false, start_mp = false;

	node = queue_to_node(queue);
	dev = video_get_drvdata(&node->vdev);
	stream = to_stream_by_id(dev, id);

	if (stream->id == RKISP1_STREAM_SP) {
		start_sp = true;
		if (dev->strm_vdevs.mp_vdev.restart &&
		    dev->strm_vdevs.mp_vdev.updt_cfg)
			start_mp = true;

	} else {
		start_mp = true;
		if (dev->strm_vdevs.sp_vdev.restart &&
		    dev->strm_vdevs.sp_vdev.updt_cfg)
			start_sp = true;
	}

	if (start_sp &&
	    (dev->strm_vdevs.sp_vdev.state != RKISP1_STATE_STREAMING)) {
		ret = rkisp1_config_stream_path(dev, RKISP1_STREAM_SP);
		if (ret < 0)
			return ret;

		ret = rkisp1_config_mi_sp(dev);
		if (ret < 0)
			return ret;

		mi_start(dev, true, false);
		dev->strm_vdevs.sp_vdev.state = RKISP1_STATE_STREAMING;
	}
	if (start_mp &&
	    (dev->strm_vdevs.mp_vdev.state != RKISP1_STATE_STREAMING)) {
		ret = rkisp1_config_stream_path(dev, RKISP1_STREAM_MP);
		if (ret < 0)
			return ret;
		ret = rkisp1_config_mi_mp(dev);
		if (ret < 0)
			return ret;

		mi_start(dev, false, true);
		dev->strm_vdevs.mp_vdev.state = RKISP1_STATE_STREAMING;
	}

	/* pipeline stream on */
	ret = (node->pipe.set_stream) (&node->pipe, true);
	if (ret < 0)
		return ret;

	if (start_mp) {
		dev->strm_vdevs.mp_vdev.updt_cfg = false;
		dev->strm_vdevs.mp_vdev.restart = false;
	}

	if (start_sp) {
		dev->strm_vdevs.sp_vdev.updt_cfg = false;
		dev->strm_vdevs.sp_vdev.restart = false;
	}

	return 0;
}

static struct vb2_ops rkisp1_vb2_ops = {
	.queue_setup = rkisp1_queue_setup,
	.buf_queue = rkisp1_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.stop_streaming = rkisp1_stop_streaming,
	.start_streaming = rkisp1_start_streaming,
};

static int rkisp_init_vb2_queue(struct vb2_queue *q,
				    struct rkisp1_device *dev,
				    enum v4l2_buf_type buf_type)
{
	struct rkisp1_vdev_node *node;

	memset(q, 0, sizeof(*q));
	node = queue_to_node(q);

	q->type = buf_type;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = dev;
	q->ops = &rkisp1_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct rkisp1_buffer);
	q->min_buffers_needed = 4;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	return vb2_queue_init(q);
}

/***************************** v4l2_file_operations*******************************/
static void _init_default_fmt(struct rkisp1_stream *stream)
{
	struct cif_frm_fmt strm_fmt;
	struct rkisp1_device *dev = stream_to_dev(stream);
	const u32 def_fmt = V4L2_PIX_FMT_YUYV;

	strm_fmt.width = STREAM_PATH_DEFAULT_WIDTH;
	strm_fmt.height = STREAM_PATH_DEFAULT_HEIGHT;
	strm_fmt.quantization = 0;
	strm_fmt.frm_fmt = *find_fmt(stream->id, &def_fmt, -1);
	rkisp1_s_fmt(dev, stream->id, &strm_fmt);
}

static void _init_stream(struct rkisp1_device *dev,
				  enum rkisp1_stream_id stream_id)
{
	struct rkisp1_stream *stream = NULL;

	stream = to_stream_by_id(dev, stream_id);
	switch (stream_id) {
	case RKISP1_STREAM_SP:
		dev->strm_vdevs.sp_vdev.path_cfg.busy = false;
		break;
	case RKISP1_STREAM_MP:
		dev->strm_vdevs.mi_config.raw_enable = false;
		dev->strm_vdevs.mp_vdev.path_cfg.busy = false;
		break;
	default:
		break;
	}

	INIT_LIST_HEAD(&stream->buf_queue);
	stream->next_buf = NULL;
	stream->curr_buf = NULL;
	stream->updt_cfg = false;
	stream->stop = false;
	stream->stall = false;
	stream->restart = false;

	_init_default_fmt(stream);
}

int rkisp1_stream_init(struct rkisp1_device *dev, u32 stream_ids)
{
	v4l2_info(&dev->v4l2_dev, "0x%08x\n", stream_ids);

	if (stream_ids & ~(RKISP1_ALL_STREAMS)) {
		v4l2_err(&dev->v4l2_dev,
			 "unknown/unsupported stream IDs 0x%08x\n", stream_ids);
		return -EINVAL;
	}

	if (stream_ids & RKISP1_STREAM_SP)
		_init_stream(dev, RKISP1_STREAM_SP);
	if (stream_ids & RKISP1_STREAM_MP)
		_init_stream(dev, RKISP1_STREAM_MP);

	return 0;
}

int rkisp1_stream_release(struct rkisp1_device *dev, int stream_ids)
{
	int ret;
	struct rkisp1_stream *strm_dev;
	struct rkisp1_vdev_node *node;

	v4l2_info(&dev->v4l2_dev, "0x%08x\n", stream_ids);

	if ((dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_DISABLED) &&
	    (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_DISABLED))
		return 0;

	if (stream_ids & ~(RKISP1_ALL_STREAMS)) {
		v4l2_err(&dev->v4l2_dev,
			 "unknown/unsupported stream IDs 0x%08x\n", stream_ids);
		return -EINVAL;
	}

	if (stream_ids & RKISP1_STREAM_SP) {
		strm_dev = &dev->strm_vdevs.sp_vdev;

		if (strm_dev->state == RKISP1_STATE_STREAMING) {
			v4l2_warn(&dev->v4l2_dev,
				  "CIF SP in streaming state, should be stopped"
				  "before release, trying to stop it\n");
			rkisp1_stream_stop_sp(dev);
			node = &strm_dev->vnode;

			ret = (node->pipe.set_stream) (&node->pipe, false);
			if (ret < 0)
				return ret;
		}
		strm_dev->state = RKISP1_STATE_DISABLED;
	}
	if (stream_ids & RKISP1_STREAM_MP) {
		strm_dev = &dev->strm_vdevs.mp_vdev;

		if (strm_dev->state == RKISP1_STATE_STREAMING) {
			v4l2_warn(&dev->v4l2_dev,
				  "CIF MP in streaming state, should be stopped"
				  "before release, trying to stop it\n");
			rkisp1_stream_stop_mp(dev);
			node = &strm_dev->vnode;

			ret = (node->pipe.set_stream) (&node->pipe, false);
			if (ret < 0)
				return ret;
		}
		strm_dev->state = RKISP1_STATE_DISABLED;
	}

	return 0;
}

static int rkisp1_open(struct file *file)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_device *dev = video_get_drvdata(vdev);

	ret = v4l2_fh_open(file);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "v4l2_fh_open failed %d\n", ret);
		return ret;
	}

	/* Already initialized */
	if (!v4l2_fh_is_singular_file(file))
		return 0;

	/* First open of the device, so initialize everything */
	ret = rkisp1_stream_init(dev, file_to_stream_id(file));
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		v4l2_fh_release(file);
		return ret;
	}

	return 0;
}

static int rkisp1_release(struct file *file)
{
	struct vb2_queue *queue = to_vb2_queue(file);
	struct rkisp1_device *dev = queue->drv_priv;
	enum rkisp1_stream_id stream_id = file_to_stream_id(file);

	if (v4l2_fh_is_singular_file(file))
		rkisp1_stream_release(dev, stream_id);

	vb2_fop_release(file);
	return 0;
}

static const struct v4l2_file_operations rkisp1_fops = {
	.open = rkisp1_open,
	.release = rkisp1_release,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

/*****************************mp and sp v4l2_ioctl_ops*******************************/
static int rkisp1_querycap(struct file *file,
				   void *priv, struct v4l2_capability *cap)
{
	struct video_device *vdev = video_devdata(file);
	u32 stream_ids = file_to_stream_id(file);

	strcpy(cap->driver, DRIVER_NAME);
	strlcpy(cap->card, vdev->name, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform: " DRIVER_NAME, sizeof(cap->bus_info));

	if ((stream_ids == RKISP1_STREAM_SP) ||
	    (stream_ids == RKISP1_STREAM_MP))
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				   V4L2_CAP_STREAMING;

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
	    V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int rkisp1_subscribe_event(struct v4l2_fh *fh,
					  const struct v4l2_event_subscription
					  *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

static int rkisp1_enum_input(struct file *file, void *priv,
				     struct v4l2_input *input)
{
	struct vb2_queue *queue = to_vb2_queue(file);
	struct rkisp1_device *dev = queue->drv_priv;
	const char *inp_name;

	if (queue->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		v4l2_err(&dev->v4l2_dev, "wrong buffer queue %d\n",
			 queue->type);
		return -EINVAL;
	}

	inp_name = dev_driver_string(dev->subdevs[RKISP1_SD_SENSOR]->dev);
	if (IS_ERR(inp_name))
		return PTR_ERR(inp_name);

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_UNKNOWN;
	strncpy(input->name, inp_name, sizeof(input->name) - 1);

	return 0;
}

static int rkisp1_enum_fmt_vid_cap_mplane(struct file *file, void *priv,
					      struct v4l2_fmtdesc *f)
{
	struct vb2_queue *queue = to_vb2_queue(file);
	struct rkisp1_device *dev = queue->drv_priv;
	u32 stream_id = file_to_stream_id(file);
	const struct cif_fmt_info *cif_fmt = NULL, *check_fmt;
	int i = f->index, fmt_index = 0;

	do {
		cif_fmt = find_fmt(stream_id, NULL, i);
		if (!cif_fmt) {
			v4l2_err(&dev->v4l2_dev, "index %d\n", f->index);
			return -EINVAL;
		}

		/* check whether this format is supported by specific stream
		  * and skip the queried formats.
		  */
		check_fmt = find_fmt(stream_id, &cif_fmt->fourcc, -1);
		if ((check_fmt == cif_fmt) && (fmt_index++ == f->index)) {
			f->pixelformat = cif_fmt->fourcc;
			return 0;
		}
	} while (++i < ARRAY_SIZE(rkisp1_output_formats));

	return -EINVAL;
}

static const struct v4l2_ioctl_ops sp_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = rkisp1_v4l2_streamon,
	.vidioc_streamoff = rkisp1_v4l2_streamoff,
	.vidioc_enum_input = rkisp1_enum_input,
	.vidioc_try_fmt_vid_cap_mplane = rkisp1_try_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = rkisp1_enum_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = rkisp1_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = rkisp1_g_fmt_vid_cap_mplane,
	.vidioc_querycap = rkisp1_querycap,
	.vidioc_subscribe_event = rkisp1_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_ioctl_ops mp_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = rkisp1_v4l2_streamon,
	.vidioc_streamoff = rkisp1_v4l2_streamoff,
	.vidioc_enum_input = rkisp1_enum_input,
	.vidioc_try_fmt_vid_cap_mplane = rkisp1_try_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = rkisp1_enum_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = rkisp1_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = rkisp1_g_fmt_vid_cap_mplane,
	.vidioc_querycap = rkisp1_querycap,
};

int rkisp1_register_videodev(struct rkisp1_device *dev,
					 		enum rkisp1_stream_id stream_id)
{
	struct rkisp1_vdev_node *node;
	struct video_device *vdev;
	int ret, major, qtype;

	if (stream_id == RKISP1_STREAM_SP){
		vdev = &dev->strm_vdevs.sp_vdev.vnode.vdev;
		strlcpy(vdev->name, SP_VDEV_NAME, sizeof(vdev->name));
		vdev->ioctl_ops = &sp_v4l2_ioctl_ops;
		major = RKISP1_V4L2_SP_DEV_MAJOR;
		qtype = V4L2_CAP_VIDEO_CAPTURE;
	}else{
		vdev = &dev->strm_vdevs.mp_vdev.vnode.vdev;
		strlcpy(vdev->name, MP_VDEV_NAME, sizeof(vdev->name));
		vdev->ioctl_ops = &mp_v4l2_ioctl_ops;
		major = RKISP1_V4L2_MP_DEV_MAJOR;
		qtype = V4L2_CAP_VIDEO_CAPTURE;
	}
	node = vdev_to_node(vdev);
	node->pipe.open = isp_pipeline_open;
	node->pipe.close = isp_pipeline_close;
	node->pipe.set_stream = isp_pipeline_set_stream;
	mutex_init(&node->vlock);

	vdev->release = video_device_release_empty;
	vdev->fops = &rkisp1_fops;
	video_set_drvdata(vdev, dev);
	vdev->minor = -1;
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->lock = &node->vlock;
	if (qtype == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		vdev->vfl_dir = VFL_DIR_TX;
		node->pad.flags = MEDIA_PAD_FL_SOURCE;
	} else {
		vdev->vfl_dir = VFL_DIR_RX;
		node->pad.flags = MEDIA_PAD_FL_SINK;
	}

	ret = media_entity_init(&vdev->entity, 1, &node->pad, 0);
	if (ret < 0)
		return ret;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, major);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "video_register_device failed with error %d\n", ret);
		return ret;
	}

	dev->alloc_ctx = vb2_dma_contig_init_ctx(dev->dev);
	rkisp_init_vb2_queue(&node->buf_queue, dev,
				 V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	vdev->queue = &node->buf_queue;

	return 0;
}

void rkisp1_unregister_video_device(struct rkisp1_stream *stream)
{
	media_entity_cleanup(&stream->vnode.vdev.entity);
	video_unregister_device(&stream->vnode.vdev);
}

void rkisp1_mi_isr(unsigned int mi_mis, struct rkisp1_device *dev)
{
	unsigned int mi_mis_tmp, d;
	void *a;

	if (mi_mis & CIF_MI_SP_FRAME) {
		dev->strm_vdevs.sp_vdev.path_cfg.busy = false;
		writel(CIF_MI_SP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		mi_mis_tmp = readl(dev->config.base_addr + CIF_MI_MIS);
		if (mi_mis_tmp & CIF_MI_SP_FRAME)
			v4l2_err(&dev->v4l2_dev, "sp icr err: 0x%x\n",
				 mi_mis_tmp);
	}

	if (mi_mis & CIF_MI_MP_FRAME) {
		dev->strm_vdevs.mp_vdev.path_cfg.busy = false;
		writel(CIF_MI_MP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		mi_mis_tmp = readl(dev->config.base_addr + CIF_MI_MIS);
		if (mi_mis_tmp & CIF_MI_MP_FRAME)
			v4l2_err(&dev->v4l2_dev, "mp icr err: 0x%x\n",
				 mi_mis_tmp);
	}

	if (!RKISP1_MI_IS_BUSY(dev)) {
		if (dev->strm_vdevs.mi_config.async_updt) {
			u32 mp_y_off_cnt_shd =
			    readl(dev->config.base_addr +
				     CIF_MI_MP_Y_OFFS_CNT_SHD);
			u32 sp_y_off_cnt_shd =
			    readl(dev->config.base_addr +
				     CIF_MI_SP_Y_OFFS_CNT_SHD);

			writel(CIF_MI_INIT_SOFT_UPD,
				  dev->config.base_addr + CIF_MI_INIT);
			if ((dev->strm_vdevs.mi_config.async_updt &
			     RKISP1_ASYNC_ISM))
				dev->strm_vdevs.mi_config.async_updt &=
				    ~RKISP1_ASYNC_ISM;
			if (mp_y_off_cnt_shd != 0 || sp_y_off_cnt_shd != 0) {
				v4l2_info(&dev->v4l2_dev,
					  "soft update too late (SP offset %d, "
					  "MP offset %d)\n",
					  sp_y_off_cnt_shd, mp_y_off_cnt_shd);
			}
		}

		if (dev->strm_vdevs.mp_vdev.stop &&
		    (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING)) {
			rkisp1_stop_mi(dev, false, true);
			dev->strm_vdevs.mp_vdev.state = RKISP1_STATE_READY;
			dev->strm_vdevs.mp_vdev.stop = false;

			/* Turn off MRSZ since it is not needed */
			writel(0, dev->config.base_addr + CIF_MRSZ_CTRL);
			d = CIF_RSZ_CTRL_CFG_UPD;
			a = dev->config.base_addr + CIF_MRSZ_CTRL;
			writel(readl(a) | d, a);
			v4l2_info(&dev->v4l2_dev, "MP has stopped\n");
			wake_up_interruptible(&dev->strm_vdevs.mp_vdev.done);
		}
		if (dev->strm_vdevs.sp_vdev.stop &&
		    (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING)) {
			rkisp1_stop_mi(dev, true, false);
			dev->strm_vdevs.sp_vdev.state = RKISP1_STATE_READY;
			dev->strm_vdevs.sp_vdev.stop = false;

			/* Turn off SRSZ since it is not needed */
			writel(0, dev->config.base_addr + CIF_SRSZ_CTRL);
			d = CIF_RSZ_CTRL_CFG_UPD;
			a = dev->config.base_addr + CIF_SRSZ_CTRL;
			writel((readl(a) | (u32)(d)), a);
			v4l2_info(&dev->v4l2_dev, "SP has stopped\n");
			wake_up_interruptible(&dev->strm_vdevs.sp_vdev.done);
		}

		if (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING) {
			spin_lock(&dev->strm_vdevs.vbq_lock);
			(void)mi_frame_end(dev, RKISP1_STREAM_SP);
			spin_unlock(&dev->strm_vdevs.vbq_lock);
		}
		if (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING) {
			spin_lock(&dev->strm_vdevs.vbq_lock);
			(void)mi_frame_end(dev, RKISP1_STREAM_MP);
			spin_unlock(&dev->strm_vdevs.vbq_lock);
		}

		dev->b_mi_frame_end = true;

		if ((dev->strm_vdevs.sp_vdev.state ==
		     RKISP1_STATE_STREAMING) &&
		    dev->strm_vdevs.sp_vdev.curr_buf)
			dev->strm_vdevs.sp_vdev.path_cfg.busy = true;
		if ((dev->strm_vdevs.mp_vdev.state ==
		     RKISP1_STATE_STREAMING) &&
		    dev->strm_vdevs.mp_vdev.curr_buf)
			dev->strm_vdevs.mp_vdev.path_cfg.busy = true;
		/* TODO: update crop & resize */
	}

	writel((u32) (~(CIF_MI_MP_FRAME | CIF_MI_SP_FRAME)),
		  dev->config.base_addr + CIF_MI_ICR);
}
