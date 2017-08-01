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

#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include "path_video.h"
#include "regs.h"
#include "rkisp1.h"

/* spinlock define */
spinlock_t iowrite32_verify_lock;

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


static const struct cif_fmt_info cif_isp10_output_formats[] = {
	/* yuv422 */
	CIF_FMT_YUV(V4L2_PIX_FMT_YUYV, MEDIA_BUS_FMT_YUYV8_2X8, 16, 0, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_YVYU, MEDIA_BUS_FMT_YVYU8_2X8, 16, 0, 1, 1),
	CIF_FMT_YUV(V4L2_PIX_FMT_UYVY, MEDIA_BUS_FMT_UYVY8_2X8, 16, 0, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_VYUY, MEDIA_BUS_FMT_VYUY8_2X8, 16, 0, 1, 1),
	CIF_FMT_YUV(V4L2_PIX_FMT_YUV422P, MEDIA_BUS_FMT_YUYV8_2X8, 16, 2, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV16, MEDIA_BUS_FMT_YUYV8_2X8, 16, 1, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV61, MEDIA_BUS_FMT_YUYV8_2X8, 16, 1, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_YVU422M, MEDIA_BUS_FMT_YUYV8_2X8, 16, 2, 3, 0),
	/* yuv420 */
	CIF_FMT_YUV(V4L2_PIX_FMT_NV21, MEDIA_BUS_FMT_YUYV8_1_5X8, 12, 1, 1, 1),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV12, MEDIA_BUS_FMT_YVYU8_1_5X8, 12, 1, 1, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV21M, MEDIA_BUS_FMT_YUYV8_1_5X8, 12, 1, 2, 1),
	CIF_FMT_YUV(V4L2_PIX_FMT_NV12M, MEDIA_BUS_FMT_YVYU8_1_5X8, 12, 1, 2, 0),
	CIF_FMT_YUV(V4L2_PIX_FMT_YVU420, MEDIA_BUS_FMT_YVYU8_1_5X8, 12, 2, 1, 1),
	/* yuv444 */
	CIF_FMT_YUV(V4L2_PIX_FMT_YUV444M, 0, 24, 2, 3, 0),
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

static const struct cif_fmt_info *cif_isp10_find_fmt(enum
						     cif_isp10_stream_id
						     id,
						     const u32 *
						     pixelfmt, int index)
{
	unsigned int i, array_size;
	const struct cif_fmt_info *fmt, *def_fmt = &cif_isp10_output_formats[0];

	if ((id != RKISP1_STREAM_SP) && (id != RKISP1_STREAM_MP))
		return NULL;

	array_size = ARRAY_SIZE(cif_isp10_output_formats);
	if (index >= (int)array_size)
		return NULL;

	for (i = 0; i < array_size; i++) {
		fmt = &cif_isp10_output_formats[i];
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

static struct cif_isp10_buffer *to_cif_isp10_vb(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct cif_isp10_buffer, vb);
}

static struct vb2_queue *to_vb2_queue(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct cif_isp10_vdev_node *node = vdev_to_node(vdev);

	WARN_ON(!vdev);

	return &node->buf_queue;
}

static inline
struct cif_isp10_stream *node_to_stream(struct cif_isp10_vdev_node *node)
{
	return container_of(node, struct cif_isp10_stream, vnode);
}

static enum cif_isp10_stream_id file_to_stream_id(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct cif_isp10_vdev_node *node = vdev_to_node(vdev);
	struct cif_isp10_stream *stream =
	    container_of(node, struct cif_isp10_stream, vnode);

	return stream->id;
}

static enum cif_isp10_stream_id queue_to_stream_id(struct vb2_queue *queue)
{
	struct cif_isp10_vdev_node *node = queue_to_node(queue);

	return node_to_stream(node)->id;
}

static int cif_isp10_s_fmt_mp(struct rkisp1_device *dev,
			      struct cif_frm_fmt *strm_fmt)
{
	const struct cif_fmt_info *cif_fmt =
	    cif_isp10_find_fmt(RKISP1_STREAM_MP,
			       &strm_fmt->frm_fmt.fourcc,
			       -1);
	strm_fmt->frm_fmt = *cif_fmt;

	v4l2_info(&dev->v4l2_dev,
		  "%c%c%c%c %dx%d, "
		  "quantization: %d\n",
		  (strm_fmt->frm_fmt.fourcc & 0xff),
		  (strm_fmt->frm_fmt.fourcc >> 8) & 0xff,
		  (strm_fmt->frm_fmt.fourcc >> 16) & 0xff,
		  (strm_fmt->frm_fmt.fourcc >> 24) & 0xff,
		  strm_fmt->width, strm_fmt->height, strm_fmt->quantization);

	if (CIF_PIX_FMT_IS_RAW_BAYER(cif_fmt)) {
		if ((dev->strm_vdevs.sp_vdev.state ==
		     RKISP1_STATE_READY) ||
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
	v4l2_info(&dev->v4l2_dev, "mp llength=0x%x\n",
		  dev->strm_vdevs.mp_vdev.path_cfg.llength);

	dev->strm_vdevs.mp_vdev.updt_cfg = true;
	dev->strm_vdevs.mp_vdev.state = RKISP1_STATE_READY;

	return 0;
}

static int cif_isp10_s_fmt_sp(struct rkisp1_device *dev,
			      struct cif_frm_fmt *strm_fmt)
{
	const struct cif_fmt_info *cif_fmt =
	    cif_isp10_find_fmt(RKISP1_STREAM_SP,
			       &strm_fmt->frm_fmt.fourcc,
			       -1);
	strm_fmt->frm_fmt = *cif_fmt;

	v4l2_info(&dev->v4l2_dev,
		  "%c%c%c%c %dx%d, "
		  "quantization: %d\n",
		  (strm_fmt->frm_fmt.fourcc & 0xff),
		  (strm_fmt->frm_fmt.fourcc >> 8) & 0xff,
		  (strm_fmt->frm_fmt.fourcc >> 16) & 0xff,
		  (strm_fmt->frm_fmt.fourcc >> 24) & 0xff,
		  strm_fmt->width, strm_fmt->height, strm_fmt->quantization);

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

static int cif_isp10_s_fmt(struct rkisp1_device *dev,
			   enum cif_isp10_stream_id stream_id,
			   struct cif_frm_fmt *strm_fmt)
{
	v4l2_info(&dev->v4l2_dev, "stream: %d\n", stream_id);

	switch (stream_id) {
	case RKISP1_STREAM_SP:
		return cif_isp10_s_fmt_sp(dev, strm_fmt);
	case RKISP1_STREAM_MP:
		return cif_isp10_s_fmt_mp(dev, strm_fmt);
	default:
		v4l2_err(&dev->v4l2_dev, "unknown/unsupported stream ID %d\n",
			 stream_id);
		return -EINVAL;
	}

	return 0;
}

static int rkisp1_config_stream_path(struct rkisp1_device *dev, u32 stream_ids)
{
	u32 dpcl = ioread32(dev->config.base_addr + CIF_VI_DPCL);

	/* chan_mode */
	if (stream_ids & RKISP1_STREAM_SP)
		dpcl |= CIF_VI_DPCL_CHAN_MODE_SP;

	if (stream_ids & RKISP1_STREAM_MP) {
		dpcl |= CIF_VI_DPCL_CHAN_MODE_MP;
		dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_MI;
	}
	iowrite32(dpcl, dev->config.base_addr + CIF_VI_DPCL);

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
	u32 swap_cb_cr = 0;
	u32 bpp = cif_fmt->bpp;
	u32 size = llength * height * bpp / 8;
	u32 mi_ctrl, d;
	void *a;

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

	cif_iowrite32_verify(dev->strm_vdevs.mp_vdev.path_cfg.y_size,
			     dev->config.base_addr + CIF_MI_MP_Y_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	cif_iowrite32_verify(dev->strm_vdevs.mp_vdev.path_cfg.cb_size,
			     dev->config.base_addr + CIF_MI_MP_CB_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	cif_iowrite32_verify(dev->strm_vdevs.mp_vdev.path_cfg.cr_size,
			     dev->config.base_addr + CIF_MI_MP_CR_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	d = CIF_MI_MP_FRAME;
	a = dev->config.base_addr + CIF_MI_IMSC;
	cif_iowrite32_verify(d | ioread32(a), a, ~0);

	if (swap_cb_cr) {
		d = swap_cb_cr;
		a = dev->config.base_addr + CIF_MI_XTD_FORMAT_CTRL;
		iowrite32(ioread32(a) | d, a);
	}

	mi_ctrl = ioread32(dev->config.base_addr + CIF_MI_CTRL) |
	    CIF_MI_CTRL_MP_WRITE_FMT(writeformat) |
	    CIF_MI_CTRL_BURST_LEN_LUM_64 |
	    CIF_MI_CTRL_BURST_LEN_CHROM_64 |
	    CIF_MI_CTRL_INIT_BASE_EN |
	    CIF_MI_CTRL_INIT_OFFSET_EN | CIF_MI_MP_AUTOUPDATE_ENABLE;

	cif_iowrite32_verify(mi_ctrl, dev->config.base_addr + CIF_MI_CTRL, ~0);

	v4l2_info(&dev->v4l2_dev,
		  "\n  MI_CTRL 0x%08x\n"
		  "  MI_STATUS 0x%08x\n"
		  "  MI_MP_Y_SIZE %d\n"
		  "  MI_MP_CB_SIZE %d\n"
		  "  MI_MP_CR_SIZE %d\n",
		  ioread32(dev->config.base_addr + CIF_MI_CTRL),
		  ioread32(dev->config.base_addr + CIF_MI_STATUS),
		  ioread32(dev->config.base_addr + CIF_MI_MP_Y_SIZE_INIT),
		  ioread32(dev->config.base_addr + CIF_MI_MP_CB_SIZE_INIT),
		  ioread32(dev->config.base_addr + CIF_MI_MP_CR_SIZE_INIT));

	return 0;
}

static struct v4l2_subdev *cif_isp10_remote_subdev(struct cif_isp10_vdev_node
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

	for (i = 0; i < ARRAY_SIZE(cif_isp10_output_formats); ++i) {
		/* TODO: should do more checks */
		if (cif_isp10_output_formats[i].mbus_code == mbus->code)
			break;
	}

	if (WARN_ON(i == ARRAY_SIZE(cif_isp10_output_formats)))
		return 0;

	cifmt->frm_fmt = cif_isp10_output_formats[i];

	return 0;
}

static int
__cif_video_get_format(struct cif_isp10_vdev_node *video,
		       struct cif_frm_fmt *cifmt)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = cif_isp10_remote_subdev(video, &pad);
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
cif_isp10_check_format(struct cif_isp10_vdev_node *video)
{
	struct cif_isp10_stream *strm = node_to_stream(video);
	struct cif_frm_fmt *video_fmt = &strm->path_cfg.output;
	struct cif_frm_fmt remote_fmt;
	int ret;

	ret = __cif_video_get_format(video, &remote_fmt);
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
	u32 size = llength * height * bpp / 8;
	u32 input_format = 0;
	u32 output_format;
	u32 mi_ctrl, d;
	void *a;

	v4l2_info(&dev->v4l2_dev,
		  "%c%c%c%c %dx%d, llength = %d\n",
		  (out_cif_fmt->fourcc & 0xff),
		  (out_cif_fmt->fourcc >> 8) & 0xff,
		  (out_cif_fmt->fourcc >> 16) & 0xff,
		  (out_cif_fmt->fourcc >> 24) & 0xff, width, height, llength);

	if (__cif_video_get_format(&dev->strm_vdevs.sp_vdev.vnode, &in_frm_fmt))
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

	cif_iowrite32_verify(dev->strm_vdevs.sp_vdev.path_cfg.y_size,
			     dev->config.base_addr + CIF_MI_SP_Y_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK);

	cif_iowrite32_verify(dev->strm_vdevs.sp_vdev.path_cfg.y_size,
			     dev->config.base_addr + CIF_MI_SP_Y_PIC_SIZE,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	cif_iowrite32_verify(dev->strm_vdevs.sp_vdev.path_cfg.cb_size,
			     dev->config.base_addr + CIF_MI_SP_CB_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	cif_iowrite32_verify(dev->strm_vdevs.sp_vdev.path_cfg.cr_size,
			     dev->config.base_addr + CIF_MI_SP_CR_SIZE_INIT,
			     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	cif_iowrite32_verify(width,
			     dev->config.base_addr + CIF_MI_SP_Y_PIC_WIDTH,
			     ~0x3);
	cif_iowrite32_verify(height,
			     dev->config.base_addr + CIF_MI_SP_Y_PIC_HEIGHT,
			     ~0x3);
	cif_iowrite32_verify(llength,
			     dev->config.base_addr + CIF_MI_SP_Y_LLENGTH, ~0x3);
	d = CIF_MI_SP_FRAME;
	a = dev->config.base_addr + CIF_MI_IMSC;
	cif_iowrite32_verify(d | ioread32(a), a, ~0);

	if (swap_cb_cr) {
		d = swap_cb_cr;
		a = dev->config.base_addr + CIF_MI_XTD_FORMAT_CTRL;
		iowrite32(ioread32(a) | d, a);
	}

	mi_ctrl = ioread32(dev->config.base_addr + CIF_MI_CTRL) |
	    CIF_MI_CTRL_SP_WRITE_FMT(writeformat) |
	    input_format |
	    output_format |
	    CIF_MI_CTRL_BURST_LEN_LUM_64 |
	    CIF_MI_CTRL_BURST_LEN_CHROM_64 |
	    CIF_MI_CTRL_INIT_BASE_EN |
	    CIF_MI_CTRL_INIT_OFFSET_EN | CIF_MI_SP_AUTOUPDATE_ENABLE;
	cif_iowrite32_verify(mi_ctrl, dev->config.base_addr + CIF_MI_CTRL, ~0);

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
		  ioread32(dev->config.base_addr + CIF_MI_CTRL),
		  ioread32(dev->config.base_addr + CIF_MI_STATUS),
		  ioread32(dev->config.base_addr + CIF_MI_SP_Y_SIZE_INIT),
		  ioread32(dev->config.base_addr + CIF_MI_SP_CB_SIZE_INIT),
		  ioread32(dev->config.base_addr + CIF_MI_SP_CR_SIZE_INIT),
		  ioread32(dev->config.base_addr + CIF_MI_SP_Y_PIC_WIDTH),
		  ioread32(dev->config.base_addr + CIF_MI_SP_Y_PIC_HEIGHT),
		  ioread32(dev->config.base_addr + CIF_MI_SP_Y_LLENGTH),
		  ioread32(dev->config.base_addr + CIF_MI_SP_Y_PIC_SIZE));

	return 0;
}

static void cif_isp10_stop_mi(struct rkisp1_device *dev,
			      bool stop_mi_sp, bool stop_mi_mp)
{
	u32 d;
	void *a;

	if (stop_mi_sp &&
	    (dev->strm_vdevs.sp_vdev.state != RKISP1_STATE_STREAMING))
		stop_mi_sp = false;
	if (stop_mi_mp &&
	    (dev->strm_vdevs.mp_vdev.state != RKISP1_STATE_STREAMING))
		stop_mi_mp = false;

	if (!stop_mi_sp && !stop_mi_mp)
		return;

	if (stop_mi_sp && stop_mi_mp) {
		d = (u32)~(CIF_MI_SP_FRAME | CIF_MI_MP_FRAME);
		a = dev->config.base_addr + CIF_MI_IMSC;
		cif_iowrite32_verify(d & ioread32(a), a, ~0);
		d = CIF_MI_SP_FRAME | CIF_MI_MP_FRAME;
		a = dev->config.base_addr + CIF_MI_ICR;
		iowrite32(d, a);
		d = (u32)~CIF_MI_CTRL_SP_ENABLE;
		a = dev->config.base_addr + CIF_MI_CTRL;
		cif_iowrite32_verify(d & ioread32(a), a, ~0);
		d = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN |
			CIF_MI_CTRL_SP_ENABLE | CIF_MI_CTRL_RAW_ENABLE);
		a = dev->config.base_addr + CIF_MI_CTRL;
		cif_iowrite32_verify(d & ioread32(a), a, ~0);
		iowrite32(CIF_MI_INIT_SOFT_UPD,
			  dev->config.base_addr + CIF_MI_INIT);
	} else if (stop_mi_sp) {
		iowrite32(CIF_MI_SP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		d = (u32)~CIF_MI_CTRL_SP_ENABLE;
		a = dev->config.base_addr + CIF_MI_CTRL;
		cif_iowrite32_verify(d & ioread32(a), a, ~0);
	} else if (stop_mi_mp) {
		iowrite32(CIF_MI_MP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		d = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN | CIF_MI_CTRL_RAW_ENABLE);
		a = dev->config.base_addr + CIF_MI_CTRL;
		cif_iowrite32_verify(d & ioread32(a), a,	~0);
	}
}

static int cif_isp10_stream_stop_sp(struct rkisp1_device *dev)
{
	int ret = 0;
	unsigned long flags = 0;

	if (!dev->strm_vdevs.mi_config.async_updt) {
		local_irq_save(flags);
		cif_isp10_stop_mi(dev, true, false);
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

static int cif_isp10_stream_stop_mp(struct rkisp1_device *dev)
{
	int ret = 0;
	unsigned long flags = 0;

	if (!dev->strm_vdevs.mi_config.async_updt) {
		local_irq_save(flags);
		cif_isp10_stop_mi(dev, false, true);
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

int cif_isp10_streamon(struct rkisp1_device *dev, u32 stream_ids)
{
	int ret = 0;
	bool streamon_sp = stream_ids & RKISP1_STREAM_SP;
	bool streamon_mp = stream_ids & RKISP1_STREAM_MP;
	struct cif_isp10_vdev_node *node;

	v4l2_info(&dev->v4l2_dev,
		  "state: SP(%d) MP(%d), streamon: SP(%d) MP(%d)\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state, streamon_sp, streamon_mp);

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
		cif_isp10_stream_stop_sp(dev);
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
		cif_isp10_stream_stop_mp(dev);
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

int cif_isp10_streamoff(struct rkisp1_device *dev, u32 stream_ids)
{
	int ret = 0;
	bool streamoff_sp = stream_ids & RKISP1_STREAM_SP;
	bool streamoff_mp = stream_ids & RKISP1_STREAM_MP;
	struct cif_isp10_vdev_node *node;

	v4l2_info(&dev->v4l2_dev,
		  "state: SP(%d) MP(%d), streamoff: SP(%d) MP(%d)\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state, streamoff_sp, streamoff_mp);

	/* TODO power off flash. Flash subdev is not ready yet */

	/* pipeline stream off  */
	if (streamoff_sp) {
		cif_isp10_stream_stop_sp(dev);
		node = &dev->strm_vdevs.sp_vdev.vnode;
		ret = (node->pipe.set_stream) (&node->pipe, false);
		if (ret < 0)
			return ret;
	}

	if (streamoff_mp) {
		cif_isp10_stream_stop_mp(dev);
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
		dev->isp_sdev.isp_config.ism_config.ism_en = 0;
	}

	return 0;
}

static int cif_isp10_v4l2_streamon(struct file *file,
				   void *priv, enum v4l2_buf_type buf_type)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct cif_isp10_vdev_node *node = vdev_to_node(vdev);
	struct rkisp1_device *dev = video_get_drvdata(vdev);
	struct vb2_queue *queue = to_vb2_queue(file);
	u32 stream_ids = file_to_stream_id(file);

	ret = media_entity_pipeline_start(&node->vdev.entity, &node->pipe.pipe);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "start pipeline  failed %d\n", ret);
		return ret;
	}

	ret = cif_isp10_check_format(node);
	if (ret < 0) {
		/* TODO: just warning now */
		v4l2_warn(&dev->v4l2_dev, "check video format failed\n");
	}

	ret = node->pipe.open(&node->pipe, &node->vdev.entity, true);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "open cif pipeline failed %d\n", ret);
		return ret;
	}

	ret = cif_isp10_streamon(dev, stream_ids);
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

static int cif_isp10_v4l2_streamoff(struct file *file,
				    void *priv, enum v4l2_buf_type buf_type)
{
	int ret = 0;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct video_device *vdev = video_devdata(file);
	struct cif_isp10_vdev_node *node = vdev_to_node(vdev);

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

static int cif_fmt_calc_plane_size(const struct cif_fmt_info *fmt,
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

static void cif_fmt_to_v4l2_mplane_fmt(struct cif_frm_fmt *strm_fmt,
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
		cif_fmt_calc_plane_size(&strm_fmt->frm_fmt, pix->width,
					pix->height, i, &plane_fmt->sizeimage,
					&plane_fmt->bytesperline);
	}

}

static int cif_isp10_v4l2_s_fmt_mplane(struct file *file,
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
	ret = cif_isp10_s_fmt(dev, file_to_stream_id(file), &strm_fmt);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	cif_fmt_to_v4l2_mplane_fmt(&strm_fmt, &f->fmt.pix_mp);

	return 0;
}

static int cif_isp10_v4l2_g_fmt_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct cif_isp10_vdev_node *node = vdev_to_node(vdev);
	struct cif_isp10_stream *strm = node_to_stream(node);
	struct cif_frm_fmt *strm_fmt = &strm->path_cfg.output;

	cif_fmt_to_v4l2_mplane_fmt(strm_fmt, &f->fmt.pix_mp);

	return 0;
}

static int cif_isp10_v4l2_try_fmt_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct cif_frm_fmt strm_fmt;
	const struct cif_fmt_info *cif_fmt =
	    cif_isp10_find_fmt(file_to_stream_id(file),
			       &f->fmt.pix_mp.pixelformat,
			       -1);

	strm_fmt.frm_fmt = *cif_fmt;
	/* TODO: do more checks on resolution */
	strm_fmt.width = f->fmt.pix_mp.width;
	strm_fmt.height = f->fmt.pix_mp.height;
	/* TODO: quantization */
	strm_fmt.quantization = 0;
	cif_fmt_to_v4l2_mplane_fmt(&strm_fmt, &f->fmt.pix_mp);

	return 0;
}

static int cif_isp10_v4l2_vb2_queue_setup(struct vb2_queue *queue,
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
	enum cif_isp10_stream_id id = queue_to_stream_id(queue);
	const struct cif_fmt_info *cif_fmt = NULL;
	struct cif_isp10_stream *stream = to_stream_by_id(dev, id);
	u32 imagsize = 0;

	if (pfmt) {
		pixm = &pfmt->fmt.pix_mp;
		cif_fmt = cif_isp10_find_fmt(id, &pixm->pixelformat, -1);
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
			cif_fmt_calc_plane_size(cif_fmt, pixm->width,
						pixm->height, i,
						&imagsize, NULL);
		} else {
			struct cif_frm_fmt *strm_fmt = &stream->path_cfg.output;

			cif_fmt_calc_plane_size(cif_fmt, strm_fmt->width,
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

static void cif_isp10_v4l2_vb2_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cif_isp10_buffer *ispbuf = to_cif_isp10_vb(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct rkisp1_device *dev = queue->drv_priv;
	enum cif_isp10_stream_id id = queue_to_stream_id(queue);
	struct cif_isp10_stream *stream = to_stream_by_id(dev, id);
	unsigned long lock_flags = 0;

	v4l2_info(&dev->v4l2_dev,
		  "buffer type %s\n", v4l2_type_names[queue->type]);

	spin_lock_irqsave(&dev->strm_vdevs.vbq_lock, lock_flags);
	list_add_tail(&ispbuf->queue, &stream->buf_queue);
	spin_unlock_irqrestore(&dev->strm_vdevs.vbq_lock, lock_flags);
}

static void cif_isp10_v4l2_vb2_stop_streaming(struct vb2_queue *queue)
{
	struct cif_isp10_vdev_node *node;
	enum cif_isp10_stream_id id = queue_to_stream_id(queue);
	struct cif_isp10_stream *stream = NULL;
	struct rkisp1_device *dev;
	struct cif_isp10_buffer *buf, *buf_tmp;
	unsigned long lock_flags = 0;

	node = queue_to_node(queue);

	dev = video_get_drvdata(&node->vdev);

	stream = to_stream_by_id(dev, id);

	if (cif_isp10_streamoff(dev, id))
		return;

	spin_lock_irqsave(&dev->strm_vdevs.vbq_lock, lock_flags);

	if (stream->curr_buf) {
		vb2_buffer_done(&stream->curr_buf->vb.vb2_buf,
				VB2_BUF_STATE_ERROR);
		stream->curr_buf = NULL;
	}
	if (stream->next_buf) {
		vb2_buffer_done(&stream->next_buf->vb.vb2_buf,
				VB2_BUF_STATE_ERROR);
		stream->next_buf = NULL;
	}

	list_for_each_entry_safe(buf, buf_tmp, &stream->buf_queue, queue) {
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&dev->strm_vdevs.vbq_lock, lock_flags);
}

static void cif_isp10_mi_update_buff_addr(struct rkisp1_device *dev,
					  enum cif_isp10_stream_id strm_id)
{
	if (strm_id == RKISP1_STREAM_SP) {
		cif_iowrite32_verify(dev->strm_vdevs.sp_vdev.path_cfg.
				     next_buff_addr,
				     dev->config.base_addr +
				     CIF_MI_SP_Y_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(dev->strm_vdevs.sp_vdev.path_cfg.
				     next_buff_addr_cb,
				     dev->config.base_addr +
				     CIF_MI_SP_CB_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(dev->strm_vdevs.sp_vdev.path_cfg.
				     next_buff_addr_cr,
				     dev->config.base_addr +
				     CIF_MI_SP_CR_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		/*
		 * There have bee repeatedly issues with
		 * the offset registers, it is safer to write
		 * them each time, even though it is always
		 * 0 and even though that is the
		 * register's default value
		 */
		cif_iowrite32_verify(0,
				     dev->config.base_addr +
				     CIF_MI_SP_Y_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(0,
				     dev->config.base_addr +
				     CIF_MI_SP_CB_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(0,
				     dev->config.base_addr +
				     CIF_MI_SP_CR_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	} else if (strm_id == RKISP1_STREAM_MP) {
		cif_iowrite32_verify(dev->strm_vdevs.mp_vdev.path_cfg.
				     next_buff_addr,
				     dev->config.base_addr +
				     CIF_MI_MP_Y_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(dev->strm_vdevs.mp_vdev.path_cfg.
				     next_buff_addr_cb,
				     dev->config.base_addr +
				     CIF_MI_MP_CB_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(dev->strm_vdevs.mp_vdev.path_cfg.
				     next_buff_addr_cr,
				     dev->config.base_addr +
				     CIF_MI_MP_CR_BASE_AD_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		/*
		 * There have bee repeatedly issues with
		 * the offset registers, it is safer to write
		 * them each time, even though it is always
		 * 0 and even though that is the
		 * register's default value
		 */
		cif_iowrite32_verify(0,
				     dev->config.base_addr +
				     CIF_MI_MP_Y_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(0,
				     dev->config.base_addr +
				     CIF_MI_MP_CB_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
		cif_iowrite32_verify(0,
				     dev->config.base_addr +
				     CIF_MI_MP_CR_OFFS_CNT_INIT,
				     CIF_MI_ADDR_SIZE_ALIGN_MASK);
	}
}

static int cif_isp10_update_mi_mp(struct rkisp1_device *dev)
{
	struct cif_frm_fmt *out_frm_fmt =
	    &dev->strm_vdevs.mp_vdev.path_cfg.output;
	struct cif_fmt_info *cif_fmt = &out_frm_fmt->frm_fmt;
	u32 d;
	void *a;

	if (dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr !=
	    dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr) {
		if (dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr ==
		    RKISP1_INVALID_BUFF_ADDR) {
			/* disable MI MP */
			v4l2_info(&dev->v4l2_dev, "disabling MP MI\n");
			d = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN | CIF_MI_CTRL_RAW_ENABLE);
			a = dev->config.base_addr + CIF_MI_CTRL;
			cif_iowrite32_verify(d  & ioread32(a), a, ~0);
		} else if (dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr ==
			   RKISP1_INVALID_BUFF_ADDR) {
			/* re-enable MI MP */
			v4l2_info(&dev->v4l2_dev, "enabling MP MI\n");
			iowrite32(CIF_MI_MP_FRAME,
				  dev->config.base_addr + CIF_MI_ICR);
			d = (u32)~(CIF_MI_CTRL_MP_ENABLE_IN | CIF_MI_CTRL_RAW_ENABLE);
			a = dev->config.base_addr + CIF_MI_CTRL;
			cif_iowrite32_verify(d  & ioread32(a), a, ~0);
			if (CIF_PIX_FMT_IS_RAW_BAYER(cif_fmt)) {
				d = CIF_MI_CTRL_RAW_ENABLE;
				a = dev->config.base_addr + CIF_MI_CTRL;
				cif_iowrite32_verify(d | ioread32(a), a, ~0);
			} else if (CIF_PIX_FMT_IS_YUV(cif_fmt)) {
				d = CIF_MI_CTRL_MP_ENABLE_IN;
				a = dev->config.base_addr + CIF_MI_CTRL;
				cif_iowrite32_verify(d | ioread32(a), a, ~0);
			}
		}
		cif_isp10_mi_update_buff_addr(dev, RKISP1_STREAM_MP);
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr =
		    dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr;
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr_cb =
		    dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr_cb;
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr_cr =
		    dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr_cr;
	}

	return 0;
}

static int cif_isp10_update_mi_sp(struct rkisp1_device *dev)
{
	u32 d;
	void *a;

	v4l2_info(&dev->v4l2_dev,
		  "curr 0x%08x next 0x%08x\n",
		  dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr,
		  dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr);
	if (dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr !=
	    dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr) {
		if (dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr ==
		    RKISP1_INVALID_BUFF_ADDR) {
			/* disable MI SP */
			v4l2_info(&dev->v4l2_dev, "disabling SP MI\n");
			/* 'switch off' MI interface */
			d = (u32)~CIF_MI_CTRL_SP_ENABLE;
			a = dev->config.base_addr + CIF_MI_CTRL;
			cif_iowrite32_verify(d & ioread32(a), a, ~0);
		} else if (dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr ==
			   RKISP1_INVALID_BUFF_ADDR) {
			/* re-enable MI SP */
			v4l2_info(&dev->v4l2_dev, "enabling SP MI\n");
			iowrite32(CIF_MI_SP_FRAME,
				  dev->config.base_addr + CIF_MI_ICR);
			d = CIF_MI_CTRL_SP_ENABLE;
			a = dev->config.base_addr + CIF_MI_CTRL;
			cif_iowrite32_verify(d | ioread32(a), a, ~0);
		}
		cif_isp10_mi_update_buff_addr(dev, RKISP1_STREAM_SP);
		dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr =
		    dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr;
		dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr_cb =
		    dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cb;
		dev->strm_vdevs.sp_vdev.path_cfg.curr_buff_addr_cr =
		    dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cr;
	}

	return 0;
}

static int cif_isp10_mi_frame_end(struct rkisp1_device *dev,
				  enum cif_isp10_stream_id stream_id)
{
	struct cif_isp10_stream *stream = NULL;
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
		update_mi = cif_isp10_update_mi_mp;
	} else {
		stream = &dev->strm_vdevs.sp_vdev;
		y_base_addr = dev->config.base_addr + CIF_MI_SP_Y_BASE_AD_SHD;
		next_buff_addr =
		    &dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr;
		next_buff_addr_cb =
		    &dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cb;
		next_buff_addr_cr =
		    &dev->strm_vdevs.sp_vdev.path_cfg.next_buff_addr_cr;
		update_mi = cif_isp10_update_mi_sp;
	}

	fmt = &stream->path_cfg.output;
	cif_fmt = &fmt->frm_fmt;

	if (!stream->next_buf && stream_id == RKISP1_STREAM_MP) {
		stream->stall = dev->config.out_of_buffer_stall;
	} else if ((stream->next_buf) &&
		   (vb2_dma_contig_plane_dma_addr
		    (&stream->next_buf->vb.vb2_buf, 0)
		    != ioread32(y_base_addr))) {
		v4l2_warn(&dev->v4l2_dev,
			  "stream id: %d buffer queue is not advancing "
			  "(0x%08x/0x%08x)\n",
			  stream_id, (stream_id & RKISP1_STREAM_MP) ?
			  ioread32(dev->config.base_addr +
				   CIF_MI_MP_Y_BASE_AD_INIT)
			  : ioread32(dev->config.base_addr +
				     CIF_MI_SP_Y_BASE_AD_INIT),
			  ioread32(y_base_addr));
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
				cif_fmt_calc_plane_size(cif_fmt, fmt->width,
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
					     struct cif_isp10_buffer, queue);
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

static void cif_isp10_start_mi(struct rkisp1_device *dev,
			       bool start_mi_sp, bool start_mi_mp)
{
	u32 d;
	void *a;

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
		cif_isp10_mi_frame_end(dev, RKISP1_STREAM_SP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		dev->strm_vdevs.sp_vdev.stall = false;
	}

	if (start_mi_mp) {
		dev->strm_vdevs.mp_vdev.path_cfg.next_buff_addr =
		    RKISP1_INVALID_BUFF_ADDR;
		dev->strm_vdevs.mp_vdev.path_cfg.curr_buff_addr =
		    RKISP1_INVALID_BUFF_ADDR;
		spin_lock(&dev->strm_vdevs.vbq_lock);
		cif_isp10_mi_frame_end(dev, RKISP1_STREAM_MP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		dev->strm_vdevs.mp_vdev.stall = false;
	}

	a = dev->config.base_addr + CIF_MI_INIT;
	iowrite32(ioread32(a) | CIF_MI_INIT_SOFT_UPD,a);

	if (start_mi_sp) {
		spin_lock(&dev->strm_vdevs.vbq_lock);
		cif_isp10_mi_frame_end(dev, RKISP1_STREAM_SP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		if (dev->strm_vdevs.sp_vdev.curr_buf)
			dev->strm_vdevs.sp_vdev.path_cfg.busy = true;
	}

	if (start_mi_mp) {
		spin_lock(&dev->strm_vdevs.vbq_lock);
		cif_isp10_mi_frame_end(dev, RKISP1_STREAM_MP);
		spin_unlock(&dev->strm_vdevs.vbq_lock);
		if (dev->strm_vdevs.mp_vdev.curr_buf)
			dev->strm_vdevs.mp_vdev.path_cfg.busy = true;
	}

	if (!dev->strm_vdevs.mi_config.async_updt){
		d = CIF_ISP_CTRL_ISP_GEN_CFG_UPD;
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) | d, a);
	}
}

static int
cif_isp10_v4l2_vb2_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	int ret = 0;
	struct cif_isp10_vdev_node *node;
	enum cif_isp10_stream_id id = queue_to_stream_id(queue);
	struct cif_isp10_stream *stream = NULL;
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

		cif_isp10_start_mi(dev, true, false);
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

		cif_isp10_start_mi(dev, false, true);
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

static struct vb2_ops cif_isp10_v4l2_vb2_ops = {
	.queue_setup = cif_isp10_v4l2_vb2_queue_setup,
	.buf_queue = cif_isp10_v4l2_vb2_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.stop_streaming = cif_isp10_v4l2_vb2_stop_streaming,
	.start_streaming = cif_isp10_v4l2_vb2_start_streaming,
};

static int cif_isp10_init_vb2_queue(struct vb2_queue *q,
				    struct rkisp1_device *dev,
				    enum v4l2_buf_type buf_type)
{
	struct cif_isp10_vdev_node *node;

	memset(q, 0, sizeof(*q));
	node = queue_to_node(q);
	mutex_init(&node->qlock);

	q->type = buf_type;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = dev;
	q->ops = &cif_isp10_v4l2_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct cif_isp10_buffer);
	q->min_buffers_needed = 4;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &node->qlock;

	return vb2_queue_init(q);
}

static void cifisp_isp10_video_init_default_fmt(struct cif_isp10_stream *stream)
{
	struct cif_frm_fmt strm_fmt;
	struct rkisp1_device *dev = stream_to_dev(stream);
	const u32 def_fmt = V4L2_PIX_FMT_YUYV;

	strm_fmt.width = STREAM_PATH_DEFAULT_WIDTH;
	strm_fmt.height = STREAM_PATH_DEFAULT_HEIGHT;
	strm_fmt.quantization = 0;
	strm_fmt.frm_fmt = *cif_isp10_find_fmt(stream->id, &def_fmt, -1);
	cif_isp10_s_fmt(dev, stream->id, &strm_fmt);
}

static void cif_isp10_init_stream(struct rkisp1_device *dev,
				  enum cif_isp10_stream_id stream_id)
{
	struct cif_isp10_stream *stream = NULL;

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

	cifisp_isp10_video_init_default_fmt(stream);
}

int cif_isp10_stream_init(struct rkisp1_device *dev, u32 stream_ids)
{
	v4l2_info(&dev->v4l2_dev, "0x%08x\n", stream_ids);

	if (stream_ids & ~(RKISP1_ALL_STREAMS)) {
		v4l2_err(&dev->v4l2_dev,
			 "unknown/unsupported stream IDs 0x%08x\n", stream_ids);
		return -EINVAL;
	}

	/* set default input, failure is not fatal here */
	if ((dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_DISABLED) &&
	    (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_DISABLED)) {
		dev->isp_sdev.isp_config.si_enable = false;
		dev->isp_sdev.isp_config.ie_config.effect = RKISP1_IE_NONE;
	}

	if (stream_ids & RKISP1_STREAM_SP)
		cif_isp10_init_stream(dev, RKISP1_STREAM_SP);
	if (stream_ids & RKISP1_STREAM_MP)
		cif_isp10_init_stream(dev, RKISP1_STREAM_MP);

	return 0;
}

int cif_isp10_stream_release(struct rkisp1_device *dev, int stream_ids)
{
	int ret;
	struct cif_isp10_stream *strm_dev;
	struct cif_isp10_vdev_node *node;

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
			cif_isp10_stream_stop_sp(dev);
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
			cif_isp10_stream_stop_mp(dev);
			node = &strm_dev->vnode;

			ret = (node->pipe.set_stream) (&node->pipe, false);
			if (ret < 0)
				return ret;
		}
		strm_dev->state = RKISP1_STATE_DISABLED;
	}

	return 0;
}

/* fops **********************************************************************/
static int cif_isp10_v4l2_open(struct file *file)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct rkisp1_device *dev = video_get_drvdata(vdev);

	v4l2_info(&dev->v4l2_dev, "video device video%d.%d (%s)\n",
		  vdev->num, vdev->minor, vdev->name);

	ret = v4l2_fh_open(file);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "v4l2_fh_open failed %d\n", ret);
		return ret;
	}

	/* Already initialized */
	if (!v4l2_fh_is_singular_file(file))
		return 0;

	/* First open of the device, so initialize everything */
	ret = cif_isp10_stream_init(dev, file_to_stream_id(file));
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		v4l2_fh_release(file);
		return ret;
	}

	return 0;
}

static int cif_isp10_v4l2_release(struct file *file)
{
	int ret = 0;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct rkisp1_device *dev = queue->drv_priv;
	enum cif_isp10_stream_id stream_id = file_to_stream_id(file);

	v4l2_info(&dev->v4l2_dev, "buf type: %d\n", queue->type);

	if (v4l2_fh_is_singular_file(file)) {
		/* Last close, so uninitialize hardware */
		ret = cif_isp10_stream_release(dev, stream_id);
		if (ret < 0)
			v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
	}

	vb2_fop_release(file);

	return ret;
}

static const struct v4l2_file_operations cif_isp10_v4l2_fops = {
	.open = cif_isp10_v4l2_open,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
	.release = cif_isp10_v4l2_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int cif_isp10_v4l2_querycap(struct file *file,
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

static int cif_isp10_v4l2_subscribe_event(struct v4l2_fh *fh,
					  const struct v4l2_event_subscription
					  *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

void cif_isp10_v4l2_event(struct rkisp1_device *dev, __u32 frame_sequence)
{
	struct v4l2_event ev = {
		.type = V4L2_EVENT_FRAME_SYNC,
		.u.frame_sync.frame_sequence = frame_sequence,
	};

	v4l2_event_queue(&dev->strm_vdevs.sp_vdev.vnode.vdev, &ev);
}

/* TODO remove RK_VIDIOC_SENSOR_MODE_DATA, revise later */

static const char *cif_isp10_g_input_name(struct rkisp1_device *dev,
					  unsigned int input_index)
{
	if (input_index != 0) {
		v4l2_err(&dev->v4l2_dev, "index %d out of bounds\n",
			 input_index);
		return ERR_PTR(-EINVAL);
	}

	return dev_driver_string(dev->subdevs[RKISP1_SD_SENSOR]->dev);
}

/* TODO: support only one input now */

static int cif_isp10_v4l2_enum_input(struct file *file, void *priv,
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

	inp_name = cif_isp10_g_input_name(dev, input->index);
	if (IS_ERR(inp_name))
		return PTR_ERR(inp_name);

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_UNKNOWN;
	strncpy(input->name, inp_name, sizeof(input->name) - 1);

	return 0;
}

static int cif_isp10_v4l2_enum_fmt_cap_mplane(struct file *file, void *priv,
					      struct v4l2_fmtdesc *f)
{
	struct vb2_queue *queue = to_vb2_queue(file);
	struct rkisp1_device *dev = queue->drv_priv;
	u32 stream_id = file_to_stream_id(file);
	const struct cif_fmt_info *cif_fmt = NULL, *check_fmt;
	int i = f->index, fmt_index = 0;

	do {
		cif_fmt = cif_isp10_find_fmt(stream_id, NULL, i);
		if (!cif_fmt) {
			v4l2_err(&dev->v4l2_dev, "index %d\n", f->index);
			return -EINVAL;
		}

		/* check whether this format is supported by specific stream
		  * and skip the queried formats.
		  */
		check_fmt = cif_isp10_find_fmt(stream_id, &cif_fmt->fourcc, -1);
		if ((check_fmt == cif_fmt) && (fmt_index++ == f->index)) {
			f->pixelformat = cif_fmt->fourcc;
			return 0;
		}
	} while (++i < ARRAY_SIZE(cif_isp10_output_formats));

	return -EINVAL;
}

static const struct v4l2_ioctl_ops cif_isp10_v4l2_sp_ioctlops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = cif_isp10_v4l2_streamon,
	.vidioc_streamoff = cif_isp10_v4l2_streamoff,
	.vidioc_enum_input = cif_isp10_v4l2_enum_input,
	/* TODO: use ctrl handler instead
	   .vidioc_s_ctrl = cif_isp10_v4l2_s_ctrl, */
	.vidioc_try_fmt_vid_cap_mplane = cif_isp10_v4l2_try_fmt_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = cif_isp10_v4l2_enum_fmt_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = cif_isp10_v4l2_s_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane = cif_isp10_v4l2_g_fmt_mplane,
	.vidioc_querycap = cif_isp10_v4l2_querycap,
	.vidioc_subscribe_event = cif_isp10_v4l2_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_ioctl_ops cif_isp10_v4l2_mp_ioctlops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = cif_isp10_v4l2_streamon,
	.vidioc_streamoff = cif_isp10_v4l2_streamoff,
	.vidioc_enum_input = cif_isp10_v4l2_enum_input,
	/* TODO: use ctrl handler instead
	   .vidioc_g_ctrl = mainpath_g_ctrl,
	   .vidioc_s_ctrl = cif_isp10_v4l2_s_ctrl, */
	.vidioc_try_fmt_vid_cap_mplane = cif_isp10_v4l2_try_fmt_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = cif_isp10_v4l2_enum_fmt_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = cif_isp10_v4l2_s_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane = cif_isp10_v4l2_g_fmt_mplane,
	.vidioc_querycap = cif_isp10_v4l2_querycap,
};

static int
cif_isp10_v4l2_register_video_device(struct rkisp1_device *dev,
				     struct video_device *vdev,
				     const char *name, int qtype,
				     int major,
				     const struct v4l2_file_operations *fops,
				     const struct v4l2_ioctl_ops *ioctl_ops)
{
	int ret;
	struct cif_isp10_vdev_node *node = vdev_to_node(vdev);

	node->pipe.open = isp_pipeline_open;
	node->pipe.close = isp_pipeline_close;
	node->pipe.set_stream = isp_pipeline_set_stream;

	vdev->release = video_device_release_empty;
	strlcpy(vdev->name, name, sizeof(vdev->name));
	vdev->fops = fops;
	video_set_drvdata(vdev, dev);
	vdev->minor = -1;
	vdev->ioctl_ops = ioctl_ops;
	vdev->v4l2_dev = &dev->v4l2_dev;
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

	v4l2_info(&dev->v4l2_dev,
		  "video device video%d.%d (%s) successfully registered\n",
		  major, vdev->minor, name);

	dev->alloc_ctx = vb2_dma_contig_init_ctx(dev->dev);
	cif_isp10_init_vb2_queue(&node->buf_queue, dev,
				 V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	vdev->queue = &node->buf_queue;

	return 0;
}

int cif_isp10_register_videodev(struct rkisp1_device *dev,
				enum cif_isp10_stream_id stream_id)
{
	if (stream_id == RKISP1_STREAM_SP)
		return cif_isp10_v4l2_register_video_device(
						dev,
						&dev->
						strm_vdevs.sp_vdev.
						vnode.vdev,
						SP_VDEV_NAME,
						V4L2_CAP_VIDEO_CAPTURE,
						RKISP1_V4L2_SP_DEV_MAJOR,
						&cif_isp10_v4l2_fops,
						&cif_isp10_v4l2_sp_ioctlops);
	else
		return cif_isp10_v4l2_register_video_device(
						dev,
						&dev->
						strm_vdevs.mp_vdev.
						vnode.vdev,
						MP_VDEV_NAME,
						V4L2_CAP_VIDEO_CAPTURE,
						RKISP1_V4L2_MP_DEV_MAJOR,
						&cif_isp10_v4l2_fops,
						&cif_isp10_v4l2_mp_ioctlops);
}

void cif_isp10_unregister_videodev(struct rkisp1_device *dev,
				   enum cif_isp10_stream_id stream_id)
{
	if (stream_id == RKISP1_STREAM_SP) {
		media_entity_cleanup(&dev->strm_vdevs.sp_vdev.vnode.vdev.
				     entity);
		video_unregister_device(&dev->strm_vdevs.sp_vdev.vnode.vdev);
	} else {
		media_entity_cleanup(&dev->strm_vdevs.mp_vdev.vnode.vdev.
				     entity);
		video_unregister_device(&dev->strm_vdevs.mp_vdev.vnode.vdev);
	}
}

void cif_isp10_mi_isr(unsigned int mi_mis, void *cntxt)
{
	struct rkisp1_device *dev = cntxt;
	unsigned int mi_mis_tmp, d;
	void *a;

	if (mi_mis & CIF_MI_SP_FRAME) {
		dev->strm_vdevs.sp_vdev.path_cfg.busy = false;
		iowrite32(CIF_MI_SP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		mi_mis_tmp = ioread32(dev->config.base_addr + CIF_MI_MIS);
		if (mi_mis_tmp & CIF_MI_SP_FRAME)
			v4l2_err(&dev->v4l2_dev, "sp icr err: 0x%x\n",
				 mi_mis_tmp);
	}

	if (mi_mis & CIF_MI_MP_FRAME) {
		dev->strm_vdevs.mp_vdev.path_cfg.busy = false;
		iowrite32(CIF_MI_MP_FRAME, dev->config.base_addr + CIF_MI_ICR);
		mi_mis_tmp = ioread32(dev->config.base_addr + CIF_MI_MIS);
		if (mi_mis_tmp & CIF_MI_MP_FRAME)
			v4l2_err(&dev->v4l2_dev, "mp icr err: 0x%x\n",
				 mi_mis_tmp);
	}

	if (!RKISP1_MI_IS_BUSY(dev)) {
		if (dev->strm_vdevs.mi_config.async_updt) {
			u32 mp_y_off_cnt_shd =
			    ioread32(dev->config.base_addr +
				     CIF_MI_MP_Y_OFFS_CNT_SHD);
			u32 sp_y_off_cnt_shd =
			    ioread32(dev->config.base_addr +
				     CIF_MI_SP_Y_OFFS_CNT_SHD);

			iowrite32(CIF_MI_INIT_SOFT_UPD,
				  dev->config.base_addr + CIF_MI_INIT);
			if (!dev->isp_sdev.isp_config.ism_config.ism_en &&
			    (dev->strm_vdevs.mi_config.async_updt &
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
			cif_isp10_stop_mi(dev, false, true);
			dev->strm_vdevs.mp_vdev.state = RKISP1_STATE_READY;
			dev->strm_vdevs.mp_vdev.stop = false;

			/* Turn off MRSZ since it is not needed */
			iowrite32(0, dev->config.base_addr + CIF_MRSZ_CTRL);
			d = CIF_RSZ_CTRL_CFG_UPD;
			a = dev->config.base_addr + CIF_MRSZ_CTRL;
			iowrite32(ioread32(a) | d, a);
			v4l2_info(&dev->v4l2_dev, "MP has stopped\n");
			wake_up_interruptible(&dev->strm_vdevs.mp_vdev.done);
		}
		if (dev->strm_vdevs.sp_vdev.stop &&
		    (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING)) {
			cif_isp10_stop_mi(dev, true, false);
			dev->strm_vdevs.sp_vdev.state = RKISP1_STATE_READY;
			dev->strm_vdevs.sp_vdev.stop = false;

			/* Turn off SRSZ since it is not needed */
			iowrite32(0, dev->config.base_addr + CIF_SRSZ_CTRL);
			d = CIF_RSZ_CTRL_CFG_UPD;
			a = dev->config.base_addr + CIF_SRSZ_CTRL;
			iowrite32((ioread32(a) | (u32)(d)), a);
			v4l2_info(&dev->v4l2_dev, "SP has stopped\n");
			wake_up_interruptible(&dev->strm_vdevs.sp_vdev.done);
		}

		if (dev->strm_vdevs.sp_vdev.state == RKISP1_STATE_STREAMING) {
			spin_lock(&dev->strm_vdevs.vbq_lock);
			(void)cif_isp10_mi_frame_end(dev, RKISP1_STREAM_SP);
			spin_unlock(&dev->strm_vdevs.vbq_lock);
		}
		if (dev->strm_vdevs.mp_vdev.state == RKISP1_STATE_STREAMING) {
			spin_lock(&dev->strm_vdevs.vbq_lock);
			(void)cif_isp10_mi_frame_end(dev, RKISP1_STREAM_MP);
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

	iowrite32((u32) (~(CIF_MI_MP_FRAME | CIF_MI_SP_FRAME)),
		  dev->config.base_addr + CIF_MI_ICR);
}
