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

#include <linux/delay.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include "capture.h"
#include "regs.h"
#include "rkisp1.h"

#define RKISP1_INVALID_BUFF_ADDR ((u32)~0)
#define CIF_ISP_REQ_BUFS_MIN 2
#define CIF_ISP_REQ_BUFS_MAX 8

/*
 * mp sink source: isp
 * sp sink source : isp, no dma now
 * mp sink pad fmts: yuv 422, raw
 * sp sink pad fmts: yuv422( source isp), yuv420......
 * mp source fmts: yuv, raw, no jpeg now
 * sp source fmts: yuv, rgb?
 *
 */

#define STREAM_PAD_SINK				0
#define STREAM_PAD_SOURCE			1

#define STREAM_MAX_MP_RSZ_OUTPUT_WIDTH		4416
#define STREAM_MAX_MP_RSZ_OUTPUT_HEIGHT		3312
#define STREAM_MAX_SP_RSZ_OUTPUT_WIDTH		1920
#define STREAM_MAX_SP_RSZ_OUTPUT_HEIGHT		1080
#define STREAM_MIN_RSZ_OUTPUT_WIDTH		32
#define STREAM_MIN_RSZ_OUTPUT_HEIGHT		16

#define STREAM_MAX_MP_SP_INPUT_WIDTH STREAM_MAX_MP_RSZ_OUTPUT_WIDTH
#define STREAM_MAX_MP_SP_INPUT_HEIGHT STREAM_MAX_MP_RSZ_OUTPUT_HEIGHT
#define STREAM_MIN_MP_SP_INPUT_WIDTH		32
#define STREAM_MIN_MP_SP_INPUT_HEIGHT		32

/*
 * crop only accept yuv422 format
 * resizer can accept yuv444,yuv422,yuv420 format, can output yuv422, yuv420,
 * yuv444 format
 * sp resizer has tow data source: DMA-reader or crop
 * mp resizer has only one data source:  crop
 * if format is unsupported by path, crop and resizer should be bypassed
 * (disabled)
 */

static const struct rkisp1_fmt mp_fmts[] = {
		/* yuv422 */
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_YVYU,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_VYUY,
		.mbus_code = MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_NV16,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
		.input_format = 0,
		.output_format = 0,
	},
		/* yuv420 */
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_1_5X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 12,
		.xsubs = 2,
		.ysubs = 2,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_1_5X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 12,
		.xsubs = 2,
		.ysubs = 2,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
		.input_format = 0,
		.output_format = 0,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU420,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_1_5X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 12,
		.xsubs = 2,
		.ysubs = 2,
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
		.input_format = 0,
		.output_format = 0,
	},
		/* yuv444 */
		/* yuv400 */
	{
		.fourcc = V4L2_PIX_FMT_GREY,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 8,
		.xsubs = 0,
		.ysubs = 0,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
		.input_format = 0,
		.output_format = 0,
	},
		/* raw */
	{
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 8,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 8,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 8,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 8,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 10,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 10,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 10,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 10,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 12,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.mbus_code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 12,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.mbus_code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 12,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_BAYER,
		.bpp = 12,
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, 
};

static const struct rkisp1_fmt sp_fmts[] = {
		/* yuv422 */
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.input_format = MI_CTRL_SP_INPUT_YUV422,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_YVYU,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.input_format = MI_CTRL_SP_INPUT_YUV422,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.input_format = MI_CTRL_SP_INPUT_YUV422,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_VYUY,
		.mbus_code = MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.input_format = MI_CTRL_SP_INPUT_YUV422,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.input_format = MI_CTRL_SP_INPUT_YUV422,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_NV16,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.input_format = MI_CTRL_SP_INPUT_YUV422,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 16,
		.xsubs = 2,
		.ysubs = 4,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.input_format = MI_CTRL_SP_INPUT_YUV422,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	},
		/* yuv420 */
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_1_5X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 12,
		.xsubs = 2,
		.ysubs = 2,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.input_format = MI_CTRL_SP_INPUT_YUV420,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_1_5X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 12,
		.xsubs = 2,
		.ysubs = 2,
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.input_format = MI_CTRL_SP_INPUT_YUV420,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU420,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_1_5X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 12,
		.xsubs = 2,
		.ysubs = 2,
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.input_format = MI_CTRL_SP_INPUT_YUV420,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	},
		/* yuv400 */
	{
		.fourcc = V4L2_PIX_FMT_GREY,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt_type = FMT_YUV,
		.bpp = 8,
		.xsubs = 0,
		.ysubs = 0,
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.input_format = MI_CTRL_SP_INPUT_YUV400,
		.output_format = MI_CTRL_SP_OUTPUT_YUV400,
	},
		/* rgb */
	{
		.fourcc = V4L2_PIX_FMT_RGB24,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_RGB,
		.bpp = 24,
		.mplanes = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_RGB888,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB565,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_RGB,
		.bpp = 16,
		.mplanes = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_RGB565,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR666,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.fmt_type = FMT_RGB,
		.bpp = 18,
		.mplanes = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_RGB666,
	}, 
};

static const
struct rkisp1_fmt *find_fmt(const struct rkisp1_fmt *fmt_array,
				int array_size, const u32 pixelfmt, int index)
{
	int i;
	const struct rkisp1_fmt *fmt;;

	for (i = 0; i < array_size; i++) {
		fmt = &fmt_array[i];
		if (fmt->fourcc == pixelfmt)
			return fmt;
		if (index == i)
			return fmt;
	}
	return 0;
}

static struct
v4l2_subdev *rkisp1_remote_subdev(struct rkisp1_vdev_node *video, u32 *pad)
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

static int
get_format(struct rkisp1_stream *stream,
		       struct cif_frm_fmt *cifmt)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret, i;

	subdev = rkisp1_remote_subdev(&stream->vnode, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);

	if (ret < 0)
		return ret;

	cifmt->mbus = fmt.format;
	for (i = 0; i < stream->fmt_size; ++i) {
		/* TODO: should do more checks */
		if (stream->fmts[i].mbus_code == cifmt->mbus.code)
			break;
	}

	if (i == stream->fmt_size)
		return -EPERM;

	cifmt->fmt = &(stream->fmts[i]);
	return 0;
}

static int rkisp1_set_fmt(struct rkisp1_stream *stream,
				struct cif_frm_fmt *strm_fmt)
{
	struct rkisp1_device *dev = stream->ispdev;
	const struct rkisp1_fmt *cif_fmt;

	cif_fmt = find_fmt(stream->fmts, stream->fmt_size,
					strm_fmt->fmt->fourcc, -1);
	strm_fmt->fmt = cif_fmt;

	switch (stream->id) {
	case RKISP1_STREAM_SP:
		/* TBD: more detailed check whether format is a valid format for SP */
		/* TBD: remove the mode stuff */
		if ((cif_fmt->fmt_type != FMT_YUV) && (cif_fmt->fmt_type != FMT_RGB)) {
			v4l2_warn(&dev->v4l2_dev,
				"format %c%c%c%c %dx%d, not supported on SP, "
				"set to the first supported format\n",
				(strm_fmt->fmt->fourcc & 0xff),
				(strm_fmt->fmt->fourcc >> 8) & 0xff,
				(strm_fmt->fmt->fourcc >> 16) & 0xff,
				(strm_fmt->fmt->fourcc >> 24) & 0xff,
				strm_fmt->mbus.width, strm_fmt->mbus.height);
			cif_fmt = stream->fmts;
		}
		break;
	case RKISP1_STREAM_MP:
		if (cif_fmt->fmt_type == FMT_BAYER) {
			if ((stream->state == RKISP1_STATE_READY) ||
				(stream->state == RKISP1_STATE_STREAMING))
				v4l2_warn(&dev->v4l2_dev,
					"cannot output RAW data "
					"when SP is active, you "
					"will not be able to "
					"(re-)start streaming\n");
			stream->raw_enable = true;
		}
	}

	stream->path_cfg.output = *strm_fmt;
	stream->path_cfg.llength = strm_fmt->mbus.width;
	return stream->ops->calc_size(stream);
}

static int fmt_calc_plane_size(const struct rkisp1_fmt *fmt,
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
			image_size = max(image_size, height * (bytesperline * fmt->ysubs /4));
		} else {
			bytesperline = (fmt->xsubs / 4) * width;
			image_size = max(image_size, height * (bytesperline * fmt->ysubs /4));
		}
	} else {
		if (cplanes > 1)
			bytesperline = width;
		if ((cplanes <= 1) && (bytesperline == 0 || (bytesperline * 8) / fmt->bpp < width))
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

	pix->colorspace = strm_fmt->fmt->colorspace;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = strm_fmt->fmt->mplanes;
	pix->pixelformat = strm_fmt->fmt->fourcc;
	pix->height = strm_fmt->mbus.height;
	pix->width = strm_fmt->mbus.width;

	for (i = 0; i < pix->num_planes; i++) {
		plane_fmt = &pix->plane_fmt[i];
		fmt_calc_plane_size(strm_fmt->fmt, pix->width,
					pix->height, i, &plane_fmt->sizeimage,
					&plane_fmt->bytesperline);
	}
}

static int rkisp1_config_dcrop(struct rkisp1_stream *stream,
						bool async)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	struct v4l2_rect *dcrop = &stream->dcrop;
	struct cif_frm_fmt input_fmt;

	if (get_format(stream, &input_fmt))
		return -EINVAL;

	if (dcrop->width == input_fmt.mbus.width &&
	    dcrop->height == input_fmt.mbus.height &&
	    dcrop->left == 0 && dcrop->top == 0) {
		stream->ops->disable_dcrop(base, async);
		return 0;
	}
	stream->ops->config_dcrop(base, dcrop, async);
	return 0;
}

static int rkisp1_config_rsz(struct rkisp1_stream *stream,
							bool async)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem	*base = dev->config.base_addr;
	struct cif_frm_fmt input_fmt, output_fmt;
	struct rkisp1_win in_y, in_c, out_y,  out_c;

	if (get_format(stream, &input_fmt))
		return -EINVAL;
	output_fmt = stream->path_cfg.output;

	/* Convert input size to cropped size */
	if (stream->dcrop.width != input_fmt.mbus.width ||
	    stream->dcrop.height != input_fmt.mbus.height ||
	    stream->dcrop.left != 0 || stream->dcrop.top != 0) {
		input_fmt.mbus.width = stream->dcrop.width;
		input_fmt.mbus.height = stream->dcrop.height;
	}

	if ((input_fmt.mbus.width == output_fmt.mbus.width &&
	     input_fmt.mbus.height == output_fmt.mbus.height) ||
	    (input_fmt.fmt->fmt_type != FMT_YUV)) {
		stream->ops->disable_rsz(base, async);
		return 0;
	}

	stream->ops->set_phase(base);
	stream->ops->set_lut(base);

	/* set RSZ input and output */
	v4l2_info(&dev->v4l2_dev,
		  "stream: %d fmt: %08x %dx%d -> fmt: %08x %dx%d\n",
		  stream->id, input_fmt.fmt->mbus_code,
		  input_fmt.mbus.width, input_fmt.mbus.height,
		  output_fmt.fmt->mbus_code, output_fmt.mbus.width,
		  output_fmt.mbus.height);

	/* set input and output sizes for scale calculation */
	in_c.w = in_y.w = input_fmt.mbus.width;
	in_c.h = in_y.h = input_fmt.mbus.height;
	out_c.w = out_y.w = output_fmt.mbus.width;
	out_c.h = out_y.h = output_fmt.mbus.height;

	if (input_fmt.fmt->fmt_type == FMT_YUV) {
		in_c.w = in_y.w * input_fmt.fmt->xsubs / 4;
		in_c.h = in_y.h * input_fmt.fmt->ysubs / 4;
		out_c.w = out_y.w * input_fmt.fmt->xsubs / 4;
		out_c.h = out_y.h * input_fmt.fmt->ysubs / 4;

		v4l2_info(&dev->v4l2_dev, "chroma scaling %dx%d -> %dx%d\n",
			  in_c.w, in_c.h, out_c.w, out_c.h);

		if (((in_c.w == 0) && (out_c.w > 0)) ||
			((in_c.h == 0) && (out_c.h > 0))) {
			v4l2_err(&dev->v4l2_dev,
			"Invalid convert from monochrome to chromatic\n");
			return -EINVAL;
		}
	} else {
		if ((in_y.w != out_y.w) || (in_y.w != out_y.h)) {
			v4l2_err(&dev->v4l2_dev,
			"%dx%d -> %dx%d isn't support,only scale YUV input\n", 
			in_y.w, in_y.w, out_y.w, out_y.h);
			return -EINVAL;
		}
	}

	/* calculate and set scale */
	stream->ops->set_scale(base, &in_y, &in_c, &out_y, &out_c);
	if (async)
		stream->ops->update_shadow_reg(base);
	stream->ops->dump_rsz_regs(base);
	return 0;
}

static void rkisp1_calc_yuv_size(struct rkisp1_stream *stream)
{
	struct cif_frm_fmt *out_frm_fmt = &stream->path_cfg.output;
	const struct rkisp1_fmt *out_cif_fmt = out_frm_fmt->fmt;
	u32 llength = stream->path_cfg.llength;
	u32 height = out_frm_fmt->mbus.height;
	u32 bpp = out_cif_fmt->bpp;
	u32 size = llength * height * bpp / 8;
	u32 num_cplanes = out_cif_fmt->cplanes;

	stream->path_cfg.y_size = size;
	stream->path_cfg.cb_size = 0;
	stream->path_cfg.cr_size = 0;
	if(num_cplanes == 1)
		return;
	stream->path_cfg.y_size = (size * 4) /(4 + out_cif_fmt->xsubs * out_cif_fmt->ysubs / 2);
	stream->path_cfg.cb_size = size - stream->path_cfg.y_size;
	/* for U<->V swapping: */
	stream->path_cfg.cr_size = stream->path_cfg.cb_size;
	if (num_cplanes == 3)
		stream->path_cfg.cb_size /= 2;
}

/***************************** stream operations*******************************/
static int mp_config_mi(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	struct cif_frm_fmt *out_frm_fmt = &stream->path_cfg.output;
	const struct rkisp1_fmt *cif_fmt = out_frm_fmt->fmt;

	mp_mi_set_y_size(base, stream->path_cfg.y_size);
	mp_mi_set_cb_size(base, stream->path_cfg.cb_size);
	mp_mi_set_cr_size(base, stream->path_cfg.cr_size);
	mp_frame_end_int_enable(base);
	if (cif_fmt->uv_swap)
		mp_set_uv_swap(base);

	mp_mi_ctrl_set_format(base, cif_fmt->write_format);
	mi_ctrl_set_lum_burst(base);
	mi_ctrl_set_chrom_burst(base);
	mi_ctrl_init_base_en(base);
	mi_ctrl_init_offset_en(base);
	mp_mi_ctrl_autoupdate_en(base);
	return 0;
}

static int sp_config_mi(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	struct cif_frm_fmt *out_frm_fmt = &stream->path_cfg.output;
	const struct rkisp1_fmt *out_cif_fmt = out_frm_fmt->fmt;
	struct cif_frm_fmt in_frm_fmt;

	if (get_format(stream, &in_frm_fmt))
		return -EINVAL;
	sp_mi_set_y_size(base, stream->path_cfg.y_size);
	sp_mi_set_cb_size(base, stream->path_cfg.cb_size);
	sp_mi_set_cr_size(base, stream->path_cfg.cr_size);
	sp_set_y_width(base, out_frm_fmt->mbus.width);
	sp_set_y_height(base, out_frm_fmt->mbus.height);
	sp_set_y_line_length(base, stream->path_cfg.llength);
	sp_frame_end_int_enable(base);
	if (out_cif_fmt->uv_swap)
		sp_set_uv_swap(base);

	sp_mi_ctrl_set_format(base, out_cif_fmt->write_format |
			      in_frm_fmt.fmt->input_format |
			      out_cif_fmt->output_format);
	mi_ctrl_set_lum_burst(base);
	mi_ctrl_set_chrom_burst(base);
	mi_ctrl_init_base_en(base);
	mi_ctrl_init_offset_en(base);
	sp_mi_ctrl_autoupdate_en(base);
	return 0;
}

static struct rkisp1_stream * mp_get_other_stream(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;

	return &dev->sp_stream;
}

static struct rkisp1_stream * sp_get_other_stream(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	
	return &dev->mp_stream;
}

static int mp_check_against(struct rkisp1_stream *stream)
{
	struct rkisp1_stream *sp = mp_get_other_stream( stream);
	

	if (sp->state == RKISP1_STATE_STREAMING)
		return -EAGAIN;
	else
		return 0;
}

static int sp_check_against(struct rkisp1_stream *stream)
{
	struct rkisp1_stream *mp = sp_get_other_stream( stream);
	struct v4l2_device *v4l2_dev = &stream->ispdev->v4l2_dev;

	if ( mp->raw_enable && mp->state == RKISP1_STATE_STREAMING) {
		v4l2_err(v4l2_dev,
			 "cannot start streaming on SP path when MP is active "
			 "and set to RAW output\n");
		return -EBUSY;
	}

	if (mp->state == RKISP1_STATE_STREAMING)
		return -EAGAIN;
	else
		return 0;
}

void mp_stream_init(struct rkisp1_stream *stream)
{
	stream->raw_enable = false;
	stream->fmts = mp_fmts;
	stream->fmt_size = ARRAY_SIZE(mp_fmts);
}

void sp_stream_init(struct rkisp1_stream *stream)
{
	stream->fmts = sp_fmts;
	stream->fmt_size = ARRAY_SIZE(sp_fmts);
}

static int mp_update_mi(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	struct cif_frm_fmt *out_frm_fmt = &stream->path_cfg.output;
	const struct rkisp1_fmt *cif_fmt = out_frm_fmt->fmt;
	u32 val;
	void *addr;

	if (stream->path_cfg.next_buff_addr != stream->path_cfg.curr_buff_addr) {
		if (stream->path_cfg.next_buff_addr ==
		    RKISP1_INVALID_BUFF_ADDR) {
			/* disable MI MP */
			v4l2_info(&dev->v4l2_dev, "disabling MP MI\n");
			mi_ctrl_mp_disable(base);
		} else if (stream->path_cfg.curr_buff_addr ==
			   RKISP1_INVALID_BUFF_ADDR) {
			/* re-enable MI MP */
			v4l2_info(&dev->v4l2_dev, "enabling MP MI\n");
			stream->ops->clr_frame_end_int(base);
			mi_ctrl_mp_disable(base);
			if (cif_fmt->fmt_type == FMT_BAYER) 
				mi_ctrl_mpraw_enable(base);
			else if (cif_fmt->fmt_type == FMT_YUV)
				mi_ctrl_mpyuv_enable(base);
		}
		val = stream->path_cfg.next_buff_addr;
		addr = base + CIF_MI_MP_Y_BASE_AD_INIT;
		writel(val, addr);
		val = stream->path_cfg.next_buff_addr_cb;
		addr = base + CIF_MI_MP_CB_BASE_AD_INIT;
		writel(val, addr);
		val = stream->path_cfg.next_buff_addr_cr;
		addr = base + CIF_MI_MP_CR_BASE_AD_INIT;
		writel(val, addr);
		/*
		 * There have bee repeatedly issues with
		 * the offset registers, it is safer to write
		 * them each time, even though it is always
		 * 0 and even though that is the
		 * register's default value
		 */
		addr = base + CIF_MI_MP_Y_OFFS_CNT_INIT;
		writel(0, addr);
		addr = base + CIF_MI_MP_CB_OFFS_CNT_INIT;
		writel(0, addr);
		addr = base + CIF_MI_MP_CR_OFFS_CNT_INIT;
		writel(0, addr);
		stream->path_cfg.curr_buff_addr =
		    stream->path_cfg.next_buff_addr;
		stream->path_cfg.curr_buff_addr_cb =
		    stream->path_cfg.next_buff_addr_cb;
		stream->path_cfg.curr_buff_addr_cr =
		    stream->path_cfg.next_buff_addr_cr;
	}

	return 0;
}

static int sp_update_mi(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	u32 val;
	void *addr;

	if (stream->path_cfg.next_buff_addr !=
	    stream->path_cfg.curr_buff_addr) {
		if (stream->path_cfg.next_buff_addr ==
		    RKISP1_INVALID_BUFF_ADDR) {
			/* disable MI SP */
			mi_ctrl_spyuv_disable(base);
		} else if (stream->path_cfg.curr_buff_addr ==
			   RKISP1_INVALID_BUFF_ADDR) {
			/* re-enable MI SP */
			stream->ops->clr_frame_end_int(base);
			mi_ctrl_spyuv_enable(base);
		}
		val = stream->path_cfg.next_buff_addr;
		addr = base + CIF_MI_SP_Y_BASE_AD_INIT;
		writel(val, addr);
		val = stream->path_cfg.next_buff_addr_cb;
		addr = base + CIF_MI_SP_CB_BASE_AD_INIT;
		writel(val, addr);
		val = stream->path_cfg.next_buff_addr_cr;
		addr = base + CIF_MI_SP_CR_BASE_AD_INIT;
		writel(val, addr);
		/*
		 * There have bee repeatedly issues with
		 * the offset registers, it is safer to write
		 * them each time, even though it is always
		 * 0 and even though that is the
		 * register's default value
		 */
		addr = base + CIF_MI_SP_Y_OFFS_CNT_INIT;
		writel(0, addr);
		addr = base + CIF_MI_SP_CB_OFFS_CNT_INIT;
		writel(0, addr);
		addr = base + CIF_MI_SP_CR_OFFS_CNT_INIT;
		writel(0, addr);
		stream->path_cfg.curr_buff_addr =
		    stream->path_cfg.next_buff_addr;
		stream->path_cfg.curr_buff_addr_cb =
		    stream->path_cfg.next_buff_addr_cb;
		stream->path_cfg.curr_buff_addr_cr =
		    stream->path_cfg.next_buff_addr_cr;
	}

	return 0;
}

static int mp_stop_mi(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;

	if (stream->state != RKISP1_STATE_STREAMING)
		return -EINVAL;
	//mp_frame_end_int_disable(base);
	stream->ops->clr_frame_end_int(base);
	mi_ctrl_mp_disable(base);
	return 0;
}

static int sp_stop_mi(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;

	if (stream->state != RKISP1_STATE_STREAMING)
		return -EINVAL;
	//sp_frame_end_int_disable(base);
	stream->ops->clr_frame_end_int(base);
	mi_ctrl_spyuv_disable(base);
	return 0;
}

static int mp_set_data_path(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	
	mp_set_chain_mode(base);
	/*we don't support jpeg,  force to mi*/
	mp_set_mux(base);
	return 0;
}

static int sp_set_data_path(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	
	sp_set_chain_mode(base);
	return 0;
}

static int mp_calc_size(struct rkisp1_stream *stream)
{
	struct cif_frm_fmt *out_frm_fmt = &stream->path_cfg.output;
	const struct rkisp1_fmt *fmt = out_frm_fmt->fmt;
	u32 width = out_frm_fmt->mbus.width;
	u32 height = out_frm_fmt->mbus.height;

	if (fmt->fmt_type == FMT_YUV){
		rkisp1_calc_yuv_size( stream);
		return 0;
	}
	if (fmt->fmt_type != FMT_BAYER)
		return -EINVAL;

	if (fmt->bpp > 8)
		stream->path_cfg.y_size = width * height * 2;
	else
		stream->path_cfg.y_size = width * height;
	stream->path_cfg.cb_size = 0;
	stream->path_cfg.cr_size = 0;
	return 0;
}

static int sp_calc_size(struct rkisp1_stream *stream)
{
	struct cif_frm_fmt *out_frm_fmt = &stream->path_cfg.output;
	const struct rkisp1_fmt *out_cif_fmt = out_frm_fmt->fmt;
	u32 height = out_frm_fmt->mbus.height;
	u32 bpp = out_cif_fmt->bpp;

	stream->path_cfg.y_size = stream->path_cfg.llength * height * bpp / 8;
	stream->path_cfg.cb_size = 0;
	stream->path_cfg.cr_size = 0;
	if (out_cif_fmt->fmt_type != FMT_YUV)
		return 0;
	rkisp1_calc_yuv_size( stream);
	return 0;
}

static struct streams_ops rkisp1_mp_streams_ops = {
	.stream_init = mp_stream_init,
	.check_against = mp_check_against,
	.config_mi = mp_config_mi,
	.update_mi = mp_update_mi,
	.stop_mi = mp_stop_mi,
	.set_data_path = mp_set_data_path,
	.calc_size = mp_calc_size,
	.config_dcrop = mp_config_dcrop,
	.disable_dcrop = mp_disable_dcrop,
	.disable_rsz = mp_disable_rsz,
	.set_phase = mp_set_phase,
	.set_lut = mp_set_lut,
	.set_scale = mp_set_scale,
	.update_shadow_reg = mp_update_shadow_reg,
	.dump_rsz_regs = mp_dump_rsz_regs,
	.clr_frame_end_int = mp_clr_frame_end_int,
	.is_frame_end_int_masked = mp_is_frame_end_int_masked,
	.get_y_offset_counter_shd = mp_get_y_offset_counter_shd,
	.get_other_stream = mp_get_other_stream,
};

static struct streams_ops rkisp1_sp_streams_ops = {
	.stream_init = sp_stream_init,
	.check_against = sp_check_against,
	.config_mi = sp_config_mi,
	.update_mi = sp_update_mi,
	.stop_mi = sp_stop_mi,
	.set_data_path = sp_set_data_path,
	.calc_size = sp_calc_size,
	.config_dcrop = sp_config_dcrop,
	.disable_dcrop = sp_disable_dcrop,
	.disable_rsz = sp_disable_rsz,
	.set_phase = sp_set_phase,
	.set_lut = sp_set_lut,
	.set_scale = sp_set_scale,
	.update_shadow_reg = sp_update_shadow_reg,
	.dump_rsz_regs = sp_dump_rsz_regs,
	.clr_frame_end_int = sp_clr_frame_end_int,
	.is_frame_end_int_masked = sp_is_frame_end_int_masked,
	.get_y_offset_counter_shd = sp_get_y_offset_counter_shd,
	.get_other_stream = sp_get_other_stream,
};
#if 0
static int rkisp1_stop_mi(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;

	if (stream->state != RKISP1_STATE_STREAMING)
		return -EINVAL;
	frame_end_int_disable(base);
	clr_mpsp_frame_end_int(base);
	mi_ctrl_spyuv_disable(base);
	mi_ctrl_mp_disable(base);
	force_cfg_update(base);
	return 0;
}
#endif
static int rkisp1_stream_stop(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	int ret = 0;

	stream->stop = true;
	ret = wait_event_timeout(stream->done,
		stream->state != RKISP1_STATE_STREAMING, msecs_to_jiffies(1000));
	if (ret < 0) {
		v4l2_warn(v4l2_dev, "waiting on event return error %d\n", ret);
		return ret;
	}
	return 0;
}

static int mi_frame_end(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	u32 *next_buff_addr = NULL, *next_buff_addr_cb, *next_buff_addr_cr;
	int i = 0;
	struct cif_frm_fmt *fmt;
	struct rkisp1_fmt *cif_fmt;
	void __iomem *y_base_addr;

	if (stream->id == RKISP1_STREAM_MP) {
		y_base_addr = base + CIF_MI_MP_Y_BASE_AD_SHD;
	} else {
		y_base_addr = base + CIF_MI_SP_Y_BASE_AD_SHD;
	}

	next_buff_addr = &stream->path_cfg.next_buff_addr;
	next_buff_addr_cb = &stream->path_cfg.next_buff_addr_cb;
	next_buff_addr_cr = &stream->path_cfg.next_buff_addr_cr;

	fmt = &stream->path_cfg.output;
	cif_fmt = (struct rkisp1_fmt*)fmt->fmt;

	if (!stream->next_buf && stream->id == RKISP1_STREAM_MP) {
		stream->stall = dev->config.out_of_buffer_stall;
	} else if ((stream->next_buf) &&
		   (vb2_dma_contig_plane_dma_addr
		    (&stream->next_buf->vb.vb2_buf, 0)
		    != readl(y_base_addr))) {
		v4l2_warn(&dev->v4l2_dev,
			  "stream id: %d buffer queue is not advancing "
			  "(0x%08x/0x%08x)\n",
			  stream->id, (stream->id & RKISP1_STREAM_MP) ?
			  readl(base +
				   CIF_MI_MP_Y_BASE_AD_INIT)
			  : readl(base +
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
				fmt_calc_plane_size(cif_fmt, fmt->mbus.width,
							fmt->mbus.height, i,
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

		/* for contiguous buffer */
		if (cif_fmt->mplanes == 1 && cif_fmt->cplanes > 1) {
			*next_buff_addr_cb = *next_buff_addr + stream->path_cfg.y_size;
			*next_buff_addr_cr = *next_buff_addr_cb;
			if (cif_fmt->cplanes > 2) {
				/* swap uv */
				if (cif_fmt->uv_swap) {
					*next_buff_addr_cr = *next_buff_addr +
						stream->path_cfg.y_size;
					*next_buff_addr_cb = *next_buff_addr_cr +
						stream->path_cfg.cr_size;
				} else {
					*next_buff_addr_cr = *next_buff_addr_cb +
						stream->path_cfg.cb_size;
				}
			}
		}
	}
	stream->ops->update_mi(stream);

	stream->stall = false;
	return 0;
}

/***************************** vb2 operations*******************************/
static int rkisp1_start(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	int ret;
	
	ret = stream->ops->set_data_path(stream);
	if (ret < 0)
		return ret;
	
	ret = stream->ops->config_mi(stream);
	if (ret < 0)
		return ret;
	
	stream->path_cfg.next_buff_addr = RKISP1_INVALID_BUFF_ADDR;
	stream->path_cfg.curr_buff_addr = RKISP1_INVALID_BUFF_ADDR;
	spin_lock(&stream->vbq_lock);
	mi_frame_end(stream);
	spin_unlock(&stream->vbq_lock);
	stream->stall = false;
	force_cfg_update(base);
	spin_lock(&stream->vbq_lock);
	mi_frame_end(stream);
	spin_unlock(&stream->vbq_lock);
	stream->state = RKISP1_STATE_STREAMING;
	return 0;
}

static int rkisp1_restart(struct rkisp1_stream *stream)
{
	struct rkisp1_vdev_node *node = &stream->vnode;
	int ret;

	ret = rkisp1_stream_stop(stream);
	if (ret < 0)
		return ret;

	
	ret = node->pipe.set_stream(&node->pipe, false);
	if (ret < 0)
		return ret;
	
	ret = rkisp1_start(stream);
	if (ret < 0)
		return ret;
	
	return 0;
}

static int rkisp1_queue_setup(struct vb2_queue *queue,
					  const void *parg,
					  unsigned int *num_buffers,
					  unsigned int *num_planes,
					  unsigned int sizes[],
					  void *alloc_ctxs[])
{
	struct rkisp1_stream *stream = queue->drv_priv;
	struct rkisp1_device *dev = stream->ispdev;
	const struct v4l2_format *pfmt = parg;
	const struct v4l2_pix_format_mplane *pixm = NULL;
	const struct rkisp1_fmt *cif_fmt = NULL;
	u32 i, imagsize = 0;

	if (pfmt) {
		pixm = &pfmt->fmt.pix_mp;
		cif_fmt = find_fmt(stream->fmts, stream->fmt_size,
				   pixm->pixelformat, -1);
	} else {
		cif_fmt = stream->path_cfg.output.fmt;
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

			fmt_calc_plane_size(cif_fmt, strm_fmt->mbus.width,
						strm_fmt->mbus.height, i,
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
	struct rkisp1_buffer *ispbuf = to_rkisp1_buffer(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct rkisp1_stream *stream = queue->drv_priv;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	list_add_tail(&ispbuf->queue, &stream->buf_queue);
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
}

static void rkisp1_stop_streaming(struct vb2_queue *queue)
{
	struct rkisp1_stream *stream = queue->drv_priv;
	struct v4l2_device *v4l2_dev = &stream->ispdev->v4l2_dev;
	struct rkisp1_vdev_node *node = &stream->vnode;
	struct rkisp1_buffer *buf;
	unsigned long lock_flags = 0;
	int ret, i = 0;

	if (stream->state != RKISP1_STATE_STREAMING)
		return;

	rkisp1_stream_stop(stream);
	ret = node->pipe.set_stream (&node->pipe, false);
	if (ret < 0)
		return;

	stream->state = RKISP1_STATE_READY;
	stream->raw_enable = false;
	stream->path_cfg.output.mbus.width = 0;
	stream->path_cfg.output.mbus.height = 0;
	/*TODO: maybe set in isp_params.c*/
	/*dev->isp_sdev.isp_dev.input_width = 0;
	dev->isp_sdev.isp_dev.input_height = 0;*/

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	buf = stream->curr_buf;
	stream->curr_buf = NULL;
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
	if (buf)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	buf = stream->next_buf;
	stream->next_buf = NULL;
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
	if (buf)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	for (i = 0; i < CIF_ISP_REQ_BUFS_MAX; i++) {
		spin_lock_irqsave(&stream->vbq_lock, lock_flags);
		if (list_empty(&stream->buf_queue)) {
			spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
			break;
		}
		buf = list_first_entry(&stream->buf_queue, struct rkisp1_buffer, queue);
		list_del(&buf->queue);
		spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
		if (buf)
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	ret = node->pipe.close(&node->pipe);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "pipeline close failed with error %d\n", ret);
		return;
	}

	media_entity_pipeline_stop(&node->vdev.entity);
}

static int
rkisp1_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct rkisp1_stream *stream = queue->drv_priv;
	struct rkisp1_vdev_node *node = &stream->vnode;
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &stream->ispdev->v4l2_dev;
	struct cif_frm_fmt *video_fmt = &stream->path_cfg.output;
	struct cif_frm_fmt remote_fmt;
	int ret = 0;

	ret = media_entity_pipeline_start(&node->vdev.entity, &node->pipe.pipe);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "start pipeline failed %d\n", ret);
		return ret;
	}

	ret = get_format(stream, &remote_fmt);
	if (ret < 0)
		return ret;

	if (video_fmt->fmt->fourcc != remote_fmt.fmt->fourcc ||
	    video_fmt->mbus.height != remote_fmt.mbus.height ||
	    video_fmt->mbus.width != remote_fmt.mbus.width){
		v4l2_warn(v4l2_dev, "check video format failed\n");
	}

	if (stream->dcrop.width > remote_fmt.mbus.width ||
	    stream->dcrop.height > remote_fmt.mbus.height ||
	    stream->dcrop.height == 0 || stream->dcrop.width == 0 ){
		v4l2_warn(v4l2_dev, "validate crop values failed\n");
		stream->dcrop.width = remote_fmt.mbus.width;
		stream->dcrop.height = remote_fmt.mbus.height;
	}

	ret = node->pipe.open(&node->pipe, &node->vdev.entity, true);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "open cif pipeline failed %d\n", ret);
		return ret;
	}

	if (stream->state == RKISP1_STATE_STREAMING)
		return 0;

	if (stream->state != RKISP1_STATE_READY) {
		v4l2_err(v4l2_dev, "stream not enabled\n");
		return -EBUSY;
	}

	ret = stream->ops->check_against(stream);
	if (ret == (-EAGAIN)) {
		struct rkisp1_stream *other;
		other = stream->ops->get_other_stream(stream);
		if(other)
			rkisp1_restart(other);
	} else if(ret == (-EBUSY))
		return -EBUSY;

	ret = rkisp1_start(stream);
	if (ret < 0)
		return ret;
	
	ret = rkisp1_config_rsz(stream, true);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	ret = rkisp1_config_dcrop(stream, true);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	/* pipeline stream on */
	ret = node->pipe.set_stream (&node->pipe, true);
	if (ret < 0)
		return ret;
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
				    struct rkisp1_stream *stream,
				    enum v4l2_buf_type buf_type)
{
	struct rkisp1_vdev_node *node;

	memset(q, 0, sizeof(*q));
	node = queue_to_node(q);

	q->type = buf_type;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = stream;
	q->ops = &rkisp1_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct rkisp1_buffer);
	q->min_buffers_needed = 4;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	return vb2_queue_init(q);
}

/***************************** v4l2_file_operations*******************************/
int rkisp1_stream_init(struct rkisp1_stream *stream, u32 id)
{
	struct rkisp1_device *dev = stream->ispdev;
	struct cif_frm_fmt strm_fmt;

	INIT_LIST_HEAD(&stream->buf_queue);
	stream->next_buf = NULL;
	stream->curr_buf = NULL;
	stream->stop = false;
	stream->stall = false;
	stream->id = id;
	stream->ispdev = dev;
	if (stream->id == RKISP1_STREAM_SP){
		stream->ops = &rkisp1_sp_streams_ops;
	}else{
		stream->ops =& rkisp1_mp_streams_ops;
	}
	stream->ops->stream_init(stream);
	strm_fmt.mbus.width = RKISP1_DEFAULT_WIDTH;
	strm_fmt.mbus.height = RKISP1_DEFAULT_HEIGHT;
	strm_fmt.quantization = 0;
	strm_fmt.fmt = find_fmt(stream->fmts, stream->fmt_size,
		V4L2_PIX_FMT_YUYV, -1);
	stream->state = RKISP1_STATE_READY;
	rkisp1_set_fmt(stream, &strm_fmt);
	stream->dcrop.left = 0;
	stream->dcrop.top = 0;
	stream->dcrop.width = RKISP1_DEFAULT_WIDTH;
	stream->dcrop.height = RKISP1_DEFAULT_HEIGHT;

	return 0;
}

int rkisp1_stream_release(struct rkisp1_stream *stream)
{
	struct rkisp1_vdev_node *node = &stream->vnode;
	int ret;

	if (stream->state == RKISP1_STATE_DISABLED)
		return 0;

	if (stream->state == RKISP1_STATE_STREAMING) {
		rkisp1_stream_stop(stream);
		ret = node->pipe.set_stream (&node->pipe, false);
		if (ret < 0)
			return ret;
	}
	stream->state = RKISP1_STATE_DISABLED;
	return 0;
}

static int rkisp1_open(struct file *file)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	int ret;

	ret = v4l2_fh_open(file);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "v4l2_fh_open failed %d\n", ret);
		return ret;
	}

	/* Already initialized */
	if (!v4l2_fh_is_singular_file(file))
		return 0;

	/* First open of the device, so initialize everything */
	ret = rkisp1_stream_init(stream, stream->id);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed with error %d\n", ret);
		v4l2_fh_release(file);
		return ret;
	}

	return 0;
}

static int rkisp1_release(struct file *file)
{
	struct rkisp1_stream *stream = video_drvdata(file);

	if (v4l2_fh_is_singular_file(file))
		rkisp1_stream_release(stream);

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

static int subscribe_event(struct v4l2_fh *fh,
			const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

/*****************************mp and sp v4l2_ioctl_ops*******************************/
static int rkisp1_enum_input(struct file *file, void *priv,
				     struct v4l2_input *input)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct v4l2_device *v4l2_dev = &stream->ispdev->v4l2_dev;
	struct vb2_queue *queue = &stream->vnode.buf_queue;
	struct rkisp1_device *dev = stream->ispdev;
	const char *inp_name;

	if (queue->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		v4l2_err(v4l2_dev, "wrong buffer queue %d\n",
			 queue->type);
		return -EINVAL;
	}

	if (input->index > dev->num_sensors)
		return -EINVAL;

	inp_name = dev_driver_string(dev->subdevs[RKISP1_SD_SENSOR]->dev);
	if (IS_ERR(inp_name))
		return PTR_ERR(inp_name);

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_UNKNOWN;
	strncpy(input->name, inp_name, sizeof(input->name) - 1);

	return 0;
}

static int rkisp1_try_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct cif_frm_fmt strm_fmt;
	const struct rkisp1_fmt *fmt;

	fmt = find_fmt(stream->fmts, stream->fmt_size,
				f->fmt.pix_mp.pixelformat, -1);

	strm_fmt.fmt = fmt;
	/* TODO: do more checks on resolution */
	strm_fmt.mbus.width = f->fmt.pix_mp.width;
	strm_fmt.mbus.height = f->fmt.pix_mp.height;
	/* TODO: quantization */
	strm_fmt.quantization = 0;
	fmt_to_v4l2_mplane_fmt(&strm_fmt, &f->fmt.pix_mp);

	return 0;
}

static int rkisp1_enum_fmt_vid_cap_mplane(struct file *file, void *priv,
					      struct v4l2_fmtdesc *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct v4l2_device *v4l2_dev = &stream->ispdev->v4l2_dev;
	const struct rkisp1_fmt *fmt = NULL, *check_fmt;
	int i, fmt_index = 0;

	for(i = f->index; i < stream->fmt_size; i++){
		fmt = find_fmt(stream->fmts, stream->fmt_size, 0, i);
		if (!fmt) {
			v4l2_err(v4l2_dev, "index %d\n", f->index);
			return -EINVAL;
		}
		check_fmt = find_fmt(stream->fmts, stream->fmt_size, fmt->fourcc, -1);
		if ((check_fmt == fmt) && (fmt_index++ == f->index)) {
			f->pixelformat = fmt->fourcc;
			return 0;
		}
	}
	return -EINVAL;
}

static int rkisp1_s_fmt_vid_cap_mplane(struct file *file,
				       void *priv, struct v4l2_format *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct rkisp1_device *dev = stream->ispdev;
	struct cif_frm_fmt strm_fmt;
	const struct rkisp1_fmt *fmt;
	int ret;

	rkisp1_try_fmt_vid_cap_mplane(file, priv, f);

	fmt = find_fmt(stream->fmts, stream->fmt_size,
			f->fmt.pix_mp.pixelformat, -1);

	strm_fmt.fmt = fmt;
	/* TODO: do more checks on resolution */
	strm_fmt.mbus.width = f->fmt.pix_mp.width;
	strm_fmt.mbus.height = f->fmt.pix_mp.height;
	/* TODO: quantization */
	strm_fmt.quantization = 0;

	ret = rkisp1_set_fmt(stream, &strm_fmt);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	return 0;
}

static int rkisp1_g_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct cif_frm_fmt *strm_fmt = &stream->path_cfg.output;

	fmt_to_v4l2_mplane_fmt(strm_fmt, &f->fmt.pix_mp);
	return 0;
}

static int rkisp1_g_selection(struct file *file, void *prv,
				       struct v4l2_selection *sel)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct cif_frm_fmt input_fmt;
	struct v4l2_rect *dcrop;

	if (get_format(stream, &input_fmt))
		return -EINVAL;
	dcrop = &stream->dcrop;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.width = input_fmt.mbus.width;
		sel->r.height = input_fmt.mbus.height;
		sel->r.left = 0;
		sel->r.top = 0;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r.width = dcrop->width;
		sel->r.height = dcrop->height;
		sel->r.left = dcrop->left;
		sel->r.top = dcrop->top;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rkisp1_s_selection(struct file *file, void *prv,
				       struct v4l2_selection *sel)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct cif_frm_fmt input_fmt;
	struct v4l2_rect *dcrop;

	if (sel->target != V4L2_SEL_TGT_CROP ||
	    sel->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	if (get_format(stream, &input_fmt))
		return -EINVAL;
	dcrop = &stream->dcrop;

	/* check  whether input fmt is raw */
	if (stream->id == RKISP1_STREAM_MP &&
	    (input_fmt.fmt->fmt_type == FMT_BAYER)) {
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = input_fmt.mbus.width;
		sel->r.height = input_fmt.mbus.height;
		return 0;
	}

	if (sel->target == V4L2_SEL_TGT_CROP) {
		/* crop */
		sel->r.left =
		    clamp_t(u32, sel->r.left, 0,
			    input_fmt.mbus.width - STREAM_MIN_MP_SP_INPUT_WIDTH);
		sel->r.top =
		    clamp_t(u32, sel->r.top, 0,
			    input_fmt.mbus.height - STREAM_MIN_MP_SP_INPUT_HEIGHT);
		sel->r.width =
		    clamp_t(u32, sel->r.width,
			    STREAM_MIN_MP_SP_INPUT_WIDTH,
			    input_fmt.mbus.width - sel->r.left);
		sel->r.height =
		    clamp_t(u32, sel->r.height,
			    STREAM_MIN_MP_SP_INPUT_HEIGHT,
			    input_fmt.mbus.height - sel->r.top);
		/* TODO: should deal with align ? */
		dcrop->left = sel->r.left;
		dcrop->top = sel->r.top;
		dcrop->width = sel->r.width;
		dcrop->height = sel->r.height;

		rkisp1_config_dcrop(stream, false);
		/* should propagate to source ? */
	}

	return 0;
}

static int rkisp1_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct device *dev = stream->ispdev->dev;

	strlcpy(cap->driver, dev->driver->name, sizeof(cap->driver));
	strlcpy(cap->card, dev->driver->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
	"platform:%s", dev_name(dev));
	if (stream->id & RKISP1_STREAM_ALL)
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				   V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static const struct v4l2_ioctl_ops sp_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enum_input = rkisp1_enum_input,
	.vidioc_try_fmt_vid_cap_mplane = rkisp1_try_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = rkisp1_enum_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = rkisp1_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = rkisp1_g_fmt_vid_cap_mplane,
	.vidioc_s_selection = rkisp1_s_selection,
	.vidioc_g_selection = rkisp1_g_selection,
	.vidioc_querycap = rkisp1_querycap,
	.vidioc_subscribe_event = subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_ioctl_ops mp_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enum_input = rkisp1_enum_input,
	.vidioc_try_fmt_vid_cap_mplane = rkisp1_try_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = rkisp1_enum_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = rkisp1_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = rkisp1_g_fmt_vid_cap_mplane,
	.vidioc_querycap = rkisp1_querycap,
};

/*
 * pipeline management
 */

static int __isp_pipeline_prepare(struct rkisp1_pipeline *p,
				  struct media_entity *me)
{
	struct v4l2_subdev *sd;
	struct v4l2_subdev *sensor = NULL;
	int i;

	memset(p->subdevs, 0, sizeof(p->subdevs));

	while (1) {
		struct media_pad *pad = NULL;

		/* Find remote source pad */
		for (i = 0; i < me->num_pads; i++) {
			struct media_pad *spad = &me->pads[i];

			if (!(spad->flags & MEDIA_PAD_FL_SINK))
				continue;
			pad = media_entity_remote_pad(spad);
			if (pad)
				break;
		}

		if (pad == NULL ||
		    media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;
		sd = media_entity_to_v4l2_subdev(pad->entity);
		switch (sd->grp_id) {
		case GRP_ID_SENSOR:
			sensor = sd;
			p->subdevs[IDX_SENSOR] = sd;
			break;
		case GRP_ID_MIPIPHY:
			p->subdevs[IDX_MIPIPHY] = sd;
			break;
		case GRP_ID_ISP:
			p->subdevs[IDX_ISP] = sd;
			break;
		default:
			break;
		}
		me = &sd->entity;
		if (me->num_pads == 1)
			break;
	}
	return 0;
}

static int __subdev_set_power(struct v4l2_subdev *sd, int on)
{
	int *use_count;
	int ret;

	v4l2_info(sd, "%d: name %s,on %d\n", __LINE__, sd->name, on);

	if (sd == NULL)
		return -ENXIO;

	use_count = &sd->entity.use_count;
	if (on && (*use_count)++ > 0)
		return 0;
	else if (!on && (*use_count == 0 || --(*use_count) > 0))
		return 0;
	ret = v4l2_subdev_call(sd, core, s_power, on);

	return ret != -ENOIOCTLCMD ? ret : 0;
}

static int isp_pipeline_s_power(struct rkisp1_pipeline *p, bool on)
{
	static const u8 seq[2][IDX_MAX] = {
		{ IDX_ISP, IDX_SENSOR, IDX_MIPIPHY },
		{ IDX_ISP, IDX_SENSOR, IDX_MIPIPHY },
	};
	int i, ret = 0;

	if (p->subdevs[IDX_SENSOR] == NULL)
		return -ENXIO;

	for (i = 0; i < IDX_MAX; i++) {
		unsigned int idx = seq[on][i];

		ret = __subdev_set_power(p->subdevs[idx], on);
		if (ret < 0 && ret != -ENXIO)
			goto error;
	}

	return 0;
error:
	for (; i >= 0; i--) {
		unsigned int idx = seq[on][i];

		__subdev_set_power(p->subdevs[idx], !on);
	}
	return ret;
}

static int isp_pipeline_open(struct rkisp1_pipeline *p, struct media_entity *me,
		      bool prepare)
{
	int ret;
	struct v4l2_subdev *sd;

	if (WARN_ON(p == NULL || me == NULL))
		return -EINVAL;

	if (prepare)
		__isp_pipeline_prepare(p, me);

	sd = p->subdevs[IDX_SENSOR];
	if (sd == NULL)
		return -EINVAL;

	ret = isp_pipeline_s_power(p, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int isp_pipeline_close(struct rkisp1_pipeline *p)
{
	int ret;

	ret = isp_pipeline_s_power(p, 0);

	return ret == -ENXIO ? 0 : ret;
}

static int isp_pipeline_set_stream(struct rkisp1_pipeline *p, bool on)
{
	static const u8 seq[2][IDX_MAX] = {
		{ IDX_MIPIPHY, IDX_SENSOR, IDX_ISP },
		{ IDX_ISP, IDX_MIPIPHY, IDX_SENSOR },
	};
	int i, ret = 0;

	if (p->subdevs[IDX_SENSOR] == NULL)
		return -ENODEV;

	for (i = 0; i < IDX_MAX; i++) {
		unsigned int idx = seq[on][i];

		ret = v4l2_subdev_call(p->subdevs[idx], video, s_stream, on);

		if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
			goto error;
	}

	return 0;
error:
	for (; i >= 0; i--) {
		unsigned int idx = seq[on][i];

		v4l2_subdev_call(p->subdevs[idx], video, s_stream, !on);
	}
	return ret;
}

static void rkisp1_unregister_stream_vdev(struct rkisp1_stream *stream)
{
	media_entity_cleanup(&stream->vnode.vdev.entity);
	video_unregister_device(&stream->vnode.vdev);
}

void rkisp1_unregister_stream_vdevs(struct rkisp1_device *dev)
{
	struct rkisp1_stream *mp_stream = &dev->mp_stream;
	struct rkisp1_stream *sp_stream = &dev->sp_stream;

	rkisp1_unregister_stream_vdev(mp_stream);
	rkisp1_unregister_stream_vdev(sp_stream);
}

static int rkisp1_register_stream_vdev(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct video_device *vdev = &stream->vnode.vdev;
	struct rkisp1_vdev_node *node;
	int ret, qtype;

	if (stream->id == RKISP1_STREAM_SP) {
		strlcpy(vdev->name, SP_VDEV_NAME, sizeof(vdev->name));
		vdev->ioctl_ops = &sp_v4l2_ioctl_ops;
		qtype = V4L2_CAP_VIDEO_CAPTURE;
		stream->fmts = sp_fmts;
		stream->fmt_size = ARRAY_SIZE(sp_fmts);
		stream->ops = &rkisp1_sp_streams_ops;
	} else {
		strlcpy(vdev->name, MP_VDEV_NAME, sizeof(vdev->name));
		vdev->ioctl_ops = &mp_v4l2_ioctl_ops;
		qtype = V4L2_CAP_VIDEO_CAPTURE;
		stream->fmts = mp_fmts;
		stream->fmt_size = ARRAY_SIZE(mp_fmts);
		stream->ops =& rkisp1_mp_streams_ops;
	}
	node = vdev_to_node(vdev);
	node->pipe.open = isp_pipeline_open;
	node->pipe.close = isp_pipeline_close;
	node->pipe.set_stream = isp_pipeline_set_stream;
	mutex_init(&node->vlock);

	vdev->release = video_device_release_empty;
	vdev->fops = &rkisp1_fops;
	vdev->minor = -1;
	vdev->v4l2_dev = v4l2_dev;
	vdev->lock = &node->vlock;
	video_set_drvdata(vdev, stream);
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

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		v4l2_err(v4l2_dev,
			 "video_register_device failed with error %d\n", ret);
		return ret;
	}

	stream->ispdev->alloc_ctx = vb2_dma_contig_init_ctx(v4l2_dev->dev);
	rkisp_init_vb2_queue(&node->buf_queue, stream,
				 V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	vdev->queue = &node->buf_queue;

	return 0;
}

int rkisp1_register_stream_vdevs(struct rkisp1_device *dev)
{
	struct rkisp1_stream *mp_stream = &dev->mp_stream;
	struct rkisp1_stream *sp_stream = &dev->sp_stream;
	int ret;

	dev->mp_stream.ispdev = dev;
	ret = rkisp1_register_stream_vdev(mp_stream);
	if (ret < 0)
		return ret;

	dev->sp_stream.ispdev = dev;
	ret = rkisp1_register_stream_vdev(sp_stream);
	if (ret < 0)
		goto err;

	return 0;
err:
	rkisp1_unregister_stream_vdev(mp_stream);
	return ret;
}

void rkisp1_mi_isr(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = dev->config.base_addr;
	volatile u32 val;

	stream->ops->clr_frame_end_int(base);
	val = stream->ops->is_frame_end_int_masked(base);
	if (val){
		val = mi_get_masked_int_status(base);
		v4l2_err(&dev->v4l2_dev, "icr err: 0x%x\n", val);
	}
	if(stream->state != RKISP1_STATE_STREAMING)
		return;
	
	if (stream->stop) {
		stream->ops->stop_mi(stream);
		stream->ops->disable_dcrop(base, true);
		stream->ops->disable_rsz(base, true);
		stream->stop = false;
		stream->state = RKISP1_STATE_READY;
		v4l2_info(&dev->v4l2_dev, "stream %d has stopped\n", stream->id);
		wake_up(&stream->done);
	}

	spin_lock(&stream->vbq_lock);
	mi_frame_end(stream);
	spin_unlock(&stream->vbq_lock);
}

