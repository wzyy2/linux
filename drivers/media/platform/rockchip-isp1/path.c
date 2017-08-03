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

#include <media/v4l2-subdev.h>
#include "regs.h"
#include "rkisp1.h"
#include "path.h"


/*
 * mp sink source: isp
 * sp sink source : isp, no dma now
 * mp sink pad fmts: yuv 422, raw
 * sp sink pad fmts: yuv422( source isp), yuv420......
 * mp source fmts: yuv, raw, no jpeg now
 * sp source fmts: yuv, rgb?
 *
 */

#define STREAM_PAD_SINK 0
#define STREAM_PAD_SOURCE 1

#define STREAM_MAX_MP_RSZ_OUTPUT_WIDTH (4416)
#define STREAM_MAX_MP_RSZ_OUTPUT_HEIGHT (3312)
#define STREAM_MAX_SP_RSZ_OUTPUT_WIDTH (1920)
#define STREAM_MAX_SP_RSZ_OUTPUT_HEIGHT (1080)
#define STREAM_MIN_RSZ_OUTPUT_WIDTH (32)
#define STREAM_MIN_RSZ_OUTPUT_HEIGHT (16)

#define STREAM_MAX_MP_SP_INPUT_WIDTH STREAM_MAX_MP_RSZ_OUTPUT_WIDTH
#define STREAM_MAX_MP_SP_INPUT_HEIGHT STREAM_MAX_MP_RSZ_OUTPUT_HEIGHT
#define STREAM_MIN_MP_SP_INPUT_WIDTH (32)
#define STREAM_MIN_MP_SP_INPUT_HEIGHT (32)

#ifndef DIV_TRUNCATE
#define DIV_TRUNCATE(x, y) ((x) / (y))
#endif

/*
 * crop only accept yuv422 format
 * resizer can accept yuv444,yuv422,yuv420 format, can output yuv422, yuv420,
 * yuv444 format
 * sp resizer has tow data source: DMA-reader or crop
 * mp resizer has only one data source:  crop
 * if format is unsupported by path, crop and resizer should be bypassed
 * (disabled)
 */

#define CIF_PATH_MAIN    0
#define CIF_PATH_SELF    1
#define CIF_PATH_NUM     2
#define CIF_PATH_IN      0
#define CIF_PATH_OUT     1
#define CIF_PATH_PAD_NUM 2
#define CIF_FMT_NUM_MAX  25

static const u32
cifisp_path_mbus_codes[CIF_PATH_PAD_NUM][CIF_PATH_NUM][CIF_FMT_NUM_MAX] = {
	/* input */
	[CIF_PATH_IN] = {
			 /* mpath */
			 [CIF_PATH_MAIN] = {
					    MEDIA_BUS_FMT_YUYV8_1X16,
					    MEDIA_BUS_FMT_UYVY8_1X16,
					    MEDIA_BUS_FMT_VYUY8_1X16,
					    MEDIA_BUS_FMT_YVYU8_1X16,
					    MEDIA_BUS_FMT_SBGGR8_1X8,
					    MEDIA_BUS_FMT_SGBRG8_1X8,
					    MEDIA_BUS_FMT_SGRBG8_1X8,
					    MEDIA_BUS_FMT_SRGGB8_1X8,
					    MEDIA_BUS_FMT_SBGGR10_1X10,
					    MEDIA_BUS_FMT_SGBRG10_1X10,
					    MEDIA_BUS_FMT_SGRBG10_1X10,
					    MEDIA_BUS_FMT_SRGGB10_1X10,
					    MEDIA_BUS_FMT_SBGGR12_1X12,
					    MEDIA_BUS_FMT_SGBRG12_1X12,
					    MEDIA_BUS_FMT_SGRBG12_1X12,
					    MEDIA_BUS_FMT_SRGGB12_1X12,
					    /* for array search stop */
					    0},
			 /* spath */
			 [CIF_PATH_SELF] = {
					    /* from isp */
					    MEDIA_BUS_FMT_YUYV8_1X16,
					    MEDIA_BUS_FMT_UYVY8_1X16,
					    MEDIA_BUS_FMT_VYUY8_1X16,
					    MEDIA_BUS_FMT_YVYU8_1X16,
					    /* from dma-reader */
					    MEDIA_BUS_FMT_YUYV8_2X8,
					    MEDIA_BUS_FMT_UYVY8_2X8,
					    MEDIA_BUS_FMT_VYUY8_2X8,
					    MEDIA_BUS_FMT_YVYU8_2X8,
					    MEDIA_BUS_FMT_YUYV8_1_5X8,
					    MEDIA_BUS_FMT_UYVY8_1_5X8,
					    MEDIA_BUS_FMT_VYUY8_1_5X8,
					    MEDIA_BUS_FMT_YVYU8_1_5X8,
					    /* TODO: YUV444 ? */
					    /* for array search stop */
					    0}
			 },
	/* output */
	[CIF_PATH_OUT] = {
			  /* mpath */
			  [CIF_PATH_MAIN] = {
					     MEDIA_BUS_FMT_YUYV8_2X8,
					     MEDIA_BUS_FMT_UYVY8_2X8,
					     MEDIA_BUS_FMT_VYUY8_2X8,
					     MEDIA_BUS_FMT_YVYU8_2X8,
					     MEDIA_BUS_FMT_YUYV8_1_5X8,
					     MEDIA_BUS_FMT_UYVY8_1_5X8,
					     MEDIA_BUS_FMT_VYUY8_1_5X8,
					     MEDIA_BUS_FMT_YVYU8_1_5X8,
					     /* TODO: YUV444 ? */
					     MEDIA_BUS_FMT_SBGGR8_1X8,
					     MEDIA_BUS_FMT_SGBRG8_1X8,
					     MEDIA_BUS_FMT_SGRBG8_1X8,
					     MEDIA_BUS_FMT_SRGGB8_1X8,
					     MEDIA_BUS_FMT_SBGGR10_1X10,
					     MEDIA_BUS_FMT_SGBRG10_1X10,
					     MEDIA_BUS_FMT_SGRBG10_1X10,
					     MEDIA_BUS_FMT_SRGGB10_1X10,
					     MEDIA_BUS_FMT_SBGGR12_1X12,
					     MEDIA_BUS_FMT_SGBRG12_1X12,
					     MEDIA_BUS_FMT_SGRBG12_1X12,
					     MEDIA_BUS_FMT_SRGGB12_1X12,
					     /* for array search stop */
					     0},
			  /* spath */
			  [CIF_PATH_SELF] = {
					     MEDIA_BUS_FMT_YUYV8_2X8,
					     MEDIA_BUS_FMT_UYVY8_2X8,
					     MEDIA_BUS_FMT_VYUY8_2X8,
					     MEDIA_BUS_FMT_YVYU8_2X8,
					     MEDIA_BUS_FMT_YUYV8_1_5X8,
					     MEDIA_BUS_FMT_UYVY8_1_5X8,
					     MEDIA_BUS_FMT_VYUY8_1_5X8,
					     MEDIA_BUS_FMT_YVYU8_1_5X8,
					     /* TODO: YUV444 ? */
					     /* for array search stop */
					     0}
			  }
};

static struct rkisp1_stream_subdev *sd_to_pathdev(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct rkisp1_stream_subdev, subdev);
}

static inline
struct rkisp1_device *v4l2dev_to_ispdev(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct rkisp1_device, v4l2_dev);
}

/*
 * return 0 means format not found
 */

static u32 cifisp_stream_find_format(enum rkisp1_stream_id id, const u32 pad,
				     const u32 *code, int index)
{
	u32 bus_code, def_bus_code = 0;
	const u32 *array_bus_code;
	unsigned int i, array_size;

	if ((id != RKISP1_STREAM_SP) && (id != RKISP1_STREAM_MP))
		return 0;

	if ((pad != STREAM_PAD_SINK) && (pad != STREAM_PAD_SOURCE))
		return 0;

	if (id == RKISP1_STREAM_MP) {
		if (pad == STREAM_PAD_SINK)
			array_bus_code = cifisp_path_mbus_codes[CIF_PATH_IN]
			    [CIF_PATH_MAIN];
		else
			array_bus_code = cifisp_path_mbus_codes[CIF_PATH_OUT]
			    [CIF_PATH_MAIN];
	} else {
		if (pad == STREAM_PAD_SINK)
			array_bus_code = cifisp_path_mbus_codes[CIF_PATH_IN]
			    [CIF_PATH_SELF];
		else
			array_bus_code = cifisp_path_mbus_codes[CIF_PATH_OUT]
			    [CIF_PATH_SELF];
	}

	array_size = CIF_FMT_NUM_MAX;

	if (index >= (int)array_size)
		return 0;

	for (i = 0; i < array_size; i++) {
		bus_code = array_bus_code[i];
		if (bus_code == 0)
			break;
		if (code && bus_code == *code)
			return bus_code;
		if (index == i)
			def_bus_code = bus_code;
	}

	return def_bus_code;
}

static void __rkisp1_disable_dcrop(struct rkisp1_device *dev,
				   enum rkisp1_stream_id stream_id,
				   bool async)
{
	void __iomem *dc_ctrl_addr = dev->config.base_addr + CIF_DUAL_CROP_CTRL;
	unsigned int dc_ctrl = readl(dc_ctrl_addr);

	if (stream_id == RKISP1_STREAM_SP)
		dc_ctrl &= ~(CIF_DUAL_CROP_SP_MODE_YUV |
			     CIF_DUAL_CROP_SP_MODE_RAW);
	else
		dc_ctrl &= ~(CIF_DUAL_CROP_MP_MODE_YUV |
			     CIF_DUAL_CROP_MP_MODE_RAW);

	if (async)
		dc_ctrl |= CIF_DUAL_CROP_GEN_CFG_UPD;
	else
		dc_ctrl |= CIF_DUAL_CROP_CFG_UPD;
	writel(readl(dc_ctrl_addr) | dc_ctrl, dc_ctrl_addr);
}

static int rkisp1_config_dcrop(struct rkisp1_device *dev,
			       enum rkisp1_stream_id stream_id, bool async)
{
	struct cif_isp10_dcrop_config *dcrop;
	struct v4l2_mbus_framefmt *input_fmt;
	unsigned int dc_ctrl =
	    readl(dev->config.base_addr + CIF_DUAL_CROP_CTRL);

	if (stream_id == RKISP1_STREAM_SP) {
		input_fmt = &dev->path_sdevs.sp_sdev.input_fmt;
		dcrop = &dev->path_sdevs.sp_config.crop_config;
	} else {
		input_fmt = &dev->path_sdevs.mp_sdev.input_fmt;
		dcrop = &dev->path_sdevs.mp_config.crop_config;
	}

	if (dcrop->crop.width == input_fmt->width &&
	    dcrop->crop.height == input_fmt->height &&
	    dcrop->crop.left == 0 && dcrop->crop.top == 0) {
		__rkisp1_disable_dcrop(dev, stream_id, async);
		return 0;
	}

	if (stream_id == RKISP1_STREAM_MP) {
		writel(dcrop->crop.left,
			  dev->config.base_addr + CIF_DUAL_CROP_M_H_OFFS);
		writel(dcrop->crop.top,
			  dev->config.base_addr + CIF_DUAL_CROP_M_V_OFFS);
		writel(dcrop->crop.width,
			  dev->config.base_addr + CIF_DUAL_CROP_M_H_SIZE);
		writel(dcrop->crop.height,
			  dev->config.base_addr + CIF_DUAL_CROP_M_V_SIZE);

		dc_ctrl |= CIF_DUAL_CROP_MP_MODE_YUV;
		if (async)
			dc_ctrl |= CIF_DUAL_CROP_GEN_CFG_UPD;
		else
			dc_ctrl |= CIF_DUAL_CROP_CFG_UPD;

		writel(dc_ctrl, dev->config.base_addr + CIF_DUAL_CROP_CTRL);
	} else if (stream_id == RKISP1_STREAM_SP) {
		writel(dcrop->crop.left,
			  dev->config.base_addr + CIF_DUAL_CROP_S_H_OFFS);
		writel(dcrop->crop.top,
			  dev->config.base_addr + CIF_DUAL_CROP_S_V_OFFS);
		writel(dcrop->crop.width,
			  dev->config.base_addr + CIF_DUAL_CROP_S_H_SIZE);
		writel(dcrop->crop.height,
			  dev->config.base_addr + CIF_DUAL_CROP_S_V_SIZE);

		dc_ctrl |= CIF_DUAL_CROP_SP_MODE_YUV;
		if (async)
			dc_ctrl |= CIF_DUAL_CROP_GEN_CFG_UPD;
		else
			dc_ctrl |= CIF_DUAL_CROP_CFG_UPD;

		writel(dc_ctrl, dev->config.base_addr + CIF_DUAL_CROP_CTRL);
	}

	return 0;
}

static int __rkisp1_get_yuv_fmt_subs(u32 bus_code, int *xsubs, int *ysubs)
{
	switch (bus_code) {
		/* yuv422 */
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		{
			*xsubs = 2;
			*ysubs = 4;
			break;
		}
		/* yuv420 */
	case MEDIA_BUS_FMT_YUYV8_1_5X8:
	case MEDIA_BUS_FMT_UYVY8_1_5X8:
	case MEDIA_BUS_FMT_VYUY8_1_5X8:
	case MEDIA_BUS_FMT_YVYU8_1_5X8:
		{
			*xsubs = 2;
			*ysubs = 2;
			break;
		}
		/* TODO: yuv444 */
	default:
		*xsubs = 4;
		*ysubs = 4;

	}

	return 0;
}

static void __rkisp1_disable_rsz(struct rkisp1_device *dev,
				 enum rkisp1_stream_id stream_id, bool async)
{
	void __iomem *rsz_ctrl_addr = dev->config.base_addr;

	if (stream_id == RKISP1_STREAM_SP)
		rsz_ctrl_addr += CIF_MRSZ_CTRL;
	else
		rsz_ctrl_addr += CIF_SRSZ_CTRL;

	writel(0, rsz_ctrl_addr);
	if (async){
		writel(readl(rsz_ctrl_addr) | CIF_RSZ_CTRL_CFG_UPD,
			rsz_ctrl_addr);
	}
}

static int rkisp1_config_rsz(struct rkisp1_device *dev,
			     enum rkisp1_stream_id stream_id, bool async)
{
	u32 i;
	void __iomem *scale_h_y_addr = dev->config.base_addr;
	void __iomem *scale_h_cr_addr = dev->config.base_addr;
	void __iomem *scale_h_cb_addr = dev->config.base_addr;
	void __iomem *scale_v_y_addr = dev->config.base_addr;
	void __iomem *scale_v_c_addr = dev->config.base_addr;
	void __iomem *rsz_ctrl_addr = dev->config.base_addr;
	struct cif_isp10_rsz_config *rsz_config;
	struct v4l2_mbus_framefmt *input_fmt;
	struct v4l2_mbus_framefmt *output_fmt;
	u32 rsz_ctrl;
	u32 input_width_y;
	u32 output_width_y;
	u32 input_height_y;
	u32 output_height_y;
	u32 input_width_c;
	u32 output_width_c;
	u32 input_height_c;
	u32 output_height_c;
	u32 scale_h_c;

	if (stream_id == RKISP1_STREAM_SP) {
		input_fmt = &dev->path_sdevs.sp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.sp_sdev.output_fmt;
		rsz_config = &dev->path_sdevs.sp_config.rsz_config;
	} else {
		input_fmt = &dev->path_sdevs.mp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.mp_sdev.output_fmt;
		rsz_config = &dev->path_sdevs.mp_config.rsz_config;
	}

	if ((rsz_config->input_width == rsz_config->output_width &&
	     rsz_config->input_height == rsz_config->output_height) ||
	    !CIFISP_BUSCODE_FMT_IS_YUV(input_fmt->code)) {
		__rkisp1_disable_rsz(dev, stream_id, async);
		return 0;
	}

	if (stream_id == RKISP1_STREAM_MP) {
		rsz_ctrl_addr += CIF_MRSZ_CTRL;
		scale_h_y_addr += CIF_MRSZ_SCALE_HY;
		scale_v_y_addr += CIF_MRSZ_SCALE_VY;
		scale_h_cb_addr += CIF_MRSZ_SCALE_HCB;
		scale_h_cr_addr += CIF_MRSZ_SCALE_HCR;
		scale_v_c_addr += CIF_MRSZ_SCALE_VC;
		/* No phase offset */
		writel(0, dev->config.base_addr + CIF_MRSZ_PHASE_HY);
		writel(0, dev->config.base_addr + CIF_MRSZ_PHASE_HC);
		writel(0, dev->config.base_addr + CIF_MRSZ_PHASE_VY);
		writel(0, dev->config.base_addr + CIF_MRSZ_PHASE_VC);
		/* Linear interpolation */
		for (i = 0; i < 64; i++) {
			writel(i,
				  dev->config.base_addr +
				  CIF_MRSZ_SCALE_LUT_ADDR);
			writel(i,
				  dev->config.base_addr + CIF_MRSZ_SCALE_LUT);
		}
	} else {
		rsz_ctrl_addr += CIF_SRSZ_CTRL;
		scale_h_y_addr += CIF_SRSZ_SCALE_HY;
		scale_v_y_addr += CIF_SRSZ_SCALE_VY;
		scale_h_cb_addr += CIF_SRSZ_SCALE_HCB;
		scale_h_cr_addr += CIF_SRSZ_SCALE_HCR;
		scale_v_c_addr += CIF_SRSZ_SCALE_VC;
		/* No phase offset */
		writel(0, dev->config.base_addr + CIF_SRSZ_PHASE_HY);
		writel(0, dev->config.base_addr + CIF_SRSZ_PHASE_HC);
		writel(0, dev->config.base_addr + CIF_SRSZ_PHASE_VY);
		writel(0, dev->config.base_addr + CIF_SRSZ_PHASE_VC);
		/* Linear interpolation */
		for (i = 0; i < 64; i++) {
			writel(i,
				  dev->config.base_addr +
				  CIF_SRSZ_SCALE_LUT_ADDR);
			writel(i,
				  dev->config.base_addr + CIF_SRSZ_SCALE_LUT);
		}
	}

	/* set RSZ input and output */
	v4l2_info(&dev->v4l2_dev,
		  "stream: %d fmt: %08x %dx%d -> fmt: %08x %dx%d\n",
		  stream_id, input_fmt->code,
		  rsz_config->input_width, rsz_config->input_height,
		  output_fmt->code, rsz_config->output_width,
		  rsz_config->output_height);

	/* set input and output sizes for scale calculation */
	input_width_y = rsz_config->input_width;
	output_width_y = rsz_config->output_width;
	input_height_y = rsz_config->input_height;
	output_height_y = rsz_config->output_height;
	input_width_c = input_width_y;
	output_width_c = output_width_y;
	input_height_c = input_height_y;
	output_height_c = output_height_y;

	if (CIFISP_BUSCODE_FMT_IS_YUV(input_fmt->code)) {
		int xsubs, ysubs;

		__rkisp1_get_yuv_fmt_subs(input_fmt->code, &xsubs, &ysubs);
		input_width_c = input_width_c * xsubs / 4;
		input_height_c = input_height_c * ysubs / 4;

		__rkisp1_get_yuv_fmt_subs(output_fmt->code, &xsubs, &ysubs);
		output_width_c = output_width_c * xsubs / 4;
		output_height_c = output_height_c * ysubs / 4;

		v4l2_info(&dev->v4l2_dev, "chroma scaling %dx%d -> %dx%d\n",
			  input_width_c, input_height_c,
			  output_width_c, output_height_c);

		if (((input_width_c == 0) && (output_width_c > 0)) ||
		    ((input_height_c == 0) && (output_height_c > 0))) {
			v4l2_err(&dev->v4l2_dev,
				 "Invalid convert from monochrome to chromatic\n");
			return -EINVAL;
		}
	} else {
		if ((input_width_y != output_width_y) ||
		    (input_height_y != output_height_y)) {
			v4l2_err(&dev->v4l2_dev,
				 "%dx%d -> %dx%d isn't support, "
				 "can only scale YUV input\n", input_width_y,
				 input_height_y, output_width_y,
				 output_height_y);
			return -EINVAL;
		}
	}

	/* calculate and set scale */
	rsz_ctrl = 0;
	if (input_width_y < output_width_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HY_ENABLE |
		    CIF_RSZ_CTRL_SCALE_HY_UP;
		writel(DIV_TRUNCATE((input_width_y - 1)
				       * CIF_RSZ_SCALER_BYPASS,
				       output_width_y - 1), scale_h_y_addr);
	} else if (input_width_y > output_width_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HY_ENABLE;
		writel(DIV_TRUNCATE
			  ((output_width_y - 1) * CIF_RSZ_SCALER_BYPASS,
			   input_width_y - 1) + 1, scale_h_y_addr);
	}
	if (input_width_c < output_width_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HC_ENABLE |
		    CIF_RSZ_CTRL_SCALE_HC_UP;
		scale_h_c = DIV_TRUNCATE((input_width_c - 1)
					 * CIF_RSZ_SCALER_BYPASS,
					 output_width_c - 1);
		writel(scale_h_c, scale_h_cb_addr);
		writel(scale_h_c, scale_h_cr_addr);
	} else if (input_width_c > output_width_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_HC_ENABLE;
		scale_h_c = DIV_TRUNCATE((output_width_c - 1)
					 * CIF_RSZ_SCALER_BYPASS,
					 input_width_c - 1) + 1;
		writel(scale_h_c, scale_h_cb_addr);
		writel(scale_h_c, scale_h_cr_addr);
	}

	if (input_height_y < output_height_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VY_ENABLE |
		    CIF_RSZ_CTRL_SCALE_VY_UP;
		writel(DIV_TRUNCATE
			  ((input_height_y - 1) * CIF_RSZ_SCALER_BYPASS,
			   output_height_y - 1), scale_v_y_addr);
	} else if (input_height_y > output_height_y) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VY_ENABLE;
		writel(DIV_TRUNCATE
			  ((output_height_y - 1) * CIF_RSZ_SCALER_BYPASS,
			   input_height_y - 1) + 1, scale_v_y_addr);
	}

	if (input_height_c < output_height_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VC_ENABLE |
		    CIF_RSZ_CTRL_SCALE_VC_UP;
		writel(DIV_TRUNCATE
			  ((input_height_c - 1) * CIF_RSZ_SCALER_BYPASS,
			   output_height_c - 1), scale_v_c_addr);
	} else if (input_height_c > output_height_c) {
		rsz_ctrl |= CIF_RSZ_CTRL_SCALE_VC_ENABLE;
		writel(DIV_TRUNCATE((output_height_c - 1)
				       * CIF_RSZ_SCALER_BYPASS,
				       input_height_c - 1) + 1, scale_v_c_addr);
	}

	writel(rsz_ctrl, rsz_ctrl_addr);

	if (stream_id == RKISP1_STREAM_MP) {
		if (async){
			int d = CIF_RSZ_CTRL_CFG_UPD;
			void *a = dev->config.base_addr + CIF_MRSZ_CTRL;
			writel(readl(a) | d, a);
		}

		v4l2_info(&dev->v4l2_dev,
			  "\n  MRSZ_CTRL 0x%08x/0x%08x\n"
			  "  MRSZ_SCALE_HY %d/%d\n"
			  "  MRSZ_SCALE_HCB %d/%d\n"
			  "  MRSZ_SCALE_HCR %d/%d\n"
			  "  MRSZ_SCALE_VY %d/%d\n"
			  "  MRSZ_SCALE_VC %d/%d\n"
			  "  MRSZ_PHASE_HY %d/%d\n"
			  "  MRSZ_PHASE_HC %d/%d\n"
			  "  MRSZ_PHASE_VY %d/%d\n"
			  "  MRSZ_PHASE_VC %d/%d\n",
			  readl(dev->config.base_addr + CIF_MRSZ_CTRL),
			  readl(dev->config.base_addr + CIF_MRSZ_CTRL_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_SCALE_HY),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_SCALE_HY_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_SCALE_HCB),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_SCALE_HCB_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_SCALE_HCR),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_SCALE_HCR_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_SCALE_VY),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_SCALE_VY_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_SCALE_VC),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_SCALE_VC_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_PHASE_HY),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_PHASE_HY_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_PHASE_HC),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_PHASE_HC_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_PHASE_VY),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_PHASE_VY_SHD),
			  readl(dev->config.base_addr + CIF_MRSZ_PHASE_VC),
			  readl(dev->config.base_addr +
				   CIF_MRSZ_PHASE_VC_SHD));
	} else {
		if (async){
			int d = CIF_RSZ_CTRL_CFG_UPD;
			void *a = dev->config.base_addr + CIF_SRSZ_CTRL;
			writel(readl(a) | d, a);
		}

		v4l2_info(&dev->v4l2_dev,
			  "\n  SRSZ_CTRL 0x%08x/0x%08x\n"
			  "  SRSZ_SCALE_HY %d/%d\n"
			  "  SRSZ_SCALE_HCB %d/%d\n"
			  "  SRSZ_SCALE_HCR %d/%d\n"
			  "  SRSZ_SCALE_VY %d/%d\n"
			  "  SRSZ_SCALE_VC %d/%d\n"
			  "  SRSZ_PHASE_HY %d/%d\n"
			  "  SRSZ_PHASE_HC %d/%d\n"
			  "  SRSZ_PHASE_VY %d/%d\n"
			  "  SRSZ_PHASE_VC %d/%d\n",
			  readl(dev->config.base_addr + CIF_SRSZ_CTRL),
			  readl(dev->config.base_addr + CIF_SRSZ_CTRL_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_SCALE_HY),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_SCALE_HY_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_SCALE_HCB),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_SCALE_HCB_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_SCALE_HCR),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_SCALE_HCR_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_SCALE_VY),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_SCALE_VY_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_SCALE_VC),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_SCALE_VC_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_PHASE_HY),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_PHASE_HY_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_PHASE_HC),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_PHASE_HC_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_PHASE_VY),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_PHASE_VY_SHD),
			  readl(dev->config.base_addr + CIF_SRSZ_PHASE_VC),
			  readl(dev->config.base_addr +
				   CIF_SRSZ_PHASE_VC_SHD));
	}

	return 0;
}

static int cifisp_check_sp_fmt(const u32 bus_code, int input_sel)
{
	if (input_sel != RKISP1_SP_INP_ISP)
		return 0;

	if (bus_code != MEDIA_BUS_FMT_YUYV8_1X16 &&
	    bus_code != MEDIA_BUS_FMT_UYVY8_1X16 &&
	    bus_code != MEDIA_BUS_FMT_VYUY8_1X16 &&
	    bus_code != MEDIA_BUS_FMT_YVYU8_1X16)
		return -EINVAL;

	return 0;
}

static int
cifisp_stream_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	u32 bus_code;
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);

	bus_code = cifisp_stream_find_format(path_dev->id, code->pad, NULL,
					     code->index);
	if (!bus_code)
		return -EINVAL;

	if (path_dev->id == RKISP1_STREAM_SP &&
	    cifisp_check_sp_fmt(bus_code, dev->path_sdevs.sp_config.input_sel))
		return -EINVAL;

	code->code = bus_code;
	return 0;
}

static int
cifisp_stream_subdev_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	struct v4l2_mbus_framefmt *input_fmt;
	struct v4l2_mbus_framefmt *output_fmt;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	if (path_dev->id == RKISP1_STREAM_SP) {
		input_fmt = &dev->path_sdevs.sp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.sp_sdev.output_fmt;
	} else {
		input_fmt = &dev->path_sdevs.mp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.mp_sdev.output_fmt;
	}

	if (fmt->pad == STREAM_PAD_SINK)
		/* full camera input frame size */
		*mf = *input_fmt;
	else
		/* crop or compose(scale) size */
		*mf = *output_fmt;

	return 0;
}

static int cifisp_stream_subdev_try_format(struct v4l2_subdev *sd,
					   struct v4l2_subdev_pad_config *cfg,
					   unsigned int pad,
					   struct v4l2_mbus_framefmt *fmt,
					   enum v4l2_subdev_format_whence which)
{
	struct v4l2_mbus_framefmt *output_fmt;
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	u32 bus_code = 0;

	if (path_dev->id == RKISP1_STREAM_SP)
		output_fmt = &dev->path_sdevs.sp_sdev.output_fmt;
	else
		output_fmt = &dev->path_sdevs.mp_sdev.output_fmt;

	bus_code = cifisp_stream_find_format(path_dev->id, pad, &fmt->code, -1);
	if (bus_code)
		fmt->code = bus_code;

	switch (pad) {
	case STREAM_PAD_SINK:
		if (!bus_code) {
			if (path_dev->id == RKISP1_STREAM_SP &&
			    (dev->path_sdevs.sp_config.input_sel ==
			     RKISP1_SP_INP_DMA_SP))
				fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;
			else
				fmt->code = MEDIA_BUS_FMT_YUYV8_1X16;
		}

		fmt->width = clamp_t(u32, fmt->width,
				     STREAM_MIN_MP_SP_INPUT_WIDTH,
				     STREAM_MAX_MP_SP_INPUT_WIDTH);
		fmt->height = clamp_t(u32, fmt->height,
				      STREAM_MIN_MP_SP_INPUT_HEIGHT,
				      STREAM_MAX_MP_SP_INPUT_HEIGHT);
		break;
	case STREAM_PAD_SOURCE:
		if (!bus_code)
			fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;

		if (path_dev->id == RKISP1_STREAM_MP) {
			/* check  whether input fmt is raw */
			if (CIFISP_BUSCODE_FMT_IS_RAW
			    (dev->path_sdevs.mp_sdev.input_fmt.code))
				fmt->code =
				    dev->path_sdevs.mp_sdev.input_fmt.code;
		}

		/* Hardcode the output size to the compose rectangle size. */
		fmt->width = output_fmt->width;
		fmt->height = output_fmt->height;
		break;
	default:
		v4l2_err(&dev->v4l2_dev, "unsupported pad %d", pad);
	}

	if (CIFISP_BUSCODE_FMT_IS_RAW(bus_code))
		fmt->colorspace = V4L2_COLORSPACE_SRGB;
	else
		fmt->colorspace = V4L2_COLORSPACE_JPEG;

	fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int cifisp_stream_subdev_set_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	struct rkisp1_stream *stream = NULL;
	struct v4l2_mbus_framefmt *input_fmt;
	struct v4l2_mbus_framefmt *output_fmt;
	struct cif_isp10_dcrop_config *dcrop;
	struct cif_isp10_rsz_config *rsz;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	if (path_dev->id == RKISP1_STREAM_SP)
		stream = &dev->strm_vdevs.sp_vdev;
	else
		stream = &dev->strm_vdevs.mp_vdev;

	if (((stream->id == RKISP1_STREAM_SP) &&
	     (stream->state == RKISP1_STATE_STREAMING)) ||
	    ((stream->id == RKISP1_STREAM_MP) &&
	     (stream->state == RKISP1_STATE_STREAMING)))
		return -EBUSY;

	cifisp_stream_subdev_try_format(sd, cfg, fmt->pad, mf, fmt->which);

	if (path_dev->id == RKISP1_STREAM_SP) {
		input_fmt = &dev->path_sdevs.sp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.sp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.sp_config.crop_config;
		rsz = &dev->path_sdevs.sp_config.rsz_config;
	} else {
		input_fmt = &dev->path_sdevs.mp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.mp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.mp_config.crop_config;
		rsz = &dev->path_sdevs.mp_config.rsz_config;
	}

	if (fmt->pad == STREAM_PAD_SINK) {
		*input_fmt = *mf;
		/* TODO:  quantization */
		/* reset crop */
		dcrop->crop.width = input_fmt->width;
		dcrop->crop.height = input_fmt->height;
		dcrop->crop.left = 0;
		dcrop->crop.top = 0;

		rsz->input_height = dcrop->crop.width;
		rsz->input_width = dcrop->crop.height;
		rsz->output_height = output_fmt->height;
		rsz->output_width = output_fmt->width;
		/* propagate to source pad */
		*output_fmt = *mf;
		cifisp_stream_subdev_try_format(sd, cfg, STREAM_PAD_SOURCE,
						output_fmt, fmt->which);
	} else {
		*output_fmt = *mf;
	}

	return 0;
}

static int
cifisp_stream_subdev_try_crop_scale(struct v4l2_subdev *sd,
				    struct v4l2_rect *input, u32 target)
{
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	struct v4l2_mbus_framefmt *input_fmt;
	struct v4l2_mbus_framefmt *output_fmt;
	struct cif_isp10_dcrop_config *dcrop;
	struct cif_isp10_rsz_config *rsz;
	u32 max_rsz_out_h, max_rsz_out_v;

	if (path_dev->id == RKISP1_STREAM_SP) {
		input_fmt = &dev->path_sdevs.sp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.sp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.sp_config.crop_config;
		rsz = &dev->path_sdevs.sp_config.rsz_config;
	} else {
		input_fmt = &dev->path_sdevs.mp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.mp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.mp_config.crop_config;
		rsz = &dev->path_sdevs.mp_config.rsz_config;
	}

	/* check  whether input fmt is raw */
	if (path_dev->id == RKISP1_STREAM_MP &&
	    CIFISP_BUSCODE_FMT_IS_RAW(dev->path_sdevs.mp_sdev.input_fmt.code)) {
		input->left = 0;
		input->top = 0;
		input->width = input_fmt->width;
		input->height = input_fmt->height;
		return 0;
	}

	if (target == V4L2_SEL_TGT_CROP) {
		/* crop */
		input->left =
		    clamp_t(u32, input->left, 0,
			    input_fmt->width - STREAM_MIN_MP_SP_INPUT_WIDTH);
		input->top =
		    clamp_t(u32, input->top, 0,
			    input_fmt->height - STREAM_MIN_MP_SP_INPUT_HEIGHT);
		input->width =
		    clamp_t(u32, input->width,
			    STREAM_MIN_MP_SP_INPUT_WIDTH,
			    input_fmt->width - input->left);
		input->height =
		    clamp_t(u32, input->height,
			    STREAM_MIN_MP_SP_INPUT_HEIGHT,
			    input_fmt->height - input->top);
		/* TODO: should deal with align ? */
		dcrop->crop.left = input->left;
		dcrop->crop.top = input->top;
		dcrop->crop.width = input->width;
		dcrop->crop.height = input->height;
		rsz->input_width = dcrop->crop.width;
		rsz->input_height = dcrop->crop.height;
		rkisp1_config_dcrop(dev, path_dev->id, false);
		/* should propagate to source ? */
	} else if (target == V4L2_SEL_TGT_COMPOSE) {
		/* compose for scaling, offset must be 0 */
		input->left = 0;
		input->top = 0;
		input->width = clamp_t(u32, input->width,
				       STREAM_MIN_RSZ_OUTPUT_WIDTH,
				       max_rsz_out_h);
		input->height = clamp_t(u32, input->height,
					STREAM_MIN_RSZ_OUTPUT_HEIGHT,
					max_rsz_out_v);
		output_fmt->width = input->width;
		output_fmt->height = input->height;
		rsz->output_width = output_fmt->width;
		rsz->output_height = output_fmt->height;
		rkisp1_config_rsz(dev, path_dev->id, false);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int
cifisp_stream_subdev_set_selection(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_selection *sel)
{
	if (sel->target != V4L2_SEL_TGT_CROP ||
	    sel->target != V4L2_SEL_TGT_COMPOSE || sel->pad != STREAM_PAD_SINK)
		return -EINVAL;

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		/*
		 * TODO
		 * v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		 * v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		 */
		return 0;
	}

	return cifisp_stream_subdev_try_crop_scale(sd, &sel->r, sel->target);
}

static int
cifisp_stream_subdev_get_selection(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_selection *sel)
{
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	struct v4l2_mbus_framefmt *input_fmt;
	struct v4l2_mbus_framefmt *output_fmt;
	struct cif_isp10_dcrop_config *dcrop;

	if (sel->pad != STREAM_PAD_SINK)
		return -EINVAL;

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		/*
		 *TODO
		 *v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		 *v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		 */
		return 0;
	}

	if (path_dev->id == RKISP1_STREAM_SP) {
		input_fmt = &dev->path_sdevs.sp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.sp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.sp_config.crop_config;
	} else {
		input_fmt = &dev->path_sdevs.mp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.mp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.mp_config.crop_config;
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE:
		sel->r.width = output_fmt->width;
		sel->r.height = output_fmt->height;
		sel->r.left = 0;
		sel->r.top = 0;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.width = input_fmt->width;
		sel->r.height = input_fmt->height;
		sel->r.left = 0;
		sel->r.top = 0;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r.width = dcrop->crop.width;
		sel->r.height = dcrop->crop.height;
		sel->r.left = dcrop->crop.left;
		sel->r.top = dcrop->crop.top;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rkisp1_config_path_chain_mode(struct rkisp1_device *dev,
					 u32 stream_ids)
{
	u32 dpcl = readl(dev->config.base_addr + CIF_VI_DPCL);

	/* chan_mode */
	if (stream_ids & RKISP1_STREAM_SP)
		dpcl |= CIF_VI_DPCL_CHAN_MODE_SP;

	if ((stream_ids & RKISP1_STREAM_MP) &&
	    !(dev->path_sdevs.sp_config.input_sel == RKISP1_SP_INP_DMA_SP)) {
		dpcl |= CIF_VI_DPCL_CHAN_MODE_MP;
		dpcl |= CIF_VI_DPCL_MP_MUX_MRSZ_MI;
	}

	writel(dpcl, dev->config.base_addr + CIF_VI_DPCL);

	return 0;
}

static int rkisp1_config_sp(struct rkisp1_device *dev)
{
	int ret;

	ret = rkisp1_config_rsz(dev, RKISP1_STREAM_SP, true);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	ret = rkisp1_config_dcrop(dev, RKISP1_STREAM_SP, true);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	return 0;
}

static int rkisp1_config_mp(struct rkisp1_device *dev)
{
	int ret;

	ret = rkisp1_config_rsz(dev, RKISP1_STREAM_MP, true);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	ret = rkisp1_config_dcrop(dev, RKISP1_STREAM_MP, true);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	return 0;
}

static void cif_isp10_stop_sp(struct rkisp1_device *dev)
{
	__rkisp1_disable_dcrop(dev, RKISP1_STREAM_SP, true);
	__rkisp1_disable_rsz(dev, RKISP1_STREAM_SP, true);
}

static void cif_isp10_stop_mp(struct rkisp1_device *dev)
{
	__rkisp1_disable_dcrop(dev, RKISP1_STREAM_MP, true);
	__rkisp1_disable_rsz(dev, RKISP1_STREAM_MP, true);
}

static int cif_isp10_start_mp(struct rkisp1_device *dev)
{
	int ret;

	/* config path , including rsz & crop */
	ret = rkisp1_config_mp(dev);
	if (ret < 0)
		return ret;
	rkisp1_config_path_chain_mode(dev, RKISP1_STREAM_MP);

	return 0;
}

static int cif_isp10_start_sp(struct rkisp1_device *dev)
{
	int ret;

	/* config path , including rsz & crop */
	ret = rkisp1_config_sp(dev);
	if (ret < 0)
		return ret;
	rkisp1_config_path_chain_mode(dev, RKISP1_STREAM_SP);

	return 0;
}

static int cifisp_stream_subdev_s_stream(struct v4l2_subdev *sd, int on)
{
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	u32 stream_id = path_dev->id;
	int ret;

	if (on) {
		if (stream_id & RKISP1_STREAM_SP) {
			ret = cif_isp10_start_sp(dev);
			if (ret < 0)
				return ret;
		} else {
			ret = cif_isp10_start_mp(dev);
			if (ret < 0)
				return ret;
		}
	} else {
		/* stop path */
		if (stream_id & RKISP1_STREAM_SP)
			cif_isp10_stop_sp(dev);
		if (stream_id & RKISP1_STREAM_MP)
			cif_isp10_stop_mp(dev);
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops cifisp_isp_stream_subdev_pad_ops = {
	.enum_mbus_code = cifisp_stream_subdev_enum_mbus_code,
	.get_selection = cifisp_stream_subdev_get_selection,
	.set_selection = cifisp_stream_subdev_set_selection,
	.get_fmt = cifisp_stream_subdev_get_fmt,
	.set_fmt = cifisp_stream_subdev_set_fmt,
};

static const struct v4l2_subdev_video_ops cifisp_isp_stream_subdev_video_ops = {
	.s_stream = cifisp_stream_subdev_s_stream,
};

static const struct v4l2_subdev_ops cifisp_isp_stream_subdev_ops = {
	.video = &cifisp_isp_stream_subdev_video_ops,
	.pad = &cifisp_isp_stream_subdev_pad_ops,
};

static void
cifisp_isp_stream_init_default_fmt(struct rkisp1_stream_subdev *path_dev)
{
	struct rkisp1_device *dev = sd_to_isp_dev(&path_dev->subdev);
	struct v4l2_mbus_framefmt *input_fmt;
	struct v4l2_mbus_framefmt *output_fmt;
	struct cif_isp10_dcrop_config *dcrop;
	struct cif_isp10_rsz_config *rsz;

	if (path_dev->id == RKISP1_STREAM_SP) {
		input_fmt = &dev->path_sdevs.sp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.sp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.sp_config.crop_config;
		rsz = &dev->path_sdevs.sp_config.rsz_config;
	} else {
		input_fmt = &dev->path_sdevs.mp_sdev.input_fmt;
		output_fmt = &dev->path_sdevs.mp_sdev.output_fmt;
		dcrop = &dev->path_sdevs.mp_config.crop_config;
		rsz = &dev->path_sdevs.mp_config.rsz_config;
	}

	input_fmt->code = MEDIA_BUS_FMT_YUYV8_1X16;
	input_fmt->width = STREAM_PATH_DEFAULT_WIDTH;
	input_fmt->height = STREAM_PATH_DEFAULT_HEIGHT;
	input_fmt->colorspace = V4L2_COLORSPACE_JPEG;
	/* TODO: quantization */
	/* input_fmt->quantization = 0; */

	/* propagate to source */
	output_fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;
	output_fmt->width = input_fmt->width;
	output_fmt->height = input_fmt->height;
	output_fmt->colorspace = input_fmt->colorspace;

	dcrop->crop.width = input_fmt->width;
	dcrop->crop.height = input_fmt->height;
	dcrop->crop.left = 0;
	dcrop->crop.top = 0;

	rsz->input_width = dcrop->crop.width;
	rsz->input_height = dcrop->crop.height;
	rsz->output_height = output_fmt->height;
	rsz->output_width = output_fmt->width;
}

static int cifisp_isp_stream_link_setup(struct media_entity *entity,
					const struct media_pad *local,
					const struct media_pad *remote,
					u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct rkisp1_stream_subdev *path_dev =
	    (struct rkisp1_stream_subdev *)v4l2_get_subdevdata(sd);
	struct rkisp1_device *dev = sd_to_isp_dev(sd);

	if ((path_dev->id != RKISP1_STREAM_SP)
	    || (local->flags == STREAM_PAD_SINK))
		return 0;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		dev->path_sdevs.sp_config.input_sel = 0;
		return 0;
	}

	if (dev->path_sdevs.sp_config.input_sel != 0)
		return -EBUSY;

	if (media_entity_type(remote->entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
		dev->path_sdevs.sp_config.input_sel = RKISP1_SP_INP_ISP;
	} else {
		if (RKISP1_INP_IS_DMA(dev->config.input_sel))
			return -EBUSY;
		dev->path_sdevs.sp_config.input_sel = RKISP1_SP_INP_DMA_SP;
	}

	return 0;
}

static const
struct media_entity_operations cifisp_isp_stream_subdev_media_ops = {
	.link_setup = cifisp_isp_stream_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

int register_stream_subdev(struct v4l2_subdev *subdev,
			   struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = subdev;
	const struct v4l2_subdev_ops *subdev_ops = NULL;
	const struct media_entity_operations *me_ops = NULL;
	struct rkisp1_stream_subdev *strm_subdev = sd_to_pathdev(subdev);
	struct rkisp1_device *dev = v4l2dev_to_ispdev(v4l2_dev);
	const char *sd_name = '\0';
	int ret;
	u32 group_id;

	if (strm_subdev == &dev->path_sdevs.sp_sdev) {
		subdev_ops = &cifisp_isp_stream_subdev_ops;
		me_ops = &cifisp_isp_stream_subdev_media_ops;
		group_id = GRP_ID_ISP_SP;
		sd_name = "cif_isp10_subdev_sp";
		strm_subdev->id = RKISP1_STREAM_SP;
	} else if (strm_subdev == &dev->path_sdevs.mp_sdev) {
		subdev_ops = &cifisp_isp_stream_subdev_ops;
		me_ops = &cifisp_isp_stream_subdev_media_ops;
		group_id = GRP_ID_ISP_MP;
		sd_name = "cif_isp10_subdev_mp";
		strm_subdev->id = RKISP1_STREAM_MP;
	} else {
		return -EINVAL;
	}

	v4l2_subdev_init(sd, subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), sd_name);
	sd->grp_id = group_id;

	strm_subdev->pads[0].flags = MEDIA_PAD_FL_SINK;
	strm_subdev->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 2, strm_subdev->pads, 0);
	if (ret < 0)
		return ret;

	sd->entity.ops = me_ops;
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, strm_subdev);

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		media_entity_cleanup(&sd->entity);
		v4l2_set_subdevdata(sd, NULL);
		v4l2_err(v4l2_dev, "Failed to register %s\n", sd_name);
		return ret;
	}

	cifisp_isp_stream_init_default_fmt(strm_subdev);

	return 0;
}

void unregister_stream_subdev(struct v4l2_subdev *subdev)
{
	media_entity_cleanup(&subdev->entity);
	v4l2_device_unregister_subdev(subdev);
	v4l2_set_subdevdata(subdev, NULL);
}
