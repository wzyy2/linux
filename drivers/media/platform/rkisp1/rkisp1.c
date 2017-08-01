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

#include <linux/iopoll.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include "mipiphy.h"
#include "path.h"
#include "path_video.h"
#include "regs.h"
#include "rkisp1.h"

struct cif_isp10_hw_error {
	const char *name;
	unsigned int count;
	unsigned int mask;
	unsigned int type;	/* isp:0 ;mipi:1 */
};

static struct cif_isp10_hw_error cif_isp10_hw_errors[] = {
	{
		.name = "isp_data_loss",
		.count = 0,
		.mask = CIF_ISP_DATA_LOSS,
		.type = 0,
	},
	{
		.name = "isp_pic_size_err",
		.count = 0,
		.mask = CIF_ISP_PIC_SIZE_ERROR,
		.type = 0,
	},
	{
		.name = "mipi_fifo_err",
		.count = 0,
		.mask = CIF_MIPI_SYNC_FIFO_OVFLW(1),
		.type = 1,
	},
	{
		.name = "dphy_err_sot",
		.count = 0,
		.mask = CIF_MIPI_ERR_SOT(3),
		.type = 1,
	},
	{
		.name = "dphy_err_sot_sync",
		.count = 0,
		.mask = CIF_MIPI_ERR_SOT_SYNC(3),
		.type = 1,
	},
	{
		.name = "dphy_err_eot_sync",
		.count = 0,
		.mask = CIF_MIPI_ERR_EOT_SYNC(3),
		.type = 1,
	},
	{
		.name = "dphy_err_ctrl",
		.count = 0,
		.mask = CIF_MIPI_ERR_CTRL(3),
		.type = 1,
	},
	{
		.name = "csi_err_protocol",
		.count = 0,
		.mask = CIF_MIPI_ERR_PROTOCOL,
		.type = 2,
	},
	{
		.name = "csi_err_ecc1",
		.count = 0,
		.mask = CIF_MIPI_ERR_ECC1,
		.type = 2,
	},
	{
		.name = "csi_err_ecc2",
		.count = 0,
		.mask = CIF_MIPI_ERR_ECC2,
		.type = 2,
	},
	{
		.name = "csi_err_cs",
		.count = 0,
		.mask = CIF_MIPI_ERR_CS,
		.type = 2,
	},
	{
		.name = "fifo_ovf",
		.count = 0,
		.mask = (3 << 0),
		.type = 2,
	},
	{
		.name = "isp_outform",
		.count = 0,
		.mask = CIF_ISP_ERR_OUTFORM_SIZE,
		.type = 0,
	},
	{
		.name = "isp_stab",
		.count = 0,
		.mask = CIF_ISP_ERR_IS_SIZE,
		.type = 0,
	},
	{
		.name = "isp_inform",
		.count = 0,
		.mask = CIF_ISP_ERR_INFORM_SIZE,
		.type = 0,
	}
};

/* TODO: define the isp frame size constrains */
#define CIF_ISP_INPUT_WIDTH_MAX		4032
#define CIF_ISP_INPUT_HEIGHT_MAX	3024
#define CIF_ISP_INPUT_WIDTH_MIN		32
#define CIF_ISP_INPUT_HEIGHT_MIN	32
#define CIF_ISP_OUTPUT_WIDTH_MAX	CIF_ISP_INPUT_WIDTH_MAX
#define CIF_ISP_OUTPUT_HEIGHT_MAX	CIF_ISP_INPUT_HEIGHT_MAX
#define CIF_ISP_OUTPUT_WIDTH_MIN	CIF_ISP_INPUT_WIDTH_MIN
#define CIF_ISP_OUTPUT_HEIGHT_MIN	CIF_ISP_INPUT_HEIGHT_MIN

#define CIF_ISP_DEFAULT_WIDTH		(800)
#define CIF_ISP_DEFAULT_HEIGHT		(600)

/*
 * This should only be called when configuring CIF
 * or at the frame end interrupt
 */
static void rkisp1_config_ism(struct rkisp1_device *dev, bool async)
{
	const struct rkisp1_ism_config *pconfig =
	    &dev->isp_sdev.isp_config.ism_config;
	u32 d;
	void *a;

	if (pconfig->ism_en) {

		iowrite32(pconfig->ism_params.recenter,
			  dev->config.base_addr + CIF_ISP_IS_RECENTER);
		iowrite32(pconfig->ism_params.max_dx,
			  dev->config.base_addr + CIF_ISP_IS_MAX_DX);
		iowrite32(pconfig->ism_params.max_dy,
			  dev->config.base_addr + CIF_ISP_IS_MAX_DY);
		iowrite32(pconfig->ism_params.displace,
			  dev->config.base_addr + CIF_ISP_IS_DISPLACE);
		iowrite32(pconfig->ism_params.h_offs,
			  dev->config.base_addr + CIF_ISP_IS_H_OFFS);
		iowrite32(pconfig->ism_params.v_offs,
			  dev->config.base_addr + CIF_ISP_IS_V_OFFS);
		iowrite32(pconfig->ism_params.h_size,
			  dev->config.base_addr + CIF_ISP_IS_H_SIZE);
		iowrite32(pconfig->ism_params.v_size,
			  dev->config.base_addr + CIF_ISP_IS_V_SIZE);
		a = dev->config.base_addr + CIF_ISP_IS_CTRL;
		iowrite32(ioread32(a) | 1, a);
		dev->isp_sdev.isp_config.output.width =
		    dev->isp_sdev.isp_config.ism_config.ism_params.h_size;
		dev->isp_sdev.isp_config.output.height =
		    dev->isp_sdev.isp_config.ism_config.ism_params.v_size;
	} else {
		iowrite32(pconfig->ism_params.recenter,
			  dev->config.base_addr + CIF_ISP_IS_RECENTER);
		iowrite32(pconfig->ism_params.max_dx,
			  dev->config.base_addr + CIF_ISP_IS_MAX_DX);
		iowrite32(pconfig->ism_params.max_dy,
			  dev->config.base_addr + CIF_ISP_IS_MAX_DY);
		iowrite32(pconfig->ism_params.displace,
			  dev->config.base_addr + CIF_ISP_IS_DISPLACE);
		iowrite32(0, dev->config.base_addr + CIF_ISP_IS_H_OFFS);
		iowrite32(0, dev->config.base_addr + CIF_ISP_IS_V_OFFS);
		iowrite32(dev->isp_sdev.isp_config.output.width,
			  dev->config.base_addr + CIF_ISP_IS_H_SIZE);
		iowrite32(dev->isp_sdev.isp_config.output.height,
			  dev->config.base_addr + CIF_ISP_IS_V_SIZE);
		iowrite32(0, dev->config.base_addr + CIF_ISP_IS_CTRL);
	}

	if (async){
		d = CIF_ISP_CTRL_ISP_CFG_UPD;
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) | d, a);
	}

	v4l2_info(&dev->v4l2_dev,
		  "\n  ISP_IS_H_OFFS %d/%d\n"
		  "  ISP_IS_V_OFFS %d/%d\n"
		  "  ISP_IS_H_SIZE %d/%d\n"
		  "  ISP_IS_V_SIZE %d/%d\n"
		  "  ISP_IS_RECENTER 0x%08x\n"
		  "  ISP_IS_MAX_DX %d\n"
		  "  ISP_IS_MAX_DY %d\n"
		  "  ISP_IS_DISPLACE 0x%08x\n"
		  "  ISP_IS_CTRL 0x%08x\n",
		  ioread32(dev->config.base_addr + CIF_ISP_IS_H_OFFS),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_H_OFFS_SHD),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_V_OFFS),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_V_OFFS_SHD),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_H_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_H_SIZE_SHD),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_V_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_V_SIZE_SHD),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_RECENTER),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_MAX_DX),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_MAX_DY),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_DISPLACE),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_CTRL));
}

static int rkisp1_config_isp(struct rkisp1_device *dev)
{
	int ret = 0;
	u32 yuv_seq = 0;
	u32 bpp, d;
	u32 isp_input_sel = 0;
	u32 isp_bayer_pat = 0;
	u32 acq_mult = 1;
	u32 irq_mask = 0;
	u32 signal = 0;
	void *a;
	enum cif_isp10_pix_fmt in_pix_fmt;
	struct cif_isp10_frm_fmt *output;
	struct pltfrm_cam_itf *cam_itf;

	d = CIF_ICCL_ISP_CLK;
	a = dev->config.base_addr + CIF_ICCL;
	iowrite32(ioread32(a) | d, a);

	in_pix_fmt = dev->isp_sdev.isp_config.input.pix_fmt;

	output = &dev->isp_sdev.isp_config.output;
	cam_itf = &dev->config.cam_itf;

	if (RKISP1_PIX_FMT_IS_RAW_BAYER(in_pix_fmt)) {
		if (!dev->strm_vdevs.mi_config.raw_enable) {
			if ((dev->cif_streamon_cnt > 0) &&
			    ((dev->strm_vdevs.mp_vdev.state ==
			      RKISP1_STATE_STREAMING) ||
			     (dev->strm_vdevs.sp_vdev.state ==
			      RKISP1_STATE_STREAMING))) {
				if (dev->strm_vdevs.mp_vdev.path_cfg.output.
				    quantization !=
				    dev->strm_vdevs.sp_vdev.path_cfg.output.
				    quantization) {
					v4l2_err(&dev->v4l2_dev,
						 "colorspace quantization (mp: %d, sp: %d) is not support!\n",
						 dev->strm_vdevs.
						 mp_vdev.path_cfg.output.
						 quantization,
						 dev->strm_vdevs.
						 sp_vdev.path_cfg.output.
						 quantization);
				}
			}

			if (dev->strm_vdevs.sp_vdev.state ==
			    RKISP1_STATE_STREAMING) {
				output->quantization =
				    dev->strm_vdevs.sp_vdev.path_cfg.output.
				    quantization;
			}

			if (dev->strm_vdevs.mp_vdev.state ==
			    RKISP1_STATE_STREAMING) {
				output->quantization =
				    dev->strm_vdevs.mp_vdev.path_cfg.output.
				    quantization;
			}

			iowrite32(0xc,
				  dev->config.base_addr + CIF_ISP_DEMOSAIC);

			if (PLTFRM_CAM_ITF_IS_BT656(dev->config.cam_itf.type)) {
				iowrite32(CIF_ISP_CTRL_ISP_MODE_BAYER_ITU656,
					  dev->config.base_addr + CIF_ISP_CTRL);
			} else {
				iowrite32(CIF_ISP_CTRL_ISP_MODE_BAYER_ITU601,
					  dev->config.base_addr + CIF_ISP_CTRL);
			}
		} else {
			if (PLTFRM_CAM_ITF_IS_BT656(dev->config.cam_itf.type)) {
				iowrite32(CIF_ISP_CTRL_ISP_MODE_RAW_PICT_ITU656,
					  dev->config.base_addr + CIF_ISP_CTRL);
			} else {
				iowrite32(CIF_ISP_CTRL_ISP_MODE_RAW_PICT,
					  dev->config.base_addr + CIF_ISP_CTRL);
			}
		}

		bpp = RKISP1_PIX_FMT_GET_BPP(in_pix_fmt);
		if (bpp == 8) {
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_8B_MSB;
		} else if (bpp == 10) {
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_10B_MSB;
		} else if (bpp == 12) {
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
		} else {
			v4l2_err(&dev->v4l2_dev,
				 "%d bits per pixel not supported\n", bpp);
			ret = -EINVAL;
			goto err;
		}
		if (RKISP1_PIX_FMT_BAYER_PAT_IS_BGGR(in_pix_fmt)) {
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_BGGR;
		} else if (RKISP1_PIX_FMT_BAYER_PAT_IS_GBRG(in_pix_fmt)) {
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GBRG;
		} else if (RKISP1_PIX_FMT_BAYER_PAT_IS_GRBG(in_pix_fmt)) {
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GRBG;
		} else if (RKISP1_PIX_FMT_BAYER_PAT_IS_RGGB(in_pix_fmt)) {
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_RGGB;
		} else {
			v4l2_err(&dev->v4l2_dev,
				 "BAYER pattern not supported\n");
			ret = -EINVAL;
			goto err;
		}
	} else if (RKISP1_PIX_FMT_IS_YUV(in_pix_fmt)) {
		acq_mult = 2;
		if (PLTFRM_CAM_ITF_IS_MIPI(dev->config.cam_itf.type)) {
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
			iowrite32(CIF_ISP_CTRL_ISP_MODE_ITU601,
				  dev->config.base_addr + CIF_ISP_CTRL);
		} else if (PLTFRM_CAM_ITF_IS_DVP(dev->config.cam_itf.type)) {
			if (PLTFRM_CAM_ITF_IS_BT656(dev->config.cam_itf.type)) {
				iowrite32(CIF_ISP_CTRL_ISP_MODE_ITU656,
					  dev->config.base_addr + CIF_ISP_CTRL);
			} else {
				iowrite32(CIF_ISP_CTRL_ISP_MODE_ITU601,
					  dev->config.base_addr + CIF_ISP_CTRL);
			}

			switch (PLTFRM_CAM_ITF_DVP_BW(dev->config.cam_itf.type)) {
			case 8:
				isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_8B_ZERO;
				break;
			case 10:
				isp_input_sel =
				    CIF_ISP_ACQ_PROP_IN_SEL_10B_ZERO;
				break;
			case 12:
				isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
				break;
			default:
				v4l2_err(&dev->v4l2_dev,
					 "cam itf type: %d isn't supported\n",
					 dev->config.cam_itf.type);
				break;
			}
		} else {
			v4l2_err(&dev->v4l2_dev,
				 "cam itf type %d isn't supported\n",
				 dev->config.cam_itf.type);
		}
		/*
		 * ISP DATA LOSS is only meaningful
		 * when input is not DMA
		 */
		irq_mask |= CIF_ISP_DATA_LOSS;

		if (RKISP1_PIX_FMT_YUV_IS_YC_SWAPPED(in_pix_fmt)) {
			yuv_seq = CIF_ISP_ACQ_PROP_CBYCRY;
			cif_isp10_pix_fmt_set_yc_swapped(output->pix_fmt, 0);
		} else if (RKISP1_PIX_FMT_YUV_IS_UV_SWAPPED(in_pix_fmt)) {
			yuv_seq = CIF_ISP_ACQ_PROP_YCRYCB;
		} else {
			yuv_seq = CIF_ISP_ACQ_PROP_YCBYCR;
		}
	} else {
		v4l2_err(&dev->v4l2_dev, "format %d not supported\n",
			 in_pix_fmt);
		ret = -EINVAL;
		goto err;
	}

	/* Set up input acquisition properties */
	if (PLTFRM_CAM_ITF_IS_DVP(cam_itf->type)) {
		signal =
		    ((cam_itf->cfg.dvp.pclk == PLTFRM_CAM_SDR_POS_EDG) ?
		     CIF_ISP_ACQ_PROP_POS_EDGE : CIF_ISP_ACQ_PROP_NEG_EDGE);

		if (PLTFRM_CAM_ITF_IS_BT601(cam_itf->type)) {
			signal |=
			    (cam_itf->cfg.dvp.vsync ==
			     PLTFRM_CAM_SIGNAL_HIGH_LEVEL) ?
			    CIF_ISP_ACQ_PROP_VSYNC_HIGH :
			    CIF_ISP_ACQ_PROP_VSYNC_LOW;

			signal |=
			    (cam_itf->cfg.dvp.hsync ==
			     PLTFRM_CAM_SIGNAL_HIGH_LEVEL) ?
			    CIF_ISP_ACQ_PROP_HSYNC_HIGH :
			    CIF_ISP_ACQ_PROP_HSYNC_LOW;

		} else {
			signal |= CIF_ISP_ACQ_PROP_HSYNC_HIGH |
			    CIF_ISP_ACQ_PROP_VSYNC_HIGH;
		}
	} else {
		signal = CIF_ISP_ACQ_PROP_NEG_EDGE |
		    CIF_ISP_ACQ_PROP_HSYNC_HIGH | CIF_ISP_ACQ_PROP_VSYNC_HIGH;
	}

	iowrite32(signal | yuv_seq | CIF_ISP_ACQ_PROP_FIELD_SEL_ALL | isp_input_sel | isp_bayer_pat | (0 << 20),	/* input_selection_no_app */
		  dev->config.base_addr + CIF_ISP_ACQ_PROP);
	iowrite32(0, dev->config.base_addr + CIF_ISP_ACQ_NR_FRAMES);

	/* Acquisition Size */
	iowrite32(dev->isp_sdev.isp_config.input.defrect.left,
		  dev->config.base_addr + CIF_ISP_ACQ_H_OFFS);
	iowrite32(dev->isp_sdev.isp_config.input.defrect.top,
		  dev->config.base_addr + CIF_ISP_ACQ_V_OFFS);
	iowrite32(acq_mult * dev->isp_sdev.isp_config.input.defrect.width,
		  dev->config.base_addr + CIF_ISP_ACQ_H_SIZE);
	iowrite32(dev->isp_sdev.isp_config.input.defrect.height,
		  dev->config.base_addr + CIF_ISP_ACQ_V_SIZE);

	iowrite32(output->defrect.top,
		  dev->config.base_addr + CIF_ISP_OUT_V_OFFS);
	iowrite32(output->defrect.left,
		  dev->config.base_addr + CIF_ISP_OUT_H_OFFS);
	iowrite32(output->width, dev->config.base_addr + CIF_ISP_OUT_H_SIZE);
	iowrite32(output->height, dev->config.base_addr + CIF_ISP_OUT_V_SIZE);

	dev->isp_sdev.isp_dev.input_width =
	    dev->isp_sdev.isp_config.input.defrect.width;
	dev->isp_sdev.isp_dev.input_height =
	    dev->isp_sdev.isp_config.input.defrect.height;

	/* interrupt mask */
	irq_mask |=
	    CIF_ISP_FRAME |
	    CIF_ISP_PIC_SIZE_ERROR | CIF_ISP_FRAME_IN | CIF_ISP_V_START;
	iowrite32(irq_mask, dev->config.base_addr + CIF_ISP_IMSC);

	if (!dev->strm_vdevs.mi_config.raw_enable)
		cifisp_configure_isp(&dev->isp_sdev.isp_dev,
				     in_pix_fmt, output->quantization);
	else
		cifisp_disable_isp(&dev->isp_sdev.isp_dev);

	v4l2_info(&dev->v4l2_dev,
		  "\n  ISP_CTRL 0x%08x\n"
		  "  ISP_IMSC 0x%08x\n"
		  "  ISP_ACQ_PROP 0x%08x\n"
		  "  ISP_ACQ %dx%d@(%d,%d)\n"
		  "  ISP_OUT %dx%d@(%d,%d)\n"
		  "  ISP_IS %dx%d@(%d,%d)\n",
		  ioread32(dev->config.base_addr + CIF_ISP_CTRL),
		  ioread32(dev->config.base_addr + CIF_ISP_IMSC),
		  ioread32(dev->config.base_addr + CIF_ISP_ACQ_PROP),
		  ioread32(dev->config.base_addr + CIF_ISP_ACQ_H_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_ACQ_V_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_ACQ_H_OFFS),
		  ioread32(dev->config.base_addr + CIF_ISP_ACQ_V_OFFS),
		  ioread32(dev->config.base_addr + CIF_ISP_OUT_H_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_OUT_V_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_OUT_H_OFFS),
		  ioread32(dev->config.base_addr + CIF_ISP_OUT_V_OFFS),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_H_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_V_SIZE),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_H_OFFS),
		  ioread32(dev->config.base_addr + CIF_ISP_IS_V_OFFS));

	return 0;
err:
	v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
	return ret;
}

static int rkisp1_config_mipi(struct rkisp1_device *dev)
{
	int ret = 0;
	u32 data_type;
	u32 mipi_ctrl;
	u32 shutdown_lanes;
	u32 i, d;
	void *a;
	enum cif_isp10_pix_fmt in_pix_fmt;

	if (!RKISP1_INP_IS_MIPI(dev->config.input_sel)) {
		d = (u32)~CIF_ICCL_MIPI_CLK;
		a = dev->config.base_addr + CIF_ICCL;
		iowrite32(ioread32(a) & d, a);
		v4l2_err(&dev->v4l2_dev, "MIPI disabled\n");
		return 0;
	}
	d = CIF_ICCL_MIPI_CLK;
	a = dev->config.base_addr + CIF_ICCL;
	iowrite32(ioread32(a) | d, a);

	in_pix_fmt = dev->isp_sdev.isp_config.input.pix_fmt;

	v4l2_info(&dev->v4l2_dev, "input 0x%x, vc = %d, nb_lanes = %d\n",
		  dev->config.input_sel, dev->config.cam_itf.cfg.mipi.vc,
		  dev->config.cam_itf.cfg.mipi.nb_lanes);

	if ((dev->config.cam_itf.cfg.mipi.nb_lanes == 0) ||
	    (dev->config.cam_itf.cfg.mipi.nb_lanes > 4)) {
		v4l2_err(&dev->v4l2_dev,
			 "invalid number (%d) of MIPI lanes, valid range is [1..4]\n",
			 dev->config.cam_itf.cfg.mipi.nb_lanes);
		ret = -EINVAL;
		goto err;
	}

	shutdown_lanes = 0;
	for (i = 0; i < dev->config.cam_itf.cfg.mipi.nb_lanes; i++)
		shutdown_lanes |= (1 << i);

	mipi_ctrl =
	    CIF_MIPI_CTRL_NUM_LANES(dev->config.cam_itf.cfg.mipi.nb_lanes - 1) |
	    CIF_MIPI_CTRL_ERR_SOT_HS_ENA |
	    CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_SKIP |
	    CIF_MIPI_CTRL_SHUTDOWNLANES(0xf) |
	    CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_ENA |
	    CIF_MIPI_CTRL_ERR_SOT_HS_ENA | CIF_MIPI_CTRL_CLOCKLANE_ENA;

	iowrite32(mipi_ctrl, dev->config.base_addr + CIF_MIPI_CTRL);

	iowrite32(mipi_ctrl | 0xf << 8,	//TODO(zsq): for phy0/phy1, shutdown lanes is different?
		  dev->config.base_addr + CIF_MIPI_CTRL);

	/* Configure Data Type and Virtual Channel */
	if (RKISP1_PIX_FMT_IS_YUV(in_pix_fmt)) {
		if ((RKISP1_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
		    (RKISP1_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2) &&
		    (RKISP1_PIX_FMT_GET_BPP(in_pix_fmt) == 12))
			data_type = CIF_CSI2_DT_YUV420_8b;
		else if ((RKISP1_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			 (RKISP1_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 2) &&
			 (RKISP1_PIX_FMT_GET_BPP(in_pix_fmt) == 15))
			data_type = CIF_CSI2_DT_YUV420_10b;
		else if ((RKISP1_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			 (RKISP1_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 4) &&
			 (RKISP1_PIX_FMT_GET_BPP(in_pix_fmt) == 16))
			data_type = CIF_CSI2_DT_YUV422_8b;
		else if ((RKISP1_PIX_FMT_YUV_GET_X_SUBS(in_pix_fmt) == 2) &&
			 (RKISP1_PIX_FMT_YUV_GET_Y_SUBS(in_pix_fmt) == 4) &&
			 (RKISP1_PIX_FMT_GET_BPP(in_pix_fmt) == 20))
			data_type = CIF_CSI2_DT_YUV422_10b;
		else {
			v4l2_err(&dev->v4l2_dev, "unsupported format 0x%08x\n",
				 in_pix_fmt);
			ret = -EINVAL;
			goto err;
		}
	} else if (RKISP1_PIX_FMT_IS_RAW_BAYER(in_pix_fmt)) {
		if (RKISP1_PIX_FMT_GET_BPP(in_pix_fmt) == 8) {
			data_type = CIF_CSI2_DT_RAW8;
		} else if (RKISP1_PIX_FMT_GET_BPP(in_pix_fmt) == 10) {
			data_type = CIF_CSI2_DT_RAW10;
		} else if (RKISP1_PIX_FMT_GET_BPP(in_pix_fmt) == 12) {
			data_type = CIF_CSI2_DT_RAW12;
		} else {
			v4l2_err(&dev->v4l2_dev, "unsupported format 0x%08x\n",
				 in_pix_fmt);
			ret = -EINVAL;
			goto err;
		}
	} else if (in_pix_fmt == CIF_RGB565) {
		data_type = CIF_CSI2_DT_RGB565;
	} else if (in_pix_fmt == CIF_RGB666) {
		data_type = CIF_CSI2_DT_RGB666;
	} else if (in_pix_fmt == CIF_RGB888) {
		data_type = CIF_CSI2_DT_RGB888;
	} else {
		v4l2_err(&dev->v4l2_dev, "unsupported format 0x%08x\n",
			 in_pix_fmt);
		ret = -EINVAL;
		goto err;
	}

	iowrite32(CIF_MIPI_DATA_SEL_DT(data_type) |
		  CIF_MIPI_DATA_SEL_VC(dev->config.cam_itf.cfg.mipi.vc),
		  dev->config.base_addr + CIF_MIPI_IMG_DATA_SEL);

	/* Enable MIPI interrupts */
	iowrite32(~0, dev->config.base_addr + CIF_MIPI_ICR);
	/*
	 * Disable CIF_MIPI_ERR_DPHY interrupt here temporary for
	 * isp bus may be dead when switch isp.
	 */
	iowrite32(CIF_MIPI_FRAME_END |
		  CIF_MIPI_ERR_CSI |
		  CIF_MIPI_ERR_DPHY |
		  CIF_MIPI_SYNC_FIFO_OVFLW(3) |
		  CIF_MIPI_ADD_DATA_OVFLW,
		  dev->config.base_addr + CIF_MIPI_IMSC);

	v4l2_info(&dev->v4l2_dev, "\n  MIPI_CTRL 0x%08x\n"
		  "  MIPI_IMG_DATA_SEL 0x%08x\n"
		  "  MIPI_STATUS 0x%08x\n"
		  "  MIPI_IMSC 0x%08x\n",
		  ioread32(dev->config.base_addr + CIF_MIPI_CTRL),
		  ioread32(dev->config.base_addr + CIF_MIPI_IMG_DATA_SEL),
		  ioread32(dev->config.base_addr + CIF_MIPI_STATUS),
		  ioread32(dev->config.base_addr + CIF_MIPI_IMSC));

	return 0;
err:
	v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
	return ret;
}

static int rkisp1_config_path(struct rkisp1_device *dev)
{
	u32 dpcl = ioread32(dev->config.base_addr + CIF_VI_DPCL);

	/* if_sel */
	if (PLTFRM_CAM_ITF_IS_DVP(dev->config.cam_itf.type)) {
		dpcl |= CIF_VI_DPCL_IF_SEL_PARALLEL;
	} else if (PLTFRM_CAM_ITF_IS_MIPI(dev->config.cam_itf.type)) {
		dpcl |= CIF_VI_DPCL_IF_SEL_MIPI;
	} else {
		v4l2_err(&dev->v4l2_dev,
			 "Sensor Interface: 0x%x isn't support\n",
			 dev->config.cam_itf.type);
		return -EINVAL;
	}

	iowrite32(dpcl, dev->config.base_addr + CIF_VI_DPCL);

	v4l2_info(&dev->v4l2_dev, "CIF_DPCL 0x%08x\n", dpcl);

	return 0;
}

static void rkisp1_config_clk(struct rkisp1_device *dev)
{
	iowrite32(CIF_CCL_CIF_CLK_ENA, dev->config.base_addr + CIF_CCL);
	iowrite32(0x0000187B, dev->config.base_addr + CIF_ICCL);

	v4l2_info(&dev->v4l2_dev, "\n  CIF_CCL 0x%08x\nCIF_ICCL 0x%08x\n",
		  ioread32(dev->config.base_addr + CIF_CCL),
		  ioread32(dev->config.base_addr + CIF_ICCL));
}

static int rkisp1_config_cif(struct rkisp1_device *dev)
{
	int ret = 0;
	u32 cif_id;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state);

	v4l2_info(&dev->v4l2_dev, "start configuring CIF...\n");

	cif_id = ioread32(dev->config.base_addr + CIF_VI_ID);
	dev->config.out_of_buffer_stall = RKISP1_ALWAYS_STALL_ON_NO_BUFS;
	v4l2_err(&dev->v4l2_dev, "CIF_ID 0x%08x\n", cif_id);

	/*
	 * Cancel isp reset internal here temporary for
	 * isp bus may be dead when switch isp.
	 */
	/*
	 * iowrite32(CIF_IRCL_CIF_SW_RST,
	 * dev->config.base_addr + CIF_IRCL);
	 */

	/* Decide when to switch to asynchronous mode */
	/*
	 * TODO: remove dev->isp_dev.ycflt_en check for
	 * HW with the scaler fix.
	 */
	dev->strm_vdevs.mi_config.async_updt = RKISP1_ALWAYS_ASYNC;
	if (dev->isp_sdev.isp_config.ism_config.ism_en)
		dev->strm_vdevs.mi_config.async_updt |= RKISP1_ASYNC_ISM;

	/* TODO : get config of dev->config.cam_itf firstly */
	if (PLTFRM_CAM_ITF_IS_MIPI(dev->config.cam_itf.type)) {
		ret = rkisp1_config_mipi(dev);
		if (ret < 0)
			goto err;
	}

	ret = rkisp1_config_isp(dev);
	if (ret < 0)
		goto err;

	ret = rkisp1_config_path(dev);
	if (ret < 0)
		goto err;

	rkisp1_config_ism(dev, true);
	dev->isp_sdev.isp_config.ism_config.ism_update_needed = false;

	/* Turn off XNR vertical subsampling when ism cropping is enabled */
	if (dev->isp_sdev.isp_config.ism_config.ism_en) {
		if (!dev->isp_sdev.isp_dev.cif_ism_cropping)
			dev->isp_sdev.isp_dev.cif_ism_cropping = true;
	} else {
		if (dev->isp_sdev.isp_dev.cif_ism_cropping)
			dev->isp_sdev.isp_dev.cif_ism_cropping = false;
	}

	if (dev->strm_vdevs.mi_config.async_updt)
		v4l2_err(&dev->v4l2_dev, "CIF in asynchronous mode (0x%08x)\n",
			 dev->strm_vdevs.mi_config.async_updt);
	/* rkisp1_config_cif is alway called when cif is at off state,
	 * so here is safe to clear cif_streamon_cnt.
	 */
	dev->cif_streamon_cnt = 0;
	return 0;
err:
	v4l2_err(&dev->v4l2_dev, "failed with error %d\n", ret);
	return ret;
}

static int cif_isp10_stop(struct rkisp1_device *dev)
{
	unsigned long flags = 0;
	unsigned int val, d;
	void *a;

	v4l2_info(&dev->v4l2_dev, "SP state = %d, MP state = %d\n",
		  dev->strm_vdevs.sp_vdev.state, dev->strm_vdevs.mp_vdev.state);

	if (--dev->cif_streamon_cnt == 0) {
		/*
		 * Modify ISP stop sequence for isp bus dead:
		 * ISP(mi) stop in mi frame end -> Stop ISP(mipi) ->
		 * Stop ISP(isp) ->wait for ISP isp off
		 */
		local_irq_save(flags);
		/* stop and clear MI, MIPI, and ISP interrupts */
		iowrite32(0, dev->config.base_addr + CIF_MIPI_IMSC);
		iowrite32(~0, dev->config.base_addr + CIF_MIPI_ICR);

		iowrite32(0, dev->config.base_addr + CIF_ISP_IMSC);
		iowrite32(~0, dev->config.base_addr + CIF_ISP_ICR);

		cif_iowrite32_verify(0,
				     dev->config.base_addr + CIF_MI_IMSC, ~0);
		iowrite32(~0, dev->config.base_addr + CIF_MI_ICR);
		d = (u32)~CIF_MIPI_CTRL_OUTPUT_ENA;
		a = dev->config.base_addr + CIF_MIPI_CTRL;
		iowrite32(ioread32(a) & d, a);
		/* stop ISP */
		d = (u32)~(CIF_ISP_CTRL_ISP_INFORM_ENABLE |
				   CIF_ISP_CTRL_ISP_ENABLE);
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) & d, a);
		d = CIF_ISP_CTRL_ISP_CFG_UPD;
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) | d, a);

		local_irq_restore(flags);
		readx_poll_timeout(ioread32,
				   dev->config.base_addr + CIF_ISP_RIS,
				   val, val & CIF_ISP_OFF, 20, 100);
	}
	v4l2_info(&dev->v4l2_dev,
		  "SP state %d, MP state %d MI_CTRL 0x%08x"
		  "ISP_CTRL 0x%08x MIPI_CTRL 0x%08x\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state,
		  ioread32(dev->config.base_addr + CIF_MI_CTRL),
		  ioread32(dev->config.base_addr + CIF_ISP_CTRL),
		  ioread32(dev->config.base_addr + CIF_MIPI_CTRL));

	return 0;
}

static int cif_isp10_start(struct rkisp1_device *dev)
{
	unsigned int d, ret;
	void *a;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d, isp start cnt = %d\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state, dev->cif_streamon_cnt);

	if (dev->cif_streamon_cnt++ > 0)
		return 0;

	/* Activate MIPI */
	if (RKISP1_INP_IS_MIPI(dev->config.input_sel)){
		d = CIF_MIPI_CTRL_OUTPUT_ENA;
		a = dev->config.base_addr + CIF_MIPI_CTRL;
		iowrite32(ioread32(a) | d, a);
	}

	/* Activate ISP ! */
	if (RKISP1_INP_NEED_ISP(dev->config.input_sel)){
		d = CIF_ISP_CTRL_ISP_CFG_UPD |
			CIF_ISP_CTRL_ISP_INFORM_ENABLE |
			CIF_ISP_CTRL_ISP_ENABLE;
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) | d, a);
	}

	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0)
		return ret;

	/*
	 * CIF spec says to wait for sufficient time after enabling
	 * the MIPI interface and before starting the sensor output.
	 */
	mdelay(1);
	/* start sensor output! */
	dev->isp_sdev.isp_dev.frame_id = 0;
	//TODO(zsq): spinlock bad magic: spin_lock_irq(&dev->isp_dev.lock);

	v4l2_info(&dev->v4l2_dev, "starting image source...\n");

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d MI_CTRL 0x%08x\n"
		  "  ISP_CTRL 0x%08x MIPI_CTRL 0x%08x\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state,
		  ioread32(dev->config.base_addr + CIF_MI_CTRL),
		  ioread32(dev->config.base_addr + CIF_ISP_CTRL),
		  ioread32(dev->config.base_addr + CIF_MIPI_CTRL));

	return 0;
}

/* Function to be called inside ISR to update CIF ISM/DCROP/RSZ */
static int cif_isp10_update_ism_dcr_rsz(struct rkisp1_device *dev)
{
	int d, ret = 0;
	void *a;

	if (dev->isp_sdev.isp_config.ism_config.ism_update_needed)
		dev->isp_sdev.isp_dev.cif_ism_cropping =
		    dev->isp_sdev.isp_config.ism_config.ism_en;

	/*
	 * Update ISM, rkisp1_config_ism() changes the output size of isp,
	 * so it must be called before rkisp1_config_rsz()
	 */
	if (dev->isp_sdev.isp_config.ism_config.ism_update_needed) {
		rkisp1_config_ism(dev, false);
		dev->isp_sdev.isp_config.ism_config.ism_update_needed = false;
		d = CIF_ISP_CTRL_ISP_CFG_UPD;
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) | d, a);

		if (dev->isp_sdev.isp_config.ism_config.ism_en)
			dev->strm_vdevs.mi_config.async_updt |=
			    RKISP1_ASYNC_ISM;
	}

	v4l2_err(&dev->v4l2_dev, "failed with err %d\n", ret);
	return ret;
}

// TODO revise the img_src_exps

/* Public Functions */
void cif_isp10_sensor_mode_data_sync(struct rkisp1_device *dev,
				     unsigned int frame_id,
				     struct isp_supplemental_sensor_mode_data
				     *data)
{
	struct rkisp1_img_src_data *last_data;
	struct rkisp1_img_src_data *new_data;

	mutex_lock(&dev->img_src_exps.mutex);
	if (dev->img_src_exps.data[0].v_frame_id <
	    dev->img_src_exps.data[1].v_frame_id) {
		last_data = &dev->img_src_exps.data[0];
		new_data = &dev->img_src_exps.data[1];
	} else {
		last_data = &dev->img_src_exps.data[1];
		new_data = &dev->img_src_exps.data[0];
	}

	if (frame_id >= new_data->v_frame_id) {
		memcpy(data,
		       &new_data->data,
		       sizeof(struct isp_supplemental_sensor_mode_data));
	} else {
		memcpy(data,
		       &last_data->data,
		       sizeof(struct isp_supplemental_sensor_mode_data));
	}
	mutex_unlock(&dev->img_src_exps.mutex);
}

static const struct cif_isp10_fmt cifisp_isp_output_formats[] = {
	{
	 .name = "YUV422-Interleaved",
	 .fourcc = V4L2_PIX_FMT_YUYV,
	 .mbus_code = MEDIA_BUS_FMT_YUYV8_1X16,
	 .cif_fmt = CIF_YUV422I,
	 .flags = 0,
	 .depth = 16,
	 .rotation = false,
	 .overlay = false,
	},
	{
	 .name = "BAYERRAW-SRGGB12",
	 .fourcc = V4L2_PIX_FMT_SRGGB12,
	 .mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
	 .cif_fmt = CIF_BAYER_SRGGB12,
	 .flags = 0,
	 .depth = 16,
	 .rotation = false,
	 .overlay = false,
	},
};

static const struct cif_isp10_fmt cifisp_isp_input_formats[] = {
	/* ************* YUV422 ************* */
	{
	 .name = "YUV422-Interleaved",
	 .fourcc = V4L2_PIX_FMT_YUYV,
	 .mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
	 .cif_fmt = CIF_YUV422I,
	 .flags = 0,
	 .depth = 16,
	 .rotation = false,
	 .overlay = false,
	},
	{
	 .name = "YVU422-Interleaved",
	 .fourcc = V4L2_PIX_FMT_UYVY,
	 .mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
	 .cif_fmt = CIF_UYV422I,
	 .flags = 0,
	 .depth = 16,
	 .rotation = false,
	 .overlay = false,
	},

	{
	 .name = "BAYERRAW-SRGGB10",
	 .fourcc = V4L2_PIX_FMT_SRGGB10,
	 .mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
	 .cif_fmt = CIF_BAYER_SRGGB10,
	 .flags = 0,
	 .depth = 16,
	 .rotation = false,
	 .overlay = false,
	},
};

static const struct cif_isp10_fmt *cifisp_isp_subdev_find_format(const u32 pad,
								 const u32 *
								 pixelfmt,
								 const u32 *
								 mbus_code,
								 const u32 *
								 ciffmt,
								 int index)
{
	const struct cif_isp10_fmt *fmt, *def_fmt = NULL, *array_fmt;
	unsigned int i, array_size;

	if ((pad != RKISP1_ISP_PAD_SINK) && (pad != RKISP1_ISP_PAD_SOURCE_PATH))
		return NULL;

	if (pad == RKISP1_ISP_PAD_SINK) {
		array_fmt = cifisp_isp_input_formats;
		array_size = ARRAY_SIZE(cifisp_isp_input_formats);
	} else {
		array_fmt = cifisp_isp_output_formats;
		array_size = ARRAY_SIZE(cifisp_isp_output_formats);
	}

	if (index >= (int)array_size)
		return NULL;

	for (i = 0; i < array_size; i++) {
		fmt = &array_fmt[i];
		if (pixelfmt && fmt->fourcc == *pixelfmt)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (ciffmt && fmt->cif_fmt == *ciffmt)
			return fmt;
		if (index == i)
			def_fmt = fmt;
	}

	return def_fmt;
}

static int
cifisp_isp_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct cif_isp10_fmt *fmt;

	fmt = cifisp_isp_subdev_find_format(code->pad, NULL, NULL, NULL,
					    code->index);
	if (!fmt)
		return -EINVAL;
	code->code = fmt->mbus_code;

	return 0;
}

static int cifisp_isp_subdev_get_fmt(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	struct cif_isp10_frm_fmt *input_fmt =
	    &isp_dev->isp_sdev.isp_config.input;
	struct cif_isp10_frm_fmt *output_fmt =
	    &isp_dev->isp_sdev.isp_config.output;
	const struct cif_isp10_fmt *cif_fmt;

	if ((fmt->pad != RKISP1_ISP_PAD_SINK) ||
	    (fmt->pad != RKISP1_ISP_PAD_SOURCE_PATH))
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	if (fmt->pad == RKISP1_ISP_PAD_SINK) {
		/* full camera input frame size */
		mf->width = input_fmt->width;
		mf->height = input_fmt->height;
		cif_fmt = cifisp_isp_subdev_find_format(fmt->pad, NULL, NULL,
							&input_fmt->pix_fmt,
							-1);
		if (!cif_fmt)
			return -EINVAL;
		if (RKISP1_PIX_FMT_IS_YUV(input_fmt->pix_fmt))
			mf->colorspace = V4L2_COLORSPACE_JPEG;
		else
			mf->colorspace = V4L2_COLORSPACE_SRGB;
		mf->code = cif_fmt->mbus_code;
	} else {
		/* crop size */
		mf->width = output_fmt->width;
		mf->height = output_fmt->height;
		cif_fmt = cifisp_isp_subdev_find_format(fmt->pad, NULL, NULL,
							&output_fmt->pix_fmt,
							-1);
		if (!cif_fmt)
			return -EINVAL;
		if (RKISP1_PIX_FMT_IS_YUV(input_fmt->pix_fmt))
			mf->colorspace = V4L2_COLORSPACE_JPEG;
		else
			mf->colorspace = V4L2_COLORSPACE_SRGB;
		mf->code = cif_fmt->mbus_code;
	}
	return 0;
}

static const struct cif_isp10_fmt *cifisp_isp_subdev_try_format(struct
								v4l2_subdev *sd, struct
								v4l2_subdev_pad_config
								*cfg,
								unsigned int
								pad, struct
								v4l2_mbus_framefmt
								*fmt, enum
								v4l2_subdev_format_whence
								which)
{
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	struct cif_isp10_frm_fmt *output_fmt =
	    &isp_dev->isp_sdev.isp_config.output;
	const struct cif_isp10_fmt *cif_fmt = NULL;

	cif_fmt = cifisp_isp_subdev_find_format(pad, NULL, &fmt->code, NULL,
						-1);
	switch (pad) {
	case RKISP1_ISP_PAD_SINK:
		if (cif_fmt)
			fmt->code = cif_fmt->mbus_code;
		else
			fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
		fmt->width = clamp_t(u32, fmt->width,
				     CIF_ISP_INPUT_WIDTH_MIN,
				     CIF_ISP_INPUT_WIDTH_MAX);
		fmt->height = clamp_t(u32, fmt->height,
				      CIF_ISP_INPUT_HEIGHT_MIN,
				      CIF_ISP_INPUT_HEIGHT_MAX);
		break;
	case RKISP1_ISP_PAD_SOURCE_PATH:
		if (cif_fmt)
			fmt->code = cif_fmt->mbus_code;
		else
			fmt->code = MEDIA_BUS_FMT_YUYV8_1X16;
		/* Hardcode the output size to the crop rectangle size. */
		fmt->width = output_fmt->defrect.width;
		fmt->height = output_fmt->defrect.height;
		break;
	default:
		break;
	}

	if (cif_fmt && RKISP1_PIX_FMT_IS_RAW_BAYER(cif_fmt->cif_fmt))
		fmt->colorspace = V4L2_COLORSPACE_SRGB;
	else
		fmt->colorspace = V4L2_COLORSPACE_JPEG;
	fmt->field = V4L2_FIELD_NONE;
	return cif_fmt;
}

static int cifisp_isp_subdev_set_fmt(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	struct cif_isp10_frm_fmt *input_fmt =
	    &isp_dev->isp_sdev.isp_config.input;
	struct cif_isp10_frm_fmt *output_fmt =
	    &isp_dev->isp_sdev.isp_config.output;
	const struct cif_isp10_fmt *cif_fmt;

	if ((fmt->pad != RKISP1_ISP_PAD_SINK) ||
	    (fmt->pad != RKISP1_ISP_PAD_SOURCE_PATH))
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	cif_fmt = cifisp_isp_subdev_try_format(sd, cfg, fmt->pad, mf,
					       fmt->which);

	if (fmt->pad == RKISP1_ISP_PAD_SINK) {
		input_fmt->width = mf->width;
		input_fmt->height = mf->height;
		/* TODO: 
		 * not set crop size here ? 
		 * input crop size should be from sensor DTS ?
		 */
		//input_fmt->defrect.left = 0;
		//input_fmt->defrect.top = 0;
		//input_fmt->defrect.width = input_fmt->width;
		//input_fmt->defrect.height = input_fmt->height;
		input_fmt->pix_fmt = cif_fmt->cif_fmt;
		//TODO
		//input_fmt->quantization
		//propagate to source
		output_fmt->width = input_fmt->defrect.width;
		output_fmt->height = input_fmt->defrect.height;
		output_fmt->defrect.left = 0;
		output_fmt->defrect.top = 0;
		output_fmt->defrect.width = output_fmt->width;
		output_fmt->defrect.height = output_fmt->height;
		output_fmt->pix_fmt = input_fmt->pix_fmt;
	} else {
		output_fmt->pix_fmt = cif_fmt->cif_fmt;
	}
	return 0;
}

static int cifisp_stream_subdev_try_crop(struct v4l2_subdev *sd,
					 struct v4l2_rect *input, u32 target)
{
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	struct cif_isp10_frm_fmt *input_fmt =
	    &isp_dev->isp_sdev.isp_config.input;
	struct cif_isp10_frm_fmt *output_fmt =
	    &isp_dev->isp_sdev.isp_config.output;
	/* TODO :
	 *  should implement V4L2_SEL_TGT_CROP for pad RKISP1_ISP_PAD_SINK ?
	 */
	if (target == V4L2_SEL_TGT_CROP) {
		//crop
		input->left = clamp_t(u32, input->left, 0,
				      input_fmt->defrect.width -
				      CIF_ISP_OUTPUT_WIDTH_MIN);
		input->top = clamp_t(u32, input->top, 0,
				     input_fmt->defrect.height -
				     CIF_ISP_OUTPUT_HEIGHT_MIN);
		input->width = clamp_t(u32, input->width,
				       CIF_ISP_OUTPUT_WIDTH_MIN,
				       input_fmt->defrect.width - input->left);
		input->height = clamp_t(u32, input->height,
					CIF_ISP_INPUT_HEIGHT_MIN,
					input_fmt->defrect.height - input->top);
		//TODO: should deal with align ?
		output_fmt->width = input->width;
		output_fmt->height = input->height;
		output_fmt->defrect.left = input->left;
		output_fmt->defrect.top = input->top;
		output_fmt->defrect.width = input->width;
		output_fmt->defrect.height = input->height;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int
cifisp_isp_subdev_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	struct cif_isp10_frm_fmt *input_fmt =
	    &isp_dev->isp_sdev.isp_config.input;
	struct cif_isp10_frm_fmt *output_fmt =
	    &isp_dev->isp_sdev.isp_config.output;

	if (sel->pad != RKISP1_ISP_PAD_SOURCE_PATH)
		return -EINVAL;
	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		//TODO
		//v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		return 0;
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.width = input_fmt->defrect.width;
		sel->r.height = input_fmt->defrect.height;
		sel->r.left = 0;
		sel->r.top = 0;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = output_fmt->defrect;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int
cifisp_isp_subdev_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	/*  TODO: 
	 *  should implement V4L2_SEL_TGT_CROP for pad RKISP1_ISP_PAD_SINK ?
	 */
	if (sel->target != V4L2_SEL_TGT_CROP ||
	    sel->pad != RKISP1_ISP_PAD_SOURCE_PATH)
		return -EINVAL;

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		//TODO
		//v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		return 0;
	}
	return cifisp_stream_subdev_try_crop(sd, &sel->r, sel->target);
}

static int cifisp_isp_subdev_s_stream(struct v4l2_subdev *sd, int on)
{
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	int ret = 0;

	if (on) {
		ret = rkisp1_config_cif(isp_dev);
		if (ret < 0)
			return ret;

		ret = cif_isp10_start(isp_dev);
		if (ret < 0)
			return ret;
	} else {
		ret = cif_isp10_stop(isp_dev);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int cifisp_isp_subdev_s_power(struct v4l2_subdev *sd, int on)
{
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	int ret = 0;

	if (on) {
		v4l2_info(sd, "streaming count %d", dev->cif_streamon_cnt);

		ret = pm_runtime_get_sync(dev->dev);
		if (ret < 0)
			return ret;

		/*
		 * Cancel isp reset internal here temporary for
		 * isp bus may be dead when switch isp.
		 */
		/*
		 * iowrite32(CIF_IRCL_CIF_SW_RST,
		 * dev->config.base_addr + CIF_IRCL);
		 */

		rkisp1_config_clk(dev);

	} else {
		if (dev->cif_streamon_cnt == 0) {
			ret = pm_runtime_put(dev->dev);
			if (ret < 0)
				return ret;
		} else {
			v4l2_warn(sd,
				  "%d path is streaming, ignore the PM off request.",
				  dev->cif_streamon_cnt);
		}
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops cifisp_isp_subdev_pad_ops = {
	.enum_mbus_code = cifisp_isp_subdev_enum_mbus_code,
	.get_selection = cifisp_isp_subdev_get_selection,
	.set_selection = cifisp_isp_subdev_set_selection,
	.get_fmt = cifisp_isp_subdev_get_fmt,
	.set_fmt = cifisp_isp_subdev_set_fmt,
};

static const struct v4l2_subdev_video_ops cifisp_isp_subdev_video_ops = {
	.s_stream = cifisp_isp_subdev_s_stream,
};

static const struct v4l2_subdev_core_ops cifisp_isp_core_ops = {
	//.log_status = ,
	.s_power = cifisp_isp_subdev_s_power,
};

static struct v4l2_subdev_ops cifisp_isp_subdev_ops = {
	.core = &cifisp_isp_core_ops,
	.video = &cifisp_isp_subdev_video_ops,
	.pad = &cifisp_isp_subdev_pad_ops,
};

/* called when link up */
static int
subdev_get_camitf_info(struct rkisp1_device *isp_dev,
		       struct v4l2_subdev *subdev)
{
	v4l2_info(subdev, "subdev %p, %p, id: %d, %d,name %s, %s",
		  subdev, isp_dev->subdevs[RKISP1_SD_SENSOR], subdev->grp_id,
		  isp_dev->subdevs[RKISP1_SD_SENSOR]->grp_id, subdev->name,
		  isp_dev->subdevs[RKISP1_SD_SENSOR]->name);
	if (isp_dev->subdevs[RKISP1_SD_SENSOR] != subdev)
		return -EINVAL;

	/* TODO: get camitf from sensor */
	if (PLTFRM_CAM_ITF_IS_MIPI(isp_dev->config.cam_itf.type)) {
		isp_dev->config.cam_itf.cfg.mipi.dphy_index = 0;
		isp_dev->config.cam_itf.cfg.mipi.nb_lanes = 2;
		isp_dev->config.cam_itf.cfg.mipi.bit_rate = 1000;
		isp_dev->config.cam_itf.cfg.mipi.vc = 0;
	}

	return 0;
}

/* usually called by IOCTL MEDIA_IOC_SETUP_LINK */
static int cifisp_isp_link_setup(struct media_entity *entity,
				 const struct media_pad *local,
				 const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);

	v4l2_info(sd, "local name %s, remote name %s, falgs %u\n",
		  local->entity->name, remote->entity->name, flags);
	/* TODO: get cam_itf.type from sensor */
	if (local->index == RKISP1_ISP_PAD_SINK) {
		if (!(flags & MEDIA_LNK_FL_ENABLED)) {
			isp_dev->config.input_sel = 0;
			isp_dev->config.cam_itf.type = 0;
			return 0;
		}
		v4l2_info(sd, "cam_itf.type %d, isp input type 0x%x\n",
			  isp_dev->config.cam_itf.type,
			  isp_dev->config.input_sel);
		if (isp_dev->config.cam_itf.type != 0)
			return -EBUSY;

		if (media_entity_type(remote->entity) ==
		    MEDIA_ENT_T_V4L2_SUBDEV) {
			struct media_pad *csi_pad_src;
			struct media_entity *sensor_entity;

			//mipi
			isp_dev->config.input_sel = RKISP1_INP_CSI;
			isp_dev->config.cam_itf.type = PLTFRM_CAM_ITF_MIPI;
			//get sensor entity linked to csi
			csi_pad_src = &remote->entity->pads[MIPIPHY_PAD_SINK];
			sensor_entity =
			    media_entity_remote_pad(csi_pad_src)->entity;
			subdev_get_camitf_info(isp_dev,
					       media_entity_to_v4l2_subdev
					       (sensor_entity));
		} else if (media_entity_type(remote->entity) ==
			   MEDIA_ENT_T_V4L2_SUBDEV_SENSOR) {
			//parallel
			isp_dev->config.input_sel = RKISP1_INP_CPI;
			isp_dev->config.cam_itf.type = PLTFRM_CAM_ITF_BT601_8;
		} else {
			isp_dev->config.cam_itf.type = 0;
			// DMA
			if (isp_dev->path_sdevs.sp_config.input_sel !=
			    RKISP1_SP_INP_DMA_SP)
				isp_dev->config.input_sel = RKISP1_INP_DMA;
			else
				return -EBUSY;
		}
	}

	return 0;
}

static const struct media_entity_operations cifisp_isp_subdev_media_ops = {
	.link_setup = cifisp_isp_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static void cifisp_isp_sd_init_default_fmt(struct rkisp1_device *isp_dev)
{
	struct cif_isp10_frm_fmt *input_fmt =
	    &isp_dev->isp_sdev.isp_config.input;
	struct cif_isp10_frm_fmt *output_fmt =
	    &isp_dev->isp_sdev.isp_config.output;

	input_fmt->width = 640;
	input_fmt->height = 480;
	input_fmt->defrect.left = 0;
	input_fmt->defrect.top = 0;
	input_fmt->defrect.width = input_fmt->width;
	input_fmt->defrect.height = input_fmt->height;
	input_fmt->pix_fmt = CIF_BAYER_SBGGR8;
	//TODO
	//input_fmt->quantization
	//propagate to source
	output_fmt->width = input_fmt->defrect.width;
	output_fmt->height = input_fmt->defrect.height;
	output_fmt->defrect.left = 0;
	output_fmt->defrect.top = 0;
	output_fmt->defrect.width = output_fmt->width;
	output_fmt->defrect.height = output_fmt->height;
	output_fmt->pix_fmt = CIF_YUV422I;
}

int register_cifisp_isp_subdev(struct rkisp1_device *isp_dev,
			       struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &isp_dev->isp_sdev.isp_sub_dev;
	struct v4l2_ctrl_handler *handler = &isp_dev->isp_sdev.ctrl_handler;
	int ret;

	v4l2_subdev_init(sd, &cifisp_isp_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "cif-isp10-subdev-isp");

	isp_dev->isp_sdev.pads[RKISP1_ISP_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	isp_dev->isp_sdev.pads[RKISP1_ISP_PAD_SOURCE_PATH].flags =
	    MEDIA_PAD_FL_SOURCE;
	isp_dev->isp_sdev.pads[RKISP1_ISP_PAD_SOURCE_STATS].flags =
	    MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, RKISP1_ISP_PAD_MAX,
				isp_dev->isp_sdev.pads, 0);
	if (ret)
		return ret;

	//TODO: setup ctrls, eg. 3A controls
	v4l2_ctrl_handler_init(handler, 1);

	if (handler->error) {
		ret = handler->error;
		goto err_cleanup_media_entity;
	}

	sd->ctrl_handler = handler;
	sd->entity.ops = &cifisp_isp_subdev_media_ops;
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, isp_dev);

	//group id?
	sd->grp_id = GRP_ID_ISP;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		v4l2_err(sd, "Failed to register isp subdev\n");
		goto err_free_ctrl_handler;
	}

	cifisp_isp_sd_init_default_fmt(isp_dev);

	return 0;
err_free_ctrl_handler:
	v4l2_ctrl_handler_free(handler);
	v4l2_set_subdevdata(sd, NULL);
err_cleanup_media_entity:
	media_entity_cleanup(&sd->entity);
	return ret;
}

void unregister_cifisp_isp_subdev(struct rkisp1_device *isp_dev)
{
	struct v4l2_subdev *sd = &isp_dev->isp_sdev.isp_sub_dev;

	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&isp_dev->isp_sdev.ctrl_handler);
	v4l2_set_subdevdata(sd, NULL);
}

enum {
	isp_data_loss = 0,
	isp_pic_size_err,
	mipi_fifo_err,
	dphy_err_sot,
	dphy_err_sot_sync,
	dphy_err_eot_sync,
	dphy_err_ctrl,
	csi_err_protocol,
	csi_ecc1_err,
	csi_ecc2_err,
	csi_cs_err,
};

static void cif_isp10_hw_restart(struct rkisp1_device *dev)
{
	u32 d;
	void *a;

	cif_isp10_hw_errors[isp_pic_size_err].count = 0;
	cif_isp10_hw_errors[isp_data_loss].count = 0;
	cif_isp10_hw_errors[csi_err_protocol].count = 0;
	cif_isp10_hw_errors[csi_ecc1_err].count = 0;
	cif_isp10_hw_errors[csi_ecc2_err].count = 0;
	cif_isp10_hw_errors[csi_cs_err].count = 0;
	iowrite32(0x00000841, dev->config.base_addr + CIF_IRCL);
	iowrite32(0x0, dev->config.base_addr + CIF_IRCL);

	/* enable mipi frame end interrupt */
	iowrite32(CIF_MIPI_FRAME_END, dev->config.base_addr + CIF_MIPI_IMSC);
	/* enable csi protocol errors interrupts */
	d = CIF_MIPI_ERR_CSI;
	a = dev->config.base_addr + CIF_MIPI_IMSC;
	iowrite32(ioread32(a) | d, a);
	/* enable dphy errors interrupts */
	d = CIF_MIPI_ERR_DPHY;
	a = dev->config.base_addr + CIF_MIPI_IMSC;
	iowrite32(ioread32(a) | d, a);
	/* add fifo error */
	d = CIF_MIPI_SYNC_FIFO_OVFLW(3);
	a = dev->config.base_addr + CIF_MIPI_IMSC;
	iowrite32(ioread32(a) | d, a);
	/* add data overflow_error */
	d = CIF_MIPI_ADD_DATA_OVFLW;
	a = dev->config.base_addr + CIF_MIPI_IMSC;
	iowrite32(ioread32(a) | d, a);
	iowrite32(0x0, dev->config.base_addr + CIF_MI_MP_Y_OFFS_CNT_INIT);
	iowrite32(0x0, dev->config.base_addr + CIF_MI_MP_CR_OFFS_CNT_INIT);
	iowrite32(0x0, dev->config.base_addr + CIF_MI_MP_CB_OFFS_CNT_INIT);
	iowrite32(0x0, dev->config.base_addr + CIF_MI_SP_Y_OFFS_CNT_INIT);
	iowrite32(0x0, dev->config.base_addr + CIF_MI_SP_CR_OFFS_CNT_INIT);
	iowrite32(0x0, dev->config.base_addr + CIF_MI_SP_CB_OFFS_CNT_INIT);
	d = CIF_MI_CTRL_INIT_OFFSET_EN;
	a = dev->config.base_addr + CIF_MI_CTRL;
	iowrite32(ioread32(a) | d, a);
	/* Enable ISP ! */
	d = CIF_ISP_CTRL_ISP_CFG_UPD |
		CIF_ISP_CTRL_ISP_INFORM_ENABLE |
		CIF_ISP_CTRL_ISP_ENABLE;
	a = dev->config.base_addr + CIF_ISP_CTRL;
	iowrite32(ioread32(a) | d, a);
	/* enable MIPI */
	d = CIF_MIPI_CTRL_OUTPUT_ENA;
	a = dev->config.base_addr + CIF_MIPI_CTRL;
	iowrite32(ioread32(a) | d, a);
}

void cif_isp10_mipi_isr(unsigned int mipi_mis, void *cntxt)
{
	struct rkisp1_device *dev = (struct rkisp1_device *)cntxt;
	unsigned int mipi_ris, d;
	void *a;

	mipi_ris = ioread32(dev->config.base_addr + CIF_MIPI_RIS);
	mipi_mis = ioread32(dev->config.base_addr + CIF_MIPI_MIS);

	iowrite32(~0, dev->config.base_addr + CIF_MIPI_ICR);

	if (mipi_mis & CIF_MIPI_ERR_DPHY) {
		v4l2_warn(&dev->v4l2_dev, "CIF_MIPI_ERR_DPHY: 0x%x\n",
			  mipi_mis);

		/*
		 * Disable DPHY errctrl interrupt, because this dphy
		 * erctrl signal is assert and until the next changes
		 * in line state. This time is may be too long and cpu
		 * is hold in this interrupt.
		 */
		if (mipi_mis & CIF_MIPI_ERR_CTRL(3)){
			d = (u32)~(CIF_MIPI_ERR_CTRL(3));
			a = dev->config.base_addr + CIF_MIPI_IMSC;
			iowrite32(ioread32(a) & d, a);
		}
	}

	if (mipi_mis & CIF_MIPI_ERR_CSI) {
		v4l2_warn(&dev->v4l2_dev, "CIF_MIPI_ERR_CSI: 0x%x\n", mipi_mis);
	}

	if (mipi_mis & CIF_MIPI_SYNC_FIFO_OVFLW(3)) {
		v4l2_warn(&dev->v4l2_dev, "CIF_MIPI_SYNC_FIFO_OVFLW:0x%x\n",
			  mipi_mis);
	}

	if (mipi_mis == CIF_MIPI_FRAME_END) {
		/*
		 * Enable DPHY errctrl interrupt again, if mipi have receive
		 * the whole frame without any error.
		 */
		d = CIF_MIPI_ERR_CTRL(3);
		a = dev->config.base_addr + CIF_MIPI_IMSC;
		iowrite32(ioread32(a) | d, a);
	}

	mipi_mis = ioread32(dev->config.base_addr + CIF_MIPI_MIS);
	if (mipi_mis)
		v4l2_err(&dev->v4l2_dev, "mipi_mis icr err: 0x%x\n", mipi_mis);
}

/* ======================================================================== */
void cif_isp10_isp_isr(unsigned int isp_mis, void *cntxt)
{
	struct rkisp1_device *dev = (struct rkisp1_device *)cntxt;
	unsigned int isp_mis_tmp;
	unsigned int isp_err, d;
	struct timeval tv;
	void *a;

	if (isp_mis & CIF_ISP_V_START) {
		do_gettimeofday(&tv);
		dev->b_mi_frame_end = false;
		cifisp_v_start(&dev->isp_sdev.isp_dev, &tv);

		iowrite32(CIF_ISP_V_START, dev->config.base_addr + CIF_ISP_ICR);
		isp_mis_tmp = ioread32(dev->config.base_addr + CIF_ISP_MIS);
		if (isp_mis_tmp & CIF_ISP_V_START)
			v4l2_err(&dev->v4l2_dev, "isp icr v_statr err: 0x%x\n",
				 isp_mis_tmp);

		if (!dev->strm_vdevs.mi_config.async_updt){
			d = CIF_ISP_CTRL_ISP_GEN_CFG_UPD;
			a = dev->config.base_addr + CIF_ISP_CTRL;
			iowrite32(ioread32(a) | d, a);
		}

		if (dev->sof_event)
			dev->sof_event(dev,
				       dev->isp_sdev.isp_dev.frame_id >> 1);
	}

	if (isp_mis & CIF_ISP_FRAME_IN) {
		do_gettimeofday(&tv);
		iowrite32(CIF_ISP_FRAME_IN,
			  dev->config.base_addr + CIF_ISP_ICR);
		cifisp_frame_in(&dev->isp_sdev.isp_dev, &tv);
	}

	if ((isp_mis & (CIF_ISP_DATA_LOSS | CIF_ISP_PIC_SIZE_ERROR))) {
		dev->strm_vdevs.sp_vdev.stall = true;
		dev->strm_vdevs.mp_vdev.stall = true;

		if ((isp_mis & CIF_ISP_PIC_SIZE_ERROR)) {
			/* Clear pic_size_error */
			iowrite32(CIF_ISP_PIC_SIZE_ERROR,
				  dev->config.base_addr + CIF_ISP_ICR);
			cif_isp10_hw_errors[isp_pic_size_err].count++;
			isp_err = ioread32(dev->config.base_addr + CIF_ISP_ERR);
			v4l2_err(&dev->v4l2_dev,
				 "CIF_ISP_PIC_SIZE_ERROR (0x%08x)", isp_err);
			iowrite32(isp_err,
				  dev->config.base_addr + CIF_ISP_ERR_CLR);
		} else if ((isp_mis & CIF_ISP_DATA_LOSS)) {
			/* Clear data_loss */
			iowrite32(CIF_ISP_DATA_LOSS,
				  dev->config.base_addr + CIF_ISP_ICR);
			cif_isp10_hw_errors[isp_data_loss].count++;
			v4l2_err(&dev->v4l2_dev, "CIF_ISP_DATA_LOSS\n");
			iowrite32(CIF_ISP_DATA_LOSS,
				  dev->config.base_addr + CIF_ISP_ICR);
		}

		/* Stop ISP */
		d = (u32)~CIF_ISP_CTRL_ISP_INFORM_ENABLE &
				 ~CIF_ISP_CTRL_ISP_ENABLE;
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) & d, a);
		/* isp_update */
		d = CIF_ISP_CTRL_ISP_CFG_UPD;
		a = dev->config.base_addr + CIF_ISP_CTRL;
		iowrite32(ioread32(a) | d, a);
		cif_isp10_hw_restart(dev);
	}

	if (isp_mis & CIF_ISP_FRAME_IN) {
		iowrite32(CIF_ISP_FRAME_IN,
			  dev->config.base_addr + CIF_ISP_ICR);
		isp_mis_tmp = ioread32(dev->config.base_addr + CIF_ISP_MIS);
		if (isp_mis_tmp & CIF_ISP_FRAME_IN)
			v4l2_err(&dev->v4l2_dev, "isp icr frame_in err: 0x%x\n",
				 isp_mis_tmp);

		/* restart MI if CIF has run out of buffers */
		if (!RKISP1_MI_IS_BUSY(dev) &&
		    (dev->strm_vdevs.mi_config.async_updt ||
		     (!dev->strm_vdevs.sp_vdev.next_buf &&
		      !dev->strm_vdevs.mp_vdev.next_buf))) {
			u32 mi_isr = 0;

			if (dev->strm_vdevs.sp_vdev.state ==
			    RKISP1_STATE_STREAMING)
				mi_isr |= CIF_MI_SP_FRAME;
			if (dev->strm_vdevs.mp_vdev.state ==
			    RKISP1_STATE_STREAMING)
				mi_isr |= CIF_MI_MP_FRAME;
			iowrite32(mi_isr, dev->config.base_addr + CIF_MI_ISR);
		}
	}

	if (isp_mis & CIF_ISP_FRAME) {
		/* Clear Frame In (ISP) */
		iowrite32(CIF_ISP_FRAME, dev->config.base_addr + CIF_ISP_ICR);
		isp_mis_tmp = ioread32(dev->config.base_addr + CIF_ISP_MIS);
		if (isp_mis_tmp & CIF_ISP_FRAME)
			v4l2_err(&dev->v4l2_dev,
				 "isp icr frame end err: 0x%x\n", isp_mis_tmp);

		if (dev->b_mi_frame_end)
			cif_isp10_update_ism_dcr_rsz(dev);
	}

	cifisp_isp_isr(&dev->isp_sdev.isp_dev, isp_mis);
}
