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

/* TODO: define the isp frame size constrains */
#define CIF_ISP_INPUT_W_MAX	 	4032
#define CIF_ISP_INPUT_H_MAX 		3024
#define CIF_ISP_INPUT_W_MIN		32
#define CIF_ISP_INPUT_H_MIN		32
#define CIF_ISP_OUTPUT_W_MAX	 	CIF_ISP_INPUT_W_MAX
#define CIF_ISP_OUTPUT_H_MAX 		CIF_ISP_INPUT_H_MAX
#define CIF_ISP_OUTPUT_W_MIN		CIF_ISP_INPUT_W_MIN
#define CIF_ISP_OUTPUT_H_MIN		CIF_ISP_INPUT_H_MIN

#define CIF_ISP_DEFAULT_W		800
#define CIF_ISP_DEFAULT_H		600

#define writel_and(val, addr) \
		writel((val) & readl(addr), (addr))

#define writel_or(val, addr) \
		writel((val) | readl(addr), (addr))
/*
 * This should only be called when configuring CIF
 * or at the frame end interrupt
 */
static void rkisp1_config_ism(struct rkisp1_device *dev)
{
	void __iomem *base = dev->config.base_addr;
	struct v4l2_rect *win = &dev->isp_sdev.out_win;

	writel(0, base + CIF_ISP_IS_RECENTER);
	writel(0, base + CIF_ISP_IS_MAX_DX);
	writel(0, base + CIF_ISP_IS_MAX_DY);
	writel(0, base + CIF_ISP_IS_DISPLACE);
	writel(win->left, base + CIF_ISP_IS_H_OFFS);
	writel(win->top, base + CIF_ISP_IS_V_OFFS);
	writel(win->width, base + CIF_ISP_IS_H_SIZE);
	writel(win->height, base + CIF_ISP_IS_V_SIZE);

	/* IS(Image Stabilization) is always off, working as window selection */
	writel(0, base + CIF_ISP_IS_CTRL);

	writel_or(CIF_ISP_CTRL_ISP_CFG_UPD, base + CIF_ISP_CTRL);
}

static int rkisp1_config_isp(struct rkisp1_device *dev)
{
	u32 yuv_seq = 0;
	u32 isp_input_sel = 0;
	u32 isp_bayer_pat = 0;
	u32 acq_mult = 1;
	u32 irq_mask = 0;
	u32 signal = 0;
	u32 isp_ctrl;
	struct cif_fmt_info *in_fmt, *out_fmt;
	struct pltfrm_cam_itf *cam_itf;
	struct v4l2_rect *in_acqui, *out_win;
	void __iomem *base = dev->config.base_addr;

	in_fmt = &dev->isp_sdev.in_fmt;
	out_fmt = &dev->isp_sdev.out_fmt;
	cam_itf = &dev->config.cam_itf;
	in_acqui = &dev->isp_sdev.in_acqui;
	out_win = &dev->isp_sdev.out_win;

	writel_or(CIF_ICCL_ISP_CLK, base + CIF_ICCL);

	if (CIF_PIX_FMT_IS_RAW_BAYER(in_fmt)) {
		if (CIF_PIX_FMT_IS_RAW_BAYER(out_fmt)) {
			if (PLTFRM_CAM_ITF_IS_BT656(dev->config.cam_itf.type))
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_RAW_PICT_ITU656;
			else
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_RAW_PICT;
		} else {
			writel(0xc, base + CIF_ISP_DEMOSAIC);

			if (PLTFRM_CAM_ITF_IS_BT656(dev->config.cam_itf.type))
				    isp_ctrl = CIF_ISP_CTRL_ISP_MODE_BAYER_ITU656;
			else
				    isp_ctrl = CIF_ISP_CTRL_ISP_MODE_BAYER_ITU601;
		}

		switch (in_fmt->bpp) {
		case 8:
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_8B_MSB;
			break;
		case 10:
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_10B_MSB;
			break;
		case 12:
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
			break;
		default:
			v4l2_err(&dev->v4l2_dev, "invalid bpp(%d)\n",
				 in_fmt->bpp);
			return -EINVAL;
		}

		switch(in_fmt->bayer_pat) {
		case CIF_FMT_RAW_PAT_TYPE_BGGR:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_BGGR;
			break;
		case CIF_FMT_RAW_PAT_TYPE_GBRG:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GBRG;
			break;
		case CIF_FMT_RAW_PAT_TYPE_GRBG:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GRBG;
			break;
		case CIF_FMT_RAW_PAT_TYPE_RGGB:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_RGGB;
			break;
		}
	} else if (CIF_PIX_FMT_IS_YUV(in_fmt)) {
		acq_mult = 2;
		if (PLTFRM_CAM_ITF_IS_MIPI(dev->config.cam_itf.type)) {
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
			isp_ctrl = CIF_ISP_CTRL_ISP_MODE_ITU601;
		} else if (PLTFRM_CAM_ITF_IS_DVP(dev->config.cam_itf.type)) {
			if (PLTFRM_CAM_ITF_IS_BT656(dev->config.cam_itf.type))
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_ITU656;
			else
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_ITU601;

			switch (PLTFRM_CAM_ITF_DVP_BW(dev->config.cam_itf.type)) {
			case 8:
				isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_8B_ZERO;
				break;
			case 10:
				isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_10B_ZERO;
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
		}

		irq_mask |= CIF_ISP_DATA_LOSS;

		if (in_fmt->yc_swap) {
			yuv_seq = CIF_ISP_ACQ_PROP_CBYCRY;
			//TODO: what is this case for output
			//cif_isp10_pix_fmt_set_yc_swapped(output->pix_fmt, 0);
		} else if (in_fmt->uv_swap) {
			yuv_seq = CIF_ISP_ACQ_PROP_YCRYCB;
		} else {
			yuv_seq = CIF_ISP_ACQ_PROP_YCBYCR;
		}
	}

	/* Set up input acquisition properties */
	if (PLTFRM_CAM_ITF_IS_DVP(cam_itf->type)) {
		struct pltfrm_cam_dvp_config *dvp = &cam_itf->cfg.dvp;

		if (dvp->pclk == PLTFRM_CAM_SDR_POS_EDG)
			signal = CIF_ISP_ACQ_PROP_POS_EDGE;
		else
			signal = CIF_ISP_ACQ_PROP_NEG_EDGE;

		if (PLTFRM_CAM_ITF_IS_BT601(cam_itf->type)) {
			if (dvp->vsync == PLTFRM_CAM_SIGNAL_HIGH_LEVEL)
				signal |= CIF_ISP_ACQ_PROP_VSYNC_HIGH;
			else
				signal |= CIF_ISP_ACQ_PROP_VSYNC_LOW;

			if (dvp->hsync == PLTFRM_CAM_SIGNAL_HIGH_LEVEL)
				signal |= CIF_ISP_ACQ_PROP_HSYNC_HIGH;
			else
				signal |= CIF_ISP_ACQ_PROP_HSYNC_LOW;

		} else {
			signal |= CIF_ISP_ACQ_PROP_HSYNC_HIGH;
			signal |= CIF_ISP_ACQ_PROP_VSYNC_HIGH;
		}
	} else {
		signal = CIF_ISP_ACQ_PROP_NEG_EDGE;
		signal |= CIF_ISP_ACQ_PROP_HSYNC_HIGH;
		signal |= CIF_ISP_ACQ_PROP_VSYNC_HIGH;
	}

	writel(isp_ctrl, base + CIF_ISP_CTRL);
	writel(signal | yuv_seq | isp_input_sel | isp_bayer_pat |
		  CIF_ISP_ACQ_PROP_FIELD_SEL_ALL, base + CIF_ISP_ACQ_PROP);
	writel(0, base + CIF_ISP_ACQ_NR_FRAMES);

	/* Acquisition Size */
	writel(acq_mult * in_acqui->left, base + CIF_ISP_ACQ_H_OFFS);
	writel(in_acqui->top, base + CIF_ISP_ACQ_V_OFFS);
	writel(acq_mult * in_acqui->width, base + CIF_ISP_ACQ_H_SIZE);
	writel(in_acqui->height, base + CIF_ISP_ACQ_V_SIZE);

	/* TODO: Acquisition area - black level = out area*/
	writel(in_acqui->top, base + CIF_ISP_OUT_V_OFFS);
	writel(in_acqui->left, base + CIF_ISP_OUT_H_OFFS);
	writel(in_acqui->width, base + CIF_ISP_OUT_H_SIZE);
	writel(in_acqui->height, base + CIF_ISP_OUT_V_SIZE);

	/* interrupt mask */
	irq_mask |= CIF_ISP_FRAME | CIF_ISP_PIC_SIZE_ERROR |
		    CIF_ISP_FRAME_IN | CIF_ISP_V_START;
	writel(irq_mask, base + CIF_ISP_IMSC);

	if (CIF_PIX_FMT_IS_RAW_BAYER(out_fmt))
		cifisp_disable_isp(&dev->isp_sdev.isp_dev);
	else
		cifisp_configure_isp(&dev->isp_sdev.isp_dev, in_fmt,
				     V4L2_QUANTIZATION_FULL_RANGE);

	return 0;
}

static int rkisp1_config_mipi(struct rkisp1_device *dev)
{
	int i;
	u32 data_type, mipi_ctrl, shutdown_lanes = 0;
	struct cif_fmt_info *in_fmt;
	struct pltfrm_cam_mipi_config *mipi_conf;

	writel_or(CIF_ICCL_MIPI_CLK, dev->config.base_addr + CIF_ICCL);

	in_fmt = &dev->isp_sdev.in_fmt;
	mipi_conf = &dev->config.cam_itf.cfg.mipi;

	v4l2_info(&dev->v4l2_dev, "input %d, vc = %d, nb_lanes = %d\n",
		  dev->config.input_sel, mipi_conf->vc, mipi_conf->nb_lanes);

	if (mipi_conf->nb_lanes == 0 || mipi_conf->nb_lanes > 4) {
		v4l2_err(&dev->v4l2_dev, "MIPI lanes(%d) out of range[1..4]\n",
			 mipi_conf->nb_lanes);
		return -EINVAL;
	}

	for (i = 0; i < mipi_conf->nb_lanes; i++)
		shutdown_lanes |= (1 << i);

	mipi_ctrl = CIF_MIPI_CTRL_NUM_LANES(mipi_conf->nb_lanes - 1) |
		    CIF_MIPI_CTRL_ERR_SOT_HS_ENA |
		    CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_SKIP |
		    CIF_MIPI_CTRL_SHUTDOWNLANES(0xf) |
		    CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_ENA |
		    CIF_MIPI_CTRL_ERR_SOT_HS_ENA | CIF_MIPI_CTRL_CLOCKLANE_ENA;

	writel(mipi_ctrl, dev->config.base_addr + CIF_MIPI_CTRL);

	/* TODO: shutdown lanes
	writel(mipi_ctrl | CIF_MIPI_CTRL_SHUTDOWNLANES(0xf),
		      dev->config.base_addr + CIF_MIPI_CTRL);
	*/

	/* Configure Data Type and Virtual Channel */
	// TODO: enumerate all supported mbus code instead
	if (CIF_PIX_FMT_IS_YUV(in_fmt)) {
		if (in_fmt->xsubs == 2 && in_fmt->ysubs == 2 &&
		    in_fmt->bpp == 12)
			data_type = CIF_CSI2_DT_YUV420_8b;
		else if (in_fmt->xsubs == 2 && in_fmt->ysubs == 2 &&
			 in_fmt->bpp == 15)
			data_type = CIF_CSI2_DT_YUV420_10b;
		else if (in_fmt->xsubs == 2 && in_fmt->ysubs == 4 &&
			 in_fmt->bpp == 16)
			data_type = CIF_CSI2_DT_YUV422_8b;
		else if (in_fmt->xsubs == 2 && in_fmt->ysubs == 4 &&
			 in_fmt->bpp == 20)
			data_type = CIF_CSI2_DT_YUV422_10b;
	} else if (CIF_PIX_FMT_IS_RAW_BAYER(in_fmt)) {
		if (in_fmt->bpp == 8)
			data_type = CIF_CSI2_DT_RAW8;
		else if (in_fmt->bpp == 10)
			data_type = CIF_CSI2_DT_RAW10;
		else if (in_fmt->bpp == 12)
			data_type = CIF_CSI2_DT_RAW12;
	} else if (in_fmt->mbus_code == MEDIA_BUS_FMT_RGB565_1X16) {
		data_type = CIF_CSI2_DT_RGB565;
	} else if (in_fmt->mbus_code == MEDIA_BUS_FMT_RGB666_1X18) {
		data_type = CIF_CSI2_DT_RGB666;
	} else if (in_fmt->mbus_code == MEDIA_BUS_FMT_RGB888_1X24) {
		data_type = CIF_CSI2_DT_RGB888;
	}

	if (!data_type) {
		v4l2_err(&dev->v4l2_dev, "Invalid mipi input fmt: 0x%08x\n",
			 in_fmt->mbus_code);
		return -EINVAL;
	}

	writel(CIF_MIPI_DATA_SEL_DT(data_type) |
		  CIF_MIPI_DATA_SEL_VC(mipi_conf->vc),
		  dev->config.base_addr + CIF_MIPI_IMG_DATA_SEL);

	/* Clear MIPI interrupts */
	writel(~0, dev->config.base_addr + CIF_MIPI_ICR);
	/*
	 * Disable CIF_MIPI_ERR_DPHY interrupt here temporary for
	 * isp bus may be dead when switch isp.
	 */
	writel(CIF_MIPI_FRAME_END |
		  CIF_MIPI_ERR_CSI |
		  CIF_MIPI_ERR_DPHY |
		  CIF_MIPI_SYNC_FIFO_OVFLW(3) |
		  CIF_MIPI_ADD_DATA_OVFLW,
		  dev->config.base_addr + CIF_MIPI_IMSC);

	v4l2_info(&dev->v4l2_dev, "\n  MIPI_CTRL 0x%08x\n"
		  "  MIPI_IMG_DATA_SEL 0x%08x\n"
		  "  MIPI_STATUS 0x%08x\n"
		  "  MIPI_IMSC 0x%08x\n",
		  readl(dev->config.base_addr + CIF_MIPI_CTRL),
		  readl(dev->config.base_addr + CIF_MIPI_IMG_DATA_SEL),
		  readl(dev->config.base_addr + CIF_MIPI_STATUS),
		  readl(dev->config.base_addr + CIF_MIPI_IMSC));

	return 0;
}

static int rkisp1_config_path(struct rkisp1_device *dev)
{
	u32 dpcl = readl(dev->config.base_addr + CIF_VI_DPCL);
	int ret = 0;

	if (PLTFRM_CAM_ITF_IS_DVP(dev->config.cam_itf.type)) {
		dpcl |= CIF_VI_DPCL_IF_SEL_PARALLEL;
	} else if (PLTFRM_CAM_ITF_IS_MIPI(dev->config.cam_itf.type)) {
		ret = rkisp1_config_mipi(dev);
		dpcl |= CIF_VI_DPCL_IF_SEL_MIPI;
	}

	writel(dpcl, dev->config.base_addr + CIF_VI_DPCL);

	return ret;
}

static void rkisp1_config_clk(struct rkisp1_device *dev)
{
	writel(CIF_CCL_CIF_CLK_ENA, dev->config.base_addr + CIF_CCL);
	writel(0x0000187B, dev->config.base_addr + CIF_ICCL);
}

static int rkisp1_config_cif(struct rkisp1_device *dev)
{
	int ret = 0;
	u32 cif_id;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state);

	cif_id = readl(dev->config.base_addr + CIF_VI_ID);
	v4l2_info(&dev->v4l2_dev, "CIF_ID 0x%08x\n", cif_id);

	dev->config.out_of_buffer_stall = RKISP1_ALWAYS_STALL_ON_NO_BUFS;
	/*
	 * Cancel isp reset internal here temporary for
	 * isp bus may be dead when switch isp.
	 */
	/*
	 * writel(CIF_IRCL_CIF_SW_RST,
	 * dev->config.base_addr + CIF_IRCL);
	 */

	ret = rkisp1_config_path(dev);
	if (ret<0)
		return ret;
	ret = rkisp1_config_isp(dev);
	if (ret<0)
		return ret;
	rkisp1_config_ism(dev);

	dev->cif_streamon_cnt = 0;

	return 0;
}

static int cif_isp10_stop(struct rkisp1_device *dev)
{
	unsigned long flags = 0;
	unsigned int val;
	void __iomem *base = dev->config.base_addr;

	v4l2_info(&dev->v4l2_dev, "SP state = %d, MP state = %d\n",
		  dev->strm_vdevs.sp_vdev.state, dev->strm_vdevs.mp_vdev.state);

	if (--dev->cif_streamon_cnt == 0) {
		/*
		 * ISP(mi) stop in mi frame end -> Stop ISP(mipi) ->
		 * Stop ISP(isp) ->wait for ISP isp off
		 */
		local_irq_save(flags);
		/* stop and clear MI, MIPI, and ISP interrupts */
		writel(0, base + CIF_MIPI_IMSC);
		writel(~0, base + CIF_MIPI_ICR);

		writel(0, base + CIF_ISP_IMSC);
		writel(~0, base + CIF_ISP_ICR);

		//TODO: write verify?
		writel(0, base + CIF_MI_IMSC);
		writel(~0, base + CIF_MI_ICR);

		writel_and(~CIF_MIPI_CTRL_OUTPUT_ENA, base + CIF_MIPI_CTRL);
		/* stop ISP */
		writel_and(~(CIF_ISP_CTRL_ISP_INFORM_ENABLE |
				CIF_ISP_CTRL_ISP_ENABLE),
			      base + CIF_ISP_CTRL);
		writel_or(CIF_ISP_CTRL_ISP_CFG_UPD, base + CIF_ISP_CTRL);
		local_irq_restore(flags);

		readx_poll_timeout(readl, base + CIF_ISP_RIS,
				   val, val & CIF_ISP_OFF, 20, 100);
	}
	v4l2_info(&dev->v4l2_dev,
		  "SP state %d, MP state %d MI_CTRL 0x%08x"
		  "ISP_CTRL 0x%08x MIPI_CTRL 0x%08x\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state,
		  readl(base + CIF_MI_CTRL),
		  readl(base + CIF_ISP_CTRL),
		  readl(base + CIF_MIPI_CTRL));

	return 0;
}

static int cif_isp10_start(struct rkisp1_device *dev)
{
	unsigned int ret;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d, isp start cnt = %d\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state, dev->cif_streamon_cnt);

	if (dev->cif_streamon_cnt++ > 0)
		return 0;

	/* Activate MIPI */
	if (RKISP1_INP_IS_MIPI(dev->config.input_sel))
		writel_or(CIF_MIPI_CTRL_OUTPUT_ENA,
			     dev->config.base_addr + CIF_MIPI_CTRL);

	/* Activate ISP ! */
	if (RKISP1_INP_NEED_ISP(dev->config.input_sel))
		writel_or(CIF_ISP_CTRL_ISP_CFG_UPD |
			     CIF_ISP_CTRL_ISP_INFORM_ENABLE |
			     CIF_ISP_CTRL_ISP_ENABLE,
			     dev->config.base_addr + CIF_ISP_CTRL);

	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0)
		return ret;

	/*
	 * CIF spec says to wait for sufficient time after enabling
	 * the MIPI interface and before starting the sensor output.
	 */
	mdelay(1);

	dev->isp_sdev.isp_dev.frame_id = 0;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d MI_CTRL 0x%08x\n"
		  "  ISP_CTRL 0x%08x MIPI_CTRL 0x%08x\n",
		  dev->strm_vdevs.sp_vdev.state,
		  dev->strm_vdevs.mp_vdev.state,
		  readl(dev->config.base_addr + CIF_MI_CTRL),
		  readl(dev->config.base_addr + CIF_ISP_CTRL),
		  readl(dev->config.base_addr + CIF_MIPI_CTRL));

	return 0;
}

static const struct cif_fmt_info cifisp_isp_output_formats[] = {
	{
	 .mbus_code = MEDIA_BUS_FMT_YUYV8_1X16,
	 .fmt_type = CIF_FMT_PIX_TYPE_YUV,
	 .bpp = 16,
	 .uv_swap = 0,
	 .yc_swap = 0,
	 .xsubs = 2,
	 .ysubs = 4,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	}, {
	 .mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
	 .fmt_type = CIF_FMT_PIX_TYPE_BAYER,
	 .bayer_pat = CIF_FMT_RAW_PAT_TYPE_RGGB,
	 .bpp = 12,
	 .colorspace = V4L2_COLORSPACE_SRGB,  //TODO: _COLORSPACE_RAW ?
	},
};

static const struct cif_fmt_info cifisp_isp_input_formats[] = {
	{
	 .mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
	 .fmt_type = CIF_FMT_PIX_TYPE_BAYER,
	 .bayer_pat = CIF_FMT_RAW_PAT_TYPE_RGGB,
	 .bpp = 10,
	 .colorspace = V4L2_COLORSPACE_SRGB,  //TODO: _COLORSPACE_RAW ?
	},{
	 .mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
	 .fmt_type = CIF_FMT_PIX_TYPE_YUV,
	 .bpp = 16,
	 .uv_swap = 0,
	 .yc_swap = 0,
	 .xsubs = 2,
	 .ysubs = 4,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	}, {
	 .mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
	 .fmt_type = CIF_FMT_PIX_TYPE_YUV,
	 .bpp = 16,
	 .uv_swap = 0,
	 .yc_swap = 1,
	 .xsubs = 2,
	 .ysubs = 4,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	},
};

static const struct cif_fmt_info *cifisp_isp_sd_find_fmt(const u32 pad,
							 u32 mbus_code,
							 int index)
{
	const struct cif_fmt_info *fmt, *array_fmt;
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
		if (fmt->mbus_code == mbus_code || index == i)
			return fmt;
	}

	return NULL;
}

static int cifisp_isp_sd_enum_mbus_code(struct v4l2_subdev *sd,
				        struct v4l2_subdev_pad_config *cfg,
				        struct v4l2_subdev_mbus_code_enum *code)
{
	const struct cif_fmt_info *fmt;

	fmt = cifisp_isp_sd_find_fmt(code->pad, 0, code->index);
	if (!fmt)
		return -EINVAL;
	code->code = fmt->mbus_code;

	return 0;
}

#define sd_to_isp_sd(_sd) container_of(_sd, struct rkisp1_isp_subdev, sd)
static int cifisp_isp_sd_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct rkisp1_isp_subdev *isp_sd = sd_to_isp_sd(sd);

	if ((fmt->pad != RKISP1_ISP_PAD_SINK) ||
	    (fmt->pad != RKISP1_ISP_PAD_SOURCE_PATH))
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	switch (fmt->pad) {
	case RKISP1_ISP_PAD_SINK:
		mf->width = isp_sd->in_acqui.width;
		mf->height = isp_sd->in_acqui.height;
		mf->colorspace = isp_sd->in_fmt.colorspace;
		mf->code = isp_sd->in_fmt.mbus_code;
		break;
	case RKISP1_ISP_PAD_SOURCE_PATH:
		mf->width = isp_sd->out_win.width;
		mf->height = isp_sd->out_win.height;
		mf->colorspace = isp_sd->out_fmt.colorspace;
		mf->code = isp_sd->out_fmt.mbus_code;
		break;
	}

	return 0;
}

static void cifisp_isp_sd_try_fmt(struct v4l2_subdev *sd,
				  unsigned int pad,
				  struct v4l2_mbus_framefmt *fmt)
{
	struct rkisp1_isp_subdev *isp_sd = sd_to_isp_sd(sd);
	const struct cif_fmt_info *cif_fmt;

	cif_fmt = cifisp_isp_sd_find_fmt(pad, fmt->code, -1);
	switch (pad) {
	case RKISP1_ISP_PAD_SINK:
		if (cif_fmt) {
			fmt->code = cif_fmt->mbus_code;
			fmt->colorspace = cif_fmt->colorspace;
		} else {
			fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
			fmt->colorspace = V4L2_COLORSPACE_SRGB;
		}
		fmt->width  = clamp_t(u32, fmt->width, CIF_ISP_INPUT_W_MIN,
				      CIF_ISP_INPUT_W_MAX);
		fmt->height = clamp_t(u32, fmt->height, CIF_ISP_INPUT_H_MIN,
				      CIF_ISP_INPUT_H_MAX);
		break;
	case RKISP1_ISP_PAD_SOURCE_PATH:
		if (cif_fmt) {
			fmt->code = cif_fmt->mbus_code;
			fmt->colorspace = cif_fmt->colorspace;
		} else {
			fmt->code = MEDIA_BUS_FMT_YUYV8_1X16;
			fmt->colorspace = V4L2_COLORSPACE_JPEG;
		}
		/*TODO: Hardcode the output size to the crop rectangle size. */
		fmt->width = isp_sd->out_win.width;
		fmt->height = isp_sd->out_win.height;
		break;
	}

	fmt->field = V4L2_FIELD_NONE;
}

static int rkisp1_isp_sd_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	struct rkisp1_isp_subdev *isp_sd = &isp_dev->isp_sdev;
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct cif_fmt_info *in_fmt = &isp_sd->in_fmt;
	struct cif_fmt_info *out_fmt = &isp_sd->out_fmt;

	if ((fmt->pad != RKISP1_ISP_PAD_SINK) ||
	    (fmt->pad != RKISP1_ISP_PAD_SOURCE_PATH))
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		//TODO: this just return a empty .try_format from fh?
		fmt->format = *mf;
		return 0;
	}

	cifisp_isp_sd_try_fmt(sd, fmt->pad, mf);

	if (fmt->pad == RKISP1_ISP_PAD_SINK) {
		/* TODO: how to set crop size ? */
		isp_sd->in_acqui.top = 0;
		isp_sd->in_acqui.left = 0;
		isp_sd->in_acqui.width = mf->width;
		isp_sd->in_acqui.height = mf->height;
		in_fmt->mbus_code = mf->code;
		/* propagate to source */
		isp_sd->out_win = isp_sd->in_acqui;
		out_fmt->mbus_code = in_fmt->mbus_code;
	} else {
		out_fmt->mbus_code = mf->code;
	}

	return 0;
}

static int cifisp_stream_sd_try_crop(struct v4l2_subdev *sd,
				     struct v4l2_rect *input, u32 target)
{
	struct rkisp1_isp_subdev *isp_sd = sd_to_isp_sd(sd);
	struct v4l2_rect *in = &isp_sd->in_acqui;
	struct v4l2_rect *out = &isp_sd->out_win;

	if (target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	in->left = clamp_t(u32, input->left, 0,
					in->left - CIF_ISP_OUTPUT_W_MIN);
	in->top = clamp_t(u32, input->top, 0,
					in->top - CIF_ISP_OUTPUT_H_MIN);
	in->width = clamp_t(u32, input->width, CIF_ISP_OUTPUT_W_MIN,
					in->width - in->left);
	in->height = clamp_t(u32, input->height, CIF_ISP_INPUT_H_MIN,
					in->height - in->top);
	*out = *in;

	return 0;
}

static int cifisp_isp_sd_get_selection(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_selection *sel)
{
	struct rkisp1_isp_subdev *isp_sd = sd_to_isp_sd(sd);
	struct v4l2_rect *in = &isp_sd->in_acqui;
	struct v4l2_rect *out = &isp_sd->out_win;

	if (sel->pad != RKISP1_ISP_PAD_SOURCE_PATH)
		return -EINVAL;
	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		//TODO: v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		return 0;
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = *in;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = *out;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cifisp_isp_sd_set_selection(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_selection *sel)
{
	/* TODO: implement V4L2_SEL_TGT_CROP for SINK/acquisition pad? */
	if (sel->target != V4L2_SEL_TGT_CROP ||
	    sel->pad != RKISP1_ISP_PAD_SOURCE_PATH)
		return -EINVAL;

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		//TODO: v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		return 0;
	}

	return cifisp_stream_sd_try_crop(sd, &sel->r, sel->target);
}

static int cifisp_isp_sd_s_stream(struct v4l2_subdev *sd, int on)
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

static int cifisp_isp_sd_s_power(struct v4l2_subdev *sd, int on)
{
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	int ret = 0;

	v4l2_info(sd, "streaming count %d, s_power: %d\n",
		  dev->cif_streamon_cnt, on);

	if (on) {
		ret = pm_runtime_get_sync(dev->dev);
		if (ret < 0)
			return ret;

		/*
		 * Cancel isp reset internal here temporary for
		 * isp bus may be dead when switch isp.
		 */
		/*
		 * writel(CIF_IRCL_CIF_SW_RST,
		 * dev->config.base_addr + CIF_IRCL);
		 */

		rkisp1_config_clk(dev);

	} else {
		if (dev->cif_streamon_cnt == 0) {
			ret = pm_runtime_put(dev->dev);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops cifisp_isp_sd_pad_ops = {
	.enum_mbus_code = cifisp_isp_sd_enum_mbus_code,
	.get_selection = cifisp_isp_sd_get_selection,
	.set_selection = cifisp_isp_sd_set_selection,
	.get_fmt = cifisp_isp_sd_get_fmt,
	.set_fmt = rkisp1_isp_sd_set_fmt,
};

static const struct v4l2_subdev_video_ops cifisp_isp_sd_video_ops = {
	.s_stream = cifisp_isp_sd_s_stream,
};

static const struct v4l2_subdev_core_ops cifisp_isp_core_ops = {
	//.log_status = ,
	.s_power = cifisp_isp_sd_s_power,
};

static struct v4l2_subdev_ops cifisp_isp_sd_ops = {
	.core = &cifisp_isp_core_ops,
	.video = &cifisp_isp_sd_video_ops,
	.pad = &cifisp_isp_sd_pad_ops,
};

static int rkisp1_sd_get_camitf_info(struct rkisp1_device *isp_dev,
				     struct v4l2_subdev *subdev)
{
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

static int rkisp1_isp_link_setup(struct media_entity *entity,
				 const struct media_pad *local,
				 const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct rkisp1_device *isp_dev = sd_to_isp_dev(sd);
	u32 type;

	v4l2_info(sd, "local name %s, remote name %s, falgs %u\n",
		  local->entity->name, remote->entity->name, flags);

	if (local->index != RKISP1_ISP_PAD_SINK)
		return 0;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		isp_dev->config.input_sel = 0;
		isp_dev->config.cam_itf.type = 0;
		return 0;
	}
	if (isp_dev->config.cam_itf.type != 0)
		return -EBUSY;

	type = media_entity_type(remote->entity);
	if (type == MEDIA_ENT_T_V4L2_SUBDEV) {
		struct media_pad *csi_pad_src;
		struct media_entity *sensor;

		/* mipi interface */
		isp_dev->config.cam_itf.type = PLTFRM_CAM_ITF_MIPI;
		isp_dev->config.input_sel = RKISP1_INP_CSI;

		csi_pad_src = &remote->entity->pads[MIPIPHY_PAD_SINK];
		sensor = media_entity_remote_pad(csi_pad_src)->entity;
		rkisp1_sd_get_camitf_info(isp_dev,
					  media_entity_to_v4l2_subdev(sensor));
	} else if (type == MEDIA_ENT_T_V4L2_SUBDEV_SENSOR) {
		/* parallel interface */
		isp_dev->config.input_sel = RKISP1_INP_CPI;
		isp_dev->config.cam_itf.type = PLTFRM_CAM_ITF_BT601_8;
	}

	return 0;
}

static const struct media_entity_operations cifisp_isp_sd_media_ops = {
	.link_setup = rkisp1_isp_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static void cifisp_isp_sd_init_default_fmt(struct rkisp1_isp_subdev *isp_sd)
{
	struct v4l2_rect *in_acqui = &isp_sd->in_acqui;
	struct v4l2_rect *out_win = &isp_sd->out_win;
	struct cif_fmt_info *in_fmt = &isp_sd->in_fmt;
	struct cif_fmt_info *out_fmt = &isp_sd->out_fmt;

	in_acqui->top = in_acqui->left = 0;
	in_acqui->width = CIF_ISP_DEFAULT_W;
	in_acqui->height = CIF_ISP_DEFAULT_H;
	*in_fmt = cifisp_isp_input_formats[0];

	/* propagate to source */
	*out_win = *in_acqui;
	*out_fmt = cifisp_isp_output_formats[0];
}

int register_cifisp_isp_subdev(struct rkisp1_device *isp_dev,
			       struct v4l2_device *v4l2_dev)
{
	struct rkisp1_isp_subdev *isp_sdev = &isp_dev->isp_sdev;
	struct v4l2_subdev *sd = &isp_sdev->sd;
	struct v4l2_ctrl_handler *handler = &isp_sdev->ctrl_handler;
	int ret;

	v4l2_subdev_init(sd, &cifisp_isp_sd_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "cif-isp10-subdev-isp");

	isp_sdev->pads[RKISP1_ISP_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	isp_sdev->pads[RKISP1_ISP_PAD_SOURCE_PATH].flags = MEDIA_PAD_FL_SOURCE;
	isp_sdev->pads[RKISP1_ISP_PAD_SOURCE_STATS].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, RKISP1_ISP_PAD_MAX,
				isp_sdev->pads, 0);
	if (ret<0)
		return ret;

	//TODO: setup ctrls, eg. 3A controls
	v4l2_ctrl_handler_init(handler, 1);

	if (handler->error) {
		ret = handler->error;
		goto err_cleanup_media_entity;
	}

	sd->ctrl_handler = handler;
	sd->entity.ops = &cifisp_isp_sd_media_ops;
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, isp_dev);

	//TODO: group id?
	sd->grp_id = GRP_ID_ISP;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		v4l2_err(sd, "Failed to register isp subdev\n");
		goto err_free_ctrl_handler;
	}

	cifisp_isp_sd_init_default_fmt(isp_sdev);
	return 0;
err_free_ctrl_handler:
	v4l2_ctrl_handler_free(handler);
err_cleanup_media_entity:
	media_entity_cleanup(&sd->entity);
	return ret;
}

void unregister_cifisp_isp_subdev(struct rkisp1_device *isp_dev)
{
	struct v4l2_subdev *sd = &isp_dev->isp_sdev.sd;

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&isp_dev->isp_sdev.ctrl_handler);
	media_entity_cleanup(&sd->entity);
}

static void cif_isp10_hw_restart(struct rkisp1_device *dev)
{
	void __iomem *base = dev->config.base_addr;

#define RESET_MIPI	BIT(11)
#define RESET_MI	BIT(6)
#define RESET_ISP	BIT(0)
	writel(RESET_MIPI | RESET_ISP | RESET_MI, base + CIF_IRCL);
	writel(0x0, base + CIF_IRCL);

	/* enable mipi interrupts */
	writel(CIF_MIPI_FRAME_END | CIF_MIPI_ERR_CSI |
		      CIF_MIPI_ERR_DPHY | CIF_MIPI_SYNC_FIFO_OVFLW(3) |
		      CIF_MIPI_ADD_DATA_OVFLW, base + CIF_MIPI_IMSC);

	writel(0x0, base + CIF_MI_MP_Y_OFFS_CNT_INIT);
	writel(0x0, base + CIF_MI_MP_CR_OFFS_CNT_INIT);
	writel(0x0, base + CIF_MI_MP_CB_OFFS_CNT_INIT);
	writel(0x0, base + CIF_MI_SP_Y_OFFS_CNT_INIT);
	writel(0x0, base + CIF_MI_SP_CR_OFFS_CNT_INIT);
	writel(0x0, base + CIF_MI_SP_CB_OFFS_CNT_INIT);
	writel_or(CIF_MI_CTRL_INIT_OFFSET_EN, base + CIF_MI_CTRL);

	/* Enable ISP */
	writel_or(CIF_ISP_CTRL_ISP_CFG_UPD | CIF_ISP_CTRL_ISP_ENABLE |
		     CIF_ISP_CTRL_ISP_INFORM_ENABLE , base + CIF_ISP_CTRL);
	/* enable MIPI */
	writel_or(CIF_MIPI_CTRL_OUTPUT_ENA, base + CIF_MIPI_CTRL);
}

void rkisp1_mipi_isr(unsigned int mis, struct rkisp1_device *dev)
{
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	void __iomem *base = dev->config.base_addr;

	mis = readl(base + CIF_MIPI_MIS); //TODO: why read again?
	writel(~0, base + CIF_MIPI_ICR);

	if (mis & CIF_MIPI_ERR_DPHY) {
		v4l2_warn(v4l2_dev, "CIF_MIPI_ERR_DPHY: 0x%x\n", mis);

		/*
		 * Disable DPHY errctrl interrupt, because this dphy
		 * erctrl signal is asserted until the next changes
		 * of line state. This time is may be too long and cpu
		 * is hold in this interrupt.
		 */
		if (mis & CIF_MIPI_ERR_CTRL(3)) //TODO: 0xf - one bit per lane?
			writel_and(~CIF_MIPI_ERR_CTRL(3),
				      base + CIF_MIPI_IMSC);
	}

	if (mis & CIF_MIPI_ERR_CSI)
		v4l2_warn(v4l2_dev, "CIF_MIPI_ERR_CSI: 0x%x\n", mis);

	if (mis & CIF_MIPI_SYNC_FIFO_OVFLW(3))
		v4l2_warn(v4l2_dev, "CIF_MIPI_SYNC_FIFO_OVFLW: 0x%x\n", mis);

	if (mis == CIF_MIPI_FRAME_END)
		/*
		 * Enable DPHY errctrl interrupt again, if mipi have receive
		 * the whole frame without any error.
		 */
		writel_or(CIF_MIPI_ERR_CTRL(3), base + CIF_MIPI_IMSC);
}

void rkisp1_isp_isr(unsigned int isp_mis, struct rkisp1_device *dev)
{
	void __iomem *base = dev->config.base_addr;
	unsigned int isp_mis_tmp = 0;
	unsigned int isp_err = 0;
	struct timeval tv;

	if (isp_mis & CIF_ISP_V_START) {
		do_gettimeofday(&tv);
		dev->b_mi_frame_end = false;
		cifisp_v_start(&dev->isp_sdev.isp_dev, &tv);

		writel(CIF_ISP_V_START, base + CIF_ISP_ICR);
		isp_mis_tmp = readl(base + CIF_ISP_MIS);
		if (isp_mis_tmp & CIF_ISP_V_START)
			v4l2_err(&dev->v4l2_dev, "isp icr v_statr err: 0x%x\n",
				 isp_mis_tmp);

		writel_or(CIF_ISP_CTRL_ISP_GEN_CFG_UPD, base + CIF_ISP_CTRL);
		if (dev->sof_event)
			dev->sof_event(dev,
				       dev->isp_sdev.isp_dev.frame_id >> 1);
	}

	if (isp_mis & CIF_ISP_FRAME_IN) {
		do_gettimeofday(&tv);
		writel(CIF_ISP_FRAME_IN, base + CIF_ISP_ICR);
		cifisp_frame_in(&dev->isp_sdev.isp_dev, &tv);
	}

	if (isp_mis & (CIF_ISP_DATA_LOSS | CIF_ISP_PIC_SIZE_ERROR)) {
		dev->strm_vdevs.sp_vdev.stall = true;
		dev->strm_vdevs.mp_vdev.stall = true;

		if ((isp_mis & CIF_ISP_PIC_SIZE_ERROR)) {
			/* Clear pic_size_error */
			writel(CIF_ISP_PIC_SIZE_ERROR,
				      base + CIF_ISP_ICR);
			isp_err = readl(base + CIF_ISP_ERR);
			v4l2_err(&dev->v4l2_dev,
				 "CIF_ISP_PIC_SIZE_ERROR (0x%08x)", isp_err);
			writel(isp_err, base + CIF_ISP_ERR_CLR);
		} else if ((isp_mis & CIF_ISP_DATA_LOSS)) {
			/* Clear data_loss */
			writel(CIF_ISP_DATA_LOSS, base + CIF_ISP_ICR);
			v4l2_err(&dev->v4l2_dev, "CIF_ISP_DATA_LOSS\n");
			writel(CIF_ISP_DATA_LOSS, base + CIF_ISP_ICR);
		}

		/* Stop ISP */
		writel_and(~CIF_ISP_CTRL_ISP_INFORM_ENABLE &
			      ~CIF_ISP_CTRL_ISP_ENABLE, base + CIF_ISP_CTRL);
		/* isp_update */
		writel_or(CIF_ISP_CTRL_ISP_CFG_UPD, base + CIF_ISP_CTRL);
		cif_isp10_hw_restart(dev);
	}

	if (isp_mis & CIF_ISP_FRAME_IN) {
		writel(CIF_ISP_FRAME_IN, base + CIF_ISP_ICR);
		isp_mis_tmp = readl(base + CIF_ISP_MIS);
		if (isp_mis_tmp & CIF_ISP_FRAME_IN)
			v4l2_err(&dev->v4l2_dev, "isp icr frame_in err: 0x%x\n",
				 isp_mis_tmp);

		/* restart MI if CIF has run out of buffers */
		//TODO: move this(restart MI) to path_video.c?
		if (!RKISP1_MI_IS_BUSY(dev) &&
		    (!dev->strm_vdevs.sp_vdev.next_buf &&
		     !dev->strm_vdevs.mp_vdev.next_buf)) {
			u32 mi_isr = 0;

			if (dev->strm_vdevs.sp_vdev.state ==
			    RKISP1_STATE_STREAMING)
				mi_isr |= CIF_MI_SP_FRAME;
			if (dev->strm_vdevs.mp_vdev.state ==
			    RKISP1_STATE_STREAMING)
				mi_isr |= CIF_MI_MP_FRAME;
			writel(mi_isr, base + CIF_MI_ISR);
		}
	}

	if (isp_mis & CIF_ISP_FRAME) {
		/* Clear Frame In (ISP) */
		writel(CIF_ISP_FRAME, base + CIF_ISP_ICR);
		isp_mis_tmp = readl(base + CIF_ISP_MIS);
		if (isp_mis_tmp & CIF_ISP_FRAME)
			v4l2_err(&dev->v4l2_dev,
				 "isp icr frame end err: 0x%x\n", isp_mis_tmp);
	}

	cifisp_isp_isr(&dev->isp_sdev.isp_dev, isp_mis);
}
