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

#include <linux/iopoll.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include "mipi_dphy_sy.h"
#include "capture.h"
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

struct rkisp1_sensor_info *get_active_sensor_info(struct rkisp1_device *dev)
{
	int i;
	struct v4l2_subdev *sd = dev->subdevs[RKISP1_SD_SENSOR];

	for (i = 0; i < dev->num_sensors; i++)
		if (dev->sensors[i].sd == sd)
			return &dev->sensors[i];

	return NULL;
}

/*
 * This should only be called when configuring CIF
 * or at the frame end interrupt
 */
static void rkisp1_config_ism(struct rkisp1_device *dev)
{
	void __iomem *base = dev->config.base_addr;
	struct v4l2_rect *win = &dev->isp_sdev.out_win;
	void *addr;

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
	addr = base + CIF_ISP_CTRL;
	writel(CIF_ISP_CTRL_ISP_CFG_UPD | readl(addr), (addr));
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
	void *addr;
	struct rkisp1_fmt *in_fmt, *out_fmt;
	struct v4l2_rect *in_acqui, *out_win;
	void __iomem *base = dev->config.base_addr;
	struct rkisp1_sensor_info *sensor = get_active_sensor_info(dev);

	in_fmt = &dev->isp_sdev.in_fmt;
	out_fmt = &dev->isp_sdev.out_fmt;
	in_acqui = &dev->isp_sdev.in_acqui;
	out_win = &dev->isp_sdev.out_win;
	addr = base + CIF_ICCL;
	writel(CIF_ICCL_ISP_CLK | readl(addr), (addr));

	if (in_fmt->fmt_type == FMT_BAYER) {
		if (out_fmt->fmt_type == FMT_BAYER) {
			if (sensor->ep.bus_type == V4L2_MBUS_BT656)
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_RAW_PICT_ITU656;
			else
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_RAW_PICT;
		} else {
			writel(0xc, base + CIF_ISP_DEMOSAIC);

			if (sensor->ep.bus_type == V4L2_MBUS_BT656)
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
		case RAW_BGGR:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_BGGR;
			break;
		case RAW_GBRG:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GBRG;
			break;
		case RAW_GRBG:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_GRBG;
			break;
		case RAW_RGGB:
			isp_bayer_pat = CIF_ISP_ACQ_PROP_BAYER_PAT_RGGB;
			break;
		}
	} else if (in_fmt->fmt_type == FMT_YUV) {
		acq_mult = 2;
		if (sensor->ep.bus_type == V4L2_MBUS_CSI2) {
			isp_input_sel = CIF_ISP_ACQ_PROP_IN_SEL_12B;
			isp_ctrl = CIF_ISP_CTRL_ISP_MODE_ITU601;
		} else {
			if (sensor->ep.bus_type == V4L2_MBUS_BT656)
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_ITU656;
			else
				isp_ctrl = CIF_ISP_CTRL_ISP_MODE_ITU601;

			switch (sensor->ep.bus.parallel.bus_width) {
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
				v4l2_err(&dev->v4l2_dev, "Invalid bus width\n");
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
	if (sensor->ep.bus_type == V4L2_MBUS_BT656 ||
	    sensor->ep.bus_type == V4L2_MBUS_PARALLEL) {
		if (sensor->ep.bus.parallel.flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
			signal = CIF_ISP_ACQ_PROP_POS_EDGE;

		if (sensor->ep.bus_type == V4L2_MBUS_PARALLEL) {
			if (sensor->ep.bus.parallel.flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
				signal |= CIF_ISP_ACQ_PROP_VSYNC_LOW;

			if (sensor->ep.bus.parallel.flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
				signal |= CIF_ISP_ACQ_PROP_HSYNC_LOW;

		}
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

	if (out_fmt->fmt_type == FMT_BAYER)
		rkisp1_disable_isp(&dev->params_vdev);
	else
		rkisp1_configure_isp(&dev->params_vdev, in_fmt,
				     V4L2_QUANTIZATION_FULL_RANGE);

	return 0;
}

static int rkisp1_config_mipi(struct rkisp1_device *dev)
{
	int i, lanes;
	u32 data_type, mipi_ctrl, shutdown_lanes = 0;
	struct rkisp1_fmt *in_fmt;
	void *addr;

	addr = dev->config.base_addr + CIF_ICCL;
	writel(CIF_ICCL_MIPI_CLK | readl(addr), (addr));

	in_fmt = &dev->isp_sdev.in_fmt;

	lanes = mipi_dphy_sy_lanes(dev->subdevs[RKISP1_SD_PHY_CSI]);
	if (lanes < 1 || lanes > 4) {
		v4l2_err(&dev->v4l2_dev, "DPHY lanes(%d) out of range[1..4]\n",
			 lanes);
		return -EINVAL;
	}

	for (i = 0; i < lanes; i++)
		shutdown_lanes |= (1 << i);

	mipi_ctrl = CIF_MIPI_CTRL_NUM_LANES(lanes - 1) |
		    CIF_MIPI_CTRL_SHUTDOWNLANES(0xf) |
		    CIF_MIPI_CTRL_ERR_SOT_SYNC_HS_SKIP |
		    CIF_MIPI_CTRL_CLOCKLANE_ENA;

	writel(mipi_ctrl, dev->config.base_addr + CIF_MIPI_CTRL);

	/* TODO: shutdown lanes
	writel(mipi_ctrl | CIF_MIPI_CTRL_SHUTDOWNLANES(0xf),
		      dev->config.base_addr + CIF_MIPI_CTRL);
	*/

	/* Configure Data Type and Virtual Channel */
	// TODO: enumerate all supported mbus code instead
	if (in_fmt->fmt_type == FMT_YUV) {
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
	} else if (in_fmt->fmt_type == FMT_BAYER) {
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
		  CIF_MIPI_DATA_SEL_VC(0),
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
	int ret = 0;
	struct rkisp1_sensor_info *sensor = get_active_sensor_info(dev);
	u32 dpcl = readl(dev->config.base_addr + CIF_VI_DPCL);

	if (sensor->ep.bus_type == V4L2_MBUS_BT656 ||
	    sensor->ep.bus_type == V4L2_MBUS_PARALLEL) {
		dpcl |= CIF_VI_DPCL_IF_SEL_PARALLEL;
	} else if (sensor->ep.bus_type == V4L2_MBUS_CSI2) {
		ret = rkisp1_config_mipi(dev);
		dpcl |= CIF_VI_DPCL_IF_SEL_MIPI;
	}

	writel(dpcl, dev->config.base_addr + CIF_VI_DPCL);

	return ret;
}

static void rkisp1_config_clk(struct rkisp1_device *dev)
{
	/* TODO: remove CIF_CCL_CIF_CLK_ENA, this is default */
	/*writel(CIF_CCL_CIF_CLK_ENA, dev->config.base_addr + CIF_CCL);*/
	u32 val = CIF_ICCL_ISP_CLK | CIF_ICCL_CP_CLK | CIF_ICCL_MRSZ_CLK |
			CIF_ICCL_SRSZ_CLK | CIF_ICCL_JPEG_CLK | CIF_ICCL_MI_CLK |
			CIF_ICCL_MIPI_CLK;

	writel(val, dev->config.base_addr + CIF_ICCL);
}

static int rkisp1_config_cif(struct rkisp1_device *dev)
{
	int ret = 0;
	u32 cif_id;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d\n",
		  dev->sp_stream.state,
		  dev->mp_stream.state);

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
	if (ret < 0)
		return ret;
	ret = rkisp1_config_isp(dev);
	if (ret < 0)
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
	void *addr;

	v4l2_info(&dev->v4l2_dev, "SP state = %d, MP state = %d\n",
		  dev->sp_stream.state, dev->mp_stream.state);

	if (--dev->cif_streamon_cnt > 0)
		return 0;
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
	addr = base + CIF_MIPI_CTRL;
	writel((~CIF_MIPI_CTRL_OUTPUT_ENA) & readl(addr), (addr));
	/* stop ISP */
	addr = base + CIF_ISP_CTRL;
	val = (u32)~(CIF_ISP_CTRL_ISP_INFORM_ENABLE |CIF_ISP_CTRL_ISP_ENABLE);
	writel(val & readl(addr), (addr));
	addr = base + CIF_ISP_CTRL;
	writel(CIF_ISP_CTRL_ISP_CFG_UPD | readl(addr), (addr));
	local_irq_restore(flags);

	readx_poll_timeout(readl, base + CIF_ISP_RIS,
			   val, val & CIF_ISP_OFF, 20, 100);
	v4l2_info(&dev->v4l2_dev,
		  "SP state %d, MP state %d MI_CTRL 0x%08x"
		  "ISP_CTRL 0x%08x MIPI_CTRL 0x%08x\n",
		  dev->sp_stream.state,
		  dev->mp_stream.state,
		  readl(base + CIF_MI_CTRL),
		  readl(base + CIF_ISP_CTRL),
		  readl(base + CIF_MIPI_CTRL));

	return 0;
}

static int cif_isp10_start(struct rkisp1_device *dev)
{
	unsigned int ret;
	struct rkisp1_sensor_info *sensor = get_active_sensor_info(dev);
	void *addr;
	u32 val;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d, isp start cnt = %d\n",
		  dev->sp_stream.state,
		  dev->mp_stream.state, dev->cif_streamon_cnt);

	if (dev->cif_streamon_cnt++ > 0)
		return 0;

	/* Activate MIPI */
	if (sensor->ep.bus_type == V4L2_MBUS_CSI2){
		addr = dev->config.base_addr + CIF_MIPI_CTRL;
		writel(CIF_MIPI_CTRL_OUTPUT_ENA | readl(addr), (addr));
	}
	/* Activate ISP */
	addr = dev->config.base_addr + CIF_ISP_CTRL;
	val = CIF_ISP_CTRL_ISP_CFG_UPD |
		  CIF_ISP_CTRL_ISP_INFORM_ENABLE |
		  CIF_ISP_CTRL_ISP_ENABLE;
	writel(val | readl(addr), (addr));

	ret = pm_runtime_get_sync(dev->dev);
	if (ret)
		return ret;

	/*
	 * CIF spec says to wait for sufficient time after enabling
	 * the MIPI interface and before starting the sensor output.
	 */
	mdelay(1);
	/* TODO: maybe moved to isp_stats.c*/
	dev->stats_vdev.frame_id = 0;

	v4l2_info(&dev->v4l2_dev,
		  "SP state = %d, MP state = %d MI_CTRL 0x%08x\n"
		  "  ISP_CTRL 0x%08x MIPI_CTRL 0x%08x\n",
		  dev->sp_stream.state,
		  dev->mp_stream.state,
		  readl(dev->config.base_addr + CIF_MI_CTRL),
		  readl(dev->config.base_addr + CIF_ISP_CTRL),
		  readl(dev->config.base_addr + CIF_MIPI_CTRL));

	return 0;
}

static const struct rkisp1_fmt cifisp_isp_output_formats[] = {
	{
	 .mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
	 .fmt_type = FMT_YUV,
	 .bpp = 16,
	 .uv_swap = 0,
	 .yc_swap = 0,
	 .xsubs = 2,
	 .ysubs = 4,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	}, {
	 .mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
	 .fmt_type = FMT_BAYER,
	 .bayer_pat = RAW_RGGB,
	 .bpp = 12,
	 .colorspace = V4L2_COLORSPACE_SRGB,  //TODO: _COLORSPACE_RAW ?
	},
};

static const struct rkisp1_fmt cifisp_isp_input_formats[] = {
	{
	 .mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
	 .fmt_type = FMT_BAYER,
	 .bayer_pat = RAW_RGGB,
	 .bpp = 10,
	 .colorspace = V4L2_COLORSPACE_SRGB,  //TODO: _COLORSPACE_RAW ?
	}, {
	 .mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
	 .fmt_type = FMT_BAYER,
	 .bayer_pat = RAW_BGGR,
	 .bpp = 8,
	 .colorspace = V4L2_COLORSPACE_SRGB,
	}, {
	 .mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
	 .fmt_type = FMT_YUV,
	 .bpp = 16,
	 .uv_swap = 0,
	 .yc_swap = 0,
	 .xsubs = 2,
	 .ysubs = 4,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	}, {
	 .mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
	 .fmt_type = FMT_YUV,
	 .bpp = 16,
	 .uv_swap = 0,
	 .yc_swap = 1,
	 .xsubs = 2,
	 .ysubs = 4,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	},
};

static const struct rkisp1_fmt *cifisp_isp_sd_find_fmt(const u32 pad,
							 u32 mbus_code,
							 int index)
{
	const struct rkisp1_fmt *fmt, *array_fmt;
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
	const struct rkisp1_fmt *fmt;

	fmt = cifisp_isp_sd_find_fmt(code->pad, 0, code->index);
	if (!fmt)
		return -EINVAL;
	code->code = fmt->mbus_code;

	return 0;
}

/* For compatibility with UVC program, we have to use this funciton,
 * to allow this driver run out-of-box.
 * If you are writing your own applications, please don't depend on this funciton,
 * each media entity should be configured properly by userspace before streamon.
 */
static void __reset_entities_config(struct rkisp1_device *isp_dev)
{
	struct rkisp1_isp_subdev *isp_sd = &isp_dev->isp_sdev;
	struct v4l2_subdev_format fmt;
	const struct rkisp1_fmt *cif_fmt;

	fmt.pad = 0;
	v4l2_subdev_call(isp_dev->subdevs[RKISP1_SD_SENSOR],
			       pad, get_fmt, NULL, &fmt);

	if (fmt.format.code != isp_sd->in_fmt.mbus_code ||
	    fmt.format.width != isp_sd->in_acqui.width ||
	    fmt.format.height != isp_sd->in_acqui.height) {
		v4l2_warn(&isp_dev->v4l2_dev,
			  "Reset subdev settigs to default......\n");
	}

	isp_sd->in_acqui.width = fmt.format.width;
	isp_sd->in_acqui.height = fmt.format.height;
	cif_fmt = cifisp_isp_sd_find_fmt(RKISP1_ISP_PAD_SINK,
						fmt.format.code, -1);
	if (!cif_fmt) {
		v4l2_warn(&isp_dev->v4l2_dev,
			  "Unsuppoted sensor input format......\n");
		return;
	}

	isp_sd->in_fmt = *cif_fmt;
	isp_sd->out_win = isp_sd->in_acqui;
	isp_dev->mp_stream.dcrop = isp_sd->out_win;
	isp_dev->sp_stream.dcrop = isp_sd->out_win;
}

#define sd_to_isp_sd(_sd) container_of(_sd, struct rkisp1_isp_subdev, sd)
static int cifisp_isp_sd_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct rkisp1_isp_subdev *isp_sd = sd_to_isp_sd(sd);

	__reset_entities_config(sd_to_isp_dev(sd));

	if ((fmt->pad != RKISP1_ISP_PAD_SINK) &&
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
	const struct rkisp1_fmt *cif_fmt;

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
			fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;
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
	struct rkisp1_fmt *in_fmt = &isp_sd->in_fmt;
	struct rkisp1_fmt *out_fmt = &isp_sd->out_fmt;

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
	
	if (fmt->pad != RKISP1_ISP_PAD_SINK) {
		out_fmt->mbus_code = mf->code;
		return 0;
	}

	/* TODO: how to set crop size ? */
	isp_sd->in_acqui.top = 0;
	isp_sd->in_acqui.left = 0;
	isp_sd->in_acqui.width = mf->width;
	isp_sd->in_acqui.height = mf->height;
	in_fmt->mbus_code = mf->code;
	/* propagate to source */
	isp_sd->out_win = isp_sd->in_acqui;
	out_fmt->mbus_code = in_fmt->mbus_code;

	/* reset stream vdev crop size */
	isp_dev->mp_stream.dcrop = isp_sd->out_win;
	isp_dev->sp_stream.dcrop = isp_sd->out_win;

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

	if(!on)
		return cif_isp10_stop(isp_dev);
	
	ret = rkisp1_config_cif(isp_dev);
	if (ret)
		return ret;

	return cif_isp10_start(isp_dev);
}

static int cifisp_isp_sd_s_power(struct v4l2_subdev *sd, int on)
{
	struct rkisp1_device *dev = sd_to_isp_dev(sd);
	int ret = 0;

	v4l2_info(sd, "streaming count %d, s_power: %d\n",
		  dev->cif_streamon_cnt, on);

	if (dev->cif_streamon_cnt > 0)
		return 0;

	if (on) {
		ret = pm_runtime_get_sync(dev->dev);
		if (ret)
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
			if (ret)
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

//TODO: link validate : fmt/size

static void cifisp_isp_sd_init_default_fmt(struct rkisp1_isp_subdev *isp_sd)
{
	struct v4l2_rect *in_acqui = &isp_sd->in_acqui;
	struct v4l2_rect *out_win = &isp_sd->out_win;
	struct rkisp1_fmt *in_fmt = &isp_sd->in_fmt;
	struct rkisp1_fmt *out_fmt = &isp_sd->out_fmt;

	in_acqui->top = in_acqui->left = 0;
	in_acqui->width = RKISP1_DEFAULT_WIDTH;
	in_acqui->height = RKISP1_DEFAULT_HEIGHT;
	*in_fmt = cifisp_isp_input_formats[0];

	/* propagate to source */
	*out_win = *in_acqui;
	*out_fmt = cifisp_isp_output_formats[0];
}

int rkisp1_register_isp_subdev(struct rkisp1_device *isp_dev,
			       struct v4l2_device *v4l2_dev)
{
	struct rkisp1_isp_subdev *isp_sdev = &isp_dev->isp_sdev;
	struct v4l2_subdev *sd = &isp_sdev->sd;
	struct v4l2_ctrl_handler *handler = &isp_sdev->ctrl_handler;
	int ret;

	v4l2_subdev_init(sd, &cifisp_isp_sd_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "rkisp1-isp-subdev");

	isp_sdev->pads[RKISP1_ISP_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	isp_sdev->pads[RKISP1_ISP_PAD_SINK_PARAMS].flags = MEDIA_PAD_FL_SINK;
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
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, isp_dev);

	//TODO: group id?
	sd->grp_id = GRP_ID_ISP;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret) {
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

void rkisp1_unregister_isp_subdev(struct rkisp1_device *isp_dev)
{
	struct v4l2_subdev *sd = &isp_dev->isp_sdev.sd;

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&isp_dev->isp_sdev.ctrl_handler);
	media_entity_cleanup(&sd->entity);
}

static void cif_isp10_hw_restart(struct rkisp1_device *dev)
{
	void __iomem *base = dev->config.base_addr;
	void *addr;
	u32 val;

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
	addr = base + CIF_MI_CTRL;
	writel(CIF_MI_CTRL_INIT_OFFSET_EN | readl(addr), (addr));

	/* Enable ISP */
	addr = base + CIF_ISP_CTRL;
	val = CIF_ISP_CTRL_ISP_CFG_UPD | CIF_ISP_CTRL_ISP_ENABLE |
		     CIF_ISP_CTRL_ISP_INFORM_ENABLE;
	writel(val | readl(addr), (addr));
	/* enable MIPI */
	addr = base + CIF_MIPI_CTRL;
	writel(CIF_MIPI_CTRL_OUTPUT_ENA | readl(addr), (addr));
}

void rkisp1_mipi_isr(unsigned int mis, struct rkisp1_device *dev)
{
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	void __iomem *base = dev->config.base_addr;
	u32 val;
	void *addr;

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
		if (mis & CIF_MIPI_ERR_CTRL(3)) {
			//TODO: 0xf - one bit per lane?
			val = ~CIF_MIPI_ERR_CTRL(3);
			addr = base + CIF_MIPI_IMSC;
			writel((val) & readl(addr), (addr));
		}
	}

	if (mis & CIF_MIPI_ERR_CSI)
		v4l2_warn(v4l2_dev, "CIF_MIPI_ERR_CSI: 0x%x\n", mis);

	if (mis & CIF_MIPI_SYNC_FIFO_OVFLW(3))
		v4l2_warn(v4l2_dev, "CIF_MIPI_SYNC_FIFO_OVFLW: 0x%x\n", mis);

	if (mis == CIF_MIPI_FRAME_END){
		/*
		 * Enable DPHY errctrl interrupt again, if mipi have receive
		 * the whole frame without any error.
		 */
		addr = base + CIF_MIPI_IMSC;
		val = CIF_MIPI_ERR_CTRL(3);
		writel((val) | readl(addr), (addr));
	}
}

void rkisp1_isp_isr(unsigned int isp_mis, struct rkisp1_device *dev)
{
	void __iomem *base = dev->config.base_addr;
	unsigned int isp_mis_tmp = 0;
	unsigned int isp_err = 0;
	struct timeval tv;
	void *addr;
	u32 val;

	if (isp_mis & CIF_ISP_V_START) {
		do_gettimeofday(&tv);
		rkisp1_stats_v_start(&dev->stats_vdev, &tv);
		rkisp1_params_v_start(&dev->params_vdev);

		writel(CIF_ISP_V_START, base + CIF_ISP_ICR);
		isp_mis_tmp = readl(base + CIF_ISP_MIS);
		if (isp_mis_tmp & CIF_ISP_V_START)
			v4l2_err(&dev->v4l2_dev, "isp icr v_statr err: 0x%x\n",
				 isp_mis_tmp);

		addr = base + CIF_ISP_CTRL;
		writel(CIF_ISP_CTRL_ISP_GEN_CFG_UPD | readl(addr), (addr));
		if (dev->sof_event)
			dev->sof_event(dev,
				       dev->stats_vdev.frame_id);
	}

	if (isp_mis & CIF_ISP_FRAME_IN) {
		do_gettimeofday(&tv);
		writel(CIF_ISP_FRAME_IN, base + CIF_ISP_ICR);
		rkisp1_stats_frame_in(&dev->stats_vdev, &tv);
	}

	if (isp_mis & (CIF_ISP_DATA_LOSS | CIF_ISP_PIC_SIZE_ERROR)) {
		dev->sp_stream.stall = true;
		dev->mp_stream.stall = true;

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
		addr = base + CIF_ISP_CTRL;
		val = (u32)~(CIF_ISP_CTRL_ISP_INFORM_ENABLE & CIF_ISP_CTRL_ISP_ENABLE);
		writel(val & readl(addr), (addr));
		/* isp_update */
		addr = base + CIF_ISP_CTRL;
		writel(CIF_ISP_CTRL_ISP_CFG_UPD | readl(addr), (addr));
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
		if ((!dev->sp_stream.next_buf) && (!dev->mp_stream.next_buf)) {
			u32 mi_isr = 0;

			if (dev->sp_stream.state ==
			    RKISP1_STATE_STREAMING)
				mi_isr |= CIF_MI_SP_FRAME;
			if (dev->mp_stream.state ==
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

	rkisp1_stats_isr(&dev->stats_vdev, isp_mis);

	/*
	 * Then update changed configs. Some of them involve
	 * lot of register writes. Do those only one per frame.
	 * Do the updates in the order of the processing flow.
	 */
	rkisp1_params_isr(&dev->params_vdev, isp_mis);
}
