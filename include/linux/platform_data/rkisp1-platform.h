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
#ifndef _RKISP1_PLATFORM_H
#define _RKISP1_PLATFORM_H

#include <linux/videodev2.h>

#define DRIVER_NAME "rkisp10"
#define ISP_VDEV_NAME DRIVER_NAME  "_ispdev"
#define SP_VDEV_NAME DRIVER_NAME   "_selfpath"
#define MP_VDEV_NAME DRIVER_NAME   "_mainpath"
#define DMA_VDEV_NAME DRIVER_NAME  "_dmapath"

enum pltfrm_cam_signal_polarity {
	PLTFRM_CAM_SIGNAL_HIGH_LEVEL,
	PLTFRM_CAM_SIGNAL_LOW_LEVEL,
};

enum pltfrm_cam_sample_type {
	PLTFRM_CAM_SDR_NEG_EDG = 0x10000001,
	PLTFRM_CAM_SDR_POS_EDG = 0x10000002,
	PLTFRM_CAM_DDR = 0x20000000
};

enum pltfrm_cam_itf_type {
	PLTFRM_CAM_ITF_MIPI = 0x10000000,
	PLTFRM_CAM_ITF_BT601_8 = 0x20000071,
	PLTFRM_CAM_ITF_BT656_8 = 0x20000072,
	PLTFRM_CAM_ITF_BT601_10 = 0x20000091,
	PLTFRM_CAM_ITF_BT656_10 = 0x20000092,
	PLTFRM_CAM_ITF_BT601_12 = 0x200000B1,
	PLTFRM_CAM_ITF_BT656_12 = 0x200000B2,
	PLTFRM_CAM_ITF_BT601_16 = 0x200000F1,
	PLTFRM_CAM_ITF_BT656_16 = 0x200000F2
};

#define PLTFRM_CAM_ITF_MAIN_MASK   0xF0000000
#define PLTFRM_CAM_ITF_SUB_MASK    0x0000000F
#define PLTFRM_CAM_ITF_DVP_BW_MASK 0x000000F0

#define PLTFRM_CAM_ITF_IS_MIPI(a)    \
		(((a) & PLTFRM_CAM_ITF_MAIN_MASK) == 0x10000000)
#define PLTFRM_CAM_ITF_IS_DVP(a)    \
		(((a) & PLTFRM_CAM_ITF_MAIN_MASK) == 0x20000000)
#define PLTFRM_CAM_ITF_IS_BT656(a)	(PLTFRM_CAM_ITF_IS_DVP(a) &&\
		(((a) & PLTFRM_CAM_ITF_SUB_MASK) == 0x02))
#define PLTFRM_CAM_ITF_IS_BT601(a)	(PLTFRM_CAM_ITF_IS_DVP(a) &&\
		(((a) & PLTFRM_CAM_ITF_SUB_MASK) == 0x01))
#define PLTFRM_CAM_ITF_DVP_BW(a)    \
		((((a) & PLTFRM_CAM_ITF_DVP_BW_MASK) >> 4) + 1)

struct pltfrm_cam_mipi_config {
	u32 dphy_index;
	u32 vc;
	u32 nb_lanes;
	u32 bit_rate;
};

struct pltfrm_cam_dvp_config {
	enum pltfrm_cam_signal_polarity vsync;
	enum pltfrm_cam_signal_polarity hsync;
	enum pltfrm_cam_sample_type pclk;
};

struct pltfrm_cam_itf {
	enum pltfrm_cam_itf_type type;

	union {
		struct pltfrm_cam_mipi_config mipi;
		struct pltfrm_cam_dvp_config dvp;
	} cfg;
};

#endif /* _RKISP1_PLATFORM_H */
