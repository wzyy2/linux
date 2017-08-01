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

#ifndef _RKISP1_PATH_H
#define _RKISP1_PATH_H

#include "common.h"

struct cif_isp10_dcrop_config {
	struct v4l2_rect crop;
};

struct cif_isp10_rsz_config {
	u32 input_width;
	u32 input_height;
	u32 output_width;
	u32 output_height;
};

enum cif_isp10_sp_inp {
	RKISP1_SP_INP_ISP = 0x30000000,
	RKISP1_SP_INP_DMA_SP = 0x32000000,	/* DMA -> SP */
	RKISP1_SP_INP_MAX = 0x7fffffff
};

struct cif_isp10_sp_config {
	enum cif_isp10_sp_inp input_sel;
	struct cif_isp10_rsz_config rsz_config;
	struct cif_isp10_dcrop_config crop_config;
};

struct cif_isp10_mp_config {
	struct cif_isp10_rsz_config rsz_config;
	struct cif_isp10_dcrop_config crop_config;
};

struct cif_isp10_stream_subdev {
	enum cif_isp10_stream_id id;
	struct v4l2_subdev subdev;
	/* one source pad, one sink pad */
	struct media_pad pads[2];
	struct v4l2_mbus_framefmt input_fmt;
	struct v4l2_mbus_framefmt output_fmt;
};

struct cif_isp10_path_subdevs {
	struct cif_isp10_stream_subdev mp_sdev;
	struct cif_isp10_mp_config mp_config;
	struct cif_isp10_stream_subdev sp_sdev;
	struct cif_isp10_sp_config sp_config;
};

int register_stream_subdev(struct v4l2_subdev *subdev,
			   struct v4l2_device *v4l2_dev);

void unregister_stream_subdev(struct v4l2_subdev *subdev);

#endif /* _RKISP1_PATH_H */
