// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip MIPI Synopsys DPHY driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 */

#ifndef __MIPI_DPHY_SY_H__
#define __MIPI_DPHY_SY_H__

#include <media/v4l2-subdev.h>

void rkisp1_set_mipi_dphy_sy_lanes(struct v4l2_subdev *dphy, int lanes);

#endif /* __RKISP1_MIPI_DPHY_SY_H__ */
