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

#ifndef _RKISP1_ISP_H
#define _RKISP1_ISP_H

#include <media/rkisp1-config.h>
#include "common.h"

struct rkisp1_device;

/* simple pool, max item num size is equal to max pool size subs 1 */
#define RKISP1_ISP_PARAMS_POOL_MAX (4 + 1)

struct rkisp1_isp_params_item {
	struct cifisp_isp_params_cfg configs;

	/* frme id of the params taken effect, set when params are applied */
	unsigned int frame_id;
	unsigned int module_enables;
};

struct rkisp1_isp_params_pool {
	struct rkisp1_isp_params_item items[RKISP1_ISP_PARAMS_POOL_MAX];
	/* last configs queued in pool, to be applied */
	unsigned int last_cfg_ind;
	/* current used configs, updated after applied */
	unsigned int cur_cfg_ind;
};

struct rkisp1_isp_params_vdev {
	struct rkisp1_vdev_node vnode;
	/* Current ISP parameters */
	spinlock_t config_lock;
	struct rkisp1_isp_params_pool params_pool;

	bool streamon;
	struct rkisp1_device *dev;
	unsigned int frame_id;    /* current frame id */

	bool cif_ism_cropping;
	enum v4l2_quantization quantization;
	enum cif_fmt_raw_pat_type raw_type;

	/* input resolution needed for LSC param check */
	unsigned int isp_acq_width;
	unsigned int isp_acq_height;
	unsigned int active_lsc_width;
	unsigned int active_lsc_height;

};

/* config params before ISP streaming */
void rkisp1_configure_isp(struct rkisp1_isp_params_vdev *params_vdev,
			  struct rkisp1_fmt *in_fmt,
			  enum v4l2_quantization quantization);
void rkisp1_disable_isp(struct rkisp1_isp_params_vdev *params_vdev);

int rkisp1_register_params_vdev(struct rkisp1_isp_params_vdev *params_vdev,
				struct v4l2_device *v4l2_dev,
				struct rkisp1_device *dev);

void rkisp1_unregister_params_vdev(struct rkisp1_isp_params_vdev *params_vdev);

unsigned int rkisp1_get_active_meas(struct rkisp1_isp_params_vdev *params_vdev,
				    unsigned int frame_id);

void rkisp1_params_v_start(struct rkisp1_isp_params_vdev *params_vdev);

void rkisp1_params_isr(struct rkisp1_isp_params_vdev *params_vdev, u32 isp_mis);

#endif /* _RKISP1_ISP_H */
