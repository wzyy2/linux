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

#ifndef _V4L2_CONFIG_ROCKCHIP_H
#define _V4L2_CONFIG_ROCKCHIP_H

#define CAMERA_STRLEN         32
#define CAMERA_METADATA_LEN   (2 * PAGE_SIZE)

/* Sensor resolution specific data for AE calculation.*/
struct isp_supplemental_sensor_mode_data {
	unsigned int coarse_integration_time_min;
	unsigned int coarse_integration_time_max_margin;
	unsigned int fine_integration_time_min;
	unsigned int fine_integration_time_max_margin;
	unsigned int frame_length_lines;
	unsigned int line_length_pck;
	unsigned int vt_pix_clk_freq_hz;
	unsigned int crop_horizontal_start;	/* Sensor crop start cord. (x0,y0) */
	unsigned int crop_vertical_start;
	unsigned int crop_horizontal_end;	/* Sensor crop end cord. (x1,y1) */
	unsigned int crop_vertical_end;
	unsigned int sensor_output_width;	/* input size to ISP */
	unsigned int sensor_output_height;
	unsigned int isp_input_horizontal_start;	/* cif isp input */
	unsigned int isp_input_vertical_start;
	unsigned int isp_input_width;
	unsigned int isp_input_height;
	unsigned int isp_output_width;	/* cif isp output */
	unsigned int isp_output_height;
	unsigned char binning_factor_x;	/* horizontal binning factor used */
	unsigned char binning_factor_y;	/* vertical binning factor used */
	unsigned char exposure_valid_frame;
	unsigned short gain;
};

#endif /* _V4L2_CONFIG_ROCKCHIP_H */
