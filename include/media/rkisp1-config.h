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

#ifndef _RKISP1_CONFIG_H
#define _RKISP1_CONFIG_H

#define CIFISP_MODULE_DPCC              BIT(0)
#define CIFISP_MODULE_BLS               BIT(1)
#define CIFISP_MODULE_SDG               BIT(2)
#define CIFISP_MODULE_HST               BIT(3)
#define CIFISP_MODULE_LSC               BIT(4)
#define CIFISP_MODULE_AWB_GAIN          BIT(5)
#define CIFISP_MODULE_FLT               BIT(6)
#define CIFISP_MODULE_BDM               BIT(7)
#define CIFISP_MODULE_CTK               BIT(8)
#define CIFISP_MODULE_GOC               BIT(9)
#define CIFISP_MODULE_CPROC             BIT(10)
#define CIFISP_MODULE_AFC               BIT(11)
#define CIFISP_MODULE_AWB               BIT(12)
#define CIFISP_MODULE_IE                BIT(13)
#define CIFISP_MODULE_AEC               BIT(14)
#define CIFISP_MODULE_WDR               BIT(15)
#define CIFISP_MODULE_DPF               BIT(16)
#define CIFISP_MODULE_DPF_STRENGTH      BIT(17)

#define CIFISP_CTK_COEFF_MAX            0x100
#define CIFISP_CTK_OFFSET_MAX           0x800

#define CIFISP_AE_MEAN_MAX              25
#define CIFISP_HIST_BIN_N_MAX           16
#define CIFISP_AFM_MAX_WINDOWS          3
#define CIFISP_DEGAMMA_CURVE_SIZE       17

#define CIFISP_BDM_MAX_TH               0xFF

/* maximum value for horizontal start address */
#define CIFISP_BLS_START_H_MAX             0x00000FFF
/* maximum value for horizontal stop address */
#define CIFISP_BLS_STOP_H_MAX              0x00000FFF
/* maximum value for vertical start address */
#define CIFISP_BLS_START_V_MAX             0x00000FFF
/* maximum value for vertical stop address */
#define CIFISP_BLS_STOP_V_MAX              0x00000FFF
/* maximum is 2^18 = 262144*/
#define CIFISP_BLS_SAMPLES_MAX             0x00000012
/* maximum value for fixed black level */
#define CIFISP_BLS_FIX_SUB_MAX             0x00000FFF
/* minimum value for fixed black level */
#define CIFISP_BLS_FIX_SUB_MIN             0xFFFFF000
/* 13 bit range (signed)*/
#define CIFISP_BLS_FIX_MASK                0x00001FFF
/* AWB */
#define CIFISP_AWB_MAX_GRID                1
#define CIFISP_AWB_MAX_FRAMES              7

/* Gamma out*/
/* Maximum number of color samples supported */
#define CIFISP_GAMMA_OUT_MAX_SAMPLES       17

/* LSC */
#define CIFISP_LSC_GRAD_TBL_SIZE           8
#define CIFISP_LSC_SIZE_TBL_SIZE           8
/*
 * The following matches the tuning process,
 * not the max capabilities of the chip.
 */
#define	CIFISP_LSC_DATA_TBL_SIZE           289
/* HIST */
#define CIFISP_HISTOGRAM_WEIGHT_GRIDS_SIZE 25

/* DPCC */
#define CIFISP_DPCC_METHODS_MAX       3

/* DPF */
#define CIFISP_DPF_MAX_NLF_COEFFS      17
#define CIFISP_DPF_MAX_SPATIAL_COEFFS  6

/* measurement types */
#define CIFISP_STAT_AWB           BIT(0)
#define CIFISP_STAT_AUTOEXP       BIT(1)
#define CIFISP_STAT_AFM_FIN       BIT(2)
#define CIFISP_STAT_HIST          BIT(3)

enum cifisp_histogram_mode {
	CIFISP_HISTOGRAM_MODE_DISABLE,
	CIFISP_HISTOGRAM_MODE_RGB_COMBINED,
	CIFISP_HISTOGRAM_MODE_R_HISTOGRAM,
	CIFISP_HISTOGRAM_MODE_G_HISTOGRAM,
	CIFISP_HISTOGRAM_MODE_B_HISTOGRAM,
	CIFISP_HISTOGRAM_MODE_Y_HISTOGRAM
};

enum cifisp_exp_meas_mode {
/* < Y = 16 + 0.25R + 0.5G + 0.1094B */
	CIFISP_EXP_MEASURING_MODE_0,
/* < Y = (R + G + B) x (85/256) */
	CIFISP_EXP_MEASURING_MODE_1,
};

struct cifisp_window {
	unsigned short h_offs;
	unsigned short v_offs;
	unsigned short h_size;
	unsigned short v_size;
};

enum cifisp_awb_mode_type {
	CIFISP_AWB_MODE_MANUAL,
	CIFISP_AWB_MODE_RGB,
	CIFISP_AWB_MODE_YCBCR
};

enum cifisp_flt_mode {
	CIFISP_FLT_STATIC_MODE,
	CIFISP_FLT_DYNAMIC_MODE
};

/*
 * BLS fixed subtraction values. The values will be subtracted from the sensor
 * values. Therefore a negative value means addition instead of subtraction!
 */
struct cifisp_bls_fixed_val {
	/*! Fixed (signed!) subtraction value for Bayer pattern R. */
	signed short r;
	/*! Fixed (signed!) subtraction value for Bayer pattern Gr. */
	signed short gr;
	/*! Fixed (signed!) subtraction value for Bayer pattern Gb. */
	signed short gb;
	/*! Fixed (signed!) subtraction value for Bayer pattern B. */
	signed short b;
};

/* Configuration used by black level subtraction */
struct cifisp_bls_config {
	/*
	 * Automatic mode activated means that the measured values
	 * are subtracted.Otherwise the fixed subtraction
	 * values will be subtracted.
	 */
	bool enable_auto;
	unsigned char en_windows;
	struct cifisp_window bls_window1;	/* < Measurement window 1. */
	struct cifisp_window bls_window2;	/* !< Measurement window 2 */
	/*
	 * Set amount of measured pixels for each Bayer position
	 * (A, B,C and D) to 2^bls_samples.
	 */
	unsigned char bls_samples;
	/* !< Fixed subtraction values. */
	struct cifisp_bls_fixed_val fixed_val;
};

struct cifisp_dpcc_methods_config {
	unsigned int method;
	unsigned int line_thresh;
	unsigned int line_mad_fac;
	unsigned int pg_fac;
	unsigned int rnd_thresh;
	unsigned int rg_fac;
};

struct cifisp_dpcc_config {
	unsigned int mode;
	unsigned int output_mode;
	unsigned int set_use;
	struct cifisp_dpcc_methods_config methods[CIFISP_DPCC_METHODS_MAX];
	unsigned int ro_limits;
	unsigned int rnd_offs;
};

struct cifisp_gamma_corr_curve {
	unsigned short gamma_y[CIFISP_DEGAMMA_CURVE_SIZE];
};

struct cifisp_gamma_curve_x_axis_pnts {
	unsigned int gamma_dx0;
	unsigned int gamma_dx1;
};

/* Configuration used by sensor degamma */
struct cifisp_sdg_config {
	struct cifisp_gamma_corr_curve curve_r;
	struct cifisp_gamma_corr_curve curve_g;
	struct cifisp_gamma_corr_curve curve_b;
	struct cifisp_gamma_curve_x_axis_pnts xa_pnts;
};

/* Configuration used by Lens shading correction */
struct cifisp_lsc_config {
	unsigned int r_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];
	unsigned int gr_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];
	unsigned int gb_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];
	unsigned int b_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];

	unsigned int x_grad_tbl[CIFISP_LSC_GRAD_TBL_SIZE];
	unsigned int y_grad_tbl[CIFISP_LSC_GRAD_TBL_SIZE];

	unsigned int x_size_tbl[CIFISP_LSC_SIZE_TBL_SIZE];
	unsigned int y_size_tbl[CIFISP_LSC_SIZE_TBL_SIZE];
	unsigned short config_width;
	unsigned short config_height;
};

struct cifisp_ie_config {
	enum v4l2_colorfx effect;
	unsigned short color_sel;
	/* 3x3 Matrix Coefficients for Emboss Effect 1 */
	unsigned short eff_mat_1;
	/* 3x3 Matrix Coefficients for Emboss Effect 2 */
	unsigned short eff_mat_2;
	/* 3x3 Matrix Coefficients for Emboss 3/Sketch 1 */
	unsigned short eff_mat_3;
	/* 3x3 Matrix Coefficients for Sketch Effect 2 */
	unsigned short eff_mat_4;
	/* 3x3 Matrix Coefficients for Sketch Effect 3 */
	unsigned short eff_mat_5;
	/* Chrominance increment values of tint (used for sepia effect) */
	unsigned short eff_tint;
};

struct cifisp_cproc_config {
	unsigned char c_out_range;
	unsigned char y_in_range;
	unsigned char y_out_range;
	unsigned char contrast;
	unsigned char brightness;
	unsigned char sat;
	unsigned char hue;
};

/* Configuration used by auto white balance */
struct cifisp_awb_meas_config {
	/*
	 * white balance measurement window (in pixels)
	 * Note: currently the h and v offsets are mapped to grid offsets
	 */
	struct cifisp_window awb_wnd;
	enum cifisp_awb_mode_type awb_mode;
	/*
	 * only pixels values < max_y contribute to awb measurement
	 * (set to 0 to disable this feature)
	 */
	unsigned char max_y;
	/* only pixels values > min_y contribute to awb measurement */
	unsigned char min_y;
	/*
	 * Chrominance sum maximum value, only consider pixels with Cb+Cr
	 * smaller than threshold for awb measurements
	 */
	unsigned char max_csum;
	/*
	 * Chrominance minimum value, only consider pixels with Cb/Cr
	 * each greater than threshold value for awb measurements
	 */
	unsigned char min_c;
	/*
	 * number of frames - 1 used for mean value calculation
	 * (ucFrames=0 means 1 Frame)
	 */
	unsigned char frames;
	/* reference Cr value for AWB regulation, target for AWB */
	unsigned char awb_ref_cr;
	/* reference Cb value for AWB regulation, target for AWB */
	unsigned char awb_ref_cb;
	bool enable_ymax_cmp;
};

struct cifisp_awb_gain_config {
	unsigned short gain_red;
	unsigned short gain_green_r;
	unsigned short gain_blue;
	unsigned short gain_green_b;
};

/* Configuration used by ISP filtering */
struct cifisp_flt_config {
	enum cifisp_flt_mode mode;	/* ISP_FILT_MODE register fields */
	unsigned char grn_stage1;	/* ISP_FILT_MODE register fields */
	unsigned char chr_h_mode;	/* ISP_FILT_MODE register fields */
	unsigned char chr_v_mode;	/* ISP_FILT_MODE register fields */
	unsigned int thresh_bl0;
	unsigned int thresh_bl1;
	unsigned int thresh_sh0;
	unsigned int thresh_sh1;
	unsigned int lum_weight;
	unsigned int fac_sh1;
	unsigned int fac_sh0;
	unsigned int fac_mid;
	unsigned int fac_bl0;
	unsigned int fac_bl1;
};

/* Configuration used by Bayer DeMosaic */
struct cifisp_bdm_config {
	unsigned char demosaic_th;
};

/* Configuration used by Cross Talk correction */
struct cifisp_ctk_config {
	unsigned short coeff0;
	unsigned short coeff1;
	unsigned short coeff2;
	unsigned short coeff3;
	unsigned short coeff4;
	unsigned short coeff5;
	unsigned short coeff6;
	unsigned short coeff7;
	unsigned short coeff8;
	/* offset for the crosstalk correction matrix */
	unsigned short ct_offset_r;
	unsigned short ct_offset_g;
	unsigned short ct_offset_b;
};

enum cifisp_goc_mode {
	CIFISP_GOC_MODE_LOGARITHMIC,
	CIFISP_GOC_MODE_EQUIDISTANT
};

/* Configuration used by Gamma Out correction */
struct cifisp_goc_config {
	enum cifisp_goc_mode mode;
	unsigned short gamma_y[CIFISP_GAMMA_OUT_MAX_SAMPLES];
};

/* Configuration used by Histogram */
struct cifisp_hst_config {
	enum cifisp_histogram_mode mode;
	unsigned char histogram_predivider;
	struct cifisp_window meas_window;
	unsigned char hist_weight[CIFISP_HISTOGRAM_WEIGHT_GRIDS_SIZE];
};

/* Configuration used by Auto Exposure Control */
struct cifisp_aec_config {
	enum cifisp_exp_meas_mode mode;
	u32 autostop;
	struct cifisp_window meas_window;
};

struct cifisp_afc_config {
	unsigned char num_afm_win;	/* max CIFISP_AFM_MAX_WINDOWS */
	struct cifisp_window afm_win[CIFISP_AFM_MAX_WINDOWS];
	unsigned int thres;
	unsigned int var_shift;
};

enum cifisp_dpf_gain_usage {
/* don't use any gains in preprocessing stage */
	CIFISP_DPF_GAIN_USAGE_DISABLED,
/* use only the noise function gains  from registers DPF_NF_GAIN_R, ... */
	CIFISP_DPF_GAIN_USAGE_NF_GAINS,
/* use only the gains from LSC module */
	CIFISP_DPF_GAIN_USAGE_LSC_GAINS,
/* use the moise function gains and the gains from LSC module */
	CIFISP_DPF_GAIN_USAGE_NF_LSC_GAINS,
/* use only the gains from AWB module */
	CIFISP_DPF_GAIN_USAGE_AWB_GAINS,
/* use the gains from AWB and LSC module */
	CIFISP_DPF_GAIN_USAGE_AWB_LSC_GAINS,
/* upper border (only for an internal evaluation) */
	CIFISP_DPF_GAIN_USAGE_MAX
};

enum cifisp_dpf_rb_filtersize {
/* red and blue filter kernel size 13x9 (means 7x5 active pixel) */
	CIFISP_DPF_RB_FILTERSIZE_13x9,
/* red and blue filter kernel size 9x9 (means 5x5 active pixel) */
	CIFISP_DPF_RB_FILTERSIZE_9x9,
};

enum cifisp_dpf_nll_scale_mode {
/* use a linear scaling */
	CIFISP_NLL_SCALE_LINEAR,
/* use a logarithmic scaling */
	CIFISP_NLL_SCALE_LOGARITHMIC,
};

struct cifisp_dpf_nll {
	unsigned short coeff[CIFISP_DPF_MAX_NLF_COEFFS];
	enum cifisp_dpf_nll_scale_mode scale_mode;
};

struct cifisp_dpf_rb_flt {
	enum cifisp_dpf_rb_filtersize fltsize;
	unsigned char spatial_coeff[CIFISP_DPF_MAX_SPATIAL_COEFFS];
	bool r_enable;
	bool b_enable;
};

struct cifisp_dpf_g_flt {
	unsigned char spatial_coeff[CIFISP_DPF_MAX_SPATIAL_COEFFS];
	bool gr_enable;
	bool gb_enable;
};

struct cifisp_dpf_gain {
	enum cifisp_dpf_gain_usage mode;
	unsigned short nf_r_gain;
	unsigned short nf_b_gain;
	unsigned short nf_gr_gain;
	unsigned short nf_gb_gain;
};

struct cifisp_dpf_config {
	struct cifisp_dpf_gain gain;
	struct cifisp_dpf_g_flt g_flt;
	struct cifisp_dpf_rb_flt rb_flt;
	struct cifisp_dpf_nll nll;
};

struct cifisp_dpf_strength_config {
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

struct cifisp_isp_other_cfg {
	struct cifisp_dpcc_config dpcc_config;
	struct cifisp_bls_config bls_config;
	struct cifisp_sdg_config sdg_config;
	struct cifisp_lsc_config lsc_config;
	struct cifisp_awb_gain_config awb_gain_config;
	struct cifisp_flt_config flt_config;
	struct cifisp_bdm_config bdm_config;
	struct cifisp_ctk_config ctk_config;
	struct cifisp_goc_config goc_config;
	struct cifisp_dpf_config dpf_config;
	struct cifisp_dpf_strength_config dpf_strength_config;
	/* post process configs, including hue, sharpness, brightness, contrast, color effect */
	/* TODO: removed from here ? set by standard v4l2 controls ? */
	struct cifisp_cproc_config cproc_config;
	struct cifisp_ie_config ie_config;
};

struct cifisp_isp_meas_cfg {
	struct cifisp_awb_meas_config awb_meas_config;
	struct cifisp_hst_config hst_config;
	struct cifisp_aec_config aec_config;
	struct cifisp_afc_config afc_config;
};

struct cifisp_isp_params_cfg {
	/* mask the enable bits of which module  should be updated */
	unsigned int module_en_update;
	/* mask the enable value of each module, only update the module which correspond
	  * bit was set in module_en_update
	  */
	unsigned int module_ens;
	/* mask the config bits of which module  should be updated */
	unsigned int module_cfg_update;

	struct cifisp_isp_meas_cfg meas;
	struct cifisp_isp_other_cfg others;
};

/* measurement statistics */

struct cifisp_awb_meas {
	unsigned int cnt;
	unsigned char mean_y_or_g;
	unsigned char mean_cb_or_b;
	unsigned char mean_cr_or_r;
};

struct cifisp_awb_stat {
	struct cifisp_awb_meas awb_mean[CIFISP_AWB_MAX_GRID];
};

struct cifisp_hist_stat {
	unsigned short hist_bins[CIFISP_HIST_BIN_N_MAX];
};

/*! BLS mean measured values */
struct cifisp_bls_meas_val {
	/*! Mean measured value for Bayer pattern R. */
	unsigned short meas_r;
	/*! Mean measured value for Bayer pattern Gr. */
	unsigned short meas_gr;
	/*! Mean measured value for Bayer pattern Gb. */
	unsigned short meas_gb;
	/*! Mean measured value for Bayer pattern B. */
	unsigned short meas_b;
};

struct cifisp_ae_stat {
	unsigned char exp_mean[CIFISP_AE_MEAN_MAX];
	struct cifisp_bls_meas_val bls_val;	/* available wit exposure results */
};

struct cifisp_af_meas_val {
	unsigned int sum;
	unsigned int lum;
};

struct cifisp_af_stat {
	struct cifisp_af_meas_val window[CIFISP_AFM_MAX_WINDOWS];
};

struct cifisp_stat {
	struct cifisp_awb_stat awb;
	struct cifisp_ae_stat ae;
	struct cifisp_af_stat af;
	struct cifisp_hist_stat hist;
};

struct cifisp_stat_buffer {
	unsigned int meas_type;
	unsigned int frame_id;
	struct cifisp_stat params;
};

#endif /* _RKISP1_CONFIG_H */
