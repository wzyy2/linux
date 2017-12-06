.. -*- coding: utf-8; mode: rst -*-

.. _v4l2-meta-fmt-rkisp1-params:

*******************************
V4L2_META_FMT_RK_ISP1_PARAMS
*******************************

Rockchip ISP1 Parameters Data

Description
===========

This format describes input parameters for the Rockchip ISP1.

It uses c-struct :c:type:`rkisp1_isp_params_cfg`, which is defined in
the ``linux/rkisp1-config.h`` header file, see it for details.

The parameters consist of multiple modules.
The module won't be updated if the correspond bit was not set in module_*_update.
