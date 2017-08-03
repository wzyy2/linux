/*
 * Rockchip MIPI PHY driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include "mipiphy.h"

#define RK3288_GRF_SOC_CON6	0x025c
#define RK3288_GRF_SOC_CON8	0x0264
#define RK3288_GRF_SOC_CON9	0x0268
#define RK3288_GRF_SOC_CON10	0x026c
#define RK3288_GRF_SOC_CON14	0x027c
#define RK3288_GRF_SOC_STATUS21	0x02d4
#define RK3288_GRF_IO_VSEL	0x0380
#define RK3288_GRF_SOC_CON15	0x03a4

#define RK3399_GRF_SOC_CON9	0x6224
#define RK3399_GRF_SOC_CON21	0x6254
#define RK3399_GRF_SOC_CON22	0x6258
#define RK3399_GRF_SOC_CON23	0x625c
#define RK3399_GRF_SOC_CON24	0x6260
#define RK3399_GRF_SOC_CON25	0x6264
#define RK3399_GRF_SOC_STATUS1	0xe2a4

#define HIWORD_UPDATE(val, mask, shift) \
	((val) << (shift) | (mask) << ((shift) + 16))

enum phy_reg_id {
	GRF_DPHY_RX0_TURNDISABLE = 0,
	GRF_DPHY_RX0_FORCERXMODE,
	GRF_DPHY_RX0_FORCETXSTOPMODE,
	GRF_DPHY_RX0_ENABLE,
	GRF_DPHY_RX0_TESTCLR,
	GRF_DPHY_RX0_TESTCLK,
	GRF_DPHY_RX0_TESTEN,
	GRF_DPHY_RX0_TESTDIN,
	GRF_DPHY_RX0_TURNREQUEST,
	GRF_DPHY_RX0_TESTDOUT,
	GRF_DPHY_TX0_TURNDISABLE,
	GRF_DPHY_TX0_FORCERXMODE,
	GRF_DPHY_TX0_FORCETXSTOPMODE,
	GRF_DPHY_TX0_TURNREQUEST,
	GRF_DPHY_TX1RX1_TURNDISABLE,
	GRF_DPHY_TX1RX1_FORCERXMODE,
	GRF_DPHY_TX1RX1_FORCETXSTOPMODE,
	GRF_DPHY_TX1RX1_ENABLE,
	GRF_DPHY_TX1RX1_MASTERSLAVEZ,
	GRF_DPHY_TX1RX1_BASEDIR,
	GRF_DPHY_TX1RX1_ENABLECLK,
	GRF_DPHY_TX1RX1_TURNREQUEST,
	GRF_DPHY_RX1_SRC_SEL,
	/******below is for rk3288 only********/
	GRF_CON_DISABLE_ISP,
	GRF_CON_ISP_DPHY_SEL,
	GRF_DSI_CSI_TESTBUS_SEL,
	GRF_DVP_V18SEL,
	/******below is for rk3399 only*******/
	GRF_DISABLE_ISP0,
	GRF_DISABLE_ISP1,
	GRF_DPHY_RX0_CLK_INV_SEL,
	GRF_DPHY_RX1_CLK_INV_SEL,
};

struct phy_reg {
	u32 offset;
	u32 mask;
	u32 shift;
};
#define PHY_REG(_offset, _width, _shift) \
	{ .offset = _offset, .mask = BIT(_width) - 1, .shift = _shift, }

static const struct phy_reg rk3399_grf_dphy_regs[] = {
	[GRF_DPHY_RX0_TURNREQUEST] = PHY_REG(RK3399_GRF_SOC_CON9, 4, 0),
	[GRF_DISABLE_ISP0] = PHY_REG(RK3399_GRF_SOC_CON9, 1, 8),
	[GRF_DISABLE_ISP1] = PHY_REG(RK3399_GRF_SOC_CON9, 1, 9),
	[GRF_DPHY_RX0_CLK_INV_SEL] = PHY_REG(RK3399_GRF_SOC_CON9, 1, 10),
	[GRF_DPHY_RX1_CLK_INV_SEL] = PHY_REG(RK3399_GRF_SOC_CON9, 1, 11),
	[GRF_DPHY_RX0_ENABLE] = PHY_REG(RK3399_GRF_SOC_CON21, 4, 0),
	[GRF_DPHY_RX0_FORCERXMODE] = PHY_REG(RK3399_GRF_SOC_CON21, 4, 4),
	[GRF_DPHY_RX0_FORCETXSTOPMODE] = PHY_REG(RK3399_GRF_SOC_CON21, 4, 8),
	[GRF_DPHY_RX0_TURNDISABLE] = PHY_REG(RK3399_GRF_SOC_CON21, 4, 12),
	[GRF_DPHY_TX0_FORCERXMODE] = PHY_REG(RK3399_GRF_SOC_CON22, 4, 0),
	[GRF_DPHY_TX0_FORCETXSTOPMODE] = PHY_REG(RK3399_GRF_SOC_CON22, 4, 4),
	[GRF_DPHY_TX0_TURNDISABLE] = PHY_REG(RK3399_GRF_SOC_CON22, 4, 8),
	[GRF_DPHY_TX0_TURNREQUEST] = PHY_REG(RK3399_GRF_SOC_CON22, 4, 12),
	[GRF_DPHY_TX1RX1_ENABLE] = PHY_REG(RK3399_GRF_SOC_CON23, 4, 0),
	[GRF_DPHY_TX1RX1_FORCERXMODE] = PHY_REG(RK3399_GRF_SOC_CON23, 4, 4),
	[GRF_DPHY_TX1RX1_FORCETXSTOPMODE] = PHY_REG(RK3399_GRF_SOC_CON23, 4, 8),
	[GRF_DPHY_TX1RX1_TURNDISABLE] = PHY_REG(RK3399_GRF_SOC_CON23, 4, 12),
	[GRF_DPHY_TX1RX1_TURNREQUEST] = PHY_REG(RK3399_GRF_SOC_CON24, 4, 0),
	[GRF_DPHY_RX1_SRC_SEL] = PHY_REG(RK3399_GRF_SOC_CON24, 1, 4),
	[GRF_DPHY_TX1RX1_BASEDIR] = PHY_REG(RK3399_GRF_SOC_CON24, 1, 5),
	[GRF_DPHY_TX1RX1_ENABLECLK] = PHY_REG(RK3399_GRF_SOC_CON24, 1, 6),
	[GRF_DPHY_TX1RX1_MASTERSLAVEZ] = PHY_REG(RK3399_GRF_SOC_CON24, 1, 7),
	[GRF_DPHY_RX0_TESTDIN] = PHY_REG(RK3399_GRF_SOC_CON25, 8, 0),
	[GRF_DPHY_RX0_TESTEN] = PHY_REG(RK3399_GRF_SOC_CON25, 1, 8),
	[GRF_DPHY_RX0_TESTCLK] = PHY_REG(RK3399_GRF_SOC_CON25, 1, 9),
	[GRF_DPHY_RX0_TESTCLR] = PHY_REG(RK3399_GRF_SOC_CON25, 1, 10),
	[GRF_DPHY_RX0_TESTDOUT] = PHY_REG(RK3399_GRF_SOC_STATUS1, 8, 0),
};

static const struct phy_reg rk3288_grf_dphy_regs[] = {
	[GRF_CON_DISABLE_ISP] = PHY_REG(RK3288_GRF_SOC_CON6, 1, 0),
	[GRF_CON_ISP_DPHY_SEL] = PHY_REG(RK3288_GRF_SOC_CON6, 1, 1),
	[GRF_DSI_CSI_TESTBUS_SEL] = PHY_REG(RK3288_GRF_SOC_CON6, 1, 14),
	[GRF_DPHY_TX0_TURNDISABLE] = PHY_REG(RK3288_GRF_SOC_CON8, 4, 0),
	[GRF_DPHY_TX0_FORCERXMODE] = PHY_REG(RK3288_GRF_SOC_CON8, 4, 4),
	[GRF_DPHY_TX0_FORCETXSTOPMODE] = PHY_REG(RK3288_GRF_SOC_CON8, 4, 8),
	[GRF_DPHY_TX1RX1_TURNDISABLE] = PHY_REG(RK3288_GRF_SOC_CON9, 4, 0),
	[GRF_DPHY_TX1RX1_FORCERXMODE] = PHY_REG(RK3288_GRF_SOC_CON9, 4, 4),
	[GRF_DPHY_TX1RX1_FORCETXSTOPMODE] = PHY_REG(RK3288_GRF_SOC_CON9, 4, 8),
	[GRF_DPHY_TX1RX1_ENABLE] = PHY_REG(RK3288_GRF_SOC_CON9, 4, 12),
	[GRF_DPHY_RX0_TURNDISABLE] = PHY_REG(RK3288_GRF_SOC_CON10, 4, 0),
	[GRF_DPHY_RX0_FORCERXMODE] = PHY_REG(RK3288_GRF_SOC_CON10, 4, 4),
	[GRF_DPHY_RX0_FORCETXSTOPMODE] = PHY_REG(RK3288_GRF_SOC_CON10, 4, 8),
	[GRF_DPHY_RX0_ENABLE] = PHY_REG(RK3288_GRF_SOC_CON10, 4, 12),
	[GRF_DPHY_RX0_TESTCLR] = PHY_REG(RK3288_GRF_SOC_CON14, 1, 0),
	[GRF_DPHY_RX0_TESTCLK] = PHY_REG(RK3288_GRF_SOC_CON14, 1, 1),
	[GRF_DPHY_RX0_TESTEN] = PHY_REG(RK3288_GRF_SOC_CON14, 1, 2),
	[GRF_DPHY_RX0_TESTDIN] = PHY_REG(RK3288_GRF_SOC_CON14, 8, 3),
	[GRF_DPHY_TX1RX1_ENABLECLK] = PHY_REG(RK3288_GRF_SOC_CON14, 1, 12),
	[GRF_DPHY_RX1_SRC_SEL] = PHY_REG(RK3288_GRF_SOC_CON14, 1, 13),
	[GRF_DPHY_TX1RX1_MASTERSLAVEZ] = PHY_REG(RK3288_GRF_SOC_CON14, 1, 14),
	[GRF_DPHY_TX1RX1_BASEDIR] = PHY_REG(RK3288_GRF_SOC_CON14, 1, 15),
	[GRF_DPHY_RX0_TURNREQUEST] = PHY_REG(RK3288_GRF_SOC_CON15, 4, 0),
	[GRF_DPHY_TX1RX1_TURNREQUEST] = PHY_REG(RK3288_GRF_SOC_CON15, 4, 4),
	[GRF_DPHY_TX0_TURNREQUEST] = PHY_REG(RK3288_GRF_SOC_CON15, 3, 8),
	[GRF_DVP_V18SEL] = PHY_REG(RK3288_GRF_IO_VSEL, 1, 1),
	[GRF_DPHY_RX0_TESTDOUT] = PHY_REG(RK3288_GRF_SOC_STATUS21, 8, 0),
};

struct hsfreqrange {
	u32 range_h;
	u8 cfg_bit;
};

struct phy_drv_data {
	const char * const *clks;
	int clk_tab_size;
	const struct hsfreqrange *hsfreq_range;
	int range_tab_size;
	const struct phy_reg *regs;
};



#define MIPIPHY_STATE_OFF 0
#define MIPIPHY_STATE_POWERED 1
#define MIPIPHY_STATE_STREAMING 2

#define MAX_PHY_CLK		8
struct mipiphy_priv {
	struct device *dev;
	struct regmap *regmap_grf;
	const struct phy_reg *grf_regs;
	struct clk *clks[MAX_PHY_CLK];
	struct phy_drv_data *drv_data;
	u32 lanes;
	u32 bit_rate;	/*depend on sensor link rate*/
	const struct of_device_id *of_id;
	int state;
	struct v4l2_subdev sd;
	struct media_pad pads[MIPIPHY_PADS_NUM];
};

static inline struct mipiphy_priv* to_mipiphy(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct mipiphy_priv, sd);
}

static inline void write_reg(struct mipiphy_priv *priv, int index, u8 value)
{
	const struct phy_reg *grf_regs = priv->grf_regs;
	struct regmap *grf = priv->regmap_grf;
	const struct phy_reg *preg = &grf_regs[index];
	unsigned int val = HIWORD_UPDATE(value, preg->mask, preg->shift);

	WARN_ON(!preg->offset);
	regmap_write(grf, preg->offset, val);
}

static void mipiphy_wr_reg(struct mipiphy_priv *priv, u8 test_code, u8 test_data)
{
	/*
	 * With the falling edge on TESTCLK, the TESTDIN[7:0] signal content
	 * is latched internally as the current test code. Test data is
	 * programmed internally by rising edge on TESTCLK.
	 */
	write_reg(priv, GRF_DPHY_RX0_TESTCLK, 1);
	write_reg(priv, GRF_DPHY_RX0_TESTDIN, test_code);
	write_reg(priv, GRF_DPHY_RX0_TESTEN, 1);
	write_reg(priv, GRF_DPHY_RX0_TESTCLK, 0);
	write_reg(priv, GRF_DPHY_RX0_TESTEN, 0);
	write_reg(priv, GRF_DPHY_RX0_TESTDIN, test_data);
	write_reg(priv, GRF_DPHY_RX0_TESTCLK, 1);
}

static int mipiphy_s_stream_start(struct v4l2_subdev *sd)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);
	const struct hsfreqrange *table = priv->drv_data->hsfreq_range;
	int hsfreq, i, size = priv->drv_data->range_tab_size;

	hsfreq = 0;
	for (i = 0; i < size; i++) {
		if (table[i].range_h >= priv->bit_rate) {
			hsfreq = table[i].cfg_bit;
			break;
		}
	}

	write_reg(priv, GRF_DPHY_RX0_FORCERXMODE, 0);
	write_reg(priv, GRF_DPHY_RX0_FORCETXSTOPMODE, 0);
	/* Disable lan turn around, which is ignored in receive mode */
	write_reg(priv, GRF_DPHY_RX0_TURNREQUEST, 0);
	write_reg(priv, GRF_DPHY_RX0_TURNDISABLE, 0xf);

	write_reg(priv, GRF_DPHY_RX0_ENABLE, GENMASK(priv->lanes - 1, 0));

	/* phy start */
	write_reg(priv, GRF_DPHY_RX0_TESTCLK, 1);
	write_reg(priv, GRF_DPHY_RX0_TESTCLR, 1);
	usleep_range(100, 150);
	write_reg(priv, GRF_DPHY_RX0_TESTCLR, 0);
	usleep_range(100, 150);

	/* set clock lane */
	/* HS hsfreqrange & lane 0  settle bypass */
	mipiphy_wr_reg(priv, 0x34, 0);

	if (strcmp(priv->of_id->compatible, "rockchip,rk3288-isp-mipi-phy")) {
		/*** HS RX Control of lane0 ***/
		mipiphy_wr_reg(priv, 0x44, hsfreq << 1);
		/*** HS RX Control of lane1 ***/
		mipiphy_wr_reg(priv, 0x54, 0);
		/*** HS RX Control of lane2 ***/
		mipiphy_wr_reg(priv, 0x84, 0);
		/*** HS RX Control of lane3 ***/
		mipiphy_wr_reg(priv, 0x94, 0);
		/*** HS RX Data Lanes Settle State Time Control ***/
		mipiphy_wr_reg(priv, 0x75, 0x04);
	}

	/* Normal operation */
	mipiphy_wr_reg(priv, 0x0, 0);
	return 0;
}

static int mipiphy_s_stream_stop(struct v4l2_subdev *sd)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);

	if (priv->state != MIPIPHY_STATE_STREAMING)
		return 0;
	return 0;
}

static int mipiphy_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);

	if(on){
		if (priv->state == MIPIPHY_STATE_STREAMING)
			return 0;
		mipiphy_s_stream_start(sd);
		priv->state = MIPIPHY_STATE_STREAMING;
	}else{
		if (priv->state != MIPIPHY_STATE_STREAMING)
			return 0;
		mipiphy_s_stream_stop(sd);
		priv->state = MIPIPHY_STATE_POWERED;
	}
	return 0;
}

static int mipiphy_off(struct mipiphy_priv *priv)
{
	struct clk *clk;
	int i;

	if (priv->state == MIPIPHY_STATE_OFF)
		return 0;
	for (i = MAX_PHY_CLK - 1; i >= 0; i--) {
		clk = priv->clks[i];
		if (IS_ERR(clk))
			continue;
		clk_disable_unprepare(clk);
	}
	priv->state = MIPIPHY_STATE_OFF;
	return 0;
}

static int mipiphy_on(struct mipiphy_priv *priv)
{
	struct clk *clk;
	int i, ret = -EINVAL;

	if (priv->state > MIPIPHY_STATE_OFF)
		return 0;
	for (i = 0; i < MAX_PHY_CLK; i++) {
		clk = priv->clks[i];
		if (IS_ERR(clk))
			return 0;

		ret = clk_prepare_enable(clk);
		if (ret < 0)
			goto err;

	}
	priv->state = MIPIPHY_STATE_POWERED;
	return 0;
err:
	while (i--)
		clk_disable_unprepare(priv->clks[i]);
	priv->state = MIPIPHY_STATE_OFF;
	return ret;
}

static int mipiphy_s_power(struct v4l2_subdev *sd, int on)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);

	priv->bit_rate = 1000;
	if (on)
		return mipiphy_on(priv);
	else
		return mipiphy_off(priv);
}

static const struct v4l2_subdev_core_ops mipiphy_core_ops = {
	.s_power = mipiphy_s_power,
};

static const struct v4l2_subdev_video_ops mipiphy_video_ops = {
	.s_stream = mipiphy_s_stream,
};

static const struct v4l2_subdev_ops mipiphy_subdev_ops = {
	.core = &mipiphy_core_ops,
	.video = &mipiphy_video_ops,
};

/*****below table got from D phy databook don't modify them*****/
static const struct hsfreqrange rk3288_mipi_phy_hsfreq_range[] = {
	{  90, 0x00}, { 100, 0x10}, { 110, 0x20}, { 130, 0x01},
	{ 140, 0x11}, { 150, 0x21}, { 170, 0x02}, { 180, 0x12},
	{ 200, 0x22}, { 220, 0x03}, { 240, 0x13}, { 250, 0x23},
	{ 270, 0x04}, { 300, 0x14}, { 330, 0x05}, { 360, 0x15},
	{ 400, 0x25}, { 450, 0x06}, { 500, 0x16}, { 550, 0x07},
	{ 600, 0x17}, { 650, 0x08}, { 700, 0x18}, { 750, 0x09},
	{ 800, 0x19}, { 850, 0x29}, { 900, 0x39}, { 950, 0x0a},
	{1000, 0x1a}
};

static const struct hsfreqrange rk3399_mipi_phy_hsfreq_range[] = {
	{  89, 0x00}, {  99, 0x10}, { 109, 0x20}, { 129, 0x01},
	{ 139, 0x11}, { 149, 0x21}, { 169, 0x02}, { 179, 0x12},
	{ 199, 0x22}, { 219, 0x03}, { 239, 0x13}, { 249, 0x23},
	{ 269, 0x04}, { 299, 0x14}, { 329, 0x05}, { 359, 0x15},
	{ 399, 0x25}, { 449, 0x06}, { 499, 0x16}, { 549, 0x07},
	{ 599, 0x17}, { 649, 0x08}, { 699, 0x18}, { 749, 0x09},
	{ 799, 0x19}, { 849, 0x29}, { 899, 0x39}, { 949, 0x0a},
	{ 999, 0x1a}, {1049, 0x2a}, {1099, 0x3a}, {1149, 0x0b},
	{1199, 0x1b}, {1249, 0x2b}, {1299, 0x3b}, {1349, 0x0c},
	{1399, 0x1c}, {1449, 0x2c}, {1500, 0x3c}
};

static const char * const rk3399_mipiphy_clks[] = {
	"phy-ref",
	"phy-cfg",
};

static const char * const rk3288_mipiphy_clks[] = {
	"clk-mipi_csi",
};

static const struct phy_drv_data rk3288_mipiphy_drv_data = {
	.clks = rk3288_mipiphy_clks,
	.clk_tab_size = ARRAY_SIZE(rk3288_mipiphy_clks),
	.hsfreq_range = rk3288_mipi_phy_hsfreq_range,
	.range_tab_size = ARRAY_SIZE(rk3288_mipi_phy_hsfreq_range),
	.regs = rk3288_grf_dphy_regs,
};

static const struct phy_drv_data rk3399_mipiphy_drv_data = {
	.clks = rk3399_mipiphy_clks,
	.clk_tab_size = ARRAY_SIZE(rk3399_mipiphy_clks),
	.hsfreq_range = rk3399_mipi_phy_hsfreq_range,
	.range_tab_size = ARRAY_SIZE(rk3399_mipi_phy_hsfreq_range),
	.regs = rk3399_grf_dphy_regs,
};

static const struct of_device_id rockchip_mipiphy_match_id[] = {
	{ 
	.compatible = "rockchip,rk3399-isp-mipi-phy",
	.data = &rk3399_mipiphy_drv_data,
	},{ 
	.compatible = "rockchip,rk3288-isp-mipi-phy",
	.data = &rk3288_mipiphy_drv_data,
	},{}
};
MODULE_DEVICE_TABLE(of, rockchip_mipiphy_match_id);

static int rockchip_mipiphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_subdev *sd;
	struct mipiphy_priv *priv;
	struct regmap *grf;
	const struct of_device_id *of_id;
	struct phy_drv_data *drv_data;
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;
	priv->state = MIPIPHY_STATE_OFF;

	of_id = of_match_device(rockchip_mipiphy_match_id, dev);
	if (!of_id)
		return -EINVAL;
	priv->of_id = of_id;

	grf = syscon_regmap_lookup_by_phandle(dev->of_node, "rockchip,grf");
	if (IS_ERR(grf)) {
		dev_err(dev, "Can't find GRF syscon\n");
		return -ENODEV;
	}
	priv->regmap_grf = grf;

	if (of_property_read_u32(dev->of_node, "bus-width", &priv->lanes)){
		dev_err(dev, "Can't get bus-width\n");
		return -EINVAL;
	}

	drv_data = (struct phy_drv_data *)of_id->data;
	for (i = 0; i < drv_data->clk_tab_size; i++) {
		struct clk *clk = devm_clk_get(dev, drv_data->clks[i]);

		if (IS_ERR(clk)) {
			dev_err(dev, "failed to get %s\n", drv_data->clks[i]);
			return PTR_ERR(clk);
		}
		priv->clks[i] = clk;
	}
	priv->grf_regs = drv_data->regs;
	priv->drv_data = drv_data;

	sd = &priv->sd;
	v4l2_subdev_init(sd, &mipiphy_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "rockchip-mipiphy");
	sd->dev = dev;

	priv->pads[MIPIPHY_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[MIPIPHY_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	platform_set_drvdata(pdev, &sd->entity);
	ret = media_entity_init(&sd->entity, MIPIPHY_PADS_NUM, priv->pads, 0);
	if (ret)
		return ret;

	v4l2_info(sd, "%s initialized, bus width: %d, ", of_id->compatible, priv->lanes);
	return 0;
}

static struct platform_driver rockchip_isp_mipiphy_driver = {
	.probe = rockchip_mipiphy_probe,
	//TODO remove
	.driver = {
		   .name = "rockchip-isp-mipi-phy",
		   .of_match_table = rockchip_mipiphy_match_id,
	},
};

module_platform_driver(rockchip_isp_mipiphy_driver);

MODULE_AUTHOR("Rockchip Camera/ISP team");
MODULE_DESCRIPTION("Rockchip ISP MIPI PHY driver");
MODULE_LICENSE("GPL");
