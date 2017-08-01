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

	GRF_CON_DISABLE_ISP,	//3288 only
	GRF_CON_ISP_DPHY_SEL,	//3288 only
	GRF_DSI_CSI_TESTBUS_SEL,	//3288 only
	GRF_DVP_V18SEL,		//3288 only

	GRF_DISABLE_ISP0,	//3399 only
	GRF_DISABLE_ISP1,	//3399 only
	GRF_DPHY_RX0_CLK_INV_SEL,	//3399 only
	GRF_DPHY_RX1_CLK_INV_SEL,	//3399 only
	GRF_DPHY_END,
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
	[GRF_DPHY_END] = {},
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
	[GRF_DPHY_END] = {},
};

struct mipiphy_hsfreqrange {
	u32 range_h;
	u8 cfg_bit;
};

static const struct mipiphy_hsfreqrange mipi_phy_hsfreq_range[] = {
	{  90, 0x00}, { 100, 0x10}, { 110, 0x20}, { 130, 0x01},
	{ 140, 0x11}, { 150, 0x21}, { 170, 0x02}, { 180, 0x12},
	{ 200, 0x22}, { 220, 0x03}, { 240, 0x13}, { 250, 0x23},
	{ 270, 0x04}, { 300, 0x14}, { 330, 0x05}, { 360, 0x15},
	{ 400, 0x25}, { 450, 0x06}, { 500, 0x16}, { 550, 0x07},
	{ 600, 0x17}, { 650, 0x08}, { 700, 0x18}, { 750, 0x09},
	{ 800, 0x19}, { 850, 0x29}, { 900, 0x39}, { 950, 0x0a},
	{1000, 0x1a}, {1050, 0x2a}, {1100, 0x3a}, {1150, 0x0b},
	{1200, 0x1b}, {1250, 0x2b}, {1300, 0x3b}, {1350, 0x0c},
	{1400, 0x1c}, {1450, 0x2c}, {1500, 0x3c}
};

#define MIPIPHY_STATE_OFF 0
#define MIPIPHY_STATE_POWERED 1
#define MIPIPHY_STATE_STREAMING 2

struct mipiphy_priv {
	struct device *dev;
	struct regmap *regmap_grf;
	const struct phy_reg *grf_regs;
	struct clk *clk_phy_ref;
	struct clk *clk_phy_cfg;
	struct clk *clk_mipi_csi;
	u32 lanes;
	u32 bit_rate;
	struct of_device_id *of_id;
	int state;
	struct v4l2_subdev sd;
	struct media_pad pads[MIPIPHY_PADS_NUM];
};
#define to_mipiphy(subdev) container_of(subdev, struct mipiphy_priv, sd)

static inline void write_reg(struct regmap *base, const struct phy_reg *preg,
			     u8 val)
{
	WARN_ON(!preg->offset);

	regmap_write(base, preg->offset,
		     HIWORD_UPDATE(val, preg->mask, preg->shift));
}

static void mipiphy_wr_reg(struct mipiphy_priv *priv,
			   u8 test_code, u8 test_data)
{
	const struct phy_reg *grf_regs = priv->grf_regs;
	struct regmap *grf = priv->regmap_grf;

	/*
	 * With the falling edge on TESTCLK, the TESTDIN[7:0] signal content
	 * is latched internally as the current test code. Test data is
	 * programmed internally by rising edge on TESTCLK.
	 */
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTCLK], 1);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTDIN], test_code);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTEN], 1);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTCLK], 0);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTEN], 0);

	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTDIN], test_data);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTCLK], 1);
}

static int mipiphy_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);
	struct regmap *grf = priv->regmap_grf;
	const struct phy_reg *grf_regs = priv->grf_regs;
	int hsfreqrange, i;
	int lanes = priv->lanes;
	int bit_rate = priv->bit_rate;

	if (!on) {
		if (priv->state == MIPIPHY_STATE_STREAMING) {
			priv->state = MIPIPHY_STATE_POWERED;	//TODO(zsq): to stop streaming
		}

		return 0;
	}

	if (priv->state == MIPIPHY_STATE_STREAMING)
		return 0;

	hsfreqrange = 0;
	for (i = 0; i < ARRAY_SIZE(mipi_phy_hsfreq_range); i++) {
		if (mipi_phy_hsfreq_range[i].range_h > bit_rate) {
			hsfreqrange = mipi_phy_hsfreq_range[i].cfg_bit;
			break;
		}
	}

	write_reg(grf, &grf_regs[GRF_DPHY_RX0_FORCERXMODE], 0);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_FORCETXSTOPMODE], 0);
	/* Disable lan turn around, which is ignored in receive mode */
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TURNREQUEST], 0);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TURNDISABLE], 0xf);

	write_reg(grf, &grf_regs[GRF_DPHY_RX0_ENABLE], GENMASK(lanes - 1, 0));

	/* phy start */
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTCLK], 1);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTCLR], 1);
	usleep_range(100, 150);
	write_reg(grf, &grf_regs[GRF_DPHY_RX0_TESTCLR], 0);
	usleep_range(100, 150);

	/* set clock lane */
	/* HS hsfreqrange & lane 0  settle bypass */
	mipiphy_wr_reg(priv, 0x34, 0);

	if (strcmp(priv->of_id->compatible, "rockchip,rk3288-isp-mipi-phy")) {
		mipiphy_wr_reg(priv, 0x44, hsfreqrange << 1);	// HS RX lane0
		mipiphy_wr_reg(priv, 0x54, 0);	// HS RX lane1
		mipiphy_wr_reg(priv, 0x84, 0);	// HS RX lane2
		mipiphy_wr_reg(priv, 0x94, 0);	// HS RX lane3
		mipiphy_wr_reg(priv, 0x75, 0x04);
	}

	/* Normal operation */
	mipiphy_wr_reg(priv, 0x0, 0);

	priv->state = MIPIPHY_STATE_STREAMING;

	return 0;
}

static int mipiphy_s_power(struct v4l2_subdev *sd, int on)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);
	int ret;

	priv->lanes = 2;	//TODO(zsq): how to associate with sensor?
	priv->bit_rate = 1000;	//TODO(zsq): query from remote sensor, how?

	if (on) {
		if (priv->state > MIPIPHY_STATE_OFF)
			return 0;
		if (!strcmp
		    (priv->of_id->compatible, "rockchip,rk3288-isp-mipi-phy")) {
			ret = clk_prepare_enable(priv->clk_mipi_csi);
			if (ret) {
				v4l2_err(&priv->sd,
					 "Fail to enable mipi_csi clock\n");
				return ret;
			}
		} else {
			ret = clk_prepare_enable(priv->clk_phy_cfg);
			if (ret) {
				v4l2_err(&priv->sd,
					 "Fail to enable phy_cfg clock\n");
				return ret;
			}
			ret = clk_prepare_enable(priv->clk_phy_ref);
			if (ret) {
				v4l2_err(&priv->sd,
					 "Fail to enable phy_ref clock\n");
				clk_disable_unprepare(priv->clk_phy_ref);
				return ret;
			}
		}

		priv->state = MIPIPHY_STATE_POWERED;
	} else {
		if (priv->state == MIPIPHY_STATE_OFF)
			return 0;
		if (!strcmp
		    (priv->of_id->compatible, "rockchip,rk3288-isp-mipi-phy")) {
			clk_disable_unprepare(priv->clk_mipi_csi);
		} else {
			clk_disable_unprepare(priv->clk_phy_ref);
			clk_disable_unprepare(priv->clk_phy_cfg);
		}

		priv->state = MIPIPHY_STATE_OFF;
	}

	return 0;
}

static struct v4l2_subdev_core_ops mipiphy_core_ops = {
	.s_power = mipiphy_s_power,
};

static struct v4l2_subdev_video_ops mipiphy_video_ops = {
	.s_stream = mipiphy_s_stream,
};

static struct v4l2_subdev_ops mipiphy_subdev_ops = {
	.core = &mipiphy_core_ops,
	.video = &mipiphy_video_ops,
};

static const struct of_device_id rockchip_mipiphy_match_id[] = {
	{ .compatible = "rockchip,rk3399-isp-mipi-phy",
		.data = &rk3399_grf_dphy_regs },
	{ .compatible = "rockchip,rk3288-isp-mipi-phy",
		.data = &rk3288_grf_dphy_regs },
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_mipiphy_match_id);

static int rockchip_mipiphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_subdev *sd;
	struct mipiphy_priv *priv;
	struct regmap *grf;
	const struct of_device_id *of_id;
	int ret;

	dev_info(dev, "Probing mipi phy...\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}
	priv->dev = dev;
	priv->state = MIPIPHY_STATE_OFF;

	of_id = of_match_device(rockchip_mipiphy_match_id, dev);
	if (!of_id)
		return -EINVAL;
	priv->of_id = (struct of_device_id *)of_id;

	grf = syscon_regmap_lookup_by_phandle(dev->of_node, "rockchip,grf");
	if (IS_ERR(grf)) {
		dev_err(dev, "Can't find GRF syscon\n");
		return -ENODEV;
	}
	priv->regmap_grf = grf;

	if (!strcmp(of_id->compatible, "rockchip,rk3288-isp-mipi-phy")) {
		priv->clk_mipi_csi = devm_clk_get(dev, "mipi_csi");
		if (IS_ERR(priv->clk_mipi_csi)) {
			dev_err(dev, "Unable to get mipi_csi clock\n");
			return -EINVAL;
		}
	} else {
		priv->clk_phy_ref = devm_clk_get(dev, "phy-ref");
		if (IS_ERR(priv->clk_phy_ref)) {
			dev_err(dev, "Unable to get phy-ref clock\n");
			return -EINVAL;
		}

		priv->clk_phy_cfg = devm_clk_get(dev, "phy-cfg");
		if (IS_ERR(priv->clk_phy_cfg)) {
			dev_err(dev, "Unable to get phy-cfg clock\n");
			return -EINVAL;
		}
	}

	priv->grf_regs = (const struct phy_reg *)of_id->data;

	sd = &priv->sd;
	v4l2_subdev_init(sd, &mipiphy_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "rkisp-mipiphy");
	sd->dev = dev;

	priv->pads[MIPIPHY_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[MIPIPHY_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	ret = media_entity_init(&sd->entity, MIPIPHY_PADS_NUM, priv->pads, 0);

	if (ret)
		return ret;

	platform_set_drvdata(pdev, &sd->entity);

	dev_info(dev, "mipiphy probed\n");

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
