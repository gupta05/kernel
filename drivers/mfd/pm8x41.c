/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spmi.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>

static const struct regmap_config pm8x41_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xFFFF,
};

static int pm8x41_remove_child(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int pm8x41_remove(struct spmi_device *sdev)
{
	device_for_each_child(&sdev->dev, NULL, pm8x41_remove_child);
	return 0;
}

static int pm8x41_probe(struct spmi_device *sdev)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spmi(sdev, &pm8x41_regmap_config);
	if (IS_ERR(regmap)) {
		dev_dbg(&sdev->dev, "regmap creation failed.\n");
		return PTR_ERR(regmap);
	}

	return of_platform_populate(sdev->dev.of_node, NULL, NULL, &sdev->dev);
}

static struct of_device_id pm8x41_id_table[] = {
	{ .compatible = "qcom,pm8841", },
	{ .compatible = "qcom,pm8941", },
	{},
};
MODULE_DEVICE_TABLE(of, pm8x41_id_table);

static struct spmi_driver pm8x41_driver = {
	.probe	= pm8x41_probe,
	.remove	= pm8x41_remove,
	.driver	= {
		.name		= "qpnp,pm8x41",
		.of_match_table	= pm8x41_id_table,
	},
};
module_spmi_driver(pm8x41_driver);
