/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/qcom-smd-rpm.h>
#include <dt-bindings/mfd/qcom-rpm.h>

struct qcom_rpm_reg {
	struct qcom_smd_rpm *rpm;
	u32 resource[2];

	struct regulator_desc desc;

	int is_enabled;
	int uV;
};

struct rpm_regulator_req {
	u32 key;
	u32 nbytes;
	u32 value;
};

#define RPM_KEY_SWEN	0x6e657773 /* "swen" */
#define RPM_KEY_UV	0x00007675 /* "uv" */
#define RPM_KEY_MV	0x0000616d /* "ma" */

static int rpm_reg_enable(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	struct rpm_regulator_req req;
	int ret;

	req.key = RPM_KEY_SWEN;
	req.nbytes = sizeof(u32);
	req.value = 1;

	ret = qcom_rpm_smd_write(vreg->rpm, QCOM_SMD_RPM_ACTIVE_STATE, vreg->resource[0], vreg->resource[1], &req, sizeof(req));
	if (!ret)
		vreg->is_enabled = 1;

	return ret;
}

static int rpm_reg_is_enabled(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->is_enabled;
}

static int rpm_reg_disable(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	struct rpm_regulator_req req;
	int ret;

	req.key = RPM_KEY_SWEN;
	req.nbytes = sizeof(u32);
	req.value = 0;

	ret = qcom_rpm_smd_write(vreg->rpm, QCOM_SMD_RPM_ACTIVE_STATE, vreg->resource[0], vreg->resource[1], &req, sizeof(req));
	if (!ret)
		vreg->is_enabled = 0;

	return ret;
}

static int rpm_reg_get_voltage(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->uV;
}

static int rpm_reg_set_voltage(struct regulator_dev *rdev,
			       int min_uV,
			       int max_uV,
			       unsigned *selector)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	struct rpm_regulator_req req;
	int ret = 0;

	req.key = RPM_KEY_UV;
	req.nbytes = sizeof(u32);
	req.value = min_uV;

	ret = qcom_rpm_smd_write(vreg->rpm, QCOM_SMD_RPM_ACTIVE_STATE, vreg->resource[0], vreg->resource[1], &req, sizeof(req));
	if (!ret)
		vreg->uV = min_uV;

	return ret;
}

static struct regulator_ops rpm_smps_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,

	.get_voltage = rpm_reg_get_voltage,
	.set_voltage = rpm_reg_set_voltage,
};

static struct regulator_ops rpm_ldo_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,

	.get_voltage = rpm_reg_get_voltage,
	.set_voltage = rpm_reg_set_voltage,
};

static struct regulator_ops rpm_switch_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,
};

static const struct of_device_id rpm_of_match[] = {
	{ .compatible = "qcom,smd-rpm-smps", .data = &rpm_smps_ops },
	{ .compatible = "qcom,smd-rpm-smps", .data = &rpm_smps_ops },
	{ .compatible = "qcom,smd-rpm-ldo", .data = &rpm_ldo_ops},
	{ .compatible = "qcom,smd-rpm-switch", .data = &rpm_switch_ops },
	{}
};
MODULE_DEVICE_TABLE(of, rpm_of_match);

static int rpm_reg_probe(struct platform_device *pdev)
{
	struct regulator_init_data *initdata;
	const struct of_device_id *match;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct qcom_rpm_reg *vreg;
	const char *key;
	u32 reg[2];
	int ret;

	match = of_match_device(rpm_of_match, &pdev->dev);

	initdata = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node);
	if (!initdata)
		return -EINVAL;

	vreg = devm_kzalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg)
		return -ENOMEM;

	vreg->desc.continuous_voltage_range = true;
	vreg->desc.id = -1;
	vreg->desc.name = pdev->dev.of_node->name;
	vreg->desc.ops = (struct regulator_ops *)match->data;
	vreg->desc.owner = THIS_MODULE;
	vreg->desc.type = REGULATOR_VOLTAGE;
	vreg->desc.supply_name = "vin";

	vreg->rpm = dev_get_drvdata(pdev->dev.parent);
	if (!vreg->rpm) {
		dev_err(&pdev->dev, "unable to get rpm handle\n");
		return -ENXIO;
	}

	key = "reg";
	ret = of_property_read_u32_array(pdev->dev.of_node, key, reg, 2);
	if (ret) {
		dev_err(&pdev->dev, "failed to read %s\n", key);
		return ret;
	}
	vreg->resource[0] = reg[0];
	vreg->resource[1] = reg[1];

	if (vreg->desc.ops->set_voltage &&
	    (!initdata->constraints.min_uV || !initdata->constraints.max_uV)) {
		dev_err(&pdev->dev, "no voltage specified for regulator\n");
		return -EINVAL;
	}

	config.dev = &pdev->dev;
	config.init_data = initdata;
	config.driver_data = vreg;
	config.of_node = pdev->dev.of_node;
	rdev = devm_regulator_register(&pdev->dev, &vreg->desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "can't register regulator\n");
		return PTR_ERR(rdev);
	}

	return 0;
}

static struct platform_driver rpm_reg_driver = {
	.probe = rpm_reg_probe,
	.driver = {
		.name  = "qcom_rpm_smd_regulator",
		.owner = THIS_MODULE,
		.of_match_table = rpm_of_match,
	},
};

static int __init rpm_reg_init(void)
{
	return platform_driver_register(&rpm_reg_driver);
}
subsys_initcall(rpm_reg_init);

static void __exit rpm_reg_exit(void)
{
	platform_driver_unregister(&rpm_reg_driver);
}
module_exit(rpm_reg_exit)

MODULE_DESCRIPTION("Qualcomm RPM regulator driver");
MODULE_LICENSE("GPLv2");
