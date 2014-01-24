/*
 * Copyright (c) 2013, Sony Mobile Communications AB.
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
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <linux/mfd/msm_rpm.h>

struct msm_rpm_reg {
	struct mutex lock;
	struct device *dev;
	struct regulator_desc desc;

	const struct rpm_reg_parts *parts;
	struct vreg_range *ranges;
	int n_ranges;

	const int resource;

	int is_enabled;

	u32 val[2];

	int uV;
	const int hpm_min_load;
};

#define REQUEST_MEMBER(_word, _mask, _shift)	\
{						\
	.word   = _word,			\
	.mask   = _mask,			\
	.shift  = _shift,			\
}

struct request_member {
	int                     word;
	unsigned int            mask;
	int                     shift;
};

/* Possible RPM regulator request members */
struct rpm_reg_parts {
	struct request_member   mV;     /* voltage: used if voltage is in mV */
	struct request_member   uV;     /* voltage: used if voltage is in uV */
	struct request_member   ip;             /* peak current in mA */
	struct request_member   pd;             /* pull down enable */
	struct request_member   ia;             /* average current in mA */
	struct request_member   fm;             /* force mode */
	struct request_member   pm;             /* power mode */
	struct request_member   pc;             /* pin control */
	struct request_member   pf;             /* pin function */
	struct request_member   enable_state;   /* NCP and switch */
	struct request_member   comp_mode;      /* NCP */
	struct request_member   freq;           /* frequency: NCP and SMPS */
	struct request_member   freq_clk_src;   /* clock source: SMPS */
	struct request_member   hpm;            /* switch: control OCP and SS */
	int                     request_len;
};

struct vreg_range {
	int min_uV;
	int max_uV;
	int step_uV;
};

#define VOLTAGE_RANGE(_min_uV, _max_uV, _step_uV)	\
{							\
	.min_uV  = _min_uV,				\
	.max_uV  = _max_uV,				\
	.step_uV = _step_uV,				\
}

#define SET_POINTS(_ranges)		\
{ 					\
	.range  = _ranges,		\
	.count  = ARRAY_SIZE(_ranges),	\
};

/* Minimum high power mode loads in uA. */
#define RPM_VREG_8960_LDO_50_HPM_MIN_LOAD               5000
#define RPM_VREG_8960_LDO_150_HPM_MIN_LOAD              10000
#define RPM_VREG_8960_LDO_300_HPM_MIN_LOAD              10000
#define RPM_VREG_8960_LDO_600_HPM_MIN_LOAD              10000
#define RPM_VREG_8960_LDO_1200_HPM_MIN_LOAD             10000
#define RPM_VREG_8960_SMPS_1500_HPM_MIN_LOAD            100000
#define RPM_VREG_8960_SMPS_2000_HPM_MIN_LOAD            100000

/*
 * 8960
 */
static struct vreg_range pldo_ranges[] = {
	VOLTAGE_RANGE( 750000, 1487500, 12500),
	VOLTAGE_RANGE(1500000, 3075000, 25000),
	VOLTAGE_RANGE(3100000, 4900000, 50000),
};

static const struct rpm_reg_parts msm8960_ldo_parts = {
	.request_len    = 2,
	.uV             = REQUEST_MEMBER(0, 0x007FFFFF,  0),
	.pd             = REQUEST_MEMBER(0, 0x00800000, 23),
	.ip             = REQUEST_MEMBER(1, 0x000003FF,  0),
	.ia             = REQUEST_MEMBER(1, 0x000FFC00, 10),
	.fm             = REQUEST_MEMBER(1, 0x00700000, 20),
};

static const struct msm_rpm_reg msm8960_ldo16 = {
	.resource = MSM_RPM_PM8921_L16,

	.desc.name = "pm8921_ldo16",
	.ranges = pldo_ranges,
	.n_ranges = ARRAY_SIZE(pldo_ranges),
	.parts = &msm8960_ldo_parts,
	.hpm_min_load = RPM_VREG_8960_LDO_300_HPM_MIN_LOAD,
};

static const struct of_device_id rpm_of_match[] = {
	{ .compatible = "qcom,msm8960-regulator-l16", .data = &msm8960_ldo16 },
};
MODULE_DEVICE_TABLE(of, rpm_of_match);

static int rpm_reg_write(struct msm_rpm_reg *vreg,
			 const struct request_member *req,
			 const int value)
{
	vreg->val[req->word] &= ~req->mask;
	vreg->val[req->word] |= value << req->shift;

	return msm_rpm_write(vreg->dev->parent,
			     vreg->resource,
			     vreg->val,
			     vreg->parts->request_len);
}

static int rpm_reg_enable(struct regulator_dev *rdev)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);
	const struct rpm_reg_parts *parts = vreg->parts;
	int ret;

	mutex_lock(&vreg->lock);
	ret = rpm_reg_write(vreg, &parts->uV, vreg->uV);
	if (!ret)
		vreg->is_enabled = 1;
	mutex_unlock(&vreg->lock);

	return ret;
}

static int rpm_reg_is_enabled(struct regulator_dev *rdev)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->is_enabled;
}

static int rpm_reg_disable(struct regulator_dev *rdev)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);
	const struct rpm_reg_parts *parts = vreg->parts;
	int ret;

	mutex_lock(&vreg->lock);
	ret = rpm_reg_write(vreg, &parts->uV, 0);
	if (!ret)
		vreg->is_enabled = 0;
	mutex_unlock(&vreg->lock);

	return ret;
}

static int rpm_reg_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV,
			       unsigned *selector)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);
	const struct rpm_reg_parts *parts = vreg->parts;
	struct vreg_range *range = NULL;
	int ret = 0;
	int uV;
	int i;

	dev_dbg(vreg->dev, "set_voltage(%d, %d)\n", min_uV, max_uV);

	/*
	 * Snap to the voltage to a supported level.
	 */
	for (i = 0; i < vreg->n_ranges; i++) {
		range = &vreg->ranges[i];
		if (min_uV <= range->max_uV && max_uV >= range->min_uV)
			break;
	}

	if (i == vreg->n_ranges) {
		dev_err(vreg->dev,
			"requested voltage %d-%d outside possible ranges\n",
			min_uV, max_uV);
		return -EINVAL;
	}

	if (min_uV < range->min_uV)
		uV = range->min_uV;
	else
		uV = min_uV;

	uV = roundup(uV, range->step_uV);

	dev_err(vreg->dev, "snapped voltage %duV\n", uV);

	/*
	 * Update the register.
	 */
	mutex_lock(&vreg->lock);
	vreg->uV = uV;
	if (vreg->is_enabled)
		ret = rpm_reg_write(vreg, &parts->uV, vreg->uV);
	mutex_unlock(&vreg->lock);

	return ret;
}

static struct regulator_ops ldo_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,

	.set_voltage = rpm_reg_set_voltage,
};

static int rpm_reg_probe(struct platform_device *pdev)
{
	struct regulator_init_data *initdata;
	const struct msm_rpm_reg *template;
	const struct of_device_id *match;
	struct regulator_config config = { };
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	struct msm_rpm_reg *vreg;

	match = of_match_device(rpm_of_match, &pdev->dev);
	template = match->data;

	initdata = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node);
	if (!initdata)
		return -EINVAL;

	vreg = devm_kmalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		dev_err(&pdev->dev, "failed to allocate vreg\n");
		return -ENOMEM;
	}
	memcpy(vreg, template, sizeof(*vreg));

	mutex_init(&vreg->lock);
	vreg->dev = &pdev->dev;

	/* XXX:
	 * tighten constraints of initdata
	 * round min_uV and max_uV to step values
	 */

	desc = &vreg->desc;
	desc->id = -1;
	desc->ops = &ldo_ops;
	desc->owner = THIS_MODULE;
	desc->type = REGULATOR_VOLTAGE;

	config.dev = &pdev->dev;
	config.init_data = initdata;
	config.driver_data = vreg;
	config.of_node = pdev->dev.of_node;
        rdev = devm_regulator_register(&pdev->dev, desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "can't register regulator\n");
		return PTR_ERR(rdev);
	}

	platform_set_drvdata(pdev, rdev);

	return 0;
}

static int rpm_reg_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rpm_reg_driver = {
	.probe          = rpm_reg_probe,
	.remove         = rpm_reg_remove,
	.driver  = {
		.name  = "msm_rpm_reg",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rpm_of_match),
	},
};

static int __init rpm_reg_init(void)
{
	return platform_driver_register(&rpm_reg_driver);
}
arch_initcall(rpm_reg_init);

static void __exit rpm_reg_exit(void)
{
	platform_driver_unregister(&rpm_reg_driver);
}
module_exit(rpm_reg_exit)

MODULE_DESCRIPTION("MSM RPM regulator driver");
MODULE_LICENSE("GPLv2");

