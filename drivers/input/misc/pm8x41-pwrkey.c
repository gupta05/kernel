/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2014, Sony Mobile Communications Inc.
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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/log2.h>
#include <linux/of.h>

#define PON_RT_STS		0x10
#define PON_PULL_CTL		0x70
#define PON_DBC_CTL		0x71

#define PON_DBC_DELAY_MASK	0x7
#define PON_KPDPWR_N_SET	BIT(0)
#define PON_KPDPWR_PULL_UP	BIT(1)

struct pm8x41_pwrkey {
	int irq;
	u32 baseaddr;
	struct regmap *regmap;
	struct input_dev *input;
};

static irqreturn_t pm8x41_pwrkey_irq(int irq, void *_data)
{
	struct pm8x41_pwrkey *pwrkey = _data;
	unsigned int sts;
	int rc;

	rc = regmap_read(pwrkey->regmap, pwrkey->baseaddr + PON_RT_STS, &sts);
	if (rc)
		return IRQ_HANDLED;

	input_report_key(pwrkey->input, KEY_POWER, !!(sts & PON_KPDPWR_N_SET));
	input_sync(pwrkey->input);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM_SLEEP
static int pm8x41_pwrkey_suspend(struct device *dev)
{
	struct pm8x41_pwrkey *pwrkey = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(pwrkey->irq);

	return 0;
}

static int pm8x41_pwrkey_resume(struct device *dev)
{
	struct pm8x41_pwrkey *pwrkey = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(pwrkey->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pm8x41_pwr_key_pm_ops,
		pm8x41_pwrkey_suspend, pm8x41_pwrkey_resume);

static int pm8x41_pwrkey_probe(struct platform_device *pdev)
{
	struct pm8x41_pwrkey *pwrkey;
	bool pull_up;
	u32 req_delay;
	int rc;

	if (of_property_read_u32(pdev->dev.of_node, "debounce", &req_delay))
		req_delay = 15625;

	if (req_delay > 2000000 || req_delay == 0) {
		dev_err(&pdev->dev, "invalid debounce time: %u\n", req_delay);
		return -EINVAL;
	}

	pull_up = of_property_read_bool(pdev->dev.of_node, "bias-pull-up");

	pwrkey = devm_kzalloc(&pdev->dev, sizeof(*pwrkey), GFP_KERNEL);
	if (!pwrkey)
		return -ENOMEM;

	pwrkey->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pwrkey->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	pwrkey->irq = platform_get_irq(pdev, 0);
	if (pwrkey->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return pwrkey->irq;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "reg", &pwrkey->baseaddr);
	if (rc)
		return rc;

	pwrkey->input = devm_input_allocate_device(&pdev->dev);
	if (!pwrkey->input) {
		dev_dbg(&pdev->dev, "unable to allocate input device\n");
		return -ENOMEM;
	}

	input_set_capability(pwrkey->input, EV_KEY, KEY_POWER);

	pwrkey->input->name = "pm8x41_pwrkey";
	pwrkey->input->phys = "pm8x41_pwrkey/input0";

	req_delay = (req_delay << 6) / USEC_PER_SEC;
	req_delay = ilog2(req_delay);

	rc = regmap_update_bits(pwrkey->regmap, pwrkey->baseaddr + PON_DBC_CTL,
			PON_DBC_DELAY_MASK, req_delay);
	if (rc) {
		dev_err(&pdev->dev, "failed to set debounce: %d\n", rc);
		return rc;
	}

	rc = regmap_update_bits(pwrkey->regmap,
			pwrkey->baseaddr + PON_PULL_CTL, PON_KPDPWR_PULL_UP,
			pull_up ? PON_KPDPWR_PULL_UP : 0);
	if (rc) {
		dev_err(&pdev->dev, "failed to set pull: %d\n", rc);
		return rc;
	}

	rc = devm_request_threaded_irq(&pdev->dev, pwrkey->irq,
				NULL, pm8x41_pwrkey_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
					IRQF_ONESHOT,
				"pm8x41_pwrkey", pwrkey);
	if (rc) {
		dev_err(&pdev->dev, "failed requesting IRQ: %d\n", rc);
		return rc;
	}

	rc = input_register_device(pwrkey->input);
	if (rc) {
		dev_err(&pdev->dev, "failed to register input device: %d\n",
			rc);
		return rc;
	}

	platform_set_drvdata(pdev, pwrkey);
	device_init_wakeup(&pdev->dev, 1);

	return 0;
}

static int pm8x41_pwrkey_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

static const struct of_device_id pm8x41_pwr_key_id_table[] = {
	{ .compatible = "qcom,pm8841-pwrkey" },
	{ .compatible = "qcom,pm8941-pwrkey" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8x41_pwr_key_id_table);

static struct platform_driver pm8x41_pwrkey_driver = {
	.probe		= pm8x41_pwrkey_probe,
	.remove		= pm8x41_pwrkey_remove,
	.driver		= {
		.name	= "pm8x41-pwrkey",
		.owner	= THIS_MODULE,
		.pm	= &pm8x41_pwr_key_pm_ops,
		.of_match_table = of_match_ptr(pm8x41_pwr_key_id_table),
	},
};
module_platform_driver(pm8x41_pwrkey_driver);

MODULE_DESCRIPTION("PM8x41 Power Key driver");
MODULE_LICENSE("GPL v2");
