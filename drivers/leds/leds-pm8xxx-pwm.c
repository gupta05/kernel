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

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/pwm.h>
#include <linux/module.h>
#include <linux/leds.h>

#define PM8X_LED_PWM_DEV_NAME "qcom,pm8xxx-pwm-led"

enum {
	CFG_REG_SRC	= 0x45,
#define CFG_REG_SRC_MASK	0x03
#define CFG_REG_SRC_VPH		BIT(0)
#define CFG_REG_SRC_5V		BIT(1)

	CFG_REG_EN	= 0x46,
#define CFG_REG_EN_MASK		0xe0
#define CFG_REG_EN_RED		BIT(7)
#define CFG_REG_EN_GREEN	BIT(6)
#define CFG_REG_EN_BLUE		BIT(5)
};

enum pm8xxx_led_id {
	PM8X_LED_ID_RED,
	PM8X_LED_ID_GREEN,
	PM8X_LED_ID_BLUE,
	_PM8X_LED_ID_CNT,
};

static const struct pm8xxx_led_values {
	const char *label;
	u8 enable_bits;
} pm8xxx_led_values[_PM8X_LED_ID_CNT] = {
	[PM8X_LED_ID_RED] = { "red", CFG_REG_EN_RED },
	[PM8X_LED_ID_GREEN] = { "green", CFG_REG_EN_GREEN },
	[PM8X_LED_ID_BLUE] = { "blue", CFG_REG_EN_BLUE },
};

struct pm8xxx_led {
	struct led_classdev cdev;
	enum pm8xxx_led_id id;
	struct pwm_device *pwm;
	int on;
};

struct pm8xxx_led_ctx {
	struct device *dev;
	u16 cfg_addr;
	int vph_vote;
	struct pm8xxx_led leds[_PM8X_LED_ID_CNT];
	struct regmap *regmap;
};

static int pm8xxx_pwm_is_pm8xxx(struct pwm_device *pwm)
{
	struct pwm_chip *chip;

	if (!IS_ENABLED(CONFIG_PWM_PM8XXX))
		return 0;

	chip = pwm->chip;
	if (!chip->dev)
		return 0;
	return strcmp("qcom,pm8xxx-pwm", dev_driver_string(chip->dev)) == 0;
}

#if IS_BUILTIN(CONFIG_PWM_PM8XXX) || \
	(IS_MODULE(CONFIG_PWM_PM8XXX) && IS_MODULE(CONFIG_LEDS_PM8XXX_PWM))
extern int pm8xxx_pwm_configure_pause(struct pwm_chip *, struct pwm_device *,
		unsigned int hi, unsigned int lo);
#else
static inline int pm8xxx_pwm_configure_pause(struct pwm_chip *chip,
		struct pwm_device *pwm, unsigned int hi, unsigned int lo)
{
	return 0;
}
#endif

static int pm8xxx_led_configure_pause(struct pm8xxx_led *led,
		unsigned int hi, unsigned int lo)
{
	if (!pm8xxx_pwm_is_pm8xxx(led->pwm))
		return 0;
	return pm8xxx_pwm_configure_pause(led->pwm->chip, led->pwm, hi, lo);
}

static int pm8xxx_led_vote_vph(struct pm8xxx_led *led, int on)
{
	struct pm8xxx_led_ctx *ctx;
	int rc = 0;
	int pre;

	ctx = container_of(led, struct pm8xxx_led_ctx, leds[led->id]);

	if (on == 0)
		on = -1;
	pre = ctx->vph_vote;
	ctx->vph_vote += on;
	if (!pre && ctx->vph_vote) {
		rc = regmap_update_bits(ctx->regmap,
				ctx->cfg_addr + CFG_REG_SRC,
				CFG_REG_SRC_MASK, CFG_REG_SRC_VPH);
	} else if (pre && !ctx->vph_vote) {
		rc = regmap_update_bits(ctx->regmap,
				ctx->cfg_addr + CFG_REG_SRC,
				CFG_REG_SRC_MASK, 0);
	}
	return rc;
}

static int pm8xxx_led_configure(struct led_classdev *led_cdev,
		enum led_brightness value, unsigned int on, unsigned int off)
{
	const struct pm8xxx_led_values *v;
	struct pm8xxx_led_ctx *ctx;
	struct pm8xxx_led *led;
	u32 duty = 0;
	int rc;

	led = container_of(led_cdev, struct pm8xxx_led, cdev);
	ctx = container_of(led, struct pm8xxx_led_ctx, leds[led->id]);
	v = &pm8xxx_led_values[led->id];

	switch (value) {
	case LED_OFF:
		if (!led->on)
			break;
		pwm_disable(led->pwm);

		rc = pm8xxx_led_vote_vph(led, 0);
		if (rc)
			goto err;

		rc = regmap_update_bits(ctx->regmap,
				ctx->cfg_addr + CFG_REG_EN,
				v->enable_bits, 0);
		if (rc)
			goto err;
		led->on = 0;
		break;
	case LED_FULL:
	default:
		rc = pm8xxx_led_configure_pause(led, on, off);
		if (rc)
			goto err;
		duty = led->pwm->period * value / LED_FULL;
		rc = pwm_config(led->pwm, duty, led->pwm->period);
		if (rc)
			goto err;

		if (led->on)
			break;

		rc = pm8xxx_led_vote_vph(led, 1);
		if (rc)
			goto err;

		rc = regmap_update_bits(ctx->regmap,
				ctx->cfg_addr + CFG_REG_EN,
				v->enable_bits, v->enable_bits);
		if (rc)
			goto err;

		rc = pwm_enable(led->pwm);
		if (rc)
			goto err;
		led->on = 1;
		break;
	}
	led_cdev->brightness = value;
	return 0;
err:
	dev_err(ctx->dev, "failed to set led brightness: %d\n", rc);
	return rc;
}


static void pm8xxx_led_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	pm8xxx_led_configure(led_cdev, value, 0, 0);
}

static int pm8xxx_led_blink_set(struct led_classdev *led_cdev,
		unsigned long *delay_on, unsigned long *delay_off)
{
	if (*delay_on == 0 && *delay_off == 0) {
		*delay_on = 500;
		*delay_off = 500;
	}

	return pm8xxx_led_configure(led_cdev,
			led_cdev->brightness, *delay_on, *delay_off);
}

static int pm8xxx_led_of_init(struct pm8xxx_led_ctx *ctx,
		struct pm8xxx_led *led, struct device_node *node)
{
	const char *label;
	int rc;
	int i;

	rc = of_property_read_string(node, "qcom,id", &label);
	if (rc) {
		dev_err(ctx->dev, "require qcom,id\n");
		return rc;
	}

	led->id = (enum pm8xxx_led_id)-1;
	for (i = 0; i < ARRAY_SIZE(pm8xxx_led_values); ++i) {
		if (!strcmp(label, pm8xxx_led_values[i].label)) {
			led->id = (enum pm8xxx_led_id)i;
			break;
		}
	}
	if (led->id == (enum pm8xxx_led_id)-1) {
		dev_err(ctx->dev, "invalid qcom,label\n");
		return -EINVAL;
	}

	rc = of_property_read_string(node, "label", &led->cdev.name);
	if (rc)
		led->cdev.name = node->name;

	led->cdev.default_trigger = of_get_property(node,
			"linux,default-trigger", NULL);

	return 0;
}

static int pm8xxx_led_pwm_probe(struct platform_device *pdev)
{
	struct pm8xxx_led_ctx *ctx;
	struct device_node *node;
	struct regmap *regmap;
	u32 val;
	int rc;

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap) {
		dev_err(&pdev->dev, "Unable to get regmap\n");
		return -EINVAL;
	}

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (ctx == NULL)
		return -ENOMEM;

	ctx->regmap = regmap;
	ctx->dev = &pdev->dev;

	rc = of_property_read_u32(pdev->dev.of_node, "reg", &val);
	if (rc || val > 0xffff) {
		dev_err(&pdev->dev, "invalid IO resource\n");
		return -EINVAL;
	}
	ctx->cfg_addr = val;

	for_each_child_of_node(pdev->dev.of_node, node) {
		struct pm8xxx_led led;

		memset(&led, 0, sizeof(led));

		led.pwm = devm_of_pwm_get(&pdev->dev, node, NULL);
		if (IS_ERR(led.pwm)) {
			dev_err(&pdev->dev, "invalid or unspecified pwm\n");
			continue;
		}

		rc = pm8xxx_led_of_init(ctx, &led, node);
		if (rc)
			continue;

		led.cdev.brightness_set = pm8xxx_led_brightness_set;
		if (pm8xxx_pwm_is_pm8xxx(led.pwm))
			led.cdev.blink_set = pm8xxx_led_blink_set;

		ctx->leds[led.id] = led;
		rc = led_classdev_register(&pdev->dev, &ctx->leds[led.id].cdev);
		if (rc) {
			dev_err(&pdev->dev, "unable to register led");
			continue;
		}
		dev_info(&pdev->dev, "registered led \"%s\"\n", led.cdev.name);
	}

	return 0;
}

static const struct of_device_id pm8xxx_led_pwm_match_table[] = {
	{ .compatible = PM8X_LED_PWM_DEV_NAME },
	{}
};

static struct platform_driver pm8xxx_led_pwm_driver = {
	.probe	= pm8xxx_led_pwm_probe,
	.driver	= {
		.name		= PM8X_LED_PWM_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= pm8xxx_led_pwm_match_table,
	},
};

module_platform_driver(pm8xxx_led_pwm_driver);

MODULE_DESCRIPTION("pm8xxx led pwm driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" PM8X_LED_PWM_DEV_NAME);
