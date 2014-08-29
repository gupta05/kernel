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
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>

#define LC898300_NAME "lc898300xa"

#define LC898300_REG_HBPW       0x01
#define LC898300_REG_RESOFRQ    0x02
#define LC898300_REG_STARTUP    0x03
#define LC898300_REG_BRAKE      0x04
#define LC898300_MIN_ON         8
#define LC898300_OFF_DELAY      10
#define LC898300_BRAKE_TIME     40
#define LC898300_RESUME_DELAY   100

/* TODO:
 * - Re-activate vibrator if called during break
 */

struct lc898300xa_data {
	struct i2c_client *client;
	struct led_classdev cdev;
	int enabled;

	struct hrtimer break_start_timer;
	struct hrtimer break_end_timer;

	struct regulator *vdd;

	struct gpio_desc *rstb_gpio;
	struct gpio_desc *en_gpio;

	u8 frequency;
	u8 startup_time;
	u8 break_time;

	ktime_t spinup_complete_ts;
};

static enum hrtimer_restart lc898300xa_break_end_func(struct hrtimer *timer)
{
	struct lc898300xa_data *data =
		container_of(timer, struct lc898300xa_data, break_end_timer);

	gpiod_set_value(data->rstb_gpio, 0);
	return HRTIMER_NORESTART;
}

static void lc898300xa_initiate_break(struct lc898300xa_data *data)
{
	ktime_t delay;

	gpiod_set_value(data->en_gpio, 0);

	delay = ktime_set(0, 30 * NSEC_PER_MSEC);
	hrtimer_start(&data->break_end_timer, delay, HRTIMER_MODE_REL);
}

static enum hrtimer_restart lc898300xa_break_start_func(struct hrtimer *timer)
{
	struct lc898300xa_data *data =
		container_of(timer, struct lc898300xa_data, break_start_timer);

	lc898300xa_initiate_break(data);
	return HRTIMER_NORESTART;
}

static void lc898300xa_set(struct led_classdev *led_cdev,
			 enum led_brightness intensity)
{
	struct lc898300xa_data *data =
		container_of(led_cdev, struct lc898300xa_data, cdev);
	struct i2c_client *client = data->client;
	ktime_t delay;
	ktime_t now;
	u8 params[4];
	int rc;

	now = data->break_start_timer.base->get_time();

	if (intensity > 0) {
		if (data->enabled)
			return;

		gpiod_set_value(data->rstb_gpio, 1);
		udelay(200);

		params[0] = intensity;
		params[1] = data->frequency;
		params[2] = data->startup_time;
		params[3] = data->break_time << 4 | BIT(6) | 15;
		rc = i2c_smbus_write_i2c_block_data(client,
				LC898300_REG_HBPW, sizeof(params), params);
		if (rc < 0) {
			dev_err(&client->dev, "failed to configure parameters\n");
			gpiod_set_value(data->rstb_gpio, 0);
			return;
		}

		gpiod_set_value(data->en_gpio, 1);
		data->spinup_complete_ts = ktime_add_ns(now, LC898300_MIN_ON * NSEC_PER_MSEC);

		data->enabled = 1;
	} else if (ktime_compare(now, data->spinup_complete_ts) < 0) {
		if (!data->enabled)
			return;

		delay = ktime_sub(data->spinup_complete_ts, now);
		hrtimer_start(&data->break_start_timer, delay, HRTIMER_MODE_REL);

		data->enabled = 0;
	} else {
		if (!data->enabled)
			return;

		lc898300xa_initiate_break(data);

		data->enabled = 0;
	}
}

static int lc898300xa_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct lc898300xa_data *data;
	struct device_node *of_node = client->dev.of_node;
	int ret;
	u32 val;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->client = client;

	data->vdd = devm_regulator_get(&client->dev, "vdd");
	if (!data->vdd) {
		dev_err(&client->dev, "failed to get vdd-supply\n");
		return -ENXIO;
	}

	ret = regulator_enable(data->vdd);
	if (ret) {
		dev_err(&client->dev, "failed to enable vdd-supply\n");
		return ret;
	}

	data->rstb_gpio = devm_gpiod_get(&client->dev, "rstb");
	if (IS_ERR(data->rstb_gpio)) {
		dev_err(&client->dev, "failed to get mandatory rstb\n");
		return -ENXIO;
	}
	gpiod_direction_output(data->rstb_gpio, 0);

	data->en_gpio = devm_gpiod_get(&client->dev, "en");
	if (IS_ERR(data->en_gpio)) {
		dev_err(&client->dev, "failed to get mandatory en\n");
		return -ENXIO;
	}
	/* XXX: workaround for missing functionality in pinctrl */
	gpiod_direction_output(data->en_gpio, 0);

        hrtimer_init(&data->break_start_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->break_start_timer.function = lc898300xa_break_start_func;

        hrtimer_init(&data->break_end_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->break_end_timer.function = lc898300xa_break_end_func;

	ret = of_property_read_u32(of_node, "sanyo,resonance-frequency", &val);
	if (ret) {
		dev_err(&client->dev, "failed to read sanyo,resonance-frequency\n");
		return -EINVAL;
	}
	if (val < 125 || val > 200) {
		dev_err(&client->dev, "sanyo,resonance-frequency is not valid\n");
		return -EINVAL;
	}
	data->frequency = (val - 125) / 5;

	ret = of_property_read_u32(of_node, "sanyo,startup-time", &val);
	if (ret) {
		dev_err(&client->dev, "failed to read sanyo,startup-time\n");
		return -EINVAL;
	}
	data->startup_time = val;

	ret = of_property_read_u32(of_node, "sanyo,break-time", &val);
	if (ret) {
		dev_err(&client->dev, "failed to read sanyo,break-time\n");
		return -EINVAL;
	}
	data->break_time = val;

	data->cdev.name = of_get_property(of_node, "label", NULL) ? : of_node->name;
	data->cdev.default_trigger = of_get_property(of_node, "linux,default-trigger", NULL);
	data->cdev.max_brightness = 15;
	data->cdev.brightness_set = lc898300xa_set;
	data->cdev.brightness = LED_OFF;
	data->cdev.flags |= LED_CORE_SUSPENDRESUME;

	ret = led_classdev_register(&client->dev, &data->cdev);
	if (ret < 0)
		return ret;

	i2c_set_clientdata(client, data);

	return 0;
}

static int lc898300xa_remove(struct i2c_client *client)
{
	struct lc898300xa_data *data = i2c_get_clientdata(client);

	gpiod_set_value(data->en_gpio, 0);
	gpiod_set_value(data->rstb_gpio, 0);

	hrtimer_cancel(&data->break_end_timer);
	hrtimer_cancel(&data->break_start_timer);

	led_classdev_unregister(&data->cdev);

	regulator_disable(data->vdd);

	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct of_device_id of_lc898300xa_vibrator_match[] = {
	{ .compatible = "sanyo,lc898300xa", },
	{},
};
MODULE_DEVICE_TABLE(of, of_lc898300xa_vibrator_match);

static const struct i2c_device_id lc898300xa_id[] = {
	{ LC898300_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, lc898300xa_id);

static struct i2c_driver lc898300xa_i2c_driver = {
	.probe = lc898300xa_probe,
	.remove = lc898300xa_remove,
	.id_table = lc898300xa_id,
	.driver = {
		.name = LC898300_NAME,
		.owner = THIS_MODULE,
	},
};

module_i2c_driver(lc898300xa_i2c_driver);

MODULE_DESCRIPTION("lc898300xa linear vibrator driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
