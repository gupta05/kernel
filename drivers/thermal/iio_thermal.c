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
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/thermal.h>
#include <linux/err.h>
#include <linux/of.h>

struct iio_thermal_conv {
	s32 *values;
	int ntuple;
	int (* convert)(const struct iio_thermal_conv *, int volt, long *);
};

struct iio_thermal {
	struct thermal_zone_device *tz;
	struct iio_channel *chan;
	struct iio_thermal_conv conv;
	int trip;
};

enum {
	IIO_TRIP_PASSIVE_LO,
	IIO_TRIP_PASSIVE_HI,
	IIO_TRIP_PASSIVE_BOTH,
	_IIO_TRIP_CNT,
};

static int iio_bsearch(const struct iio_thermal_conv *conv, s32 input)
{
	int h = conv->ntuple - 1;
	int l = 0;

	while (h - 1 > l) {
		int m = (l + h) & ~1;
		s32 v = conv->values[m];
		if (v < input)
			l = m >> 1;
		else
			h = m >> 1;
	}

	return h;
}

static int iio_thermal_interpolate(const struct iio_thermal_conv *conv,
		int v, long *temp)
{
	int n;

	n = iio_bsearch(conv, v);
	if (n == 0 || n == conv->ntuple) {
		BUG();
	} else {
		s32 *pts = &conv->values[(n-1)*2];
		s32 sx = pts[3] - pts[1];
		s32 sy = pts[2] - pts[0];
		*temp = sx * (v - pts[0]) / sy + pts[1];
	}
	return 0;
}

static int iio_thermal_scale(const struct iio_thermal_conv *conv,
		int v, long *temp)
{
	*temp = div_s64((s64)v * (s32)conv->values[0], (s32)conv->values[1]);

	return 0;
}

static int iio_thermal_get_temp(struct thermal_zone_device *tz,
			unsigned long *value)
{
	struct iio_thermal *iio = tz->devdata;
	int voltage;
	long temp;
	int rc;

	rc = iio_read_channel_processed(iio->chan, &voltage);
	if (rc)
		return rc;

	rc = iio->conv.convert(&iio->conv, voltage, &temp);
	if (rc)
		return rc;

	temp = temp - 273150;
	/* TZ doesn't deal with negative temperatures, but I do */
	*value = (temp < 0) ? 0 : temp;

	return 0;
}

static int iio_thermal_get_trip_temp(struct thermal_zone_device *tz,
			int trip, unsigned long *value)
{
	struct iio_thermal *iio = tz->devdata;
	if (trip != 0)
		return -EINVAL;

	*value = iio->trip;
	return 0;
}

static int iio_thermal_set_trip_temp(struct thermal_zone_device *tz,
			int trip, unsigned long value)
{
	struct iio_thermal *iio = tz->devdata;
	if (trip != 0)
		return -EINVAL;

	iio->trip = value;
	return 0;
}

static int iio_thermal_get_trip_type(struct thermal_zone_device *tz,
			int trip, enum thermal_trip_type *type)
{
	if (trip != 0)
		return -EINVAL;

	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

static const struct thermal_zone_device_ops iio_thermal_ops = {
	.get_temp = iio_thermal_get_temp,
	.get_trip_temp = iio_thermal_get_trip_temp,
	.set_trip_temp = iio_thermal_set_trip_temp,
	.get_trip_type = iio_thermal_get_trip_type,
};

static int iio_thermal_probe(struct platform_device *pdev)
{
	struct iio_thermal *iio;
	const void *values;
	const char *type;
	int rc;

	dev_info(&pdev->dev, "registering IIO thermal device\n");
	iio = devm_kzalloc(&pdev->dev, sizeof(*iio), GFP_KERNEL);
	if (!iio)
		return -ENOMEM;

	values = of_get_property(pdev->dev.of_node,
			"iio,conversion-values", &iio->conv.ntuple);
	if (values == NULL || (iio->conv.ntuple % (sizeof(u32) * 2)) != 0) {
		dev_err(&pdev->dev, "invalid/missing conversion values\n");
		return -EINVAL;
	}
	iio->conv.ntuple /= sizeof(u32) * 2;
	iio->conv.values = devm_kzalloc(&pdev->dev,
			sizeof(u32) * 2 * iio->conv.ntuple, GFP_KERNEL);
	if (!iio->conv.values)
		return -ENOMEM;

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"iio,conversion-values", (u32 *)iio->conv.values,
			iio->conv.ntuple * 2);
	if (rc) {
		dev_err(&pdev->dev, "invalid/missing conversion values\n");
		return -EINVAL;
	}

	rc = of_property_read_string(pdev->dev.of_node, "iio,conversion-method",
			&type);
	if (rc) {
		dev_err(&pdev->dev, "invalid/missing conversion method\n");
		return rc;
	}
	if (!strcmp(type, "interpolation") && iio->conv.ntuple > 1) {
		if (iio->conv.values[0] > iio->conv.values[2]) {
			dev_err(&pdev->dev, "conversion values should ascend\n");
			return rc;
		}
		iio->conv.convert = iio_thermal_interpolate;
	} else if (!strcmp(type, "scalar") && iio->conv.ntuple == 1) {
		iio->conv.convert = iio_thermal_scale;
	} else {
		dev_err(&pdev->dev, "invalid conversion method\n");
		return rc;
	}

	iio->chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(iio->chan)) {
		dev_err(&pdev->dev, "invalid/missing iio channel\n");
		return PTR_ERR(iio->chan);
	}
	if (iio->chan->channel->type != IIO_VOLTAGE) {
		dev_err(&pdev->dev, "specified iio channel is not voltage\n");
		iio_channel_release(iio->chan);
		return -EINVAL;
	}
	platform_set_drvdata(pdev, iio);

	type = iio->chan->channel->extend_name;
	if (type == NULL)
		type = iio->chan->channel->datasheet_name;
	iio->tz = thermal_zone_device_register(type, 1, 1, iio,
			&iio_thermal_ops, NULL, 0, 5000);
	if (IS_ERR(iio->tz)) {
		dev_err(&pdev->dev, "failed to register thermal device\n");
		iio_channel_release(iio->chan);
		return PTR_ERR(iio->tz);
	}
	dev_info(&pdev->dev, "successfully registered IIO thermal device\n");

	return 0;
}

static int iio_thermal_remove(struct platform_device *pdev)
{
	struct iio_thermal *iio;

	iio = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(iio->tz);
	iio_channel_release(iio->chan);

	return 0;
}

static const struct of_device_id iio_thermal_match[] = {
	{ .compatible = "iio-thermal", },
	{ }
};

static struct platform_driver iio_thermal = {
	.driver = {
		.name	= "iio-thermal",
		.owner = THIS_MODULE,
		.of_match_table = iio_thermal_match,
	},
	.probe		= iio_thermal_probe,
	.remove		= iio_thermal_remove,
};
module_platform_driver(iio_thermal);

MODULE_DESCRIPTION("Thermal driver for IIO ADCs");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:iio-thermal");
