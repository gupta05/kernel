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
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/module.h>

#define PM8X41_ADC_TIMEOUT	(msecs_to_jiffies(1000))

enum {
	ADC_TYPE_VOLTAGE,
	ADC_TYPE_CURRENT,
};


enum {
	REG_PERPH_REV0			= 0x00,
	REG_PERPH_REV1			= 0x01,
	REG_PERPH_REV2			= 0x02,
	REG_PERPH_REV3			= 0x03,
	REG_PERPH_TYPE			= 0x04,
	REG_PERPH_SUBTYPE		= 0x05,
	REG_STATUS1			= 0x08,
	REG_STATUS2			= 0x09,
	REG_STATUS_LOW			= 0x0a, /* btm only */
	REG_STATUS_HIGH			= 0x0b, /* btm only */
	REG_INT_RT_STS			= 0x10,
	REG_INT_SET_TYPE		= 0x11,
	REG_INT_POLARITY_HIGH		= 0x12,
	REG_INT_POLARITY_LOW		= 0x13,
	REG_INT_LATCHED_CLR		= 0x14,
	REG_INT_EN_SET			= 0x15,
	REG_INT_EN_CLR			= 0x16,
	REG_INT_LATCHED_STS		= 0x18,
	REG_INT_PENDING_STS		= 0x19,
	REG_INT_MID_SEL			= 0x1a,
	REG_INT_PRIORITY		= 0x1b,

	REG_MODE_CTL			= 0x40,
	REG_MULTI_MEAS_EN		= 0x41, /* btm only */
	REG_LOW_THR_INT_EN		= 0x42, /* btm only */
	REG_HIGH_THR_INT_EN		= 0x43, /* btm only */
	REG_EN_CTL1			= 0x46,
	REG_M0_ADC_CH_SEL_CTL		= 0x48,
	REG_ADC_DIG_PARAM		= 0x50,
	REG_HW_SETTLE_DELAY		= 0x51, /* vadc only */
	REG_CONV_REQ			= 0x52,
	REG_CONV_SEQ_CTL		= 0x54,
	REG_CONV_SEQ_TRIG_CTL		= 0x55,
	REG_MEAS_INTERVAL_CTL		= 0x57,
	REG_MEAS_INTERVAL_CTL2		= 0x58, /* unused */
	REG_MEAS_INTERVAL_OP_CTL	= 0x59, /* partially unused */
	REG_FAST_AVG_CTL		= 0x5a,
	REG_FAST_AVG_EN			= 0x5b,
	REG_M0_LOW_THR0			= 0x5c,
	REG_M0_LOW_THR1			= 0x5d,
	REG_M0_HIGH_THR0		= 0x5e,
	REG_M0_HIGH_THR1		= 0x5f,
	REG_M0_DATA0			= 0x60,
	REG_M0_DATA1			= 0x61,

	REG_MN_BASE			= 0x68,
#define REG_MN_SIZE 0x8

	REG_MN_DATA_BASE		= 0xa0,
#define REG_MN_DATA_SIZE 0x2

	REG_IADC_INT_TEST		= 0xe1,
};

enum {
	OFF_MN_ADC_CH_SEL_CTL		= 0x0,
	OFF_MN_LOW_THR0			= 0x1,
	OFF_MN_LOW_THR1			= 0x2,
	OFF_MN_HIGH_THR0		= 0x3,
	OFF_MN_HIGH_THR1		= 0x4,
	OFF_MN_MEAS_INTERVAL_CTL	= 0x5,
};

enum {
	OFF_MN_DATA0			= 0x0,
	OFF_MN_DATA1			= 0x1,
};

struct reg_ctrl {
	u16 reg_adc_ch_sel;
	u16 reg_meas_intv;
	u16 reg_thres_lo[2];
	u16 reg_thres_hi[2];
	u16 reg_data[2];
};

static int reg_mn_base(struct reg_ctrl *ctrl, unsigned int n)
{
	if (n >= 8)
		return -EINVAL;
	if (n == 0) {
		ctrl->reg_adc_ch_sel = REG_M0_ADC_CH_SEL_CTL;
		ctrl->reg_meas_intv = REG_MEAS_INTERVAL_CTL;
		ctrl->reg_thres_lo[0] = REG_M0_LOW_THR0;
		ctrl->reg_thres_lo[1] = REG_M0_LOW_THR1;
		ctrl->reg_thres_hi[0] = REG_M0_HIGH_THR0;
		ctrl->reg_thres_hi[1] = REG_M0_HIGH_THR1;
		ctrl->reg_data[0] = REG_M0_DATA0;
		ctrl->reg_data[1] = REG_M0_DATA1;
	} else {
		u16 base = (n - 1) * REG_MN_SIZE + REG_MN_BASE;
		u16 dbase = (n - 1) * REG_MN_DATA_SIZE + REG_MN_DATA_BASE;
		ctrl->reg_adc_ch_sel = base + OFF_MN_ADC_CH_SEL_CTL;
		ctrl->reg_meas_intv = base + OFF_MN_MEAS_INTERVAL_CTL;
		ctrl->reg_thres_lo[0] = base + OFF_MN_LOW_THR0;
		ctrl->reg_thres_lo[1] = base + OFF_MN_LOW_THR1;
		ctrl->reg_thres_hi[0] = base + OFF_MN_HIGH_THR0;
		ctrl->reg_thres_hi[1] = base + OFF_MN_HIGH_THR1;
		ctrl->reg_data[0] = dbase + OFF_MN_DATA0;
		ctrl->reg_data[1] = dbase + OFF_MN_DATA1;
	}
	return 0;
}

static const struct iio_event_spec pm8x41_adc_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	},
};

#define ADC_CHANNEL(_type, _idx, _name)				\
  [_idx] = {			\
	.type = _type,						\
	.channel = _idx,					\
	.address = _idx,					\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW) |	\
			BIT(IIO_CHAN_INFO_SCALE) |		\
			BIT(IIO_CHAN_INFO_OFFSET),		\
	.datasheet_name = _name,				\
	.extend_name = _name,					\
	.event_spec = pm8x41_adc_events,			\
	.num_event_specs = ARRAY_SIZE(pm8x41_adc_events),	\
}

#define VADC_CHANNEL(_idx, _name) \
	ADC_CHANNEL(IIO_VOLTAGE, _idx, _name)
#define VADC_CHANNEL_PMUX(_idx, _name) \
	VADC_CHANNEL(_idx +   0, _name), \
	VADC_CHANNEL(_idx +  16, _name "-div-by-3")
#define VADC_CHANNEL_LRMUX(_idx, _name) \
	VADC_CHANNEL(_idx +   0, _name), \
	VADC_CHANNEL(_idx +  64, _name "-pull-up-1"), \
	VADC_CHANNEL(_idx + 128, _name "-pull-up-2"), \
	VADC_CHANNEL(_idx + 192, _name "-pull-up-3")
#define IADC_CHANNEL(_idx, _name) \
	ADC_CHANNEL(IIO_CURRENT, _idx, _name)

static const struct iio_chan_spec pm8x41_vadc_iio_channels[] = {
	VADC_CHANNEL( 0, "usb-in"),
	VADC_CHANNEL( 1, "dc-in"),
	VADC_CHANNEL( 2, "vchg-sense"),
	VADC_CHANNEL( 3, "spare1-x03"),
	VADC_CHANNEL( 4, "spare2-x03"),
	VADC_CHANNEL( 5, "vcoin"),
	VADC_CHANNEL( 6, "vbatt-sense"),
	VADC_CHANNEL( 7, "vsys"),
	VADC_CHANNEL( 8, "die-temp"),
	VADC_CHANNEL( 9, "vref-0.625"),
	VADC_CHANNEL(10, "vref-1.25"),
	VADC_CHANNEL(11, "chg-temp"),
	VADC_CHANNEL(12, "spare1"),
	VADC_CHANNEL(13, "spare2"),
	VADC_CHANNEL(14, "vref-gnd"),
	VADC_CHANNEL(15, "vdd"),

	VADC_CHANNEL_PMUX(16, "mpp1"),
	VADC_CHANNEL_PMUX(17, "mpp2"),
	VADC_CHANNEL_PMUX(18, "mpp3"),
	VADC_CHANNEL_PMUX(19, "mpp4"),
	VADC_CHANNEL_PMUX(20, "mpp5"),
	VADC_CHANNEL_PMUX(21, "mpp6"),
	VADC_CHANNEL_PMUX(22, "mpp7"),
	VADC_CHANNEL_PMUX(23, "mpp8"),

	VADC_CHANNEL_LRMUX(48, "batt-therm"),
	VADC_CHANNEL_LRMUX(49, "batt-id"),
	VADC_CHANNEL_LRMUX(50, "xo-therm"),
	VADC_CHANNEL_LRMUX(51, "amux-therm1"),
	VADC_CHANNEL_LRMUX(52, "amux-therm2"),
	VADC_CHANNEL_LRMUX(53, "amux-therm3"),
	VADC_CHANNEL_LRMUX(54, "hw-id"),
	VADC_CHANNEL_LRMUX(55, "amux-therm4"),
	VADC_CHANNEL_LRMUX(56, "amux-therm5"),
	VADC_CHANNEL_LRMUX(57, "usb-id"),

	VADC_CHANNEL(58, "pull-up-1"),
	VADC_CHANNEL(59, "pull-up-2"),

	VADC_CHANNEL_LRMUX(60, "xo-therm-buf"),
};

static const struct iio_chan_spec pm8x41_iadc_iio_channels[] = {
	IADC_CHANNEL(0, "cs"),
	IADC_CHANNEL(1, "cs-ext"),
	IADC_CHANNEL(2, "cs2"),
	IADC_CHANNEL(3, "gain-calib"),
	IADC_CHANNEL(4, "cs-calib"),
	IADC_CHANNEL(5, "cs-ext-calib"),
	IADC_CHANNEL(6, "cs2-calib"),
	IADC_CHANNEL(7, "gain2-calib"),
};

struct pm8x41_adc_config {
	u8 decimation;
	u8 clk_sel;
	u8 hw_settle;
	u8 fast_switch_cnt;
	u8 clk_rate;
	u16 hi_thresh;
	u16 lo_thresh;
	bool hi_thresh_en;
	bool lo_thresh_en;
};

struct pm8x41_adc_comp {
	int offset;
	int scale;
	int scale_micro;
};

struct pm8x41_adc {
	struct pm8x41_adc_config cfg;
	struct iio_dev *idev;
	struct device *dev;
	struct pm8x41_adc_comp comp;
	struct completion completion;
	struct regmap *regmap;
	u32 base_addr;
	int eoc_irq;
	int hi_irq;
	int lo_irq;

};

static const struct pm8x41_adc_comp pm8x41_vadc_comp = {
	-24576,
	97,656250
};

static int pm8x41_adc_write(struct pm8x41_adc *adc, int reg, u8 val)
{
	return regmap_bulk_write(adc->regmap, adc->base_addr + reg, &val, 1);
}

static int pm8x41_adc_read(struct pm8x41_adc *adc, int reg, u8 *val)
{
	return regmap_bulk_read(adc->regmap, adc->base_addr + reg, val, 1);
}


static int pm8x41_adc_start_measurement(struct pm8x41_adc *adc, int channel)
{
	int rc;

	rc = 0;
	rc = rc ? rc : pm8x41_adc_write(adc, REG_MODE_CTL, 0x00); /* one-shot mode */
	rc = rc ? rc : pm8x41_adc_write(adc, REG_M0_ADC_CH_SEL_CTL, channel);
	rc = rc ? rc : pm8x41_adc_write(adc, REG_ADC_DIG_PARAM, adc->cfg.decimation);
	rc = rc ? rc : pm8x41_adc_write(adc, REG_HW_SETTLE_DELAY, adc->cfg.hw_settle); /* X ms */
	rc = rc ? rc : pm8x41_adc_write(adc, REG_FAST_AVG_CTL, adc->cfg.fast_switch_cnt); /* N samples */
	rc = rc ? rc : pm8x41_adc_write(adc, REG_EN_CTL1, BIT(7)); /* enable */
	rc = rc ? rc : pm8x41_adc_write(adc, REG_CONV_REQ, BIT(7)); /* start conversion */

	return rc;
}

static int pm8x41_adc_read_raw(struct iio_dev *idev,
		struct iio_chan_spec const *chan, int *val,
		int *val2, long mask)
{
	struct pm8x41_adc *adc;
	unsigned long timeout;
	u8 res[2];
	int rc;

	adc = iio_priv(idev);

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		*val = adc->comp.offset;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = adc->comp.scale;
		*val2 = adc->comp.scale_micro;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_RAW:
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(&idev->dev, "reading channel %s\n", chan->datasheet_name);

	rc = pm8x41_adc_start_measurement(adc, chan->address);
	if (rc)
		return rc;

	timeout = wait_for_completion_interruptible_timeout(&adc->completion,
			PM8X41_ADC_TIMEOUT);
	if (timeout == 0)
		return -ETIMEDOUT;

	rc = regmap_bulk_read(adc->regmap,
			adc->base_addr + REG_M0_DATA0, res, 2);
	if (rc)
		return rc;

	*val = res[1] << 8 | res[0];

	return IIO_VAL_INT;

}

static irqreturn_t pm8x41_adc_isr(int irq, void *dev_id)
{
	struct pm8x41_adc *adc = (struct pm8x41_adc *)dev_id;

	complete(&adc->completion);

	return IRQ_HANDLED;
}

static int pm8x41_adc_read_event_value(struct iio_dev *idev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int *val, int *val2)
{
	struct pm8x41_adc *adc = iio_priv(idev);

	if (dir == IIO_EV_DIR_FALLING)
		*val = adc->cfg.lo_thresh;
	else
		*val = adc->cfg.hi_thresh;
	return IIO_VAL_INT;
}

static int pm8x41_adc_write_event_value(struct iio_dev *idev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int val, int val2)
{
	struct pm8x41_adc *adc = iio_priv(idev);
	u8 v2[2] = { val & 0xff, (val & 0xff00) >> 8 };
	int rc;

	if (dir == IIO_EV_DIR_FALLING) {
		rc = regmap_bulk_write(adc->regmap, REG_M0_LOW_THR0, v2, 2);
		if (rc)
			return rc;
		adc->cfg.lo_thresh = val;
	} else {
		rc = regmap_bulk_write(adc->regmap, REG_M0_HIGH_THR0, v2, 2);
		if (rc)
			return rc;
		adc->cfg.hi_thresh = val;
	}
	return 0;
}

static int pm8x41_adc_read_event_config(struct iio_dev *idev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir)
{
	struct pm8x41_adc *adc = iio_priv(idev);

	if (dir == IIO_EV_DIR_FALLING)
		return adc->cfg.lo_thresh_en;
	return adc->cfg.hi_thresh_en;
}

static int pm8x41_adc_write_event_config(struct iio_dev *idev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, int state)
{
	struct pm8x41_adc *adc = iio_priv(idev);

	if (dir == IIO_EV_DIR_FALLING) {
		if (adc->cfg.lo_thresh_en == state)
			return 0;
		if (state)
			enable_irq(adc->lo_irq);
		else
			disable_irq(adc->lo_irq);
		adc->cfg.lo_thresh_en = state;

	} else {
		if (adc->cfg.hi_thresh_en == state)
			return 0;
		if (state)
			enable_irq(adc->hi_irq);
		else
			disable_irq(adc->hi_irq);
		adc->cfg.hi_thresh_en = state;
	}
	return 0;
}

static irqreturn_t pm8x41_adc_thresh_isr(int irq, void *dev_id)
{
	struct pm8x41_adc *adc = (struct pm8x41_adc *)dev_id;
	enum iio_event_direction dir;
	enum iio_chan_type type;
	s64 time;
	u8 chan;
	u64 ev;
	int rc;

	time = iio_get_time_ns();

	rc = pm8x41_adc_read(adc, REG_M0_ADC_CH_SEL_CTL, &chan);
	if (rc || chan >= adc->idev->num_channels)
		return IRQ_HANDLED;

	type = adc->idev->channels[chan].type;
	dir = irq == adc->hi_irq ? IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
	ev = IIO_UNMOD_EVENT_CODE(type, chan, IIO_EV_TYPE_THRESH, dir);
	iio_push_event(adc->idev, ev, time);

	return IRQ_HANDLED;
}

static u32 pm8x41_adc_decimation_values_fn(u32 idx)
{
	return 1 << (idx + 9);
}

static u32 pm8x41_adc_clk_rate_values_fn(u32 idx)
{
	return 2400 * (idx + 1);
}

static u32 pm8x41_adc_hw_settle_values_fn(u32 idx)
{
	return (idx < 11) ? 100 * idx : 2000 * (idx - 10);
}

static u32 pm8x41_adc_fast_switch_cnt_values_fn(u32 idx)
{
	return 1 << idx;
}

static int pm8x41_adc_init(struct pm8x41_adc *adc)
{
	struct device_node *node = adc->dev->of_node;
	struct pm8x41_adc_config *cfg = &adc->cfg;
	const struct {
		const char *name;
		u8 *val_ptr;
		u32 (*fn)(u32);
		int size;
	} opts[] = {
		{
			"qcom,decimation", &cfg->decimation,
			.fn = pm8x41_adc_decimation_values_fn,
			.size = 4,
		},
		{
			"qcom,sample-rate", &cfg->clk_rate,
			.fn = pm8x41_adc_clk_rate_values_fn,
			.size = 3,
		},
		{
			"qcom,hw-settle-period", &cfg->hw_settle,
			.fn = pm8x41_adc_hw_settle_values_fn,
			.size = 16,
		},
		{
			"qcom,sample-cnt", &cfg->fast_switch_cnt,
			.fn = pm8x41_adc_fast_switch_cnt_values_fn,
			.size = 10,
		},
	};
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(opts); ++i) {
		u32 val, sel, c;
		int j, rj;

		rc = of_property_read_u32(node, opts[i].name, &val);
		if (rc) {
			if (rc != -EINVAL) {
				dev_err(adc->dev, "error reading '%s'\n",
						opts[i].name);
				return rc;
			}
			continue;
		}

		sel = UINT_MAX;
		rj = -1;
		c = opts[i].fn(0);
		for (j = 0; j < opts[i].size; ++j) {
			if (c <= val && (sel == UINT_MAX || c >= sel)) {
				sel = c;
				rj = j;
			}
			c = opts[i].fn(j + 1);
		}
		if (sel == UINT_MAX) {
			dev_err(adc->dev, "invalid value for '%s'\n",
					opts[i].name);
			return rc;
		}
		if (sel != val)
			dev_warn(adc->dev, "rounding '%s' down to %u\n",
					opts[i].name, sel);
		*opts[i].val_ptr = rj;
	};

	return 0;
}

static int pm8x41_vadc_init(struct pm8x41_adc *adc)
{
	int rc;

	rc = pm8x41_adc_init(adc);
	if (rc)
		return rc;

	adc->comp = pm8x41_vadc_comp;

	return 0;
}

enum pm8x41_comp_scheme_type {
	COMP_ID_GF = 0,
	COMP_ID_SMIC,
	COMP_ID_TSMC,
};

static int pm8x41_iadc_init(struct pm8x41_adc *adc)
{
	u8 id, rev;
	int rc;

	rc = pm8x41_adc_init(adc);
	if (rc)
		return rc;

	rc = pm8x41_adc_read(adc, REG_IADC_INT_TEST, &id);
	if (rc)
		return rc;

	rc = pm8x41_adc_read(adc, REG_PERPH_REV1, &rev);
	if (rc)
		return rc;

	adc->comp.offset = 0;
	if (rev == 1) {
		if (id == COMP_ID_TSMC) {
			adc->comp.scale = 1;
			adc->comp.scale_micro = 34308;
		} else {
			adc->comp.scale = 1;
			adc->comp.scale_micro = 271027;
		}
	} else if (rev == 3) {
		if (id == COMP_ID_TSMC) {
			adc->comp.scale = 1;
			adc->comp.scale_micro = 2245;
		} else {
			adc->comp.scale = 0;
			adc->comp.scale_micro = 995262;
		}
	} else {
		adc->comp.scale = 1;
		adc->comp.scale_micro = 0;
	}

	return 0;
}

static const struct iio_info pm8x41_adc_iio_info = {
	.read_raw = &pm8x41_adc_read_raw,
	.read_event_config_new = &pm8x41_adc_read_event_config,
	.write_event_config_new = &pm8x41_adc_write_event_config,
	.read_event_value_new = &pm8x41_adc_read_event_value,
	.write_event_value_new = &pm8x41_adc_write_event_value,
	.driver_module = THIS_MODULE,
};

static const struct of_device_id pm8x41_adc_match[] = {
	{ .compatible = "qcom,pm8941-vadc", .data = (void *)ADC_TYPE_VOLTAGE, },
	{ .compatible = "qcom,pm8941-iadc", .data = (void *)ADC_TYPE_CURRENT, },
	{ },
};
MODULE_DEVICE_TABLE(of, pm8x41_adc_match);

static int pm8x41_adc_probe(struct platform_device *pdev)
{
	struct pm8x41_adc *adc;
	struct iio_dev *idev;
	int rc;

	idev = devm_iio_device_alloc(&pdev->dev, sizeof(struct pm8x41_adc));
	if (!idev) {
		dev_err(&pdev->dev, "failed to allocation iio device\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, idev);

	adc = iio_priv(idev);
	adc->dev = &pdev->dev;
	adc->idev = idev;

	adc->eoc_irq = platform_get_irq(pdev, 0);
	if (adc->eoc_irq < 0) {
		dev_err(&pdev->dev, "missing irq resource\n");
		return adc->eoc_irq;
	}

	adc->lo_irq = platform_get_irq(pdev, 1);
	if (adc->lo_irq < 0) {
		dev_err(&pdev->dev, "missing irq resource\n");
		return adc->lo_irq;
	}

	adc->hi_irq = platform_get_irq(pdev, 2);
	if (adc->hi_irq < 0) {
		dev_err(&pdev->dev, "missing irq resource\n");
		return adc->hi_irq;
	}

	adc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!adc->regmap) {
		dev_err(&pdev->dev, "unable to get regmap\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "reg", &adc->base_addr);
	if (rc) {
		dev_err(&pdev->dev, "invalid IO resource\n");
		return rc;
	}

	init_completion(&adc->completion);

	idev->name = dev_name(&pdev->dev);
	idev->dev.parent = &pdev->dev;
	idev->dev.of_node = pdev->dev.of_node;
	idev->info = &pm8x41_adc_iio_info;
	idev->modes = INDIO_DIRECT_MODE;
	rc = (int)of_match_node(pm8x41_adc_match, pdev->dev.of_node)->data;
	switch (rc) {
	case ADC_TYPE_VOLTAGE:
		idev->channels = pm8x41_vadc_iio_channels;
		idev->num_channels = ARRAY_SIZE(pm8x41_vadc_iio_channels);
		rc = pm8x41_vadc_init(adc);
		break;
	case ADC_TYPE_CURRENT:
		idev->channels = pm8x41_iadc_iio_channels;
		idev->num_channels = ARRAY_SIZE(pm8x41_iadc_iio_channels);
		rc = pm8x41_iadc_init(adc);
		break;
	default:
		BUG();
		break;
	}
	if (rc)
		return rc;

	rc = iio_device_register(idev);
	if (rc) {
		dev_err(&pdev->dev, "failed to register iio device\n");
		return rc;
	}

	/* clear all pending interrupts */
	rc = pm8x41_adc_write(adc, REG_INT_LATCHED_CLR, 0x1f);
	if (rc) {
		dev_err(&pdev->dev, "unable to clear interrupts\n");
		goto err;
	}

	rc = devm_request_irq(&pdev->dev, adc->eoc_irq, pm8x41_adc_isr,
					IRQF_TRIGGER_RISING,
					dev_name(&pdev->dev), adc);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to request EOC irq\n");
		goto err;
	}

	rc = devm_request_threaded_irq(&pdev->dev, adc->hi_irq, NULL,
					pm8x41_adc_thresh_isr,
					IRQF_TRIGGER_RISING | IRQF_DISABLED | IRQF_ONESHOT,
					"pm8x41_adc-high", adc);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to request high threshold irq\n");
		goto err;
	}

	rc = devm_request_threaded_irq(&pdev->dev, adc->lo_irq, NULL,
					pm8x41_adc_thresh_isr,
					IRQF_TRIGGER_RISING | IRQF_DISABLED | IRQF_ONESHOT,
					"pm8x41_adc-lo", adc);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to request low threshold irq\n");
		goto err;
	}

	dev_info(&pdev->dev, "successfully registered iio device\n");
	return 0;
err:
	iio_device_unregister(idev);
	return rc;
}

static int pm8x41_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *idev;

	idev = platform_get_drvdata(pdev);
	iio_device_unregister(idev);

	return 0;
}

static struct platform_driver pm8x41_adc_driver = {
	.probe		= pm8x41_adc_probe,
	.remove		= pm8x41_adc_remove,
	.driver		= {
		.name	= "pm8x41-adc",
		.owner	= THIS_MODULE,
		.of_match_table = pm8x41_adc_match,
	},
};

module_platform_driver(pm8x41_adc_driver);
