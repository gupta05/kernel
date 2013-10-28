/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/rtc.h>

/* RTC_CTRL register bit fields */
#define PM8xxx_RTC_ENABLE		BIT(7)
#define PM8xxx_RTC_ALARM_ENABLE		BIT(1)
#define PM8xxx_RTC_ALARM_CLEAR		BIT(0)

#define NUM_8_BIT_RTC_REGS		0x4
/**
 * struct pm8xxx_rtc -  rtc driver internal structure
 * @rtc:		rtc device for this driver.
 * @rtc_alarm_irq:	rtc alarm irq number.
 * @rtc_control_reg:	address of control register.
 * @rtc_read_base:	base address of read registers.
 * @rtc_write_base:	base address of write registers.
 * @alarm_rw_base:	base address of alarm registers.
 * @alarm_ctrl1:	address of alarm ctrl1 register.
 * @alarm_ctrl2:	address of alarm ctrl2 register (only used on pm8941).
 * @alarm_clear:	RTC-specific callback to clear alarm interrupt.
 * @ctrl_reg:		rtc control register.
 * @rtc_dev:		device structure.
 * @ctrl_reg_lock:	spinlock protecting access to ctrl_reg.
 */
struct pm8xxx_rtc {
	struct rtc_device *rtc;
	int rtc_alarm_irq;
	int rtc_control_reg;
	int rtc_read_base;
	int rtc_write_base;
	int alarm_rw_base;
	int alarm_ctrl1;
	int alarm_ctrl2;
	int (*alarm_clear)(struct pm8xxx_rtc *);
	u8 ctrl_reg;
	struct device *rtc_dev;
	spinlock_t ctrl_reg_lock;
	struct regmap *regmap;
};

/*
 * The RTC registers need to be read/written one byte at a time. This is a
 * hardware limitation.
 */
static inline int pm8xxx_read_wrapper(struct pm8xxx_rtc *rtc_dd, u8 *rtc_val,
				      int base, int count)
{
	return regmap_bulk_read(rtc_dd->regmap, base, rtc_val, count);
}

static inline int pm8xxx_write_wrapper(struct pm8xxx_rtc *rtc_dd,
				       const u8 *rtc_val,
				       int base, int count)
{
	return regmap_bulk_write(rtc_dd->regmap, base, rtc_val, count);
}

/*
 * Steps to write the RTC registers.
 * 1. Disable alarm if enabled.
 * 2. Write 0x00 to LSB.
 * 3. Write Byte[1], Byte[2], Byte[3] then Byte[0].
 * 4. Enable alarm if disabled in step 1.
 */
static int pm8xxx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int rc, i;
	unsigned long secs, irq_flags;
	u8 value[NUM_8_BIT_RTC_REGS], reg = 0, alarm_enabled = 0, ctrl_reg;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	rtc_tm_to_time(tm, &secs);

	for (i = 0; i < NUM_8_BIT_RTC_REGS; i++) {
		value[i] = secs & 0xFF;
		secs >>= 8;
	}

	dev_dbg(dev, "Seconds value to be written to RTC = %lu\n", secs);

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);
	ctrl_reg = rtc_dd->ctrl_reg;

	if (ctrl_reg & PM8xxx_RTC_ALARM_ENABLE) {
		alarm_enabled = 1;
		ctrl_reg &= ~PM8xxx_RTC_ALARM_ENABLE;
		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg,
					  rtc_dd->rtc_control_reg, 1);
		if (rc < 0) {
			dev_err(dev, "Write to RTC control register "
								"failed\n");
			goto rtc_rw_fail;
		}
		rtc_dd->ctrl_reg = ctrl_reg;
	} else
		spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);

	/* Write 0 to Byte[0] */
	reg = 0;
	rc = pm8xxx_write_wrapper(rtc_dd, &reg, rtc_dd->rtc_write_base, 1);
	if (rc < 0) {
		dev_err(dev, "Write to RTC write data register failed\n");
		goto rtc_rw_fail;
	}

	/* Write Byte[1], Byte[2], Byte[3] */
	rc = pm8xxx_write_wrapper(rtc_dd, value + 1,
					rtc_dd->rtc_write_base + 1, 3);
	if (rc < 0) {
		dev_err(dev, "Write to RTC write data register failed\n");
		goto rtc_rw_fail;
	}

	/* Write Byte[0] */
	rc = pm8xxx_write_wrapper(rtc_dd, value, rtc_dd->rtc_write_base, 1);
	if (rc < 0) {
		dev_err(dev, "Write to RTC write data register failed\n");
		goto rtc_rw_fail;
	}

	if (alarm_enabled) {
		ctrl_reg |= PM8xxx_RTC_ALARM_ENABLE;
		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg,
					  rtc_dd->rtc_control_reg, 1);
		if (rc < 0) {
			dev_err(dev, "Write to RTC control register "
								"failed\n");
			goto rtc_rw_fail;
		}
		rtc_dd->ctrl_reg = ctrl_reg;
	}

rtc_rw_fail:
	if (alarm_enabled)
		spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);

	return rc;
}

static int pm8xxx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int rc;
	u8 value[NUM_8_BIT_RTC_REGS], reg;
	unsigned long secs;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	rc = pm8xxx_read_wrapper(rtc_dd, value, rtc_dd->rtc_read_base,
							NUM_8_BIT_RTC_REGS);
	if (rc < 0) {
		dev_err(dev, "RTC read data register failed\n");
		return rc;
	}

	/*
	 * Read the LSB again and check if there has been a carry over.
	 * If there is, redo the read operation.
	 */
	rc = pm8xxx_read_wrapper(rtc_dd, &reg, rtc_dd->rtc_read_base, 1);
	if (rc < 0) {
		dev_err(dev, "RTC read data register failed\n");
		return rc;
	}

	if (unlikely(reg < value[0])) {
		rc = pm8xxx_read_wrapper(rtc_dd, value,
				rtc_dd->rtc_read_base, NUM_8_BIT_RTC_REGS);
		if (rc < 0) {
			dev_err(dev, "RTC read data register failed\n");
			return rc;
		}
	}

	secs = value[0] | (value[1] << 8) | (value[2] << 16) | (value[3] << 24);

	rtc_time_to_tm(secs, tm);

	rc = rtc_valid_tm(tm);
	if (rc < 0) {
		dev_err(dev, "Invalid time read from RTC\n");
		return rc;
	}

	dev_dbg(dev, "secs = %lu, h:m:s == %d:%d:%d, d/m/y = %d/%d/%d\n",
				secs, tm->tm_hour, tm->tm_min, tm->tm_sec,
				tm->tm_mday, tm->tm_mon, tm->tm_year);

	return 0;
}

static int pm8xxx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	int rc, i;
	u8 value[NUM_8_BIT_RTC_REGS], ctrl_reg;
	unsigned long secs, irq_flags;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	rtc_tm_to_time(&alarm->time, &secs);

	for (i = 0; i < NUM_8_BIT_RTC_REGS; i++) {
		value[i] = secs & 0xFF;
		secs >>= 8;
	}

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);

	rc = pm8xxx_write_wrapper(rtc_dd, value, rtc_dd->alarm_rw_base,
							NUM_8_BIT_RTC_REGS);
	if (rc < 0) {
		dev_err(dev, "Write to RTC ALARM register failed\n");
		goto rtc_rw_fail;
	}

	ctrl_reg = rtc_dd->ctrl_reg;
	ctrl_reg = alarm->enabled ? (ctrl_reg | PM8xxx_RTC_ALARM_ENABLE) :
					(ctrl_reg & ~PM8xxx_RTC_ALARM_ENABLE);

	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg,
				  rtc_dd->rtc_control_reg, 1);
	if (rc < 0) {
		dev_err(dev, "Write to RTC control register failed\n");
		goto rtc_rw_fail;
	}

	rtc_dd->ctrl_reg = ctrl_reg;

	dev_dbg(dev, "Alarm Set for h:r:s=%d:%d:%d, d/m/y=%d/%d/%d\n",
				alarm->time.tm_hour, alarm->time.tm_min,
				alarm->time.tm_sec, alarm->time.tm_mday,
				alarm->time.tm_mon, alarm->time.tm_year);
rtc_rw_fail:
	spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
	return rc;
}

static int pm8xxx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	int rc;
	u8 value[NUM_8_BIT_RTC_REGS];
	unsigned long secs;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	rc = pm8xxx_read_wrapper(rtc_dd, value, rtc_dd->alarm_rw_base,
			NUM_8_BIT_RTC_REGS);
	if (rc < 0) {
		dev_err(dev, "RTC alarm time read failed\n");
		return rc;
	}

	secs = value[0] | (value[1] << 8) | (value[2] << 16) | (value[3] << 24);

	rtc_time_to_tm(secs, &alarm->time);

	rc = rtc_valid_tm(&alarm->time);
	if (rc < 0) {
		dev_err(dev, "Invalid alarm time read from RTC\n");
		return rc;
	}

	dev_dbg(dev, "Alarm set for - h:r:s=%d:%d:%d, d/m/y=%d/%d/%d\n",
				alarm->time.tm_hour, alarm->time.tm_min,
				alarm->time.tm_sec, alarm->time.tm_mday,
				alarm->time.tm_mon, alarm->time.tm_year);

	return 0;
}

static int pm8xxx_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	int rc;
	unsigned long irq_flags;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);
	u8 ctrl_reg;

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);
	ctrl_reg = rtc_dd->ctrl_reg;
	ctrl_reg = (enable) ? (ctrl_reg | PM8xxx_RTC_ALARM_ENABLE) :
				(ctrl_reg & ~PM8xxx_RTC_ALARM_ENABLE);

	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg,
				  rtc_dd->rtc_control_reg, 1);
	if (rc < 0) {
		dev_err(dev, "Write to RTC control register failed\n");
		goto rtc_rw_fail;
	}

	rtc_dd->ctrl_reg = ctrl_reg;

rtc_rw_fail:
	spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
	return rc;
}

static struct rtc_class_ops pm8xxx_rtc_ops = {
	.read_time	= pm8xxx_rtc_read_time,
	.set_alarm	= pm8xxx_rtc_set_alarm,
	.read_alarm	= pm8xxx_rtc_read_alarm,
	.alarm_irq_enable = pm8xxx_rtc_alarm_irq_enable,
};

static irqreturn_t pm8xxx_alarm_trigger(int irq, void *dev_id)
{
	struct pm8xxx_rtc *rtc_dd = dev_id;
	u8 ctrl_reg;
	int rc;
	unsigned long irq_flags;

	rtc_update_irq(rtc_dd->rtc, 1, RTC_IRQF | RTC_AF);

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);

	/* Clear the alarm enable bit */
	ctrl_reg = rtc_dd->ctrl_reg;
	ctrl_reg &= ~PM8xxx_RTC_ALARM_ENABLE;

	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg,
				  rtc_dd->rtc_control_reg, 1);
	if (rc < 0) {
		spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
		dev_err(rtc_dd->rtc_dev, "Write to RTC control register "
								"failed\n");
		goto rtc_alarm_handled;
	}

	rtc_dd->ctrl_reg = ctrl_reg;
	spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);

	/* Clear RTC alarm register */
	rc = rtc_dd->alarm_clear(rtc_dd);
	if (rc < 0)
		dev_err(rtc_dd->rtc_dev, "failed to clear RTC alarm\n");

rtc_alarm_handled:
	return IRQ_HANDLED;
}

static int pm8941_alarm_clear(struct pm8xxx_rtc *rtc_dd)
{
	u8 ctrl = PM8xxx_RTC_ALARM_CLEAR;
	return pm8xxx_write_wrapper(rtc_dd, &ctrl, rtc_dd->alarm_ctrl2, 1);
}

static int pm8941_chip_init(struct platform_device *pdev,
			    struct pm8xxx_rtc *rtc)
{
	u32 addr[2];
	int err;

	err = of_property_read_u32_array(pdev->dev.of_node,
					 "reg", addr, 2);
	if (err || addr[0] > 0xFFFF || addr[1] > 0xFFFF) {
		dev_err(&pdev->dev, "RTC IO resources absent or invalid\n");
		return err;
	}

	rtc->alarm_clear = pm8941_alarm_clear;

	rtc->rtc_control_reg = addr[0] + 0x46;
	rtc->rtc_write_base  = addr[0] + 0x40;
	rtc->rtc_read_base   = addr[0] + 0x48;
	rtc->alarm_rw_base   = addr[1] + 0x40;
	rtc->alarm_ctrl1     = addr[1] + 0x46;
	rtc->alarm_ctrl2     = addr[1] + 0x48;
	return 0;
}

static int pm8xxx_alarm_clear(struct pm8xxx_rtc *rtc_dd)
{
	u8 ctrl_reg;
	int rc;

	rc = pm8xxx_read_wrapper(rtc_dd, &ctrl_reg, rtc_dd->alarm_ctrl1, 1);
	if (rc < 0) {
		dev_err(rtc_dd->rtc_dev, "RTC Alarm control register read "
								"failed\n");
		return rc;
	}

	ctrl_reg &= ~PM8xxx_RTC_ALARM_CLEAR;
	return pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->alarm_ctrl1, 1);
}

static int pm8xxx_chip_init(struct platform_device *pdev,
			    struct pm8xxx_rtc *rtc)
{
	const struct pm8xxx_rtc_platform_data *pdata =
				dev_get_platdata(&pdev->dev);
	struct resource *res;

	if (!pdata) {
		dev_err(&pdev->dev, "Platform data not initialized.\n");
		return -ENXIO;
	}

	if (pdata->rtc_write_enable)
		pm8xxx_rtc_ops.set_time = pm8xxx_rtc_set_time;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
					   "pmic_rtc_base");
	if (!res) {
		dev_err(&pdev->dev, "RTC IO resource absent\n");
		return -ENXIO;
	}

	rtc->rtc_control_reg = res->start;
	rtc->rtc_write_base  = res->start + 0x02;
	rtc->rtc_read_base   = res->start + 0x06;
	rtc->alarm_rw_base   = res->start + 0x0A;
	rtc->alarm_ctrl1     = res->start + 0x01;

	rtc->alarm_clear = pm8xxx_alarm_clear;
	return 0;
}

static const struct of_device_id pm8xxx_rtc_idtable[] = {
	{ .compatible = "qcom,pm8941-rtc", pm8941_chip_init },
	{},
};
MODULE_DEVICE_TABLE(of, pm8xxx_rtc_idtable);

static int pm8xxx_rtc_probe(struct platform_device *pdev)
{
	int (*chip_init)(struct platform_device *pdev, struct pm8xxx_rtc *rtc);
	const struct device_node *node = pdev->dev.of_node;
	struct pm8xxx_rtc *rtc_dd;
	u8 ctrl_reg;
	int rc;

	rtc_dd = devm_kzalloc(&pdev->dev, sizeof(*rtc_dd), GFP_KERNEL);
	if (rtc_dd == NULL) {
		dev_err(&pdev->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	chip_init = pm8xxx_chip_init;
	if (node) {
		const
		struct of_device_id *id = of_match_node(pm8xxx_rtc_idtable,
							pdev->dev.of_node);
		if (id && id->data)
			chip_init = id->data;
	}

	rc = chip_init(pdev, rtc_dd);
	if (rc) {
		dev_err(&pdev->dev, "Failed to initialize chip.\n");
		return -ENXIO;
	}

	rtc_dd->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!rtc_dd->regmap) {
		dev_err(&pdev->dev, "Unable to get regmap.\n");
		return -ENXIO;
	}

	/* Initialise spinlock to protect RTC control register */
	spin_lock_init(&rtc_dd->ctrl_reg_lock);

	rtc_dd->rtc_alarm_irq = platform_get_irq(pdev, 0);
	if (rtc_dd->rtc_alarm_irq < 0) {
		dev_err(&pdev->dev, "Alarm IRQ resource absent!\n");
		return -ENXIO;
	}

	rtc_dd->rtc_dev = &pdev->dev;

	/* Check if the RTC is on, else turn it on */
	rc = pm8xxx_read_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_control_reg, 1);
	if (rc < 0) {
		dev_err(&pdev->dev, "RTC control register read failed!\n");
		return rc;
	}

	if (!(ctrl_reg & PM8xxx_RTC_ENABLE)) {
		ctrl_reg |= PM8xxx_RTC_ENABLE;
		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg,
					  rtc_dd->rtc_control_reg, 1);
		if (rc < 0) {
			dev_err(&pdev->dev, "Write to RTC control register "
								"failed\n");
			return rc;
		}
	}

	platform_set_drvdata(pdev, rtc_dd);

	/* Register the RTC device */
	rtc_dd->rtc = devm_rtc_device_register(&pdev->dev, "pm8xxx_rtc",
				&pm8xxx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc_dd->rtc)) {
		dev_err(&pdev->dev, "%s: RTC registration failed (%ld)\n",
					__func__, PTR_ERR(rtc_dd->rtc));
		return PTR_ERR(rtc_dd->rtc);
	}

	/* Request the alarm IRQ */
	rc = request_any_context_irq(rtc_dd->rtc_alarm_irq,
				 pm8xxx_alarm_trigger, IRQF_TRIGGER_RISING,
				 "pm8xxx_rtc_alarm", rtc_dd);
	if (rc < 0) {
		dev_err(&pdev->dev, "Request IRQ failed (%d)\n", rc);
		return rc;
	}

	device_init_wakeup(&pdev->dev, 1);

	dev_dbg(&pdev->dev, "Probe success !!\n");

	return 0;
}

static int pm8xxx_rtc_remove(struct platform_device *pdev)
{
	struct pm8xxx_rtc *rtc_dd = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);
	free_irq(rtc_dd->rtc_alarm_irq, rtc_dd);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pm8xxx_rtc_resume(struct device *dev)
{
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(rtc_dd->rtc_alarm_irq);

	return 0;
}

static int pm8xxx_rtc_suspend(struct device *dev)
{
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(rtc_dd->rtc_alarm_irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pm8xxx_rtc_pm_ops, pm8xxx_rtc_suspend, pm8xxx_rtc_resume);

static struct platform_driver pm8xxx_rtc_driver = {
	.probe		= pm8xxx_rtc_probe,
	.remove		= pm8xxx_rtc_remove,
	.driver	= {
		.name	= PM8XXX_RTC_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &pm8xxx_rtc_pm_ops,
		.of_match_table	= pm8xxx_rtc_idtable,
	},
};

module_platform_driver(pm8xxx_rtc_driver);

MODULE_ALIAS("platform:rtc-pm8xxx");
MODULE_DESCRIPTION("PMIC8xxx RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Anirudh Ghayal <aghayal@codeaurora.org>");
