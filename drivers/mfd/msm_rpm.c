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
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/mfd/msm_rpm.h>

#define MSM_RPM_MAX_RESOURCES 125

struct msm_rpm_resource;

struct msm_rpm {
	struct device *dev;
	struct completion ack;
	struct mutex lock;

	int irq_ack;
	int irq_err;
	int irq_wakeup;

	void __iomem *status_regs;
	void __iomem *ctrl_regs;
	void __iomem *req_regs;
	void __iomem *ack_regs;

	void __iomem *ipc_rpm_reg;

	unsigned long ack_status;

	u32 version[3];

	const struct msm_rpm_resource *resource_table;
	unsigned nresources;
};

#define MSM_STATUS_REG(rpm, i) ((rpm)->status_regs + (i) * 4)
#define MSM_CTRL_REG(rpm, i) ((rpm)->ctrl_regs + (i) * 4)
#define MSM_REQ_REG(rpm, i) ((rpm)->req_regs + (i) * 4)
#define MSM_ACK_REG(rpm, i) ((rpm)->ack_regs + (i) * 4)

struct msm_rpm_resource {
	unsigned ctrl_id;
	unsigned target_id;
	unsigned status_id;
	unsigned count;
	unsigned sel;
};

/*
 * 8960 data
 */
static const struct msm_rpm_resource msm8960_rpm_resource_table[] = {
	[MSM_RPM_VERSION] = { .ctrl_id = 0, .status_id = 0, .count = 3 },
	[MSM_RPM_REQ_CTX] = { .ctrl_id = 3, .count = 7 },
	[MSM_RPM_REQ_SEL] = { .ctrl_id = 11, .count = 3 },
	[MSM_RPM_ACK_CTX] = { .ctrl_id = 15, .count = 7 },
	[MSM_RPM_ACK_CTX] = { .ctrl_id = 23, .count = 7 },
	[MSM_RPM_PM8921_L16] = { .target_id = 163, .status_id = 77, .count = 2, .sel = 53 },
};

static const struct msm_rpm msm8960_template = {
	.version = { 3, 0, 0 },
	.resource_table = msm8960_rpm_resource_table,
	.nresources = ARRAY_SIZE(msm8960_rpm_resource_table),
};

static const struct of_device_id msm_rpm_of_match[] = {
	{ .compatible = "qcom,msm8960-rpm", .data = &msm8960_template },
};
MODULE_DEVICE_TABLE(of, msm_rpm_of_match);

/*
 * Implementation
 */
int msm_rpm_read_status(const struct device *dev, int resource, u32 *buf, size_t count)
{
	const struct msm_rpm_resource *res;
	struct msm_rpm *rpm = dev_get_drvdata(dev);
	unsigned i;

	if (WARN_ON(resource < 0 || resource >= rpm->nresources))
		return -EINVAL;

	res = &rpm->resource_table[resource];
	if (WARN_ON(res->count != count))
		return -EINVAL;

	for (i = 0; i < count; i++)
		buf[i] = readl(MSM_STATUS_REG(rpm, res->status_id + i));

	return 0;
}
EXPORT_SYMBOL(msm_rpm_read_status);

int msm_rpm_write(const struct device *dev, int resource, u32 *buf, size_t count)
{
	const struct msm_rpm_resource *req_ctx;
	const struct msm_rpm_resource *sel_map;
	const struct msm_rpm_resource *res;
	struct msm_rpm *rpm = dev_get_drvdata(dev);
	unsigned long sel_mask[3] = { 0 };
	int ret = 0;
	int i;

	if (WARN_ON(resource < 0 || resource >= rpm->nresources))
		return -EINVAL;

	res = &rpm->resource_table[resource];
	if (WARN_ON(res->count != count))
		return -EINVAL;

	mutex_lock(&rpm->lock);

	for (i = 0; i < res->count; i++)
		writel(buf[i], MSM_REQ_REG(rpm, res->target_id + i));

	bitmap_set(sel_mask, res->sel, 1);
	sel_map = &rpm->resource_table[MSM_RPM_REQ_SEL];
	for (i = 0; i < sel_map->count; i++)
		writel(sel_mask[i], MSM_CTRL_REG(rpm, sel_map->ctrl_id + i));

	/* BIT(0) is active mode */
	req_ctx = &rpm->resource_table[MSM_RPM_REQ_CTX];
	writel(BIT(0), MSM_CTRL_REG(rpm, req_ctx->ctrl_id));

	reinit_completion(&rpm->ack);

	writel(4, rpm->ipc_rpm_reg);

	wait_for_completion(&rpm->ack);

	/* BIT(31) is rejected */
	if (rpm->ack_status & BIT(31))
		ret = -EIO;

	mutex_unlock(&rpm->lock);

	return ret;
}
EXPORT_SYMBOL(msm_rpm_write);

static irqreturn_t msm_rpm_ack_interrupt(int irq, void *dev)
{
	const struct msm_rpm_resource *ack_ctx;
	const struct msm_rpm_resource *ack_sel;
	struct msm_rpm *rpm = dev;
	unsigned long sel[3];
	unsigned long ack;
	int i;

	ack_ctx = &rpm->resource_table[MSM_RPM_ACK_CTX];
	ack_sel = &rpm->resource_table[MSM_RPM_ACK_SEL];

	ack = readl(MSM_CTRL_REG(rpm, ack_ctx->ctrl_id));
	for (i = 0; i < ack_sel->count; i++)
		sel[i] = readl(MSM_CTRL_REG(rpm, ack_sel->ctrl_id + i));

	for (i = 0; i < ack_sel->count; i++)
		writel(0, MSM_CTRL_REG(rpm, ack_sel->ctrl_id + i));
	writel(0, MSM_CTRL_REG(rpm, ack_ctx->ctrl_id));

	/* BIT(30) is notification */
	if (ack & BIT(30)) {
		dev_err(rpm->dev, "notification!\n");
	} else {
		rpm->ack_status = ack;
		complete(&rpm->ack);
		dev_dbg(rpm->dev, "completed ack\n");
	}

	return IRQ_HANDLED;
}

static irqreturn_t msm_rpm_err_interrupt(int irq, void *dev)
{
	struct msm_rpm *rpm = dev;

	writel(0x1, rpm->ipc_rpm_reg);
	panic("Fatal RPM error");

	return IRQ_HANDLED;
}

static irqreturn_t msm_rpm_wakeup_interrupt(int irq, void *dev)
{
	return IRQ_HANDLED;
}

static int msm_rpm_probe(struct platform_device *pdev)
{
	const struct msm_rpm_resource *version_ctrl;
	const struct of_device_id *match;
	const struct msm_rpm *template;
	struct resource *res;
	struct msm_rpm *rpm;
	u32 fw_version[3];
	int ret;
	int i;

	dev_dbg(&pdev->dev, "================ msm_rpm_probe ================\n");

	rpm = devm_kzalloc(&pdev->dev, sizeof(*rpm), GFP_KERNEL);
	if (!rpm) {
		dev_err(&pdev->dev, "Can't allocate msm_rpm\n");
		return -ENOMEM;
	}
	rpm->dev = &pdev->dev;
	mutex_init(&rpm->lock);
	init_completion(&rpm->ack);

	rpm->irq_ack = platform_get_irq_byname(pdev, "ack");
	if (rpm->irq_ack < 0) {
		dev_err(&pdev->dev, "required ack interrupt missing\n");
		return -EINVAL;
	}

	rpm->irq_err = platform_get_irq_byname(pdev, "err");
	if (rpm->irq_err < 0) {
		dev_err(&pdev->dev, "required err interrupt missing\n");
		return -EINVAL;
	}

	rpm->irq_wakeup = platform_get_irq_byname(pdev, "wakeup");
	if (rpm->irq_wakeup < 0) {
		dev_err(&pdev->dev, "required wakeup interrupt missing\n");
		return -EINVAL;
	}

	match = of_match_device(msm_rpm_of_match, &pdev->dev);
	template = match->data;
	memcpy(rpm->version, template->version, sizeof(rpm->version));
	rpm->resource_table = template->resource_table;
	rpm->nresources = template->nresources;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rpm->status_regs = devm_ioremap_resource(&pdev->dev, res);
	rpm->ctrl_regs = rpm->status_regs + 0x400;
	rpm->req_regs = rpm->status_regs + 0x600;
	rpm->ack_regs = rpm->status_regs + 0xa00;
	if (IS_ERR(rpm->status_regs))
		return PTR_ERR(rpm->status_regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	rpm->ipc_rpm_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rpm->ipc_rpm_reg))
		return PTR_ERR(rpm->ipc_rpm_reg);

	dev_set_drvdata(&pdev->dev, rpm);

	ret = msm_rpm_read_status(&pdev->dev, MSM_RPM_VERSION, fw_version, 3);
	if (fw_version[0] != rpm->version[0]) {
		dev_err(&pdev->dev, "RPM version %u.%u.%u incompatible with "
				    "this driver version %u.%u.%u\n",
				    fw_version[0],
				    fw_version[1],
				    fw_version[2],
				    rpm->version[0],
				    rpm->version[1],
				    rpm->version[2]);
		return -EFAULT;
	}

	dev_info(&pdev->dev, "RPM firmware %u.%u.%u\n", fw_version[0],
							fw_version[1],
							fw_version[2]);

	version_ctrl = &rpm->resource_table[MSM_RPM_VERSION];
	for (i = 0; i < version_ctrl->count; i++)
		writel(fw_version[i], MSM_CTRL_REG(rpm, version_ctrl->ctrl_id + i));

	ret = devm_request_irq(&pdev->dev,
			       rpm->irq_ack,
			       msm_rpm_ack_interrupt,
			       IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
			       "msm_rpm ack",
			       rpm);
	if (ret) {
		dev_err(&pdev->dev, "failed to request ack interrupt\n");
		return ret;
	}

#if 0
	ret = irq_set_irq_wake(rpm->irq_ack, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable wakeup for ack interrupt\n");
		return ret;
	}
#endif

	ret = devm_request_irq(&pdev->dev,
			       rpm->irq_err,
			       msm_rpm_err_interrupt,
			       IRQF_TRIGGER_RISING,
			       "msm_rpm err",
			       rpm);
	if (ret) {
		dev_err(&pdev->dev, "failed to request err interrupt\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev,
			       rpm->irq_wakeup,
			       msm_rpm_wakeup_interrupt,
			       IRQF_TRIGGER_RISING,
			       "msm_rpm wakeup",
			       rpm);
	if (ret) {
		dev_err(&pdev->dev, "failed to request wakeup interrupt\n");
		return ret;
	}

#if 0
	ret = irq_set_irq_wake(rpm->irq_wakeup, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable wakeup for wakeup interrupt\n");
		return ret;
	}
#endif

	return of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
}

static struct platform_driver msm_rpm_driver = {
	.probe          = msm_rpm_probe,
	.driver  = {
		.name  = "msm_rpm",
		.owner = THIS_MODULE,
		.of_match_table = msm_rpm_of_match,
	},
};

static int __init msm_rpm_init(void)
{
	return platform_driver_register(&msm_rpm_driver);
}
arch_initcall(msm_rpm_init);

static void __exit msm_rpm_exit(void)
{
	platform_driver_unregister(&msm_rpm_driver);
}
module_exit(msm_rpm_exit)

MODULE_DESCRIPTION("MSM RPM driver");
MODULE_LICENSE("GPLv2");
