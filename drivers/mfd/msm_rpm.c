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

#include "msm_rpm-8064.h"
#include "msm_rpm-8960.h"

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
	unsigned target_id;
	unsigned status_id;
	unsigned select_id;
	unsigned size;
};

#define MSM_RPM_VERSION		0
#define MSM_RPM_REQ_CTX		3
#define MSM_RPM_REQ_SEL		11
#define MSM_RPM_REQ_SEL_COUNT	7
#define MSM_RPM_ACK_CTX		15
#define MSM_RPM_ACK_SEL		23
#define MSM_RPM_ACK_SEL_COUNT	7

/*
 * APQ8064 data
 */
#define APQ8064_RESOURCE(id, size)		\
	[MSM_RPM_ ## id] = {			\
		MSM_RPM_8064_ID_ ## id,		\
		MSM_RPM_8064_STATUS_ID_ ## id,	\
		MSM_RPM_8064_SEL_ ## id,	\
		size				\
	}

static const struct msm_rpm_resource apq8064_rpm_resource_table[] = {
	APQ8064_RESOURCE(CXO_CLK, 1),
	APQ8064_RESOURCE(PXO_CLK, 1),
	APQ8064_RESOURCE(APPS_FABRIC_CLK, 1),
	APQ8064_RESOURCE(SYSTEM_FABRIC_CLK, 1),
	APQ8064_RESOURCE(MM_FABRIC_CLK, 1),
	APQ8064_RESOURCE(DAYTONA_FABRIC_CLK, 1),
	APQ8064_RESOURCE(SFPB_CLK, 1),
	APQ8064_RESOURCE(CFPB_CLK, 1),
	APQ8064_RESOURCE(MMFPB_CLK, 1),
	APQ8064_RESOURCE(EBI1_CLK, 1),

	APQ8064_RESOURCE(APPS_FABRIC_CFG_HALT, 1),
	APQ8064_RESOURCE(APPS_FABRIC_CFG_CLKMOD, 1),
	APQ8064_RESOURCE(APPS_FABRIC_CFG_IOCTL, 1),
	APQ8064_RESOURCE(APPS_FABRIC_ARB, 12),

	APQ8064_RESOURCE(SYS_FABRIC_CFG_HALT, 1),
	APQ8064_RESOURCE(SYS_FABRIC_CFG_CLKMOD, 1),
	APQ8064_RESOURCE(SYS_FABRIC_CFG_IOCTL, 1),
	APQ8064_RESOURCE(SYSTEM_FABRIC_ARB, 30),

	APQ8064_RESOURCE(MMSS_FABRIC_CFG_HALT, 1),
	APQ8064_RESOURCE(MMSS_FABRIC_CFG_CLKMOD, 1),
	APQ8064_RESOURCE(MMSS_FABRIC_CFG_IOCTL, 1),
	APQ8064_RESOURCE(MM_FABRIC_ARB, 21),

	APQ8064_RESOURCE(PM8921_S1, 2),
	APQ8064_RESOURCE(PM8921_S2, 2),
	APQ8064_RESOURCE(PM8921_S3, 2),
	APQ8064_RESOURCE(PM8921_S4, 2),
	APQ8064_RESOURCE(PM8921_S5, 2),
	APQ8064_RESOURCE(PM8921_S6, 2),
	APQ8064_RESOURCE(PM8921_S7, 2),
	APQ8064_RESOURCE(PM8921_S8, 2),

	APQ8064_RESOURCE(PM8921_L1, 2),
	APQ8064_RESOURCE(PM8921_L2, 2),
	APQ8064_RESOURCE(PM8921_L3, 2),
	APQ8064_RESOURCE(PM8921_L4, 2),
	APQ8064_RESOURCE(PM8921_L5, 2),
	APQ8064_RESOURCE(PM8921_L6, 2),
	APQ8064_RESOURCE(PM8921_L7, 2),
	APQ8064_RESOURCE(PM8921_L8, 2),
	APQ8064_RESOURCE(PM8921_L9, 2),
	APQ8064_RESOURCE(PM8921_L10, 2),
	APQ8064_RESOURCE(PM8921_L11, 2),
	APQ8064_RESOURCE(PM8921_L12, 2),
	APQ8064_RESOURCE(PM8921_L13, 2),
	APQ8064_RESOURCE(PM8921_L14, 2),
	APQ8064_RESOURCE(PM8921_L15, 2),
	APQ8064_RESOURCE(PM8921_L16, 2),
	APQ8064_RESOURCE(PM8921_L17, 2),
	APQ8064_RESOURCE(PM8921_L18, 2),
	APQ8064_RESOURCE(PM8921_L19, 2),
	APQ8064_RESOURCE(PM8921_L20, 2),
	APQ8064_RESOURCE(PM8921_L21, 2),
	APQ8064_RESOURCE(PM8921_L22, 2),
	APQ8064_RESOURCE(PM8921_L23, 2),
	APQ8064_RESOURCE(PM8921_L24, 2),
	APQ8064_RESOURCE(PM8921_L25, 2),
	APQ8064_RESOURCE(PM8921_L26, 2),
	APQ8064_RESOURCE(PM8921_L27, 2),
	APQ8064_RESOURCE(PM8921_L28, 2),
	APQ8064_RESOURCE(PM8921_L29, 2),

	APQ8064_RESOURCE(PM8921_CLK1, 2),
	APQ8064_RESOURCE(PM8921_CLK2, 2),

	APQ8064_RESOURCE(PM8921_LVS1, 1),
	APQ8064_RESOURCE(PM8921_LVS2, 1),
	APQ8064_RESOURCE(PM8921_LVS3, 1),
	APQ8064_RESOURCE(PM8921_LVS4, 1),
	APQ8064_RESOURCE(PM8921_LVS5, 1),
	APQ8064_RESOURCE(PM8921_LVS6, 1),
	APQ8064_RESOURCE(PM8921_LVS7, 1),

	APQ8064_RESOURCE(PM8821_S1, 2),
	APQ8064_RESOURCE(PM8821_S2, 2),

	APQ8064_RESOURCE(PM8821_L1, 2),

	APQ8064_RESOURCE(NCP, 2),

	APQ8064_RESOURCE(CXO_BUFFERS, 1),

	APQ8064_RESOURCE(USB_OTG_SWITCH, 1),
	APQ8064_RESOURCE(HDMI_SWITCH, 1),

	APQ8064_RESOURCE(DDR_DMM, 2),
	APQ8064_RESOURCE(VDDMIN_GPIO, 1),
};

static const struct msm_rpm apq8064_template = {
	.version = { 999, 0, 0 },
	.resource_table = apq8064_rpm_resource_table,
	.nresources = ARRAY_SIZE(apq8064_rpm_resource_table),
};

/*
 * MSM8960 data
 */
#define MSM8960_RESOURCE(id, size)		\
	[MSM_RPM_ ## id] = {			\
		MSM_RPM_8960_ID_ ## id,		\
		MSM_RPM_8960_STATUS_ID_ ## id,	\
		MSM_RPM_8960_SEL_ ## id,	\
		size				\
	}

static const struct msm_rpm_resource msm8960_rpm_resource_table[] = {
	MSM8960_RESOURCE(CXO_CLK, 1),
	MSM8960_RESOURCE(PXO_CLK, 1),
	MSM8960_RESOURCE(APPS_FABRIC_CLK, 1),
	MSM8960_RESOURCE(SYSTEM_FABRIC_CLK, 1),
	MSM8960_RESOURCE(MM_FABRIC_CLK, 1),
	MSM8960_RESOURCE(DAYTONA_FABRIC_CLK, 1),
	MSM8960_RESOURCE(SFPB_CLK, 1),
	MSM8960_RESOURCE(CFPB_CLK, 1),
	MSM8960_RESOURCE(MMFPB_CLK, 1),
	MSM8960_RESOURCE(EBI1_CLK, 1),

	MSM8960_RESOURCE(APPS_FABRIC_CFG_HALT, 1),
	MSM8960_RESOURCE(APPS_FABRIC_CFG_CLKMOD, 1),
	MSM8960_RESOURCE(APPS_FABRIC_CFG_IOCTL, 1),
	MSM8960_RESOURCE(APPS_FABRIC_ARB, 12),

	MSM8960_RESOURCE(SYS_FABRIC_CFG_HALT, 1),
	MSM8960_RESOURCE(SYS_FABRIC_CFG_CLKMOD, 1),
	MSM8960_RESOURCE(SYS_FABRIC_CFG_IOCTL, 1),
	MSM8960_RESOURCE(SYSTEM_FABRIC_ARB, 29),

	MSM8960_RESOURCE(MMSS_FABRIC_CFG_HALT, 1),
	MSM8960_RESOURCE(MMSS_FABRIC_CFG_CLKMOD, 1),
	MSM8960_RESOURCE(MMSS_FABRIC_CFG_IOCTL, 1),
	MSM8960_RESOURCE(MM_FABRIC_ARB, 23),

	MSM8960_RESOURCE(PM8921_S1, 2),
	MSM8960_RESOURCE(PM8921_S2, 2),
	MSM8960_RESOURCE(PM8921_S3, 2),
	MSM8960_RESOURCE(PM8921_S4, 2),
	MSM8960_RESOURCE(PM8921_S5, 2),
	MSM8960_RESOURCE(PM8921_S6, 2),
	MSM8960_RESOURCE(PM8921_S7, 2),
	MSM8960_RESOURCE(PM8921_S8, 2),
	MSM8960_RESOURCE(PM8921_L1, 2),
	MSM8960_RESOURCE(PM8921_L2, 2),
	MSM8960_RESOURCE(PM8921_L3, 2),
	MSM8960_RESOURCE(PM8921_L4, 2),
	MSM8960_RESOURCE(PM8921_L5, 2),
	MSM8960_RESOURCE(PM8921_L6, 2),
	MSM8960_RESOURCE(PM8921_L7, 2),
	MSM8960_RESOURCE(PM8921_L8, 2),
	MSM8960_RESOURCE(PM8921_L9, 2),
	MSM8960_RESOURCE(PM8921_L10, 2),
	MSM8960_RESOURCE(PM8921_L11, 2),
	MSM8960_RESOURCE(PM8921_L12, 2),
	MSM8960_RESOURCE(PM8921_L13, 2),
	MSM8960_RESOURCE(PM8921_L14, 2),
	MSM8960_RESOURCE(PM8921_L15, 2),
	MSM8960_RESOURCE(PM8921_L16, 2),
	MSM8960_RESOURCE(PM8921_L17, 2),
	MSM8960_RESOURCE(PM8921_L18, 2),
	MSM8960_RESOURCE(PM8921_L19, 2),
	MSM8960_RESOURCE(PM8921_L20, 2),
	MSM8960_RESOURCE(PM8921_L21, 2),
	MSM8960_RESOURCE(PM8921_L22, 2),
	MSM8960_RESOURCE(PM8921_L23, 2),
	MSM8960_RESOURCE(PM8921_L24, 2),
	MSM8960_RESOURCE(PM8921_L25, 2),
	MSM8960_RESOURCE(PM8921_L26, 2),
	MSM8960_RESOURCE(PM8921_L27, 2),
	MSM8960_RESOURCE(PM8921_L28, 2),
	MSM8960_RESOURCE(PM8921_L29, 2),
	MSM8960_RESOURCE(PM8921_CLK1, 2),
	MSM8960_RESOURCE(PM8921_CLK2, 2),
	MSM8960_RESOURCE(PM8921_LVS1, 1),
	MSM8960_RESOURCE(PM8921_LVS2, 1),
	MSM8960_RESOURCE(PM8921_LVS3, 1),
	MSM8960_RESOURCE(PM8921_LVS4, 1),
	MSM8960_RESOURCE(PM8921_LVS5, 1),
	MSM8960_RESOURCE(PM8921_LVS6, 1),
	MSM8960_RESOURCE(PM8921_LVS7, 1),

	MSM8960_RESOURCE(NCP, 2),
	MSM8960_RESOURCE(CXO_BUFFERS, 1),
	MSM8960_RESOURCE(USB_OTG_SWITCH, 1),
	MSM8960_RESOURCE(HDMI_SWITCH, 1),
	MSM8960_RESOURCE(DDR_DMM, 2),
};

static const struct msm_rpm msm8960_template = {
	.version = { 3, 0, 0 },
	.resource_table = msm8960_rpm_resource_table,
	.nresources = ARRAY_SIZE(msm8960_rpm_resource_table),
};

static const struct of_device_id msm_rpm_of_match[] = {
	{ .compatible = "qcom,apq8064-rpm", .data = &apq8064_template },
	{ .compatible = "qcom,msm8960-rpm", .data = &msm8960_template },
};
MODULE_DEVICE_TABLE(of, msm_rpm_of_match);

int msm_rpm_write(const struct device *dev, enum msm_rpm_resource_id resource, u32 *buf, size_t size)
{
	const struct msm_rpm_resource *res;
	struct msm_rpm *rpm = dev_get_drvdata(dev);
	unsigned long sel_mask[MSM_RPM_REQ_SEL_COUNT] = { 0 };
	int ret = 0;
	int i;

	if (WARN_ON(resource < 0 || resource >= rpm->nresources))
		return -EINVAL;

	res = &rpm->resource_table[resource];
	if (WARN_ON(res->size != size))
		return -EINVAL;

	mutex_lock(&rpm->lock);

	for (i = 0; i < res->size; i++)
		writel(buf[i], MSM_REQ_REG(rpm, res->target_id + i));

	bitmap_set(sel_mask, res->select_id, 1);
	for (i = 0; i < ARRAY_SIZE(sel_mask); i++)
		writel(sel_mask[i], MSM_CTRL_REG(rpm, MSM_RPM_REQ_SEL + i));

	/* BIT(0) is active mode */
	writel(BIT(0), MSM_CTRL_REG(rpm, MSM_RPM_REQ_CTX));

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
	struct msm_rpm *rpm = dev;
	unsigned long sel[MSM_RPM_ACK_SEL_COUNT];
	unsigned long ack;
	int i;

	ack = readl(MSM_CTRL_REG(rpm, MSM_RPM_ACK_CTX));
	for (i = 0; i < MSM_RPM_ACK_SEL_COUNT; i++) {
		sel[i] = readl(MSM_CTRL_REG(rpm, MSM_RPM_ACK_SEL + i));
		writel(0, MSM_CTRL_REG(rpm, MSM_RPM_ACK_SEL + i));
	}
	writel(0, MSM_CTRL_REG(rpm, MSM_RPM_ACK_CTX));

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
	const struct of_device_id *match;
	const struct msm_rpm *template;
	struct resource *res;
	struct msm_rpm *rpm;
	u32 fw_version[3];
	int ret;

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

	fw_version[0] = readl(MSM_STATUS_REG(rpm, MSM_RPM_VERSION + 0));
	fw_version[1] = readl(MSM_STATUS_REG(rpm, MSM_RPM_VERSION + 1));
	fw_version[2] = readl(MSM_STATUS_REG(rpm, MSM_RPM_VERSION + 2));
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

	writel(fw_version[0], MSM_CTRL_REG(rpm, MSM_RPM_VERSION + 0));
	writel(fw_version[1], MSM_CTRL_REG(rpm, MSM_RPM_VERSION + 1));
	writel(fw_version[2], MSM_CTRL_REG(rpm, MSM_RPM_VERSION + 2));

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
