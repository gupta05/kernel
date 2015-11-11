/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>

#include "qcom_mdt_loader.h"
#include "remoteproc_internal.h"

#define WCNSS_PAS_ID			6

#define WCNSS_SPARE_NVBIN_DLND		BIT(25)

#define WCNSS_PMU_IRIS_XO_CFG		BIT(3)
#define WCNSS_PMU_IRIS_XO_EN		BIT(4)
#define WCNSS_PMU_GC_BUS_MUX_SEL_TOP	BIT(5)
#define WCNSS_PMU_IRIS_XO_CFG_STS	BIT(6) /* 1: in progress, 0: done */

#define WCNSS_PMU_IRIS_RESET		BIT(7)
#define WCNSS_PMU_IRIS_RESET_STS	BIT(8) /* 1: in progress, 0: done */
#define WCNSS_PMU_IRIS_XO_READ		BIT(9)
#define WCNSS_PMU_IRIS_XO_READ_STS	BIT(10)

#define WCNSS_PMU_XO_MODE_MASK		GENMASK(2, 1)
#define WCNSS_PMU_XO_MODE_19p2		0
#define WCNSS_PMU_XO_MODE_48		3

struct wcnss {
	struct device *dev;
	struct rproc *rproc;

	void __iomem *pmu_cfg;
	void __iomem *spare_out;

	bool use_48mhz_xo;

	int wdog_irq;
	int fatal_irq;
	int ready_irq;
	int handover_irq;
	int stop_ack_irq;

	struct qcom_smem_state *state;
	unsigned stop_bit;

	struct clk *xo_clk;
	struct clk *rf_clk;

	struct regulator_bulk_data *vregs;
	size_t num_vregs;

	struct completion start_done;
	struct completion stop_done;

	unsigned crash_reason;
};

struct wcnss_vreg_info {
	const char * const name;
	int min_voltage;
	int max_voltage;

	int load_uA;

	bool super_turbo;
};

struct wcnss_data {
	size_t pmu_offset;
	size_t spare_offset;

	const struct wcnss_vreg_info *vregs;
	size_t num_vregs;

	bool use_48mhz_xo;
};

static const struct wcnss_data riva_data = {
	.pmu_offset = 0x28,
	.spare_offset = 0xb4,

	.vregs = (struct wcnss_vreg_info[]) {
		{ "qcom,iris_vddxo",  1800000, 1800000, 10000 },
		{ "qcom,iris_vddrfa", 1300000, 1300000, 100000 },
		{ "qcom,iris_vddpa",  2900000, 3000000, 515000 },
		{ "qcom,iris_vdddig", 1200000, 1225000, 10000 },
		{ "qcom,riva_vddmx",  1050000, 1150000, 0 },
		{ "qcom,riva_vddcx",  1050000, 1150000, 0 },
		{ "qcom,riva_vddpx",  1800000, 1800000, 0 },
	},
	.num_vregs = 7,

	.use_48mhz_xo = false,
};

static const struct wcnss_data pronto_v1_data = {
	.pmu_offset = 0x1004,
	.spare_offset = 0x1088,

	.vregs = (struct wcnss_vreg_info[]) {
		{ "qcom,iris-vddxo",  1800000, 1800000, 10000 },
		{ "qcom,iris-vddrfa", 1300000, 1300000, 100000 },
		{ "qcom,iris-vddpa",  2900000, 3000000, 515000 },
		{ "qcom,iris-vdddig", 1225000, 1800000, 10000 },
		{ "qcom,pronto-vddmx", 950000, 1150000, 0 },
		{ "qcom,pronto-vddcx", .super_turbo = true},
		{ "qcom,pronto-vddpx", 1800000, 1800000, 0 },
	},
	.num_vregs = 7,

	.use_48mhz_xo = true,
};

static const struct wcnss_data pronto_v2_data = {
	.pmu_offset = 0x1004,
	.spare_offset = 0x1088,

	.vregs = (struct wcnss_vreg_info[]) {
		{ "qcom,iris-vddxo",  1800000, 1800000, 10000 },
		{ "qcom,iris-vddrfa", 1300000, 1300000, 100000 },
		{ "qcom,iris-vddpa",  3300000, 3300000, 515000 },
		{ "qcom,iris-vdddig", 1800000, 1800000, 10000 },
		{ "qcom,pronto-vddmx", 1287500, 1287500, 0 },
		{ "qcom,pronto-vddcx", .super_turbo = true },
		{ "qcom,pronto-vddpx", 1800000, 1800000, 0 },
	},
	.num_vregs = 7,

	.use_48mhz_xo = true,
};

static int wcnss_load(struct rproc *rproc, const struct firmware *fw)
{
	return qcom_mdt_load(rproc, WCNSS_PAS_ID, fw);
}

static const struct rproc_fw_ops wcnss_fw_ops = {
	.find_rsc_table = qcom_mdt_find_rsc_table,
	.sanity_check = qcom_mdt_sanity_check,
	.load = wcnss_load,
};

static void wcnss_indicate_nv_download(struct wcnss *wcnss)
{
	u32 val;

	/* Indicate NV download capability */
	val = readl(wcnss->spare_out);
	val |= WCNSS_SPARE_NVBIN_DLND;
	writel(val, wcnss->spare_out);
}

static void wcnss_configure_iris(struct wcnss *wcnss)
{
	u32 val;

	/* Clear PMU cfg register */
	writel(0, wcnss->pmu_cfg);

	val = WCNSS_PMU_GC_BUS_MUX_SEL_TOP | WCNSS_PMU_IRIS_XO_EN;
	writel(val, wcnss->pmu_cfg);

	/* Clear XO_MODE */
	val &= ~WCNSS_PMU_XO_MODE_MASK;
	if (wcnss->use_48mhz_xo)
		val |= WCNSS_PMU_XO_MODE_48 << 1;
	else
		val |= WCNSS_PMU_XO_MODE_19p2 << 1;
	writel(val, wcnss->pmu_cfg);

	/* Reset IRIS */
	val |= WCNSS_PMU_IRIS_RESET;
	writel(val, wcnss->pmu_cfg);

	/* Wait for PMU.iris_reg_reset_sts */
	while (readl(wcnss->pmu_cfg) & WCNSS_PMU_IRIS_RESET_STS)
		cpu_relax();

	/* Clear IRIS reset */
	val &= ~WCNSS_PMU_IRIS_RESET;
	writel(val, wcnss->pmu_cfg);

	/* Start IRIS XO configuration */
	val |= WCNSS_PMU_IRIS_XO_CFG;
	writel(val, wcnss->pmu_cfg);

	/* Wait for XO configuration to finish */
	while (readl(wcnss->pmu_cfg) & WCNSS_PMU_IRIS_XO_CFG_STS)
		cpu_relax();

	/* Stop IRIS XO configuration */
	val &= ~WCNSS_PMU_GC_BUS_MUX_SEL_TOP;
	val &= ~WCNSS_PMU_IRIS_XO_CFG;
	writel(val, wcnss->pmu_cfg);

	/* Add some delay for XO to settle */
	msleep(20);
}

static int wcnss_start(struct rproc *rproc)
{
	struct wcnss *wcnss = (struct wcnss *)rproc->priv;
	int ret;

	ret = regulator_bulk_enable(wcnss->num_vregs, wcnss->vregs);
	if (ret)
		return ret;

	ret = clk_prepare_enable(wcnss->xo_clk);
	if (ret) {
		dev_err(wcnss->dev, "failed to enable xo clk\n");
		goto release_regs;
	}

	ret = clk_prepare_enable(wcnss->rf_clk);
	if (ret) {
		dev_err(wcnss->dev, "failed to enable rf clk\n");
		goto release_regs;
	}

	wcnss_indicate_nv_download(wcnss);
	wcnss_configure_iris(wcnss);

	ret = qcom_scm_pas_auth_and_reset(WCNSS_PAS_ID);
	if (ret) {
		dev_err(wcnss->dev,
			"failed to authenticate image and release reset\n");
		goto release_xo;
	}

	ret = wait_for_completion_timeout(&wcnss->start_done, msecs_to_jiffies(10000));
	if (ret == 0) {
		dev_err(wcnss->dev, "start timed out\n");

		qcom_scm_pas_shutdown(WCNSS_PAS_ID);
		ret = -ETIMEDOUT;
		goto release_xo;
	}

	ret = 0;

release_xo:
	clk_disable_unprepare(wcnss->rf_clk);
	clk_disable_unprepare(wcnss->xo_clk);
release_regs:
	regulator_bulk_disable(wcnss->num_vregs, wcnss->vregs);

	return ret;
}

static int wcnss_stop(struct rproc *rproc)
{
	struct wcnss *wcnss = (struct wcnss *)rproc->priv;
	int ret;

	qcom_smem_state_update_bits(wcnss->state, BIT(wcnss->stop_bit), BIT(wcnss->stop_bit));

	ret = wait_for_completion_timeout(&wcnss->stop_done, msecs_to_jiffies(1000));
	if (ret == 0)
		dev_err(wcnss->dev, "timed out on wait\n");

	qcom_smem_state_update_bits(wcnss->state, BIT(wcnss->stop_bit), 0);

	ret = qcom_scm_pas_shutdown(WCNSS_PAS_ID);
	if (ret)
		dev_err(wcnss->dev, "failed to shutdown: %d\n", ret);

	return ret;
}

static const struct rproc_ops wcnss_ops = {
	.start = wcnss_start,
	.stop = wcnss_stop,
};

static irqreturn_t wcnss_wdog_interrupt(int irq, void *dev)
{
	struct wcnss *wcnss = dev;

	rproc_report_crash(wcnss->rproc, RPROC_WATCHDOG);
	return IRQ_HANDLED;
}

static irqreturn_t wcnss_fatal_interrupt(int irq, void *dev)
{
	struct wcnss *wcnss = dev;
	size_t len;
	char *msg;

	msg = qcom_smem_get(-1, wcnss->crash_reason, &len);
	if (!IS_ERR(msg) && len > 0 && msg[0])
		dev_err(wcnss->dev, "fatal error received: %s\n", msg);

	rproc_report_crash(wcnss->rproc, RPROC_FATAL_ERROR);

	if (!IS_ERR(msg))
		msg[0] = '\0';

	return IRQ_HANDLED;
}

static irqreturn_t wcnss_ready_interrupt(int irq, void *dev)
{
	struct wcnss *wcnss = dev;

	complete(&wcnss->start_done);

	return IRQ_HANDLED;
}

static irqreturn_t wcnss_handover_interrupt(int irq, void *dev)
{
	/*
	 * XXX: At this point we're supposed to release the resources that we
	 * have been holding on behalf of the WCNSS. Unfortunately this
	 * interrupt comes way before the other side seems to be done.
	 *
	 * So we're currently relying on the ready interrupt firing later then
	 * this and we just disable the resources at the end of wcnss_start().
	 */

	return IRQ_HANDLED;
}

static irqreturn_t wcnss_stop_ack_interrupt(int irq, void *dev)
{
	struct wcnss *wcnss = dev;

	complete(&wcnss->stop_done);
	return IRQ_HANDLED;
}

static ssize_t wcnss_boot_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct wcnss *wcnss = dev_get_drvdata(dev);
	int ret;

	ret = rproc_boot(wcnss->rproc);
	return ret ? : size;
}

static ssize_t wcnss_shutdown_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct wcnss *wcnss = dev_get_drvdata(dev);

	rproc_shutdown(wcnss->rproc);
	return size;
}

static const struct device_attribute wcnss_attrs[] = {
	__ATTR(boot, S_IWUSR, NULL, wcnss_boot_store),
	__ATTR(shutdown, S_IWUSR, NULL, wcnss_shutdown_store),
};

static int wcnss_init_regulators(struct wcnss *wcnss,
				 const struct wcnss_vreg_info *info,
				 int num_vregs)
{
	struct regulator_bulk_data *bulk;
	int ret;
	int i;

	bulk = devm_kcalloc(wcnss->dev,
			    num_vregs, sizeof(struct regulator_bulk_data),
			    GFP_KERNEL);
	if (!bulk)
		return -ENOMEM;

	for (i = 0; i < num_vregs; i++)
		bulk[i].supply = info[i].name;

	ret = devm_regulator_bulk_get(wcnss->dev, num_vregs, bulk);
	if (ret) {
		dev_err(wcnss->dev, "failed to get regulators\n");
		return ret;
	}

	for (i = 0; i < num_vregs; i++) {
		if (info[i].max_voltage)
			regulator_set_voltage(bulk[i].consumer,
					      info[i].min_voltage,
					      info[i].max_voltage);

		if (info[i].load_uA)
			regulator_set_load(bulk[i].consumer, info[i].load_uA);
	}

	wcnss->vregs = bulk;
	wcnss->num_vregs = num_vregs;

	return 0;
}

static int wcnss_request_irq(struct wcnss *wcnss,
			     struct platform_device *pdev,
			     const char *name,
			     irq_handler_t thread_fn)
{
	int ret;

	ret = platform_get_irq_byname(pdev, name);
	if (ret < 0) {
		dev_err(&pdev->dev, "no %s IRQ defined\n", name);
		return ret;
	}

	ret = devm_request_threaded_irq(&pdev->dev, ret,
					NULL, thread_fn,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"wcnss", wcnss);
	if (ret)
		dev_err(&pdev->dev, "request %s IRQ failed\n", name);
	return ret;
}

static int wcnss_probe(struct platform_device *pdev)
{
	const struct wcnss_data *data;
	struct resource *res;
	struct wcnss *wcnss;
	struct rproc *rproc;
	void __iomem *mmio;
	char *fw_name;
	const char *name;
	const char *key;
	int ret;
	int i;

	data = of_device_get_match_data(&pdev->dev);

	if (!qcom_scm_pas_supported(WCNSS_PAS_ID)) {
		dev_err(&pdev->dev, "PAS is not available for WCNSS\n");
		return -ENXIO;
	}

	key = "qcom,firmware-name";
	ret = of_property_read_string(pdev->dev.of_node, key, &name);
	if (ret) {
		dev_err(&pdev->dev, "missing or incorrect %s\n", key);
		return -EINVAL;
	}

	fw_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s.mdt", name);
	if (!fw_name)
		return -ENOMEM;

	rproc = rproc_alloc(&pdev->dev, pdev->name, &wcnss_ops,
			    fw_name, sizeof(*wcnss));
	if (!rproc) {
		dev_err(&pdev->dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	rproc->fw_ops = &wcnss_fw_ops;

	wcnss = (struct wcnss *)rproc->priv;
	wcnss->dev = &pdev->dev;
	wcnss->rproc = rproc;
	platform_set_drvdata(pdev, wcnss);

	init_completion(&wcnss->start_done);
	init_completion(&wcnss->stop_done);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmio = devm_ioremap_resource(&pdev->dev, res);
	if (!mmio) {
		ret = -ENOMEM;
		goto free_rproc;
	};

	wcnss->pmu_cfg = mmio + data->pmu_offset;
	wcnss->spare_out = mmio + data->spare_offset;

	wcnss->use_48mhz_xo = of_property_read_bool(pdev->dev.of_node,
						    "qcom,has-48mhz-xo");
	if (!wcnss->use_48mhz_xo)
		wcnss->use_48mhz_xo = data->use_48mhz_xo;

	wcnss->xo_clk = devm_clk_get(&pdev->dev, "xo");
	if (IS_ERR(wcnss->xo_clk)) {
		if (PTR_ERR(wcnss->xo_clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to acquire xo clk\n");
		ret = PTR_ERR(wcnss->xo_clk);
		goto free_rproc;
	}

	wcnss->rf_clk = devm_clk_get(&pdev->dev, "rf_clk");
	if (IS_ERR(wcnss->rf_clk))
		wcnss->rf_clk = NULL;

	ret = of_property_read_u32(pdev->dev.of_node, "qcom,crash-reason",
				   &wcnss->crash_reason);
	if (ret) {
		dev_err(&pdev->dev, "failed to read crash reason id\n");
		goto free_rproc;
	}

	ret = wcnss_init_regulators(wcnss, data->vregs, data->num_vregs);
	if (ret)
		goto free_rproc;

	ret = wcnss_request_irq(wcnss, pdev, "wdog", wcnss_wdog_interrupt);
	if (ret < 0)
		goto free_rproc;
	wcnss->wdog_irq = ret;

	ret = wcnss_request_irq(wcnss, pdev, "fatal", wcnss_fatal_interrupt);
	if (ret < 0)
		goto free_rproc;
	wcnss->fatal_irq = ret;

	ret = wcnss_request_irq(wcnss, pdev, "ready", wcnss_ready_interrupt);
	if (ret < 0)
		goto free_rproc;
	wcnss->ready_irq = ret;

	ret = wcnss_request_irq(wcnss, pdev, "handover", wcnss_handover_interrupt);
	if (ret < 0)
		goto free_rproc;
	wcnss->handover_irq = ret;

	ret = wcnss_request_irq(wcnss, pdev, "stop-ack", wcnss_stop_ack_interrupt);
	if (ret < 0)
		goto free_rproc;
	wcnss->stop_ack_irq = ret;

	wcnss->state = qcom_smem_state_get(&pdev->dev, "stop", &wcnss->stop_bit);
	if (IS_ERR(wcnss->state))
		goto free_rproc;

	for (i = 0; i < ARRAY_SIZE(wcnss_attrs); i++) {
		ret = device_create_file(&pdev->dev, &wcnss_attrs[i]);
		if (ret) {
			dev_err(&pdev->dev, "unable to create sysfs file\n");
			goto remove_device_files;
		}
	}

	ret = rproc_add(rproc);
	if (ret)
		goto remove_device_files;

	return 0;

remove_device_files:
	for (i--; i >= 0; i--)
		device_remove_file(&pdev->dev, &wcnss_attrs[i]);

free_rproc:
	rproc_put(rproc);

	return ret;
}

static int wcnss_remove(struct platform_device *pdev)
{
	struct wcnss *wcnss = platform_get_drvdata(pdev);
	int i;

	qcom_smem_state_put(wcnss->state);

	for (i = 0; i < ARRAY_SIZE(wcnss_attrs); i++)
		device_remove_file(&pdev->dev, &wcnss_attrs[i]);

	rproc_put(wcnss->rproc);

	return 0;
}

static const struct of_device_id wcnss_of_match[] = {
	{ .compatible = "qcom,riva-pil", &riva_data },
	{ .compatible = "qcom,pronto-v1-pil", &pronto_v1_data },
	{ .compatible = "qcom,pronto-v2-pil", &pronto_v2_data },
	{ },
};

static struct platform_driver wcnss_driver = {
	.probe = wcnss_probe,
	.remove = wcnss_remove,
	.driver = {
		.name = "qcom-wcnss-pil",
		.of_match_table = wcnss_of_match,
	},
};

module_platform_driver(wcnss_driver);
