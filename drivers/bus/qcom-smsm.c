/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/hwspinlock.h>

#include <linux/delay.h>

#include <linux/qcom_smem.h>

#define SMSM_APPS_STATE 0
#define SMEM_SMSM_SHARED_STATE	85

struct qcom_smsm {
	struct device *dev;

	struct qcom_smem *smem;

	u32 *shared_state;
	size_t shared_state_size;

	int irq;
	int signal_offset;
	int signal_bit;
};

static struct qcom_smsm *only_smsm;

struct qcom_smsm *dev_get_qcom_smsm(struct device *dev)
{
	return only_smsm;
}

int qcom_smsm_change_state(struct qcom_smsm *smsm, u32 clear_mask, u32 set_mask)
{
	u32 state;

	dev_dbg(smsm->dev, "SMSM_APPS_STATE clear 0x%x set 0x%x\n", clear_mask, set_mask);
	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smsm->shared_state, smsm->shared_state_size, true);

	state = readl(&smsm->shared_state[SMSM_APPS_STATE]);
	state &= ~clear_mask;
	state |= set_mask;
	writel(state, &smsm->shared_state[SMSM_APPS_STATE]);

	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smsm->shared_state, smsm->shared_state_size, true);

	qcom_smem_signal(smsm->smem, smsm->signal_offset, smsm->signal_bit);

	return 0;
}
EXPORT_SYMBOL(qcom_smsm_change_state);

static int qcom_smsm_probe(struct platform_device *pdev)
{
	struct qcom_smsm *smsm;
	char *key;
	int ret;

	smsm = devm_kzalloc(&pdev->dev, sizeof(*smsm), GFP_KERNEL);
	if (!smsm) {
		dev_err(&pdev->dev, "failed to allocate struct smsm\n");
		return -ENOMEM;
	}
	smsm->dev = &pdev->dev;

	smsm->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (smsm->irq < 0 && smsm->irq != -EINVAL) {
		dev_err(&pdev->dev, "failed to parse smsm interrupt\n");
		return -EINVAL;
	}

	key = "qcom,signal-offset";
	ret = of_property_read_u32(pdev->dev.of_node, key, &smsm->signal_offset);
	if (ret) {
		dev_err(&pdev->dev, "channel missing %s property\n", key);
		return -EINVAL;
	}

	key = "qcom,signal-bit";
	ret = of_property_read_u32(pdev->dev.of_node, key, &smsm->signal_bit);
	if (ret) {
		dev_err(&pdev->dev, "channel missing %s property\n", key);
		return -EINVAL;
	}

	smsm->smem = dev_get_qcom_smem(pdev->dev.parent);
	if (!smsm->smem) {
		dev_err(&pdev->dev, "failed to acquire smem handle\n");
		return -EINVAL;
	}

	ret = qcom_smem_alloc(smsm->smem, SMEM_SMSM_SHARED_STATE, 8 * sizeof(uint32_t));
	if (ret < 0 && ret != -EEXIST) {
		dev_err(&pdev->dev, "unable to allocate shared state entry\n");
		return ret;
	}

	ret = qcom_smem_get(smsm->smem, SMEM_SMSM_SHARED_STATE, (void**)&smsm->shared_state, &smsm->shared_state_size);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to acquire shared state entry\n");
		return ret;
	}

	dev_err(smsm->dev, "SMEM_SMSM_SHARED_STATE: %d, %d\n", ret, smsm->shared_state_size);
	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smsm->shared_state, smsm->shared_state_size, true);

	only_smsm = smsm;

	return 0;
}

static const struct of_device_id qcom_smsm_of_match[] = {
	{ .compatible = "qcom,smsm" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smsm_of_match);

static struct platform_driver qcom_smsm_driver = {
	.probe          = qcom_smsm_probe,
	.driver  = {
		.name  = "qcom_smsm",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smsm_of_match,
	},
};

static int __init qcom_smsm_init(void)
{
	return platform_driver_register(&qcom_smsm_driver);
}
arch_initcall(qcom_smsm_init);

static void __exit qcom_smsm_exit(void)
{
	platform_driver_unregister(&qcom_smsm_driver);
}
module_exit(qcom_smsm_exit)

MODULE_DESCRIPTION("Qualcomm Shared Memory Signaling Mechanism");
MODULE_LICENSE("GPLv2");
