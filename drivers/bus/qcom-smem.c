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
#include <linux/qcom_smd.h>
#include <linux/hwspinlock.h>

#include <linux/delay.h>

#include <linux/qcom_smem.h>

struct smem_proc_comm {
	unsigned command;
	unsigned status;
	unsigned data1;
	unsigned data2;
};

struct smem_heap_info {
	unsigned initialized;
	unsigned free_offset;
	unsigned heap_remaining;
	unsigned reserved;
};

struct smem_heap_entry {
	unsigned allocated;
	unsigned offset;
	unsigned size;
	unsigned reserved; /* bits 1:0 reserved, bits 31:2 aux smem base addr */
};
#define BASE_ADDR_MASK 0xfffffffc

#define SMD_HEAP_SIZE 512

struct smem_shared {
	struct smem_proc_comm proc_comm[4];
	unsigned version[32];
	struct smem_heap_info heap_info;
	struct smem_heap_entry heap_toc[SMD_HEAP_SIZE];
};

struct qcom_smem {
	struct device *dev;

	struct hwspinlock *hwlock;

	void __iomem *signal_base;
	void __iomem *base;
	void __iomem *aux_base;
};

struct qcom_smem *dev_get_qcom_smem(struct device *dev)
{
	return dev_get_drvdata(dev);
}

int qcom_smem_alloc(struct qcom_smem *smem, int smem_id, size_t size)
{
	struct smem_shared *shared = smem->base;
	struct smem_heap_entry *toc = shared->heap_toc;
	struct smem_heap_entry *entry;
	unsigned long flags;
	int ret;

	size = ALIGN(size, 8);

	ret = hwspin_lock_timeout_irqsave(smem->hwlock, 100, &flags);
	if (ret) {
		dev_err(smem->dev, "failed to lock mutex\n");
		return ret;
	}

	entry = &toc[smem_id];
	if (entry->allocated) {
		ret = -EEXIST;
		goto out;
	}

	if (size > shared->heap_info.heap_remaining) {
		dev_err(smem->dev, "unable to allocate %d bytes from smem heap\n", size);
		ret = -ENOMEM;
		goto out;
	}

	entry->offset = shared->heap_info.free_offset;
	entry->size = size;
	entry->allocated = 1;

	shared->heap_info.free_offset += size;
	shared->heap_info.heap_remaining -= size;

	/* Commit the changes before we release the spin lock */
	wmb();
out:
	hwspin_unlock_irqrestore(smem->hwlock, &flags);

	return ret;
}

/*
 * Resolves the address and size of smem_id.
 */
int qcom_smem_get(struct qcom_smem *smem, int smem_id, void **ptr, size_t *size)
{
	struct smem_shared *shared = smem->base;
	struct smem_heap_entry *toc = shared->heap_toc;
	struct smem_heap_entry *entry;
	unsigned long flags;
	int ret;

	ret = hwspin_lock_timeout_irqsave(smem->hwlock, 100, &flags);
	if (ret)
		return ret;

	entry = &toc[smem_id];

	if (!entry->allocated) {
		ret = -EIO;
		goto out;
	}

	if (ptr != NULL) {
		if (entry->reserved)
			*ptr = smem->aux_base + entry->offset;
		else
			*ptr = smem->base + entry->offset;
	}
	if (size != NULL)
		*size = entry->size;

out:
	hwspin_unlock_irqrestore(smem->hwlock, &flags);

	return ret;
}
EXPORT_SYMBOL(qcom_smem_get);

int qcom_smem_signal(struct qcom_smem *smem, int offset, int bit)
{
	writel(BIT(bit), smem->signal_base + offset);
	return 0;
}
EXPORT_SYMBOL(qcom_smem_signal);

static int qcom_smem_probe(struct platform_device *pdev)
{
	struct qcom_smem *smem;
	struct resource *res;

	smem = devm_kzalloc(&pdev->dev, sizeof(*smem), GFP_KERNEL);
	if (!smem) {
		dev_err(&pdev->dev, "failed to allocate struct smd\n");
		return -ENOMEM;
	}
	smem->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smem->signal_base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!smem->signal_base)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	smem->base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!smem->base)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	smem->aux_base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!smem->aux_base)
		return -ENOMEM;

	smem->hwlock = of_hwspin_lock_request(pdev->dev.of_node, NULL);
	if (IS_ERR(smem->hwlock))
		return PTR_ERR(smem->hwlock);

	dev_set_drvdata(&pdev->dev, smem);

	dev_dbg(&pdev->dev, "Qualcomm Shared Memory Interface probed\n");

	return of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
}

static const struct of_device_id qcom_smem_of_match[] = {
	{ .compatible = "qcom,smem" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smem_of_match);

static struct platform_driver qcom_smem_driver = {
	.probe          = qcom_smem_probe,
	.driver  = {
		.name  = "qcom_smem",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smem_of_match,
	},
};

static int __init qcom_smem_init(void)
{
	return platform_driver_register(&qcom_smem_driver);
}
arch_initcall(qcom_smem_init);

static void __exit qcom_smem_exit(void)
{
	platform_driver_unregister(&qcom_smem_driver);
}
module_exit(qcom_smem_exit)

MODULE_DESCRIPTION("Qualcomm Shared Memory Interface");
MODULE_LICENSE("GPLv2");
