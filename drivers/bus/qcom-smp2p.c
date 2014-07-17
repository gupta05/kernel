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

#define SMP2P_MAX_ENTRY 16
#define SMP2P_MAX_ENTRY_NAME 16

#define SMP2P_MAGIC 0x504D5324

/**
 * struct smp2p_smem - SMP2P SMEM Item Header
 *
 * @magic:  Set to "$SMP" -- used for identification / debug purposes
 * @feature_version:  Feature and version fields
 * @rem_loc_proc_id:  Remote (31:16) and Local (15:0) processor IDs
 * @valid_total_ent:  Valid (31:16) and total (15:0) entries
 * @flags:  Flags (bits 31:2 reserved)
 */
struct smp2p_smem {
	u32 magic;
	u8 version;
	u8 features[3];
	u16 local_id;
	u16 remote_id;
	u16 total_entries;
	u16 valid_entries;
	u32 flags;
};

struct smp2p_entry_v1 {
	u8 name[SMP2P_MAX_ENTRY_NAME];
	u32 entry;
};

struct smp2p_smem_item {
	struct smp2p_smem header;
	struct smp2p_entry_v1 entries[SMP2P_MAX_ENTRY];
};

struct qcom_smp2p {
	struct device *dev;

	struct qcom_smem *smem;

	int local_id;
	int remote_id;

	int irq;

	int signal_offset;
	int signal_bit;

	struct smp2p_smem_item *local_item;
	struct smp2p_smem_item *remote_item;

	struct work_struct negotiate_work;

	struct mutex lock;
};

static void qcom_smp2p_negotiate(struct work_struct *work)
{
	struct qcom_smp2p *smp2p = container_of(work, struct qcom_smp2p, negotiate_work);
	struct smp2p_entry_v1 *entry;
	struct smp2p_smem *hdr;
	size_t size;
	int ret;

	mutex_lock(&smp2p->lock);

	if (!smp2p->remote_item) {
		ret = qcom_smem_get(smp2p->smem, smp2p->remote_id, (void**)&smp2p->remote_item, &size);
		if (ret < 0 || size != ALIGN(sizeof(struct smp2p_smem_item), 8)) {
			dev_err(smp2p->dev, "Unable to acquire remote smp2p item (%d): %d (%d %d)\n", smp2p->remote_id, ret, size, sizeof(struct smp2p_smem_item));
		//	return;
		}
	}

	if (!smp2p->local_item) {
		ret = qcom_smem_alloc(smp2p->smem, smp2p->local_id, sizeof(struct smp2p_smem_item));
		if (ret < 0 && ret != -EEXIST) {
			dev_err(smp2p->dev, "unable to allocate local smp2p item\n");
			return;
		}

		ret = qcom_smem_get(smp2p->smem, smp2p->local_id, (void**)&smp2p->local_item, NULL);
		if (ret < 0) {
			dev_err(smp2p->dev, "Unable to acquire local smp2p item\n");
			return;
		}

		memset(smp2p->local_item, 0, sizeof(struct smp2p_smem_item));
		hdr = &smp2p->local_item->header;
		hdr->magic = SMP2P_MAGIC;
		hdr->local_id = 0;
		hdr->remote_id = 4;
		hdr->total_entries = SMP2P_MAX_ENTRY;
		hdr->valid_entries = 0;

		wmb();
		hdr->version = 1;

		wmb();

		qcom_smem_signal(smp2p->smem, smp2p->signal_offset, smp2p->signal_bit);

		entry = &smp2p->local_item->entries[1];
		strlcpy(entry->name, "master-kernel", sizeof(entry->name));
		hdr->valid_entries++;

		wmb();

		qcom_smem_signal(smp2p->smem, smp2p->signal_offset, smp2p->signal_bit);
	}

	if (smp2p->local_item)
		print_hex_dump(KERN_DEBUG, "local:  ", DUMP_PREFIX_OFFSET, 16, 1, smp2p->local_item, sizeof(struct smp2p_smem_item), true);

	if (smp2p->remote_item)
		print_hex_dump(KERN_DEBUG, "remote: ", DUMP_PREFIX_OFFSET, 16, 1, smp2p->remote_item, sizeof(struct smp2p_smem_item), true);

	mutex_unlock(&smp2p->lock);
}

static irqreturn_t qcom_smp2p_intr(int irq, void *data)
{
	struct qcom_smp2p *smp2p = data;

	dev_err(smp2p->dev, "INTR!\n");
	schedule_work(&smp2p->negotiate_work);

	return IRQ_HANDLED;
}

static int qcom_smp2p_probe(struct platform_device *pdev)
{
	struct qcom_smp2p *smp2p;
	u32 regs[2];
	char *key;
	int ret;

	smp2p = devm_kzalloc(&pdev->dev, sizeof(*smp2p), GFP_KERNEL);
	if (!smp2p) {
		dev_err(&pdev->dev, "failed to allocate struct smp2p\n");
		return -ENOMEM;
	}
	smp2p->dev = &pdev->dev;

	INIT_WORK(&smp2p->negotiate_work, qcom_smp2p_negotiate);
	mutex_init(&smp2p->lock);

	ret = of_property_read_u32_array(pdev->dev.of_node, "reg", regs, 2);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to parse reg property\n");
		return -EINVAL;
	}
	smp2p->local_id = regs[0];
	smp2p->remote_id = regs[1];

	key = "qcom,signal-offset";
	ret = of_property_read_u32(pdev->dev.of_node, key, &smp2p->signal_offset);
	if (ret) {
		dev_err(&pdev->dev, "channel missing %s property\n", key);
		return -EINVAL;
	}

	key = "qcom,signal-bit";
	ret = of_property_read_u32(pdev->dev.of_node, key, &smp2p->signal_bit);
	if (ret) {
		dev_err(&pdev->dev, "channel missing %s property\n", key);
		return -EINVAL;
	}

	smp2p->smem = dev_get_qcom_smem(pdev->dev.parent);
	if (!smp2p->smem) {
		dev_err(&pdev->dev, "failed to acquire smem handle\n");
		return -EINVAL;
	}

	smp2p->irq = platform_get_irq(pdev, 0);
	if (smp2p->irq < 0) {
		dev_err(&pdev->dev, "unable to acquire smp2p interrupt\n");
		return -EINVAL;
	}

	ret = devm_request_irq(&pdev->dev, smp2p->irq, qcom_smp2p_intr,
			IRQF_TRIGGER_RISING, "smp2p", (void *)smp2p);
	if (ret) {
		dev_err(&pdev->dev, "failed to request interrupt\n");
		return -EINVAL;
	}

	dev_dbg(smp2p->dev, "Qualcomm Shared Memory Point to Point initialized\n");

	return 0;
}

static const struct of_device_id qcom_smp2p_of_match[] = {
	{ .compatible = "qcom,smp2p" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smp2p_of_match);

static struct platform_driver qcom_smp2p_driver = {
	.probe          = qcom_smp2p_probe,
	.driver  = {
		.name  = "qcom_smp2p",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smp2p_of_match,
	},
};

static int __init qcom_smp2p_init(void)
{
	return platform_driver_register(&qcom_smp2p_driver);
}
arch_initcall(qcom_smp2p_init);

static void __exit qcom_smp2p_exit(void)
{
	platform_driver_unregister(&qcom_smp2p_driver);
}
module_exit(qcom_smp2p_exit)

MODULE_DESCRIPTION("Qualcomm Shared Memory Point to Point");
MODULE_LICENSE("GPLv2");
