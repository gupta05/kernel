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
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/mfd/syscon.h>

#include <linux/delay.h>

#include <linux/soc/qcom/qcom_smem.h>

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

struct smp2p_entry {
	struct list_head node;
	struct qcom_smp2p *smp2p;

	const char *name;
	struct smp2p_entry_v1 *entry;

	u32 last_value;
	struct irq_domain *domain;
	DECLARE_BITMAP(irq_mask, 32);
	DECLARE_BITMAP(irq_rising, 32);
	DECLARE_BITMAP(irq_falling, 32);

	struct gpio_chip chip;
};

struct qcom_smp2p {
	struct device *dev;

	struct qcom_smem *smem;
	struct smp2p_smem_item *smem_out;
	struct smp2p_smem_item *smem_in;

	int local_id;
	int remote_id;

	int irq;

	struct regmap *ipc_regmap;
	int ipc_offset;
	int ipc_bit;

	struct mutex lock;

	struct list_head inbound;
	struct list_head outbound;
};

static void qcom_smp2p_kick(struct qcom_smp2p *smp2p)
{
	wmb();
	regmap_write(smp2p->ipc_regmap, smp2p->ipc_offset, BIT(smp2p->ipc_bit));
}

static irqreturn_t qcom_smp2p_intr(int irq, void *data)
{
	struct qcom_smp2p *smp2p = data;
	struct smp2p_entry_v1 *entry_v1;
	struct smp2p_entry *entry;
	struct smp2p_smem_item *in;
	size_t size;
	int irq_pin;
	u32 status;
	u32 val;
	int ret;
	int i;

	mutex_lock(&smp2p->lock);

	if (!smp2p->smem_in) {
		ret = qcom_smem_get(smp2p->smem, smp2p->remote_id, (void**)&smp2p->smem_in, &size);
		if (ret < 0 || size != ALIGN(sizeof(struct smp2p_smem_item), 8)) {
			dev_err(smp2p->dev, "Unable to acquire remote smp2p item (%d): %d (%d %d)\n", smp2p->remote_id, ret, size, sizeof(struct smp2p_smem_item));
			smp2p->smem_in = NULL;
		}
	}

	if (smp2p->smem_in /* XXX: && valid entries have changed */) {
		list_for_each_entry(entry, &smp2p->inbound, node) {
			if (entry->entry)
				continue;

			in = smp2p->smem_in;

			for (i = 0; i < in->header.valid_entries; i++) {
				if (strcmp(in->entries[i].name, entry->name) == 0) {
					entry->entry = &in->entries[i];
					break;
				}
			}
		}
	}

	list_for_each_entry(entry, &smp2p->inbound, node) {
		if (!entry->entry)
			continue;

		entry_v1 = entry->entry;
		val = entry_v1->entry;

		status = val ^ entry->last_value;
		entry->last_value = val;

		for_each_set_bit(i, entry->irq_mask, 32) {
			if (!(status & BIT(i)))
				continue;

			irq_pin = irq_find_mapping(entry->domain, i);

			if (val & BIT(i)) {
				if (test_bit(i, entry->irq_rising))
					handle_nested_irq(irq_pin);
			} else {
				if (test_bit(i, entry->irq_falling))
					handle_nested_irq(irq_pin);
			}

		}
	}

#if 0
	if (smp2p->smem_out)
		print_hex_dump(KERN_DEBUG, "local:  ", DUMP_PREFIX_OFFSET, 16, 1, smp2p->smem_out, sizeof(struct smp2p_smem_item), true);

	if (smp2p->smem_in)
		print_hex_dump(KERN_DEBUG, "remote: ", DUMP_PREFIX_OFFSET, 16, 1, smp2p->smem_in, sizeof(struct smp2p_smem_item), true);
#endif

	mutex_unlock(&smp2p->lock);
	return IRQ_HANDLED;
}

static void smp2p_mask_irq(struct irq_data *irqd)
{
	struct smp2p_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);

	clear_bit(irq, entry->irq_mask);
}

static void smp2p_unmask_irq(struct irq_data *irqd)
{
	struct smp2p_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);

	set_bit(irq, entry->irq_mask);
}

static int smp2p_set_irq_type(struct irq_data *irqd, unsigned int type)
{
	struct smp2p_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);

	if (!(type & IRQ_TYPE_EDGE_BOTH))
		return -EINVAL;

	if (type & IRQ_TYPE_EDGE_RISING)
		set_bit(irq, entry->irq_rising);
	else
		clear_bit(irq, entry->irq_rising);

	if (type & IRQ_TYPE_EDGE_FALLING)
		set_bit(irq, entry->irq_falling);
	else
		clear_bit(irq, entry->irq_falling);

	return 0;
}

static struct irq_chip smp2p_irq_chip = {
	.name           = "smp2p",
	.irq_mask       = smp2p_mask_irq,
	.irq_unmask     = smp2p_unmask_irq,
	.irq_set_type	= smp2p_set_irq_type,
};

static int smp2p_irq_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	struct smp2p_entry *entry = d->host_data;

	irq_set_chip_and_handler(irq, &smp2p_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, entry);
	irq_set_nested_thread(irq, 1);

#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif

	return 0;
}

static const struct irq_domain_ops smp2p_irq_ops = {
	.map = smp2p_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static int smp2p_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct smp2p_entry *entry = container_of(chip, struct smp2p_entry, chip);

	if (value)
		entry->entry->entry |= BIT(offset);
	else
		entry->entry->entry &= ~BIT(offset);

	qcom_smp2p_kick(entry->smp2p);

	return 0;
}

static void smp2p_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	smp2p_gpio_direction_output(chip, offset, value);
}

static int qcom_smp2p_outgoing_entry(struct qcom_smp2p *smp2p,
				     struct smp2p_entry *entry,
				     struct device_node *node)
{
	struct gpio_chip *chip;
	int ret;

	chip = &entry->chip;
	chip->base = -1;
	chip->ngpio = 32;
	chip->label = entry->name;
	chip->dev = smp2p->dev;
	chip->owner = THIS_MODULE;
	chip->of_node = node;

	chip->set = smp2p_gpio_set;
	chip->direction_output = smp2p_gpio_direction_output;

	ret = gpiochip_add(chip);
	if (ret)
		dev_err(smp2p->dev, "failed register gpiochip\n");

	return 0;
}

static int qcom_smp2p_alloc_outgoing(struct qcom_smp2p *smp2p)
{
	struct smp2p_entry_v1 *entry_v1;
	struct smp2p_entry *entry;
	struct smp2p_smem *hdr;
	int ret;

	ret = qcom_smem_alloc(smp2p->smem, smp2p->local_id, sizeof(struct smp2p_smem_item));
	if (ret < 0 && ret != -EEXIST) {
		dev_err(smp2p->dev, "unable to allocate local smp2p item\n");
		return ret;
	}

	ret = qcom_smem_get(smp2p->smem, smp2p->local_id, (void**)&smp2p->smem_out, NULL);
	if (ret < 0) {
		dev_err(smp2p->dev, "Unable to acquire local smp2p item\n");
		return ret;
	}

	memset(smp2p->smem_out, 0, sizeof(struct smp2p_smem_item));
	hdr = &smp2p->smem_out->header;
	hdr->magic = SMP2P_MAGIC;
	hdr->local_id = 0;
	hdr->remote_id = 4;
	hdr->total_entries = SMP2P_MAX_ENTRY;
	hdr->valid_entries = 0;

	wmb();
	hdr->version = 1;

	qcom_smp2p_kick(smp2p);

	list_for_each_entry(entry, &smp2p->outbound, node) {
		entry_v1 = &smp2p->smem_out->entries[hdr->valid_entries++];
		strlcpy(entry_v1->name, entry->name, sizeof(entry_v1->name));

		entry->entry = entry_v1;
	}

	qcom_smp2p_kick(smp2p);

	return 0;
}

static int qcom_smp2p_probe(struct platform_device *pdev)
{
	struct device_node *syscon_np;
	struct smp2p_entry *entry;
	struct device_node *node;
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

	mutex_init(&smp2p->lock);

	ret = of_property_read_u32_array(pdev->dev.of_node, "qcom,smem-ids", regs, 2);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to parse qcom,smem-ids property\n");
		return -EINVAL;
	}
	smp2p->local_id = regs[0];
	smp2p->remote_id = regs[1];

	syscon_np = of_parse_phandle(pdev->dev.of_node, "qcom,ipc", 0);
	if (!syscon_np) {
		dev_err(&pdev->dev, "no qcom,ipc node\n");
		return -ENODEV;
	}

	smp2p->ipc_regmap = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(smp2p->ipc_regmap))
		return PTR_ERR(smp2p->ipc_regmap);

	key = "qcom,ipc";
	ret = of_property_read_u32_index(pdev->dev.of_node, key, 1, &smp2p->ipc_offset);
	if (ret < 0) {
		dev_err(&pdev->dev, "no offset in %s\n", key);
		return -EINVAL;
	}

	ret = of_property_read_u32_index(pdev->dev.of_node, key, 2, &smp2p->ipc_bit);
	if (ret < 0) {
		dev_err(&pdev->dev, "no bit in %s\n", key);
		return -EINVAL;
	}

	smp2p->smem = of_get_qcom_smem(pdev->dev.of_node);
	if (IS_ERR(smp2p->smem)) {
		if (PTR_ERR(smp2p->smem) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to acquire smem handle\n");
		return PTR_ERR(smp2p->smem);
	}

	smp2p->irq = platform_get_irq(pdev, 0);
	if (smp2p->irq < 0) {
		dev_err(&pdev->dev, "unable to acquire smp2p interrupt\n");
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(&pdev->dev, smp2p->irq, NULL, qcom_smp2p_intr,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "smp2p", (void *)smp2p);
	if (ret) {
		dev_err(&pdev->dev, "failed to request interrupt\n");
		return -EINVAL;
	}

	INIT_LIST_HEAD(&smp2p->inbound);
	INIT_LIST_HEAD(&smp2p->outbound);

	for_each_available_child_of_node(pdev->dev.of_node, node) {
		entry = devm_kzalloc(&pdev->dev, sizeof(*entry), GFP_KERNEL);
		if (!entry)
			return -ENOMEM;

		entry->smp2p = smp2p;

		ret = of_property_read_string(node, "qcom,entry-name", &entry->name);
		if (ret < 0)
			return ret;

		if (of_property_read_bool(node, "qcom,outbound")) {
			ret = qcom_smp2p_outgoing_entry(smp2p, entry, node);
			if (ret < 0)
				return ret;

			list_add(&entry->node, &smp2p->outbound);
		} else if (of_property_read_bool(node, "qcom,inbound")) {
			entry->domain = irq_domain_add_linear(node, 32, &smp2p_irq_ops, entry);
			if (!entry->domain)
				return -ENOMEM;

			list_add(&entry->node, &smp2p->inbound);
		} else {
			dev_err(&pdev->dev, "neither inbound nor outbound\n");
			return -EINVAL;
		}
	}

	ret = qcom_smp2p_alloc_outgoing(smp2p);
	if (ret < 0)
		return ret;

	dev_err(smp2p->dev, "Qualcomm Shared Memory Point to Point initialized\n");

	return 0;
}

static const struct of_device_id qcom_smp2p_of_match[] = {
	{ .compatible = "qcom,smp2p-v1" },
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
