/*
 * Copyright (c) 2015, Sony Mobile Communications Inc.
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
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/mfd/syscon.h>

#include <linux/delay.h>

#include <linux/soc/qcom/smem.h>

/*
 * SMSM a set of entries, each 32 bits wide
 *

 * kick_mask are layouted as
 * entry0: [host0 ... hostN]
 *	.
 *	.
 * entryN: [host0 ... hostN]

 */

#define SMEM_SMSM_SHARED_STATE		85
#define SMEM_SMSM_CPU_INTR_MASK		333
#define SMEM_SMSM_SIZE_INFO		419

#define SMSM_DEFAULT_NUM_ENTRIES	8
#define SMSM_DEFAULT_NUM_HOSTS		3

struct qcom_smsm_entry;

/**
 * @kick_mask:	reference to the kick mask SMEM item
 */
struct qcom_smsm {
	struct device *dev;

	void *mem;
	size_t size;

	u32 local_host;

	u32 num_hosts;
	u32 num_entries;

	struct qcom_smsm_entry *entries;
};

/**
 * struct qcom_smsm_entry - per entry context
 * @kick_mask:	array of kick mask for each host
 */
struct qcom_smsm_entry {
	struct qcom_smsm *smsm;

	struct irq_domain *domain;
	DECLARE_BITMAP(irq_enabled, 32);
	DECLARE_BITMAP(irq_rising, 32);
	DECLARE_BITMAP(irq_falling, 32);
	u32 last_value;

	struct gpio_chip chip;
	bool chip_added;
	struct regmap *ipc_regmap;
	int ipc_bit;
	int ipc_offset;

	u32 *mem;
	u32 *kick_mask;
};

static int smsm_gpio_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct qcom_smsm_entry *entry = container_of(chip, struct qcom_smsm_entry, chip);
	struct qcom_smsm *smsm = entry->smsm;
	u32 host;
	u32 val;

	/* Update the entry */
	val = readl(entry->mem);
	if (value)
		val |= BIT(offset);
	else
		val &= ~BIT(offset);
	writel(val, entry->mem);

	if (entry->kick_mask) {
		/* Iterate over all hosts to check whom wants a kick */
		for (host = 0; host < smsm->num_hosts; host++) {
			val = readl(entry->kick_mask + host);

			if (val & BIT(offset)) {
				regmap_write(entry->ipc_regmap,
					     entry->ipc_offset,
					     BIT(entry->ipc_bit));
			}
		}
	}

	return 0;
}

static void smsm_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	smsm_gpio_output(chip, offset, value);
}

static int smsm_parse_ipc(struct device *dev,
			  struct qcom_smsm_entry *entry,
			  struct device_node *node)
{
	struct device_node *syscon;
	const char *key;
	int ret;

	syscon = of_parse_phandle(node, "qcom,ipc", 0);
	if (!syscon)
		return 0;

	entry->ipc_regmap = syscon_node_to_regmap(syscon);
	if (IS_ERR(entry->ipc_regmap))
		return PTR_ERR(entry->ipc_regmap);

	key = "qcom,ipc";
	ret = of_property_read_u32_index(node, key, 1, &entry->ipc_offset);
	if (ret < 0) {
		dev_err(dev, "no offset in %s\n", key);
		return -EINVAL;
	}

	ret = of_property_read_u32_index(node, key, 2, &entry->ipc_bit);
	if (ret < 0) {
		dev_err(dev, "no bit in %s\n", key);
		return -EINVAL;
	}

	return 0;
}

static int smsm_get_size_info(struct qcom_smsm *smsm)
{
	size_t size;
	int ret;
	struct {
		u32 num_hosts;
		u32 num_entries;
		u32 reserved0;
		u32 reserved1;
	} *info;

	ret = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_SMSM_SIZE_INFO,
			    (void **)&info, &size);
	if (ret == -ENOENT || size != sizeof(*info)) {
		dev_warn(smsm->dev, "no smsm size info, using defaults\n");
		smsm->num_entries = SMSM_DEFAULT_NUM_ENTRIES;
		smsm->num_hosts = SMSM_DEFAULT_NUM_HOSTS;
		return 0;
	} else if (ret) {
		dev_err(smsm->dev, "unable to retrieve smsm size info\n");
		return ret;
	}

	smsm->num_entries = info->num_entries;
	smsm->num_hosts = info->num_hosts;

	dev_dbg(smsm->dev, "found custom size of smsm: %d entries %d hosts\n", smsm->num_entries, smsm->num_hosts);

	return 0;
}

static irqreturn_t smsm_intr(int irq, void *data)
{
	struct qcom_smsm_entry *entry = data;
	unsigned i;
	int irq_pin;
	u32 changed;
	u32 val;

	val = readl(entry->mem);
	changed = val ^ entry->last_value;
	entry->last_value = val;

	for_each_set_bit(i, entry->irq_enabled, 32) {
		if (!(changed & BIT(i)))
			continue;

		if (val & BIT(i)) {
			if (test_bit(i, entry->irq_rising)) {
				irq_pin = irq_find_mapping(entry->domain, i);
				handle_nested_irq(irq_pin);
			}
		} else {
			if (test_bit(i, entry->irq_falling)) {
				irq_pin = irq_find_mapping(entry->domain, i);
				handle_nested_irq(irq_pin);
			}
		}
	}

	return IRQ_HANDLED;
}

static void smsm_mask_irq(struct irq_data *irqd)
{
	struct qcom_smsm_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);
	struct qcom_smsm *smsm = entry->smsm;
	u32 val;

	if (entry->kick_mask) {
		val = readl(entry->kick_mask + smsm->local_host);
		val &= ~BIT(irq);
		writel(val, entry->kick_mask + smsm->local_host);
	}

	clear_bit(irq, entry->irq_enabled);
}

static void smsm_unmask_irq(struct irq_data *irqd)
{
	struct qcom_smsm_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);
	struct qcom_smsm *smsm = entry->smsm;
	u32 val;

	set_bit(irq, entry->irq_enabled);

	if (entry->kick_mask) {
		val = readl(entry->kick_mask + smsm->local_host);
		val |= BIT(irq);
		writel(val, entry->kick_mask + smsm->local_host);
	}
}

static int smsm_set_irq_type(struct irq_data *irqd, unsigned int type)
{
	struct qcom_smsm_entry *entry = irq_data_get_irq_chip_data(irqd);
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

static struct irq_chip smsm_irq_chip = {
	.name           = "smsm",
	.irq_mask       = smsm_mask_irq,
	.irq_unmask     = smsm_unmask_irq,
	.irq_set_type	= smsm_set_irq_type,
};

static int smsm_irq_map(struct irq_domain *d,
			unsigned int irq,
			irq_hw_number_t hw)
{
	struct qcom_smsm_entry *entry = d->host_data;

	irq_set_chip_and_handler(irq, &smsm_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, entry);
	irq_set_nested_thread(irq, 1);

#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif

	return 0;
}

static const struct irq_domain_ops smsm_irq_ops = {
	.map = smsm_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static int smsm_inbound_entry(struct qcom_smsm *smsm,
			      struct qcom_smsm_entry *entry,
			      struct device_node *node)
{
	int ret;
	int irq;

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		dev_err(smsm->dev, "failed to parse smsm interrupt\n");
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(smsm->dev, irq,
					NULL, smsm_intr,
					IRQF_ONESHOT,
					"smsm", (void *)entry);
	if (ret) {
		dev_err(smsm->dev, "failed to request interrupt\n");
		return ret;
	}

	entry->domain = irq_domain_add_linear(node, 32, &smsm_irq_ops, entry);
	if (!entry->domain) {
		dev_err(smsm->dev, "failed to add irq_domain\n");
		return -ENOMEM;
	}

	return 0;
}

static int smsm_outbound_entry(struct qcom_smsm *smsm,
			       struct qcom_smsm_entry *entry,
			       struct device_node *node)
{
	int ret;

	entry->chip.base = -1;
	entry->chip.dev = smsm->dev;
	entry->chip.direction_output = smsm_gpio_output;
	entry->chip.label = node->name;
	entry->chip.ngpio = 32;
	entry->chip.of_node = node;
	entry->chip.owner = THIS_MODULE;
	entry->chip.set = smsm_gpio_set;
	ret = gpiochip_add(&entry->chip);
	if (ret)
		dev_err(smsm->dev, "failed register gpiochip\n");

	entry->chip_added = true;

	return ret;
};

static int qcom_smsm_probe(struct platform_device *pdev)
{
	struct qcom_smsm_entry *entry;
	struct device_node *node;
	struct qcom_smsm *smsm;
	void *intr_mask;
	size_t size;
	size_t k;
	u32 sid;
	int ret;

	smsm = devm_kzalloc(&pdev->dev, sizeof(*smsm), GFP_KERNEL);
	if (!smsm)
		return -ENOMEM;
	smsm->dev = &pdev->dev;

	ret = smsm_get_size_info(smsm);
	if (ret)
		return ret;

	smsm->entries = devm_kcalloc(&pdev->dev,
				     smsm->num_entries,
				     sizeof(struct qcom_smsm_entry),
				     GFP_KERNEL);
	if (!smsm->entries)
		return -ENOMEM;

	of_property_read_u32(pdev->dev.of_node,
			     "qcom,local-host",
			     &smsm->local_host);

	ret = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, SMEM_SMSM_SHARED_STATE,
			      smsm->num_entries * sizeof(u32));
	if (ret < 0 && ret != -EEXIST) {
		dev_err(&pdev->dev, "unable to allocate shared state entry\n");
		return ret;
	}

	ret = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_SMSM_SHARED_STATE,
			    (void **)&smsm->mem,
			    &smsm->size);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to acquire shared state entry\n");
		return ret;
	}

	size = smsm->num_entries * smsm->num_hosts * sizeof(u32);
	ret = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, SMEM_SMSM_CPU_INTR_MASK, size);
	if (ret < 0 && ret != -EEXIST) {
		dev_err(&pdev->dev, "unable to allocate smsm interrupt mask\n");
		return -ret;
	}

	ret = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_SMSM_CPU_INTR_MASK,
			    &intr_mask, &size);
	if (!ret) {
		/* Setup kick_mask pointers and unsubscribe to any kicks */
		for (k = 0; k < smsm->num_entries; k++) {
			smsm->entries[k].kick_mask = intr_mask + k * smsm->num_hosts;
			writel(0, smsm->entries[k].kick_mask + smsm->local_host);
		}
	}

	for_each_available_child_of_node(pdev->dev.of_node, node) {
		ret = of_property_read_u32(node, "reg", &sid);
		if (ret || sid >= smsm->num_entries) {
			dev_err(&pdev->dev, "invalid reg of entry\n");
			return -EINVAL;
		}
		entry = &smsm->entries[sid];

		ret = smsm_parse_ipc(&pdev->dev, entry, node);
		if (ret < 0)
			goto unwind_interfaces;

		if (of_property_read_bool(node, "interrupt-controller")) {
			smsm_inbound_entry(smsm, entry, node);
		} else if (of_property_read_bool(node, "gpio-controller")) {
			smsm_outbound_entry(smsm, entry, node);
		}
	}

	platform_set_drvdata(pdev, smsm);

	return 0;

unwind_interfaces:
	for (sid = 0; sid < smsm->num_entries; sid++) {
		if (smsm->entries[sid].domain)
			irq_domain_remove(smsm->entries[sid].domain);
		else if (smsm->entries[sid].chip_added)
			gpiochip_remove(&smsm->entries[sid].chip);
	}

	return ret;
}

static int qcom_smsm_remove(struct platform_device *pdev)
{
	struct qcom_smsm *smsm = platform_get_drvdata(pdev);
	unsigned sid;

	for (sid = 0; sid < smsm->num_entries; sid++) {
		if (smsm->entries[sid].domain)
			irq_domain_remove(smsm->entries[sid].domain);
		else if (smsm->entries[sid].chip_added)
			gpiochip_remove(&smsm->entries[sid].chip);
	}

	return 0;
}

static const struct of_device_id qcom_smsm_of_match[] = {
	{ .compatible = "qcom,smsm" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smsm_of_match);

static struct platform_driver qcom_smsm_driver = {
	.probe = qcom_smsm_probe,
	.remove = qcom_smsm_remove,
	.driver  = {
		.name  = "qcom-smsm",
		.of_match_table = qcom_smsm_of_match,
	},
};
module_platform_driver(qcom_smsm_driver);

MODULE_DESCRIPTION("Qualcomm Shared Memory State Machine driver");
MODULE_LICENSE("GPLv2");
