/*
 * Copyright (c) 2015, Sony Mobile Communications AB.
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
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/hwspinlock.h>
#include <linux/soc/qcom/smem.h>

#define AUX_BASE_MASK		0xfffffffc
#define HWSPINLOCK_TIMEOUT	1000
#define SMEM_MAX_ITEMS		512

/**
  * struct smem_proc_comm - proc_comm communication struct (legacy)
  * @command:	current command to be executed
  * @status:	status of the currently requested command
  * @params:	parameters to the command
  */
struct smem_proc_comm {
	u32 command;
	u32 status;
	u32 params[2];
};

/**
 * struct smem_entry - entry to reference smem items on the heap
 * @allocated:	boolean to indicate if this entry is used
 * @offset:	offset to the allocated space
 * @size:	size of the allocated space, 8 byte aligned
 * @aux_base:	base address for the memory region used by this unit, or 0 for
 *		the default region. bits 0,1 are reserved
 */
struct smem_entry {
	u32 allocated;
	u32 offset;
	u32 size;
	u32 aux_base; /* bits 1:0 reserved */
};

/**
 * struct smem_header - header found in beginning of primary smem region
 * @proc_comm:		proc_comm communication interface (legacy)
 * @version:		array of versions for the various subsystems
 * @smem_initialized:	boolean to indicate that smem is initialized
 * @free_offset:	index of the first unallocated byte in smem
 * @available:		number of bytes available for allocation
 * @unused:		reserved field
 * toc:			array of references to smem entries
 */
struct smem_header {
	struct smem_proc_comm proc_comm[4];
	u32 version[32];
	u32 smem_initialized;
	u32 free_offset;
	u32 available;
	u32 unused;
	struct smem_entry toc[SMEM_MAX_ITEMS];
};

/**
  * struct smem_area - memory region used for smem heap
  * @aux_base:	physical base address of the region, used for entry lookup
  * @virt_base:	virtual address of the mapping
  */
struct smem_region {
	u32 aux_base;
	void __iomem *virt_base;
};

/**
  * struct qcom_smem - control struct for the smem driver
  * @dev:		device pointer
  * @hwlock:		hwspinlock to be held during heap operations
  * @num_regions:	number of entires in the regions array
  * @regions:		array of memory regions, region 0 contains smem_header
  */
struct qcom_smem {
	struct device *dev;

	struct hwspinlock *hwlock;

	DECLARE_BITMAP(busy, SMEM_MAX_ITEMS);

	unsigned num_regions;
	struct smem_region regions[0];
};

/* Pointer to the one and only smem handle */
static struct qcom_smem *smem;

/**
 * qcom_smem_alloc - allocate space for a smem item
 * @item:	smem item handle
 * @size:	number of bytes to be allocated
 *
 * Allocate space for a given smem item of size @size, given that the item is
 * not yet allocated.
 */
int qcom_smem_alloc(unsigned item, size_t size)
{
	struct smem_header *header;
	struct smem_entry *entry;
	unsigned long flags;
	int ret;

	if (!smem)
		return -EPROBE_DEFER;

	if (WARN_ON(item >= SMEM_MAX_ITEMS))
		return -EINVAL;

	if (test_bit(item, smem->busy))
		return -EBUSY;

	ret = hwspin_lock_timeout_irqsave(smem->hwlock,
					  HWSPINLOCK_TIMEOUT,
					  &flags);
	if (ret)
		return ret;

	header = smem->regions[0].virt_base;
	entry = &header->toc[item];
	if (entry->allocated) {
		ret = -EEXIST;
		goto out;
	}

	size = ALIGN(size, 8);
	if (WARN_ON(size > header->available)) {
		ret = -ENOMEM;
		goto out;
	}

	entry->offset = header->free_offset;
	entry->size = size;

	/* Ensure 'allocated' is the last field to be updated */
	wmb();
	entry->allocated = 1;

	header->free_offset += size;
	header->available -= size;

out:
	hwspin_unlock_irqrestore(smem->hwlock, &flags);
	return ret;
}
EXPORT_SYMBOL(qcom_smem_alloc);

/**
 * qcom_smem_get - resolve ptr of size of a smem item
 * @item:	smem item handle
 * @ptr:	pointer to be filled out with address of the item
 * @size:	pointer to be filled out with size of the item
 *
 * Looks up pointer and size of a smem item.
 */
int qcom_smem_get(unsigned item, void **ptr, size_t *size)
{
	struct smem_header *header = smem->regions[0].virt_base;
	struct smem_region *area;
	struct smem_entry *entry;
	u32 aux_base;
	unsigned i;

	if (!smem)
		return -EPROBE_DEFER;

	if (WARN_ON(item >= SMEM_MAX_ITEMS))
		return -EINVAL;

	if (test_bit(item, smem->busy))
		return -EBUSY;

	header = smem->regions[0].virt_base;
	entry = &header->toc[item];
	if (!entry->allocated)
		return -ENXIO;

	if (ptr != NULL) {
		aux_base = entry->aux_base & AUX_BASE_MASK;

		for (i = 0; i < smem->num_regions; i++) {
			area = &smem->regions[i];

			if (area->aux_base == aux_base || !aux_base) {
				*ptr = area->virt_base + entry->offset;
				break;
			}
		}
	}
	if (size != NULL)
		*size = entry->size;

	set_bit(item, smem->busy);
	return 0;
}
EXPORT_SYMBOL(qcom_smem_get);

int qcom_smem_put(unsigned item)
{
	if (WARN_ON(!test_bit(item, smem->busy)))
		return -EINVAL;

	clear_bit(item, smem->busy);
	return 0;
}
EXPORT_SYMBOL(qcom_smem_put);

unsigned qcom_smem_get_free_space(void)
{
	struct smem_header *header = smem->regions[0].virt_base;

	return header->available;
}
EXPORT_SYMBOL(qcom_smem_get_free_space);

static int qcom_smem_count_mem_regions(struct platform_device *pdev)
{
	struct resource *res;
	int num_regions = 0;
	int i;

	for (i = 0; i < pdev->num_resources; i++) {
		res = &pdev->resource[i];

		if (resource_type(res) == IORESOURCE_MEM)
			num_regions++;
	}

	return num_regions;
}

static int qcom_smem_probe(struct platform_device *pdev)
{
	struct qcom_smem *s;
	struct resource *res;
	size_t array_size;
	int num_regions = 0;
	int hwlock_id;
	int i;

	num_regions = qcom_smem_count_mem_regions(pdev);
	if (num_regions == 0) {
		dev_err(&pdev->dev, "no smem regions specified\n");
		return -EINVAL;
	}

	array_size = num_regions * sizeof(struct smem_region);
	s = devm_kzalloc(&pdev->dev, sizeof(*s) + array_size, GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = &pdev->dev;
	s->num_regions = num_regions;

	for (i = 0; i < num_regions; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);

		s->regions[i].aux_base = (u32)res->start;
		s->regions[i].virt_base = devm_ioremap(&pdev->dev,
							  res->start,
							  resource_size(res));
		if (!s->regions[i].virt_base)
			return -ENOMEM;
	}

	hwlock_id = of_hwspin_lock_get_id(pdev->dev.of_node, 0);
	if (hwlock_id < 0) {
		dev_err(&pdev->dev, "failed to retrieve hwlock\n");
		return hwlock_id;
	}

	s->hwlock = hwspin_lock_request_specific(hwlock_id);
	if (!s->hwlock)
		return -ENXIO;

	smem = s;

	return 0;
}

static int qcom_smem_remove(struct platform_device *pdev)
{
	if (!bitmap_empty(smem->busy, SMEM_MAX_ITEMS))
		return -EBUSY;

	hwspin_lock_free(smem->hwlock);

	return 0;
}

static const struct of_device_id qcom_smem_of_match[] = {
	{ .compatible = "qcom,smem" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smem_of_match);

static struct platform_driver qcom_smem_driver = {
	.probe = qcom_smem_probe,
	.remove = qcom_smem_remove,
	.driver  = {
		.name = "qcom_smem",
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

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm Shared Memory Interface");
MODULE_LICENSE("GPLv2");
