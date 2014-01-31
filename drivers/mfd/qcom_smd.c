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

#include <linux/delay.h>

#define SMD_CHANNEL_NAME_LEN 20

struct smd_half_channel_word_access;

struct qcom_smd_channel {
	struct qcom_smem *smem;
	struct qcom_smd_device qsdev;

	int state;

	struct smd_half_channel_word_access *tx_info;
	struct smd_half_channel_word_access *rx_info;

	struct mutex tx_lock;

	void *tx_fifo;
	void *rx_fifo;
	int fifo_size;

	int (*cb)(struct qcom_smd_device *, void *, size_t);

	int pkt_size;

	int smd_irq;
	int smsm_irq;

	unsigned signal_bit;
	unsigned signal_offset;

};

struct qcom_smem_child {
	int smd_irq;
	int smsm_irq;
	int edge;

	int signal_bit;
	int signal_offset;

	int disabled;
	struct device_node *of_node;

	struct qcom_smd_channel *channel;
};

struct qcom_smem {
	struct device *dev;

	void __iomem *base;
	void __iomem *aux_base;
	void __iomem *signal_base;

	struct qcom_smem_child *children;
	unsigned n_children;
};

#define SMEM_CHANNEL_ALLOC_TBL	13
#define SMEM_SMD_BASE_ID	14
#define SMEM_SMD_FIFO_BASE_ID	338

#define SMEM_NUM_SMD_STREAM_CHANNELS        64

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

/* 'type' field of smd_alloc_elm structure
 * has the following breakup
 * bits 0-7   -> channel type
 * bits 8-11  -> xfer type
 * bits 12-31 -> reserved
 */
struct smd_alloc_elm {
	char name[SMD_CHANNEL_NAME_LEN];
	uint32_t cid;
	uint32_t type;
	uint32_t ref_count;
};

#define SMD_SS_CLOSED            0x00000000
#define SMD_SS_OPENING           0x00000001
#define SMD_SS_OPENED            0x00000002
#define SMD_SS_FLUSHING          0x00000003
#define SMD_SS_CLOSING           0x00000004
#define SMD_SS_RESET             0x00000005
#define SMD_SS_RESET_OPENING     0x00000006

static int smem_find(struct qcom_smem *smem, int smem_id, void **ptr, size_t *size)
{
	struct smem_shared *shared = smem->base;
	struct smem_heap_entry *toc = shared->heap_toc;
	struct smem_heap_entry *entry;

	entry = &toc[smem_id];

	if (!entry->allocated)
		return -EIO;

	if (ptr != NULL) {
		if (entry->reserved)
			*ptr = smem->aux_base + entry->offset;
		else
			*ptr = smem->base + entry->offset;
	}
	if (size != NULL)
		*size = entry->size;

	return 0;
}

static int smem_find_smd_channel(struct qcom_smem *smem, int edge, char *name)
{
	struct smd_alloc_elm *elm;
	int ret;
	int i;

	ret = smem_find(smem, SMEM_CHANNEL_ALLOC_TBL, (void**)&elm, NULL);
	if (ret < 0) {
		dev_err(smem->dev, "%d is not allocated\n", SMEM_CHANNEL_ALLOC_TBL);
		return ret;
	};

	for (i = 0; i < SMEM_NUM_SMD_STREAM_CHANNELS; i++) {
		if (!elm[i].ref_count)
			continue;
		if (!elm[i].name[0])
			continue;

		if ((elm[i].type & 0xff) == edge) {
			if (name)
				strlcpy(name, elm[i].name, SMD_CHANNEL_NAME_LEN);
			BUG_ON(elm[i].cid != i);
			return i;
		}
	}

	return -ENOENT;
}

static int qcom_smd_dev_match(struct device *dev, struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static int qcom_smd_dev_probe(struct device *dev)
{
	struct qcom_smd_device *qsdev = container_of(dev, struct qcom_smd_device, dev);
	struct qcom_smd_driver *qsdrv = container_of(qsdev->dev.driver, struct qcom_smd_driver, driver);
	struct qcom_smd_channel *channel = container_of(qsdev, struct qcom_smd_channel, qsdev);

	channel->cb = qsdrv->callback;

	return qsdrv->probe(qsdev);
}

static int qcom_smd_dev_remove(struct device *dev)
{
	struct qcom_smd_device *qsdev = container_of(dev, struct qcom_smd_device, dev);
	struct qcom_smd_driver *qsdrv = container_of(qsdev->dev.driver, struct qcom_smd_driver, driver);
	struct qcom_smd_channel *channel = container_of(qsdev, struct qcom_smd_channel, qsdev);

	if (qsdrv->remove)
		qsdrv->remove(qsdev);

	channel->cb = NULL;

	return 0;
}

static struct bus_type qcom_smd_bus = {
	.name = "qcom_smd",
	.match = qcom_smd_dev_match,
	.probe = qcom_smd_dev_probe,
	.remove = qcom_smd_dev_remove,
};

int register_qcom_smd_driver(struct qcom_smd_driver *qsdrv)
{
	qsdrv->driver.bus = &qcom_smd_bus;
	return driver_register(&qsdrv->driver);
}
EXPORT_SYMBOL(register_qcom_smd_driver);

struct smd_half_channel_word_access {
	unsigned state;
	unsigned fDSR;
	unsigned fCTS;
	unsigned fCD;
	unsigned fRI;
	unsigned fHEAD;
	unsigned fTAIL;
	unsigned fSTATE;
	unsigned fBLOCKREADINTR;
	unsigned tail;
	unsigned head;
};

struct smd_shared_v2_word_access {
	struct smd_half_channel_word_access ch0;
	struct smd_half_channel_word_access ch1;
};

static void qcom_smd_signal_channel(struct qcom_smd_channel *channel)
{
	struct qcom_smem *smem = channel->smem;

	writel(BIT(channel->signal_bit), smem->signal_base + channel->signal_offset);
}

static int qcom_smd_write_fifo(struct qcom_smd_channel *channel, void *data, size_t count)
{
	size_t len;

	len = min(count, channel->fifo_size - channel->tx_info->head);
	if (len)
		memcpy(channel->tx_fifo + channel->tx_info->head, data, len);

	if (len != count)
		memcpy(channel->tx_fifo, data + len, count - len);

	channel->tx_info->head = (channel->tx_info->head + count) & (channel->fifo_size - 1);

	return count;
}

static size_t qcom_smd_write_avail(struct qcom_smd_channel *channel)
{
	return channel->fifo_size - ((channel->tx_info->head - channel->tx_info->tail) & (channel->fifo_size - 1));
}

int qcom_smd_send(struct qcom_smd_device *qsdev, void *data, int len)
{
	struct qcom_smd_channel *channel = container_of(qsdev, struct qcom_smd_channel, qsdev);
	u32 hdr[5] = {len,};

	mutex_lock(&channel->tx_lock);

	while (qcom_smd_write_avail(channel) < sizeof(hdr) + len)
		msleep(1);

	qcom_smd_write_fifo(channel, hdr, sizeof(hdr));

	qcom_smd_write_fifo(channel, data, len);

	channel->tx_info->fHEAD = 1;
	qcom_smd_signal_channel(channel);

	mutex_unlock(&channel->tx_lock);

	return len;
}
EXPORT_SYMBOL(qcom_smd_send);

static int qcom_smd_read_fifo(struct qcom_smd_channel *channel, void *buf, size_t count)
{
	size_t len;

	len = min(count, channel->fifo_size - channel->rx_info->head);
	if (len)
		memcpy(buf, channel->rx_fifo + channel->rx_info->head, len);

	if (len != count)
		memcpy(buf + len, channel->rx_fifo, count - len);

	channel->rx_info->tail = (channel->rx_info->tail + count) & (channel->fifo_size - 1);

	return count;
}

static size_t qcom_smd_read_avail(struct qcom_smd_channel *channel)
{
	return (channel->rx_info->head - channel->rx_info->tail) & (channel->fifo_size - 1);
}

static irqreturn_t qcom_smd_channel_intr(int irq, void *dev)
{
	struct qcom_smd_device *qsdev = dev;
	struct qcom_smd_channel *channel = container_of(qsdev, struct qcom_smd_channel, qsdev);
	char buf[256];
	int count;
	void *data;
	int state;

	if (channel->state == SMD_SS_CLOSED)
		return IRQ_HANDLED;

	state = channel->rx_info->state;
	if (state != channel->state) {
		if (channel->state == SMD_SS_OPENING && state == SMD_SS_OPENED) {
			channel->tx_info->fDSR = 1;
			channel->tx_info->fCTS = 1;
			channel->tx_info->fCD = 1;

			channel->tx_info->state = SMD_SS_OPENED;
			channel->tx_info->fSTATE = 1;

			qcom_smd_signal_channel(channel);
		}

		channel->state = state;
	}

	if (channel->rx_info->tail != channel->rx_info->head) {
		if (channel->rx_info->head - channel->rx_info->tail >= 20) {
			memcpy(&channel->pkt_size, channel->rx_fifo, sizeof(channel->pkt_size));
			channel->rx_info->tail += 20;
		}

		if (channel->pkt_size && qcom_smd_read_avail(channel) == channel->pkt_size) {
			if (channel->rx_info->head > channel->rx_info->tail) {
				data = channel->rx_fifo + channel->rx_info->tail;
				count = channel->rx_info->head - channel->rx_info->tail;
				if (channel->cb)
					channel->cb(qsdev, data, count);
				channel->rx_info->tail = channel->rx_info->head;
				channel->pkt_size = 0;
			} else {
				count = qcom_smd_read_fifo(channel, buf, channel->pkt_size);
				if (channel->cb)
					channel->cb(qsdev, buf, count);
			}
		}
	}

	return IRQ_HANDLED;
}

static void qcom_smd_release_device(struct device *dev)
{
	struct qcom_smd_device *qsdev = container_of(dev, struct qcom_smd_device, dev);
	struct qcom_smd_channel *channel = container_of(qsdev, struct qcom_smd_channel, qsdev);

	kfree(channel);
}

static struct qcom_smd_channel *
qcom_smd_register_device(struct qcom_smem *smem, struct qcom_smem_child *child, int channel_idx, char *name)
{
	struct smd_shared_v2_word_access *info;
	static unsigned qcom_smd_dev_index;
	struct qcom_smd_channel *channel;
	struct qcom_smd_device *qsdev;
	size_t fifo_size;
	void *fifo_base;
	int ret;

	channel = kzalloc(sizeof(*channel), GFP_KERNEL);
	if (!channel) {
		dev_err(smem->dev, "failed to allocate qcom smd device\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_init(&channel->tx_lock);

	channel->smem = smem;
	channel->smd_irq = child->smd_irq;
	channel->signal_offset = child->signal_offset;
	channel->signal_bit = child->signal_bit;

	smem_find(smem, SMEM_SMD_BASE_ID + channel_idx, (void**)&info, NULL);

	channel->tx_info = &info->ch0;
	channel->rx_info = &info->ch1;

	smem_find(smem, SMEM_SMD_FIFO_BASE_ID + channel_idx, &fifo_base, &fifo_size);
	fifo_size /= 2;

	channel->tx_fifo = fifo_base;
	channel->rx_fifo = fifo_base + fifo_size;
	channel->fifo_size = fifo_size;

	channel->rx_info->tail = 0;
	channel->tx_info->head = 0;
	channel->tx_info->fBLOCKREADINTR = 0;
	channel->tx_info->fDSR = 0;
	channel->tx_info->fCTS = 0;
	channel->tx_info->fCD = 0;
	channel->tx_info->state = SMD_SS_OPENING;
	channel->tx_info->fSTATE = 1;

	channel->state = channel->tx_info->state;

	qsdev = &channel->qsdev;
	dev_set_name(&qsdev->dev, "qcom_smd%d", qcom_smd_dev_index++);
	qsdev->dev.parent = smem->dev;
	qsdev->dev.bus = &qcom_smd_bus;
	qsdev->dev.release = qcom_smd_release_device;
	qsdev->dev.of_node = of_get_next_child(child->of_node, NULL);
	ret = devm_request_irq(smem->dev, channel->smd_irq, qcom_smd_channel_intr,
			       IRQF_TRIGGER_RISING, name, qsdev);
	if (ret) {
		dev_err(smem->dev, "failed to request smd channel interrupt\n");
		return ERR_PTR(ret);
	}

	ret = device_register(&qsdev->dev);
	if (ret) {
		dev_err(smem->dev, "device_register failed: %d\n", ret);
		put_device(&qsdev->dev);
		return ERR_PTR(ret);
	}

	qcom_smd_signal_channel(channel);

	return channel;
}

static void qcom_smd_unregister_device(struct qcom_smem_child *child)
{
	struct device *dev = &child->channel->qsdev.dev;

	device_unregister(dev);
	put_device(dev);
}

static int qcom_smd_scan_channels(struct qcom_smem *smem)
{
	char name[SMD_CHANNEL_NAME_LEN] = {0};
	int cid;
	struct qcom_smem_child *child;
	int i;

	for (i = 0; i < smem->n_children; i++) {
		child = &smem->children[i];

		if (child->disabled)
			continue;

		cid = smem_find_smd_channel(smem, child->edge, name);
		if (cid >= 0 && !child->channel) {
			child->channel = qcom_smd_register_device(smem, child, cid, name);
		} else if (cid < 0 && child->channel) {
			qcom_smd_unregister_device(child);
			child->channel = NULL;
		}
	}

	return 0;
}

static int qcom_smem_probe(struct platform_device *pdev)
{
	struct qcom_smem_child *child;
	struct device_node *node;
	struct qcom_smem *smem;
	struct resource *res;
	const char *key;
	int count;
	int ret;
	int i = 0;

	smem = devm_kzalloc(&pdev->dev, sizeof(*smem), GFP_KERNEL);
	if (!smem) {
		dev_err(&pdev->dev, "failed to allocate struct smem\n");
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

	dev_set_drvdata(&pdev->dev, smem);

	count = of_get_child_count(pdev->dev.of_node);
	smem->children = devm_kcalloc(&pdev->dev, sizeof(struct qcom_smem_child), count, GFP_KERNEL);
	smem->n_children = count;
	for_each_child_of_node(pdev->dev.of_node, node) {
		child = &smem->children[i++];
		child->of_node = node;

		ret = of_get_child_count(node);
		if (ret > 1) {
			dev_err(&pdev->dev, "channels can only have a singel child\n");
			return -EINVAL;
		}
		child->disabled = !ret;

		child->smd_irq = irq_of_parse_and_map(node, 0);
		if (child->smd_irq < 0) {
			dev_err(&pdev->dev, "required smd interrupt missing\n");
			return -EINVAL;
		}

		child->smsm_irq = irq_of_parse_and_map(node, 1);
		if (child->smsm_irq < 0 && child->smsm_irq != -EINVAL) {
			dev_err(&pdev->dev, "failed to parse smsm interrupt\n");
			return -EINVAL;
		}

		key = "qcom,smd-edge";
		ret = of_property_read_u32(node, key, &child->edge);
		if (ret) {
			dev_err(&pdev->dev, "channel missing %s property\n", key);
			return -EINVAL;
		}

		key = "qcom,smd-irq-offset";
		ret = of_property_read_u32(node, key, &child->signal_offset);
		if (ret) {
			dev_err(&pdev->dev, "channel missing %s property\n", key);
			return -EINVAL;
		}

		key = "qcom,smd-irq-bit";
		ret = of_property_read_u32(node, key, &child->signal_bit);
		if (ret) {
			dev_err(&pdev->dev, "channel missing %s property\n", key);
			return -EINVAL;
		}
	}

	qcom_smd_scan_channels(smem);

	return 0;
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

static void qcom_smem_remove_smem_from_memory(void)
{
	struct device_node *node;
	struct resource res;

	node = of_find_matching_node(NULL, qcom_smem_of_match);
	if (node == NULL)
		return;

	if (of_address_to_resource(node, 1, &res))
		return;

	if (of_get_property(node, "reserve-memory", NULL))
		memblock_remove(res.start, resource_size(&res));
}

static int __init qcom_smem_init(void)
{
	int ret;

	qcom_smem_remove_smem_from_memory();

	ret = bus_register(&qcom_smd_bus);

	return platform_driver_register(&qcom_smem_driver);
}
arch_initcall(qcom_smem_init);

static void __exit qcom_smem_exit(void)
{
	platform_driver_unregister(&qcom_smem_driver);
}
module_exit(qcom_smem_exit)

MODULE_DESCRIPTION("Qualcomm Shared Memory Driver");
MODULE_LICENSE("GPLv2");
