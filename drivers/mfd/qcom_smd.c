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

#define SMD_CHANNEL_NAME_LEN 20

#define SMEM_CHANNEL_ALLOC_TBL	13
#define SMEM_SMD_BASE_ID	14
#define SMEM_SMSM_SHARED_STATE	85
#define SMEM_SMD_FIFO_BASE_ID	338

#define SMEM_NUM_SMD_STREAM_CHANNELS        64

#define SMSM_APPS_STATE 0

struct qcom_smd_edge;
struct smd_half_channel;
struct smd_half_channel_word_access;

struct qcom_smd_channel {
	struct qcom_smd *smd;
	struct qcom_smd_edge *edge;

	struct qcom_smd_device qsdev;

	char *name;

	int state;

	unsigned word_access_info;
	struct smd_half_channel *tx_info;
	struct smd_half_channel *rx_info;

	struct smd_half_channel_word_access *tx_info_word;
	struct smd_half_channel_word_access *rx_info_word;

	struct mutex tx_lock;

	void *tx_fifo;
	void *rx_fifo;
	int fifo_size;

	int (*cb)(struct qcom_smd_channel *, void *, size_t, void*);
	void *cb_data;

	int pkt_size;

	struct list_head list;
};

struct qcom_smd_edge {
	struct qcom_smd *smd;

	int smd_irq;
	int smsm_irq;
	int edge;

	int signal_bit;
	int signal_offset;

	struct list_head lookups;

	struct list_head used_channels;
	struct list_head free_channels;

	struct work_struct probe_work;
};

struct qcom_smd_lookup {
	struct qcom_smd_device qsdev;

	const char **ids;
	const char **names;
	struct qcom_smd_channel **channels;

	int num_channels;
	int found_channels;

	struct device_node *of_node;

	struct list_head list;

	const char *edge_name;
};

struct qcom_smd {
	struct device *dev;

	struct hwspinlock *hwlock;

	void __iomem *base;
	void __iomem *aux_base;
	void __iomem *signal_base;

	struct qcom_smd_edge *edges;
	unsigned n_edges;

	u32 *shared_state;
	size_t shared_state_size;

	DECLARE_BITMAP(allocated, SMEM_NUM_SMD_STREAM_CHANNELS);
};

struct smd_half_channel {
	unsigned state;
	unsigned char fDSR;
	unsigned char fCTS;
	unsigned char fCD;
	unsigned char fRI;
	unsigned char fHEAD;
	unsigned char fTAIL;
	unsigned char fSTATE;
	unsigned char fBLOCKREADINTR;
	unsigned tail;
	unsigned head;
};

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

struct smd_shared_v2 {
	struct smd_half_channel ch0;
	struct smd_half_channel ch1;
};

struct smd_shared_v2_word_access {
	struct smd_half_channel_word_access ch0;
	struct smd_half_channel_word_access ch1;
};

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

#define SMD_CHANNEL_EDGE(x)	((x) & 0xff)
#define SMD_CHANNEL_TYPE(x)	(((x) >> 8) & 0xf)

#define SMD_CHANNEL_TYPE_STREAM	0x1
#define SMD_CHANNEL_TYPE_PACKET	0x2

#define SMD_SS_CLOSED            0x00000000
#define SMD_SS_OPENING           0x00000001
#define SMD_SS_OPENED            0x00000002
#define SMD_SS_FLUSHING          0x00000003
#define SMD_SS_CLOSING           0x00000004
#define SMD_SS_RESET             0x00000005
#define SMD_SS_RESET_OPENING     0x00000006

/*
 * Resolves the address and size of smem_id.
 *
 * The shared memory remote spinlock must be held when calling this function.
 */
static int smem_find(struct qcom_smd *smd, int smem_id, void **ptr, size_t *size)
{
	struct smem_shared *shared = smd->base;
	struct smem_heap_entry *toc = shared->heap_toc;
	struct smem_heap_entry *entry;
	unsigned long flags;
	int ret;

	ret = hwspin_lock_timeout_irqsave(smd->hwlock, 100, &flags);
	if (ret)
		return ret;

	entry = &toc[smem_id];

	if (!entry->allocated) {
		ret = -EIO;
		goto out;
	}

	if (ptr != NULL) {
		if (entry->reserved)
			*ptr = smd->aux_base + entry->offset;
		else
			*ptr = smd->base + entry->offset;
	}
	if (size != NULL)
		*size = entry->size;

out:
	hwspin_unlock_irqrestore(smd->hwlock, &flags);

	return ret;
}

static int qcom_smd_dev_match(struct device *dev, struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static int qcom_smd_dev_probe(struct device *dev)
{
	struct qcom_smd_device *qsdev = container_of(dev, struct qcom_smd_device, dev);
	struct qcom_smd_driver *qsdrv = container_of(qsdev->dev.driver, struct qcom_smd_driver, driver);

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
	channel->cb_data = NULL;

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

static void qcom_smd_signal_channel(struct qcom_smd_channel *channel)
{
	struct qcom_smd *smd = channel->smd;
	struct qcom_smd_edge *edge = channel->edge;

	writel(BIT(edge->signal_bit), smd->signal_base + edge->signal_offset);
}

static void qcom_smsm_signal(struct qcom_smd *smd, struct qcom_smd_edge *edge)
{
	writel(BIT(edge->signal_bit + 2), smd->signal_base + edge->signal_offset);
}

static int qcom_smd_channel_reset(struct qcom_smd_channel *channel)
{
	if (channel->word_access_info) {
		channel->rx_info_word->tail = 0;
		channel->tx_info_word->head = 0;
		channel->tx_info_word->fBLOCKREADINTR = 0;
		channel->tx_info_word->fDSR = 0;
		channel->tx_info_word->fCTS = 0;
		channel->tx_info_word->fCD = 0;
		channel->tx_info_word->state = SMD_SS_OPENING;
		channel->tx_info_word->fSTATE = 1;

		return channel->tx_info_word->state;
	} else {
		channel->rx_info->tail = 0;
		channel->tx_info->head = 0;
		channel->tx_info->fBLOCKREADINTR = 0;
		channel->tx_info->fDSR = 0;
		channel->tx_info->fCTS = 0;
		channel->tx_info->fCD = 0;
		channel->tx_info->state = SMD_SS_OPENING;
		channel->tx_info->fSTATE = 1;

		return channel->tx_info->state;
	}
}

static int qcom_smd_channel_get_state(struct qcom_smd_channel *channel)
{
	if (channel->word_access_info)
		return channel->rx_info_word->state;
	else
		return channel->rx_info->state;
}

static int qcom_smd_channel_rx_head(struct qcom_smd_channel *channel)
{
	if (channel->word_access_info)
		return channel->rx_info_word->head;
	else
		return channel->rx_info->head;
}

static int qcom_smd_channel_rx_tail(struct qcom_smd_channel *channel)
{
	if (channel->word_access_info)
		return channel->rx_info_word->tail;
	else
		return channel->rx_info->tail;
}

static int qcom_smd_channel_tx_head(struct qcom_smd_channel *channel)
{
	if (channel->word_access_info)
		return channel->tx_info_word->head;
	else
		return channel->tx_info->head;
}

static int qcom_smd_channel_tx_tail(struct qcom_smd_channel *channel)
{
	if (channel->word_access_info)
		return channel->tx_info_word->tail;
	else
		return channel->tx_info->tail;
}

static void qcom_smd_channel_advance_rx(struct qcom_smd_channel *channel, unsigned amount)
{
	unsigned tail;

	tail = qcom_smd_channel_rx_tail(channel);
	tail = (tail + amount) & (channel->fifo_size - 1);

	if (channel->word_access_info)
		channel->rx_info_word->tail = tail;
	else
		channel->rx_info->tail = tail;
}

static void qcom_smd_channel_advance_tx(struct qcom_smd_channel *channel, unsigned amount)
{
	unsigned head;

	head = qcom_smd_channel_tx_head(channel);
	head = (head + amount) & (channel->fifo_size - 1);

	if (channel->word_access_info) {
		channel->tx_info_word->head = head;
		channel->tx_info_word->fHEAD = 1;
	} else {
		channel->tx_info->head = head;
		channel->tx_info->fHEAD = 1;
	}
}

static size_t qcom_smd_read_avail(struct qcom_smd_channel *channel)
{
	unsigned head;
	unsigned tail;

	head = qcom_smd_channel_rx_head(channel);
	tail = qcom_smd_channel_rx_tail(channel);

	return (head - tail) & (channel->fifo_size - 1);
}

static int qcom_smd_read_fifo(struct qcom_smd_channel *channel, void *buf, size_t count)
{
	unsigned tail;
	size_t len;

	tail = qcom_smd_channel_rx_tail(channel);

	len = min(count, channel->fifo_size - tail);
	if (len)
		memcpy(buf, channel->rx_fifo + tail, len);

	if (len != count)
		memcpy(buf + len, channel->rx_fifo, count - len);

	qcom_smd_channel_advance_rx(channel, count);

	return count;
}

static void qcom_smd_channel_set_state(struct qcom_smd_channel *channel, int state)
{
	if (channel->state == SMD_SS_OPENING && state == SMD_SS_OPENED) {
		if (channel->word_access_info) {
			channel->tx_info_word->fDSR = 1;
			channel->tx_info_word->fCTS = 1;
			channel->tx_info_word->fCD = 1;

			channel->tx_info_word->state = SMD_SS_OPENED;
			channel->tx_info_word->fSTATE = 1;
		} else {
			channel->tx_info->fDSR = 1;
			channel->tx_info->fCTS = 1;
			channel->tx_info->fCD = 1;

			channel->tx_info->state = SMD_SS_OPENED;
			channel->tx_info->fSTATE = 1;
		}

		qcom_smd_signal_channel(channel);
	} else {
		pr_err("Unkown state transition\n");
	}

	channel->state = state;
}

static void qcom_smd_channel_ack_flags(struct qcom_smd_channel *channel)
{
	if (channel->state != SMD_SS_OPENED)
		return;

	if (channel->word_access_info) {
		if (channel->rx_info_word->fHEAD)
			channel->rx_info_word->fHEAD = 0;

		if (channel->rx_info_word->fTAIL)
			channel->rx_info_word->fTAIL = 0;

		if (channel->rx_info_word->fSTATE)
			channel->rx_info_word->fSTATE = 0;

		qcom_smd_signal_channel(channel);
	}
}

static irqreturn_t qcom_smd_channel_intr(struct qcom_smd_channel *channel)
{
	u32 pkt_header[5];
	char buf[256];
	int avail;
	int count;
	int state;

	if (channel->state == SMD_SS_CLOSED)
		return IRQ_HANDLED;

	/* Handle state changes */
	state = qcom_smd_channel_get_state(channel);
	if (state != channel->state)
		qcom_smd_channel_set_state(channel, state);

	qcom_smd_channel_ack_flags(channel);

	/* Consume data */
	for (;;) {
		memset(buf, 0, sizeof(buf));
		avail = qcom_smd_read_avail(channel);

		if (channel->pkt_size == 0 && avail >= 20) {
			qcom_smd_read_fifo(channel, pkt_header, sizeof(pkt_header));
			channel->pkt_size = pkt_header[0];
		} else if (channel->pkt_size && avail == channel->pkt_size) {
			count = qcom_smd_read_fifo(channel, buf, channel->pkt_size);
			if (channel->cb)
				channel->cb(channel, buf, count, channel->cb_data);

			channel->pkt_size = 0;
		} else {
			break;
		}
	}

	return IRQ_HANDLED;
}

static size_t qcom_smd_write_avail(struct qcom_smd_channel *channel)
{
	unsigned head;
	unsigned tail;

	head = qcom_smd_channel_tx_head(channel);
	tail = qcom_smd_channel_tx_tail(channel);

	return channel->fifo_size - ((head - tail) & (channel->fifo_size - 1));
}

static int qcom_smd_write_fifo(struct qcom_smd_channel *channel, void *data, size_t count)
{
	unsigned head;
	size_t len;

	head = qcom_smd_channel_tx_head(channel);

	len = min(count, channel->fifo_size - head);
	if (len)
		memcpy(channel->tx_fifo + head, data, len);

	if (len != count)
		memcpy(channel->tx_fifo, data + len, count - len);

	qcom_smd_channel_advance_tx(channel, count);

	return count;
}

int qcom_smd_send(struct qcom_smd_channel *channel, void *data, int len)
{
	u32 hdr[5] = {len,};

	mutex_lock(&channel->tx_lock);

	while (qcom_smd_write_avail(channel) < sizeof(hdr) + len)
		msleep(1);

	qcom_smd_write_fifo(channel, hdr, sizeof(hdr));
	qcom_smd_write_fifo(channel, data, len);

	qcom_smd_signal_channel(channel);

	mutex_unlock(&channel->tx_lock);

	return 0;
}
EXPORT_SYMBOL(qcom_smd_send);

static struct qcom_smd_channel *qcom_smd_alloc_channel(struct qcom_smd *smd,
						       struct qcom_smd_edge *edge,
						       int channel_idx,
						       char *name)
{
	struct qcom_smd_channel *channel;
	size_t fifo_size;
	size_t info_size;
	void *fifo_base;
	void *info;

	channel = kzalloc(sizeof(*channel), GFP_KERNEL);
	if (!channel) {
		dev_err(smd->dev, "failed to allocate qcom smd device\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_init(&channel->tx_lock);

	channel->name = kstrdup(name, GFP_KERNEL);
	channel->smd = smd;
	channel->edge = edge;

	smem_find(smd, SMEM_SMD_BASE_ID + channel_idx, (void**)&info, &info_size);
	if (info_size == sizeof(struct smd_shared_v2_word_access)) {
		channel->tx_info_word = &((struct smd_shared_v2_word_access*)info)->ch0;
		channel->rx_info_word = &((struct smd_shared_v2_word_access*)info)->ch1;
		channel->word_access_info = true;
	} else if (info_size == sizeof(struct smd_shared_v2)) {
		channel->tx_info = &((struct smd_shared_v2*)info)->ch0;
		channel->rx_info = &((struct smd_shared_v2*)info)->ch1;
		channel->word_access_info = false;
	} else {
		dev_err(smd->dev, "channel info of size %d not supported\n", info_size);
		return ERR_PTR(-EINVAL);
	}

	smem_find(smd, SMEM_SMD_FIFO_BASE_ID + channel_idx, &fifo_base, &fifo_size);
	/* Each channel uses half the fifo */
	fifo_size /= 2;

	dev_dbg(smd->dev, "channel '%s' info-size: %d fifo-size: %d\n", name, info_size, fifo_size);

	channel->tx_fifo = fifo_base;
	channel->rx_fifo = fifo_base + fifo_size;
	channel->fifo_size = fifo_size;

	channel->state = qcom_smd_channel_reset(channel);

	return channel;
}

static void qcom_smd_release_device(struct device *dev)
{
	struct qcom_smd_device *qsdev = container_of(dev, struct qcom_smd_device, dev);

	kfree(qsdev);
}

static void qcom_smsm_try_open(struct qcom_smd *smd)
{
	size_t size;
	void *mem;
	int ret;

	if (smd->shared_state)
		return;

	ret = smem_find(smd, SMEM_SMSM_SHARED_STATE, &mem, &size);
	if (ret < 0)
		return;

	dev_err(smd->dev, "SMEM_SMSM_SHARED_STATE: %d, %d\n", ret, size);
	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, mem, size, true);

	smd->shared_state = mem;
	smd->shared_state_size = size;
}

int qcom_smsm_change_state(struct qcom_smd_device *qsdev, u32 clear_mask, u32 set_mask)
{
	struct qcom_smd *smd = qsdev->smd;
	u32 state;

	if (!smd->shared_state)
		return -EINVAL;

	dev_dbg(smd->dev, "SMSM_APPS_STATE clear 0x%x set 0x%x\n", clear_mask, set_mask);
	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smd->shared_state, smd->shared_state_size, true);

	state = readl(&smd->shared_state[SMSM_APPS_STATE]);
	state &= ~clear_mask;
	state |= set_mask;
	writel(state, &smd->shared_state[SMSM_APPS_STATE]);

	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smd->shared_state, smd->shared_state_size, true);

	qcom_smsm_signal(smd, &smd->edges[smd->n_edges-1]);

	return 0;
}
EXPORT_SYMBOL(qcom_smsm_change_state);

static int qcom_smd_register_device(struct qcom_smd *smd, struct qcom_smd_lookup *lookup)
{
	struct qcom_smd_device *qsdev;
	int ret;
	int i;

	qsdev = kzalloc(sizeof(*qsdev), GFP_KERNEL);
	if (!qsdev)
		return -ENOMEM;

	dev_set_name(&qsdev->dev, "%s.%s", lookup->edge_name, lookup->of_node->name);
	qsdev->dev.parent = smd->dev;
	qsdev->dev.bus = &qcom_smd_bus;
	qsdev->dev.release = qcom_smd_release_device;
	qsdev->dev.of_node = lookup->of_node;

	qsdev->smd = smd;
	qsdev->lookup = lookup;

	ret = device_register(&qsdev->dev);
	if (ret) {
		dev_err(smd->dev, "device_register failed: %d\n", ret);
		put_device(&qsdev->dev);
		return ret;
	}

	for (i = 0; i < lookup->num_channels; i++)
		qcom_smd_signal_channel(lookup->channels[i]);

	return 0;
}

struct qcom_smd_channel *qcom_smd_request_channel(struct qcom_smd_device *qsdev,
						  const char *name,
						  int (*callback)(struct qcom_smd_channel *, void *, size_t, void *),
						  void *dev)
{
	struct qcom_smd_lookup *lookup = qsdev->lookup;
	struct qcom_smd_channel *channel;
	int i;

	for (i = 0; i < lookup->num_channels; i++) {
		if (name && strcmp(lookup->names[i], name) == 0)
			continue;

		channel = lookup->channels[i];
		channel->cb = callback;
		channel->cb_data = dev;

		return channel;
	}

	return ERR_PTR(-ENOENT);
}

#if 0
static void qcom_smd_unregister_device(struct qcom_smd_edge *edge)
{
	struct device *dev = &edge->channel->qsdev.dev;

	device_unregister(dev);
	put_device(dev);
}
#endif

static void qcom_channel_scan_worker(struct work_struct *work)
{
	struct qcom_smd_channel *channel;
	struct qcom_smd_channel *tmp;
	struct qcom_smd_lookup *lookup;
	struct qcom_smd_edge *edge = container_of(work, struct qcom_smd_edge, probe_work);
	struct smd_alloc_elm *elm;
	struct qcom_smd *smd = edge->smd;
	int ret;
	int i;

	ret = smem_find(smd, SMEM_CHANNEL_ALLOC_TBL, (void**)&elm, NULL);
	if (ret < 0) {
		dev_err(smd->dev, "%d is not allocated\n", SMEM_CHANNEL_ALLOC_TBL);
		return;
	};

	for (i = 0; i < SMEM_NUM_SMD_STREAM_CHANNELS; i++) {
		if (test_bit(i, smd->allocated))
			continue;

		if (elm[i].ref_count == 0)
			continue;
		if (!elm[i].name[0])
			continue;

		if (SMD_CHANNEL_TYPE(elm[i].type) != SMD_CHANNEL_TYPE_PACKET)
			continue;

		if (SMD_CHANNEL_EDGE(elm[i].type) != edge->edge)
			continue;

		channel = qcom_smd_alloc_channel(smd, edge, elm[i].cid, elm[i].name);
		if (IS_ERR(channel))
			continue;

		list_add(&channel->list, &edge->free_channels);

		dev_dbg(smd->dev, "added %s\n", channel->name);
		set_bit(i, smd->allocated);
	}

	list_for_each_entry(lookup, &edge->lookups, list) {
		if (lookup->found_channels == lookup->num_channels)
			continue;

		list_for_each_entry_safe(channel, tmp, &edge->free_channels, list) {
			for (i = 0; i < lookup->num_channels; i++) {
				if (strcmp(channel->name, lookup->ids[i]) == 0) {
					list_del(&channel->list);
					lookup->channels[i] = channel;
					lookup->found_channels++;
				}
			}
		}

		if (lookup->found_channels != lookup->num_channels)
			continue;

		qcom_smd_register_device(smd, lookup);
	}

	qcom_smsm_try_open(smd);
}

static irqreturn_t qcom_smd_edge_intr(int irq, void *data)
{
	struct qcom_smd_lookup *lookup;
	struct qcom_smd_edge *edge = data;
	int i;

	list_for_each_entry(lookup, &edge->lookups, list) {
		for (i = 0; i < lookup->num_channels; i++) {
			if (!lookup->channels[i])
				continue;

			qcom_smd_channel_intr(lookup->channels[i]);
		}
	}

	schedule_work(&edge->probe_work);

	return IRQ_HANDLED;
}

static int qcom_smd_parse_edge(struct device *dev, struct device_node *node, struct qcom_smd_edge *edge)
{
	struct qcom_smd_lookup *lookup;
	struct device_node *child;
	const char *key;
	int count;
	int ret;
	int i;

	INIT_LIST_HEAD(&edge->lookups);
	INIT_LIST_HEAD(&edge->used_channels);
	INIT_LIST_HEAD(&edge->free_channels);

	INIT_WORK(&edge->probe_work, qcom_channel_scan_worker);

	for_each_available_child_of_node(node, child) {
		key = "qcom,smd-channels";
		count = of_property_count_strings(child, key);
		if (count < 0) {
			dev_err(dev, "error parsing %s\n", key);
			continue;
		}

		lookup = devm_kzalloc(dev, sizeof(*lookup), GFP_KERNEL);
		if (!lookup)
			return -ENOMEM;

		lookup->ids = devm_kcalloc(dev, count, sizeof(char*), GFP_KERNEL);
		if (!lookup->ids)
			return -ENOMEM;

		lookup->names = devm_kcalloc(dev, count, sizeof(char*), GFP_KERNEL);
		if (!lookup->names)
			return -ENOMEM;

		lookup->channels = devm_kcalloc(dev, count, sizeof(struct qcom_smd_channel*), GFP_KERNEL);
		if (!lookup->channels)
			return -ENOMEM;

		for (i = 0; i < count; i++) {
			key = "qcom,smd-channels";
			ret = of_property_read_string_index(child, key, i, &lookup->ids[i]);
			if (ret < 0) {
				dev_err(dev, "error parsing part %d of %s\n", i, key);
				continue;
			}

			key = "qcom,smd-channel-names";
			ret = of_property_read_string_index(child, key, i, &lookup->names[i]);
			if (ret < 0 && ret != -EINVAL) {
				dev_err(dev, "error parsing part %d of %s\n", i, key);
				continue;
			}
		}

		lookup->num_channels = count;
		lookup->of_node = child;
		lookup->edge_name = node->name;

		list_add(&lookup->list, &edge->lookups);
	}

	if (list_empty(&edge->lookups)) {
		dev_info(dev, "%s does not contain any children\n", node->name);
		return -ENOENT;
	}

	edge->smd_irq = irq_of_parse_and_map(node, 0);
	if (edge->smd_irq < 0) {
		dev_err(dev, "required smd interrupt missing\n");
		return -EINVAL;
	}

	ret = devm_request_irq(dev, edge->smd_irq,
			qcom_smd_edge_intr, IRQF_TRIGGER_RISING,
			node->name, edge);
	if (ret) {
		dev_err(dev, "failed to request smd irq\n");
		return ret;
	}

	edge->smsm_irq = irq_of_parse_and_map(node, 1);
	if (edge->smsm_irq < 0 && edge->smsm_irq != -EINVAL) {
		dev_err(dev, "failed to parse smsm interrupt\n");
		return -EINVAL;
	}

	key = "qcom,smd-edge";
	ret = of_property_read_u32(node, key, &edge->edge);
	if (ret) {
		dev_err(dev, "channel missing %s property\n", key);
		return -EINVAL;
	}

	key = "qcom,smd-irq-offset";
	ret = of_property_read_u32(node, key, &edge->signal_offset);
	if (ret) {
		dev_err(dev, "channel missing %s property\n", key);
		return -EINVAL;
	}

	key = "qcom,smd-irq-bit";
	ret = of_property_read_u32(node, key, &edge->signal_bit);
	if (ret) {
		dev_err(dev, "channel missing %s property\n", key);
		return -EINVAL;
	}

	return 0;
}

static int qcom_smd_probe(struct platform_device *pdev)
{
	struct qcom_smd_edge *edge;
	struct device_node *node;
	struct qcom_smd *smd;
	struct resource *res;
	int count;
	int ret;
	int i = 0;

	smd = devm_kzalloc(&pdev->dev, sizeof(*smd), GFP_KERNEL);
	if (!smd) {
		dev_err(&pdev->dev, "failed to allocate struct smd\n");
		return -ENOMEM;
	}
	smd->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smd->signal_base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!smd->signal_base)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	smd->base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!smd->base)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	smd->aux_base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!smd->aux_base)
		return -ENOMEM;

	smd->hwlock = of_hwspin_lock_request(pdev->dev.of_node, NULL);
	if (IS_ERR(smd->hwlock))
		return PTR_ERR(smd->hwlock);

	dev_set_drvdata(&pdev->dev, smd);

	count = of_get_child_count(pdev->dev.of_node);
	smd->edges = devm_kcalloc(&pdev->dev, sizeof(struct qcom_smd_edge), count, GFP_KERNEL);
	smd->n_edges = count;
	for_each_child_of_node(pdev->dev.of_node, node) {
		edge = &smd->edges[i++];
		edge->smd = smd;

		ret = qcom_smd_parse_edge(&pdev->dev, node, edge);
		if (ret)
			continue;

		schedule_work(&edge->probe_work);
	}

	return 0;
}

static const struct of_device_id qcom_smd_of_match[] = {
	{ .compatible = "qcom,smd" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smd_of_match);

static struct platform_driver qcom_smd_driver = {
	.probe          = qcom_smd_probe,
	.driver  = {
		.name  = "qcom_smd",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smd_of_match,
	},
};

static int __init qcom_smd_init(void)
{
	int ret;

	ret = bus_register(&qcom_smd_bus);

	return platform_driver_register(&qcom_smd_driver);
}
arch_initcall(qcom_smd_init);

static void __exit qcom_smd_exit(void)
{
	platform_driver_unregister(&qcom_smd_driver);
}
module_exit(qcom_smd_exit)

MODULE_DESCRIPTION("Qualcomm Shared Memory Driver");
MODULE_LICENSE("GPLv2");
