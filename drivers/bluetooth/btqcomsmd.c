/*
 * Copyright (c) 2015, Sony Mobile Communications Inc.
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
#include <linux/slab.h>
#include <linux/soc/qcom/smd.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>
#include "btqca.h"

struct btqcomsmd {
	struct hci_dev *hdev;

	struct qcom_smd_channel *acl_channel;
	struct qcom_smd_channel *cmd_channel;
};

static int btqcomsmd_recv(struct hci_dev *hdev, unsigned type, const void *data,
			  size_t count)
{
	struct sk_buff *skb;

	/* Use GFP_ATOMIC as we're in IRQ context */
	skb = bt_skb_alloc(count, GFP_ATOMIC);
	if (!skb) {
		hdev->stat.err_rx++;
		return -ENOMEM;
	}

	bt_cb(skb)->pkt_type = type;

	/* Use io accessor as data might be ioremapped */
	memcpy_fromio(skb_put(skb, count), data, count);

	return hci_recv_frame(hdev, skb);
}

static int btqcomsmd_acl_callback(struct qcom_smd_device *qsdev,
				  const void *data,
				  size_t count)
{
	struct btqcomsmd *btq = dev_get_drvdata(&qsdev->dev);

	btq->hdev->stat.byte_rx += count;
	return btqcomsmd_recv(btq->hdev, HCI_ACLDATA_PKT, data, count);
}

static int btqcomsmd_cmd_callback(struct qcom_smd_device *qsdev,
				  const void *data,
				  size_t count)
{
	struct btqcomsmd *btq = dev_get_drvdata(&qsdev->dev);

	return btqcomsmd_recv(btq->hdev, HCI_EVENT_PKT, data, count);
}

static int btqcomsmd_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btqcomsmd *btq = hci_get_drvdata(hdev);
	int ret;

	switch (bt_cb(skb)->pkt_type) {
	case HCI_ACLDATA_PKT:
		ret = qcom_smd_send(btq->acl_channel, skb->data, skb->len);
		hdev->stat.acl_tx++;
		hdev->stat.byte_tx += skb->len;
		break;
	case HCI_COMMAND_PKT:
		ret = qcom_smd_send(btq->cmd_channel, skb->data, skb->len);
		hdev->stat.cmd_tx++;
		break;
	default:
		ret = -EILSEQ;
		break;
	}

	return ret;
}

static int btqcomsmd_open(struct hci_dev *hdev)
{
	return 0;
}

static int btqcomsmd_close(struct hci_dev *hdev)
{
	return 0;
}

static int btqcomsmd_probe(struct qcom_smd_device *sdev)
{
	struct qcom_smd_channel *acl;
	struct btqcomsmd *btq;
	struct hci_dev *hdev;
	int ret;

	acl = qcom_smd_open_channel(sdev, "APPS_RIVA_BT_ACL",
				    btqcomsmd_acl_callback);
	if (IS_ERR(acl))
		return PTR_ERR(acl);

	btq = devm_kzalloc(&sdev->dev, sizeof(*btq), GFP_KERNEL);
	if (!btq)
		return -ENOMEM;

	btq->acl_channel = acl;
	btq->cmd_channel = sdev->channel;

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hci_set_drvdata(hdev, btq);
	btq->hdev = hdev;
	SET_HCIDEV_DEV(hdev, &sdev->dev);

	hdev->bus = HCI_SMD;
	hdev->open = btqcomsmd_open;
	hdev->close = btqcomsmd_close;
	hdev->send = btqcomsmd_send;
	hdev->set_bdaddr = qca_set_bdaddr_rome;

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		hci_free_dev(hdev);
		return ret;
	}

	dev_set_drvdata(&sdev->dev, btq);

	return 0;
}

static void btqcomsmd_remove(struct qcom_smd_device *sdev)
{
	struct hci_dev *hdev = dev_get_drvdata(&sdev->dev);;

	dev_set_drvdata(&sdev->dev, NULL);

	hci_unregister_dev(hdev);
	hci_free_dev(hdev);
}

static const struct qcom_smd_id btqcomsmd_match[] = {
	{ .name = "APPS_RIVA_BT_CMD" },
	{}
};

static struct qcom_smd_driver btqcomsmd_cmd_driver = {
	.probe = btqcomsmd_probe,
	.remove = btqcomsmd_remove,
	.callback = btqcomsmd_cmd_callback,
	.smd_match_table = btqcomsmd_match,
	.driver  = {
		.name  = "btqcomsmd",
		.owner = THIS_MODULE,
	},
};

module_qcom_smd_driver(btqcomsmd_cmd_driver);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm SMD HCI driver");
MODULE_LICENSE("GPL v2");
