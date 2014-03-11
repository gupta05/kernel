/*
 *  HCI_SMD (HCI Shared Memory Driver) is Qualcomm's Shared memory driver
 *  for the BT HCI protocol.
 *
 *  Copyright (c) 2000-2001, 2011-2012 The Linux Foundation. All rights reserved.
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2004-2006  Marcel Holtmann <marcel@holtmann.org>
 *
 *  This file is based on drivers/bluetooth/hci_vhci.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/semaphore.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/qcom_smd.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>

#define EVENT_CHANNEL           "APPS_RIVA_BT_CMD"
#define DATA_CHANNEL            "APPS_RIVA_BT_ACL"

struct hci_smd {
	struct device *dev;

	struct qcom_smd_channel *data_channel;
	struct qcom_smd_channel *event_channel;
};

static int hci_smd_open(struct hci_dev *hdev)
{
	set_bit(HCI_RUNNING, &hdev->flags);
	return 0;
}

static int hci_smd_close(struct hci_dev *hdev)
{
	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		return 0;
	else
		return -EPERM;
}

static int hci_smd_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_smd *hs = hci_get_drvdata(hdev);
	int len;
	int ret = 0;

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		len = qcom_smd_send(hs->event_channel, skb->data, skb->len);
		if (len < 0) {
			dev_err(hs->dev, "Failed to write Command %d", len);
			ret = -ENODEV;
		}
		break;
	case HCI_ACLDATA_PKT:
	case HCI_SCODATA_PKT:
		len = qcom_smd_send(hs->data_channel, skb->data, skb->len);
		if (len < 0) {
			dev_err(hs->dev, "Failed to write Data %d", len);
			ret = -ENODEV;
		}
		break;
	default:
		dev_err(hs->dev, "Uknown packet type");
		ret = -ENODEV;
		break;
	}

	kfree_skb(skb);
	return ret;
}

static int hci_smd_callback(struct qcom_smd_channel *channel, void *buf, size_t count, void *_hs)
{
	struct hci_smd *hs = _hs;

	dev_err(hs->dev, "hci_smd_callback!\n");

	return -EBUSY;
}

static int hci_smd_probe(struct qcom_smd_device *sdev)
{
	struct hci_dev *hdev;
	struct hci_smd *hs;

	hs = devm_kzalloc(&sdev->dev, sizeof(*hs), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	hs->event_channel = qcom_smd_request_channel(sdev, "event", hci_smd_callback, hs);
	if (IS_ERR(hs->event_channel))
		return PTR_ERR(hs->event_channel);
	hs->data_channel = qcom_smd_request_channel(sdev, "data", hci_smd_callback, hs);
	if (IS_ERR(hs->data_channel))
		return PTR_ERR(hs->data_channel);

	/* Initialize and register HCI device */
	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hdev->bus = HCI_SMD;
	hdev->open  = hci_smd_open;
	hdev->close = hci_smd_close;
	hdev->send  = hci_smd_send_frame;

	if (hci_register_dev(hdev) < 0) {
		dev_err(&sdev->dev, "Can't register HCI device");
		hci_free_dev(hdev);
		return -ENODEV;
	}

	hci_set_drvdata(hdev, hs);

	dev_info(&sdev->dev, "Qualcomm HCI SMD driver\n");

	return 0;
}

static const struct of_device_id hci_smd_of_match[] = {
	{ .compatible = "qcom,hci-smd" },
	{}
};
MODULE_DEVICE_TABLE(of, hci_smd_of_match);

static struct qcom_smd_driver hci_smd_driver = {
	.probe = hci_smd_probe,
	.driver  = {
		.name  = "hci_smd",
		.owner = THIS_MODULE,
		.of_match_table = hci_smd_of_match,
	},
};

static int __init hci_smd_init(void)
{
        return register_qcom_smd_driver(&hci_smd_driver);
}
module_init(hci_smd_init);

MODULE_DESCRIPTION("Qualcomm HCI SMD");
MODULE_LICENSE("GPLv2");
