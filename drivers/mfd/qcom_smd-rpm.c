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

#define DEBUG

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/qcom_smd.h>
#include <linux/qcom_smd-rpm.h>

struct qcom_smd_rpm_resource;

struct qcom_smd_rpm {
	struct qcom_smd_device *dev;

	struct completion ack;
	unsigned ack_status;

	struct mutex lock;

	const struct qcom_smd_rpm_resource *resource_table;
	unsigned nresources;
};

struct qcom_smd_rpm_resource {
	unsigned resource_type;
	unsigned resource_id;
};

enum msm_rpm_set {
	MSM_RPM_CTX_ACTIVE_SET,
	MSM_RPM_CTX_SLEEP_SET,
};

enum {
	MSM_RPM_MSG_REQUEST_TYPE = 0,
	MSM_RPM_MSG_TYPE_NR,
};

enum rpm_regulator_param_index {
	RPM_REGULATOR_PARAM_ENABLE,
	RPM_REGULATOR_PARAM_VOLTAGE,
	RPM_REGULATOR_PARAM_CURRENT,
	RPM_REGULATOR_PARAM_MODE_LDO,
	RPM_REGULATOR_PARAM_MODE_SMPS,
	RPM_REGULATOR_PARAM_PIN_CTRL_ENABLE,
	RPM_REGULATOR_PARAM_PIN_CTRL_MODE,
	RPM_REGULATOR_PARAM_FREQUENCY,
	RPM_REGULATOR_PARAM_HEAD_ROOM,
	RPM_REGULATOR_PARAM_QUIET_MODE,
	RPM_REGULATOR_PARAM_FREQ_REASON,
	RPM_REGULATOR_PARAM_CORNER,
	RPM_REGULATOR_PARAM_BYPASS,
	RPM_REGULATOR_PARAM_FLOOR_CORNER,
	RPM_REGULATOR_PARAM_MAX,
};

struct rpm_request_header {
	uint32_t service_type;
	uint32_t request_len;
};

struct rpm_message_header {
	uint32_t msg_id;
	enum msm_rpm_set set;
	uint32_t resource_type;
	uint32_t resource_id;
	uint32_t data_len;
};

#define MSM_RPM_REQUEST_SERVICE_TYPE 0x716572 /* 'req\0' */

#define RESOURCE_TYPE_SMPA 0x61706d73 /* 'smpa' */
#define RESOURCE_TYPE_SMPB 0x62706d73 /* 'smpb' */
#define RESOURCE_TYPE_LDOA 0x616f646c /* 'ldoa' */

static const struct qcom_smd_rpm_resource rpm8x74_resource_table[] = {
	[MSM_RPM_PM8841_S1] = { .resource_type = RESOURCE_TYPE_SMPB, .resource_id = 1 },
	[MSM_RPM_PM8841_S2] = { .resource_type = RESOURCE_TYPE_SMPB, .resource_id = 2 },
	[MSM_RPM_PM8941_S1] = { .resource_type = RESOURCE_TYPE_SMPA, .resource_id = 1 },
	[MSM_RPM_PM8941_S2] = { .resource_type = RESOURCE_TYPE_SMPA, .resource_id = 2 },
	[MSM_RPM_PM8941_S3] = { .resource_type = RESOURCE_TYPE_SMPA, .resource_id = 3 },
	[MSM_RPM_PM8941_L3] = { .resource_type = RESOURCE_TYPE_LDOA, .resource_id = 3 },
	[MSM_RPM_PM8941_L6] = { .resource_type = RESOURCE_TYPE_LDOA, .resource_id = 6 },
	[MSM_RPM_PM8941_L9] = { .resource_type = RESOURCE_TYPE_LDOA, .resource_id = 9 },
	[MSM_RPM_PM8941_L11] = { .resource_type = RESOURCE_TYPE_LDOA, .resource_id = 11 },
	[MSM_RPM_PM8941_L19] = { .resource_type = RESOURCE_TYPE_LDOA, .resource_id = 19 },
	[MSM_RPM_PM8941_L20] = { .resource_type = RESOURCE_TYPE_LDOA, .resource_id = 20 },
	[MSM_RPM_PM8941_L22] = { .resource_type = RESOURCE_TYPE_LDOA, .resource_id = 22 },
};

static const struct qcom_smd_rpm rpm8x74_template = {
	.resource_table = rpm8x74_resource_table,
	.nresources = ARRAY_SIZE(rpm8x74_resource_table),
};

static const struct of_device_id qcom_smd_rpm_of_match[] = {
	{ .compatible = "qcom,rpm8x74-smd", .data = &rpm8x74_template },
};
MODULE_DEVICE_TABLE(of, qcom_smd_rpm_of_match);

#define MAX_RESOURCE_PARAM_LEN 84

int qcom_rpm_smd_write(const struct device *dev, int resource, void *buf, size_t count)
{
	const struct qcom_smd_rpm_resource *res;
	struct qcom_smd_rpm *rpm = dev_get_drvdata(dev);
	static unsigned msg_id = 1;
	int ret = 0;

	struct {
		struct rpm_request_header req;
		struct rpm_message_header msg;
		u8 payload[MAX_RESOURCE_PARAM_LEN];
	} pkt;

	if (WARN_ON(resource < 0 || resource >= rpm->nresources))
		return -EINVAL;

	res = &rpm->resource_table[resource];
	if (WARN_ON(!res->resource_id || !res->resource_type))
		return -EINVAL;

	mutex_lock(&rpm->lock);

	pkt.req.service_type = MSM_RPM_REQUEST_SERVICE_TYPE;
	pkt.req.request_len = sizeof(struct rpm_message_header) + count;

	pkt.msg.msg_id = msg_id++;
	pkt.msg.set = MSM_RPM_CTX_ACTIVE_SET;
	pkt.msg.resource_type = res->resource_type;
	pkt.msg.resource_id = res->resource_id;
	pkt.msg.data_len = count;

	memcpy(pkt.payload, buf, count);

#if 0
	print_hex_dump(KERN_DEBUG, "rpm-write", DUMP_PREFIX_OFFSET, 16, 4, &pkt, pkt.req.request_len, true);
#endif
	qcom_smd_send(rpm->dev, &pkt, sizeof(struct rpm_request_header) + pkt.req.request_len);

	wait_for_completion(&rpm->ack);

	ret = rpm->ack_status;
	mutex_unlock(&rpm->lock);

	return ret;
}
EXPORT_SYMBOL(qcom_rpm_smd_write);

struct qcom_rpm_ack_hdr {
	u32 req;
	u32 req_len;
};

struct qcom_rpm_ack_msg {
	u32 rsc_id;
	u32 msg_len;
	u32 id_ack;
};

struct qcom_rpm_ack_payload {
	u32 unknown1;
	u32 unknown2;
	u8 message[0];
};

#define RPM_ACK_PAYLOAD_ERR 0x727265 /* 'err\0' */
#define RPM_ACK_PAYLOAD_INV_RSC "resource does not exist"

static int qcom_smd_rpm_callback(struct qcom_smd_device *sdev, void *buf, size_t count)
{
	struct qcom_smd_rpm *rpm = dev_get_drvdata(&sdev->dev);
	struct qcom_rpm_ack_hdr *hdr = buf;
	struct qcom_rpm_ack_msg *msg = (struct qcom_rpm_ack_msg*)(hdr + 1);
	struct qcom_rpm_ack_payload *payload = (struct qcom_rpm_ack_payload*)(msg + 1);

	if (hdr->req_len == sizeof(struct qcom_rpm_ack_msg)) {
		rpm->ack_status = 0;
		goto out;
	}

	/* XXX: test this error handling */
	print_hex_dump(KERN_DEBUG, "rpm-callback", DUMP_PREFIX_OFFSET, 16, 4, buf, count, true);

	BUG_ON(payload->unknown1 == RPM_ACK_PAYLOAD_ERR);

	if (!memcmp(payload->message, RPM_ACK_PAYLOAD_INV_RSC, sizeof(RPM_ACK_PAYLOAD_INV_RSC) - 1)) {
		dev_err(&sdev->dev, "RPM NACK: Unsupported resource: 0%x\n", msg->rsc_id);
		rpm->ack_status = -EINVAL;
	} else {
		dev_err(&sdev->dev, "RPM NACK: Invalid header\n");
		rpm->ack_status = -ENODEV;
	}

out:
	complete(&rpm->ack);
	return 0;
}

static int qcom_smd_rpm_probe(struct qcom_smd_device *sdev)
{
	const struct qcom_smd_rpm *template;
	const struct of_device_id *match;
	struct qcom_smd_rpm *rpm;

	rpm = devm_kzalloc(&sdev->dev, sizeof(*rpm), GFP_KERNEL);
	if (!rpm) {
		dev_err(&sdev->dev, "Can't allocate msm_rpm\n");
		return -ENOMEM;
	}
	rpm->dev = sdev;
	mutex_init(&rpm->lock);
	init_completion(&rpm->ack);

	match = of_match_device(qcom_smd_rpm_of_match, &sdev->dev);
	template = match->data;
	rpm->resource_table = template->resource_table;
	rpm->nresources = template->nresources;

	dev_set_drvdata(&sdev->dev, rpm);

	return of_platform_populate(sdev->dev.of_node, NULL, NULL, &sdev->dev);
}

static struct qcom_smd_driver qcom_smd_rpm_driver = {
	.probe = qcom_smd_rpm_probe,
	.callback = qcom_smd_rpm_callback,
	.driver  = {
		.name  = "qcom_smd_rpm",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smd_rpm_of_match,
	},
};

static int __init qcom_smd_rpm_init(void)
{
        return register_qcom_smd_driver(&qcom_smd_rpm_driver);
}
module_init(qcom_smd_rpm_init);

MODULE_DESCRIPTION("Qualcomm SMD backed RPM driver");
MODULE_LICENSE("GPLv2");

