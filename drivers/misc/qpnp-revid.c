/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/regmap.h>
#include <linux/of_platform.h>

#define REVID_REVISION1	0x0
#define REVID_REVISION2	0x1
#define REVID_REVISION3	0x2
#define REVID_REVISION4	0x3
#define REVID_TYPE	0x4
#define REVID_SUBTYPE	0x5
#define REVID_STATUS1	0x8

#define QPNP_REVID_DEV_NAME "qcom,qpnp-revid"

static const char *const pmic_names[] = {
	"Unknown PMIC",
	"PM8941",
	"PM8841",
	"PM8019",
	"PM8226",
	"PM8110"
};

static struct of_device_id qpnp_revid_match_table[] = {
	{ .compatible = QPNP_REVID_DEV_NAME },
	{}
};

static u8 qpnp_read_byte(struct regmap *regmap, u16 addr)
{
	u8 val;
	int rc;

	rc = regmap_bulk_read(regmap, addr, &val, 1);
	if (rc) {
		pr_err("SPMI read failed rc=%d\n", rc);
		return 0;
	}
	return val;
}

#define PM8941_PERIPHERAL_SUBTYPE	0x01
static size_t build_pmic_string(char *buf, size_t n, int sid,
		u8 subtype, u8 rev1, u8 rev2, u8 rev3, u8 rev4)
{
	size_t pos = 0;
	/*
	 * In early versions of PM8941, the major revision number started
	 * incrementing from 0 (eg 0 = v1.0, 1 = v2.0).
	 * Increment the major revision number here if the chip is an early
	 * version of PM8941.
	 */
	if ((int)subtype == PM8941_PERIPHERAL_SUBTYPE && rev4 < 0x02)
		rev4++;

	pos += snprintf(buf + pos, n - pos, "PMIC@SID%d", sid);
	if (subtype >= ARRAY_SIZE(pmic_names) || subtype == 0)
		pos += snprintf(buf + pos, n - pos, ": %s (subtype: 0x%02X)",
				pmic_names[0], subtype);
	else
		pos += snprintf(buf + pos, n - pos, ": %s",
				pmic_names[subtype]);
	pos += snprintf(buf + pos, n - pos, " v%d.%d", rev4, rev3);
	if (rev2 || rev1)
		pos += snprintf(buf + pos, n - pos, ".%d", rev2);
	if (rev1)
		pos += snprintf(buf + pos, n - pos, ".%d", rev1);
	return pos;
}

#define PMIC_PERIPHERAL_TYPE		0x51
#define PMIC_STRING_MAXLENGTH		80
static int qpnp_revid_probe(struct platform_device *pdev)
{
	u8 rev1, rev2, rev3, rev4, pmic_type, pmic_subtype, pmic_status;
	u8 option1, option2, option3, option4;
	char pmic_string[PMIC_STRING_MAXLENGTH] = {'\0'};
	struct regmap *regmap;
	u32 base;
	int rc;

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap) {
		dev_err(&pdev->dev, "Unable to get regmap for REVID\n");
		return -EINVAL;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node, "reg", &base, 1);
	if (rc || base > 0xffff) {
		dev_err(&pdev->dev, "Invalid IO resources\n");
		return rc;
	}

	pmic_type = qpnp_read_byte(regmap, base + REVID_TYPE);
	if (pmic_type != PMIC_PERIPHERAL_TYPE) {
		dev_err(&pdev->dev, "Invalid REVID peripheral type: %02X\n", pmic_type);
		return -EINVAL;
	}

	rev1 = qpnp_read_byte(regmap, base + REVID_REVISION1);
	rev2 = qpnp_read_byte(regmap, base + REVID_REVISION2);
	rev3 = qpnp_read_byte(regmap, base + REVID_REVISION3);
	rev4 = qpnp_read_byte(regmap, base + REVID_REVISION4);

	pmic_subtype = qpnp_read_byte(regmap, base + REVID_SUBTYPE);
	pmic_status = qpnp_read_byte(regmap, base + REVID_STATUS1);

	option1 = pmic_status & 0x3;
	option2 = (pmic_status >> 2) & 0x3;
	option3 = (pmic_status >> 4) & 0x3;
	option4 = (pmic_status >> 6) & 0x3;

	build_pmic_string(pmic_string, PMIC_STRING_MAXLENGTH, 0,
			pmic_subtype, rev1, rev2, rev3, rev4);
	dev_info(&pdev->dev, "%s options: %d, %d, %d, %d\n",
			pmic_string, option1, option2, option3, option4);
	return 0;
}

static struct platform_driver qpnp_revid_driver = {
	.probe	= qpnp_revid_probe,
	.driver	= {
		.name		= QPNP_REVID_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= qpnp_revid_match_table,
	},
};

module_platform_driver(qpnp_revid_driver);

MODULE_DESCRIPTION("qpnp revid driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_REVID_DEV_NAME);
