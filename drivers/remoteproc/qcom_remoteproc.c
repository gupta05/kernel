/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/elf.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include "../../arch/arm/mach-msm/scm.h"

extern int scm_is_call_available(u32 svc_id, u32 cmd_id);

#define PAS_INIT_IMAGE_CMD      1
#define PAS_MEM_SETUP_CMD       2
#define PAS_AUTH_AND_RESET_CMD  5
#define PAS_SHUTDOWN_CMD        6
#define PAS_IS_SUPPORTED_CMD    7

struct qcom_rproc {
	struct device *dev;

	int pas_id;

	const char *name;
};

static int pas_supported(int id)
{
	u32 periph = id;
	u32 ret_val;
	int ret;

	ret = scm_is_call_available(SCM_SVC_PIL, PAS_IS_SUPPORTED_CMD);
	if (ret <= 0)
		return 0;

        ret = scm_call(SCM_SVC_PIL, PAS_IS_SUPPORTED_CMD,
		       &periph, sizeof(periph),
		       &ret_val, sizeof(ret_val));

	return ret ? : ret_val;
}

static int pas_init_image(int id, const char *metadata, size_t size)
{
	dma_addr_t mdata_phys;
	void *mdata_buf;
	u32 scm_ret;
	int ret;
	struct pas_init_image_req {
		u32     proc;
		u32     image_addr;
	} request;

	/* Make memory physically contiguous, 4K aligned and non-cacheable */
	mdata_buf = dma_alloc_coherent(NULL, size, &mdata_phys, GFP_KERNEL);
	if (!mdata_buf) {
		pr_err("Allocation for metadata failed.\n");
		return -ENOMEM;
	}

	memcpy(mdata_buf, metadata, size);

	request.proc = id;
	request.image_addr = mdata_phys;

	/* Flush metadata to ensure secure world doesn't read stale data */
	wmb();
#if 0
	__cpuc_flush_dcache_area(mdata_buf, size);
#endif
	outer_flush_range(request.image_addr, request.image_addr + size);

	ret = scm_call(SCM_SVC_PIL, PAS_INIT_IMAGE_CMD, &request,
			sizeof(request), &scm_ret, sizeof(scm_ret));

	dma_free_coherent(NULL, size, mdata_buf, mdata_phys);

	return ret ? : scm_ret;
}

int pas_mem_setup(int id, phys_addr_t start_addr, phys_addr_t size)
{
	u32 scm_ret;
	int ret;
	struct pas_init_image_req {
		u32     proc;
		u32     start_addr;
		u32     len;
	} request;

	request.proc = id;
	request.start_addr = start_addr;
	request.len = size;

	ret = scm_call(SCM_SVC_PIL, PAS_MEM_SETUP_CMD,
		       &request, sizeof(request),
		       &scm_ret, sizeof(scm_ret));

	return ret ? : scm_ret;
}

static int pas_auth_and_reset(int id)
{
	u32 proc = id;
	u32 scm_ret;
	int ret;

	ret = scm_call(SCM_SVC_PIL, PAS_AUTH_AND_RESET_CMD,
		       &proc, sizeof(proc),
		       &scm_ret, sizeof(scm_ret));

	return ret ? : scm_ret;
}

static int pas_shutdown(int id)
{
	u32 scm_ret;
	u32 proc = id;
	int ret;

	ret = scm_call(SCM_SVC_PIL, PAS_SHUTDOWN_CMD,
		       &proc, sizeof(proc),
		       &scm_ret, sizeof(scm_ret));

	return ret ? : scm_ret;
}

/**
 * struct pil_mdt - Representation of <name>.mdt file in memory
 * @hdr: ELF32 header
 * @phdr: ELF32 program headers
 */
struct pil_mdt {
	struct elf32_hdr hdr;
	struct elf32_phdr phdr[];
};

#define segment_is_hash(flag) (((flag) & (0x7 << 24)) == (0x2 << 24))

static int segment_is_loadable(const struct elf32_phdr *p)
{
	return (p->p_type == PT_LOAD) &&
	       !segment_is_hash(p->p_flags) &&
	       p->p_memsz;
}

static bool segment_is_relocatable(const struct elf32_phdr *p)
{
	return !!(p->p_flags & BIT(27));
}

static int pil_load_segment(struct qcom_rproc *qproc, const char *fw_name, const struct elf32_phdr *phdr)
{
	const struct firmware *fw;
	u8 __iomem *buf;
	int ret = 0;

	buf = ioremap(phdr->p_paddr, phdr->p_memsz);
	if (!buf) {
		dev_err(qproc->dev, "Failed to ioremap segment area\n");
		return -EBUSY;
	}
	memset(buf, 0, phdr->p_memsz);

	if (phdr->p_filesz) {
		ret = request_firmware(&fw, fw_name, qproc->dev);
		if (ret) {
			dev_err(qproc->dev, "Failed to load %s\n", fw_name);
			goto out;
		}

		memcpy(buf, fw->data, fw->size);

		release_firmware(fw);
	}

out:
	iounmap(buf);

	return ret;
}

static int qcom_rproc_boot(struct qcom_rproc *qproc)
{
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	const struct firmware *fw;
	const struct pil_mdt *mdt;
	phys_addr_t min_addr;
	phys_addr_t max_addr;
	char *fw_name;
	int ret;
	int i;

	fw_name = kasprintf(GFP_KERNEL, "%s.mdt", qproc->name);
	if (!fw_name)
		return -ENOMEM;

	ret = request_firmware(&fw, fw_name, qproc->dev);
	if (ret) {
		dev_err(qproc->dev, "Failed to locate %s\n", fw_name);
		return -EINVAL;
	}

	kfree(fw_name);

	if (fw->size < sizeof(*ehdr)) {
		dev_err(qproc->dev, "Not big enough to be an elf header\n");
		ret = -EIO;
		goto release_fw;
	}

	mdt = (const struct pil_mdt *)fw->data;
	ehdr = &mdt->hdr;

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(qproc->dev, "Not an elf header\n");
		ret = -EIO;
		goto release_fw;
	}

	if (ehdr->e_phnum == 0) {
		dev_err(qproc->dev, "No loadable segments\n");
		ret = -EIO;
		goto release_fw;
	}
	if (sizeof(struct elf32_phdr) * ehdr->e_phnum +
			sizeof(struct elf32_hdr) > fw->size) {
		dev_err(qproc->dev, "Program headers not within mdt\n");
		ret = -EIO;
		goto release_fw;
	}

	min_addr = (phys_addr_t)ULLONG_MAX;
	max_addr = 0;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		if (segment_is_relocatable(phdr)) {
			dev_err(qproc->dev, "Relocation unsupported\n");
			ret = -EINVAL;
			goto release_fw;
		}

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = ALIGN(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	dev_dbg(qproc->dev, "allocation range: 0x%x - 0x%x\n", min_addr, max_addr);

	if (memblock_is_region_memory(min_addr, max_addr - min_addr)) {
		dev_err(qproc->dev, "Image overlaps kernel memory\n");
		ret = -EINVAL;
		goto release_fw;
	}

	ret = pas_init_image(qproc->pas_id, fw->data, fw->size);
	if (ret) {
		dev_err(qproc->dev, "Invalid firmware metadata\n");
		goto release_fw;
	}

	ret = pas_mem_setup(qproc->pas_id, min_addr, max_addr - min_addr);
	if (ret) {
		dev_err(qproc->dev, "Unable to setup memory for image\n");
		goto release_fw;
	}

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		fw_name = kasprintf(GFP_KERNEL, "%s.b%02d", qproc->name, i);
		ret = pil_load_segment(qproc, fw_name, phdr);
		if (ret)
			goto release_fw;

		kfree(fw_name);
	}

	ret = pas_auth_and_reset(qproc->pas_id);
	if (ret)
		dev_err(qproc->dev, "Failed to authenticate image and release reset\n");

release_fw:
	release_firmware(fw);

	return 0;
}

static int qcom_rproc_shutdown(struct qcom_rproc *qproc)
{
	int ret;

	ret = pas_shutdown(qproc->pas_id);
	dev_err(qproc->dev, "%d\n", ret);

	return ret;
}

static ssize_t qcom_rproc_boot_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct qcom_rproc *qproc = dev_get_drvdata(dev);
	int ret;

	ret = qcom_rproc_boot(qproc);

	return ret ? : size;
}

static ssize_t qcom_rproc_shutdown_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct qcom_rproc *qproc = dev_get_drvdata(dev);
	int ret;

	ret = qcom_rproc_shutdown(qproc);

	return ret ? : size;
}

static const struct device_attribute qcom_rproc_attrs[] = {
	__ATTR(boot, S_IWUSR, 0, qcom_rproc_boot_store),
	__ATTR(shutdown, S_IWUSR, 0, qcom_rproc_shutdown_store),
};

static int qcom_rproc_probe(struct platform_device *pdev)
{
	struct qcom_rproc *qproc;
	struct regulator *vreg;
	struct clk *clk;
	char *key;
	int ret;
	int i;

	qproc = devm_kzalloc(&pdev->dev, sizeof(*qproc), GFP_KERNEL);
	if (!qproc)
		return -ENOMEM;
	qproc->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, qproc);

	key = "qcom,rproc-name";
	ret = of_property_read_string(pdev->dev.of_node, key, &qproc->name);
	if (ret) {
		dev_err(&pdev->dev, "Missing or incorrect %s\n", key);
		return -EINVAL;
	}

	key = "qcom,pas-id";
	ret = of_property_read_u32(pdev->dev.of_node, key, &qproc->pas_id);
	if (ret) {
		dev_err(&pdev->dev, "Missing or incorrect %s\n", key);
		return -EINVAL;
	}

	clk = devm_clk_get(&pdev->dev, "core");
	clk_prepare_enable(clk);

	clk = devm_clk_get(&pdev->dev, "bus");
	clk_prepare_enable(clk);

	clk = devm_clk_get(&pdev->dev, "iface");
	clk_prepare_enable(clk);

	vreg = devm_regulator_get(&pdev->dev, "pll");
	if (IS_ERR(vreg)) {
		dev_err(&pdev->dev, "Failed to retrieve regulator\n");
		return PTR_ERR(vreg);
	}

	if (!pas_supported(qproc->pas_id)) {
		dev_err(&pdev->dev, "PAS is not available for %d\n", qproc->pas_id);
		return -EIO;
	}

	ret = regulator_enable(vreg);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable regulator\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(qcom_rproc_attrs); i++) {
		ret = device_create_file(&pdev->dev, &qcom_rproc_attrs[i]);
		if (ret) {
			dev_err(&pdev->dev, "Unable to create sysfs file\n");
			return ret;
		}
	}

	dev_info(&pdev->dev, "Qualcomm Peripheral Image Loader\n");

	return 0;
}

static int qcom_rproc_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &qcom_rproc_attrs[0]);

	return 0;
}

static const struct of_device_id qcom_rproc_of_match[] = {
	{ .compatible = "qcom,rproc", },
	{ },
};

static struct platform_driver qcom_rproc_driver = {
	.probe = qcom_rproc_probe,
	.remove = qcom_rproc_remove,
	.driver = {
		.name = "qcom-rproc",
		.owner = THIS_MODULE,
		.of_match_table = qcom_rproc_of_match,
	},
};

module_platform_driver(qcom_rproc_driver);
