/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2015 Sony Mobile Communications Inc
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
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/elf.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>

#include "remoteproc_internal.h"

#define segment_is_hash(flag) (((flag) & (0x7 << 24)) == (0x2 << 24))

static int segment_is_loadable(const struct elf32_phdr *p)
{
	return (p->p_type == PT_LOAD) &&
	       !segment_is_hash(p->p_flags) &&
	       p->p_memsz;
}

/**
 * rproc_mdt_sanity_check() - sanity check mdt firmware header
 * @rproc: the remote processor handle
 * @fw: the mdt header firmware image
 *
 * Returns 0 for a valid header, -EINVAL otherwise.
 */
int qcom_mdt_sanity_check(struct rproc *rproc,
			  const struct firmware *fw)
{
	struct elf32_hdr *ehdr;

	if (!fw) {
		dev_err(&rproc->dev, "failed to load %s\n", rproc->name);
		return -EINVAL;
	}

	if (fw->size < sizeof(struct elf32_hdr)) {
		dev_err(&rproc->dev, "image is too small\n");
		return -EINVAL;
	}

	ehdr = (struct elf32_hdr *)fw->data;
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(&rproc->dev, "image is corrupted (bad magic)\n");
		return -EINVAL;
	}

	if (ehdr->e_phnum == 0) {
		dev_err(&rproc->dev, "no loadable segments\n");
		return -EINVAL;
	}

	if (sizeof(struct elf32_phdr) * ehdr->e_phnum +
	    sizeof(struct elf32_hdr) > fw->size) {
		dev_err(&rproc->dev, "firmware size is too small\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_mdt_sanity_check);

/**
 * qcom_mdt_find_rsc_table() - provide dummy resource table for remoteproc
 * @rproc:	remoteproc handle
 * @fw:		firmware header
 * @tablesz:	outgoing size of the table
 *
 * Returns a dummy table.
 */
struct resource_table *qcom_mdt_find_rsc_table(struct rproc *rproc,
					       const struct firmware *fw,
					       int *tablesz)
{
	static struct resource_table table = { .ver = 1, };

	*tablesz = sizeof(table);
	return &table;
}
EXPORT_SYMBOL_GPL(qcom_mdt_find_rsc_table);

static int qproc_load_segment(struct rproc *rproc,
			      const char *fw_name,
			      const struct elf32_phdr *phdr)
{
	const struct firmware *fw;
	void *ptr;
	int ret = 0;

	ptr = ioremap(phdr->p_paddr, phdr->p_memsz);
	if (!ptr) {
		dev_err(&rproc->dev,
			"failed to ioremap segment area (0x%x+0x%x)\n",
			phdr->p_paddr,
			phdr->p_memsz);
		return -EBUSY;
	}

	if (phdr->p_filesz) {
		ret = request_firmware(&fw, fw_name, &rproc->dev);
		if (ret) {
			dev_err(&rproc->dev, "failed to load %s\n", fw_name);
			goto out;
		}

		memcpy(ptr, fw->data, fw->size);

		release_firmware(fw);
	}

	if (phdr->p_memsz > phdr->p_filesz)
		memset(ptr + phdr->p_filesz, 0, phdr->p_memsz - phdr->p_filesz);

out:
	iounmap(ptr);
	return ret;
}

/**
 * qcom_mdt_load() - load the firmware which header is defined in fw
 * @rproc:	rproc handle
 * @pas_id:	PAS identifier to load this firmware into
 * @fw:		frimware object for the header
 *
 * Returns 0 on success, negative errno otherwise.
 */
int qcom_mdt_load(struct rproc *rproc,
		  unsigned pas_id,
		  const struct firmware *fw)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_hdr *ehdr;
	phys_addr_t min_addr = (phys_addr_t)ULLONG_MAX;
	phys_addr_t max_addr = 0;
	unsigned fw_name_len;
	char *fw_name;
	int ret;
	int i;

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf_phdr *)(ehdr + 1);

	for (i = 0; i < ehdr->e_phnum; i++) {
		if (!segment_is_loadable(&phdrs[i]))
			continue;

		if (phdrs[i].p_paddr < min_addr)
			min_addr = phdrs[i].p_paddr;

		if (phdrs[i].p_paddr + phdrs[i].p_memsz > max_addr)
			max_addr = round_up(phdrs[i].p_paddr + phdrs[i].p_memsz, SZ_4K);
	}

	ret = qcom_scm_pas_init_image(pas_id, fw->data, fw->size);
	if (ret) {
		dev_err(&rproc->dev, "Invalid firmware metadata\n");
		return -EINVAL;
	}

	ret = qcom_scm_pas_mem_setup(pas_id, min_addr, max_addr - min_addr);
	if (ret) {
		dev_err(&rproc->dev, "unable to setup memory for image\n");
		return -EINVAL;
	}

	fw_name = kstrdup(rproc->firmware, GFP_KERNEL);
	if (!fw_name)
		return -ENOMEM;

	fw_name_len = strlen(fw_name);
	if (fw_name_len <= 4)
		return -EINVAL;

	for (i = 0; i < ehdr->e_phnum; i++) {
		if (!segment_is_loadable(&phdrs[i]))
			continue;

		sprintf(fw_name + fw_name_len - 3, "b%02d", i);
		ret = qproc_load_segment(rproc, fw_name, &phdrs[i]);
		if (ret)
			break;
	}

	kfree(fw_name);

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_mdt_load);
