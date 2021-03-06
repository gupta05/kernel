/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/io.h>
#include <linux/errno.h>
#include <linux/qcom_scm.h>

/**
 * qcom_scm_set_cold_boot_addr() - Set the cold boot address for cpus
 * @entry: Entry point function for the cpus
 * @cpus: The cpumask of cpus that will use the entry point
 *
 * Set the cold boot address of the cpus. Any cpu outside the supported
 * range would be removed from the cpu present mask.
 */
int __qcom_scm_set_cold_boot_addr(void *entry, const cpumask_t *cpus)
{
	return -ENOTSUPP;
}

/**
 * qcom_scm_set_warm_boot_addr() - Set the warm boot address for cpus
 * @entry: Entry point function for the cpus
 * @cpus: The cpumask of cpus that will use the entry point
 *
 * Set the Linux entry point for the SCM to transfer control to when coming
 * out of a power down. CPU power down may be executed on cpuidle or hotplug.
 */
int __qcom_scm_set_warm_boot_addr(void *entry, const cpumask_t *cpus)
{
	return -ENOTSUPP;
}

/**
 * qcom_scm_cpu_power_down() - Power down the cpu
 * @flags - Flags to flush cache
 *
 * This is an end point to power down cpu. If there was a pending interrupt,
 * the control would return from this function, otherwise, the cpu jumps to the
 * warm boot entry point set for this cpu upon reset.
 */
void __qcom_scm_cpu_power_down(u32 flags)
{
}

int __qcom_scm_is_call_available(u32 svc_id, u32 cmd_id)
{
	return -ENOTSUPP;
}

int __qcom_scm_hdcp_req(struct qcom_scm_hdcp_req *req, u32 req_cnt, u32 *resp)
{
	return -ENOTSUPP;
}

int __qcom_scm_restore_sec_config(u32 sec_id, u32 ctx_bank_num)
{
	return -ENOTSUPP;
}

int __qcom_scm_ocmem_lock(uint32_t id, uint32_t offset, uint32_t size,
		uint32_t mode)
{
	return -ENOTSUPP;
}

int __qcom_scm_ocmem_unlock(uint32_t id, uint32_t offset, uint32_t size)
{
	return -ENOTSUPP;
}

bool __qcom_scm_pas_supported(u32 peripheral)
{
	return false;
}

int __qcom_scm_pas_init_image(u32 peripheral, dma_addr_t metadata_phys)
{
	return -ENOTSUPP;
}

int __qcom_scm_pas_mem_setup(u32 peripheral, phys_addr_t addr, phys_addr_t size)
{
	return -ENOTSUPP;
}

int __qcom_scm_pas_auth_and_reset(u32 peripheral)
{
	return -ENOTSUPP;
}

int __qcom_scm_pas_shutdown(u32 peripheral)
{
	return -ENOTSUPP;
}
