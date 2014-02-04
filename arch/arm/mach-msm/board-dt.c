/* Copyright (c) 2010-2012,2013 The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/setup.h>

#include "common.h"

static const char * const msm_dt_match[] __initconst = {
	"qcom,msm8660-fluid",
	"qcom,msm8660-surf",
	"qcom,msm8960-cdp",
	"qcom,msm8960",
	NULL
};

static const char * const apq8074_dt_match[] __initconst = {
	"qcom,apq8074-dragonboard",
	NULL
};

static void msm_dt_init_meminfo(void)
{
	struct membank *bank;
	int i;

	for (i = 0; i < meminfo.nr_banks; i++) {
		bank = &meminfo.bank[i];
		if (bank->start == 0x80200000) {
			bank->start -= 0x200000;
			bank->size += 0x200000;
		}
	}
}

DT_MACHINE_START(MSM_DT, "Qualcomm MSM (Flattened Device Tree)")
	.smp = smp_ops(msm_smp_ops),
	.dt_compat = msm_dt_match,
	.init_meminfo = msm_dt_init_meminfo,
MACHINE_END

DT_MACHINE_START(APQ_DT, "Qualcomm MSM (Flattened Device Tree)")
	.dt_compat = apq8074_dt_match,
MACHINE_END
