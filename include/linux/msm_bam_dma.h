/*
 * msm_bam_dma.h - MSM BAM DMA engine Driver
 *
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#ifndef __MSM_BAM_DMA_H__
#define __MSM_BAM_DMA_H__

#include <linux/dmaengine.h>

struct bam_dma_slave_config {
	struct dma_slave_config slave;

	/* BAM specific slave config starts here */
	u16 desc_threshold;
};

#endif /* __MSM_BAM_DMA_H__ */
