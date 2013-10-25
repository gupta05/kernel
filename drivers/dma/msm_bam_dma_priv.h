/*
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
#ifndef __MSM_BAM_DMA_PRIV_H__
#define __MSM_BAM_DMA_PRIV_H__

#include <linux/dmaengine.h>

enum bam_channel_mode {
	BAM_PIPE_MODE_BAM2BAM = 0,	/* BAM to BAM aka device to device */
	BAM_PIPE_MODE_SYSTEM,		/* BAM to/from System Memory */
};

enum bam_channel_dir {
	BAM_PIPE_CONSUMER = 0,	/* channel reads from data-fifo or memory */
	BAM_PIPE_PRODUCER,	/* channel writes to data-fifo or memory */
};

struct bam_desc_hw {
	u32 addr;		/* Buffer physical address */
	u32 size:16;		/* Buffer size in bytes */
	u32 flags:16;
};

#define DESC_FLAG_INT	(1<<15)
#define DESC_FLAG_EOT	(1<<14)
#define DESC_FLAG_EOB	(1<<13)

struct bam_async_desc {
	struct list_head node;
	struct dma_async_tx_descriptor txd;
	u32 num_desc;
	enum bam_channel_dir dir;
	u32 fifo_pos;
	struct bam_desc_hw desc[0];
};

static inline struct bam_async_desc *to_bam_async_desc(
		struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct bam_async_desc, txd);
}


#define BAM_CTRL			0x0000
#define BAM_REVISION			0x0004
#define BAM_SW_REVISION			0x0080
#define BAM_NUM_PIPES			0x003C
#define BAM_TIMER			0x0040
#define BAM_TIMER_CTRL			0x0044
#define BAM_DESC_CNT_TRSHLD		0x0008
#define BAM_IRQ_SRCS			0x000C
#define BAM_IRQ_SRCS_MSK		0x0010
#define BAM_IRQ_SRCS_UNMASKED		0x0030
#define BAM_IRQ_STTS			0x0014
#define BAM_IRQ_CLR			0x0018
#define BAM_IRQ_EN			0x001C
#define BAM_CNFG_BITS			0x007C
#define BAM_IRQ_SRCS_EE(x)		(0x0800 + ((x) * 0x80))
#define BAM_IRQ_SRCS_MSK_EE(x)		(0x0804 + ((x) * 0x80))
#define BAM_P_CTRL(x)			(0x1000 + ((x) * 0x1000))
#define BAM_P_RST(x)			(0x1004 + ((x) * 0x1000))
#define BAM_P_HALT(x)			(0x1008 + ((x) * 0x1000))
#define BAM_P_IRQ_STTS(x)		(0x1010 + ((x) * 0x1000))
#define BAM_P_IRQ_CLR(x)		(0x1014 + ((x) * 0x1000))
#define BAM_P_IRQ_EN(x)			(0x1018 + ((x) * 0x1000))
#define BAM_P_EVNT_DEST_ADDR(x)		(0x182C + ((x) * 0x1000))
#define BAM_P_EVNT_REG(x)		(0x1818 + ((x) * 0x1000))
#define BAM_P_SW_OFSTS(x)		(0x1800 + ((x) * 0x1000))
#define BAM_P_DATA_FIFO_ADDR(x)		(0x1824 + ((x) * 0x1000))
#define BAM_P_DESC_FIFO_ADDR(x)		(0x181C + ((x) * 0x1000))
#define BAM_P_EVNT_TRSHLD(x)		(0x1828 + ((x) * 0x1000))
#define BAM_P_FIFO_SIZES(x)		(0x1820 + ((x) * 0x1000))

union bam_ctrl {
	u32 value;
	struct {
		u32 bam_sw_rst:1;
		u32 bam_en:1;
		u32 reserved3:2;
		u32 bam_en_accum:1;
		u32 testbus_sel:7;
		u32 reserved2:1;
		u32 bam_desc_cache_sel:2;
		u32 bam_cached_desc_store:1;
		u32 ibc_disable:1;
		u32 reserved1:15;
	} bits;
};

union bam_revision {
	u32 value;
	struct {
		u32 revision:8;
		u32 num_ees:4;
		u32 reserved1:1;
		u32 ce_buffer_size:1;
		u32 axi_active:1;
		u32 use_vmidmt:1;
		u32 secured:1;
		u32 bam_has_no_bypass:1;
		u32 high_frequency_bam:1;
		u32 inactiv_tmrs_exst:1;
		u32 num_inactiv_tmrs:1;
		u32 desc_cache_depth:2;
		u32 cmd_desc_en:1;
		u32 inactiv_tmr_base:8;
	} bits;
};

union bam_sw_revision {
	u32 value;
	struct {
		u32 step:16;
		u32 minor:12;
		u32 major:4;
	} bits;
};

union bam_num_pipes {
	u32 value;
	struct {
		u32 bam_num_pipes:8;
		u32 reserved:8;
		u32 periph_non_pipe_grp:8;
		u32 bam_non_pipe_grp:8;
	} bits;
};

union bam_irq_srcs_msk {
	u32 value;
	struct {
		u32 p_irq_msk:31;
		u32 bam_irq_msk:1;
	} bits;
};

union bam_cnfg_bits {
	u32 value;
	struct {
		u32 obsolete:2;
		u32 bam_pipe_cnfg:1;
		u32 obsolete2:1;
		u32 reserved:7;
		u32 bam_full_pipe:1;
		u32 bam_no_ext_p_rst:1;
		u32 bam_ibc_disable:1;
		u32 bam_sb_clk_req:1;
		u32 bam_psm_csw_req:1;
		u32 bam_psm_p_res:1;
		u32 bam_au_p_res:1;
		u32 bam_si_p_res:1;
		u32 bam_wb_p_res:1;
		u32 bam_wb_blk_csw:1;
		u32 bam_wb_csw_ack_idl:1;
		u32 bam_wb_retr_svpnt:1;
		u32 bam_wb_dsc_avl_p_rst:1;
		u32 bam_reg_p_en:1;
		u32 bam_psm_p_hd_data:1;
		u32 bam_au_accumed:1;
		u32 bam_cd_enable:1;
		u32 reserved2:4;
	} bits;
};

union bam_pipe_ctrl {
	u32 value;
	struct {
		u32 reserved:1;
		u32 p_en:1;
		u32 reserved2:1;
		u32 p_direction:1;
		u32 p_sys_strm:1;
		u32 p_sys_mode:1;
		u32 p_auto_eob:1;
		u32 p_auto_eob_sel:2;
		u32 p_prefetch_limit:2;
		u32 p_write_nwd:1;
		u32 reserved3:4;
		u32 p_lock_group:5;
		u32 reserved4:11;
	} bits;
};

/* BAM_DESC_CNT_TRSHLD */
#define CNT_TRSHLD		0xffff
#define DEFAULT_CNT_THRSHLD	0x4

/* BAM_IRQ_SRCS */
#define BAM_IRQ			(0x1 << 31)
#define P_IRQ			0x7fffffff

/* BAM_IRQ_SRCS_MSK */
#define BAM_IRQ_MSK		(0x1 << 31)
#define P_IRQ_MSK		0x7fffffff

/* BAM_IRQ_STTS */
#define BAM_TIMER_IRQ		(0x1 << 4)
#define BAM_EMPTY_IRQ		(0x1 << 3)
#define BAM_ERROR_IRQ		(0x1 << 2)
#define BAM_HRESP_ERR_IRQ	(0x1 << 1)

/* BAM_IRQ_CLR */
#define BAM_TIMER_CLR		(0x1 << 4)
#define BAM_EMPTY_CLR		(0x1 << 3)
#define BAM_ERROR_CLR		(0x1 << 2)
#define BAM_HRESP_ERR_CLR	(0x1 << 1)

/* BAM_IRQ_EN */
#define BAM_TIMER_EN		(0x1 << 4)
#define BAM_EMPTY_EN		(0x1 << 3)
#define BAM_ERROR_EN		(0x1 << 2)
#define BAM_HRESP_ERR_EN	(0x1 << 1)

/* BAM_P_IRQ_EN */
#define P_PRCSD_DESC_EN		(0x1 << 0)
#define P_TIMER_EN		(0x1 << 1)
#define P_WAKE_EN		(0x1 << 2)
#define P_OUT_OF_DESC_EN	(0x1 << 3)
#define P_ERR_EN		(0x1 << 4)
#define P_TRNSFR_END_EN		(0x1 << 5)
#define P_DEFAULT_IRQS_EN	(P_PRCSD_DESC_EN | P_ERR_EN | P_TRNSFR_END_EN)

/* BAM_P_SW_OFSTS */
#define P_SW_OFSTS_MASK		0xffff

#define BAM_DESC_FIFO_SIZE	SZ_32K
#define MAX_DESCRIPTORS (BAM_DESC_FIFO_SIZE / sizeof(struct bam_desc_hw) - 1)
#define BAM_MAX_DATA_SIZE	(SZ_32K - 8)

struct bam_chan {
	struct dma_chan common;
	struct bam_device *device;
	u32 id;
	u32 ee;
	bool idle;
	struct bam_dma_slave_config bam_slave;	/* runtime configuration */

	struct tasklet_struct tasklet;
	spinlock_t lock;		/* descriptor lock */

	struct list_head pending;	/* desc pending queue */
	struct list_head active;	/* desc running queue */

	struct bam_desc_hw *fifo_virt;
	dma_addr_t fifo_phys;

	/* fifo markers */
	unsigned short head;		/* start of active descriptor entries */
	unsigned short tail;		/* end of active descriptor entries */
};

static inline struct bam_chan *to_bam_chan(struct dma_chan *common)
{
	return container_of(common, struct bam_chan, common);
}

struct bam_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct device_dma_parameters dma_parms;
	struct bam_chan *channels;
	u32 num_channels;
	u32 num_ees;
	unsigned long enabled_ees;
	u32 feature;
	int irq;
	struct clk *bamclk;
};

static inline struct bam_device *to_bam_device(struct dma_device *common)
{
	return container_of(common, struct bam_device, common);
}

#endif /* __MSM_BAM_DMA_PRIV_H__ */
