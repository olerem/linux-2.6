/*
 * NAND controller driver for Alphascale ASM9260, which is probably
 * based on Evatronix NANDFLASH-CTRL IP (version unknown)
 *
 * Copyright (C), 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * Inspired by asm9260_nand.c,
 *	Copyright (C), 2007-2013, Alphascale Tech. Co., Ltd.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_mtd.h>
#include <linux/pinctrl/consumer.h>

#define ASM9260_ECC_STEP		512
#define ASM9260_ECC_MAX_BIT		16
#define ASM9260_MAX_CHIPS		2

#define mtd_to_priv(m)	container_of(m, struct asm9260_nand_priv, mtd)

#define HW_CMD				0x00
#define BM_CMD_CMD2_S			24
#define BM_CMD_CMD1_S			16
#define BM_CMD_CMD0_S			8
/* 0 - ADDR0, 1 - ADDR1 */
#define BM_CMD_ADDR1			BIT(7)
/* 0 - PIO, 1 - DMA */
#define BM_CMD_DMA			BIT(6)
#define BM_CMD_CMDSEQ_S			0
/*
 * ASM9260 Sequences:
 * SEQ0:  single command, wait for RnB
 * SEQ1:  send  cmd, addr, wait tWHR, fetch data
 * SEQ2:  send  cmd, addr, wait RnB, fetch data
 * SEQ3:  send  cmd, addr, wait tADL, send data, wait RnB
 * SEQ4:  send  cmd, wait tWHR, fetch data
 * SEQ5:  send  cmd, 3 x addr, wait tWHR, fetch data
 * SEQ6:  wait tRHW, send  cmd, 2 x addr, cmd, wait tCCS, fetch data
 * SEQ7:  wait tRHW, send  cmd, 35 x addr, cmd, wait tCCS, fetch data
 * SEQ8:  send  cmd, 2 x addr, wait tCCS, fetch data
 * SEQ9:  send  cmd, 5 x addr, wait RnB
 * SEQ10: send  cmd, 5 x addr, cmd, wait RnB, fetch data
 * SEQ11: send  cmd, wait RnB, fetch data
 * SEQ12: send  cmd, 5 x addr, wait tADL, send data, cmd
 * SEQ13: send  cmd, 5 x addr, wait tADL, send data
 * SEQ14: send  cmd, 3 x addr, cmd, wait RnB
 * SEQ15: send  cmd, 5 x addr, cmd, 5 x addr, cmd, wait RnB, fetch data
 * SEQ17: send  cmd, 5 x addr, wait RnB, fetch data
*/
#define  SEQ0				0x00
#define  SEQ1				0x21
#define  SEQ2				0x22
#define  SEQ3				0x03
#define  SEQ4				0x24
#define  SEQ5				0x25
#define  SEQ6				0x26
#define  SEQ7				0x27
#define  SEQ8				0x08
#define  SEQ9				0x29
#define  SEQ10				0x2a
#define  SEQ11				0x2b
#define  SEQ12				0x0c
#define  SEQ13				0x0d
#define  SEQ14				0x0e
#define  SEQ15				0x2f
#define  SEQ17				0x15

#define HW_CTRL				0x04
#define BM_CTRL_DIS_STATUS		BIT(23)
#define BM_CTRL_READ_STAT		BIT(22)
#define BM_CTRL_SMALL_BLOCK_EN		BIT(21)
#define BM_CTRL_ADDR_CYCLE1_S		18
#define  ADDR_CYCLE_0			0x0
#define  ADDR_CYCLE_1			0x1
#define  ADDR_CYCLE_2			0x2
#define  ADDR_CYCLE_3			0x3
#define  ADDR_CYCLE_4			0x4
#define  ADDR_CYCLE_5			0x5
#define BM_CTRL_ADDR1_AUTO_INCR		BIT(17)
#define BM_CTRL_ADDR0_AUTO_INCR		BIT(16)
#define BM_CTRL_WORK_MODE		BIT(15)
#define BM_CTRL_PORT_EN			BIT(14)
#define BM_CTRL_LOOKU_EN		BIT(13)
#define BM_CTRL_IO_16BIT		BIT(12)
/* Overwrite BM_CTRL_PAGE_SIZE with HW_DATA_SIZE */
#define BM_CTRL_CUSTOM_PAGE_SIZE	BIT(11)
#define BM_CTRL_PAGE_SIZE_S		8
#define BM_CTRL_PAGE_SIZE(x)		((ffs((x) >> 8) - 1) & 0x7)
#define  PAGE_SIZE_256B			0x0
#define  PAGE_SIZE_512B			0x1
#define  PAGE_SIZE_1024B		0x2
#define  PAGE_SIZE_2048B		0x3
#define  PAGE_SIZE_4096B		0x4
#define  PAGE_SIZE_8192B		0x5
#define  PAGE_SIZE_16384B		0x6
#define  PAGE_SIZE_32768B		0x7
#define BM_CTRL_BLOCK_SIZE_S		6
#define BM_CTRL_BLOCK_SIZE(x)		((ffs((x) >> 5) - 1) & 0x3)
#define  BLOCK_SIZE_32P			0x0
#define  BLOCK_SIZE_64P			0x1
#define  BLOCK_SIZE_128P		0x2
#define  BLOCK_SIZE_256P		0x3
#define BM_CTRL_ECC_EN			BIT(5)
#define BM_CTRL_INT_EN			BIT(4)
#define BM_CTRL_SPARE_EN		BIT(3)
/* same values as BM_CTRL_ADDR_CYCLE1_S */
#define BM_CTRL_ADDR_CYCLE0_S		0

#define HW_STATUS			0x08
#define	BM_CTRL_NFC_BUSY		BIT(8)
/* MEM1_RDY (BIT1) - MEM7_RDY (BIT7) */
#define	BM_CTRL_MEM0_RDY		BIT(0)

#define HW_INT_MASK			0x0c
#define HW_INT_STATUS			0x10
#define BM_INT_FIFO_ERROR		BIT(12)
#define BM_INT_MEM_RDY_S		4
/* MEM1_RDY (BIT5) - MEM7_RDY (BIT11) */
#define BM_INT_MEM0_RDY			BIT(4)
#define BM_INT_ECC_TRSH_ERR		BIT(3)
#define BM_INT_ECC_FATAL_ERR		BIT(2)
#define BM_INT_CMD_END			BIT(1)

#define HW_ECC_CTRL			0x14
/* bits per 512 bytes */
#define	BM_ECC_CAP_S			5
/* support ecc strength 2, 4, 6, 8, 10, 12, 14, 16. */
#define BM_ECC_CAPn(x)			((((x) >> 1) - 1) & 0x7)
/* Warn if some bitflip level (threshold) reached. Max 15 bits. */
#define BM_ECC_ERR_THRESHOLD_S		8
#define BM_ECC_ERR_THRESHOLD_M		0xf
#define BM_ECC_ERR_OVER			BIT(2)
/* Uncorrected error. */
#define BM_ECC_ERR_UNC			BIT(1)
/* Corrected error. */
#define BM_ECC_ERR_CORRECT		BIT(0)

#define HW_ECC_OFFSET			0x18
#define HW_ADDR0_0			0x1c
#define HW_ADDR1_0			0x20
#define HW_ADDR0_1			0x24
#define HW_ADDR1_1			0x28
#define HW_SPARE_SIZE			0x30
#define HW_DMA_ADDR			0x64
#define HW_DMA_CNT			0x68

#define HW_DMA_CTRL			0x6c
#define BM_DMA_CTRL_START		BIT(7)
/* 0 - to device; 1 - from device */
#define BM_DMA_CTRL_FROM_DEVICE		BIT(6)
/* 0 - software maneged; 1 - scatter-gather */
#define BM_DMA_CTRL_SG			BIT(5)
#define BM_DMA_CTRL_BURST_S		2
#define  DMA_BURST_INCR4		0x0
#define  DMA_BURST_STREAM		0x1
#define  DMA_BURST_SINGLE		0x2
#define  DMA_BURST_INCR			0x3
#define  DMA_BURST_INCR8		0x4
#define  DMA_BURST_INCR16		0x5
#define BM_DMA_CTRL_ERR			BIT(1)
#define BM_DMA_CTRL_RDY			BIT(0)

#define HW_MEM_CTRL			0x80
#define	BM_MEM_CTRL_WP_STATE_MASK	0xff00
#define	BM_MEM_CTRL_UNWPn(x)		(1 << ((x) + 8))
#define BM_MEM_CTRL_CEn(x)		(((x) & 7) << 0)

/* BM_CTRL_CUSTOM_PAGE_SIZE should be set */
#define HW_DATA_SIZE			0x84
#define HW_READ_STATUS			0x88
#define HW_TIM_SEQ_0			0x8c
#define HW_TIMING_ASYN			0x90
#define HW_TIMING_SYN			0x94

#define HW_FIFO_DATA			0x98
#define HW_TIME_MODE			0x9c
#define HW_FIFO_INIT			0xb0
/*
 * Counter for ecc related errors.
 * For each 512 byte block it has 5bit counter.
 */
#define HW_ECC_ERR_CNT			0xb8

#define HW_TIM_SEQ_1			0xc8

struct asm9260_nand_priv {
	struct device		*dev;
	struct mtd_info		mtd;
	struct nand_chip	nand;
	struct nand_ecclayout	ecc_layout;

	struct clk		*clk;
	struct clk		*clk_ahb;

	void __iomem *base;
	int irq_done;

	u32 read_cache;
	int read_cache_cnt;
	int read_req_cnt;
	u32 cmd_cache;
	u32 ctrl_cache;
	u32 mem_mask;
	u32 page_cache;
	unsigned int wait_time;

	unsigned int spare_size;
};

static void asm9260_reg_update_bits(struct asm9260_nand_priv *priv,
		u32 reg_offset, u32 set, u32 clr)
{
	u32 val;

	val = ioread32(priv->base + reg_offset);
	val &= ~clr;
	val |= set;
	iowrite32(val, priv->base + reg_offset);
}

static void asm9260_select_chip(struct mtd_info *mtd, int chip)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	if (chip == -1) {
		iowrite32(BM_MEM_CTRL_WP_STATE_MASK, priv->base + HW_MEM_CTRL);
	} else {
		priv->mem_mask = BM_CTRL_MEM0_RDY << chip;
		iowrite32(BM_MEM_CTRL_UNWPn(chip) | BM_MEM_CTRL_CEn(chip),
			  priv->base + HW_MEM_CTRL);
	}
}

/* 3 commands are supported by HW. 3-d can be used for TWO PLANE. */
static void asm9260_nand_cmd_prep(struct asm9260_nand_priv *priv,
		u8 cmd0, u8 cmd1, u8 cmd2, u8 seq)
{
	priv->cmd_cache  = (cmd0 << BM_CMD_CMD0_S) | (cmd1 << BM_CMD_CMD1_S);
	priv->cmd_cache |= seq << BM_CMD_CMDSEQ_S;
}

static dma_addr_t asm9260_nand_dma_set(struct mtd_info *mtd, void *buf,
		enum dma_data_direction dir, size_t size)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	dma_addr_t dma_addr;

	dma_addr = dma_map_single(priv->dev, buf, size, dir);
	if (dma_mapping_error(priv->dev, dma_addr)) {
		dev_err(priv->dev, "dma_map_single failed!\n");
		return dma_addr;

	}

	iowrite32(dma_addr, priv->base + HW_DMA_ADDR);
	iowrite32(size, priv->base + HW_DMA_CNT);
	iowrite32(BM_DMA_CTRL_START
		  | (dir == DMA_FROM_DEVICE ? BM_DMA_CTRL_FROM_DEVICE : 0)
		  | (DMA_BURST_INCR16 << BM_DMA_CTRL_BURST_S),
		  priv->base + HW_DMA_CTRL);
	return dma_addr;
}

/* complete command request */
static void asm9260_nand_cmd_comp(struct mtd_info *mtd, int dma)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	int timeout;
	u32 cmd;

	if (!priv->cmd_cache)
		return;

	if (dma) {
		priv->cmd_cache |= BM_CMD_DMA;
		priv->irq_done = 0;
		iowrite32(priv->mem_mask << BM_INT_MEM_RDY_S,
				priv->base + HW_INT_MASK);
	} else
		priv->read_req_cnt = ioread32(priv->base + HW_DATA_SIZE);

	iowrite32(priv->cmd_cache, priv->base + HW_CMD);
	cmd = priv->cmd_cache;
	priv->cmd_cache = 0;

	if (dma) {
		struct nand_chip *nand = &priv->nand;

		timeout = wait_event_timeout(nand->controller->wq,
				priv->irq_done,
				msecs_to_jiffies(priv->wait_time ?
					priv->wait_time : 20));
		if (timeout <= 0) {
			dev_info(priv->dev,
					"Request 0x%08x timed out\n", cmd);
			/* TODO: Do something useful here? */
			/* FIXME: if we have problems on DMA or PIO, we need to
			 * reset NFC. On asm9260 it is possible only with global
			 * reset register. How can we use it here? */
		}
		priv->wait_time = 0;
	} else
		nand_wait_ready(mtd);
}

static int asm9260_nand_dev_ready(struct mtd_info *mtd)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u32 tmp;

	tmp = ioread32(priv->base + HW_STATUS);

	return (!(tmp & BM_CTRL_NFC_BUSY) &&
			(tmp & priv->mem_mask));
}

static void asm9260_nand_ctrl(struct asm9260_nand_priv *priv, u32 set)
{
	iowrite32(priv->ctrl_cache | set, priv->base + HW_CTRL);
}

static void asm9260_nand_set_addr(struct asm9260_nand_priv *priv,
		u32 row_addr, u32 column)
{
	u32 addr[2];

	addr[0] = (column & 0xffff) | (0xffff0000 & (row_addr << 16));
	addr[1] = (row_addr >> 16) & 0xff;

	iowrite32(addr[0], priv->base + HW_ADDR0_0);
	iowrite32(addr[1], priv->base + HW_ADDR0_1);
}

static void asm9260_nand_command_lp(struct mtd_info *mtd,
		unsigned int command, int column, int page_addr)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	switch (command) {
	case NAND_CMD_RESET:
		asm9260_nand_cmd_prep(priv, NAND_CMD_RESET, 0, 0, SEQ0);
		asm9260_nand_cmd_comp(mtd, 0);
		break;

	case NAND_CMD_READID:
		iowrite32(1, priv->base + HW_FIFO_INIT);
		iowrite32((ADDR_CYCLE_1 << BM_CTRL_ADDR_CYCLE1_S)
			| BM_CTRL_CUSTOM_PAGE_SIZE
			| (PAGE_SIZE_4096B << BM_CTRL_PAGE_SIZE_S)
			| (BLOCK_SIZE_32P << BM_CTRL_BLOCK_SIZE_S)
			| BM_CTRL_INT_EN
			| (ADDR_CYCLE_1 << BM_CTRL_ADDR_CYCLE0_S),
			priv->base + HW_CTRL);

		iowrite32(8, priv->base + HW_DATA_SIZE);
		iowrite32(column, priv->base + HW_ADDR0_0);
		asm9260_nand_cmd_prep(priv, NAND_CMD_READID, 0, 0, SEQ1);

		priv->read_cache_cnt = 0;
		break;

	case NAND_CMD_READOOB:
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	case NAND_CMD_READ0:
		iowrite32(1, priv->base + HW_FIFO_INIT);

		if (column == 0) {
			asm9260_nand_ctrl(priv, 0);
			iowrite32(priv->spare_size, priv->base + HW_SPARE_SIZE);
		} else if (column == mtd->writesize) {
			asm9260_nand_ctrl(priv, BM_CTRL_CUSTOM_PAGE_SIZE);
			iowrite32(mtd->oobsize, priv->base + HW_SPARE_SIZE);
			iowrite32(mtd->oobsize, priv->base + HW_DATA_SIZE);
		} else {
			dev_err(priv->dev, "Couldn't support the column\n");
			break;
		}

		asm9260_nand_set_addr(priv, page_addr, column);

		asm9260_nand_cmd_prep(priv, NAND_CMD_READ0,
				NAND_CMD_READSTART, 0, SEQ10);

		priv->read_cache_cnt = 0;
		break;
	case NAND_CMD_SEQIN:
		iowrite32(1, priv->base + HW_FIFO_INIT);

		if (column == 0) {
			priv->page_cache = page_addr;
			asm9260_nand_ctrl(priv, 0);
			iowrite32(priv->spare_size, priv->base + HW_SPARE_SIZE);
		} else if (column == mtd->writesize) {
			asm9260_nand_ctrl(priv, BM_CTRL_CUSTOM_PAGE_SIZE);
			iowrite32(mtd->oobsize, priv->base + HW_DATA_SIZE);
		}

		asm9260_nand_set_addr(priv, page_addr, column);

		asm9260_nand_cmd_prep(priv, NAND_CMD_SEQIN, NAND_CMD_PAGEPROG,
			0, SEQ12);

		break;
	case NAND_CMD_STATUS:
		iowrite32(1, priv->base + HW_FIFO_INIT);
		asm9260_nand_ctrl(priv, BM_CTRL_CUSTOM_PAGE_SIZE);

		/*
		 * Workaround for status bug.
		 * Instead of SEQ4 we need to use SEQ1 here, which will
		 * send cmd with address. For this case we need to make sure
		 * ADDR == 0.
		 */
		asm9260_nand_set_addr(priv, 0, 0);
		iowrite32(4, priv->base + HW_DATA_SIZE);
		asm9260_nand_cmd_prep(priv, NAND_CMD_STATUS, 0, 0, SEQ1);

		priv->read_cache_cnt = 0;
		break;

	case NAND_CMD_ERASE1:
		priv->wait_time = 400;
		asm9260_nand_set_addr(priv, page_addr, column);

		asm9260_nand_ctrl(priv, 0);

		/*
		 * Prepare and send command now. We don't need to split it in
		 * two stages.
		 */
		asm9260_nand_cmd_prep(priv, NAND_CMD_ERASE1, NAND_CMD_ERASE2,
				0, SEQ14);
		asm9260_nand_cmd_comp(mtd, 0);
		break;
	case NAND_CMD_ERASE2:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_READSTART:
		break;
	default:
		dev_err(priv->dev, "FIXME: Unsupported cmd 0x%x\n", command);
		break;
	}
}


/**
 * We can't read less then 32 bits on HW_FIFO_DATA. So, to make
 * read_byte and read_word happy, we use sort of cached 32bit read.
 * Note: expected values for size should be 1 or 2 bytes.
 */
static u32 asm9260_nand_read_cached(struct mtd_info *mtd, int size)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u32 tmp;

	if (priv->read_cache_cnt <= 0) {
		asm9260_nand_cmd_comp(mtd, 0);

		/* if we read more then requested, the SoC will die. */
		if (priv->read_req_cnt >= sizeof(u32))
			priv->read_cache = ioread32(priv->base + HW_FIFO_DATA);
		else
			BUG();

		priv->read_req_cnt -= sizeof(u32);
		priv->read_cache_cnt = 4;
	}

	tmp = priv->read_cache >> (8 * (4 - priv->read_cache_cnt));
	priv->read_cache_cnt -= size;

	return tmp;
}

static u8 asm9260_nand_read_byte(struct mtd_info *mtd)
{
	return 0xff & asm9260_nand_read_cached(mtd, sizeof(u8));
}

static u16 asm9260_nand_read_word(struct mtd_info *mtd)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	/*
	 * In case some one used mixed sequnece like:
	 * [read_byte, read_word, read_word] > u32, we will try not to fail.
	 * If priv->read_req_cnt triggers BUG warning, make sure
	 * we requested enough (u32 * x) data before reading it.
	 */
	if (priv->read_cache_cnt &&
			priv->read_cache_cnt - sizeof(u16) < 0) {
		u16 val;

		val = 0xff & asm9260_nand_read_cached(mtd, sizeof(u8));
		val |= (0xff & asm9260_nand_read_cached(mtd, sizeof(u8))) << 8;
		return val;
	} else
		return 0xffff & asm9260_nand_read_cached(mtd, sizeof(u16));
}

static void asm9260_nand_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	dma_addr_t dma_addr;
	int dma_ok = 0;

	if (!is_vmalloc_addr(buf)) {
		dma_addr = asm9260_nand_dma_set(mtd, buf, DMA_FROM_DEVICE, len);
		dma_ok = !(dma_mapping_error(priv->dev, dma_addr));
	}
	asm9260_nand_cmd_comp(mtd, dma_ok);

	if (dma_ok) {
		dma_sync_single_for_cpu(priv->dev, dma_addr, len,
				DMA_FROM_DEVICE);
		dma_unmap_single(priv->dev, dma_addr, len, DMA_FROM_DEVICE);
		return;
	}

	/* We can't use fifo if requested size is not aligned on 4 bytes. */
	BUG_ON(len & 0x3);

	/* fall back to pio mode */
	len >>= 2;
	ioread32_rep(priv->base + HW_FIFO_DATA, buf, len);
}

static void asm9260_nand_write_buf(struct mtd_info *mtd,
		const u8 *buf, int len)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	dma_addr_t dma_addr;
	int dma_ok = 0;

	if (!is_vmalloc_addr(buf)) {
		dma_addr = asm9260_nand_dma_set(mtd,
				(void *)buf, DMA_TO_DEVICE, len);
		dma_ok = !(dma_mapping_error(priv->dev, dma_addr));
	}

	if (dma_ok)
		dma_sync_single_for_device(priv->dev, dma_addr, len,
				DMA_TO_DEVICE);
	asm9260_nand_cmd_comp(mtd, dma_ok);

	if (dma_ok) {
		dma_unmap_single(priv->dev, dma_addr, len, DMA_TO_DEVICE);
		return;
	}

	/* We can't use fifo if requested size is not aligned on 4 bytes. */
	BUG_ON(len & 0x3);

	/* fall back to pio mode */
	len >>= 2;
	iowrite32_rep(priv->base + HW_FIFO_DATA, buf, len);
}

static int asm9260_nand_write_page_raw(struct mtd_info *mtd,
		struct nand_chip *chip, const u8 *buf,
		int oob_required)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	chip->write_buf(mtd, buf, mtd->writesize);
	if (oob_required)
		chip->ecc.write_oob(mtd, chip, priv->page_cache &
				chip->pagemask);
	return 0;
}

static int asm9260_nand_write_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, const u8 *buf,
		int oob_required)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	asm9260_reg_update_bits(priv, HW_CTRL, BM_CTRL_ECC_EN, 0);
	chip->ecc.write_page_raw(mtd, chip, buf, oob_required);

	return 0;
}

static unsigned int asm9260_nand_count_ecc(struct asm9260_nand_priv *priv)
{
	u32 tmp, i, count, maxcount = 0;

	/* FIXME: this layout was tested only on 2048byte NAND.
	 * NANDs with bigger page size should use more registers. */
	tmp = ioread32(priv->base + HW_ECC_ERR_CNT);
	for (i = 0; i < 4; i++) {
		count = 0x1f & (tmp >> (5 * i));
		maxcount = max_t(unsigned int, maxcount, count);
	}

	return count;
}

static int asm9260_nand_read_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, u8 *buf,
		int oob_required, int page)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u8 *temp_ptr;
	u32 status, max_bitflips = 0;

	temp_ptr = buf;

	/* enable ecc */
	asm9260_reg_update_bits(priv, HW_CTRL, BM_CTRL_ECC_EN, 0);
	chip->read_buf(mtd, temp_ptr, mtd->writesize);

	status = ioread32(priv->base + HW_ECC_CTRL);

	if (status & BM_ECC_ERR_UNC) {
		u32 ecc_err;

		ecc_err = ioread32(priv->base + HW_ECC_ERR_CNT);
		/* check if it is erased page (all_DATA_OOB == 0xff) */
		/* FIXME: should be tested if it is a bullet proof solution.
		 *  if not, use is_buf_blank. */
		if (ecc_err != 0x8421)
			mtd->ecc_stats.failed++;

	} else if (status & BM_ECC_ERR_CORRECT) {
		max_bitflips = asm9260_nand_count_ecc(priv);
		mtd->ecc_stats.corrected += max_bitflips;
	}

	if (oob_required)
		chip->ecc.read_oob(mtd, chip, page);

	return max_bitflips;
}

static irqreturn_t asm9260_nand_irq(int irq, void *device_info)
{
	struct asm9260_nand_priv *priv = device_info;
	struct nand_chip *nand = &priv->nand;
	u32 status;

	status = ioread32(priv->base + HW_INT_STATUS);
	if (!(status & (priv->mem_mask << BM_INT_MEM_RDY_S)))
		return IRQ_NONE;

	iowrite32(0, priv->base + HW_INT_MASK);
	iowrite32(0, priv->base + HW_INT_STATUS);
	priv->irq_done = 1;
	wake_up(&nand->controller->wq);

	return IRQ_HANDLED;
}

static void __init asm9260_nand_init_chip(struct nand_chip *nand_chip)
{
	nand_chip->select_chip	= asm9260_select_chip;
	nand_chip->cmdfunc	= asm9260_nand_command_lp;
	nand_chip->read_byte	= asm9260_nand_read_byte;
	nand_chip->read_word	= asm9260_nand_read_word;
	nand_chip->read_buf	= asm9260_nand_read_buf;
	nand_chip->write_buf	= asm9260_nand_write_buf;

	nand_chip->dev_ready	= asm9260_nand_dev_ready;
	nand_chip->chip_delay	= 100;
	nand_chip->options	|= NAND_NO_SUBPAGE_WRITE;

	nand_chip->ecc.write_page	= asm9260_nand_write_page_hwecc;
	nand_chip->ecc.write_page_raw	= asm9260_nand_write_page_raw;
	nand_chip->ecc.read_page	= asm9260_nand_read_page_hwecc;
}

static int __init asm9260_nand_cached_config(struct asm9260_nand_priv *priv)
{
	struct nand_chip *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	u32 addr_cycles, col_cycles, pages_per_block;

	pages_per_block = mtd->erasesize / mtd->writesize;
	/* max 256P, min 32P */
	if (pages_per_block & ~(0x000001e0)) {
		dev_err(priv->dev, "Unsupported erasesize 0x%x\n",
				mtd->erasesize);
		return -EINVAL;
	}

	/* max 32K, min 256. */
	if (mtd->writesize & ~(0x0000ff00)) {
		dev_err(priv->dev, "Unsupported writesize 0x%x\n",
				mtd->erasesize);
		return -EINVAL;
	}

	col_cycles  = 2;
	addr_cycles = col_cycles +
		(((mtd->size >> mtd->writesize) > 65536) ? 3 : 2);

	priv->ctrl_cache = addr_cycles << BM_CTRL_ADDR_CYCLE1_S
		| BM_CTRL_PAGE_SIZE(mtd->writesize) << BM_CTRL_PAGE_SIZE_S
		| BM_CTRL_BLOCK_SIZE(pages_per_block) << BM_CTRL_BLOCK_SIZE_S
		| BM_CTRL_INT_EN
		| addr_cycles << BM_CTRL_ADDR_CYCLE0_S;

	iowrite32(BM_ECC_CAPn(nand->ecc.strength) << BM_ECC_CAP_S,
			priv->base + HW_ECC_CTRL);

	iowrite32(mtd->writesize + priv->spare_size,
			priv->base + HW_ECC_OFFSET);

	return 0;
}

static unsigned long __init clk_get_cyc_from_ns(struct clk *clk,
		unsigned long ns)
{
	unsigned int cycle;

	cycle = NSEC_PER_SEC / clk_get_rate(clk);
	return DIV_ROUND_CLOSEST(ns, cycle);
}

static void __init asm9260_nand_timing_config(struct asm9260_nand_priv *priv)
{
	struct nand_chip *nand = &priv->nand;
	const struct nand_sdr_timings *time;
	u32 twhr, trhw, trwh, trwp, tadl, tccs, tsync, trr, twb;
	int mode;

	mode = onfi_get_async_timing_mode(nand);
	if (mode == ONFI_TIMING_MODE_UNKNOWN)
		mode = nand->onfi_timing_mode_default;

	dev_info(priv->dev, "ONFI timing mode: %i\n", mode);

	time = onfi_async_timing_mode_to_sdr_timings(mode);
	if (IS_ERR_OR_NULL(time)) {
		dev_err(priv->dev, "Can't get onfi_timing_mode: %i\n", mode);
		return;
	}

	trwh = clk_get_cyc_from_ns(priv->clk, time->tWH_min / 1000);
	trwp = clk_get_cyc_from_ns(priv->clk, time->tWP_min / 1000);

	iowrite32((trwh << 4) | (trwp), priv->base + HW_TIMING_ASYN);

	twhr = clk_get_cyc_from_ns(priv->clk, time->tWHR_min / 1000);
	trhw = clk_get_cyc_from_ns(priv->clk, time->tRHW_min / 1000);
	tadl = clk_get_cyc_from_ns(priv->clk, time->tADL_min / 1000);
	/* tCCS - change read/write collumn. Time between last cmd and data. */
	tccs = clk_get_cyc_from_ns(priv->clk,
			(time->tCLR_min + time->tCLH_min + time->tRC_min)
			/ 1000);

	iowrite32((twhr << 24) | (trhw << 16)
		| (tadl << 8) | (tccs), priv->base + HW_TIM_SEQ_0);

	trr = clk_get_cyc_from_ns(priv->clk, time->tRR_min / 1000);
	tsync = 0; /* ??? */
	twb = clk_get_cyc_from_ns(priv->clk, time->tWB_max / 1000);
	iowrite32((tsync << 16) | (trr << 9) | (twb),
			priv->base + HW_TIM_SEQ_1);
}

static int __init asm9260_nand_ecc_conf(struct asm9260_nand_priv *priv)
{
	struct device_node *np = priv->dev->of_node;
	struct nand_chip *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	struct nand_ecclayout *ecc_layout = &priv->ecc_layout;
	int i, ecc_strength;

	nand->ecc.mode = of_get_nand_ecc_mode(np);
	switch (nand->ecc.mode) {
	case NAND_ECC_SOFT:
	case NAND_ECC_SOFT_BCH:
		dev_info(priv->dev, "Using soft ECC %i\n", nand->ecc.mode);
		/* nand_base will find needed settings */
		return 0;
	case NAND_ECC_HW:
	default:
		dev_info(priv->dev, "Using default NAND_ECC_HW\n");
		nand->ecc.mode = NAND_ECC_HW;
		break;
	}

	ecc_strength = of_get_nand_ecc_strength(np);
	nand->ecc.size = ASM9260_ECC_STEP;
	nand->ecc.steps = mtd->writesize / nand->ecc.size;

	if (ecc_strength < 0) {
		/* Let's check if ONFI can help us. */
		if (nand->ecc_strength_ds <= 0) {
			/* No ONFI and no DT - it is bad. */
			dev_err(priv->dev,
					"nand-ecc-strength is not set by DT, ONFI or nand_ids table. Please set nand-ecc-strength in DT or add chip quirk in nand_ids.c.\n");
			return -EINVAL;
		}

		dev_info(priv->dev, "ONFI:nand-ecc-strength = %i\n",
				nand->ecc_strength_ds);
	} else
		dev_info(priv->dev, "DT:nand-ecc-strength = %i\n",
				ecc_strength);

	if (ecc_strength == 0 || ecc_strength > ASM9260_ECC_MAX_BIT) {
		dev_err(priv->dev,
				"Not supported ecc_strength value: %i\n",
				ecc_strength);
		return -EINVAL;
	}

	if (ecc_strength & 0x1) {
		ecc_strength++;
		dev_info(priv->dev,
				"Only even ecc_strength value is supported. Recalculating: %i\n",
		       ecc_strength);
	}

	/* 13 - the smallest integer for 512 (ASM9260_ECC_STEP). Div to 8bit. */
	nand->ecc.bytes = DIV_ROUND_UP(ecc_strength * 13, 8);

	if (nand->ecc_step_ds != 0 && nand->ecc_step_ds != ASM9260_ECC_STEP) {
		int step = of_get_nand_ecc_step_size(np);
		if (step < 0 || step != ASM9260_ECC_STEP) {
			dev_err(priv->dev, "This ECC step size (%i) is not supported by HW_ECC. Please, configure DT with nand-ecc-mode = \"soft\" or \"soft_bch\"; or set nand-ecc-step-size = %i\n",
					step > 0 ? step : nand->ecc_step_ds,
					ASM9260_ECC_STEP);
			return -EINVAL;
		} else
			dev_info(priv->dev, "Overwriting ECC step size: %i -> %i\n",
					nand->ecc_step_ds, step);
	}

	ecc_layout->eccbytes = nand->ecc.bytes * nand->ecc.steps;

	/* 2 bytes for bad block marker. */
	if (ecc_layout->eccbytes + 2 > mtd->oobsize) {
		dev_err(priv->dev, "ECC need more place then OOB can provide: ECC = %i + 2, OOB = %i. Try to reduce nand-ecc-strength.\n",
				ecc_layout->eccbytes,
				mtd->oobsize);
		return -EINVAL;
	}

	nand->ecc.layout = ecc_layout;
	nand->ecc.strength = ecc_strength;

	priv->spare_size = mtd->oobsize - ecc_layout->eccbytes;

	ecc_layout->oobfree[0].offset = 2;
	ecc_layout->oobfree[0].length = priv->spare_size - 2;

	/* FIXME: can we use same layout as SW_ECC? */
	for (i = 0; i < ecc_layout->eccbytes; i++)
		ecc_layout->eccpos[i] = priv->spare_size + i;

	return 0;
}

static int __init asm9260_nand_get_dt_clks(struct asm9260_nand_priv *priv)
{
	int clk_idx = 0, err;

	priv->clk = devm_clk_get(priv->dev, "mod");
	if (IS_ERR(priv->clk))
		goto out_err;

	/* configure AHB clock */
	clk_idx = 1;
	priv->clk_ahb = devm_clk_get(priv->dev, "ahb");
	if (IS_ERR(priv->clk_ahb))
		goto out_err;

	err = clk_prepare_enable(priv->clk_ahb);
	if (err)
		dev_err(priv->dev, "Failed to enable ahb_clk!\n");

	err = clk_set_rate(priv->clk, clk_get_rate(priv->clk_ahb));
	if (err) {
		clk_disable_unprepare(priv->clk_ahb);
		dev_err(priv->dev, "Failed to set rate!\n");
	}

	err = clk_prepare_enable(priv->clk);
	if (err) {
		clk_disable_unprepare(priv->clk_ahb);
		dev_err(priv->dev, "Failed to enable clk!\n");
	}

	return 0;
out_err:
	dev_err(priv->dev, "%s: Failed to get clk (%i)\n", __func__, clk_idx);
	return 1;
}

static int __init asm9260_nand_probe(struct platform_device *pdev)
{
	struct asm9260_nand_priv *priv;
	struct nand_chip *nand;
	struct mtd_info *mtd;
	struct device_node *np = pdev->dev.of_node;
	struct mtd_part_parser_data ppdata;
	struct resource *r;
	struct pinctrl *p;
	int ret;
	unsigned int irq;
	u32 val;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct asm9260_nand_priv),
			GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, r);
	if (!priv->base) {
		dev_err(&pdev->dev, "Unable to map resource!\n");
		return -EINVAL;
	}

	priv->dev = &pdev->dev;
	nand = &priv->nand;
	nand->priv = priv;

	platform_set_drvdata(pdev, priv);
	mtd = &priv->mtd;
	mtd->priv = nand;
	mtd->owner = THIS_MODULE;
	mtd->name = dev_name(&pdev->dev);

	if (asm9260_nand_get_dt_clks(priv))
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (!irq)
		return -ENODEV;

	iowrite32(0, priv->base + HW_INT_MASK);
	ret = devm_request_irq(priv->dev, irq, asm9260_nand_irq,
				IRQF_ONESHOT | IRQF_SHARED,
				dev_name(&pdev->dev), priv);

	asm9260_nand_init_chip(nand);

	ret = of_property_read_u32(np, "nand-max-chips", &val);
	if (ret)
		val = 1;

	if (val > ASM9260_MAX_CHIPS) {
		dev_err(&pdev->dev, "Unsupported nand-max-chips value: %i\n",
				val);
		return -ENODEV;
	}

	if (val > 1) {
		dev_err(&pdev->dev, "FIXME: This driver was designed and tested only with one chip HW. If you have dual chip HW pleas contact author of this driver or add support by your self.\n");
		return -ENODEV;
	}

	p = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(p))
		dev_warn(&pdev->dev, "pins are not configured\n");

	ret = nand_scan_ident(mtd, val, NULL);
	if (ret) {
		dev_err(&pdev->dev, "scan_ident filed!\n");
		return ret;
	}

	/*
	 * If you got this message, please add new page size to this filter
	 * and make sure asm9260_nand_count_ecc() and other parts working
	 * correctly.
	 */
	switch(mtd->writesize) {
	case 2048:
		break;
	default:
		dev_err(&pdev->dev, "Found NAND with not tested pagesize: %i! Exit.\n",
				mtd->writesize);
		return -ENODEV;
	}

	if (of_get_nand_on_flash_bbt(np)) {
		dev_info(&pdev->dev, "Use On Flash BBT\n");
		nand->bbt_options = NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB_BBM
			| NAND_BBT_LASTBLOCK;
	}

	asm9260_nand_timing_config(priv);

	ret = asm9260_nand_ecc_conf(priv);
	if (ret)
		return ret;

	ret = asm9260_nand_cached_config(priv);
	if (ret)
		return ret;

	/* second phase scan */
	if (nand_scan_tail(mtd)) {
		dev_err(&pdev->dev, "scan_tail filed!\n");
		return -ENXIO;
	}


	ppdata.of_node = np;
	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);

	return ret;
}


static int asm9260_nand_remove(struct platform_device *pdev)
{
	struct asm9260_nand_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk);
	clk_disable_unprepare(priv->clk_ahb);
	nand_release(&priv->mtd);

	return 0;
}

static const struct of_device_id asm9260_nand_match[] = {
	{
		.compatible   = "alphascale,asm9260-nand",
	},
	{},
};
MODULE_DEVICE_TABLE(of, asm9260_nand_match);

static struct platform_driver asm9260_nand_driver = {
	.probe		= asm9260_nand_probe,
	.remove		= asm9260_nand_remove,
	.driver		= {
		.name	= "asm9260_nand",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(asm9260_nand_match),
	},
};

module_platform_driver(asm9260_nand_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ASM9260 NAND driver");
