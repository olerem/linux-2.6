/*
 * Copyright (C) 2014 Oleksij Rempel.
 *
 * Authors: Oleksij Rempel <linux@rempel-privat.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>

#define DRVNAME					"au6601-pci"
#define PCI_ID_ALCOR_MICRO			0x1aea
#define PCI_ID_AU6601				0x6601
#define PCI_ID_AU6621				0x6621

#define MHZ_TO_HZ(freq)				((freq) * 1000 * 1000)

#define AU6601_BASE_CLOCK			MHZ_TO_HZ(31)
#define AU6601_MIN_CLOCK			(150 * 1000)
#define AU6601_MAX_CLOCK			MHZ_TO_HZ(208)
#define AU6601_MAX_SEGMENTS			512
#define AU6601_MAX_BLOCK_LENGTH			512
#define AU6601_MAX_DMA_BLOCKS			8
#define AU6601_DMA_LOCAL_SEGMENTS		3
#define AU6601_MAX_BLOCK_COUNT			65536

/* SDMA phy address. Higer then 0x0800.0000? */
#define AU6601_REG_SDMA_ADDR			0x00
#define AU6601_SDMA_MASK			0xfffffe00

#define AU6601_DMA_BOUNDARY			0x05
#define AU6621_DMA_PAGE_CNT			0x05
/* PIO */
#define AU6601_REG_BUFFER			0x08
/* ADMA ctrl? AU6621 only. */
#define AU6621_DMA_CTRL				0x0c
#define  AU6621_DMA_ENABLE			BIT(0)
/* ADMA phy address. AU6621 only. */
#define REG_10					0x10
/* CMD index */
#define AU6601_REG_CMD_OPCODE			0x23
/* CMD parametr */
#define AU6601_REG_CMD_ARG			0x24
/* CMD response 4x4 Bytes */
#define AU6601_REG_CMD_RSP0			0x30
#define AU6601_REG_CMD_RSP1			0x34
#define AU6601_REG_CMD_RSP2			0x38
#define AU6601_REG_CMD_RSP3			0x3C
/* LED ctrl? */
#define REG_51					0x51
/* ??? */
#define REG_52					0x52
/* LED related? Always toggled BIT0 */
#define REG_61					0x61
/* Same as REG_61? */
#define REG_63					0x63
/* default timeout set to 125: 125 * 40ms = 5 sec
 * how exactly it is calculated? */
#define AU6601_TIME_OUT_CTRL			0x69
/* Block size for SDMA or PIO */
#define AU6601_REG_BLOCK_SIZE			0x6c
/* Some power related reg, used together with REG_7A */
#define AU6601_POWER_CONTROL			0x70
/* PLL ctrl */
#define AU6601_CLK_SELECT			0x72
#define	AU6601_CLK_OVER_CLK			0x80
#define	AU6601_CLK_384_MHZ			0x30
#define	AU6601_CLK_125_MHZ			0x20
#define	AU6601_CLK_48_MHZ			0x10
#define	AU6601_CLK_EXT_PLL			0x04
#define AU6601_CLK_X2_MODE			0x02
#define AU6601_CLK_ENABLE			0x01
#define AU6601_CLK_31_25_MHZ			0x00

#define AU6601_CLK_DIVIDER			0x73

#if 1
/* recheck clock */
#define AU6601_REG_PLL_CTRL	0x72
 #define AU6601_PLL_DIV8_MASK	0xff
 #define AU6601_PLL_DIV4_MASK	0xf
 #define AU6601_PLL_DIV_S	8
 #define AU6601_PLL_MOD4	0xb	/* x 13,5 */
 #define AU6601_PLL_MOD3	0x3	/* x 12,5 */
 #define AU6601_PLL_MOD2	0x2	/* x 4	  */
 #define AU6601_PLL_MOD1	0x1	/* x 1,5  */
 #define AU6601_PLL_MOD0	0x0	/* x 1	  */
 #define AU6601_PLL_MOD_S	4
 #define AU6601_PLL_EN		BIT(0)
#endif

#define AU6601_INTERFACE_MODE_CTRL		0x74
#define AU6601_DLINK_MODE			0x80
#define	AU6601_INTERRUPT_DELAY_TIME		0x40
#define	AU6601_SIGNAL_REQ_CTRL			0x30
#define	AU6601_WRITE_PROTECT			BIT(0)

/* ??? */
#define AU6601_ACTIVE_CTRL			0x75
#define AU6601_XD_CARD_ACTIVE			BIT(4)
/* AU6601_MS_CARD_ACTIVE - will cativate MS card section? */
#define AU6601_MS_CARD_ACTIVE			BIT(3)
#define AU6601_SD_CARD_ACTIVE			BIT(0)

/* card slot state? */
#define AU6601_DETECT_STATUS			0x76
#define AU6601_DETECT_EN			BIT(7)
#define AU6601_MS_DETECTED			BIT(3)
#define AU6601_SD_DETECTED			BIT(0)
#define AU6601_DETECT_STATUS_M			0xf
/* ??? */
#define REG_77					0x77
/* looks like soft reset? */
#define AU6601_REG_SW_RESET			0x79
#define AU6601_BUF_CTRL_RESET			BIT(7)
#define AU6601_RESET_DATA			BIT(3)
#define AU6601_RESET_CMD			BIT(0)

#define AU6601_OUTPUT_ENABLE			0x7a

#define AU6601_PAD_DRIVE0			0x7b
#define AU6601_PAD_DRIVE1			0x7c
#define AU6601_PAD_DRIVE2			0x7d
/* read EEPROM? */
#define AU6601_FUNCTION				0x7f

#define AU6601_CMD_XFER_CTRL			0x81
#define	AU6601_CMD_17_BYTE_CRC			0xc0
#define	AU6601_CMD_6_BYTE_WO_CRC		0x80
#define	AU6601_CMD_6_BYTE_CRC			0x40
#define	AU6601_CMD_START_XFER			0x20
#define	AU6601_CMD_STOP_WAIT_RDY		0x10
#define	AU6601_CMD_NO_RESP			0x00

#define AU6601_REG_BUS_CTRL			0x82
#define AU6601_BUS_WIDTH_4BIT			0x20
#define AU6601_BUS_WIDTH_8BIT			0x10
#define AU6601_BUS_WIDTH_1BIT			0x00

#define AU6601_DATA_XFER_CTRL			0x83
#define AU6601_DATA_WRITE			BIT(7)
#define AU6601_DATA_DMA_MODE			BIT(6)
#define AU6601_DATA_START_XFER			BIT(0)

#define AU6601_DATA_PIN_STATE			0x84
#define AU6601_BUS_STAT_CMD			BIT(15)
/* BIT(4) - BIT(7) are permanently 1.
 * May be reseved or not attached DAT4-DAT7 */
#define AU6601_BUS_STAT_DAT3			BIT(3)
#define AU6601_BUS_STAT_DAT2			BIT(2)
#define AU6601_BUS_STAT_DAT1			BIT(1)
#define AU6601_BUS_STAT_DAT0			BIT(0)
#define AU6601_BUS_STAT_DAT_MASK		0xf

#define AU6601_OPT				0x85
/* line level here?? really? not in AU6601_DATA_PIN_STATE? */
#define	AU6601_OPT_CMD_LINE_LEVEL		0x80
#define	AU6601_OPT_NCRC_16_CLK			BIT(4)
#define	AU6601_OPT_CMD_NWT			BIT(3)
#define	AU6601_OPT_STOP_CLK			BIT(2)
#define	AU6601_OPT_DDR_MODE			BIT(1)
#define	AU6601_OPT_SD_18V			BIT(0)

#define AU6601_CLK_DELAY			0x86
#define	AU6601_CLK_DATA_POSITIVE_EDGE		0x80
#define	AU6601_CLK_CMD_POSITIVE_EDGE		0x40

#define AU6601_REG_INT_STATUS			0x90
#define AU6601_REG_INT_ENABLE			0x94
#define AU6601_INT_CMD_END			0x00000001
#define AU6601_INT_DATA_END			0x00000002
#define AU6601_INT_DMA_END			0x00000008
#define AU6601_INT_WRITE_BUF_RDY		0x00000010
#define AU6601_INT_READ_BUF_RDY			0x00000020
#define AU6601_INT_CARD_REMOVE			0x00000040
#define AU6601_INT_CARD_INSERT			0x00000080
#define AU6601_INT_OVER_CURRENT_ERR		0x00000100
#define AU6601_INT_ERROR			0x00008000
#define AU6601_INT_CMD_TIMEOUT_ERR		0x00010000
#define AU6601_INT_CMD_CRC_ERR			0x00020000
#define AU6601_INT_CMD_END_BIT_ERR		0x00040000
#define AU6601_INT_CMD_INDEX_ERR		0x00080000
#define AU6601_INT_DATA_TIMEOUT_ERR		0x00100000
#define AU6601_INT_DATA_CRC_ERR			0x00200000
#define AU6601_INT_DATA_END_BIT_ERR		0x00400000

#define AU6601_INT_NORMAL_MASK			0x00007FFF
#define AU6601_INT_ERROR_MASK			0xFFFF8000

#define AU6601_INT_CMD_MASK	(AU6601_INT_CMD_END | AU6601_INT_CMD_TIMEOUT_ERR | \
		AU6601_INT_CMD_CRC_ERR | AU6601_INT_CMD_END_BIT_ERR | AU6601_INT_CMD_INDEX_ERR)
#define AU6601_INT_DATA_MASK	(AU6601_INT_DATA_END | AU6601_INT_DMA_END | \
		AU6601_INT_READ_BUF_RDY | AU6601_INT_WRITE_BUF_RDY | \
		AU6601_INT_DATA_TIMEOUT_ERR | AU6601_INT_DATA_CRC_ERR | \
		AU6601_INT_DATA_END_BIT_ERR)
#define AU6601_INT_ALL_MASK			((u32)-1)

/* MS_CARD mode registers */

#define AU6601_MS_STATUS			0xa0

#define AU6601_MS_BUS_MODE_CTRL			0xa1
#define AU6601_MS_BUS_1BIT_MODE			0x00
#define AU6601_MS_BUS_4BIT_MODE			0x01
#define AU6601_MS_BUS_8BIT_MODE			0x03

#define AU6601_MS_TPC_CMD			0xa2
#define AU6601_MS_TPC_READ_PAGE_DATA		0x02
#define AU6601_MS_TPC_READ_REG			0x04
#define AU6601_MS_TPC_GET_INT			0x07
#define AU6601_MS_TPC_WRITE_PAGE_DATA		0x0D
#define AU6601_MS_TPC_WRITE_REG			0x0B
#define AU6601_MS_TPC_SET_RW_REG_ADRS		0x08
#define AU6601_MS_TPC_SET_CMD			0x0E
#define AU6601_MS_TPC_EX_SET_CMD		0x09
#define AU6601_MS_TPC_READ_SHORT_DATA		0x03
#define AU6601_MS_TPC_WRITE_SHORT_DATA		0x0C

#define AU6601_MS_TRANSFER_MODE			0xa3
#define	AU6601_MS_XFER_START			0x01
#define	AU6601_MS_XFER_DMA_ENABLE		0x02
#define	AU6601_MS_XFER_INT_TIMEOUT_CHK		0x04

#define AU6601_MS_DATA_PIN_STATE		0xa4

#define AU6601_MS_INT_STATUS			0xb0
#define AU6601_MS_INT_ENABLE			0xb4
#define AU6601_MS_INT_TPC_END			0x00000002
#define AU6601_MS_INT_DMA_END			0x00000008
#define AU6601_MS_INT_BUF_WRITE_RDY		0x00000010
#define AU6601_MS_INT_BUF_READ_RDY		0x00000020
#define AU6601_MS_INT_CARD_REMOVE		0x00000040
#define AU6601_MS_INT_CARD_INSERT		0x00000080
#define AU6601_MS_INT_ERROR			0x00008000

#define AU6601_MS_INT_DATA_MASK			0x00000038

#define AU6601_MS_INT_TPC_TIMEOUT		0x00010000
#define AU6601_MS_INT_CED_ERROR			0x00040000
#define AU6601_MS_INT_INT_RESP_ERROR		0x00080000
#define AU6601_MS_INT_INT_TIMEOUT		0x00100000
#define AU6601_MS_INT_DATA_CRC_ERROR		0x00200000

#define AU6601_MS_INT_OVER_CURRENT_ERROR	0x00800000

#define AU6601_MS_INT_TPC_MASK			0x003d8002
#define AU6601_MS_INT_TPC_ERROR			0x003d0000

struct au6601_dev_cfg {
	u32	flags;
	u8	dma;
};

struct au6601_pll_conf {
	unsigned int ratio;
	unsigned int mod;
	unsigned int max_div;
	unsigned int min_div;
};

struct au6601_host {
	struct pci_dev *pdev;
	struct  device *dev;
	void __iomem *iobase;
	void __iomem *virt_base;
	dma_addr_t phys_base;

	struct mmc_host *mmc;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	unsigned int data_early:1;      /* Data finished before cmd */
	unsigned int dma_on:1;
	unsigned int trigger_dma_dac:1;	/* Trigger Data after Command.
					 * In some cases data ragister
					 * should be triggered after
					 * command was done */

	struct mutex cmd_mutex;

	struct delayed_work timeout_work;

	struct sg_mapping_iter sg_miter;	/* SG state for PIO */
	unsigned int blocks;		/* remaining PIO blocks */
	unsigned int requested_blocks;		/* count of requested */
	int sg_count;	   /* Mapped sg entries */

	u32			irq_status_sd;
	struct au6601_dev_cfg	*cfg;
};

static bool disable_dma;

static const struct au6601_pll_conf au6601_pll_cfg[] = {
	{10,	0x0,	0x1ff,	1},
	{15,	0x1,	0x1ff,	1},
	{40,	0x2,	0xf,	1},
	{125,	0x3,	0xf,	2},
	{135,	0xb,	0xf,	2},
};

static void au6601_send_cmd(struct au6601_host *host,
			    struct mmc_command *cmd);

static void au6601_prepare_data(struct au6601_host *host,
				struct mmc_command *cmd);
static void au6601_finish_data(struct au6601_host *host);
static void au6601_request_complete(struct au6601_host *host);

static const struct au6601_dev_cfg au6601_cfg = {
	.dma = 0,
};

static const struct au6601_dev_cfg au6621_cfg = {
	.dma = 1,
};

static const struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(PCI_ID_ALCOR_MICRO, PCI_ID_AU6601),
		.driver_data = (kernel_ulong_t)&au6601_cfg },
	{ PCI_DEVICE(PCI_ID_ALCOR_MICRO, PCI_ID_AU6621),
		.driver_data = (kernel_ulong_t)&au6621_cfg },
	{ },
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static void au6601_reg_decode(int write, int size, u32 val,
			      volatile void __iomem *addr)
{
	unsigned int addr_short = (unsigned int)addr & 0xff;
	const char *reg;

	switch (addr_short)
	{
	case 0x00: reg = "SDMA_ADDR"; break;
	case 0x05: reg = "DMA_BOUNDARY"; break;
	case 0x08: reg = "PIO_BUFFER"; break;
	case 0x0c: reg = "DMA_CTRL"; break;
	case 0x23: reg = "CMD_OPCODE"; break;
	case 0x24: reg = "CMD_ARG"; break;
	case 0x30: reg = "CMD_RSP0"; break;
	case 0x34: reg = "CMD_RSP1"; break;
	case 0x38: reg = "CMD_RSP2"; break;
	case 0x3C: reg = "CMD_RSP3"; break;
	case 0x69: reg = "TIME_OUT_CTRL"; break;
	case 0x6c: reg = "BLOCK_SIZE"; break;
	case 0x70: reg = "POWER_CONTROL"; break;
	case 0x72: reg = "CLK_SELECT"; break;
	case 0x73: reg = "CLK_DIVIDER"; break;
	case 0x74: reg = "INTERFACE_MODE_CTRL"; break;
	case 0x75: reg = "ACTIVE_CTRL"; break;
	case 0x76: reg = "DETECT_STATUS"; break;
	case 0x79: reg = "SW_RESE"; break;
	case 0x7a: reg = "OUTPUT_ENABLE"; break;
	case 0x7b: reg = "PAD_DRIVE0"; break;
	case 0x7c: reg = "PAD_DRIVE1"; break;
	case 0x7d: reg = "PAD_DRIVE2"; break;
	case 0x7f: reg = "EEPROM"; break;
	case 0x81: reg = "CMD_XFER_CTRL"; break;
	case 0x82: reg = "BUS_CTRL"; break;
	case 0x83: reg = "DATA_XFER_CTRL"; break;
	case 0x84: reg = "DATA_PIN_STATE"; break;
	case 0x85: reg = "OPT"; break;
	case 0x86: reg = "CLK_DELAY"; break;
	case 0x90: reg = "INT_STATUS"; break;
	case 0x94: reg = "INT_ENABLE"; break;
	case 0xa0: reg = "MS_STATUS"; break;
	default: reg = "unkn"; break;
	}

	pr_debug("%s.%i: 0x%02x 0x%08x (%s)\n", write ? "> w" : "< r",
		 size, addr_short, val, reg);
}

static void au6601_write8(u8 val, volatile void __iomem *addr)
{
	au6601_reg_decode(1, 1, val, addr);
	writeb(val, addr);
}

static void au6601_write16(u16 val, volatile void __iomem *addr)
{
	au6601_reg_decode(1, 2, val, addr);
	writew(val, addr);
}

static void au6601_write32(u32 val, volatile void __iomem *addr)
{
	au6601_reg_decode(1, 4, val, addr);
	writel(val, addr);
}

static u8 au6601_read8(volatile void __iomem *addr)
{
	u8 val;
	val = readb(addr);
	au6601_reg_decode(0, 1, val, addr);
	return val;
}

static u32 au6601_read32(volatile void __iomem *addr)
{
	u32 val;
	val = readl(addr);
	au6601_reg_decode(0, 4, val, addr);
	return val;
}

static u32 au6601_read32be(volatile void __iomem *addr)
{
	u32 val;
	val = ioread32be(addr);
	au6601_reg_decode(0, 4, val, addr);
	return val;
}

static void au6601_write32be(u32 val, volatile void __iomem *addr)
{
	au6601_reg_decode(1, 4, val, addr);
	iowrite32be(val, addr);
}

static inline void au6601_rmw(void __iomem *reg, u32 clear, u32 set)
{
	u32 var;

	var = au6601_read32(reg);
	var &= ~clear;
	var |= set;
	au6601_write32(var, reg);
}

static inline void au6601_mask_sd_irqs(struct au6601_host *host)
{
	au6601_write32(0, host->iobase + AU6601_REG_INT_ENABLE);
}

static inline void au6601_unmask_sd_irqs(struct au6601_host *host)
{
	au6601_write32(AU6601_INT_CMD_MASK | AU6601_INT_DATA_MASK |
		  AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE |
		  AU6601_INT_OVER_CURRENT_ERR,
		  host->iobase + AU6601_REG_INT_ENABLE);
}

static inline void au6601_mask_ms_irqs(struct au6601_host *host)
{
	au6601_write32(0, host->iobase + AU6601_MS_INT_ENABLE);
}

static inline void au6601_unmask_ms_irqs(struct au6601_host *host)
{
	au6601_write32(0x3d00fa, host->iobase + AU6601_MS_INT_ENABLE);
}

static void au6601_clear_set_reg86(struct au6601_host *host, u32 clear, u32 set)
{
	au6601_rmw(host->iobase + AU6601_CLK_DELAY, clear, set);
}

/*
 * check if one of data line is pulled down
 */
static inline int au6601_card_busy(struct au6601_host *host)
{
	u8 status;

	status = (au6601_read8(host->iobase + AU6601_DATA_PIN_STATE) &
		AU6601_BUS_STAT_DAT_MASK);
	/* If all data lines are up, then card is not busy */
	if (status == (AU6601_BUS_STAT_DAT0 | AU6601_BUS_STAT_DAT1 |
		       AU6601_BUS_STAT_DAT2 | AU6601_BUS_STAT_DAT3))
		return 0;

	return 1;
}

/* val = 0x1 abort command; 0x8 abort data? */
static void au6601_reset(struct au6601_host *host, u8 val)
{
	int i;

	au6601_write8(val | AU6601_BUF_CTRL_RESET, host->iobase + AU6601_REG_SW_RESET);
	for (i = 0; i < 100; i++) {
		if (!(au6601_read8(host->iobase + AU6601_REG_SW_RESET) & val))
			return;
		udelay(50);
	}
	dev_err(host->dev, "%s: timeout\n", __func__);
}

/*
 * - 0x8	only Vcc is on
 * - 0x1	Vcc and other pins are on
 * - 0x1 | 0x8	like 0x1, but DAT2 is off
 */
static void au6601_set_power(struct au6601_host *host,
			     unsigned int value, unsigned int set)
{
	u8 tmp1, tmp2;

	tmp1 = au6601_read8(host->iobase + AU6601_POWER_CONTROL);
	tmp2 = au6601_read8(host->iobase + AU6601_OUTPUT_ENABLE);
	if (set) {
		au6601_write8(tmp1 | value, host->iobase + AU6601_POWER_CONTROL);
		au6601_write8(tmp2 | value, host->iobase + AU6601_OUTPUT_ENABLE);
	} else {
		au6601_write8(tmp2 & ~value, host->iobase + AU6601_OUTPUT_ENABLE);
		au6601_write8(tmp1 & ~value, host->iobase + AU6601_POWER_CONTROL);
	}
}

static void au6601_trigger_data_transfer(struct au6601_host *host,
		unsigned int dma)
{
	struct mmc_data *data = host->data;
	u8 ctrl = 0;

	if (data->flags & MMC_DATA_WRITE)
		ctrl |= AU6601_DATA_WRITE;

	if (dma) {
		au6601_write32(host->phys_base, host->iobase + AU6601_REG_SDMA_ADDR);
		ctrl |= AU6601_DATA_DMA_MODE;
		host->dma_on = 1;

		if (data->flags & MMC_DATA_WRITE)
			goto done;
		/* prepare first DMA buffer for write operation */
		if (host->blocks > AU6601_MAX_DMA_BLOCKS)
			host->requested_blocks = AU6601_MAX_DMA_BLOCKS;
		else
			host->requested_blocks = host->blocks;

	}

done:
	au6601_write32(data->blksz * host->requested_blocks,
		host->iobase + AU6601_REG_BLOCK_SIZE);
	au6601_write8(ctrl | AU6601_DATA_START_XFER, host->iobase + AU6601_DATA_XFER_CTRL);
}

/*****************************************************************************\
 *									     *
 * Core functions							     *
 *									     *
\*****************************************************************************/

static void au6601_read_block(struct au6601_host *host)
{
	size_t blksize, len, chunk = 0;
	void __iomem *virt_base = host->virt_base;
	u8 *buf;

	blksize = host->data->blksz * host->requested_blocks;
	dev_dbg(host->dev, "read block size: 0x%x, %s \n", blksize,
		host->dma_on ? "DMA" : "PIO");

	while (blksize) {
		if (!sg_miter_next(&host->sg_miter))
			BUG();

		len = min(host->sg_miter.length, blksize);

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = host->sg_miter.addr;

		if (host->dma_on) {
			memcpy_fromio(buf, virt_base, len);
			virt_base += len;
			len = 0;
		} else {
			while (len) {
				u32 scratch;

				if (chunk == 0) {
					scratch = au6601_read32(host->iobase +
							AU6601_REG_BUFFER);
					chunk = 4;
				}

				*buf = scratch & 0xFF;

				buf++;
				scratch >>= 8;
				chunk--;
				len--;
			}
		}
	}

	sg_miter_stop(&host->sg_miter);
}

static void au6601_write_block(struct au6601_host *host)
{
	size_t blksize, len, chunk = 0;
	void __iomem *virt_base = host->virt_base;
	u32 scratch = 0;
	u8 *buf;

	blksize = host->data->blksz * host->requested_blocks;

	dev_dbg(host->dev, "write block size: 0x%x, %s \n", blksize,
		host->dma_on ? "DMA" : "PIO");

	while (blksize) {
		if (!sg_miter_next(&host->sg_miter))
			BUG();

		len = min(host->sg_miter.length, blksize);

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = host->sg_miter.addr;

		if (host->dma_on) {
			memcpy_toio(virt_base, buf, len);
			virt_base += len;
			len = 0;
		} else {
			while (len) {
				scratch |= (u32)*buf << (chunk * 8);

				buf++;
				chunk++;
				len--;

				if ((chunk == 4) || ((len == 0)
						&& (blksize == 0))) {
					au6601_write32(scratch, host->iobase +
						AU6601_REG_BUFFER);
					chunk = 0;
					scratch = 0;
				}
			}
		}
	}

	sg_miter_stop(&host->sg_miter);
}

static void au6601_transfer_data(struct au6601_host *host)
{
	dev_dbg(host->dev, "transfer data.\n");

	if (host->blocks == 0)
		return;

	if (host->data->flags & MMC_DATA_READ)
		au6601_read_block(host);
	else
		au6601_write_block(host);

	host->blocks -= host->requested_blocks;
	if (host->dma_on) {
		host->dma_on = 0;
		if (host->blocks || (!host->blocks &&
				(host->data->flags & MMC_DATA_WRITE)))
			au6601_trigger_data_transfer(host, 1);
		else
			au6601_finish_data(host);
	}
}

static void au6601_finish_command(struct au6601_host *host)
{
	struct mmc_command *cmd = host->cmd;

	dev_dbg(host->dev, "Finish CMD\n");

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = au6601_read32be(host->iobase + AU6601_REG_CMD_RSP0);
		dev_dbg(host->dev, "RSP0: 0x%02\n", cmd->resp[0]);
		if (host->cmd->flags & MMC_RSP_136) {
			cmd->resp[1] =
				au6601_read32be(host->iobase + AU6601_REG_CMD_RSP1);
			cmd->resp[2] =
				au6601_read32be(host->iobase + AU6601_REG_CMD_RSP2);
			cmd->resp[3] =
				au6601_read32be(host->iobase + AU6601_REG_CMD_RSP3);
			dev_dbg(host->dev, "RSP1,2,3: 0x%02 0x%02 0x%02\n",
				cmd->resp[1], cmd->resp[2], cmd->resp[3]);
		}

	}

	host->cmd->error = 0;

	if (host->cmd == host->mrq->sbc) {
		dev_dbg(host->dev, "Finished CMD23, now send actual command.\n");
		host->cmd = NULL;
		au6601_send_cmd(host, host->mrq->cmd);
	} else {
		/* Processed actual command. */
		if (!host->data)
			au6601_request_complete(host);
		else if (host->data_early)
			au6601_finish_data(host);
		else if (host->trigger_dma_dac) {
			host->dma_on = 1;
			au6601_transfer_data(host);
		}

		host->cmd = NULL;
	}
}

static void au6601_finish_data(struct au6601_host *host)
{
	struct mmc_data *data;

	data = host->data;
	host->data = NULL;
	host->dma_on = 0;
	host->trigger_dma_dac = 0;

	dev_dbg(host->dev, "Finish DATA\n");
	/*
	 * The specification states that the block count register must
	 * be updated, but it does not specify at what point in the
	 * data flow. That makes the register entirely useless to read
	 * back so we have to assume that nothing made it to the card
	 * in the event of an error.
	 */
	if (data->error)
		data->bytes_xfered = 0;
	else
		data->bytes_xfered = data->blksz * data->blocks;

	/*
	 * Need to send CMD12 if -
	 * a) open-ended multiblock transfer (no CMD23)
	 * b) error in multiblock transfer
	 */
	if (data->stop &&
	    (data->error ||
	     !host->mrq->sbc)) {

		/*
		 * The controller needs a reset of internal state machines
		 * upon error conditions.
		 */
		if (data->error) {
			au6601_reset(host, AU6601_RESET_CMD);
			au6601_reset(host, AU6601_RESET_DATA);
		}
		au6601_send_cmd(host, data->stop);
	} else
		au6601_request_complete(host);
}

static void au6601_prepare_sg_miter(struct au6601_host *host)
{
	unsigned int flags = SG_MITER_ATOMIC;
	struct mmc_data *data = host->data;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;
	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
}

static void au6601_prepare_data(struct au6601_host *host,
				struct mmc_command *cmd)
{
	unsigned int dma = 0;
	struct mmc_data *data = cmd->data;

	if (!data)
		return;

	dev_dbg(host->dev, "prepare DATA\n");
	/* Sanity checks */
	//BUG_ON(data->blksz * data->blocks > 524288);
	BUG_ON(data->blksz > host->mmc->max_blk_size);
	BUG_ON(data->blocks > AU6601_MAX_BLOCK_COUNT);

	host->data = data;
	host->data_early = 0;
	host->data->bytes_xfered = 0;
	host->requested_blocks = 1;

	au6601_prepare_sg_miter(host);
	host->blocks = data->blocks;

	if (!disable_dma &&
			host->blocks > 1 &&
			data->blksz == host->mmc->max_blk_size) {
		dma = 1;

		if (data->flags & MMC_DATA_WRITE) {
			/* prepare first write buffer */
			/* Don't trigger data transfer now.
			 * DMA may start it too eraly */
			host->trigger_dma_dac = 1;
			return;
		}
	}

	au6601_trigger_data_transfer(host, dma);
}

static void au6601_send_cmd(struct au6601_host *host,
			    struct mmc_command *cmd)
{
	u8 ctrl; /* some mysterious flags and control */
	unsigned long timeout;

	cancel_delayed_work_sync(&host->timeout_work);

	timeout = jiffies;
	if (!cmd->data && cmd->busy_timeout > 9000)
		timeout += DIV_ROUND_UP(cmd->busy_timeout, 1000) * HZ + HZ;
	else
		timeout += 10 * HZ;

	host->cmd = cmd;
	au6601_prepare_data(host, cmd);

	dev_dbg(host->dev, "send CMD. opcode: 0x%02x, arg; 0x%08x\n", cmd->opcode,
		cmd->arg);
	au6601_write8(cmd->opcode | 0x40, host->iobase + AU6601_REG_CMD_OPCODE);
	au6601_write32be(cmd->arg, host->iobase + AU6601_REG_CMD_ARG);

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		ctrl = AU6601_CMD_NO_RESP;
		break;
	case MMC_RSP_R1:
		ctrl = AU6601_CMD_6_BYTE_CRC;
		break;
	case MMC_RSP_R1B:
		ctrl = AU6601_CMD_6_BYTE_CRC | AU6601_CMD_STOP_WAIT_RDY;
		break;
	case MMC_RSP_R2:
		ctrl = AU6601_CMD_17_BYTE_CRC;
		break;
	case MMC_RSP_R3:
		ctrl = AU6601_CMD_6_BYTE_WO_CRC;
		break;
	default:
		dev_err(host->dev, "%s: cmd->flag (0x%02x) is not valid\n",
			mmc_hostname(host->mmc), mmc_resp_type(cmd));
		break;
	}

	dev_dbg(host->dev, "xfer ctrl: 0x%02x; \n", ctrl);
	au6601_write8(ctrl | AU6601_CMD_START_XFER,
		 host->iobase + AU6601_CMD_XFER_CTRL);

	schedule_delayed_work(&host->timeout_work, timeout);
}

/*****************************************************************************\
 *									     *
 * Interrupt handling							     *
 *									     *
\*****************************************************************************/

static void au6601_cmd_irq(struct au6601_host *host, u32 intmask)
{
	if (!host->cmd) {
		dev_err(host->dev,
			"Got command interrupt 0x%08x even though no command operation was in progress.\n",
			intmask);
		return;
	}

	if (intmask & AU6601_INT_CMD_TIMEOUT_ERR)
		host->cmd->error = -ETIMEDOUT;
	else if (intmask & (AU6601_INT_CMD_CRC_ERR | AU6601_INT_CMD_END_BIT_ERR |
			AU6601_INT_CMD_INDEX_ERR))
		host->cmd->error = -EILSEQ;

	if (host->cmd->error) {
		au6601_request_complete(host);
		return;
	}

	/*
	 * The host can send and interrupt when the busy state has
	 * ended, allowing us to wait without wasting CPU cycles.
	 * Unfortunately this is overloaded on the "data complete"
	 * interrupt, so we need to take some care when handling
	 * it.
	 *
	 * Note: The 1.0 specification is a bit ambiguous about this
	 *       feature so there might be some problems with older
	 *       controllers.
	 */
	if (host->cmd->flags & MMC_RSP_BUSY) {
		if (host->cmd->data)
			dev_warn(host->dev,
				 "Cannot wait for busy signal when also doing a data transfer");
	}

	if (intmask & AU6601_INT_CMD_END)
		au6601_finish_command(host);
}

static void au6601_data_irq(struct au6601_host *host, u32 intmask)
{
	if (!host->data) {
		/* FIXME: Ist is same for AU6601
		 * The "data complete" interrupt is also used to
		 * indicate that a busy state has ended. See comment
		 * above in au6601_cmd_irq().
		 */
		if (host->cmd && (host->cmd->flags & MMC_RSP_BUSY)) {
			if (intmask & AU6601_INT_DATA_END) {
				au6601_finish_command(host);
				return;
			}
		}

		dev_err(host->dev,
			"Got data interrupt 0x%08x even though no data operation was in progress.\n",
			(unsigned)intmask);

		if (host->cmd && intmask & AU6601_INT_ERROR_MASK) {
			host->cmd->error = -ETIMEDOUT;
			au6601_request_complete(host);
		}
		return;
	}

	if (intmask & AU6601_INT_DATA_TIMEOUT_ERR)
		host->data->error = -ETIMEDOUT;
	else if (intmask & AU6601_INT_DATA_END_BIT_ERR)
		host->data->error = -EILSEQ;
	else if (intmask & AU6601_INT_DATA_CRC_ERR)
		host->data->error = -EILSEQ;

	if (host->data->error)
		au6601_finish_data(host);
	else {
		if (intmask & (AU6601_INT_READ_BUF_RDY | AU6601_INT_WRITE_BUF_RDY))
			au6601_transfer_data(host);

		if (intmask & AU6601_INT_DATA_END) {
			if (host->cmd) {
				/*
				 * Data managed to finish before the
				 * command completed. Make sure we do
				 * things in the proper order.
				 */
				host->data_early = 1;
			} else if (host->blocks && !host->dma_on) {
				/*
				 * Probably we do multi block operation.
				 * Prepare PIO for next block.
				 */
				au6601_trigger_data_transfer(host, 0);
			} else if (host->blocks && host->dma_on) {
				au6601_transfer_data(host);
			} else {
				if (host->dma_on)
					au6601_transfer_data(host);
				au6601_finish_data(host);
			}
		}
	}
}

static irqreturn_t au6601_irq_thread(int irq, void *d)
{
	struct au6601_host *host = d;
	irqreturn_t ret = IRQ_HANDLED;
	u32 intmask;

	mutex_lock(&host->cmd_mutex);

	intmask = host->irq_status_sd;

	/* some thing bad */
	if (unlikely(!intmask || AU6601_INT_ALL_MASK == intmask)) {

		dev_warn(host->dev, "0xFFFF7FFF got unhandled IRQ with %x\n",
			 intmask);
		ret = IRQ_NONE;
		goto exit;
	}

	dev_dbg(host->dev, "IRQ %x\n", intmask);

	if (intmask & AU6601_INT_CMD_MASK) {
		dev_dbg(host->dev, "CMD IRQ %x\n", intmask);

		au6601_cmd_irq(host, intmask & AU6601_INT_CMD_MASK);
		intmask &= ~AU6601_INT_CMD_MASK;
	}

	if (intmask & AU6601_INT_DATA_MASK) {
		dev_dbg(host->dev, "DATA IRQ %x\n", intmask);
		au6601_data_irq(host, intmask & AU6601_INT_DATA_MASK);
		intmask &= ~AU6601_INT_DATA_MASK;
	}

	if (intmask & (AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE)) {
		/* this check can be remove */
		if (intmask & AU6601_INT_CARD_REMOVE)
			dev_dbg(host->dev, "card removed\n");
		else
			dev_dbg(host->dev, "card inserted\n");

		intmask &= ~(AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE);
		mmc_detect_change(host->mmc, msecs_to_jiffies(200));
	}

	if (intmask & AU6601_INT_OVER_CURRENT_ERR) {
		dev_warn(host->dev,
			 "warning: over current detected!\n");
		intmask &= ~AU6601_INT_OVER_CURRENT_ERR;
	}

	if (intmask & 0xFFFF7FFF) {
		dev_warn(host->dev, "0xFFFF7FFF got unhandled IRQ with %x\n",
			 intmask);
	}

exit:
	mutex_unlock(&host->cmd_mutex);
	au6601_unmask_sd_irqs(host);
	return ret;
}


static irqreturn_t au6601_irq(int irq, void *d)
{
	struct au6601_host *host = d;
	u32 status;

	status = au6601_read32(host->iobase + AU6601_REG_INT_STATUS);

	/* some thing bad */
	if (status) {
		host->irq_status_sd = status;
		au6601_write32(status, host->iobase + AU6601_REG_INT_STATUS);
		au6601_mask_sd_irqs(host);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_NONE;

}

static void au6601_sdc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct au6601_host *host;
	u32 detect;

	host = mmc_priv(mmc);
	mutex_lock(&host->cmd_mutex);

	host->mrq = mrq;

	detect = au6601_read8(host->iobase + AU6601_DETECT_STATUS)
		& AU6601_DETECT_STATUS_M;
	/* check if card is present then send command and data */
	if (AU6601_SD_DETECTED == detect)
		au6601_send_cmd(host, mrq->cmd);
	else {
		if (detect)
			dev_warn(host->dev, "unexpected detect status  %x\n",
				 detect);
		mrq->cmd->error = -ENOMEDIUM;
		au6601_request_complete(host);
	}

	mutex_unlock(&host->cmd_mutex);
}

#if 0
static unsigned int au6601_calc_div(unsigned int clock, unsigned int clock_mod,
		 const struct au6601_pll_conf *cfg)
{
	unsigned int tmp;

	tmp = DIV_ROUND_UP(clock_mod, clock);
	if (tmp > cfg->max_div)
		tmp = cfg->max_div;
	else if (tmp < cfg->min_div)
		tmp = cfg->min_div;

	return tmp;
}

static void au6601_set_clock(struct au6601_host *host, unsigned int clock)
{
	unsigned int clock_out = 0, div = 0, mod = 0, ctrl = AU6601_PLL_EN;
	//int i, diff = MAX_INT;
	int i, diff = 0x7fffffff;

	if (clock == 0) {
		ctrl &= ~AU6601_PLL_EN;
		goto done;
	}

	for (i = 0; i < ARRAY_SIZE(au6601_pll_cfg); i++) {
		int tmp_diff, tmp_clock, tmp_div, tmp_clock_mult;
		const struct au6601_pll_conf *cfg = &au6601_pll_cfg[i];

		tmp_clock_mult = cfg->ratio * (AU6601_BASE_CLOCK / 10);
		tmp_div = au6601_calc_div(clock, tmp_clock_mult, cfg);
		tmp_clock = DIV_ROUND_UP(tmp_clock_mult, tmp_div);
		tmp_diff = clock - tmp_clock;

		if (tmp_diff >= 0 && tmp_diff < diff) {
			diff = tmp_diff;
			mod = cfg->mod;
			div = tmp_div;
			clock_out = tmp_clock;
		}
	}

done:
	dev_dbg(host->dev, "set freq %d, use freq %d, %d, %x\n",
		clock, clock_out, div, mod);

	au6601_write16((div - 1) << AU6601_PLL_DIV_S
		  | mod << AU6601_PLL_MOD_S | ctrl,
		  host->iobase + AU6601_REG_PLL_CTRL);
}

#else
static void au6601_set_clock(struct au6601_host *host, unsigned int clock)
{
	u16 clk_src;
	u8 clk_div;

	if (clock == 0) {
		//writew(0, host->iobase + AU6601_CLK_SELECT);
		au6601_write8(0, host->iobase + AU6601_DATA_XFER_CTRL);
		return;
	}

	if (clock <= MHZ_TO_HZ(1)) {
		clk_src = AU6601_CLK_31_25_MHZ;
		clk_div = 200;
	}

	if ((MHZ_TO_HZ(1) < clock) && (clock < MHZ_TO_HZ(5))) {
		clk_src = AU6601_CLK_31_25_MHZ;
		clk_div = 16;
	}

	if ((MHZ_TO_HZ(5) <= clock) && (clock < MHZ_TO_HZ(10))) {
		clk_src = AU6601_CLK_48_MHZ;
		clk_div = 10;
	}

	if ((MHZ_TO_HZ(10) <= clock) && (clock < MHZ_TO_HZ(20))) {
		clk_src = AU6601_CLK_48_MHZ;
		clk_div = 5;
	}

	if ((MHZ_TO_HZ(20) <= clock) && (clock < MHZ_TO_HZ(25))) {
		clk_src = AU6601_CLK_125_MHZ;
		clk_div = 7;
	}

	if ((MHZ_TO_HZ(25) <= clock) && (clock < MHZ_TO_HZ(40))) {
		clk_src = AU6601_CLK_48_MHZ;
		clk_div = 2;
	}

	if ((MHZ_TO_HZ(40) <= clock) && (clock < MHZ_TO_HZ(50))) {
		clk_src = AU6601_CLK_384_MHZ;
		clk_div = 10;
	}

	if ((MHZ_TO_HZ(50) <= clock) && (clock < MHZ_TO_HZ(60))) {
		clk_src = AU6601_CLK_48_MHZ;
		clk_div = 1;
	}

	if ((MHZ_TO_HZ(60) <= clock) && (clock < MHZ_TO_HZ(80))) {
		clk_src = AU6601_CLK_384_MHZ;
		clk_div = 7;
	}

	if ((MHZ_TO_HZ(80) <= clock) && (clock < MHZ_TO_HZ(100))) {
		clk_src = AU6601_CLK_384_MHZ;
		clk_div = 5;
	}

	if ((MHZ_TO_HZ(100) <= clock) && (clock < MHZ_TO_HZ(130))) {
		clk_src = AU6601_CLK_384_MHZ;
		clk_div = 4;
	}

	if ((MHZ_TO_HZ(130) <= clock) && (clock < MHZ_TO_HZ(194))) {
		clk_src = AU6601_CLK_384_MHZ;
		clk_div = 3;
	}

	if ((MHZ_TO_HZ(194) <= clock) && (clock < MHZ_TO_HZ(208))) {
		clk_src = AU6601_CLK_384_MHZ;
		clk_div = 2;
	}

	if (MHZ_TO_HZ(208) <= clock) {
		clk_src = AU6601_CLK_384_MHZ | AU6601_CLK_OVER_CLK;
		clk_div = 2;
	}

	clk_src |= ((clk_div - 1) << 8);
	clk_src |= AU6601_CLK_ENABLE;

	dev_dbg(host->dev, "set freq %d, use div %d, mod %x\n",
			clock, clk_div, clk_src);

	au6601_write16(clk_src, host->iobase + AU6601_CLK_SELECT);
}
#endif

static void au6601_sdc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct au6601_host *host;

	host = mmc_priv(mmc);
	mutex_lock(&host->cmd_mutex);

	dev_dbg(host->dev, "set ios. bus width: %x, power mode: %x\n",
		ios->bus_width, ios->power_mode);
	if (ios->bus_width == MMC_BUS_WIDTH_1) {
		au6601_write8(0x0,
			 host->iobase + AU6601_REG_BUS_CTRL);
		au6601_clear_set_reg86(host, 0xc0, 0);
	} else if (ios->bus_width == MMC_BUS_WIDTH_4) {
		au6601_write8(AU6601_BUS_WIDTH_4BIT,
			 host->iobase + AU6601_REG_BUS_CTRL);
		au6601_clear_set_reg86(host, 0, 0xc0);
	} else
		dev_err(host->dev, "Unknown BUS mode\n");

	au6601_set_clock(host, ios->clock);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		au6601_set_power(host, 0x1 | 0x8, 0);
		break;
	case MMC_POWER_UP:
		au6601_set_power(host, 0x8, 1);
		break;
	case MMC_POWER_ON:
		au6601_set_power(host, 0x1, 1);
		au6601_set_power(host, 0x8, 0);
		break;
	default:
		dev_err(host->dev, "Unknown power parametr\n");
	}

	mutex_unlock(&host->cmd_mutex);
}

static int au6601_signal_voltage_switch(struct mmc_host *mmc,
        struct mmc_ios *ios)
{
	struct au6601_host *host = mmc_priv(mmc);

	mutex_lock(&host->cmd_mutex);

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		au6601_rmw(host->iobase + AU6601_OPT, AU6601_OPT_SD_18V, 0);
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		au6601_rmw(host->iobase + AU6601_OPT, 0, AU6601_OPT_SD_18V);
		break;
	default:
		/* No signal voltage switch required */
		break;
	}

	mutex_unlock(&host->cmd_mutex);
	return 0;
}

static int au6601_ops_card_busy(struct mmc_host *mmc)
{
	struct au6601_host *host = mmc_priv(mmc);

	return au6601_card_busy(host);
}

static const struct mmc_host_ops au6601_sdc_ops = {
	.request	= au6601_sdc_request,
	.set_ios	= au6601_sdc_set_ios,
	.start_signal_voltage_switch = au6601_signal_voltage_switch,

	.card_busy	= au6601_ops_card_busy,
};

static void au6601_request_complete(struct au6601_host *host)
{
	struct mmc_request *mrq;

	/*
	 * If this tasklet gets rescheduled while running, it will
	 * be run again afterwards but without any active request.
	 */
	if (!host->mrq)
		return;

	cancel_delayed_work_sync(&host->timeout_work);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if ((mrq->cmd && mrq->cmd->error) ||
		 (mrq->data && (mrq->data->error ||
		  (mrq->data->stop && mrq->data->stop->error)))) {

		au6601_reset(host, AU6601_RESET_CMD);
		au6601_reset(host, AU6601_RESET_DATA);
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
	host->dma_on = 0;
	host->trigger_dma_dac = 0;

	mmc_request_done(host->mmc, mrq);
}

static void au6601_timeout_timer(struct work_struct *work)
{
	struct delayed_work *d = to_delayed_work(work);
	struct au6601_host *host = container_of(d, struct au6601_host,
						timeout_work);
	mutex_lock(&host->cmd_mutex);

	if (host->mrq) {
		dev_err(host->dev,
			"Timeout waiting for hardware interrupt.\n");

		if (host->data) {
			host->data->error = -ETIMEDOUT;
			au6601_finish_data(host);
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;

			au6601_request_complete(host);
		}
	}

	mmiowb();
	mutex_unlock(&host->cmd_mutex);
}



static void au6601_init_mmc(struct au6601_host *host)
{
	struct mmc_host *mmc = host->mmc;

	mmc->f_min = AU6601_MIN_CLOCK;
	mmc->f_max = AU6601_MAX_CLOCK;
	/* mesured Vdd: 3.4 and 1.8 */
	mmc->ocr_avail = MMC_VDD_165_195 | MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED
		  | MMC_CAP_WAIT_WHILE_BUSY;
	mmc->ops = &au6601_sdc_ops;

	/* Hardware cannot do scatter lists? */
	mmc->max_segs = AU6601_MAX_SEGMENTS;

	mmc->max_blk_size = AU6601_MAX_BLOCK_LENGTH;
	mmc->max_blk_count = AU6601_MAX_BLOCK_COUNT;

	mmc->max_seg_size = AU6601_MAX_BLOCK_LENGTH * AU6601_MAX_DMA_BLOCKS;
	mmc->max_req_size = mmc->max_seg_size * mmc->max_segs;
}

static void au6601_hw_init(struct au6601_host *host)
{
	struct au6601_dev_cfg *cfg = host->cfg;

	au6601_write8(0, host->iobase + AU6601_INTERFACE_MODE_CTRL);

	au6601_reset(host, AU6601_RESET_CMD);
	au6601_reset(host, AU6601_RESET_DATA);

	au6601_write8(AU6601_SD_CARD_ACTIVE, host->iobase + AU6601_ACTIVE_CTRL);

	au6601_write8(AU6601_BUS_WIDTH_1BIT, host->iobase + AU6601_REG_BUS_CTRL);
	au6601_write8(0, host->iobase + AU6601_DATA_XFER_CTRL);

	au6601_write8(0x7d, host->iobase + AU6601_TIME_OUT_CTRL);
	au6601_write8(0x44, host->iobase + AU6601_PAD_DRIVE0);
	au6601_write8(0x44, host->iobase + AU6601_PAD_DRIVE1);
	au6601_write8(0x00, host->iobase + AU6601_PAD_DRIVE2);

	/* for 6601 - dma_boundary; for 6621 - dma_page_cnt */
	au6601_write8(cfg->dma, host->iobase + AU6601_DMA_BOUNDARY);
	au6601_write8(0x0, host->iobase + AU6601_OPT);

	au6601_set_power(host, 0x1, 0);
	au6601_set_power(host, 0x8, 0);

	host->dma_on = 0;

	/* now we should be safe to enable IRQs */
	au6601_unmask_sd_irqs(host);
	/* currently i don't know how to properly handle MS IRQ
	 * and HW to test it. */
	au6601_mask_ms_irqs(host);

	au6601_write8(AU6601_DETECT_EN, host->iobase + AU6601_DETECT_STATUS);
}

static int au6601_dma_alloc(struct au6601_host *host)
{
	int ret;

	ret = pci_set_dma_mask(host->pdev, AU6601_SDMA_MASK);
	if (ret) {
		dev_err(host->dev, "Failed to set DMA mask\n");
		return ret;
	}

	host->virt_base = dmam_alloc_coherent(host->dev,
		AU6601_MAX_BLOCK_LENGTH * AU6601_MAX_DMA_BLOCKS
		* AU6601_DMA_LOCAL_SEGMENTS,
		&host->phys_base, GFP_KERNEL);

	if (!host->virt_base) {
		dev_err(host->dev, "Failed to alloc DMA\n");
		return -ENOMEM;
	}

	return 0;
}

static int au6601_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct au6601_dev_cfg *cfg;
	struct mmc_host *mmc;
	struct au6601_host *host;
	int ret, bar = 0;

	dev_info(&pdev->dev, "AU6601 controller found [%04x:%04x] (rev %x)\n",
		 (int)pdev->vendor, (int)pdev->device, (int)pdev->revision);
	cfg = (void *)ent->driver_data;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	/* FIXME: create managed version of mmc_alloc_host and use it */
	mmc = mmc_alloc_host(sizeof(struct au6601_host *), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "Can't allocate MMC\n");
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;
	host->dev = &pdev->dev;
	host->cfg = cfg;

	ret = pci_request_regions(pdev, DRVNAME);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request region\n");
		return -ENOMEM;
	}

	if (!(pci_resource_flags(pdev, bar) & IORESOURCE_MEM)) {
		dev_err(&pdev->dev, "BAR %d is not iomem. Aborting.\n", bar);
		ret = -ENODEV;
		goto error_release_regions;
	}

	host->iobase = pcim_iomap(pdev, bar, 0);
	if (!host->iobase) {
		ret = -ENOMEM;
		goto error_release_regions;
	}

	/* make sure irqs are disabled */
	au6601_mask_sd_irqs(host);
	au6601_mask_ms_irqs(host);

	ret = devm_request_threaded_irq(&pdev->dev, pdev->irq,
			au6601_irq, au6601_irq_thread, IRQF_SHARED,
					"au6601", host);

	if (ret) {
		dev_err(&pdev->dev, "Failed to get irq for data line\n");
		ret = -ENOMEM;
		goto error_release_regions;
	}

	ret = au6601_dma_alloc(host);
	if (ret)
		goto error_release_regions;

	pci_set_master(pdev);
	pci_set_drvdata(pdev, host);

	mutex_init(&host->cmd_mutex);
	/*
	 * Init tasklets.
	 */
	INIT_DELAYED_WORK(&host->timeout_work, au6601_timeout_timer);

	au6601_init_mmc(host);
	au6601_hw_init(host);

	mmc_add_host(mmc);
	return 0;

error_release_regions:
	pci_release_regions(pdev);
	return ret;
}

static void au6601_hw_uninit(struct au6601_host *host)
{

	au6601_reset(host, AU6601_RESET_CMD);
	au6601_reset(host, AU6601_RESET_DATA);

	au6601_write8(0x0, host->iobase + AU6601_DETECT_STATUS);
	au6601_mask_sd_irqs(host);
	au6601_mask_ms_irqs(host);

	au6601_set_power(host, 0x1, 0);

	au6601_write8(0x0, host->iobase + AU6601_OPT);

	au6601_set_power(host, 0x8, 0);
}

static void au6601_pci_remove(struct pci_dev *pdev)
{
	struct au6601_host *host;

	host = pci_get_drvdata(pdev);

	mutex_lock(&host->cmd_mutex);

	cancel_delayed_work_sync(&host->timeout_work);

	au6601_hw_uninit(host);
	mutex_unlock(&host->cmd_mutex);

	mmc_remove_host(host->mmc);
	mmc_free_host(host->mmc);

	pci_release_regions(pdev);
	pci_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM_SLEEP
static int au6601_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct au6601_host *host = pci_get_drvdata(pdev);

	mutex_lock(&host->cmd_mutex);
	au6601_hw_uninit(host);
	mutex_unlock(&host->cmd_mutex);
	return 0;
}

static int au6601_resume(struct device *dev)
{

	struct pci_dev *pdev = to_pci_dev(dev);
	struct au6601_host *host = pci_get_drvdata(pdev);

	mutex_lock(&host->cmd_mutex);
	au6601_hw_init(host);
	mutex_unlock(&host->cmd_mutex);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(au6601_pm_ops, au6601_suspend, au6601_resume);

static struct pci_driver au6601_driver = {
	.name	=	DRVNAME,
	.id_table =	pci_ids,
	.probe	=	au6601_pci_probe,
	.remove =	au6601_pci_remove,
	.driver	=	{
		.pm	= &au6601_pm_ops
	},
};

module_pci_driver(au6601_driver);

module_param(disable_dma, bool, S_IRUGO);
MODULE_PARM_DESC(disable_dma, "Disable DMA");

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("PCI driver for Alcor Micro AU6601 Secure Digital Host Controller Interface");
MODULE_LICENSE("GPL");
