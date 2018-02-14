// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2014-2018 Oleksij Rempel <linux@rempel-privat.de>
 *
 * Direver for Alcor Micro AU6601 and AU6621 controllers
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
#define PCI_ID_ALCOR_MICRO			0x1AEA
#define PCI_ID_AU6601				0x6601
#define PCI_ID_AU6621				0x6621

#define MHZ_TO_HZ(freq)				((freq) * 1000 * 1000)

#define AU6601_BASE_CLOCK			MHZ_TO_HZ(31)
#define AU6601_MIN_CLOCK			(150 * 1000)
#define AU6601_MAX_CLOCK			MHZ_TO_HZ(208)
#define AU6601_MAX_DMA_SEGMENTS			(8 * 64)
#define AU6601_MAX_PIO_SEGMENTS			1
#define AU6601_MAX_DMA_BLOCK_SIZE		0x1000
#define AU6601_MAX_PIO_BLOCK_SIZE		0x200
#define AU6601_MAX_DMA_BLOCKS			1
#define AU6601_DMA_LOCAL_SEGMENTS		1

/* SDMA phy address. Higer then 0x0800.0000?
 * The au6601 and au6621 have different DMA engines with different issues. One
 * For example au6621 engine is triggered by addr change. No other interaction
 * is needed. This means, if we get two buffers with same address, then engine
 * will stall.
 */
#define AU6601_REG_SDMA_ADDR			0x00
#define AU6601_SDMA_MASK			0xffff0000

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
/* Some power related reg, used together with AU6601_OUTPUT_ENABLE */
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
#define AU6601_MS_CARD_WP			BIT(3)
#define AU6601_SD_CARD_WP			BIT(0)

/* ???
 *  same register values are used for:
 *  - AU6601_OUTPUT_ENABLE
 *  - AU6601_POWER_CONTROL
 */
#define AU6601_ACTIVE_CTRL			0x75
#define AU6601_XD_CARD				BIT(4)
/* AU6601_MS_CARD_ACTIVE - will cativate MS card section? */
#define AU6601_MS_CARD				BIT(3)
#define AU6601_SD_CARD				BIT(0)

/* card slot state. It should automatically detect type of
 * the card
 */
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
#define	AU6601_OPT_CMD_LINE_LEVEL		0x80
#define	AU6601_OPT_NCRC_16_CLK			BIT(4)
#define	AU6601_OPT_CMD_NWT			BIT(3)
#define	AU6601_OPT_STOP_CLK			BIT(2)
#define	AU6601_OPT_DDR_MODE			BIT(1)
/* TODO: according to spec, we support 1.2, 1.8 and 3.3.
 * How to enable 1.2?
 */
#define	AU6601_OPT_SD_18V			BIT(0)

/* TODO: how to use clock delay */
#define AU6601_CLK_DELAY			0x86
#define	AU6601_CLK_DATA_POSITIVE_EDGE		0x80
#define	AU6601_CLK_CMD_POSITIVE_EDGE		0x40
#define	AU6601_CLK_POSITIVE_EDGE_ALL \
	AU6601_CLK_CMD_POSITIVE_EDGE | AU6601_CLK_DATA_POSITIVE_EDGE


#define AU6601_REG_INT_STATUS			0x90
#define AU6601_REG_INT_ENABLE			0x94
#define AU6601_INT_DATA_END_BIT_ERR		BIT(22)
#define AU6601_INT_DATA_CRC_ERR			BIT(21)
#define AU6601_INT_DATA_TIMEOUT_ERR		BIT(20)
#define AU6601_INT_CMD_INDEX_ERR		BIT(19)
#define AU6601_INT_CMD_END_BIT_ERR		BIT(18)
#define AU6601_INT_CMD_CRC_ERR			BIT(17)
#define AU6601_INT_CMD_TIMEOUT_ERR		BIT(16)
#define AU6601_INT_ERROR			BIT(15)
#define AU6601_INT_OVER_CURRENT_ERR		BIT(8)
#define AU6601_INT_CARD_INSERT			BIT(7)
#define AU6601_INT_CARD_REMOVE			BIT(6)
#define AU6601_INT_READ_BUF_RDY			BIT(5)
#define AU6601_INT_WRITE_BUF_RDY		BIT(4)
#define AU6601_INT_DMA_END			BIT(3)
#define AU6601_INT_DATA_END			BIT(1)
#define AU6601_INT_CMD_END			BIT(0)

#define AU6601_INT_NORMAL_MASK			0x00007FFF
#define AU6601_INT_ERROR_MASK			0xFFFF8000

#define AU6601_INT_CMD_MASK	(AU6601_INT_CMD_END | \
		AU6601_INT_CMD_TIMEOUT_ERR | AU6601_INT_CMD_CRC_ERR | \
		AU6601_INT_CMD_END_BIT_ERR | AU6601_INT_CMD_INDEX_ERR)
#define AU6601_INT_DATA_MASK	(AU6601_INT_DATA_END | AU6601_INT_DMA_END | \
		AU6601_INT_READ_BUF_RDY | AU6601_INT_WRITE_BUF_RDY | \
		AU6601_INT_DATA_TIMEOUT_ERR | AU6601_INT_DATA_CRC_ERR | \
		AU6601_INT_DATA_END_BIT_ERR)
#define AU6601_INT_ALL_MASK			((u32)-1)

/* MS_CARD mode registers */

#define AU6601_MS_STATUS			0xa0

#define AU6601_MS_BUS_MODE_CTRL			0xa1
#define AU6601_MS_BUS_8BIT_MODE			0x03
#define AU6601_MS_BUS_4BIT_MODE			0x01
#define AU6601_MS_BUS_1BIT_MODE			0x00

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
#define	AU6601_MS_XFER_INT_TIMEOUT_CHK		BIT(2)
#define	AU6601_MS_XFER_DMA_ENABLE		BIT(1)
#define	AU6601_MS_XFER_START			BIT(0)

#define AU6601_MS_DATA_PIN_STATE		0xa4

#define AU6601_MS_INT_STATUS			0xb0
#define AU6601_MS_INT_ENABLE			0xb4
#define AU6601_MS_INT_OVER_CURRENT_ERROR	BIT(23)
#define AU6601_MS_INT_DATA_CRC_ERROR		BIT(21)
#define AU6601_MS_INT_INT_TIMEOUT		BIT(20)
#define AU6601_MS_INT_INT_RESP_ERROR		BIT(19)
#define AU6601_MS_INT_CED_ERROR			BIT(18)
#define AU6601_MS_INT_TPC_TIMEOUT		BIT(16)
#define AU6601_MS_INT_ERROR			BIT(15)
#define AU6601_MS_INT_CARD_INSERT		BIT(7)
#define AU6601_MS_INT_CARD_REMOVE		BIT(6)
#define AU6601_MS_INT_BUF_READ_RDY		BIT(5)
#define AU6601_MS_INT_BUF_WRITE_RDY		BIT(4)
#define AU6601_MS_INT_DMA_END			BIT(3)
#define AU6601_MS_INT_TPC_END			BIT(1)

#define AU6601_MS_INT_DATA_MASK			0x00000038
#define AU6601_MS_INT_TPC_MASK			0x003d8002
#define AU6601_MS_INT_TPC_ERROR			0x003d0000

enum au6601_cookie {
	COOKIE_UNMAPPED,
	COOKIE_PRE_MAPPED,	/* mapped by pre_req() of dwmmc */
	COOKIE_MAPPED,		/* mapped by prepare_data() of dwmmc */
};

struct au6601_dev_cfg {
	u32	flags;
	u8	dma;
};

struct au6601_pll_conf {
	unsigned int clk_src_freq;
	unsigned int clk_src_reg;
	unsigned int max_div;
	unsigned int min_div;
};

struct au6601_host {
	struct pci_dev *pdev;
	struct pci_dev *parent_pdev;
	struct  device *dev;
	void __iomem *iobase;
	void __iomem *dma_trap_virt;
	dma_addr_t dma_trap_phys;

	struct mmc_host *mmc;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	unsigned int dma_on:1;
	unsigned int early_data:1;
	bool use_dma;

	struct mutex cmd_mutex;
	spinlock_t	lock;

	struct delayed_work timeout_work;

	struct sg_mapping_iter sg_miter;	/* SG state for PIO */
	struct scatterlist *sg;
	unsigned int blocks;		/* remaining PIO blocks */
	int sg_count;

	u32			irq_status_sd;
	struct au6601_dev_cfg	*cfg;
	unsigned char		cur_power_mode;

	/* aspm section */
	int pdev_cap_off;
	u8  pdev_aspm_cap;
	int parent_cap_off;
	u8  parent_aspm_cap;
	u8 ext_config_dev_aspm;
};

static const struct au6601_pll_conf au6601_pll_cfg[] = {
	/* MHZ,		CLK src,		max div, min div */
	{ 31250000,	AU6601_CLK_31_25_MHZ,	1,	511},
	{ 48000000,	AU6601_CLK_48_MHZ,	1,	511},
	{125000000,	AU6601_CLK_125_MHZ,	1,	511},
	{384000000,	AU6601_CLK_384_MHZ,	1,	511},
};

static void au6601_send_cmd(struct au6601_host *host,
			    struct mmc_command *cmd);

static void au6601_prepare_data(struct au6601_host *host,
				struct mmc_command *cmd);
static void au6601_finish_data(struct au6601_host *host);
static void au6601_request_complete(struct au6601_host *host,
				    bool cancel_timeout);
static int au6601_get_cd(struct mmc_host *mmc);

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
			      unsigned int addr_short)
{
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

static void au6601_write8(struct au6601_host *host, u8 val,
			  unsigned int addr)
{
	au6601_reg_decode(1, 1, val, addr);
	writeb(val, host->iobase + addr);
}

static void au6601_write16(struct au6601_host *host, u16 val,
			   unsigned int addr)
{
	au6601_reg_decode(1, 2, val, addr);
	writew(val, host->iobase + addr);
}

static void au6601_write32(struct au6601_host *host, u32 val,
			   unsigned int addr)
{
	au6601_reg_decode(1, 4, val, addr);
	writel(val, host->iobase + addr);
}

static u8 au6601_read8(struct au6601_host *host,
		       unsigned int addr)
{
	u8 val;
	val = readb(host->iobase + addr);
	au6601_reg_decode(0, 1, val, addr);
	return val;
}

static u32 au6601_read32(struct au6601_host *host,
			 unsigned int addr)
{
	u32 val;
	val = readl(host->iobase + addr);
	au6601_reg_decode(0, 4, val, addr);
	return val;
}

static u32 au6601_read32be(struct au6601_host *host,
			   unsigned int addr)
{
	u32 val;
	val = ioread32be(host->iobase + addr);
	au6601_reg_decode(0, 4, val, addr);
	return val;
}

static void au6601_write32be(struct au6601_host *host,
			     u32 val, unsigned int addr)
{
	au6601_reg_decode(1, 4, val, addr);
	iowrite32be(val, host->iobase + addr);
}

static inline void au6601_rmw8(struct au6601_host *host, unsigned int addr,
			       u8 clear, u8 set)
{
	u32 var;

	var = au6601_read8(host, addr);
	var &= ~clear;
	var |= set;
	au6601_write8(host, var, addr);
}

/* TODO: find if we have some mainline generic way to do it */
static int pci_find_cap_offset(struct au6601_host *host, struct pci_dev *pci)
{
	int where;
	u8 val8;
	u32 val32;

#define CAP_START_OFFSET	0x34

	where = CAP_START_OFFSET;
	pci_read_config_byte(pci, where, &val8);
	if (!val8) {
		return 0;
	}

	where = (int)val8;
	while (1) {
		pci_read_config_dword(pci, where, &val32);
		if (val32 == 0xffffffff) {
			dev_dbg(host->dev, "pci_find_cap_offset invailid value %x.\n", val32);
			return 0;
		}

		if ((val32 & 0xff) == 0x10) {
			dev_dbg(host->dev, "pcie cap offset: %x\n", where);
			return where;
		}

		if ((val32 & 0xff00) == 0x00) {
			dev_dbg(host->dev, "pci_find_cap_offset invailid value %x.\n", val32);
			break;
		}
		where = (int)((val32 >> 8) & 0xff);
	}

	return 0;
}

/* FIXME: return results are currently ignored */
static int pci_init_check_aspm(struct au6601_host *host)
{
#define PCIE_LINK_CAP_OFFSET	0x0c

	struct pci_dev *pci;
	int where;
	u32 val32;

	dev_dbg(host->dev, "pci_init_check_aspm\n");

	host->pdev_cap_off    = pci_find_cap_offset(host, host->pdev);
	host->parent_cap_off = pci_find_cap_offset(host, host->parent_pdev);

	if ((host->pdev_cap_off == 0) || (host->parent_cap_off == 0)) {
		dev_dbg(host->dev, "pci_cap_off: %x, parent_cap_off: %x\n",
			host->pdev_cap_off, host->parent_cap_off);
		return 0;
	}

	/* link capability */
	pci   = host->pdev;
	where = host->pdev_cap_off + PCIE_LINK_CAP_OFFSET;
	pci_read_config_dword(pci, where, &val32);
	host->pdev_aspm_cap = (u8)(val32 >> 10) & 0x03;

	pci   = host->parent_pdev;
	where = host->parent_cap_off + PCIE_LINK_CAP_OFFSET;
	pci_read_config_dword(pci, where, &val32);
	host->parent_aspm_cap = (u8)(val32 >> 10) & 0x03;

	if (host->pdev_aspm_cap != host->parent_aspm_cap) {
		u8 aspm_cap;
		dev_dbg(host->dev, "host->pdev_aspm_cap: %x\n",
			host->pdev_aspm_cap);
		dev_dbg(host->dev, "host->parent_aspm_cap: %x\n",
			host->parent_aspm_cap);
		aspm_cap = host->pdev_aspm_cap & host->parent_aspm_cap;
		host->pdev_aspm_cap    = aspm_cap;
		host->parent_aspm_cap = aspm_cap;
	}

	dev_dbg(host->dev, "ext_config_dev_aspm: %x, host->pdev_aspm_cap: %x\n",
		host->ext_config_dev_aspm, host->pdev_aspm_cap);
	host->ext_config_dev_aspm &= host->pdev_aspm_cap;
	return 1;
}

static void pci_aspm_ctrl(struct au6601_host *host, u8 aspm_enable)
{
#define PCIE_LINK_CTRL_OFFSET	0x10

	struct pci_dev *pci;
	u8 aspm_ctrl, i;
	int where;
	u32 val32;

	dev_dbg(host->dev, "pci_aspm_ctrl, aspm_enable: %x\n", aspm_enable);

	if ((host->pdev_cap_off == 0) || (host->parent_cap_off == 0)) {
		dev_dbg(host->dev, "pci_cap_off: %x, parent_cap_off: %x\n",
			host->pdev_cap_off, host->parent_cap_off);
		return;
	}

	if (host->pdev_aspm_cap == 0) {
		return;
	}

	aspm_ctrl = 0;
	if (aspm_enable) {
		aspm_ctrl = host->ext_config_dev_aspm;

		if (aspm_ctrl == 0) {
			dev_dbg(host->dev, "aspm_ctrl == 0\n");
			return;
		}
	}

	for (i=0; i < 2; i++) {

		if (i==0) {
			pci   = host->pdev;
			where = host->pdev_cap_off + PCIE_LINK_CTRL_OFFSET;
		}
		else {
			pci   = host->parent_pdev;
			where = host->parent_cap_off + PCIE_LINK_CTRL_OFFSET;
		}

		pci_read_config_dword(pci, where, &val32);
		val32 &= (~0x03);
		val32 |= (aspm_ctrl & host->pdev_aspm_cap);
		pci_write_config_byte(pci, where, (u8)val32);
	}

}

static inline void au6601_mask_sd_irqs(struct au6601_host *host)
{
	au6601_write32(host, 0, AU6601_REG_INT_ENABLE);
}

static inline void au6601_unmask_sd_irqs(struct au6601_host *host)
{
	au6601_write32(host, AU6601_INT_CMD_MASK | AU6601_INT_DATA_MASK |
		  AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE |
		  AU6601_INT_OVER_CURRENT_ERR,
		  AU6601_REG_INT_ENABLE);
}

static inline void au6601_mask_ms_irqs(struct au6601_host *host)
{
	au6601_write32(host, 0, AU6601_MS_INT_ENABLE);
}

static inline void au6601_unmask_ms_irqs(struct au6601_host *host)
{
	au6601_write32(host, 0x3d00fa, AU6601_MS_INT_ENABLE);
}

static void au6601_reset(struct au6601_host *host, u8 val)
{
	int i;

	au6601_write8(host, val | AU6601_BUF_CTRL_RESET,
		      AU6601_REG_SW_RESET);
	for (i = 0; i < 100; i++) {
		if (!(au6601_read8(host, AU6601_REG_SW_RESET) & val))
			return;
		udelay(50);
	}
	dev_err(host->dev, "%s: timeout\n", __func__);
}

static void au6601_data_set_dma(struct au6601_host *host)
{
	struct mmc_data *data = host->data;
	u32 addr, len;

	if (!host->sg_count)
		return;

	if (!host->sg) {
		dev_err(host->dev, "have blocks, but no SG\n");
		return;
	}
	dev_dbg(host->dev, "%s\n", __func__);

	addr = (u32)sg_dma_address(host->sg);
	len = sg_dma_len(host->sg);

	au6601_write32(host, addr, AU6601_REG_SDMA_ADDR);
	host->sg = sg_next(host->sg);
	host->sg_count--;

	if (data->flags & MMC_DATA_WRITE)
		return;
}

static void au6601_trigger_data_transfer(struct au6601_host *host, bool early)
{
	struct mmc_data *data = host->data;
	u8 ctrl = 0;

	dev_dbg(host->dev, "%s\n", __func__);

	if (data->flags & MMC_DATA_WRITE)
		ctrl |= AU6601_DATA_WRITE;

	if (data->host_cookie == COOKIE_MAPPED) {
		if (host->early_data) {
			host->early_data = false;
			return;
		}

		host->early_data = early;

		au6601_data_set_dma(host);
		ctrl |= AU6601_DATA_DMA_MODE;
		host->dma_on = 1;
		au6601_write32(host, AU6601_MAX_DMA_BLOCK_SIZE, AU6601_REG_BLOCK_SIZE);
	} else {
		au6601_write32(host, data->blksz, AU6601_REG_BLOCK_SIZE);
	}

	au6601_write8(host, ctrl | AU6601_DATA_START_XFER, AU6601_DATA_XFER_CTRL);
}

/*****************************************************************************\
 *									     *
 * Core functions							     *
 *									     *
\*****************************************************************************/

static void au6601_trf_block_pio(struct au6601_host *host, bool read)
{
	size_t blksize, len;
	u8 *buf;

	if (!host->blocks)
		return;
	dev_dbg(host->dev, "%s\n", __func__);

	if (host->dma_on) {
		dev_err(host->dev, "configured DMA but got PIO request.\n");
		return;
	}

	if (!!(host->data->flags & MMC_DATA_READ) != read) {
		dev_err(host->dev, "got unexpected direction %i != %i\n",
			!!(host->data->flags & MMC_DATA_READ), read);
	}

	if (!sg_miter_next(&host->sg_miter))
		return;

	blksize = host->data->blksz;
	len = min(host->sg_miter.length, blksize);

	dev_dbg(host->dev, "PIO, %s block size: 0x%lx\n",
		read ? "read" : "write", blksize);

	host->sg_miter.consumed = len;
	host->blocks--;

	buf = host->sg_miter.addr;

	if (read)
		ioread32_rep(host->iobase + AU6601_REG_BUFFER, buf, len >> 2);
	else
		iowrite32_rep(host->iobase + AU6601_REG_BUFFER, buf, len >> 2);

	sg_miter_stop(&host->sg_miter);
}

static void au6601_finish_data(struct au6601_host *host)
{
	struct mmc_data *data;

	data = host->data;
	host->data = NULL;
	host->dma_on = 0;

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
		if (data->error)
			au6601_reset(host, AU6601_RESET_CMD | AU6601_RESET_DATA);

		au6601_send_cmd(host, data->stop);
		udelay(100);
	}

	au6601_request_complete(host, 1);
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
	struct mmc_data *data = cmd->data;
	bool dma = false;

	if (!data)
		return;


	host->data = data;
	host->data->bytes_xfered = 0;
	host->blocks = data->blocks;
	host->sg = data->sg;
	host->sg_count = data->sg_count;
	dev_dbg(host->dev, "prepare DATA: sg %i, blocks: %i\n",
		host->sg_count, host->blocks);

	if (data->host_cookie == COOKIE_MAPPED)
		dma = true;
	else
		au6601_prepare_sg_miter(host);

	au6601_write8(host, 0, AU6601_DATA_XFER_CTRL);
	au6601_trigger_data_transfer(host, true);
}

static void au6601_send_cmd(struct au6601_host *host,
			    struct mmc_command *cmd)
{
	unsigned long timeout;
	u8 ctrl = 0;

	cancel_delayed_work_sync(&host->timeout_work);

	/* TODO: Do we support busy timeout? */
	if (!cmd->data && cmd->busy_timeout)
		timeout = cmd->busy_timeout;
	else
		timeout = 10000;

	host->cmd = cmd;
	au6601_prepare_data(host, cmd);

	dev_dbg(host->dev, "send CMD. opcode: 0x%02x, arg; 0x%08x\n", cmd->opcode,
		cmd->arg);
	au6601_write8(host, cmd->opcode | 0x40, AU6601_REG_CMD_OPCODE);
	au6601_write32be(host, cmd->arg, AU6601_REG_CMD_ARG);

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

	dev_dbg(host->dev, "xfer ctrl: 0x%02x; timeout: %lu\n", ctrl, timeout);
	au6601_write8(host, ctrl | AU6601_CMD_START_XFER,
		 AU6601_CMD_XFER_CTRL);

	schedule_delayed_work(&host->timeout_work, msecs_to_jiffies(timeout));
}

/*****************************************************************************\
 *									     *
 * Interrupt handling							     *
 *									     *
\*****************************************************************************/


static void au6601_err_irq(struct au6601_host *host, u32 intmask)
{
	dev_dbg(host->dev, "ERR IRQ %x\n", intmask);

	if (host->cmd) {
		if (intmask & AU6601_INT_CMD_TIMEOUT_ERR)
			host->cmd->error = -ETIMEDOUT;
		else
			host->cmd->error = -EILSEQ;
	}

	if (host->data) {
		if (intmask & AU6601_INT_DATA_TIMEOUT_ERR)
			host->data->error = -ETIMEDOUT;
		else
			host->data->error = -EILSEQ;

		host->data->bytes_xfered = 0;
	}

	au6601_request_complete(host, 1);
}

static int au6601_cmd_irq_done(struct au6601_host *host, u32 intmask)
{
	intmask &= AU6601_INT_CMD_END;

	if (!intmask)
		return true;

	/* got CMD_END but no CMD is in progress, wake thread an process the
	 * error
	 */
	if (!host->cmd)
		return false;

	dev_dbg(host->dev, "%s %x\n", __func__, intmask);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		struct mmc_command *cmd = host->cmd;

		cmd->resp[0] = au6601_read32be(host, AU6601_REG_CMD_RSP0);
		/* TODO should we check card status here? */
		dev_dbg(host->dev, "RSP0: 0x%04x\n", cmd->resp[0]);
		if (host->cmd->flags & MMC_RSP_136) {
			cmd->resp[1] =
				au6601_read32be(host, AU6601_REG_CMD_RSP1);
			cmd->resp[2] =
				au6601_read32be(host, AU6601_REG_CMD_RSP2);
			cmd->resp[3] =
				au6601_read32be(host, AU6601_REG_CMD_RSP3);
			dev_dbg(host->dev, "RSP1,2,3: 0x%04x 0x%04x 0x%04x\n",
				cmd->resp[1], cmd->resp[2], cmd->resp[3]);
		}

	}

	host->cmd->error = 0;

	/* Processed actual command. */
	if (!host->data)
		return false;

	au6601_trigger_data_transfer(host, false);
	host->cmd = NULL;
	return true;
}

static void au6601_cmd_irq_thread(struct au6601_host *host, u32 intmask)
{
	intmask &= AU6601_INT_CMD_END;

	if (!intmask)
		return;

	if (!host->cmd && intmask & AU6601_INT_CMD_END) {
		dev_err(host->dev,
			"Got command interrupt 0x%08x even though no command operation was in progress.\n",
			intmask);
	}

	dev_dbg(host->dev, "%s %x\n", __func__, intmask);

	/* Processed actual command. */
	if (!host->data)
		au6601_request_complete(host, 1);
	else
		au6601_trigger_data_transfer(host, false);
	host->cmd = NULL;
}

static int au6601_data_irq_done(struct au6601_host *host, u32 intmask)
{
	u32 tmp;

	intmask &= AU6601_INT_DATA_MASK;

	/* nothing here to do */
	if (!intmask)
		return 1;

	dev_dbg(host->dev, "%s %x\n", __func__, intmask);

	/* we was too fast and got DATA_END after it was processed?
	 * lets ignore it for now.
	 */
	if (!host->data && intmask == AU6601_INT_DATA_END)
		return 1;

	/* looks like an error, so lets handle it. */
	if (!host->data)
		return 0;

	tmp = intmask & (AU6601_INT_READ_BUF_RDY | AU6601_INT_WRITE_BUF_RDY
			 | AU6601_INT_DMA_END);
	switch (tmp)
	{
	case 0:
		break;
	case AU6601_INT_READ_BUF_RDY:
		au6601_trf_block_pio(host, true);
		if (!host->blocks)
			return 0;
		au6601_trigger_data_transfer(host, false);
		return 1;
		break;
	case AU6601_INT_WRITE_BUF_RDY:
		au6601_trf_block_pio(host, false);
		if (!host->blocks)
			return 0;
		au6601_trigger_data_transfer(host, false);
		return 1;
		break;
	case AU6601_INT_DMA_END:
		if (!host->sg_count) {
			au6601_write8(host, 0, AU6601_DATA_XFER_CTRL);
			au6601_reset(host, AU6601_RESET_DATA);
			return 0;
		}
		au6601_data_set_dma(host);
		break;
	default:
		dev_err(host->dev, "Got READ_BUF_RDY and WRITE_BUF_RDY at same time\n");
		break;
	}

	/* FIXME: why we get DATA END, even if we still have data and able to
	 * process it?
	 */
	if (intmask & AU6601_INT_DATA_END)
		return 0;

	return 1;
}

static void au6601_data_irq_thread(struct au6601_host *host, u32 intmask)
{
	intmask &= AU6601_INT_DATA_MASK;

	if (!intmask)
		return;

	dev_dbg(host->dev, "DATA thread IRQ %x\n", intmask);

	if (!host->data) {
		dev_err(host->dev,
			"Got data interrupt 0x%08x even though no data operation was in progress.\n",
			(unsigned)intmask);
		au6601_reset(host, AU6601_RESET_DATA);
		return;
	}

	if (au6601_data_irq_done(host, intmask))
		return;

	if ((intmask & AU6601_INT_DATA_END) || !host->blocks ||
	    (host->dma_on && !host->sg_count))
		au6601_finish_data(host);
}

static void au6601_cd_irq(struct au6601_host *host, u32 intmask)
{
	dev_dbg(host->dev, "card %s\n",
		intmask & AU6601_INT_CARD_REMOVE ? "removed" : "inserted");

	if (host->mrq) {
		dev_dbg(host->dev,
			"cancel all pending tasks.\n");

		if (host->data)
			host->data->error = -ENOMEDIUM;

		if (host->cmd)
			host->cmd->error = -ENOMEDIUM;
		else
			host->mrq->cmd->error = -ENOMEDIUM;

		au6601_request_complete(host, 1);
	}

	mmc_detect_change(host->mmc, msecs_to_jiffies(1));
}

static irqreturn_t au6601_irq_thread(int irq, void *d)
{
	struct au6601_host *host = d;
	irqreturn_t ret = IRQ_HANDLED;
	u32 intmask, tmp;

	mutex_lock(&host->cmd_mutex);

	intmask = host->irq_status_sd;

	/* some thing bad */
	if (unlikely(!intmask || AU6601_INT_ALL_MASK == intmask)) {
		dev_dbg(host->dev, "unexpected IRQ: 0x%04x\n",
			 intmask);
		ret = IRQ_NONE;
		goto exit;
	}

	dev_dbg(host->dev, "IRQ %x\n", intmask);

	tmp = intmask & (AU6601_INT_CMD_MASK | AU6601_INT_DATA_MASK);
	if (tmp) {
		if (tmp & AU6601_INT_ERROR_MASK)
			au6601_err_irq(host, tmp);
		else {
			au6601_cmd_irq_thread(host, tmp);
			au6601_data_irq_thread(host, tmp);
		}
		intmask &= ~(AU6601_INT_CMD_MASK | AU6601_INT_DATA_MASK);
	}

	if (intmask & (AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE)) {
		au6601_cd_irq(host, intmask);
		intmask &= ~(AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE);
	}

	if (intmask & AU6601_INT_OVER_CURRENT_ERR) {
		/* TODO: provide over current handler */
		dev_warn(host->dev,
			 "warning: over current detected!\n");
		intmask &= ~AU6601_INT_OVER_CURRENT_ERR;
	}

	if (intmask)
		dev_dbg(host->dev, "got not handled IRQ: 0x%04x\n", intmask);

exit:
	mutex_unlock(&host->cmd_mutex);
	au6601_unmask_sd_irqs(host);
	return ret;
}


static irqreturn_t au6601_irq(int irq, void *d)
{
	struct au6601_host *host = d;
	u32 status, tmp;
	irqreturn_t ret;
	int cmd_done, data_done;

	status = au6601_read32(host, AU6601_REG_INT_STATUS);
	if (!status)
		return IRQ_NONE;

	spin_lock(&host->lock);
	au6601_write32(host, status, AU6601_REG_INT_STATUS);

	tmp = status & (AU6601_INT_READ_BUF_RDY | AU6601_INT_WRITE_BUF_RDY
			| AU6601_INT_DATA_END | AU6601_INT_DMA_END
			| AU6601_INT_CMD_END);
	if (tmp == status) {
		cmd_done = au6601_cmd_irq_done(host, tmp);
		data_done = au6601_data_irq_done(host, tmp);
		/* use fast path for simple tasks */
		if (cmd_done && data_done) {
			ret = IRQ_HANDLED;
			goto au6601_irq_done;
		}
	}

	host->irq_status_sd = status;
	ret = IRQ_WAKE_THREAD;
	au6601_mask_sd_irqs(host);
au6601_irq_done:
	spin_unlock(&host->lock);
	return ret;
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

	au6601_write16(host, (div - 1) << AU6601_PLL_DIV_S
		  | mod << AU6601_PLL_MOD_S | ctrl,
		  AU6601_REG_PLL_CTRL);
}

#else
static void au6601_set_clock(struct au6601_host *host, unsigned int clock)
{
	u16 clk_src;
	u8 clk_div;

	if (clock == 0) {
		//writew(0, AU6601_CLK_SELECT);
		au6601_write16(host, 0, AU6601_CLK_SELECT);
		au6601_write8(host, 0, AU6601_DATA_XFER_CTRL);
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

	au6601_write16(host, clk_src, AU6601_CLK_SELECT);
}
#endif

static void au6601_set_bus_width(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct au6601_host *host = mmc_priv(mmc);

	if (ios->bus_width == MMC_BUS_WIDTH_1) {
		au6601_write8(host, 0, AU6601_REG_BUS_CTRL);
		au6601_rmw8(host, AU6601_CLK_DELAY,
			    AU6601_CLK_POSITIVE_EDGE_ALL, 0);
	} else if (ios->bus_width == MMC_BUS_WIDTH_4) {
		au6601_write8(host, AU6601_BUS_WIDTH_4BIT,
			      AU6601_REG_BUS_CTRL);
		au6601_rmw8(host, AU6601_CLK_DELAY,
			    0, AU6601_CLK_POSITIVE_EDGE_ALL);
	} else
		dev_err(host->dev, "Unknown BUS mode\n");

}

static int au6601_card_busy(struct mmc_host *mmc)
{
	struct au6601_host *host = mmc_priv(mmc);
	u8 status;

	dev_dbg(host->dev, "%s:%i\n", __func__, __LINE__);
	/* Check whether dat[0:3] low */
	status = au6601_read8(host, AU6601_DATA_PIN_STATE);

	return !(status & AU6601_BUS_STAT_DAT_MASK);
}

static int au6601_get_cd(struct mmc_host *mmc)
{
	struct au6601_host *host = mmc_priv(mmc);
	u8 detect;

	detect = au6601_read8(host, AU6601_DETECT_STATUS)
		& AU6601_DETECT_STATUS_M;
	/* check if card is present then send command and data */
	return (AU6601_SD_DETECTED == detect);
}

static int au6601_get_ro(struct mmc_host *mmc)
{
	struct au6601_host *host = mmc_priv(mmc);
	u8 status;

	/* get write protect pin status */
	status = au6601_read8(host, AU6601_INTERFACE_MODE_CTRL);
	dev_dbg(host->dev, "get write protect status %x\n", status);

	return !!(status & AU6601_SD_CARD_WP);
}

static void au6601_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct au6601_host *host = mmc_priv(mmc);

	mutex_lock(&host->cmd_mutex);

	dev_dbg(host->dev, "got request\n");
	host->mrq = mrq;

	/* check if card is present then send command and data */
	if (au6601_get_cd(mmc))
		au6601_send_cmd(host, mrq->cmd);
	else {
		dev_dbg(host->dev, "card is not present\n");
		mrq->cmd->error = -ENOMEDIUM;
		au6601_request_complete(host, 1);
	}

	mutex_unlock(&host->cmd_mutex);
}

static void au6601_pre_req(struct mmc_host *mmc,
			   struct mmc_request *mrq)
{
	struct au6601_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct mmc_command *cmd = mrq->cmd;
	struct scatterlist *sg;
	unsigned int i, sg_len;

	if (!host->use_dma || !data || !cmd)
		return;

	data->host_cookie = COOKIE_UNMAPPED;

	if (cmd->opcode != 18)
		return;

	/*
	 * We don't do DMA on "complex" transfers, i.e. with
	 * non-word-aligned buffers or lengths. Also, we don't bother
	 * with all the DMA setup overhead for short transfers.
	 */
	if (data->blocks * data->blksz < AU6601_MAX_DMA_BLOCK_SIZE)
		return;

	if (data->blksz & 3)
		return;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length != AU6601_MAX_DMA_BLOCK_SIZE)
			return;
	}

	dev_dbg(host->dev, "do pre request\n");
	/* This data might be unmapped at this time */

	sg_len = dma_map_sg(host->dev, data->sg, data->sg_len,
			    mmc_get_dma_dir(data));
	if (sg_len) {
		if (data->flags & MMC_DATA_WRITE)
			dma_sync_sg_for_device(host->dev, data->sg,
					       data->sg_len,
					       mmc_get_dma_dir(data));
		data->host_cookie = COOKIE_MAPPED;
	}
	data->sg_count = sg_len;
}

static void au6601_post_req(struct mmc_host *mmc,
			    struct mmc_request *mrq,
			    int err)
{
	struct au6601_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!host->use_dma || !data)
		return;

	dev_dbg(host->dev, "do post request\n");

	if (data->host_cookie != COOKIE_UNMAPPED) {
		if (!(data->flags & MMC_DATA_WRITE))
			dma_sync_sg_for_cpu(host->dev, data->sg,
					    data->sg_len,
					    mmc_get_dma_dir(data));
		dma_unmap_sg(host->dev,
			     data->sg,
			     data->sg_len,
			     mmc_get_dma_dir(data));
	}

	data->host_cookie = COOKIE_UNMAPPED;
}

static void au6601_set_power_mode(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct au6601_host *host = mmc_priv(mmc);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		au6601_set_clock(host, ios->clock);
		/* set all pins to input */
		au6601_write8(host, 0, AU6601_OUTPUT_ENABLE);
		/* turn of Vcc */
		au6601_write8(host, 0, AU6601_POWER_CONTROL);
		pci_aspm_ctrl(host, 1);
		break;
	case MMC_POWER_UP:
		break;
	case MMC_POWER_ON:
		pci_aspm_ctrl(host, 0);
		au6601_write8(host, AU6601_SD_CARD,
			      AU6601_ACTIVE_CTRL);
		au6601_write8(host, 0, AU6601_OPT);
		au6601_write8(host, 0x20, AU6601_CLK_DELAY);
		au6601_write8(host, 0, AU6601_REG_BUS_CTRL);
		au6601_set_clock(host, ios->clock);
		/* set power on Vcc */
		au6601_write8(host, AU6601_SD_CARD,
			      AU6601_POWER_CONTROL);
		mdelay(20);
		au6601_set_clock(host, ios->clock);

		au6601_write8(host, AU6601_SD_CARD,
			      AU6601_OUTPUT_ENABLE);
		/* The clk will not work on au6621. We need read some thing out */
		au6601_write8(host, AU6601_DATA_WRITE,
			      AU6601_DATA_XFER_CTRL);
		au6601_write8(host, 0x7d, AU6601_TIME_OUT_CTRL);
		mdelay(100);
		break;
	default:
		dev_err(host->dev, "Unknown power parametr\n");
	}
}

static void au6601_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct au6601_host *host = mmc_priv(mmc);

	mutex_lock(&host->cmd_mutex);

	dev_dbg(host->dev, "set ios. bus width: %x, power mode: %x\n",
		ios->bus_width, ios->power_mode);

	if (ios->power_mode != host->cur_power_mode) {
		au6601_set_power_mode(mmc, ios);
		host->cur_power_mode = ios->power_mode;
	} else {
		au6601_set_bus_width(mmc, ios);
		au6601_set_clock(host, ios->clock);
	}

	mutex_unlock(&host->cmd_mutex);
}

static int au6601_signal_voltage_switch(struct mmc_host *mmc,
        struct mmc_ios *ios)
{
	struct au6601_host *host = mmc_priv(mmc);

	mutex_lock(&host->cmd_mutex);

	dev_dbg(host->dev, "%s:%i\n", __func__, __LINE__);
	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		au6601_rmw8(host, AU6601_OPT, AU6601_OPT_SD_18V, 0);
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		au6601_rmw8(host, AU6601_OPT, 0, AU6601_OPT_SD_18V);
		break;
	default:
		/* No signal voltage switch required */
		break;
	}

	mutex_unlock(&host->cmd_mutex);
	return 0;
}

static const struct mmc_host_ops au6601_sdc_ops = {
	.card_busy	= au6601_card_busy,
	.get_cd		= au6601_get_cd,
	.get_ro		= au6601_get_ro,
	.post_req	= au6601_post_req,
	.pre_req	= au6601_pre_req,
	.request	= au6601_request,
	.set_ios	= au6601_set_ios,
	.start_signal_voltage_switch = au6601_signal_voltage_switch,
};

static void au6601_request_complete(struct au6601_host *host,
				    bool cancel_timeout)
{
	struct mmc_request *mrq;

	au6601_reset(host, AU6601_RESET_CMD | AU6601_RESET_DATA);
	/*
	 * If this tasklet gets rescheduled while running, it will
	 * be run again afterwards but without any active request.
	 */
	if (!host->mrq) {
		dev_dbg(host->dev, "nothing to complete\n");
		return;
	}

	if (cancel_timeout)
		cancel_delayed_work_sync(&host->timeout_work);

	mrq = host->mrq;

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
	host->dma_on = 0;

	dev_dbg(host->dev, "request complete\n");
	mmc_request_done(host->mmc, mrq);
}

static void au6601_timeout_timer(struct work_struct *work)
{
	struct delayed_work *d = to_delayed_work(work);
	struct au6601_host *host = container_of(d, struct au6601_host,
						timeout_work);
	mutex_lock(&host->cmd_mutex);

	dev_dbg(host->dev, "triggered timeout\n");
	if (host->mrq) {
		dev_err(host->dev,
			"Timeout waiting for hardware interrupt.\n");

		if (host->data) {
			host->data->error = -ETIMEDOUT;
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;
		}
		au6601_request_complete(host, 0);
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
	mmc->ocr_avail = MMC_VDD_165_195 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED;
	mmc->caps2 = MMC_CAP2_NO_SDIO;
	mmc->ops = &au6601_sdc_ops;

	/* Hardware cannot do scatter lists */
	mmc->max_segs = host->use_dma ? AU6601_MAX_DMA_SEGMENTS
		: AU6601_MAX_PIO_SEGMENTS;
	mmc->max_seg_size = host->use_dma ? AU6601_MAX_DMA_BLOCK_SIZE
		: AU6601_MAX_PIO_BLOCK_SIZE;

	mmc->max_blk_size = mmc->max_seg_size;
	mmc->max_blk_count = mmc->max_segs;

	mmc->max_req_size = mmc->max_seg_size * mmc->max_segs;
}

static void au6601_hw_init(struct au6601_host *host)
{
	struct au6601_dev_cfg *cfg = host->cfg;

	au6601_reset(host, AU6601_RESET_CMD | AU6601_RESET_DATA);

	au6601_write8(host, 0, AU6601_DMA_BOUNDARY);
	au6601_write8(host, AU6601_SD_CARD, AU6601_ACTIVE_CTRL);

	au6601_write8(host, AU6601_BUS_WIDTH_1BIT, AU6601_REG_BUS_CTRL);

#if 1
	/* TODO: actual meaning of this values? How it should be used? */
	au6601_write8(host, 0x44, AU6601_PAD_DRIVE0);
	au6601_write8(host, 0x44, AU6601_PAD_DRIVE1);
	au6601_write8(host, 0x00, AU6601_PAD_DRIVE2);

	/* set default phase value */
	/* TODO: same here: actual meaning of this values?
	 * How it should be used? */
	au6601_write8(host, 0x20, AU6601_CLK_DELAY);
#endif
	/* for 6601 - dma_boundary; for 6621 - dma_page_cnt */
	au6601_write8(host, cfg->dma, AU6601_DMA_BOUNDARY);

	au6601_write8(host, 0, AU6601_OUTPUT_ENABLE);
	au6601_write8(host, 0, AU6601_POWER_CONTROL);
	pci_aspm_ctrl(host, 1);

	host->dma_on = 0;

	au6601_write8(host, AU6601_DETECT_EN, AU6601_DETECT_STATUS);
	/* now we should be safe to enable IRQs */
	au6601_unmask_sd_irqs(host);
	/* currently i don't know how to properly handle MS IRQ
	 * and HW to test it. */
	au6601_mask_ms_irqs(host);
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
	host->parent_pdev = pdev->bus->self;
	host->dev = &pdev->dev;
	host->cfg = cfg;
	host->cur_power_mode = MMC_POWER_UNDEFINED;
	//host->use_dma = false;
	host->use_dma = true;

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

	ret = pci_set_dma_mask(host->pdev, AU6601_SDMA_MASK);
	if (ret) {
		dev_err(host->dev, "Failed to set DMA mask\n");
		goto error_release_regions;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, host);
	pci_init_check_aspm(host);

	spin_lock_init(&host->lock);
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
	au6601_mask_sd_irqs(host);
	au6601_mask_ms_irqs(host);

	au6601_reset(host, AU6601_RESET_CMD | AU6601_RESET_DATA);

	au6601_write8(host, 0, AU6601_DETECT_STATUS);

	au6601_write8(host, 0, AU6601_OUTPUT_ENABLE);
	au6601_write8(host, 0, AU6601_POWER_CONTROL);

	au6601_write8(host, 0, AU6601_OPT);
	pci_aspm_ctrl(host, 1);
}

static void au6601_pci_remove(struct pci_dev *pdev)
{
	struct au6601_host *host;

	host = pci_get_drvdata(pdev);

	flush_delayed_work(&host->timeout_work);

	mmc_remove_host(host->mmc);

	au6601_hw_uninit(host);

	mmc_free_host(host->mmc);

	pci_release_regions(pdev);
	pci_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM_SLEEP
static int au6601_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct au6601_host *host = pci_get_drvdata(pdev);

	flush_delayed_work(&host->timeout_work);
	au6601_hw_uninit(host);
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

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("PCI driver for Alcor Micro AU6601 Secure Digital Host Controller Interface");
MODULE_LICENSE("GPL");
