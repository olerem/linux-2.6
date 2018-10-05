// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Oleksij Rempel <linux@rempel-privat.de>
 *
 * Direver for Alcor Micro AU6601 and AU6621 controllers
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <linux/alcor_pci.h>

static bool msi_en = true;
module_param(msi_en, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(msi_en, "Enable MSI");

static unsigned use_dma = 1;
module_param(use_dma, uint, 0);
MODULE_PARM_DESC(use_dma, "Whether to use DMA or not. Default = 1");

static DEFINE_IDR(alcor_pci_idr);
static DEFINE_SPINLOCK(alcor_pci_lock);

static struct mfd_cell alcor_pci_cells[] = {
	[ALCOR_SD_CARD] = {
		.name = DRV_NAME_ALCOR_PCI_SDMMC,
	},
	[ALCOR_MS_CARD] = {
		.name = DRV_NAME_ALCOR_PCI_MS,
	},
};

static const struct alcor_dev_cfg alcor_cfg = {
	.dma = 0,
};

static const struct alcor_dev_cfg au6621_cfg = {
	.dma = 1,
};

static const struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(PCI_ID_ALCOR_MICRO, PCI_ID_AU6601),
		.driver_data = (kernel_ulong_t)&alcor_cfg },
	{ PCI_DEVICE(PCI_ID_ALCOR_MICRO, PCI_ID_AU6621),
		.driver_data = (kernel_ulong_t)&au6621_cfg },
	{ },
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static void alcor_reg_decode(int write, int size, u32 val,
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

void alcor_write8(struct alcor_pci_priv *priv, u8 val,
			  unsigned int addr)
{
	alcor_reg_decode(1, 1, val, addr);
	writeb(val, priv->iobase + addr);
}
EXPORT_SYMBOL_GPL(alcor_write8);

void alcor_write16(struct alcor_pci_priv *priv, u16 val,
			   unsigned int addr)
{
	alcor_reg_decode(1, 2, val, addr);
	writew(val, priv->iobase + addr);
}
EXPORT_SYMBOL_GPL(alcor_write16);

void alcor_write32(struct alcor_pci_priv *priv, u32 val,
			   unsigned int addr)
{
	alcor_reg_decode(1, 4, val, addr);
	writel(val, priv->iobase + addr);
}
EXPORT_SYMBOL_GPL(alcor_write32);

u8 alcor_read8(struct alcor_pci_priv *priv,
		       unsigned int addr)
{
	u8 val;
	val = readb(priv->iobase + addr);
	alcor_reg_decode(0, 1, val, addr);
	return val;
}
EXPORT_SYMBOL_GPL(alcor_read8);

u32 alcor_read32(struct alcor_pci_priv *priv,
			 unsigned int addr)
{
	u32 val;
	val = readl(priv->iobase + addr);
	alcor_reg_decode(0, 4, val, addr);
	return val;
}
EXPORT_SYMBOL_GPL(alcor_read32);

u32 alcor_read32be(struct alcor_pci_priv *priv,
			   unsigned int addr)
{
	u32 val;
	val = ioread32be(priv->iobase + addr);
	alcor_reg_decode(0, 4, val, addr);
	return val;
}
EXPORT_SYMBOL_GPL(alcor_read32be);

void alcor_write32be(struct alcor_pci_priv *priv,
			     u32 val, unsigned int addr)
{
	alcor_reg_decode(1, 4, val, addr);
	iowrite32be(val, priv->iobase + addr);
}
EXPORT_SYMBOL_GPL(alcor_write32be);

static int pci_find_cap_offset(struct alcor_pci_priv *priv, struct pci_dev *pci)
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
			dev_dbg(priv->dev, "pci_find_cap_offset invailid value %x.\n", val32);
			return 0;
		}

		if ((val32 & 0xff) == 0x10) {
			dev_dbg(priv->dev, "pcie cap offset: %x\n", where);
			return where;
		}

		if ((val32 & 0xff00) == 0x00) {
			dev_dbg(priv->dev, "pci_find_cap_offset invailid value %x.\n", val32);
			break;
		}
		where = (int)((val32 >> 8) & 0xff);
	}

	return 0;
}

/* FIXME: return results are currently ignored */
static int pci_init_check_aspm(struct alcor_pci_priv *priv)
{
#define PCIE_LINK_CAP_OFFSET	0x0c

	struct pci_dev *pci;
	int where;
	u32 val32;

	dev_dbg(priv->dev, "pci_init_check_aspm\n");

	priv->pdev_cap_off    = pci_find_cap_offset(priv, priv->pdev);
	priv->parent_cap_off = pci_find_cap_offset(priv, priv->parent_pdev);

	if ((priv->pdev_cap_off == 0) || (priv->parent_cap_off == 0)) {
		dev_dbg(priv->dev, "pci_cap_off: %x, parent_cap_off: %x\n",
			priv->pdev_cap_off, priv->parent_cap_off);
		return 0;
	}

	/* link capability */
	pci   = priv->pdev;
	where = priv->pdev_cap_off + PCIE_LINK_CAP_OFFSET;
	pci_read_config_dword(pci, where, &val32);
	priv->pdev_aspm_cap = (u8)(val32 >> 10) & 0x03;

	pci   = priv->parent_pdev;
	where = priv->parent_cap_off + PCIE_LINK_CAP_OFFSET;
	pci_read_config_dword(pci, where, &val32);
	priv->parent_aspm_cap = (u8)(val32 >> 10) & 0x03;

	if (priv->pdev_aspm_cap != priv->parent_aspm_cap) {
		u8 aspm_cap;
		dev_dbg(priv->dev, "priv->pdev_aspm_cap: %x\n",
			priv->pdev_aspm_cap);
		dev_dbg(priv->dev, "priv->parent_aspm_cap: %x\n",
			priv->parent_aspm_cap);
		aspm_cap = priv->pdev_aspm_cap & priv->parent_aspm_cap;
		priv->pdev_aspm_cap    = aspm_cap;
		priv->parent_aspm_cap = aspm_cap;
	}

	dev_dbg(priv->dev, "ext_config_dev_aspm: %x, priv->pdev_aspm_cap: %x\n",
		priv->ext_config_dev_aspm, priv->pdev_aspm_cap);
	priv->ext_config_dev_aspm &= priv->pdev_aspm_cap;
	return 1;
}

static void pci_aspm_ctrl(struct alcor_pci_priv *priv, u8 aspm_enable)
{
#define PCIE_LINK_CTRL_OFFSET	0x10

	struct pci_dev *pci;
	u8 aspm_ctrl, i;
	int where;
	u32 val32;

	dev_dbg(priv->dev, "pci_aspm_ctrl, aspm_enable: %x\n", aspm_enable);

	if ((priv->pdev_cap_off == 0) || (priv->parent_cap_off == 0)) {
		dev_dbg(priv->dev, "pci_cap_off: %x, parent_cap_off: %x\n",
			priv->pdev_cap_off, priv->parent_cap_off);
		return;
	}

	if (priv->pdev_aspm_cap == 0) {
		return;
	}

	aspm_ctrl = 0;
	if (aspm_enable) {
		aspm_ctrl = priv->ext_config_dev_aspm;

		if (aspm_ctrl == 0) {
			dev_dbg(priv->dev, "aspm_ctrl == 0\n");
			return;
		}
	}

	for (i=0; i < 2; i++) {

		if (i==0) {
			pci   = priv->pdev;
			where = priv->pdev_cap_off + PCIE_LINK_CTRL_OFFSET;
		}
		else {
			pci   = priv->parent_pdev;
			where = priv->parent_cap_off + PCIE_LINK_CTRL_OFFSET;
		}

		pci_read_config_dword(pci, where, &val32);
		val32 &= (~0x03);
		val32 |= (aspm_ctrl & priv->pdev_aspm_cap);
		pci_write_config_byte(pci, where, (u8)val32);
	}

}

static inline void alcor_mask_sd_irqs(struct alcor_pci_priv *priv)
{
	alcor_write32(priv, 0, AU6601_REG_INT_ENABLE);
}

static inline void alcor_unmask_sd_irqs(struct alcor_pci_priv *priv)
{
	alcor_write32(priv, AU6601_INT_CMD_MASK | AU6601_INT_DATA_MASK |
		  AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE |
		  AU6601_INT_OVER_CURRENT_ERR,
		  AU6601_REG_INT_ENABLE);
}

static inline void alcor_mask_ms_irqs(struct alcor_pci_priv *priv)
{
	alcor_write32(priv, 0, AU6601_MS_INT_ENABLE);
}

static inline void alcor_unmask_ms_irqs(struct alcor_pci_priv *priv)
{
	alcor_write32(priv, 0x3d00fa, AU6601_MS_INT_ENABLE);
}

static int alcor_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct alcor_dev_cfg *cfg;
	struct alcor_pci_priv *priv;
	int ret, i, bar = 0;

	dev_info(&pdev->dev, "AU6601 controller found [%04x:%04x] (rev %x)\n",
		 (int)pdev->vendor, (int)pdev->device, (int)pdev->revision);
	cfg = (void *)ent->driver_data;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	idr_preload(GFP_KERNEL);
	spin_lock(&alcor_pci_lock);
	ret = idr_alloc(&alcor_pci_idr, priv, 0, 0, GFP_NOWAIT);
	if (ret >= 0)
		priv->id = ret;
	spin_unlock(&alcor_pci_lock);
	idr_preload_end();
	if (ret < 0)
		goto error_release_regions;

	priv->pdev = pdev;
	priv->parent_pdev = pdev->bus->self;
	priv->dev = &pdev->dev;
	priv->cfg = cfg;
	priv->irq = pdev->irq;

	ret = pci_request_regions(pdev, DRV_NAME_ALCOR_PCI);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request region\n");
		return -ENOMEM;
	}

	if (!(pci_resource_flags(pdev, bar) & IORESOURCE_MEM)) {
		dev_err(&pdev->dev, "BAR %d is not iomem. Aborting.\n", bar);
		ret = -ENODEV;
		goto error_release_regions;
	}

	priv->iobase = pcim_iomap(pdev, bar, 0);
	if (!priv->iobase) {
		ret = -ENOMEM;
		goto error_release_regions;
	}

	/* make sure irqs are disabled */
	alcor_write32(priv, 0, AU6601_REG_INT_ENABLE);
	alcor_write32(priv, 0, AU6601_MS_INT_ENABLE);

	ret = dma_set_mask_and_coherent(priv->dev, AU6601_SDMA_MASK);
	if (ret) {
		dev_err(priv->dev, "Failed to set DMA mask\n");
		goto error_release_regions;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, priv);
	pci_init_check_aspm(priv);

	for (i = 0; i < ARRAY_SIZE(alcor_pci_cells); i++) {
		alcor_pci_cells[i].platform_data = priv;
		alcor_pci_cells[i].pdata_size = sizeof(*priv);
	}
	ret = mfd_add_devices(&pdev->dev, priv->id, alcor_pci_cells,
			ARRAY_SIZE(alcor_pci_cells), NULL, 0, NULL);
	if (ret < 0)
		goto error_release_regions;

	pci_aspm_ctrl(priv, 0);

	return 0;

error_release_regions:
	pci_release_regions(pdev);
	return ret;
}

static void alcor_pci_remove(struct pci_dev *pdev)
{
	struct alcor_pci_priv *priv;

	priv = pci_get_drvdata(pdev);

	pci_aspm_ctrl(priv, 1);

	mfd_remove_devices(&pdev->dev);

  spin_lock(&alcor_pci_lock);
	idr_remove(&alcor_pci_idr, priv->id);
	spin_unlock(&alcor_pci_lock);

  pci_release_regions(pdev);
	pci_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM_SLEEP
static int alcor_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct alcor_pci_priv *priv = pci_get_drvdata(pdev);

	pci_aspm_ctrl(priv, 1);
	return 0;
}

static int alcor_resume(struct device *dev)
{

	struct pci_dev *pdev = to_pci_dev(dev);
	struct alcor_pci_priv *priv = pci_get_drvdata(pdev);

	mutex_lock(&priv->cmd_mutex);
	pci_aspm_ctrl(priv, 0);
	mutex_unlock(&priv->cmd_mutex);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(alcor_pm_ops, alcor_suspend, alcor_resume);

static struct pci_driver alcor_driver = {
	.name	=	DRV_NAME_ALCOR_PCI,
	.id_table =	pci_ids,
	.probe	=	alcor_pci_probe,
	.remove =	alcor_pci_remove,
	.driver	=	{
		.pm	= &alcor_pm_ops
	},
};

module_pci_driver(alcor_driver);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("PCI driver for Alcor Micro AU6601 Secure Digital Host Controller Interface");
MODULE_LICENSE("GPL");
