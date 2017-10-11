
#include <asm/io.h>
#include <asm/bitops.h>
#include <linux/kernel.h>

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

void au6601_writeb(u8 val, volatile void __iomem *addr)
{
	au6601_reg_decode(1, 1, val, addr);
	writeb(val, addr);
}

void au6601_writew(u16 val, volatile void __iomem *addr)
{
	au6601_reg_decode(1, 2, val, addr);
	writew(val, addr);
}

void au6601_writel(u32 val, volatile void __iomem *addr)
{
	au6601_reg_decode(1, 4, val, addr);
	writel(val, addr);
}

u8 au6601_readb(volatile void __iomem *addr)
{
	u8 val;
	val = readb(addr);
	au6601_reg_decode(0, 1, val, addr);
	return val;
}

u16 au6601_readw(volatile void __iomem *addr)
{
	u16 val;
	val = readw(addr);
	au6601_reg_decode(0, 2, val, addr);
	return val;
}

u32 au6601_readl(volatile void __iomem *addr)
{
	u32 val;
	val = readl(addr);
	au6601_reg_decode(0, 4, val, addr);
	return val;
}
