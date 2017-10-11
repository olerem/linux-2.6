
#include <asm/io.h>
#include <asm/bitops.h>
#include <linux/kernel.h>

static void au6601_reg_decode(int write, int size, u32 val,
			      volatile void __iomem *addr)
{
	unsigned int addr_short = (unsigned int)addr & 0xff;

	pr_debug("%s.%i: 0x%02x 0x%x\n", write ? "> w" : "< r",
		 size, val);
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
