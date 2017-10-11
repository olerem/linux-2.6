
#include <asm/io.h>
#include <asm/bitops.h>
#include <linux/kernel.h>

void au6601_writeb(u8 b, volatile void __iomem *addr)
{
	pr_debug("> wb: 0x%02x 0x%x\n", (int)addr & 0xff, b);
	writeb(b, addr);
}

void au6601_writew(u16 b, volatile void __iomem *addr)
{
	pr_debug("> ww: 0x%02x 0x%x\n", (int)addr & 0xff, b);
	writew(b, addr);
}

void au6601_writel(u32 b, volatile void __iomem *addr)
{
	pr_debug("> wl: 0x%02x 0x%x\n", (int)addr & 0xff, b);
	writel(b, addr);
}

u8 au6601_readb(volatile void __iomem *addr)
{
	u8 val;
	val = readb(addr);
	pr_debug("< rb: 0x%02x 0x%x\n", (int)addr & 0xff, val);
	return val;
}

u16 au6601_readw(volatile void __iomem *addr)
{
	u16 val;
	val = readw(addr);
	pr_debug("< rw: 0x%02x 0x%x\n", (int)addr & 0xff, val);
	return val;
}

u32 au6601_readl(volatile void __iomem *addr)
{
	u32 val;
	val = readl(addr);
	pr_debug("< rl: 0x%02x 0x%x\n", (int)addr & 0xff, val);
	return val;
}
