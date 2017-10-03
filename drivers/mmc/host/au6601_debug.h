


void au6601_writeb(u8 b, volatile void __iomem *addr);
void au6601_writew(u16 b, volatile void __iomem *addr);
void au6601_writel(u32 b, volatile void __iomem *addr);
u8 au6601_readb(volatile void __iomem *addr);
u16 au6601_readw(volatile void __iomem *addr);
u32 au6601_readl(volatile void __iomem *addr);
#define au6601_write8 au6601_writeb
#define au6601_write16 au6601_writew
#define au6601_write32 au6601_writel

#define au6601_read8 au6601_readb
#define au6601_read16 au6601_readw
#define au6601_read32 au6601_readl

void au6601_write32be(u32 b, volatile void __iomem *addr);
u32 au6601_read32be(volatile void __iomem *addr);
