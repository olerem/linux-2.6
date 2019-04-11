// SPDX-License-Identifier: GPL-2.0
/*
 *  Driver for the built-in ethernet switch of the Atheros AR7240 SoC
 *  Copyright (c) 2010 Gabor Juhos <juhosg@openwrt.org>
 *  Copyright (c) 2010 Felix Fietkau <nbd@nbd.name>
 */

#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>

#include "ag71xx.h"

#define BITM(_count)	(BIT(_count) - 1)
#define BITS(_shift, _count)	(BITM(_count) << _shift)

#define AR7240_REG_MASK_CTRL		0x00
#define AR7240_MASK_CTRL_REVISION_M	BITM(8)
#define AR7240_MASK_CTRL_VERSION_M	BITM(8)
#define AR7240_MASK_CTRL_VERSION_S	8
#define   AR7240_MASK_CTRL_VERSION_AR7240 0x01
#define   AR7240_MASK_CTRL_VERSION_AR934X 0x02
#define AR7240_MASK_CTRL_SOFT_RESET	BIT(31)

#define AR7240_REG_MAC_ADDR0		0x20
#define AR7240_REG_MAC_ADDR1		0x24

#define AR7240_REG_FLOOD_MASK		0x2c
#define AR7240_FLOOD_MASK_BROAD_TO_CPU	BIT(26)

#define AR7240_REG_GLOBAL_CTRL		0x30
#define AR7240_GLOBAL_CTRL_MTU_M	BITM(11)
#define AR9340_GLOBAL_CTRL_MTU_M	BITM(14)

#define AR7240_REG_VTU			0x0040
#define   AR7240_VTU_OP			BITM(3)
#define   AR7240_VTU_OP_NOOP		0x0
#define   AR7240_VTU_OP_FLUSH		0x1
#define   AR7240_VTU_OP_LOAD		0x2
#define   AR7240_VTU_OP_PURGE		0x3
#define   AR7240_VTU_OP_REMOVE_PORT	0x4
#define   AR7240_VTU_ACTIVE		BIT(3)
#define   AR7240_VTU_FULL		BIT(4)
#define   AR7240_VTU_PORT		BITS(8, 4)
#define   AR7240_VTU_PORT_S		8
#define   AR7240_VTU_VID		BITS(16, 12)
#define   AR7240_VTU_VID_S		16
#define   AR7240_VTU_PRIO		BITS(28, 3)
#define   AR7240_VTU_PRIO_S		28
#define   AR7240_VTU_PRIO_EN		BIT(31)

#define AR7240_REG_VTU_DATA		0x0044
#define   AR7240_VTUDATA_MEMBER		BITS(0, 10)
#define   AR7240_VTUDATA_VALID		BIT(11)

#define AR7240_REG_ATU			0x50
#define AR7240_ATU_FLUSH_ALL		0x1

#define AR7240_REG_AT_CTRL		0x5c
#define AR7240_AT_CTRL_AGE_TIME		BITS(0, 15)
#define AR7240_AT_CTRL_AGE_EN		BIT(17)
#define AR7240_AT_CTRL_LEARN_CHANGE	BIT(18)
#define AR7240_AT_CTRL_RESERVED		BIT(19)
#define AR7240_AT_CTRL_ARP_EN		BIT(20)

#define AR7240_REG_TAG_PRIORITY		0x70

#define AR7240_REG_SERVICE_TAG		0x74
#define AR7240_SERVICE_TAG_M		BITM(16)

#define AR7240_REG_CPU_PORT		0x78
#define AR7240_MIRROR_PORT_S		4
#define AR7240_MIRROR_PORT_M		BITM(4)
#define AR7240_CPU_PORT_EN		BIT(8)

#define AR7240_REG_MIB_FUNCTION0	0x80
#define AR7240_MIB_TIMER_M		BITM(16)
#define AR7240_MIB_AT_HALF_EN		BIT(16)
#define AR7240_MIB_BUSY			BIT(17)
#define AR7240_MIB_FUNC_S		24
#define AR7240_MIB_FUNC_M		BITM(3)
#define AR7240_MIB_FUNC_NO_OP		0x0
#define AR7240_MIB_FUNC_FLUSH		0x1
#define AR7240_MIB_FUNC_CAPTURE		0x3

#define AR7240_REG_MDIO_CTRL		0x98
#define AR7240_MDIO_CTRL_DATA_M		BITM(16)
#define AR7240_MDIO_CTRL_REG_ADDR_S	16
#define AR7240_MDIO_CTRL_PHY_ADDR_S	21
#define AR7240_MDIO_CTRL_CMD_WRITE	0
#define AR7240_MDIO_CTRL_CMD_READ	BIT(27)
#define AR7240_MDIO_CTRL_MASTER_EN	BIT(30)
#define AR7240_MDIO_CTRL_BUSY		BIT(31)

#define AR7240_REG_PORT_BASE(_port)	(0x100 + (_port) * 0x100)

#define AR7240_REG_PORT_STATUS(_port)	(AR7240_REG_PORT_BASE((_port)) + 0x00)
#define AR7240_PORT_STATUS_SPEED_S	0
#define AR7240_PORT_STATUS_SPEED_M	BITM(2)
#define AR7240_PORT_STATUS_SPEED_10	0
#define AR7240_PORT_STATUS_SPEED_100	1
#define AR7240_PORT_STATUS_SPEED_1000	2
#define AR7240_PORT_STATUS_TXMAC	BIT(2)
#define AR7240_PORT_STATUS_RXMAC	BIT(3)
#define AR7240_PORT_STATUS_TXFLOW	BIT(4)
#define AR7240_PORT_STATUS_RXFLOW	BIT(5)
#define AR7240_PORT_STATUS_DUPLEX	BIT(6)
#define AR7240_PORT_STATUS_LINK_UP	BIT(8)
#define AR7240_PORT_STATUS_LINK_AUTO	BIT(9)
#define AR7240_PORT_STATUS_LINK_PAUSE	BIT(10)

#define AR7240_REG_PORT_CTRL(_port)	(AR7240_REG_PORT_BASE((_port)) + 0x04)
#define AR7240_PORT_CTRL_STATE_M	BITM(3)
#define	AR7240_PORT_CTRL_STATE_DISABLED	0
#define AR7240_PORT_CTRL_STATE_BLOCK	1
#define AR7240_PORT_CTRL_STATE_LISTEN	2
#define AR7240_PORT_CTRL_STATE_LEARN	3
#define AR7240_PORT_CTRL_STATE_FORWARD	4
#define AR7240_PORT_CTRL_LEARN_LOCK	BIT(7)
#define AR7240_PORT_CTRL_VLAN_MODE_S	8
#define AR7240_PORT_CTRL_VLAN_MODE_KEEP	0
#define AR7240_PORT_CTRL_VLAN_MODE_STRIP 1
#define AR7240_PORT_CTRL_VLAN_MODE_ADD	2
#define AR7240_PORT_CTRL_VLAN_MODE_DOUBLE_TAG 3
#define AR7240_PORT_CTRL_IGMP_SNOOP	BIT(10)
#define AR7240_PORT_CTRL_HEADER		BIT(11)
#define AR7240_PORT_CTRL_MAC_LOOP	BIT(12)
#define AR7240_PORT_CTRL_SINGLE_VLAN	BIT(13)
#define AR7240_PORT_CTRL_LEARN		BIT(14)
#define AR7240_PORT_CTRL_DOUBLE_TAG	BIT(15)
#define AR7240_PORT_CTRL_MIRROR_TX	BIT(16)
#define AR7240_PORT_CTRL_MIRROR_RX	BIT(17)

#define AR7240_REG_PORT_VLAN(_port)	(AR7240_REG_PORT_BASE((_port)) + 0x08)

#define AR7240_PORT_VLAN_DEFAULT_ID_S	0
#define AR7240_PORT_VLAN_DEST_PORTS_S	16
#define AR7240_PORT_VLAN_MODE_S		30
#define AR7240_PORT_VLAN_MODE_PORT_ONLY	0
#define AR7240_PORT_VLAN_MODE_PORT_FALLBACK	1
#define AR7240_PORT_VLAN_MODE_VLAN_ONLY	2
#define AR7240_PORT_VLAN_MODE_SECURE	3


#define AR7240_REG_STATS_BASE(_port)	(0x20000 + (_port) * 0x100)

#define AR7240_STATS_RXBROAD		0x00
#define AR7240_STATS_RXPAUSE		0x04
#define AR7240_STATS_RXMULTI		0x08
#define AR7240_STATS_RXFCSERR		0x0c
#define AR7240_STATS_RXALIGNERR		0x10
#define AR7240_STATS_RXRUNT		0x14
#define AR7240_STATS_RXFRAGMENT		0x18
#define AR7240_STATS_RX64BYTE		0x1c
#define AR7240_STATS_RX128BYTE		0x20
#define AR7240_STATS_RX256BYTE		0x24
#define AR7240_STATS_RX512BYTE		0x28
#define AR7240_STATS_RX1024BYTE		0x2c
#define AR7240_STATS_RX1518BYTE		0x30
#define AR7240_STATS_RXMAXBYTE		0x34
#define AR7240_STATS_RXTOOLONG		0x38
#define AR7240_STATS_RXGOODBYTE		0x3c
#define AR7240_STATS_RXBADBYTE		0x44
#define AR7240_STATS_RXOVERFLOW		0x4c
#define AR7240_STATS_FILTERED		0x50
#define AR7240_STATS_TXBROAD		0x54
#define AR7240_STATS_TXPAUSE		0x58
#define AR7240_STATS_TXMULTI		0x5c
#define AR7240_STATS_TXUNDERRUN		0x60
#define AR7240_STATS_TX64BYTE		0x64
#define AR7240_STATS_TX128BYTE		0x68
#define AR7240_STATS_TX256BYTE		0x6c
#define AR7240_STATS_TX512BYTE		0x70
#define AR7240_STATS_TX1024BYTE		0x74
#define AR7240_STATS_TX1518BYTE		0x78
#define AR7240_STATS_TXMAXBYTE		0x7c
#define AR7240_STATS_TXOVERSIZE		0x80
#define AR7240_STATS_TXBYTE		0x84
#define AR7240_STATS_TXCOLLISION	0x8c
#define AR7240_STATS_TXABORTCOL		0x90
#define AR7240_STATS_TXMULTICOL		0x94
#define AR7240_STATS_TXSINGLECOL	0x98
#define AR7240_STATS_TXEXCDEFER		0x9c
#define AR7240_STATS_TXDEFER		0xa0
#define AR7240_STATS_TXLATECOL		0xa4

#define AR7240_PORT_CPU		0
#define AR7240_NUM_PORTS	6
#define AR7240_NUM_PHYS		5

#define AR7240_PHY_ID1		0x004d
#define AR7240_PHY_ID2		0xd041

#define AR934X_PHY_ID1		0x004d
#define AR934X_PHY_ID2		0xd042

#define AR7240_MAX_VLANS	16

#define AR934X_REG_OPER_MODE0		0x04
#define   AR934X_OPER_MODE0_MAC_GMII_EN	BIT(6)
#define   AR934X_OPER_MODE0_PHY_MII_EN	BIT(10)

#define AR934X_REG_OPER_MODE1		0x08
#define   AR934X_REG_OPER_MODE1_PHY4_MII_EN	BIT(28)

#define AR934X_REG_FLOOD_MASK		0x2c
#define   AR934X_FLOOD_MASK_MC_DP(_p)	BIT(16 + (_p))
#define   AR934X_FLOOD_MASK_BC_DP(_p)	BIT(25 + (_p))

#define AR934X_REG_QM_CTRL		0x3c
#define   AR934X_QM_CTRL_ARP_EN		BIT(15)

#define AR934X_REG_AT_CTRL		0x5c
#define   AR934X_AT_CTRL_AGE_TIME	BITS(0, 15)
#define   AR934X_AT_CTRL_AGE_EN		BIT(17)
#define   AR934X_AT_CTRL_LEARN_CHANGE	BIT(18)

#define AR934X_MIB_ENABLE		BIT(30)

#define AR934X_REG_PORT_BASE(_port)	(0x100 + (_port) * 0x100)

#define AR934X_REG_PORT_VLAN1(_port)	(AR934X_REG_PORT_BASE((_port)) + 0x08)
#define   AR934X_PORT_VLAN1_DEFAULT_SVID_S		0
#define   AR934X_PORT_VLAN1_FORCE_DEFAULT_VID_EN 	BIT(12)
#define   AR934X_PORT_VLAN1_PORT_TLS_MODE		BIT(13)
#define   AR934X_PORT_VLAN1_PORT_VLAN_PROP_EN		BIT(14)
#define   AR934X_PORT_VLAN1_PORT_CLONE_EN		BIT(15)
#define   AR934X_PORT_VLAN1_DEFAULT_CVID_S		16
#define   AR934X_PORT_VLAN1_FORCE_PORT_VLAN_EN		BIT(28)
#define   AR934X_PORT_VLAN1_ING_PORT_PRI_S		29

#define AR934X_REG_PORT_VLAN2(_port)	(AR934X_REG_PORT_BASE((_port)) + 0x0c)
#define   AR934X_PORT_VLAN2_PORT_VID_MEM_S		16
#define   AR934X_PORT_VLAN2_8021Q_MODE_S		30
#define   AR934X_PORT_VLAN2_8021Q_MODE_PORT_ONLY	0
#define   AR934X_PORT_VLAN2_8021Q_MODE_PORT_FALLBACK	1
#define   AR934X_PORT_VLAN2_8021Q_MODE_VLAN_ONLY	2
#define   AR934X_PORT_VLAN2_8021Q_MODE_SECURE		3

#define sw_to_ar7240(_dev) container_of(_dev, struct ar7240sw, swdev)

struct ar7240sw {
	struct mii_bus	*mii_bus;
	struct mii_bus	*switch_mii_bus;
	struct device_node *of_node;
	struct device_node *mdio_node;
	u8 ver;
};

static DEFINE_MUTEX(reg_mutex);

static inline int sw_is_ar7240(struct ar7240sw *as)
{
	return as->ver == AR7240_MASK_CTRL_VERSION_AR7240;
}

static inline int sw_is_ar934x(struct ar7240sw *as)
{
	return as->ver == AR7240_MASK_CTRL_VERSION_AR934X;
}

static inline u16 mk_phy_addr(u32 reg)
{
	return 0x17 & ((reg >> 4) | 0x10);
}

static inline u16 mk_phy_reg(u32 reg)
{
	return (reg << 1) & 0x1e;
}

static inline u16 mk_high_addr(u32 reg)
{
	return (reg >> 7) & 0x1ff;
}

static u32 __ar7240sw_reg_read(struct mii_bus *mii, u32 reg)
{
	unsigned long flags;
	u16 phy_addr;
	u16 phy_reg;
	u32 hi, lo;

	reg = (reg & 0xfffffffc) >> 2;
	phy_addr = mk_phy_addr(reg);
	phy_reg = mk_phy_reg(reg);

	local_irq_save(flags);
	mutex_lock(&mii->mdio_lock);
	mii->write(mii, 0x1f, 0x10, mk_high_addr(reg));
	lo = (u32) mii->read(mii, phy_addr, phy_reg);
	hi = (u32) mii->read(mii, phy_addr, phy_reg + 1);
	mutex_unlock(&mii->mdio_lock);
	local_irq_restore(flags);

	return (hi << 16) | lo;
}

static void __ar7240sw_reg_write(struct mii_bus *mii, u32 reg, u32 val)
{
	unsigned long flags;
	u16 phy_addr;
	u16 phy_reg;

	reg = (reg & 0xfffffffc) >> 2;
	phy_addr = mk_phy_addr(reg);
	phy_reg = mk_phy_reg(reg);

	local_irq_save(flags);
	mutex_lock(&mii->mdio_lock);
	mii->write(mii, 0x1f, 0x10, mk_high_addr(reg));
	mii->write(mii, phy_addr, phy_reg + 1, (val >> 16));
	mii->write(mii, phy_addr, phy_reg, (val & 0xffff));
	mutex_unlock(&mii->mdio_lock);
	local_irq_restore(flags);
}

static u32 ar7240sw_reg_read(struct mii_bus *mii, u32 reg_addr)
{
	u32 ret;

	mutex_lock(&reg_mutex);
	ret = __ar7240sw_reg_read(mii, reg_addr);
	mutex_unlock(&reg_mutex);

	return ret;
}

static void ar7240sw_reg_write(struct mii_bus *mii, u32 reg_addr, u32 reg_val)
{
	mutex_lock(&reg_mutex);
	__ar7240sw_reg_write(mii, reg_addr, reg_val);
	mutex_unlock(&reg_mutex);
}

static u32 ar7240sw_reg_rmw(struct mii_bus *mii, u32 reg, u32 mask, u32 val)
{
	u32 t;

	mutex_lock(&reg_mutex);
	t = __ar7240sw_reg_read(mii, reg);
	t &= ~mask;
	t |= val;
	__ar7240sw_reg_write(mii, reg, t);
	mutex_unlock(&reg_mutex);

	return t;
}

static void ar7240sw_reg_set(struct mii_bus *mii, u32 reg, u32 val)
{
	u32 t;

	mutex_lock(&reg_mutex);
	t = __ar7240sw_reg_read(mii, reg);
	t |= val;
	__ar7240sw_reg_write(mii, reg, t);
	mutex_unlock(&reg_mutex);
}

static int __ar7240sw_reg_wait(struct mii_bus *mii, u32 reg, u32 mask, u32 val,
			       unsigned timeout)
{
	int i;

	for (i = 0; i < timeout; i++) {
		u32 t;

		t = __ar7240sw_reg_read(mii, reg);
		if ((t & mask) == val)
			return 0;

		usleep_range(1000, 2000);
	}

	return -ETIMEDOUT;
}

static int ar7240sw_reg_wait(struct mii_bus *mii, u32 reg, u32 mask, u32 val,
			     unsigned timeout)
{
	int ret;

	mutex_lock(&reg_mutex);
	ret = __ar7240sw_reg_wait(mii, reg, mask, val, timeout);
	mutex_unlock(&reg_mutex);
	return ret;
}

static int ar7240sw_phy_read(struct mii_bus *bus, int phy_addr, int reg_addr)
{
	u32 t, val = 0xffff;
	int err;
	struct ar7240sw *as = bus->priv;
	struct mii_bus *mii = as->mii_bus;

	if (phy_addr >= AR7240_NUM_PHYS)
		return 0xffff;

	mutex_lock(&reg_mutex);
	t = (reg_addr << AR7240_MDIO_CTRL_REG_ADDR_S) |
	    (phy_addr << AR7240_MDIO_CTRL_PHY_ADDR_S) |
	    AR7240_MDIO_CTRL_MASTER_EN |
	    AR7240_MDIO_CTRL_BUSY |
	    AR7240_MDIO_CTRL_CMD_READ;

	__ar7240sw_reg_write(mii, AR7240_REG_MDIO_CTRL, t);
	err = __ar7240sw_reg_wait(mii, AR7240_REG_MDIO_CTRL,
				  AR7240_MDIO_CTRL_BUSY, 0, 5);
	if (!err)
		val = __ar7240sw_reg_read(mii, AR7240_REG_MDIO_CTRL);
	mutex_unlock(&reg_mutex);

	return val & AR7240_MDIO_CTRL_DATA_M;
}

static int ar7240sw_phy_write(struct mii_bus *bus, int phy_addr, int reg_addr,
		       u16 reg_val)
{
	u32 t;
	int ret;
	struct ar7240sw *as = bus->priv;
	struct mii_bus *mii = as->mii_bus;

	if (phy_addr >= AR7240_NUM_PHYS)
		return -EINVAL;

	mutex_lock(&reg_mutex);
	t = (phy_addr << AR7240_MDIO_CTRL_PHY_ADDR_S) |
	    (reg_addr << AR7240_MDIO_CTRL_REG_ADDR_S) |
	    AR7240_MDIO_CTRL_MASTER_EN |
	    AR7240_MDIO_CTRL_BUSY |
	    AR7240_MDIO_CTRL_CMD_WRITE |
	    reg_val;

	__ar7240sw_reg_write(mii, AR7240_REG_MDIO_CTRL, t);
	ret = __ar7240sw_reg_wait(mii, AR7240_REG_MDIO_CTRL,
				  AR7240_MDIO_CTRL_BUSY, 0, 5);
	mutex_unlock(&reg_mutex);

	return ret;
}

static void ar7240sw_disable_port(struct ar7240sw *as, unsigned port)
{
	ar7240sw_reg_write(as->mii_bus, AR7240_REG_PORT_CTRL(port),
			   AR7240_PORT_CTRL_STATE_DISABLED);
}

static void ar7240sw_setup(struct ar7240sw *as)
{
	struct mii_bus *mii = as->mii_bus;

	/* Enable CPU port, and disable mirror port */
	ar7240sw_reg_write(mii, AR7240_REG_CPU_PORT,
			   AR7240_CPU_PORT_EN |
			   (15 << AR7240_MIRROR_PORT_S));

	/* Setup TAG priority mapping */
	ar7240sw_reg_write(mii, AR7240_REG_TAG_PRIORITY, 0xfa50);

	if (sw_is_ar934x(as)) {
		/* Enable aging, MAC replacing */
		ar7240sw_reg_write(mii, AR934X_REG_AT_CTRL,
			0x2b /* 5 min age time */ |
			AR934X_AT_CTRL_AGE_EN |
			AR934X_AT_CTRL_LEARN_CHANGE);
		/* Enable ARP frame acknowledge */
		ar7240sw_reg_set(mii, AR934X_REG_QM_CTRL,
				 AR934X_QM_CTRL_ARP_EN);
		/* Enable Broadcast/Multicast frames transmitted to the CPU */
		ar7240sw_reg_set(mii, AR934X_REG_FLOOD_MASK,
				 AR934X_FLOOD_MASK_BC_DP(0) |
				 AR934X_FLOOD_MASK_MC_DP(0));

		/* setup MTU */
		ar7240sw_reg_rmw(mii, AR7240_REG_GLOBAL_CTRL,
				 AR9340_GLOBAL_CTRL_MTU_M,
				 AR9340_GLOBAL_CTRL_MTU_M);

		/* Enable MIB counters */
		ar7240sw_reg_set(mii, AR7240_REG_MIB_FUNCTION0,
				 AR934X_MIB_ENABLE);

	} else {
		/* Enable ARP frame acknowledge, aging, MAC replacing */
		ar7240sw_reg_write(mii, AR7240_REG_AT_CTRL,
			AR7240_AT_CTRL_RESERVED |
			0x2b /* 5 min age time */ |
			AR7240_AT_CTRL_AGE_EN |
			AR7240_AT_CTRL_ARP_EN |
			AR7240_AT_CTRL_LEARN_CHANGE);
		/* Enable Broadcast frames transmitted to the CPU */
		ar7240sw_reg_set(mii, AR7240_REG_FLOOD_MASK,
				 AR7240_FLOOD_MASK_BROAD_TO_CPU);

		/* setup MTU */
		ar7240sw_reg_rmw(mii, AR7240_REG_GLOBAL_CTRL,
				 AR7240_GLOBAL_CTRL_MTU_M,
				 AR7240_GLOBAL_CTRL_MTU_M);
	}

	/* setup Service TAG */
	ar7240sw_reg_rmw(mii, AR7240_REG_SERVICE_TAG, AR7240_SERVICE_TAG_M, 0);
}

/* inspired by phy_poll_reset in drivers/net/phy/phy_device.c */
static int
ar7240sw_phy_poll_reset(struct mii_bus *bus)
{
	const unsigned int sleep_msecs = 20;
	int ret, elapsed, i;

	for (elapsed = sleep_msecs; elapsed <= 600;
	     elapsed += sleep_msecs) {
		msleep(sleep_msecs);
		for (i = 0; i < AR7240_NUM_PHYS; i++) {
			ret = ar7240sw_phy_read(bus, i, MII_BMCR);
			if (ret < 0)
				return ret;
			if (ret & BMCR_RESET)
				break;
			if (i == AR7240_NUM_PHYS - 1) {
				usleep_range(1000, 2000);
				return 0;
			}
		}
	}
	return -ETIMEDOUT;
}

static int ar7240sw_reset(struct ar7240sw *as)
{
	struct mii_bus *mii = as->mii_bus;
	struct mii_bus *swmii = as->switch_mii_bus;
	int ret;
	int i;

	/* Set all ports to disabled state. */
	for (i = 0; i < AR7240_NUM_PORTS; i++)
		ar7240sw_disable_port(as, i);

	/* Wait for transmit queues to drain. */
	usleep_range(2000, 3000);

	/* Reset the switch. */
	ar7240sw_reg_write(mii, AR7240_REG_MASK_CTRL,
			   AR7240_MASK_CTRL_SOFT_RESET);

	ret = ar7240sw_reg_wait(mii, AR7240_REG_MASK_CTRL,
				AR7240_MASK_CTRL_SOFT_RESET, 0, 1000);

	/* setup PHYs */
	for (i = 0; i < AR7240_NUM_PHYS; i++) {
		ar7240sw_phy_write(swmii, i, MII_ADVERTISE,
				   ADVERTISE_ALL | ADVERTISE_PAUSE_CAP |
				   ADVERTISE_PAUSE_ASYM);
		ar7240sw_phy_write(swmii, i, MII_BMCR,
				   BMCR_RESET | BMCR_ANENABLE);
	}
	ret = ar7240sw_phy_poll_reset(swmii);
	if (ret)
		return ret;

	ar7240sw_setup(as);
	return ret;
}

static int
ag71xx_ar7240_probe(struct mdio_device *mdiodev)
{
	struct mii_bus *mii = mdiodev->bus;
	struct ar7240sw *as;
	struct reset_control *switch_reset;
	u32 ctrl;
	int phy_if_mode, err;

	as = devm_kzalloc(&mdiodev->dev, sizeof(*as), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->mii_bus = mii;
	as->of_node = mdiodev->dev.of_node;
	as->mdio_node = of_get_child_by_name(as->of_node, "mdio-bus");

	switch_reset = devm_reset_control_get_optional(&mdiodev->dev, "switch");
	if (switch_reset) {
		reset_control_assert(switch_reset);
		msleep(50);
		reset_control_deassert(switch_reset);
		msleep(200);
	}

	ctrl = ar7240sw_reg_read(mii, AR7240_REG_MASK_CTRL);
	as->ver = (ctrl >> AR7240_MASK_CTRL_VERSION_S) &
		  AR7240_MASK_CTRL_VERSION_M;

	if (sw_is_ar934x(as)) {
		phy_if_mode = of_get_phy_mode(as->of_node);

		if (phy_if_mode == PHY_INTERFACE_MODE_GMII) {
			ar7240sw_reg_set(mii, AR934X_REG_OPER_MODE0,
					 AR934X_OPER_MODE0_MAC_GMII_EN);
		} else if (phy_if_mode == PHY_INTERFACE_MODE_MII) {
			ar7240sw_reg_set(mii, AR934X_REG_OPER_MODE0,
					 AR934X_OPER_MODE0_PHY_MII_EN);
		} else {
			pr_err("%s: invalid PHY interface mode\n",
			       dev_name(&mdiodev->dev));
			return -EINVAL;
		}

		if (of_property_read_bool(as->of_node, "phy4-mii-enable")) {
			ar7240sw_reg_set(mii, AR934X_REG_OPER_MODE1,
					 AR934X_REG_OPER_MODE1_PHY4_MII_EN);
		}
	} else if (!sw_is_ar7240(as)) {
		pr_err("%s: unsupported chip, ctrl=%08x\n",
			dev_name(&mdiodev->dev), ctrl);
		return -EINVAL;
	}

	as->switch_mii_bus = devm_mdiobus_alloc(&mdiodev->dev);
	as->switch_mii_bus->name = "ar7240sw_mdio";
	as->switch_mii_bus->read = ar7240sw_phy_read;
	as->switch_mii_bus->write = ar7240sw_phy_write;
	as->switch_mii_bus->priv = as;
	as->switch_mii_bus->parent = &mdiodev->dev;
	snprintf(as->switch_mii_bus->id, MII_BUS_ID_SIZE, "%s", dev_name(&mdiodev->dev));

	if(as->mdio_node) {
		err = of_mdiobus_register(as->switch_mii_bus, as->mdio_node);
		if (err)
			return err;
	}

	ar7240sw_reset(as);
	dev_set_drvdata(&mdiodev->dev, as);
	return 0;
}

static void
ag71xx_ar7240_remove(struct mdio_device *mdiodev)
{
	struct ar7240sw *as = dev_get_drvdata(&mdiodev->dev);
	if(as->mdio_node)
		mdiobus_unregister(as->switch_mii_bus);
}

static const struct of_device_id ag71xx_sw_of_match[] = {
	{ .compatible = "qca,ar8216-builtin" },
	{ .compatible = "qca,ar8229-builtin" },
	{ /* sentinel */ },
};

static struct mdio_driver ag71xx_sw_driver = {
	.probe  = ag71xx_ar7240_probe,
	.remove = ag71xx_ar7240_remove,
	.mdiodrv.driver = {
		.name = "ag71xx-switch",
		.of_match_table = ag71xx_sw_of_match,
	},
};

mdio_module_driver(ag71xx_sw_driver);
MODULE_LICENSE("GPL");
