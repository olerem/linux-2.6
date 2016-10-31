/*
 * CAN driver for ICC bus.
 *
 * Copyright (C) Oleksij Rempel <linux@rempel-privat.de>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/icc/icc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <linux/netdevice.h>
#include <linux/skbuff.h>

#include <linux/can/dev.h>
#include <linux/can/error.h>

#include <asm/uaccess.h>

static unsigned int icc_can_major;

struct icc_can_priv {
	struct icc_device	*icc;
	struct can_priv can;	/* must be the first member */
	struct net_device *dev;

	struct icc_prot_trf	trf;
	void			*rx_buf;
	size_t			rx_buf_size;
	struct mutex            rx_buf_lock;
	wait_queue_head_t	rx_wq;


	atomic_t		offline;
};

static int icc_can_int_trf(struct icc_can_priv *priv)
{
	struct icc_device *icc = priv->icc;
	struct icc_master *iccm = icc->master;
	struct icc_prot_trf *trf = &priv->trf;
	int ret;

	ret = icc_trf_alloc(iccm, trf, icc->lun_number,
				iccm->max_data_size);
	if (ret)
		return ret;

	priv->rx_buf = devm_kzalloc(priv->dev, iccm->max_data_size,
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	return 0;
}

static ssize_t icc_can_fop_write(struct file *file, const char __user *buf,
				size_t count, loff_t *pos)
{
	struct icc_can_priv *priv = file->private_data;
	struct icc_device *icc = priv->icc;
	struct icc_master *iccm = icc->master;
	int ret;

	if (count > iccm->max_data_size)
		return -EINVAL;

	count = min(count, (size_t)iccm->max_data_size);
	if (copy_from_user(priv->trf.data, buf, count))
		return -EFAULT;

	priv->trf.data_size = count;

	ret = icc_trf_xmit(icc->master, &priv->trf);
	if (ret)
		return ret;

	return count;
}

static int icc_can_wait(struct icc_can_priv *priv) {
	return wait_event_interruptible(priv->rx_wq, priv->rx_buf_size);
}

static void icc_can_wake(struct icc_can_priv *priv) {
	wake_up_interruptible(&priv->rx_wq);
}

static ssize_t icc_can_fop_read(struct file *file, char *buf,
			  size_t count, loff_t *ppos)
{
	struct icc_can_priv *priv = file->private_data;
	size_t ret = 0;

	if (*ppos != 0)
		return 0;

	ret = icc_can_wait(priv);
	if (ret)
		return ret;

	mutex_lock(&priv->rx_buf_lock);

	if (!priv->rx_buf_size) {
		ret = 0;
		goto done;
	}

	if (count < priv->rx_buf_size) {
		ret = -EINVAL;
		goto done;
	}

	if (copy_to_user(buf, priv->rx_buf, priv->rx_buf_size)) {
		ret = -EINVAL;
		goto done;
	}

	ret = *ppos = priv->rx_buf_size;
	priv->rx_buf_size = 0;

	mutex_unlock(&priv->rx_buf_lock);

done:
	return ret;
}

static int icc_can_fop_open(struct inode *ino, struct file *file)
{
	file->private_data =
		container_of(ino->i_cdev, struct icc_can_priv, cdev);

	return 0;
}

static const struct file_operations icc_can_fops = {
	.owner		= THIS_MODULE,
	.open		= icc_can_fop_open,
	.read		= icc_can_fop_read,
	.write		= icc_can_fop_write,
};

static int icc_can_rx_cb(struct icc_device *icc, void *rx_buf, size_t size)
{
	struct icc_can_priv *priv = icc_get_drvdata(icc);

	/* TODO find proper return value */
	if (atomic_read(&priv->offline))
		return -EBUSY;

	/*
	 * if we can't transfer it no, drop it
	 * allow other do the job
	 */
	if (!mutex_trylock(&priv->rx_buf_lock))
		return -EBUSY;

	/*
	 * TODO: we should decide what kind of logic do we use.
	 * For example for GPS it would make no match sense to
	 * keep old and not valid data.
	 * For now, let's be save and keep it.
	 */
	if (priv->rx_buf_size) {
		mutex_unlock(&priv->rx_buf_lock);
		return -EBUSY;
	}

	memcpy(priv->rx_buf, rx_buf, size);
	priv->rx_buf_size = size;

	mutex_unlock(&priv->rx_buf_lock);
	icc_can_wake(priv);

	return 0;
}

/* ^^ old code */


static int icc_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int icc_can_get_berr_counter(const struct net_device *dev,
				     struct can_berr_counter *bec)
{
	struct icc_can_priv *priv = netdev_priv(dev);
	struct icc_can_regs __iomem *reg = priv->membase;

	u16 cec = bfin_read(&reg->cec);

	bec->txerr = cec >> 8;
	bec->rxerr = cec;

	return 0;
}

static int icc_can_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct icc_can_priv *priv = netdev_priv(dev);
	struct icc_can_regs __iomem *reg = priv->membase;
	struct can_frame *cf = (struct can_frame *)skb->data;
	u8 dlc = cf->can_dlc;
	canid_t id = cf->can_id;
	u8 *data = cf->data;
	u16 val;
	int i;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	/* fill id */
	if (id & CAN_EFF_FLAG) {
		bfin_write(&reg->chl[TRANSMIT_CHL].id0, id);
		val = ((id & 0x1FFF0000) >> 16) | IDE;
	} else
		val = (id << 2);
	if (id & CAN_RTR_FLAG)
		val |= RTR;
	bfin_write(&reg->chl[TRANSMIT_CHL].id1, val | AME);

	/* fill payload */
	for (i = 0; i < 8; i += 2) {
		val = ((7 - i) < dlc ? (data[7 - i]) : 0) +
			((6 - i) < dlc ? (data[6 - i] << 8) : 0);
		bfin_write(&reg->chl[TRANSMIT_CHL].data[i], val);
	}

	/* fill data length code */
	bfin_write(&reg->chl[TRANSMIT_CHL].dlc, dlc);

	can_put_echo_skb(skb, dev, 0);

	/* set transmit request */
	bfin_write(&reg->trs2, BIT(TRANSMIT_CHL - 16));

	return 0;
}

static void icc_can_rx(struct net_device *dev, u16 isrc)
{
	struct icc_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct icc_can_regs __iomem *reg = priv->membase;
	struct can_frame *cf;
	struct sk_buff *skb;
	int obj;
	int i;
	u16 val;

	skb = alloc_can_skb(dev, &cf);
	if (skb == NULL)
		return;

	/* get id */
	if (isrc & BIT(RECEIVE_EXT_CHL)) {
		/* extended frame format (EFF) */
		cf->can_id = ((bfin_read(&reg->chl[RECEIVE_EXT_CHL].id1)
			     & 0x1FFF) << 16)
			     + bfin_read(&reg->chl[RECEIVE_EXT_CHL].id0);
		cf->can_id |= CAN_EFF_FLAG;
		obj = RECEIVE_EXT_CHL;
	} else {
		/* standard frame format (SFF) */
		cf->can_id = (bfin_read(&reg->chl[RECEIVE_STD_CHL].id1)
			     & 0x1ffc) >> 2;
		obj = RECEIVE_STD_CHL;
	}
	if (bfin_read(&reg->chl[obj].id1) & RTR)
		cf->can_id |= CAN_RTR_FLAG;

	/* get data length code */
	cf->can_dlc = get_can_dlc(bfin_read(&reg->chl[obj].dlc) & 0xF);

	/* get payload */
	for (i = 0; i < 8; i += 2) {
		val = bfin_read(&reg->chl[obj].data[i]);
		cf->data[7 - i] = (7 - i) < cf->can_dlc ? val : 0;
		cf->data[6 - i] = (6 - i) < cf->can_dlc ? (val >> 8) : 0;
	}

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
}

static int icc_can_err(struct net_device *dev, u16 isrc, u16 status)
{
	struct icc_can_priv *priv = netdev_priv(dev);
	struct icc_can_regs __iomem *reg = priv->membase;
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state state = priv->can.state;

	skb = alloc_can_err_skb(dev, &cf);
	if (skb == NULL)
		return -ENOMEM;

	if (isrc & RMLIS) {
		/* data overrun interrupt */
		netdev_dbg(dev, "data overrun interrupt\n");
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;
	}

	if (isrc & BOIS) {
		netdev_dbg(dev, "bus-off mode interrupt\n");
		state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		priv->can.can_stats.bus_off++;
		can_bus_off(dev);
	}

	if (isrc & EPIS) {
		/* error passive interrupt */
		netdev_dbg(dev, "error passive interrupt\n");
		state = CAN_STATE_ERROR_PASSIVE;
	}

	if ((isrc & EWTIS) || (isrc & EWRIS)) {
		netdev_dbg(dev, "Error Warning Transmit/Receive Interrupt\n");
		state = CAN_STATE_ERROR_WARNING;
	}

	if (state != priv->can.state && (state == CAN_STATE_ERROR_WARNING ||
				state == CAN_STATE_ERROR_PASSIVE)) {
		u16 cec = bfin_read(&reg->cec);
		u8 rxerr = cec;
		u8 txerr = cec >> 8;

		cf->can_id |= CAN_ERR_CRTL;
		if (state == CAN_STATE_ERROR_WARNING) {
			priv->can.can_stats.error_warning++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_WARNING :
				CAN_ERR_CRTL_RX_WARNING;
		} else {
			priv->can.can_stats.error_passive++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_PASSIVE :
				CAN_ERR_CRTL_RX_PASSIVE;
		}
	}

	if (status) {
		priv->can.can_stats.bus_error++;

		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		if (status & BEF)
			cf->data[2] |= CAN_ERR_PROT_BIT;
		else if (status & FER)
			cf->data[2] |= CAN_ERR_PROT_FORM;
		else if (status & SER)
			cf->data[2] |= CAN_ERR_PROT_STUFF;
		else
			cf->data[2] |= CAN_ERR_PROT_UNSPEC;
	}

	priv->can.state = state;

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 0;
}

static irqreturn_t icc_can_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct icc_can_priv *priv = netdev_priv(dev);
	struct icc_can_regs __iomem *reg = priv->membase;
	struct net_device_stats *stats = &dev->stats;
	u16 status, isrc;

	if ((irq == priv->tx_irq) && bfin_read(&reg->mbtif2)) {
		/* transmission complete interrupt */
		bfin_write(&reg->mbtif2, 0xFFFF);
		stats->tx_packets++;
		stats->tx_bytes += bfin_read(&reg->chl[TRANSMIT_CHL].dlc);
		can_get_echo_skb(dev, 0);
		netif_wake_queue(dev);
	} else if ((irq == priv->rx_irq) && bfin_read(&reg->mbrif1)) {
		/* receive interrupt */
		isrc = bfin_read(&reg->mbrif1);
		bfin_write(&reg->mbrif1, 0xFFFF);
		icc_can_rx(dev, isrc);
	} else if ((irq == priv->err_irq) && bfin_read(&reg->gis)) {
		/* error interrupt */
		isrc = bfin_read(&reg->gis);
		status = bfin_read(&reg->esr);
		bfin_write(&reg->gis, 0x7FF);
		icc_can_err(dev, isrc, status);
	} else {
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int icc_can_open(struct net_device *dev)
{
	struct icc_can_priv *priv = netdev_priv(dev);
	int err;

	/* common open */
	err = open_candev(dev);
	if (err)
		return err;

	atomic_set(&priv->offline, 0);

	netif_start_queue(dev);

	return 0;
}

static int icc_can_close(struct net_device *dev)
{
	struct icc_can_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);

	atomic_set(&priv->offline, 1);

	close_candev(dev);

	return 0;
}

static struct net_device *alloc_icc_candev(void)
{
	struct net_device *dev;
	struct icc_can_priv *priv;

	dev = alloc_candev(sizeof(*priv), TX_ECHO_SKB_MAX);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->can.do_set_mode = icc_can_set_mode;
	priv->can.do_get_berr_counter = icc_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES;

	return dev;
}

static const struct net_device_ops icc_can_netdev_ops = {
	.ndo_open               = icc_can_open,
	.ndo_stop               = icc_can_close,
	.ndo_start_xmit         = icc_can_start_xmit,
	.ndo_change_mtu         = can_change_mtu,
};

static int icc_can_probe(struct icc_device *icc)
{
	struct icc_master *iccm = icc->master;
	struct device *dev = &icc->dev;
	struct icc_can_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct icc_can_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->icc = icc;
	mutex_init(&priv->rx_buf_lock);
	init_waitqueue_head(&priv->rx_wq);

	icc_set_drvdata(icc, priv);
	icc_set_rxcb(icc, icc_can_rx_cb);

	ret = icc_can_int_trf(priv);
	if (ret)
		return ret;


	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &icc_can_netdev_ops;

	err = register_candev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering failed (err=%d)\n", err);
		goto exit_candev_free;
	}

	dev_info(&pdev->dev,
		"%s device registered"
		"(&reg_base=%p, rx_irq=%d, tx_irq=%d, err_irq=%d, sclk=%d)\n",
		DRV_NAME, priv->membase, priv->rx_irq,
		priv->tx_irq, priv->err_irq, priv->can.clock.freq);
	return 0;

exit_candev_free:
	free_candev(dev);

	return 0;
}

static int icc_can_remove(struct icc_device *icc)
{
	struct icc_master *iccm = icc->master;

	device_destroy(iccm->icc_class,
		       MKDEV(icc_can_major, icc->lun_number));
	return 0;
}

static const struct of_device_id icc_can_of_match[] = {
	{ .compatible = "icc,can" },
	{},
};
MODULE_DEVICE_TABLE(of, icc_can_of_match);

static struct icc_driver icc_can_driver = {
	.driver = {
		.name = "icc_can",
		.owner = THIS_MODULE,
		.of_match_table	= icc_can_of_match,
	},
	.probe = icc_can_probe,
	.remove = icc_can_remove,
};

static int __init icc_can_init(void)
{
	static dev_t dev;
	int ret;

	ret = alloc_chrdev_region(&dev, 0, ICC_MAX_LUNS, "icc_can");
	if (ret)
		pr_err("Unable to register ICC chdev region\n");

	icc_can_major = MAJOR(dev);

	ret = icc_register_driver(&icc_can_driver);
	if (ret)
		pr_err("Unable to register ICC chdev\n");

	return ret;
}

module_init(icc_can_init);

static void __exit
icc_can_exit(void)
{
	icc_unregister_driver(&icc_can_driver);
	unregister_chrdev_region(MKDEV(icc_can_major, 0), ICC_MAX_LUNS);
}

module_exit(icc_can_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("ICC CAN driver");
