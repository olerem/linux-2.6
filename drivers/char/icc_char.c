/*
 * Char device for ICC bus.
 *
 * Copyright (C) 2016 Oleksij Rempel <linux@rempel-privat.de>
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

#include <asm/uaccess.h>

static unsigned int icc_ch_major;

struct icc_ch_priv {
	struct device		*dev;
	struct icc_device	*iccd;
	struct cdev		cdev;


	struct icc_trf	trf;
	void			*rx_buf;
	size_t			rx_buf_size;
	struct mutex            rx_buf_lock;
	wait_queue_head_t	rx_wq;
};

static int icc_ch_int_trf(struct icc_ch_priv *priv)
{
	struct icc_device *iccd = priv->iccd;
	struct icc_master *iccm = iccd->iccm;
	struct icc_trf *trf = &priv->trf;
	int ret;

	ret = icc_trf_alloc(iccm, trf, iccd->lun_number,
				iccm->max_data_size);
	if (ret)
		return ret;

	priv->rx_buf = devm_kzalloc(priv->dev, iccm->max_data_size,
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	return 0;
}

static ssize_t icc_ch_fop_write(struct file *file, const char __user *buf,
				size_t count, loff_t *pos)
{
	struct icc_ch_priv *priv = file->private_data;
	struct icc_device *iccd = priv->iccd;
	struct icc_master *iccm = iccd->iccm;
	int ret;

	if (count > iccm->max_data_size)
		return -EINVAL;

	count = min(count, (size_t)iccm->max_data_size);
	if (copy_from_user(priv->trf.data, buf, count))
		return -EFAULT;

	priv->trf.data_size = count;

	ret = icc_trf_xmit(iccm, &priv->trf);
	if (ret)
		return ret;

	return count;
}

static int icc_ch_wait(struct icc_ch_priv *priv) {
	return wait_event_interruptible(priv->rx_wq, priv->rx_buf_size);
}

static void icc_ch_wake(struct icc_ch_priv *priv) {
	wake_up_interruptible(&priv->rx_wq);
}

static ssize_t icc_ch_fop_read(struct file *file, char *buf,
			  size_t count, loff_t *ppos)
{
	struct icc_ch_priv *priv = file->private_data;
	size_t ret = 0;

	if (*ppos != 0)
		return 0;

	ret = icc_ch_wait(priv);
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

static int icc_ch_fop_open(struct inode *ino, struct file *file)
{
	file->private_data =
		container_of(ino->i_cdev, struct icc_ch_priv, cdev);

	return 0;
}

static const struct file_operations icc_ch_fops = {
	.owner		= THIS_MODULE,
	.open		= icc_ch_fop_open,
	.read		= icc_ch_fop_read,
	.write		= icc_ch_fop_write,
};

static int icc_ch_rx_cb(struct icc_device *iccd, void *rx_buf, size_t size)
{
	struct icc_ch_priv *priv = icc_get_drvdata(iccd);

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
	icc_ch_wake(priv);

	return 0;
}

static int icc_ch_probe(struct icc_device *iccd)
{
	struct icc_master *iccm = iccd->iccm;
	struct device *dev = &iccd->dev;
	struct device *dev1;
	struct icc_ch_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct icc_ch_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->iccd = iccd;
	mutex_init(&priv->rx_buf_lock);
	init_waitqueue_head(&priv->rx_wq);

	icc_set_drvdata(iccd, priv);
	icc_set_rxcb(iccd, icc_ch_rx_cb);

	ret = icc_ch_int_trf(priv);
	if (ret)
		return ret;

	cdev_init(&priv->cdev, &icc_ch_fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, MKDEV(icc_ch_major, iccd->lun_number), 1);
	if (ret) {
		dev_err(dev, "filed to add cdev\n");
		return ret;
	}

	dev1 = device_create(iccm->icc_class, dev,
		      MKDEV(icc_ch_major, iccd->lun_number),
		      priv, "iccch%03d", iccd->lun_number);

	if (IS_ERR(dev1)) {
		dev_info(dev, "ICC chardev filed: %i\n", PTR_ERR(dev1));
		return PTR_ERR(dev1);
	}

	dev_info(dev, "ICC chardev enabled\n");
	return 0;
}

static int icc_ch_remove(struct icc_device *iccd)
{
	struct icc_master *iccm = iccd->iccm;

	device_destroy(iccm->icc_class,
		       MKDEV(icc_ch_major, iccd->lun_number));
	return 0;
}

static const struct of_device_id icc_ch_of_match[] = {
	{ .compatible = "icc,char" },
	{},
};
MODULE_DEVICE_TABLE(of, icc_ch_of_match);

static struct icc_driver icc_ch_driver = {
	.driver = {
		.name = "icc-ch",
		.owner = THIS_MODULE,
		.of_match_table	= icc_ch_of_match,
	},
	.probe = icc_ch_probe,
	.remove = icc_ch_remove,
};

static int __init icc_ch_init(void)
{
	static dev_t dev;
	int ret;

	ret = alloc_chrdev_region(&dev, 0, ICC_MAX_LUNS, "iccch");
	if (ret)
		pr_err("Unable to register ICC chdev region\n");

	icc_ch_major = MAJOR(dev);

	ret = icc_register_driver(&icc_ch_driver);
	if (ret)
		pr_err("Unable to register ICC chdev\n");

	return ret;
}

module_init(icc_ch_init);

static void __exit
icc_ch_exit(void)
{
	icc_unregister_driver(&icc_ch_driver);
	unregister_chrdev_region(MKDEV(icc_ch_major, 0), ICC_MAX_LUNS);
}

module_exit(icc_ch_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("ICC chardev");
