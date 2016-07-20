/*
 * Copyright (C) 2016 Oleksij Rempel <linux@rempel-privat.de>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reset.h>
#include <linux/watchdog.h>
#include <linux/icc/icc.h>

struct icc_wdt_priv {
	struct device		*dev;
	struct icc_device	*iccd;
	struct watchdog_device	wdd;

	struct icc_trf	trf;
	bool			check_pong;
	atomic_t		pong;
};

static int icc_wdt_feed(struct watchdog_device *wdd)
{
	struct icc_wdt_priv *priv = watchdog_get_drvdata(wdd);
	struct icc_device *iccd = priv->iccd;

	if (priv->check_pong && atomic_read(&priv->pong))
		dev_warn_ratelimited(priv->dev, "making ping without getting pong\n");

	atomic_set(&priv->pong, 1);

	return icc_trf_xmit(iccd->iccm, &priv->trf);
}

static int icc_wdt_enable(struct watchdog_device *wdd)
{
	return 0;
}

static int icc_wdt_stop(struct watchdog_device *wdd)
{
	return 0;
}

static const struct watchdog_info icc_wdt_ident = {
	.identity         =	"ICC, WatchDog",
};

static struct watchdog_ops icc_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= icc_wdt_enable,
	.stop		= icc_wdt_stop,
	.ping		= icc_wdt_feed,
};

static int icc_wdt_rx_cb(struct icc_device *iccd, void *rx_buf, size_t size)
{
	struct icc_wdt_priv *priv = icc_get_drvdata(iccd);

	atomic_set(&priv->pong, 0);

	return 0;
}

static int icc_wdt_int_trf(struct icc_wdt_priv *priv)
{
	struct icc_device *iccd = priv->iccd;
	struct icc_trf *trf = &priv->trf;

	u8 *buf;
	int ret;

	ret = icc_trf_alloc(iccd->iccm, trf, 9, 1);
	if (ret)
		return ret;

	buf = trf->data;
	buf[0] = 0x62;

	return 0;
}

static int icc_wdt_probe(struct icc_device *iccd)
{
	struct device *dev = &iccd->dev;
	struct icc_wdt_priv *priv;
	struct watchdog_device *wdd;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct icc_wdt_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->iccd = iccd;

	wdd = &priv->wdd;
	wdd->info = &icc_wdt_ident;
	wdd->ops = &icc_wdt_ops;
	wdd->min_timeout = 1;
	wdd->max_timeout = 10;
	wdd->parent = dev;

	icc_set_drvdata(iccd, priv);
	icc_set_rxcb(iccd, icc_wdt_rx_cb);
	watchdog_set_drvdata(wdd, priv);

	/*
	 * If 'timeout-sec' unspecified in devicetree, assume a 30 second
	 * default, unless the max timeout is less than 30 seconds, then use
	 * the max instead.
	 */
	watchdog_init_timeout(wdd, 10, dev);

	ret = watchdog_register_device(wdd);
	if (ret)
		return ret;

	ret = icc_wdt_int_trf(priv);
	if (ret)
		return ret;

	dev_info(dev, "Watchdog enabled\n");
	return 0;
}

static int icc_wdt_remove(struct icc_device *iccd)
{
	struct icc_wdt_priv *priv = icc_get_drvdata(iccd);

	watchdog_unregister_device(&priv->wdd);

	return 0;
}

static const struct of_device_id icc_wdt_of_match[] = {
	{ .compatible = "icc,wdt" },
	{},
};
MODULE_DEVICE_TABLE(of, icc_wdt_of_match);

static struct icc_driver icc_wdt_driver = {
	.driver = {
		.name = "icc-wdt",
		.owner = THIS_MODULE,
		.of_match_table	= icc_wdt_of_match,
	},
	.probe = icc_wdt_probe,
	.remove = icc_wdt_remove,
};

static int __init icc_wdt_init(void)
{
	return icc_register_driver(&icc_wdt_driver);
}
module_init(icc_wdt_init);

static void __exit icc_wdt_exit(void)
{
	icc_unregister_driver(&icc_wdt_driver);
}
module_exit(icc_wdt_exit);

MODULE_DESCRIPTION("ICC WatchDog Driver");
MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_LICENSE("GPL");
