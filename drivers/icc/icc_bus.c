/*
 * Copyright (C) 2016 Oleksij Rempel <linux@rempel-privat.de>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/compiler.h>
#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/icc/icc.h>

static struct class *icc_class;

static ssize_t modalias_show(struct device *dev,
			     struct device_attribute *a __maybe_unused,
			     char *buf)
{
	return sprintf(buf, "icc:%s\n", dev_name(dev));
}
static DEVICE_ATTR_RO(modalias);

static struct attribute *icc_bus_dev_attrs[] = {
	&dev_attr_modalias.attr,
	NULL,
};
ATTRIBUTE_GROUPS(icc_bus_dev);

static int icc_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "MODALIAS=icc:%s", dev_name(dev));

	return 0;
}

static int icc_bus_match(struct device *dev, struct device_driver *drv)
{
	if (of_driver_match_device(dev, drv))
		return true;

	if (strcmp(dev_name(dev), drv->name) == 0)
		return true;

	return false;
}

static struct bus_type icc_bus_type = {
	.name           = "icc",
	.dev_groups	= icc_bus_dev_groups,
	.match		= icc_bus_match,
	.uevent		= icc_bus_uevent,
};

static int icc_drv_probe(struct device *dev)
{
	const struct icc_driver *icc_drv = to_icc_driver(dev->driver);
	struct icc_device *iccd = to_icc_device(dev);

	return icc_drv->probe(iccd);
}

static int icc_drv_remove(struct device *dev)
{
	const struct icc_driver *icc_drv = to_icc_driver(dev->driver);
	struct icc_device *iccd = to_icc_device(dev);

	return icc_drv->remove(iccd);
}

static void icc_drv_shutdown(struct device *dev)
{
	const struct icc_driver *icc_drv = to_icc_driver(dev->driver);
	struct icc_device *iccd = to_icc_device(dev);

	icc_drv->shutdown(iccd);
}

int icc_register_driver(struct icc_driver *icc_drv)
{
	icc_drv->driver.bus = &icc_bus_type;

	if (icc_drv->probe)
		icc_drv->driver.probe = icc_drv_probe;
	if (icc_drv->remove)
		icc_drv->driver.remove = icc_drv_remove;
	if (icc_drv->shutdown)
		icc_drv->driver.shutdown = icc_drv_shutdown;

	return driver_register(&icc_drv->driver);
}
EXPORT_SYMBOL_GPL(icc_register_driver);

void icc_unregister_driver(struct icc_driver *icc_drv)
{
	return driver_unregister(&icc_drv->driver);
}
EXPORT_SYMBOL_GPL(icc_unregister_driver);

static void icc_lun_release(struct device *dev)
{
}

static int icc_add_lun_from_dt(struct icc_master *iccm,
				 struct device_node *np)
{
	struct icc_device *iccd;
	u32 reg;
	int ret;

	ret = of_property_read_u32(np, "reg", &reg);
	if (ret) {
		dev_warn(iccm->dev, "can't parse \"reg\" for %s\n",
			 np->full_name);
		return ret;
	}

	if (reg > ICC_MAX_LUNS) {
		dev_warn(iccm->dev, "reg is more then %i\n",
			 ICC_MAX_LUNS);
		return -EINVAL;
	}

	iccd = &iccm->lun[reg];

	if (iccd->configured) {
		dev_warn(iccm->dev, "lun %i is already configured\n",
			 reg);
		return -EINVAL;
	}

	/* TODO: set some usaable name */
	dev_set_name(&iccd->dev, "%s:lun.%i:%s", dev_name(iccm->dev),
		     reg, np->name);

	iccd->dev.bus = &icc_bus_type;
	iccd->dev.parent = iccm->dev;
	iccd->dev.release = icc_lun_release;
	iccd->dev.of_node = np;
	iccd->iccm = iccm;
	iccd->lun_number = reg;

	/* TODO use device_create? to move major/minor registration here? */
	ret = device_register(&iccd->dev);
	if (ret < 0) {
		dev_warn(iccm->dev, "filed to registr lun %i\n", reg);
		put_device(&iccd->dev);
	}

	iccd->configured = true;

	return 0;
}

static void icc_add_luns_from_dt(struct icc_master *iccm)
{
	struct device_node *np = iccm->dev->of_node;
	struct device_node *child;

	for_each_available_child_of_node(np, child)
		icc_add_lun_from_dt(iccm, child);

}

void icc_add_luns(struct icc_master *iccm)
{
	iccm->icc_class = icc_class;
	icc_add_luns_from_dt(iccm);
}
EXPORT_SYMBOL_GPL(icc_add_luns);

static int __init icc_bus_init(void)
{
	int ret;

	icc_class = class_create(THIS_MODULE, "icc");
	if (IS_ERR(icc_class)) {
		ret = PTR_ERR(icc_class);
		return ret;
	}

	return bus_register(&icc_bus_type);
}
postcore_initcall(icc_bus_init);

static void __exit icc_bus_exit(void)
{
	class_destroy(icc_class);
	bus_unregister(&icc_bus_type);
}
module_exit(icc_bus_exit);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("ICC framework");
MODULE_LICENSE("GPL");

