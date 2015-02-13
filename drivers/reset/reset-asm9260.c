/*
 * Alphascale SoCs Reset Controller driver
 *
 * Copyright 2015, Oleksij Rempel <linux@rempel-privat.de>
 *
 * Based on:
 * Allwinner SoCs Reset Controller driver
 *  Copyright 2013 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#define SET_REG	0x4
#define CLR_REG	0x8

struct asm9260_reset_data {
	spinlock_t			lock;
	void __iomem			*membase;
	struct reset_controller_dev	rcdev;
};

static int asm9260_reset_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct asm9260_reset_data *data = container_of(rcdev,
						     struct asm9260_reset_data,
						     rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;

	iowrite32(BIT(offset), data->membase + (bank * 0x10) + CLR_REG);

	return 0;
}

static int asm9260_reset_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct asm9260_reset_data *data = container_of(rcdev,
						     struct asm9260_reset_data,
						     rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;

	iowrite32(BIT(offset), data->membase + (bank * 0x10) + SET_REG);

	return 0;
}

static struct reset_control_ops asm9260_reset_ops = {
	.assert		= asm9260_reset_assert,
	.deassert	= asm9260_reset_deassert,
};

/*
 * And these are the controllers we can register through the regular
 * device model.
 */
static const struct of_device_id asm9260_reset_dt_ids[] = {
	 { .compatible = "alphascale,asm9260-reset", },
	 { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, asm9260_reset_dt_ids);

static int asm9260_reset_probe(struct platform_device *pdev)
{
	struct asm9260_reset_data *data;
	struct resource *res;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->membase))
		return PTR_ERR(data->membase);

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = resource_size(res) / 0x10 * 32;
	data->rcdev.ops = &asm9260_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;

	return reset_controller_register(&data->rcdev);
}

static int asm9260_reset_remove(struct platform_device *pdev)
{
	struct asm9260_reset_data *data = platform_get_drvdata(pdev);

	reset_controller_unregister(&data->rcdev);

	return 0;
}

static struct platform_driver asm9260_reset_driver = {
	.probe	= asm9260_reset_probe,
	.remove	= asm9260_reset_remove,
	.driver = {
		.name		= "asm9260-reset",
		.of_match_table	= asm9260_reset_dt_ids,
	},
};
module_platform_driver(asm9260_reset_driver);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("Alphascale asm9260 SoCs Reset Controller Driver");
MODULE_LICENSE("GPL");
