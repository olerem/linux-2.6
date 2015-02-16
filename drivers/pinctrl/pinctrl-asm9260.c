/*
 * Pinctrl driver for the Alphascale ASM9260 SoC
 *
 * Copyright (c) 2014, Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>

#include "core.h"
#include "pinctrl-utils.h"

/* pinctrl register */
#define IOCON_PIO0_0			0x0000
/* only two modes are supported NONE and PULL UP */
#define IOCON_MODE_SHIFT		3
#define IOCON_MODE_MASK			(0x3 << IOCON_MODE_SHIFT)
/* Only GPIO0_* pins support pull up. */
#define IOCON_MODE_PULL_UP		(0x2 << IOCON_MODE_SHIFT)
/* Only GPIO0_* pins don't support pull down. */
#define IOCON_MODE_PULL_DOWN		(0x1 << IOCON_MODE_SHIFT)
#define IOCON_MODE_NONE			(0x0 << IOCON_MODE_SHIFT)
/* up to 8 functions per pin */
#define IOCON_PINMUX_MASK		(0x7 << 0)

#define MUX_OFFSET(bank, pin)		((bank) * 32 + (pin) * 4)

enum asm9260_mux {
	ASM9260_MUX_na = -1,

	ASM9260_MUX_gpio0,
	ASM9260_MUX_cam0,
	ASM9260_MUX_can0,
	ASM9260_MUX_can1,
	ASM9260_MUX_ct0,
	ASM9260_MUX_ct1,
	ASM9260_MUX_ct2,
	ASM9260_MUX_ct3,
	ASM9260_MUX_i2c0,
	ASM9260_MUX_i2c1,
	ASM9260_MUX_i2s0,
	ASM9260_MUX_i2s1,
	ASM9260_MUX_jtag,
	ASM9260_MUX_lcd0,
	ASM9260_MUX_lcd_if0,
	ASM9260_MUX_mc,
	ASM9260_MUX_mc0,
	ASM9260_MUX_mii0,
	ASM9260_MUX_nand0,
	ASM9260_MUX_outclk,
	ASM9260_MUX_qei0,
	ASM9260_MUX_qspi0,
	ASM9260_MUX_rmii0,
	ASM9260_MUX_sd0,
	ASM9260_MUX_spi0,
	ASM9260_MUX_spi1,
	ASM9260_MUX_uart0,
	ASM9260_MUX_uart1,
	ASM9260_MUX_uart2,
	ASM9260_MUX_uart3,
	ASM9260_MUX_uart4,
	ASM9260_MUX_uart5,
	ASM9260_MUX_uart6,
	ASM9260_MUX_uart7,
	ASM9260_MUX_uart8,
	ASM9260_MUX_uart9,
};

struct asm9260_function {
	const char		*name;
	const char		**groups;
	unsigned int		ngroups;
};

#define FUNCTION(mux)			\
	[(ASM9260_MUX_ ## mux)] = {		\
		.name = #mux,			\
		.groups = NULL,			\
		.ngroups = 0,		\
	}

static struct asm9260_function asm9260_functions[] = {
	FUNCTION(gpio0),
	FUNCTION(cam0),
	FUNCTION(can0),
	FUNCTION(can1),
	FUNCTION(ct0),
	FUNCTION(ct1),
	FUNCTION(ct2),
	FUNCTION(ct3),
	FUNCTION(i2c0),
	FUNCTION(i2c1),
	FUNCTION(i2s0),
	FUNCTION(i2s1),
	FUNCTION(jtag),
	FUNCTION(lcd0),
	FUNCTION(lcd_if0),
	FUNCTION(mc),
	FUNCTION(mc0),
	FUNCTION(mii0),
	FUNCTION(nand0),
	FUNCTION(outclk),
	FUNCTION(qei0),
	FUNCTION(qspi0),
	FUNCTION(rmii0),
	FUNCTION(sd0),
	FUNCTION(spi0),
	FUNCTION(spi1),
	FUNCTION(uart0),
	FUNCTION(uart1),
	FUNCTION(uart2),
	FUNCTION(uart3),
	FUNCTION(uart4),
	FUNCTION(uart5),
	FUNCTION(uart6),
	FUNCTION(uart7),
	FUNCTION(uart8),
	FUNCTION(uart9),
};

struct asm9260_pingroup {
	const char		*name;
	const unsigned int	number;
	const unsigned int	bank;
	const unsigned int	pin;

#define MAX_FUNCS_PER_PIN	8
	const int		funcs[MAX_FUNCS_PER_PIN];
};

#define PMUX(p_number, p_bank, p_pin, p_name, f1, f2, f3, f4, f5, f6, f7) \
	{						\
		.number = p_number,			\
		.bank = p_bank,				\
		.pin = p_pin,				\
		.name = #p_name,			\
		.funcs = {				\
			ASM9260_MUX_gpio0,		\
			ASM9260_MUX_ ## f1,		\
			ASM9260_MUX_ ## f2,		\
			ASM9260_MUX_ ## f3,		\
			ASM9260_MUX_ ## f4,		\
			ASM9260_MUX_ ## f5,		\
			ASM9260_MUX_ ## f6,		\
			ASM9260_MUX_ ## f7,		\
		},					\
	}

static struct asm9260_pingroup asm9260_mux_table[] = {
	PMUX(120,	0,	0,	GPIO0_0,
		na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(121,	0,	1,	GPIO0_1,
		na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(122,	0,	2,	GPIO0_2,
		na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(123,	0,	3,	GPIO0_3,
		na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(124,	0,	4,	GPIO0_4,
		na,	uart1,	i2s0,	spi0,	na,	jtag,	i2c0),
	PMUX(128,	1,	4,	GPIO1_4,
		ct0,	uart0,	lcd_if0,	spi0,	mii0,	lcd0,	na),
	PMUX(129,	1,	5,	GPIO1_5,
		na,	uart0,	lcd_if0,	spi0,	rmii0,	lcd0,	na),
	PMUX(130,	1,	6,	GPIO1_6,
		na,	uart0,	lcd_if0,	spi0,	rmii0,	lcd0,	i2c1),
	PMUX(131,	1,	7,	GPIO1_7,
		na,	uart0,	lcd_if0,	spi0,	rmii0,	lcd0,	i2c1),
	PMUX(132,	2,	0,	GPIO2_0,
		ct1,	uart2,	lcd_if0,	spi1,	mii0,	lcd0,	can0),
	PMUX(133,	2,	1,	GPIO2_1,
		ct1,	uart2,	lcd_if0,	spi1,	mii0,	lcd0,	can0),
	PMUX(134,	2,	2,	GPIO2_2,
		ct1,	uart3,	lcd_if0,	spi1,	mii0,	lcd0,	na),
	PMUX(135,	2,	3,	GPIO2_3,
		ct1,	uart3,	lcd_if0,	spi1,	mii0,	lcd0,	na),
	PMUX(136,	2,	4,	GPIO2_4,
		ct1,	uart3,	lcd_if0,	na,	mii0,	lcd0,	na),
	PMUX(137,	2,	5,	GPIO2_5,
		na,	uart3,	lcd_if0,	na,	mii0,	lcd0,	outclk),
	PMUX(138,	2,	6,	GPIO2_6,
		na,	uart3,	lcd_if0,	na,	mii0,	lcd0,	can1),
	PMUX(139,	2,	7,	GPIO2_7,
		na,	uart4,	lcd_if0,	na,	mii0,	lcd0,	can1),
	PMUX(140,	3,	0,	GPIO3_0,
		ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	na),
	PMUX(141,	3,	1,	GPIO3_1,
		ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	na),
	PMUX(142,	3,	2,	GPIO3_2,
		ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	can1),
	PMUX(143,	3,	3,	GPIO3_3,
		ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	can1),
	PMUX(144,	3,	4,	GPIO3_4,
		ct2,	uart5,	lcd_if0,	sd0,	rmii0,	lcd0,	outclk),
	PMUX(145,	3,	5,	GPIO3_5,
		na,	uart5,	lcd_if0,	sd0,	rmii0,	lcd0,	i2c0),
	PMUX(146,	3,	6,	GPIO3_6,
		na,	uart5,	lcd_if0,	na,	rmii0,	lcd0,	i2c0),
	PMUX(147,	3,	7,	GPIO3_7,
		na,	uart5,	lcd_if0,	na,	rmii0,	lcd0,	na),
	PMUX(151,	4,	0,	GPIO4_0,
		ct3,	uart5,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(152,	4,	1,	GPIO4_1,
		ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(153,	4,	2,	GPIO4_2,
		ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(154,	4,	3,	GPIO4_3,
		ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(155,	4,	4,	GPIO4_4,
		ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(156,	4,	5,	GPIO4_5,
		na,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(157,	4,	6,	GPIO4_6,
		na,	na,	na,	na,	mii0,	lcd0,	i2c1),
	PMUX(158,	4,	7,	GPIO4_7,
		na,	na,	na,	na,	mii0,	lcd0,	i2c1),
	PMUX(169,	5,	0,	GPIO5_0,
		mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	na),
	PMUX(170,	5,	1,	GPIO5_1,
		mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	na),
	PMUX(171,	5,	2,	GPIO5_2,
		mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	can0),
	PMUX(172,	5,	3,	GPIO5_3,
		mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	can0),
	PMUX(173,	5,	4,	GPIO5_4,
		mc0,	uart8,	i2s1,	sd0,	rmii0,	na,	na),
	PMUX(51,	8,	1,	GPIO8_1,
		na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(52,	8,	2,	GPIO8_2,
		na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(53,	8,	3,	GPIO8_3,
		na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(54,	8,	4,	GPIO8_4,
		na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(55,	8,	5,	GPIO8_5,
		na,	uart3,	cam0,	na,	mii0,	na,	na),
	PMUX(56,	8,	6,	GPIO8_6,
		na,	uart3,	cam0,	na,	mii0,	na,	na),
	PMUX(57,	8,	7,	GPIO8_7,
		na,	uart3,	cam0,	na,	mii0,	na,	na),
	PMUX(45,	9,	0,	GPIO9_0,
		ct0,	uart3,	cam0,	na,	rmii0,	na,	i2c0),
	PMUX(46,	9,	1,	GPIO9_1,
		ct0,	uart3,	cam0,	na,	rmii0,	na,	i2c0),
	PMUX(47,	9,	2,	GPIO9_2,
		ct0,	uart4,	cam0,	na,	rmii0,	na,	na),
	PMUX(48,	9,	3,	GPIO9_3,
		ct0,	uart4,	cam0,	na,	rmii0,	na,	na),
	PMUX(49,	9,	4,	GPIO9_4,
		ct0,	uart4,	cam0,	na,	rmii0,	na,	na),
	PMUX(50,	9,	5,	GPIO9_5,
		na,	uart4,	cam0,	na,	rmii0,	na,	i2c1),
	PMUX(4,		10,	0,	GPIO10_0,
		ct1,	uart5,	i2s0,	spi0,	na,	na,	na),
	PMUX(5,		10,	1,	GPIO10_1,
		ct1,	uart5,	i2s0,	spi0,	rmii0,	na,	na),
	PMUX(6,		10,	2,	GPIO10_2,
		ct1,	uart5,	i2s0,	spi0,	rmii0,	na,	na),
	PMUX(7,		10,	3,	GPIO10_3,
		ct1,	uart5,	i2s0,	spi0,	na,	na,	can0),
	PMUX(8,		10,	4,	GPIO10_4,
		ct1,	uart6,	i2s0,	spi1,	rmii0,	na,	na),
	PMUX(9,		10,	5,	GPIO10_5,
		na,	uart6,	i2s0,	spi1,	rmii0,	na,	can1),
	PMUX(10,	10,	6,	GPIO10_6,
		na,	uart6,	i2s0,	spi1,	rmii0,	na,	can1),
	PMUX(11,	10,	7,	GPIO10_7,
		na,	uart6,	cam0,	spi1,	rmii0,	na,	na),
	PMUX(12,	11,	0,	GPIO11_0,
		ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(13,	11,	1,	GPIO11_1,
		ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(14,	11,	2,	GPIO11_2,
		ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(15,	11,	3,	GPIO11_3,
		ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(16,	11,	4,	GPIO11_4,
		ct2,	uart8,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(17,	11,	5,	GPIO11_5,
		na,	uart8,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(18,	11,	6,	GPIO11_6,
		na,	uart9,	i2s1,	na,	nand0,	na,	i2c0),
	PMUX(19,	11,	7,	GPIO11_7,
		na,	uart9,	cam0,	na,	nand0,	na,	i2c0),
	PMUX(23,	12,	0,	GPIO12_0,
		ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(24,	12,	1,	GPIO12_1,
		ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(25,	12,	2,	GPIO12_2,
		ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(26,	12,	3,	GPIO12_3,
		ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(27,	12,	4,	GPIO12_4,
		ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(28,	12,	5,	GPIO12_5,
		na,	uart8,	na,	sd0,	nand0,	na,	na),
	PMUX(29,	12,	6,	GPIO12_6,
		na,	uart8,	cam0,	na,	nand0,	na,	i2c1),
	PMUX(30,	12,	7,	GPIO12_7,
		na,	na,	cam0,	na,	nand0,	na,	i2c1),
	PMUX(31,	13,	4,	GPIO13_4,
		mc,	uart2,	na,	spi1,	nand0,	na,	na),
	PMUX(32,	13,	5,	GPIO13_5,
		mc0,	uart9,	na,	spi1,	nand0,	na,	na),
	PMUX(33,	13,	6,	GPIO13_6,
		mc0,	uart9,	na,	spi1,	nand0,	na,	na),
	PMUX(34,	13,	7,	GPIO13_7,
		mc0,	na,	na,	spi1,	nand0,	na,	na),
	PMUX(38,	14,	0,	GPIO14_0,
		mc0,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(39,	14,	1,	GPIO14_1,
		mc0,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(40,	14,	2,	GPIO14_2,
		na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(41,	14,	3,	GPIO14_3,
		na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(42,	14,	4,	GPIO14_4,
		na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(43,	14,	5,	GPIO14_5,
		na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(44,	15,	0,	GPIO15_0,
		na,	uart4,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(61,	15,	1,	GPIO15_1,
		na,	uart4,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(62,	15,	2,	GPIO15_2,
		na,	uart5,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(63,	15,	3,	GPIO15_3,
		na,	uart5,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(64,	15,	4,	GPIO15_4,
		na,	uart6,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(65,	15,	5,	GPIO15_5,
		na,	uart6,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(66,	15,	6,	GPIO15_6,
		na,	uart7,	i2s0,	na,	rmii0,	na,	na),
	PMUX(67,	15,	7,	GPIO15_7,
		na,	uart7,	na,	na,	rmii0,	na,	na),
	PMUX(73,	16,	0,	GPIO16_0,
		ct2,	uart4,	na,	na,	mii0,	na,	na),
	PMUX(74,	16,	1,	GPIO16_1,
		ct2,	uart4,	na,	na,	mii0,	na,	na),
	PMUX(75,	16,	2,	GPIO16_2,
		ct2,	uart5,	na,	na,	mii0,	na,	na),
	PMUX(76,	16,	3,	GPIO16_3,
		ct2,	uart5,	na,	na,	mii0,	na,	na),
	PMUX(77,	16,	4,	GPIO16_4,
		ct2,	uart6,	na,	na,	mii0,	na,	na),
	PMUX(78,	16,	5,	GPIO16_5,
		qei0,	uart6,	na,	na,	mii0,	na,	na),
	PMUX(79,	16,	6,	GPIO16_6,
		qei0,	uart6,	na,	na,	mii0,	na,	can1),
	PMUX(80,	16,	7,	GPIO16_7,
		qei0,	uart6,	na,	na,	mii0,	na,	can1),
	PMUX(81,	17,	0,	GPIO17_0,
		ct3,	uart7,	i2s1,	na,	rmii0,	na,	na),
	PMUX(82,	17,	1,	GPIO17_1,
		ct3,	uart7,	i2s1,	na,	na,	na,	na),
	PMUX(83,	17,	2,	GPIO17_2,
		ct3,	uart7,	i2s1,	na,	na,	na,	i2c1),
	PMUX(84,	17,	3,	GPIO17_3,
		ct3,	uart7,	i2s1,	na,	na,	na,	i2c1),
	PMUX(85,	17,	4,	GPIO17_4,
		ct3,	uart8,	i2s1,	na,	na,	na,	na),
	PMUX(86,	17,	5,	GPIO17_5,
		qei0,	uart8,	i2s1,	na,	rmii0,	na,	na),
	PMUX(87,	17,	6,	GPIO17_6,
		qei0,	uart9,	i2s1,	na,	mii0,	na,	na),
	PMUX(88,	17,	7,	GPIO17_7,
		qei0,	uart9,	na,	na,	mii0,	na,	na),
};

#define MUX_TABLE_SIZE		ARRAY_SIZE(asm9260_mux_table)
struct asm9260_pmx_priv {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
	void __iomem		*iobase;

	struct clk		*clk;
	spinlock_t		lock;

	struct pinctrl_pin_desc	pin_desc[MUX_TABLE_SIZE];
};

static void __init asm9260_init_mux_pins(struct asm9260_pmx_priv *priv)
{
	unsigned int i;

	for (i = 0; i < MUX_TABLE_SIZE; i++) {
		priv->pin_desc[i].name = asm9260_mux_table[i].name;
		priv->pin_desc[i].number = asm9260_mux_table[i].number;
	}
}

/*
 * Pin control operations
 */

/* each GPIO pin has it's own pseudo pingroup containing only itself */
static int asm9260_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return MUX_TABLE_SIZE;
}

static const char *asm9260_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						 unsigned int group)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->pin_desc[group].name;
}

static int asm9260_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					 unsigned int group,
					 const unsigned int **pins,
					 unsigned int *num_pins)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	*pins = &priv->pin_desc[group].number;
	*num_pins = 1;

	return 0;
}

static struct pinctrl_ops asm9260_pinctrl_ops = {
	.get_groups_count	= asm9260_pinctrl_get_groups_count,
	.get_group_name		= asm9260_pinctrl_get_group_name,
	.get_group_pins		= asm9260_pinctrl_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_pin,
	.dt_free_map		= pinctrl_utils_dt_free_map,
};

/*
 * Pin mux operations
 */
static int asm9260_pinctrl_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(asm9260_functions);
}

static const char *asm9260_pinctrl_get_func_name(struct pinctrl_dev *pctldev,
						unsigned int function)
{
	return asm9260_functions[function].name;
}

static int asm9260_pinctrl_get_func_groups(struct pinctrl_dev *pctldev,
					  unsigned int function,
					  const char * const **groups,
					  unsigned int * const num_groups)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct asm9260_pingroup *table;
	int a, b, count = 0, *tmp;
	const char **gr;

	if (asm9260_functions[function].groups != NULL)
		goto done;

	tmp = devm_kmalloc(priv->dev, sizeof(tmp) * MUX_TABLE_SIZE, GFP_KERNEL);
	if (!tmp) {
		dev_err(priv->dev, "Can't allocate func/pin array\n");
		return PTR_ERR(tmp);
	}

	for (a = 0; a < MUX_TABLE_SIZE; a++) {
		table = &asm9260_mux_table[a];

		for (b = 0; b < MAX_FUNCS_PER_PIN; b++) {
			if (table->funcs[b] == function) {
				tmp[count] = a;
				count++;
			}

		}

	}

	gr = devm_kmalloc(priv->dev,
			sizeof(gr) * count, GFP_KERNEL);
	if (!gr) {
		dev_err(priv->dev, "Can't allocate func group\n");
		devm_kfree(priv->dev, tmp);
		return PTR_ERR(gr);
	}

	for (a = 0; a < count; a++)
		gr[a] = asm9260_mux_table[tmp[a]].name;

	asm9260_functions[function].groups = gr;
	asm9260_functions[function].ngroups = count;
done:
	*groups = asm9260_functions[function].groups;
	*num_groups = asm9260_functions[function].ngroups;
	return 0;
}

static int asm9260_pinctrl_set_mux(struct pinctrl_dev *pctldev,
				 unsigned int function, unsigned int pin)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct asm9260_pingroup *table;
	void __iomem            *offset;
	int		mux;
	u32		val;

	table = &asm9260_mux_table[pin];
	for (mux = 0; mux < MAX_FUNCS_PER_PIN; ++mux) {
		if (table->funcs[mux] == function)
			goto found_mux;
	}

	return -EINVAL;

found_mux:
	offset = priv->iobase + MUX_OFFSET(table->bank, table->pin);
	spin_lock(&priv->lock);
	val = ioread32(offset);
	val &= ~IOCON_PINMUX_MASK;
	val |= mux;
	iowrite32(val, offset);
	spin_unlock(&priv->lock);

	return 0;
}

static int asm9260_gpio_request_enable(struct pinctrl_dev *pctrldev,
		struct pinctrl_gpio_range *range, unsigned pin)
{
	struct asm9260_pingroup *table;
	int a;

	for (a = 0; a < MUX_TABLE_SIZE; a++) {
		table = &asm9260_mux_table[a];
		if (table->number == pin)
			break;
	}

	return asm9260_pinctrl_set_mux(pctrldev, ASM9260_MUX_gpio0, a);
}

static struct pinmux_ops asm9260_pinmux_ops = {
	.get_functions_count	= asm9260_pinctrl_get_funcs_count,
	.get_function_name	= asm9260_pinctrl_get_func_name,
	.get_function_groups	= asm9260_pinctrl_get_func_groups,
	.set_mux		= asm9260_pinctrl_set_mux,
	.gpio_request_enable	= asm9260_gpio_request_enable,
};

static int asm9260_pinconf_reg(struct pinctrl_dev *pctldev,
			      unsigned int pin,
			      enum pin_config_param param,
			      void __iomem **reg, u32 *val)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct asm9260_pingroup *table;
	int a;

	for (a = 0; a < MUX_TABLE_SIZE; a++) {
		table = &asm9260_mux_table[a];
		if (table->number == pin)
			break;
	}

	*reg = priv->iobase + MUX_OFFSET(table->bank, table->pin);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		*val = IOCON_MODE_NONE;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (table->bank != 0)
			return -ENOTSUPP;
		*val = IOCON_MODE_PULL_UP;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (table->bank == 0)
			return -ENOTSUPP;
		*val = IOCON_MODE_PULL_DOWN;
		break;
	default:
		return -ENOTSUPP;
	};

	return 0;
}

static int asm9260_pinconf_get(struct pinctrl_dev *pctldev,
			      unsigned int pin, unsigned long *config)
{
	enum pin_config_param param = pinconf_to_config_param(*config);
	void __iomem	*reg;
	u32 val, tmp, arg;
	int ret;

	/* Get register information */
	ret = asm9260_pinconf_reg(pctldev, pin, param,
				 &reg, &val);
	if (ret < 0)
		return ret;

	/* Extract field from register */
	tmp = ioread32(reg);
	arg = (tmp & IOCON_MODE_MASK) == val;
	if (!arg)
		return -EINVAL;

	/* And pack config */
	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int asm9260_pinconf_set(struct pinctrl_dev *pctldev,
			      unsigned int pin, unsigned long *configs,
			      unsigned num_configs)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	unsigned int arg;
	int ret;
	u32 val, tmp;
	void __iomem *reg;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		/* Get register information */
		ret = asm9260_pinconf_reg(pctldev, pin, param,
					 &reg, &val);
		if (ret < 0)
			return ret;

		spin_lock(&priv->lock);
		tmp = ioread32(reg);
		tmp &= ~IOCON_MODE_MASK;
		if (arg)
			tmp |= val;
		iowrite32(tmp, reg);
		spin_unlock(&priv->lock);
	}

	return 0;
}

static struct pinconf_ops asm9260_pinconf_ops = {
	.is_generic			= true,
	.pin_config_get			= asm9260_pinconf_get,
	.pin_config_set			= asm9260_pinconf_set,
};

/*
 * Pin control driver setup
 */
static struct pinctrl_desc asm9260_pinctrl_desc = {
	.pctlops	= &asm9260_pinctrl_ops,
	.pmxops		= &asm9260_pinmux_ops,
	.confops	= &asm9260_pinconf_ops,
	.owner		= THIS_MODULE,
};

static int asm9260_pinctrl_probe(struct platform_device *pdev)
{
	struct asm9260_pmx_priv *priv;
	struct resource	*res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Can't alloc asm9260_priv\n");
		return -ENOMEM;
	}
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->lock);

	asm9260_init_mux_pins(priv);

	asm9260_pinctrl_desc.name = dev_name(&pdev->dev);
	asm9260_pinctrl_desc.pins = priv->pin_desc;
	asm9260_pinctrl_desc.npins = MUX_TABLE_SIZE;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->iobase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->iobase))
		return PTR_ERR(priv->iobase);

	priv->clk = devm_clk_get(&pdev->dev, "ahb");
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clk!\n");
		return ret;
	}

	priv->pctl = pinctrl_register(&asm9260_pinctrl_desc, &pdev->dev, priv);
	if (!priv->pctl) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		ret = -ENODEV;
		goto err_return;
	}

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "ASM9260 pinctrl driver initialised\n");

	return 0;
err_return:
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int asm9260_pinctrl_remove(struct platform_device *pdev)
{
	struct asm9260_pmx_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk);
	pinctrl_unregister(priv->pctl);

	return 0;
}

static struct of_device_id asm9260_pinctrl_of_match[] = {
	{ .compatible = "alphascale,asm9260-pinctrl", },
	{ },
};
MODULE_DEVICE_TABLE(of, asm9260_pinctrl_of_match);

static struct platform_driver asm9260_pinctrl_driver = {
	.driver = {
		.name		= "asm9260-pinctrl",
		.owner		= THIS_MODULE,
		.of_match_table	= asm9260_pinctrl_of_match,
	},
	.probe	= asm9260_pinctrl_probe,
	.remove	= asm9260_pinctrl_remove,
};
module_platform_driver(asm9260_pinctrl_driver);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("Alphascale ASM9260 pinctrl driver");
MODULE_LICENSE("GPL");
