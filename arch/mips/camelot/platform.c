/*
 * Donald Chan, donald@plasternetworks.com
 * Copyright (C) 2009 Plaster Networks, LLC.  All rights reserved.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */
#include <asm/camelot/camelot.h>
#include <asm/camelot/gpio.h>

#include <linux/gpio_mouse.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>

static struct mtd_partition camelot_partitions[] = {
	{
		.name	= "boot",
		.offset	= 0x00000000,
		.size	= 0x00010000,
	},
	{
		.name	= "config",
		.offset	= 0x00010000,
		.size	= 0x00010000,
	},
	{
		.name	= "kernel",
		.offset = 0x00020000,
		.size	= 0x00100000,
	},
	{
		.name	= "other",
		.offset	= 0x00120000,
		.size	= 0x00280000,
	},
	{
		.name	= "data",
		.offset	= 0x003a0000,
		.size	= 0x00060000,
	},
};

static struct flash_platform_data camelot_spi_slave_data[] = {
	{
		.type		= "mx25l12805d",
		.nr_parts	= ARRAY_SIZE(camelot_partitions),
		.parts		= camelot_partitions,
	},
};

static struct flash_platform_data camelot_spi_slave_data1[] = {
	{
		.type		= "mx25l3205d",
		.nr_parts	= ARRAY_SIZE(camelot_partitions),
		.parts		= camelot_partitions,
	},
};

static struct spi_board_info camelot_spi_slave_info[] = {
	{
		.modalias       = "m25p80",
		.platform_data  = &camelot_spi_slave_data,
		.max_speed_hz   = 50000000,
		.bus_num        = 0,
		.chip_select    = 0,
	},
	{
		.modalias	= "m25p80",
		.platform_data	= &camelot_spi_slave_data1,
		.max_speed_hz	= 50000000,
		.bus_num	= 0,
		.chip_select	= 1,
	}
};

		
static struct resource resources_spi[] = {
	{
		.start	= 0xbffffff0,
		.end	= 0xbffffffb,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device camelot_spi = {
	.name	= "camelot_spi",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_spi),
	.resource	= resources_spi,
};

static struct resource resources_uart[] = {
	{
		.start	= IRQ_UART,
		.end	= IRQ_UART,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= (phys_addr_t)(UART_BASE),
		.end	= (phys_addr_t)(UART_BASE + 0x8 - 1),
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device camelot_uart = {
	.name	= "camelot_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart),
	.resource	= resources_uart,
};

static struct gpio_led default_leds[] = {
	{
		.name = "status",
		.gpio = 2,
		.active_low = 1,
	},
};

static struct gpio_led_platform_data camelot_led_data = {
	.num_leds = 1,
	.leds = default_leds,
};

static struct platform_device camelot_gpio_leds = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &camelot_led_data,
	}
};

static struct resource camelot_wdt_res = {
	.name = "regs",
	.start = (phys_addr_t)WATCHDOG_COUNT,
	.end = (phys_addr_t)(WATCHDOG_COUNT + 0x14),
	.flags = IORESOURCE_MEM,
};

static struct platform_device camelot_wdt = {
	.id = -1,
	.name = "camelot_wdt",
	.resource = &camelot_wdt_res,
	.num_resources = 1,
};

static struct gpio_mouse_platform_data camelot_gpio_mouse_data = {
	.scan_ms	= 100,
	.polarity	= GPIO_MOUSE_POLARITY_ACT_LOW,
	{
		{
			.up 		= 0,
			.down 		= 0,
			.left		= 0,
			.right		= 0,
			.bleft		= 5,
			.bmiddle	= 0,
			.bright		= 0
		},
	}
};

static struct platform_device camelot_gpio_mouse = {
	.name = "gpio_mouse",
	.id = -1,
	.dev = {
		.platform_data = &camelot_gpio_mouse_data
	}
};

static struct platform_device camelot_mac = {
	.name = "camelot_mac",
	.id = -1
};

static int __init camelot_register_devices(void)
{
	int res = 0;

	spi_register_board_info(camelot_spi_slave_info, ARRAY_SIZE(camelot_spi_slave_info));

	res = platform_device_register(&camelot_spi);
	
	if (res)
		return res;

	res = platform_device_register(&camelot_uart);

	if (res)
		return res;

	/* Enable status LED */
	camelot_gpio_enable(2);

	/* Enable reset button */
	camelot_gpio_enable(5);

	res = platform_device_register(&camelot_gpio_leds);

	if (res)
		return res;

	res = platform_device_register(&camelot_wdt);

	if (res)
		return res;

	res = platform_device_register(&camelot_gpio_mouse);

	if (res)
		return res;

	res = platform_device_register(&camelot_mac);

	return res;
}
arch_initcall(camelot_register_devices);
