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
#ifndef __CAMELOT_GPIO_H
#define __CAMELOT_GPIO_H

#include <asm/camelot/camelot.h>

#include <linux/errno.h>
#include <linux/io.h>

#define CAMELOT_GPIO_MAX	(20)

extern int gpio_request(unsigned gpio, const char *label);
extern void gpio_free(unsigned gpio);

/* Common GPIO layer */
static inline int gpio_get_value(unsigned gpio)
{
	return (readl(GPIO_DATA) & (1 << gpio)) ? 1 : 0;
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	unsigned tmp;

	tmp = readl(GPIO_DATA) & ~(1 << gpio);

	if (value)
		tmp |= (1 << gpio);

	writel(tmp, GPIO_DATA);
}

static inline int gpio_direction_input(unsigned gpio)
{
	if (gpio >= CAMELOT_GPIO_MAX)
		return -EINVAL;

	writel(readl(GPIO_DIR) & ~(1 << gpio), GPIO_DIR);

	return 0;
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	if (gpio >= CAMELOT_GPIO_MAX)
		return -EINVAL;

	gpio_set_value(gpio, value);
	writel(readl(GPIO_DIR) | (1 << gpio), GPIO_DIR);

	return 0;
}

static inline int gpio_to_irq(unsigned gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

/* Board specific GPIO functions */
static inline int camelot_gpio_enable(unsigned gpio)
{
	writel(readl(GPIO_ENABLE) | (1 << gpio), GPIO_ENABLE);

	return 0;
}

static inline int camelot_gpio_disable(unsigned gpio)
{
	writel(readl(GPIO_ENABLE) & ~(1 << gpio), GPIO_ENABLE);

	return 0;
}

#include <asm-generic/gpio.h>

#endif
