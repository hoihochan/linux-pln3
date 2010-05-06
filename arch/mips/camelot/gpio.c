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
#include <asm/camelot/gpio.h>

#include <linux/module.h>

static const char *camelot_gpio_list[CAMELOT_GPIO_MAX];

int gpio_request(unsigned gpio, const char *label)
{
	if (gpio >= CAMELOT_GPIO_MAX)
		return -EINVAL;

	if (camelot_gpio_list[gpio] && strcmp(label, camelot_gpio_list[gpio]))
		return -EBUSY;

	if (label) {
		camelot_gpio_list[gpio] = label;
	} else {
		camelot_gpio_list[gpio] = "busy";
	}

	return 0;
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned gpio)
{
	BUG_ON(!camelot_gpio_list[gpio]);
	camelot_gpio_list[gpio] = NULL;
}
EXPORT_SYMBOL(gpio_free);
