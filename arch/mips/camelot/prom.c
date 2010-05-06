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
#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/camelot/camelot.h>

#include <linux/init.h>
#include <linux/string.h>

int prom_argc;
int *_prom_argv, *_prom_envp;

void  __init prom_init_cmdline(void)
{
	strncpy(arcs_cmdline, CONFIG_CMDLINE, strlen(CONFIG_CMDLINE) + 1);
}

void __init prom_meminit(void)
{
	/* Add 32MB of RAM */
	add_memory_region(0, 0x2000000, BOOT_MEM_RAM);
}

void __init prom_init(void)
{
	prom_argc = fw_arg0;
	_prom_argv = (int *) fw_arg1;
	_prom_envp = (int *) fw_arg2;

	prom_init_cmdline();
}

void __init prom_free_prom_memory(void)
{
	;
}

/* Needed for early printk */
static inline void serial_out(char c)
{
	writel(c << UART_BUFFER_SHIFT, UART_BUFFER);
}

int prom_putchar(char c)
{
	while (readl(UART_CONTROL) & UART_TX_FULL)
		;

	serial_out(c);

	return 1;
}
