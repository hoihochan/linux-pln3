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

#include <asm/cacheflush.h>
#include <asm/cpu-features.h>
#include <asm/reboot.h>
#include <asm/traps.h>
#include <asm/camelot/camelot.h>
#include <asm/mach-generic/spaces.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/string.h>

extern void prom_meminit(void);

#if defined(CONFIG_CAMELOT_WDT)
extern void camelot_wdt_disable(void);
#endif

const char *get_system_type(void)
{
	return "Camelot";
}

static void __init mips_nmi_setup(void)
{
	void *base;
	extern char except_vec_nmi;

	base = cpu_has_veic ?
		(void *)(CAC_BASE + 0xa80) :
		(void *)(CAC_BASE + 0x380);
	memcpy(base, &except_vec_nmi, 0x80);
	flush_icache_range((unsigned long)base, (unsigned long)base + 0x80);
}

static void __init mips_ejtag_setup(void)
{
	void *base;
	extern char except_vec_ejtag_debug;

	base = cpu_has_veic ?
		(void *)(CAC_BASE + 0xa00) :
		(void *)(CAC_BASE + 0x300);
	memcpy(base, &except_vec_ejtag_debug, 0x80);
	flush_icache_range((unsigned long)base, (unsigned long)base + 0x80);
}

static void camelot_machine_restart(char *command)
{
	unsigned int i = WATCHDOG_ENABLE | TIMER_ENABLE;

	/* Reset after 200 cycles */
	writel(1, WATCHDOG_PRESCALE);
	writel(200, WATCHDOG_COMPARE);
	writel(0, WATCHDOG_COUNT);
	writel(i, WATCHDOG_SET);
}

static void camelot_machine_halt(void)
{
	while (1)
		;
}

static void camelot_machine_power_off(void)
{
	camelot_machine_halt();
}

static void wbflush_camelot(void)
{
        __fast_iob();
}

void (*__wbflush) (void);
EXPORT_SYMBOL(__wbflush);

void __init plat_mem_setup(void)
{
	__wbflush = wbflush_camelot;
	_machine_restart = camelot_machine_restart;
	_machine_halt = camelot_machine_halt;
	pm_power_off = camelot_machine_power_off;
	panic_timeout = 3;

	set_io_port_base(KUSEG);

	prom_meminit();

	board_nmi_handler_setup = mips_nmi_setup;
	board_ejtag_handler_setup = mips_ejtag_setup;
	board_be_init = NULL;
	board_be_handler = NULL;
}
