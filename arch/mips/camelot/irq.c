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
#include <asm/io.h>
#include <asm/camelot/camelot.h>

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#ifdef CONFIG_MIPS_MT_SMP
    #error CONFIG_MIPS_MT_SMP not supported!
#endif

#define CAMELOT_ALL_INTS	(IE_IRQ0 | IE_IRQ1 | IE_IRQ2 | \
					IE_IRQ3 | IE_IRQ4 | IE_IRQ5)

static DEFINE_SPINLOCK(camelot_irq_lock);

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = 0;
	int i = 0;

	/* We only need bits 8 - 15 */
	pending = (read_c0_cause() & read_c0_status()) >> 8;

	for (i = 7; i > 1; i--)
		if (pending & (1 << i)) {
			do_IRQ(i);
			return;
		}

	spurious_interrupt();
}

static void camelot_mask_irq(unsigned int irq)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&camelot_irq_lock, flags);
	writel(readl(IRQ_MASK) | (1 << irq), IRQ_MASK);
	spin_unlock_irqrestore(&camelot_irq_lock, flags);
}

static void camelot_ack_irq(unsigned int irq)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&camelot_irq_lock, flags);
	writel(readl(IRQ_SET) | (1 << irq), IRQ_SET);
	spin_unlock_irqrestore(&camelot_irq_lock, flags);
}

static void camelot_mask_ack_irq(unsigned int irq)
{
	unsigned long flags = 0;
	
	spin_lock_irqsave(&camelot_irq_lock, flags);
	writel(readl(IRQ_MASK) | (1 << irq), IRQ_MASK);
	writel(readl(IRQ_SET) | (1 << irq), IRQ_SET);
	spin_unlock_irqrestore(&camelot_irq_lock, flags);
}

static void camelot_unmask_irq(unsigned int irq)
{
	unsigned long flags = 0;
	
	spin_lock_irqsave(&camelot_irq_lock, flags);
	writel(readl(IRQ_MASK) & ~(1 << irq), IRQ_MASK);
	spin_unlock_irqrestore(&camelot_irq_lock, flags);
}

static struct irq_chip camelot_irq_controller = {
	.name       = "Camelot Interrupt Controller",
	.ack        = camelot_ack_irq,
	.mask       = camelot_mask_irq,
	.mask_ack   = camelot_mask_ack_irq,
	.unmask     = camelot_unmask_irq,
};

void __init arch_init_irq(void)
{
	unsigned int i = 0;
	const int shift = 2;

	/* Disable all hardware interrupts */
	change_c0_status(ST0_IM, 0x00);

	/*
	 * Hardware interrupt sources will get mapped to CPU interrupts
	 * 2 through 7. Below is the mapping:
	 *
	 * Timer 0 (15)		-> IRQ Source 7 -> CPU IRQ 5
	 * Timer 1 (14)		-> IRQ Source 6	-> CPU IRQ 4 
	 * UART (9)		-> IRQ Source 5	-> CPU IRQ 3
	 * Watchdog (8)		-> IRQ Source 4	-> CPU IRQ 2
	 * Ethernet (13)	-> IRQ Source 3	-> CPU IRQ 1
	 *
	 */
	writel(0, IRQ_MAP1);

	i = (IRQ_SOURCE_TIMER0 << (IRQ_TIMER0 << shift));
	i |= (IRQ_SOURCE_TIMER1 << (IRQ_TIMER1 << shift));
	i |= (IRQ_SOURCE_UART << (IRQ_UART << shift));
	i |= (IRQ_SOURCE_WATCHDOG << (IRQ_WATCHDOG << shift));
	i |= (IRQ_SOURCE_ETHERNET << (IRQ_ETHERNET << shift));

	writel(i, IRQ_MAP2);

	/*
	 * Disable all interrupts
	 * 
	 * 0xfffcfffc is the power-on default value
	 */
	writel(0xfffcfffc, IRQ_SET);
	writel(0xfffcfffc, IRQ_MASK);

    	/* Initialize IRQ action handlers */
	for (i = 0; i < 16; i++)
    	{
		set_irq_chip_and_handler(i, &camelot_irq_controller, handle_level_irq);
    	}

	/* Enable all interrupts */
	change_c0_status(ST0_IM, CAMELOT_ALL_INTS);
}
