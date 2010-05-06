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

#include <linux/clockchips.h>
#include <linux/init.h>
#include <linux/interrupt.h>

static void camelot_set_mode(enum clock_event_mode mode,
                             struct clock_event_device *evt)
{
	unsigned int i = readl(TIMER1_SET);

	switch (mode) {
		case CLOCK_EVT_MODE_UNUSED:
			break;
		case CLOCK_EVT_MODE_SHUTDOWN:
			i &= ~TIMER_ENABLE;
			break;
		case CLOCK_EVT_MODE_PERIODIC:
			i |= (TIMER_PERIODIC | TIMER_ENABLE);
			break;
		case CLOCK_EVT_MODE_ONESHOT:
			i &= ~TIMER_PERIODIC;
			/* Fall through */
		case CLOCK_EVT_MODE_RESUME:
			i |= TIMER_ENABLE;
			break;
	}

	writel(i, TIMER1_SET);
}

struct clock_event_device camelot_clockevent = {
	.name       	= "camelot",
	.features   	= CLOCK_EVT_FEAT_PERIODIC,
	.rating     	= 100,
	.irq        	= IRQ_TIMER1,
	.set_mode	= camelot_set_mode,
	.mult		= 1
};

#define	TIMER_PRESCALE	(4)
#define TICKS_PER_SEC	(SYS_CLK / (TIMER_PRESCALE + 1))
#define TICKS_PER_JIFFY	(TICKS_PER_SEC / HZ)

static void camelot_timer_init(void)
{
	/* Write to compare register */
	writel(TICKS_PER_JIFFY, TIMER1_COMPARE);

	/* Set prescaler */
	writel(TIMER_PRESCALE, TIMER1_PRESCALE);

	/* Enable interrupt */
	writel(readl(TIMER1_SET) | TIMER_INT_ENABLE, TIMER1_SET);
}

static irqreturn_t camelot_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd = dev_id;

	/* Clear interrupt */
	writel(0x01, TIMER1_CLEAR);

	cd->event_handler(cd);

	return IRQ_HANDLED;
}

static struct irqaction camelot_timer_irqaction = {
	.handler	= camelot_timer_interrupt,
	.flags		= IRQF_DISABLED,
	.name		= "camelot_timer",
};

void __init plat_time_init(void)
{
	unsigned int cpu = smp_processor_id();

	BUG_ON(HZ != 100);

	camelot_clockevent.cpumask = cpumask_of(cpu);

	clockevents_register_device(&camelot_clockevent);
	camelot_timer_irqaction.dev_id = &camelot_clockevent;
	setup_irq(IRQ_TIMER1, &camelot_timer_irqaction);

	camelot_timer_init();
}
