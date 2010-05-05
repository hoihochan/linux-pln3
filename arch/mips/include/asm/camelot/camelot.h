/*
 * Donald Chan, donald@plasternetworks.com
 * Copyright (C) 2010 Plaster Networks, LLC.  All rights reserved.
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
#ifndef __CAMELOT_H__
#define __CAMELOT_H__

#include <asm/addrspace.h>

/* MAC */
#define MAC_BASE		(void __iomem *)(0xa3003000)

#define MAC_MDIO_REG		(MAC_BASE + 0x00)
#define MAC_MDIO_START		(1 << 31)
#define MAC_MDIO_BUSY		(MAC_MDIO_START)
#define MAC_MDIO_WRITE		(1 << 30)
#define MAC_MDIO_PHY_SHIFT	(24)
#define MAC_MDIO_REG_SHIFT	(16)
#define MAC_MDIO_VALUE(val)	((val) & 0xffff)

#define MAC_CTRL_REG		(MAC_BASE + 0x04)
#define MAC_LOOK_BACK_ENABLE	(1 << 0)
#define MAC_RX_ENABLE		(1 << 1)
#define MAC_RX_FLOW_CTRL_ENABLE	(1 << 2)
#define MAC_LINK_UP_ENABLE	(1 << 3)
#define MAC_RX_DMA_ENABLE	(1 << 4)
#define MAC_TX_ENABLE		(1 << 8)
#define MAC_TX_FLOW_CTRL_ENABLE	(1 << 9)
#define MAC_BYPASS_LOOKUP	(1 << 10)
#define MAC_TX_DMA_ENABLE	(1 << 12)
#define MAC_TX_RETRY_ENABLE	(1 << 13)
#define MAC_BW_CTRL_DROP_ENABLE	(1 << 20)
#define MAC_BW_CTRL_TRIG_ENABLE	(1 << 21)
#define MAC_MDIO_CLK(x)		((x) << 24)
#define MAC_CTRL_INIT		(MAC_RX_FLOW_CTRL_ENABLE | \
				MAC_TX_FLOW_CTRL_ENABLE | \
				MAC_LINK_UP_ENABLE | \
				MAC_TX_RETRY_ENABLE | \
				MAC_BYPASS_LOOKUP | \
				MAC_BW_CTRL_DROP_ENABLE | \
				MAC_BW_CTRL_TRIG_ENABLE | \
				MAC_MDIO_CLK(0xd))

#define MAC_RX_DESC_REG		(MAC_BASE + 0x08)
#define MAC_FIRST_BUFFER_REG	(MAC_BASE + 0x10)
#define MAC_ADDR0_REG		(MAC_BASE + 0x3c)
#define MAC_ADDR1_REG		(MAC_BASE + 0x40)
#define MAC_RESET_REG		(MAC_BASE + 0x5c)
#define MAC_TX_DESC_REG		(MAC_BASE + 0x64)

#define MAC_IRQ_MASK_REG	(MAC_BASE + 0x84)
#define MAC_IRQ_CLEAR_REG	(MAC_BASE + 0x88)
#define MAC_IRQ_STATUS_REG	(MAC_BASE + 0x8c)
#define MAC_IRQ_RES		(1 << 0)	/* Reserved */
#define MAC_IRQ_TX		(1 << 1)	/* TX available */
#define MAC_IRQ_SDE		(1 << 2)	/* Software Descriptor Empty */
#define MAC_IRQ_HDE		(1 << 3)	/* Hardware Descriptor Empty */
#define MAC_IRQ_BE		(1 << 4)	/* Buffer Empty */
#define MAC_IRQ_RX		(1 << 5)	/* RX available */
#define MAC_IRQ_MASK_ENABLE	(MAC_IRQ_RES | MAC_IRQ_TX | MAC_IRQ_SDE | \
				MAC_IRQ_BE | MAC_IRQ_RX)

#define MAC_DROPS_REG		(MAC_BASE + 0x30)
#define MAC_START_TX_REG	(MAC_BASE + 0xc0)
#define MAC_LAST_BUFFER_REG	(MAC_BASE + 0xd4)

#define	TIMER_BASE		(void __iomem *)(0xa3001000)

/* Timer Flags */
#define WATCHDOG_ENABLE		(0x8)
#define TIMER_ENABLE		(0x4)
#define TIMER_INT_ENABLE	(0x2)
#define TIMER_PERIODIC		(0x1)

/* Timer 0 */
#define TIMER0_COUNT		(TIMER_BASE + 0x0)
#define TIMER0_COMPARE		(TIMER_BASE + 0x4)
#define TIMER0_PRESCALE		(TIMER_BASE + 0x8)
#define TIMER0_SET		(TIMER_BASE + 0xc)
#define TIMER0_CLEAR		(TIMER_BASE + 0x10)

/* Timer 1 */
#define TIMER1_COUNT		(TIMER_BASE + 0x14)
#define TIMER1_COMPARE		(TIMER_BASE + 0x18)
#define TIMER1_PRESCALE		(TIMER_BASE + 0x1c)
#define TIMER1_SET		(TIMER_BASE + 0x20)
#define TIMER1_CLEAR		(TIMER_BASE + 0x24)

/* Timer 2 (Watchdog) */
#define WATCHDOG_COUNT		(TIMER_BASE + 0x28)
#define WATCHDOG_COMPARE	(TIMER_BASE + 0x2c)
#define WATCHDOG_PRESCALE	(TIMER_BASE + 0x30)
#define WATCHDOG_SET		(TIMER_BASE + 0x34)
#define WATCHDOG_CLEAR		(TIMER_BASE + 0x38)

#define GPIO_BASE		(void __iomem *)(0xa3005000)

/* GPIO data */
#define GPIO_DATA		(GPIO_BASE + 0x0)

/* GPIO direction (1 for output) */
#define GPIO_DIR		(GPIO_BASE + 0xc)

/* GPIO enable */
#define GPIO_ENABLE		(GPIO_BASE + 0x10)

#define IRQ_BASE		(void __iomem *)(0xa3004000)

#define IRQ_SOURCE_TIMER0	(15)
#define IRQ_SOURCE_TIMER1	(14)
#define IRQ_SOURCE_UART		(9)
#define IRQ_SOURCE_WATCHDOG	(8)
#define IRQ_SOURCE_ETHERNET	(13)

#define IRQ_TIMER0		(7)
#define IRQ_TIMER1		(6)
#define IRQ_UART		(5)
#define IRQ_WATCHDOG		(4)
#define IRQ_ETHERNET		(3)

#define IRQ_SET			(IRQ_BASE + 0x00)
#define IRQ_MASK		(IRQ_BASE + 0x04)
#define IRQ_MAP1		(IRQ_BASE + 0x08)
#define IRQ_MAP2		(IRQ_BASE + 0x0c)

#define UART_BASE		(void __iomem *)(0xa3002000)
#define UART_BUFFER		(UART_BASE + 0x00)
#define UART_CONTROL		(UART_BASE + 0x04)

#define UART_TX_BUSY		(1 << 0)
#define UART_RX_FULL		(1 << 1)
#define UART_TX_EMPTY		(1 << 2)
#define UART_TX_FULL		(1 << 3)
#define UART_RX_THRESHOLD	(1 << 4)
#define UART_FRAME_ERR		(1 << 7)
#define UART_PARITY_ERR		(1 << 8)
#define UART_PARITY_NOW		(1 << 9)
#define UART_ENABLE_LOOPBACK	(1 << 10)
#define UART_TWO_STOP_BIT	(1 << 11)
#define UART_EVEN_PARITY	(1 << 12)
#define UART_PARITY_ENABLE	(1 << 13)
#define UART_RX_INT_ENABLE	(1 << 14)
#define UART_TX_INT_ENABLE	(1 << 15)
#define UART_BAUD_RATE_SHIFT	(16)
#define UART_RX_READY		(1 << 23)
#define UART_BUFFER_SHIFT	(24)

#define	SYS_CLK			(150000000)

#endif
