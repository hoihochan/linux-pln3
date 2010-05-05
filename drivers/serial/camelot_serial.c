/*
 * Donald Chan, donald@plasternetworks.com
 * Copyright (C) 2009 Plaster Networks, LLC.  All rights reserved.
 *
 * Code derived from drivers/serial/msm_serial.c
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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty.h>

struct camelot_port {
	struct uart_port	uart;
	char			name[16];
};

#define UART_TO_CAMELOT(uart_port)	((struct camelot_port *) uart_port)

#define UART_BUFFER_READ(x)		(readl((void __iomem *)(x)))
#define UART_BUFFER_WRITE(y, x)		(writel((y), (void __iomem *)(x)))
#define UART_CTRL_READ(x)		(readl((void __iomem *)((x) + 4)))
#define UART_CTRL_WRITE(y, x)		(writel((y), (void __iomem *)((x) + 4)))

static void camelot_stop_tx(struct uart_port *port)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&port->lock, flags);
	UART_CTRL_WRITE(UART_CTRL_READ(port->mapbase) & ~UART_TX_INT_ENABLE,
			port->mapbase);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void camelot_start_tx(struct uart_port *port)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&port->lock, flags);
	UART_CTRL_WRITE(UART_CTRL_READ(port->mapbase) | UART_TX_INT_ENABLE,
			port->mapbase);
	/*
	 * FIXME: Seems we need to fill the buffer with
	 * something to cause an interrupt, strange...
	 */
	UART_BUFFER_WRITE(0 << UART_BUFFER_SHIFT, port->mapbase);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void camelot_stop_rx(struct uart_port *port)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&port->lock, flags);
	UART_CTRL_WRITE(UART_CTRL_READ(port->mapbase) & ~UART_RX_INT_ENABLE,
			port->mapbase);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void camelot_enable_ms(struct uart_port *port)
{
	/* Do nothing, we don't support it */
	;
}

static void handle_rx(struct uart_port *port)
{
	struct tty_struct *tty = port->state->port.tty;
	unsigned int c = 0;

	while ((c = UART_BUFFER_READ(port->mapbase)) & UART_RX_READY) {
		char flag = TTY_NORMAL;

		c = (c >> UART_BUFFER_SHIFT) & 0xff;

		port->icount.rx++;

		if (!uart_handle_sysrq_char(port, c))
			tty_insert_flip_char(tty, c, flag);
	}

	tty_flip_buffer_push(tty);
}

static void handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int max_count = port->fifosize;

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		goto txq_empty;

	while (max_count--) {
		unsigned int c;

		if (port->x_char) {
			UART_BUFFER_WRITE(port->x_char << UART_BUFFER_SHIFT,
					port->mapbase);
			port->icount.tx++;
			port->x_char = 0;
		} else {
			c = xmit->buf[xmit->tail];
			UART_BUFFER_WRITE(c << UART_BUFFER_SHIFT,
					port->mapbase);
	
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
		}

		if (uart_circ_empty(xmit))
			goto txq_empty;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return;

txq_empty:
	camelot_stop_tx(port);

	return;
}

static irqreturn_t camelot_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned int status = 0;

	spin_lock(&port->lock);

	status = UART_CTRL_READ(port->mapbase);

	if (status & UART_TX_EMPTY)
		handle_tx(port);

	if (status & UART_RX_FULL)
		handle_rx(port);

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static unsigned int camelot_tx_empty(struct uart_port *port)
{
	return (UART_CTRL_READ(port->mapbase) & UART_TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int camelot_get_mctrl(struct uart_port *port)
{
	/* Return hard coded values according to Documentation/serial/driver */
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CD;
}

static void camelot_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* Do nothing, we don't support it */
	;
}

static void camelot_break_ctl(struct uart_port *port, int break_ctl)
{
	/* Do nothing, we don't support it */
	;
}

static int camelot_set_baud_rate(struct uart_port *port, unsigned int baud)
{
	unsigned int i = port->uartclk / baud;

	UART_CTRL_WRITE(UART_CTRL_READ(port->mapbase) |
			(i << UART_BAUD_RATE_SHIFT), port->mapbase);
	
	return baud;
}

static int camelot_startup(struct uart_port *port)
{
	int ret;

	ret = request_irq(IRQ_UART, camelot_irq, IRQF_DISABLED,
					"camelot_uart", port);

	if (unlikely(ret))
		return ret;

	/* Turn on RX interrupt */
	UART_CTRL_WRITE(UART_CTRL_READ(port->mapbase) | UART_RX_INT_ENABLE,
			port->mapbase);

	return 0;
}

static void camelot_shutdown(struct uart_port *port)
{
	UART_CTRL_WRITE(UART_CTRL_READ(port->mapbase) & ~UART_RX_INT_ENABLE,
			port->mapbase);

	free_irq(port->irq, port);
}

static void camelot_set_termios(struct uart_port *port, struct ktermios *termios,
			    struct ktermios *old)
{
	unsigned long flags = 0;
	unsigned int baud, i;

	spin_lock_irqsave(&port->lock, flags);

	/* calculate and set baud rate */
	baud = uart_get_baud_rate(port, termios, old, 300, 115200);
	baud = camelot_set_baud_rate(port, baud);

	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);

	i = UART_CTRL_READ(port->mapbase);

	if (termios->c_cflag & PARENB) {
		i |= UART_PARITY_ENABLE;

		if (termios->c_cflag & PARODD) {
			i &= ~UART_EVEN_PARITY;
		} else {
			i |= UART_EVEN_PARITY;
		}
	}
	
	if (termios->c_cflag & CSTOPB) {
		i |= UART_TWO_STOP_BIT;
	} else {
		i &= ~UART_TWO_STOP_BIT;
	}
	
	UART_CTRL_WRITE(i, port->mapbase);
	
	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *camelot_type(struct uart_port *port)
{
	return "Camelot UART";
}

static void camelot_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *resource;
	resource_size_t size;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return;
	size = resource->end - resource->start + 1;

	release_mem_region(port->mapbase, size);
	iounmap(port->membase);
	port->membase = NULL;
}

static int camelot_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *resource;
	resource_size_t size;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return -ENXIO;
	size = resource->end - resource->start + 1;

	if (unlikely(!request_mem_region(port->mapbase, size, "camelot_serial")))
		return -EBUSY;

	port->membase = ioremap(port->mapbase, size);
	if (!port->membase) {
		release_mem_region(port->mapbase, size);
		return -EBUSY;
	}

	return 0;
}

static void camelot_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_CAMELOT;
		camelot_request_port(port);
	}
}

static int camelot_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_CAMELOT))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}

static struct uart_ops camelot_uart_pops = {
	.tx_empty = camelot_tx_empty,
	.set_mctrl = camelot_set_mctrl,
	.get_mctrl = camelot_get_mctrl,
	.stop_tx = camelot_stop_tx,
	.start_tx = camelot_start_tx,
	.stop_rx = camelot_stop_rx,
	.enable_ms = camelot_enable_ms,
	.break_ctl = camelot_break_ctl,
	.startup = camelot_startup,
	.shutdown = camelot_shutdown,
	.set_termios = camelot_set_termios,
	.type = camelot_type,
	.release_port = camelot_release_port,
	.request_port = camelot_request_port,
	.config_port = camelot_config_port,
	.verify_port = camelot_verify_port,
};

static struct camelot_port camelot_uart_ports[] = {
	{
		.uart = {
			.iotype = UPIO_MEM32,
			.ops = &camelot_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.line = 0,
			.uartclk = SYS_CLK,
			.fifosize = 16,
		},
	},
};

#define UART_NR	ARRAY_SIZE(camelot_uart_ports)

static inline struct uart_port *get_port_from_line(unsigned int line)
{
	return &camelot_uart_ports[line].uart;
}

static void camelot_console_putchar(struct uart_port *port, int c)
{
	while (UART_CTRL_READ(port->mapbase) & UART_TX_FULL)
		;
	
	UART_BUFFER_WRITE(c << UART_BUFFER_SHIFT, port->mapbase);
}

static void camelot_console_write(struct console *co, const char *s,
			      unsigned int count)
{
	struct uart_port *port;
	struct camelot_port *camelot_port;

	BUG_ON(co->index < 0 || co->index >= UART_NR);

	port = get_port_from_line(co->index);
	camelot_port = UART_TO_CAMELOT(port);

	spin_lock(&port->lock);
	uart_console_write(port, s, count, camelot_console_putchar);
	spin_unlock(&port->lock);
}

static int __init camelot_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud, flow, bits, parity;
	unsigned int i = 0;

	if (unlikely(co->index >= UART_NR || co->index < 0))
		return -ENXIO;

	port = get_port_from_line(co->index);

	if (unlikely(!port->membase))
		return -ENXIO;

	port->cons = co;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	bits = 8;
	parity = 'n';
	flow = 'n';

	i = UART_CTRL_READ(port->mapbase);
	i &= ~UART_TWO_STOP_BIT;
	i &= ~UART_PARITY_ENABLE;

	UART_CTRL_WRITE(i, port->mapbase);

	/* FIXME: Fix baud rate for now */
	if (baud < 300 || baud > 115200)
		baud = 115200;

	camelot_set_baud_rate(port, 115200);

	printk(KERN_INFO "camelot_serial: console setup on port #%d\n", port->line);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver camelot_uart_driver;

static struct console camelot_console = {
	.name = "ttyS",
	.write = camelot_console_write,
	.device = uart_console_device,
	.setup = camelot_console_setup,
	.flags = CON_PRINTBUFFER | CON_ENABLED,
	.index = -1,
	.data = &camelot_uart_driver,
};

static struct uart_driver camelot_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "camelot_serial",
	.dev_name = "ttyS",
	.nr = UART_NR,
	.major = TTY_MAJOR,
	.minor = 64,
	.cons = &camelot_console,
};

static int __init camelot_serial_probe(struct platform_device *pdev)
{
	struct camelot_port *camelot_port;
	struct resource *resource;
	struct uart_port *port;

	if (unlikely(pdev->id < 0 || pdev->id >= UART_NR))
		return -ENXIO;

	printk(KERN_INFO "camelot_serial: detected port #%d\n", pdev->id);

	port = get_port_from_line(pdev->id);
	port->dev = &pdev->dev;
	camelot_port = UART_TO_CAMELOT(port);

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return -ENXIO;
	port->mapbase = resource->start;

	port->irq = platform_get_irq(pdev, 0);
	if (unlikely(port->irq < 0))
		return -ENXIO;

	platform_set_drvdata(pdev, port);

	return uart_add_one_port(&camelot_uart_driver, port);
}

static int __devexit camelot_serial_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver camelot_platform_driver = {
	.remove = camelot_serial_remove,
	.driver = {
		.name = "camelot_serial",
		.owner = THIS_MODULE,
	},
};

static int __init camelot_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&camelot_uart_driver);
	if (unlikely(ret))
		return ret;

	ret = platform_driver_probe(&camelot_platform_driver, camelot_serial_probe);
	if (unlikely(ret))
		uart_unregister_driver(&camelot_uart_driver);

	printk(KERN_INFO "camelot_serial: driver initialized\n");

	register_console(&camelot_console);

	return ret;
}

static void __exit camelot_serial_exit(void)
{
	unregister_console(&camelot_console);
	platform_driver_unregister(&camelot_platform_driver);
	uart_unregister_driver(&camelot_uart_driver);
}

module_init(camelot_serial_init);
module_exit(camelot_serial_exit);

MODULE_AUTHOR("Donald Chan <donald@plasternetworks.com>");
MODULE_DESCRIPTION("Driver for Camelot serial device");
MODULE_LICENSE("GPL");
