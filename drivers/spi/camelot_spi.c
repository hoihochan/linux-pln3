/*
 * Donald Chan, donald@plasternetworks.com
 * Copyright (C) 2009 - 2010 Plaster Networks, LLC.  All rights reserved.
 *
 * Code derived from drivers/spi/orion_spi.c
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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME			"camelot_spi"

#define SPI_CTRL			(0)
#define SPI_ADDR			(4)
#define SPI_DATA			(8)
#define SPI_CLK				(0x330)
#define SPI_DATA_CNT			(4)
#define SPI_D_SHIFT			(24)

struct camelot_spi {
	struct work_struct	work;

	/* Lock access to transfer list.	*/
	spinlock_t		lock;

	struct list_head	msg_queue;
	struct spi_master	*master;
	void __iomem		*base;
	unsigned int		max_speed;
	unsigned int		min_speed;
	struct camelot_spi_info	*spi_info;
	unsigned long		state;
};

static struct workqueue_struct *camelot_spi_wq;

static inline void __iomem *spi_reg(struct camelot_spi *camelot_spi, u32 reg)
{
	return camelot_spi->base + reg;
}

static void inline camelot_wait_till_ready(struct spi_device *spi)
{
	struct camelot_spi *camelot_spi = spi_master_get_devdata(spi->master);
	u32 value = 0;

	while (1) {
		writel(SPI_CLK | (0x05 << SPI_D_SHIFT) | 0x01,
			spi_reg(camelot_spi, SPI_CTRL));

		mb();

		value = readl(spi_reg(camelot_spi, SPI_DATA)) >> 24;

		if (value & 0x01) {
			cond_resched();
		} else {
			break;
		}
	}
}

static unsigned int camelot_spi_write_read(struct spi_device *spi,
						struct spi_transfer *xfer)
{
	struct camelot_spi *camelot_spi = spi_master_get_devdata(spi->master);
	int count = xfer->len;
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;
	u32 value = 0;
	int i = 0;

	if (tx) {
		clear_bit(0, &camelot_spi->state);

		if (count >= 4 && !test_bit(1, &camelot_spi->state)) {
			/* Write address to address register */
			writel((tx[1] << 16) | (tx[2] << 8) | (tx[3]),
					spi_reg(camelot_spi, SPI_ADDR));
			mb();

			if (tx[0] == 0x02) {
				set_bit(1, &camelot_spi->state);
				return xfer->len;
			}
		}

		if (test_bit(1, &camelot_spi->state)) {
			const u8 *ptr = &tx[0];

			/*
		 	 * HACK: Too bad our SoC can only write 4
		 	 * bytes on the SPI bus at a time. The only command
			 * that will write more than 4 bytes is a page write
			 * on the flash memory, we are "faking" as if we can
			 * write more than 4 bytes by waiting for the status
			 * register to clear and then continue issuing data
			 * writes that will be transparent to the upper layer.
			 * This is fine if ONLY the flash chip resides on the
			 * SPI bus.
			 */
			while (count > 0) {
				value = 0;

				/*
				 * Shift at most 4 bytes into our
				 * 32-bit word buffer
				 */
				for (i = SPI_D_SHIFT; i >= 0 && count > 0;
						i -= 8, count--)
					value |= (*ptr++ << i);

				/* Write the value to the data register */
				writel(value, spi_reg(camelot_spi, SPI_DATA));

				mb();

				/* Issue the flash write command */
				writel(SPI_CLK | (0x02 << SPI_D_SHIFT) |
						((SPI_D_SHIFT - i) >> 3),
					spi_reg(camelot_spi, SPI_CTRL));

				mb();

				/* Wait till write is complete */
				camelot_wait_till_ready(spi);

				/* Increment address register by 4 */
				value = readl(spi_reg(camelot_spi, SPI_ADDR));
				value += 0x04;
				writel(value, spi_reg(camelot_spi, SPI_ADDR));
				mb();

				/* Write Enable */
				writel(SPI_CLK | (0x06 << SPI_D_SHIFT) | 0x01,
					spi_reg(camelot_spi, SPI_CTRL));

				mb();
			}

			clear_bit(1, &camelot_spi->state);

			return xfer->len;
		}

		if (count >= 5) {
			if (tx[0] == 0x0b && !test_bit(1, &camelot_spi->state))
				set_bit(0, &camelot_spi->state);

			/* Write data to data register */
			writel(tx[4] << SPI_D_SHIFT,
				spi_reg(camelot_spi, SPI_DATA));
		}

		writel(SPI_CLK | (tx[0] << SPI_D_SHIFT) | SPI_DATA_CNT,
				spi_reg(camelot_spi, SPI_CTRL));
	}

	count = xfer->len;

	if (rx && test_bit(0, &camelot_spi->state)) {
		/*
		 * HACK: Too bad our SoC can only read 4
		 * bytes on the SPI bus at a time. The only command
		 * that will read more than 4 bytes is a read on the
		 * flash memory, we are "faking" as if we can read more
		 * than 4 bytes by incrementing the address register and
		 * issuing additional SPI requests that will be transparent
		 * to the upper layer. This is fine ONLY if the
		 * flash chip resides on the SPI bus.
		 */
		while (count > 0) {
			value = readl(spi_reg(camelot_spi, SPI_DATA));

			for (i = SPI_D_SHIFT; i >= 0 && count > 0;
							i -= 8, count--)
				*rx++ = (value >> i) & 0xff;

			/* Still got data? */
			if (count > 0) {
				/* Increment address register by 4 */
				value = readl(spi_reg(camelot_spi, SPI_ADDR));
				value += 0x04;

				writel(value, spi_reg(camelot_spi, SPI_ADDR));

				mb();

				/* Write the dummy byte to data register */
				writel(0x01, spi_reg(camelot_spi, SPI_DATA));

				mb();

				/* Issue the flash read command */
				writel(SPI_CLK | (0x0b << SPI_D_SHIFT) |
					(min(count, SPI_DATA_CNT)),
					spi_reg(camelot_spi, SPI_CTRL));

				mb();
			}
		}

		clear_bit(0, &camelot_spi->state);
	} else if (rx) {
		value = readl(spi_reg(camelot_spi, SPI_DATA));

		/* Copy contents of data register to rx buffer */
		for (i = SPI_D_SHIFT; i >= 0 && count >= 0;
						i -= 8, count--)
			*rx++ = (value >> i) & 0xff;
	}

	return xfer->len;
}

static void camelot_spi_work(struct work_struct *work)
{
	struct camelot_spi *camelot_spi =
		container_of(work, struct camelot_spi, work);

	spin_lock_irq(&camelot_spi->lock);
	while (!list_empty(&camelot_spi->msg_queue)) {
		struct spi_message *m;
		struct spi_device *spi;
		struct spi_transfer *t = NULL;

		m = container_of(camelot_spi->msg_queue.next, struct spi_message,
				 queue);

		list_del_init(&m->queue);
		spin_unlock_irq(&camelot_spi->lock);

		spi = m->spi;

		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (t->len)
				m->actual_length +=
					camelot_spi_write_read(spi, t);

			if (t->delay_usecs)
				udelay(t->delay_usecs);
		}

		m->status = 0;
		m->complete(m->context);

		spin_lock_irq(&camelot_spi->lock);
	}

	spin_unlock_irq(&camelot_spi->lock);
}

static int __init camelot_spi_reset(struct camelot_spi *camelot_spi)
{
	/* Do nothing as we don't have a way to reset the SPI controller */
	return 0;
}

static int camelot_spi_setup(struct spi_device *spi)
{
	/* Do nothing as we don't allow changing SPI bus speed */
	return 0;
}

static int camelot_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct camelot_spi *camelot_spi;
	struct spi_transfer *t = NULL;
	unsigned long flags;

	m->actual_length = 0;
	m->status = 0;

	/* reject invalid messages and transfers */
	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	camelot_spi = spi_master_get_devdata(spi->master);

	list_for_each_entry(t, &m->transfers, transfer_list) {
		unsigned int bits_per_word = spi->bits_per_word;

		if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
			dev_err(&spi->dev,
				"message rejected : "
				"invalid transfer data buffers\n");
			goto msg_rejected;
		}

		if ((t != NULL) && t->bits_per_word)
			bits_per_word = t->bits_per_word;

		if (bits_per_word != 8) {
			dev_err(&spi->dev,
				"message rejected : "
				"invalid transfer bits_per_word (%d bits)\n",
				bits_per_word);
			goto msg_rejected;
		}

		if (t->speed_hz && t->speed_hz < camelot_spi->min_speed) {
			dev_err(&spi->dev,
				"message rejected : "
				"device min speed (%d Hz) exceeds "
				"required transfer speed (%d Hz)\n",
				camelot_spi->min_speed, t->speed_hz);
			goto msg_rejected;
		}
	}


	spin_lock_irqsave(&camelot_spi->lock, flags);
	list_add_tail(&m->queue, &camelot_spi->msg_queue);
	queue_work(camelot_spi_wq, &camelot_spi->work);
	spin_unlock_irqrestore(&camelot_spi->lock, flags);

	return 0;
msg_rejected:
	/* Message rejected and not queued */
	m->status = -EINVAL;
	if (m->complete)
		m->complete(m->context);
	return -EINVAL;
}

static int __init camelot_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct camelot_spi *spi;
	struct resource *r;
	struct camelot_spi_info *spi_info;
	int status = 0;

	spi_info = pdev->dev.platform_data;

	master = spi_alloc_master(&pdev->dev, sizeof *spi);
	if (master == NULL) {
		dev_dbg(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	if (pdev->id != -1)
		master->bus_num = pdev->id;

	/* we support only mode 0, and no options */
	master->mode_bits = 0;

	master->setup = camelot_spi_setup;
	master->transfer = camelot_spi_transfer;
	master->num_chipselect = 2;

	dev_set_drvdata(&pdev->dev, master);

	spi = spi_master_get_devdata(master);
	spi->master = master;
	spi->spi_info = spi_info;

	/* Clock speed fixed at 150 MHz / 3 = 50 MHz */
	spi->max_speed = DIV_ROUND_UP(SYS_CLK, 3);
	spi->min_speed = DIV_ROUND_UP(SYS_CLK, 3);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		status = -ENODEV;
		goto out;
	}

	if (!request_mem_region(r->start, (r->end - r->start) + 1,
				dev_name(&pdev->dev))) {
		status = -EBUSY;
		goto out;
	}

	spi->base = (void __iomem *)r->start;
	
	clear_bit(0, &spi->state);
	clear_bit(1, &spi->state);

	INIT_WORK(&spi->work, camelot_spi_work);

	spin_lock_init(&spi->lock);
	INIT_LIST_HEAD(&spi->msg_queue);

	if (camelot_spi_reset(spi) < 0)
		goto out_rel_mem;

	status = spi_register_master(master);
	if (status < 0)
		goto out_rel_mem;

	return status;

out_rel_mem:
	release_mem_region(r->start, (r->end - r->start) + 1);

out:
	spi_master_put(master);
	return status;
}

static int __devexit camelot_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master;
	struct camelot_spi *spi;
	struct resource *r;

	master = dev_get_drvdata(&pdev->dev);
	spi = spi_master_get_devdata(master);

	cancel_work_sync(&spi->work);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, (r->end - r->start) + 1);

	spi_unregister_master(master);

	return 0;
}

MODULE_ALIAS("platform:camelot_spi");

static struct platform_driver camelot_spi_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.remove		= __devexit_p(camelot_spi_remove),
};

static int __init camelot_spi_init(void)
{
	camelot_spi_wq = create_singlethread_workqueue(
				camelot_spi_driver.driver.name);
	if (camelot_spi_wq == NULL)
		return -ENOMEM;

	return platform_driver_probe(&camelot_spi_driver, camelot_spi_probe);
}
module_init(camelot_spi_init);

static void __exit camelot_spi_exit(void)
{
	flush_workqueue(camelot_spi_wq);
	platform_driver_unregister(&camelot_spi_driver);

	destroy_workqueue(camelot_spi_wq);
}
module_exit(camelot_spi_exit);

MODULE_DESCRIPTION("Camelot SPI driver");
MODULE_AUTHOR("Donald Chan <donald@plasternetworks.com>");
MODULE_LICENSE("GPL");
