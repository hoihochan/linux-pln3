/*
 * Donald Chan, donald@plasternetworks.com
 * Copyright (C) 2009 Plaster Networks, LLC.  All rights reserved.
 *
 * Some code taken from:
 * TI AR7 Watchdog Timer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <asm/addrspace.h>
#include <asm/camelot/camelot.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#define DRVNAME "camelot_wdt"
#define LONGNAME "Camelot Watchdog Timer"
#define PRESCALE_VALUE 0xfffff

MODULE_AUTHOR("Donald Chan <donald@plasternetworks.com>");
MODULE_DESCRIPTION(LONGNAME);
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

static int margin = 60;
module_param(margin, int, 0);
MODULE_PARM_DESC(margin, "Watchdog margin in seconds");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Disable watchdog shutdown on close");

struct camelot_wdt {
	u32 count;
	u32 compare;
	u32 prescale;
	u32 set;
	u32 clear;
};

static unsigned long wdt_is_open;
static spinlock_t wdt_lock;
static unsigned expect_close;

static struct resource *camelot_regs_wdt;
static struct camelot_wdt *camelot_wdt;

static void camelot_wdt_kick(void)
{
	writel(0, (void __iomem *)&camelot_wdt->count);
}

static void camelot_wdt_prescale(u32 value)
{
	writel(value, (void __iomem *)&camelot_wdt->prescale);
}

static void camelot_wdt_disable(void)
{
	writel(readl(WATCHDOG_SET) & ~(WATCHDOG_ENABLE | TIMER_ENABLE),
					(void __iomem *)&camelot_wdt->set);
}

static void camelot_wdt_change(u32 value)
{
	writel(value, (void __iomem *)&camelot_wdt->compare);
}

static void camelot_wdt_update_margin(int new_margin)
{
	u32 change;

	change = new_margin * (SYS_CLK / (PRESCALE_VALUE + 1));
	if (change < 1)
		change = 1;
	if (change > 0xfffff)
		change = 0xfffff;
	camelot_wdt_change(change);
	margin = change * ((PRESCALE_VALUE + 1) * 1000 / SYS_CLK);
	margin /= 1000;
	printk(KERN_INFO DRVNAME
	       ": timer margin %d seconds (prescale %d, change %d, freq %d)\n",
	       margin, PRESCALE_VALUE, change, SYS_CLK);
}

static void camelot_wdt_enable_wdt(void)
{
	camelot_wdt_kick();
	writel(readl(WATCHDOG_SET) | (WATCHDOG_ENABLE | TIMER_ENABLE),
					(void __iomem *)&camelot_wdt->set);
	printk(KERN_DEBUG DRVNAME ": enabling watchdog timer\n");
}

static void camelot_wdt_disable_wdt(void)
{
	printk(KERN_DEBUG DRVNAME ": disabling watchdog timer\n");
	camelot_wdt_disable();
}

static int camelot_wdt_open(struct inode *inode, struct file *file)
{
	/* only allow one at a time */
	if (test_and_set_bit(0, &wdt_is_open))
		return -EBUSY;
	camelot_wdt_enable_wdt();
	expect_close = 0;

	return nonseekable_open(inode, file);
}

static int camelot_wdt_release(struct inode *inode, struct file *file)
{
	if (!expect_close)
		printk(KERN_WARNING DRVNAME
		": watchdog device closed unexpectedly,"
		"will not disable the watchdog timer\n");
	else if (!nowayout)
		camelot_wdt_disable_wdt();
	clear_bit(0, &wdt_is_open);
	return 0;
}

static ssize_t camelot_wdt_write(struct file *file, const char *data,
			     size_t len, loff_t *ppos)
{
	/* check for a magic close character */
	if (len) {
		size_t i;

		spin_lock(&wdt_lock);
		camelot_wdt_kick();
		spin_unlock(&wdt_lock);

		expect_close = 0;
		for (i = 0; i < len; ++i) {
			char c;
			if (get_user(c, data + i))
				return -EFAULT;
			if (c == 'V')
				expect_close = 1;
		}

	}
	return len;
}

static long camelot_wdt_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	static struct watchdog_info ident = {
		.identity = LONGNAME,
		.firmware_version = 1,
		.options = (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
						WDIOF_MAGICCLOSE),
	};
	int new_margin;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		if (copy_to_user((struct watchdog_info *)arg, &ident,
				sizeof(ident)))
			return -EFAULT;
		return 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		if (put_user(0, (int *)arg))
			return -EFAULT;
		return 0;
	case WDIOC_KEEPALIVE:
		camelot_wdt_kick();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int *)arg))
			return -EFAULT;
		if (new_margin < 1)
			return -EINVAL;

		spin_lock(&wdt_lock);
		camelot_wdt_update_margin(new_margin);
		camelot_wdt_kick();
		spin_unlock(&wdt_lock);

	case WDIOC_GETTIMEOUT:
		if (put_user(margin, (int *)arg))
			return -EFAULT;
		return 0;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations camelot_wdt_fops = {
	.owner		= THIS_MODULE,
	.write		= camelot_wdt_write,
	.unlocked_ioctl	= camelot_wdt_ioctl,
	.open		= camelot_wdt_open,
	.release	= camelot_wdt_release,
};

static struct miscdevice camelot_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &camelot_wdt_fops,
};

static int __devinit camelot_wdt_probe(struct platform_device *pdev)
{
	int rc;

	spin_lock_init(&wdt_lock);

	camelot_regs_wdt =
		platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	if (!camelot_regs_wdt) {
		printk(KERN_ERR DRVNAME ": could not get registers resource\n");
		rc = -ENODEV;
		goto out;
	}

	if (!request_mem_region(camelot_regs_wdt->start,
				resource_size(camelot_regs_wdt), LONGNAME)) {
		printk(KERN_WARNING DRVNAME ": watchdog I/O region busy\n");
		rc = -EBUSY;
		goto out;
	}

	camelot_wdt = ioremap(camelot_regs_wdt->start, resource_size(camelot_regs_wdt));
	if (!camelot_wdt) {
		printk(KERN_ERR DRVNAME ": could not ioremap registers\n");
		rc = -ENXIO;
		goto out_mem_region;
	}

	camelot_wdt_disable_wdt();
	camelot_wdt_prescale(PRESCALE_VALUE);
	camelot_wdt_update_margin(margin);

	rc = misc_register(&camelot_wdt_miscdev);
	if (rc) {
		printk(KERN_ERR DRVNAME ": unable to register misc device\n");
		goto out_alloc;
	}
	goto out;

out_alloc:
	iounmap(camelot_wdt);
out_mem_region:
	release_mem_region(camelot_regs_wdt->start, resource_size(camelot_regs_wdt));
out:
	return rc;
}

static int __devexit camelot_wdt_remove(struct platform_device *pdev)
{
	misc_deregister(&camelot_wdt_miscdev);
	iounmap(camelot_wdt);
	release_mem_region(camelot_regs_wdt->start, resource_size(camelot_regs_wdt));

	return 0;
}

static void camelot_wdt_shutdown(struct platform_device *pdev)
{
	if (!nowayout)
		camelot_wdt_disable_wdt();
}

static struct platform_driver camelot_wdt_driver = {
	.probe = camelot_wdt_probe,
	.remove = __devexit_p(camelot_wdt_remove),
	.shutdown = camelot_wdt_shutdown,
	.driver = {
		.owner = THIS_MODULE,
		.name = "camelot_wdt",
	},
};

static int __init camelot_wdt_init(void)
{
	return platform_driver_register(&camelot_wdt_driver);
}

static void __exit camelot_wdt_cleanup(void)
{
	platform_driver_unregister(&camelot_wdt_driver);
}

module_init(camelot_wdt_init);
module_exit(camelot_wdt_cleanup);
