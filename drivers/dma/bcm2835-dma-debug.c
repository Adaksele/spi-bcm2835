/*
 * helper module for bcm2835 dma debugging
 *
 * Copyright (C) 2014 Martin Sperl
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
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */
#include <linux/dma/bcm2835-dma.h>
#include <linux/printk.h>
#include <linux/module.h>

/* the code to debug the DMAs */
static int debug_dma = 0;
static int bcm2835_dma_param_set_debug_dma(
	const char *val, struct kernel_param *kp)
{
	int ret;
	void *base;
	dma_addr_t cbaddr;
	char buffer[768];

	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	/* extra checks */
	if (debug_dma<0)
		return -ENODEV;
	if (debug_dma>15)
		return -ENODEV;

	/* get the control registers adresses for access */
	base = ioremap(0x20007000+256*debug_dma,SZ_16K);
	if (!base)
		return -EPERM;
	/* start dumping DMA */
	/* maybe we should stop it first ?*/
	printk(KERN_INFO "Dumping DMA %i at bus address %pf\n",
		debug_dma,base);
	bcm2835_dma_reg_dump_str(
		base,1,
		buffer,sizeof(buffer)
		);
	printk(KERN_INFO "%s",buffer);

	/* get the control-block that is there right now */
	cbaddr = readl(base + BCM2835_DMA_ADDR);

	/* now dump the CBs and their loops */
	while (cbaddr) {
		cbaddr = 0;
	}

	/* and unmap the registers */
	iounmap(base);

	return 0;
}

module_param_call(debug_dma, bcm2835_dma_param_set_debug_dma,
		param_get_int,&debug_dma,0664);
MODULE_PARM_DESC(debug_dma, " write a value here to enable dumping dma");

MODULE_DESCRIPTION("bcm2835 dma debug");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL v2");
