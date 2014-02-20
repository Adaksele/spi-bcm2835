/*
 * helper code for bcm2835 dma controlblocks
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
#include <asm/io.h>

void bcm2835_dma_cb_dump(
	char *prefix,
	struct device *dev,
	struct bcm2835_dma_cb *dmablock,
	dma_addr_t dmablock_dma,
	int flags)
{
	struct bcm2835_dma_cb_stride *stride =
		(struct bcm2835_dma_cb_stride *)&dmablock->stride;
	dev_printk(KERN_INFO,dev,
		"%saddr:\t%pK\n"
		,prefix,dmablock);
	if (dmablock_dma)
		dev_printk(KERN_INFO,dev,
			"%sd_addr:\t%08llx\n"
			,prefix,(unsigned long long)dmablock_dma);
	dev_printk(KERN_INFO,dev,
		"%sti:\t%08x\n",
		prefix,dmablock->ti);
	dev_printk(KERN_INFO,dev,
		"%ssrc:\t%08x\n",
		prefix,dmablock->src);
	dev_printk(KERN_INFO,dev,
		"%sdst:\t%08x\n",
		prefix,dmablock->dst);
	dev_printk(KERN_INFO,dev,
		"%slength:\t%u\n",
		prefix,dmablock->length);
	dev_printk(KERN_INFO,dev,
		"%ss_str.:\t%i\n",
		prefix,stride->src);
	dev_printk(KERN_INFO,dev,
		"%sd_str.:\t%i\n",
		prefix,stride->dst);
	dev_printk(KERN_INFO,dev,
		"%snext:\t%08x\n",
		prefix,dmablock->next);
	dev_printk(KERN_INFO,dev,
		"%spad0:\t%08x\n",
		prefix,dmablock->pad[0]);
	dev_printk(KERN_INFO,dev,
		"%spad1:\t%08x\n",
		prefix,dmablock->pad[1]);
}
