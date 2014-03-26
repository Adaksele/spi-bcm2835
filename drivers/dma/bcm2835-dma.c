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
#include <linux/module.h>
#include <asm/io.h>

static const char *_tab_indent_string = "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
static inline const char *_tab_indent(int indent) {
	return &_tab_indent_string[16-min(16,indent)];
}

void bcm2835_dma_cb_dump(
	struct bcm2835_dma_cb *dmablock,
	dma_addr_t dmablock_dma,
	struct device *dev,
	int tindent)
{
	const char* prefix=_tab_indent(tindent);
	struct bcm2835_dma_cb_stride *stride =
		(struct bcm2835_dma_cb_stride *)&dmablock->stride;
	dev_printk(KERN_INFO,dev,"%saddr:\t%pK\n",prefix,
		dmablock);
	if (dmablock_dma)
		dev_printk(KERN_INFO,dev,"%sd_addr:\t%08llx\n",prefix,
			(unsigned long long)dmablock_dma);
	dev_printk(KERN_INFO,dev,"%sti:\t%08x\n",prefix,
		dmablock->ti);
	if (dmablock->src ==
		dmablock_dma + offsetof(struct bcm2835_dma_cb,pad[0]))
		dev_printk(KERN_INFO,dev,"%ssrc:\t%08x - pad0\n",prefix,
			dmablock->src);
	else if (dmablock->src ==
		dmablock_dma + offsetof(struct bcm2835_dma_cb,pad[1]))
		dev_printk(KERN_INFO,dev,"%ssrc:\t%08x - pad1\n",prefix,
			dmablock->src);
	else
		dev_printk(KERN_INFO,dev, "%ssrc:\t%08x\n", prefix,
			dmablock->src);
	if (dmablock->dst ==
		dmablock_dma + offsetof(struct bcm2835_dma_cb,pad[0]))
		dev_printk(KERN_INFO,dev,"%sdst:\t%08x - pad0\n",prefix,
			dmablock->dst);
	else if (dmablock->dst ==
		dmablock_dma + offsetof(struct bcm2835_dma_cb,pad[1]))
		dev_printk(KERN_INFO,dev,"%sdst:\t%08x - pad1\n",prefix,
			dmablock->dst);
	else
		dev_printk(KERN_INFO,dev, "%sdst:\t%08x\n", prefix,
			dmablock->dst);
	dev_printk(KERN_INFO,dev,"%slength:\t%u\n",prefix,
		dmablock->length);
	if (dmablock->stride)
		dev_printk(KERN_INFO,dev,
			"%sstride:\tsrc+=%i dst+=%i\n",prefix,
			stride->src,stride->dst);
	dev_printk(KERN_INFO,dev,"%snext:\t%08x\n",prefix,
		dmablock->next);
	dev_printk(KERN_INFO,dev,"%spad0:\t%08x\n",prefix,
		dmablock->pad[0]);
	dev_printk(KERN_INFO,dev,"%spad1:\t%08x\n",prefix,
		dmablock->pad[1]);
}
EXPORT_SYMBOL_GPL(bcm2835_dma_cb_dump);

int bcm2835_dma_reg_dump_str(
	void *base,int tindent,
	char *buffer, size_t size)
{
	const char *prefix=_tab_indent(tindent);
	size_t len=0;
        len+=snprintf(buffer+len,size-len, "%s.addr   = %pf\n",
		prefix, base);
        len+=snprintf(buffer+len,size-len, "%s.cs     = %08x\n",
		prefix, readl(base+BCM2835_DMA_CS));
        len+=snprintf(buffer+len,size-len,
		"%s.cbaddr = %08x\n",
		prefix, readl(base+BCM2835_DMA_ADDR));
        len+=snprintf(buffer+len,size-len, "%s.ti     = %08x\n",
		prefix, readl(base+BCM2835_DMA_TI));
        len+=snprintf(buffer+len,size-len, "%s.src    = %08x\n",
		prefix, readl(base+BCM2835_DMA_S_ADDR));
        len+=snprintf(buffer+len,size-len, "%s.dst    = %08x\n",
		prefix, readl(base+BCM2835_DMA_D_ADDR));
        len+=snprintf(buffer+len,size-len, "%s.length = %08x\n",
		prefix, readl(base+BCM2835_DMA_LEN));
        len+=snprintf(buffer+len,size-len, "%s.stride = %08x\n",
		prefix, readl(base+BCM2835_DMA_STRIDE));
        len+=snprintf(buffer+len,size-len, "%s.next   = %08x\n",
		prefix, readl(base+BCM2835_DMA_NEXT));
        len+=snprintf(buffer+len,size-len, "%s.debug  = %08x\n",
		prefix, readl(base+BCM2835_DMA_DEBUG));
	return len;
}
EXPORT_SYMBOL_GPL(bcm2835_dma_reg_dump_str);

void bcm2835_dma_reg_dump(
	void *base,
	struct device *dev,
	int tindent)
{
	char buffer[768];
	char *start=buffer;
	char *line;
	bcm2835_dma_reg_dump_str(
		base,tindent,
		buffer,sizeof(buffer)
		);
	while((start) && (line=strsep(&start,"\n"))) {
		dev_printk(KERN_INFO, dev,line);
	}
}
EXPORT_SYMBOL_GPL(bcm2835_dma_reg_dump);

MODULE_DESCRIPTION("bcm2835 dma tools");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL v2");
