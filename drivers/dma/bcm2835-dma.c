/*
 * Driver for DMA Fragments - initially used for BCM2835 DMA implementation
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
 */
#include <linux/dma/bcm2835-dma.h>
#include <linux/printk.h>
#include <asm/io.h>

static void bcm2835_dma_dump_cb(struct bcm2835_dma_cb* cb)
{
        printk(KERN_DEBUG "        .info       = %08x\n",readl(&cb->ti));
        printk(KERN_DEBUG "        .src        = %08x\n",readl(&cb->src));
        printk(KERN_DEBUG "        .dst        = %08x\n",readl(&cb->dst));
        printk(KERN_DEBUG "        .length     = %08x\n",readl(&cb->length));
        printk(KERN_DEBUG "        .stride_src = %08x\n",readl(&cb->stride_src));
        printk(KERN_DEBUG "        .stride_dst = %08x\n",readl(&cb->stride_dst));
        printk(KERN_DEBUG "        .next       = %08x\n",readl(&cb->next));
}
