/*
 * DMA control block definition - may be coming from dmaengine
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
 * 4567890123456789012345678901234567890123456789012345678901234567890123456789
 */

#ifndef __BCM2835_DMA_H
#define __BCM2835_DMA_H

#include <linux/types.h>
#include <linux/device.h>

/* the DMA registers and their bitflags */
#define BCM2835_DMA_CS                                  0x00
#define BCM2835_DMA_CS_ACTIVE				(1 << 0)
#define BCM2835_DMA_CS_END				(1 << 1)
#define BCM2835_DMA_CS_INT				(1 << 2)
#define BCM2835_DMA_CS_DREQ				(1 << 3)
#define BCM2835_DMA_CS_ISPAUSED				(1 << 4)
#define BCM2835_DMA_CS_ISHELD				(1 << 5)
#define BCM2835_DMA_CS_WAITING_FOR_OUTSTANDING_WRITES	(1 << 6)
/* bit 7: reserved */
#define BCM2835_DMA_CS_ERR				(1 << 8)
/* bit 9-15: reserved */
#define BCM2835_DMA_CS_PRIORITY(x)			(((x)&0x0f) <<16)
#define BCM2835_DMA_CS_PANICPRIORITY(x)			(((x)&0x0f) <<20)
/* bit 24-27: reserved */
#define BCM2835_DMA_CS_WAIT_FOR_OUTSTANDING_WRITES	(1 <<28)
#define BCM2835_DMA_CS_DISDEBUG				(1 <<29)
#define BCM2835_DMA_CS_ABORT				(1 <<30)
#define BCM2835_DMA_CS_RESET				(1 <<31)

#define BCM2835_DMA_ADDR                                0x04
#define BCM2835_DMA_TI                                  0x08
#define BCM2835_DMA_TI_INT_EN				(1 << 0)
#define BCM2835_DMA_TI_TDMODE				(1 << 1)
#define BCM2835_DMA_TI_RESERVED				(1 << 2)
#define BCM2835_DMA_TI_WAIT_RESP		       	(1 << 3)
#define BCM2835_DMA_TI_D_INC				(1 << 4)
#define BCM2835_DMA_TI_D_WIDTH				(1 << 5)
#define BCM2835_DMA_TI_D_DREQ				(1 << 6)
#define BCM2835_DMA_TI_D_IGNORE				(1 << 7)
#define BCM2835_DMA_TI_S_INC				(1 << 8)
#define BCM2835_DMA_TI_S_WIDTH				(1 << 9)
#define BCM2835_DMA_TI_S_DREQ				(1 <<10)
#define BCM2835_DMA_TI_S_IGNORE				(1 <<11)
#define BCM2835_DMA_TI_BURST(x)				(((x)&0x0f) <<12)
#define BCM2835_DMA_TI_PER_MAP(x)		       	(((x)&0x1f) <<16)
#define BCM2835_DMA_TI_WAITS(x)				(((x)&0x1f) <<21)
#define BCM2835_DMA_TI_NO_WIDE_BURSTS			(1 <<26)
/* bit 27-31: reserved */

#define BCM2835_DMA_S_ADDR                              0x0C
#define BCM2835_DMA_D_ADDR                              0x10
#define BCM2835_DMA_LEN                                 0x14
#define BCM2835_DMA_STRIDE                              0x18
#define BCM2835_DMA_NEXT                                0x1C
#define BCM2835_DMA_DEBUG                               0x20

/**
 * struct bcm2835_dma_cb the DMA control block
 * @ti: configuration register
 * @src: source-bus address for transfer
 * @dst: destination-bus address for transfer
 * @length: the length of the transfer
 * @stride_src: striding for src in 2D DMA-mode
 * @stride_dst: striding for dst in 2D DMA-mode
 * @next: the link to the next DMA control block to get exectued
 *        (0 for none)
 * @pad: padding - can get used for some data without having to
 *       allocate extra dma memory
 */
struct bcm2835_dma_cb {
	u32        ti;
	dma_addr_t src;
	dma_addr_t dst;
	u32        length;
	u32        stride;
	dma_addr_t next;
	u32        pad[2];
};

struct bcm2835_dma_cb_stride {
	s16 src;
	s16 dst;
};

static inline u32 bcm2835_dma_cb_compose_stride(s16 src_stride, s16 dst_stride) {
	struct bcm2835_dma_cb_stride tmp;
	tmp.src=src_stride;
	tmp.dst=dst_stride;
	return *((u32*)&tmp);
}

void bcm2835_dma_cb_dump(
	char *prefix,
	struct device *dev,
	struct bcm2835_dma_cb *dmablock,
	dma_addr_t dmablock_dma,
	int flags);

#endif /* __BCM2835_DMA_H */
