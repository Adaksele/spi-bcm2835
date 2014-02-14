/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2013 Martin Sperl
 *
 * This driver is inspired by:
 * spi-ath79.c, Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c, Copyright (C) 2006 Atmel Corporation
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
#include <linux/spi/bcm2835.h>
#include <linux/dma-fragment.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <mach/dma.h>

struct bcm2835_dmachannel {
	void __iomem *base;
	dma_addr_t bus_addr;
        int chan;
        int irq;
	irq_handler_t handler;
	const char *desc;
};

struct bcm2835dma_spi {
	/* the SPI registers */
	void __iomem *spi_regs;
	/* the clock */
	struct clk *clk;
	/* the DMA channels allocated */
	struct bcm2835_dmachannel dma_tx;
	struct bcm2835_dmachannel dma_rx;
	/* the DMA-able pool we use to allocate control blocks from */
	struct dma_pool *pool;
	/* some DMA able blocks for some read/write buffers */
	struct {
		void *addr;
		dma_addr_t bus_addr;
	} buffer_write_dummy,buffer_read_0x00;
	/* the fragment caches */
	struct dma_fragment_cache fragment_setup_spi_plus_transfer;
	struct dma_fragment_cache fragment_transfer;
	struct dma_fragment_cache fragment_cs_deselect;
	struct dma_fragment_cache fragment_delay;
	struct dma_fragment_cache fragment_trigger_irq;
};

struct dma_fragment *bcm2835_dmafragment_create_setup_spi_plus_transfer(
	struct spi_master *,gfp_t);
struct dma_fragment *bcm2835_dmafragment_create_transfer(
	struct spi_master *,gfp_t);
struct dma_fragment *bcm2835_dmafragment_create_cs_deselect(
	struct spi_master *,gfp_t);
struct dma_fragment *bcm2835_dmafragment_create_delay(
	struct spi_master *,gfp_t);
struct dma_fragment *bcm2835_dmafragment_create_trigger_irq(
	struct spi_master *,gfp_t);

/* the interrupt-handlers */
static irqreturn_t bcm2835dma_spi_interrupt_dma_tx(int irq, void *dev_id);
//static irqreturn_t bcm2835dma_spi_interrupt_dma_rx(int irq, void *dev_id);
