/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2014 Martin Sperl
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
 *
 * 4567890123456789012345678901234567890123456789012345678901234567890123456789
 */

#ifndef __SPI_DMAFRAGMENT_H
#define __SPI_DMAFRAGMENT_H
#include <linux/dma-fragment.h>
#include <linux/spi/spi.h>


#define SPI_OPTIMIZE_VARY_TX                   (1<<0)
#define SPI_OPTIMIZE_VARY_RX                   (1<<1)
#define SPI_OPTIMIZE_VARY_FRQUENCY             (1<<2)
#define SPI_OPTIMIZE_VARY_DELAY                (1<<3)
#define SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_1    (1<<4)
#define SPI_OPTIMIZE_VARY_LENGTH            \
	SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_1
#define SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_4    (1<<5) \
	| SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_1
#define SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_8    (1<<6) \
	| SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_4
#define SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_16   (1<<7) \
	| SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_8
#define SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_32   (1<<8) \
	|SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_16

/**
 * spi_dma_fragment_functions: structure with functions to call
 *   to fill the dma_composite structure with components
 * @fragment_composite_cache: the cache fo dma_fragment_composite_spi objects
 *   that can get used/reused
 * @add_setup_spi_transfer: the function to setup SPI correctly
 *   (including CS,...) and initiate the first transfer this also gets
 *   called whenever something important changes (like bits, speed,...)
 * @add_transfer: the function to schedule a single SPI transfer
 *   (transmit/receive)
 * @add_cs_deselect: the function to deselect CS and an (optinal) delay prior
     to the CS - may trigger a subsequent setup_spi for the next transfer.
 * @add_delay: function that adds a delay of delay_usec
 * @add_trigger_interrupt: function that adds triggering an interrupt
 * Notes:
 * * right now we assume that the master_device_data contains this structure
 *   right at the first address - that is until we get this structure into
 *   spi_master...
 */
struct spi_dma_fragment_functions {
	struct dma_fragment_cache* fragment_composite_cache;
	int (*add_transfer)(struct spi_message *,
			struct spi_transfer *,
			struct dma_fragment *,
			u32,
			gfp_t);
	int (*add_setup_spi_transfer)(struct spi_message *,
				struct spi_transfer *,
				struct dma_fragment *,
				u32,
				gfp_t);
	int (*add_cs_deselect)(struct spi_message *,
			struct spi_transfer *,
			struct dma_fragment *,
			u32,
			gfp_t);
	int (*add_delay)(struct spi_message *,
			struct spi_transfer *,
			struct dma_fragment *,
			u32,
			gfp_t);
	int (*add_trigger_interrupt)(struct spi_message *,
				struct spi_transfer *,
				struct dma_fragment *,
				u32,
				gfp_t);
};

/**
 * spi_message_to_dma_fragment - converts a spi_message to a dma_fragment
 * @msg:  the spi message to convert
 * @flags: some flags
 * @gfpflags: flags for allocation
 * notes:
 * * this is essentially generic and could go into generic spi
 * * we could also create an automatically prepared version
 *     via a spi_message flag (e.g prepare on first use)
 */
struct dma_fragment *spi_message_to_dma_fragment(
	struct spi_message *msg,
	int flags,
	gfp_t gfpflags);

#endif /* __SPI_DMAFRAGMENT_H */
