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
#define SPI_OPTIMIZE_VARY_LENGTH               (1<<4)

/**
 * spi_dma_fragment_functions: structure with functions to call
 *   to fill the dma_composite structure with components
 * @fragment_composite_cache: the cache fo dma_fragment_composite_spi objects
 *   that can get used/reused
 * Notes:
 * * right now we assume that the master_device_data contains this structure
 *   right at the first address - that is until we get this structure into
 *   spi_master...
 */
struct spi_dma_fragment_functions {
	struct dma_fragment_cache* fragment_composite_cache;
};

struct spi_merged_dma_fragments {
	struct dma_fragment fragment;

	struct spi_message *message;
	struct spi_transfer *transfer;
};

static inline struct dma_fragment *spi_merged_dma_fragments_alloc(
	struct device *device,gfp_t gfpflags)
{
	return dma_fragment_alloc(device,
				sizeof(struct spi_merged_dma_fragments),
				gfpflags);
}


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
