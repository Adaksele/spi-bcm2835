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
 * spi_message_transform - copies/transforms data from spi_message and
 *   spi_transfers into the DMA structure
 * @message_transform_chain: the list structure
 * @src: the source address from which to fetch the data
 * @dst: the destination address to which to copy the data
 * @extra: some extra data
 * @transformation: the transformation method
 */
struct spi_message_transform {
	struct list_head message_transform_chain;
	int              (*transformation)(void* src, void* dst, void* extra);
	void             *src;
	void             *dst;
	void             *extra;
};

/**
 * spi_message_transform_add - add a transformation to the list of 
 *   message transforms
 * @head: message transform list to which to add
 * @transformation: the transformation function to call
 * @src: 1st argument to transformation function
 * @dst: 2nd argument to transformation function
 * @extra: 3rd argument to transformation function
 * @gfpflags: gfpflags to use during allocation
 */
int spi_message_transform_add(
	struct list_head *head,
	int              (*transformation)(void* src, void* dst, void* extra),
	void             *src,
	void             *dst,
	void             *extra,
	gfp_t            gfpflags
	);

/**
 * spi_message_transform_release_all - release all message transformations
 *   from this list freeing the memory
 * @head: message transform list from which to remove all entries
 */
void spi_message_transform_release_all(struct list_head *head);

/**
 * spi_dma_fragment_composite - a composite structure with some extra data
 * @composite: the main composite structure
 * @last_setup_transfer: the pointer to the last setup_transfer structure
 * @last_transfer: the pointer to the last transfer structure 
 *   (may be identical to setup_transfer)
 */
struct spi_dma_fragment_composite {
	/* the main composit structure */
	struct dma_fragment_composite composite;
	/* additional data */
	void *last_setup_transfer;
	void *last_transfer;
	struct spi_transfer *last_xfer;
};

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
			struct spi_dma_fragment_composite *,
			u32,
			gfp_t);
	int (*add_setup_spi_transfer)(struct spi_message *,
				struct spi_transfer *,
				struct spi_dma_fragment_composite *,
				u32,
				gfp_t);
	int (*add_cs_deselect)(struct spi_message *,
			struct spi_transfer *,
			struct spi_dma_fragment_composite *,
			u32,
			gfp_t);
	int (*add_delay)(struct spi_message *,
			struct spi_transfer *,
			struct spi_dma_fragment_composite *,
			u32, 
			gfp_t);
	int (*add_trigger_interrupt)(struct spi_message *,
				struct spi_transfer *,
				struct spi_dma_fragment_composite *,
				u32,
				gfp_t);
};

/**
 * spi_dmafragment_create_composite - create a composite DMA fragment
 *   which will contain several other DMA fragments to create a complete
 *   transfer of an SPI message
 *   this is used for both prepared and unprepared messages
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */
struct dma_fragment *spi_dmafragment_create_composite(
	struct device * device,gfp_t gfpflags);


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
struct spi_dma_fragment_composite *spi_message_to_dma_fragment(
	struct spi_message *msg, 
	int flags, 
	gfp_t gfpflags);


#endif /* __SPI_DMAFRAGMENT_H */
