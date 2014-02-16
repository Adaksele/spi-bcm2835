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

#include "spi-bcm2835dma.h"
extern bool debug_msg;
extern bool debug_dma;
extern int delay_1us;

/**
 * bcm2835dma_cb_dump - dumping wrapper arround the generic CB controll block
 * @prefix: the prefix on each line
 * @link: the dma_link to dump
 * @flags: some flags
 */
void bcm2835dma_cb_dump(
	char *prefix,
	struct dma_link *link,
	int flags) {
	bcm2835_dma_cb_dump(
		prefix,
		link->device,
		link->dmablock,
		link->dmablock_dma,
		flags);
}

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
	void             *src;
	void             *dst;
	void             *extra;
	int              (*transformation)(void* src, void* dst, void* extra);
};

/**
 * message_transform_copy_word - copy one word of data from src to dest
 * @src: source address to copy from
 * @dst: destination address to copy to
 * @extra: extra data needed for transform
 * Note that we might want to inline this for efficiency
 */
int message_transform_copy_word(void* src, void* dst, void* extra);

/**
 * message_transform_copy_checkmax - copy src to dst, but checks that 
 *   the value does not exceed (u32) extra
 * @src: source address to copy from
 * @dst: destination address to copy to
 * @extra: extra data needed for transform
 * Note that we might want to inline this for efficiency
 */
int message_transform_copy_checkmax(void* src, void* dst, void* extra);

/**
 * message_transform_speed_to_clockdivider - manipulates the source so
 *   that we get the necessary clock divider for configuring SPI
 * @src: source address to copy from
 * @dst: destination address to copy to
 * @extra: extra data needed for transform
 */
int message_transform_speed_to_clockdivider(void* src, void* dst, void* extra);

/**
 * message_transform_usec_to_delay - manipulates the source so
 *   that we get the necessary clock divider for configuring SPI
 * @src: source address to copy from
 * @dst: destination address to copy to
 * @extra: extra data needed for transform
 */
int message_transform_usec_to_delay(void* src, void* dst, void* extra);

/**
 * message_transform_speed_to_half_clock_cycle_delay - manipulates the source so
 *   that we get the necessary delay value to create delay of .5 clock cycles
 * @src: source address to copy from
 * @dst: destination address to copy to
 * @extra: extra data needed for transform
 */
int message_transform_speed_to_half_clock_cycle_delay(
	void* src, void* dst, void* extra);

/**
 * message_transform_dmamap_region - dma maps source and copies the dma_addr
 *   to dest
 * @src: source address to copy from
 * @dst: destination address to copy to
 * @extra: extra data needed for transform
 */
int message_transform_speed_dmamap_region(void* src, void* dst, void* extra);

/**
 * message_transform_dmaunmap_region - dma unmaps source
 * @src: source address to copy from
 * @dst: destination address to copy to
 * @extra: extra data needed for transform
 * typically used on the post-transform chain
 */
int message_transform_speed_dmaunmap_region(void* src, void* dst, void* extra);

/**
 * dma_fragment_transfer - structure used to initiate an additional
 *   transfer
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments later
 */
struct dma_fragment_transfer {
	struct dma_fragment fragment;
	/* the individual objects */
	struct dma_link     *schedule_transfer_rx;
	struct dma_link     *schedule_transfer_tx;
};

/**
 * dma_fragment_setup_transfer - structure used to set up SPI and initiate
 *   a transfer
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments later
 * note that we extend the simple dma_fragment_transfer with the same pattern 
 *   as above, so that we can simplify the linking pattern
 */
struct dma_fragment_setup_transfer {
	struct dma_fragment fragment;
	/* the individual objects */
	struct dma_link     *schedule_transfer_rx;
	struct dma_link     *schedule_transfer_tx;
	struct dma_link     *cs_select;
	struct dma_link     *reset_spi_and_config_speed;
	struct dma_link     *config_length;
	struct dma_link     *config_spi;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
};

/**
 * dma_fragment_cs_deselect - structure used to deselect CS
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments with
 * a spi_message/spi_transfer
 */
struct dma_fragment_cs_deselect {
	struct dma_fragment fragment;
	struct dma_link     *delay_pre;
	struct dma_link     *cs_deselect;
	struct dma_link     *delay_post;
};

/**
 * dma_fragment_delay - structure used to delay the next DMAs by some usecs
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments with
 * a spi_message/spi_transfer
 */
struct dma_fragment_delay {
	struct dma_fragment fragment;
	struct dma_link     *delay;
};
/**
 * dma_fragment_trigger_irq - structure used to trigger an IRQ on DMA completion
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments with
 * a spi_message/spi_transfer
 */

struct dma_fragment_trigger_irq {
	struct dma_fragment fragment;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
	struct dma_link     *message_finished;
};

/**
 * ALLOCATE_RXDMA_IN_FRAGMENT - helper for basic allocation code for RXDMA
 * @frag: the dma_fragment to which this belongs
 * @field: the field which we want to get populated with the pointer
 */
#define ALLOCATE_RXDMA_IN_FRAGMENT(frag,field)			\
	if (! ( frag->field = dma_link_alloc(			\
				device,pool,			\
				bs->dma_rx.chan,		\
				gfpflags			\
				)))				\
		goto error;					\
	if ( dma_fragment_add(					\
			(struct dma_fragment *)frag,		\
			(struct dma_link *)frag->field		\
			) )					\
		goto error;

/**
 * ALLOCATE_TXDMA_IN_FRAGMENT - helper for basic allocation code for TXDMA
 * @frag: the dma_fragment to which this belongs
 * @field: the field which we want to get populated with the pointer
 */
#define ALLOCATE_TXDMA_IN_FRAGMENT(frag,field)			\
	if (! ( frag->field = dma_link_alloc(			\
				device,pool,			\
				bs->dma_tx.chan,		\
				gfpflags			\
				)))				\
		goto error;					\
	if ( dma_fragment_add(					\
			(struct dma_fragment *)frag,		\
			(struct dma_link *)frag->field		\
			) )					\
		goto error;

/**
 * FREE_RMA_IN_FRAGMENT - helper for freeing the dma_link in the fragment
 * @frag: the dma_fragment to which this belongs
 * @field: the field which we want to get populated with the pointer
 */
#define FREE_DMA_IN_FRAGMENT(frag,field)   \
	if (frag->field)		     \
		dma_link_free(frag->field);

/**
 * LINK_TO_BCM2835_DMA_CB - helper to get the casted pointer of the DMA CB
 *   from the dmalink given
 * @dmalink: dma_link to cast
 */
#define LINK_TO_BCM2835_DMA_CB(dmalink)			\
	((struct bcm2835_dma_cb *)dmalink->dmablock)

/**
 * LINK_FIELD_DMAADDR - for a field in the DMA CB create the corresponding 
 *   dma-address
 * @dmalink: dma_link to base this on
 * @field: the member field in the dma CB
 */
#define LINK_FIELD_DMAADDR(dmalink,field)				\
	( dmalink->dmablock_dma + offsetof(struct bcm2835_dma_cb,field) )

/**
 * bcm2835_spi_dmafragment_create_composite - create a composite DMA fragment
 *   which will contain several other DMA fragments to create a complete
 *   transfer of an SPI message
 *   this is used for both prepared and unprepared messages
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */
struct dma_fragment *bcm2835_spi_dmafragment_create_composite(
	struct device * device,gfp_t gfpflags)
{
	struct dma_fragment_composite *frag 
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));
	return &frag->fragment;
}

/**
 * bcm2835_spi_dmafragment_create_setup_transfer- create a DMA fragment
 *   which sets up SPI to work propperly with Speed, CS and also adds
 *   the initial transfer afterwards
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */
struct dma_fragment *bcm2835_spi_dmafragment_create_setup_transfer(
	struct device * device,gfp_t gfpflags)
{
	struct spi_master * master = (struct spi_master *)device;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dma_fragment_setup_transfer *frag 
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */
	/* start with the CSSelect */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,cs_select);
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->ti =
		BCM2835_DMA_TI_WAIT_RESP;
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->length = 
		4;
	/* now reset SPI and configure the speed */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,reset_spi_and_config_speed);
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->next =
		frag->reset_spi_and_config_speed->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_TDMODE
		| BCM2835_DMA_TI_S_INC
		| BCM2835_DMA_TI_D_INC;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->length =
		8;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->stride_src = 
	        4;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->stride_dst =
		-8;
	/* the clock speed as a divider in pad[0]
	 * the RESET values in pad[1] - need to set:
	 * * BCM2835_SPI_CS_CSPOL correctly
	 * * BCM2835_SPI_CS_CLEAR_RX
	 * * BCM2835_SPI_CS_CLEAR_TX
	 */

	/* set the DMA-transfer length */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,config_length);
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->next =
		frag->config_length->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_DLEN;
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->length =
		4;
	/* the clock speed as a divider in pad[0] */

	/* reenable the spi config */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,config_spi);
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->next =
		frag->config_spi->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_DLEN;
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->length =
		4;
	/* the correct flags for TI in pad[0] - need to have:
	 * BCM2835_SPI_CS_CPHA correctly
	 * BCM2835_SPI_CS_CPOL correctly
	 * BCM2835_SPI_CS_DMAEN
	 * BCM2835_SPI_CS_TA
	 * BCM2835_SPI_CS_CS_01
	 * BCM2835_SPI_CS_CS_10
	 */

	/* set the tx-DMA next pointer */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,set_tx_dma_next);
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->next =
		frag->set_tx_dma_next->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->src =
		LINK_FIELD_DMAADDR(frag->set_tx_dma_next,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->dst =
		bs->dma_tx.bus_addr+BCM2835_DMA_ADDR;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->length =
		4;
	
	/* start tx-DMA */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,start_tx_dma);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->next =
		frag->start_tx_dma->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->src =
		LINK_FIELD_DMAADDR(frag->start_tx_dma,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->dst =
		bs->dma_tx.bus_addr+BCM2835_DMA_ADDR;

	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->length =
		4;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->pad[0] =
		BCM2835_DMA_CS_ACTIVE;

	/* the initial transfer for RX */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,schedule_transfer_rx);
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->next =
		frag->schedule_transfer_rx->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->src =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */
	/* and the initial transfer for TX */
	ALLOCATE_TXDMA_IN_FRAGMENT(frag,schedule_transfer_tx);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->pad[0] =
		frag->schedule_transfer_tx->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */

	dma_fragment_dump(&frag->fragment,&bcm2835dma_cb_dump,0);

	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,cs_select);
	FREE_DMA_IN_FRAGMENT(frag,reset_spi_and_config_speed);
	FREE_DMA_IN_FRAGMENT(frag,config_length);
	FREE_DMA_IN_FRAGMENT(frag,config_spi);
	FREE_DMA_IN_FRAGMENT(frag,set_tx_dma_next);
	FREE_DMA_IN_FRAGMENT(frag,start_tx_dma);
	FREE_DMA_IN_FRAGMENT(frag,schedule_transfer_rx);
	FREE_DMA_IN_FRAGMENT(frag,schedule_transfer_tx);

	dma_fragment_free(&frag->fragment);

	return NULL;
}
/**
 * bcm2835_spi_dmafragment_create_transfer- create a DMA fragment
 *   which adds the initial transfer afterwards
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */
	
struct dma_fragment *bcm2835_spi_dmafragment_create_transfer(
	struct device *device,gfp_t gfpflags)
{
	struct spi_master * master = (struct spi_master *)device;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dma_fragment_transfer *frag 
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */

	/* the initial transfer for RX */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,schedule_transfer_rx);
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->src =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */
	/* and the initial transfer for TX */
	ALLOCATE_TXDMA_IN_FRAGMENT(frag,schedule_transfer_tx);
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */

	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,schedule_transfer_rx);
	FREE_DMA_IN_FRAGMENT(frag,schedule_transfer_tx);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

/**
 * bcm2835_spi_dmafragment_create_cs_deselect - create a DMA fragment
 *   which deselects the CS
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */
struct dma_fragment *bcm2835_spi_dmafragment_create_cs_deselect(
	struct device *device,gfp_t gfpflags)
{
	struct spi_master * master = (struct spi_master *)device;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dma_fragment_cs_deselect *frag 
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */

	/* the pre-half-clock delay */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,delay_pre);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_WAITS(0x1f)
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| BCM2835_DMA_TI_S_IGNORE
		| BCM2835_DMA_TI_D_IGNORE;
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->src =
		LINK_FIELD_DMAADDR(frag->delay_pre,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->dst =
		LINK_FIELD_DMAADDR(frag->delay_pre,pad[1]);
	/* need to set:
	 * * length to correct length to get to the delay we need based on the clock
	 */
	/* start with the CSSelect */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,cs_deselect);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->next =
		frag->cs_deselect->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->ti =
		BCM2835_DMA_TI_WAIT_RESP;
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->src =
		LINK_FIELD_DMAADDR(frag->cs_deselect,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->length = 
		4;
	/* need to set:
	 * * pad0 to correct bitmask
	 * * dst to correct register to pull GPIO pin up
	 */

	/* the post-half-clock delay */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,delay_post);
	LINK_TO_BCM2835_DMA_CB(frag->delay_post)->next =
		frag->cs_deselect->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->delay_post)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_WAITS(0x1f)
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| BCM2835_DMA_TI_S_IGNORE
		| BCM2835_DMA_TI_D_IGNORE;
	LINK_TO_BCM2835_DMA_CB(frag->delay_post)->src =
		LINK_FIELD_DMAADDR(frag->delay_post,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->dst =
		LINK_FIELD_DMAADDR(frag->delay_post,pad[1]);
	/* need to set:
	 * * length to correct length to get to the delay we need based on the clock
	 */

	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,delay_pre);
	FREE_DMA_IN_FRAGMENT(frag,cs_deselect);
	FREE_DMA_IN_FRAGMENT(frag,delay_post);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

/**
 * bcm2835_spi_dmafragment_create_cs_deselect - create a DMA fragment
 *   which delays the processing by a requested number of usec
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */

struct dma_fragment *bcm2835_spi_dmafragment_create_delay(
	struct device *device,gfp_t gfpflags)
{
	struct spi_master * master = (struct spi_master *)device;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dma_fragment_delay *frag 
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */

	/* the pre-half-clock delay */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,delay);
	LINK_TO_BCM2835_DMA_CB(frag->delay)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_WAITS(0x1f)
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| BCM2835_DMA_TI_S_IGNORE
		| BCM2835_DMA_TI_D_IGNORE;
	LINK_TO_BCM2835_DMA_CB(frag->delay)->src =
		LINK_FIELD_DMAADDR(frag->delay,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->delay)->dst =
		LINK_FIELD_DMAADDR(frag->delay,pad[1]);
	/* need to set:
	 * * length to correct length to get to the delay we need based on the clock
	 */

	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,delay);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

/**
 * bcm2835_spi_dmafragment_create_trigger_irq - create a DMA fragment
 *   which triggers an IRQ on end of transfer
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */
struct dma_fragment *bcm2835_spi_dmafragment_create_trigger_irq(
	struct device *device,gfp_t gfpflags)
{
	struct spi_master * master = (struct spi_master *)device;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dma_fragment_trigger_irq *frag 
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */
	/* set the tx-DMA next pointer */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,set_tx_dma_next);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->src =
		LINK_FIELD_DMAADDR(frag->set_tx_dma_next,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->dst =
		bs->dma_tx.bus_addr+BCM2835_DMA_ADDR;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->length =
		4;
	
	/* start tx-DMA */
	ALLOCATE_RXDMA_IN_FRAGMENT(frag,start_tx_dma);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->next =
		frag->start_tx_dma->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->src =
		LINK_FIELD_DMAADDR(frag->start_tx_dma,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->dst =
		bs->dma_tx.bus_addr+BCM2835_DMA_CS;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->length =
		4;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->pad[0] =
		BCM2835_DMA_CS_ACTIVE;

	/* message_finished + interrupt */
	ALLOCATE_TXDMA_IN_FRAGMENT(frag,message_finished);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->pad[0] =
		frag->message_finished->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->message_finished)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_S_INC
		| BCM2835_DMA_TI_D_INC
		;
	LINK_TO_BCM2835_DMA_CB(frag->message_finished)->src =
		LINK_FIELD_DMAADDR(frag->message_finished,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->message_finished)->length =
		8;
	/* need to set:
	 * * dst = The address which gets the latest tranfer
	 * * pad[0] = the fragment that was finished
	 * * pad[1] = possibly spi_message pointer
	 */ 
	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,set_tx_dma_next);
	FREE_DMA_IN_FRAGMENT(frag,start_tx_dma);
	FREE_DMA_IN_FRAGMENT(frag,message_finished);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

int bcm2835dma_spi_compo_add_config_transfer(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	return -1;
}

int bcm2835dma_spi_compo_add_transfer(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	return -1;
}

int bcm2835dma_spi_compo_add_cs_change(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	return -1;
}

int bcm2835dma_spi_compo_add_delay(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	return -1;
}

int bcm2835dma_spi_compo_add_trigger_irq(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	return -1;
}

/**
 * spi_message_to_dmafragment - converts a spi_message to a dma_fragment
 * @msg:  the spi message to convert
 * @gfpflags: flags for allocation
 * notes:
 * * this is essentially generic and could go into generic spi
 * * we could also create an automatically prepared version 
 *     via a spi_message flag (e.g prepare on first use)
 */
struct dma_fragment *spi_message_to_fragment(
	struct spi_message *msg, gfp_t gfpflags)
{
	struct spi_device *spi = msg->spi;
	struct spi_master *master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	
	struct dma_fragment_composite *compo;
	struct spi_transfer *xfer,*last_xfer;
	int needs_reset;

	/* fetch a composite fragment */
	compo = (struct dma_fragment_composite *)
		dma_fragment_cache_fetch(&bs->fragment_composite,gfpflags);
	if (! compo)
		return NULL;

	/* now start iterating the transfers */
	last_xfer=NULL;
	needs_reset=1;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* check if we are the last in the list */
		int is_last=list_is_last(&msg->transfers,&xfer->transfer_list);
		/* do we need to reconfigure compared to the last transfer */
		if (!needs_reset) {
			if (last_xfer->speed_hz != xfer->speed_hz)
				needs_reset=1;
			else if (last_xfer->tx_nbits != xfer->tx_nbits)
				needs_reset=1;
			else if (last_xfer->rx_nbits != xfer->rx_nbits)
				needs_reset=1;
			else if (last_xfer->bits_per_word!=xfer->bits_per_word)
				needs_reset=1;
		}
		/* now decide which transfer to use, 
		   the normal or the reset version */
		if (needs_reset) {
			needs_reset=bcm2835dma_spi_compo_add_config_transfer(
				spi,xfer,compo,gfpflags);
		} else {
			needs_reset=bcm2835dma_spi_compo_add_transfer(
				spi,xfer,compo,gfpflags);
		}
		/* error handling */
		if (needs_reset<0)
			goto error;
		/* add cs_change with optional extra delay 
		   if requested or last in sequence */
		if ((xfer->cs_change)||(is_last))
			needs_reset=bcm2835dma_spi_compo_add_cs_change(
				spi,
				xfer,
				compo,
				gfpflags);
		else if (xfer->delay_usecs)
			/* or add a delay if requested */
			needs_reset=bcm2835dma_spi_compo_add_delay(
				spi,
				xfer,
				compo,
				gfpflags);
		/* handle errors */
		if (needs_reset<0)
			goto error;
		/* and set the last_transfer */
		last_xfer=xfer;
	}
	/* and add an interrupt if we got a callback to handle
	 * if there is no callback, then we do not need to release it
	 * immediately - even for prepared messages
	 */
	if (msg->complete)
		if (bcm2835dma_spi_compo_add_trigger_irq(
			spi,
			NULL,
			compo,
			gfpflags))
			goto error;
	
	/* and return it */
	return &compo->fragment;

error:
	return NULL;
	
}
