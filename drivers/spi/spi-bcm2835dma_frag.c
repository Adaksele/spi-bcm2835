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
	/* the individual objects - the first ones need to be identical 
	   to the ones of dma_fragment_setup_transfer */
	struct dma_link     *schedule_transfer_rx;
	struct dma_link     *schedule_transfer_tx;
	/* additional data */
	struct dma_link     *cs_select;
	struct dma_link     *reset_spi_and_config_speed;
	struct dma_link     *config_length;
	struct dma_link     *config_spi;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
	/* some of the timing data that we need 
	   - filled in based on the SPI-clock */
	u32 delay_transfers_half_cycle
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
 * dma_fragment_composite_spi - a composite structure with some extra data
 * @composite: the main composite structure
 * @last_setup_transfer: the pointer to the last setup_transfer structure
 * @last_transfer: the pointer to the last transfer structure 
 *   (may be identical to setup_transfer)
 */
struct dma_fragment_composite_spi {
	/* the main composit structure */
	struct dma_fragment_composite composite;
	/* additional data */
	struct dma_fragment_setup_transfer *last_setup_transfer;
	struct dma_fragment_transfer *last_transfer;
};

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
	struct dma_fragment_composite_spi *frag 
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));
	
	frag->last_setup_transfer = NULL;
	frag->last_transfer = NULL;

	return &(frag->composite.fragment);
}

/**
 * ADD_TO_DMA_FRAGMENT - add data to field in fragment and assign 
 *   the correct values
 * @frag: the fragment to which to add it
 * @field: the dma_link field which to set
 * @dmachannel: the dma channel to which this belongs
 * @do_<field>: do assign the following if set to 1
 *   - see also helper macros to document where these fields are 
 *     supposed to get set:
 *     * _IGNORE(field): do not set it
 *     * _FIXED(field): set this field fixed and never modified
 *     * _SPI(field): set during setup/optimization - value from spi_device
 *     * _MESG(field): set value based on spi_message/spi_transfer
 *     * _VARY(field): set value based on spi_message/spi_transfer for
 *                     optimized messages
 * @v_<Field>: the values to assign
 * Fields are:
 * @ti: dma configuration values
 * @src: the source dma_addr from which to copy data
 * @dst: the destination dma_addr to wich 
 * @length: the number of bytes to transfer
 * @stride_s: the source_stride length in 2d-mode
 * @stride_d: the destination_stride length in 2d-mode
 * @pad0: the pad[0] value
 * @pad1: the pad[1] value
 * Note that the macro assumes that the compiler will optimize dead code away
 */

#define ADD_TO_DMA_FRAGMENT(frag,field,dmachannel,			\
		do_ti, v_ti,						\
		do_src, v_src,						\
		do_dst, v_dst,						\
		do_length, v_length,					\
		do_stride, v_stride_s, v_stride_d,			\
		do_pad0, v_pad0,					\
		do_pad1, v_pad1						\
	)								\
	_ADD_TO_DMA_FRAGMENT(frag,field,dmachannel,			\
		do_ti, v_ti,						\
		do_src, v_src,						\
		do_dst, v_dst,						\
		do_length, v_length,					\
		do_stride, v_stride_s, v_stride_d,			\
		do_pad0, v_pad0,					\
		do_pad1, v_pad1						\
		)
/**
 * _ADD_TO_DMA_FRAGMENT: the internal version of ADD_TO_DMA_FRAGMENT,
 *   so that macro expansion is working propperly
 */
#define _ADD_TO_DMA_FRAGMENT(frag,field,dmachannel,			\
		do_ti, v_ti,						\
		do_src, v_src,						\
		do_dst, v_dst,						\
		do_length, v_length,					\
		do_stride, v_stride_s, v_stride_d,			\
		do_pad0, v_pad0,					\
		do_pad1, v_pad1						\
	)								\
	if (! ( frag->field = dma_link_alloc(				\
				device,pool,				\
				bs->dmachannel.chan,			\
				gfpflags				\
				)))					\
		goto error;						\
	if ( dma_fragment_add(						\
			(struct dma_fragment *)frag,			\
			(struct dma_link *)frag->field			\
			) )						\
		goto error;						\
	else {								\
		struct bcm2835_dma_cb *block =				\
			(struct bcm2835_dma_cb *)			\
			(frag->field->dmablock);			\
		if ( do_ti == 1 )					\
			block->ti = v_ti;				\
		if ( do_src == 1 )					\
			block->src = (u32) v_src;			\
		if ( do_dst == 1 )					\
			block->dst = (u32) v_dst;			\
		if ( do_length == 1)					\
			block->length = v_length;			\
		if ( do_stride == 1) {					\
			block->stride_src = v_stride_s;			\
			block->stride_dst = v_stride_d;			\
		} else {						\
			block->stride_src = 0;				\
			block->stride_dst = 0;				\
		}							\
		block->next=0;						\
		if ( do_pad0 == 1 )					\
			block->pad[0] = v_pad0;				\
		if ( do_pad1 ==1 )					\
			block->pad[1] = v_pad1;				\
	}

/**
 * _HELPER - helper macros to define what we are doing with witch field
 * these are defined mostly to give some insight into where the
 * data for each is coming from
 * @X: a dummy argument mostly there to clarify which data-field we 
 *   are talking about to make it more readable...
 */
#define _IGNORE(X) 0
#define _FIXED(X)  1
#define _SPI(X)    2
#define _MESG(X)   3
#define _VARY(X)   4

/**
 * DMA_LINK - links the second dma_link to get executed after the first
 * @frag: the fragment for which we do this
 * @first: the dma_link that is linked to the next
 * @second: the dma_link that is being linked to the first
 */
#define DMA_LINK(frag,first,second)					\
		((struct bcm2835_dma_cb *)(frag->first->dmablock))	\
		->next = frag->second->dmablock_dma;

/**
 * FIELD_DMA_ADDR - for a field in the DMA CB create the corresponding 
 *   dma-address
 * @dmalink: dma_link to base this on
 * @field: the member field in the dma CB
 */
#define FIELD_DMA_ADDR(dmalink,field)					\
	( dmalink->dmablock_dma + offsetof(struct bcm2835_dma_cb,field) )

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

	/* start with setting CS - some of the data
	   is taken data from spi_device_data */
	ADD_TO_DMA_FRAGMENT(frag,cs_select,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->cs_select,pad[0]),
			_SPI(DST),      0, /* spi_device_data
					      -> chipselect_select_gpio_reg */
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0, 0,
			_SPI(PAD0),     0, /* spi_device_data
					      ->chipselect_bitfield */
			_IGNORE(PAD1),  0
		);

	/* now reset SPI FIFOS and configure SPI_SPEED */
	ADD_TO_DMA_FRAGMENT(frag,reset_spi_and_config_speed,dma_rx,
			_FIXED(TI),     (BCM2835_DMA_TI_WAIT_RESP
					| BCM2835_DMA_TI_TDMODE
					| BCM2835_DMA_TI_S_INC
					| BCM2835_DMA_TI_D_INC),
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->reset_spi_and_config_speed,pad[0]),
			_FIXED(DST),    (BCM2835_SPI_BASE_BUS
					+ BCM2835_SPI_CLK),
			_FIXED(LEN),    8,
			_FIXED(STRIDE), 4, 8,
			_SPI(PAD0),     0, /* spi_device_data->spi_cs_set */
			_VARY(PAD1),    0  /* SPI_SPEED as a divider */
		);
	DMA_LINK(frag,cs_select,reset_spi_and_config_speed);

	/* set DMA transfer length in SPI */
	ADD_TO_DMA_FRAGMENT(frag,config_length,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->config_length,pad[0]),
			_SPI(DST),      (BCM2835_SPI_BASE_BUS
					+ BCM2835_SPI_DLEN),
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0,0,
			_MESG(PAD0),    0, /* SPI_DMA_LENGTH - this is a sum of individual */
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,reset_spi_and_config_speed,config_length);

	/* enable SPI+DMA */
	ADD_TO_DMA_FRAGMENT(frag,config_spi,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(frag->config_spi,pad[0]),
			_SPI(DST),      (BCM2835_SPI_BASE_BUS
					+ BCM2835_SPI_CS),
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0,0,
			_SPI(PAD0),     0, /* spi_device_data->spi_cs_set */
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,config_length,config_spi);

	/* configure the tx transfer itself */
	ADD_TO_DMA_FRAGMENT(frag,schedule_transfer_tx,dma_tx,
			_MESG(TI),      BCM2835_DMA_TI_WAIT_RESP,
			_VARY(SRC),     0, /* the source from transfer */
			_SPI(DST),      (BCM2835_SPI_BASE_BUS
					+ BCM2835_SPI_FIFO),
			_VARY(LEN),     0, /* the length from transfer */
			_FIXED(STRIDE), 0,0,
			_IGNORE(PAD0),  0,
			_IGNORE(PAD1),  0
		);
	/* note that it gets linked below in set_tx_dma_next */

	/* prepare the tx-DMA - setting the next address from which to load
	 the DMA control block */
	ADD_TO_DMA_FRAGMENT(frag,set_tx_dma_next,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->set_tx_dma_next,pad[0]),
			_FIXED(DST),    (bs->dma_tx.bus_addr
					+BCM2835_DMA_ADDR),
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0,0,
			_FIXED(PAD0),   FIELD_DMA_ADDR(
				frag->schedule_transfer_tx,ti),
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,config_spi,set_tx_dma_next);

	/* and start the tx-DMA */
	ADD_TO_DMA_FRAGMENT(frag,start_tx_dma,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->start_tx_dma,pad[0]),
			_FIXED(DST),    (bs->dma_tx.bus_addr
					+BCM2835_DMA_CS),
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0,0,
			_FIXED(PAD0),   BCM2835_DMA_CS_ACTIVE,
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,config_spi,set_tx_dma_next);

	/* configure the rx transfer itself */
	ADD_TO_DMA_FRAGMENT(frag,schedule_transfer_rx,dma_rx,
			_VARY(TI),      BCM2835_DMA_TI_WAIT_RESP,
			_SPI(SRC),      (BCM2835_SPI_BASE_BUS
					+ BCM2835_SPI_FIFO),
			_SPI(DST),      0, /* the destination from transfer */
			_VARY(LEN),     0, /* the length from transfer */
			_FIXED(STRIDE), 0,0,
			_IGNORE(PAD0),  0,
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,set_tx_dma_next,schedule_transfer_rx);

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

	/* configure the rx transfer itself */
	ADD_TO_DMA_FRAGMENT(frag,schedule_transfer_rx,dma_rx,
			_VARY(TI),      BCM2835_DMA_TI_WAIT_RESP,
			_SPI(SRC),      (BCM2835_SPI_BASE_BUS
					+ BCM2835_SPI_FIFO),
			_SPI(DST),      0, /* the destination from transfer */
			_VARY(LEN),     0, /* the length from transfer */
			_FIXED(STRIDE), 0,0,
			_IGNORE(PAD0),  0,
			_IGNORE(PAD1),  0
		);

	/* configure the tx transfer itself */
	ADD_TO_DMA_FRAGMENT(frag,schedule_transfer_tx,dma_tx,
			_MESG(TI),      BCM2835_DMA_TI_WAIT_RESP,
			_VARY(SRC),     0, /* the source from the transfer */
			_SPI(DST),      (BCM2835_SPI_BASE_BUS
					+ BCM2835_SPI_FIFO),
			_VARY(LEN),     0, /* the length from transfer */
			_FIXED(STRIDE), 0,0,
			_IGNORE(PAD0),  0,
			_IGNORE(PAD1),  0
		);

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

	/* delay by 0.5 of the clock cycle */
	ADD_TO_DMA_FRAGMENT(frag,delay_pre,dma_rx,
			_FIXED(TI),     (
				BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_WAITS(0x1f)
				| BCM2835_DMA_TI_NO_WIDE_BURSTS
				| BCM2835_DMA_TI_S_IGNORE
				| BCM2835_DMA_TI_D_IGNORE
				),
			_FIXED(SRC),    FIELD_DMA_ADDR(frag->delay_pre,pad[0]),
			_FIXED(DST),    FIELD_DMA_ADDR(frag->delay_pre,pad[1]),
			_VARY(LEN),     0, /* last_config_setup delay_half_clock
					     or the value given in transfer */
			_FIXED(STRIDE), 0, 0,
			_IGNORE(PAD0),  0,
			_IGNORE(PAD1),  0
		);

	/* resetting CS */
	ADD_TO_DMA_FRAGMENT(frag,cs_deselect,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->
				cs_deselect,
				pad[0]),
			_SPI(DST),      0, /* spi_device_data
					      -> chipselect_deselect_gpio_reg */
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0, 0,
			_SPI(PAD0),     0, /* spi_device_data
					      ->chipselect_bitfield */
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,delay_pre,cs_deselect);

	/* delay by 0.5 of the clock cycle */
	ADD_TO_DMA_FRAGMENT(frag,delay_post,dma_rx,
			_FIXED(TI),     (
				BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_WAITS(0x1f)
				| BCM2835_DMA_TI_NO_WIDE_BURSTS
				| BCM2835_DMA_TI_S_IGNORE
				| BCM2835_DMA_TI_D_IGNORE
				),
			_FIXED(SRC),    FIELD_DMA_ADDR(frag->delay_post,pad[0]),
			_FIXED(DST),    FIELD_DMA_ADDR(frag->delay_post,pad[1]),
			_VARY(LEN),     0, /* last_config_setup delay_half_clock
					     or the value given in transfer */
			_FIXED(STRIDE), 0, 0,
			_IGNORE(PAD0),  0,
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,cs_deselect,delay_post);

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

	/* delay by 0.5 of the clock cycle */
	ADD_TO_DMA_FRAGMENT(frag,delay,dma_rx,
			_FIXED(TI),     (
				BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_WAITS(0x1f)
				| BCM2835_DMA_TI_NO_WIDE_BURSTS
				| BCM2835_DMA_TI_S_IGNORE
				| BCM2835_DMA_TI_D_IGNORE
				),
			_FIXED(SRC),    FIELD_DMA_ADDR(frag->delay,pad[0]),
			_FIXED(DST),    FIELD_DMA_ADDR(frag->delay,pad[1]),
			_VARY(LEN),     0, /* the value given in transfer */
			_FIXED(STRIDE), 0, 0,
			_IGNORE(PAD0),  0,
			_IGNORE(PAD1),  0
		);

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

	/* the trigger IRQ message in TX */
	ADD_TO_DMA_FRAGMENT(frag,message_finished,dma_tx,
			_FIXED(TI),     (
				BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_S_INC
				| BCM2835_DMA_TI_D_INC),
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->message_finished,pad[0]),
			_FIXED(DST),    bs->dma_status_bus_addr,
			_FIXED(LEN),    8,
			_FIXED(STRIDE), 0,0,
			_VARY(PAD0),   0,/* not sure what is needed here yet */
			_VARY(PAD0),   1 /* not sure what is needed here yet */
		);

	/* prepare the tx-DMA - setting the next address from which to load
	 the DMA control block */
	ADD_TO_DMA_FRAGMENT(frag,set_tx_dma_next,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->set_tx_dma_next,pad[0]),
			_FIXED(DST),    (bs->dma_tx.bus_addr
					+BCM2835_DMA_ADDR),
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0,0,
			_FIXED(PAD0),   FIELD_DMA_ADDR(
				frag->message_finished,ti),
			_IGNORE(PAD1),  0
		);

	/* and start the tx-DMA */
	ADD_TO_DMA_FRAGMENT(frag,start_tx_dma,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    FIELD_DMA_ADDR(
				frag->start_tx_dma,pad[0]),
			_FIXED(DST),    (bs->dma_tx.bus_addr
					+BCM2835_DMA_CS),
			_FIXED(LEN),    4,
			_FIXED(STRIDE), 0,0,
			_FIXED(PAD0),   BCM2835_DMA_CS_ACTIVE,
			_IGNORE(PAD1),  0
		);
	DMA_LINK(frag,set_tx_dma_next,start_tx_dma);

	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,set_tx_dma_next);
	FREE_DMA_IN_FRAGMENT(frag,start_tx_dma);
	FREE_DMA_IN_FRAGMENT(frag,message_finished);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

#define transfer_to_dma(x) 0
static int _bcm2835dma_spi_compo_add_transfer_helper (
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	struct dma_fragment_transfer* frag,
	gfp_t gfpflags);

int bcm2835dma_spi_compo_add_config_transfer(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int cs,ret;
#if 0
	/* get the object from the correct chain */
	struct dma_fragment_setup_transfer* 
		frag = (struct dma_fragment_setup_transfer*)
		dma_fragment_cache_fetch(
			&bs->fragment_transfer,
			gfpflags);
	if (!frag)
		return -ENOMEM;

	/* calc CS/GPIO */
	cs = spi->chip_select;
	if (cs==0) 
		cs=BCM2835_SPI_GPIO_CS0;
	else if (cs==1) 
		cs=BCM2835_SPI_GPIO_CS1;

	
	/* select correct address based on cs and CS_POLARITY*/
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->dst =
		(spi->mode & SPI_CS_HIGH) ?
		0x7e20001C:0x7e200028
		+ (cs < 32) ? 0:4
		;

	/* and the bitmap */
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->pad[0] =
		1<<(cs%32);
	
	/* now calculate and assign the clock divider */
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->pad[0] =
		/* TODO */
		0 ;
#ifdef WITH_PREPARED
	if (xfer->prepared_mask & SPI_TRANSFER_PREPARED_MAY_CHANGE_SPEED) {
		/* ADD the pre-transform to reconfig speed */
	}
#endif

	/* and the CS flags */
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->pad[1] =
		BCM2835_SPI_CS_TA
		| BCM2835_SPI_CS_CLEAR_RX
		| BCM2835_SPI_CS_CLEAR_TX
		| BCM2835_SPI_CS_CS_01
		| BCM2835_SPI_CS_CS_10
		| ((spi->mode & SPI_CPOL) ? BCM2835_SPI_CS_CPOL : 0)
		| ((spi->mode & SPI_CPHA) ? BCM2835_SPI_CS_CPHA : 0)
		;
	
	/* and the final SPI_CS settings */
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->pad[0] =
		BCM2835_SPI_CS_TA
		| BCM2835_SPI_CS_DMAEN
		| BCM2835_SPI_CS_CLEAR_RX
		| BCM2835_SPI_CS_CLEAR_TX
		| BCM2835_SPI_CS_CS_01
		| BCM2835_SPI_CS_CS_10
		| ((spi->mode & SPI_CPOL) ? BCM2835_SPI_CS_CPOL : 0)
		| ((spi->mode & SPI_CPHA) ? BCM2835_SPI_CS_CPHA : 0)
		;


	/* and the length */
	frag->length = 
		&LINK_TO_BCM2835_DMA_CB(frag->config_length) -> pad[0];

	*frag->length=xfer->len;
#ifdef WITH_PREPARED
	if ( (is_prepared)
		&& (xfer->prepared_mask
			& SPI_TRANSFER_PREPARED_MAY_CHANGE_LEN
			)) {
		/* ADD the pre-transform to copy length */
	}
#endif

	ret=_bcm2835dma_spi_compo_add_transfer_helper(
		xfer,compo,(struct dma_fragment_transfer*)frag,gfpflags
		);

	/* and now we can link it */
#endif
	return ret;
}

int bcm2835dma_spi_compo_add_transfer(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int ret;

	/* get the object from the correct chain */
	struct dma_fragment_transfer* 
		frag = (struct dma_fragment_transfer*)
		dma_fragment_cache_fetch(
			&bs->fragment_transfer,
			gfpflags);
	if (!frag)
		return -ENOMEM;
#if 0
	ret=_bcm2835dma_spi_compo_add_transfer_helper(
		xfer,compo,frag,gfpflags
		);
#endif
	/* and now we can link it */

	return ret;
}

static int _bcm2835dma_spi_compo_add_transfer_helper (
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	struct dma_fragment_transfer* frag,
	gfp_t gfpflags)
{
	/* configure the tx part */
	if (xfer->tx_buf) {
		if (xfer->tx_dma) {
			LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->src
				= xfer->tx_dma;
		} else {
			LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->src
				= transfer_to_dma(xfer->tx_buf);
			/* NOTE: we need to unmap after the transfer as well */
		}
		LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->ti =
			BCM2835_DMA_TI_S_INC
			| BCM2835_DMA_TI_WAIT_RESP
			;
#ifdef WITH_PREPARED
		if (is_prepared) {
			if (xfer->prepared_mask
				& SPI_TRANSFER_PREPARED_MAY_CHANGE_TX_BUF_ADDR
				) {
			/* ADD the pre-transform to map Memory to DMA */
			/* ADD the post-transform to unmap Memory to DMA */
			} else if (xfer->prepared_mask
				& SPI_TRANSFER_PREPARED_MAY_CHANGE_TX_DMA_ADDR
				) {
			/* ADD the pre-transform to copy the pointer */
			}
		}
#endif

	} else {
		LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->src = 0;
		LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->ti = 0;
	}

	/* configure the rx part */
	if (xfer->rx_buf) {
		if (xfer->rx_dma) {
			LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->dst
				= xfer->rx_dma;
		} else {
			LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->dst
				= transfer_to_dma(xfer->rx_buf);
			/* NOTE: we need to unmap after the transfer as well */
		}
		LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->ti =
			BCM2835_DMA_TI_D_INC
			| BCM2835_DMA_TI_WAIT_RESP
			;
#ifdef WITH_PREPARED
		if (is_prepared) {
			if (xfer->prepared_mask
				& SPI_TRANSFER_PREPARED_MAY_CHANGE_RX_BUF_ADDR
				) {
			/* ADD the pre-transform to map Memory to DMA */
			/* ADD the post-transform to unmap Memory to DMA */
			} else if (xfer->prepared_mask
				& SPI_TRANSFER_PREPARED_MAY_CHANGE_RX_DMA_ADDR
				) {
			/* ADD the pre-transform to copy the pointer */
			}
		}
#endif
	} else {
		LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->src = 0;
		LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->ti = 0;
	}

	/* the return flags */
	return (xfer->len%4)?1:0;
}

int bcm2835dma_spi_compo_add_cs_change(
	struct spi_device *spi,
	struct spi_transfer *xfer,
	struct dma_fragment_composite *compo,
	gfp_t gfpflags)
{
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int cs;

#if 0
	/* get the object from the correct chain */
	struct dma_fragment_cs_deselect* 
		frag = (struct dma_fragment_cs_deselect*)
		dma_fragment_cache_fetch(
			&bs->fragment_cs_deselect,
			gfpflags);
	if (!frag)
		return -ENOMEM;

	/* calc CS/GPIO */
	cs = spi->chip_select;
	if (cs==0) 
		cs=BCM2835_SPI_GPIO_CS0;
	else if (cs==1) 
		cs=BCM2835_SPI_GPIO_CS1;
	
	/* select correct address based on cs and CS_POLARITY*/
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->dst =
		(spi->mode & SPI_CS_HIGH) ?
		0x7e200028:0x7e20001C
		+ (cs < 32) ? 0:4
		;

	/* and the bitmap */
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->pad[0] =
		1<<(cs%32);
#endif
	/* a cs-change requires a reconfig, so we return 1 */
	return 1;

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
