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
#include <linux/dma-mapping.h>

extern bool debug_msg;
extern bool debug_dma;
extern int delay_1us;
static u32 static_zero = 0;

/**
 * bcm2835dma_dump_dma_link - dumping wrapper arround the generic CB controll block
 * @prefix: the prefix on each line
 * @link: the dma_link to dump
 * @flags: some flags
 */
void bcm2835dma_dump_dma_link(
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
 * *dma_link_to_bcm2835_dma_cb - helper to get the casted pointer of the DMA CB
 *   from the dmalink given
 * @dmalink: dma_link to cast
 */
static inline struct bcm2835_dma_cb *dma_link_to_bcm2835_dma_cb(
	struct dma_link * dmalink)
{
	return (struct bcm2835_dma_cb *)dmalink->dmablock;
}

/**
 * link_dma_link - links the second dma_link to get executed after the first
 * @first: the dma_link that is linked to the next
 * @second: the dma_link that is being linked to the first
 */
static inline void link_dma_link(struct dma_link *first,struct dma_link * second) {
	dma_link_to_bcm2835_dma_cb(first)->next = second->dmablock_dma;
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
	int              (*transformation)(void* src, void* dst, void* extra);
	void             *src;
	void             *dst;
	void             *extra;
};

static inline int spi_message_transform_add(
	struct list_head *head,
	int              (*transformation)(void* src, void* dst, void* extra),
	void             *src,
	void             *dst,
	void             *extra,
	gfp_t            gfpflags
	)
{
	struct spi_message_transform* trans =
		kmalloc(sizeof(trans),gfpflags);
	if (!trans)
		return -ENOMEM;

	list_add_tail(&trans->message_transform_chain,head);

	trans->transformation=transformation;
	trans->src=src;
	trans->dst=dst;
	trans->extra=extra;
	
	return 0;
}

static inline void spi_message_transform_release(struct list_head *head)
{
	while( !list_empty(head)) {
		struct spi_message_transform *trans
			= list_first_entry(head,
                                        typeof(*trans),
					message_transform_chain);
		list_del(&trans->message_transform_chain);
		kfree(trans);
        }
}

/******************************************************************************
 * the fragments themselves
 *****************************************************************************/

/*-----------------------------------------------------------------------------
 * dma_fragment_composite_spi
 *---------------------------------------------------------------------------*/

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

/*-----------------------------------------------------------------------------
 * some helper functions/macros
 *---------------------------------------------------------------------------*/

/**
 * FREE_RMA_IN_FRAGMENT - helper for freeing the dma_link in the fragment
 * @frag: the dma_fragment to which this belongs
 * @field: the field which we want to get populated with the pointer
 */
#define FREE_DMA_IN_FRAGMENT(frag,field)   \
	if (frag->field)		     \
		dma_link_free(frag->field);

/**
 * BCM2835_DMA_CB_MEMBER_DMA_ADDR - for a member in the DMA CB create the corresponding 
 *   dma-address
 * @dmalink: dma_link to base this on
 * @member: the member field in the dma CB
 */
#define BCM2835_DMA_CB_MEMBER_DMA_ADDR(dmalink,member)					\
	( dmalink->dmablock_dma + offsetof(struct bcm2835_dma_cb,member))
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
		do_pad0, v_pad0,					\
		do_pad1, v_pad1						\
	)								\
	_ADD_TO_DMA_FRAGMENT(frag,field,dmachannel,			\
		do_ti, v_ti,						\
		do_src, v_src,						\
		do_dst, v_dst,						\
		do_length, v_length,					\
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
		do_pad0, v_pad0,					\
		do_pad1, v_pad1						\
	)								\
	if (! ( frag->field = dma_link_alloc(				\
				device,pool,				\
				bs->dmachannel.chan,			\
				gfpflags				\
				)))					\
		goto error;						\
	dma_fragment_add(						\
		(struct dma_fragment *)frag,				\
		(struct dma_link *)frag->field				\
		);							\
	{								\
		struct dma_link *link =					\
			(struct dma_link *)frag->field;			\
		struct bcm2835_dma_cb *block =				\
			dma_link_to_bcm2835_dma_cb(link);		\
		if ( do_ti == 1 )					\
			block->ti = v_ti;				\
		if ( do_src == 1 )					\
			block->src = (u32) v_src;			\
		if ( do_dst == 1 )					\
			block->dst = (u32) v_dst;			\
		if ( do_length == 1)					\
			block->length = v_length;			\
		block->next=0;						\
		if ( do_pad0 == 1 )					\
			block->pad[0] = v_pad0;				\
		if ( do_pad1 ==1 )					\
			block->pad[1] = v_pad1;				\
		block->stride = 0;					\
	}

/**
 * _HELPER - helper macros mostly to document when/from where each field
 * gets populated.
 * @X: a dummy argument to describe the field for which this applies
 */

#define _IGNORE(X) 0 /* do not set the value */
#define _FIXED(X)  1 /* set the value once during initialization */
#define _SPI(X)    2 /* set the value when linking fragments 
			- data from spi_device */
#define _MESG(X)   3 /* set the value when linking fragments
			- data from spi_message/spi_transfer */
#define _VARY(X)   4 /* set the value on each invocation if flagged
			as optimize_vary, otherwise assign when linking
			- data from spi_message/spi_transfer */

/**
 * THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR - shortcut for a member in the 
 * current DMA CB create the corresponding dma-address.
 *   Only valid within a ADD_TO_DMA_FRAGMENT macro
 * @member: the member field in the dma CB
 * assumes an existing struct dma_link variable link
 */
#define THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(member)	\
	BCM2835_DMA_CB_MEMBER_DMA_ADDR(link,member)

/**
 * VARY_HELPER - macro that reduces coding effort
 *   depending on the flag it will either execute the code directly or
 *   add it to the structure for later executing it (optimized messages)
 * @flag: the bit to check in vary variable
 * @function: the function to call
 * @src: the src argument for the function to call
 * @dst: the dst argument for the function to call
 * @extra: the extra argument for the function to call
 */
#define VARY_HELPER(flag,function,src,dst,extra)			\
	if (vary & flag) {						\
		err = spi_message_transform_add(			\
			&composite->composite.message_pre_transform_chain, \
			function,src,dst,extra,gfpflags);		\
	} else {							\
		err = function(src,dst,extra);				\
	}								\
	if (err)							\
		goto error;

#define VARY_HELPER_POST(flag,function,src,dst,extra)			\
	err = spi_message_transform_add(				\
		&composite->composite.message_post_transform_chain,	\
		function,src,dst,extra,gfpflags);			\
	if (err)							\
		goto error;

/**
 * link_to_composite - link a dma_fragment to a dma_fragment_composite
 * @frag: the fragment to link
 * @composite: the composite to link to
 * this links at the end of the list
 */
static inline void link_to_composite(
	struct dma_fragment *frag,
	struct dma_fragment_composite_spi *composite)
{
	struct dma_link *last_link,*first_link;
	struct dma_fragment *prev_frag;

	dma_fragment_composite_add(
		frag,
		(struct dma_fragment_composite *) composite);

	if (list_empty(&frag->dma_fragment_chain))
		return;
	
	if (list_is_singular(&frag->dma_fragment_chain))
		return;

	prev_frag = list_prev_entry(frag,dma_fragment_chain);		       
	if (list_empty(&prev_frag->dma_link_chain))
		return;
	
	last_link = list_last_entry(
		&prev_frag->dma_link_chain,
		typeof(*last_link),
		dma_link_chain);

	first_link = list_first_entry(
		&frag->dma_link_chain,
		typeof(*last_link),
		dma_link_chain);

	dma_link_to_bcm2835_dma_cb(last_link)->next =
		first_link->dmablock_dma;
}

static inline int bcm2835dma_spi_compo_add_transfer_helper(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct dma_fragment_composite_spi *composite,
	struct dma_fragment_transfer* frag,
	u32 vary,
	gfp_t gfpflags);

/*-----------------------------------------------------------------------------
 * message transforms
 *---------------------------------------------------------------------------*/
/**
 * message_transform_copy_add - copy value to different location
 *   and optionally also adding it to another location
 * @src: pointer to source value
 * @dst: pointer to destination where source gets copied to
 * @extra: pointer to destination where source gets added to
 */
static inline int message_transform_copyadd_word(
	void *src, void *dst, void *extra)
{
	if (dst)
		*(u32*)dst = *(u32*)src;
	if (extra)
		*(u32*)extra += *(u32*)src;
	return 0;
}

/**
 * message_transform_dma_map_xfer - fill in some data in the cb
 *  for receiving data based on the buffer address
 * @src: pointer to spi_xfer
 * @dst: pointer to spi_master
 * @extra: which field to modify (0 is rx)
 */
static inline int message_transform_dma_map_xfer(
	void *src, void *dst, void *extra)
{
	struct spi_transfer *xfer = src;
	struct spi_master *master = dst;

	if (extra) {
		if ( xfer->tx_buf )
			xfer->tx_dma = dma_map_single_attrs(
				&master->dev,
				(void *)xfer->tx_buf,
				xfer->len,
				DMA_TO_DEVICE,
				NULL);
	} else {
		if ( xfer->rx_buf )
			xfer->rx_dma = dma_map_single_attrs(
				&master->dev,
				(void *)xfer->rx_buf,
				xfer->len,
				DMA_FROM_DEVICE,
				NULL);
	}

	return 0;
}
/**
 * message_transform_dma_unmap_xfer - fill in some data in the cb
 *  for receiving data based on the buffer address
 * @src: pointer to spi_xfer
 * @dst: pointer to spi_master
 * @extra: which field to modify (0 is rx)
 */
static inline int message_transform_dma_unmap_xfer(
	void *src, void *dst, void *extra)
{
	struct spi_transfer *xfer = src;
	struct spi_master *master = dst;

	if (extra) {
		if ( xfer->tx_dma )
			dma_unmap_single_attrs(
				&master->dev,
				xfer->tx_dma,
				xfer->len,
				DMA_TO_DEVICE,
				NULL);
		xfer->tx_dma = 0;
	} else {
		if ( xfer->rx_dma )
			dma_unmap_single_attrs(
				&master->dev,
				xfer->rx_dma,
				xfer->len,
				DMA_FROM_DEVICE,
				NULL);
		xfer->rx_dma = 0;
	}

	return 0;
}

/*-----------------------------------------------------------------------------
 * dma_fragment_transfer
 *---------------------------------------------------------------------------*/

/**
 * dma_fragment_transfer - structure used to initiate an additional
 *   transfer
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments later
 */
struct dma_fragment_transfer {
	struct dma_fragment fragment;
	/* the individual objects */
	struct dma_link     *transfer_rx;
	struct dma_link     *transfer_tx;
};

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
	ADD_TO_DMA_FRAGMENT(
		frag,transfer_rx,dma_rx,
		_VARY(TI),      BCM2835_DMA_TI_WAIT_RESP,
		_SPI(SRC),      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO),
		_VARY(DST),     0, /* transfer.rx_dma */
		_VARY(LEN),     0, /* transfer.length */
		_IGNORE(PAD0),  0,
		_IGNORE(PAD1),  0
		);

	/* configure the tx transfer itself */
	ADD_TO_DMA_FRAGMENT(
		frag,transfer_tx,dma_tx,
		_MESG(TI),      BCM2835_DMA_TI_WAIT_RESP,
		_VARY(SRC),     0, /* transfer.tx_dma */
		_SPI(DST),      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO),
		_VARY(LEN),     0, /* transfer.length */
		_IGNORE(PAD0),  0,
		_IGNORE(PAD1),  0
		);
	
	return &frag->fragment;
	
error:
	FREE_DMA_IN_FRAGMENT(frag,transfer_rx);
	FREE_DMA_IN_FRAGMENT(frag,transfer_tx);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

/**
 * message_transform_calc_fill_dmarx - fill in some data in the cb
 *  for receiving data based on the buffer address
 * @src: pointer to rx-buffer
 * @dst: pointer to dma_cb
 * @extra: ignore
 */
static inline int message_transform_fill_rxdma(
	void *src, void *dst, void *extra)
{
	struct bcm2835_dma_cb *cb = (struct bcm2835_dma_cb *)dst;
	/* assign the source */
	cb->dst = (dma_addr_t)src;
	/* and set the flags */
	cb->ti = BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| ( (src) ? BCM2835_DMA_TI_D_INC
			: BCM2835_DMA_TI_D_IGNORE);

	return 0;
}

/**
 * message_transform_calc_fill_dmarx - fill in some data in the cb
 *  for receiving data based on the buffer address
 * @src: pointer to rx-buffer
 * @dst: pointer to dma_cb
 * @extra: ignore
 */
static inline int message_transform_fill_txdma(
	void *src, void *dst, void *extra)
{
	struct bcm2835_dma_cb *cb = (struct bcm2835_dma_cb*)dst;
	/* assign the source */
	cb->src = (dma_addr_t)src;
	/* and set the flags */
	cb->ti = BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| ( (src) ? BCM2835_DMA_TI_S_INC
			: BCM2835_DMA_TI_S_IGNORE);
	return 0;
}


/**
 * bcm2835dma_spi_compo_add_transfer - merges a setup_transfer
 *   dma_fragment into a dma_composite fragment
 * @mesg: the spi message for which we run this
 * @xfer: the corresponding transfer in question
 * @composite: the composite to which we add this
 * @flags: flags for this process - e.g: do we optimize now?
 * @gfpflags: flags used when we need to allocate memory
 * notes:
 *   will set composite->last_setup_transfer
 *   may set/clear composite->last_transfer to flag that we need to
 *     reset the SPI-FIFOs prior to the next transfer
 */
int bcm2835dma_spi_compo_add_transfer(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct dma_fragment_composite_spi *composite,
	u32 vary,
	gfp_t gfpflags)
{
	struct spi_device * spi = mesg->spi;
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int err=0;

	/* fetch fragment from fragment_cache */
	struct dma_fragment_transfer* frag
		= (typeof(frag)) dma_fragment_cache_fetch(
			&bs->fragment_transfer,
			gfpflags);
	if (!frag)
		return -ENOMEM;

	/* call the common code for transfer */
	err=bcm2835dma_spi_compo_add_transfer_helper(
		mesg,xfer,composite,
		(struct dma_fragment_transfer *)frag,
		vary,gfpflags);
	if (err)
		goto error;

	/* link it to composite */
	/* TODO */

	return 0;
error:
	return err;
}

/*-----------------------------------------------------------------------------
 * dma_fragment_setup_transfer
 *---------------------------------------------------------------------------*/

/**
 * dma_fragment_setup_transfer - structure used to set up SPI and initiate
 *   a transfer
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments later
 * note that we extend the simple dma_fragment_transfer with the same pattern 
 *   as above, so that we can simplify the linking pattern
 */
struct dma_fragment_setup_transfer {
	struct dma_fragment_transfer fragment;
	/* additional data */
	struct dma_link     *cs_select;
	struct dma_link     *reset_spi;
	struct dma_link     *config_clock_length;
	struct dma_link     *config_spi;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
	/* some of the timing data that we need 
	   - filled in based on the SPI-clock */
	u32 delay_transfers_half_cycle;
};

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
	ADD_TO_DMA_FRAGMENT(
		frag,cs_select,dma_rx,
		_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),      0, /* spi_device_data->cs_select_gpio_reg */
		_FIXED(LEN),    4,
		_SPI(PAD0),     0, /* spi_device_data->cs_bitfield */
		_IGNORE(PAD1),  0
		);

	/* now reset SPI FIFOS and configure SPI_SPEED */
	ADD_TO_DMA_FRAGMENT(
		frag,reset_spi,dma_rx,
		_FIXED(TI),     (BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_TDMODE
				| BCM2835_DMA_TI_S_INC
				| BCM2835_DMA_TI_D_INC),
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK),
		_FIXED(LEN),    4,
		_SPI(PAD0),     0, /* spi_device_data->spi_cs_reset */
		_IGNORE(PAD1),  0
		);
	link_dma_link(frag->cs_select,frag->reset_spi);

	/* set DMA transfer length and the clock speed in SPI */
	ADD_TO_DMA_FRAGMENT(
		frag,config_clock_length,dma_rx,
		_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK),
		_FIXED(LEN),    8,
		_VARY(PAD0),    0,  /* SPI_SPEED as a divider */
		_VARY(PAD1),    0 /* SUM of transfer.length */
		);
	link_dma_link(frag->reset_spi,frag->config_clock_length);

	/* enable SPI+DMA */
	ADD_TO_DMA_FRAGMENT(
		frag,config_spi,dma_rx,
		_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),      (BCM2835_SPI_BASE_BUS
				+ BCM2835_SPI_CS),
		_FIXED(LEN),    4,
		_SPI(PAD0),     0, /* spi_device_data->spi_cs_set */
		_IGNORE(PAD1),  0
		);
	link_dma_link(frag->config_clock_length,frag->config_spi);

	/* configure the tx transfer itself */
	ADD_TO_DMA_FRAGMENT(
		frag,fragment.transfer_tx,dma_tx,
		_MESG(TI),      BCM2835_DMA_TI_WAIT_RESP,
		_VARY(SRC),     0, /* transfer.tx_dma */
		_SPI(DST),      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO),
		_VARY(LEN),     0, /* transfer.length */
		_IGNORE(PAD0),  0,
		_IGNORE(PAD1),  0
		);
	/* note that it gets linked below in set_tx_dma_next */
	
	/* prepare the tx-DMA - setting the next address from which to load
	 the DMA control block */
	ADD_TO_DMA_FRAGMENT(
		frag,set_tx_dma_next,dma_rx,
		_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),    (bs->dma_tx.bus_addr + BCM2835_DMA_ADDR),
		_FIXED(LEN),    4,
		_FIXED(PAD0),   (/* the dma address of the TX-transfer */
			frag->fragment.transfer_tx->dmablock_dma),
		_IGNORE(PAD1),  0
		);
	link_dma_link(frag->config_spi,frag->set_tx_dma_next);

	/* and start the tx-DMA */
	ADD_TO_DMA_FRAGMENT(
		frag,start_tx_dma,dma_rx,
		_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),    (bs->dma_tx.bus_addr + BCM2835_DMA_CS),
		_FIXED(LEN),    4,
		_FIXED(PAD0),   BCM2835_DMA_CS_ACTIVE,
		_IGNORE(PAD1),  0
		);
	link_dma_link(frag->config_spi,frag->set_tx_dma_next);
	
	/* configure the rx transfer itself */
	ADD_TO_DMA_FRAGMENT(
		frag,fragment.transfer_rx,dma_rx,
		_VARY(TI),      BCM2835_DMA_TI_WAIT_RESP,
		_SPI(SRC),      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO),
		_SPI(DST),      0, /* transfer.rx_dma */
		_VARY(LEN),     0, /* transfer.length */
		_IGNORE(PAD0),  0,
		_IGNORE(PAD1),  0
		);
	link_dma_link(frag->set_tx_dma_next,frag->fragment.transfer_rx);
	
	return &frag->fragment.fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,cs_select);
	FREE_DMA_IN_FRAGMENT(frag,reset_spi);
	FREE_DMA_IN_FRAGMENT(frag,config_clock_length);
	FREE_DMA_IN_FRAGMENT(frag,config_spi);
	FREE_DMA_IN_FRAGMENT(frag,set_tx_dma_next);
	FREE_DMA_IN_FRAGMENT(frag,start_tx_dma);
	FREE_DMA_IN_FRAGMENT(frag,fragment.transfer_rx);
	FREE_DMA_IN_FRAGMENT(frag,fragment.transfer_tx);

	dma_fragment_free(&frag->fragment.fragment);

	return NULL;
}

/**
 * message_transform_speed_to_clockdivider - calculate clock divider
 *   and also half cycle delay
 * @src: void pointer to xfer->speed_hz
 * @dst: void pointer to dma_fragment_setup_transfer
 * @extra: void pointer to spi_device
 */
static inline int message_transform_speed_to_clockdivider(
	void *src, void *dst, void *extra)
{
	/* transform to what we need */
	u32 spi_hz = *((u32*)src);
	struct dma_fragment_setup_transfer * frag = dst;
	struct spi_device *spi = extra;
	struct spi_master *master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	u32 clk_hz = clk_get_rate(bs->clk);
	u32 cdiv;

	/* now calculate the clock divider and other delay cycles */
        if (spi_hz >= clk_hz / 2) {
                cdiv = 2; /* clk_hz/2 is the fastest we can go */
        } else if (spi_hz) {
                /* CDIV must be a power of two as per documentation
                 * cdiv = roundup_pow_of_two(DIV_ROUND_UP(clk_hz, spi_hz));
		 * but empirically notro found that it is not needed,
		 * so:
		 */
		cdiv = DIV_ROUND_UP(clk_hz,spi_hz);
                if (cdiv >= 65536)
                        cdiv = 0; /* 0 is the slowest we can go */
        } else
                cdiv = 0; /* 0 is the slowest we can go */

	/* and set the clock divider */
	dma_link_to_cb(frag->config_clock_length)->pad[0]=cdiv;

	/* and now calculate the delay for a half clock cycle
	   - for now we assume that it is equal to clk_div */
	if (cdiv)
		frag->delay_transfers_half_cycle=cdiv;
	else 
		frag->delay_transfers_half_cycle=65535;
	
	return 0;
}


/**
 * message_transform_delay_us - calculate the value needed for delaying 
 *    by a requested amount of usecs
 * @src: pointer to the value of delay_us (if null, then delay by 0.5 clock
 * @dst: pointer to where to store it (typically length)
 * @extra: dma_fragment_setup_transfer for the timing data
 */
static inline int message_transform_delay_us(
	void *src, void *dst, void *extra)
{
	struct dma_fragment_setup_transfer * frag = extra;
	u32 delay_us=(*(u32*)src);
	if (delay_us)
		*(u32*)dst = delay_us * delay_1us;
	else
		*(u32*)dst = frag->delay_transfers_half_cycle;
	return 0;
}

/**
 * bcm2835dma_spi_compo_add_setup_transfer - merges a setup_transfer
 *   dma_fragment into a dma_composite fragment
 * @mesg: the spi message for which we run this
 * @xfer: the corresponding transfer in question
 * @composite: the composite to which we add this
 * @flags: flags for this process - e.g: do we optimize now?
 * @gfpflags: flags used when we need to allocate memory
 * notes:
 *   will set composite->last_setup_transfer
 *   may set/clear composite->last_transfer to flag that we need to
 *     reset the SPI-FIFOs prior to the next transfer
 */
int bcm2835dma_spi_compo_add_setup_transfer(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct dma_fragment_composite_spi *composite,
	u32 vary,
	gfp_t gfpflags)
{
	struct spi_device * spi = mesg->spi;
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct bcm2835dma_spi_device_data *spi_device_data=
		dev_get_drvdata(&spi->dev);
	int err=0;

	/* fetch fragment from fragment_cache */
	struct dma_fragment_setup_transfer* frag
		= (typeof(frag)) dma_fragment_cache_fetch(
			&bs->fragment_setup_transfer,
			gfpflags);
	if (!frag)
		return -ENOMEM;

	/* now start setting things up */

	/* the CS part */
	dma_link_to_cb(frag->cs_select->dmablock)->dst =
		spi_device_data->cs_select_gpio_reg;
	dma_link_to_cb(frag->cs_select->dmablock)->pad[0] =
		spi_device_data->cs_bitfield;

	/* the SPI reset part */
	dma_link_to_cb(frag->reset_spi)->pad[0] =
		spi_device_data->spi_cs_reset;

	/* the SPI speed and DMA_transfer length */
	VARY_HELPER(SPI_OPTIMIZE_VARY_FRQUENCY,
		message_transform_speed_to_clockdivider,
		&xfer->speed_hz,frag,spi);
	/* set the length  */
	VARY_HELPER(SPI_OPTIMIZE_VARY_LENGTH,
		message_transform_copyadd_word,
		&static_zero,
		&dma_link_to_cb(frag->config_clock_length)->pad[1],
		NULL);

	/* the SPI config part */
	dma_link_to_cb(frag->config_spi)->pad[0] =
		spi_device_data->spi_cs_set;

	/* set last_setup_transfer for subsequent processing steps */
	composite->last_setup_transfer = frag;
	
	/* and call the common code for transfer */
	err=bcm2835dma_spi_compo_add_transfer_helper(
		mesg,xfer,composite,
		(struct dma_fragment_transfer *)frag,
		vary,gfpflags);
	if (err)
		goto error;
	/* link it to composite and also on DMA level*/
	link_to_composite((struct dma_fragment *)frag,composite);

	return 0;
error:
	return err;
}

/**
 * bcm2835dma_spi_compo_add_transfer_helper - merges a setup_transfer
 *   dma_fragment into a dma_composite fragment
 * @mesg: the spi message for which we run this
 * @xfer: the corresponding transfer in question
 * @composite: the composite to which we add this
 * @flags: flags for this process - e.g: do we optimize now?
 * @gfpflags: flags used when we need to allocate memory
 * notes:
 *   will set composite->last_setup_transfer
 *   may set/clear composite->last_transfer to flag that we need to
 *     reset the SPI-FIFOs prior to the next transfer
 */

static inline int bcm2835dma_spi_compo_add_transfer_helper(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct dma_fragment_composite_spi *composite,
	struct dma_fragment_transfer* frag,
	u32 vary,
	gfp_t gfpflags)
{
	struct spi_device * spi = mesg->spi;
	struct spi_master * master = spi->master;
	int err=0;

	/* now the transfer part - first map the DMA address 
	   if it is not fully mapped */
	if (!mesg->is_dma_mapped) {
		VARY_HELPER(SPI_OPTIMIZE_VARY_TX,
			message_transform_dma_map_xfer,
			xfer,
			master,
			(void *)1);
		VARY_HELPER_POST(SPI_OPTIMIZE_VARY_TX,
				message_transform_dma_unmap_xfer,
				xfer,
				master,
				(void *)1);
		VARY_HELPER(SPI_OPTIMIZE_VARY_RX,
			message_transform_dma_map_xfer,
			xfer,
			master,
			0);
		VARY_HELPER_POST(SPI_OPTIMIZE_VARY_RX,
				message_transform_dma_unmap_xfer,
				xfer,
				master,
				0);
	}

	/* and based on src set the flags */
	VARY_HELPER(SPI_OPTIMIZE_VARY_TX,
		message_transform_fill_txdma,
		&xfer->tx_dma,
		dma_link_to_cb(frag->transfer_tx),
		0
		);
	VARY_HELPER(SPI_OPTIMIZE_VARY_RX,
		message_transform_fill_rxdma,
		&xfer->rx_dma,
		dma_link_to_cb(frag->transfer_rx),
		0
		);

	VARY_HELPER(SPI_OPTIMIZE_VARY_LENGTH,
		message_transform_copyadd_word,
		&xfer->len,
		&dma_link_to_cb(frag->transfer_tx)->length,
		&dma_link_to_cb(composite->last_setup_transfer
			->config_clock_length)->pad[1]
		);
	VARY_HELPER(SPI_OPTIMIZE_VARY_LENGTH,
		message_transform_copyadd_word,
		&xfer->len,
		&dma_link_to_cb(frag->transfer_rx)->length,
		NULL
		);
	
	/* set last_transfer ONLY if a multiple of 4 */
	/* note this assumes that the fields of
	 *   dma_fragment_transfer
	 * are identical to the ones in 
	 *   dma_fragment_config_transfer
	 */
	if (vary | SPI_OPTIMIZE_VARY_LENGTH) {
		/* if we vary length then we he have to see what the lultiple is */
		if (vary | SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_4)
			composite->last_transfer = (struct dma_fragment_transfer*)frag;
		else
			composite->last_transfer = NULL;
	} else {
		composite->last_transfer = 
			(xfer->len % 4) ? NULL : (struct dma_fragment_transfer*)frag;
	}

	return 0;
error:
	return err;
}

/*-----------------------------------------------------------------------------
 * dma_fragment_cs_deselect
 *---------------------------------------------------------------------------*/

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
	ADD_TO_DMA_FRAGMENT(
		frag,delay_pre,dma_rx,
		_FIXED(TI),     (
			BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE
			),
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[1]),
		_VARY(LEN),     0, /* last_config_setup.delay_half_clock
				      or scaled transfer.delay_us */
		_IGNORE(PAD0),  0,
		_IGNORE(PAD1),  0
		);

	/* resetting CS */
	ADD_TO_DMA_FRAGMENT(
		frag,cs_deselect,dma_rx,
		_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),      0, /* spi_device_data->cs_deselect_gpio_reg */
		_FIXED(LEN),    4,
		_SPI(PAD0),     0, /* spi_device_data->cs_bitfield */
		_IGNORE(PAD1),  0
		);
	link_dma_link(frag->delay_pre,frag->cs_deselect);

	/* delay by 0.5 of the clock cycle */
	ADD_TO_DMA_FRAGMENT(
		frag,delay_post,dma_rx,
		_FIXED(TI),     (
			BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE
			),
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[1]),
		_SPI(LEN),      0, /* last_config_setup.delay_half_clock */
		_IGNORE(PAD0),  0,
		_IGNORE(PAD1),  0
		);
	link_dma_link(frag->cs_deselect,frag->delay_post);

	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,delay_pre);
	FREE_DMA_IN_FRAGMENT(frag,cs_deselect);
	FREE_DMA_IN_FRAGMENT(frag,delay_post);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

/**
 * bcm2835dma_spi_compo_add_cs_deselect - merges a cs_deselect
 *   dma_fragment into a dma_composite fragment
 * @mesg: the spi message for which we run this
 * @xfer: the corresponding transfer in question
 * @composite: the composite to which we add this
 * @flags: flags for this process - e.g: do we optimize now?
 * @gfpflags: flags used when we need to allocate memory
 */
int bcm2835dma_spi_compo_add_cs_deselect(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct dma_fragment_composite_spi *composite,
	u32 vary,
	gfp_t gfpflags)
{
	struct spi_device * spi = mesg->spi;
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct bcm2835dma_spi_device_data *spi_device_data=
		dev_get_drvdata(&spi->dev);
	int err=0;

	/* fetch fragment from fragment_cache */
	struct dma_fragment_cs_deselect* frag
		= (typeof(frag)) dma_fragment_cache_fetch(
			&bs->fragment_cs_deselect,
			gfpflags);
	if (!frag)
		return -ENOMEM;

	/* delay pre */
	VARY_HELPER(SPI_OPTIMIZE_VARY_DELAY,
		message_transform_delay_us,
		&xfer->delay_usecs,
		&dma_link_to_cb(frag->delay_pre)->length,
		composite->last_setup_transfer
		);

	dma_link_to_cb(frag->cs_deselect)->dst = 
		spi_device_data->cs_deselect_gpio_reg;
	dma_link_to_cb(frag->cs_deselect)->pad[0] =
		spi_device_data->cs_bitfield;

	dma_link_to_cb(frag->delay_post)->length =
		composite->last_setup_transfer->delay_transfers_half_cycle;

	/* we need to reset SPI after this */
	composite->last_transfer=NULL;

	/* link it to composite */
	/* TODO */

	return 0;
error:
	return err;
}

/*-----------------------------------------------------------------------------
 * dma_fragment_delay
 *---------------------------------------------------------------------------*/

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
 * bcm2835_spi_dmafragment_create_delay - create a DMA fragment
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
	ADD_TO_DMA_FRAGMENT(
		frag,delay,dma_rx,
		_FIXED(TI),     (
			BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE
			),
		_FIXED(SRC),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[1]),
		_VARY(LEN),     0, /* scaled transfer.delay_us */
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
 * bcm2835dma_spi_compo_add_delay - merges a delay
 *   dma_fragment into a dma_composite fragment
 * @mesg: the spi message for which we run this
 * @xfer: the corresponding transfer in question
 * @composite: the composite to which we add this
 * @flags: flags for this process - e.g: do we optimize now?
 * @gfpflags: flags used when we need to allocate memory
 */
int bcm2835dma_spi_compo_add_delay(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct dma_fragment_composite_spi *composite,
	u32 vary,
	gfp_t gfpflags)
{
	struct spi_device * spi = mesg->spi;
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int err=0;

	/* fetch fragment from fragment_cache */
	struct dma_fragment_delay* frag
		= (struct dma_fragment_delay*)
		dma_fragment_cache_fetch(
			&bs->fragment_delay,
			gfpflags);
	if (!frag)
		return -ENOMEM;
	
	/* delay pre */
	VARY_HELPER(SPI_OPTIMIZE_VARY_DELAY,
		message_transform_delay_us,
		&xfer->delay_usecs,
		&dma_link_to_cb(frag->delay)->length,
		composite->last_setup_transfer
		);
	/* we need to reset SPI */
	composite->last_transfer=NULL;
	
	/* link it to composite */
	/* TODO */

	return 0;

error:
	return err;
}

/*-----------------------------------------------------------------------------
 * dma_fragment_trigger_irq
 *---------------------------------------------------------------------------*/

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

	/* the trigger IRQ message in TX and copy the end 64-bit counter 
	   to the PAD fields */
	ADD_TO_DMA_FRAGMENT(
		frag,message_finished,dma_tx,
		_FIXED(TI),     (
			BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_INT_EN
			| BCM2835_DMA_TI_S_INC
			| BCM2835_DMA_TI_D_INC),
		_FIXED(SRC),    BCM2835_REG_COUNTER_64BIT_BUS,	
		_FIXED(DST),    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(LEN),    8,
		/* the destination of the transfer 
		   these need to get reset to 0
		*/
		_VARY(PAD0),   0,
		_VARY(PAD1),   0 
		);

	/* prepare the tx-DMA - setting the next address from which to load
	 the DMA control block */
	ADD_TO_DMA_FRAGMENT(
		frag,set_tx_dma_next,dma_rx,
		_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),    BCM2835_DMA_CB_MEMBER_DMA_ADDR(
			frag->set_tx_dma_next,pad[0]),
		_FIXED(DST),    (bs->dma_tx.bus_addr+BCM2835_DMA_ADDR),
		_FIXED(LEN),    4,
		_FIXED(PAD0),   BCM2835_DMA_CB_MEMBER_DMA_ADDR(
			frag->message_finished,ti),
		_IGNORE(PAD1),  0
		);

	/* and start the tx-DMA */
	ADD_TO_DMA_FRAGMENT(frag,start_tx_dma,dma_rx,
			_FIXED(TI),     BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),    BCM2835_DMA_CB_MEMBER_DMA_ADDR(
				frag->start_tx_dma,pad[0]),
			_FIXED(DST),    (bs->dma_tx.bus_addr
					+BCM2835_DMA_CS),
			_FIXED(LEN),    4,
			_FIXED(PAD0),   BCM2835_DMA_CS_ACTIVE,
			_IGNORE(PAD1),  0
		);
	link_dma_link(frag->set_tx_dma_next,frag->start_tx_dma);

	return &frag->fragment;

error:
	FREE_DMA_IN_FRAGMENT(frag,set_tx_dma_next);
	FREE_DMA_IN_FRAGMENT(frag,start_tx_dma);
	FREE_DMA_IN_FRAGMENT(frag,message_finished);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

/**
 * bcm2835dma_spi_compo_add_trigger_irq - merges a cs_deselect
 *   dma_fragment into a dma_composite fragment
 * @mesg: the spi message for which we run this
 * @xfer: the corresponding transfer in question
 * @composite: the composite to which we add this
 * @flags: flags for this process - e.g: do we optimize now?
 * @gfpflags: flags used when we need to allocate memory
 */
int bcm2835dma_spi_compo_add_trigger_irq(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct dma_fragment_composite_spi *composite,
	u32 vary,
	gfp_t gfpflags)
{
	struct spi_device * spi = mesg->spi;
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int err=0;

	/* fetch fragment from fragment_cache */
	struct dma_fragment_trigger_irq* frag
		= (typeof(frag)) dma_fragment_cache_fetch(
			&bs->fragment_trigger_irq,
			gfpflags);
	if (!frag)
		return -ENOMEM;
	
	/* forced vary to clean PAD0/PAD1 */
	vary=1;
	VARY_HELPER(1,
		message_transform_copyadd_word,
		&static_zero,
		&dma_link_to_cb(frag->message_finished)->pad[0],
		NULL);
	VARY_HELPER(1,
		message_transform_copyadd_word,
		&static_zero,
		&dma_link_to_cb(frag->message_finished)->pad[1],
		NULL);

	/* link it to composite */
	/* TODO */

	return 0;

error:
	return err;
}


/*
 * helper functions  that are used for static-temporary and optimized cases
 */





/**
 * spi_message_to_dmafragment - converts a spi_message to a dma_fragment
 * @msg:  the spi message to convert
 * @flags: some flags
 * @gfpflags: flags for allocation
 * notes:
 * * this is essentially generic and could go into generic spi
 * * we could also create an automatically prepared version 
 *     via a spi_message flag (e.g prepare on first use)
 */
struct dma_fragment *spi_message_to_fragment(
	struct spi_message *msg, int flags, gfp_t gfpflags)
{
	struct spi_device *spi = msg->spi;
	struct spi_master *master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	
	struct dma_fragment_composite_spi *compo;
	struct spi_transfer *xfer,*last_xfer;
	int err=0;

	/* fetch a composite fragment */
	compo = (typeof(compo))
		dma_fragment_cache_fetch(&bs->fragment_composite,gfpflags);
	if (! compo)
		return NULL;
	compo->last_setup_transfer = NULL;
	compo->last_transfer = NULL;
	last_xfer = NULL;

	/* now start iterating the transfers */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* check if we are the last in the list */
		int is_last=list_is_last(&msg->transfers,&xfer->transfer_list);
		/* do we need to reconfigure compared to the last transfer */
		if (compo->last_transfer) {
			if (last_xfer->speed_hz != xfer->speed_hz)
				compo->last_transfer=NULL;
			else if (last_xfer->tx_nbits != xfer->tx_nbits)
				compo->last_transfer=NULL;
			else if (last_xfer->rx_nbits != xfer->rx_nbits)
				compo->last_transfer=NULL;
			else if (last_xfer->bits_per_word!=xfer->bits_per_word)
				compo->last_transfer=NULL;
		}
		/* now decide which transfer to use, 
		   the normal or the reset version */
		if (compo->last_transfer) {
			err=bcm2835dma_spi_compo_add_transfer(
				msg,xfer,compo,flags,gfpflags);
		} else {
			err=bcm2835dma_spi_compo_add_setup_transfer(
				msg,xfer,compo,flags,gfpflags);
		}
		/* error handling */
		if (err)
			goto error;
		/* add cs_change with optional extra delay 
		   if requested or last in sequence */
		if ((xfer->cs_change)||(is_last))
			err = bcm2835dma_spi_compo_add_cs_deselect(
				msg,
				xfer,
				compo,
				flags,
				gfpflags);
		else if (xfer->delay_usecs)
			/* or add a delay if requested */
			err = bcm2835dma_spi_compo_add_delay(
				msg,
				xfer,
				compo,
				flags,
				gfpflags);
		/* handle errors */
		if (err)
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
			msg,
			NULL,
			compo,
			flags,
			gfpflags))
			goto error;
	
	/* and return it */
	return &compo->composite.fragment;

error:
	return NULL;	
}
