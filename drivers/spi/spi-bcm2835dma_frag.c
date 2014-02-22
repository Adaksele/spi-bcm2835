/*
 * Driver for Broadcom BCM2835 SPI Controllers using DMA-FRAGMENTS
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2014 Martin Sperl
 *
 * This driver is inspired by:
 * spi-bcm2835.c, Copyright (C) 2012 Chris Boot, 2013 Stephen Warren
 * spi-ath79.c,   Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c,   Copyright (C) 2006 Atmel Corporation
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
#include "spi-bcm2835dma.h"
#include <linux/spi/spi-dmafragment.h>
#include <linux/dma-mapping.h>

extern bool debug_msg;
extern bool debug_dma;
extern int delay_1us;

/*************************************************************************
 * the function creating dma_fragments - mostly used by dma_fragment_cache
 ************************************************************************/

/*------------------------------------------------------------------------
 * Helpers - some of these could be inlined functions
 *----------------------------------------------------------------------*/

/**
 * THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR - dma_addr_t of a DMA_CB.member
 * @member: the member field in the dma CB
 */
#define THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(member)			\
	( link->cb_dma + offsetof(struct bcm2835_dma_cb,member))

/**
 * START_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 *   used by all allocate functions
 * @struct_name: name of structure to allocate as frag
 */
#define START_CREATE_FRAGMENT_COMMON()					\
	struct spi_master * master = (struct spi_master *)device;	\
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);	\
	struct dma_pool *pool = bs->pool;				\
	struct dma_link *link;						\
	struct bcm2835_dma_cb *cb;

#define START_CREATE_FRAGMENT_ALLOCATE(struct_name)			\
	START_CREATE_FRAGMENT_COMMON()					\
	struct struct_name *frag =					\
		(struct struct_name *) dma_fragment_alloc(		\
			device,gfpflags,				\
			sizeof(struct struct_name));			\
	if (! frag)							\
		return NULL;

/**
 * END_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 *   used by all alloc functions
 */
#define END_CREATE_FRAGMENT_ALLOCATE()			\
	return (struct dma_fragment*)frag;	\
error:							\
        dma_fragment_free((struct dma_fragment*)frag);	\
	return NULL;

#define ADD_DMA_LINK_TO_FRAGMENT(field)					\
	link = frag->field = dma_link_alloc(				\
		pool,							\
		sizeof(*frag->field),					\
		gfpflags						\
		);							\
	if (!link)							\
		goto error;						\
	dma_fragment_add_dma_link(					\
		(struct dma_fragment *)frag,				\
		(struct dma_link *)frag->field				\
		);							\
	cb = (struct bcm2835_dma_cb *)link->cb;				\
	cb->next=0;							\
	cb->stride=0;

#define LINKTO(field)					   \
	((struct bcm2835_dma_cb *)frag->field->cb)->next = \
		link->cb_dma;

#define FIXED(field,value) _FIXED(field,value)
#define _FIXED(field,value)			\
	cb->field = value;

static inline int bcm2835dma_schedule_fragment_transform(
	struct dma_fragment *frag,
	enum dma_fragment_transform_type type,
	int (*transform)(struct dma_fragment_transform *,
                        void* data1, void* data2),
	size_t offset,
	gfp_t gfpflags)
{
	struct dma_fragment_transform *trans =
		dma_fragment_transform_alloc(
			type,transform,
			(void*) offset,
			frag,
			NULL,
			gfpflags
			);
	if (trans) {
		dma_fragment_add_dma_fragment_transform(
			frag,trans);
		return 0;
	} else
		return 1;
}

static int bcm2835dma_fragment_transform_spi(
	struct dma_fragment_transform * transform,
	void *data1, void *data2)
{
	struct spi_message *msg = data1;
	/* struct spi_transfer *xfer = data2; */
	struct spi_device *spi = msg->spi;
	struct bcm2835dma_spi_device_data *data
                = dev_get_drvdata(&spi->dev);
	memcpy(transform->dst,
		((char *)data)+(u32)transform->src,
		4);
	return 0;
}

#define SPI(field,value) _SPI(field,value)
#define _SPI(field,value)						\
	if (bcm2835dma_schedule_fragment_transform(			\
			&frag->fragment,				\
			DMA_FRAGMENT_TRANSFORM_TYPE_LINK,		\
			bcm2835dma_fragment_transform_spi,		\
			offsetof(struct bcm2835dma_spi_device_data,	\
				value),					\
			gfpflags)					\
		)							\
		goto error;

static int bcm2835dma_fragment_transform_txdma(
	struct dma_fragment_transform * transform,
	void *data1, void *data2)
{
	struct spi_message *msg = data1;
	/* struct spi_transfer *xfer = data2; */
	struct spi_master *master = msg->spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	*((u32*)transform->dst) = bs->dma_tx.bus_addr
		+ (u32)transform->src;

	return 0;
}

#define TXDMA(field,offset) _TXDMA(field,offset)
#define _TXDMA(field,offset)						\
	if (bcm2835dma_schedule_fragment_transform(			\
			&frag->fragment,				\
			DMA_FRAGMENT_TRANSFORM_TYPE_LINK,		\
			bcm2835dma_fragment_transform_txdma,		\
			offset,						\
			gfpflags))					\
		goto error;

/* these vary messages */
#define VARYMSG(a,b)
#define VARYXFER(a,b)

#define IGNORE(...)

/*------------------------------------------------------------------------
 * simple allocator for merged fragments
 *----------------------------------------------------------------------*/
static struct dma_fragment *bcm2835dma_spi_create_fragment_merged(
	struct device *device,gfp_t gfpflags)
{
	return dma_fragment_alloc(device,0,gfpflags);
}

/*------------------------------------------------------------------------
 * allocator for transfers
 *----------------------------------------------------------------------*/
struct dma_fragment_transfer_rxtx {
	/* the individual objects */
	struct dma_link     *rx;
	struct dma_link     *tx;
};

static int bcm2835dma_spi_create_fragment_transfer_common(
	struct dma_fragment_transfer_rxtx *frag,
	struct device *device,
	gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_COMMON();
	if (0==1)
		goto error;
	return 0;
error:
	return 1;
}


struct dma_fragment_transfer {
	struct dma_fragment fragment;
	struct dma_fragment_transfer_rxtx transfer;
};

static struct dma_fragment *bcm2835dma_spi_create_fragment_transfer(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_transfer);
	if (
		bcm2835dma_spi_create_fragment_transfer_common(
			&frag->transfer,device, gfpflags)
		)
		goto error;

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for setting up spi and transfer
 *----------------------------------------------------------------------*/
struct dma_fragment_config_spi_transfer {
	struct dma_fragment fragment;
	/* the rx/tx structure */
	struct dma_fragment_transfer_rxtx transfer;
	/* additional data */
	struct dma_link     *cs_select;
	struct dma_link     *reset_spi_fifo;
	struct dma_link     *config_clock_length;
	struct dma_link     *config_spi;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
	/* some of the timing data that we need
	   - filled in based on the SPI-clock */
	u32 clock_divider;
	u32 delay_transfers_half_cycle;
};

struct dma_fragment *bcm2835dma_spi_create_fragment_config_spi_transfer(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_config_spi_transfer);

	/* before we do any of this we need to schedule a cdiv calculator */
	/* todo */

	/* select chipselect - equivalent to:
	   writel(spi_dev_data->cs_bitfield,cs_select_gpio_reg);
	*/
	ADD_DMA_LINK_TO_FRAGMENT(cs_select);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	SPI  (dst,      cs_select_gpio_reg);
	FIXED(length,   4);
	SPI  (pad[0],   cs_bitfield);

	/* reset SPI fifos - equivalent to:
	 * writel(spi_dev_data->spi_reset_fifo,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(reset_spi_fifo);
	LINKTO(cs_select);
	FIXED(ti,BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED(length,   4);
	SPI(pad[0],       spi_reset_fifo);

	/* configure clock divider and transfer length  - equivalent to:
	 * writel(cdiv, BCM2835_SPI_CLK);
	 * writel(total transfer length, BCM2835_SPI_DLEN);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_clock_length);
	LINKTO(reset_spi_fifo);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK));
	FIXED(length,   8);
	VARYMSG(pad[0], clock_speed);
	/* the total DMA length we can only set on link time */
	IGNORE(pad[1]);

	/* configure and start spi - equivalent to:
	 * writel(spi_dev_data->spi_config,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_spi);
	LINKTO(config_clock_length);
	FIXED(ti,BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED(length,   4);
	SPI(pad[0],       spi_config);

	/* set up the tx-dma start address - equivalent to:
	 * writel(dma_address_of_tx_transfer,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(set_tx_dma_next);
	LINKTO(config_clock_length);
	FIXED(ti,BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,      BCM2835_DMA_ADDR);
	FIXED(length,   4);
	/* this is set later, when we know the dma_addr
	   of the TX-DMA-transfer */
	IGNORE(pad[0]);

	/* start the tx-dma - equivalent to:
	 * writel(BCM2835_DMA_CS_ACTIVE,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(start_tx_dma);
	LINKTO(config_clock_length);
	FIXED(ti,BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,      BCM2835_DMA_CS);
	FIXED(length,   4);
	FIXED(pad[0],     BCM2835_DMA_CS_ACTIVE);

	/* the Transfer portion is a bit tricky...
	 * so most of the varies will get managed by a specific transform
	 */

	/* and the tx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   writel(xfer->tx_buf[i],BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT(transfer.tx);
	/* we can not link here in a normal way
	 * but we have to assign it to pad0 instead...
	 */
	((struct bcm2835_dma_cb *)(frag->set_tx_dma_next->cb))
		->pad[0]=link->cb_dma;
	VARYXFER(ti,    xfer.tx_addr);
	VARYXFER(src,    xfer.tx_addr);
	FIXED(dst,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARYXFER(length,xfer.length);
	FIXED(pad[0],   0); /* in case we have no pointer, so use this */

	/* and the rx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   xfer->rx_buf[i] = readl(BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT(transfer.rx);
	LINKTO(start_tx_dma);
	VARYXFER(ti,xfer.rx_addr);
	FIXED(src,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARYXFER(dst,    xfer.rx_addr);
	VARYXFER(length,xfer.length);



	/* TODO: mark it as a single transfer */

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for deselecting cs
 *----------------------------------------------------------------------*/
static struct dma_fragment *bcm2835dma_spi_create_fragment_cs_deselect(
	struct device *device,gfp_t gfpflags)
{
	return NULL;
}

/*------------------------------------------------------------------------
 * allocator for adding some delay
 *----------------------------------------------------------------------*/
static struct dma_fragment *bcm2835dma_spi_create_fragment_delay(
	struct device *device,gfp_t gfpflags)
{
	return NULL;
}

/*------------------------------------------------------------------------
 * allocator for triggering an interrupt
 *----------------------------------------------------------------------*/
static struct dma_fragment *bcm2835dma_spi_create_fragment_trigger_irq(
	struct device *device,gfp_t gfpflags)
{
	return NULL;
}

/*************************************************************************
 * the release and initialization of dma_fragment caches
 * and function pointers
 ************************************************************************/
void bcm2835dma_release_dmafragment_components(
	struct spi_master* master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	if (!bs->pool)
		return;

	dma_fragment_cache_release(
		&bs->fragment_composite);
	dma_fragment_cache_release(
			&bs->fragment_setup_transfer);
	dma_fragment_cache_release(
			&bs->fragment_transfer);
	dma_fragment_cache_release(
			&bs->fragment_cs_deselect);
	dma_fragment_cache_release(
			&bs->fragment_delay);
	dma_fragment_cache_release(
			&bs->fragment_trigger_irq);

	dma_pool_destroy(bs->pool);
        bs->pool=NULL;
}

/* register all the stuff needed to control dmafragments
   note that the below requires that master has already been registered
   otherwise you get an oops...
 */
int bcm2835dma_register_dmafragment_components(
	struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int err;

	/* allocate pool - need to use pdev here */
	bs->pool=dma_pool_create(
                "DMA-CB-pool",
                &master->dev,
                sizeof(struct bcm2835_dma_cb),
                64,
                0
                );
	if (!bs->pool) {
		dev_err(&master->dev,
			"could not allocate DMA-memory pool\n");
		return -ENOMEM;
	}

	/* initialize DMA Fragment pools */
	err=dma_fragment_cache_initialize(
		&bs->fragment_composite,
		&master->dev,
		"merged_fragments",
		&bcm2835dma_spi_create_fragment_merged,
		3
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_setup_transfer,
		&master->dev,
		"setup_spi_plus_transfer",
		&bcm2835dma_spi_create_fragment_config_spi_transfer,
		6
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_transfer,
		&master->dev,
		"transfer",
		&bcm2835dma_spi_create_fragment_transfer,
		3
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_cs_deselect,
		&master->dev,
		"fragment_cs_deselect",
		&bcm2835dma_spi_create_fragment_cs_deselect,
		3
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_delay,
		&master->dev,
		"fragment_delay",
		&bcm2835dma_spi_create_fragment_delay,
		1
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_trigger_irq,
		&master->dev,
		"fragment_trigger_irq",
		&bcm2835dma_spi_create_fragment_trigger_irq,
		3
		);
	if (err)
		goto error;
#if 0
	/* and assign spi_dma_fragment_functions */
	bs->spi_dma_functions.fragment_composite_cache =
		&bs->fragment_composite;
	bs->spi_dma_functions.add_setup_spi_transfer =
		bcm2835dma_spi_compo_add_setup_transfer;
	bs->spi_dma_functions.add_transfer =
		bcm2835dma_spi_compo_add_transfer;
	bs->spi_dma_functions.add_cs_deselect =
		bcm2835dma_spi_compo_add_cs_deselect;
	bs->spi_dma_functions.add_delay =
		bcm2835dma_spi_compo_add_delay;
	bs->spi_dma_functions.add_trigger_interrupt =
		bcm2835dma_spi_compo_add_trigger_irq;
#endif

	return 0;
error:
	bcm2835dma_release_dmafragment_components(master);
	return err;
}

#if 0
/*************************************************************************
 * the fragments themselves
 ************************************************************************/

/*------------------------------------------------------------------------
 * some helper functions/macros
 *----------------------------------------------------------------------*/

/**
 * FREE_RMA_IN_FRAGMENT - helper for freeing the dma_link in the fragment
 * @frag: the dma_fragment to which this belongs
 * @field: the field which we want to get populated with the pointer
 */
#define FREE_DMA_IN_FRAGMENT(frag,field)   \
	if (frag->field)		     \
		dma_link_free(frag->field);

/**
 * BCM2835_DMA_CB_MEMBER_DMA_ADDR - for a member in the DMA CB create the
 *   corresponding dma-address
 * @dmalink: dma_link to base this on
 * @member: the member field in the dma CB
 */
#define BCM2835_DMA_CB_MEMBER_DMA_ADDR(dmalink,member)			\
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
 *     * _SPI(field): set during setup/optimization
 *         - value from spi_device
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
 * Note:
 *   that the macro assumes that the compiler will optimize dead code away
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
			dma_link_to_cb(link);				\
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
		block->next = 0;					\
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
			&(composite->					\
			composite.message_pre_transform_chain),		\
			&function,src,dst,extra,gfpflags);		\
	} else {							\
		err = function(src,dst,extra);				\
	}								\
	if (err)							\
		goto error;

#define VARY_HELPER_POST(flag,function,src,dst,extra)			\
	err = spi_message_transform_add(				\
		&(composite->composite.message_post_transform_chain),	\
		&function,src,dst,extra,gfpflags);			\
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
	struct spi_dma_fragment_composite *composite)
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

	dma_link_to_cb(last_link)->next =
		first_link->dmablock_dma;
}

struct dma_fragment_transfer;
static inline int bcm2835dma_spi_compo_add_transfer_helper(
	struct spi_message *mesg,
	struct spi_transfer *xfer,
	struct spi_dma_fragment_composite *composite,
	struct dma_fragment_transfer* frag,
	u32 vary,
	gfp_t gfpflags);

/*------------------------------------------------------------------------
 * message transforms
 *----------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------
 * dma_fragment_transfer
 *----------------------------------------------------------------------*/

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
	struct spi_dma_fragment_composite *composite,
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

/*------------------------------------------------------------------------
 * dma_fragment_setup_transfer
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_setup_transfer - structure used to set up SPI and initiate
 *   a transfer
 * @fragment: the normal dma_fragment
 * all others are dma_links for quicker connection of fragments later
 * note:
 * that we extend the simple dma_fragment_transfer with the same pattern
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
		_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),     0, /* spi_device_data->cs_select_gpio_reg*/
		_FIXED(LEN),   4,
		_SPI(PAD0),    0, /* spi_device_data->cs_bitfield */
		_IGNORE(PAD1), 0
		);

	/* now reset SPI FIFOS and configure SPI_SPEED */
	ADD_TO_DMA_FRAGMENT(
		frag,reset_spi,dma_rx,
		_FIXED(TI),    (BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_TDMODE
				| BCM2835_DMA_TI_S_INC
				| BCM2835_DMA_TI_D_INC),
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),   (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK),
		_FIXED(LEN),   4,
		_SPI(PAD0),    0, /* spi_device_data->spi_cs_reset */
		_IGNORE(PAD1), 0
		);
	link_dma_link(frag->cs_select,frag->reset_spi);

	/* set DMA transfer length and the clock speed in SPI */
	ADD_TO_DMA_FRAGMENT(
		frag,config_clock_length,dma_rx,
		_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),     (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK),
		_FIXED(LEN),   8,
		_VARY(PAD0),   0, /* SPI_SPEED as a divider */
		_VARY(PAD1),   0  /* SUM of transfer.length */
		);
	link_dma_link(frag->reset_spi,frag->config_clock_length);

	/* enable SPI+DMA */
	ADD_TO_DMA_FRAGMENT(
		frag,config_spi,dma_rx,
		_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),     (BCM2835_SPI_BASE_BUS
				+ BCM2835_SPI_CS),
		_FIXED(LEN),   4,
		_SPI(PAD0),    0, /* spi_device_data->spi_cs_set */
		_IGNORE(PAD1), 0
		);
	link_dma_link(frag->config_clock_length,frag->config_spi);

	/* configure the tx transfer itself */
	ADD_TO_DMA_FRAGMENT(
		frag,fragment.transfer_tx,dma_tx,
		_MESG(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_VARY(SRC),    0, /* transfer.tx_dma */
		_SPI(DST),     (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO),
		_VARY(LEN),    0, /* transfer.length */
		_IGNORE(PAD0), 0,
		_IGNORE(PAD1), 0
		);
	/* note that it gets linked below in set_tx_dma_next */

	/* prepare the tx-DMA - setting the next address from which
	   to load the DMA control block */
	ADD_TO_DMA_FRAGMENT(
		frag,set_tx_dma_next,dma_rx,
		_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),   (bs->dma_tx.bus_addr + BCM2835_DMA_ADDR),
		_FIXED(LEN),   4,
		_FIXED(PAD0),  (/* the dma address of the TX-transfer */
			frag->fragment.transfer_tx->dmablock_dma),
		_IGNORE(PAD1), 0
		);
	link_dma_link(frag->config_spi,frag->set_tx_dma_next);

	/* and start the tx-DMA */
	ADD_TO_DMA_FRAGMENT(
		frag,start_tx_dma,dma_rx,
		_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),   (bs->dma_tx.bus_addr + BCM2835_DMA_CS),
		_FIXED(LEN),   4,
		_FIXED(PAD0),  BCM2835_DMA_CS_ACTIVE,
		_IGNORE(PAD1), 0
		);
	link_dma_link(frag->config_spi,frag->set_tx_dma_next);

	/* configure the rx transfer itself */
	ADD_TO_DMA_FRAGMENT(
		frag,fragment.transfer_rx,dma_rx,
		_VARY(TI),     BCM2835_DMA_TI_WAIT_RESP,
		_SPI(SRC),     (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO),
		_SPI(DST),     0, /* transfer.rx_dma */
		_VARY(LEN),    0, /* transfer.length */
		_IGNORE(PAD0), 0,
		_IGNORE(PAD1), 0
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
		cdiv = DIV_ROUND_UP(clk_hz,spi_hz);
                /* as per documentation CDIV must be a power of two
		   but empirically NOTRO found that it is not needed,
		   so we do not add the following:
		   cdiv = roundup_pow_of_two(cdiv);
		*/
                if (cdiv >= 65536)
                        cdiv = 0; /* 0 is the slowest we can go */
        } else
                cdiv = 0; /* 0 is the slowest we can go */

	/* and set the clock divider */
	dma_link_to_cb(frag->config_clock_length)->pad[0] = cdiv;

	/* and now calculate the delay for a half clock cycle
	   - for now we assume that it is equal to clk_div */
	if (cdiv)
		frag->delay_transfers_half_cycle = cdiv;
	else
		frag->delay_transfers_half_cycle = 65535;

	return 0;
}


/**
 * message_transform_delay_us - calculate the value needed for delaying
 *    by a requested amount of usecs
 * @src: pointer to the value of delay_us
 *        (if zero, then delay by 0.5 clock)
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
	struct spi_dma_fragment_composite *composite,
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
	struct dma_fragment_setup_transfer* frag =
		(typeof(frag)) dma_fragment_cache_fetch(
			&bs->fragment_setup_transfer,
			gfpflags);
	if (!frag)
		return -ENOMEM;

	/* now start setting things up */
	/* the CS part */
	dma_link_to_cb(frag->cs_select)->dst =
		spi_device_data->cs_select_gpio_reg;
	dma_link_to_cb(frag->cs_select)->pad[0] =
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
	struct spi_dma_fragment_composite *composite,
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
		&dma_link_to_cb(
			((struct dma_fragment_setup_transfer *)
				(composite->last_setup_transfer))
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
		/* if we vary length then we he have to see what the
		   multiple is */
		if (vary | SPI_OPTIMIZE_VARY_LENGTH_MULTIPLE_4)
			composite->last_transfer =
				(struct dma_fragment_transfer *)frag;
		else
			composite->last_transfer = NULL;
	} else {
		composite->last_transfer =
			(xfer->len % 4) ? NULL :
			(struct dma_fragment_transfer *)frag;
	}

	return 0;
error:
	return err;
}

/*------------------------------------------------------------------------
 * dma_fragment_cs_deselect
 *----------------------------------------------------------------------*/

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
		_FIXED(TI),    (
			BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE
			),
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[1]),
		_VARY(LEN),    0, /* last_config_setup.delay_half_clock
				      or scaled transfer.delay_us */
		_IGNORE(PAD0), 0,
		_IGNORE(PAD1), 0
		);

	/* resetting CS */
	ADD_TO_DMA_FRAGMENT(
		frag,cs_deselect,dma_rx,
		_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_SPI(DST),     0, /* spi_device_data->cs_deselect_gpio_reg */
		_FIXED(LEN),   4,
		_SPI(PAD0),    0, /* spi_device_data->cs_bitfield */
		_IGNORE(PAD1), 0
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
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[1]),
		_SPI(LEN),     0, /* last_config_setup.delay_half_clock */
		_IGNORE(PAD0), 0,
		_IGNORE(PAD1), 0
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
	struct spi_dma_fragment_composite *composite,
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
		((struct dma_fragment_setup_transfer *)
			(composite->last_setup_transfer))
		->delay_transfers_half_cycle;

	/* we need to reset SPI after this */
	composite->last_transfer=NULL;

	/* link it to composite */
	link_to_composite((struct dma_fragment *)frag,composite);

	return 0;
error:
	return err;
}

/*------------------------------------------------------------------------
 * dma_fragment_delay
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_delay - structure used to delay the next transfer
 *    by some usecs
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
		_FIXED(TI),    (
			BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE
			),
		_FIXED(SRC),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(DST),   THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[1]),
		_VARY(LEN),    0, /* scaled transfer.delay_us */
		_IGNORE(PAD0), 0,
		_IGNORE(PAD1), 0
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
	struct spi_dma_fragment_composite *composite,
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
	link_to_composite((struct dma_fragment *)frag,composite);

	return 0;

error:
	return err;
}

/*------------------------------------------------------------------------
 * dma_fragment_trigger_irq
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_trigger_irq - structure used to trigger an IRQ
 *   on DMA completion
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
		_FIXED(TI),   (
			BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_INT_EN
			| BCM2835_DMA_TI_S_INC
			| BCM2835_DMA_TI_D_INC),
		_FIXED(SRC),  BCM2835_REG_COUNTER_64BIT_BUS,
		_FIXED(DST),  THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]),
		_FIXED(LEN),  8,
		/* the destination of the transfer
		   these need to get reset to 0
		*/
		_VARY(PAD0), 0,
		_VARY(PAD1), 0
		);

	/* prepare the tx-DMA - setting the next address from which
	   to load the DMA control block */
	ADD_TO_DMA_FRAGMENT(
		frag,set_tx_dma_next,dma_rx,
		_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
		_FIXED(SRC),   BCM2835_DMA_CB_MEMBER_DMA_ADDR(
			frag->set_tx_dma_next,pad[0]),
		_FIXED(DST),   (bs->dma_tx.bus_addr+BCM2835_DMA_ADDR),
		_FIXED(LEN),   4,
		_FIXED(PAD0),  BCM2835_DMA_CB_MEMBER_DMA_ADDR(
			frag->message_finished,ti),
		_IGNORE(PAD1), 0
		);

	/* and start the tx-DMA */
	ADD_TO_DMA_FRAGMENT(frag,start_tx_dma,dma_rx,
			_FIXED(TI),    BCM2835_DMA_TI_WAIT_RESP,
			_FIXED(SRC),   BCM2835_DMA_CB_MEMBER_DMA_ADDR(
				frag->start_tx_dma,pad[0]),
			_FIXED(DST),   (bs->dma_tx.bus_addr
					+BCM2835_DMA_CS),
			_FIXED(LEN),   4,
			_FIXED(PAD0),  BCM2835_DMA_CS_ACTIVE,
			_IGNORE(PAD1), 0
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
	struct spi_dma_fragment_composite *composite,
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
	link_to_composite((struct dma_fragment *)frag,composite);

	return 0;

error:
	return err;
}


/*************************************************************************
 * the release and initialization of dma_fragment caches
 * and function pointers
 ************************************************************************/
void bcm2835dma_release_dmafragment_components(
	struct spi_master* master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	if (!bs->pool)
		return;

	dma_fragment_cache_release(
		&bs->fragment_composite);
	dma_fragment_cache_release(
			&bs->fragment_setup_transfer);
	dma_fragment_cache_release(
			&bs->fragment_transfer);
	dma_fragment_cache_release(
			&bs->fragment_cs_deselect);
	dma_fragment_cache_release(
			&bs->fragment_delay);
	dma_fragment_cache_release(
			&bs->fragment_trigger_irq);

	dma_pool_destroy(bs->pool);
        bs->pool=NULL;

}

/* register all the stuff needed to control dmafragments
   note that the below requires that master has already been registered
   otherwise you get an oops...
 */
int bcm2835dma_register_dmafragment_components(
	struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int err;

	/* allocate pool - need to use pdev here */
	bs->pool=dma_pool_create(
                "DMA-CB-pool",
                &master->dev,
                sizeof(struct bcm2835_dma_cb),
                64,
                0
                );
	if (!bs->pool) {
		dev_err(&master->dev,
			"could not allocate DMA-memory pool\n");
		return -ENOMEM;
	}

	/* initialize DMA Fragment pools */
	err=dma_fragment_cache_initialize(
		&bs->fragment_composite,
		"composit fragments",
		&spi_dmafragment_create_composite,
		&master->dev,3
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_setup_transfer,
		"setup_spi_plus_transfer",
		&bcm2835_spi_dmafragment_create_setup_transfer,
		&master->dev,6
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_transfer,
		"transfer",
		&bcm2835_spi_dmafragment_create_transfer,
		&master->dev,3
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_cs_deselect,
		"fragment_cs_deselect",
		&bcm2835_spi_dmafragment_create_cs_deselect,
		&master->dev,3
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_delay,
		"fragment_delay",
		&bcm2835_spi_dmafragment_create_delay,
		&master->dev,1
		);
	if (err)
		goto error;
	dma_fragment_cache_initialize(
		&bs->fragment_trigger_irq,
		"fragment_trigger_irq",
		&bcm2835_spi_dmafragment_create_trigger_irq,
		&master->dev,3
		);
	if (err)
		goto error;

	/* and assign spi_dma_fragment_functions */
	bs->spi_dma_functions.fragment_composite_cache =
		&bs->fragment_composite;
	bs->spi_dma_functions.add_setup_spi_transfer =
		bcm2835dma_spi_compo_add_setup_transfer;
	bs->spi_dma_functions.add_transfer =
		bcm2835dma_spi_compo_add_transfer;
	bs->spi_dma_functions.add_cs_deselect =
		bcm2835dma_spi_compo_add_cs_deselect;
	bs->spi_dma_functions.add_delay =
		bcm2835dma_spi_compo_add_delay;
	bs->spi_dma_functions.add_trigger_interrupt =
		bcm2835dma_spi_compo_add_trigger_irq;

	return 0;
error:
	bcm2835dma_release_dmafragment_components(master);
	return err;
}


#endif
