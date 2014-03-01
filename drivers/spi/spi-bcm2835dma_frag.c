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

#define TODO(...)

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
 *   and variable definitions used by most allocate functions
 * note: no semicolon for the last define, as we run into compiler
 *   warnings otherwise when it sees ";;" and then some more variable
 *   definitions and then complains about "mixed declaration and code"
 * @struct_name: name of structure to allocate as frag
 */
#define START_CREATE_FRAGMENT_ALLOCATE(struct_name)			\
	struct spi_master * master = (struct spi_master *)device;	\
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);	\
	struct dma_pool *pool = bs->pool;				\
	struct dma_link *link;						\
	struct bcm2835_dma_cb *cb;					\
	struct struct_name *frag =					\
		(struct struct_name *) dma_fragment_alloc(		\
			device,						\
			sizeof(struct struct_name),			\
			gfpflags);					\
	if (! frag)							\
		return NULL;						\
	((struct dma_fragment *)frag)->desc = #struct_name;

/**
 * END_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 * used by all alloc functions on exit of function - including error
 * handling.
 */

#define END_CREATE_FRAGMENT_ALLOCATE()			\
	dma_fragment_set_default_links(			\
		(struct dma_fragment*)frag);		\
	return (struct dma_fragment*)frag;		\
error:							\
        dma_fragment_free((struct dma_fragment*)frag);	\
	return NULL;

/**
 * ADD_DMA_LINK_TO_FRAGMENT - macro that allocates a new dma_link
 *  and adds it to the member field in the structure
 *  it also does some basic linking and sets up some fields with defaults
 * @field: the field to which to assign the dma_link to
 */
#define ADD_DMA_LINK_TO_FRAGMENT(field)					\
	link = frag->field = dma_link_alloc(				\
		pool,							\
		sizeof(*frag->field),					\
		gfpflags						\
		);							\
	if (!link)							\
		goto error;						\
	link->desc = #field;						\
	dma_fragment_add_dma_link(					\
		(struct dma_fragment *)frag,				\
		(struct dma_link *)frag->field				\
		);							\
	cb = (struct bcm2835_dma_cb *)link->cb;				\
	cb->next=0;							\
	cb->stride=0;

/**
 * LINK_TO - macro that links the dma_link pointed to by field in the
 * custom dma_fragment structure to the currently active dma_link
 * @field: the field name
 */
#define LINKTO(field)					   \
	((struct bcm2835_dma_cb *)frag->field->cb)->next = \
		link->cb_dma;

/*------------------------------------------------------------------------
 * helpers and macros to to make the basic "setup" of dma_fragments
 * easier to read
 *----------------------------------------------------------------------*/

/**
 * SCHEDULE_LINKTIME_TRANSFORM - schedules a transform during link time
 * @function: function to schedule
 * @src: values to set in transform
 * @dst: values to set in transform
 * @extra: values to set in transform
 */
#define SCHEDULE_LINKTIME_TRANSFORM(function,src,dst,extra)	 \
	if (! dma_fragment_addnew_transform(			 \
			&function,				 \
			&frag->fragment,			 \
			src,dst,extra,				 \
			0,gfpflags))				 \
		goto error;

/**
 * SCHEDULE_LINKTIME_VARY_TRANSFORM - schedules a transform during link
 * time which may further defer to pre-dma time if VARY is given.
 * @function: function to schedule - needs to get created via the
 *   VARY_LINK_TRANSFORM_HELPER macro.
 * @extra: values to set in transform
 * Note:
 * the function called have the assumption that src,dst contain
 * spi_message and spi_transfer respectively.
 */
#define SCHEDULE_LINKTIME_VARY_TRANSFORM(function,extra)	 \
	SCHEDULE_LINKTIME_TRANSFORM(function,NULL,NULL,extra);	 \

/**
 * FIXED - macro that defines the field as one that can get assigned with
 * a static value when creating the fragment.
 * this is guaranteed to never get modified
 * @field: the field in the dma-controlblock
 * @value: the value to assign
 */
#define FIXED(field,value) cb->field = value;

/**
 * VARY_LINK_TRANSFORM_HELPER - macro that depending on the vary bitflags
 * either schedules the transfer to get done on DMA scheduling
 * or getting executed immediately during link time (possibly inlined)
 * @varyflags: the flags on which we need to vary
 * @functionname: the function to call either now or later
 *   this function is actually of a different type than the one used
 *   with dma_fragment_transform so that we see a warning about a
 *   changed interface requirement (see below the notes on src and dst)
 * @suffix: the suffix for the created function
 * Note:
 * this helper requres that it is called ONLY in link context
 * it also assumes that transform->src contains the spi_message
 * and transform->dst contains the spi_transfer to process
 * this is needed to map the functions during link time as well...
 */
#define VARY_LINK_TRANSFORM_HELPER(varyflags,functionname,suffix)	\
	static int _ ## functionname ## suffix (			\
		struct dma_fragment_transform *transform,		\
		void *vp,						\
		gfp_t gfpflags)						\
	{								\
	return functionname(						\
		transform->fragment,					\
		transform->src,						\
		transform->dst,						\
		transform->extra,					\
		(struct bcm2835dma_spi_merged_dma_fragment *)vp,	\
		gfpflags						\
		);							\
	}								\
	static inline int functionname ## suffix (			\
		struct dma_fragment_transform *transform,		\
		void *vp,						\
		gfp_t gfpflags)						\
	{								\
		struct dma_fragment *frag  = transform->fragment;	\
		struct spi_merged_dma_fragment *merged =		\
			(typeof(merged))vp;				\
		struct spi_message  *mesg  = merged->message;		\
		struct spi_transfer *xfer  = merged->transfer;		\
		void                *extra = transform->extra;		\
		int                  vary  = 0; /*xfer->vary*/		\
		if ( (!varyflags) || (vary & varyflags)	) {		\
			if (! spi_merged_dma_fragment_addnew_transform(	\
					merged,				\
					& _ ## functionname ## suffix,	\
					frag, mesg, xfer, extra,	\
					0, 0, gfpflags) )		\
				return -EPERM;				\
			return 0;					\
		} else {						\
			/* call the function directly			\
			 * - hopefully it is inlined... */		\
			return functionname(				\
				frag, mesg, xfer, extra,vp, gfpflags);	\
		}							\
	}

/**
 * bcm2835dma_fragment_transform_spi_data_offset - transform function
 *   to fetch a u32 value from the spi_device specific data and copy it to
 *   the requested destination
 * @transform: the transform that contains all additional parameters
 * @transform_src: the offset in bytes from the start of
 *    spi_device_driver_data
 * @transform_dst: the (u32 *)pointer to which to assigne the value of src
 * @fragment: the fragment to which this transform belongs
 * @vp: the fragment into which this fragment is getting merged
 * Implicit assumption that we run during link time, so vp contains valid
 * message and transfer....
 */
static int bcm2835dma_fragment_transform_spi_data_offset(
	struct dma_fragment_transform * transform,
	void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment from the extra void pointer argument */
	struct spi_merged_dma_fragment *merged_frag =
		(typeof(merged_frag)) vp;

	/* the spi device */
	struct spi_device *spi = merged_frag->message->spi;

	/* pointer to spi_device_data as a char pointer*/
	char *base=dev_get_drvdata(&spi->dev);
	base += (u32)transform->src;

	/* copy the value */
	*((u32*)transform->dst) = *((u32*)(base));

	return 0;
}

/**
 * SPI - macro that copies some value from the spi_device_data structure
 * to the field in the dma_controlblock
 * @field: the field name in the dma controlblock
 * @spi_data_field: the field in the spi_device_data structure
 *   this get executed during link time
 * Note: this is a bit inefficient, as it requires more calls/entries
 *   than necessary when we need to update multiple fields.
 *   in this case a "common" function works best...
 */
#define SPI(field,spi_data_field)					\
	if (! dma_fragment_addnew_transform(				\
			&bcm2835dma_fragment_transform_spi_data_offset,	\
			&frag->fragment,				\
			(void*)offsetof(\
				struct bcm2835dma_spi_device_data,	\
				spi_data_field),			\
			&((struct bcm2835_dma_cb*)link->cb)->field,	\
			NULL,0,gfpflags)				\
		)							\
		goto error;


/**
 * TXDMA - macro that assignes the DMA BASE_REGISTER of the transmit DMA
 *  + offset to the specific dma_controlblock field.
 * @field: the field name in the dma controlblock
 * @offset: the offset of the register we need to access
 */
#define TXDMA(field,offset) cb->field = bs->dma_tx.bus_addr+offset

/**
 * VARY - macro that is just there to document that this value is set via
 * a dma_fragment_transform - for optimized spi_messages this also takes
 * xfer->vary into account as to when to execute the function.
 * this is only here to document what needs to get done
 * @field: the field name in the dma controlblock
 * @function: the function name that is repsonsible for setting this
 */
#define VARY(field,function,...)

/**
 * LATER - macro that is there just to document that we are setting
 * the field a bit later in the sequence
 * @field: the field name in the dma controlblock
 */
#define LATER(field,...)

/*------------------------------------------------------------------------
 * general remark on dma_fragment_transforms
 *
 * the transforms are applied to the individual dma_chains when linking
 * of a dma_fragment to the bcm2835dma_spi_merged_dma_fragment happens.
 *
 * the extra data argument to the transform function, is used to give
 * a pointer to the merged fragment to which this fragment gets linked.
 * this allows for scheduling additional transforms in the merged fragment
 *
 * for the merged fragment its transform are called:
 * * when unlinking the fragments (i.e. releasing the individual fragments
 *   back to cache)
 * * there are also special pre/post DMA queues that are executed before
 *   /after a DMA occurs.
 *----------------------------------------------------------------------*/

/*------------------------------------------------------------------------
 * allocator for setting up spi
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_config_spi - the dma_fragment structure needed to configure
 *  spi and select Chip Select
 * @fragment: the main embedded dma_fragment structure
 * @cs_select: the dma_link that selects CS
 * @reset_spi_fifo: the dma_link responsible for resetting the spi fifo buffers
 * @config_clock_length: the dma_link responsible for setting the
 *    SPI clock divider and configure the number of bytes to transfer
 * @config_spi: the dma_link responsible for configuring the spi device
 *    to start DMA processing
 * @set_tx_dma_next: the dma_link responsible for setting the next address
 *    from which the TX-DMA will restart
 * @start_tx_dma: the dma_link responsible for starting the tx-dma
 */
struct dma_fragment_config_spi {
	struct dma_fragment fragment;
	/* additional data */
	struct dma_link     *cs_select;
	struct dma_link     *reset_spi_fifo;
	struct dma_link     *config_clock_length;
	struct dma_link     *config_spi;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
};

/**
 * bcm2835dma_fragment_transform_speed_hz - the dma_fragment_transfor that
 *  will calculate the spi clock-divider as well as the DMA loop length
 *  required for a half clock cycle
 * @transform: the transform that contains all additional parameters
 * @vp: the fragment into which this fragment is getting merged
 * @gfpflags: gfp flags used to allocate memory
 * Note:
 *   vp is a pointer to bcm2835dma_spi_merged_dma_fragment and contains:
 *   a pointer to spi_message and spi_dev
 *   as well as some fields that contain the clock divider and delay count
 *   it also sets fragment.config_clock_length.pad[0] to the clock-divider
 *
 * depending on vary flags in spi_transfer this may get executed during
 * link time or prior to executing the dma (in the optimized case)
 */
static inline int bcm2835dma_fragment_transform_speed_hz(
	struct dma_fragment *fragment,
	struct spi_message  *mesg,
	struct spi_transfer *xfer,
	void                *extra,
	struct bcm2835dma_spi_merged_dma_fragment *vp,
	gfp_t gfpflags)
{
	/* the info about the current message and transfer */
	struct spi_master *master=mesg->spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	u32 spi_hz = xfer->speed_hz;
	u32 clk_hz = clk_get_rate(bs->clk);

	/* get the config_clock dma_link controlblock */
	struct dma_fragment_config_spi *setup =
		(typeof(setup)) fragment;
	struct bcm2835_dma_cb *cb = setup->config_clock_length->cb;

	/* transform to what we need */
	u32 cdiv;

	/* set the current speed */
	vp->speed_hz = spi_hz;

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
	vp->speed_cdiv = cdiv;
	cb->pad[0] = cdiv;

	/* and now calculate the delay for a half clock cycle
	   - for now we assume that it is equal to clk_div */
	if (cdiv)
		vp->delay_half_cycle_dma_length  = cdiv;
	else
		vp->delay_half_cycle_dma_length  = 65535;

	return 0;
}
VARY_LINK_TRANSFORM_HELPER(SPI_OPTIMIZE_VARY_SPEED_HZ,
		bcm2835dma_fragment_transform_speed_hz,
		_vary);

/**
 * bcm2835dma_fragment_transform_prepare_txlink - transfor that
 *   will initialize bcm2835dma_spi_merged_dma_fragment.txdma_link_to_here
 *   to the correct value.
 * @transform: the transform that contains all additional parameters
 * @vp: the fragment into which this fragment is getting merged
 * @gfpflags: gfp flags used to allocate memory
 * Implicit assumption that we run during link time, so vp contains valid
 * message and transfer....
 */
static inline int bcm2835dma_fragment_transform_prepare_txlink(
	struct dma_fragment_transform *transform,
	void *vp, gfp_t gfpflags)
{
	/* the merged fragment */
	struct bcm2835dma_spi_merged_dma_fragment *merged_frag =
		(typeof(merged_frag)) vp;

	/* assign the value of src/dst */
	merged_frag->txdma_link_to_here = transform->src;
	merged_frag->total_length = transform->dst;

	/* and set total length to 0 */
	*merged_frag->total_length = 0;

	/* and return OK */
	return 0;
}

/**
 *  bcm2835dma_spi_create_fragment_config_spi - allocate and set up the
 *   dma_fragment to configure the SPI device
 * @device: the spi_master.device for  which we set this up
 * @gpfflags: the gpf_t flags to use when allocating memory
 */
struct dma_fragment *bcm2835dma_spi_create_fragment_config_spi(
	struct device *device,gfp_t gfpflags)
{
	u32 *total_length_ptr;
	u32 *txdma_link_to_here;
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_config_spi);

	/* before we do any of this we need to
	 * schedule a cdiv calculation */
	/* schedule the SPI divider calculation */
	SCHEDULE_LINKTIME_VARY_TRANSFORM(
		bcm2835dma_fragment_transform_speed_hz_vary,
		NULL);

	/* select chipselect - equivalent to:
	   writel(spi_dev_data->cs_bitfield,
  	          spi_dev_data->cs_select_gpio_reg);
	*/
	ADD_DMA_LINK_TO_FRAGMENT(cs_select);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	SPI  (dst,    cs_select_gpio_reg);
	FIXED(length, 4);
	SPI  (pad[0], cs_bitfield);

	/* reset SPI fifos - equivalent to:
	 * writel(spi_dev_data->spi_reset_fifo,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(reset_spi_fifo);
	LINKTO(cs_select);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED(length, 4);
	SPI(pad[0],   spi_reset_fifo);

	/* configure clock divider and transfer length  - equivalent to:
	 * writel(cdiv, BCM2835_SPI_CLK);
	 * writel(total transfer length, BCM2835_SPI_DLEN);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_clock_length);
	LINKTO(reset_spi_fifo);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK));
	FIXED(length, 8);
	VARY (pad[0], bcm2835dma_fragment_transform_speed_hz);
	LATER (pad[1], bcm2835dma_fragment_transform_prepare_txlink
		/* set to 0 during link time */);
	total_length_ptr = &cb->pad[1];

	/* configure and start spi - equivalent to:
	 * writel(spi_dev_data->spi_config,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_spi);
	LINKTO(config_clock_length);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED(length, 4);
	SPI  (pad[0], spi_config);

	/* set up the tx-dma start address - equivalent to:
	 * writel(dma_address_of_tx_transfer,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(set_tx_dma_next);
	LINKTO(config_spi);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,    BCM2835_DMA_ADDR);
	FIXED(length, 4);
	LATER(pad[0], bcm2835dma_fragment_transform_prepare_txlink
		/* this is set later, when we know the dma_addr
		   of the TX-DMA-transfer */);
	txdma_link_to_here = &cb->pad[0];

	/* start the tx-dma - equivalent to:
	 * writel(BCM2835_DMA_CS_ACTIVE,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(start_tx_dma);
	LINKTO(set_tx_dma_next);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,    BCM2835_DMA_CS);
	FIXED(length, 4);
	FIXED(pad[0], BCM2835_DMA_CS_ACTIVE);

	/* schedule link time handling of settings */
	SCHEDULE_LINKTIME_TRANSFORM(
		bcm2835dma_fragment_transform_prepare_txlink,
		txdma_link_to_here,
		total_length_ptr,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for transfers
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_config_spi - the dma_fragment structure to schedule a
 *   single spi_transfer
 * @fragment: the main embedded dma_fragment structure
 * @xfer_rx: the dma_link responsible for receiving data from SPI
 * @xfer_tx: the dma_link responsible for transmitting data over SPI
 */

struct dma_fragment_transfer {
	struct dma_fragment fragment;
	struct dma_link     *xfer_rx;
	struct dma_link     *xfer_tx;
};

/**
 * bcm2835dma_fragment_transform_length - transform which sets
 *   length correctly
 * @transform: the transform that contains all additional parameters
 *  @src: spi_message to handle - primarily needed during vary calls
 *  @dst: spi_transfer to handle - primarily needed during vary calls
 * @vp: the fragment into which this fragment is getting merged
 * @gfpflags: the GFP flags used for allocation (if needed)
 */
static inline int bcm2835dma_fragment_transform_length(
	struct dma_fragment *fragment,
	struct spi_message  *mesg,
	struct spi_transfer *xfer,
	void                *extra,
	struct bcm2835dma_spi_merged_dma_fragment *merged,
	gfp_t gfpflags)
{
	/* the fragments real type */
	struct dma_fragment_transfer *frag = (typeof(frag)) fragment;
	/* the transfer links */
	struct dma_link *link_tx = frag->xfer_tx;
	struct dma_link *link_rx = frag->xfer_rx;
	/* and the controlblocks */
	struct bcm2835_dma_cb *cb_tx =
		(struct bcm2835_dma_cb *)link_tx->cb;
	struct bcm2835_dma_cb *cb_rx =
		(struct bcm2835_dma_cb *)link_rx->cb;

	/* add set length of dma transfers */
	cb_tx->length = xfer->len;
	cb_rx->length = xfer->len;
	/* and add to total */
	*merged->total_length += xfer->len;

	/* if it is not a multiple of 4,
	 * then we need to schedule new setup_spi */
	if (xfer->len & 3)
		merged->spi_fragment.last_transfer = NULL;

	/* we should also check for possible length */
	if (*merged->total_length > 65535)
		return -EINVAL;

	return 0;
}
VARY_LINK_TRANSFORM_HELPER(SPI_OPTIMIZE_VARY_LENGTH,
		bcm2835dma_fragment_transform_length,
		_vary);

/**
 * bcm2835dma_fragment_transform_length - transform which sets
 *   length correctly
 * @transform: the transform that contains all additional parameters
 *  @src: spi_message to handle - primarily needed during vary calls
 *  @dst: spi_transfer to handle - primarily needed during vary calls
 * @vp: the fragment into which this fragment is getting merged
 * @gfpflags: the GFP flags used for allocation (if needed)
 */
static inline int bcm2835dma_fragment_transform_buffer(
	struct dma_fragment *fragment,
	struct spi_message  *mesg,
	struct spi_transfer *xfer,
	void                *extra,
	struct bcm2835dma_spi_merged_dma_fragment *merged,
	gfp_t gfpflags)
{
	/* the fragments real type */
	struct dma_fragment_transfer *frag = (typeof(frag)) fragment;

	/* the transfer links */
	struct dma_link *link_tx = frag->xfer_tx;
	struct dma_link *link_rx = frag->xfer_rx;

	/* and the controlblocks */
	struct bcm2835_dma_cb *cb_tx =
		(struct bcm2835_dma_cb *)link_tx->cb;
	struct bcm2835_dma_cb *cb_rx =
		(struct bcm2835_dma_cb *)link_rx->cb;

	/* copy rx/tx_dma */
	cb_rx->dst = xfer->rx_dma;
	cb_tx->src = xfer->tx_dma;

	/* and also keep rx/tx_buf as a copy for debugging */
	cb_rx->pad[0]=(u32)xfer->rx_buf;
	cb_tx->pad[0]=(u32)xfer->tx_buf;

	/* and set the corresponding values for dma sources
	 * this is especially important for values that may be 0
	 */
	if (xfer->rx_buf) {
		cb_rx->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_D_INC;
	} else {
		cb_rx->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_D_IGNORE;
	}
	if (xfer->tx_buf) {
		cb_tx->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_D_INC;
	} else {
		cb_tx->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE;
	}

	return 0;
}

VARY_LINK_TRANSFORM_HELPER(
	(SPI_OPTIMIZE_VARY_RX_BUF|SPI_OPTIMIZE_VARY_TX_BUF),
	bcm2835dma_fragment_transform_buffer,
	_vary);

/**
 * bcm2835dma_fragment_transform_dmamap - transform which sets
 *   the dma_mapping
 * @transform: the transform that contains all additional parameters
 *  @src: spi_message to handle - primarily needed during vary calls
 *  @dst: spi_transfer to handle - primarily needed during vary calls
 * @vp: the fragment into which this fragment is getting merged
 * @gfpflags: the GFP flags used for allocation (if needed)
 */
static inline int bcm2835dma_fragment_transform_dmamap(
	struct dma_fragment *fragment,
	struct spi_message  *mesg,
	struct spi_transfer *xfer,
	void                *extra,
	struct bcm2835dma_spi_merged_dma_fragment *merged,
	gfp_t gfpflags)
{
	if (xfer->tx_buf) {
		xfer->tx_dma = dma_map_single_attrs(
			&mesg->spi->master->dev,
			(void *)xfer->tx_buf,
			xfer->len,
			DMA_TO_DEVICE,
			NULL);
	} else {
		xfer->tx_dma = 0;
	}

	if (xfer->rx_buf) {
		xfer->rx_dma = dma_map_single_attrs(
			&mesg->spi->master->dev,
			(void *)xfer->rx_buf,
			xfer->len,
			DMA_FROM_DEVICE,
			NULL);
	} else {
		xfer->rx_dma = 0;
	}

	/* and always call the buffer_set functions */
	return bcm2835dma_fragment_transform_buffer(
		fragment,mesg,xfer,extra,merged,gfpflags);
}
VARY_LINK_TRANSFORM_HELPER(
	(SPI_OPTIMIZE_VARY_RX_BUF|SPI_OPTIMIZE_VARY_TX_BUF),
	bcm2835dma_fragment_transform_dmamap,
	_vary);

/**
 * bcm2835dma_fragment_transform_dmaunmap - transform which sets
 *   the dma_mapping
 * @transform: the transform that contains all additional parameters
 *  @src: spi_message to handle - primarily needed during vary calls
 *  @dst: spi_transfer to handle - primarily needed during vary calls
 *  @extra: device used during dmamap
 * @vp: the fragment into which this fragment is getting merged
 * @gfpflags: the GFP flags used for allocation (if needed)
 */
static inline int bcm2835dma_fragment_transform_dmaunmap(
	struct dma_fragment_transform * transform,
	void *vp,
	gfp_t gfpflags)
{
	/* the fragment's real type */
	struct dma_fragment_transfer *frag =
		(typeof(frag)) transform->fragment;

	/* the transfer links */
	struct dma_link *link_tx = frag->xfer_tx;
	struct dma_link *link_rx = frag->xfer_rx;

	/* and the controlblocks */
	struct bcm2835_dma_cb *cb_tx =
		(struct bcm2835_dma_cb *)link_tx->cb;
	struct bcm2835_dma_cb *cb_rx =
		(struct bcm2835_dma_cb *)link_rx->cb;

	/* get the real values */
	u32           length = cb_tx->length;
	dma_addr_t    tx_dma = cb_tx->src;
	dma_addr_t    rx_dma = cb_rx->dst;

	/* and get the device */
	struct device *dev = transform->src;

	/* now do the conditional unmap if it is not 0 */
	if (tx_dma)
                dma_unmap_single_attrs(
                        dev,
                        tx_dma,
                        length,
                        DMA_TO_DEVICE,
                        NULL);

	if (rx_dma)
                dma_unmap_single_attrs(
                        dev,
                        rx_dma,
                        length,
                        DMA_FROM_DEVICE,
                        NULL);
	return 0;
}

/**
 * bcm2835dma_fragment_transform_linktx - dma_fragment_transform which
 *  links the current transfer to the previous one
 * @transform: the transform that contains all additional parameters
 * @vp: the fragment into which this fragment is getting merged
 * @gfpflags: the GFP flags used for allocation (if needed)
 */
static int bcm2835dma_fragment_transform_linktx(
	struct dma_fragment_transform * transform,
	void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment */
	struct bcm2835dma_spi_merged_dma_fragment *merged_frag =
		(typeof(merged_frag)) vp;

	/* the dma_fragment_transfer to chain */
	struct dma_fragment_transfer *frag =
		(typeof(frag)) transform->fragment;
	/* the tx-transfer link itself */
	struct dma_link *link_tx = frag->xfer_tx;
	/* and the controlblock */
	struct bcm2835_dma_cb *cb_tx =
		(struct bcm2835_dma_cb *)link_tx->cb;

	/* now link the tx dma */
	*merged_frag->txdma_link_to_here = link_tx->cb_dma;
	merged_frag->txdma_link_to_here = &cb_tx->next;
	cb_tx->next=0;

	/* for the below we need the transfer in src,
	   as this information is not available during vary time */
	transform->src = merged_frag->spi_fragment.message;
	transform->dst = merged_frag->spi_fragment.transfer;

	/* and process length - possibly varied */
	if (bcm2835dma_fragment_transform_length_vary(
			transform,vp,gfpflags))
		return 1;
	/* do we need to DMA-map? */
	if (!merged_frag->spi_fragment.message->is_dma_mapped) {
		/* in principle this means varying the whole thing
		 * so we need to schedule it really for PRE/POST
		 * immediately
		 */
		if (bcm2835dma_fragment_transform_dmamap_vary(
				transform,vp,gfpflags))
			return -ENOMEM;
		/* and schedule unmap as a post-dma step */
		if (! spi_merged_dma_fragment_addnew_transform(
				(struct spi_merged_dma_fragment *) vp,
				&bcm2835dma_fragment_transform_dmaunmap,
				transform->fragment,
				merged_frag->spi_fragment.message,
				merged_frag->spi_fragment.transfer,
				NULL,0,1,gfpflags) )
			return -ENOMEM;
 	} else {
		if (bcm2835dma_fragment_transform_buffer_vary(
				transform,vp,gfpflags))
			return -ENOMEM;
	}

	/* and return without any issues */
	return 0;
}

/**
 * bcm2835dma_spi_create_fragment_transfer - allocate and setup the
 *  dma_fragment to configure the DMA transfer
 * @device: the spi_master.device for  which we set this up
 * @gpfflags: the gpf_t flags to use when allocating memory
 */
static struct dma_fragment *bcm2835dma_spi_create_fragment_transfer(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_transfer);

	/* the tx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   writel(xfer->tx_buf[i],BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT(xfer_tx);
	VARY (ti,     bcm2835dma_fragment_transform_linktx,xfer.tx_addr);
	VARY (src,    bcm2835dma_fragment_transform_linktx,xfer.tx_addr);
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARY (length, bcm2835dma_fragment_transform_copyadd_length,
		xfer.length);
	FIXED(pad[0], 0); /* in case we have no pointer, so use this */

	/* the rx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   xfer->rx_buf[i] = readl(BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT(xfer_rx);
	VARY (ti,     bcm2835dma_fragment_transform_linktx,xfer.rx_addr);
	FIXED(src,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARY (dst,    bcm2835dma_fragment_transform_linktx,xfer.rx_addr);
	VARY (length, bcm2835dma_fragment_transform_copyadd_length,
		xfer.length);

	/* we also need to link the tx_channel to the previous, but as
	   this only happens during dma-fragment linking, we need to
	   schedule it...
	 */
	SCHEDULE_LINKTIME_TRANSFORM(
			bcm2835dma_fragment_transform_linktx,
			NULL,NULL,NULL);

	/* need to add special linktos for the case where we can not
	   have start and end being in the same dma channel
	   (relevant for transfers with only 2 DMA CBs on 2 channels)
	   in this case we link the RX DMA so we can only use the
	   rx transfer
	 */
	frag->fragment.link_head = frag->xfer_rx;
	frag->fragment.link_tail = frag->xfer_rx;

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for deselecting cs
 *----------------------------------------------------------------------*/
struct dma_fragment_cs_deselect {
	struct dma_fragment fragment;
	struct dma_link     *delay_pre;
	struct dma_link     *cs_deselect;
	struct dma_link     *delay_post;
};

static inline int bcm2835dma_fragment_transform_set_cs_delaylength(
	struct dma_fragment *fragment,
	struct spi_message  *mesg,
	struct spi_transfer *xfer,
	void                *extra,
	struct bcm2835dma_spi_merged_dma_fragment *merged,
	gfp_t gfpflags)
{
	struct bcm2835_dma_cb *cb;
	/* the fragments real type */
	struct dma_fragment_cs_deselect *frag = (typeof(frag)) fragment;

	/* and the half cycle loop length */
	u32 delay = merged -> delay_half_cycle_dma_length;


	/* first the pre delay */
	cb = (struct bcm2835_dma_cb *)frag->delay_pre->cb;
	cb->length = delay + xfer->delay_usecs * delay_1us;

	/* and the post delay */
	cb = (struct bcm2835_dma_cb *)frag->delay_post->cb;
	cb->length = delay;

	return 0;
}

VARY_LINK_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_DELAY_USECS
				| SPI_OPTIMIZE_VARY_SPEED_HZ),
		bcm2835dma_fragment_transform_set_cs_delaylength,_vary);

static struct dma_fragment *bcm2835dma_spi_create_fragment_cs_deselect(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_cs_deselect);

	/* delay by half a clock cycle or by the delay given in xfer
	 * equivalent to: udelay(max(xfer->delay_usecs,
	 *   500000/xfer->clock_frequency)))
	 */
	ADD_DMA_LINK_TO_FRAGMENT(delay_pre);
	FIXED(ti,     ( BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE));
	FIXED(src,    0);
	FIXED(dst,    0);
	VARY (length, bcm2835dma_fragment_transform_set_delaylength,
		xfer->delay_usec);

	/* deselect chipselect - equivalent to:
	 * writel(spi_dev_data->cs_bitfield,
	 *        spi_dev_data->cs_deselect_gpio_reg);
	*/
	ADD_DMA_LINK_TO_FRAGMENT(cs_deselect);
	LINKTO(delay_pre);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	SPI  (dst,      cs_deselect_gpio_reg);
	FIXED(length,   4);
	SPI  (pad[0],   cs_bitfield);

	/* delay by half a clock cycle
	 * equivalent to: udelay(500000/xfer->clock_frequency)
	 */
	ADD_DMA_LINK_TO_FRAGMENT(delay_post);
	LINKTO(cs_deselect);
	FIXED(ti,     ( BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE));
	FIXED(src,    0);
	FIXED(dst,    0);
	VARY (length, bcm2835dma_fragment_transform_set_delaylength,
		half_clock_cycle);

	/* schedule the vary transform for link-time
	 * assigning the half clock cycle delay to pre and post
	 * and add a xfer.delay_usec if set
	 */
	SCHEDULE_LINKTIME_VARY_TRANSFORM(
		bcm2835dma_fragment_transform_set_cs_delaylength_vary,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for adding some delay
 *----------------------------------------------------------------------*/
struct dma_fragment_delay {
	struct dma_fragment fragment;
	struct dma_link     *delay;
};

static inline int bcm2835dma_fragment_transform_set_delaylength(
	struct dma_fragment *fragment,
	struct spi_message  *mesg,
	struct spi_transfer *xfer,
	void                *extra,
	struct bcm2835dma_spi_merged_dma_fragment *merged,
	gfp_t gfpflags)
{

	/* the fragments real type */
	struct dma_fragment_delay *frag = (typeof(frag)) fragment;

	/* set delay */
	struct bcm2835_dma_cb *cb =
		(struct bcm2835_dma_cb *)frag->delay->cb;
	cb->length = merged -> delay_half_cycle_dma_length;

	return 0;
}
VARY_LINK_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_DELAY_USECS),
		bcm2835dma_fragment_transform_set_delaylength,
		_vary);

static struct dma_fragment *bcm2835dma_spi_create_fragment_delay(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_delay);

	/* delay by the delay given in xfer
	 * equivalent to: udelay(xfer->delay_usecs);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(delay);
	FIXED(ti,     ( BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE));
	FIXED(src,    0);
	FIXED(dst,    0);
	VARY (length, bcm2835dma_fragment_set_delaylength,
		xfer->delay_usec);

	/* schedule the vary transform for link-time
	   onyl set the xfer.delay_usec to the delay length
	 */
	SCHEDULE_LINKTIME_VARY_TRANSFORM(
		bcm2835dma_fragment_transform_set_delaylength_vary,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for triggering an interrupt
 *----------------------------------------------------------------------*/
struct dma_fragment_trigger_irq {
	struct dma_fragment fragment;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *message_finished;
	struct dma_link     *start_tx_dma;
};

static int spi_merged_dma_fragment_call_complete(
	struct dma_fragment_transform *transform,
	void *vp, gfp_t gfpflags)
{
	struct spi_merged_dma_fragment *merged = (typeof(merged)) vp;
	struct spi_message *mesg = merged->message;
	mesg->complete(mesg->context);
	return 0;
}

static inline int spi_merged_dma_fragment_complete(
	struct dma_fragment_transform *transform,
	void *vp, gfp_t gfpflags)
{
	/* the merged fragment */
	struct spi_merged_dma_fragment *merged = (typeof(merged)) vp;
	/* check for callback in mesg */
	if ( merged->message->complete)
		/* schedule post-dma callback */
		if (! spi_merged_dma_fragment_addnew_transform(
				vp,
				&spi_merged_dma_fragment_call_complete,
				transform->fragment,
				NULL,NULL,NULL,
				0,1,gfpflags) )
			return -ENOMEM;
		return 0;
}


static struct dma_fragment *bcm2835dma_spi_create_fragment_trigger_irq(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_trigger_irq);
	/* set up the tx-dma start address - equivalent to:
	 * writel(dma_address_of_tx_transfer,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(set_tx_dma_next);
	FIXED(ti,BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,      BCM2835_DMA_ADDR);
	FIXED(length,   4);
	LATER(pad[0],  /* this is set later, when we know the dma_addr
			   of the TX-DMA-transfer */);

	/* copy the timestamp from the counter to a fixed address */
	ADD_DMA_LINK_TO_FRAGMENT(message_finished);
	FIXED(ti,       ( BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_INT_EN
				| BCM2835_DMA_TI_S_INC
				| BCM2835_DMA_TI_D_INC));
	FIXED(src,      BCM2835_REG_COUNTER_64BIT_BUS);
	FIXED(dst,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(length,   8);

	/* start the tx-dma - equivalent to:
	 * writel(BCM2835_DMA_CS_ACTIVE,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(start_tx_dma);
	LINKTO(set_tx_dma_next);
	FIXED(ti,BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_BCM2835_DMA_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,      BCM2835_DMA_CS);
	FIXED(length,   4);
	FIXED(pad[0],     BCM2835_DMA_CS_ACTIVE);

	/* schedule link time handling of complete callback */
	SCHEDULE_LINKTIME_TRANSFORM(
		spi_merged_dma_fragment_complete,
		NULL,NULL,NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
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
		&bs->fragment_merged);
	dma_fragment_cache_release(
			&bs->fragment_setup_spi);
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

static struct dma_fragment *bcm2835dma_merged_dma_fragments_alloc(
	struct device *device,gfp_t gfpflags)
{
	return spi_merged_dma_fragment_alloc(
		device,
		sizeof(struct bcm2835dma_spi_merged_dma_fragment),
		gfpflags);
}

/* register all the stuff needed to control dmafragments
   note that the below requires that master has already been registered
   otherwise you get an oops...
 */
#define PREPARE 10 /* prepare the caches with a typical 3 messages */
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
                64,0);
	if (!bs->pool) {
		dev_err(&master->dev,
			"could not allocate DMA-memory pool\n");
		return -ENOMEM;
	}

	/* initialize DMA Fragment pools */
	err=dma_fragment_cache_initialize(
		&bs->fragment_merged,
		&master->dev,
		"fragment_merged",
		&bcm2835dma_merged_dma_fragments_alloc,
		PREPARE*1
		);
	if (err)
		goto error;

	dma_fragment_cache_initialize(
		&bs->fragment_setup_spi,
		&master->dev,
		"config_spi",
		&bcm2835dma_spi_create_fragment_config_spi,
		PREPARE*2
		);
	if (err)
		goto error;

	dma_fragment_cache_initialize(
		&bs->fragment_transfer,
		&master->dev,
		"transfer",
		&bcm2835dma_spi_create_fragment_transfer,
		PREPARE*3
		);
	if (err)
		goto error;

	dma_fragment_cache_initialize(
		&bs->fragment_cs_deselect,
		&master->dev,
		"fragment_cs_deselect",
		&bcm2835dma_spi_create_fragment_cs_deselect,
		PREPARE*1
		);
	if (err)
		goto error;

	dma_fragment_cache_initialize(
		&bs->fragment_delay,
		&master->dev,
		"fragment_delay",
		&bcm2835dma_spi_create_fragment_delay,
		PREPARE/2
		);
	if (err)
		goto error;

	dma_fragment_cache_initialize(
		&bs->fragment_trigger_irq,
		&master->dev,
		"fragment_trigger_irq",
		&bcm2835dma_spi_create_fragment_trigger_irq,
		PREPARE
		);
	if (err)
		goto error;

	return 0;
error:
	bcm2835dma_release_dmafragment_components(master);
	return err;
}
