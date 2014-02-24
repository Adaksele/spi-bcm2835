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
			device,gfpflags,				\
			sizeof(struct struct_name));			\
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
 * FIXED - macro that defines the field as one that can get assigned with
 * a static value when creating the fragment.
 * this is guaranteed to never get modified
 * @field: the field in the dma-controlblock
 * @value: the value to assign
 */
#define FIXED(field,value) cb->field = value;

/**
 * bcm2835dma_schedule_fragment_transform - adds a dma_fragment_transform
 * to an existing dma_fragment
 * @frag: the fragment to whicht to add it
 * @transformer: the transform function
 * @src: the value that gets set as src for the transform
 *   (available to the function)
 * @dst: the value that gets set as dst for the transform
 *   (available to the function)
 * @extra: the value that gets set as extra for the transform
 *   (available to the function)
 * @gfpflags: used to allocate the dma_fragment_transform structure
 */
static inline int bcm2835dma_schedule_fragment_transform(
	struct dma_fragment *frag,
	int (*transformer)(struct dma_fragment_transform *,
			struct dma_fragment *, void *,gfp_t),
	void *src, void *dst, void *extra,
	gfp_t gfpflags)
{
	struct dma_fragment_transform *trans =
		dma_fragment_transform_alloc(
			transformer,
			src,dst,extra,
			0,gfpflags
			);
	if (trans) {
		/* TODO: add to pre-dma */;
		dma_fragment_add_dma_fragment_transform(
			frag,trans);
		return 0;
	} else
		return 1;
}

/**
 * VARY_TRANSFORM_HELPER - macro that depending on the vary bitflags
 * either schedules the transfer to get done on DMA scheduling
 * or getting executed immediately.
 */
#define VARY_TRANSFORM_HELPER(varyflags,functionname,suffix)		\
	int functionname ## suffix (					\
		struct dma_fragment_transform * transform,		\
		struct dma_fragment *frag, void *vp,			\
		gfp_t gfpflags)						\
	{								\
		u32 vary=0;/*((struct spi_merged_dma_fragments*)vp)	\
			     ->transfer->vary */			\
		/* reschedule the part for delay if set */		\
		if ( (varyflags == 0) || (vary & (varyflags) ) )	\
			return bcm2835dma_schedule_fragment_transform(	\
				frag,functionname,			\
				transform->src, transform->dst,		\
				transform->extra, gfpflags);		\
		else  /* else run the real code now */			\
			return functionname(				\
				transform,frag,vp,gfpflags);		\
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
 */
static int bcm2835dma_fragment_transform_spi_data_offset(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragment, void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment */
	struct spi_merged_dma_fragments *merged_frag =
		(typeof(merged_frag)) vp;
	/* the dma_fragment to chain */
	struct dma_fragment *frag = (typeof(frag)) fragment;

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
 * this get executed during link time
 */
#define SPI(field,spi_data_field)					\
	if (bcm2835dma_schedule_fragment_transform(			\
			(struct dma_fragment*)frag, /* dirty cast */	\
			/* assuming the fragment is at the beginning */	\
			/* of the structure - needed for simplicity */	\
			bcm2835dma_fragment_transform_spi_data_offset,	\
			(void*)						\
			offsetof(struct bcm2835dma_spi_device_data,	\
				spi_data_field),			\
			&((struct bcm2835_dma_cb*)link->cb)->field,	\
			NULL,						\
			gfpflags)					\
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
 * of a dma_fragment to the bcm2835dma_spi_merged_dma_fragments happens.
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
 * @fragment: the fragment to which this transform belongs
 * @vp: the fragment into which this fragment is getting merged
 * Note:
 *   vp is a pointer to bcm2835dma_spi_merged_dma_fragments and contains:
 *   a pointer to spi_message and spi_dev
 *   as well as some fields that contain the clock divider and delay count
 *   it also sets fragment.config_clock_length.pad[0] to the clock-divider
 *
 * depending on vary flags in spi_transfer this may get executed during
 * link time or prior to executing the dma (in the optimized case)
 */
static int bcm2835dma_fragment_transform_speed_hz(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragment, void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment */
	struct bcm2835dma_spi_merged_dma_fragments *merged_frag =
		(typeof(merged_frag)) vp;

	/* src is defining if we are a setup_transfer or not */
	struct dma_fragment_config_spi *setup =
		(typeof(setup)) fragment;
	struct bcm2835_dma_cb *cb = setup->config_clock_length->cb;

	/* the info about the current message and transfer */
	struct spi_message  *mesg = merged_frag->spi_fragments.message;
	struct spi_transfer *xfer = merged_frag->spi_fragments.transfer;

	struct spi_master *master=mesg->spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	/* transform to what we need */
	u32 spi_hz = xfer->speed_hz;
	u32 clk_hz = clk_get_rate(bs->clk);
	u32 cdiv;

	/* set the current speed */
	merged_frag->speed_hz = spi_hz;

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
	merged_frag->speed_cdiv = cdiv;
	cb->pad[0] = cdiv;

	/* and now calculate the delay for a half clock cycle
	   - for now we assume that it is equal to clk_div */
	if (cdiv)
		merged_frag->delay_half_cycle_dma_length  = cdiv;
	else
		merged_frag->delay_half_cycle_dma_length  = 65535;

	return 0;
}
VARY_TRANSFORM_HELPER(SPI_OPTIMIZE_VARY_FRQUENCY,
		bcm2835dma_fragment_transform_speed_hz,_vary)

/**
 *  bcm2835dma_spi_create_fragment_config_spi - allocate and set up the
 *   dma_fragment to configure the SPI device
 * @device: the spi_master.device for  which we set this up
 * @gpfflags: the gpf_t flags to use when allocating memory
 */
struct dma_fragment *bcm2835dma_spi_create_fragment_config_spi(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_config_spi);

	/* before we do any of this we need to
	 * schedule a cdiv calculation */
	/* schedule the SPI divider calculation */
	if (bcm2835dma_schedule_fragment_transform(
		&frag->fragment,
		&bcm2835dma_fragment_transform_speed_hz_vary,
		NULL,
		NULL,
		NULL,
		gfpflags))
		goto error;

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
	VARY (pad[1], bcm2835dma_fragment_transform_copyadd_length
		/* the total DMA length we can only set on link time */);

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
	LATER(pad[0], /* this is set later, when we know the dma_addr
			 of the TX-DMA-transfer */);

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
 * bcm2835dma_fragment_transform_buffer_addr - the helper function will
 *   do all the transforms on the xfer.rx/tx_buf and assign them to
 *   the xfer_rx/tx dma control-blocks
 *   it will also dmamap the rx/tx buffers if necessary
 * @xfer: the xfer which we process
 * @frag the dma_fragment_transfer structure to which we assign this
 * @dev: the device (needed primarily for dmamap)
 * @is_dma_mapped: flags if we are already dma-mapped
 */
static inline int bcm2835dma_fragment_transform_buffer_addr(
	struct spi_transfer *xfer,
	struct dma_fragment_transfer *frag,
	struct device *dev,
	int is_dma_mapped
	)
{
	struct bcm2835_dma_cb *cb = frag->xfer_tx->cb;

	if (xfer->tx_buf) {
		if (is_dma_mapped) {
			cb->src = xfer->tx_dma;
			cb->pad[0] = 0;
		} else {
			cb->src = dma_map_single_attrs(
                                dev,
                                (void *)xfer->tx_buf,
                                xfer->len,
                                DMA_TO_DEVICE,
                                NULL);
			/* TODO: error-handling */
			/* needed to unmap */
			cb->pad[0] = (u32) xfer->tx_buf;
		}
		cb->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_INC;
	} else {
		cb->src = 0;
		cb->pad[0] = 0;
		cb->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE;
	}

	cb = frag->xfer_rx->cb;
	if (xfer->rx_buf) {
		if (is_dma_mapped) {
			cb->dst = xfer->rx_dma;
			cb->pad[0] = 0;
		} else {
			cb->dst = dma_map_single_attrs(
                                dev,
                                (void *)xfer->rx_buf,
                                xfer->len,
                                DMA_FROM_DEVICE,
                                NULL);
			/* TODO: error-handling */
			/* needed to unmap */
			cb->pad[0] = (u32) xfer->rx_buf;
		}
		cb->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_D_INC;
	} else {
		cb->dst = 0;
		cb->ti = BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_D_IGNORE;
	}

	return 0;
}

/**
 * bcm2835dma_fragment_transform_buffer_do_dma_map -dma_fragment_transform
 *  that will call bcm2835dma_fragment_transform_buffer_addr with the
 *  correct arguments.
 * @transform: the transform that contains all additional parameters
 * @transform_src: the spi_transfer, which we are observing
 * @transform_dst: the dma_fragment_transfer which we modify
 * @transform_extra: the device we need to use for dmamap
 * @fragment: the fragment to which this transform belongs
 * @vp: the fragment into which this fragment is getting merged
 * @vp_message: the message for which we do this exercise
 * @vp_transfer: the spi_transfer for which we run this exercise
 * note that we also allow to vary this based on the VARY_RX/TX flags.
 *  this does not make use of the vp->transfer structure, because this
 *  will change after linking (it contains transient data)
 */
static inline int bcm2835dma_fragment_transform_buffer_do_dma_map(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragment, void *vp,
	gfp_t gfpflags)
{
	return bcm2835dma_fragment_transform_buffer_addr(
		transform->src,transform->dst,transform->extra,1);
}
VARY_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_RX|SPI_OPTIMIZE_VARY_TX),
		bcm2835dma_fragment_transform_buffer_do_dma_map,
		_vary);

/**
 * bcm2835dma_fragment_transform_buffer_is_dma_mapped -dma_fragment_transform
 *  that will call bcm2835dma_fragment_transform_buffer_addr with the
 *  correct arguments.
 * @transform: the transform that contains all additional parameters
 * @transform_src: the spi_transfer, which we are observing
 * @transform_dst: the dma_fragment_transfer which we modify
 * @transform_extra: the device we need to use for dmamap
 * @fragment: the fragment to which this transform belongs
 * @vp: the fragment into which this fragment is getting merged
 * @vp_message: the message for which we do this exercise
 * @vp_transfer: the spi_transfer for which we run this exercise
 * note that we also allow to vary this based on the VARY_RX/TX flags.
 *  this does not make use of the vp->transfer structure, because this
 *  will change after linking (it contains transient data)
 */
static inline int bcm2835dma_fragment_transform_buffer_is_dma_mapped(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragment, void *vp,
	gfp_t gfpflags)
{
	return bcm2835dma_fragment_transform_buffer_addr(
		transform->src,transform->dst,transform->extra,0);
}
VARY_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_RX|SPI_OPTIMIZE_VARY_TX),
		bcm2835dma_fragment_transform_buffer_is_dma_mapped
		,_vary);


/**
 * bcm2835dma_fragment_transform_buffer_is_dma_mapped -dma_fragment_transform
 *  that will call dmaunmap for pointers
 * @transform: the transform that contains all additional parameters
 * @transform_dst: the dma_fragment_transfer which we need to unmap
 * @transform_extra: the device we need to use for dmamap
 * @fragment: the fragment to which this transform belongs
 * @vp: the fragment into which this fragment is getting merged
 * @vp_message: the message for which we do this exercise
 * @vp_transfer: the spi_transfer for which we run this exercise
 * note that we also allow to vary this based on the VARY_RX/TX flags.
 *  this does not make use of the vp->transfer structure, because this
 *  will change after linking (it contains transient data)
 */
static int bcm2835dma_fragment_transform_buffer_do_dma_unmap(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragment, void *vp,
	gfp_t gfpflags)
{
	/*struct spi_transfer *xfer = transform->src;*/
	struct dma_fragment_transfer *frag = transform->dst;
	struct device *dev = transform->extra;

	struct bcm2835_dma_cb *cb;

	/* we should only run this on post transforms */

	cb = frag->xfer_tx->cb;
	if (cb->src)
                dma_unmap_single_attrs(
                        dev,
                        cb->src,
                        cb->length,
                        DMA_TO_DEVICE,
                        NULL);

	cb = frag->xfer_rx->cb;
	if (cb->dst)
                dma_unmap_single_attrs(
                        dev,
                        cb->dst,
                        cb->length,
                        DMA_FROM_DEVICE,
                        NULL);

	return 0;
}
VARY_TRANSFORM_HELPER(0,
		bcm2835dma_fragment_transform_buffer_do_dma_unmap,
		_vary);

/**
 * bcm2835dma_fragment_transform_copyadd_length - the dma_fragment_transform
 *  that adds the dma length of the current xfer.len to the global DMA
 *  transfer counter.
 * @transform: the transform that contains all additional parameters
 * @transform_src: the source pointer of the variable length
 * @transform_dst: the destination fragment which we need to update the length
 * @transform_extra: the value to start from or -1 for just adding
 * @fragment: the fragment to which this transform belongs
 * @fragment_total_length: the pointer to which to add/update
 * @vp: the fragment into which this fragment is getting merged
 * @vp_message: the message for which we do this exercise
 * @vp_transfer: the spi_transfer for which we run this exercise
 * Note:
 *   this sets fragment->xfer_tx->length and fragment->xfer_rx->length
 *   and it also adds the length to vp_
 */
static int bcm2835dma_fragment_transform_copyadd_length(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragment, void *vp,
	gfp_t gfpflags)
{
	struct bcm2835dma_spi_merged_dma_fragments *mergedfrag =
		(typeof(mergedfrag)) fragment;

	u32 length = *((u32*)transform->src);
	struct dma_fragment_transfer *frag = transform->dst;
	s32 setbasevalue = *((s32*)transform->extra);

	/* set the value if necessary */
	if (setbasevalue >= 0)
		*mergedfrag->total_length = setbasevalue;

	/* copy the length where it belongs */
	((struct bcm2835_dma_cb *)frag->xfer_tx->cb)->length = length;
	((struct bcm2835_dma_cb *)frag->xfer_rx->cb)->length = length;
	/* and add it to the total */
	*mergedfrag->total_length += length;

	/* check that we do not exceed 65535 bytes in a transfer
	   at least per documentation this is not allowed */
	if (*mergedfrag->total_length > 65536) {
		printk(KERN_ERR "spi-bcm2835dma - the total transfer length"
			" requested (%i) exceeds 65535 bytes,"
			" which is not allowed on this device\n",
			*mergedfrag->total_length);
		return -E2BIG;
	}

	return 0;
}
VARY_TRANSFORM_HELPER(SPI_OPTIMIZE_VARY_LENGTH,
		bcm2835dma_fragment_transform_copyadd_length,_vary)

/**
 * bcm2835dma_fragment_transform_linktx - dma_fragment_transform which
 *  links the current transfer to the previous one
 * @transform: the transform that contains all additional parameters
 * @transform_dst: the dma_fragment_transfer which we need to unmap
 * @transform_extra: the device we need to use for dmamap
 * @fragment: the fragment to which this transform belongs
 * @vp: the fragment into which this fragment is getting merged
 * @vp_message: the message for which we do this exercise
 * @vp_transfer: the spi_transfer for which we run this exercise
 * note that we also allow to vary this based on the VARY_RX/TX flags.
 *  this does not make use of the vp->transfer structure, because this
 *  will change after linking (it contains transient data)
 */
static int bcm2835dma_fragment_transform_linktx(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragment, void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment */
	struct bcm2835dma_spi_merged_dma_fragments *merged_frag =
		(typeof(merged_frag)) vp;

	/* the dma_fragment_transfer to chain */
	struct dma_fragment_transfer *frag =
		(typeof(frag)) fragment;

	/* the info about the current message and transfer */
	struct spi_message  *mesg = merged_frag->spi_fragments.message;
	struct spi_transfer *xfer = merged_frag->spi_fragments.transfer;

	/* set the effective vary flags */
	u32 vary = 0; /* (xfer->var & merged_frag->vary_mask) */

	/* link already assigned to tx */
	struct dma_link *link_tx = frag->xfer_tx;
	struct bcm2835_dma_cb *cb_tx =
		(struct bcm2835_dma_cb *)link_tx->cb;

	int ret=0;

	/* link the tx dma */
	*merged_frag->txdma_link_to_here = link_tx->cb_dma;
	merged_frag->txdma_link_to_here = &cb_tx->next;
	cb_tx->next=0;

	/* if we vary length, then we need to reset SPI afterwards*/
	if (vary & SPI_OPTIMIZE_VARY_LENGTH) {
		/* if we define a SPI_OPTIMIZE_VARY_LENGTH_X4 for for
		   transfers of a multiple of 4, we could avoid this,
		   but then other stuff needs to get taken care of, like
		   mixing FIXED Length and VARY length in sequences
		*/
		merged_frag->txdma_link_to_here=NULL;
	}

	/* as for the fixed 4 byte multiple followed by vary case
	   we need to keep a copy of the length so far and start
	   with this value, so schedule an assignment of xfer */
	ret = bcm2835dma_schedule_fragment_transform(
		&merged_frag->spi_fragments.fragment,
		bcm2835dma_fragment_transform_copyadd_length_vary,
		&xfer->len,
		fragment,
		(void*)*merged_frag->total_length,
		gfpflags
		);
	if (ret)
		return ret;

	/* now handle the xfer rx/tx parts */
	ret = bcm2835dma_schedule_fragment_transform(
		&merged_frag->spi_fragments.fragment,
		(mesg->is_dma_mapped) ?
		&bcm2835dma_fragment_transform_buffer_is_dma_mapped_vary
		: &bcm2835dma_fragment_transform_buffer_do_dma_map_vary,
		xfer,
		frag,
		&mesg->spi->master->dev,
		gfpflags
		);
	if (ret)
		return ret;

	/* in both cases we need to set up transforms that unmap */
	if (!mesg->is_dma_mapped) {
		/* schedule unmap */
		ret = bcm2835dma_schedule_fragment_transform(
			&merged_frag->spi_fragments.fragment,
			&bcm2835dma_fragment_transform_buffer_do_dma_unmap_vary,
			xfer,
			frag,
			&mesg->spi->master->dev,
			gfpflags
			);
	}

	return ret;
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
	if (bcm2835dma_schedule_fragment_transform(
			&frag->fragment,
			bcm2835dma_fragment_transform_linktx,
			NULL,NULL,NULL,
			gfpflags)
		)
		goto error;

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

/**
    bcm2835dma_fragment_set_delay_half_cycle - allows setting up to two
      delay addresses (src,dst) to the half frequency delay value according
      to the current speed_hz. If extra is set, then set the xfer->delay
      here if value of xfer->delay is non-zero.
*/
static inline int bcm2835dma_fragment_set_delay(
		struct dma_fragment_transform * transform,
		struct dma_fragment *fragment, void *vp,
		gfp_t gfpflags)
{
	/* the merged fragment */
	struct bcm2835dma_spi_merged_dma_fragments *merged_frag =
		(typeof(merged_frag)) vp;
	struct spi_transfer *xfer = merged_frag->spi_fragments.transfer;

	u32 delay = merged_frag -> delay_half_cycle_dma_length;

	if (transform->src)
		*(u32*)transform->src = delay;
	if (transform->dst)
		*(u32*)transform->dst = delay;
	if (transform->extra) {
		delay = xfer->delay_usecs * delay_1us;

		if (transform->extra == transform->src)
			*(u32*)transform->src += delay;
		else if (transform->extra == transform->dst)
			*(u32*)transform->dst += delay;
		else
			*(u32*)transform->extra = delay;
	}

	return 0;
}
VARY_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_DELAY
				| SPI_OPTIMIZE_VARY_FRQUENCY),
		bcm2835dma_fragment_set_delay,_vary_delay_frequency);
VARY_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_DELAY),
		bcm2835dma_fragment_set_delay,_vary_delay);


static struct dma_fragment *bcm2835dma_spi_create_fragment_cs_deselect(
	struct device *device,gfp_t gfpflags)
{
	u32 *set_delay_pre;
	u32 *set_delay_post;
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_cs_deselect);

	/* delay by half a clock cycle or by the delay given in xfer
	 * equivalent to: udelay(max(xfer->delay_usecs,
	 *   500000/xfer->clock_frequency)))
	 */
	ADD_DMA_LINK_TO_FRAGMENT(delay_pre);
	FIXED   (ti,     ( BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_WAITS(0x1f)
				| BCM2835_DMA_TI_NO_WIDE_BURSTS
				| BCM2835_DMA_TI_S_IGNORE
				| BCM2835_DMA_TI_D_IGNORE));
	FIXED(src,    0);
	FIXED(dst,    0);
	VARY    (length, bcm2835dma_fragment_set_delay,xfer->delay_usec);
	set_delay_pre  = &cb->length;

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
	VARY (length, bcm2835dma_fragment_set_delay,half_clock_cycle);
	set_delay_post  = &cb->length;

	/* schedule the vary transform for link-time
	 * assigning the half clock cycle delay to pre and post
	 * and add a xfer.delay_usec if set
	 */
	if (bcm2835dma_schedule_fragment_transform(
			&frag->fragment,
			bcm2835dma_fragment_set_delay_vary_delay_frequency,
			set_delay_pre,set_delay_post,set_delay_pre,
			gfpflags)
		)
		goto error;

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for adding some delay
 *----------------------------------------------------------------------*/
struct dma_fragment_delay {
	struct dma_fragment fragment;
	struct dma_link     *delay;
};

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
	VARY (length, bcm2835dma_fragment_set_delay,xfer->delay_usec);

	/* schedule the vary transform for link-time
	   onyl set the xfer.delay_usec to the delay length
	 */
	if (bcm2835dma_schedule_fragment_transform(
			&frag->fragment,
			bcm2835dma_fragment_set_delay_vary_delay,
			NULL,NULL,&cb->length,
			gfpflags)
		)
		goto error;

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
	return dma_fragment_alloc(
		device,
		sizeof(struct bcm2835dma_spi_merged_dma_fragments),
		gfpflags);
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
		&bs->fragment_merged,
		&master->dev,
		"fragment_merged",
		&bcm2835dma_merged_dma_fragments_alloc,
		3
		);
	if (err)
		goto error;

	dma_fragment_cache_initialize(
		&bs->fragment_setup_spi,
		&master->dev,
		"config_spi",
		&bcm2835dma_spi_create_fragment_config_spi,
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

	bs->spi_dma_functions.fragment_merged_cache=&bs->fragment_merged;

	return 0;
error:
	bcm2835dma_release_dmafragment_components(master);
	return err;
}
