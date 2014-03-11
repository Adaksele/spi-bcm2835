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
#define GET_VARY_FROM_XFER(xfer)  0*xfer->len /*xfer->vary*/

/**
 * THIS_CB_MEMBER_DMA_ADDR - dma_addr_t of a DMA_CB.member
 * @member: the member field in the dma CB
 */
#define THIS_CB_MEMBER_DMA_ADDR(member)	\
	BCM2835_DMA_CB_MEMBER_DMA_ADDR(link,member)

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

#define START_CREATE_FRAGMENT_USE_TRANS()				\
	struct dma_fragment_transform *trans				\

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
 * ADD_DMA_LINK_TO_FRAGMENT_FLAGS - macro that allocates a new dma_link
 *  and adds it to the member field in the structure
 *  it also does some basic linking and sets up some fields with defaults
 * @field: the field to which to assign the dma_link to
 */
#define ADD_DMA_LINK_TO_FRAGMENT_FLAGS(field,astail)			\
	link = dma_link_init(&frag->field,				\
			pool,						\
			0,1,						\
			gfpflags					\
		);							\
	if (!link)							\
		goto error;						\
	link->desc = #field;						\
	dma_fragment_add_dma_link(					\
		(struct dma_fragment *)frag,				\
		(struct dma_link *)&frag->field,			\
		astail							\
		);							\
	cb = (struct bcm2835_dma_cb *)link->cb;				\
	cb->next=0;							\
	cb->stride=0;

/**
 * LINK_TO - macro that links the dma_link pointed to by field in the
 * custom dma_fragment structure to the currently active dma_link
 * @field: the field name
 */
#define LINKTO(field)					\
	bcm2835_link_dma_link(&frag->field,link);

#define ADD_DMA_LINK_TO_FRAGMENT(field)		\
	ADD_DMA_LINK_TO_FRAGMENT_FLAGS(field,1)
#define ADD_DMA_LINK_TO_FRAGMENT_NOLINK(field)	\
	ADD_DMA_LINK_TO_FRAGMENT_FLAGS(field,0)


/*------------------------------------------------------------------------
 * helpers and macros to to make the basic "setup" of dma_fragments
 * easier to read
 *----------------------------------------------------------------------*/

#define GENERIC_TRANSFORM_WRAPPER(functionname,suffix)			\
	static int functionname ## suffix (				\
		struct dma_fragment_transform *transform,		\
		void *vp, gfp_t gfpflags)				\
	{								\
		return functionname(					\
			(void*)transform->fragment,			\
			(struct spi_merged_dma_fragment *)vp,		\
			transform->data,				\
			gfpflags);					\
	}
#define LINK_TRANSFORM_WRAPPER(functionname,suffix)		\
	GENERIC_TRANSFORM_WRAPPER(functionname,suffix)
#define VARY_TRANSFORM_WRAPPER(functionname,suffix)		\
	GENERIC_TRANSFORM_WRAPPER(functionname,suffix)

#define SCHEDULE_LINKTIME_TRANSFORM(function,data)		 \
	if (! dma_fragment_addnew_link_transform(		 \
			&frag->dma_fragment,0,			 \
			&function,data,				 \
			gfpflags))				 \
		goto error;

#define SCHEDULE_VARY_TRANSFORM(function,data)			 \
		goto error;

#define LINKVARY_TRANSFORM_WRAPPER(varymask,function,suffix)		\
	GENERIC_TRANSFORM_WRAPPER(function,suffix##_wrap)		\
	static int function ## suffix (					\
		struct dma_fragment_transform *transform,		\
		void *vp,						\
		gfp_t gfpflags)						\
	{								\
		struct dma_fragment *frag  = transform->fragment;	\
		struct spi_merged_dma_fragment *merged = vp;		\
		/* get the vary flag from the transfer */		\
		int vary = GET_VARY_FROM_XFER(merged->transfer);	\
		/* depending on vary and varymask make the decission */	\
		if (vary & varymask) {					\
			/* TODO: put into PRE-DMA phase */		\
			return -EPERM;					\
		} else {						\
			return function(				\
				(void*)frag,				\
				merged,					\
				transform->data,			\
				gfpflags);				\
		}							\
	}

#define SCHEDULE_LINKVARY_TRANSFORM(function,data)		 \
	if (! dma_fragment_addnew_link_transform(		 \
			&frag->dma_fragment,0,			 \
			&function,data,				 \
			gfpflags))				 \
		goto error;



#if 0

/**
 * SCHEDULE_LINKTIME_TRANSFORM - schedules a transform during link time
 * @function: function to schedule
 * @src: values to set in transform
 * @dst: values to set in transform
 * @extra: values to set in transform
 */
#define SCHEDULE_LINKTIME_TRANSFORM(function,data)		 \
	if (! dma_fragment_addnew_transform(			 \
			&function,				 \
			&frag->fragment,			 \
			data,					 \
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
		if ( 1 ||(!varyflags) || (vary & varyflags) ) {		\
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
#endif
/**
 * FIXED - macro that defines the field as one that can get assigned with
 * a static value when creating the fragment.
 * this is guaranteed to never get modified
 * @field: the field in the dma-controlblock
 * @value: the value to assign
 */
#define FIXED(field,value) cb->field = value;

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
struct  dma_fragment_transform_spi_drvdata {
	struct dma_fragment_transform transform;
	size_t offset;
};

static int bcm2835dma_fragment_transform_spi_data_offset(
	struct dma_fragment_transform *tr,
	void *vp,
	gfp_t gfpflags)
{
	struct dma_fragment_transform_spi_drvdata *transform =
		(typeof(transform)) tr;
	/* the merged fragment from the extra void pointer argument */
	struct spi_merged_dma_fragment *merged_frag = vp;

	/* the spi device from the message pointer */
	struct spi_device *spi = merged_frag->message->spi;

	/* pointer to spi_device_data as a char pointer*/
	char *base=dev_get_drvdata(&spi->dev);
	base += transform->offset;

	/* copy the value */
	*((u32*)transform->transform.data) = *((u32*)(base));

	return 0;
}

/**
 * SPI - macro that copies some value from the spi_device_data structure
 * to the field in the dma_controlblock during link time
 * @field: the field name in the dma controlblock
 * @spi_data_field: the field in the spi_device_data structure
 *   this get executed during link time
 * Note: this is a bit inefficient, as it requires more calls/entries
 *   than necessary when we need to update multiple fields.
 *   in this case a "common" function works best...
 *   see SPISET and SPIDONE to get used to ducument those cases.
 */
#define SPI(field,spi_data_field)					\
	trans = dma_fragment_addnew_link_transform(			\
		&frag->dma_fragment,					\
		sizeof(struct dma_fragment_transform_spi_drvdata),	\
		&bcm2835dma_fragment_transform_spi_data_offset,		\
		&dma_link_to_cb(link)->field,				\
		gfpflags);						\
	if (! trans )							\
		goto error;						\
	((struct  dma_fragment_transform_spi_drvdata *)trans)->offset =	\
		offsetof(struct bcm2835dma_spi_device_data,		\
			spi_data_field);

/**
 * SPISET - macro do set the field to the field from spi_data
 * @field: the field name in the dma controlblock
 * @spi_data_field: the field in the spi_device_data structure
 * @linkname: the dma_link for which we set it
 */
#define SPISET(field,spi_data_field,linkname)				\
	dma_link_to_cb(&frag->linkname)->field = spi_data->spi_data_field;

/**
 * SPIDONE - macro that marks that this field is supposed to get done by
 *  transform function - no code generated
 * @field: the field name in the dma controlblock
 * @spi_data_field: the field in the spi_device_data structure
 * @linkname: the dma_link for which we set it
 * note:
 *   macro is empty and mostly there to test that we did not forget
 *   setting anything...
 */
#define SPIDONE(field,spi_data_field,linkname)

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
	struct dma_fragment dma_fragment;
	/* additional data - allocated already to reduce overhead */
	struct dma_link     cs_select;
	struct dma_link     reset_spi_fifo;
	struct dma_link     config_clock_length;
	struct dma_link     config_spi;
	struct dma_link     set_tx_dma_next;
	struct dma_link     start_tx_dma;
};

/**
 * bcm2835dma_fragment_transform_link_config_spi - transform that
 *   will initialize bcm2835dma_spi_merged_dma_fragment.txdma_link_to_here
 *   to the correct value.
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data - not used
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_link_speed(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged,typeof(*merged),spi_fragment);
	/* cast to correct type */
	struct dma_fragment_config_spi *frag =
		container_of(dma_frag,typeof(*frag),dma_fragment);
	/* the spi-device and transfer for which we run this */
	struct spi_master *master = spi_merged->message->spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct spi_transfer *xfer = spi_merged->transfer;
	/* get the speed values */
	u32 spi_hz = xfer->speed_hz;
	u32 clk_hz = clk_get_rate(bs->clk);
	/* now calc the clock divider */
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
	merged->speed_hz = spi_hz;
	merged->speed_cdiv = cdiv;
	dma_link_to_cb(&frag->config_clock_length)->pad[0] = cdiv;

	/* and now calculate the delay for a half clock cycle
	   - for now we assume that it is equal to clk_div */
	if (cdiv)
		merged->delay_half_cycle_dma_length  = cdiv*2;
	else
		merged->delay_half_cycle_dma_length  = 1<<17;

	return 0;
}
LINKVARY_TRANSFORM_WRAPPER(SPI_OPTIMIZE_VARY_SPEED_HZ,
			bcm2835dma_fragment_transform_link_speed,
			_vary);

/**
 * bcm2835dma_fragment_transform_link_config_spi - transform that
 *   will initialize bcm2835dma_spi_merged_dma_fragment.txdma_link_to_here
 *   to the correct value.
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data - not used
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_link_config_spi(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged,typeof(*merged),spi_fragment);
	/* cast to correct type */
	struct dma_fragment_config_spi *frag =
		container_of(dma_frag,typeof(*frag),dma_fragment);
	/* the spi-device for which we run this */
	struct bcm2835dma_spi_device_data *spi_data =
		dev_get_drvdata(&spi_merged->message->spi->dev);

	/* set up the address where to link a transfer */
	merged->txdma_link_to_here =
		&dma_link_to_cb(&frag->set_tx_dma_next)->pad[0];
	/* and setup "initial" total length and reset to 0 */
	merged->total_length =
		&dma_link_to_cb(&frag->config_clock_length)->pad[1];
	/* actually we would need to vary this part... */
	*merged->total_length = 0;

	/* implementing the SPIDONE below during link time*/
	SPISET(dst,    cs_select_gpio_reg, cs_select);
	SPISET(pad[0], cs_bitfield       , cs_select);
	SPISET(pad[0], spi_reset_fifo    , reset_spi_fifo);
	SPISET(pad[0], spi_config        , config_spi);

	/* we also schedule our transfrom of speed based on vary */
	/* and return OK */
	return 0;
}
LINK_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_link_config_spi,
		_wrap);

/**
 *  bcm2835dma_spi_create_fragment_config_spi - allocate and set up the
 *   dma_fragment to configure the SPI device
 * @device: the spi_master.device for  which we set this up
 * @gpfflags: the gpf_t flags to use when allocating memory
 */
struct dma_fragment *bcm2835dma_spi_create_fragment_config_spi(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_USE_TRANS();
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_config_spi);

	/* before we do any of this we need to
	 * schedule a cdiv calculation */
	/* schedule the SPI divider calculation */
	SCHEDULE_LINKVARY_TRANSFORM(
		bcm2835dma_fragment_transform_link_speed_vary,
		NULL);

	/* select chipselect - equivalent to:
	   writel(spi_dev_data->cs_bitfield,
  	          spi_dev_data->cs_select_gpio_reg);
	*/
	ADD_DMA_LINK_TO_FRAGMENT(cs_select);
	FIXED  (ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED  (src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	SPIDONE(dst,    cs_select_gpio_reg, cs_select);
	FIXED  (length, 4);
	SPIDONE(pad[0], cs_bitfield, cs_select);

	/* reset SPI fifos - equivalent to:
	 * writel(spi_dev_data->spi_reset_fifo,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(reset_spi_fifo);
	LINKTO(cs_select);
	FIXED  (ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED  (src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED  (dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED  (length, 4);
	SPIDONE(pad[0], spi_reset_fifo, reset_spi_fifo);

	/* configure clock divider and transfer length  - equivalent to:
	 * writel(cdiv, BCM2835_SPI_CLK);
	 * writel(total transfer length, BCM2835_SPI_DLEN);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_clock_length);
	LINKTO(reset_spi_fifo);
	FIXED(ti,     ( BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_S_INC
			| BCM2835_DMA_TI_D_INC
			));
	FIXED(src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK));
	FIXED(length, 8);
	VARY (pad[0], bcm2835dma_fragment_transform_speed_hz);
	LATER(pad[1], bcm2835dma_fragment_transform_prepare_txlink
		/* set to 0 during link time */);

	/* configure and start spi - equivalent to:
	 * writel(spi_dev_data->spi_config,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_spi);
	LINKTO(config_clock_length);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED(length,   4);
	SPIDONE(pad[0], spi_config, config_spi);

	/* set up the tx-dma start address - equivalent to:
	 * writel(dma_address_of_tx_transfer,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(set_tx_dma_next);
	LINKTO(config_spi);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,    BCM2835_DMA_ADDR);
	FIXED(length, 4);
	LATER(pad[0], bcm2835dma_fragment_transform_prepare_txlink
		/* this is set later, when we know the dma_addr
		   of the TX-DMA-transfer */);

	/* start the tx-dma - equivalent to:
	 * writel(BCM2835_DMA_CS_ACTIVE,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(start_tx_dma);
	LINKTO(set_tx_dma_next);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,    BCM2835_DMA_CS);
	FIXED(length, 4);
	FIXED(pad[0], BCM2835_DMA_CS_ACTIVE);

	/* schedule link time handling of settings including SPIDONE */
	SCHEDULE_LINKTIME_TRANSFORM(
		bcm2835dma_fragment_transform_link_config_spi_wrap,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for transfers
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_transfer - the dma_fragment structure to schedule a
 *   single spi_transfer
 * @fragment: the main embedded dma_fragment structure
 * @xfer_rx: the dma_link responsible for receiving data from SPI
 * @xfer_tx: the dma_link responsible for transmitting data over SPI
 */

struct dma_fragment_transfer {
	struct dma_fragment dma_fragment;
	struct dma_link     xfer_rx;
	struct dma_link     xfer_tx;
};

/**
 * bcm2835dma_fragment_transform_link_length - transform that
 *   will set up the length for the transfers adding values if needed
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_length(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged,typeof(*merged),spi_fragment);
	/* cast to correct type */
	struct dma_fragment_transfer *frag =
		container_of(dma_frag,typeof(*frag),dma_fragment);
	/* the spi-device for which we run this */
	struct bcm2835dma_spi_device_data *spi_data =
		dev_get_drvdata(&spi_merged->message->spi->dev);
	/* the transfer for which we run this */
	struct spi_master *master = spi_merged->message->spi->master;
	struct spi_transfer *xfer = spi_merged->transfer;

	/* set length of transfers */
	dma_link_to_cb(&frag->xfer_tx)->length = xfer->len;
	dma_link_to_cb(&frag->xfer_rx)->length = xfer->len;

	/* set total length to (u32)data if set */
	if (data)
		*merged->total_length = (u32)data;

	/* and add to total length value - or set it if data is non 0 */
	*merged->total_length += xfer->len;

	/* and return */
	return 0;
}
VARY_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_length,
		_vary);

/**
 * bcm2835dma_fragment_transform_link_transfer - handle all the vary
 *   decissions for the linking of transfers
 *
 */
static int bcm2835dma_fragment_transform_transfer(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	int err;
	/* a new transform for predma */
	struct dma_fragment_transform *predma;
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged,typeof(*merged),spi_fragment);
	/* cast to correct type */
	struct dma_fragment_transfer *frag =
		container_of(dma_frag,typeof(*frag),dma_fragment);
	/* the transfer */
	struct spi_message *message = spi_merged->message;
	struct spi_transfer *xfer   = spi_merged->transfer;
	/* get the vary information */
	int vary = GET_VARY_FROM_XFER(xfer);

	printk(KERN_ERR "HERE-----------\n");

	/* link tx-dma to last one - unfortunately we
	 * can't use the generic bcm2835_link_dma_link */
	*merged->txdma_link_to_here = frag->xfer_tx.cb_dma;

	/* connect link to here */
	merged->txdma_link_to_here =
		&dma_link_to_cb(&frag->xfer_tx)->next;

	/* and based on vary we act differently */
	if (vary & SPI_OPTIMIZE_VARY_LENGTH) {
		/* need to link length at pre-dma time */
#if 0
		predma=spi_dma_fragment_addnew_predma_transform(
			merged,0,
			&bcm2835dma_fragment_transform_length_vary,
			(void*)*merged->total_length,
			gfpflags);
		if (!predma)
			return -ENOMEM;
#endif
		/* and mark txdma_link_to_here as NULL
		 * as we have to schedule a new spi config
		 * to reset dma fifos
		 * note: with some "extra" VARY_LENGTH_X4
		 * we could avoid this
		 */
		merged->txdma_link_to_here = NULL;
	} else {
		/* otherwise we call it immediately,
		 *  but with NULL as data so that we just increment */
		err=bcm2835dma_fragment_transform_length(
			dma_frag,spi_merged,NULL,gfpflags);
		if (err)
			return err;
	}

	/* see if we need to mmap the data or not */
	if (message->is_dma_mapped) {
	} else  {
	}

	return 0;
}
LINK_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_transfer,_link);

#if 0
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

	printk("XXX-transform_length: %i - %i - %pf - %i\n",
		cb_rx->length,
		cb_tx->length,
		merged->total_length,
		*merged->total_length
		);

	/* if it is not a multiple of 4,
	 * then we need to schedule new setup_spi */
	if (xfer->len & 3)
		merged->spi_fragment.last_transfer = NULL;

	/* we should also check for possible length */
	if (*merged->total_length > 65535)
		return -EINVAL;

	return 0;
}
/*VARY_LINK_TRANSFORM_HELPER(SPI_OPTIMIZE_VARY_LENGTH,
		bcm2835dma_fragment_transform_length,
		_vary);*/

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
		cb_rx->ti =
			BCM2835_DMA_TI_PER_MAP(BCM2835_DMA_DREQ_SPI_RX)
			| BCM2835_DMA_TI_S_DREQ
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_D_INC;
	} else {
		cb_rx->ti =
			BCM2835_DMA_TI_PER_MAP(BCM2835_DMA_DREQ_SPI_RX)
			| BCM2835_DMA_TI_S_DREQ
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_D_IGNORE;
	}
	if (xfer->tx_buf) {
		cb_tx->ti =
			BCM2835_DMA_TI_PER_MAP(BCM2835_DMA_DREQ_SPI_TX)
			| BCM2835_DMA_TI_D_DREQ
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_INC;
	} else {
		cb_tx->ti =
			BCM2835_DMA_TI_PER_MAP(BCM2835_DMA_DREQ_SPI_TX)
			| BCM2835_DMA_TI_D_DREQ
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE;
	}

	return 0;
}
/*VARY_LINK_TRANSFORM_HELPER(
	(SPI_OPTIMIZE_VARY_RX_BUF|SPI_OPTIMIZE_VARY_TX_BUF),
	bcm2835dma_fragment_transform_buffer,
	_vary);*/

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
/*VARY_LINK_TRANSFORM_HELPER(
	(SPI_OPTIMIZE_VARY_RX_BUF|SPI_OPTIMIZE_VARY_TX_BUF),
	bcm2835dma_fragment_transform_dmamap,
	_vary);*/

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
	struct spi_message *msg = transform->data;
	struct device *dev = &msg->spi->master->dev;

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
#endif
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
	ADD_DMA_LINK_TO_FRAGMENT_NOLINK(xfer_tx);
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
			bcm2835dma_fragment_transform_transfer_link,
			NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for deselecting cs
 *----------------------------------------------------------------------*/
struct dma_fragment_cs_deselect {
	struct dma_fragment dma_fragment;
	struct dma_link     delay_pre;
	struct dma_link     cs_deselect;
	struct dma_link     delay_post;
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
	cb = (struct bcm2835_dma_cb *)frag->delay_pre.cb;
	cb->length = delay + xfer->delay_usecs * delay_1us;

	/* and the post delay */
	cb = (struct bcm2835_dma_cb *)frag->delay_post.cb;
	cb->length = delay;

	return 0;
}
/*VARY_LINK_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_DELAY_USECS
				| SPI_OPTIMIZE_VARY_SPEED_HZ),
				bcm2835dma_fragment_transform_set_cs_delaylength,_vary);*/

static struct dma_fragment *bcm2835dma_spi_create_fragment_cs_deselect(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_USE_TRANS();
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
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
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
#if 0
	SCHEDULE_LINKTIME_VARY_TRANSFORM(
		bcm2835dma_fragment_transform_set_cs_delaylength_vary,
		NULL);
#endif
	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for adding some delay
 *----------------------------------------------------------------------*/
struct dma_fragment_delay {
	struct dma_fragment dma_fragment;
	struct dma_link     delay;
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
		(struct bcm2835_dma_cb *)frag->delay.cb;
	cb->length = merged -> delay_half_cycle_dma_length;

	return 0;
}
/*VARY_LINK_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_DELAY_USECS),
		bcm2835dma_fragment_transform_set_delaylength,
		_vary);*/

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
#if 0
	SCHEDULE_LINKTIME_VARY_TRANSFORM(
		bcm2835dma_fragment_transform_set_delaylength_vary,
		NULL);
#endif
	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for triggering an interrupt
 *----------------------------------------------------------------------*/
struct dma_fragment_trigger_irq {
	struct dma_fragment dma_fragment;
	struct dma_link     set_tx_dma_next;
	struct dma_link     message_finished;
	struct dma_link     start_tx_dma;
};

static inline int bcm2835dma_transforms_prepare_for_schedule(
	struct dma_fragment_transform *transform,
	void *vp, gfp_t gfpflags)
{
	struct spi_merged_dma_fragment *merged = (typeof(merged)) vp;
	struct dma_fragment_trigger_irq *frag =
		(typeof(frag)) transform->fragment;
	struct bcm2835_dma_cb *cb =
		(struct bcm2835_dma_cb *)frag->message_finished.cb;

	/* set the pad0/pad1 of message_finished to 0 */
	cb->pad[0] = 0;
	cb->pad[1] = 0;

	/* and set the pointer so that the interrupt-handler may use it */
	merged->complete_data = &cb->pad[0];

	/* and the other stuff that we should do prior to scheduling
	   the transfer */
	return spi_merged_dma_fragment_prepare_for_schedule(
		transform,vp,gfpflags);
}

static struct dma_fragment *bcm2835dma_spi_create_fragment_trigger_irq(
	struct device *device,gfp_t gfpflags)
{
	u32 *link_tx;
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_trigger_irq);
	/* set up the tx-dma start address - equivalent to:
	 * writel(dma_address_of_tx_transfer,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(set_tx_dma_next);
	FIXED(ti,BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,      BCM2835_DMA_ADDR);
	FIXED(length,   4);
	LATER(pad[0],  /* this is set later, when we know the dma_addr
			   of the TX-DMA-transfer */);
	link_tx = &cb->pad[0];

	/* copy the timestamp from the counter to a fixed address */
	ADD_DMA_LINK_TO_FRAGMENT_FLAGS(message_finished,0);
	FIXED(ti,       ( BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_INT_EN
				| BCM2835_DMA_TI_S_INC
				| BCM2835_DMA_TI_D_INC));
	FIXED(src,      BCM2835_REG_COUNTER_64BIT_BUS);
	FIXED(dst,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(length,   8);
	*link_tx = link->cb_dma;

	/* start the tx-dma - equivalent to:
	 * writel(BCM2835_DMA_CS_ACTIVE,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(start_tx_dma);
	LINKTO(set_tx_dma_next);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,      BCM2835_DMA_CS);
	FIXED(length,   4);
	FIXED(pad[0],   BCM2835_DMA_CS_ACTIVE);

	/* schedule link time scheduling of complete callback */
#if 0
	SCHEDULE_LINKTIME_VARY_TRANSFORM(
		bcm2835dma_transforms_prepare_for_schedule,NULL);
#endif
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
	return &spi_merged_dma_fragment_alloc(
		&bcm2835_link_dma_link,
		sizeof(struct bcm2835dma_spi_merged_dma_fragment),
		gfpflags)->dma_fragment;
}

/* register all the stuff needed to control dmafragments
   note that the below requires that master has already been registered
   otherwise you get an oops...
 */
#define PREPARE 1+0 /* prepare the caches with a typical 3 messages */
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
