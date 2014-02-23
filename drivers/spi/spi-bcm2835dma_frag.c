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
 * START_CREATE_FRAGMENT_COMMON- macro that contains repetitive
 *   variable definitions used by all allocate functions
 *    (a bit separated to avoid some warnings)
 * note: no semicolon for the last define, as we run into compiler
 *   warnings otherwise when it sees ";;" and then some more variable
 *   definitions and then complains about "mixed declaration and code"
 */
#define START_CREATE_FRAGMENT_COMMON()					\
	struct spi_master * master = (struct spi_master *)device;	\
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);	\
	struct dma_pool *pool = bs->pool;				\
	struct dma_link *link;						\
	struct bcm2835_dma_cb *cb

/**
 * START_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 *   used by all allocate functions forfragment allocation
 * @struct_name: name of structure to allocate as frag
 */

#define START_CREATE_FRAGMENT_COMMON_ALLOCATE(struct_name)		\
	struct struct_name *frag =					\
		(struct struct_name *) dma_fragment_alloc(		\
			device,gfpflags,				\
			sizeof(struct struct_name));			\
	if (! frag)							\
		return NULL;

/**
 * START_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 *   used by most allocate functions
 * @struct_name: name of structure to allocate as frag
 */
#define START_CREATE_FRAGMENT_ALLOCATE(struct_name)			\
	START_CREATE_FRAGMENT_COMMON();					\
	START_CREATE_FRAGMENT_COMMON_ALLOCATE(struct_name);

/**
 * END_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 *   used by all alloc functions
 */
#define END_CREATE_FRAGMENT_ALLOCATE()			\
	dma_fragment_set_default_links(			\
		(struct dma_fragment*)frag);		\
	return (struct dma_fragment*)frag;		\
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
		dma_fragment_add_dma_fragment_transform(
			frag,trans);
		return 0;
	} else
		return 1;
}

#define VARY_TRANSFORM_HELPER(varyflags,functionname,suffix)	\
	_VARY_TRANSFORM_HELPER(varyflags,functionname,suffix)
#define _VARY_TRANSFORM_HELPER(varyflags,functionname,suffix)		\
	int functionname ## suffix (					\
		struct dma_fragment_transform * transform,		\
		struct dma_fragment *frag, void *vp,			\
		gfp_t gfpflags)						\
	{								\
		u32 vary=0;						\
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

static int bcm2835dma_fragment_transform_spi_data_offset(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragtocast, void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment */
	struct spi_merged_dma_fragments *merged_frag =
		(typeof(merged_frag)) vp;
	/* the dma_fragment to chain */
	struct dma_fragment *frag =
		(typeof(frag)) fragtocast;

	/* the spi device */
	struct spi_device *spi = merged_frag->message->spi;

	/* pointer to spi_device_data as a char pointer*/
	char *base=dev_get_drvdata(&spi->dev);
	base += (u32)transform->src;
	*((u32*)transform->dst) = *((u32*)(base));
	return 0;
}

#define SPI(field,value) _SPI(field,value)
#define _SPI(field,value)						\
	if (bcm2835dma_schedule_fragment_transform(			\
			(struct dma_fragment*)frag, /* dirty cast */	\
			/* assuming the fragment is at the beginning */	\
			/* of the structure - needed for simplicity */	\
			bcm2835dma_fragment_transform_spi_data_offset,	\
			(void*)						\
			offsetof(struct bcm2835dma_spi_device_data,	\
				value),					\
			&((struct bcm2835_dma_cb*)link->cb)->field,	\
			NULL,						\
			gfpflags)					\
		)							\
		goto error;

/* note: we may have this information when creating the fragment,
   so no real need, if we have the information at that time available */
#define TXDMA(field,offset) cb->field = bs->dma_tx.bus_addr+offset

#define VARY(...)

#define IGNORE(...)

/*------------------------------------------------------------------------
 * allocator for transfers
 *----------------------------------------------------------------------*/
struct dma_fragment_transfer {
	struct dma_fragment fragment;
	struct dma_link     *xfer_rx;
	struct dma_link     *xfer_tx;
};

struct dma_fragment_config_spi_transfer {
	struct dma_fragment_transfer fragment;
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

static int bcm2835dma_spi_create_fragment_transfer_common(
	struct dma_fragment_transfer *frag,
	struct device *device,
	gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_COMMON();
	/* the rx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   xfer->rx_buf[i] = readl(BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT(xfer_rx);
	VARY (ti,     xfer.rx_addr);
	FIXED(src,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARY (dst,    xfer.rx_addr);
	VARY (length, xfer.length);

	/* the tx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   writel(xfer->tx_buf[i],BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT(xfer_tx);
	VARY (ti,     xfer.tx_addr);
	VARY (src,    xfer.tx_addr);
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARY (length, xfer.length);
	FIXED(pad[0], 0); /* in case we have no pointer, so use this */

	return 0;
error:
	return 1;
}

static int bcm2835dma_fragment_transform_copyadd_length(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragtocast, void *vp,
	gfp_t gfpflags)
{
	u32 length = (u32)transform->src;
	struct dma_fragment_transfer *frag = transform->dst;
	u32 *addto = transform->extra;

	/* copy the length where it belongs */
	((struct bcm2835_dma_cb *)frag->xfer_tx->cb)->length = length;
	((struct bcm2835_dma_cb *)frag->xfer_rx->cb)->length = length;
	/* and add it */
	*addto += length;

	if (*addto<65536)
		return 0;

	/* compain about non-obvious fact */
	printk(KERN_ERR "spi-bcm2835dma - the total (variable) transfer"
		" requested (%i) exceeds 65535 bytes,"
		" which is not allowed\n",*addto);
	return -E2BIG;
}

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

static inline int bcm2835dma_fragment_transform_buffer_do_dma_map(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragtocast, void *vp,
	gfp_t gfpflags)
{
	return bcm2835dma_fragment_transform_buffer_addr(
		transform->src,transform->dst,transform->extra,1);
}
VARY_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_RX|SPI_OPTIMIZE_VARY_RX),
		bcm2835dma_fragment_transform_buffer_do_dma_map,
		_vary);


static inline int bcm2835dma_fragment_transform_buffer_is_dma_mapped(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragtocast, void *vp,
	gfp_t gfpflags)
{
	return bcm2835dma_fragment_transform_buffer_addr(
		transform->src,transform->dst,transform->extra,0);
}
VARY_TRANSFORM_HELPER((SPI_OPTIMIZE_VARY_RX|SPI_OPTIMIZE_VARY_RX),
		bcm2835dma_fragment_transform_buffer_is_dma_mapped
		,_vary);

static int bcm2835dma_fragment_transform_buffer_do_dma_unmap(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragtocast, void *vp,
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


static int bcm2835dma_fragment_transform_linktx(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragtocast, void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment */
	struct bcm2835dma_spi_merged_dma_fragments *merged_frag =
		(typeof(merged_frag)) vp;

	/* the dma_fragment_transfer to chain */
	struct dma_fragment_transfer *frag =
		(typeof(frag)) fragtocast;

	/* src is defining if we are a setup_transfer or not */
	struct dma_fragment_config_spi_transfer *setup = transform->src;

	/* the info about the current message and transfer */
	struct spi_message  *mesg = merged_frag->spi_fragments.message;
	struct spi_transfer *xfer = merged_frag->spi_fragments.transfer;

	/* set the effective vary flags */
	u32 vary = 0; /* (xfer->var & merged_frag->vary_mask) */

	/* link already assigned to tx */
	struct dma_link *link_tx = frag->xfer_tx;
	struct bcm2835_dma_cb *cb_tx =
		(struct bcm2835_dma_cb *)link_tx->cb;
	struct dma_link *link_rx = frag->xfer_rx;
	struct bcm2835_dma_cb *cb_rx =
		(struct bcm2835_dma_cb *)link_rx->cb;

	int ret=0;

	/* link the fragments */
	if (merged_frag->link_txdma_next) {
		((struct bcm2835_dma_cb *)(
                        merged_frag->link_txdma_next->cb))
			->next = link_tx->cb_dma;
	}
	/* link this one against 0 - just in case we are ther last one */
	cb_tx->next = 0;

	/* need to reset total length if we have started */
	if (setup) {
		merged_frag->total_length =
			&((struct bcm2835_dma_cb *)
				setup->config_clock_length->cb)->pad[1];
		*merged_frag->total_length = 0;
	}
	/* check fixed->vary and vary->fixed situations -
	   also there is an issue with resetting the total xfer
	 */

	/* first handle length vary */
	if (vary & SPI_OPTIMIZE_VARY_LENGTH) { /* the case of vary */
		/* we always reset SPI after such a transfer,
		 * but we allow the one before to be of fixed length!
		 */
		merged_frag -> link_txdma_next = NULL;
		/* if there is a value in total_length, then this means:
		   we have had a fixed transfer previously,
		   so we do not start at 0 from the transfer,
		   thus we need to set it to this value via the transforms
		*/
		if (*merged_frag->total_length) {
			ret = bcm2835dma_schedule_fragment_transform(
				&merged_frag->spi_fragments.fragment,
				&dma_fragment_transform_write_u32,
				(void*)*merged_frag->total_length,
				merged_frag->total_length,
				NULL,
				gfpflags
				);
			if (ret)
				return ret;
		}
		/* schedule copy add */
		ret = bcm2835dma_schedule_fragment_transform(
			&merged_frag->spi_fragments.fragment,
			&bcm2835dma_fragment_transform_copyadd_length,
			&xfer->len,
			frag,
			merged_frag->total_length,
			gfpflags
			);
		if (ret)
			return ret;
	} else { /* the static case */
		/* if length is a multiple of 4,
		 * then allow link with next */
		merged_frag -> link_txdma_next =
			(xfer->len % 4) ? NULL : link_tx;
		/* copy length to rx/tx */
		cb_rx->length = xfer->len;
		cb_tx->length = xfer->len;
		*(merged_frag->total_length) += xfer->len;
	}

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

static struct dma_fragment *bcm2835dma_spi_create_fragment_transfer(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_COMMON_ALLOCATE(dma_fragment_transfer);
	if (
		bcm2835dma_spi_create_fragment_transfer_common(
			frag,device, gfpflags)
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

	/* we also need to link the tx_channel to the previous, but as
	   this only happens during dma-fragment linking, we need to
	   schedule it
	 */
	if (bcm2835dma_schedule_fragment_transform(
			&frag->fragment,
			bcm2835dma_fragment_transform_linktx,
			NULL,NULL,NULL,
			gfpflags)
		)
		goto error;


	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for setting up spi and transfer
 *----------------------------------------------------------------------*/

static int bcm2835dma_fragment_transform_speed_hz(
	struct dma_fragment_transform * transform,
	struct dma_fragment *fragtocast, void *vp,
	gfp_t gfpflags)
{
	/* the merged fragment */
	struct bcm2835dma_spi_merged_dma_fragments *merged_frag =
		(typeof(merged_frag)) vp;

	/* src is defining if we are a setup_transfer or not */
	struct dma_fragment_config_spi_transfer *setup =
		(typeof(setup)) fragtocast;
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

struct dma_fragment *bcm2835dma_spi_create_fragment_config_spi_transfer(
	struct device *device,gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_config_spi_transfer);

	/* before we do any of this we need to
	 * schedule a cdiv calculator */
	/* TODO */

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
	VARY (pad[0], clock_divider);
	VARY (pad[1],/* the total DMA length we can only set on link time */);

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
	IGNORE(pad[0],/* this is set later, when we know the dma_addr
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

	/* the Transfer portion is a bit tricky...
	 * so most of the varies will get managed by a specific transform
	 */
	if (
		bcm2835dma_spi_create_fragment_transfer_common(
			&frag->fragment,device, gfpflags)
		)
		goto error;
	/* we can not link tx here in a normal way
	 * but we have to assign it to pad0 instead...
	 */
	((struct bcm2835_dma_cb *)(frag->set_tx_dma_next->cb))
		->pad[0]=frag->fragment.xfer_tx->cb_dma;

	/* and we need to link xfer_rx to the correct dma */
	link = frag->fragment.xfer_rx;
	LINKTO(start_tx_dma);

	/* schedule the SPI divider calculation */
	if (bcm2835dma_schedule_fragment_transform(
		&frag->fragment.fragment,
		&bcm2835dma_fragment_transform_speed_hz_vary,
		NULL,
		NULL,
		NULL,
		gfpflags))
		goto error;

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
		struct dma_fragment *fragtocast, void *vp,
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
	VARY    (length, delay_usecs);
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
	VARY (length, delay_usecs);
	set_delay_post  = &cb->length;

	/* schedule the vary transform for link-time */
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
	VARY (length, delay_usecs);

	/* schedule the vary transform for link-time */
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
	/* this is set later, when we know the dma_addr
	   of the TX-DMA-transfer */
	IGNORE(pad[0]);

	/* copy the timestamp from the counter to a fixed address */
	ADD_DMA_LINK_TO_FRAGMENT(set_tx_dma_next);
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
		&bs->fragment_composite,
		&master->dev,
		"merged_fragments",
		&bcm2835dma_merged_dma_fragments_alloc,
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
 * message transforms
 *----------------------------------------------------------------------*/

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

#endif
