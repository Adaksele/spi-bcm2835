/*
 * generic spi_dma_fragment support
 *
 * Copyright (C) 2014 Martin Sperl
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi-dmafragment.h>

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
	int              (*transformation)(void* src,
					void* dst,
					void* extra),
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
EXPORT_SYMBOL_GPL(spi_message_transform_add);

/**
 * spi_message_transform_release_all - release all message transformations
 *   from this list freeing the memory
 * @head: message transform list from which to remove all entries
 */
void spi_message_transform_release_all(
	struct list_head *head)
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
EXPORT_SYMBOL_GPL(spi_message_transform_release_all);

/**
 * spi_dmafragment_create_composite - create a composite DMA fragment
 *   which will contain several other DMA fragments to create a complete
 *   transfer of an SPI message
 *   this is used for both prepared and unprepared messages
 * @device: for which device is this created
 * @gfpflags: which flags should get used for memory allocation purposes
 */
struct dma_fragment *spi_dmafragment_create_composite(
	struct device * device,gfp_t gfpflags)
{
	struct spi_dma_fragment_composite *frag
		= (typeof(frag))dma_fragment_alloc(
			device,gfpflags,sizeof(*frag));

	frag->last_setup_transfer = NULL;
	frag->last_transfer = NULL;

	return &(frag->composite.fragment);
}
EXPORT_SYMBOL_GPL(spi_dmafragment_create_composite);

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
	struct spi_message *msg, int flags, gfp_t gfpflags)
{
	struct spi_device *spi = msg->spi;
	struct spi_master *master = spi->master;
	struct spi_dma_fragment_functions *bs =
		spi_master_get_devdata(master);

	struct spi_dma_fragment_composite *compo;
	struct spi_transfer *xfer;
	int err=0;

	/* fetch a composite fragment */
	compo = (typeof(compo))
		dma_fragment_cache_fetch(
			bs->fragment_composite_cache,
			gfpflags);
	if (! compo)
		return NULL;
	compo->last_setup_transfer = NULL;
	compo->last_transfer = NULL;
	compo->last_xfer = NULL;

	/* now start iterating the transfers */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* check if we are the last in the list */
		int is_last=list_is_last(&msg->transfers,
					&xfer->transfer_list);
		/* do we need to reconfigure spi
		   compared to the last transfer */
		if (compo->last_transfer) {
			if (compo->last_xfer->speed_hz
				!= xfer->speed_hz)
				compo->last_transfer=NULL;
			else if (compo->last_xfer->tx_nbits
				!= xfer->tx_nbits)
				compo->last_transfer=NULL;
			else if (compo->last_xfer->rx_nbits
				!= xfer->rx_nbits)
				compo->last_transfer=NULL;
			else if (compo->last_xfer->bits_per_word
				!= xfer->bits_per_word)
				compo->last_transfer=NULL;
		}
		/* now decide which transfer to use,
		   the normal or the reset version */
		if (compo->last_transfer) {
			err=bs->add_transfer(
				msg,xfer,compo,flags,gfpflags);
		} else {
			err=bs->add_setup_spi_transfer(
				msg,xfer,compo,flags,gfpflags);
		}
		/* error handling */
		if (err)
			goto error;
		/* add cs_change with optional extra delay
		   if requested or last in sequence */
		if ((xfer->cs_change)||(is_last))
			err=bs->add_cs_deselect(
				msg,xfer,compo,flags,gfpflags);
		else if (xfer->delay_usecs)
			/* or add a delay if requested */
			err=bs->add_delay(
				msg,xfer,compo,flags,gfpflags);
		/* handle errors */
		if (err)
			goto error;
		/* and set the last_transfer */
		compo->last_xfer=xfer;
	}
	/* and add an interrupt if we got a callback to handle
	 * if there is no callback, then we do not need to release it
	 * immediately - even for prepared messages
	 */
	if (
		(msg->complete)
		&& (err = bs->add_delay(msg,xfer,compo,flags,gfpflags))
		)
		goto error;

	/* and return it */
	return compo;

error:
	return NULL;
}
EXPORT_SYMBOL_GPL(spi_message_to_dma_fragment);

MODULE_DESCRIPTION("spi specific dma-fragment infrastructure");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL");
