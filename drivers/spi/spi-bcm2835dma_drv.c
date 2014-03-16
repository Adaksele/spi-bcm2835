/*
 * Driver for Broadcom BCM2835 SPI Controllers using DMA-FRAGMENTS
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2013 Martin Sperl
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

/* known limitations:
 *  * cs maps directly to GPIO (except for 0 and 1, which also map to 7+8)
 *    and the mode is not reverted when not used
 *  * if there is a transfer of say 13 bytes, then a total of 16 bytes
 *    will get (over)written, so any tightly packed data would get
 *    overwritten.
 *    Not sure how we should approach such a situation - if this is not a
 *    valid situation with drivers right now, then maybe we should make
 *    it a policy
 */
#include "spi-bcm2835dma.h"

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include <linux/gpio.h>

#define SPI_TIMEOUT_MS	3000

#define DRV_NAME	"spi-bcm2835dma"

/* module parameter to dump the dma transforms */
bool debug_dma = 0;
module_param(debug_dma, bool, 0);
MODULE_PARM_DESC(debug_dma,
		"Run the driver with dma debugging enabled");


/* some functions to measure delays on a logic analyzer */
static u32* gpio=0;
static inline void set_low(void) {
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x28/4]=1<<24;
}
static inline void set_high(void) {
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x1C/4]=1<<24;
}

/* schedule a DMA fragment on a specific DMA channel */
static int bcm2835dma_schedule_dma_fragment(
	struct spi_message *msg)
{
	unsigned long flags;
	struct spi_master *master = msg->spi->master;
	struct spi_merged_dma_fragment *frag = msg->state;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct spi_message *last_msg;
	struct spi_merged_dma_fragment *last_frag;

	printk(KERN_INFO "SCHEDULE-DMA fragment %pf %pf\n",msg,frag);

	spin_lock_irqsave(&master->queue_lock,flags);

	last_msg = (!list_empty(&master->queue) ?
		list_last_entry(&master->queue, struct spi_message, queue)
		: NULL);
	printk(KERN_INFO "SCHEDULE-DMA last: %pf\n",last_msg);
	/* link it to the last one on a spi_message level
	 * as well as on a dma level
	 */
	list_add_tail(&msg->queue,&master->queue);

	if (last_msg) {
		last_frag = last_msg->state;
		bcm2835_link_dma_link(
			last_frag->dma_fragment.link_tail,
			frag->dma_fragment.link_head);
		dsb();
	}
	/* now see if DMA is still running */
	if ( !( readl(bs->dma_rx.base+BCM2835_DMA_CS)
			& BCM2835_DMA_CS_ACTIVE )) {
		writel(frag->dma_fragment.link_head->cb_dma,
			bs->dma_rx.base+BCM2835_DMA_ADDR);
		dsb();

		writel(BCM2835_DMA_CS_ACTIVE,
			bs->dma_rx.base+BCM2835_DMA_CS);
		printk(KERN_INFO "SCHEDULE-DMA starting_dma\n");
	}
	spin_unlock_irqrestore(&master->queue_lock,flags);

	return 0;
}

void bcm2835dma_release_cb_chain_complete(struct spi_master *master)
{
	//struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct spi_message *msg;
	struct spi_merged_dma_fragment *frag;
	u32 *complete;
	unsigned long flags;


	/* check the message queue */
	while( 1 ) {
		/* check if we got an entry */
		spin_lock_irqsave(&master->queue_lock,flags);
		msg = list_first_entry_or_null(
			&master->queue,typeof(*msg),queue);
		spin_unlock_irqrestore(&master->queue_lock,flags);
		/* return immediately on NULL
		 * - this actually should not happen...*/
		if (!msg) {
			return;
		}

		printk(KERN_INFO "processing message: %pf\n",msg);
		frag = msg->state;
		printk(KERN_INFO "processing frag: %pf\n",frag);

		/* if we do not have a complete data pointer */
		if (frag->complete_data) {
			complete = (u32*)frag->complete_data;
			/* and if the values are 0, then we stop
			 * further processing */
			if (
				( complete[0] == 0 )
				&& ( complete[1] == 0 ) )
				return;
		}

		/* otherwise we can release the message */
		spin_lock_irqsave(&master->queue_lock,flags);
		list_del_init(&frag->message->queue);
		spin_unlock_irqrestore(&master->queue_lock,flags);

		/* reset status */
		msg->status = 0;

		/* and execute the post-dma-fragments */
		spi_merged_dma_fragment_execute_post_dma_transforms(
			frag,frag,GFP_ATOMIC);

		/* and release fragment - if not optimized */
		if (
#ifdef HAVE_SPI_OPTIMIZE
			! msg->is_optimized
#else
			(1)
#endif
			)
			dma_fragment_release(&frag->dma_fragment);

		/* call the callback
		 * - this could get actually get done also via
		 * a scheduled post_dma */
		if (msg->complete)
			msg->complete(msg->context);
	}
}
irqreturn_t bcm2835dma_spi_interrupt_dma_tx(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	/* write interrupt message to debug */
	if (unlikely(debug_dma))
		printk(KERN_ERR "TX-Interrupt %i triggered\n",irq);

	/* we need to clean the IRQ flag as well
	 * otherwise it will trigger again...
	 * as we are on the READ DMA queue, we should not have an issue
	 * setting/clearing WAIT_FOR_OUTSTANDING_WRITES
	 * and we are not using it for the RX path
	 */
	writel(BCM2835_DMA_CS_INT, bs->dma_tx.base+BCM2835_DMA_CS);

	/* release the control block chains until we reach the CB
	 * this will also call complete
	 */
	bcm2835dma_release_cb_chain_complete(master);

	/* and return with the IRQ marked as handled */
	return IRQ_HANDLED;
}

/**
 * bcm2835dma_spi_message_to_dma_fragment - converts a spi_message to a
 *  dma_fragment
 * @msg:  the spi message to convert
 * @flags: some flags
 * @gfpflags: flags for allocation
 * notes:
 *   with minimal effort this probably could get added to the spi framework
 */
struct spi_merged_dma_fragment *bcm2835dma_spi_message_to_dma_fragment(
	struct spi_message *msg, int flags, gfp_t gfpflags)
{
	struct spi_device *spi    = msg->spi;
	struct spi_master *master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	struct spi_merged_dma_fragment *merged;
	struct spi_transfer *xfer;
	int err=0;
		printk(KERN_INFO "processing message: %pf\n",msg);

	/* some optimizations - it might help if we knew the length... */
	/* check if we got a frame that is of a single transfer */
	if ( list_is_singular(&msg->transfers) ) {
		/* check if we got something in the structure we could use */
	}

	/* fetch a merged fragment */
	merged = (typeof(merged))
		dma_fragment_cache_fetch(
			&bs->fragment_merged,
			gfpflags);
	if (! merged)
		return NULL;

	/* initialize some fields */
	merged->message       = msg;
	merged->transfer      = NULL;
	merged->last_transfer = NULL;
	merged->dma_fragment.link_head = NULL;
	merged->dma_fragment.link_tail = NULL;
	merged->complete_data = NULL;
	merged->needs_spi_setup = 1;

	/* now start iterating the transfers */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* check if we are the last in the list */
		int is_last = list_is_last(&xfer->transfer_list,
					&msg->transfers);
		/* assign the current transfer */
		merged->transfer = xfer;
		/* do we need to reconfigure spi
		   compared to the last transfer */
		if (! merged->needs_spi_setup) {
			if (merged->last_transfer->speed_hz
				!= xfer->speed_hz)
				merged->needs_spi_setup = 1;
			else if (merged->last_transfer->tx_nbits
				!= xfer->tx_nbits)
				merged->needs_spi_setup = 1;
			else if (merged->last_transfer->rx_nbits
				!= xfer->rx_nbits)
				merged->needs_spi_setup = 1;
			else if (merged->last_transfer->bits_per_word
				!= xfer->bits_per_word)
				merged->needs_spi_setup = 1;
		}
		/* if we have no last_transfer,
		   then we need to setup spi */
		if (merged->needs_spi_setup) {
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_setup_spi,
				merged,
				gfpflags);
			if (err)
				goto error;
			merged->needs_spi_setup = 0;
		}
		/* add transfer if the transfer length is not 0
		   or if we vary length */
		if ( (xfer->len)
			/* || (xfer->vary & SPI_OPTIMIZE_VARY_LENGTH) */) {
			/* schedule transfer */
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_transfer,
				merged,
				gfpflags);
			if (err)
				goto error;
			/* set last transfer */
			merged->last_transfer=xfer;
		}
		/* add cs_change with optional extra delay
		   if requested or last in sequence */
		if ((xfer->cs_change)||(is_last)) {
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_cs_deselect,
				merged,
				gfpflags);
		} else if ( (xfer->delay_usecs)
			/* || (xfer->vary & SPI_OPTIMIZE_VARY_DELAY) */) {
			/* or add a delay if requested */
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_delay,
				merged,
				gfpflags);
		}
		if (err)
			goto error;
	}
	/* and add an interrupt if we got a callback to handle
	 * if there is no callback, then we do not need to release it
	 * immediately - even for prepared messages
	 */
	if (msg->complete) {
		err=spi_merged_dma_fragment_merge_fragment_cache(
			&bs->fragment_trigger_irq,
			merged,
			gfpflags);
		if (err)
			goto error;
	}

	/* reset transfers, as these are invalid by the time
	 * we run the transforms */
	merged->transfer      = NULL;
	merged->last_transfer = NULL;

	/* and return it */
	return merged;

error:
	printk(KERN_ERR "bcm2835dma_spi_message_to_dma_fragment:"
		" err=%i\n",err);
	spi_merged_dma_fragment_dump(
		merged,
		&msg->spi->dev,
		0,0,
		&bcm2835_dma_link_dump
		);
	return NULL;
}

static int bcm2835dma_spi_transfer(struct spi_device *spi,
				struct spi_message *message)
{
	int err=0;
	struct spi_merged_dma_fragment *merged;
	printk(KERN_INFO "Starting transfer: %pf %pf %pf\n",spi,message,message->spi);
	/* fetch DMA fragment */
	set_low();
	merged = bcm2835dma_spi_message_to_dma_fragment(
		message,
		0,
		GFP_ATOMIC);
	set_high();
	if (!merged)
		return -ENOMEM;
	/* assign some values */
	message->state = merged;
	message->actual_length = 0;

	/* and execute the pre-transforms */
	err = spi_merged_dma_fragment_execute_pre_dma_transforms(
		merged,merged,GFP_ATOMIC);
	if (err)
		goto error;

	if (unlikely(debug_dma))
		spi_merged_dma_fragment_dump(merged,
					&message->spi->dev,
					0,0,
					&bcm2835_dma_link_dump
			);
	set_low();

	/* and schedule it */
	if (merged) {
		bcm2835dma_schedule_dma_fragment(message);
		set_high();
	}

	/* and return */
	return 0;
error:
	dev_printk(KERN_ERR,&spi->dev,"spi_transfer_failed: %i",err);
	dma_fragment_release(&merged->dma_fragment);
	return -EPERM;
}

static void bcm2835dma_release_dmachannel(struct spi_master *master,
			struct bcm2835_dmachannel *d)
{
	/* if no base, then we are not configured, so return */
	if (!d->base)
                return;
	/* reset the DMA */
	writel(BCM2835_DMA_CS_RESET,d->base+BCM2835_DMA_CS);
	writel(0,d->base+BCM2835_DMA_ADDR);

	/* release interrupt handler and dma */
	if (d->handler)
		free_irq(d->irq,master);
        bcm_dma_chan_free(d->chan);

	/* cleanup structure */
        d->base = NULL;
	d->bus_addr=0;
        d->chan = 0;
        d->irq = 0;
	d->handler=NULL;
	d->desc=NULL;
}

static int bcm2835dma_allocate_dmachannel(struct spi_master *master,
					struct bcm2835_dmachannel *d,
					irq_handler_t handler,
					const char* desc
	)
{
        int ret;
	/* fill in defaults */
	d->base = NULL;
	d->bus_addr = 0;
	d->chan = 0;
	d->irq = 0;
	d->handler = NULL;
	d->desc = NULL;
        /* register DMA channel */
#ifdef CONFIG_MACH_BCM2708
        ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST,
				(void**)&d->base, &d->irq);
#else
	/* TODO - use dmaengine allocate */
	ret = -1;
#endif
        if (ret < 0) {
		d->base = NULL;
		d->chan = 0;
		d->irq = 0;
                dev_err(&master->dev,
			"couldn't allocate a DMA channel\n");
                return ret;
        }
        d->chan = ret;
	if (handler)
		dev_info(&master->dev,
			"DMA channel %d at address %pK with irq %d"
			" and handler at %pf\n",
			d->chan, d->base, d->irq,handler);
	else
		dev_info(&master->dev,
			"DMA channel %d at address %pK with irq %d"
			" and no handler\n",
			d->chan, d->base, d->irq);
	/* and reset the DMA - just in case */
	writel(BCM2835_DMA_CS_RESET,d->base+BCM2835_DMA_CS);
	writel(0,d->base+BCM2835_DMA_ADDR);
	/* and add the irq handler */
	if (handler) {
		ret = request_irq(d->irq,
				handler,
				0,
				dev_name(&master->dev),
				master);
		if (ret) {
			dev_err(&master->dev,
				"could not request IRQ: %d\n", d->irq);
			bcm2835dma_release_dmachannel(master,d);
			return ret;
		}
	}
	/* assign other data */
	d->desc = desc;
	d->handler = handler;
	/* calculate the bus_addr */
	d->bus_addr = (d->chan==15) ? BCM2835_REG_DMA15_BASE_BUS
		: BCM2835_REG_DMA0_BASE_BUS + 256*(d->chan);
	/* and return */
        return 0;
}

static void bcm2835dma_release_dma(struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	bcm2835dma_release_dmachannel(master,&bs->dma_tx);
	bcm2835dma_release_dmachannel(master,&bs->dma_rx);

	bcm2835dma_release_dmafragment_components(master);
}

static int bcm2835dma_allocate_dma(struct spi_master *master,
				struct platform_device *pdev)
{
	int err;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	/* allocate DMA-channels */
	err=bcm2835dma_allocate_dmachannel(
		master,&bs->dma_tx,bcm2835dma_spi_interrupt_dma_tx,"tx");
	if (err)
		goto error;
	err=bcm2835dma_allocate_dmachannel(
		master,&bs->dma_rx,NULL,"rx");
	if (err)
		goto error;

	/* and register the dmafragment_caches */
	err=bcm2835dma_register_dmafragment_components(master);
	if (!err)
		return 0;
error:
	bcm2835dma_release_dma(master);
	return -ENOMEM;
}

static void bcm2835dma_set_gpio_mode(u8 pin,u32 mode) {
	/* this is a bit of a hack, as there seems to be no official
	   way of doing this... */
	/* TODO - PINMUX */
	u32 *gpio = ioremap(0x20200000, SZ_16K);

	/* map pin */
	u32 *reg = &gpio[pin/10];
	u8 shift = ((pin)%10)*3;
	u32 v = *reg;
	v &= ~( ((u32)(7)) << shift );
	v |= (mode & 7) << shift;
	*reg= v;
	iounmap(gpio);
}

static int bcm2835dma_spi_init_pinmode(struct device *dev) {
	int err;
	err=devm_gpio_request_one(dev,BCM2835_SPI_GPIO_MISO,GPIOF_IN,
			DRV_NAME":MISO");
	if (err) {
		printk(KERN_ERR DRV_NAME": problems requesting SPI:MISO"
			" on GPIO %i - err %i\n",BCM2835_SPI_GPIO_MISO,
			err);
		return err;
	}
	err=devm_gpio_request_one(dev,BCM2835_SPI_GPIO_MOSI,GPIOF_OUT_INIT_HIGH,
			DRV_NAME":MOSI");
	if (err) {
		printk(KERN_ERR DRV_NAME": problems requesting SPI:MOSI"
			" on GPIO %i - err %i\n",BCM2835_SPI_GPIO_MOSI,
			err);
		goto error_miso;
	}
	err=devm_gpio_request_one(dev,BCM2835_SPI_GPIO_SCK,GPIOF_OUT_INIT_HIGH,
			DRV_NAME":SCK");
	if (err) {
		printk(KERN_ERR DRV_NAME": problems requesting SPI:SCK"
			"on GPIO %i - err %i\n",BCM2835_SPI_GPIO_SCK,
			err);
		goto error_mosi;
	}
	/* SET modes to ALT0=4 - unfortunately there is not API for that...
	* (which I found) */
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MISO,4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MOSI,4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_SCK, 4);

	bcm2835dma_set_gpio_mode(24, 1);

	return 0;
error_mosi:
	gpio_free(BCM2835_SPI_GPIO_MOSI);
error_miso:
	gpio_free(BCM2835_SPI_GPIO_MISO);
	return err;
}

static void bcm2835dma_spi_restore_pinmodes(struct device *dev)
{
	/* we assume this will reset the MODE */
	devm_gpio_free(dev,BCM2835_SPI_GPIO_MISO);
	devm_gpio_free(dev,BCM2835_SPI_GPIO_MOSI);
	devm_gpio_free(dev,BCM2835_SPI_GPIO_SCK);
}

static void bcm2835dma_cleanup_spi_device_data(
	struct bcm2835dma_spi_device_data *data)
{
	/* remove from chain */
	list_del(&data->spi_device_data_chain);
	/* release GPIO */
	gpio_free(data->cs_gpio);
	/* and release memory */
	kfree(data);
}

static void bcm2835dma_spi_cleanup(struct spi_device *spi)
{
	/* note that surprisingly this does not necessarily
	   get called on driver unload */
	struct bcm2835dma_spi_device_data *data
		= spi_get_ctldata(spi);
	/* release the memory and GPIO allocated for the SPI device */
	if (data) {
		bcm2835dma_cleanup_spi_device_data(data);
		spi_set_ctldata(spi,NULL);
	}
}

static int bcm2835dma_spi_setup(struct spi_device *spi) {
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct bcm2835dma_spi_device_data *data;
	u32 tmp;
	int err;

	/* allocate data - if not allocated yet */
	data=spi_get_ctldata(spi);
	if (!data) {
		data=kzalloc(sizeof(*data),GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		spi_set_ctldata(spi,data);
		list_add(&data->spi_device_data_chain,
			&bs->spi_device_data_chain);
	}
	/* calculate the real GPIO to use */
	if (spi->master->cs_gpios) {
		data->cs_gpio = spi->cs_gpio;
	} else {
		switch (spi->chip_select) {
		case 0:  data->cs_gpio = BCM2835_SPI_GPIO_CS0 ; break;
		case 1:  data->cs_gpio = BCM2835_SPI_GPIO_CS1 ; break;
		default: data->cs_gpio = spi->chip_select     ; break;
		}
	}

	/* check gpio for prohibited pins
	   - MISO, MOSI, SCK are not allowed... */
	if ((spi->mode & SPI_NO_CS)) {
		switch (data->cs_gpio) {
		case BCM2835_SPI_GPIO_MISO:
		case BCM2835_SPI_GPIO_MOSI:
		case BCM2835_SPI_GPIO_SCK:
			dev_err(&spi->dev,
				"Chipselect GPIO %i is not allowed"
				" as it is conflicting with the"
				" standard SPI lines\n",
				data->cs_gpio);
			err=-EPERM;
			goto error_free;
		default: break;
		}
	}

	/* set the chip_select fields correctly */
	data->cs_bitfield = ((u32)1) << (data->cs_gpio % 32);
	/* shift right gpio by 5 bit to get the
	   register offset we need to use*/
	data->cs_deselect_gpio_reg =
	data->cs_select_gpio_reg = BCM2835_REG_GPIO_OUTPUT_SET_BASE_BUS
		+ 4 * (data->cs_gpio >> 5);
	/* and create the name for the gpio */
	snprintf(data->cs_name,sizeof(data->cs_name),
		DRV_NAME":CS%i",spi->chip_select);

	/* based od SPI_CS configure the registers */
	if (spi->mode & SPI_CS_HIGH) {
		/* increment the registers accordingly */
		data->cs_deselect_gpio_reg +=
			(BCM2835_REG_GPIO_OUTPUT_CLEAR_BASE_BUS
				- BCM2835_REG_GPIO_OUTPUT_SET_BASE_BUS);
		/* request the GPIO with correct defaults*/
		err=gpio_request_one(data->cs_gpio,GPIOF_OUT_INIT_LOW,
				data->cs_name);
		if (err)
			goto error_gpio;
	} else {
		/* set the registers accordingly */
		data->cs_select_gpio_reg +=
			(BCM2835_REG_GPIO_OUTPUT_CLEAR_BASE_BUS
				- BCM2835_REG_GPIO_OUTPUT_SET_BASE_BUS);
		/* request the GPIO */
		err=gpio_request_one(data->cs_gpio,GPIOF_OUT_INIT_HIGH,
				data->cs_name);
		if (err)
			goto error_gpio;
	}

	/* and the CS register values used to configure SPI */
	tmp=
		BCM2835_SPI_CS_TA
	        | BCM2835_SPI_CS_CS_01
		| BCM2835_SPI_CS_CS_10
		| ((spi->mode & SPI_CPOL) ? BCM2835_SPI_CS_CPOL : 0)
		| ((spi->mode & SPI_CPHA) ? BCM2835_SPI_CS_CPHA : 0)
		;
	/* the values used to reset SPI FIFOs*/
	data->spi_reset_fifo = tmp
		| BCM2835_SPI_CS_CLEAR_RX
		| BCM2835_SPI_CS_CLEAR_TX
		;
	/* the values used to reenable DMA transfers */
	data->spi_config = tmp
		| BCM2835_SPI_CS_DMAEN
		;

	return 0;

error_gpio:
	dev_err(&spi->dev,"Error allocating GPIO%i - error %i\n"
		,data->cs_gpio,err);
error_free:
	kfree(data);
	dev_set_drvdata(&spi->dev,NULL);
	return err;
}

static int bcm2835dma_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct bcm2835dma_spi *bs;
	struct resource *res;
	int err;

	master = spi_alloc_master(&pdev->dev, sizeof(*bs));
	if (!master) {
		dev_err(&pdev->dev, "spi_alloc_master() failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);

	master->mode_bits = BCM2835_SPI_MODE_BITS;
	master->bits_per_word_mask = BIT(8 - 1);
#ifdef CONFIG_MACH_BCM2708
	master->bus_num = pdev->id;
#endif
	master->num_chipselect = BCM2835_SPI_MAX_CS;
	master->setup = bcm2835dma_spi_setup;
	master->cleanup = bcm2835dma_spi_cleanup;
	master->dev.of_node = pdev->dev.of_node;
	master->rt = 1;

	master->transfer = bcm2835dma_spi_transfer;

	/* not sure if this is needed for the device tree case */
	master->dev.coherent_dma_mask = pdev->dev.coherent_dma_mask;

	bs = spi_master_get_devdata(master);

	/* initialize the queue - it does not get set up when using transfer*/
	INIT_LIST_HEAD(&master->queue);
	/* and the list of per device data - needed for cleanup */
	INIT_LIST_HEAD(&bs->spi_device_data_chain);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"could not get memory resource\n");
		err = -ENODEV;
		goto out_master_put;
	}

	bs->spi_regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!bs->spi_regs) {
		dev_err(&pdev->dev,
			"could not request/map memory region\n");
		err = -ENODEV;
		goto out_master_put;
	}

	bs->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(bs->clk)) {
		err = PTR_ERR(bs->clk);
		dev_err(&pdev->dev,
			"could not get clk: %d\n",
			err);
		goto out_master_put;
	}

	clk_prepare_enable(bs->clk);

	/* configure pin function for SPI */
	err = bcm2835dma_spi_init_pinmode(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev,
			"could not register pins and set the mode: %d\n",
			err);
		goto out_release_clock;
	}

	err = spi_register_master(master);
	if (err) {
		dev_err(&pdev->dev,
			"could not register SPI master: %d\n",
			err);
		goto out_release_gpio;
	}

	err = bcm2835dma_allocate_dma(master,pdev);
	if (err) {
		dev_err(&pdev->dev,
			"could not register SPI master: %d\n",
			err);
		goto out_release_dma;
	}

	/* initialise the hardware with a reset of the SPI FIFO
	   disabling an existing transfer */
	writel(
		BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX,
		bs->spi_regs+BCM2835_SPI_CS);

	return 0;
out_release_dma:
	bcm2835dma_release_dma(master);
/*out_release_dev: not used */
	spi_unregister_master(master);
out_release_gpio:
	bcm2835dma_spi_restore_pinmodes(&pdev->dev);
out_release_clock:
	clk_disable_unprepare(bs->clk);
out_master_put:
	spi_master_put(master);
	return err;
}

static int bcm2835dma_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	/* release the spi_dev parts that do not get released otherwise
	 * cleanup does not get called on the spi_device when unloading
	 * the module
	 */
	while( !list_empty(&bs->spi_device_data_chain)) {
		struct bcm2835dma_spi_device_data *data
			= list_first_entry(&bs->spi_device_data_chain,
                                        typeof(*data),
					spi_device_data_chain);
		bcm2835dma_cleanup_spi_device_data(data);
        }

	/* release pool - also releases all objects */
	bcm2835dma_release_dma(master);

	/* in correct sequence */
	spi_unregister_master(master);

	/* Clear FIFOs, and disable the HW block */
	if (bs->spi_regs)
		writel(
			BCM2835_SPI_CS_CLEAR_RX
			| BCM2835_SPI_CS_CLEAR_TX,
			bs->spi_regs+BCM2835_SPI_CS);

	bcm2835dma_spi_restore_pinmodes(&pdev->dev);

	clk_disable_unprepare(bs->clk);

	spi_master_put(master);

	return 0;
}

static const struct of_device_id bcm2835dma_spi_match[] = {
	{ .compatible = "brcm,bcm2835-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2835dma_spi_match);

/* and "normal" aliases */
#ifdef CONFIG_MACH_BCM2708
static const struct platform_device_id bcm2835dma_id_table[] = {
        { "bcm2835_spi", 2835 },
        { "bcm2708_spi", 2708 },
        { },
};
#endif

static struct platform_driver bcm2835dma_spi_driver = {
	.driver		= {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= bcm2835dma_spi_match,
	},
	.probe		= bcm2835dma_spi_probe,
	.remove		= bcm2835dma_spi_remove,
#ifdef CONFIG_MACH_BCM2708
        .id_table = bcm2835dma_id_table,
#endif
};
module_platform_driver(bcm2835dma_spi_driver);

MODULE_DESCRIPTION("SPI controller driver for Broadcom BCM2835");
MODULE_AUTHOR("Chris Boot <bootc@bootc.net>, "
	"Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL v2");
