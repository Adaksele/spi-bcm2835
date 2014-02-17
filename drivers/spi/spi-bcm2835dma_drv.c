/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2013 Martin Sperl
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

/* known limitations:
 *  * cs maps directly to GPIO (except for 0 and 1, which also map to 7+8)
 *    and the mode is not reverted when not used
 *  * if there is a transfer of say 13 bytes, then a total of 16 bytes will get
 *    (over)written, so any tightly packed data would get overwritten
 *    Not sure how we should approach such a situation  - if this is not a
 *    valid situation with drivers right now, then maybe we should make it a
 *    policy
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

/* some module-parameters for debugging and corresponding */
static bool debug_msg = 0;
module_param(debug_msg, bool, 0);
MODULE_PARM_DESC(debug_msg, "Run the driver with message debugging enabled");

static bool debug_dma = 0;
module_param(debug_dma, bool, 0);
MODULE_PARM_DESC(debug_dma, "Run the driver with dma debugging enabled");

static int delay_1us = 889;
module_param(delay_1us, int, 0);
MODULE_PARM_DESC(delay_1us,
		"the value we need to use for a 1 us delay via dma transfers");
/* note that this value has some variation - based on other
 * activities happening on the bus...
 * this also adds memory overhead slowing down the whole system when there
 * is lots of memory access ...
 */

static bool use_transfer_one = 1;
module_param(use_transfer_one, bool, 0);
MODULE_PARM_DESC(use_transfer_one,
		"Run the driver with the transfer_one_message interface");

/*
 * define the BCM2835 registers need to see where these go in the end
 * this should possibly go to bcm2835.h
 */


static void bcm2835dma_release_dmachannel(struct spi_master *master,
					struct bcm2835_dmachannel *d);
static int bcm2835dma_allocate_dmachannel(struct spi_master *master,
					struct bcm2835_dmachannel *d,
					irq_handler_t handler,
					const char* desc
	)
{
        int ret;
	/* fill in defaults */
	d->base=NULL;
	d->bus_addr=0;
	d->chan=0;
	d->irq=0;
	d->handler=NULL;
	d->desc=NULL;
        /* register DMA channel */
        ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST, 
				(void**)&d->base, &d->irq);
        if (ret < 0) {
		d->base=NULL;
		d->chan=0;
		d->irq=0;
                dev_err(&master->dev, "couldn't allocate a DMA channel\n");
                return ret;
        }
        d->chan = ret;
        dev_info(&master->dev, 
		"DMA channel %d at address %pK with irq %d"
		" and handler at %pf\n",
                d->chan, d->base, d->irq,handler);
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
	d->desc=desc;
	d->handler=handler;
	/* calculate the bus_addr */
	d->bus_addr=(d->chan==15)?
		0x7EE05000
		: 0x7E007000 + 256*(d->chan);
	/* and return */
        return 0;
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

static int bcm2835dma_allocate_dma(struct spi_master *master,
				struct platform_device *pdev)
{
	int err;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	/* allocate DMA-channels */
	err=bcm2835dma_allocate_dmachannel(
		master,&bs->dma_tx,bcm2835dma_spi_interrupt_dma_tx,"tx");
	if (err)
		return err;
	err=bcm2835dma_allocate_dmachannel(
		master,&bs->dma_rx,NULL,"rx");
	if (err)
		return err;

	/* allocate pool - need to use pdev here */
	bs->pool=dma_pool_create(
                "DMA-CB-pool",
                &pdev->dev,
                sizeof(struct bcm2835_dma_cb),
                64,
                0
                );
	if (!bs->pool) {
		dev_err(&master->dev, "could not allocate DMA-memory pool\n");
		return -ENOMEM;
	}
	/* allocate some pages from pool for "standard" pages */
	bs->buffer_receive_dummy.addr=
		dma_pool_alloc(bs->pool,GFP_KERNEL,
			&bs->buffer_receive_dummy.bus_addr);
	if (!bs->buffer_receive_dummy.addr)
		/* TODO: errorhandling */
		return -ENOMEM;

	bs->buffer_transmit_0x00.addr=
		dma_pool_alloc(bs->pool,GFP_KERNEL,
			&bs->buffer_transmit_0x00.bus_addr);
	if (!bs->buffer_transmit_0x00.addr)
		/* TODO: errorhandling */
		return -ENOMEM;
	memset(bs->buffer_transmit_0x00.addr,0x00,
		sizeof(struct bcm2835_dma_cb));
	
	bs->dma_status.addr=
		dma_pool_alloc(bs->pool,GFP_KERNEL,
			&bs->dma_status.bus_addr);
	if (!bs->dma_status.addr)
		/* TODO: errorhandling */
		return -ENOMEM;
	memset(bs->dma_status.addr,0x00,sizeof(struct bcm2835_dma_cb));

	/* initialize DMA Fragment pools */
	dma_fragment_cache_initialize(&bs->fragment_composite,
				"composit fragments",
				&bcm2835_spi_dmafragment_create_composite,
				&master->dev,
				5
		);
	dma_fragment_cache_initialize(&bs->fragment_setup_transfer,
				"setup_spi_plus_transfer",
				&bcm2835_spi_dmafragment_create_setup_transfer,
				&master->dev,
				5
		);
	dma_fragment_cache_initialize(&bs->fragment_transfer,
				"transfer",
				&bcm2835_spi_dmafragment_create_transfer,
				&master->dev,
				3
		);
	dma_fragment_cache_initialize(&bs->fragment_cs_deselect,
				"fragment_cs_deselect",
				&bcm2835_spi_dmafragment_create_cs_deselect,
				&master->dev,
				3
		);
	dma_fragment_cache_initialize(&bs->fragment_delay,
				"fragment_delay",
				&bcm2835_spi_dmafragment_create_delay,
				&master->dev,
				3
		);
	dma_fragment_cache_initialize(&bs->fragment_trigger_irq,
				"fragment_trigger_irq",
				&bcm2835_spi_dmafragment_create_trigger_irq,
				&master->dev,
				3
		);

	return 0;
}

static void bcm2835dma_release_dma(struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	bcm2835dma_release_dmachannel(master,&bs->dma_tx);
	bcm2835dma_release_dmachannel(master,&bs->dma_rx);

	if (!bs->pool)
		return;

	dma_pool_free(bs->pool,
		bs->buffer_transmit_0x00.addr,
		bs->buffer_transmit_0x00.bus_addr);
	dma_pool_free(bs->pool,
		bs->buffer_receive_dummy.addr,
		bs->buffer_receive_dummy.bus_addr);
	dma_pool_free(bs->pool,
		bs->dma_status.addr,
		bs->dma_status.bus_addr);

	dma_fragment_cache_release(&bs->fragment_composite);
	dma_fragment_cache_release(&bs->fragment_setup_transfer);
	dma_fragment_cache_release(&bs->fragment_transfer);
	dma_fragment_cache_release(&bs->fragment_cs_deselect);
	dma_fragment_cache_release(&bs->fragment_delay);
	dma_fragment_cache_release(&bs->fragment_trigger_irq);
	dma_pool_destroy(bs->pool);
        bs->pool=NULL;
}



irqreturn_t bcm2835dma_spi_interrupt_dma_tx(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	/* write interrupt message to debug */
	if (unlikely(debug_dma))
		printk(KERN_DEBUG "TX-Interrupt %i triggered\n",irq);

	/* we need to clean the IRQ flag as well
	 * otherwise it will trigger again...
	 * as we are on the READ DMA queue, we should not have an issue
	 * setting/clearing WAIT_FOR_OUTSTANDING_WRITES
	 * and we are not using it for the RX path
	 */
	writel(BCM2835_DMA_CS_INT, bs->dma_rx.base+BCM2708_DMA_CS);

	/* release the control block chains until we reach the CB
	 * this will also call complete
	 */
	//bcm2835dma_release_cb_chain_complete(master);

	/* and return with the IRQ marked as handled */
	return IRQ_HANDLED;
}

static int bcm2835dma_spi_transfer(struct spi_device *spi,
				struct spi_message *message)
{
	//struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int status=-EPERM;
	
	/* fetch DMA fragment */

	/* and schedule it */

	/* and return */
	return status;
}


static void bcm2835dma_set_gpio_mode(u8 pin,u32 mode) {
#ifdef CONFIG_MACH_BCM2708
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

#endif
}

static int bcm2835dma_spi_init_pinmode(void) {
	int err;
	err=gpio_request_one(BCM2835_SPI_GPIO_MISO,GPIOF_IN,
			DRV_NAME":MISO");
	if (err) {
		printk(KERN_ERR DRV_NAME": problems requesting SPI:MISO"
			" on GPIO %i - err %i\n",BCM2835_SPI_GPIO_MISO,err);
		return err;
	}
	err=gpio_request_one(BCM2835_SPI_GPIO_MOSI,GPIOF_OUT_INIT_HIGH,
			DRV_NAME":MOSI");
	if (err) {
		printk(KERN_ERR DRV_NAME": problems requesting SPI:MOSI"
			" on GPIO %i - err %i\n",BCM2835_SPI_GPIO_MOSI,err);
		return err;
	}
	err=gpio_request_one(BCM2835_SPI_GPIO_SCK,GPIOF_OUT_INIT_HIGH,
			DRV_NAME":SCK");
	if (err) {
		printk(KERN_ERR DRV_NAME": problems requesting SPI:SCK"
			"on GPIO %i - err %i\n",BCM2835_SPI_GPIO_SCK,err);
		return err;
	}
	/* SET modes to ALT0=4 */
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MISO,4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MOSI,4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_SCK, 4);

	return 0;
}

static void bcm2835dma_spi_restore_pinmodes(void)
{
	gpio_free(BCM2835_SPI_GPIO_MISO);
	gpio_free(BCM2835_SPI_GPIO_MOSI);
	gpio_free(BCM2835_SPI_GPIO_SCK);
}

static void bcm2835dma_cleanup_spi_device_data(	struct bcm2835dma_spi_device_data *data)
{
	/* remove from chain */
	list_del(&data->spi_device_data_chain);
	/* release GPIO */
	gpio_free(data->chipselect_gpio);
	/* and release memory */
	kfree(data);
}

static void bcm2835dma_spi_cleanup(struct spi_device *spi) 
{
	/* note that surprisingly this does not get called on driver unload */
	struct bcm2835dma_spi_device_data *data=dev_get_drvdata(&spi->dev);
	/* release the memory and GPIO allocated for the SPI device */
	if (data) {
		bcm2835dma_cleanup_spi_device_data(data);
		dev_set_drvdata(&spi->dev,NULL);
	}
}

static int bcm2835dma_spi_setup(struct spi_device *spi) {
	struct spi_master * master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct bcm2835dma_spi_device_data *data;
	u32 tmp;
	int err;

	/* allocate data - if not allocated yet */
	data=dev_get_drvdata(&spi->dev);
	if (!data) {
		data=kzalloc(sizeof(*data),GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		dev_set_drvdata(&spi->dev,data);
		list_add(&data->spi_device_data_chain,
			&bs->spi_device_data_chain);
	}

	/* calculate the real GPIO to use */
	if (spi->master->cs_gpios) {
		data->chipselect_gpio = spi->cs_gpio;
	} else {
		switch (spi->chip_select) {
		case 0:  data->chipselect_gpio = BCM2835_SPI_GPIO_CS0 ; break;
		case 1:  data->chipselect_gpio = BCM2835_SPI_GPIO_CS1 ; break;
		default: data->chipselect_gpio = spi->chip_select     ; break;
		}
	}

	/* check gpio for prohibited pins - MISO, MOSI, SCK are not allowed... */
	if ((spi->mode & SPI_NO_CS)) {
		switch (data->chipselect_gpio) {
		case BCM2835_SPI_GPIO_MISO:
		case BCM2835_SPI_GPIO_MOSI:
		case BCM2835_SPI_GPIO_SCK:
			dev_err(&spi->dev, "Chipselect GPIO %i is not allowed"
				" as it is conflicting with the standard SPI lines\n",
				data->chipselect_gpio);
			err=-EPERM;
			goto error_free;
		default: break;
		}
	}

	/* set the chip_select fields correctly */
	data->chipselect_bitfield = ((u32)1) << (data->chipselect_gpio % 32);
	/* shift right gpio by 5 bit to get the register offset we need to use*/
	data->chipselect_release_gpio_reg = 
	data->chipselect_select_gpio_reg = 0x7e20001C 
		+ 4 * (data->chipselect_gpio >> 5);
	/* and create the name */
	snprintf(data->chipselect_name,sizeof(data->chipselect_name),
		DRV_NAME":CS%i",spi->chip_select);

	/* based od SPI_CS configure the registers */
	if (spi->mode & SPI_CS_HIGH) {
		/* set the registers accordingly */
		data->chipselect_release_gpio_reg += 12;
		/* request the GPIO with correct defaults*/
		err=gpio_request_one(data->chipselect_gpio,GPIOF_OUT_INIT_LOW,
				data->chipselect_name);
		if (err)
			goto error_gpio;
	} else {
		/* set the registers accordingly */
		data->chipselect_select_gpio_reg += 12;
		/* request the GPIO */
		err=gpio_request_one(data->chipselect_gpio,GPIOF_OUT_INIT_HIGH,
				data->chipselect_name);
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
	data->spi_cs_reset = tmp 
		| BCM2835_SPI_CS_CLEAR_RX
		| BCM2835_SPI_CS_CLEAR_TX
		;
	/* the values used to reenable DMA transfers */
	data->spi_cs_set = tmp 
		| BCM2835_SPI_CS_DMAEN
		;

	return 0;

error_gpio:
	dev_err(&spi->dev,"Error allocating GPIO%i - error %i\n",data->chipselect_gpio,err);
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

	bs = spi_master_get_devdata(master);

	INIT_LIST_HEAD(&bs->spi_device_data_chain);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "could not get memory resource\n");
		err = -ENODEV;
		goto out_master_put;
	}

	bs->spi_regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!bs->spi_regs) {
		dev_err(&pdev->dev, "could not request/map memory region\n");
		err = -ENODEV;
		goto out_master_put;
	}

	bs->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(bs->clk)) {
		err = PTR_ERR(bs->clk);
		dev_err(&pdev->dev, "could not get clk: %d\n", err);
		goto out_master_put;
	}

	clk_prepare_enable(bs->clk);

	/* configure pin function for SPI */
	err = bcm2835dma_spi_init_pinmode();
	if (err) {
		dev_err(&pdev->dev, "could not register pins and set the mode: %d\n", err);
		goto out_release_clock;
	}
	
	err = bcm2835dma_allocate_dma(master,pdev);
	if (err) {
		dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
		goto out_release_dma;
	}

	/* initialise the hardware with a reset of the SPI FIFO disabling an existing transfer */
	writel(
		BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX,
		bs->spi_regs+BCM2835_SPI_CS);

	err = spi_register_master(master);
	if (err) {
		dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
		goto out_release_dma;
	}

	return 0;
out_release_dma:
	bcm2835dma_release_dma(master);
/*out_release_gpio: not used*/
	bcm2835dma_spi_restore_pinmodes();
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
	spi_unregister_master(master);

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

	/* Clear FIFOs, and disable the HW block */
	if (bs->spi_regs)
		writel(
			BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX,
			bs->spi_regs+BCM2835_SPI_CS);

	bcm2835dma_spi_restore_pinmodes();
	
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
