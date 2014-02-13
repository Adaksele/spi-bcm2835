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
 */

#include "bcm2835-dma.h"
#include "DMAFragment.h"

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
#include <linux/spi/spi.h>

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include <linux/gpio.h>
//#include 

/* SPI register offsets */
#define BCM2835_SPI_BASE_BUS		0x7E204000

#define BCM2835_SPI_CS			0x00
#define BCM2835_SPI_FIFO		0x04
#define BCM2835_SPI_CLK			0x08
#define BCM2835_SPI_DLEN		0x0c
#define BCM2835_SPI_LTOH		0x10
#define BCM2835_SPI_DC			0x14

/* Bitfields in CS */
#define BCM2835_SPI_CS_LEN_LONG		0x02000000
#define BCM2835_SPI_CS_DMA_LEN		0x01000000
#define BCM2835_SPI_CS_CSPOL2		0x00800000
#define BCM2835_SPI_CS_CSPOL1		0x00400000
#define BCM2835_SPI_CS_CSPOL0		0x00200000
#define BCM2835_SPI_CS_RXF		0x00100000
#define BCM2835_SPI_CS_RXR		0x00080000
#define BCM2835_SPI_CS_TXD		0x00040000
#define BCM2835_SPI_CS_RXD		0x00020000
#define BCM2835_SPI_CS_DONE		0x00010000
#define BCM2835_SPI_CS_LEN		0x00002000
#define BCM2835_SPI_CS_REN		0x00001000
#define BCM2835_SPI_CS_ADCS		0x00000800
#define BCM2835_SPI_CS_INTR		0x00000400
#define BCM2835_SPI_CS_INTD		0x00000200
#define BCM2835_SPI_CS_DMAEN		0x00000100
#define BCM2835_SPI_CS_TA		0x00000080
#define BCM2835_SPI_CS_CSPOL		0x00000040
#define BCM2835_SPI_CS_CLEAR_RX		0x00000020
#define BCM2835_SPI_CS_CLEAR_TX		0x00000010
#define BCM2835_SPI_CS_CPOL		0x00000008
#define BCM2835_SPI_CS_CPHA		0x00000004
#define BCM2835_SPI_CS_CS_10		0x00000002
#define BCM2835_SPI_CS_CS_01		0x00000001

#define SPI_TIMEOUT_MS	3000
#define BCM2835_SPI_MODE_BITS	(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_NO_CS)

#define BCM2835_SPI_GPIO_MISO    9
#define BCM2835_SPI_GPIO_MOSI   10
#define BCM2835_SPI_GPIO_SCK    11
#define BCM2835_SPI_MAX_CS	53

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


/* the DMA channel definition and functions */
#include <mach/dma.h>

/* the structure that defines the DMAs we use */
struct bcm2835_dmachannel {
	void __iomem *base;
	dma_addr_t bus_addr;
        int chan;
        int irq;
	irq_handler_t handler;
	const char *desc;
};

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
	writel(0,d->base+BCM2835_DMA_CB_ADDR);
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
	writel(0,d->base+BCM2835_DMA_CB_ADDR);

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

static void bcm2835_dmachannel_dump(struct bcm2835_dmachannel *dma)
{
	printk(KERN_DEBUG " DMA[%2s].base     = %pK\n",dma->desc,dma->base);
	printk(KERN_DEBUG "        .bus_addr = %08x\n",dma->bus_addr);
	printk(KERN_DEBUG "        .channel  = %i\n",dma->chan);
	printk(KERN_DEBUG "        .irq      = %i\n",dma->irq);
	printk(KERN_DEBUG "        .handler  = %pS\n",dma->handler);
        printk(KERN_DEBUG "        .status   = %08x\n",
		readl(dma->base+BCM2835_DMA_CS));
        printk(KERN_DEBUG "        .cbaddr   = %08x\n",
		readl(dma->base+BCM2835_DMA_CB_ADDR));

#if 0
	bcm2835_dma_dump_cb(
		(struct bcm2835_dma_cb*)(dma->base+BCM2835_DMA_TI)
		);
#endif
        printk(KERN_DEBUG "        .debug    = %08x\n",
		readl(dma->base+BCM2835_DMA_DEBUG));
}

/* the SPI controller structures */

struct bcm2835dma_spi {
	/* the SPI registers */
	void __iomem *spi_regs;
	/* the clock */
	struct clk *clk;
	/* the DMA channels allocated */
	struct bcm2835_dmachannel dma_tx;
	struct bcm2835_dmachannel dma_rx;
	/* the DMA-able pool we use to allocate control blocks from */
	struct dma_pool *pool;
	/* some DMA able blocks for some read/write buffers */
	struct {
		void *addr;
		dma_addr_t bus_addr;
	} buffer_write_dummy,buffer_read_0x00;
	/* the fragment caches */
	struct DMAFragmentCache fragment_setup_spi_plus_transfer;
	struct DMAFragmentCache fragment_transfer;
};

struct DMAFragment *bcm2835_dmafragment_create_setup_spi_plus_transfer(struct dma_pool *,gfp_t);
struct DMAFragment *bcm2835_dmafragment_create_transfer(struct dma_pool *,gfp_t);

/* the interrupt-handlers */
static irqreturn_t bcm2835dma_spi_interrupt_dma_tx(int irq, void *dev_id);
//static irqreturn_t bcm2835dma_spi_interrupt_dma_rx(int irq, void *dev_id);

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
	bs->buffer_write_dummy.addr=
		dma_pool_alloc(bs->pool,GFP_KERNEL,
			&bs->buffer_write_dummy.bus_addr);

	bs->buffer_read_0x00.addr=
		dma_pool_alloc(bs->pool,GFP_KERNEL,
			&bs->buffer_read_0x00.bus_addr);
	if (bs->buffer_read_0x00.addr)
		memset(bs->buffer_read_0x00.addr,0x00,
			sizeof(*bs->buffer_read_0x00.addr));

	/* initialize DMA Fragment pools */
	DMAFragmentCache_initialize(&bs->fragment_setup_spi_plus_transfer,
				"setup_spi_plus_transfer",
				&bcm2835_dmafragment_create_setup_spi_plus_transfer,
				bs->pool,
				5
		);
	DMAFragmentCache_initialize(&bs->fragment_transfer,
				"transfer",
				&bcm2835_dmafragment_create_transfer,
				bs->pool,
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
		bs->buffer_read_0x00.addr,bs->buffer_read_0x00.bus_addr);
	dma_pool_free(bs->pool,
		bs->buffer_write_dummy.addr,bs->buffer_write_dummy.bus_addr);

	DMAFragmentCache_release(&bs->fragment_setup_spi_plus_transfer);
	DMAFragmentCache_release(&bs->fragment_transfer);

	dma_pool_destroy(bs->pool);
        bs->pool=NULL;
}

static void bcm2835dma_dump_spi(struct spi_master* master) {
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
        printk(KERN_DEBUG"  SPI-REGS:\n");
        printk(KERN_DEBUG"    SPI-CS:   %08x\n",
		readl(bs->spi_regs + BCM2835_SPI_CS));
        printk(KERN_DEBUG"    SPI-CLK:  %08x\n",
		readl(bs->spi_regs + BCM2835_SPI_CLK));
        printk(KERN_DEBUG"    SPI-DLEN: %08x\n",
		readl(bs->spi_regs + BCM2835_SPI_DLEN));
        printk(KERN_DEBUG"    SPI-LOTH: %08x\n",
		readl(bs->spi_regs + BCM2835_SPI_LTOH));
        printk(KERN_DEBUG"    SPI-DC:   %08x\n",
		readl(bs->spi_regs + BCM2835_SPI_DC));
}

static void spi_print_debug_message(struct spi_message *mesg,u32 max_dump_len)
{
	struct spi_transfer *xfer;
	/* some statistics to gather */
	u32 transfers=0;
	u32 transfers_len=0;
	u32 transfers_tx_len=0;
	u32 transfers_rx_len=0;
	/* first general */
	dev_info(&mesg->spi->dev,"SPI message:\n");
	printk(KERN_DEBUG " msg.status         = %i\n",mesg->status);
	printk(KERN_DEBUG "    .actual_length  = %i\n",mesg->actual_length);
	printk(KERN_DEBUG "    .is_dma_mapped  = %i\n",mesg->is_dma_mapped);
	printk(KERN_DEBUG "    .complete       = %pf\n",mesg->complete);
	printk(KERN_DEBUG "    .context        = %pK\n",mesg->context);
	/* now iterate over list again */
	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
		/* the max data-length to print */
		u32 dump_len=(xfer->len<max_dump_len)?xfer->len:max_dump_len;
		/* first some stats */
		transfers++;
		transfers_len+=xfer->len;
		if (xfer->tx_buf)
			transfers_tx_len+=xfer->len;
		if (xfer->rx_buf)
			transfers_rx_len+=xfer->len;
		/* now write out details for this transfer */
		printk(KERN_DEBUG " xfer[%02i].len           = %i\n",
			transfers,xfer->len);
		printk(KERN_DEBUG "         .speed_hz      = %i\n",
			xfer->speed_hz);
		printk(KERN_DEBUG "         .delay_usecs   = %i\n",
			xfer->delay_usecs);
		printk(KERN_DEBUG "         .cs_change     = %i\n",
			xfer->cs_change);
		printk(KERN_DEBUG "         .bits_per_word = %i\n",
			xfer->bits_per_word);
		printk(KERN_DEBUG "         .tx_buf        = %pK\n",
			xfer->tx_buf);
		printk(KERN_DEBUG "         .tx_dma        = %08x\n",
			xfer->tx_dma);
		if ((xfer->tx_buf)&&(dump_len)) {
			print_hex_dump(KERN_DEBUG,
				"         .tx_data       = ",
				DUMP_PREFIX_ADDRESS,
				32,4,
				xfer->tx_buf,
				dump_len,
				false
				);
		}
		printk(KERN_DEBUG "         .rx_buf        = %pK\n",
			xfer->rx_buf);
		printk(KERN_DEBUG "         .rx_dma        = %08x\n",
			xfer->rx_dma);
		if ((xfer->rx_buf)&&(dump_len)) {
			print_hex_dump(KERN_DEBUG,
				"         .rx_data       = ",
				DUMP_PREFIX_ADDRESS,
				32,4,
				xfer->rx_buf,
				dump_len,
				false
				);
		}
	}
	printk(KERN_DEBUG " msg.transfers      = %i\n",transfers);
	printk(KERN_DEBUG "    .total_len      = %i\n",transfers_len);
	printk(KERN_DEBUG "    .tx_length      = %i\n",transfers_tx_len);
	printk(KERN_DEBUG "    .rx_length      = %i\n",transfers_rx_len);
}

static void bcm2835dma_dump_state(struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	bcm2835dma_dump_spi(master);
	dev_info(&master->dev,"DMA status:\n");
	bcm2835_dmachannel_dump(&bs->dma_rx);
	bcm2835_dmachannel_dump(&bs->dma_tx);
}

static irqreturn_t bcm2835dma_spi_interrupt_dma_tx(int irq, void *dev_id)
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

#ifdef CONFIG_MACH_BCM2708

static void bcm2835dma_set_gpio_mode(u8 pin,u32 mode) {
	/* TODO - PINMUX */
	u32 *gpio = ioremap(0x20200000, SZ_16K);

	/* map pin */
	u32 *reg = &gpio[pin/10];
	u8 shift = ((pin)%10)*3;
	u32 v = *reg;
	v &= ~( ((u32)(7)) << shift );
	v |= (mode & 7) << shift;
	*reg= v;
	printk(KERN_INFO "pin %i mode %i\n",pin,mode);
	printk(KERN_INFO "ADDR %08x %2i %010o %010o %010o\n",(u32)reg,shift,*reg,v,*reg);
	iounmap(gpio);
}

#endif

static void bcm2835dma_spi_init_pinmode(void) {
	/* SET modes to ALT0=4 */
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MISO,4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MOSI,4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_SCK, 4);
}

static int bcm2835dma_spi_setup(struct spi_device *spi) {
	struct bcm2835dma_spi *bs = spi_master_get_devdata(spi->master);
	u8 cs = spi->chip_select;
	u32 mode = spi->mode;

	/* map cs=0 and cs=1 to the correct ones for the RPI */
	if (cs == 0)
		cs = 8;
	if (cs == 1)
		cs = 7;

	/* set the "correct" cs level */
	if (!(mode & SPI_NO_CS)) {
	} else {
		/* check cs for prohibited pins */
		if (
			(cs == BCM2835_SPI_GPIO_MISO)
			||
			(cs == BCM2835_SPI_GPIO_MOSI)
			||
			(cs == BCM2835_SPI_GPIO_SCK)
			) {
			dev_err(&spi->dev, "Chipselect GPIO %i is not allowed as it is already used\n",cs);
			return -EPERM;
		}
		/* set pin mode as Output */
		/* and set the polarity */
		if (mode & SPI_CS_HIGH) {
			/* configure the GPIO as low and output */
			gpio_direction_output(cs,0);
		} else {
			/* configure the GPIO as high and output */
			gpio_direction_output(cs,1);
		}
	}
	return 0;
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
	master->dev.of_node = pdev->dev.of_node;
	master->rt = 1;

	master->transfer = bcm2835dma_spi_transfer;

	bs = spi_master_get_devdata(master);

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

#ifdef CONFIG_MACH_BCM2708
	/* configure pin function for SPI */
	bcm2835dma_spi_init_pinmode();
#endif

	err=bcm2835dma_allocate_dma(master,pdev);
	if (err) {
		dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
		goto out_release_dma;
	}

	/* initialise the hardware */
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

	/* release pool - also releases all objects */
	bcm2835dma_release_dma(master);

	/* Clear FIFOs, and disable the HW block */
	if (bs->spi_regs)
		writel(
			BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX,
			bs->spi_regs+BCM2835_SPI_CS);
	
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
