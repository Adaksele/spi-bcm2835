/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
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
#include <linux/dmapool.h>

/* SPI register offsets */
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

#define BCM2835_SPI_NUM_CS	3

#define DRV_NAME	"spi-bcm2835dma"

static bool realtime = 1;
module_param(realtime, bool, 0);
MODULE_PARM_DESC(realtime, "Run the driver with realtime priority");

/* some module-parameters for debugging and corresponding */
static bool debug_msg = 0;
module_param(debug_msg, bool, 0);
MODULE_PARM_DESC(realtime, "Run the driver with message debugging enabled");

static bool debug_dma = 0;
module_param(debug_dma, bool, 0);
MODULE_PARM_DESC(realtime, "Run the driver with dma debugging enabled");


/* the layout of the DMA register itself - use writel/readl for it to work correctly */
struct bcm2835_dma_regs {
	/* the dma control registers */
	u32 cs;
	dma_addr_t addr;
	u32 ti;
	dma_addr_t src;
	dma_addr_t dst;
	u32 len;
	u32 stride;
	dma_addr_t next;
	u32 debug;
};

/* the structure that defines the DMAs we use */
struct bcm2835dma_spi_dma {
	/* some extra information */
	struct bcm2835_dma_regs __iomem *base;
        int chan;
        int irq;
	char *desc;
};

/* the spi device data structure */
struct bcm2835dma_spi {
	void __iomem *regs;
	struct clk *clk;
	struct completion done;
	/* the chip-select flags that store the polarities and other flags for transfer */
	u32 cs_device_flags_idle;
	u32 cs_device_flags[BCM2835_SPI_NUM_CS];
	/* the listhead of control blocks we have allocated*/
	struct list_head cb_list;
	/* the dmas we require */
	struct bcm2835dma_spi_dma dma_tx;
	struct bcm2835dma_spi_dma dma_rx;
	/* the DMA-able pool we use to allocate control blocks from */
	struct dma_pool *pool;
	/* some helper segments from the dma pool used for transfers */
};

/* the control block structure - this should be 64 bytes in size*/
struct bcm2835dma_dma_cb {
	/* first the "real" control-block used by the DMA - 32 bytes */
	u32 info;
	dma_addr_t src;
	dma_addr_t dst;
	u32 length;
	u32 stride;
	dma_addr_t next;
	u32 padding[2];
	/* the list of control-blocks*/
	struct list_head cb_list;
	/* the physical address */
	dma_addr_t self;
	/* if length is <=8 then the union below is used directly (word), 
	 * otherwise we allocate a frame and use the address in pool_ptr */
	union {
		u32 word[2];
		void *pool_ptr;
	} src_data;
	union {
		u32 word[2];
		void *pool_ptr;
	} dst_data;
	/* finally there is the pointer to the corresponding SPI message - used for callbacks,... */
	struct spi_message* msg;
};


/* the following are in need of a rewrite to get them use the
   linux-dma-manager instead */
#include <mach/dma.h>
static void bcm2835dma_release_dma(struct platform_device *pdev,
			struct bcm2835dma_spi_dma *d)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	if (!d->base)
                return;
	if (d->irq)
		free_irq(d->irq,master);
        bcm_dma_chan_free(d->chan);
        d->base = NULL;
        d->chan = 0;
        d->irq = 0;
}

static int bcm2835dma_allocate_dma(struct platform_device *pdev,
                                struct bcm2835dma_spi_dma *d,
				irq_handler_t handler
	)
{
	struct spi_master *master = platform_get_drvdata(pdev);
        int ret;
	/* fill in defaults */
	d->base=NULL;
	d->chan=0;
	d->irq=0;
        /* register DMA channel */
        ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST, (void**)&d->base, &d->irq);
        if (ret < 0) {
		d->base=NULL;
		d->chan=0;
		d->irq=0;
                dev_err(&pdev->dev, "couldn't allocate a DMA channel\n");
                return ret;
        }
        d->chan = ret;
        dev_info(&pdev->dev, "DMA channel %d at address %pK with irq %d and handler at %pf\n",
                d->chan, d->base, d->irq,handler);
	/* and the irq handler */
	if (handler) {
		ret = request_irq(d->irq,
				handler,
				0,
				dev_name(&pdev->dev),
				master);
		if (ret) {
			dev_err(&pdev->dev, "could not request IRQ: %d\n", d->irq);
			bcm2835dma_release_dma(pdev,d);
			return ret;
		}
	} else {
		/* clear irq, as we do not use it */
		d->irq=0;
	}
        return 0;
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
		printk(KERN_DEBUG " xfer[%02i].len           = %i\n",transfers,xfer->len);
		printk(KERN_DEBUG "         .speed_hz      = %i\n",xfer->speed_hz);
		printk(KERN_DEBUG "         .delay_usecs   = %i\n",xfer->delay_usecs);
		printk(KERN_DEBUG "         .cs_change     = %i\n",xfer->cs_change);
		printk(KERN_DEBUG "         .bits_per_word = %i\n",xfer->bits_per_word);
		printk(KERN_DEBUG "         .tx_buf        = %pK\n",xfer->tx_buf);
		printk(KERN_DEBUG "         .tx_dma        = %08x\n",xfer->tx_dma);
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
		printk(KERN_DEBUG "         .rx_buf        = %pK\n",xfer->rx_buf);
		printk(KERN_DEBUG "         .rx_dma        = %08x\n",xfer->rx_dma);
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
	/* the statistics - only after the fact - to avoid unecessary debugging statistics code in the default path */
	printk(KERN_DEBUG " msg.transfers      = %i\n",transfers);
	printk(KERN_DEBUG "    .total_len      = %i\n",transfers_len);
	printk(KERN_DEBUG "    .tx_length      = %i\n",transfers_tx_len);
	printk(KERN_DEBUG "    .rx_length      = %i\n",transfers_rx_len);
}

static inline u32 bcm2835dma_rd(struct bcm2835dma_spi *bs, unsigned reg)
{
	return readl(bs->regs + reg);
}

static inline void bcm2835dma_wr(struct bcm2835dma_spi *bs, unsigned reg, u32 val)
{
	writel(val, bs->regs + reg);
}

static irqreturn_t bcm2835dma_spi_interrupt(int irq, void *dev_id) {
	struct spi_master *master = dev_id;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	/* wake up task */
	complete(&bs->done);

	/* and return with the IRQ marked as handled */
	return IRQ_HANDLED;
}

/* most likley we will need to move away from the transfer_one at a time approach, if we want to pipeline the Transfers.. */
static int bcm2835dma_spi_transfer_one(struct spi_master *master,
		struct spi_message *mesg)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct spi_transfer *xfer;
	struct spi_device *spi = mesg->spi;
	/* the status */
	u32 status=0;
	/* the default cs and speed values*/
	u32 last_cdiv=0;
	u32 last_cs=0;
	/* the spi bus speed */
	u32 clk_hz = clk_get_rate(bs->clk);

	/* loop all transfers */
	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
		/* the chip-select and cdiv */
		u32 cs=bs->cs_device_flags[spi->chip_select];
		/* calculate cdiv */
		u32 speed_hz=(xfer->speed_hz)?xfer->speed_hz:spi->max_speed_hz;
		u32 cdiv=2; /* by default the max is half the BUS speed */
		if (speed_hz*2<clk_hz) {
			if (speed_hz) {
				cdiv=DIV_ROUND_UP(clk_hz,speed_hz);
				/* actually the document says that cdiv must be a power of 2,
				   but empirically (found out by notro) this is found to be not true, so not included:
				   cdiv = roundup_pow_of_two(cdiv);
				*/
			} else { 
				cdiv=0;
			}
			/* if the ratio is too big, then use the slowest we can go... */
			if (cdiv>65535)
				cdiv=0; /* the slowest we can go */
		}
		/* check if bits/word have changed */
		if ((xfer->bits_per_word)&&(xfer->bits_per_word!=8)) {
			dev_err(&spi->dev,"Unsupported bits/word in transfer config - only 8 bits are allowed!!!");
			status=-EINVAL;
			goto error_exit;
		}
		/* if the values have changed, then set them - we may want to consolidate both into one transfer with strides */
		if (cdiv!=last_cdiv) {
			/* configure cdiv via DMA */
		}
		if (cs!=last_cs) {
			/* configure cs via DMA */
		}
		/* set the "old" values to the current ones */
		last_cdiv=cdiv;
		last_cs=cs;
	}

	/* wait for us to get woken up again after the transfer */
        if (wait_for_completion_timeout(
                        &bs->done,
                        msecs_to_jiffies(SPI_TIMEOUT_MS)) == 0) {
		dev_err(&master->dev,"DMA transfer timed out\n");
		status=-ETIMEDOUT;
	}

	/* set the status - before we run the debug code and before we finalize the message */
error_exit:
	mesg->status=status;

	/* write the debug message information */
	if (unlikely(debug_msg)) {
		spi_print_debug_message(mesg,128);
	}

	/* finalize message */
	spi_finalize_current_message(master);
	/* and return */
	return status;
}

#ifdef CONFIG_MACH_BCM2708
static void bcm2835dma_spi_init_pinmode(void) {
	/* taken from spi-bcm2708.c, where it says: */
/*
 * This function sets the ALT mode on the SPI pins so that we can use them with
 * the SPI hardware.
 *
 * FIXME: This is a hack. Use pinmux / pinctrl.
 */
	/* maybe someone has an Idea how to fix this... */
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

	int pin;
	u32 *gpio = ioremap(0x20200000, SZ_16K);

	/* SPI is on GPIO 7..11 */
	for (pin = 7; pin <= 11; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
	}

	iounmap(gpio);

#undef INP_GPIO
#undef SET_GPIO_ALT
}
#endif

static int bcm2835dma_spi_setup(struct spi_device *spi) {
	struct bcm2835dma_spi *bs = spi_master_get_devdata(spi->master);
	u8 cs = spi->chip_select;
	u32 mode = spi->mode;

	/* fill in cs flags based on device configs*/
	if (!(mode & SPI_NO_CS)) {
		/* if we are not configured with CS_HIGH */
		if (mode & SPI_CS_HIGH) {
			int i;
			/* fill in the flags for all devices */
			for (i=0;i<=BCM2835_SPI_NUM_CS;i++) {
				bs->cs_device_flags[i] |= BCM2835_SPI_CS_CSPOL0 << cs;
			}
			/* the idle mode as well */
			bs->cs_device_flags_idle |= BCM2835_SPI_CS_CSPOL0 << cs;
			/* and the specific flag for this device */
			bs->cs_device_flags[cs] |= BCM2835_SPI_CS_CSPOL;
		}
		bs->cs_device_flags[cs] |= spi->chip_select;
	}
	/* and set up the other stuff */ 
	if (mode & SPI_CPOL)
		bs->cs_device_flags[cs] |= BCM2835_SPI_CS_CPOL;
	if (mode & SPI_CPHA)
		bs->cs_device_flags[cs] |= BCM2835_SPI_CS_CPHA;
	/* we could fail this device here immediately for 8 bit */
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
#else
	master->bus_num = -1;
#endif
	master->num_chipselect = BCM2835_SPI_NUM_CS;
	master->setup = bcm2835dma_spi_setup;
	master->transfer_one_message = bcm2835dma_spi_transfer_one;
	master->dev.of_node = pdev->dev.of_node;
	master->rt = realtime;

	bs = spi_master_get_devdata(master);

	init_completion(&bs->done);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "could not get memory resource\n");
		err = -ENODEV;
		goto out_master_put;
	}

	bs->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!bs->regs) {
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

	/* allocate DMA */
	if ((err=bcm2835dma_allocate_dma(pdev,&bs->dma_tx,NULL))) {
		goto out_release_dma;
	}
	if ((err=bcm2835dma_allocate_dma(pdev,&bs->dma_rx,NULL))) {
		goto out_release_dma;
	}

	/* allocate pool */
	bs->pool=dma_pool_create(
                DRV_NAME "_dmapool",
                &pdev->dev,
                sizeof(struct bcm2835dma_dma_cb),
                64, 
                0 
                );
	if (!bs->pool) {
		dev_err(&pdev->dev, "could not allocate memory pool\n");
		err = -ENOMEM;
		goto out_clk_disable;
	}
	/* allocate some pages from pool for "standard" pages */


#ifdef CONFIG_MACH_BCM2708
	/* configure pin function for SPI */
	bcm2835dma_spi_init_pinmode();
#endif

	/* initialise the hardware */
	bcm2835dma_wr(bs, BCM2835_SPI_CS,
		   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);

	err = spi_register_master(master);
	if (err) {
		dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
		goto out_release_pool;
	}

	return 0;
out_release_pool:
	dma_pool_destroy(bs->pool);
        bs->pool=NULL;
out_release_dma:
	bcm2835dma_release_dma(pdev,&bs->dma_tx);
	bcm2835dma_release_dma(pdev,&bs->dma_rx);
out_clk_disable:
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
	dma_pool_destroy(bs->pool);
        bs->pool=NULL;
	
	/* Clear FIFOs, and disable the HW block */
	bcm2835dma_wr(bs, BCM2835_SPI_CS,
		   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);

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
MODULE_AUTHOR("Chris Boot <bootc@bootc.net>, Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL v2");
