/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2012 Chris Boot, Martin Sperl
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <linux/moduleparam.h>

static bool realtime = 1;
module_param(realtime, bool, 0);
MODULE_PARM_DESC(realtime, "Run the driver with realtime priority");

static bool debugio = 0;
module_param(debugio, bool, 0);
MODULE_PARM_DESC(debugio, "debug the messages of the specific CS");

static bool debugdma = 1;
module_param(debugdma, bool, 0);
MODULE_PARM_DESC(debugdma, "debug the DMA setup");

static unsigned debug=0;
module_param(debug, uint, 0);
MODULE_PARM_DESC(debug, "Turn on debug output");

#undef dev_dbg
#define dev_dbg(dev, fmt, args...)           \
do {                                         \
	if (unlikely(debug > 0))             \
		dev_info(dev, fmt, ##args);  \
} while (0)

/* SPI register offsets */
#define SPI_CS			0x00
#define SPI_FIFO		0x04
#define SPI_CLK			0x08
#define SPI_DLEN		0x0c
#define SPI_LTOH		0x10
#define SPI_DC			0x14

/* Bitfields in CS */
#define SPI_CS_LEN_LONG		0x02000000
#define SPI_CS_DMA_LEN		0x01000000
#define SPI_CS_CSPOL2		0x00800000
#define SPI_CS_CSPOL1		0x00400000
#define SPI_CS_CSPOL0		0x00200000
#define SPI_CS_RXF		0x00100000
#define SPI_CS_RXR		0x00080000
#define SPI_CS_TXD		0x00040000
#define SPI_CS_RXD		0x00020000
#define SPI_CS_DONE		0x00010000
#define SPI_CS_LEN		0x00002000
#define SPI_CS_REN		0x00001000
#define SPI_CS_ADCS		0x00000800
#define SPI_CS_INTR		0x00000400
#define SPI_CS_INTD		0x00000200
#define SPI_CS_DMAEN		0x00000100
#define SPI_CS_TA		0x00000080
#define SPI_CS_CSPOL		0x00000040
#define SPI_CS_CLEAR_RX		0x00000020
#define SPI_CS_CLEAR_TX		0x00000010
#define SPI_CS_CPOL		0x00000008
#define SPI_CS_CPHA		0x00000004
#define SPI_CS_CS_10		0x00000002
#define SPI_CS_CS_01		0x00000001

#define SPI_TIMEOUT_MS	150

#define DRV_NAME	"bcm2835_spi"

#define FLAGS_FIRST_TRANSFER 0x01
#define FLAGS_LAST_TRANSFER  0x02

/* the defines that are missing in arch/arm/mach-bcm2708/include/mach/dma.h */
/* the Base address for DMA on the (VideoCore) bus */
#define DMA_SPI_BASE 0x7E204000

/* some flags */
#ifndef BCM2708_DMA_D_IGNORE
#define BCM2708_DMA_D_IGNORE (1<<7)
#endif
#ifndef BCM2708_DMA_S_IGNORE
#define BCM2708_DMA_S_IGNORE (1<<11)
#endif

struct bcm2835_spi_dma {
	int chan;
	int irq;
	void __iomem *base;
};

struct bcm2835_spi {
	spinlock_t lock;
	void __iomem *base;
	struct clk *clk;
	bool stopping;

	struct completion done;

	/* dma buffer structures */
	/* the dma region for CBs */
	struct bcm2708_dma_cb *dma_buffer;
	dma_addr_t dma_buffer_handle;

	/* the dma bounce buffer */
	void *dma_bouncebuffer;
	dma_addr_t dma_bouncebuffer_handle;

	/* and the information on the DMA channels we use */
	struct bcm2835_spi_dma dma_tx;
	struct bcm2835_spi_dma dma_rx;

	/* structures from the transfer buffer needed during the transfer */
	const char *tx_buf;
	int tx_len;
	char *rx_buf;
	int rx_len;
	int cs;
};

struct bcm2835_spi_state {
	u32 cs;
	u16 cdiv;
	u8 bits_per_word;
	u32 speed_hz;
	u32 csel;
	u8 mode;
};

/*
 * This function sets the ALT mode on the SPI pins so that we can use them with
 * the SPI hardware.
 *
 * FIXME: This is a hack. Use pinmux / pinctrl.
 */
static void bcm2835_init_pinmode(void)
{
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g, a) *(gpio+(((g)/10))) |= (((a) <= 3 ? (a)+4 : (a) == 4 ? 3 : 2)<<(((g)%10)*3))

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

static inline u32 bcm2835_rd(struct bcm2835_spi *bs, unsigned reg)
{
	return readl(bs->base + reg);
}

static inline void bcm2835_wr(struct bcm2835_spi *bs, unsigned reg, u32 val)
{
	writel(val, bs->base + reg);
}

static int bcm2835_setup_state(struct spi_master *master,
			struct device *dev, struct bcm2835_spi_state *state,
			u32 hz, u8 csel, u8 mode, u8 bpw)
{
	struct bcm2835_spi *bs = spi_master_get_devdata(master);
	int cdiv;
	unsigned long bus_hz;
	u32 cs = 0;

	bus_hz = clk_get_rate(bs->clk);

	if (hz >= bus_hz) {
		cdiv = 2; /* bus_hz / 2 is as fast as we can go */
	} else if (hz) {
		cdiv = DIV_ROUND_UP(bus_hz, hz);

		if (cdiv > 65536) {
			dev_err(dev,
				"setup: %d Hz too slow, cdiv %u; min %ld Hz\n",
				hz, cdiv, bus_hz / 65536);
			return -EINVAL;
		} else if (cdiv == 65536) {
			cdiv = 0;
		} else if (cdiv == 1) {
			cdiv = 2; /* 1 gets rounded down to 0; == 65536 */
		}
	} else {
		cdiv = 0;
	}

	switch (bpw) {
	case 8:
		break;
	default:
		dev_err(dev, "setup: invalid bits_per_word %u (must be 8)\n",
			bpw);
		return -EINVAL;
	}

	if (mode & SPI_CPOL)
		cs |= SPI_CS_CPOL;
	if (mode & SPI_CPHA)
		cs |= SPI_CS_CPHA;

	if (!(mode & SPI_NO_CS)) {
		if (mode & SPI_CS_HIGH) {
			cs |= SPI_CS_CSPOL;
			cs |= SPI_CS_CSPOL0 << csel;
		}
		cs |= csel;
	} else {
		cs |= SPI_CS_CS_10 | SPI_CS_CS_01;
	}

	if (state) {
		state->cs = cs;
		state->cdiv = cdiv;
		state->speed_hz=hz;
		state->bits_per_word=bpw;
		state->csel=csel;
		state->mode=mode;
	}

	dev_dbg(dev, "%s(hz=%d, csel=%d, mode=0x%02X, bpw=%d) => "
		"bus_hz=%lu / cdiv=%u == %lu Hz; cs 0x%08X\n",
		__func__, hz, csel, mode, bpw, bus_hz, cdiv, bus_hz/cdiv, cs);

	return 0;
}

static int bcm2835_register_dma(struct platform_device *pdev,
				struct bcm2835_spi_dma *d,
				struct bcm2708_dma_cb *dmabuffer,
				const char *name)
{
	int ret;

	/* register DMA channel */
	ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST, &d->base, &d->irq);
	if (ret < 0) {
		dev_err(&pdev->dev, "couldn't allocate a DMA channel\n");
		return ret;
	}
	d->chan = ret;
	dev_info(&pdev->dev, "DMA channel %d at address 0x%08lx with irq %d\n",
		d->chan, (unsigned long)d->base, d->irq);
	return 0;
}

static int bcm2835_release_dma(struct platform_device *pdev,
			struct bcm2835_spi_dma *d)
{
	if (!d->base)
		return 0;
	bcm_dma_chan_free(d->chan);
	d->base = NULL;
	d->chan = 0;
	d->irq = 0;
	return 0;
}

static int bcm2835_register_dmabuffer(struct platform_device *pdev,
				struct bcm2835_spi *bs)
{
	/* for this to work you need to have set the following:
	   in the bcm2835_spi_device definition:
	   .dev = {
	   .coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON),
	   },
	   otherwise you get the message:
	   coherent DMA mask is unset
	   and the allocation fails...
	   learned the hard way, so as a hint for all
	   who take this as a base...
	*/
	bs->dma_buffer = dma_alloc_writecombine(&pdev->dev,
					SZ_4K,
					&bs->dma_buffer_handle,
					GFP_KERNEL);
	if (!bs->dma_buffer) {
		dev_err(&pdev->dev, "cannot allocate DMA CBs\n");
		return -ENOMEM;
	}
	/* allocate the dma bounce buffer */
	bs->dma_bouncebuffer = dma_alloc_writecombine(&pdev->dev,
					SZ_4K,
					&bs->dma_bouncebuffer_handle,
					GFP_KERNEL);
	if (!bs->dma_bouncebuffer) {
		dev_err(&pdev->dev, "cannot allocate DMA bounce-buffer\n");
		dma_free_writecombine(&pdev->dev, SZ_4K,
				bs->dma_buffer,
				bs->dma_buffer_handle);
		bs->dma_buffer = NULL;
		bs->dma_buffer_handle = 0;
		return -ENOMEM;
	}
	
	return 0;
}

static int bcm2835_release_dmabuffer(struct platform_device *pdev,
				struct bcm2835_spi *bs)
{
	/* release the dma buffer */
	if (!bs->dma_buffer)
		return 0;
	dma_free_writecombine(&pdev->dev, SZ_4K,
			bs->dma_buffer,
			bs->dma_buffer_handle);
	bs->dma_buffer = NULL;
	bs->dma_buffer_handle = 0;
	/* release also the DMA bounce buffer */
	dma_free_writecombine(&pdev->dev, SZ_4K,
			bs->dma_bouncebuffer,
			bs->dma_bouncebuffer_handle);
	bs->dma_bouncebuffer = NULL;
	bs->dma_bouncebuffer_handle = 0;
	return 0;
}

irqreturn_t bcm2835_transfer_one_message_dma_irqhandler(int irq, void *dev)
{
	struct spi_master *master = dev;
	struct bcm2835_spi *bs = spi_master_get_devdata(master);

	/* mark the rx DMA-interrupt as handled
	   - it will (level) trigger otherwise again */
	writel(BCM2708_DMA_INT, bs->dma_rx.base+BCM2708_DMA_CS);

	/* and wake up the thread to continue its work - returning ...*/
	complete(&bs->done);
	/* return IRQ handled */
	return IRQ_HANDLED;
}

static int bcm2835_transfer_one_message(struct spi_master *master,
					struct spi_message *msg)
{
	struct bcm2835_spi *bs = spi_master_get_devdata(master);
	struct spi_transfer *xfer;
	struct spi_device *spi = msg->spi;
	struct bcm2835_spi_state state;
	int status = 0;
	int transfers = 0;
	int totallen=0;
	int cbs_pos=0;
	int last_tx_pos=0;
	int last_rx_pos=0;
	/* the pointer to the control blocks - ignore the first page */
	struct bcm2708_dma_cb *cbs = &bs->dma_buffer[1];
	u32 *dma_cs=(u32*) bs->dma_buffer;

	/* get the settings from the controller */
	u8 bits_per_word=((struct bcm2835_spi_state *)spi->controller_state)->bits_per_word;
	u32 speed_hz=((struct bcm2835_spi_state *)spi->controller_state)->speed_hz; 

	/* get some basic stats on the transfers
	 * * total length
	 * * speed
	 * * bits
	 */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* check for change in speed,... */
		if (xfer->bits_per_word) {
			/* we allow such changes ONLY for the first transfer */
			if (transfers) {
				if (xfer->bits_per_word!=bits_per_word) {
					dev_err(&master->dev, "change of speed/bits only allowed for the first transfer\n");
					return -EINVAL;
				}
			} else {
				bits_per_word=xfer->bits_per_word;
			}
		}
		if (xfer->speed_hz) {
			/* we allow such changes ONLY for the first transfer */
			if (transfers) {
				if (xfer->speed_hz!=speed_hz) {
					dev_err(&master->dev, "change of speed/bits only allowed for the first transfer\n");
					return -EINVAL;
				}
			} else {
				speed_hz=xfer->speed_hz;
			}
		}
		totallen+=xfer->len;
		transfers++;
	}
	/* we may not exceed the max transfer size - 64k */
	if (xfer->len >= 65536) {
		dev_err(&master->dev, "Max allowed package size 64k exceeded");
		return -EINVAL;
	}
	/* calculate our effective cs values */
	state.cs = ((struct bcm2835_spi_state *)spi->controller_state)->cs;
	state.cdiv = ((struct bcm2835_spi_state *)spi->controller_state)->cdiv;
	status = bcm2835_setup_state(spi->master, &spi->dev,
				&state,
				speed_hz,
				spi->chip_select,
				spi->mode,
				bits_per_word);
	if (status)
		goto exit;
	
	/* clear the queues */
	bcm2835_wr(bs, SPI_CS, bcm2835_rd(bs, SPI_CS) |
		SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX);


	/* now set up the Registers */
	bcm2835_wr(bs, SPI_CLK, state.cdiv);
	bcm2835_wr(bs, SPI_CS, state.cs);

	/* now set up the CS value in DMA space*/
	*dma_cs=totallen<<16   /* the length in bytes to transfer */
		| (state.cs & 0xff) /* the bottom 8 bit flags for the SPI interface */
		| SPI_CS_TA;  /* and enable transfer */


	/* the macro that helps us calculate the addresses */
#define RAM2PHY(addr,base_logic,base_phy) (base_phy+((u32)addr-(u32)base_logic))


	/* fill in the first transfer to configure SPI */
	/* tx info - set len/flags in the first CB */
	cbs[cbs_pos].info = BCM2708_DMA_PER_MAP(6) /* DREQ 6 = SPI TX in PERMAP */
		| BCM2708_DMA_D_DREQ; /* destination DREQ trigger */
	cbs[cbs_pos].src = bs->dma_buffer_handle;
	cbs[cbs_pos].dst = (unsigned long)(DMA_SPI_BASE + SPI_FIFO);
	cbs[cbs_pos].length = 4;
	cbs[cbs_pos].stride = 0;
	cbs[cbs_pos].next = (u32) 0;
	last_tx_pos=cbs_pos;

	/* and register as start of transfer */
	writel(RAM2PHY(&cbs[cbs_pos],bs->dma_buffer,bs->dma_buffer_handle),
		bs->dma_tx.base + BCM2708_DMA_ADDR
		);
	/* and increment */
	cbs_pos++;

	/* now loop the transfers */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* todo: 
		 * * bounce buffer or address translation 
		 * * check if we need to split this transfer, 
		 *   because it is split to non-adjectant pages
		 */
		/* setting xfer-dma */
		if ((xfer->rx_buf) && (!xfer->rx_dma)) {
			xfer->rx_dma=bs->dma_bouncebuffer_handle;
			memset(bs->dma_bouncebuffer,0,xfer->len);
		}
		if ((xfer->tx_buf) && (!xfer->tx_dma)) {
			xfer->tx_dma=bs->dma_bouncebuffer_handle+1024;
			memcpy(bs->dma_bouncebuffer+1024,xfer->tx_buf,xfer->len);
		}

		/* fill in tx */
		cbs[cbs_pos].info = cbs[last_tx_pos].info | BCM2708_DMA_WAIT_RESP;
		if (xfer->tx_buf) {
			cbs[cbs_pos].info |= BCM2708_DMA_S_INC; /* source increment by 4 */
			cbs[cbs_pos].src = (unsigned long)xfer->tx_dma;
		} else {
			cbs[cbs_pos].info |= BCM2708_DMA_S_IGNORE; /* ignore source */
			cbs[cbs_pos].src = bs->dma_buffer_handle;
		}
		cbs[cbs_pos].dst = cbs[last_tx_pos].dst;
		cbs[cbs_pos].length = xfer->len;
		cbs[cbs_pos].stride = 0;
		cbs[cbs_pos].next = (u32)0;
		/* set the pointer to this */ 
		cbs[last_tx_pos].next = RAM2PHY(&cbs[cbs_pos],bs->dma_buffer,bs->dma_buffer_handle);
		last_tx_pos=cbs_pos;
		cbs_pos++;
		/* fill in rx */
		cbs[cbs_pos].info = BCM2708_DMA_PER_MAP(7) /* DREQ 7 = SPI RX in PERMAP */
			| BCM2708_DMA_S_DREQ;              /* source DREQ trigger */
		if (xfer->rx_buf) {
			cbs[cbs_pos].info |= BCM2708_DMA_D_INC; /* destination inc by 4 */
			cbs[cbs_pos].dst = (unsigned long)xfer->rx_dma;
		} else {
			cbs[cbs_pos].info |= BCM2708_DMA_D_IGNORE; /* ignore destination */
			cbs[cbs_pos].src = bs->dma_buffer_handle;
		}
		cbs[cbs_pos].src = cbs[1].dst;
		cbs[cbs_pos].length = xfer->len;
		cbs[cbs_pos].stride = 0;
		cbs[cbs_pos].next = (u32)0;
		if (last_rx_pos) {
			cbs[last_tx_pos].next = RAM2PHY(&cbs[cbs_pos],bs->dma_buffer,bs->dma_buffer_handle);
		} else {
			/* and register as start of transfer */
			writel(RAM2PHY(&cbs[cbs_pos],bs->dma_buffer,bs->dma_buffer_handle),
				bs->dma_rx.base + BCM2708_DMA_ADDR
				);
		}
			
		last_rx_pos=cbs_pos;
		cbs_pos++;
		/* here we need to handle the delay */
		if (xfer->delay_usecs) {
			dev_err(&master->dev, "We do not support delay right now...");
			/* note that we might be able to change the settings via DMA writing to SPI itself - testing would be required */
		}
	}

	/* trigger an interrupt when finished - if we got a completion */
	if ((msg->complete) ) /* for now interrupt always - otherwise we do not need to sleep */
		cbs[last_rx_pos].info |= BCM2708_DMA_INT_EN;              /* enable interrupt */


	/* dump the Control-block structures */
	if (unlikely(debugdma)) {
		int i;
		dev_info(&master->dev,
			"DMA Control blocks - DMA-CS value: %08x, BounceBuffer at: %08x\n",
			*dma_cs,bs->dma_bouncebuffer,bs->dma_bouncebuffer_handle);
		/* and the SPI Registers */
		print_hex_dump(KERN_DEBUG," SPI-REGS:",DUMP_PREFIX_ADDRESS,
			16,4,bs->base,32,false);
		/* and the Control blocks themselves */
		for(i=0;i<cbs_pos;i++) {
			char text[16];
			snprintf(text,16,"  DMA-CB[%i]:",i);
			dev_info(&master->dev,
				"DMA Control block at physical address %08x\n",
				RAM2PHY(&cbs[i],bs->dma_buffer,bs->dma_buffer_handle));
			print_hex_dump(KERN_DEBUG,text,DUMP_PREFIX_ADDRESS,
				16,4,&cbs[i],32,false);
		}
	}


	/* initialize done */
	INIT_COMPLETION(bs->done);

	/* start DMA - this should also enable the DMA */
	writel(BCM2708_DMA_ACTIVE, bs->dma_tx.base + BCM2708_DMA_CS);
	writel(BCM2708_DMA_ACTIVE, bs->dma_rx.base + BCM2708_DMA_CS);

	/* now that are running - waiting to get woken by interrupt */
	/* the timeout may be too short - depend on amount of data and freq. */
	if (wait_for_completion_timeout(
			&bs->done,
			msecs_to_jiffies(SPI_TIMEOUT_MS*10)) == 0) {
		/* clear cs */

		/* inform of event and return with error */
		dev_err(&master->dev, "DMA transfer timed out\n");
		/* need to abort Interrupts */
		bcm_dma_abort(bs->dma_tx.base);
		bcm_dma_abort(bs->dma_rx.base);
		status= -ETIMEDOUT;
	}
	
	/* handle bounce-buffers - in reverse */
#if 0
	if (unlikely(debugio)) {
		if (unlikely(debugio&(1<<spi->chip_select))) {
			printk(KERN_DEBUG "spi%i.%i: Transfer\n  Transfer Length=%i\n",
				master->bus_num,spi->chip_select,
				xfer->len
				);
			if (xfer->tx_buf) {
				print_hex_dump(KERN_DEBUG,"  TXData=",DUMP_PREFIX_ADDRESS,
					16,1,
					xfer->tx_buf,xfer->len,
					false);
			}
			if (xfer->rx_buf) {
				print_hex_dump(KERN_DEBUG,"  RXData=",DUMP_PREFIX_ADDRESS,
					16,1,
					xfer->rx_buf,xfer->len,
					false);
			}
		}
	}
#endif

exit:
	msg->status = status;
	/* note that this would block other pipelining - so we might want to do it differently */
	spi_finalize_current_message(master);
	/* and return */
	return status;
}

static int bcm2835_prepare_transfer(struct spi_master *master)
{
	return 0;
}

static int bcm2835_unprepare_transfer(struct spi_master *master)
{
	return 0;
}

static int bcm2835_spi_setup(struct spi_device *spi)
{
	struct bcm2835_spi *bs = spi_master_get_devdata(spi->master);
	struct bcm2835_spi_state *state;
	int ret;

	/* configure master */

	if (bs->stopping)
		return -ESHUTDOWN;

	if (!(spi->mode & SPI_NO_CS) &&
		(spi->chip_select > spi->master->num_chipselect)) {
		dev_err(&spi->dev,
			"setup: invalid chipselect %u (%u defined)\n",
			spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	state = spi->controller_state;
	if (!state) {
		state = kzalloc(sizeof(*state), GFP_KERNEL);
		if (!state)
			return -ENOMEM;

		spi->controller_state = state;
	}

	ret = bcm2835_setup_state(spi->master, &spi->dev, state,
				spi->max_speed_hz, spi->chip_select, spi->mode,
				spi->bits_per_word);
	if (ret < 0) {
		kfree(state);
		spi->controller_state = NULL;
		return ret;
	}

	return 0;
}

static void bcm2835_spi_cleanup(struct spi_device *spi)
{
	kfree(spi->controller_state);
	spi->controller_state = NULL;
}

static int bcm2835_spi_probe(struct platform_device *pdev)
{
	struct resource *regs;
	int err = -ENOMEM;
	struct clk *clk;
	struct spi_master *master;
	struct bcm2835_spi *bs;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "could not get IO memory\n");
		return -ENXIO;
	}

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "could not find clk: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	bcm2835_init_pinmode();

	master = spi_alloc_master(&pdev->dev, sizeof(*bs));
	if (!master) {
		dev_err(&pdev->dev, "spi_alloc_master() failed\n");
		goto out_clk_put;
	}

	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_NO_CS;

	master->bus_num = pdev->id;
	master->num_chipselect = 3;
	master->setup = bcm2835_spi_setup;
	master->cleanup = bcm2835_spi_cleanup;
	master->rt = realtime;

	master->prepare_transfer_hardware       = bcm2835_prepare_transfer;
	master->transfer_one_message            = bcm2835_transfer_one_message;
	master->unprepare_transfer_hardware     = bcm2835_unprepare_transfer;

	platform_set_drvdata(pdev, master);

	bs = spi_master_get_devdata(master);
	spin_lock_init(&bs->lock);
	init_completion(&bs->done);

	/* get Register Map */
	bs->base = ioremap(regs->start, resource_size(regs));
	if (!bs->base) {
		dev_err(&pdev->dev, "could not remap memory\n");
		goto out_master_put;
	}

	bs->clk = clk;
	bs->stopping = false;

	/* enable DMA */
	/* register memory buffer for DMA */
	err = bcm2835_register_dmabuffer(pdev, bs);
	if (err)
		goto out_iounmap;

	/* register channels and irq */
	err = bcm2835_register_dma(pdev,
						&bs->dma_rx,
						bs->dma_buffer,
						DRV_NAME "(rxDMA)"
				);
	if (err)
		goto out_free_dma_buffer;
	err = bcm2835_register_dma(pdev,
						&bs->dma_tx,
						bs->dma_buffer,
						DRV_NAME "(txDMA)"
				);
	if (err)
		goto out_free_dma_rx;
	/* register IRQ for RX dma channel  */
	err = request_irq(bs->dma_rx.irq,
			bcm2835_transfer_one_message_dma_irqhandler,
			0,
			dev_name(&pdev->dev),
			master);
	if (err) {
		dev_err(&pdev->dev, "could not request IRQ: %d\n", err);
		goto out_free_dma_tx;
	}

	/* initialise the hardware */
	clk_enable(clk);
	bcm2835_wr(bs, SPI_CS, SPI_CS_REN | SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX);

	err = spi_register_master(master);
	if (err) {
		dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
		goto out_free_dma_irq;
	}

	dev_info(&pdev->dev, "SPI Controller at 0x%08lx\n",
		(unsigned long)regs->start);

	dev_info(&pdev->dev, "SPI Controller running in DMA mode\n");
	return 0;
out_free_dma_irq:
	free_irq(bs->dma_rx.irq, master);
out_free_dma_tx:
	bcm2835_release_dma(pdev, &bs->dma_tx);
out_free_dma_rx:
	bcm2835_release_dma(pdev, &bs->dma_rx);
out_free_dma_buffer:
	bcm2835_release_dmabuffer(pdev, bs);
out_iounmap:
	iounmap(bs->base);
out_master_put:
	spi_master_put(master);
out_clk_put:
	clk_put(clk);
	return err;
}

static int bcm2835_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct bcm2835_spi *bs = spi_master_get_devdata(master);

	/* reset the hardware and block queue progress */
	bs->stopping = true;

	/* release interrupts */
	free_irq(bs->dma_rx.irq, master);
	/* release DMA - disabling RX and TX SPI*/
	bcm2835_wr(bs, SPI_CS, SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX);
	/* release buffers and memory */
	bcm2835_release_dma(pdev, &bs->dma_tx);
	bcm2835_release_dma(pdev, &bs->dma_rx);
	bcm2835_release_dmabuffer(pdev, bs);

	/* and unregister device */
	spi_unregister_master(master);

	/* disable clocks */
	clk_disable_unprepare(bs->clk);
	clk_put(bs->clk);

	/* release master */
	spi_master_put(master);
	iounmap(bs->base);


	return 0;
}

/* for Device tree matching */
static const struct of_device_id bcm2835_spi_match[] = {
        { .compatible = "bcrm,bcm2708_spi", },
        {}
};
MODULE_DEVICE_TABLE(of, bcm2835_spi_match);

/* and for "compatiblity" */
static const struct platform_device_id bcm2835_id_table[] = {
        { "bcm2708_spi",   2708 },
        { "bcm2835_spi",   2835 },
        { },
};

static struct platform_driver bcm2835_spi_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
                .of_match_table = bcm2835_spi_match,
	},
	.id_table       = bcm2835_id_table,
	.probe		= bcm2835_spi_probe,
	.remove		= bcm2835_spi_remove,

};

static int __init bcm2835_spi_init(void)
{
	return platform_driver_probe(&bcm2835_spi_driver, bcm2835_spi_probe);
}
module_init(bcm2835_spi_init);

static void __exit bcm2835_spi_exit(void)
{
	platform_driver_unregister(&bcm2835_spi_driver);
}
module_exit(bcm2835_spi_exit);

MODULE_DESCRIPTION("SPI controller driver for Broadcom BCM2835");
MODULE_AUTHOR("Chris Boot <bootc@bootc.net>, Martin Sperl, Notro");
MODULE_LICENSE("GPL v2");
