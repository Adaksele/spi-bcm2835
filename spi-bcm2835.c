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
	struct bcm2708_dma_cb *last;
	struct bcm2708_dma_cb *first;
	dma_addr_t first_phy;
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
	u8 dma_end;
/* macro to translate betwee the to pointer */
#define RAM2PHY(addr,base_logic,base_phy) (base_phy+((u32)addr-(u32)base_logic))

	/* the dma bounce buffer */
	void *dma_bouncebuffer;
	dma_addr_t dma_bouncebuffer_handle;
	u8 value_end;

	/* and the information on the DMA channels we use */
	struct bcm2835_spi_dma dma_tx;
	struct bcm2835_spi_dma dma_rx;
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


void bcm2835_spi_dump(struct spi_master *master)
{
	struct bcm2835_spi *bs = spi_master_get_devdata(master);
	printk(KERN_DEBUG"  SPI-REGS:\n");
	printk(KERN_DEBUG"    SPI-CS:   %08x\n",readl(bs->base));
	/* do NOT read FIFO - even for Debug - it may produce hickups with DMA!!! */
	printk(KERN_DEBUG"    SPI-CLK:  %08x\n",readl(bs->base+8));
	printk(KERN_DEBUG"    SPI-DLEN: %08x\n",readl(bs->base+12));
	printk(KERN_DEBUG"    SPI-LOTH: %08x\n",readl(bs->base+16));
	printk(KERN_DEBUG"    SPI-DC:   %08x\n",readl(bs->base+20));
}

static void bcm2835_dma_dump_cr(void *base) {
	printk(KERN_DEBUG "     .info   = %08x\n",readl(base+0x00));
	printk(KERN_DEBUG "     .src    = %08x\n",readl(base+0x04));
	printk(KERN_DEBUG "     .dst    = %08x\n",readl(base+0x08));
	printk(KERN_DEBUG "     .length = %08x\n",readl(base+0x0c));
	printk(KERN_DEBUG "     .stride = %08x\n",readl(base+0x10));
	printk(KERN_DEBUG "     .next   = %08x\n",readl(base+0x14));
}

static void bcm2835_dma_dump_channel(struct spi_master *master, const char* prefix,struct bcm2835_spi_dma *dma)
{
	struct bcm2835_spi *bs = spi_master_get_devdata(master);
	struct bcm2708_dma_cb *cb=dma->first;
	u8 count=0;
	printk(KERN_DEBUG"%s DMA registers\n",prefix);
	printk(KERN_DEBUG "     .status = %08x\n",readl(dma->base+0x00));
	printk(KERN_DEBUG "     .cbaddr = %08x\n",readl(dma->base+0x04));
	bcm2835_dma_dump_cr(dma->base+8);
	printk(KERN_DEBUG "     .debug  = %08x\n",readl(dma->base+0x20));
	printk(KERN_DEBUG"%s DMA CBs\n",prefix);
	while(cb) {
		u32 virt=RAM2PHY(cb,bs->dma_buffer,bs->dma_buffer_handle);
		/* dump this cb */
		printk(KERN_DEBUG "   CB[%i] at address: %08x/%08x\n",count,(u32)cb,virt);
		bcm2835_dma_dump_cr(cb);
		printk(KERN_DEBUG "     .pad0   = %08lx\n",cb->pad[0]);
		printk(KERN_DEBUG "     .pad1   = %08lx\n",cb->pad[1]);
		/* and check the data for TX */
		if (cb->src) {
			/* calc the delta to the start of the bounce buffer */
			u32 delta=cb->src-bs->dma_bouncebuffer_handle;
			/* if we are within the "expected" range, then print the data */
			if (delta<sizeof(unsigned long)*bs->value_end) {
				print_hex_dump(KERN_DEBUG,
					"     .txdata =",
					DUMP_PREFIX_ADDRESS,
					32,4,
					((u8*)bs->dma_bouncebuffer+delta),cb->length,
					false);
			}
		}
		/* and calculate next */
		virt=cb->next;
		/* loop */
		cb=(struct bcm2708_dma_cb*)cb->pad[1];
		count++;
	}
}

static void bcm2835_dma_dump(struct spi_master *master, const char* prefix)
{
	struct bcm2835_spi *bs = spi_master_get_devdata(master);
	dev_info(&master->dev,
		"%s DMA setup: %i\n",
		prefix,bs->dma_end
		);
	/* dump the SPI Registers */
	bcm2835_spi_dump(master);
	/* dump channels */
	bcm2835_dma_dump_channel(master,"  TX",&bs->dma_tx);
	bcm2835_dma_dump_channel(master,"  RX",&bs->dma_rx);
	/* the Data registers */
	dev_info(&master->dev,
		"%s DMA %i vars start %08x/%08x\n",
		prefix,bs->value_end,
		(unsigned int)bs->dma_bouncebuffer,(unsigned int)bs->dma_bouncebuffer_handle
		);
	print_hex_dump(KERN_DEBUG,"  ",DUMP_PREFIX_ADDRESS,
		16,4,bs->dma_bouncebuffer,sizeof(unsigned long)*(bs->value_end),false);
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

	printk(KERN_DEBUG"DMA-IRQ triggered\n");
	//bcm2835_spi_dump(master);

	/* mark the rx DMA-interrupt as handled
	   - it will (level) trigger otherwise again */
	writel(readl(bs->dma_rx.base+BCM2708_DMA_CS)|BCM2708_DMA_INT, bs->dma_rx.base+BCM2708_DMA_CS);

	/* and wake up the thread to continue its work - returning ...*/
	complete(&bs->done);
	/* return IRQ handled */
	return IRQ_HANDLED;
}


static void bcm2835_dma_reset(struct bcm2835_spi *bs)
{
	bs->dma_end=0;
	bs->value_end=0;
	bs->dma_tx.last=NULL;
	bs->dma_tx.first=NULL;
	bs->dma_tx.first_phy=0;
	bs->dma_rx.last=NULL;
	bs->dma_rx.first=NULL;
	bs->dma_rx.first_phy=0;
}

dma_addr_t bcm2835_dma_add(struct bcm2835_spi *bs,
			struct bcm2835_spi_dma *dma,
			unsigned long info,
			unsigned long src,
			unsigned long dst,
			unsigned long length,
			unsigned long stride,
			u8 link_to_last
	)
{
	/* add the data to the "pool" */
	/* TODO: use DMA POOL of kernel instead of this list setup */
	struct bcm2708_dma_cb *cb=&bs->dma_buffer[bs->dma_end++];
	unsigned long cb_phy=RAM2PHY(cb,bs->dma_buffer,bs->dma_buffer_handle);
	/* copy the data */
	cb->info=info;
	cb->src=src;
	cb->dst=dst;
	cb->length=length;
	cb->stride=stride;
	cb->next=0;
	/* now add it to the end one */
	if (dma->last) {
		if (link_to_last)
			dma->last->next=cb_phy;
		/* primarily used for debugging */
		dma->last->pad[1]=(unsigned long)cb;
	} else {
		/* otherwise add it to the head */
		if (link_to_last)
			dma->first_phy=cb_phy;
		dma->first=cb;
	}
	/* set this to the last one */
	dma->last=cb;
	/* and return the physical pointer */
	return cb_phy;
}

dma_addr_t bcm2835_dma_add_value(struct bcm2835_spi *bs,unsigned long v,unsigned long **virtref) 
{
	/* abusing bounce buffer for now - we do not use it anyway... */
	/* calculate address */
	unsigned long *pos=(unsigned long*)(bs->dma_bouncebuffer)+bs->value_end;
	dma_addr_t virt=RAM2PHY(pos,bs->dma_bouncebuffer,bs->dma_bouncebuffer_handle);
	bs->value_end++;
	/* now assign data */
	*pos=v;
	/* and if virtref is set, then assign pos to it, so that we may change the value later... */
	if (virtref) {
		*virtref=pos;
	}
	/* and return the address */
	return virt;
}

void bcm2835_dma_run(struct bcm2835_spi *bs) {
	/* write the start addresses */
	writel(bs->dma_tx.first_phy,bs->dma_tx.base + BCM2708_DMA_ADDR);		
	/* memory barrier to make sure everything is "written to ram and not only to cache...*/
	dsb();
	/* start DMA - this should also enable the DMA */
	writel(BCM2708_DMA_ACTIVE, bs->dma_tx.base + BCM2708_DMA_CS);
}

static void bcm2835_dma_abort(struct bcm2835_spi *bs) {
	/* reset DMA */
	writel(BCM2708_DMA_RESET, bs->dma_tx.base + BCM2708_DMA_CS);
	writel(BCM2708_DMA_RESET, bs->dma_rx.base + BCM2708_DMA_CS);
}

static int bcm2835_transfer_one_message(struct spi_master *master,
					struct spi_message *msg)
{
	struct bcm2835_spi *bs = spi_master_get_devdata(master);
	struct spi_transfer *xfer;
	struct spi_device *spi = msg->spi;
	struct bcm2835_spi_state state;
	int status = 0;
	u32 cs = 0;
	u32 last_cdiv = 0;
	int transfers=0;
	u32 info=0;
	unsigned long *rx_dma_addr=NULL;

	/* set up the Registers and clean queues - maybe via DMA as well?*/
	bcm2835_dma_reset(bs);

	/* loop over all transfers */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		u8 is_last = list_is_last(&xfer->transfer_list, &msg->transfers);
		dma_addr_t rx_dma=0;
		dma_addr_t tx_dma=0;
		/* count up the transfers */
		transfers++;
		/* calculate the dma addresses */
		if (xfer->tx_buf) {
			if (xfer->tx_dma) {
				tx_dma=xfer->tx_dma;
			} else {
				/* get the physical address, but in the "correct" region */
				tx_dma=virt_to_phys(xfer->tx_buf)|0x40000000;
			}
		}
		if (xfer->rx_buf) {
			if (xfer->rx_dma) {
				rx_dma=xfer->rx_dma;
			} else {
				/* get the physical address, but in the "correct" region */
				rx_dma=virt_to_phys(xfer->rx_buf)|0x40000000;
			}
		}

		/* if we start, then clean the SPI setup */
		if (transfers==1) {
			bcm2835_dma_add(bs,&bs->dma_tx,
					BCM2708_DMA_WAIT_RESP,
					bcm2835_dma_add_value(bs,SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX,NULL),
					(unsigned long)(DMA_SPI_BASE + SPI_CS),
					4,0,1);
		}

		/* prepare transfer state */
		memcpy(&state,(struct bcm2835_spi_state *)spi->controller_state,sizeof(state));
		if (xfer->bits_per_word)
			state.bits_per_word=xfer->bits_per_word;
		if (xfer->speed_hz)
			state.speed_hz=xfer->speed_hz;
		/* should improve on the calculation */
		status = bcm2835_setup_state(spi->master, &spi->dev,
					&state,
					state.speed_hz,
					spi->chip_select,
					spi->mode,
					state.bits_per_word);
		if (status)
			goto exit;
		
		/* if the clock has changed, then set it */
		if (last_cdiv!=state.cdiv) {
			bcm2835_dma_add(bs,&bs->dma_tx,
					BCM2708_DMA_WAIT_RESP,
					bcm2835_dma_add_value(bs,state.cdiv,NULL),
					(unsigned long)(DMA_SPI_BASE + SPI_CLK),
					4,0,1);
			last_cdiv=state.cdiv;
		}


		/* calculate cs to use */
		cs=state.cs|SPI_CS_DMAEN|SPI_CS_TA;
		/* if we are the first, then reset SPI as well */

		/* if we are the last, then enable deasserting as well */
		if (is_last) 
			cs|=SPI_CS_ADCS;
		/* if we have cs_change enabled, then run it as well */
		if (xfer->cs_change) 
			cs|=SPI_CS_ADCS;
		/* set for all for now */
		//cs|=SPI_CS_ADCS;

		/* and set the FULL cs via DMA, as we can not set 16 bit with the initial DMA transfer 
		   - this is an ugly hack to make it work seemlessly, but it work... */
			bcm2835_dma_add(bs,&bs->dma_tx,
					BCM2708_DMA_WAIT_RESP,
					bcm2835_dma_add_value(bs,cs,NULL),
					(unsigned long)(DMA_SPI_BASE + SPI_CS),
					4,0,1);

		/* now set up the _real_ DMA transfer - not just all the setup above */

		/* first set up RX DMA - the reset above seems to kill it if run immediately */
		/* now just assign it as a dummy to TX - the real value gets filled in later */
		bcm2835_dma_add(bs,&bs->dma_tx,
				BCM2708_DMA_WAIT_RESP,
				bcm2835_dma_add_value(bs,0,&rx_dma_addr),
				(unsigned long)(bs->dma_rx.base + BCM2708_DMA_ADDR),
				4,0,1);
		/* and set the Mode - we possibly could use "stride" here to coalese with the above transfer */
		bcm2835_dma_add(bs,&bs->dma_tx,
				BCM2708_DMA_WAIT_RESP,
				bcm2835_dma_add_value(bs,BCM2708_DMA_ACTIVE,NULL),
				(unsigned long)(bs->dma_rx.base + BCM2708_DMA_CS),
				4,0,1);

		/* first the TX DMA */

		/* first CS and length */
		cs=     (xfer->len<<16) /* length of this transfer */
			| (cs & 0xff)   /* the relevant flags */
			| SPI_CS_TA     /* and transfer enable */
			;
		bcm2835_dma_add(bs,&bs->dma_tx,
				BCM2708_DMA_WAIT_RESP,
				bcm2835_dma_add_value(bs,cs,NULL),
				(unsigned long)(DMA_SPI_BASE + SPI_FIFO),
				4,0,1);
		
		/* fill in tx */
		info = BCM2708_DMA_PER_MAP(6)              /* DREQ 6 = SPI TX in PERMAP */
			| BCM2708_DMA_D_DREQ               /* destination DREQ trigger */
			| BCM2708_DMA_WAIT_RESP            /* and wait for WRITE response to get received */
			;
		if (tx_dma) {
			bcm2835_dma_add(bs,&bs->dma_tx,
					info|BCM2708_DMA_S_INC,
					tx_dma,
					(unsigned long)(DMA_SPI_BASE + SPI_FIFO),
					xfer->len,0,1);
		} else {
			bcm2835_dma_add(bs,&bs->dma_tx,
					info|BCM2708_DMA_S_IGNORE,
					tx_dma,
					(unsigned long)(DMA_SPI_BASE + SPI_FIFO),
					xfer->len,0,1);
		}
		/* fill in rx */

		/* first create the basic flags for this DMA transfer*/
		info = BCM2708_DMA_PER_MAP(7)              /* DREQ 7 = SPI RX in PERMAP */
			| BCM2708_DMA_S_DREQ               /* source DREQ trigger */
			| BCM2708_DMA_WAIT_RESP            /* and wait for WRITE response to get received */
			;
		/* if it is the last transfer, then trigger an IRQ - if requested */
		if (is_last) { 
			if ((msg->complete) ) {
				info |= BCM2708_DMA_INT_EN;
			}
		}
		/* assigning the RX-dma-block address */
		if (rx_dma) {
			*rx_dma_addr=
				bcm2835_dma_add(bs,&bs->dma_rx,
						info|BCM2708_DMA_D_INC,
						(unsigned long)(DMA_SPI_BASE + SPI_FIFO),
						rx_dma,
						xfer->len,0,0);
		} else {
			*rx_dma_addr=
				bcm2835_dma_add(bs,&bs->dma_rx,
						info|BCM2708_DMA_D_IGNORE,
						(unsigned long)(DMA_SPI_BASE + SPI_FIFO),
						rx_dma,
						xfer->len,0,0);
		}
		/* here we need to handle the delay */
		if (xfer->delay_usecs) {
			dev_err(&master->dev, "We do not support delay right now...");
			/* note that we might be able to change the settings via DMA writing to SPI itself - testing would be required */
		}
	}
		
	/* dump the Control-block structures */
	if (unlikely(debugdma)) {
		bcm2835_dma_dump(master,"PRE:");
	}

	/* initialize done */
	INIT_COMPLETION(bs->done);

	/* run DMA */
	bcm2835_dma_run(bs);

	/* now that are running - waiting to get woken by interrupt */
	/* the timeout may be too short - depend on amount of data and freq - better estimate needed? */
	if (wait_for_completion_timeout(
			&bs->done,
			msecs_to_jiffies(SPI_TIMEOUT_MS*10)) == 0) {
		/* inform of event and return with error */
		dev_err(&master->dev, "DMA transfer timed out\n");
		/* and dump DMA */
		bcm2835_dma_dump(master,"TIMEOUT:");
		/* need to abort Interrupts */
		bcm2835_dma_abort(bs);
		/* reset SPI */
		bcm2835_wr(bs, SPI_CS, state.cs|SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX);

		status= -ETIMEDOUT;
	}
	/* transfer debugging - even if we have had an incident*/
	if (unlikely(debugio)) {
		if (unlikely(debugio&(1<<spi->chip_select))) {
			transfers=0;
			list_for_each_entry(xfer, &msg->transfers, transfer_list) {
				transfers++;
				printk(KERN_DEBUG "spi%i.%i: Transfer %i\n  Transfer Length=%i\n",
					master->bus_num,spi->chip_select,transfers,
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
	}

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
