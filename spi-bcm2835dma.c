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

#define BCM2835_SPI_NUM_CS	3

#define DRV_NAME	"spi-bcm2835dma"

static bool realtime = 1;
module_param(realtime, bool, 0);
MODULE_PARM_DESC(realtime, "Run the driver with realtime priority");

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

/* the DMA registers and their bitflags */
#define BCM2835_DMA_CS                                  0x00
#define BCM2835_DMA_CS_ACTIVE				(1 << 0)
#define BCM2835_DMA_CS_END				(1 << 1)
#define BCM2835_DMA_CS_INT				(1 << 2)
#define BCM2835_DMA_CS_DREQ				(1 << 3)
#define BCM2835_DMA_CS_ISPAUSED				(1 << 4)
#define BCM2835_DMA_CS_ISHELD				(1 << 5)
#define BCM2835_DMA_CS_WAITING_FOR_OUTSTANDING_WRITES	(1 << 6)
/* bit 7: reserved */
#define BCM2835_DMA_CS_ERR				(1 << 8)
/* bit 9-15: reserved */
#define BCM2835_DMA_CS_PRIORITY(x)			(((x)&0x0f) <<16)
#define BCM2835_DMA_CS_PANICPRIORITY(x)			(((x)&0x0f) <<20)
/* bit 24-27: reserved */
#define BCM2835_DMA_CS_WAIT_FOR_OUTSTANDING_WRITES	(1 <<28)
#define BCM2835_DMA_CS_DISDEBUG				(1 <<29)
#define BCM2835_DMA_CS_ABORT				(1 <<30)
#define BCM2835_DMA_CS_RESET				(1 <<31)

#define BCM2835_DMA_CB_ADDR                             0x04
#define BCM2835_DMA_TI                                  0x08
#define BCM2835_DMA_TI_INT_EN				(1 << 0)
#define BCM2835_DMA_TI_TDMODE				(1 << 1)
#define BCM2835_DMA_TI_RESERVED				(1 << 2)
#define BCM2835_DMA_TI_WAIT_RESP		       	(1 << 3)
#define BCM2835_DMA_TI_D_INC				(1 << 4)
#define BCM2835_DMA_TI_D_WIDTH				(1 << 5)
#define BCM2835_DMA_TI_D_DREQ				(1 << 6)
#define BCM2835_DMA_TI_D_IGNORE				(1 << 7)
#define BCM2835_DMA_TI_S_INC				(1 << 8)
#define BCM2835_DMA_TI_S_WIDTH				(1 << 9)
#define BCM2835_DMA_TI_S_DREQ				(1 <<10)
#define BCM2835_DMA_TI_S_IGNORE				(1 <<11)
#define BCM2835_DMA_TI_BURST(x)				(((x)&0x0f) <<12)
#define BCM2835_DMA_TI_PER_MAP(x)		       	(((x)&0x1f) <<16)
#define BCM2835_DMA_TI_WAITS(x)				(((x)&0x1f) <<21)
#define BCM2835_DMA_TI_NO_WIDE_BURSTS			(1 <<26)
/* bit 27-31: reserved */

#define BCM2835_DMA_S_ADDR                              0x0C
#define BCM2835_DMA_D_ADDR                              0x10
#define BCM2835_DMA_LEN                                 0x14
#define BCM2835_DMA_STRIDE                              0x18
#define BCM2835_DMA_NEXT                                0x1C
#define BCM2835_DMA_DEBUG                               0x20

/* the bcm2835 dma control block */
struct bcm2835_dma_cb {
	/* first the "real" control-block used by the DMA - 32 bytes */
	u32 ti;
        /* the control registers */
	dma_addr_t src;
	dma_addr_t dst;
	u32 length;
	u32 stride;
	dma_addr_t next;
	u32 pad[2];
};

/* the structure that defines the DMAs we use */
struct bcm2835_dma {
	void __iomem *base;
	dma_addr_t bus_addr;
        int chan;
        int irq;
	irq_handler_t handler;
	const char *desc;
};

static void bcm2835_dma_dump_cb(struct bcm2835_dma_cb* cb)
{
        printk(KERN_DEBUG "        .info     = %08x\n",readl(&cb->ti));
        printk(KERN_DEBUG "        .src      = %08x\n",readl(&cb->src));
        printk(KERN_DEBUG "        .dst      = %08x\n",readl(&cb->dst));
        printk(KERN_DEBUG "        .length   = %08x\n",readl(&cb->length));
        printk(KERN_DEBUG "        .stride   = %08x\n",readl(&cb->stride));
        printk(KERN_DEBUG "        .next     = %08x\n",readl(&cb->next));
}

static void bcm2835_dma_dump(struct bcm2835_dma *dma)
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

	bcm2835_dma_dump_cb(
		(struct bcm2835_dma_cb*)(dma->base+BCM2835_DMA_TI)
		);

        printk(KERN_DEBUG "        .debug    = %08x\n",
		readl(dma->base+BCM2835_DMA_DEBUG));
}

/* 
 * define a spi_dmachain  and  the corresponding cache
 * this should prossibly go to spi.h
 */
struct spi_dmafragment {
	struct list_head cache_list;
	struct list_head fragment_chain;
	unsigned is_used:1;
};

struct spi_dmafragment_chain {
	struct list_head fragment_chain;
	struct list_head *fragment_head;
	void *addr;
	dma_addr_t bus_addr;
	void (*complete)(void *context);
	void *context;
	void *dma_channel;
};

static void bcm2835dma_dmafragment_chain_dump(struct bcm2835_dma *dma,
					struct list_head *list)
{
	int i=0;
	struct spi_dmafragment_chain *e;

	list_for_each_entry(e, list, fragment_chain) {
		if (e->dma_channel!=dma)
			continue;
		i++;
		printk(KERN_DEBUG "   CB[%2i].addr     = %pK\n",
			i,e->addr);
		printk(KERN_DEBUG "          .bus_addr = %08x\n",
			e->bus_addr);
		printk(KERN_DEBUG "          .complete = %pF\n",
			e->complete);
		printk(KERN_DEBUG "          .context  = %pK\n",
			e->context);
		bcm2835_dma_dump_cb(e->addr);
	}
}

struct spi_dmafragment_cache {
	struct list_head cache_list;
	spinlock_t       lock;

	struct dma_pool *pool;
	
	struct spi_dmafragment *(*create)(
		struct spi_dmafragment_cache *,gfp_t);
	void (*release)(
		struct spi_dmafragment_cache *,
		struct spi_dmafragment *);
};

static void spi_dmafragment_cache_create(struct spi_dmafragment_cache *cache,
					struct spi_dmafragment *(*create)(
						struct spi_dmafragment_cache *,
						u32),
					void (*release)(
						struct spi_dmafragment_cache *,
						struct spi_dmafragment *),
					struct dma_pool* pool,
					u32 initial_objects
	)
{
	struct spi_dmafragment *frag;

	INIT_LIST_HEAD(&cache->cache_list);

	spin_lock_init(&cache->lock);

	cache->create=create;
	cache->release=release;
	cache->pool=pool;

	while(initial_objects--) {
		frag=cache->create(cache,GFP_KERNEL);
		if (frag) {
			list_add_tail(&frag->cache_list,&cache->cache_list);
		}
	}
}

static void spi_dmafragment_cache_destroy(struct spi_dmafragment_cache *cache)
{
	struct spi_dmafragment  *entry;
	unsigned long flags;

	spin_lock_irqsave(&cache->lock,flags);

	list_for_each_entry(entry, &cache->cache_list, cache_list) {
		cache->release(cache,entry);
	}

	spin_unlock_irqrestore(&cache->lock,flags);
}

static struct spi_dmafragment *spi_dmafragment_fetch(
	struct spi_dmafragment_cache *cache,
	gfp_t gfp)
{
	unsigned long flags;
	struct spi_dmafragment *found;

	spin_lock_irqsave(&cache->lock,flags);

	found=list_first_entry_or_null(
		&cache->cache_list,
		struct spi_dmafragment,
		cache_list);
	if ((found)&&(!found->is_used)) {
		found->is_used=1;
		/* mark the item as in use and shift left, so that the next
		   in sequence is used */
		list_rotate_left(&found->cache_list);
		spin_unlock_irqrestore(&cache->lock,flags);
		return found;
	}

	found=cache->create(cache,gfp);
	if (found) {
		found->is_used=1;
		spin_lock_irqsave(&cache->lock,flags);
		list_add_tail(&found->cache_list,&cache->cache_list);
		spin_unlock_irqrestore(&cache->lock,flags);
	}

	return found;
}

static void spi_dmafragment_return(struct spi_dmafragment_cache *cache,
	struct spi_dmafragment *chain)
{
	unsigned long flags;
	spin_lock_irqsave(&cache->lock,flags);

	chain->is_used=0;
	/* move to head of chain */
	list_del(&chain->cache_list);
	list_add(&chain->cache_list,&cache->cache_list);

	spin_unlock_irqrestore(&cache->lock,flags);
}

struct spi_message_assemble_state {
	struct spi_device *spi;
	struct spi_message *message;
	struct spi_transfer *xfer;
	
	u32 speed_hz,last_speed_hz;
	u32 clock_rate;
	u32 clock_divider;

	void *link_rx_here;
	void *link_tx_here;

	struct list_head *chain;

	gfp_t gfp;
};

int __spi_message_assemble_dma_clock(
	struct spi_message_assemble_state *state);
int __spi_message_assemble_dma_prepare_transfer(
	struct spi_message_assemble_state *state);
int __spi_message_assemble_dma_transfer(
	struct spi_message_assemble_state *state);
int __spi_message_assemble_dma_delay(
	struct spi_message_assemble_state *state);
int __spi_message_assemble_dma_change_cs(
	struct spi_message_assemble_state *state);
int __spi_message_assemble_dma_interrupt(
	struct spi_message_assemble_state *state);

static int spi_message_assemble_dma(
	struct spi_device *spi,
	struct spi_message *m,
	struct list_head *chain,
	gfp_t flags)
{
	struct spi_master* master=spi->master;
	int ret=0;
	/* the state */
	struct spi_message_assemble_state state;
	state.spi=spi;
	state.message=m;
	state.chain=chain;
	state.gfp=flags;
	state.speed_hz=spi->max_speed_hz;
	state.last_speed_hz=0;
	state.clock_rate=0;
	state.clock_divider=0;
	state.link_rx_here=NULL;
	state.link_tx_here=NULL;
	/* start assembling dma transfers from fragments */
	INIT_LIST_HEAD(chain);
	list_for_each_entry(state.xfer,
			&state.message->transfers, 
			transfer_list) {
		/* first the clock config */
		ret=__spi_message_assemble_dma_clock(&state);
		if (ret)
			return ret;
		/* now set up TX/RX DMA and configure SPI registers */
		ret=__spi_message_assemble_dma_prepare_transfer(&state);
		if (ret)
			return ret;
		/* and now loop all the xfers until we find an exit condition */
		ret=__spi_message_assemble_dma_transfer(&state);
		if (ret)
			return ret;
		/* schedule some delay (possibly either way ) */
		ret=__spi_message_assemble_dma_delay(&state);
		if (ret)
			return ret;
		/* schedule cs_up if needed */
		if (
			(state.xfer->cs_change)
			||(list_is_last(
					&state.xfer->transfer_list,
					&m->transfers)
				)) {
			ret=__spi_message_assemble_dma_change_cs(&state);
			if (ret)
				return ret;
		}
	}
	/* and if we need to run an interrupt schedule that */
	if ((m->complete)||(master->transfer_one_message)) {
		ret=__spi_message_assemble_dma_interrupt(&state);
		if (ret)
			return ret;
	}

	/* and return ok */
	return 0;
}
int __spi_message_assemble_dma_clock(
	struct spi_message_assemble_state *state)
{
	if (state->xfer->speed_hz)
		state->speed_hz=state->xfer->speed_hz;
	if (state->last_speed_hz==state->speed_hz)
		return 0;
	state->last_speed_hz=state->speed_hz;

	


	return 0;
}

int __spi_message_assemble_dma_prepare_transfer(
	struct spi_message_assemble_state *state)
{ return -EPERM; }
int __spi_message_assemble_dma_transfer(
	struct spi_message_assemble_state *state)
{ return -EPERM; }
int __spi_message_assemble_dma_delay(
	struct spi_message_assemble_state *state)
{ return -EPERM; }
int __spi_message_assemble_dma_change_cs(
	struct spi_message_assemble_state *state)
{ return -EPERM; }
int __spi_message_assemble_dma_interrupt(
	struct spi_message_assemble_state *state)
{ return -EPERM; }


static void spi_release_dmachain(
	struct spi_master* master,dma_addr_t active)
{
	unsigned long flags;
	struct spi_dmafragment_chain *cb,*tmp;
	
	spin_lock_irqsave(&master->queued_dma_lock,flags);

	list_for_each_entry_safe(cb,tmp,&master->queued_dma,fragment_chain) {
		
		if (cb->bus_addr == active)
			break;

		/* TODO: unmap mmapped memory */
		
		/* return to "owning" chain */
		list_add_tail(cb->fragment_head,&cb->fragment_chain);

		/* and if it has a callback, then call that */
		if (cb->complete)
			cb->complete(cb->context);
	}

	spin_unlock_irqrestore(&master->queued_dma_lock,flags);

	return;
}

static int spi_schedule_dmachain(
	struct spi_master* master,
	struct list_head *chain)
{
	unsigned long flags;

	if (list_empty(chain))
		return -EPERM;

	spin_lock_irqsave(&master->queued_dma_lock,flags);

	list_splice_tail_init(chain,&master->queued_dma);
	
#if 0
	if (!list_empty(&bs->active_cb_chain)) {
		list_entry((list)->next, struct bcm2835dma_dma_cb, cb_channel);
		/* so we can also link now the DMA cbs*/
		last->next=first->bus_addr;
		/* and sync the memory via a barrier */
		dsb();
	}
#endif

	spin_unlock_irqrestore(&master->queued_dma_lock,flags);

	return -EPERM;
}

/*
 * REAL IMPLEMENTATION STARTS HERE
 */

struct spi_dmafragment_chain *bcm2835_dmafragment_chain_alloc(
	struct dma_pool* pool, gfp_t flags)
{
	struct spi_dmafragment_chain *frag;
	/* allocate fragment */
	frag=kzalloc(sizeof(*frag),flags);
	if (!frag)
		return NULL;

	frag->addr=dma_pool_alloc(pool,flags,&frag->bus_addr);
	if (!frag->addr)
		goto error;

	memset(frag->addr,0,sizeof(*frag->addr));

	return frag;

error:
	kfree(frag);
	return NULL;
}

void  bcm2835_dmafragment_chain_free(struct dma_pool* pool, 
				struct spi_dmafragment_chain* frag)
{
	list_del(&frag->fragment_chain);

	if (frag->addr)
		dma_pool_free(pool,frag->addr,frag->bus_addr);
	
	kfree(frag);
}

/* here the real fragments */

/* the set Clock Divider fragment */
struct bcm2835dma_dmafragment_clock {
	struct spi_dmafragment fragment;
	u32 cdiv;
};

struct spi_dmafragment *bcm2835dma_dmafragment_clock_create(
	struct spi_dmafragment_cache *cache, gfp_t gfp)
{
	/* allocate the dma_fragment itself (should be in DMA region), 
	 * so take it from the pool */
	/* now allocate the CB from a different pool */
	/* and fill in the data */
	return NULL;
}

void bcm2835dma_dmafragment_clock_release(
	struct spi_dmafragment_cache *cache,
	struct spi_dmafragment * frag)
{
}

/* the delay fragment */
struct bcm2835dma_dmafragment_delay {
	struct spi_dmafragment fragment;
	u32 *delay_loop;
};

/* the init_transfer fragment */
struct bcm2835dma_dmafragment_init_transfer {
	struct spi_dmafragment fragment;
	u32 reset;
	u32 len;
	u32 flags;
};

/* the transfer_txrx fragment */
struct bcm2835dma_dmafragment_transfer_txrx {
	struct spi_dmafragment fragment;
	u32 *len;
	u32 *dma_flags;
	dma_addr_t *source;
	dma_addr_t *dest;
};

/* the cs_change fragment */
struct bcm2835dma_dmafragment_cs_change {
	struct spi_dmafragment fragment;
	u32 spi_flags;
};

/* the end_of_message fragment = trigger IRQ */
struct bcm2835dma_dmafragment_end_of_message {
	struct spi_dmafragment fragment;
};

/* the spi device data structure */
struct bcm2835dma_spi {
	void __iomem *regs;
	struct clk *clk;
	struct completion done;

	/* the chip-select flags that store the polarities
	 * and other flags for transfer */
	u32 cs_device_flags_idle;
	u32 cs_device_flags[BCM2835_SPI_NUM_CS];

	/* the dmas we require */
	struct bcm2835_dma dma_tx;
	struct bcm2835_dma dma_rx;

	/* the DMA-able pool we use to allocate control blocks from */
	struct dma_pool *pool;

	/* some helper segments from the dma pool used for transfers */
	struct {
		void *addr;
		dma_addr_t bus_addr;
	} buffer_write_dummy,buffer_read_0x00;

	/* the dmachain caches */
	struct spi_dmafragment_cache dma_fragment_cache_clock,
		dma_fragment_cache_delay,
		dma_fragment_cache_init_transfer,
		dma_fragment_cache_cs_change,
		dma_fragment_cache_transfer_txrx,
		dma_fragment_cache_end_of_message;

};

/* the following 2 functions are in need of a rewrite to get them use the
   linux-dmaengine instead for allocation of DMAs*/
#include <mach/dma.h>
static void bcm2835dma_release_dmachannel(struct spi_master *master,
			struct bcm2835_dma *d)
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

static int bcm2835dma_allocate_dmachannel(struct spi_master *master,
                                struct bcm2835_dma *d,
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
	/* and reset the DMA */
	writel(BCM2835_DMA_CS_RESET,d->base+BCM2835_DMA_CS);
	writel(0,d->base+BCM2835_DMA_CB_ADDR);
	/* and the irq handler */
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


static irqreturn_t bcm2835dma_spi_interrupt_dma_tx(int irq, void *dev_id);

static int bcm2835dma_create_dma(struct spi_master *master)
{
	int err;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	/* allocate DMA */
	err=bcm2835dma_allocate_dmachannel(
		master,&bs->dma_tx,bcm2835dma_spi_interrupt_dma_tx,"tx");
	if (err)
		return err;
	err=bcm2835dma_allocate_dmachannel(
		master,&bs->dma_rx,NULL,"rx");
	if (err)
		return err;

	/* allocate pool */
	bs->pool=dma_pool_create(
                DRV_NAME "_dmapool",
                &master->dev,
                sizeof(struct bcm2835_dma_cb),
                64, 
                0 
                );
	if (!bs->pool) {
		dev_err(&master->dev, "could not allocate DMA-memory pool\n");
		return -ENOMEM;
	}
	
	/* initialitze dma_fragment_caches 
	 * no idea yet what would be ideal numbers for the 
	 * initially created objects in cache below...
	 * assumption is that we should be able to run:
	 * one spi_write_then_read
	 * one spi_read
	 * one spi_write
	 * without having to go thru the loops
	 */
	spi_dmafragment_cache_create(&bs->dma_fragment_cache_clock,
				bcm2835dma_dmafragment_clock_create,
				bcm2835dma_dmafragment_clock_release,
				bs->pool,
				3
		);
	
	spi_dmafragment_cache_create(&bs->dma_fragment_cache_delay,
				NULL,
				NULL,
				bs->pool,
				1
		);
	
	spi_dmafragment_cache_create(&bs->dma_fragment_cache_init_transfer,
				NULL,
				NULL,
				bs->pool,
				3
		);

	spi_dmafragment_cache_create(&bs->dma_fragment_cache_cs_change,
				NULL,
				NULL,
				bs->pool,
				3
		);

	spi_dmafragment_cache_create(&bs->dma_fragment_cache_transfer_txrx,
				NULL,
				NULL,
				bs->pool,
				5
		);

	spi_dmafragment_cache_create(&bs->dma_fragment_cache_end_of_message,
				NULL,
				NULL,
				bs->pool,
				3
		);

	/* allocate some pages from pool for "standard" pages */
	bs->buffer_write_dummy.addr=
		dma_pool_alloc(bs->pool,GFP_KERNEL,
			&bs->buffer_write_dummy.bus_addr);

	bs->buffer_read_0x00.addr=
		dma_pool_alloc(bs->pool,GFP_KERNEL,
			&bs->buffer_read_0x00.bus_addr);
	memset(bs->buffer_read_0x00.addr,0x00,
		sizeof(*bs->buffer_read_0x00.addr));

	return 0;
}

static void bcm2835dma_release_dma(struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	bcm2835dma_release_dmachannel(master,&bs->dma_tx);
	bcm2835dma_release_dmachannel(master,&bs->dma_rx);

	if (!bs->pool)
		return;

	spi_release_dmachain(master,0);

	dma_pool_free(bs->pool,
		bs->buffer_read_0x00.addr,bs->buffer_read_0x00.bus_addr);
	dma_pool_free(bs->pool,
		bs->buffer_write_dummy.addr,bs->buffer_write_dummy.bus_addr);

        spi_dmafragment_cache_destroy(&bs->dma_fragment_cache_clock);
	spi_dmafragment_cache_destroy(&bs->dma_fragment_cache_delay);
        spi_dmafragment_cache_destroy(&bs->dma_fragment_cache_init_transfer);
        spi_dmafragment_cache_destroy(&bs->dma_fragment_cache_cs_change);
        spi_dmafragment_cache_destroy(&bs->dma_fragment_cache_transfer_txrx);
        spi_dmafragment_cache_destroy(&bs->dma_fragment_cache_end_of_message);

	dma_pool_destroy(bs->pool);
        bs->pool=NULL;
}

static void bcm2835dma_dump_spi(struct spi_master* master) {
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
        printk(KERN_DEBUG"  SPI-REGS:\n");
        printk(KERN_DEBUG"    SPI-CS:   %08x\n",
		readl(bs->regs + BCM2835_SPI_CS));
        printk(KERN_DEBUG"    SPI-CLK:  %08x\n",
		readl(bs->regs + BCM2835_SPI_CLK));
        printk(KERN_DEBUG"    SPI-DLEN: %08x\n",
		readl(bs->regs + BCM2835_SPI_DLEN));
        printk(KERN_DEBUG"    SPI-LOTH: %08x\n",
		readl(bs->regs + BCM2835_SPI_LTOH));
        printk(KERN_DEBUG"    SPI-DC:   %08x\n",
		readl(bs->regs + BCM2835_SPI_DC));
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
	bcm2835_dma_dump(&bs->dma_rx);
	bcm2835dma_dmafragment_chain_dump(&bs->dma_rx,&master->queued_dma);
	bcm2835_dma_dump(&bs->dma_tx);
	bcm2835dma_dmafragment_chain_dump(&bs->dma_tx,&master->queued_dma);
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

	/* dependent on how we run */
	if (use_transfer_one) {
		/* wake up message pump task */
		complete(&bs->done);
	} else {
		/* release the control block chains until we reach the CB 
		 * this will also call complete
		 */
		//bcm2835dma_release_cb_chain_complete(master);
	}

	/* and return with the IRQ marked as handled */
	return IRQ_HANDLED;
}


/* most likley we will need to move away from the transfer_one at a 
 * time approach, if we want to pipeline the Transfers.. */
static int bcm2835dma_spi_transfer_one(struct spi_master *master,
		struct spi_message *message)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	/* the list head to use by default 
	   - if no prepared spi_message exists */
	LIST_HEAD(cb_chain_tmp);
	struct list_head *cb_chain=&cb_chain_tmp;
	/* the status */
	u32 status=0;

	/* prepare DMA chain */
	writel(bs->cs_device_flags[message->spi->chip_select]|BCM2835_SPI_CS_CSPOL1 ,bs->regs+BCM2835_SPI_CS);
	/* assemble the message - if not prepared already */
	status=spi_message_assemble_dma(
		message->spi,
		message,
		cb_chain,
		GFP_KERNEL);
	if (status)
		goto error_exit;

	writel(bs->cs_device_flags_idle,bs->regs+BCM2835_SPI_CS);
	/* if debugging dma, then dump what we got so far prior to scheduling */
	if (unlikely(debug_dma)) 
		bcm2835dma_dump_state(master);

	/* add list to DMA */
	status=spi_schedule_dmachain(master,cb_chain);
	if (status) {
		spi_print_debug_message(message,128);
		goto error_exit;
	}
	/* wait for us to get woken up again after the transfer */
        if (wait_for_completion_timeout(
                        &bs->done,
                        msecs_to_jiffies(SPI_TIMEOUT_MS)) == 0) {
		dev_emerg(&master->dev,"DMA transfer timed out\n");
		/* and set the error status and goto the exit code */
		status=-ETIMEDOUT;
	}
	/* dump the DMA status */
	if (status || unlikely(debug_dma)) {
		bcm2835dma_dump_state(master);
	}

error_exit:
	/* set the status - before we run the debug code and before we 
	   finalize the message */
	message->status=status;

	/* write the debug message information */
	if (unlikely(debug_msg)) {
		spi_print_debug_message(message,128);
	}

	/* release the control block chains until we reach the 
	   Control block that is currently active*/
	spi_release_dmachain(master,readl(bs->dma_rx.base+BCM2835_DMA_CB_ADDR));
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
				bs->cs_device_flags[i] |= 
					BCM2835_SPI_CS_CSPOL 
					| BCM2835_SPI_CS_CSPOL0 << cs;
			}
			/* the idle mode as well */
			bs->cs_device_flags_idle |=
				BCM2835_SPI_CS_CSPOL0 << cs;

			/* and the specific flag for this device */
			bs->cs_device_flags[cs] |= BCM2835_SPI_CS_CSPOL;
		}
		/* and now invert the CS polarity for this specific cs*/
		bs->cs_device_flags[cs] ^=
			BCM2835_SPI_CS_CSPOL0<<(cs&0x03);
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
	master->dev.of_node = pdev->dev.of_node;
	master->rt = realtime;

	if (use_transfer_one)
		master->transfer_one_message = bcm2835dma_spi_transfer_one;
	else {
		/* this needs to change - so it is not "fully correct" */
		printk(KERN_ERR "transfer message support not implemented yet");
		return -ENODEV;
	}

#ifdef HAVE_OPTIMIZE
	master->optimize=bcm2835dma_optimize_message;
	master->unoptimize=bcm2835dma_unoptimize_message;
#endif

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

#ifdef CONFIG_MACH_BCM2708
	/* configure pin function for SPI */
	bcm2835dma_spi_init_pinmode();
#endif

	err=bcm2835dma_create_dma(master);
	if (err) {
		dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
		goto out_release_dma;
	}
	
	/* initialise the hardware */
	writel(
		BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX,
		bs->regs+BCM2835_SPI_CS);

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
	writel(
		BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX,
		bs->regs+BCM2835_SPI_CS);
	
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
