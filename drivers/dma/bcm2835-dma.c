/*
 * helper code for bcm2835 dma controlblocks
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
#include <linux/dma/bcm2835-dmafragment.h>
#include <linux/printk.h>
#include <linux/module.h>

struct dma_link *bcm2835_addDMAPoke(
	struct dma_pool *bcm2835_dma_pool,
	struct dma_fragment *frag, bool linkit,
	dma_addr_t addr, u32 val,
	gfp_t gfp)
{
	struct bcm2835_dma_cb *cb;
	/* allocate link */
	struct dma_link *link = dma_link_alloc(bcm2835_dma_pool, 0, gfp);
	if (!link)
		goto exit;
	/* link to the fragment and mark as end */
	if (frag)
		dma_fragment_add_dma_link(frag, link, linkit);
	/* setup control-block */
	cb         = link->cb;
	cb->ti     = (
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_S_INC
		| BCM2835_DMA_TI_D_INC
		);
	cb->src    = BCM2835_DMA_CB_MEMBER_DMA_ADDR(link, pad[0]);
	cb->dst    = addr;
	cb->length = 4;
	cb->pad[0] = val;

exit:
	return link;
}
EXPORT_SYMBOL_GPL(bcm2835_addDMAPoke);

struct dma_link * bcm2835_addDMAPoke2(
	struct dma_pool *bcm2835_dma_pool,
	struct dma_fragment *frag, bool linkit,
	dma_addr_t addr, u32 val1,  u32 val2,
	gfp_t gfp)
{
	struct bcm2835_dma_cb *cb;
	struct dma_link *link = bcm2835_addDMAPoke(
		bcm2835_dma_pool,
		frag, linkit,
		addr, val1, gfp);
	if (link) {
		cb->length = 8;
		cb->pad[1] = val2;
	}
	return link;
}
EXPORT_SYMBOL_GPL(bcm2835_addDMAPoke2);

dma_addr_t* bcm2835_getDMAPokeReg(struct dma_link *link)
{
	struct bcm2835_dma_cb *cb;
	if (!link)
		return NULL;
	cb = link->cb;

	return &cb->dst;
}
EXPORT_SYMBOL_GPL(bcm2835_getDMAPokeReg);

u32* bcm2835_getDMAPokeVal(struct dma_link *link)
{
	struct bcm2835_dma_cb *cb;
	if (!link)
		return NULL;
	cb = link->cb;

	return &cb->pad[0];
}
EXPORT_SYMBOL_GPL(bcm2835_getDMAPokeVal);

struct dma_link *bcm2835_addDMADelay(
	struct dma_pool *bcm2835_dma_pool,
	struct dma_fragment *frag, bool do_dma_link,
	u32 delay,
	gfp_t gfp)
{
	return NULL;
}
EXPORT_SYMBOL_GPL(bcm2835_addDMADelay);

int bcm2835_setDMADelay(struct dma_link *link, u32 udelay)
{
	return -EPERM;
}
EXPORT_SYMBOL_GPL(bcm2835_setDMADelay);

struct dma_link *bcm2835_addDMAStart(
	struct dma_pool *bcm2835_dma_pool,
	struct dma_fragment *frag, bool linkit,
	u32 channel, gfp_t gfp)
{
	struct bcm2835_dma_cb *cb;
	/* allocate link */
	struct dma_link *link = dma_link_alloc(bcm2835_dma_pool,0,gfp);
	if (!link)
		goto exit;
	/* link to the fragment and mark as end */
	if (frag)
		dma_fragment_add_dma_link(frag, link, linkit);

	/* setup control-block */
	cb         = link->cb;
	/* configure the 2D mode */
	bcm2835_dma_cb_compose_2d(cb,
				BCM2835_DMA_TI_WAIT_RESP,
				2, 4, 4, -4);
	/* and assign addresses */
	cb->src    = BCM2835_DMA_CB_MEMBER_DMA_ADDR(link,pad[0]);
	cb->dst    = BCM2835_DMA_BASE_BUS(channel)+BCM2835_DMA_ADDR;
exit:
	return link;
}
EXPORT_SYMBOL_GPL(bcm2835_addDMAStart);

int bcm2835_setDMAStartLink(struct dma_link *link,
			struct dma_link *to_start)
{
	struct bcm2835_dma_cb *cb;
	if (!link)
		return -EPERM;
	cb = link->cb;

	cb->pad[0] = to_start->cb_dma;
	return 0;
}
EXPORT_SYMBOL_GPL(bcm2835_setDMAStartLink);

struct dma_link *bcm2835_addDMATransfer(
	struct dma_pool *bcm2835_dma_pool,
	struct dma_fragment *frag, bool linkit,
	dma_addr_t src, dma_addr_t dst,u32 length,
	gfp_t gfp)
{
	return NULL;
}
EXPORT_SYMBOL_GPL(bcm2835_addDMATransfer);

int bcm2835_setDMATransfer(
	struct dma_fragment *frag,
	dma_addr_t src, dma_addr_t dst,u32 length)
{
	return -EPERM;
}
EXPORT_SYMBOL_GPL(bcm2835_setDMATransfer);

int bcm2835_schedule_dma_fragment(
	/* probably needs some more arguments */
	struct dma_fragment *frag)
{
	return -EPERM;
}
EXPORT_SYMBOL_GPL(bcm2835_schedule_dma_fragment);

#if 0
static int __init bcm2835_dma_init(void)
{
	/* create a device */

	bcm2835_dma_pool = dma_pool_create(
		"bcm2835dma-CB-pool",
		NULL,
		sizeof(struct bcm2835_dma_cb),
		64,
		0);
	if (!bcm2835_dma_pool) {
		pr_err("could not allocate DMA-memory pool\n");
		return -ENOMEM;
	}
	return 0;
}
module_init(bcm2835_dma_init);

static void __exit bcm2835_dma_exit(void)
{
	if (bcm2835_dma_pool)
		dma_pool_destroy(bcm2835_dma_pool);
	bcm2835_dma_pool=NULL;
}
module_exit(bcm2835_dma_exit);
#endif
MODULE_DESCRIPTION("bcm2835 dma_fragment tools");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL v2");
