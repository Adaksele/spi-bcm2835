/*
 * Driver for DMA Fragments - initially used for BCM2835 DMA implementation
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
 */

#ifndef __DMAFRAGMENT_H
#define __DMAFRAGMENT_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/dmapool.h>

struct dma_link;
struct DMAFragment;
struct DMAFragmentCache;

/**
 * struct dma_link - the linked list of DMA control blocks
 * @dmablock: the pointer to the DMA control-block itself (void pointer to be of generic use)
 * @dmablock_dma: the dma-buss address of the control block
 * @dmapool: from which pool the block has been taken
 * @linked_list: the linked list connecting the individual link blocks
 */
struct dma_link {
	void              *dmablock;
	dma_addr_t        dmablock_dma;
	struct dma_pool   *dmapool;
	struct list_head  linked_list;
};

/**
 * DMA_LINK_FIELD_DMAADDR() - helper macro to get the bus address of the specific field
 * @block: the link block for which we need the DMA address
 * @field: the field name we want to get the DMA-address for
 */
#define DMA_LINK_FIELD_DMAADDR(block,field)		\
	(							\
		block->dmablock_dma+				\
		offsetof(typeof(*block->dmablock),field)	\
		)

/**
 * dma_link_alloc
 * @dmapool: the dmapool from which we have to take the block (the pool defines the size explicitly)
 * @gfpflags: the flags to use for allocation
 */
struct dma_link *dma_link_alloc(struct dma_pool *pool,gfp_t gfpflags);

/**
 * dma_link_free
 * @block: the object to free
 */
void dma_link_free(struct dma_link *block);

/**
 * struct dma_fragment - a list of dma_links with means how to link Fragments quickly
 * @fragment_head: the pointer to the head of the dma_link list for the mixed dma-queue (rx/tx)
 * @fragment_tail: the pointer to the tail of the dma_link list for the mixed dma-queue (rx/tx)
 * @fragment_rx_head: the pointer to the head of the dma_link list for the rx_queue
 * @fragment_rx_tail: the pointer to the tail of the dma_link list for the rx_queue
 * @fragment_tx_head: the pointer to the head of the dma_link list for the tx_queue
 * @fragment_tx_tail: the pointer to the tail of the dma_link list for the tx_queue
 * @length_ptr: the length pointer for the transfer in this transfer block
 * @cache: the dma_fragment_cache to which this fragment belongs to (mostly used to return the fragment to the cache
 * @cache_list: the linked list for the fragment cache
 *
 */
struct dma_fragment {
	struct dma_link * fragmenent_head;
	struct dma_link * fragmenent_tail;
	struct dma_link * fragmenent_rx_head;
	struct dma_link * fragmenent_rx_tail;
	struct dma_link * fragmenent_tx_head;
	struct dma_link * fragmenent_tx_tail;
	u32* length;
	struct dma_fragment_cache* cache;
	struct list_head cache_list;
	/* TODO: preparedModificationList - for prepared statements */
};

/**
 * dma_fragment_alloc - allocate a new dma_fragment and initialize it empty
 * @gfpflags: the allocation flags
 */
struct dma_fragment* dma_fragment_alloc(gfp_t gfpflags);
/**
 * dma_fragment_free - allocate a new dma_fragment and initialize it empty
 * @fragment: the fragment to free
 */
void dma_fragment_free(struct dma_fragment *fragment);

/**
 * dma_fragment_add - add DMA controlblock to the fragment
 * @fragment: the fragment to which to add
 * @dmalink: the link object of the DMA controlblock to add
 */
int dma_fragment_add(struct dma_fragment *fragment,
	struct dma_link *dmalink);

/**
 * struct dma_fragment_cache - a cache of several fragments
 * @lock: lock for this structure
 * @name: name of the cache
 * @active: list of currently active fragments
 * @idle: list of currently idle fragments
 * @dmapool: the dmapool from which to allocate dma blocks (this implicitly defines the size of the DMA CB)
 * @allocateFragment: the allocation code for fragments
 */
struct dma_fragment_cache {
	spinlock_t         lock;
	const char         *name;
	struct list_head   active;
	struct list_head   idle;
	struct dma_pool    *dmapool;
	unsigned long      allocated;
	unsigned long      allocated_atomic;
	struct dma_fragment *(*allocateFragment)(struct dma_pool *,gfp_t);
};

/**
 * dma_fragment_cache_initialize: initialize the DMA Fragment cache
 * @cache: the cache to initialize
 * @name: name of cache
 * @allocateFragment: callback used to allocate a new fragment (without setting it up)
 * @pool: the dma pool from which to allocate the Control block memory
 * @initial_size: the initial size of the pool
 * note this needs to get run in "normal" context
 */
int dma_fragment_cache_initialize(struct dma_fragment_cache *cache,
				const char *name,
				struct dma_fragment *(*allocateFragment)(struct dma_pool *, gfp_t),
				struct dma_pool *pool,
				int initial_size
	);
/**
 * dma_fragment_cache_release: release the DMA Fragment cache
 * @cache: the cache to release
 * note that this assumes that this assumes that the DMA engine is not working these control blocks
 */
void dma_fragment_cache_release(struct dma_fragment_cache *cache);

#endif /* __DMAFRAGMENT_H */
