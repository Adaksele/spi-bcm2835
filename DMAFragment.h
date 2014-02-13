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

struct DMALinkBlock;
struct DMAFragment;
struct DMAFragmentCache;

/**
 * struct DMALinkBlock - the linked list of DMA control blocks
 * @dmablock: the pointer to the DMA control-block itself (void pointer to be of generic use)
 * @dmablock_dma: the dma-buss address of the control block
 * @dmapool: from which pool the block has been taken
 * @linked_list: the linked list connecting the individual link blocks
 */
struct DMALinkBlock {
	void              *dmablock;
	dma_addr_t        dmablock_dma;
	struct dma_pool   *dmapool;
	struct list_head  linked_list;
};

/**
 * DMALINKBLOCK_FIELD_DMAADDR() - helper macro to get the bus address of the specific field
 * @block: the link block for which we need the DMA address
 * @field: the field name we want to get the DMA-address for
 */
#define DMALINKBLOCK_FIELD_DMAADDR(block,field)		\
	(							\
		block->dmablock_dma+				\
		offsetof(typeof(*block->dmablock),field)	\
		)

/**
 * DMALinkBlock_alloc
 * @dmapool: the dmapool from which we have to take the block (the pool defines the size explicitly)
 * @gfpflags: the flags to use for allocation
 */
struct DMALinkBlock *DMALinkBlock_alloc(struct dma_pool *pool,gfp_t gfpflags);

/**
 * DMALinkBlock_free
 * @block: the object to free
 */
void DMALinkBlock_free(struct DMALinkBlock *block);

/**
 * struct DMAFragment - a list of DMALinkBlocks with means how to link Fragments quickly
 * @fragment_head: the pointer to the head of the DMALinkBlock list for the mixed dma-queue (rx/tx)
 * @fragment_tail: the pointer to the tail of the DMALinkBlock list for the mixed dma-queue (rx/tx)
 * @fragment_rx_head: the pointer to the head of the DMALinkBlock list for the rx_queue
 * @fragment_rx_tail: the pointer to the tail of the DMALinkBlock list for the rx_queue
 * @fragment_tx_head: the pointer to the head of the DMALinkBlock list for the tx_queue
 * @fragment_tx_tail: the pointer to the tail of the DMALinkBlock list for the tx_queue
 * @length_ptr: the length pointer for the transfer in this transfer block
 * @cache: the DMAFragmentCache to which this fragment belongs to (mostly used to return the fragment to the cache
 * @cache_list: the linked list for the fragment cache
 *
 */
struct DMAFragment {
	struct DMALinkBlock * fragmenent_head;
	struct DMALinkBlock * fragmenent_tail;
	struct DMALinkBlock * fragmenent_rx_head;
	struct DMALinkBlock * fragmenent_rx_tail;
	struct DMALinkBlock * fragmenent_tx_head;
	struct DMALinkBlock * fragmenent_tx_tail;
	u32* length;
	struct DMAFragmentCache* cache;
	struct list_head cache_list;
	/* TODO: preparedModificationList - for prepared statements */
};

/**
 * DMAFragment_alloc - allocate a new DMAFragment and initialize it empty
 * @gfpflags: the allocation flags
 */
struct DMAFragment* DMAFragment_alloc(gfp_t gfpflags);
/**
 * DMAFragment_free - allocate a new DMAFragment and initialize it empty
 * @fragment: the fragment to free
 */
void DMAFragment_free(struct DMAFragment *fragment);

/**
 * DMAFragment_add - add DMA controlblock to the fragment
 * @fragment: the fragment to which to add
 * @dmalink: the link object of the DMA controlblock to add
 */
int DMAFragment_add(struct DMAFragment *fragment,
	struct DMALinkBlock *dmalink);

/**
 * struct DMAFragmentCache - a cache of several fragments
 * @lock: lock for this structure
 * @name: name of the cache
 * @active: list of currently active fragments
 * @idle: list of currently idle fragments
 * @dmapool: the dmapool from which to allocate dma blocks (this implicitly defines the size of the DMA CB)
 * @allocateFragment: the allocation code for fragments
 */
struct DMAFragmentCache {
	spinlock_t         lock;
	const char         *name;
	struct list_head   active;
	struct list_head   idle;
	struct dma_pool    *dmapool;
	unsigned long      allocated;
	unsigned long      allocated_atomic;
	struct DMAFragment *(*allocateFragment)(struct dma_pool *,gfp_t);
};

/**
 * DMAFragmentCache_initialize: initialize the DMA Fragment cache
 * @cache: the cache to initialize
 * @name: name of cache
 * @allocateFragment: callback used to allocate a new fragment (without setting it up)
 * @pool: the dma pool from which to allocate the Control block memory
 * @initial_size: the initial size of the pool
 * note this needs to get run in "normal" context
 */
int DMAFragmentCache_initialize(struct DMAFragmentCache *cache,
				const char *name,
				struct DMAFragment *(*allocateFragment)(struct dma_pool *, gfp_t),
				struct dma_pool *pool,
				int initial_size
	);
/**
 * DMAFragmentCache_release: release the DMA Fragment cache
 * @cache: the cache to release
 * note that this assumes that this assumes that the DMA engine is not working these control blocks
 */
void DMAFragmentCache_release(struct DMAFragmentCache* cache);

#endif /* __DMAFRAGMENT_H */
