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
struct dma_fragment;
struct dma_fragment_cache;

/**
 * struct dma_link - the linked list of DMA control blocks
 * @dmablock: the pointer to the DMA control-block itself (void pointer to be of generic use)
 * @dmablock_dma: the dma-buss address of the control block
 * @dmachannel: the dma channel for which this is scheduled
 * @dmapool: from which pool the block has been taken
 * @fragment: the fragment to which we belong
 * @fragment_dma_link_chain: the link chain to which we belong
 * @dma_link_chain: the active DMA link chain to which we belong
 */

struct dma_link {
	/* the control block itself */
	void                *dmablock;
	dma_addr_t          dmablock_dma;
        /* dma channel */
	u32                 dmachannel;
	/* the pool from which this has been allocated */
	struct device       *device;
	struct dma_pool     *dmapool;
	/* the membership to a fragment */
	struct dma_fragment *fragment;
	struct list_head    fragment_dma_link_chain;
	/* and for the (active) dma chain itself */
	struct list_head    dma_link_chain;
};

/**
 * dma_link_alloc
 * @dev: the device for which we allocate this.
 *   this typically contains a dma_pool structure somewhere to use)
 * @dmapool: the dma pool from which this is to get allocated
 * @dmachannel: the dma channel on which this is scheduled
 * @gfpflags: the flags to use for allocation
 */
struct dma_link *dma_link_alloc(struct device *dev,
				struct dma_pool*pool,
				u32 dmachannel,
				gfp_t gfpflags);

/**
 * dma_link_free
 * @block: the object to free
 */
void dma_link_free(struct dma_link *block);

/**
 * dma_link_dump - dump the dma_link object
 * @prefix: prefix used for output
 * @dma_dump: dump function for the dmablock
 * @block: the dma_link to dump
 * @flags: some flags (for extensions)
 */
void dma_link_dump(char* prefix,
		void (*dma_dump)(
			char *prefix,
			struct dma_link *block,
			int flags),
		struct dma_link *block,
		int flags
	);

/**
 * struct dma_fragment - a list of dma_links with means how to link Fragments quickly
 * @cache: to which fragment cache we belong
 * @cache_list: the list to link the objects in the cache
 * @fragment_dma_link_chain: the link chain connecting all the dma_link
     objects belonging to this fragment
 * @dma_link_chain: the link chain connecting all the dma_link
     objects belonging to this fragment - used for real dma scheduling
 * @size: the size of this object
 * @device: the device to which this fragment belongs
 */
struct dma_fragment {
	struct dma_fragment_cache* cache;
	struct list_head cache_list;
	struct list_head fragment_dma_link_chain;
	struct list_head dma_link_chain;
	size_t           size;
	struct device    *device;
	/* TODO: preparedModificationList - for prepared statements */
};

/**
 * dma_fragment_alloc - allocate a new dma_fragment and initialize it empty
 * @device: the device of this fragment
 * @gfpflags: the allocation flags
 * @size: the size to really allocate
 */
struct dma_fragment* dma_fragment_alloc(
	struct device *device,
	gfp_t gfpflags, size_t size);
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
 * dma_fragment_dump - dump the given fragment
 * @fragment: the fragment to dump
 * @dma_link_dump: the function which to use to dump the dmablock
 * @flags: the flags for dumping the fragment
 */
void dma_fragment_dump(struct dma_fragment *fragment,
		void (*dma_dump)(
			char* prefix,
			struct dma_link *block,
			int flags),
		int flags);

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
	struct device      *device;
	unsigned long      allocated;
	unsigned long      allocated_atomic;
	struct dma_fragment *(*allocateFragment)(struct device *,gfp_t);
};

/**
 * dma_fragment_cache_initialize: initialize the DMA Fragment cache
 * @cache: the cache to initialize
 * @name: name of cache
 * @allocateFragment: callback used to allocate a new fragment (without setting it up)
 * @device: the device for which we do this
 * @initial_size: the initial size of the pool
 * note this needs to get run in "normal" context
 */
int dma_fragment_cache_initialize(struct dma_fragment_cache *cache,
				const char *name,
				struct dma_fragment *(*allocateFragment)(
					struct device *, gfp_t),
				struct device *device,
				int initial_size
	);
/**
 * dma_fragment_cache_release: release the DMA Fragment cache
 * @cache: the cache to release
 * note that this assumes that this assumes that the DMA engine is not working these control blocks
 */
void dma_fragment_cache_release(struct dma_fragment_cache *cache);

struct dma_fragment *dma_fragment_cache_fetch(
	struct dma_fragment_cache *cache,gfp_t gfpflags);

void dma_fragment_cache_return(
	struct dma_fragment_cache *cache,struct dma_fragment *fragment);

#endif /* __DMAFRAGMENT_H */
