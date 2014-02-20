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
 *
 * 4567890123456789012345678901234567890123456789012345678901234567890123456789
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
 * @dmablock: the pointer to the DMA control-block itself
 *    (void pointer to be of generic use)
 * @dmablock_dma: the dma-buss address of the control block
 * @dmachannel: the dma channel for which this is scheduled
 * @dmapool: from which pool the block has been taken
 * @fragment: the fragment to which we belong
 * @dma_link_chain: the link chain to which we belong
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
 * struct dma_fragment - a list of dma_links with means to link
 *    Fragments quickly
 * @cache: to which fragment cache we belong
 * @cache_list: the list to link the objects in the cache
 * @dma_link_chain: the link chain connecting all the dma_link
 *   objects belonging to this fragment
 * @dma_fragment_chain: a chain of several dma-fragments all belonging
 *   together for a full transaction
 *   - this also means the dma-control-blocks are linked to each other
 * @size: the size of this object
 * @device: the device to which this fragment belongs
 */
struct dma_fragment {
	struct dma_fragment_cache* cache;
	struct list_head           cache_list;
	struct list_head           dma_link_chain;
	size_t                     size;
	struct device              *device;
	struct list_head           dma_fragment_chain;
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
static inline void dma_fragment_add(struct dma_fragment *fragment,
				struct dma_link *dmalink)
{
	list_add(
		&dmalink->dma_link_chain,
		&fragment->dma_link_chain
		);
}

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
 * dma_fragment_composite - this is a composit dma fragment, that combines
 *   several fragments to make up a full SPI transfer
 *   it can be a prepared version, in which case we also run the
 *   message transforms
 * @fragment: the normal dma_fragment
 * @is_prepared: flags that the dma_fragment is prepared already
 * @message_pre_transform_chain: list of transforms to do prior
 *   to scheduling the dma fragment
 * @message_post_transform_chain: list of transforms to do after
 *   the DMA has finished
 */
struct dma_fragment_composite {
	struct dma_fragment fragment;
	int is_prepared;
	struct list_head    message_pre_transform_chain;
	struct list_head    message_post_transform_chain;
};

/**
 * dma_fragment_composite_add: add a dma_fragment to the composite
 * @fragment: the fragment to add
 * @composite: the composite to which to add
 */
static inline void dma_fragment_composite_add(
	struct dma_fragment *fragment,
	struct dma_fragment_composite *composite)
{
	list_add(&fragment->dma_fragment_chain,
		&composite->fragment.dma_fragment_chain);
}

/**
 * dma_fragment_composite_remove: remove a dma_fragment from a composite
 */
static inline void dma_fragment_composite_remove(
	struct dma_fragment* fragment)
{
	list_del_init(&fragment->dma_fragment_chain);
	INIT_LIST_HEAD(&fragment->dma_fragment_chain);
}

/**
 * struct dma_fragment_cache - a cache of several fragments
 * @device: the device to which this cache belongs
 * @dev_attr: the device attributes of this cache (includes the name)
 * @lock: lock for this structure
 * @active: list of currently active fragments
 * @idle: list of currently idle fragments
 * @count_active: number of currently active fragments
 * @count_idle: number of currently idle fragments
 * @count_allocated: number of allocated objects
 * @count_allocated_kernel: number of allocated objects with GFP_KERNEL
 * @count_fetched: number of fetches from this cache
 * @allocateFragment: the allocation code for fragments
 */
struct dma_fragment_cache {
	struct device      *device;
	struct device_attribute dev_attr;

	spinlock_t         lock;

	struct list_head   active;
	struct list_head   idle;

	u32                count_active;
	u32                count_idle;
	u32                count_allocated;
	u32                count_allocated_kernel;
	unsigned long      count_fetched;

	struct dma_fragment *(*allocateFragment)(struct device *,gfp_t);
};

/**
 * dma_fragment_cache_initialize: initialize the DMA Fragment cache
 * @cache: the cache to initialize
 * @name: name of cache
 * @allocateFragment: callback used to allocate a new fragment
 *       initilaizing it with generic values that are not changed
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
 * note that this assumes that this assumes that the DMA-HW
 * is not working with these control blocks
 */
void dma_fragment_cache_release(struct dma_fragment_cache *cache);

/**
 * dma_fragment_cache_add - add an item to the dma cache
 * @cache: the cache to which to add an item
 * @gfpflags: the flags used for allocation
 * @flags: some allocation flags.
 */
struct dma_fragment *dma_fragment_cache_add(
	struct dma_fragment_cache *cache,
	gfp_t gfpflags,
	int flags);
#define DMA_FRAGMENT_CACHE_TO_IDLE (1<<0)

/**
 * dma_fragment_cache_fetch - fetch an object from dma_fragment_cache
 *   creating new ones if needed
 * @cache: the cache from which to fetch
 * @gfpflags: flags to use in case we need to allocate a new object
 */
static inline struct dma_fragment *dma_fragment_cache_fetch(
	struct dma_fragment_cache *cache,gfp_t gfpflags)
{
	unsigned long flags;
	int is_empty;
	struct dma_fragment *frag = NULL;

	/* fetch from cache if it exists*/
	spin_lock_irqsave(&cache->lock,flags);
	is_empty = list_empty(&cache->idle);
	if (!is_empty) {
		frag = list_first_entry(
			&cache->idle,
			typeof(*frag),
			cache_list);
		list_move(&frag->cache_list,&cache->active);
		is_empty = list_empty(&cache->idle);
	}
	cache->count_fetched++;
	spin_unlock_irqrestore(&cache->lock,flags);

	/* allocate fragment outside of lock and add to active queue */
	if (!frag)
		frag = dma_fragment_cache_add(cache,gfpflags,0);

	/* if not in GFP_KERNEL context, then return immediately */
	if (gfpflags == GFP_KERNEL) {
		/* allocate one more to keep something idle in cache */
		if (is_empty)
			dma_fragment_cache_add(cache,gfpflags,1);
	}

	return frag;
}

/**
 * dma_fragment_cache_return - return an object to dma_fragment_cache
 * @cache - the cache in question
 * @fragment - the dma_fragment to return
 */
static inline void dma_fragment_cache_return(
	struct dma_fragment_cache *cache,struct dma_fragment *fragment)
{
	unsigned long flags;
	spin_lock_irqsave(&cache->lock,flags);
	list_move(&fragment->cache_list,&cache->idle);
	spin_unlock_irqrestore(&cache->lock,flags);
}

#endif /* __DMAFRAGMENT_H */
