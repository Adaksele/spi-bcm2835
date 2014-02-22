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
 * @cb:            the pointer to the DMA control-block
 * @cb_dma:        the dma-bus address of the control block
 * @pool:          from which dma_pool the control-block has been taken
 * @fragment:      the fragment to which we belong
 * @dma_link_list: the list of linked dma_links to which we belong
 */
struct dma_fragment;
struct dma_link {
	/* the control block itself */
	void                *cb;
	dma_addr_t          cb_dma;
	size_t              size;
	/* the pool from which this has been allocated */
	struct dma_pool     *pool;
	/* the membership to a fragment */
	struct dma_fragment *fragment;
	struct list_head    dma_link_list;
};

/**
 * dma_link_alloc
 * @dev: the device for which we allocate this.
 *   this typically contains a dma_pool structure somewhere to use)
 * @dmapool: the dma pool from which this is to get allocated
 * @gfpflags: the flags to use for allocation
 */
struct dma_link *dma_link_alloc(struct dma_pool *pool,
				size_t size,
				gfp_t gfpflags);

/**
 * dma_link_free
 * @block: the object to free
 */
void dma_link_free(struct dma_link *block);

/**
 * dma_link_dump - dump the dma_link object
 * @link: the dma_link to dump
 * @dev: device used for printing
 * @tindent: the number of tab indents to add
 * @dma_cb_dump: optional dump function for the dma control block itself
 */
void dma_link_dump(
	struct dma_link *link,
	struct device *dev,
	int tindent,
	void (*dma_cb_dump)(struct dma_link *, struct device *, int)
	);

/**
 * dma_fragment_transform_channel - the transform happens during this phase
 * @DMA_FRAGMENT_TRANSFORM_TYPE_LINK: when linking dma_fragments
 * @DMA_FRAGMENT_TRANSFORM_TYPE_PRE_EXE: prior to executing the fragment
 * @DMA_FRAGMENT_TRANSFROM_TYPE_POST_EXE: after executing the fragment
 */
enum dma_fragment_transform_type {
	DMA_FRAGMENT_TRANSFORM_TYPE_LINK     = 0,
	DMA_FRAGMENT_TRANSFROM_TYPE_PRE_EXE  = 1,
	DMA_FRAGMENT_TRANSFROM_TYPE_POST_EXE = 2,
};

/**
 * dma_fragment_transform - an action taken with certain arguments
 * @transform_list: list of transforms to get executed in sequence
 * @extra_transform_list: list of transforms that to get executed
 *   in sequence
 * @src: source pointer for transform
 * @dst: destination pointer for transform
 * @extra: some extra information
 * @type: the type of transform
 */
struct dma_fragment_transform {
	struct list_head transform_list;
	int (*transform)(struct dma_fragment_transform *,
			void* data1, void* data2);
	void *src;
	void *dst;
	void *extra;
	enum dma_fragment_transform_type type;
};

static inline void dma_fragment_transform_init(
	struct dma_fragment_transform *trans,
	enum dma_fragment_transform_type type,
	int (*transform)(struct dma_fragment_transform *,
			void *, void *),
	void *src,
	void *dst,
	void *extra)
{
	INIT_LIST_HEAD(&trans->transform_list);
	trans->type      = type;
	trans->transform = transform;
	trans->src       = src;
	trans->dst       = dst;
	trans->extra     = extra;
}

struct dma_fragment_transform *dma_fragment_transform_alloc(
	enum dma_fragment_transform_type type,
	int (*transform)(struct dma_fragment_transform *,void*, void*),
	void *src,void *dst,void *extra,
	gfp_t gfpflags);

/**
 * dma_fragment - a collection of connected dma_links
 * @size: size of this fragment (may be embedded)
 * @cache: the dma_fragment_cache to which this fragment belongs
 * @cache_list: the list inside the dma_fragment_cache
 * @dma_link_list: the list of dma_links
 * @link_transform_list: list of tasks that need to get run when linking
 *   the fragment together
 * @pre_transform_list: list of tasks that need to get run prior to
 *   the fragment getting scheduled
 * @post_transform_list: list of tasks that need to get run after the
 *   the fragment dma finished.
 */
struct dma_fragment {
	size_t size;
	void* cache;
	struct list_head cache_list;
	struct list_head dma_link_list;

	struct list_head transform_list;
};

/**
 * dma_fragment_init - initialize the dma_fragment
 * @fragment: fragment to initialize
 * @size: real-size of fragment
 */
static inline void dma_fragment_init(struct dma_fragment* fragment,
				size_t size)
{
	size = max( size, sizeof(*fragment) );
	memset(fragment,0,size);
	fragment->size=size;

	INIT_LIST_HEAD(&fragment->cache_list);
	INIT_LIST_HEAD(&fragment->dma_link_list);

	INIT_LIST_HEAD(&fragment->transform_list);
}

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
 * note:
 *   this call does NOT disconnect from fragment_cache
 */
void dma_fragment_free(struct dma_fragment *fragment);

/**
 * dma_fragment_add_dma_link - add DMA controlblock to the fragment
 * @fragment: the fragment to which to add
 * @dmalink: the link object of the DMA controlblock to add
 */
static inline void dma_fragment_add_dma_link(struct dma_fragment *fragment,
				struct dma_link *dmalink)
{
	list_add(&dmalink->dma_link_list, &fragment->dma_link_list);
}
/**
 * dma_fragment_add_dma_transform - add DMA transform to the fragment
 * @fragment: the fragment to which to add
 * @dmalink: the link object of the DMA controlblock to add
 */
static inline void dma_fragment_add_dma_fragment_transform(
	struct dma_fragment *fragment,
	struct dma_fragment_transform *transform
	)
{
	list_add_tail(&transform->transform_list,
		&fragment->transform_list);
}

/**
 * dma_fragment_dump - dump the given fragment
 * @fragment: the fragment to dump
 * @tindent: the number of tab indents to add
 * @flags: the flags for dumping the fragment
 * @dma_link_dump: the function which to use to dump the dmablock
 */
void dma_fragment_dump(struct dma_fragment *fragment,
		struct device *dev,
		int tindent,
		int flags,
		void (*dma_fragment_dump)(struct dma_fragment *,
					struct device *,int),
		void (*dma_cb_dump)(struct dma_link *,
				struct device *,int)
	);

/**
 * struct dma_fragment_cache - cache of several dma_fragments
 *   used to avoid setup costs of memory allocation and initialization
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
	/* the device and device_attribute used for sysfs */
	struct device      *device;
	struct device_attribute dev_attr;

	spinlock_t         lock;

	/* idle/active list */
	struct list_head   active;
	struct list_head   idle;

	/* the counters exposed via sysfs */
	u32                count_active;
	u32                count_idle;
	u32                count_allocated;
	u32                count_allocated_kernel;
	unsigned long      count_fetched;
	u32                count_removed;

	/* allocation function */
	struct dma_fragment *(*allocateFragment)(struct device *, gfp_t);
};

/**
 * dma_fragment_cache_initialize - initialize a dma_fragment cache
 * @cache:            the cache to initialize
 * @device:           the device for which we run this cache
 * @name:             the identifier of the dma_fragment_cache
 * @allocateFragment: the fragment allocation/initialization function
 * @initial_size:     the initial size of the cache
 */
int dma_fragment_cache_initialize(
	struct dma_fragment_cache *cache,
	struct device *device,
	const char *name,
	struct dma_fragment *(*allocateFragment)(struct device *, gfp_t),
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
#define DMA_FRAGMENT_CACHE_TO_IDLE (1<<0)
struct dma_fragment *dma_fragment_cache_add(
	struct dma_fragment_cache *cache,
	gfp_t gfpflags,
	int flags);

/**
 * dma_fragment_cache_resize - add/remove items from the idle list
 * @cache: the cache to resize
 * @resizeby: increase/decrease the number of items in idle by this number
 */
int dma_fragment_cache_resize(
	struct dma_fragment_cache *cache,
	int resizeby
	);

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
