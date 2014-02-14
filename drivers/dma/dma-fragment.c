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
#include <linux/dma-fragment.h>
#include <linux/slab.h>

struct dma_link *dma_link_alloc(struct dma_pool *pool,gfp_t gfpflags) {

	struct dma_link *block = kmalloc(sizeof(*block),gfpflags);
	if (!block)
		return NULL;
	block->dmapool = pool;

	block->fragment = NULL;
	INIT_LIST_HEAD(&block->fragment_dma_link_chain);

	INIT_LIST_HEAD(&block->dma_link_chain);

	block->dmablock = dma_pool_alloc(block->dmapool,
					gfpflags,
					&block->dmablock_dma);
	if (!block->dmablock) {
		kfree(block);
		return NULL;
	}

	return block;
}

void dma_link_free(struct dma_link *block) {
	if (!block->dmablock)
		return;

	list_del(&block->fragment_dma_link_chain);
	list_del(&block->dma_link_chain);

	dma_pool_free(block->dmapool,block->dmablock,block->dmablock_dma);

	kfree(block);
}

struct dma_fragment *dma_fragment_alloc(gfp_t gfpflags,size_t size) {
	struct dma_fragment *frag;
	frag=kzalloc(max(size,sizeof(*frag)),gfpflags);
	if (! frag)
		return NULL;
	
	INIT_LIST_HEAD(&frag->cache_list);
	INIT_LIST_HEAD(&frag->fragment_dma_link_chain);
	INIT_LIST_HEAD(&frag->dma_link_chain);

	return frag;
}

void dma_fragment_free(struct dma_fragment *frag) {
	struct dma_link * link;

	while( !list_empty(&frag->fragment_dma_link_chain)) {
		link = list_first_entry(&frag->fragment_dma_link_chain,
					typeof(*link),
					fragment_dma_link_chain);
		dma_link_free(link);
	}

	kfree(frag);
}

int dma_fragment_add(struct dma_fragment *fragment,
		struct dma_link *dmalink) {
	list_add(
		&dmalink->fragment_dma_link_chain,
		&fragment->fragment_dma_link_chain
		);
	list_add(
		&dmalink->dma_link_chain,
		&fragment->dma_link_chain
		);
	return 0;
}

static struct dma_fragment *dma_fragment_cache_add(struct dma_fragment_cache *cache, gfp_t gfpflags,int toidle);

int dma_fragment_cache_initialize(struct dma_fragment_cache *cache,
				const char* name,
				struct dma_fragment *(*allocateFragment)(struct dma_pool *, gfp_t),
				struct dma_pool *pool,
				int initial_size
	) {
	int i;

	spin_lock_init(&cache->lock);
	INIT_LIST_HEAD(&cache->active);
	INIT_LIST_HEAD(&cache->idle);

	cache->name=name;
	cache->dmapool=pool;
	cache->allocateFragment=allocateFragment;

	/* now allocate new entries to fill the pool */
	for (i = 0 ; i < initial_size ; i++) {
		if (! dma_fragment_cache_add(cache,GFP_KERNEL,1))
			return -ENOMEM;
	}

	return 0;
}

void dma_fragment_cache_release(struct dma_fragment_cache* cache) {
	unsigned long flags;
	struct dma_fragment *frag;

	spin_lock_irqsave(&cache->lock,flags);

	while( !list_empty(&cache->idle)) {
		frag = list_first_entry(&cache->idle,struct dma_fragment, cache_list);
		list_del(&frag->cache_list);
		dma_fragment_free(frag);
	}

	if (! list_empty(&cache->active))
		printk(KERN_ERR "the dma_fragment_cache %s is not totally idle\n",cache->name);

	spin_unlock_irqrestore(&cache->lock,flags);

	/* we could expose this statistics via sysfs - for now just when unloading the module */
	printk(KERN_INFO "The DMA Fragment cache for %s has had %lu fragments created "
		"out of which %lu were created without the GPF_KERNEL flag\n",
		cache->name,cache->allocated,cache->allocated_atomic);
}

static struct dma_fragment *dma_fragment_cache_add(struct dma_fragment_cache *cache, gfp_t gfpflags,int toidle) {
	unsigned long flags;

	struct dma_fragment *frag=cache->allocateFragment(
		cache->dmapool,
		gfpflags);
	if (!frag)
		return NULL; 

	frag->cache=cache;

	spin_lock_irqsave(&cache->lock,flags);

	/* gather statistics */
	cache->allocated ++;
	if (gfpflags != GFP_KERNEL)
		cache->allocated_atomic ++;
	/* add to corresponding list */
	if (toidle)
		list_add(&frag->cache_list,&cache->idle);
	else
		list_add(&frag->cache_list,&cache->active);
	
	spin_unlock_irqrestore(&cache->lock,flags);
	/* and return it */
	return frag;
}
