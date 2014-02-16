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
#include <linux/dma-fragment.h>
#include <linux/device.h>
#include <linux/slab.h>

struct dma_link *dma_link_alloc(struct device *dev,
				struct dma_pool *pool,
				u32 dmachannel,
				gfp_t gfpflags)
{
	struct dma_link *block = kmalloc(sizeof(*block),gfpflags);
	if (!block)
		return NULL;
	block->dmapool = pool;
	block->dmachannel = dmachannel;
	block->device  = dev;

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

void dma_link_free(struct dma_link *block)
{
	if (!block->dmablock)
		return;

	list_del(&block->fragment_dma_link_chain);
	list_del(&block->dma_link_chain);

	dma_pool_free(block->dmapool,block->dmablock,block->dmablock_dma);

	kfree(block);
}

void dma_link_dump(char* prefix,
		void (*dma_dump)(
			char *prefix,
			struct dma_link *link,
			int flags),
		struct dma_link *link,
		int flags)
{
	struct device *dev=link->device;

	char indent[16];
	snprintf(indent,sizeof(indent),"%s\t",prefix);

	dev_printk(KERN_INFO,dev,
		"%sdma_link at: %pK\n",
		prefix,link);
	dma_dump(indent,link,flags);
}


struct dma_fragment *dma_fragment_alloc(
	struct device *device,
	gfp_t gfpflags,size_t size)
{
	struct dma_fragment *frag;
	size_t s=max(size,sizeof(*frag));
	frag=kzalloc(s,gfpflags);
	if (! frag)
		return NULL;

	frag->size=s;
	frag->device=device;
	INIT_LIST_HEAD(&frag->cache_list);
	INIT_LIST_HEAD(&frag->fragment_dma_link_chain);
	INIT_LIST_HEAD(&frag->dma_link_chain);

	return frag;
}

void dma_fragment_free(struct dma_fragment *frag)
{
	struct dma_link *link;

	while( !list_empty(&frag->fragment_dma_link_chain)) {
		link = list_first_entry(&frag->fragment_dma_link_chain,
					typeof(*link),
					fragment_dma_link_chain);
		dma_link_free(link);
	}

	kfree(frag);
}

int dma_fragment_add(
	struct dma_fragment *fragment,
	struct dma_link *dmalink)
{
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

void dma_fragment_dump(struct dma_fragment *fragment,
		void (*dma_dump)(
			char *prefix,
			struct dma_link *block,
			int flags),
		int flags)
{
	struct dma_link *link;
	struct device *dev=fragment->device;
	size_t extrasize=fragment->size-sizeof(struct dma_fragment);
	dev_printk(KERN_INFO,dev,
		"Fragment at: %pK\n",
		fragment);
	dev_printk(KERN_INFO,dev,
		"\tcache:\t%pK\n",
		fragment->cache);
	if (fragment->cache) {
		dev_printk(KERN_INFO,dev,
			"\tcachen:\t%s\n",
			fragment->cache->name);
	}
	/* dump the extra data */
	if (extrasize) {
		u32 *ptr=(u32*)((void*)fragment+sizeof(struct dma_fragment));
		int i;
		dev_printk(KERN_INFO,dev,
			"\textra\tsize: %i\n",
			extrasize);
		for( i=0 ; extrasize>0 ; i++ , ptr++ , extrasize-=4) {
		dev_printk(KERN_INFO,dev,
			"\t\tdata[%2i]:\t%08x\n",
			i,*ptr);
		}
	}
	/* dump the fragments in fragment_dma_link_chain */
	list_for_each_entry(link,
			&fragment->fragment_dma_link_chain,
			fragment_dma_link_chain) {
		dma_link_dump("\t",dma_dump,link,flags);
	}
}

static struct dma_fragment *dma_fragment_cache_add(
	struct dma_fragment_cache *cache,
	gfp_t gfpflags,
	int toidle)
{
	unsigned long flags;

	struct dma_fragment *frag=cache->allocateFragment(
		cache->device,
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

int dma_fragment_cache_initialize(
	struct dma_fragment_cache *cache,
	const char* name,
	struct dma_fragment *(*allocateFragment)(
		struct device *, gfp_t),
	struct device *device,
	int initial_size
	)
{
	int i;

	spin_lock_init(&cache->lock);
	INIT_LIST_HEAD(&cache->active);
	INIT_LIST_HEAD(&cache->idle);

	cache->name=name;
	cache->device=device;
	cache->allocateFragment=allocateFragment;

	/* now allocate new entries to fill the pool */
	for (i = 0 ; i < initial_size ; i++) {
		if (! dma_fragment_cache_add(cache,GFP_KERNEL,1))
			return -ENOMEM;
	}

	/* and expose the statistics on sysfs */

	return 0;
}

void dma_fragment_cache_release(struct dma_fragment_cache* cache)
{
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

struct dma_fragment *dma_fragment_cache_fetch(
	struct dma_fragment_cache *cache,gfp_t gfpflags) 
{
	return NULL;
}

void dma_fragment_cache_return(
	struct dma_fragment_cache *cache,struct dma_fragment *fragment)
{
}
