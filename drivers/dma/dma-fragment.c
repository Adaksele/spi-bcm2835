/*
 * Driver for DMA Fragments
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
#include <linux/kernel.h>
#include <linux/module.h>
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
EXPORT_SYMBOL_GPL(dma_link_alloc);

void dma_link_free(struct dma_link *block)
{
	if (!block->dmablock)
		return;

	list_del(&block->dma_link_chain);

	dma_pool_free(block->dmapool,block->dmablock,block->dmablock_dma);

	kfree(block);
}
EXPORT_SYMBOL_GPL(dma_link_free);

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
EXPORT_SYMBOL_GPL(dma_link_dump);

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
	INIT_LIST_HEAD(&frag->dma_link_chain);
	INIT_LIST_HEAD(&frag->dma_fragment_chain);

	return frag;
}
EXPORT_SYMBOL_GPL(dma_fragment_alloc);

void dma_fragment_free(struct dma_fragment *frag)
{
	struct dma_link *link;

	list_del(&frag->dma_fragment_chain);

	while( !list_empty(&frag->dma_link_chain)) {
		link = list_first_entry(&frag->dma_link_chain,
					typeof(*link),
					dma_link_chain);
		dma_link_free(link);
	}

	kfree(frag);
}
EXPORT_SYMBOL_GPL(dma_fragment_free);

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
			fragment->cache->dev_attr.attr.name);
	}
	/* dump the extra data */
	if (extrasize) {
		u32 *ptr = (u32*)((void*)fragment
				+ sizeof(struct dma_fragment));
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
	/* dump the fragments in dma_link_chain */
	list_for_each_entry(link,
			&fragment->dma_link_chain,
			dma_link_chain) {
		dma_link_dump("\t",dma_dump,link,flags);
	}
}
EXPORT_SYMBOL_GPL(dma_fragment_dump);

struct dma_fragment *dma_fragment_cache_add(
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
	cache->count_allocated ++;
	if (gfpflags == GFP_KERNEL)
		cache->count_allocated_kernel ++;
	/* add to corresponding list */
	if (flags && DMA_FRAGMENT_CACHE_TO_IDLE) {
		list_add(&frag->cache_list,&cache->idle);
		cache->count_idle++;
	} else {
		list_add(&frag->cache_list,&cache->active);
		cache->count_active++;
	}

	spin_unlock_irqrestore(&cache->lock,flags);
	/* and return it */
	return frag;
}
EXPORT_SYMBOL_GPL(dma_fragment_cache_add);

static ssize_t dma_fragment_cache_sysfs_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct dma_fragment_cache *cache =
		container_of(attr,typeof(*cache),dev_attr);
	ssize_t size;
	unsigned long flags;

	spin_lock_irqsave(&cache->lock,flags);
	size = scnprintf(buf, PAGE_SIZE,
			"dma_fragment_cache_info - 0.1\n"
			"name: %s\n"
			"count_active:\t%u\n"
			"count_idle:\t%u\n"
			"count_allocated:\t%u\n"
			"count_allocated_kernel:\t%u\n"
			"count_fetched:\t%lu\n"
			"count_removed:\t%u\n",
			cache->dev_attr.attr.name,
			cache->count_active,
			cache->count_idle,
			cache->count_allocated,
			cache->count_allocated_kernel,
			cache->count_fetched,
			cache->count_removed
		);
	spin_unlock_irqrestore(&cache->lock,flags);

	return size;
}

ssize_t dma_fragment_cache_sysfs_resize(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct dma_fragment_cache *cache =
		container_of(attr,typeof(*cache),dev_attr);
	s32 resize;
	int err;

	err=kstrtos32(buf,10,&resize);
	if (err)
		return -EPERM;

	err=dma_fragment_cache_resize(cache,resize);
	if (err<0)
		return err;

	return count;
}

#define SYSFS_PREFIX "dma_fragment_cache:"
int dma_fragment_cache_initialize(
	struct dma_fragment_cache *cache,
	const char* name,
	struct dma_fragment *(*allocateFragment)(
		struct device *, gfp_t),
	struct device *device,
	int initial_size
	)
{
	char *fullname;
	int i,err;

	memset(cache,0,sizeof(struct dma_fragment_cache));

	spin_lock_init(&cache->lock);
	INIT_LIST_HEAD(&cache->active);
	INIT_LIST_HEAD(&cache->idle);

	/* create name */
	i=sizeof(SYSFS_PREFIX)+strlen(name);
	fullname=kmalloc(i,GFP_KERNEL);
	if (!fullname)
		return -ENOMEM;
	strncpy(fullname,SYSFS_PREFIX,i);
	strncat(fullname,name,i);

	cache->device             = device;
	cache->dev_attr.attr.name = fullname;
	cache->dev_attr.attr.mode = S_IWUSR | S_IRUGO;
	cache->dev_attr.show      = dma_fragment_cache_sysfs_show;
	cache->dev_attr.store     = dma_fragment_cache_sysfs_resize;
	cache->allocateFragment   = allocateFragment;

	/* and expose the statistics on sysfs */
	err = device_create_file(device, &cache->dev_attr);
	if (err) {
		/* duplicate names result in errors */
		cache->dev_attr.show=NULL;
		dev_printk(KERN_ERR,cache->device,
			"duplicate dma_fragment_cache name \"%s\"\n",
			cache->dev_attr.attr.name);
		return err;
	}

	/* now allocate new entries to fill the pool */
	return dma_fragment_cache_resize(cache,initial_size);
}
EXPORT_SYMBOL_GPL(dma_fragment_cache_initialize);

int dma_fragment_cache_resize(struct dma_fragment_cache* cache,
	int size)
{
	int i;
	struct dma_fragment *frag;
	unsigned long flags;

	if (size == 0)
		return 0;
	/* add up to size */
	for (i = 0 ; i < size ; i++) {
		if (! dma_fragment_cache_add(cache,GFP_KERNEL,1) ) {
			return -ENOMEM;
		}
	}
	/* remove up to size */
	for (i = 0 ; i > size ; i--) {
		spin_lock_irqsave(&cache->lock,flags);

		/* return error on empty */
		if ( list_empty(&cache->idle) ) {
			spin_unlock_irqrestore(&cache->lock,flags);
			return -ENOMEM;
		}

		/* now get object from idle and release it */
		frag = list_first_entry(&cache->idle,struct dma_fragment,
					cache_list);
		list_del(&frag->cache_list);

		cache->count_idle--;
		cache->count_removed++;

		spin_unlock_irqrestore(&cache->lock,flags);
		/* and free it */
		dma_fragment_free(frag);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(dma_fragment_cache_resize);


void dma_fragment_cache_release(struct dma_fragment_cache* cache)
{
	unsigned long flags;
	struct dma_fragment *frag;

	spin_lock_irqsave(&cache->lock,flags);

	while( !list_empty(&cache->idle)) {
		frag = list_first_entry(&cache->idle,struct dma_fragment,
					cache_list);
		list_del(&frag->cache_list);
		dma_fragment_free(frag);
	}
	cache->count_idle=0;

	if (! list_empty(&cache->active))
		dev_printk(KERN_ERR,cache->device,
			"the dma_fragment_cache %s is not totally idle"
			" it contains %u entries\n",
			cache->dev_attr.attr.name,
			cache->count_active);

	spin_unlock_irqrestore(&cache->lock,flags);

	/* release sysfs info file */
	if (cache->dev_attr.show)
		device_remove_file(cache->device,
				&cache->dev_attr);

	/* and release name */
	kfree(cache->dev_attr.attr.name);
}
EXPORT_SYMBOL_GPL(dma_fragment_cache_release);

MODULE_DESCRIPTION("generic dma-fragment infrastructure");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL");
