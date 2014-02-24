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

/**
 * _tab_indent - helper function to return a tabbed indent string
 * @indentdepth: the depth/tabs to return  in string
 * returns string
 */
static const char *_tab_indent_string = "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";

static inline const char *_tab_indent(int indent) {
	return &_tab_indent_string[16-min(16,indent)];
}

static inline void _dump_extra_data(char* data, int size,
				struct device *dev,int tindent)
{
	char   buffer[50];
	int offset=0;
	int bytes_per_line = 16;
	while (size>0) {
		hex_dump_to_buffer(
			data, min(bytes_per_line,size),
			bytes_per_line, 1,
			buffer,sizeof(buffer),
			0
			);
		dev_printk(KERN_INFO,dev,
			"%sdata %02x: %s\n",
			_tab_indent(tindent),
			offset,buffer
			);
		data   += bytes_per_line;
		size   -= bytes_per_line;
		offset += bytes_per_line;
	}
}

struct dma_link *dma_link_alloc(struct dma_pool *pool,
				size_t size,
				gfp_t gfpflags)
{
	struct dma_link *dmalink = kzalloc(
		max(sizeof(*dmalink),size),
		gfpflags);
	if (!dmalink)
		return NULL;

	INIT_LIST_HEAD(&dmalink->dma_link_list);

	dmalink->pool = pool;

	dmalink->cb = dma_pool_alloc(
		dmalink->pool,
		gfpflags,
		&dmalink->cb_dma);
	if (!dmalink->cb) {
		kfree(dmalink);
		return NULL;
	}

	return dmalink;
}
EXPORT_SYMBOL_GPL(dma_link_alloc);

void dma_link_free(struct dma_link *dmalink)
{
	if (!dmalink->cb)
		return;

	list_del(&dmalink->dma_link_list);

	dma_pool_free(dmalink->pool,
		dmalink->cb,
		dmalink->cb_dma);

	kfree(dmalink);
}

EXPORT_SYMBOL_GPL(dma_link_free);

/**
 * dma_link_cb_dump_generic - generic dma control block dumper
 *   only dumps the binary data
 * @link: the dma_link
 * @dev: the device to dump
 * @tindent: the number of tabs indenting
 */
static void dma_link_cb_dump_generic(
		struct dma_link *link,
		struct device *dev,
		int tindent)
{
	const char *indent=_tab_indent(tindent);
	dev_printk(KERN_INFO,dev, "%scb_addr:\t%pK\n",
		indent,
		link->cb);
	dev_printk(KERN_INFO,dev, "%scb_dma:\t\t%08lx\n",
		indent,
		(long unsigned)link->cb_dma);
	if (sizeof(*link) < link->size)
		_dump_extra_data(
			((char*)link)+sizeof(*link),
			link->size-sizeof(*link),
			dev,tindent);
}

void dma_link_dump(
	struct dma_link *link,
	struct device *dev,
	int tindent,
	void (*dma_cb_dump)(struct dma_link *, struct device *, int)
	)
{
	const char *indent = _tab_indent(tindent);

	if (!dma_cb_dump)
		dma_cb_dump = &dma_link_cb_dump_generic;

	dev_printk(KERN_INFO,dev, "%sdma_link:\t%pK\n", indent,
		link);
	dev_printk(KERN_INFO,dev, "%sdma_pool:\t%pK\n", indent,
		link->pool);
	dev_printk(KERN_INFO,dev, "%sdma_fragment:\t%pK - %s\n", indent,
		link->fragment,link->fragment->desc);
	if (link->desc) {
		if ((link->fragment) && (link->fragment->desc))
			dev_printk(KERN_INFO,dev,
				"%sdescription:\t%s.%s\n", indent,
				link->fragment->desc,link->desc);
		else
			dev_printk(KERN_INFO,dev,
				"%sdescription:\t%s\n", indent,
				link->desc);
	}
	dma_cb_dump(link,dev,tindent+1);
}
EXPORT_SYMBOL_GPL(dma_link_dump);

struct dma_fragment_transform *dma_fragment_transform_alloc(
	int (*transform)(struct dma_fragment_transform *,
			struct dma_fragment *, void *,gfp_t),
	void *src, void *dst, void *extra,
	size_t size,
	gfp_t gfpflags) {
	struct dma_fragment_transform *trans =
		kzalloc(max(size,sizeof(*trans)),gfpflags);
	if (trans)
		dma_fragment_transform_init(trans,size,transform,
					src,dst,extra);
	return trans;
}
EXPORT_SYMBOL_GPL(dma_fragment_transform_alloc);

void dma_fragment_transform_dump(
	struct dma_fragment_transform *trans,
	struct device *dev,
	int tindent)
{
	const char *indent = _tab_indent(tindent);

	dev_printk(KERN_INFO,dev, "%saddr:\t%p\n", indent,
		trans);
	dev_printk(KERN_INFO,dev, "%sfunc:\t%pf\n", indent,
		trans->transformer);
	dev_printk(KERN_INFO,dev, "%ssrc:\t%08lx\n", indent,
		(long unsigned)trans->src);
	dev_printk(KERN_INFO,dev, "%sdst:\t%08lx\n", indent,
		(long unsigned)trans->dst);
	dev_printk(KERN_INFO,dev, "%sextra:\t%08lx\n", 	indent,
		(long unsigned)trans->extra);

	if (sizeof(*trans) < trans->size)
		_dump_extra_data(
			((char*)trans)+sizeof(*trans),
			trans->size-sizeof(*trans),
			dev,tindent);
}
EXPORT_SYMBOL_GPL(dma_fragment_transform_dump);

struct dma_fragment *dma_fragment_alloc(
	struct device *device,
	gfp_t gfpflags,size_t size)
{
	struct dma_fragment *frag;
	size_t s = max( size, sizeof(*frag) );
	frag=kmalloc(s,gfpflags);
	if (! frag)
		return NULL;

	dma_fragment_init(frag,s);

	return frag;
}
EXPORT_SYMBOL_GPL(dma_fragment_alloc);

void dma_fragment_free(struct dma_fragment *frag)
{
	struct dma_link *link;
	struct dma_fragment_transform *transform;

	/* note: we do not remove from fragment cache,
	   as it would require a lock */
	if (!list_empty(&frag->cache_list)) {
		printk(KERN_ERR "dma_fragment_free: %pK "
			"is still a member of a dma_cache\n",
			frag
			);
		return;
	}

	/* remove all the dma_links belonging to us */
	while( !list_empty(&frag->dma_link_list)) {
		link = list_first_entry(
			&frag->dma_link_list,
			typeof(*link),
			dma_link_list);
		dma_link_free(link);
	}

	/* remove all the dma_fragment_transforms belonging to us */
	while( !list_empty(&frag->transform_list)) {
		transform = list_first_entry(
			&frag->transform_list,
			typeof(*transform),
			transform_list);
		dma_fragment_transform_free(transform);
	}

	kfree(frag);
}
EXPORT_SYMBOL_GPL(dma_fragment_free);

void dma_fragment_dump_generic(
	struct dma_fragment *fragment,
	struct device *dev,
	int tindent) {
	if (sizeof(*fragment) < fragment->size)
		_dump_extra_data(
			((char*)fragment)+sizeof(*fragment),
			fragment->size-sizeof(*fragment),
			dev,tindent);
}

void dma_fragment_dump(
	struct dma_fragment *fragment,
	struct device *dev,
	int tindent,
	int flags,
	void (*dma_cb_dump)(struct dma_link *,
			struct device *,int)
	) {
	struct dma_link *link;
	struct dma_fragment_transform *transform;
	int i;

	dev_printk(KERN_INFO,dev,
		"%sDMA-Fragment:\t%pK\n",
		_tab_indent(tindent),
		fragment);
	tindent++;
	dev_printk(KERN_INFO,dev,
		"%saddr:\t%pK\n",
		_tab_indent(tindent),
		fragment);
	dev_printk(KERN_INFO,dev,
		"%scache:\t%pK\n",
		_tab_indent(tindent),
		fragment->cache);
	if (fragment->desc)
		dev_printk(KERN_INFO,dev,
			"%sdescr:\t%s\n",
			_tab_indent(tindent),
			fragment->desc);

	/* dump extra data */
	if (sizeof(*fragment) < fragment->size)
		_dump_extra_data(
			((char*)fragment)+sizeof(*fragment),
			fragment->size-sizeof(*fragment),
			dev,tindent);

	/* dump the individual dma_links */
	dev_printk(KERN_INFO,dev,"%sDMA-Links:\n",
		_tab_indent(tindent));
	i=0;
	list_for_each_entry(link,
			&fragment->dma_link_list,
			dma_link_list) {
		dev_printk(KERN_INFO,dev,
			"%sDMA-Link %i:\n",
			_tab_indent(tindent+1),
			i++);
		dma_link_dump(link, dev, tindent+2, dma_cb_dump);
	}

	/* dump the individual dma_fragment_transforms */
	dev_printk(KERN_INFO,dev,"%sDMA-Transforms:\n",
		_tab_indent(tindent));
	i=0;
	list_for_each_entry(transform,
			&fragment->transform_list,
			transform_list) {
		dev_printk(KERN_INFO,dev,
			"%sDMA-Transform %i:\n",
			_tab_indent(tindent+1),
			i++);
		dma_fragment_transform_dump(transform, dev, tindent+2);
	}
}
EXPORT_SYMBOL_GPL(dma_fragment_dump);

#define SYSFS_PREFIX "dma_fragment_cache:"
static ssize_t dma_fragment_cache_sysfs_show(
	struct device *, struct device_attribute *, char *buf);
static ssize_t dma_fragment_cache_sysfs_store(
	struct device *, struct device_attribute *, const char *, size_t);

int dma_fragment_cache_initialize(
	struct dma_fragment_cache *cache,
	struct device *device,
	const char* name,
	struct dma_fragment *(*allocateFragment)(struct device *, gfp_t),
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
	i = sizeof(SYSFS_PREFIX)+strlen(name);
	fullname = kmalloc(i,GFP_KERNEL);
	if (!fullname)
		return -ENOMEM;
	strncpy(fullname,SYSFS_PREFIX,i);
	strncat(fullname,name,i);

	cache->device             = device;
	cache->dev_attr.attr.name = fullname;
	cache->dev_attr.attr.mode = S_IWUSR | S_IRUGO;
	cache->dev_attr.show      = dma_fragment_cache_sysfs_show;
	cache->dev_attr.store     = dma_fragment_cache_sysfs_store;
	cache->allocateFragment   = allocateFragment;

	/* and expose the statistics on sysfs */
	err = device_create_file(device, &cache->dev_attr);
	if (err) {
		/* duplicate names result in errors */
		cache->dev_attr.show = NULL;
		dev_printk(KERN_ERR,cache->device,
			"duplicate dma_fragment_cache name \"%s\"\n",
			cache->dev_attr.attr.name);
		return err;
	}

	/* now allocate new entries to fill the pool */
	return dma_fragment_cache_resize(cache,initial_size);
}
EXPORT_SYMBOL_GPL(dma_fragment_cache_initialize);

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

static ssize_t dma_fragment_cache_sysfs_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct dma_fragment_cache *cache =
		container_of(attr,typeof(*cache),dev_attr);
	s32 resize;
	int err;

	err = kstrtos32(buf,10,&resize);
	if (err)
		return -EPERM;

	err = dma_fragment_cache_resize(cache,resize);
	if (err<0)
		return err;

	return count;
}

int dma_fragment_cache_resize(struct dma_fragment_cache* cache,
	int resizeby)
{
	int i;
	struct dma_fragment *frag;
	unsigned long flags;

	if (resizeby == 0)
		return 0;

	if ( abs(resizeby) > 1024 )
		return -EPERM;

	/* add up to size */
	for (i = 0 ; i < resizeby ; i++) {
		if (! dma_fragment_cache_add(cache,GFP_KERNEL,1) ) {
			return -ENOMEM;
		}
	}
	/* remove up to size */
	for (i = 0 ; i > resizeby ; i--) {
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

struct dma_fragment *dma_fragment_cache_add(
	struct dma_fragment_cache *cache,
	gfp_t gfpflags,
	int toidle)
{
	unsigned long flags;

	struct dma_fragment *frag =
		cache->allocateFragment(cache->device,gfpflags);
	if (!frag)
		return NULL;

	frag->cache = cache;

	spin_lock_irqsave(&cache->lock, flags);

	/* gather statistics */
	cache->count_allocated ++;
	if (gfpflags == GFP_KERNEL)
		cache->count_allocated_kernel ++;
	/* add to corresponding list */
	if (flags && DMA_FRAGMENT_CACHE_TO_IDLE) {
		list_add(&frag->cache_list, &cache->idle);
		cache->count_idle++;
	} else {
		list_add(&frag->cache_list, &cache->active);
		cache->count_active++;
	}

	spin_unlock_irqrestore(&cache->lock, flags);

	/* and return it */
	return frag;
}
EXPORT_SYMBOL_GPL(dma_fragment_cache_add);

void dma_fragment_cache_release(struct dma_fragment_cache* cache)
{
	unsigned long flags;
	struct dma_fragment *frag;

	spin_lock_irqsave(&cache->lock, flags);

	while( !list_empty(&cache->idle)) {
		frag = list_first_entry(&cache->idle,
					struct dma_fragment,
					cache_list);
		list_del_init(&frag->cache_list);
		dma_fragment_free(frag);
	}
	cache->count_idle = 0;

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
