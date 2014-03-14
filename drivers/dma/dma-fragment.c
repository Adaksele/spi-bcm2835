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
	char buffer[50];
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
			"%sdata %pf - %02x: %s\n",
			_tab_indent(tindent),
			data,offset,buffer
			);
		data   += bytes_per_line;
		size   -= bytes_per_line;
		offset += bytes_per_line;
	}
}

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

void dma_fragment_transform_dump(
	struct dma_fragment_transform *trans,
	struct device *dev,
	int tindent)
{
	const char *indent = _tab_indent(tindent);

	dev_printk(KERN_INFO,dev, "%saddr:\t%p\n", indent,
		trans);
	dev_printk(KERN_INFO,dev, "%sfunc:\t%pf\n", indent,
		trans->function);
	dev_printk(KERN_INFO,dev, "%sfrag:\t%pk\n", indent,
		trans->fragment);
	dev_printk(KERN_INFO,dev, "%sdata:\t%pk\n", indent,
		trans->data);

	if (sizeof(*trans) < trans->size)
		_dump_extra_data(
			((char*)trans)+sizeof(*trans),
			trans->size-sizeof(*trans),
			dev,tindent);
}
EXPORT_SYMBOL_GPL(dma_fragment_transform_dump);

void dma_fragment_release_subfragments(struct dma_fragment *frag)
{
	struct dma_fragment *sub;
	while( !list_empty(&frag->sub_fragment_head)) {
		sub = list_first_entry(
			&frag->sub_fragment_head,
			typeof(*sub),
			sub_fragment_list);
		list_del_init(&sub->sub_fragment_list);
		dma_fragment_release(sub);
	}
}
EXPORT_SYMBOL_GPL(dma_fragment_release_subfragments);

void dma_fragment_free_link_transforms(struct dma_fragment *frag)
{
	struct dma_fragment_transform *transform;
	while( !list_empty(&frag->link_transform_list)) {
		transform = list_first_entry(
			&frag->link_transform_list,
			typeof(*transform),
			transform_list);
		dma_fragment_transform_free(transform);
	}
}
EXPORT_SYMBOL_GPL(dma_fragment_free_link_transforms);

void dma_fragment_release(struct dma_fragment *frag)
{
	struct dma_link *link;

	/* release sub-fragments */
	dma_fragment_release_subfragments(frag);
	/* release other stuff */
	/* return to fragment cache if we are member of one */
	if (frag->cache) {
		if (frag->release_fragment)
			frag->release_fragment(frag,1);
		dma_fragment_cache_return(frag);
	} else {
		/* remove all the dma_links belonging to us */
		while( !list_empty(&frag->dma_link_list)) {
			link = list_first_entry(
				&frag->dma_link_list,
				typeof(*link),
				dma_link_list);
			list_del_init(&link->dma_link_list);
			dma_link_free(link);
		}
		/* remove all the dma_fragment_transforms belonging to us */
		dma_fragment_free_link_transforms(frag);

		/* release if allocated ourselves*/
		if (frag->release_fragment)
			frag->release_fragment(frag,0);
		else {
			if (!frag->embedded)
				kfree(frag);
		}
	}
}
EXPORT_SYMBOL_GPL(dma_fragment_release);

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
	struct dma_fragment *sfrag;
	struct dma_fragment_transform *transform;
	int i;
	const char *indent=_tab_indent(tindent+1);

	dev_printk(KERN_INFO,dev,
		"%sDMA-Fragment:\t%pK\n",
		_tab_indent(tindent),
		fragment);
	tindent++;
	dev_printk(KERN_INFO,dev,
		"%saddr:\t%pK\n",
		indent,
		fragment);
	dev_printk(KERN_INFO,dev,
		"%scache:\t%pK\n",
		indent,
		fragment->cache);
	if (fragment->desc)
		dev_printk(KERN_INFO,dev,
			"%sdescr:\t%s\n",
			indent,
			fragment->desc);
	dev_printk(KERN_INFO,dev,
		"%slink_h:\t%pK\n",
		indent,
		fragment->link_head);
	dev_printk(KERN_INFO,dev,
		"%slink_t:\t%pK\n",
		indent,
		fragment->link_tail);
	dev_printk(KERN_INFO,dev,
		"%srelease:\t%pK\n",
		indent,
		fragment->release_fragment);
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

	/* dump the sub fragments */
	if (!list_empty(&fragment->sub_fragment_head)) {
		dev_printk(KERN_INFO,dev,
			"%sSub-DMA-Fragments:\n",
			_tab_indent(tindent)
			);
		list_for_each_entry(sfrag,
				&fragment->sub_fragment_head,
				sub_fragment_list) {
			dma_fragment_dump(sfrag, dev, tindent+1,
					flags,dma_cb_dump);
		}
	}

	/* dump the individual dma_fragment_transforms */
	if (!list_empty(&fragment->link_transform_list)) {
		dev_printk(KERN_INFO,dev,"%sDMA-Transforms:\n",
			_tab_indent(tindent));
		i=0;
		list_for_each_entry(transform,
				&fragment->link_transform_list,
				transform_list) {
			dev_printk(KERN_INFO,dev,
				"%sDMA-Transform %i:\n",
				_tab_indent(tindent+1),
				i++);
			dma_fragment_transform_dump(transform, dev, tindent+2);
		}
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
		if (! dma_fragment_cache_add(cache,GFP_KERNEL,
						DMA_FRAGMENT_CACHE_TO_IDLE)
			) {
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
		frag->cache = NULL;
		dma_fragment_release(frag);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(dma_fragment_cache_resize);

struct dma_fragment *dma_fragment_cache_add(
	struct dma_fragment_cache *cache,
	gfp_t gfpflags,
	int dest)
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
	/* add to idle list */
	if (dest & DMA_FRAGMENT_CACHE_TO_IDLE) {
		list_add(&frag->cache_list, &cache->idle);
		cache->count_idle++;
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
		frag->cache = NULL;
		dma_fragment_release(frag);
	}
	cache->count_idle = 0;

	if (cache->count_active)
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
