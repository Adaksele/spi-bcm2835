/*
 * generic spi_dma_fragment support
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
#include <linux/spi/spi-dmafragment.h>

/**
 * spi_merged_dma_fragment_merge_fragment_cache - merge a
 * dma_fragment from a pool
 * into a spi_merged_dma_fragment
 * @fragment: the fragment cache from which to fetch the fragment
 * @merged: the merged fragment
 */
int spi_merged_dma_fragment_merge_fragment_cache(
	struct dma_fragment_cache *fragmentcache,
	struct spi_merged_dma_fragment *merged,
	gfp_t gfpflags
	)
{
	struct dma_fragment *frag;
	int err = 0;

	/* fetch the fragment from dma_fragment_cache */
	frag = dma_fragment_cache_fetch(fragmentcache, gfpflags);
	if (!frag)
		return -ENOMEM;

	/* run the transforms with the merged fragment */
	err = dma_fragment_add_subfragment(
		frag,
		&merged->dma_fragment,
		gfpflags
		);
	if (err)
		goto error;
	return err;

error:
	pr_err("ERROR: spi_merged_dma_fragment_merge_fragment_cache:"
		" %i\n", err);
	return err;
}
EXPORT_SYMBOL_GPL(spi_merged_dma_fragment_merge_fragment_cache);

static const char *_tab_indent_string = "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
static inline const char *_tab_indent(int indent)
{
	return &_tab_indent_string[16-min(16, indent)];
}

/**
 * spi_merged_dma_fragment_dump - dump the spi_merged_dma_fragment
 * into a spi_merged_dma_fragment
 * @fragment: the fragment cache from which to fetch the fragment
 * @dev: the devie to use during the dump
 * @tindent: the number of tab indents to add
 * @flags: the flags for dumping the fragment
 * @dma_link_dump: the function which to use to dump the dmablock
 */
void spi_merged_dma_fragment_dump(
	struct spi_merged_dma_fragment *fragment,
	struct device *dev,
	int tindent,
	void (*dma_cb_dump)(struct dma_link *,
			struct device *, int)
	)
{

	dma_fragment_dump(&fragment->dma_fragment, dev,
			tindent, dma_cb_dump);
	tindent++;
}
EXPORT_SYMBOL_GPL(spi_merged_dma_fragment_dump);

MODULE_DESCRIPTION("spi specific dma-fragment infrastructure");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL");
