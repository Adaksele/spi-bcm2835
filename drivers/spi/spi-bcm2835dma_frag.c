/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2014 Martin Sperl
 *
 * This driver is inspired by:
 * spi-ath79.c, Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c, Copyright (C) 2006 Atmel Corporation
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

#include <linux/dma/bcm2835-dma.h>
#include <linux/dma-fragment.h>

struct dma_fragment *bcm2835_dmafragment_create_setup_spi_plus_transfer(
	struct dma_pool *pool,gfp_t gfpflags) {

	return dma_fragment_alloc(gfpflags);
}
	
struct dma_fragment *bcm2835_dmafragment_create_transfer(
	struct dma_pool *pool,gfp_t gfpflags) {

	return dma_fragment_alloc(gfpflags);
}

