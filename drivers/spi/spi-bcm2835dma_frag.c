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

#include "spi-bcm2835dma.h"

#define ALLOCATE_CBFIELD_IN_STRUCT(frag,field)			\
	if (! ( frag->field = dma_link_alloc(pool,gfpflags) ) )	\
		goto error;					\
	if ( dma_fragment_add(					\
			(struct dma_fragment *)frag,		\
			(struct dma_link *)frag->field		\
			) )					\
		goto error;


#define FREE_CBFIELD_IN_STRUCT(frag,field)	     	\
	if (frag->field)		       		\
		dma_link_free(frag->field);

#define LINK_TO_BCM2835_DMA_CB(dmalink)			\
	((struct bcm2835_dma_cb *)dmalink->dmablock)

#define LINK_FIELD_DMAADDR(dmalink,field)				\
	( dmalink->dmablock_dma + offsetof(struct bcm2835_dma_cb,field) )


struct dmafragment_create_setup_spi_plus_transfer {
	struct dma_fragment fragment;
	/* the individual objects */
	struct dma_link     *cs_select;
	struct dma_link     *reset_spi_and_config_speed;
	struct dma_link     *config_length;
	struct dma_link     *config_spi;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
	struct dma_link     *schedule_transfer_rx;
	struct dma_link     *schedule_transfer_tx;
};

struct dmafragment_create_transfer {
	struct dma_fragment fragment;
	/* the individual objects */
	struct dma_link     *schedule_transfer_rx;
	struct dma_link     *schedule_transfer_tx;
};

struct dmafragment_cs_deselect {
	struct dma_fragment fragment;
	struct dma_link     *delay_pre;
	struct dma_link     *cs_deselect;
	struct dma_link     *delay_post;
};

struct dmafragment_delay {
	struct dma_fragment fragment;
	struct dma_link     *delay;
};

struct dmafragment_trigger_irq {
	struct dma_fragment fragment;
	struct dma_link     *set_tx_dma_next;
	struct dma_link     *start_tx_dma;
	struct dma_link     *message_finished;
};

struct dma_fragment *bcm2835_dmafragment_create_setup_spi_plus_transfer(
	struct spi_master * master,gfp_t gfpflags)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dmafragment_create_setup_spi_plus_transfer *frag 
		= (typeof(frag))dma_fragment_alloc(gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */

	/* start with the CSSelect */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,cs_select);
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->ti =
		BCM2835_DMA_TI_WAIT_RESP;
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->length = 
		4;
	/* now reset SPI and configure the speed */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,reset_spi_and_config_speed);
	LINK_TO_BCM2835_DMA_CB(frag->cs_select)->next =
		frag->reset_spi_and_config_speed->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_TDMODE
		| BCM2835_DMA_TI_S_INC
		| BCM2835_DMA_TI_D_INC;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->length =
		8;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->stride_src = 
	        4;
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->stride_dst =
		-8;
	/* the clock speed as a divider in pad[0]
	 * the RESET values in pad[1] - need to set:
	 * * BCM2835_SPI_CS_CSPOL correctly
	 * * BCM2835_SPI_CS_CLEAR_RX
	 * * BCM2835_SPI_CS_CLEAR_TX
	 */

	/* set the DMA-transfer length */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,config_length);
	LINK_TO_BCM2835_DMA_CB(frag->reset_spi_and_config_speed)->next =
		frag->config_length->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_DLEN;
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->length =
		4;
	/* the clock speed as a divider in pad[0] */

	/* reenable the spi config */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,config_spi);
	LINK_TO_BCM2835_DMA_CB(frag->config_length)->next =
		frag->config_spi->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->src =
		LINK_FIELD_DMAADDR(frag->cs_select,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_DLEN;
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->length =
		4;
	/* the correct flags for TI in pad[0] - need to have:
	 * BCM2835_SPI_CS_CPHA correctly
	 * BCM2835_SPI_CS_CPOL correctly
	 * BCM2835_SPI_CS_DMAEN
	 * BCM2835_SPI_CS_TA
	 * BCM2835_SPI_CS_CS_01
	 * BCM2835_SPI_CS_CS_10
	 */

	/* set the tx-DMA next pointer */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,set_tx_dma_next);
	LINK_TO_BCM2835_DMA_CB(frag->config_spi)->next =
		frag->set_tx_dma_next->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->src =
		LINK_FIELD_DMAADDR(frag->set_tx_dma_next,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->length =
		4;
	/* need to set:
	 * * dst = dma_tx.bus_addr+BCM2835_DMA_CB_ADDR
	 */ 
	
	/* start tx-DMA */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,start_tx_dma);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->next =
		frag->start_tx_dma->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->src =
		LINK_FIELD_DMAADDR(frag->start_tx_dma,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->length =
		4;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->pad[0] =
		BCM2835_DMA_CS_ACTIVE;
	/* need to set:
	 * * dst = dma_tx.bus_addr+BCM2835_DMA_CB_CS
	 */ 

	/* the initial transfer for RX */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,schedule_transfer_rx);
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->next =
		frag->schedule_transfer_rx->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->src =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */
	/* and the initial transfer for TX */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,schedule_transfer_tx);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->pad[0] =
		frag->schedule_transfer_tx->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */
	
	return &frag->fragment;

error:
	FREE_CBFIELD_IN_STRUCT(frag,cs_select);
	FREE_CBFIELD_IN_STRUCT(frag,reset_spi_and_config_speed);
	FREE_CBFIELD_IN_STRUCT(frag,config_length);
	FREE_CBFIELD_IN_STRUCT(frag,config_spi);
	FREE_CBFIELD_IN_STRUCT(frag,set_tx_dma_next);
	FREE_CBFIELD_IN_STRUCT(frag,start_tx_dma);
	FREE_CBFIELD_IN_STRUCT(frag,schedule_transfer_rx);
	FREE_CBFIELD_IN_STRUCT(frag,schedule_transfer_tx);

	dma_fragment_free(&frag->fragment);

	return NULL;
}
	
struct dma_fragment *bcm2835_dmafragment_create_transfer(
	struct spi_master * master,gfp_t gfpflags)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dmafragment_create_setup_spi_plus_transfer *frag 
		= (typeof(frag))dma_fragment_alloc(gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */

	/* the initial transfer for RX */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,schedule_transfer_rx);
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_rx)->src =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */
	/* and the initial transfer for TX */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,schedule_transfer_tx);
	LINK_TO_BCM2835_DMA_CB(frag->schedule_transfer_tx)->dst =
		BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO;
	/* need to set:
	 * * ti to correct flags
	 * * length to correct length
	 * * dst to correct bus address
	 */

	return &frag->fragment;

error:
	FREE_CBFIELD_IN_STRUCT(frag,schedule_transfer_rx);
	FREE_CBFIELD_IN_STRUCT(frag,schedule_transfer_tx);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

struct dma_fragment *bcm2835_dmafragment_create_cs_deselect(
	struct spi_master * master,gfp_t gfpflags)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dmafragment_cs_deselect *frag 
		= (typeof(frag))dma_fragment_alloc(gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */

	/* the pre-half-clock delay */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,delay_pre);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_WAITS(0x1f)
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| BCM2835_DMA_TI_S_IGNORE
		| BCM2835_DMA_TI_D_IGNORE;
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->src =
		LINK_FIELD_DMAADDR(frag->delay_pre,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->dst =
		LINK_FIELD_DMAADDR(frag->delay_pre,pad[1]);
	/* need to set:
	 * * length to correct length to get to the delay we need based on the clock
	 */
	/* start with the CSSelect */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,cs_deselect);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->next =
		frag->cs_deselect->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->ti =
		BCM2835_DMA_TI_WAIT_RESP;
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->src =
		LINK_FIELD_DMAADDR(frag->cs_deselect,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->cs_deselect)->length = 
		4;
	/* need to set:
	 * * pad0 to correct bitmask
	 * * dst to correct register to pull GPIO pin up
	 */

	/* the post-half-clock delay */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,delay_post);
	LINK_TO_BCM2835_DMA_CB(frag->delay_post)->next =
		frag->cs_deselect->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->delay_post)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_WAITS(0x1f)
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| BCM2835_DMA_TI_S_IGNORE
		| BCM2835_DMA_TI_D_IGNORE;
	LINK_TO_BCM2835_DMA_CB(frag->delay_post)->src =
		LINK_FIELD_DMAADDR(frag->delay_post,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->delay_pre)->dst =
		LINK_FIELD_DMAADDR(frag->delay_post,pad[1]);
	/* need to set:
	 * * length to correct length to get to the delay we need based on the clock
	 */

	return &frag->fragment;

error:
	FREE_CBFIELD_IN_STRUCT(frag,delay_pre);
	FREE_CBFIELD_IN_STRUCT(frag,cs_deselect);
	FREE_CBFIELD_IN_STRUCT(frag,delay_post);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

struct dma_fragment *bcm2835_dmafragment_create_delay(
	struct spi_master * master,gfp_t gfpflags)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dmafragment_delay *frag 
		= (typeof(frag))dma_fragment_alloc(gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */

	/* the pre-half-clock delay */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,delay);
	LINK_TO_BCM2835_DMA_CB(frag->delay)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_WAITS(0x1f)
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| BCM2835_DMA_TI_S_IGNORE
		| BCM2835_DMA_TI_D_IGNORE;
	LINK_TO_BCM2835_DMA_CB(frag->delay)->src =
		LINK_FIELD_DMAADDR(frag->delay,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->delay)->dst =
		LINK_FIELD_DMAADDR(frag->delay,pad[1]);
	/* need to set:
	 * * length to correct length to get to the delay we need based on the clock
	 */

	return &frag->fragment;

error:
	FREE_CBFIELD_IN_STRUCT(frag,delay);

	dma_fragment_free(&frag->fragment);

	return NULL;
}

struct dma_fragment *bcm2835_dmafragment_create_trigger_irq(
	struct spi_master * master,gfp_t gfpflags)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct dma_pool *pool = bs->pool;
	struct dmafragment_trigger_irq *frag 
		= (typeof(frag))dma_fragment_alloc(gfpflags,sizeof(*frag));
	if (! frag)
		return NULL;
	/* now allocate the blocks we need */
	/* set the tx-DMA next pointer */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,set_tx_dma_next);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->src =
		LINK_FIELD_DMAADDR(frag->set_tx_dma_next,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->length =
		4;
	/* need to set:
	 * * dst = dma_tx.bus_addr+BCM2835_DMA_CB_ADDR
	 */ 
	
	/* start tx-DMA */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,start_tx_dma);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->next =
		frag->start_tx_dma->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->ti =
		BCM2835_DMA_TI_WAIT_RESP ;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->src =
		LINK_FIELD_DMAADDR(frag->start_tx_dma,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->length =
		4;
	LINK_TO_BCM2835_DMA_CB(frag->start_tx_dma)->pad[0] =
		BCM2835_DMA_CS_ACTIVE;
	/* need to set:
	 * * dst = dma_tx.bus_addr+BCM2835_DMA_CB_CS
	 */ 

	/* message_finished */
	ALLOCATE_CBFIELD_IN_STRUCT(frag,message_finished);
	LINK_TO_BCM2835_DMA_CB(frag->set_tx_dma_next)->pad[0] =
		frag->message_finished->dmablock_dma;
	LINK_TO_BCM2835_DMA_CB(frag->message_finished)->ti =
		BCM2835_DMA_TI_WAIT_RESP
		| BCM2835_DMA_TI_S_INC
		| BCM2835_DMA_TI_D_INC
		;
	LINK_TO_BCM2835_DMA_CB(frag->message_finished)->src =
		LINK_FIELD_DMAADDR(frag->message_finished,pad[0]);
	LINK_TO_BCM2835_DMA_CB(frag->message_finished)->length =
		8;
	/* need to set:
	 * * dst = The address which gets the latest tranfer
	 * * pad[0] = the fragment that was finished
	 * * pad[1] = possibly spi_message pointer
	 */ 
	return &frag->fragment;

error:
	FREE_CBFIELD_IN_STRUCT(frag,set_tx_dma_next);
	FREE_CBFIELD_IN_STRUCT(frag,start_tx_dma);
	FREE_CBFIELD_IN_STRUCT(frag,message_finished);

	dma_fragment_free(&frag->fragment);

	return NULL;
}
