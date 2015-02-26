/*
 * helper for timing debugging of Broadcom BCM2835
 *
 * primarily used to analyze timings directly with a logic analyzer
 * on some unused GPIO pins - note that the IN/Out direction needs to get
 * set in Userland!!
 *
 * Copyright (C) 2015 Martin Sperl
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

/*
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
*/

static u32* gpio=0;
static inline void alloc_gpio(void) {
	gpio = ioremap(GPIO_BASE, SZ_16K);
}

/* a macro to create set_high/low for timing debug purposes */
#define _DEFINE_DEBUG_PIN_(number)					\
	static int debugpin##number = 0;				\
	module_param(debugpin##number,int,0);				\
	MODULE_PARM_DESC(debugpin##number,"the pin that we should toggle"); \
	static inline void __maybe_unused debug_set_low##number(void) {	\
		if (debugpin##number>0) {				\
			if (!gpio) alloc_gpio();			\
			gpio[0x28/4]=1<<debugpin##number;		\
		}							\
	}								\
	static inline void __maybe_unused debug_set_high##number(void) { \
		if (debugpin##number>0) {				\
			if (!gpio) alloc_gpio();			\
			gpio[0x1C/4]=1<<debugpin##number;		\
		}							\
	}
#define DEFINE_DEBUG_PIN(number) _DEFINE_DEBUG_PIN_(number)
