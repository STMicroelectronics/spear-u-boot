/*
 * (C) Copyright 2006
 * Vipin Kumar, ST Micoelectronics, vipin.kumar@st.com.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>

/*
 * platform specific initializations are already done in Xloader
 * Initializations already done include
 * DDR, PLLs, IP's clock enable and reset release etc
 */
.globl lowlevel_init
lowlevel_init:
	/* By default, U-Boot switches CPU to low-vector */
	/* Revert this as we work in high vector even in U-Boot */
	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #0x00002000
	mcr	p15, 0, r0, c1, c0, 0
	mov	pc, lr
