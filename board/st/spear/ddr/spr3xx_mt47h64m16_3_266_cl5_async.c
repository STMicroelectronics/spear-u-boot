/*
 * (C) Copyright 2000-2009
 * Vipin Kumar, ST Microelectronics, vipin.kumar@st.com
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/arch/hardware.h>

#if defined(CONFIG_ARCH_SPEAR3XX) && defined(CONFIG_SPEAR_DDR_PLL2) && \
	defined(CONFIG_DDR_MT47H64M16)

const u32 mpmc_conf_vals[CONFIG_SPEAR_MPMCREGS] = {
	0x00000001,
	0x00000000,
	0x01000000,
	0x00000101,
	0x00000101,
	0x01000000,
	0x00010001,
	0x00000100,
	0x00010001,
#if defined(CONFIG_SPEAR_DUAL_DDR)
	0x01020003,
	0x01000102,
	0x04000202,
#else
	0x00000001,
	0x02000001,
	0x04000201,
#endif
	0x03030104,
	0x03020202,
	0x01040000,
	0x00000001,
	0x00000000,
	0x03000405,
	0x02030002,
	0x04000204,
	0x0505053f,
	0x05050505,
	0x04040405,
	0x04040404,
	0x03030304,
	0x03030303,
	0x02020203,
	0x02020202,
	0x01010102,
	0x01010101,
	0x00000001,
	0x00000000,
	0x00000000,
	0x00000000,
	0x0a0b0a00,
	0x0000023f,
	0x00040800,
	0x0e000000,
	0x00000f02,
	0x00002020,
	0x7f000000,
	0x005f0000,
	0x22040b00,
	0x00640064,
	0x00640064,
	0x00000064,
	0x00000000,
	0x00200020,
	0x00200020,
	0x00200020,
	0x00200020,
	0x00200020,
	0x00000000,
	0x00000000,
	0x0000081b,
	0x00000000,
	0x48e100c8,
	0x00c80025,
	0x00000000,
	0x0000d056,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00230000,
	0x001f0023,
	0x00400000,
	0x00250099,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x01010001,
	0x01000000,
	0x00000001,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x006b0000,
	0x00000002
};
#endif
