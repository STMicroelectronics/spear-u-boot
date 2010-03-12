/*
 * (C) Copyright 2009
 * Vipin Kumar, STMicroelectronics, <vipin.kumar@st.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _ASM_ARCH_HARDWARE_H
#define _ASM_ARCH_HARDWARE_H

#define CONFIG_SYS_USBD_BASE			(0xE1100000)
#define CONFIG_SYS_PLUG_BASE			(0xE1200000)
#define CONFIG_SYS_FIFO_BASE			(0xE1000800)
#define CONFIG_SYS_SMI_BASE			(0xFC000000)
#define CONFIG_SPEAR_TIMERBASE			(0xE0380000)
#define CONFIG_SPEAR_MISCBASE			(0xE0700000)
#define CONFIG_SYS_I2C_BASE			(0xE0280000)
#define CONFIG_SPEAR_ETHBASE			(0xE2000000)
#define CONFIG_SYS_FSMC_BASE			(0xB0000000)

#define CONFIG_SYS_NAND_CLE			(1 << 16)
#define CONFIG_SYS_NAND_ALE			(1 << 17)

#endif /* _ASM_ARCH_HARDWARE_H */
