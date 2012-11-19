/*
 * (C) Copyright 2012
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

#ifndef __CONFIG_SPEAR600_H
#define __CONFIG_SPEAR600_H

#define CONFIG_SOC_SPEAR600
#define CONFIG_ARCH_SPEAR6XX

/* PL011 configs */
#if defined(CONFIG_PL011_SERIAL)
#define CONFIG_SYS_SERIAL0			0xD0000000
#define CONFIG_PL011_CLOCK			(48 * 1000 * 1000)
#define CONFIG_PL01x_PORTS			{(void *)CONFIG_SYS_SERIAL0}
#endif

/* FSMC NAND configs */
#if defined(CONFIG_NAND_FSMC)
#define CONFIG_SYS_NAND_BASE			0xD2000000
#endif

/* ST SMI (Serial flash) configurations */
#if defined(CONFIG_ST_SMI)
	#define CONFIG_SYS_MAX_FLASH_BANKS	2
	#define CONFIG_SYS_FLASH_BASE		0xF8000000
	#define CONFIG_SYS_CS1_FLASH_BASE	0xF9000000
	#define CONFIG_SYS_FLASH_BANK_SIZE	0x01000000
	#define CONFIG_SYS_FLASH_ADDR_BASE	{CONFIG_SYS_FLASH_BASE, \
						CONFIG_SYS_CS1_FLASH_BASE}
	#define CONFIG_SYS_MAX_FLASH_SECT	128
#endif

/* Internal memory address for spear3xx */
#define CONFIG_SYS_INIT_SP_ADDR			(0xD2800000 + 0x2000 - \
						GENERATED_GBL_DATA_SIZE)

/* SPL configurations */
#if defined(CONFIG_SPL)
	#define CONFIG_SPL_NO_CPU_SUPPORT_CODE
	#define CONFIG_SPL_LIBCOMMON_SUPPORT
	#define CONFIG_SPL_LIBGENERIC_SUPPORT
	#define CONFIG_SPL_START_S_PATH		\
		"arch/arm/cpu/arm926ejs/spear"
	#define CONFIG_SPL_LDSCRIPT		\
		"arch/arm/cpu/arm926ejs/spear/u-boot-spl.lds"
#endif

#include <configs/spear.h>
#endif /* __CONFIG_SPEAR600_H */
