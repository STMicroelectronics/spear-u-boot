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

#ifndef __CONFIG_H
#define __CONFIG_H

#if defined(CONFIG_usbtty)
	#define CONFIG_SPEAR_USBTTY
#endif

#if defined(CONFIG_nand)
	#define CONFIG_ENV_IS_IN_NAND
#else
	#define CONFIG_ENV_IS_IN_FLASH
#endif

#define CONFIG_MACH_SPEAR300EVB
#define CONFIG_MACH_TYPE			MACH_TYPE_SPEAR300

/* Designware Ethernet configurations */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_DESIGNWARE_ETH
	#define CONFIG_DW_SEARCH_PHY
	#define CONFIG_DW0_PHY				1
	#define CONFIG_PHY_RESET_DELAY			10000	/* in usec */
	#define CONFIG_DW_AUTONEG
#endif

/* Designware I2C configurations */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_DW_I2C
	#define CONFIG_I2C_CHIPADDRESS			0x50
	#define CONFIG_SYS_I2C_SPEED			400000
	#define CONFIG_SYS_I2C_SLAVE			0x02
#endif

/* GPIO configurations */
#define CONFIG_SPEAR_GPIO

/* USB EHCI configurations */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_USB_EHCI_SPEAR
#endif

/* AMBA PL011 configurations */
#define CONFIG_PL011_SERIAL
#define CONFIG_CONS_INDEX			0

/* Designware UDC configurations */
#if defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_DW_UDC
#endif

/* FSMC NAND configurations */
#define CONFIG_NAND_FSMC
#define CONFIG_SYS_FSMC_NAND_8BIT

/* ST SMI (Serial flash) configurations */
#define CONFIG_ST_SMI

/* SPL support */
#define CONFIG_SPL
#define CONFIG_SPEAR_DDR_2HCLK
#define CONFIG_DDR_MT47H64M16

#if defined(CONFIG_ENV_IS_IN_FLASH)
	/* Environment is in serial NOR flash */
	#define CONFIG_ENV_ADDR			0xF8060000
	#define CONFIG_ENV_SECT_SIZE		0x00010000
	#define CONFIG_SPEAR_ROOTFSBLK		"/dev/mtdblock5 "
	#define CONFIG_BOOTCOMMAND		"" \
		"bootm 0xf8080000 - 0xf8070000"

#elif defined(CONFIG_ENV_IS_IN_NAND)
	/* Environment is in NAND */
	#define CONFIG_ENV_OFFSET		0x00070000
	#define CONFIG_SPEAR_ROOTFSBLK		"/dev/mtdblock11 "

	#define CONFIG_BOOTCOMMAND		"" \
		"nand read.jffs2 0x800000 0x78000 0x008000; " \
		"nand read.jffs2 0x900000 0x80000 0x4C0000; " \
		"bootm 0x900000 - 0x800000"
#endif

#define CONFIG_BOOTARGS				"console=ttyAMA0,115200 " \
						"root="CONFIG_SPEAR_ROOTFSBLK \
						"rootfstype=jffs2"

#define CONFIG_BOARD_EXTRA_ENV			""			\
	"loados=tftpboot 0x900000 $(rootpath)/spear3xx_uImage\0"	\
	"loaddtb=tftpboot 0x800000 $(rootpath)/spear300-evb.dtb\0"	\
	"mtdids=nor0=m25p64,nand0=nand512w3acza6\0"			\
	"mtdparts=mtdparts=m25p64:64k(xloader)ro,320k(u-boot)ro,64k(environment)ro,64k(dtb)ro,3136k(kernel)ro,-(rootfs);"	\
		"nand512w3acza6:64k(xloader)ro,384k(u-boot)ro,32k(environment)ro,32k(dtb)ro,4M(kernel)ro,-(rootfs)\0"		\
	"partition=nor0,5\0"

#include <configs/spear300.h>
#endif /* __CONFIG_H */
