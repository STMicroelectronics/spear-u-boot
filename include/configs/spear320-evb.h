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

#define CONFIG_ENV_IS_IN_FLASH

#define CONFIG_MACH_SPEAR320EVB
#define CONFIG_MACH_TYPE			MACH_TYPE_SPEAR320

/* ARASAN SD MMC configuration */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_SPEAR_SDHCI
#endif

/* Designware Ethernet configurations */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_DESIGNWARE_ETH
	#define CONFIG_DW_SEARCH_PHY
	#define CONFIG_DW0_PHY			1
	#define CONFIG_PHY_RESET_DELAY		10000		/* in usec */
	#define CONFIG_DW_AUTONEG
#endif

/* MACB configurations */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_MACB
	#define CONFIG_MACB0_PHY		0x01
	#define CONFIG_MACB1_PHY		0x02
#endif

/* Designware I2C configurations */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_DW_I2C
	#define CONFIG_I2C_CHIPADDRESS		0x50
	#define CONFIG_SYS_I2C_SPEED		400000
	#define CONFIG_SYS_I2C_SLAVE		0x02
#endif

/* AMBA PL011 configurations */
#define CONFIG_PL011_SERIAL
#define CONFIG_CONS_INDEX			0

/* GPIO configurations */
#define CONFIG_SPEAR_GPIO

/* USB EHCI configurations */
#if !defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_USB_EHCI_SPEAR
#endif

/* Designware UDC configurations */
#if defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_DW_UDC
#endif

/* Flash configurations */
#define CONFIG_ST_SMI

/* SPL support */
#define CONFIG_SPL
#define CONFIG_SPEAR_DDR_2HCLK
#define CONFIG_DDR_MT47H64M16

/* Environment Variable configs */
#if defined(CONFIG_ENV_IS_IN_FLASH)
	/* Environment is in serial NOR flash */
	#define CONFIG_ENV_ADDR			0xF8060000
	#define CONFIG_ENV_SECT_SIZE		0x00010000
	#define CONFIG_SPEAR_ROOTFSBLK		"/dev/mtdblock5 "
	#define CONFIG_BOOTCOMMAND		"" \
		"bootm 0xf8080000 - 0xf8070000"
#endif

#define CONFIG_BOOTARGS				"console=ttyAMA0,115200 " \
						"root="CONFIG_SPEAR_ROOTFSBLK \
						"rootfstype=jffs2"

#define CONFIG_BOARD_EXTRA_ENV			""			\
	"loados=tftpboot 0x900000 $(rootpath)/spear3xx_uImage\0"	\
	"loaddtb=tftpboot 0x800000 $(rootpath)/spear320-evb.dtb\0"	\
	"mtdids=nor0=m25p64\0"						\
	"mtdparts=mtdparts=m25p64:64k(xloader)ro,320k(u-boot)ro,64k(environment)ro,64k(dtb)ro,3136k(kernel)ro,-(rootfs)\0"	\
	"partition=nor0,5\0"

#include <configs/spear320.h>
#endif /* __CONFIG_H */
