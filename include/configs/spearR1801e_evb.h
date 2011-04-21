/*
 * (C) Copyright 2010
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

#if defined(CONFIG_MK_spearR1801e)
#define CONFIG_SPEAR13XX			1
#define CONFIG_SPEAR1310			1
#define CONFIG_MACH_SPEARR1801E	1
#endif


#if defined(CONFIG_MK_usbtty)
#define CONFIG_SPEAR_USBTTY			1
#endif

#if !defined(CONFIG_SPEAR_USBTTY)
/* Solve issue #101435 - UHC blocks the BUSMATRIX */
#define CONFIG_SPEAR1300_ISSUE_101435		1
#endif

#if !defined(CONFIG_SPEAR_USBTTY)
/* MMC configuration */

#if 0
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_SPEAR_MMC
#define CONFIG_CMD_MMC
#endif

#endif

#if !defined(CONFIG_SPEAR_USBTTY)
/* Ethernet configuration */
#define CONFIG_MII
#define CONFIG_DESIGNWARE_ETH
#define CONFIG_NET_MULTI
#define CONFIG_DW_ALTDESCRIPTOR			1
#define CONFIG_DW_SEARCH_PHY			1			/* SOM has only one phy */
#define CONFIG_DW0_PHY				0x0
#define CONFIG_PHY_RESET_DELAY			(10000)		/* in usec */
/*#define CONFIG_DW_AUTONEG			1*/
#endif

/* USBD driver configuration */
#if defined(CONFIG_SPEAR_USBTTY)
#define CONFIG_DW_UDC
#define CONFIG_USB_DEVICE
#define CONFIG_USBD_HS
#define CONFIG_USB_TTY

#define CONFIG_USBD_PRODUCT_NAME		"SPEAr SoC"
#define CONFIG_USBD_MANUFACTURER		"ST Microelectronics"

#endif

#define CONFIG_EXTRA_ENV_USBTTY			"usbtty=cdc_acm\0"

/* Timer, HZ specific defines */
#define CONFIG_SYS_HZ				(1000)

#define CONFIG_SYS_NO_FLASH		1
/* NOT NEEDED??? */
#if 0
/* Flash configuration */
#define CONFIG_ST_SMI				1
#define CONFIG_SYS_MAX_FLASH_BANKS		2
#define CONFIG_SYS_FLASH_BASE			(0xE6000000)
#define CONFIG_SYS_CS1_FLASH_BASE		(0xE7000000)
#define CONFIG_SYS_FLASH_BANK_SIZE		(0x01000000)
#define CONFIG_SYS_FLASH_ADDR_BASE		{CONFIG_SYS_FLASH_BASE, \
						CONFIG_SYS_CS1_FLASH_BASE}
#define CONFIG_SYS_MAX_FLASH_SECT		128

#define CONFIG_SYS_FLASH_EMPTY_INFO		1
#define CONFIG_SYS_FLASH_ERASE_TOUT		(3 * CONFIG_SYS_HZ)
#define CONFIG_SYS_FLASH_WRITE_TOUT		(3 * CONFIG_SYS_HZ)
#endif

/*
 * Serial Configuration (PL011)
 */
#define CONFIG_PL011_SERIAL
#define CONFIG_SYS_SERIAL0			0xE0000000
#define CONFIG_PL011_CLOCK			(48 * 1000 * 1000)
#define CONFIG_CONS_INDEX			0
#define CONFIG_BAUDRATE				115200
#define CONFIG_SYS_BAUDRATE_TABLE		{ 9600, 19200, 38400, \
						57600, 115200 }

#define CONFIG_SYS_LOADS_BAUD_CHANGE
#define CONFIG_PL01x_PORTS			{(void *)CONFIG_SYS_SERIAL0}

/* NOT NEEDED??? */
#if 0
/*
 * USB/EHCI
 */
#if !defined(CONFIG_SPEAR_USBTTY)
#define CONFIG_USB_EHCI				/* Enable EHCI USB support */
#define CONFIG_USB_EHCI_SPEAR
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_USB				/* Enable USB Commands */
#endif
#endif

/*
 * NAND FLASH Configuration
 */
#define CONFIG_NAND_FSMC					1
#define CONFIG_SYS_FSMC_NAND_16BIT		1
#define CONFIG_SYS_MAX_NAND_DEVICE		1
#define CONFIG_MTD_NAND_VERIFY_WRITE		1
#define CONFIG_SYS_NAND_BASE			(0xA0000000)
#define CONFIG_CMD_NAND

/*
 * Command support defines
 */
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_RUN

#if !defined(CONFIG_SPEAR_USBTTY)
#define CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP

/* NOT NEEDED??? */
#if 0
#if defined(CONFIG_USB_STORAGE) || defined(CONFIG_MMC)
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define CONFIG_ISO_PARTITION
#endif

#endif

#endif

/* This must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <config_cmd_default.h>

#if defined(CONFIG_SPEAR_USBTTY)
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#endif

/*
 * Default Environment Varible definitions
 */
#if defined(CONFIG_SPEAR_USBTTY)
/* This disbales autobooting and stops at uboot prompt */
#define CONFIG_BOOTDELAY			-1
#else
#define CONFIG_BOOTDELAY			1
#endif

#define CONFIG_ENV_OVERWRITE

/*
 * Environment is in NAND
 */
#define CONFIG_ENV_IS_IN_NAND			1

#define CONFIG_ENV_OFFSET			0x60000
#define CONFIG_ENV_RANGE			0x10000
#define CONFIG_FSMTDBLK				"/dev/mtdblock7 "

#define CONFIG_BOOTCOMMAND			"nand read.jffs2 0x1600000 " \
						"0x80000 0x4C0000; " \
						"bootm 0x1600000"

#define CONFIG_BOOTARGS_NFS			"root=/dev/nfs ip=dhcp " \
						"console=ttyAMA0,115200 " \
						"init=/bin/sh"
#define CONFIG_BOOTARGS				"console=ttyAMA0,115200 " \
						"root="CONFIG_FSMTDBLK \
						"rootfstype=jffs2"

#define CONFIG_ENV_SIZE				0x02000

/* Miscellaneous configurable options */
#define CONFIG_ARCH_CPU_INIT			1
#define CONFIG_DISPLAY_CPUINFO			1

#define CONFIG_BOOT_PARAMS_ADDR			0x00000100
#define CONFIG_CMDLINE_TAG			1
#define CONFIG_SETUP_MEMORY_TAGS		1
#define CONFIG_MISC_INIT_R			1
#define CONFIG_ZERO_BOOTDELAY_CHECK		1
#define CONFIG_AUTOBOOT_KEYED			1
#define CONFIG_AUTOBOOT_STOP_STR		" "
#define CONFIG_AUTOBOOT_PROMPT			\
		"Hit SPACE in %d seconds to stop autoboot.\n", bootdelay

#define CONFIG_SYS_MEMTEST_START		0x00800000
#define CONFIG_SYS_MEMTEST_END			0x04000000
#define CONFIG_SYS_MALLOC_LEN			(1024*1024)
#define CONFIG_SYS_GBL_DATA_SIZE		128
#define CONFIG_IDENT_STRING			"-SPEAr"
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT			"u-boot> "
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_CBSIZE			256
#define CONFIG_SYS_PBSIZE			(CONFIG_SYS_CBSIZE + \
						sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS			16
#define CONFIG_SYS_BARGSIZE			CONFIG_SYS_CBSIZE
#define CONFIG_SYS_LOAD_ADDR			0x00800000
#define CONFIG_SYS_CONSOLE_INFO_QUIET		1
#define CONFIG_SYS_64BIT_VSPRINTF		1

#define CONFIG_EXTRA_ENV_SETTINGS		CONFIG_EXTRA_ENV_USBTTY


/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS			1
#define PHYS_SDRAM_1				0x00000000
#define PHYS_SDRAM_1_MAXSIZE			0x40000000

#endif
