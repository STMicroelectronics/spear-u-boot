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

#ifndef __CONFIG_SPEAR_H
#define __CONFIG_SPEAR_H

#include <config_cmd_default.h>

#define CONFIG_PLAT_SPEAR

#define CONFIG_SYS_TEXT_BASE			0x00700000
#define CONFIG_BOOT_PARAMS_P			0x00000100

/* Timer, HZ specific defines */
#define CONFIG_SYS_HZ				1000


#if defined(CONFIG_NAND_FSMC) || \
	defined(CONFIG_ST_SMI) || \
	defined(CONFIG_FLASH_CFI_DRIVER)

	#define CONFIG_MTD_DEVICE
	#define CONFIG_MTD_PARTITIONS

	#define CONFIG_CMD_JFFS2
	#define CONFIG_CMD_MTDPARTS
	#ifdef CONFIG_NAND_FSMC
		#define CONFIG_JFFS2_NAND
	#endif
#endif

/* Generic configuration for Designware Ethernet */
#if defined(CONFIG_DESIGNWARE_ETH) || defined(CONFIG_MACB)
	#define CONFIG_MII
	#define CONFIG_NET_MULTI

	#define CONFIG_CMD_NET
	#define CONFIG_CMD_MII
	#define CONFIG_CMD_PING
	#define CONFIG_CMD_DHCP
#else
	#undef CONFIG_CMD_NET
	#undef CONFIG_CMD_NFS
#endif

/* Generic configuration for USBD driver */
#if defined(CONFIG_DW_UDC) || defined(CONFIG_DW_OTG)
	#define CONFIG_USB_DEVICE
	#define CONFIG_USBD_HS
	#define CONFIG_USB_TTY

#ifndef CONFIG_USBD_PRODUCT_NAME
	#define CONFIG_USBD_PRODUCT_NAME	"SPEAr SoC"
#endif
#ifndef CONFIG_USBD_MANUFACTURER
	#define CONFIG_USBD_MANUFACTURER	"ST Microelectronics"
#endif

#endif /* CONFIG_DW_UDC */

#if !defined(CONFIG_BOARD_EXTRA_ENV)
	#define CONFIG_BOARD_EXTRA_ENV		""
#endif
#if !defined(CONFIG_SOC_EXTRA_ENV)
	#define CONFIG_SOC_EXTRA_ENV		""
#endif

#define CONFIG_EXTRA_ENV_SETTINGS	CONFIG_BOARD_EXTRA_ENV \
					CONFIG_SOC_EXTRA_ENV \
	"usbtty=cdc_acm\0unlock=yes\0" \
	"autoload=n\0" \
	"nfsargs=setenv bootargs root=/dev/nfs rw "			\
	"nfsroot=$(serverip):$(rootpath) "				\
	"ip=$(ipaddr):$(serverip):$(gatewayip):"			\
	"$(netmask):$(hostname):$(netdev):off "				\
	"console=ttyAMA0,115200 $(othbootargs)\0"

/* Generic configuration for I2C driver */
#if defined(CONFIG_DW_I2C)
	#define CONFIG_HARD_I2C
	#define CONFIG_CMD_I2C
#endif

/* Generic configuration for GPIO driver */
#if defined(CONFIG_SPEAR_GPIO)
	#define CONFIG_CMD_GPIO
#endif

/* Generic configuration for USB EHCI driver */
#if defined(CONFIG_USB_EHCI_SPEAR)
	#define CONFIG_USB_EHCI
	#define CONFIG_USB_STORAGE
	#define CONFIG_CMD_USB
#endif

/* Enable FAT and Partition types */
#if defined(CONFIG_USB_STORAGE)
	#define CONFIG_CMD_FAT
	#define CONFIG_DOS_PARTITION
	#define CONFIG_ISO_PARTITION
#endif

/* Generic configuration for ST SMI driver */
#if defined(CONFIG_ST_SMI)
	#define CONFIG_SYS_FLASH_ERASE_TOUT	(3 * CONFIG_SYS_HZ)
	#define CONFIG_SYS_FLASH_WRITE_TOUT	(3 * CONFIG_SYS_HZ)
#endif

/* Generic configuration for CFI driver */
#if defined(CONFIG_FLASH_CFI_DRIVER)
	#define CONFIG_SYS_FLASH_CFI
	#define CONFIG_SYS_FLASH_QUIET_TEST
	#define CONFIG_SYS_FLASH_PROTECTION
#endif

/* Generic configuration for AMBA PL011 driver */
#if defined(CONFIG_PL011_SERIAL)
	#define CONFIG_BAUDRATE			115200
	#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, \
						57600, 115200 }
#endif

/* Generic configuration for AMBA PL022 driver */
#if defined(CONFIG_PL022_SPI)
	#define CONFIG_CMD_SPI
	#if defined(CONFIG_SPI_FLASH)
		#define CONFIG_CMD_SF
		#define CONFIG_SPI_FLASH_STMICRO
	#endif
#endif

/* Generic configuration for Arasan SD/MMC driver */
#if defined(CONFIG_SPEAR_SDHCI)
	#define CONFIG_MMC
	#define CONFIG_SDHCI
	#define CONFIG_GENERIC_MMC
	#define CONFIG_CMD_MMC
#endif

/* Generic configuration for FSMC NAND driver */
#if defined(CONFIG_NAND_FSMC)
	#define CONFIG_SYS_NAND_SELF_INIT
	#define CONFIG_SYS_MAX_NAND_DEVICE	1
	#define CONFIG_SYS_NAND_ONFI_DETECTION
	#define CONFIG_SYS_NAND_QUIET_TEST

	#define CONFIG_CMD_NAND
#endif

/* Generic configuration for environment */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CMD_GREPENV

#if !defined(CONFIG_ENV_SIZE)
	#define CONFIG_ENV_SIZE			0x02000
#endif

#if defined(CONFIG_SPEAR_USBTTY)
	#undef CONFIG_ENV_IS_IN_FLASH
	#undef CONFIG_ENV_IS_IN_NAND
	#define CONFIG_ENV_IS_NOWHERE
	#define CONFIG_PREBOOT		"setenv stdout usbtty;" \
					"setenv stderr usbtty;" \
					"setenv stdin usbtty"
#endif

#if defined(CONFIG_SPEAR_USBTTY)
	#define CONFIG_BOOTDELAY		-1
#else
	#define CONFIG_BOOTDELAY		1
#endif

#define CONFIG_NFSBOOTCOMMAND						\
	"bootp; run nfsargs; run loados; run loaddtb; "			\
	"bootm 0x900000 - 0x800000"

#define CONFIG_RAMBOOTCOMMAND						\
	"setenv bootargs root=/dev/ram rw "				\
		"console=ttyAMA0,115200 $(othbootargs);"		\
	CONFIG_BOOTCOMMAND

/* Miscellaneous configurable options */
#define CONFIG_SPL_IMAGENAME			Xloader
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_SYS_EXCEPTION_VECTORS_HIGH

#ifndef CONFIG_SYS_PNOR_BOOT_BASE
	#define CONFIG_SYS_PNOR_BOOT_BASE	0x0
#endif

/*
 * There are 2 builds supported by u-boot source. DCACHE support is kept enabled
 * for SPL, so that the generic routines can be used for invalidating the
 * caches. This is a mandatory requirement from kernel at bootup
 */
#define CONFIG_POST				CONFIG_SYS_POST_MEMORY
#define CONFIG_SYS_POST_WORD_ADDR		0x0

#if !defined(CONFIG_SPL_BUILD)
	#define CONFIG_SYS_DCACHE_OFF
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS

#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_STOP_STR		" "
#define CONFIG_AUTOBOOT_PROMPT			\
		"Hit SPACE in %d seconds to stop autoboot.\n", bootdelay

#define CONFIG_SYS_MEMTEST_START		0x00800000
#define CONFIG_SYS_MEMTEST_END			0x04000000
#define CONFIG_SYS_MALLOC_LEN			(1024*1024)
#define CONFIG_IDENT_STRING			"-SPEAr"
#define CONFIG_SYS_LONGHELP

#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_PROMPT			"u-boot> "
#define CONFIG_SYS_CBSIZE			256
#define CONFIG_SYS_PBSIZE			(CONFIG_SYS_CBSIZE + \
						sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS			16
#define CONFIG_SYS_BARGSIZE			CONFIG_SYS_CBSIZE
#define CONFIG_SYS_LOAD_ADDR			0x00900000
#define CONFIG_SYS_CONSOLE_INFO_QUIET
#define CONFIG_SYS_64BIT_VSPRINTF

#define CONFIG_SYS_FLASH_EMPTY_INFO

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS			1
#define CONFIG_SYS_SDRAM_BASE			0x00000000
#define SDRAM_MAX_SIZE				0x40000000

#endif /* __CONFIG_SPEAR_H */
