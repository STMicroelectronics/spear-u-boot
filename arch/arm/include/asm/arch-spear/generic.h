/*
 * (C) Copyright 2009
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

#ifndef ASM_ARCH_SPEAR_GENERIC_H
#define ASM_ARCH_SPEAR_GENERIC_H

/* Routines exported from baord */
extern void setfreq(unsigned int, unsigned int);
extern unsigned int setfreq_sz;
extern void board_ddr_init(void);
extern void board_lowlevel_late_init(void);

/* Routines exported from SoC area */
extern void emi_init(void);
extern void spear3xx_usbh_stop(void);
extern void spear6xx_usbh_stop(void);
extern void spear3xx_ddr_comp_init(void);

extern u32 mpmc_conf_vals[];

/*
 * All SoCs should support a generic routine getboottype() which returns a
 * bitmask with one or more of the following bits set. This is a generic routine
 * which can be used on any SoC
 */
#define BOOT_TYPE_BYPASS	(1 << 1)
#define BOOT_TYPE_SMI		(1 << 2)
#define BOOT_TYPE_NAND		(1 << 3)
#define BOOT_TYPE_RESERVED	(1 << 4)
#define BOOT_TYPE_I2C		(1 << 5)
#define BOOT_TYPE_SPI		(1 << 6)
#define BOOT_TYPE_USBD		(1 << 7)
#define BOOT_TYPE_TFTP		(1 << 8)
#define BOOT_TYPE_PCIE		(1 << 9)
#define BOOT_TYPE_UART		(1 << 10)
#define BOOT_TYPE_MMC		(1 << 11)
#define BOOT_TYPE_PNOR8		(1 << 12)
#define BOOT_TYPE_PNOR16	(1 << 13)
#define BOOT_TYPE_PNOR32	(1 << 14)
#define BOOT_TYPE_UNSUPPORTED	(1 << 31)

extern u32 getboottype(void);

/* xxx_boot_selected */
#define boot_bypass_selected()	\
		(getboottype() & BOOT_TYPE_BYPASS)
#define usbd_boot_selected()	\
		(getboottype() & BOOT_TYPE_USBD)
#define snor_boot_selected()	\
		(getboottype() & BOOT_TYPE_SMI)
#define nand_boot_selected()	\
		(getboottype() & BOOT_TYPE_NAND)
#define pnor8_boot_selected()	\
		(getboottype() & BOOT_TYPE_PNOR8)
#define pnor16_boot_selected()	\
		(getboottype() & BOOT_TYPE_PNOR16)
#define pnor32_boot_selected()	\
		(getboottype() & BOOT_TYPE_PNOR32)
#define pnor_boot_selected()	\
		(pnor8_boot_selected() || \
		pnor16_boot_selected() || \
		pnor32_boot_selected())
#define tftp_boot_selected()	\
		(getboottype() & BOOT_TYPE_TFTP)
#define uart_boot_selected()	\
		(getboottype() & BOOT_TYPE_UART)
#define spi_boot_selected()	\
		(getboottype() & BOOT_TYPE_SPI)
#define i2c_boot_selected()	\
		(getboottype() & BOOT_TYPE_I2C)
#define pcie_boot_selected()	\
		(getboottype() & BOOT_TYPE_PCIE)
#define mmc_boot_selected()	\
		(getboottype() & BOOT_TYPE_MMC)

/* Revision definitions */
/*
 * The definitons have started from
 * 101 for SPEAr6xx
 * 201 for SPEAr3xx
 */
#define SOC_SPEAR_NA		0
#define SOC_SPEAR600_AA		101
#define SOC_SPEAR600_AB		102
#define SOC_SPEAR600_BA		103
#define SOC_SPEAR600_BB		104
#define SOC_SPEAR600_BC		105
#define SOC_SPEAR600_BD		106

#define SOC_SPEAR300		201
#define SOC_SPEAR310		202
#define SOC_SPEAR320		203

extern int get_socrev(void);

#endif
