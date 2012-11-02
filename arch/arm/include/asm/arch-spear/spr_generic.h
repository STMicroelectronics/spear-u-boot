/*
 * SPEAr generic declarations
 *
 * Copyright (C) 2009 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _SPR_GENERIC_
#define _SPR_GENERIC_

#include <asm/arch/padmux.h>

extern struct pmx_dev spear3xx_pmx_i2c;
extern struct pmx_dev spear3xx_pmx_ssp_cs;
extern struct pmx_dev spear3xx_pmx_ssp;
extern struct pmx_dev spear3xx_pmx_mii;
extern struct pmx_dev spear3xx_pmx_uart0;
extern struct pmx_dev spear320_pmx_emi;
extern struct pmx_dev spear320s_pmx_fsmc[];
extern struct pmx_dev spear320_pmx_sdhci[];
extern struct pmx_dev spear320_pmx_uart1;
extern struct pmx_dev spear320s_pmx_mii2;
extern struct pmx_dev spear320_pmx_mii1_2[];
extern struct pmx_mode spear320s_extended_mode;

void spear320_common_init(struct pmx_mode *pmx_mode, struct pmx_dev
		**pmx_devs, u8 pmx_dev_count);
void spear3xx_pmx_init_addr(struct pmx_driver *driver, unsigned int addr);
#if defined(CONFIG_USB_EHCI_SPEAR)
void spear_usbh_stop(void);
#endif

#endif /* _SPR_GENERIC_ */
