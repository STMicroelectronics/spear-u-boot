/*
 * (C) Copyright 2010
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

#include <common.h>
#include <linux/mtd/st_smi.h>
#include <asm/io.h>
#include <asm/arch/generic.h>
#include <asm/arch/hardware.h>
#include <asm/arch/misc.h>

int arch_cpu_init(void)
{
	struct misc_regs *const misc_p =
	    (struct misc_regs *)CONFIG_SPEAR_MISCBASE;
	u32 periph1_clken, periph_clk_cfg;

	periph1_clken = readl(&misc_p->periph1_clken);

#if defined(CONFIG_ARCH_SPEAR3XX)
	periph1_clken |= MISC_GPT2ENB;
#elif defined(CONFIG_SOC_SPEAR600)
	periph1_clken |= MISC_GPT3ENB;
#endif

#if defined(CONFIG_PL011_SERIAL)
	periph1_clken |= MISC_UART0ENB;

	periph_clk_cfg = readl(&misc_p->periph_clk_cfg);
	periph_clk_cfg &= ~MISC_UARTCLKMSK;
	periph_clk_cfg |= MISC_UART48M;
	writel(periph_clk_cfg, &misc_p->periph_clk_cfg);
#endif
#if defined(CONFIG_DESIGNWARE_ETH)
	periph1_clken |= MISC_ETHENB;
#endif
#if defined(CONFIG_DW_UDC)
	periph1_clken |= MISC_USBDENB;
#endif
#if defined(CONFIG_DW_I2C)
	periph1_clken |= MISC_I2CENB;
#endif
#if defined(CONFIG_ST_SMI)
	periph1_clken |= MISC_SMIENB;
#endif
#if defined(CONFIG_NAND_FSMC)
	periph1_clken |= MISC_FSMCENB;
#endif
#if defined(CONFIG_USB_EHCI_SPEAR)
	periph1_clken |= MISC_USBHENB;
#endif

	writel(periph1_clken, &misc_p->periph1_clken);

#if defined(CONFIG_SOC_SPEAR310) || defined(CONFIG_SOC_SPEAR320)
	writel(readl(&misc_p->amem_cfg_ctrl) | MISC_AMEM_CLKENB,
			&misc_p->amem_cfg_ctrl);
#endif

	/* Early driver initializations */
#if defined(CONFIG_ST_SMI)
	smi_init();
#endif

#ifdef CONFIG_ST_EMI
	emi_init();
#endif
	return 0;
}

#ifdef CONFIG_DISPLAY_CPUINFO
int print_cpuinfo(void)
{
#ifdef CONFIG_SOC_SPEAR300
	printf("CPU:   SPEAr300\n");
#elif defined(CONFIG_SOC_SPEAR310)
	printf("CPU:   SPEAr310\n");
#elif defined(CONFIG_SOC_SPEAR320)
	printf("CPU:   SPEAr320\n");
#elif defined(CONFIG_SOC_SPEAR600)
	printf("CPU:   SPEAr600\n");
#else
#error CPU not supported in spear platform
#endif
	return 0;
}
#endif

#ifdef CONFIG_POST
int arch_memory_test_prepare(u32 *vstart, u32 *size, phys_addr_t *phys_offset)
{
	/*
	 * Run the POST test on 64 MB memory starting from CONFIG_SYS_LOAD_ADDR
	 * The assumption here is that the DDR present on board is >= 128MB.
	 *
	 * The test runs before relocation (after the code copy has taken
	 * place), so it can not touch either before or after relocation areas
	 * of U-boot
	 *
	 * DDR usage
	 * <--------->|<---------------- / --------------->|<---------->
	 *   U-boot		Area to be used for		U-boot
	 *   before		POST test			after
	 *   relocation						relocation
	 */

	*vstart = CONFIG_SYS_LOAD_ADDR;
	*size = 64 << 20;

	return 0;
}

void arch_memory_failure_handle(void)
{
	hang();
}
#endif
