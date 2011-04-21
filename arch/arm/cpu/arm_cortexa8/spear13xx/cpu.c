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
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/spr_misc.h>

u32 get_device_type(void)
{
	return 0;
}

#ifdef CONFIG_ARCH_CPU_INIT
int arch_cpu_init(void)
{
	struct misc_regs *const misc_p =
	    (struct misc_regs *)CONFIG_SPEAR_MISCBASE;
	u32 perip1_clk_enb, perip2_clk_enb;
#if defined(CONFIG_SPEAR_MMC)
	u32 perip_cfg;
#endif
#if defined(CONFIG_NAND_FSMC)
	u32 fsmc_cfg;
#endif
	perip1_clk_enb = readl(&misc_p->perip1_clk_enb);
	perip2_clk_enb = readl(&misc_p->perip2_clk_enb);

	perip1_clk_enb |= GPT1_CLKEN;

#if defined(CONFIG_PL011_SERIAL)
	perip1_clk_enb |= UART_CLKEN;
#endif

#if defined(CONFIG_DESIGNWARE_ETH)
	writel(PHY_IF_GMII | CLK_SEL_PLL2, &misc_p->gmac_clk_cfg);
	perip1_clk_enb |= GETH_CLKEN;
#endif

#if defined(CONFIG_DW_UDC)
	perip1_clk_enb |= UDC_UPD_CLKEN;
#endif

#if defined(CONFIG_USB_EHCI_SPEAR)
	perip1_clk_enb |= UHC1_CLKEN;
#endif

#if defined(CONFIG_DW_I2C)
	perip1_clk_enb |= I2C_CLKEN;
#endif

#if defined(CONFIG_ST_SMI)
	perip1_clk_enb |= SMI_CLKEN;
#endif

#if defined(CONFIG_SPEAR_MMC)
	perip_cfg = readl(&misc_p->perip_cfg);
	perip_cfg &= ~MCIF_MSK;
	perip_cfg |= MCIF_SD;
	writel(perip_cfg, &misc_p->perip_cfg);

	writel(SYNT_X_2 | SYNT_Y_21 | SYNT_CLK_ENB,
			&misc_p->mcif_sd_clk_synt);

	perip1_clk_enb |= SD_CLKEN;
#endif

#if defined(CONFIG_NAND_FSMC)
	fsmc_cfg = readl(&misc_p->fsmc_cfg);
	fsmc_cfg &= ~DEV_SEL_MSK;
	fsmc_cfg |= DEV_SEL_NAND;
#if defined(CONFIG_SYS_FSMC_NAND_16BIT)
	fsmc_cfg |= DEV_WIDTH_16;
#elif defined(CONFIG_SYS_FSMC_NAND_8BIT)
	fsmc_cfg |= DEV_WIDTH_8;
#endif
	writel(fsmc_cfg, &misc_p->fsmc_cfg);

	perip1_clk_enb |= FSMC_CLKEN;
#endif

	writel(perip1_clk_enb, &misc_p->perip1_clk_enb);
	writel(perip2_clk_enb, &misc_p->perip2_clk_enb);

#ifdef CONFIG_SPEAR1300_ISSUE_101435
	{
		u32	tmp_var;

		/*
		 * WRKD the UHC problem (issue #101435)
		 *
		 * Bit2 in MISC sys_sw_res reg is used to
		 * discriminate whether we are coming from a power-on
		 * reset or not.
		 */
		tmp_var = readl(&misc_p->sys_sw_res);
		if ((tmp_var & 0x4) == 0) {
			/* Enable UHC1, UHC2 and UDC clocks) */
			tmp_var = readl(&misc_p->perip1_clk_enb);
			writel(tmp_var | 0xe00, &misc_p->perip1_clk_enb);

			/* Wait few cycles */
			for (tmp_var = 0; tmp_var < 0x10000; tmp_var++)
				;

			/* Reset the system */
			writel(0x1, &misc_p->sys_sw_res);
			while (1)
				;
		} else {
			/* Clear the RESET condition. */
			writel(0x2, &misc_p->sys_sw_res);
		}

	}
#endif
	return 0;
}
#endif

#ifdef CONFIG_DISPLAY_CPUINFO
int print_cpuinfo(void)
{
#if defined(CONFIG_SPEAR1300)
	printf("CPU:   SPEAr1300\n");
#elif defined(CONFIG_SPEAR1310)
	printf("CPU:   SPEAr1310\n");
#elif defined(CONFIG_SPEAR900)
	printf("CPU:   SPEAr900\n");
#endif
	return 0;
}
#endif
