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
#if defined(CONFIG_NAND_FSMC)
	u32 fsmc_cfg;
#endif
	perip1_clk_enb = readl(&misc_p->perip1_clk_enb);
	perip2_clk_enb = readl(&misc_p->perip2_clk_enb);

	perip1_clk_enb |= GPT1_CLKEN;

#if defined(CONFIG_PL011_SERIAL)
	perip1_clk_enb |= UART_CLKEN;
#endif

#if defined(CONFIG_DW_ETH)
	/* writel(SYNT_CLK_ENB | SYNT_FIN_FULL | SYNT_X_1 | SYNT_Y_5,
			&misc_p->gmac_clk_synt); */
	writel(PHY_IF_GMII | CLK_SEL_OSCI3, &misc_p->gmac_clk_cfg);

	perip1_clk_enb |= GETH_CLKEN;
#endif

#if defined(CONFIG_DW_UDC)
	perip1_clk_enb |= UDC_UPD_CLKEN;
#endif

#if defined(CONFIG_DW_I2C)
	perip1_clk_enb |= I2C_CLKEN;
#endif

#if defined(CONFIG_ST_SMI)
	perip1_clk_enb |= SMI_CLKEN;
#endif

#if defined(CONFIG_NAND_FSMC)
	fsmc_cfg = readl(&misc_p->fsmc_cfg);
	fsmc_cfg &= ~DEV_SEL_MSK;
	fsmc_cfg |= DEV_SEL_NAND;
#if defined(CONFIG_BOARD_NAND_16BIT)
	fsmc_cfg |= DEV_WIDTH_16;
#elif defined(CONFIG_BOARD_NAND_8BIT)
	fsmc_cfg |= DEV_WIDTH_8;
#endif
	writel(fsmc_cfg, &misc_p->fsmc_cfg);

	perip1_clk_enb |= FSMC_CLKEN;
#endif

	writel(perip1_clk_enb, &misc_p->perip1_clk_enb);
	writel(perip2_clk_enb, &misc_p->perip2_clk_enb);

#if defined(CONFIG_DW_ETH)
	writel(readl(&misc_p->perip1_sw_rst) | GETH_CLKEN,
			&misc_p->perip1_sw_rst);
	writel(readl(&misc_p->perip1_sw_rst) & ~GETH_CLKEN,
			&misc_p->perip1_sw_rst);
#endif
	return 0;
}
#endif

#ifdef CONFIG_DISPLAY_CPUINFO
int print_cpuinfo(void)
{
	printf("CPU:   SPEAr1300\n");
	return 0;
}
#endif
