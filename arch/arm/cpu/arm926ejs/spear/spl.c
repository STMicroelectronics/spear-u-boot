/*
 * Copyright (C) 2011
 * Heiko Schocher, DENX Software Engineering, hs@denx.de.
 *
 * Copyright (C) 2012 Stefan Roese <sr@denx.de>
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

#include <common.h>
#include <version.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/generic.h>
#include <asm/arch/misc.h>
#include <asm/arch/syscntl.h>

inline void hang(void)
{
	for (;;)
		;
}

static void ddr_clock_init(void)
{
	struct misc_regs *misc_p = (struct misc_regs *)CONFIG_SPEAR_MISCBASE;
	u32 clkenb, ddrpll;

	clkenb = readl(&misc_p->periph1_clken);
	clkenb &= ~MISC_MPMCMSK;
	clkenb |= MISC_MPMCWE;

	/* Intentionally done twice */
	writel(clkenb, &misc_p->periph1_clken);
	writel(clkenb, &misc_p->periph1_clken);

	ddrpll = readl(&misc_p->pll_ctr_reg);
	ddrpll &= ~MISC_DDRCLK_MSK;
#if defined(CONFIG_SPEAR_DDR_HCLK)
	ddrpll |= MISC_DDRCLK_HCLK;
#elif defined(CONFIG_SPEAR_DDR_2HCLK)
	ddrpll |= MISC_DDRCLK_2HCLK;
#elif defined(CONFIG_SPEAR_DDR_PLL2)
	ddrpll |= MISC_DDRCLK_PLL2;
#else
#error "please define one of CONFIG_SPEAR_DDR_(HCLK|2HCLK|PLL2)"
#endif
	writel(ddrpll, &misc_p->pll_ctr_reg);

	writel(readl(&misc_p->periph1_clken) | MISC_MPMCENB,
			&misc_p->periph1_clken);
}

static void mpmc_init_values(void)
{
	u32 i;
	u32 *mpmc_reg_p = (u32 *)CONFIG_SPEAR_MPMCBASE;
	u32 *mpmc_val_p = &mpmc_conf_vals[0];

	for (i = 0; i < CONFIG_SPEAR_MPMCREGS; i++, mpmc_reg_p++, mpmc_val_p++)
		writel(*mpmc_val_p, mpmc_reg_p);

	mpmc_reg_p = (u32 *)CONFIG_SPEAR_MPMCBASE;

	/*
	 * MPMC controller start
	 * MPMC waiting for DLLLOCKREG high
	 */
	writel(0x01000100, &mpmc_reg_p[7]);

	while (!(readl(&mpmc_reg_p[3]) & 0x10000))
		;
}

static void ddr_init(void)
{
	/* Clock related settings for DDR */
	ddr_clock_init();

	/* Borad related low level initializations */
	board_ddr_init();

	/* Initialize mpmc register values */
	mpmc_init_values();
}

static void pll_init(void)
{
	struct misc_regs *misc_p = (struct misc_regs *)CONFIG_SPEAR_MISCBASE;

	/* Initialize PLLs */
	writel(MISC_FREQ_332, &misc_p->pll1_frq);
	writel(0x1C0A, &misc_p->pll1_cntl);
	writel(0x1C0E, &misc_p->pll1_cntl);
	writel(0x1C06, &misc_p->pll1_cntl);
	writel(0x1C0E, &misc_p->pll1_cntl);

	writel(MISC_FREQ_332, &misc_p->pll2_frq);
	writel(0x1C0A, &misc_p->pll2_cntl);
	writel(0x1C0E, &misc_p->pll2_cntl);
	writel(0x1C06, &misc_p->pll2_cntl);
	writel(0x1C0E, &misc_p->pll2_cntl);

	/* wait for pll locks */
	while (!(readl(&misc_p->pll1_cntl) & 0x1))
		;
	while (!(readl(&misc_p->pll2_cntl) & 0x1))
		;
}

static void sys_init(void)
{
	struct misc_regs *misc_p = (struct misc_regs *)CONFIG_SPEAR_MISCBASE;
	struct syscntl_regs *syscntl_p =
		(struct syscntl_regs *)CONFIG_SPEAR_SYSCNTLBASE;

	/* Set system state to SLOW */
	writel(SYSCNTL_REQ_SLOW, &syscntl_p->scctrl);
	writel(SYSCNTL_PLL_TIM << 3, &syscntl_p->scpllctrl);

	/* Initialize PLLs */
	pll_init();

	writel(MISC_RTC_DISABLE | MISC_PLLTIMEEN, &misc_p->periph_clk_cfg);
	writel(0x555, &misc_p->amba_clk_cfg);

	writel(SYSCNTL_REQ_NORMAL, &syscntl_p->scctrl);

	/* Wait for system to switch to normal mode */
	while ((readl(&syscntl_p->scctrl) & SYSCNTL_STATE_MASK) !=
			SYSCNTL_STATE_NORMAL)
		;
}

static void spear_icm_init(void)
{
	struct misc_regs *misc_p = (struct misc_regs *)CONFIG_SPEAR_MISCBASE;

	writel(0x80000007, &misc_p->arb_icm_ml1);
	writel(0x80000007, &misc_p->arb_icm_ml2);
	writel(0x80000007, &misc_p->arb_icm_ml3);
	writel(0x80000007, &misc_p->arb_icm_ml4);
	writel(0x80000007, &misc_p->arb_icm_ml5);
	writel(0x80000007, &misc_p->arb_icm_ml6);
	writel(0x80000007, &misc_p->arb_icm_ml7);
	writel(0x80000007, &misc_p->arb_icm_ml8);
	writel(0x80000007, &misc_p->arb_icm_ml9);
}

void lowlevel_init(void)
{
	struct misc_regs *misc_p = (struct misc_regs *)CONFIG_SPEAR_MISCBASE;

	/* Initialize PLLs */
	sys_init();

	/* Enable IPs (release reset) */
	writel(MISC_PERIPH_RST_ALL, &misc_p->periph1_rst);
#if defined(CONFIG_ARCH_SPEAR3XX)
	/*
	 * On SPEAr3xx the reset signal from periph1_rst bit 26 is
	 * propagated only enabling corresponding bit in periph1_clken
	 */
	writel(readl(&misc_p->periph1_clken) | MISC_USBHRENB,
			&misc_p->periph1_clken);
#endif

	/* Initialize MPMC */
	ddr_init();

	/* Initialize Interconnect Matrix */
	spear_icm_init();
}
