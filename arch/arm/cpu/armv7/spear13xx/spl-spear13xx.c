/*
 * (C) Copyright 2012
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
#include <post.h>
#include <asm/io.h>
#include <asm/arch/c3.h>
#include <asm/arch/generic.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mpmc.h>

static void spear13xx_init(void)
{
	/*
	 * Enable the filtering at L2 cache controller level.
	 * This code limits the address range which can be accessed through the
	 * two ports coming out of ARM core.
	 *
	 * Port 0 access -> All - (Port 1 access range)
	 * Port 1 access -> 0x0 - 0x4000,0000 (1GB) all DDR space
	 * available on evaluation board
	 */

	/* L2 cache controller filtering registers */
	writel(0x40000000, CONFIG_SYS_L2CACHE_BASE + 0xC04);
	writel(0x00000000 | 0x1, CONFIG_SYS_L2CACHE_BASE + 0xC00);

	cpu2_wake();

	cleanup_before_linux();
}

void spear13xx_mpmc_init(u32 *mpmcbase, u32 reg_num)
{
	u32 i;
	u32 *mpmc_reg_p = mpmcbase;
	u32 *mpmc_val_p = &mpmc_conf_vals[0];

	for (i = 0; i < reg_num; i++, mpmc_reg_p++, mpmc_val_p++)
		writel(*mpmc_val_p, mpmc_reg_p);

	mpmc_reg_p = mpmcbase;

	/*
	 * Enable default DDR pad termination settings during read operations
	 * Modify MPMC registers 160-164
	 */
	for (i = 160; i <= 164; i++)
		writel((readl(&mpmc_reg_p[i]) & ~0x3FFFF) | 0x03219,
				&mpmc_reg_p[i]);

	/*
	 * MPMC register25 rewrite
	 * MPMC controller start
	 */
	writel(0x03070700, &mpmc_reg_p[25]);
	writel(0x01000101, &mpmc_reg_p[11]);

	while (!(readl(&mpmc_reg_p[105]) & 0x200))
		;
}

void wait_time(void)
{
	u32 count;

	for (count = 0; count < 1000000; count++)
		nop();
}

void spear13xx_ddr_level(void)
{
#if defined(CONFIG_DDR2)
	ddr2_lvl_write();
	wait_time();
	ddr2_lvl_read();
	wait_time();
	ddr2_lvl_gatetrn();
#elif defined(CONFIG_DDR3)
	ddr3_lvl_write();
	wait_time();
	ddr3_lvl_gatetrn();
	wait_time();
	ddr3_lvl_read();
#endif
}

static void spear13xx_ddrecc_init(void)
{
	/*
	 * The complete memory has to be initialized so that the ECC DDR gets
	 * populated and it starts working
	 *
	 * There are two ways to handle this
	 * - Through CPU. Cache has to be enabled to make it faster.
	 * - Through accelarator (C3)
	 */
#if defined(CONFIG_DDR_ECC_ENABLE)
	u32 ram_size = get_ram_size(CONFIG_SYS_SDRAM_BASE, SDRAM_MAX_SIZE);
	u32 *mpmc_reg_p = (u32 *)CONFIG_SYS_MPMC_BASE;
	u32 reg18;

	reg18 = readl(&mpmc_reg_p[18]);
	reg18 &= ~MPMC_REG18_ECC_MASK;
	reg18 |= MPMC_REG18_ECC_CORR_ON;
	writel(reg18, &mpmc_reg_p[18]);

	while ((readl(&mpmc_reg_p[18]) & MPMC_REG18_ECC_MASK) !=
			MPMC_REG18_ECC_CORR_ON)
		;
#if defined(CONFIG_C3)
	c3_init();
	c3_memset((void *)0, 0, ram_size);
#else
#error Define CONFIG_C3 for DDR ECC support
#endif
#endif /* CONFIG_DDR_ECC_ENABLE */
}

void __def_board_init(void)
{
	return;
}
void board_pre_ddrinit(void) __attribute__((weak, alias("__def_board_init")));
void board_post_ddrinit(void) __attribute__((weak, alias("__def_board_init")));

void lowlevel_init(void)
{
	/* ARCH specific initialization */
	spear13xx_init();

	/* Board initializations required for ddr initialization */
	board_pre_ddrinit();

	/* Initialize the SoC */
	soc_init();

	/* Initialize RAM */
	ddr_init();

	/* DDR ECC related initialization */
	spear13xx_ddrecc_init();

	/* Board initializations independent of DDR */
	board_post_ddrinit();

	/* A memory test is placed in U-Boot */
}

inline void hang(void)
{
	while(1)
		;
}
