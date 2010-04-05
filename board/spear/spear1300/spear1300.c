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
#include <netdev.h>
#include <nand.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>

#if defined(CONFIG_NAND_FSMC)
int fsmc_nand_init(struct nand_chip *nand);
#endif

DECLARE_GLOBAL_DATA_PTR;

int board_init(void)
{
	gd->bd->bi_arch_number = MACH_TYPE_SPEAR1300;
	gd->bd->bi_boot_params = CONFIG_BOOT_PARAMS_ADDR;

	return 0;
}

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = get_ram_size(PHYS_SDRAM_1,
					       PHYS_SDRAM_1_MAXSIZE);

	return 0;
}

int misc_init_r(void)
{
	setenv("verify", "n");

#if defined(CONFIG_SPEAR_USBTTY)
	setenv("stdin", "usbtty");
	setenv("stdout", "usbtty");
	setenv("stderr", "usbtty");
#endif
	return 0;
}

/*
 * board_nand_init - Board specific NAND initialization
 * @nand:	mtd private chip structure
 *
 * Called by nand_init_chip to initialize the board specific functions
 */

int board_nand_init(struct nand_chip *nand)
{
#if defined(CONFIG_NAND_FSMC)
	return fsmc_nand_init(nand);
#endif
}

#if defined(CONFIG_CMD_NET)
int board_eth_init(bd_t *bis)
{
	/*
	 * Board specific ethernet initialization comes here. eg. gmac clock
	 * configuration etc
	 */

	return dw_mii_initialize(0, CONFIG_SPEAR_ETHBASE);
}
#endif
