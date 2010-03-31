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
#include <nand.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/mtd/nand_ecc.h>
#include <asm/arch/hardware.h>
#include "fsmc_nand.h"

static u32 fsmc_version;
static struct fsmc_regs *const fsmc_regs_p =
    (struct fsmc_regs *)CONFIG_SYS_FSMC_BASE;

#if defined(CONFIG_BOARD_NAND_LP)
static struct nand_ecclayout fsmc_ecc13_layout = {
	.eccbytes = 104,
	.eccpos = {  2,   3,   4,   5,   6,   7,   8,
		9,  10,  11,  12,  13,  14,
		18,  19,  20,  21,  22,  23,  24,
		25,  26,  27,  28,  29,  30,
		34,  35,  36,  37,  38,  39,  40,
		41,  42,  43,  44,  45,  46,
		50,  51,  52,  53,  54,  55,  56,
		57,  58,  59,  60,  61,  62,
		66,  67,  68,  69,  70,  71,  72,
		73,  74,  75,  76,  77,  78,
		82,  83,  84,  85,  86,  87,  88,
		89,  90,  91,  92,  93,  94,
		98,  99, 100, 101, 102, 103, 104,
		105, 106, 107, 108, 109, 110,
		114, 115, 116, 117, 118, 119, 120,
		121, 122, 123, 124, 125, 126
	},
	.oobfree = {
		{.offset = 15, .length = 3},
		{.offset = 31, .length = 3},
		{.offset = 47, .length = 3},
		{.offset = 63, .length = 3},
		{.offset = 79, .length = 3},
		{.offset = 95, .length = 3},
		{.offset = 111, .length = 3},
		{.offset = 127, .length = 1}
	}
};

/*
 * ECC placement definitions in oobfree type format
 * There are 13 bytes of ecc for every 512 byte block and it has to be read
 * consicutively and immidiately after the 512 byte data block for hardware to
 * generate the error bit offsets in 512 byte data
 * Managing the ecc bytes in the following way makes it easier for software to
 * read ecc bytes consicutive to data bytes. This way is similar to
 * oobfree structure maintained already in u-boot nand driver
 */
static struct fsmc_eccplace fsmc_eccpl = {
	.eccplace = {
		{.offset = 2, .length = 13},
		{.offset = 18, .length = 13},
		{.offset = 34, .length = 13},
		{.offset = 50, .length = 13},
		{.offset = 66, .length = 13},
		{.offset = 82, .length = 13},
		{.offset = 98, .length = 13},
		{.offset = 114, .length = 13}
	}
};

#elif defined(CONFIG_BOARD_NAND_SP)
static struct nand_ecclayout fsmc_ecc13_layout = {
	.eccbytes = 13,
	.eccpos = { 0,  1,  2,  3,  6,  7, 8,
		9, 10, 11, 12, 13, 14
	},
	.oobfree = {
		{.offset = 15, .length = 1},
	}
};

static struct fsmc_eccplace fsmc_eccpl = {
	.eccplace = {
		    {.offset = 0, .length = 4},
		    {.offset = 6, .length = 9}
		    }
};

#else
#error Please define one of CONFIG_BOARD_NAND_SP or CONFIG_BOARD_NAND_LP
#endif

static struct nand_ecclayout fsmc_ecc3_layout = {
	.eccbytes = 24,
	.eccpos = {2, 3, 4, 18, 19, 20, 34, 35, 36, 50, 51, 52,
		   66, 67, 68, 82, 83, 84, 98, 99, 100, 114, 115, 116},
	.oobfree = {
		    {.offset = 8, .length = 8},
		    {.offset = 24, .length = 8},
		    {.offset = 40, .length = 8},
		    {.offset = 56, .length = 8},
		    {.offset = 72, .length = 8},
		    {.offset = 88, .length = 8},
		    {.offset = 104, .length = 8},
		    {.offset = 120, .length = 8}
		    }
};

static void fsmc_nand_hwcontrol(struct mtd_info *mtd, int cmd, uint ctrl)
{
	struct nand_chip *this = mtd->priv;
	ulong IO_ADDR_W;

	if (ctrl & NAND_CTRL_CHANGE) {
		IO_ADDR_W = (ulong)this->IO_ADDR_W;

		IO_ADDR_W &= ~(CONFIG_SYS_NAND_CLE | CONFIG_SYS_NAND_ALE);
		if (ctrl & NAND_CLE)
			IO_ADDR_W |= CONFIG_SYS_NAND_CLE;
		if (ctrl & NAND_ALE)
			IO_ADDR_W |= CONFIG_SYS_NAND_ALE;

		if (ctrl & NAND_NCE) {
			writel(readl(&fsmc_regs_p->genmemctrl_pc) |
			       FSMC_ENABLE, &fsmc_regs_p->genmemctrl_pc);
		} else {
			writel(readl(&fsmc_regs_p->genmemctrl_pc) &
			       ~FSMC_ENABLE, &fsmc_regs_p->genmemctrl_pc);
		}
		this->IO_ADDR_W = (void *)IO_ADDR_W;
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, this->IO_ADDR_W);
}

static int fsmc_correct_data(struct mtd_info *mtd, u_char *dat,
		      u_char *read_ecc, u_char *calc_ecc)
{
	/* The calculated ecc is actually the correction index in data */
	u16 err_idx[8];
	u64 ecc_data[2];
	u32 num_err, i = 0;

	memcpy(ecc_data, calc_ecc, 13);
	err_idx[0] = (ecc_data[0] & 0x1FFF);
	ecc_data[0] >>= 13;
	err_idx[1] = (ecc_data[0] & 0x1FFF);
	ecc_data[0] >>= 13;
	err_idx[2] = (ecc_data[0] & 0x1FFF);
	ecc_data[0] >>= 13;
	err_idx[3] = (ecc_data[0] & 0x1FFF);
	ecc_data[0] >>= 13;
	err_idx[4] = ((ecc_data[1] & 0x1) << 12) | ecc_data[0];
	ecc_data[1] >>= 1;
	err_idx[5] = (ecc_data[1] & 0x1FFF);
	ecc_data[1] >>= 13;
	err_idx[6] = (ecc_data[1] & 0x1FFF);
	ecc_data[1] >>= 13;
	err_idx[7] = (ecc_data[1] & 0x1FFF);

	num_err = (readl(&fsmc_regs_p->genmemctrl_sts) >> 10) & 0xF;

	if (num_err == 0xF)
		return -EBADMSG;

	while (num_err--) {
		change_bit(0, &err_idx[i]);
		change_bit(1, &err_idx[i]);

		if (err_idx[i] <= 512 * 8) {
			change_bit(err_idx[i], dat);
			i++;
		}
	}
	return i;
}

static int fsmc_read_hwecc(struct mtd_info *mtd,
			    const u_char *data, u_char *ecc)
{
	u_int ecc_tmp;

	switch (fsmc_version) {
	case FSMC_VER8:
		while (!(readl(&fsmc_regs_p->genmemctrl_sts) & FSMC_CODE_RDY))
			;

		ecc_tmp = readl(&fsmc_regs_p->genmemctrl_ecc1);
		ecc[0] = (u_char) (ecc_tmp >> 0);
		ecc[1] = (u_char) (ecc_tmp >> 8);
		ecc[2] = (u_char) (ecc_tmp >> 16);
		ecc[3] = (u_char) (ecc_tmp >> 24);

		ecc_tmp = readl(&fsmc_regs_p->genmemctrl_ecc2);
		ecc[4] = (u_char) (ecc_tmp >> 0);
		ecc[5] = (u_char) (ecc_tmp >> 8);
		ecc[6] = (u_char) (ecc_tmp >> 16);
		ecc[7] = (u_char) (ecc_tmp >> 24);

		ecc_tmp = readl(&fsmc_regs_p->genmemctrl_ecc3);
		ecc[8] = (u_char) (ecc_tmp >> 0);
		ecc[9] = (u_char) (ecc_tmp >> 8);
		ecc[10] = (u_char) (ecc_tmp >> 16);
		ecc[11] = (u_char) (ecc_tmp >> 24);

		ecc_tmp = readl(&fsmc_regs_p->genmemctrl_sts);
		ecc[12] = (u_char) (ecc_tmp >> 16);
		break;

	default:
		ecc_tmp = readl(&fsmc_regs_p->genmemctrl_ecc1);
		ecc[0] = (u_char) (ecc_tmp >> 0);
		ecc[1] = (u_char) (ecc_tmp >> 8);
		ecc[2] = (u_char) (ecc_tmp >> 16);
		break;
	}

	return 0;
}

void fsmc_enable_hwecc(struct mtd_info *mtd, int mode)
{
	writel(readl(&fsmc_regs_p->genmemctrl_pc) & ~FSMC_ECCPLEN_256,
	       &fsmc_regs_p->genmemctrl_pc);
	writel(readl(&fsmc_regs_p->genmemctrl_pc) & ~FSMC_ECCEN,
	       &fsmc_regs_p->genmemctrl_pc);
	writel(readl(&fsmc_regs_p->genmemctrl_pc) | FSMC_ECCEN,
	       &fsmc_regs_p->genmemctrl_pc);
}

/**
 * fsmc_read_page_hwecc
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 * @page:	page number to read
 *
 * This routine is needed for fsmc verison 8 as reading from NAND chip has to be
 * performed in a strict sequence as follows:
 * data(512 byte) -> ecc(13 byte)
 * After this read, fsmc hardware generates and reports error data bits(upto a
 * max of 8 bits)
 */
static int fsmc_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				 uint8_t *buf, int page)
{
	int i, j, s, stat, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	int off, len;
	/*
	 * ecc_oob is intentionally taken as u16. In 16bit devices, we end up
	 * reading 14 bytes (7 words) from oob. The local array is to maintain
	 * word alignment
	 */
	uint16_t ecc_oob[7];

	for (i = 0, s = 0; s < eccsteps; s++, i += eccbytes, p += eccsize) {

		chip->cmdfunc(mtd, NAND_CMD_READ0, s * eccsize, page);
		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		chip->read_buf(mtd, p, eccsize);

		for (j = 0; j < eccbytes;) {
			off = fsmc_eccpl.eccplace[s].offset;
			len = fsmc_eccpl.eccplace[s].length;

			/*
			 * length is intentionally kept a higher multiple of 2
			 * to read at least 13 bytes even in case of 16 bit NAND
			 * devices
			 */
			len = ((len + 1) >> 1) << 1;
			chip->cmdfunc(mtd, NAND_CMD_READOOB, off, page);
			chip->read_buf(mtd, (uint8_t *)&ecc_oob[j], len);
			j += len;
		}

		memcpy(&ecc_code[i], ecc_oob, 13);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);

		stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}

	return 0;
}

int fsmc_nand_init(struct nand_chip *nand)
{
	u32 peripid2 = readl(&fsmc_regs_p->genmemctrl_peripid2);

	fsmc_version = (peripid2 >> FSMC_REVISION_SHFT) & \
		       FSMC_REVISION_MSK;
#if defined(CONFIG_BOARD_NAND_16BIT)
	writel(FSMC_DEVWID_16 | FSMC_DEVTYPE_NAND | FSMC_ENABLE | FSMC_WAITON,
	       &fsmc_regs_p->genmemctrl_pc);
#elif defined(CONFIG_BOARD_NAND_8BIT)
	writel(FSMC_DEVWID_8 | FSMC_DEVTYPE_NAND | FSMC_ENABLE | FSMC_WAITON,
	       &fsmc_regs_p->genmemctrl_pc);
#else
#error Please define one of CONFIG_BOARD_NAND_16BIT or CONFIG_BOARD_NAND_8BIT
#endif
	writel(readl(&fsmc_regs_p->genmemctrl_pc) | FSMC_TCLR_1 | FSMC_TAR_1,
	       &fsmc_regs_p->genmemctrl_pc);
	writel(FSMC_THIZ_1 | FSMC_THOLD_4 | FSMC_TWAIT_6 | FSMC_TSET_0,
	       &fsmc_regs_p->genmemctrl_comm);
	writel(FSMC_THIZ_1 | FSMC_THOLD_4 | FSMC_TWAIT_6 | FSMC_TSET_0,
	       &fsmc_regs_p->genmemctrl_attrib);

	nand->options = 0;
#if defined(CONFIG_BOARD_NAND_16BIT)
	nand->options |= NAND_BUSWIDTH_16;
#endif
	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.size = 512;
	nand->ecc.calculate = fsmc_read_hwecc;
	nand->ecc.hwctl = fsmc_enable_hwecc;
	nand->cmd_ctrl = fsmc_nand_hwcontrol;

	switch (fsmc_version) {
	case FSMC_VER8:
		nand->ecc.bytes = 13;
		nand->ecc.layout = &fsmc_ecc13_layout;
		nand->ecc.correct = fsmc_correct_data;
		nand->ecc.read_page = fsmc_read_page_hwecc;
		break;
	default:
		nand->ecc.bytes = 3;
		nand->ecc.layout = &fsmc_ecc3_layout;
		nand->ecc.correct = nand_correct_data;
		break;
	}

	return 0;
}
