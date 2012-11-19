/*
 * (C) Copyright 2000-2009
 * Vipin Kumar, ST Microelectronics, vipin.kumar@st.com
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <image.h>
#include <linux/compiler.h>
#include <linux/mtd/st_smi.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/generic.h>
#include <asm/arch/spl_nand.h>
#include <asm/arch/spl_pnor.h>

uint32_t crc32(uint32_t, const unsigned char *, uint);

int image_check_header(image_header_t *hdr)
{
	if (image_check_magic(hdr) && image_check_hcrc(hdr))
		return 1;

	return 0;
}

int image_check_data(image_header_t *hdr)
{
	ulong data = image_get_load(hdr);
	ulong len = image_get_data_size(hdr);
	ulong dcrc = crc32(0, (unsigned char *)data, len);

	return (dcrc == image_get_dcrc(hdr));
}

/*
 * SNOR (Serial NOR flash) related functions
 */
void snor_init(void)
{
	struct smi_regs *const smicntl =
		(struct smi_regs * const)CONFIG_SYS_SMI_BASE;

	/* Setting the fast mode values. SMI working at 166/4 = 41.5 MHz */
	writel(HOLD1 | FAST_MODE | BANK_EN | DSEL_TIME | PRESCAL4,
	       &smicntl->smi_cr1);
}

static int snor_image_load(u8 *load_addr, void (**image_p)(void))
{
	image_header_t header;

	memcpy(&header, load_addr, sizeof(image_header_t));

	if (image_check_header(&header)) {
		/* Copy the image to load address */
		memcpy((void *)image_get_load(&header),
		       load_addr + sizeof(image_header_t),
		       image_get_data_size(&header));

		if (image_check_data(&header)) {
			/* Jump to boot image */
			*image_p = (void (*)(void))image_get_load(&header);
			return 1;
		}
	}

	return 0;
}

static int nand_image_load(u32 blkstart, void (**image_p)(void))
{
	image_header_t header;
	int ret = 0, blknum = blkstart;
	size_t size;
	ulong load_address;

	do {
		size = sizeof(image_header_t);
		ret = nand_read_skip_bad(blknum, 0, &size, (u_char *)&header);

		if ((ret >= 0) && image_check_header(&header)) {
			size = image_get_data_size(&header);
			load_address = image_get_load(&header);

			ret = nand_read_skip_bad(blknum,
						 sizeof(image_header_t),
						 &size, (void *)load_address);
			if (image_check_data(&header)) {
				/* Jump to boot image */
				*image_p = (void (*)(void))image_get_load(&header);
				return 1;
			}
		}
	} while (++blknum < blkstart + 4);

	return 0;
}

static void pnorcopy(void *dest, const void *src, size_t len,
		pnor_width_t width)
{
	const u32 *src_32 = src;
	const u16 *src_16 = src;
	const u8 *src_8 = src;
	u32 *dest_32 = dest;
	u16 *dest_16 = dest;
	u8 *dest_8 = dest;
	int i;

	switch (width) {
	case PNOR_WIDTH_32:
		for (i = 0; i < len >> 2; i++)
			*dest_32++ = *src_32++;
		break;
	case PNOR_WIDTH_16:
		for (i = 0; i < len >> 1; i++)
			*dest_16++ = *src_16++;
		break;
	case PNOR_WIDTH_8:
	default:
		for (i = 0; i < len; i++)
			*dest_8++ = *src_8++;
		break;
	}
}

static int pnor_image_load(const void *load_addr, void (**image_p)(void),
		pnor_width_t width)
{
	image_header_t header;
	u32 numbytes;

	pnorcopy((void *)&header, load_addr, sizeof(header), width);

	if (image_check_header(&header)) {
		numbytes = image_get_data_size(&header);

		/* Copy the image to load address */
		pnorcopy((void *)image_get_load(&header),
				load_addr + sizeof(header), numbytes, width);

		if (image_check_data(&header)) {
			/* Jump to boot image */
			*image_p = (void (*)(void))image_get_load(&header);
			return 1;
		}
	}

	return 0;
}

static void boot_image(void (*image)(void))
{
	void (*funcp)(void) __noreturn = (void *)image;

	(*funcp)();
}

static pnor_width_t __def_get_pnor_width(void)
{
	return PNOR_WIDTH_SEARCH;
}
pnor_width_t get_pnor_width(void)
	__attribute__((weak, alias("__def_get_pnor_width")));

static void __def_board_lowlevel_late_init(void)
{
}
void board_lowlevel_late_init(void)
	__attribute__((weak, alias("__def_board_lowlevel_late_init")));

/*
 * spl_boot:
 *
 * All supported booting types of all supported SoCs are listed here.
 * Generic readback APIs are provided for each supported booting type
 * eg. nand_read_skip_bad
 */
u32 spl_boot(void)
{
	void (*image)(void);

#ifdef CONFIG_SPEAR_USBTTY
	board_lowlevel_late_init();
	return 1;
#endif

	/*
	 * All the supported booting devices are listed here. Each of
	 * the booting type supported by the platform would define the
	 * macro xxx_BOOT_SUPPORTED to TRUE.
	 */

	if (SNOR_BOOT_SUPPORTED && snor_boot_selected()) {
		/* SNOR-SMI initialization */
		snor_init();

		/* Serial NOR booting */
		if (snor_image_load((u8 *)CONFIG_SYS_SNOR_BOOT_BASE, &image)) {
			/* Platform related late initialasations */
			board_lowlevel_late_init();

			/* Jump to boot image */
			boot_image(image);
			return 1;
		}
	}

	if (NAND_BOOT_SUPPORTED && nand_boot_selected()) {
		/* NAND booting */
		/* NAND-FSMC initialization */
		spl_nand_init();

		/* NAND booting */
		if (nand_image_load(CONFIG_SYS_NAND_BOOT_BLK, &image)) {
			/* Platform related late initialasations */
			board_lowlevel_late_init();

			/* Jump to boot image */
			boot_image(image);
			return 1;
		}
	}

	if (PNOR_BOOT_SUPPORTED && pnor_boot_selected()) {
		/* PNOR booting */
		/* PNOR initialization */
		pnor_width_t width = get_pnor_width();

		if (width == PNOR_WIDTH_SEARCH)
			width = PNOR_WIDTH_8;

		/* NAND booting */
		if (pnor_image_load((const void *)CONFIG_SYS_PNOR_BOOT_BASE,
					&image, width)) {
			/* Platform related late initialasations */
			board_lowlevel_late_init();

			/* Jump to boot image */
			boot_image(image);
			return 1;
		}

		return 0;
	}

	if (MMC_BOOT_SUPPORTED && mmc_boot_selected()) {
		/* MMC booting */
		/* Not ported from XLoader to SPL yet */
		return 0;
	}

	if (SPI_BOOT_SUPPORTED && spi_boot_selected()) {
		/* SPI booting */
		/* Not supported for any platform as of now */
		return 0;
	}

	if (I2C_BOOT_SUPPORTED && i2c_boot_selected()) {
		/* I2C booting */
		/* Not supported for any platform as of now */
		return 0;
	}

	/*
	 * All booting types without memory are listed as below
	 * Control has to be returned to BootROM in case of all
	 * the following booting scenarios
	 */

	if (USBD_BOOT_SUPPORTED && usbd_boot_selected()) {
		board_lowlevel_late_init();
		return 1;
	}

	if (TFTP_BOOT_SUPPORTED && tftp_boot_selected()) {
		board_lowlevel_late_init();
		return 1;
	}

	if (UART_BOOT_SUPPORTED && uart_boot_selected()) {
		board_lowlevel_late_init();
		return 1;
	}

	/* Ideally, the control should not reach here. */
	hang();
}
