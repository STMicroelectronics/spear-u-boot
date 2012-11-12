/*
 * (C) Copyright 2009
 * Vipin Kumar, ST Microelectronics, vipin.kumar@st.com.
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
#include <flash.h>
#include <linux/err.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/st_smi.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>

#if !defined(CONFIG_SYS_NO_FLASH)

static struct smi_regs *const smicntl =
    (struct smi_regs * const)CONFIG_SYS_SMI_BASE;
static ulong bank_base[CONFIG_SYS_MAX_FLASH_BANKS] =
    CONFIG_SYS_FLASH_ADDR_BASE;
flash_info_t flash_info[CONFIG_SYS_MAX_FLASH_BANKS];

#if defined(CONFIG_MTD_DEVICE)
/* MTD interface for SMI devices */
static struct mtd_info smi_mtd_info[CONFIG_SYS_MAX_FLASH_BANKS];
static char smi_mtd_names[CONFIG_SYS_MAX_FLASH_BANKS][16];

static int smi_mtd_init(void);
#endif

/* data structure to maintain flash ids from different vendors */
struct flash_device {
	char *name;
	u8 erase_cmd;
	u32 device_id;
	u32 pagesize;
	unsigned long sectorsize;
	unsigned long size_in_bytes;
};

#define FLASH_ID(n, es, id, psize, ssize, size)	\
{				\
	.name = n,		\
	.erase_cmd = es,	\
	.device_id = id,	\
	.pagesize = psize,	\
	.sectorsize = ssize,	\
	.size_in_bytes = size	\
}

/*
 * List of supported flash devices.
 * Currently the erase_cmd field is not used in this driver.
 */
static struct flash_device flash_devices[] = {
	FLASH_ID("st m25p16"     , 0xd8, 0x00152020, 0x100, 0x10000, 0x200000),
	FLASH_ID("st m25p32"     , 0xd8, 0x00162020, 0x100, 0x10000, 0x400000),
	FLASH_ID("st m25p64"     , 0xd8, 0x00172020, 0x100, 0x10000, 0x800000),
	FLASH_ID("st m25p128"    , 0xd8, 0x00182020, 0x100, 0x40000, 0x1000000),
	FLASH_ID("st m25p05"     , 0xd8, 0x00102020, 0x80 , 0x8000 , 0x10000),
	FLASH_ID("st m25p10"     , 0xd8, 0x00112020, 0x80 , 0x8000 , 0x20000),
	FLASH_ID("st m25p20"     , 0xd8, 0x00122020, 0x100, 0x10000, 0x40000),
	FLASH_ID("st m25p40"     , 0xd8, 0x00132020, 0x100, 0x10000, 0x80000),
	FLASH_ID("st m25p80"     , 0xd8, 0x00142020, 0x100, 0x10000, 0x100000),
	FLASH_ID("st m45pe10"    , 0xd8, 0x00114020, 0x100, 0x10000, 0x20000),
	FLASH_ID("st m45pe20"    , 0xd8, 0x00124020, 0x100, 0x10000, 0x40000),
	FLASH_ID("st m45pe40"    , 0xd8, 0x00134020, 0x100, 0x10000, 0x80000),
	FLASH_ID("st m45pe80"    , 0xd8, 0x00144020, 0x100, 0x10000, 0x100000),
	FLASH_ID("mcr n25q128"   , 0xd8, 0x0018BA20, 0x100, 0x10000, 0x1000000),
	FLASH_ID("sp s25fl004"   , 0xd8, 0x00120201, 0x100, 0x10000, 0x80000),
	FLASH_ID("sp s25fl008"   , 0xd8, 0x00130201, 0x100, 0x10000, 0x100000),
	FLASH_ID("sp s25fl016"   , 0xd8, 0x00140201, 0x100, 0x10000, 0x200000),
	FLASH_ID("sp s25fl032"   , 0xd8, 0x00150201, 0x100, 0x10000, 0x400000),
	FLASH_ID("sp s25fl064"   , 0xd8, 0x00160201, 0x100, 0x10000, 0x800000),
	FLASH_ID("mac 25l512"    , 0xd8, 0x001020C2, 0x010, 0x10000, 0x10000),
	FLASH_ID("mac 25l1005"   , 0xd8, 0x001120C2, 0x010, 0x10000, 0x20000),
	FLASH_ID("mac 25l2005"   , 0xd8, 0x001220C2, 0x010, 0x10000, 0x40000),
	FLASH_ID("mac 25l4005"   , 0xd8, 0x001320C2, 0x010, 0x10000, 0x80000),
	FLASH_ID("mac 25l4005a"  , 0xd8, 0x001320C2, 0x010, 0x10000, 0x80000),
	FLASH_ID("mac 25l8005"   , 0xd8, 0x001420C2, 0x010, 0x10000, 0x100000),
	FLASH_ID("mac 25l1605"   , 0xd8, 0x001520C2, 0x100, 0x10000, 0x200000),
	FLASH_ID("mac 25l1605a"  , 0xd8, 0x001520C2, 0x010, 0x10000, 0x200000),
	FLASH_ID("mac 25l3205"   , 0xd8, 0x001620C2, 0x100, 0x10000, 0x400000),
	FLASH_ID("mac 25l3205a"  , 0xd8, 0x001620C2, 0x100, 0x10000, 0x400000),
	FLASH_ID("mac 25l6405"   , 0xd8, 0x001720C2, 0x100, 0x10000, 0x800000),
	FLASH_ID("wbd w25q128" , 0xd8, 0x001840EF, 0x100, 0x10000, 0x1000000),
};

/*
 * get_flash_device - Return flash_device pointer for a particular device id
 * @id:	 Device id
 *
 * Return flash_device pointer for a particular device id
 */
static struct flash_device *get_flash_device(u32 id)
{
	struct flash_device *flash_dev_p = &flash_devices[0];
	int i;

	for (i = 0; i < ARRAY_SIZE(flash_devices); i++, flash_dev_p++) {
		if (flash_dev_p->device_id == id)
			return flash_dev_p;
	}

	return NULL;
}

/*
 * smi_wait_xfer_finish - Wait until TFF is set in status register
 * @timeout:	 timeout in milliseconds
 *
 * Wait until TFF is set in status register
 */
static int smi_wait_xfer_finish(int timeout)
{
	ulong start = get_timer(0);

	while (get_timer(start) < timeout) {
		if (readl(&smicntl->smi_sr) & TFF)
			return 0;

		/* Try after 10 ms */
		udelay(10);
	};

	return -1;
}

/*
 * smi_read_id - Read flash id
 * @info:	 flash_info structure pointer
 * @banknum:	 bank number
 *
 * Read the flash id present at bank #banknum
 */
static unsigned int smi_read_id(flash_info_t *info, int banknum)
{
	unsigned int value;

	writel(readl(&smicntl->smi_cr1) | SW_MODE, &smicntl->smi_cr1);
	writel(READ_ID, &smicntl->smi_tr);
	writel((banknum << BANKSEL_SHIFT) | SEND | TX_LEN_1 | RX_LEN_3,
	       &smicntl->smi_cr2);

	if (smi_wait_xfer_finish(XFER_FINISH_TOUT))
		return -EIO;

	value = (readl(&smicntl->smi_rr) & 0x00FFFFFF);

	writel(readl(&smicntl->smi_sr) & ~TFF, &smicntl->smi_sr);
	writel(readl(&smicntl->smi_cr1) & ~SW_MODE, &smicntl->smi_cr1);

	return value;
}

/*
 * flash_get_size - Detect the SMI flash by reading the ID.
 * @base:	 Base address of the flash area bank #banknum
 * @banknum:	 Bank number
 *
 * Detect the SMI flash by reading the ID. Initializes the flash_info structure
 * with size, sector count etc.
 */
static ulong flash_get_size(ulong base, int banknum)
{
	flash_info_t *info = &flash_info[banknum];
	int value;
	int i;

	value = smi_read_id(info, banknum);

	if (value < 0) {
		printf("Flash id could not be read\n");
		return 0;
	}

	/* Matches chip-id to entire list of 'serial-nor flash' ids */
	for (i = 0; i < ARRAY_SIZE(flash_devices); i++) {
		if (flash_devices[i].device_id == value) {
			info->size = flash_devices[i].size_in_bytes;
			info->flash_id = value;
			info->start[0] = base;
			info->sector_count =
					info->size/flash_devices[i].sectorsize;

			return info->size;
		}
	}

	return 0;
}

/*
 * smi_read_sr - Read status register of SMI
 * @bank:	 bank number
 *
 * This routine will get the status register of the flash chip present at the
 * given bank
 */
static int smi_read_sr(int bank)
{
	u32 ctrlreg1, val;

	/* store the CTRL REG1 state */
	ctrlreg1 = readl(&smicntl->smi_cr1);

	/* Program SMI in HW Mode */
	writel(readl(&smicntl->smi_cr1) & ~(SW_MODE | WB_MODE),
	       &smicntl->smi_cr1);

	/* Performing a RSR instruction in HW mode */
	writel((bank << BANKSEL_SHIFT) | RD_STATUS_REG, &smicntl->smi_cr2);

	if (smi_wait_xfer_finish(XFER_FINISH_TOUT))
		return -1;

	val = readl(&smicntl->smi_sr);

	/* Restore the CTRL REG1 state */
	writel(ctrlreg1, &smicntl->smi_cr1);

	return val;
}

/*
 * smi_wait_till_ready - Wait till last operation is over.
 * @bank:	 bank number shifted.
 * @timeout:	 timeout in milliseconds.
 *
 * This routine checks for WIP(write in progress)bit in Status register(SMSR-b0)
 * The routine checks for #timeout loops, each at interval of 1 milli-second.
 * If successful the routine returns 0.
 */
static int smi_wait_till_ready(int bank, int timeout)
{
	int sr;
	ulong start = get_timer(0);

	/* One chip guarantees max 5 msec wait here after page writes,
	   but potentially three seconds (!) after page erase. */
	while (get_timer(start) < timeout) {
		sr = smi_read_sr(bank);
		if ((sr >= 0) && (!(sr & WIP_BIT)))
			return 0;

		/* Try again after 10 usec */
		udelay(10);
	} while (timeout--);

	printf("SMI controller is still in wait, timeout=%d\n", timeout);
	return -EIO;
}

/*
 * smi_write_enable - Enable the flash to do write operation
 * @bank:	 bank number
 *
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static int smi_write_enable(int bank)
{
	u32 ctrlreg1;
	u32 start;
	int timeout = WMODE_TOUT;
	int sr;

	/* Store the CTRL REG1 state */
	ctrlreg1 = readl(&smicntl->smi_cr1);

	/* Program SMI in H/W Mode */
	writel(readl(&smicntl->smi_cr1) & ~SW_MODE, &smicntl->smi_cr1);

	/* Give the Flash, Write Enable command */
	writel((bank << BANKSEL_SHIFT) | WE, &smicntl->smi_cr2);

	if (smi_wait_xfer_finish(XFER_FINISH_TOUT))
		return -1;

	/* Restore the CTRL REG1 state */
	writel(ctrlreg1, &smicntl->smi_cr1);

	start = get_timer(0);
	while (get_timer(start) < timeout) {
		sr = smi_read_sr(bank);
		if ((sr >= 0) && (sr & (1 << (bank + WM_SHIFT))))
			return 0;

		/* Try again after 10 usec */
		udelay(10);
	};

	return -1;
}

/*
 * smi_init - SMI initialization routine
 *
 * SMI initialization routine. Sets SMI control register1.
 */
void smi_init(void)
{
	/* Setting the fast mode values. SMI working at 166/4 = 41.5 MHz */
	writel(HOLD1 | FAST_MODE | BANK_EN | DSEL_TIME | PRESCAL4,
	       &smicntl->smi_cr1);
}

/*
 * smi_sector_erase - Erase flash sector
 * @info:	 flash_info structure pointer
 * @sector:	 sector number
 *
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static int smi_sector_erase(flash_info_t *info, unsigned int sector)
{
	int bank;
	unsigned int sect_add;
	unsigned int instruction;

	switch (info->start[0]) {
	case SMIBANK0_BASE:
		bank = BANK0;
		break;
	case SMIBANK1_BASE:
		bank = BANK1;
		break;
	case SMIBANK2_BASE:
		bank = BANK2;
		break;
	case SMIBANK3_BASE:
		bank = BANK3;
		break;
	default:
		return -1;
	}

	sect_add = sector * (info->size / info->sector_count);
	instruction = ((sect_add >> 8) & 0x0000FF00) | SECTOR_ERASE;

	writel(readl(&smicntl->smi_sr) & ~(ERF1 | ERF2), &smicntl->smi_sr);

	/* Wait until finished previous write command. */
	if (smi_wait_till_ready(bank, CONFIG_SYS_FLASH_ERASE_TOUT))
		return -EBUSY;

	/* Send write enable, before erase commands. */
	if (smi_write_enable(bank))
		return -EIO;

	/* Put SMI in SW mode */
	writel(readl(&smicntl->smi_cr1) | SW_MODE, &smicntl->smi_cr1);

	/* Send Sector Erase command in SW Mode */
	writel(instruction, &smicntl->smi_tr);
	writel((bank << BANKSEL_SHIFT) | SEND | TX_LEN_4,
		       &smicntl->smi_cr2);
	if (smi_wait_xfer_finish(XFER_FINISH_TOUT))
		return -EIO;

	if (smi_wait_till_ready(bank, CONFIG_SYS_FLASH_ERASE_TOUT))
		return -EBUSY;

	/* Put SMI in HW mode */
	writel(readl(&smicntl->smi_cr1) & ~SW_MODE,
		       &smicntl->smi_cr1);

	return 0;
}

/*
 * smi_write - Write to SMI flash
 * @info:	 flash info structure
 * @src_addr:	 source buffer
 * @dst_addr:	 destination buffer
 * @length:	 length to write in bytes
 *
 * Write to SMI flash
 */
static int smi_write(flash_info_t *info, unsigned char *src_addr,
		unsigned char *dst_addr, unsigned int length)
{
	struct flash_device *flash_device_p = get_flash_device(info->flash_id);
	u32 page_size;
	int banknum;
	int issue_we;

	if (!flash_device_p)
		return -EIO;

	page_size = flash_device_p->pagesize;

	switch (info->start[0]) {
	case SMIBANK0_BASE:
		banknum = BANK0;
		break;
	case SMIBANK1_BASE:
		banknum = BANK1;
		break;
	case SMIBANK2_BASE:
		banknum = BANK2;
		break;
	case SMIBANK3_BASE:
		banknum = BANK3;
		break;
	default:
		return -1;
	}

	writel(readl(&smicntl->smi_sr) & ~(ERF1 | ERF2), &smicntl->smi_sr);
	issue_we = 1;

	/* Set SMI in Hardware Mode */
	writel(readl(&smicntl->smi_cr1) & ~SW_MODE, &smicntl->smi_cr1);

	/* Perform the write command */
	while (length) {
		int k;
		unsigned int wlen = min(page_size, length);

		if (issue_we || (((ulong)(dst_addr) % page_size) == 0)) {
			issue_we = 0;

			if (smi_wait_till_ready(banknum,
						CONFIG_SYS_FLASH_WRITE_TOUT))
				return -EBUSY;

			if (smi_write_enable(banknum))
				return -EIO;
		}

		writel(readl(&smicntl->smi_cr1) | WB_MODE, &smicntl->smi_cr1);

		for (k = 0; k < wlen; k++)
			*dst_addr++ = *src_addr++;

		writel(readl(&smicntl->smi_cr1) & ~WB_MODE, &smicntl->smi_cr1);

		length -= wlen;
		if ((readl(&smicntl->smi_sr) & (ERF1 | ERF2)))
			return -EIO;
	}

	if (smi_wait_till_ready(banknum, CONFIG_SYS_FLASH_WRITE_TOUT))
		return -EBUSY;

	writel(readl(&smicntl->smi_sr) & ~(WCF), &smicntl->smi_sr);

	return 0;
}

/*
 * write_buff - Write to SMI flash
 * @info:	 flash info structure
 * @src:	 source buffer
 * @dest_addr:	 destination buffer
 * @length:	 length to write in words
 *
 * Write to SMI flash
 */
int write_buff(flash_info_t *info, uchar *src, ulong dest_addr, ulong length)
{
	return smi_write(info, src, (unsigned char *)dest_addr, length);
}

/*
 * flash_init - SMI flash initialization
 *
 * SMI flash initialization
 */
unsigned long flash_init(void)
{
	unsigned long size = 0;
	int i, j;
#if defined(CONFIG_MTD_DEVICE)
	int error;
#endif
	smi_init();

	for (i = 0; i < CONFIG_SYS_MAX_FLASH_BANKS; i++) {
		flash_info[i].flash_id = FLASH_UNKNOWN;
		size += flash_info[i].size = flash_get_size(bank_base[i], i);
	}

	for (j = 0; j < CONFIG_SYS_MAX_FLASH_BANKS; j++) {
		for (i = 1; i < flash_info[j].sector_count; i++)
			flash_info[j].start[i] =
			    flash_info[j].start[i - 1] +
			    flash_info->size / flash_info->sector_count;

	}

#if defined(CONFIG_MTD_DEVICE)
	error = smi_mtd_init();
	if (error < 0)
		return 0;
#endif

	return size;
}

/*
 * flash_print_info - Print SMI flash information
 *
 * Print SMI flash information
 */
void flash_print_info(flash_info_t *info)
{
	int i;
	if (info->flash_id == FLASH_UNKNOWN) {
		puts("missing or unknown FLASH type\n");
		return;
	}

	if (info->size >= 0x100000)
		printf("  Size: %ld MB in %d Sectors\n",
		       info->size >> 20, info->sector_count);
	else
		printf("  Size: %ld KB in %d Sectors\n",
		       info->size >> 10, info->sector_count);

	puts("  Sector Start Addresses:");
	for (i = 0; i < info->sector_count; ++i) {
#ifdef CONFIG_SYS_FLASH_EMPTY_INFO
		int size;
		int erased;
		u32 *flash;

		/*
		 * Check if whole sector is erased
		 */
		size = (info->size) / (info->sector_count);
		flash = (u32 *) info->start[i];
		size = size / sizeof(int);

		while ((size--) && (*flash++ == ~0))
			;

		size++;
		if (size)
			erased = 0;
		else
			erased = 1;

		if ((i % 5) == 0)
			printf("\n");

		printf(" %08lX%s%s",
		       info->start[i],
		       erased ? " E" : "  ", info->protect[i] ? "RO " : "   ");
#else
		if ((i % 5) == 0)
			printf("\n   ");
		printf(" %08lX%s",
		       info->start[i], info->protect[i] ? " (RO)  " : "     ");
#endif
	}
	putc('\n');
	return;
}

/*
 * flash_erase - Erase SMI flash
 *
 * Erase SMI flash
 */
int flash_erase(flash_info_t *info, int s_first, int s_last)
{
	int rcode = 0;
	int prot = 0;
	flash_sect_t sect;

	if ((s_first < 0) || (s_first > s_last)) {
		puts("- no sectors to erase\n");
		return 1;
	}

	for (sect = s_first; sect <= s_last; ++sect) {
		if (info->protect[sect])
			prot++;
	}
	if (prot) {
		printf("- Warning: %d protected sectors will not be erased!\n",
		       prot);
	} else {
		putc('\n');
	}

	for (sect = s_first; sect <= s_last; sect++) {
		if (info->protect[sect] == 0) {
			if (smi_sector_erase(info, sect))
				rcode = 1;
			else
				putc('.');
		}
	}
	puts(" done\n");
	return rcode;
}

#if defined(CONFIG_MTD_DEVICE)
static int smi_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	flash_info_t *fi = mtd->priv;
	size_t a_start = fi->start[0] + instr->addr;
	size_t a_end = a_start + instr->len;
	int s_first = -1;
	int s_last = -1;
	int error, sect;

	for (sect = 0; sect < fi->sector_count; sect++) {
		if (a_start == fi->start[sect])
			s_first = sect;

		if (sect < fi->sector_count - 1) {
			if (a_end == fi->start[sect + 1]) {
				s_last = sect;
				break;
			}
		} else {
			s_last = sect;
			break;
		}
	}

	if (s_first >= 0 && s_first <= s_last) {
		instr->state = MTD_ERASING;

		error = flash_erase(fi, s_first, s_last);
		if (error) {
			instr->state = MTD_ERASE_FAILED;
			return -EIO;
		}

		instr->state = MTD_ERASE_DONE;
		mtd_erase_callback(instr);
		return 0;
	}

	return -EINVAL;
}

static int smi_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	flash_info_t *fi = mtd->priv;
	u_char *f = (u_char*)(fi->start[0]) + from;

	memcpy(buf, f, len);
	*retlen = len;

	return 0;
}

static int smi_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	flash_info_t *fi = mtd->priv;
	u_long t = fi->start[0] + to;
	int error;

	error = write_buff(fi, (u_char*)buf, t, len);
	if (!error) {
		*retlen = len;
		return 0;
	}

	return -EIO;
}

static void smi_mtd_sync(struct mtd_info *mtd)
{
	/*
	 * This function should wait until all pending operations
	 * finish. However this driver is fully synchronous, so
	 * this function returns immediately
	 */
}

static int smi_mtd_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	flash_info_t *fi = mtd->priv;

	flash_protect(FLAG_PROTECT_SET, fi->start[0] + ofs,
					fi->start[0] + ofs + len - 1, fi);
	return 0;
}

static int smi_mtd_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	flash_info_t *fi = mtd->priv;

	flash_protect(FLAG_PROTECT_CLEAR, fi->start[0] + ofs,
					fi->start[0] + ofs + len - 1, fi);
	return 0;
}

static int smi_mtd_init(void)
{
	struct flash_device *flash_device_p;
	struct mtd_info *mtd;
	flash_info_t *fi;
	int i;

	for (i = 0; i < CONFIG_SYS_MAX_FLASH_BANKS; i++) {
		fi = &flash_info[i];

		if (!fi->size)
			continue;

		flash_device_p = get_flash_device(fi->flash_id);
		if (!flash_device_p)
			return -EIO;

		mtd = &smi_mtd_info[i];

		memset(mtd, 0, sizeof(struct mtd_info));

		/* Uniform erase sizes for all sectors */
		mtd->numeraseregions = 0;
		mtd->erasesize = flash_device_p->sectorsize;

		sprintf(smi_mtd_names[i], "nor%d", i);
		mtd->name		= smi_mtd_names[i];
		mtd->type		= MTD_NORFLASH;
		mtd->flags		= MTD_CAP_NORFLASH;
		mtd->size		= fi->size;
		mtd->writesize		= 1;

		mtd->erase		= smi_mtd_erase;
		mtd->read		= smi_mtd_read;
		mtd->write		= smi_mtd_write;
		mtd->sync		= smi_mtd_sync;
		mtd->lock		= smi_mtd_lock;
		mtd->unlock		= smi_mtd_unlock;
		mtd->priv		= fi;

		if (add_mtd_device(mtd))
			return -ENOMEM;
	}

	return 0;
}
#endif /* CONFIG_MTD_DEVICE */

#endif /* CONFIG_SYS_NO_FLASH */
