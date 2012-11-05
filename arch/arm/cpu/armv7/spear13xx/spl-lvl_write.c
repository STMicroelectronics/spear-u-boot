/*
 * (C) Copyright 2012
 * Vipin Kumar, ST Microelectronics, vipin.kumar@st.com
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
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mpmc.h>

static struct mpmc_regs *mpmc_p = (struct mpmc_regs *)CONFIG_SYS_MPMC_BASE;

static u32 source_sel_cal(u32 pattern)
{
	if ((pattern & 0xE1) == 0x60)
		return 1;
	else if ((pattern & 0xF0) == 0x30)
		return 2;
	else if ((pattern & 0x78) == 0x18)
		return 3;
	else if ((pattern & 0x3C) == 0x0C)
		return 4;
	else if ((pattern & 0x1E) == 0x06)
		return 5;
	else if ((pattern & 0x0F) == 0x03)
		return 6;
	else if ((pattern & 0x87) == 0x81)
		return 7;
	else
		return 0;
}

static void reset_phy_ctrl_reg(u32 *reg)
{
	int i;

	writel(readl(reg) | 0x1, reg);
	for (i = 0; i < PHY_CTRL_DELAY; i++)
		;
	writel(readl(reg) & ~0x1, reg);
}

static void prog_wrlvl_delay(u32 slice, u32 wrlvl_delay)
{
	u32 *wrlvl_delay_reg = &mpmc_p->reg100;
	u32 shift;

	wrlvl_delay_reg += slice >> 1;
	shift = (slice & 0x1) ? 16 : 0;
	writel_field(wrlvl_delay << shift, 0xFFFF << shift, wrlvl_delay_reg);
}

static u32 get_wrlvl_start(u32 wrlvl_base_off)
{
	u32 cal_clk_byte_patt, cal_ref_byte_patt;
	u32 clk_source_sel_cal_pre, ref_source_sel_cal_pre;
	int wrlvl_base, wrlvl_start;

	writel_field(wrlvl_base_off << 20, 7 << 20, &mpmc_p->reg129);
	cal_clk_byte_patt = readl_field(0, 0xFF, &mpmc_p->reg181);
	cal_ref_byte_patt = readl_field(8, 0xFF << 8, &mpmc_p->reg181);

	clk_source_sel_cal_pre = source_sel_cal(cal_clk_byte_patt);
	ref_source_sel_cal_pre = source_sel_cal(cal_ref_byte_patt);

	wrlvl_base = ref_source_sel_cal_pre - clk_source_sel_cal_pre;
	wrlvl_base &= 7;
	wrlvl_start = wrlvl_base - wrlvl_base_off;
	wrlvl_start &= 7;

	return wrlvl_start;
}

static u32 get_match_pre(u32 slice)
{
	u32 *obs_reg = &mpmc_p->reg175;
	u32 dqs_byte_patt_mux_dqs = (readl(obs_reg + slice) >> 8) & 0xFF;

	if ((dqs_byte_patt_mux_dqs & 0xE1) == 0x60)
		return 1;
	else if ((dqs_byte_patt_mux_dqs & 0xF0) == 0x30)
		return 2;
	else if ((dqs_byte_patt_mux_dqs & 0x78) == 0x18)
		return 3;
	else if ((dqs_byte_patt_mux_dqs & 0x3C) == 0x0C)
		return 4;
	else if ((dqs_byte_patt_mux_dqs & 0x1E) == 0x06)
		return 5;
	else if ((dqs_byte_patt_mux_dqs & 0x0F) == 0x03)
		return 6;
	else if ((dqs_byte_patt_mux_dqs & 0x87) == 0x81)
		return 7;
	else
		return 0;
}

static u32 get_match0_pre(u32 slice)
{
	u32 *obs_reg = &mpmc_p->reg175;
	u32 dqs_byte_patt_mux_data = (readl(obs_reg + slice) >> 16) & 0xFF;

	if ((dqs_byte_patt_mux_data & 0x1F) == 0x00)
		return 8;
	else if ((dqs_byte_patt_mux_data & 0xF8) == 0xF8)
		return 9;
	else if ((dqs_byte_patt_mux_data & 0xF8) == 0x78)
		return 1;
	else if ((dqs_byte_patt_mux_data & 0xFC) == 0x3C)
		return 2;
	else if ((dqs_byte_patt_mux_data & 0xFE) == 0x1E)
		return 3;
	else if ((dqs_byte_patt_mux_data & 0xFF) == 0x0F)
		return 4;
	else if ((dqs_byte_patt_mux_data & 0x7F) == 0x07)
		return 5;
	else if ((dqs_byte_patt_mux_data & 0x3F) == 0x03)
		return 6;
	else if ((dqs_byte_patt_mux_data & 0x1F) == 0x01)
		return 7;
	else
		return 0xF;
}

static u32 get_match1_pre(u32 slice)
{
	u32 *obs_reg = &mpmc_p->reg170;
	u32 dqs_byte_patt_mux_data1 = (readl(obs_reg + slice) >> 23) & 0xFF;

	if ((dqs_byte_patt_mux_data1 & 0x1F) == 0x00)
		return 8;
	else if ((dqs_byte_patt_mux_data1 & 0xF8) == 0xF8)
		return 9;
	else if ((dqs_byte_patt_mux_data1 & 0xF8) == 0x78)
		return 1;
	else if ((dqs_byte_patt_mux_data1 & 0xFC) == 0x3C)
		return 2;
	else if ((dqs_byte_patt_mux_data1 & 0xFE) == 0x1E)
		return 3;
	else if ((dqs_byte_patt_mux_data1 & 0xFF) == 0x0F)
		return 4;
	else if ((dqs_byte_patt_mux_data1 & 0x7F) == 0x07)
		return 5;
	else if ((dqs_byte_patt_mux_data1 & 0x3F) == 0x03)
		return 6;
	else if ((dqs_byte_patt_mux_data1 & 0x1F) == 0x01)
		return 7;
	else
		return 0xF;
}

static void set_dqs_parms(u32 wrlvl_start, u32 *final_wrlvl_delay)
{
	u32 *phy_ctrl_reg1 = &mpmc_p->reg130;
	u32 *phy_ctrl_reg4 = &mpmc_p->reg145;
	u32 *phy_ctrl_reg5 = &mpmc_p->reg150;
	u32 *phy_ctrl_reg6 = &mpmc_p->reg155;
	u32 *phy_ctrl_reg7 = &mpmc_p->reg160;

	u32 wr_dq_a_timing_reg, start_wrlvl_delay_mod, slice, dqtim;
	u32 phy_clk_phase_match_pre, dq_clk_phase_match0_pre, dq_clk_phase_match1_pre;
	u32 dq_clk_phase_match_pre;

	u32 dqs_dq_clk_phase_match_delta;

	u8 dqs_dq_clk_phase_match_delta_value[DATA_SLICE_MAX];
	u8 dqs_dq_clk_phase_match_delta_offset[DATA_SLICE_MAX];

	u32 dqs_dq_clk_phase_match_delta_expected;

	u8 start_wrlvl_delay_mod_value[DATA_SLICE_MAX];
	u8 start_wrlvl_delay_mod_offset_value[DATA_SLICE_MAX];

	static u32 phy_ctrl_reg1_dqsgate_assertval[] = {
		(0 << 0) | (1 << 3),
		(0 << 0) | (2 << 3),
		(1 << 0) | (1 << 3),
		(1 << 0) | (2 << 3)
	};
	static u32 phy_ctrl_reg1_dqsgate_deassertval[] = {
		(0 << 5) | (1 << 8),
		(0 << 5) | (2 << 8),
		(1 << 5) | (1 << 8),
		(1 << 5) | (2 << 8)
	};
	static u32 phy_ctrl_reg1_dqsoe_assertval[] = {
		(0 << 10) | (1 << 13),
		(0 << 10) | (2 << 13),
		(1 << 10) | (1 << 13),
		(1 << 10) | (2 << 13)
	};
	static u32 phy_ctrl_reg1_dqsoe_deassertval[] = {
		(1 << 15) | (2 << 18),
		(2 << 15) | (1 << 18),
		(2 << 15) | (2 << 18),
		(3 << 15) | (1 << 18)
	};
	static u32 phy_ctrl_reg6_dqs_assertval[] = {
		(0 << 10) | (1 << 13),
		(0 << 10) | (2 << 13),
		(1 << 10) | (1 << 13),
		(1 << 10) | (2 << 13)
	};
	static u32 phy_ctrl_reg6_dqs_deassertval[] = {
		(1 << 15) | (2 << 18),
		(2 << 15) | (1 << 18),
		(2 << 15) | (2 << 18),
		(3 << 15) | (1 << 18)
	};
	static u16 phy_ctrl_reg5_assertval[4][15] = {
		{0, 0, 0x08, 0x08, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x09, 0x09, 0x09, 0x09, 0x11},
		{0, 0, 0x00, 0x08, 0x08, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x09, 0x09, 0x09, 0x09},
		{0, 0, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x09, 0x09, 0x09},
		{0, 0, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x09, 0x09}
	};
	static u16 phy_ctrl_reg5_deassertval[4][15] = {
		{0, 0, 0x100, 0x100, 0x100, 0x100, 0x200, 0x200, 0x200, 0x200, 0x120, 0x120, 0x120, 0x120, 0x220},
		{0, 0, 0x000, 0x100, 0x100, 0x100, 0x100, 0x200, 0x200, 0x200, 0x200, 0x120, 0x120, 0x120, 0x120},
		{0, 0, 0x000, 0x000, 0x100, 0x100, 0x100, 0x100, 0x200, 0x200, 0x200, 0x200, 0x120, 0x120, 0x120},
		{0, 0, 0x000, 0x000, 0x000, 0x100, 0x100, 0x100, 0x100, 0x200, 0x200, 0x200, 0x200, 0x120, 0x120}
	};
	static u32 phy_ctrl_reg1_assertval[] = {
		(0 << 20) | (1 << 23),
		(0 << 20) | (2 << 23),
		(1 << 20) | (1 << 23),
		(1 << 20) | (2 << 23)
	};
	static u32 phy_ctrl_reg1_deassertval[] = {
		(1 << 25) | (2 << 28),
		(2 << 25) | (1 << 28),
		(2 << 25) | (2 << 28),
		(3 << 25) | (1 << 28)
	};
	static u32 phy_ctrl_reg7_assertval[] = {
		(0 << 18) | (1 << 21),
		(0 << 18) | (2 << 21),
		(1 << 18) | (1 << 21),
		(1 << 18) | (2 << 21)
	};
	static u32 phy_ctrl_reg7_deassertval[] = {
		(1 << 23) | (2 << 26),
		(2 << 23) | (1 << 26),
		(2 << 23) | (2 << 26),
		(3 << 23) | (1 << 26)
	};

	/*
	 * first for cycle
	 * data collection on dqs_dq_clk_phase_match_delta_value[slice]
	 * data collection on start_wrlvl_delay_mod_value[slice]
	 */
	for (slice = 0; slice < DATA_SLICE_MAX; slice++) {

		start_wrlvl_delay_mod = wrlvl_start + final_wrlvl_delay[slice];

		writel_field(phy_ctrl_reg1_dqsgate_assertval[start_wrlvl_delay_mod/4],
				DQSGATE_ASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg1_dqsgate_deassertval[start_wrlvl_delay_mod/4],
				DQSGATE_DEASSERT_MSK, phy_ctrl_reg1 + slice);

		writel_field(phy_ctrl_reg1_dqsoe_assertval[start_wrlvl_delay_mod/4],
				DQSOE_ASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg1_dqsoe_deassertval[start_wrlvl_delay_mod/4],
				DQSOE_DEASSERT_MSK, phy_ctrl_reg1 + slice);

		writel_field(phy_ctrl_reg6_dqs_assertval[start_wrlvl_delay_mod/4],
				REG6_DQS_ASSERT_MSK, phy_ctrl_reg6 + slice);
		writel_field(phy_ctrl_reg6_dqs_deassertval[start_wrlvl_delay_mod/4],
				REG6_DQS_DEASSERT_MSK, phy_ctrl_reg6 + slice);

		phy_clk_phase_match_pre = get_match_pre(slice);
		dq_clk_phase_match0_pre = get_match0_pre(slice);
		dq_clk_phase_match1_pre = get_match1_pre(slice);

		if ((dq_clk_phase_match0_pre & 0x8) == 0)
			dq_clk_phase_match_pre = dq_clk_phase_match0_pre & 0xF;
		else
			dq_clk_phase_match_pre = dq_clk_phase_match1_pre & 0xF;

		phy_clk_phase_match_pre &= 0xF;

		if (phy_clk_phase_match_pre > dq_clk_phase_match_pre)
			dqs_dq_clk_phase_match_delta = phy_clk_phase_match_pre - dq_clk_phase_match_pre;
		else
			dqs_dq_clk_phase_match_delta = dq_clk_phase_match_pre - phy_clk_phase_match_pre;
		dqs_dq_clk_phase_match_delta &= 0xF;

		if (dqs_dq_clk_phase_match_delta > 3) {
			/* if delta is too high recalculate */
			if ((dq_clk_phase_match0_pre & 0x8) == 0)
				dq_clk_phase_match_pre = dq_clk_phase_match1_pre & 0xF;
			else
				dq_clk_phase_match_pre = dq_clk_phase_match0_pre & 0xF;

			phy_clk_phase_match_pre &= 0xF;

			if (phy_clk_phase_match_pre > dq_clk_phase_match_pre)
				dqs_dq_clk_phase_match_delta = phy_clk_phase_match_pre - dq_clk_phase_match_pre;
			else
				dqs_dq_clk_phase_match_delta = dq_clk_phase_match_pre - phy_clk_phase_match_pre;
			dqs_dq_clk_phase_match_delta &= 0xF;

			if (dqs_dq_clk_phase_match_delta > 3)
				hang();
		}

		dqs_dq_clk_phase_match_delta_value[slice] = dqs_dq_clk_phase_match_delta;
		start_wrlvl_delay_mod_value[slice] = start_wrlvl_delay_mod;
	}

	/* dqs_dq_clk_phase_match_delta_expected calculation on slices 0,1,2 */
	dqs_dq_clk_phase_match_delta_expected = (unsigned int) (((dqs_dq_clk_phase_match_delta_value[0] +
						dqs_dq_clk_phase_match_delta_value[1] +
						dqs_dq_clk_phase_match_delta_value[2] + (3/2))/3));

	/*
	 * second for cycle
	 * start_wrlvl_delay_mod_offset_value calculation
	 */
	for (slice = 0; slice < DATA_SLICE_MAX; slice++) {
			dqs_dq_clk_phase_match_delta_offset[slice] =
		(u32) (dqs_dq_clk_phase_match_delta_expected -
			dqs_dq_clk_phase_match_delta_value[slice]);

		start_wrlvl_delay_mod_offset_value[slice] =
		(u32) (start_wrlvl_delay_mod_value[slice] -
			dqs_dq_clk_phase_match_delta_offset[slice]);
	}

	/* third for cycle, write leveling settings */
	for (slice = 0; slice < DATA_SLICE_MAX; slice++) {

		start_wrlvl_delay_mod = wrlvl_start + final_wrlvl_delay[slice];

		writel_field(phy_ctrl_reg1_dqsgate_assertval[start_wrlvl_delay_mod/4],
				DQSGATE_ASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg1_dqsgate_deassertval[start_wrlvl_delay_mod/4],
				DQSGATE_DEASSERT_MSK, phy_ctrl_reg1 + slice);

		writel_field(phy_ctrl_reg1_dqsoe_assertval[start_wrlvl_delay_mod/4],
				DQSOE_ASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg1_dqsoe_deassertval[start_wrlvl_delay_mod/4],
				DQSOE_DEASSERT_MSK, phy_ctrl_reg1 + slice);

		writel_field(phy_ctrl_reg6_dqs_assertval[start_wrlvl_delay_mod/4],
				REG6_DQS_ASSERT_MSK, phy_ctrl_reg6 + slice);
		writel_field(phy_ctrl_reg6_dqs_deassertval[start_wrlvl_delay_mod/4],
				REG6_DQS_DEASSERT_MSK, phy_ctrl_reg6 + slice);

		phy_clk_phase_match_pre = get_match_pre(slice);
		dq_clk_phase_match0_pre = get_match0_pre(slice);
		dq_clk_phase_match1_pre = get_match1_pre(slice);

		if ((dq_clk_phase_match0_pre & 0x8) == 0)
			dq_clk_phase_match_pre = dq_clk_phase_match0_pre & 0xF;
		else
			dq_clk_phase_match_pre = dq_clk_phase_match1_pre & 0xF;

		phy_clk_phase_match_pre &= 0xF;

		if (phy_clk_phase_match_pre > dq_clk_phase_match_pre)
			dqs_dq_clk_phase_match_delta = phy_clk_phase_match_pre - dq_clk_phase_match_pre;
		else
			dqs_dq_clk_phase_match_delta = dq_clk_phase_match_pre - phy_clk_phase_match_pre;
		dqs_dq_clk_phase_match_delta &= 0xF;

		if (dqs_dq_clk_phase_match_delta > 3) {
			/* if delta is too high recalculate */
			if ((dq_clk_phase_match0_pre & 0x8) == 0)
				dq_clk_phase_match_pre = dq_clk_phase_match1_pre & 0xF;
			else
				dq_clk_phase_match_pre = dq_clk_phase_match0_pre & 0xF;

			phy_clk_phase_match_pre &= 0xF;

			if (phy_clk_phase_match_pre > dq_clk_phase_match_pre)
				dqs_dq_clk_phase_match_delta = phy_clk_phase_match_pre - dq_clk_phase_match_pre;
			else
				dqs_dq_clk_phase_match_delta = dq_clk_phase_match_pre - phy_clk_phase_match_pre;
			dqs_dq_clk_phase_match_delta &= 0xF;

			if (dqs_dq_clk_phase_match_delta > 3)
				hang();
		}

		/* here we use start_wrlvl_delay_mod_offset_value[slice], phy_ctrl_reg4 and phy_ctrl_reg5 are related to DQ phase  */
		dqtim = (start_wrlvl_delay_mod_offset_value[slice] - 2) & 0x7;
		wr_dq_a_timing_reg = (dqtim << 0) | (dqtim << 3) | (dqtim << 6) | (dqtim << 9) |
			(dqtim << 12) | (dqtim << 15) | (dqtim << 18) | (dqtim << 21) | (dqtim << 24);

		writel(wr_dq_a_timing_reg, phy_ctrl_reg4 + slice);

		writel_field(phy_ctrl_reg5_assertval[dqs_dq_clk_phase_match_delta][start_wrlvl_delay_mod_offset_value[slice]],
				REG5_ASSERT_MSK, phy_ctrl_reg5 + slice);
		writel_field(phy_ctrl_reg5_deassertval[dqs_dq_clk_phase_match_delta][start_wrlvl_delay_mod_offset_value[slice]],
				REG5_DEASSERT_MSK, phy_ctrl_reg5 + slice);

		/* here we use start_wrlvl_delay_mod, phy_ctrl_reg1 and phy_ctrl_reg7 are related to DQS phase  */
		writel_field(phy_ctrl_reg1_assertval[start_wrlvl_delay_mod/4],
				REG1_ASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg1_deassertval[start_wrlvl_delay_mod/4],
				REG1_DEASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg7_assertval[start_wrlvl_delay_mod/4],
				REG7_ASSERT_MSK, phy_ctrl_reg7 + slice);
		writel_field(phy_ctrl_reg7_deassertval[start_wrlvl_delay_mod/4],
				REG7_DEASSERT_MSK, phy_ctrl_reg7 + slice);
	}
}

#if defined(CONFIG_DDR2)
static void set_wrlvldelay_ddr2(u32 wrlvl_start, u32 *final_wrlvl_delay)
{
	u32 *phy_ctrl_reg1 = &mpmc_p->reg130;
	u32 *phy_ctrl_reg0 = &mpmc_p->reg124;
	u32 slice;
	u32 start_wrlvl_delay_mod;
	u32 phy_ctrl_reg1_dqsgate_assertval[] = {
		(0 << 0) | (1 << 3),
		(0 << 0) | (2 << 3),
		(0 << 0) | (1 << 3),
		(0 << 0) | (2 << 3)
	};
	u32 phy_ctrl_reg1_dqsgate_deassertval[] = {
		(0 << 5) | (2 << 8),
		(0 << 5) | (1 << 8),
		(0 << 5) | (2 << 8),
		(0 << 5) | (1 << 8)
	};
	int WRLVL_DELAY_VALUE[5] = {2, 2, 2, 2, 2};

	WRLVL_DELAY_VALUE[0] = WRLVL_DELAY_VALUE_0;
	WRLVL_DELAY_VALUE[1] = WRLVL_DELAY_VALUE_1;
	WRLVL_DELAY_VALUE[2] = WRLVL_DELAY_VALUE_2;
	WRLVL_DELAY_VALUE[3] = WRLVL_DELAY_VALUE_3;
	WRLVL_DELAY_VALUE[4] = WRLVL_DELAY_VALUE_4;

	for (slice = 0; slice < DATA_SLICE_MAX; slice++) {

		reset_phy_ctrl_reg(phy_ctrl_reg0 + slice);

		final_wrlvl_delay[slice] = WRLVL_DELAY_VALUE[slice];

		prog_wrlvl_delay(slice, final_wrlvl_delay[slice]);
		start_wrlvl_delay_mod = wrlvl_start + final_wrlvl_delay[slice];

		writel_field(phy_ctrl_reg1_dqsgate_assertval[start_wrlvl_delay_mod/4],
				DQSGATE_ASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg1_dqsgate_deassertval[start_wrlvl_delay_mod/4],
				DQSGATE_DEASSERT_MSK, phy_ctrl_reg1 + slice);
	}
}

void ddr2_lvl_write(void)
{

	int wrlvl_start;
	u32 final_wrlvl_delay[DATA_SLICE_MAX];
	u32 wrlvl_base_offset_reg = 0;

	wrlvl_base_offset_reg = WRLVL_BASE_OFFSET_REG_VALUE;

	/* enable also wrlvl_reg_en, rdlvl_gate_reg_en, rdlvl_reg_en when START MPMC parameter is enabled  */
	writel(readl(&mpmc_p->reg15) | WRLVL_REG_EN, &mpmc_p->reg15);
	writel(readl(&mpmc_p->reg8) | RDLVL_GATE_REG_EN, &mpmc_p->reg8);
	writel(readl(&mpmc_p->reg9) | RDLVL_REG_EN, &mpmc_p->reg9);

	wrlvl_start = get_wrlvl_start(wrlvl_base_offset_reg);

	set_wrlvldelay_ddr2(wrlvl_start, final_wrlvl_delay);

	set_dqs_parms(wrlvl_start, final_wrlvl_delay);
}
#elif defined(CONFIG_DDR3)

static void set_wrlvldelay_ddr3(u32 wrlvl_start, u32 *final_wrlvl_delay)
{
	u32 *phy_ctrl_reg1 = &mpmc_p->reg130;
	u32 *phy_ctrl_reg0 = &mpmc_p->reg124;
	u32 wrlvl_delay, slice, start_search;
	u32 start_wrlvl_delay_mod, i;
	u32 phy_ctrl_reg1_dqsgate_assertval[] = {
		(0 << 0) | (1 << 3),
		(0 << 0) | (2 << 3),
		(0 << 0) | (1 << 3),
		(0 << 0) | (2 << 3)
	};
	u32 phy_ctrl_reg1_dqsgate_deassertval[] = {
		(0 << 5) | (2 << 8),
		(0 << 5) | (1 << 8),
		(0 << 5) | (2 << 8),
		(0 << 5) | (1 << 8)
	};
	u8 resp_total[DATA_SLICE_MAX][WRLVL_DELAY_MAX];

	/*
	 * Start write leveling operation to find the wrlvldelay parameters for
	 * each data slice
	 */
	for (slice = 0; slice < DATA_SLICE_MAX; slice++) {
		swlvl_start();
		wait_op_done();

		for (wrlvl_delay = 0; wrlvl_delay < WRLVL_DELAY_MAX; wrlvl_delay++) {
			start_wrlvl_delay_mod = wrlvl_start + wrlvl_delay;
			reset_phy_ctrl_reg(phy_ctrl_reg0 + slice);
			prog_wrlvl_delay(slice, wrlvl_delay);

			writel_field(phy_ctrl_reg1_dqsgate_assertval[start_wrlvl_delay_mod/4],
					DQSGATE_ASSERT_MSK, phy_ctrl_reg1 + slice);
			writel_field(phy_ctrl_reg1_dqsgate_deassertval[start_wrlvl_delay_mod/4],
					DQSGATE_DEASSERT_MSK, phy_ctrl_reg1 + slice);

			resp_total[slice][wrlvl_delay] = 0;

			for (i = 0; i < 4; i++) {
				swlvl_load();
				wait_op_done();

				resp_total[slice][wrlvl_delay] += read_resp(slice);
			}
		}

		start_search = 0;
		for (wrlvl_delay = 0; wrlvl_delay < WRLVL_DELAY_MAX; wrlvl_delay++) {
			if ((resp_total[slice][wrlvl_delay] < 4) && !start_search)
				start_search = 1;
			if ((resp_total[slice][wrlvl_delay] == 4) && start_search)
				break;
		}

		/*
		 * added 1 phase to DQS position
		 * original was final_wrlvl_delay[slice] = wrlvl_delay - 1
		 */
		final_wrlvl_delay[slice] = wrlvl_delay;

		prog_wrlvl_delay(slice, final_wrlvl_delay[slice]);
		start_wrlvl_delay_mod = wrlvl_start + final_wrlvl_delay[slice];

		writel_field(phy_ctrl_reg1_dqsgate_assertval[start_wrlvl_delay_mod/4],
				DQSGATE_ASSERT_MSK, phy_ctrl_reg1 + slice);
		writel_field(phy_ctrl_reg1_dqsgate_deassertval[start_wrlvl_delay_mod/4],
				DQSGATE_DEASSERT_MSK, phy_ctrl_reg1 + slice);

		swlvl_load();
		wait_op_done();

		swlvl_exit();
		wait_op_done();
	}
}

void ddr3_lvl_write(void)
{
	int wrlvl_start;
	u32 wrlvl_base_offset_reg;
	u32 final_wrlvl_delay[DATA_SLICE_MAX];
	u32 wrlvl_base_offset_check;
	u32 slice;

	wrlvl_base_offset_reg = 2; /* start from default value */

	do {
		set_swlvl_mode(WRITE_LVL);
		swlvl_start();
		wait_op_done();

		wrlvl_start = get_wrlvl_start(wrlvl_base_offset_reg);
		swlvl_exit();
		wait_op_done();

		/* Set wrlvl_delay parameters through write leveling */
		set_wrlvldelay_ddr3(wrlvl_start, final_wrlvl_delay);

		wrlvl_base_offset_check = 0;

		for (slice = 0; slice < DATA_SLICE_MAX; slice++)
			if (final_wrlvl_delay[slice] >= 8)
				wrlvl_base_offset_check++;

		if (wrlvl_base_offset_check > 0)
			wrlvl_base_offset_reg++;

		if (wrlvl_base_offset_reg >= 8)
			wrlvl_base_offset_reg = 0;

	} while ((wrlvl_base_offset_check != 0));

	set_dqs_parms(wrlvl_start, final_wrlvl_delay);
}
#endif
