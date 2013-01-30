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

static void prog_rdlvl_delay(u32 slice, u32 rdlvl_delay)
{
	u32 *rdlvl_delay_reg = &mpmc_p->reg81;
	u32 shift;

	rdlvl_delay_reg += (slice + 1) >> 1;
	shift = (slice & 0x1) ? 0 : 16;
	writel_field(rdlvl_delay << shift, 0xFFFF << shift, rdlvl_delay_reg);
}

u32 find_rdlvl_value(u32 *start, u32 *end)
{
	u32 edge_delta[2];
	int diff;
	u32 rdlvl_value;

	edge_delta[0] = end[0] - start[0];
	edge_delta[1] = end[1] - start[1];

	if ((edge_delta[0] == 0) && (edge_delta[1] == 0)) {
		if (start[0] != start[1])
			goto error;
		else
			rdlvl_value = start[0];
	} else if ((edge_delta[0] == 0) && (edge_delta[1] != 0)) {
		if ((start[0] == start[1]) || (start[0] == (start[1] + 1)))
			rdlvl_value = start[0];
		else
			goto error;
	} else if ((edge_delta[0] != 0) && (edge_delta[1] == 0)) {
		if ((start[1] == start[0]) || (start[1] == (start[0] + 1)))
			rdlvl_value = start[1];
		else
			goto error;
	} else {
		diff = start[0] - start[1];
		if (diff < 0)
			diff = start[1] - start[0];

		if ((start[1] == start[0]) && (edge_delta[0] > 2) && (edge_delta[1] > 2)) {
			if (start[0] > 6)
				rdlvl_value = start[0] + 1;
			else
				rdlvl_value = start[0] + 2;
		} else {

			if (start[1] == start[0]) {
				rdlvl_value = start[1] + 1;
			} else if (diff < 2) {
				if ((edge_delta[0] == 1) && (edge_delta[1] == 1)) {
					if (start[1] == (start[0] + 1))
						rdlvl_value = start[1];
					else if (start[0] == (start[1] + 1))
						rdlvl_value = start[0];
					else
						goto error;
				} else if (edge_delta[0] == edge_delta[1]) {
					if (start[1] == (start[0] + 1))
						rdlvl_value = start[1] + 1;
					else if (start[0] == (start[1] + 1))
						rdlvl_value = start[0] + 1;
					else
						goto error;
				} else if (edge_delta[0] > edge_delta[1]) {
					if (start[1] == (start[0] + 1))
						rdlvl_value = start[0] + 2;
					else if (start[0] == (start[1] + 1)) {
						if (edge_delta[1] == 1)
							rdlvl_value = start[1] + 1;
						else
							rdlvl_value = start[1] + 2;
					} else
						goto error;
				} else {
					if (start[0] == (start[1] + 1))
						rdlvl_value = start[1] + 2;
					else if (start[1] == (start[0] + 1)) {
						if (edge_delta[0] == 1)
							rdlvl_value = start[0] + 1;
						else
							rdlvl_value = start[0] + 2;
					} else
						goto error;
				}
			} else
				goto error;
		}
	}

	/* if allowed from actual read leveling data, adjust read phase to the default centered value 8 */
	if ((rdlvl_value == 9) && (start[0] < 9) && (start[1] < 9))
		return --rdlvl_value;
	else if ((rdlvl_value == 7) && (end[0] > 7) && (end[1] > 7))
		return ++rdlvl_value;
	else
		return rdlvl_value;

error:
	/* force default value */
	rdlvl_value = 8;
	return rdlvl_value;

	/*
	 * other possibility is to reset the system
	 * in case of a leveling error
	 */
	reset_cpu(0);
}

#if defined(CONFIG_DDR2)
void ddr2_lvl_read(void)
{
	u32 slice;
	int RDLVL_DELAY_VALUE[5] = {8, 8, 8, 8, 8};

	RDLVL_DELAY_VALUE[0] = RDLVL_DELAY_VALUE_0;
	RDLVL_DELAY_VALUE[1] = RDLVL_DELAY_VALUE_1;
	RDLVL_DELAY_VALUE[2] = RDLVL_DELAY_VALUE_2;
	RDLVL_DELAY_VALUE[3] = RDLVL_DELAY_VALUE_3;
	RDLVL_DELAY_VALUE[4] = RDLVL_DELAY_VALUE_4;

	for (slice = 0; slice < DATA_SLICE_MAX; slice++)
		prog_rdlvl_delay(slice, RDLVL_DELAY_VALUE[slice]);
}

#elif defined(CONFIG_DDR3)
void ddr3_lvl_read(void)
{
	u8 resp_array[DATA_SLICE_MAX][RDLVL_DELAY_VALS][2];
	u32 i, j, k;
	u32 count = 0;
	u32 *phy_ctrl_reg0 = &mpmc_p->reg124;
	u32 *phy_ctrl_reg2 = &mpmc_p->reg135;
	u32 *phy_ctrl_reg3 = &mpmc_p->reg140;
	u32 slice, rdlvl_delay, start_point_found, rdlvl_edge;
	u32 delay_vals;
	u32 start_point_0[2], end_point_0[2];

	writel_field(0xffff, 0xffff, &mpmc_p->reg182);
	writel_field(0x10 << 16, 0xff << 16, &mpmc_p->reg64);
	writel_field(0x12 << 8, 0xff << 8, &mpmc_p->reg59);

	for (i = 0; i < DATA_SLICE_MAX; i++)
		for (j = 0; j < RDLVL_DELAY_VALS; j++)
			for (k = 0; k < 2; k++)
				resp_array[i][j][k] = 0x0;

	for (slice = 0; slice < DATA_SLICE_MAX; slice++) {
		writel_field(0x21 << 14, 0x3F << 14, phy_ctrl_reg0 + slice);
		writel_field(0x21 << 8, 0x3F << 8, phy_ctrl_reg0 + slice);

		writel_field(0x1 << 24, 0x3F << 24, phy_ctrl_reg2 + slice);
		writel_field(0x1 << 24, 0x3F << 24, phy_ctrl_reg3 + slice);
	}

	set_swlvl_mode(READ_LVL);
	swlvl_start();
	wait_op_done();

	swlvl_load();
	wait_op_done();
	for (count = 0; count < 100; count++) {
		for (slice = 0; slice < DATA_SLICE_MAX; slice++) {
			for (rdlvl_edge = 0; rdlvl_edge <= 1; rdlvl_edge++) {
				writel_field(rdlvl_edge << 0, 1 << 0, &mpmc_p->reg7);
				for (delay_vals = 0; delay_vals < RDLVL_DELAY_VALS; delay_vals++) {
					prog_rdlvl_delay(slice, delay_vals + RDLVL_DELAY_INIT);
					swlvl_load();
					wait_op_done();

					resp_array[slice][delay_vals][rdlvl_edge] |= read_resp(slice);
				}
			}
		}
	}
	for (slice = 0; slice < DATA_SLICE_MAX; slice++) {
		for (rdlvl_edge = 0; rdlvl_edge <= 1; rdlvl_edge++) {
			start_point_found = 0;
			for (delay_vals = 0; delay_vals < RDLVL_DELAY_VALS; delay_vals++) {
				if ((resp_array[slice][delay_vals][rdlvl_edge] == 0) && !start_point_found) {
					start_point_0[rdlvl_edge] = delay_vals + RDLVL_DELAY_INIT;
					start_point_found = 1;
				}
				if ((resp_array[slice][delay_vals][rdlvl_edge] != 0) && start_point_found) {
					end_point_0[rdlvl_edge] = delay_vals + RDLVL_DELAY_INIT - 1;
					break;
				}
			}
		}

		rdlvl_delay = find_rdlvl_value(start_point_0, end_point_0);
		prog_rdlvl_delay(slice, rdlvl_delay);
		swlvl_load();
		wait_op_done();
	}
	swlvl_exit();
	wait_op_done();
}
#endif
