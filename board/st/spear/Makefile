#
# (C) Copyright 2000-2004
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB	= $(obj)lib$(BOARD).o

SOBJS-y	:=
COBJS-y	:=

ifdef CONFIG_SPL_BUILD
COBJS-$(CONFIG_DDR_MT47H64M16) += spear600_mt47h64m16_3_333_cl5_psync.o
COBJS-$(CONFIG_DDR_MT47H32M16) += spear600_mt47h32m16_333_cl5_psync.o
COBJS-$(CONFIG_DDR_MT47H32M16) += spear600_mt47h32m16_37e_166_cl4_sync.o
COBJS-$(CONFIG_DDR_MT47H128M8) += spear600_mt47h128m8_3_266_cl5_async.o
else
SOBJS-y	+= spear_lowlevel_init.o
COBJS-y	+= spear_common.o
COBJS-$(CONFIG_MACH_SPEAR300EVB) += spear300evb.o
COBJS-$(CONFIG_MACH_SPEAR310EVB) += spear310evb.o
COBJS-$(CONFIG_MACH_SPEAR320EVB) += spear320plc.o
COBJS-$(CONFIG_MACH_SPEAR320HMI) += spear320hmi.o
COBJS-$(CONFIG_MACH_SPEAR600EVB) += spear600evb.o
endif

COBJS	:= $(sort $(COBJS-y))
SOBJS	:= $(sort $(SOBJS-y))
SRCS	:= $(SOBJS:.o=.S) $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))
SOBJS	:= $(addprefix $(obj),$(SOBJS))

$(LIB):	$(obj).depend $(OBJS) $(SOBJS)
	$(call cmd_link_o_target, $(OBJS) $(SOBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################