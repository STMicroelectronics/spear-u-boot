/*
 * (C) Copyright 2011
 * Pratyush Anand, ST Micoelectronics, pratyush.anand@st.com.
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

#ifndef __DW_OTG_H
#define __DW_OTG_H
/* temp def: will be removed TBD */
#undef APH_BOARD	1
#ifdef APG_BOARD
#include "types.h"
#define CONFIG_USBD_HS	1
#define CONFIG_DW_OTG	1
#endif

#include "usbdevice.h"
/* USBTTY definitions */
#define  EP0_MAX_PACKET_SIZE		64
#define  UDC_INT_ENDPOINT		1
#define  UDC_INT_PACKET_SIZE		64
#define  UDC_OUT_ENDPOINT		2
#define  UDC_BULK_PACKET_SIZE		512
#if defined(CONFIG_USBD_HS)
#define  UDC_BULK_HS_PACKET_SIZE	512
#endif
#define  UDC_IN_ENDPOINT		3
#define  UDC_OUT_PACKET_SIZE		64
#define  UDC_IN_PACKET_SIZE		64

/* UDC endpoint definitions */
#define  UDC_EP0			0
#define  UDC_EP1			1
#define  UDC_EP2			2
#define  UDC_EP3			3

#define CMD_SIZE	12
/* OTG Register Definitions */

/*
 * The application interfaces with the HS OTG core by reading from and
 * writing to the Control and Status Register (CSR) space through the
 * AHB Slave interface. These registers are 32 bits wide, and the
 * addresses are 32-bit-block aligned.
 * CSRs are classified as follows:
 * - Core Global Registers
 * - Device Mode Registers
 * - Device Global Registers
 * - Device Endpoint Specific Registers
 * - Host Mode Registers
 * - Host Global Registers
 * - Host Port CSRs
 * - Host Channel Specific Registers
 *
 * Only the Core Global registers can be accessed in both Device and
 * Host modes. When the HS OTG core is operating in one mode, either
 * Device or Host, the application must not access registers from the
 * other mode. When the core switches from one mode to another, the
 * registers in the new mode of operation must be reprogrammed as they
 * would be after a power-on reset.
 */

/*
 * DWC_otg Core registers. The core_global_regs structure defines the
 * size and relative field offsets for the Core Global registers.
 */
struct core_global_regs {
	/* OTG Control and Status Register.		Offset: 000h */
	u32 gotgctl;
	/* OTG Interrupt Register.			Offset: 004h */
	u32 gotgint;
	/* Core AHB Configuration Register.		Offset: 008h */
	u32 gahbcfg;

#define DWC_GLBINTRMASK				0x0001
#define DWC_DMAENABLE				0x0020
#define DWC_NPTXEMPTYLVL_EMPTY			0x0080
#define DWC_NPTXEMPTYLVL_HALFEMPTY		0x0000
#define DWC_PTXEMPTYLVL_EMPTY			0x0100
#define DWC_PTXEMPTYLVL_HALFEMPTY		0x0000

	/* Core USB Configuration Register.		Offset: 00Ch */
	u32 gusbcfg;
#define PHYIF_16BIT				(1 << 3)
#define SRPCAP					(1 << 8)
#define HNPCAP					(1 << 9)
#define TERM_SEL_DL_PULSE			(1 << 22)
#define USBTRDTIM_SHIFT				10
#define USBTRDTIMMSK				(0xF << USBTRDTIM_SHIFT)
	/* Core Reset Register.				Offset: 010h */
	u32 grstctl;
#define DWC_GRSTCTL_TXFNUM_ALL			0x10
#define CSFTRST					(1 << 0)
#define INTKNQFLSH				(1 << 3)
#define RXFFLSH					(1 << 4)
#define	TXFFLSH					(1 << 5)
#define TXFNUM_SHIFT				6
#define TXFNUM					(0x1F << TXFNUM_SHIFT)
#define AHBIDLE					((u32)1 << 31)
	/* Core Interrupt Register.			Offset: 014h */
	u32 gintsts;
#define RXSTSQLVL				(1 << 4)
#define NPTXFEMPTY				(1 << 5)
#define GOUTNAKEFF				(1 << 7)
#define USBRESET				(1 << 12)
#define ENUMDONE				(1 << 13)
#define INEPINTR				(1 << 18)
#define OUTEPINTR				(1 << 19)
	/* Core Interrupt Mask Register.		Offset: 018h */
	u32 gintmsk;
	/*
	 * Receive Status Queue Read Register
	 * (Read Only)					Offset: 01Ch
	 */
	u32 grxstsr;
	/*
	 * Receive Status Queue Read & POP Register
	 * (Read Only)					Offset: 020h
	 */
	u32 grxstsp;
#define DWC_STS_DATA_UPDT		0x2 /* OUT Data Packet */
#define DWC_STS_XFER_COMP		0x3 /* OUT Data Transfer Complete */
#define DWC_DSTS_GOUT_NAK		0x1 /* Global OUT NAK */
#define DWC_DSTS_SETUP_COMP		0x4 /* Setup Phase Complete */
#define DWC_DSTS_SETUP_UPDT		0x6 /* SETUP Packet */
#define EPNUM_SHIFT					0
#define EPNUMMSK					(0xF << EPNUM_SHIFT)
#define BCNT_SHIFT					4
#define BCNTMSK						(0x7FF << BCNT_SHIFT)
#define PKTSTS_SHIFT					17
#define PKTSTSMSK					(0xF << PKTSTS_SHIFT)
	/* Receive FIFO Size Register.			Offset: 024h */
	u32 grxfsiz;
#define dwc_param_dev_rx_fifo_size_default		1064
	/* Non Periodic Transmit FIFO Size Register.	Offset: 028h */
	u32 gnptxfsiz;
#define dwc_param_dev_nperio_tx_fifo_size_default	1024
	/*
	 * Non Periodic Transmit FIFO/Queue Status Register
	 * (Read Only).					Offset: 02Ch
	 */
	u32 gnptxsts;
#define NPTXQSPCAVAIL_SHIFT			16
#define NPTXQSPCAVAILMSK			(0xFF << NPTXQSPCAVAIL_SHIFT)
#define NPTXFSPCAVAIL_SHIFT			0
#define NPTXFSPCAVAILMSK			(0xFFFF << NPTXFSPCAVAIL_SHIFT)
	/* I2C Access Register.				Offset: 030h */
	u32 gi2cctl;
	/* PHY Vendor Control Register.			Offset: 034h */
	u32 gpvndctl;
	/* General Purpose Input/Output Register.	Offset: 038h */
	u32 ggpio;
	/* User ID Register.				Offset: 03Ch */
	u32 guid;
	/* Synopsys ID Register (Read Only).		Offset: 040h */
	u32 gsnpsid;
	/* User HW Config1 Register (Read Only).	Offset: 044h */
	u32 ghwcfg1;
	/* User HW Config2 Register (Read Only).	Offset: 048h */

	u32 ghwcfg2;
#define DWC_SLAVE_ONLY_ARCH			0
#define DWC_EXT_DMA_ARCH			1
#define DWC_INT_DMA_ARCH			2

#define DWC_MODE_HNP_SRP_CAPABLE		0
#define DWC_MODE_SRP_ONLY_CAPABLE		1
#define DWC_MODE_NO_HNP_SRP_CAPABLE		2
#define DWC_MODE_SRP_CAPABLE_DEVICE		3
#define DWC_MODE_NO_SRP_CAPABLE_DEVICE		4
#define DWC_MODE_SRP_CAPABLE_HOST		5
#define DWC_MODE_NO_SRP_CAPABLE_HOST		6
#define DYNAMIC_FIFO				(1 << 19)
#define	NUM_DEV_EP_SHIFT	10
#define	NUM_DEV_EP	(0xF << NUM_DEV_EP_SHIFT)
#define HSPHYTYPE_SHIFT				6
#define HSPHYTYPEMSK				(3 << HSPHYTYPE_SHIFT)
#define DWC_HWCFG2_HS_PHY_TYPE_NOT_SUPPORTED		0
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI			1
#define DWC_HWCFG2_HS_PHY_TYPE_ULPI			2
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI_ULPI		3
#define TKNQDEPTH_SHIFT					26
#define TKNQDEPTHMSK				(0x1F << TKNQDEPTH_SHIFT)

	/* User HW Config3 Register (Read Only).	Offset: 04Ch */
	u32 ghwcfg3;
#define	DFIFO_DEPTH_SHIFT	16
#define DFIFO_DEPTH	((u32)0xFFFF << DFIFO_DEPTH_SHIFT)
	/* User HW Config4 Register (Read Only).	Offset: 050h */
	u32 ghwcfg4;
#define NUM_DEV_PERIO_IN_EP_SHIFT	0
#define NUM_DEV_PERIO_IN_EP (0xF << NUM_DEV_PERIO_IN_EP_SHIFT)
#define DED_FIFO_EN	(1 << 25)
#define NUM_IN_EPS_SHIFT	26
#define NUM_IN_EPS	(0xF << NUM_IN_EPS_SHIFT)
#define UTMI_PHY_DATA_WIDTH_SHIFT	14
#define UTMI_PHY_DATA_WIDTH	(0x3 << UTMI_PHY_DATA_WIDTH_SHIFT)
	/* Reserved					Offset: 054h-0FFh */
	u32 reserved[43];
	/* Host Periodic Transmit FIFO Size Register.	Offset: 100h */
	u32 hptxfsiz;

	/*
	 * Device Periodic Transmit FIFO#n Register, if dedicated fifos are
	 * disabled. Otherwise Device Transmit FIFO#n Register.
	 *
	 * Offset: 104h + (FIFO_Number-1)*04h, 1 <= FIFO Number <= 15 (1<=n<=15)
	 */
	u32 dptxfsiz_dieptxf[15];
#define dwc_param_dev_tx_fifo_size_default	256
#define dwc_param_dev_perio_tx_fifo_size_default	256
};

/*
 * Device Global Registers. Offsets 800h-BFFh
 *
 * The following structures define the size and relative field offsets for the
 * Device Mode Registers.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_global_regs {		/* CONFIG_DWC_OTG_REG_LE */
	/* Device Configuration Register.			Offset: 800h */
	u32 dcfg;
#define DWC_DCFG_FRAME_INTERVAL_80		0
#define DWC_DCFG_FRAME_INTERVAL_85		1
#define DWC_DCFG_FRAME_INTERVAL_90		2
#define DWC_DCFG_FRAME_INTERVAL_95		3
#define DWC_DCFG_FRAME_INTERVAL_MASK		3
#define	PERFRINT_SHIFT				11
#define DEVSPDMSK				(0x3 << 0)
#define DEVADDR_SHIFT				4
#define DEVADDRMSK				(0x7F << DEVADDR_SHIFT)
#define NZSTSOUTHSHK				(1 << 2)
	/* Device Control Register.				Offset: 804h */
	u32 dctl;
#define RMTWKUPSIG	(1 << 0)
#define SFTDISCON	(1 << 1)
#define CGNPINNAK	(1 << 7)
	/* Device Status Register (Read Only).			Offset: 808h */
	u32 dsts;
#define ENUMSPD_SHIFT				1
#define ENUMSPDMSK				(3 << ENUMSPD_SHIFT)
#define DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ		0
#define DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ		1
#define DWC_DSTS_ENUMSPD_LS_PHY_6MHZ			2
#define DWC_DSTS_ENUMSPD_FS_PHY_48MHZ			3
	/* Reserved.						Offset: 80Ch */
	u32 unused;
	/* Device IN Endpoint Common Interrupt Mask Register.	Offset: 810h */
	u32 diepmsk;
#define TIMEOUTMSK				(1 << 3)
#define INTKNTXFEMP				(1 << 4)
#define INTKNEPMISMSK				(1 << 5)
#define INEPNAKEFFMSK				(1 << 6)
#define TXFIFOUNDRN				(1 << 8)
	/* Device OUT Endpoint Common Interrupt Mask Register.	Offset: 814h */
	u32 doepmsk;
#define XFERCOMPLMSK				(1 << 0)
#define EPDISABLEDMSK				(1 << 1)
#define AHBERRMSK				(1 << 2)
#define SETUPMSK				(1 << 3)
#define INTKNTXFEMPMSK				(1 << 4)
	/* Device All Endpoints Interrupt Register.		Offset: 818h */
	u32 daint;
	/* Device All Endpoints Interrupt Mask Register.	Offset: 81Ch */
	u32 daintmsk;
#define DAINTMASK_IN_SHIFT	0
#define DAINTMASK_OUT_SHIFT	16
	/* Device IN Token Queue Read Register-1 (Read Only).	Offset: 820h */
	u32 dtknqr1;
#define EPTK0_5_SHIFT				8
#define EPTK0_5MSK				((u32)0xFFFFFF << EPTK0_5_SHIFT)
#define INTKNWPTR_SHIFT				0
#define INTKNWPTRMSK				((u32)0x1F << INTKNWPTR_SHIFT)
	/* Device IN Token Queue Read Register-2 (Read Only).	Offset: 824h */
	u32 dtknqr2;
	/* Device VBUS discharge Register.			Offset: 828h */
	u32 dvbusdis;
	/* Device VBUS Pulse Register.				Offset: 82Ch */
	u32 dvbuspulse;
	/* Device IN Token Queue Read Register-3 (Read Only).	Offset: 830h */
	u32 dtknqr3_dthrctl;
	/* Device IN Token Queue Read Register-4 (Read Only).	Offset: 834h */
	u32 dtknqr4_fifoemptymsk;
};
/*
 * Device Logical IN Endpoint-Specific Registers. Offsets 900h-AFCh
 *
 * There will be one set of endpoint registers per logical endpoint implemented.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_in_ep_regs {
	/*
	 * Device IN Endpoint Control Register.
	 * Offset:900h + (ep_num * 20h) + 00h
	 */
	u32 diepctl;
#define	EPENA				((u32)1 << 31)
#define EPDIS				(1 << 30)
#define	SNAK				(1 << 27)
#define	CNAK				(1 << 26)
#define	SSTALL				(1 << 21)
#define MPS_SHIFT			0
#define MPSMSK0				(3 << MPS_SHIFT)
#define DWC_DEP0CTL_MPS_64			0
#define DWC_DEP0CTL_MPS_32			1
#define DWC_DEP0CTL_MPS_16			2
#define DWC_DEP0CTL_MPS_8			3
#define DIEPCTL_MPSMSK				(0x7FF << MPS_SHIFT)
	/* Reserved. Offset:900h + (ep_num * 20h) + 04h */
	u32 reserved04;
	/*
	 * Device IN Endpoint Interrupt Register.
	 * Offset:900h + (ep_num * 20h) + 08h
	 */
	u32 diepint;
#define TXFEMP				(1 << 7)
#define INTKNTXFEMP			(1 << 4)
#define XFERCOMPL			(1 << 0)
	/* Reserved. Offset:900h + (ep_num * 20h) + 0Ch */
	u32 reserved0C;
	/* Device IN Endpoint Transfer Size Register.
	 * Offset:900h + (ep_num * 20h) + 10h
	 */
	u32 dieptsiz;
#define PKTCNT_SHIFT	19
	/*
	 * Device IN Endpoint DMA Address Register.
	 * Offset:900h + (ep_num * 20h) + 14h
	 */
	u32 diepdma;
	/* Reserved.
	 * Offset:900h + (ep_num * 20h) + 18h - 900h + (ep_num * 20h) + 1Ch
	 */
	u32 dtxfsts;
	/*
	 * Reserved.
	 * Offset:900h + (ep_num * 20h) + 1Ch - 900h + (ep_num * 20h) + 1Ch
	 */
	u32 reserved18;
};

/*
 * Device Logical OUT Endpoint-Specific Registers. Offsets: B00h-CFCh
 *
 * There will be one set of endpoint registers per logical endpoint implemented.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_out_ep_regs {
	/*
	 * Device OUT Endpoint Control Register.
	 * Offset:B00h + (ep_num * 20h) + 00h
	 */
	u32 doepctl;
#define DOEPCTL_MPSMSK		0x7FF
#define USBACTEP		(1 << 15)
#define EPTYPE_SHIFT		18
#define EPTYPEMSK		(0x3 << EPTYPE_SHIFT)
#define EPTYPE_BULK		0x2
#define EPTYPE_INT		0x3
#define DATA0PID		(1 << 28)
#define DATA1PID		(1 << 29)
#define DPIDMSK			(1 << 16)
	/*
	 * Device OUT Endpoint Frame number Register.
	 * Offset: B00h + (ep_num * 20h) + 04h
	 */
	u32 doepfn;
	/*
	 * Device OUT Endpoint Interrupt Register.
	 * Offset:B00h + (ep_num * 20h) + 08h
	 */
	u32 doepint;
#define XFERCOMPL			(1 << 0)
#define EPDISBLD			(1 << 1)
#define AHBERR				(1 << 2)
#define SETUP				(1 << 3)
	/* Reserved. Offset:B00h + (ep_num * 20h) + 0Ch */
	u32 reserved0C;
	/*
	 * Device OUT Endpoint Transfer Size Register.
	 * Offset: B00h + (ep_num * 20h) + 10h
	 */
	u32 doeptsiz;
#define XFERSIZE_SHIFT			0
#define XFERSIZEMSK			0x3F
#define PKTCNT_SHIFT			19
#define PKTCNT				(3 << 19)
#define SUPCNT_SHIFT			29
#define SUPCNTMSK			(3 << SUPCNT_SHIFT)
	/*
	 * Device OUT Endpoint DMA Address Register.
	 * Offset:B00h + (ep_num * 20h) + 14h
	 */
	u32 doepdma;
	/*
	 * Reserved.
	 * Offset:B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch
	 */
	u32 unused[2];
};
#define MAX_EPS_CHANNELS 4

/*
 * The dwc_ep structure represents the state of a single endpoint when acting in
 * device mode. It contains the data items needed for an endpoint to be
 * activated and transfer packets.
 */
struct dwc_ep {
	/* EP number used for register address lookup */
	u8	 num;
	/* EP direction 0 = OUT */
#if 0
	u8 is_in;
#endif
	/* pointer to the transfer buffer */
	u8 *xfer_buff;
	/* Number of bytes to transfer */
	u32 xfer_len;
};

/*
 * DWC_otg PCD Structure.
 * This structure encapsulates the data for the dwc_otg PCD.
 */
struct dwc_pcd {
#if 0
	/* USB gadget */
	/* Current configuration */
	u8	configuration;
	/* Current interface */
	u8	interface;
	/* Current alternate settinng */
	u8	alternate;
	/* Current Address */
	u16 address;
	/* device state */
/*	usb_device_state_t device_state; */	/* current USB Device state */
	/*
	 * SETUP packet for EP0. This structure is allocated as a DMA buffer on
	 * PCD initialization with enough space for up to 3 setup packets.
	 */
#endif
	struct usb_device_request *req;
	/* Array of EPs. */
	struct dwc_ep ep0;
	/* Array of IN EPs. */
	struct dwc_ep in_ep[MAX_EPS_CHANNELS - 1];
	/* Array of OUT EPs. */
	struct dwc_ep out_ep[MAX_EPS_CHANNELS - 1];
};

/*
 * The device_if structure contains information needed to manage the DWC_otg
 * controller acting in device mode. It represents the programming view of the
 * device-specific aspects of the controller.
 */
struct device_if {
	struct core_global_regs *core_global_regs;
	/* Common configuration information */

	/* Device Global Registers starting at offset 800h */
	struct device_global_regs *dev_global_regs;
#define DWC_DEV_GLOBAL_REG_OFFSET		0x800

	/* Device Logical IN Endpoint-Specific Registers 900h-AFCh */
	struct device_in_ep_regs *in_ep_regs[MAX_EPS_CHANNELS];
#define DWC_DEV_IN_EP_REG_OFFSET		0x900
#define DWC_EP_REG_OFFSET			0x20

	/* Device Logical OUT Endpoint-Specific Registers B00h-CFCh */
	struct device_out_ep_regs *out_ep_regs[MAX_EPS_CHANNELS];
#define DWC_DEV_OUT_EP_REG_OFFSET		0xB00

	/* Push/pop addresses for endpoints or host channels.*/
	u32 *data_fifo[MAX_EPS_CHANNELS];
#define DWC_OTG_DATA_FIFO_OFFSET		0x1000
#define DWC_OTG_DATA_FIFO_SIZE			0x1000

	struct dwc_pcd pcd;
	int speed;
};


/* Function declarations */

void phy_init(void);
void udc_irq(void);

void udc_set_nak(int epid);
void udc_unset_nak(int epid);
int udc_endpoint_write(struct usb_endpoint_instance *endpoint);
int udc_init(void);
/* void udc_enable(struct usb_device_instance *device);*/
void udc_disable(void);
void udc_connect(void);
void udc_disconnect(void);
void udc_startup_events(struct usb_device_instance *device);
void udc_setup_ep(struct usb_device_instance *device, unsigned int ep,
		  struct usb_endpoint_instance *endpoint);
void udc_set_configuration_controller(u32);
void udc_set_address_controller(u32);

#endif /* __DW_UDC_H */
