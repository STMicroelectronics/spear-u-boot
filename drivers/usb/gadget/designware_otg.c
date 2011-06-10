/*
 * Based on drivers/usb/gadget/designware_otg.c
 * Synopsys DW OTG Device bus interface driver
 *
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
/* temp def: will be removed TBD*/
#undef APG_BOARD	1

#ifndef APG_BOARD
#include <common.h>
#include <asm/io.h>
#include <usbdevice.h>
#include "ep0.h"
#include <usb/designware_otg.h>
#include <asm/arch/hardware.h>
#else
#include "types.h"
#include <designware_otg.h>
#include <usbdevice.h>
#endif

#define UDC_INIT_MDELAY		80	/* Device settle delay */

/* Some kind of debugging output... */
#ifndef DEBUG_DWUSBTTY
#define UDCDBG(str)
#define UDCDBGA(fmt, args...)
#else
#define UDCDBG(str) serial_printf(str "\n")
#define UDCDBGA(fmt, args...) serial_printf(fmt "\n", ##args)
#endif

static struct urb *ep0_urb;
static struct usb_device_instance *udc_device;

static struct device_if	device_if_mem;
static struct device_if	*dev_if = &device_if_mem;
#if defined(CONFIG_USBD_HS)
#define CONFIG_USBD_SERIAL_BULK_PKTSIZE	UDC_BULK_HS_PACKET_SIZE
#endif

static struct usb_endpoint_instance *dw_find_ep(int ep)
{
	int i;

	for (i = 0; i < udc_device->bus->max_endpoints; i++) {
		if ((udc_device->bus->endpoint_array[i].endpoint_address &
					USB_ENDPOINT_NUMBER_MASK) == ep)
			return &udc_device->bus->endpoint_array[i];
	}
	return NULL;
}

/*
 * This function reads a packet from the Rx FIFO into the destination buffer.
 * To read SETUP data use dwc_otg_read_setup_packet.
 */
void dwc_otg_read_packet(struct dwc_ep *ep, u16 _bytes)
{
	u32 i;
	int word_count = (_bytes + 3) / 4;
	u32 *fifo = dev_if->data_fifo[0];
	u32 *data_buff = (u32 *) ep->xfer_buff;
	u32 unaligned;
	/*
	 * This requires reading data from the FIFO into a u32 temp buffer,
	 * then moving it into the data buffer.
	 */
	if ((_bytes < 4) && (_bytes > 0)) {
		unaligned = readl(fifo);
		memcpy(data_buff, &unaligned, _bytes);
	} else {
		for (i = 0; i < word_count; i++, data_buff++)
			*data_buff = readl(fifo);
	}
}

/* Handle RX transaction on non-ISO endpoint. */
static void dw_udc_epn_rx(struct dwc_ep *ep, int bcnt)
{
	struct urb *urb;
	struct usb_endpoint_instance *endpoint = dw_find_ep(ep->num);

	if (endpoint) {
		urb = endpoint->rcv_urb;

		if (urb) {
			ep->xfer_buff = urb->buffer + urb->actual_length;
			dwc_otg_read_packet(ep, bcnt);
			usbd_rcv_complete(endpoint, bcnt, 0);
		}
	}
}

/*
 * This function writes a packet into the Tx FIFO associated with the EP.
 * The buffer is padded to DWORD on a per packet basis in
 * slave/dma mode if the MPS is not DWORD aligned. The last packet, if
 * short, is also padded to a multiple of DWORD.
 *
 * ep->xfer_buff always starts DWORD aligned in memory and is a
 * multiple of DWORD in length
 *
 * ep->xfer_len can be any number of bytes
 *
 * FIFO access is DWORD
 */
static void dwc_otg_ep_write_packet(struct dwc_ep *ep)
{
	u32 i;
	u32 dword_count;
	u32 *fifo;
	u32 *data_buff = (u32 *) ep->xfer_buff;
	u32 temp, unaligned;
	struct device_in_ep_regs *in_ep_regs = dev_if->in_ep_regs[ep->num];
	struct core_global_regs *core_global_regs = dev_if->core_global_regs;

	/*
	 * Find the DWORD length, padded by extra bytes as neccessary if MPS
	 * is not a multiple of DWORD
	 */
	dword_count = (ep->xfer_len + 3) / 4;
	fifo = dev_if->data_fifo[ep->num];

	/* program pkt count */
	temp = ep->xfer_len;
	temp |= (1 << PKTCNT_SHIFT);
	writel(temp, &in_ep_regs->dieptsiz);

	/* enable EP*/
	temp = readl(&in_ep_regs->diepctl);
	temp |= (EPENA | CNAK);
	writel(temp, &in_ep_regs->diepctl);

	/* clear TX Fifo Empty intr*/
	writel(NPTXFEMPTY, &core_global_regs->gintsts);

	temp = readl(&core_global_regs->gintmsk);
	temp |= NPTXFEMPTY;
	writel(temp, &core_global_regs->gintmsk);

	while (!(readl(&core_global_regs->gintsts) & NPTXFEMPTY))
		;

	/* write to fifo */
	if ((ep->xfer_len < 4) && (ep->xfer_len > 0)) {
		memcpy(&unaligned, data_buff, ep->xfer_len);
		*fifo = unaligned;
	} else {
		for (i = 0; i < dword_count; i++, data_buff++)
			*fifo = *data_buff;
	}

	writel(NPTXFEMPTY, &core_global_regs->gintsts);

	/* check for transfer completion*/
	while (!(readl(&in_ep_regs->diepint) & XFERCOMPL))
		;

	writel(XFERCOMPL, &in_ep_regs->diepint);

	temp = readl(&core_global_regs->gintmsk);
	temp &= ~NPTXFEMPTY;
	writel(temp, &core_global_regs->gintmsk);
}

/* Handle TX transaction on non-ISO endpoint. */
static void dw_udc_epn_tx(struct dwc_ep *ep)
{
	struct usb_endpoint_instance *endpoint = dw_find_ep(ep->num);
	struct urb *urb = endpoint->tx_urb;
	int align;

	if (!endpoint)
		return;

	/*
	 * We need to transmit a terminating zero-length packet now if
	 * we have sent all of the data in this URB and the transfer
	 * size was an exact multiple of the packet size.
	 */
	if (urb && (endpoint->last == endpoint->tx_packetSize) &&
			(urb->actual_length - endpoint->sent -
			 endpoint->last == 0)) {
		/* handle zero length packet here */
		ep->xfer_len = 0;
		dwc_otg_ep_write_packet(ep);
	}

	if (urb && urb->actual_length) {
		/* retire the data that was just sent */
		usbd_tx_complete(endpoint);
		/*
		 * Check to see if we have more data ready to transmit
		 * now.
		 */
		if (urb && urb->actual_length) {
			/* write data to FIFO */
			ep->xfer_len = MIN(urb->actual_length - endpoint->sent,
					endpoint->tx_packetSize);

			if (ep->xfer_len) {
				ep->xfer_buff = urb->buffer + endpoint->sent;

				/*
				 * This ensures that USBD packet fifo is
				 * accessed through word aligned pointer or
				 * through non word aligned pointer but only
				 * with a max length to make the next packet
				 * word aligned
				 */

				align = ((ulong)ep->xfer_buff % sizeof(int));
				if (align)
					ep->xfer_len = MIN(ep->xfer_len,
							sizeof(int) - align);

				dwc_otg_ep_write_packet(ep);
			}
			endpoint->last = ep->xfer_len;

		} else if (urb && (urb->actual_length == 0)) {
			/* udc_set_nak(ep); */
		}
	}
}

/* This function returns pointer to out ep struct with number num */
struct dwc_ep *get_out_ep(u32 num)
{
	u32 i;
	int num_out_eps = MAX_EPS_CHANNELS;
	struct dwc_pcd *pcd = &dev_if->pcd;

	if (num == 0) {
		return &pcd->ep0;
	} else {
		for (i = 0; i < num_out_eps; ++i) {
			if (pcd->out_ep[i].num == num)
				return &pcd->out_ep[i];
		}
	}
	return 0;
}

/* This function returns pointer to in ep struct with number num */
struct dwc_ep *get_in_ep(u32 num)
{
	u32 i;
	int num_out_eps = MAX_EPS_CHANNELS;
	struct dwc_pcd *pcd = &dev_if->pcd;

	if (num == 0) {
		return &pcd->ep0;
	} else {
		for (i = 0; i < num_out_eps; ++i) {
			if (pcd->in_ep[i].num == num)
				return &pcd->in_ep[i];
		}
	}
	return 0;
}

/*
 * This function reads the 8 bytes of the setup packet from the Rx FIFO into the
 * destination buffer. It is called from the Rx Status Queue Level (RxStsQLvl)
 * interrupt routine when a SETUP packet has been received in Slave mode.
 */
static void dwc_otg_read_setup_packet(u32 *dest)
{
	dest[0] = readl(dev_if->data_fifo[0]);
	dest[1] = readl(dev_if->data_fifo[0]);
}

/*
 * This function handles the Rx Status Queue Level Interrupt, which
 * indicates that there is a least one packet in the Rx FIFO. The
 * packets are moved from the FIFO to memory, where they will be
 * processed when the Endpoint Interrupt Register indicates Transfer
 * Complete or SETUP Phase Done.
 *
 * Repeat the following until the Rx Status Queue is empty:
 *	 -# Read the Receive Status Pop Register (GRXSTSP) to get Packet
 *		info
 *	 -# If Receive FIFO is empty then skip to step Clear the interrupt
 *		and exit
 *	 -# If SETUP Packet call dwc_otg_read_setup_packet to copy the
 *		SETUP data to the buffer
 *	 -# If OUT Data Packet call dwc_otg_read_packet to copy the data
 *		to the destination buffer
 */
static int dwc_otg_pcd_handle_rx_status_q_level_intr(void)
{
	struct core_global_regs *global_regs = dev_if->core_global_regs;
	struct dwc_pcd *pcd = &dev_if->pcd;
	u32	gintmsk;
	u32	status;
	struct dwc_ep *ep;
	u32	bcnt;

	/* Disable the Rx Status Queue Level interrupt */
	gintmsk = readl(&global_regs->gintmsk);
	gintmsk &= ~RXSTSQLVL;
	writel(gintmsk, &global_regs->gintmsk);

	/* Get the Status from the top of the FIFO */
	status = readl(&global_regs->grxstsp);
	/* Get pointer to EP structure */
	ep = get_out_ep((status & EPNUMMSK) >> EPNUM_SHIFT);
	bcnt = (status & BCNTMSK) >> BCNT_SHIFT;

	switch ((status & PKTSTSMSK) >> PKTSTS_SHIFT) {
	case DWC_DSTS_GOUT_NAK:
		break;
	case DWC_STS_DATA_UPDT:
		if (bcnt)
			dw_udc_epn_rx(ep, bcnt);
		break;
	case DWC_STS_XFER_COMP:
		break;
	case DWC_DSTS_SETUP_COMP:
		break;
	case DWC_DSTS_SETUP_UPDT:
		dwc_otg_read_setup_packet((u32 *)pcd->req);
		break;
	default:
		break;
	}

	/* Enable the Rx Status Queue Level interrupt */
	gintmsk = readl(&global_regs->gintmsk);
	gintmsk |= RXSTSQLVL;
	writel(gintmsk, &global_regs->gintmsk);

	/* Clear interrupt */
	writel(RXSTSQLVL, &global_regs->gintsts);

	return 1;
}

/*
 * This function starts the Zero-Length Packet for the IN status phase
 * of a 2 stage control transfer.
 */
static void do_setup_in_status_phase(struct device_if *dev_if)
{
	struct device_out_ep_regs *out_regs =
		dev_if->out_ep_regs[0];
	u32 doepctl, doeptsiz;
	doeptsiz = 0;
	doeptsiz |= (1 << PKTCNT_SHIFT);
	writel(doeptsiz, &out_regs->doeptsiz);
	doepctl = readl(&out_regs->doepctl);
	doepctl |= (CNAK | EPENA);
	writel(doepctl, &out_regs->doepctl);
}

static void udc_set_stall(int epid, int dir)
{
	if (dir)
		writel(readl(&dev_if->in_ep_regs[epid]->diepctl) | SSTALL,
			&dev_if->in_ep_regs[epid]->diepctl);
	else
		writel(readl(&dev_if->out_ep_regs[epid]->doepctl) | SSTALL,
			&dev_if->out_ep_regs[epid]->doepctl);
}
/*
 * This function handles EP0 Control transfers.
 *
 * The state of the control tranfers are tracked in ep0state
 *
 * A flag set indicates that it is not the first packet, so do not
 * process setup data now. it has alreday been processed, just send the
 * next data packet
 */
void handle_ep0(int in_flag)
{
	struct dwc_pcd *pcd = &dev_if->pcd;
	struct dwc_ep *ep0 = &pcd->ep0;
	struct usb_device_request *ctrl = pcd->req;

	/* handle inepint, only when more than 64 bytes to transfer*/
	if (in_flag & !ep0_urb->actual_length)
		return;

	if (!ep0_urb->actual_length) {
		if (ep0_recv_setup(ep0_urb)) {
			udc_set_stall(0, ctrl->bmRequestType & USB_DIR_IN);
			return;
		}
		ep0->xfer_buff = (u8 *)ep0_urb->buffer;
	} else
		ep0->xfer_buff += EP0_MAX_PACKET_SIZE;

	if (ep0_urb->actual_length <= EP0_MAX_PACKET_SIZE) {
		ep0->xfer_len = ep0_urb->actual_length;
		ep0_urb->actual_length = 0;
	} else {
		ep0->xfer_len = EP0_MAX_PACKET_SIZE;
		ep0_urb->actual_length -= EP0_MAX_PACKET_SIZE;
	}

	if (ctrl->bmRequestType & USB_DIR_IN) {
		dwc_otg_ep_write_packet(ep0);
		if (!ep0_urb->actual_length)
			do_setup_in_status_phase(dev_if);
	} else {
		if (!ctrl->wLength)
			dwc_otg_ep_write_packet(ep0);
		else
			udc_set_stall(0, ctrl->bmRequestType & USB_DIR_OUT);
	}
}

/*
 * This function reads the Device All Endpoints Interrupt register and
 * returns the OUT endpoint interrupt bits.
 */
static u32 dwc_otg_read_dev_all_out_ep_intr(void)
{
	u32 v;

	v = readl(&dev_if->dev_global_regs->daint) &
		readl(&dev_if->dev_global_regs->daintmsk);
	return (v & 0xffff0000) >> 16;
}

/*
 * This function reads the Device All Endpoints Interrupt register and
 * returns the IN endpoint interrupt bits.
 */
static u32 dwc_otg_read_dev_all_in_ep_intr(void)
{
	u32 v;

	v = readl(&dev_if->dev_global_regs->daint) &
		readl(&dev_if->dev_global_regs->daintmsk);
	return v & 0xffff;
}

/* This function returns the Device OUT EP Interrupt register */
static u32 dwc_otg_read_doep_intr(struct dwc_ep *ep)
{
	u32 v;

	v = readl(&dev_if->out_ep_regs[ep->num]->doepint) &
		readl(&dev_if->dev_global_regs->doepmsk);
	return v;
}

/*This function returns the Device IN EP Interrupt register */
static u32 dwc_otg_read_diep_intr(struct dwc_ep *ep)
{
	u32 v;

	v = readl(&dev_if->in_ep_regs[ep->num]->diepint) &
		readl(&dev_if->dev_global_regs->diepmsk);
	return v;
}

/*
 * This function configures EPO to receive SETUP packets.
 *
 * Program the following fields in the endpoint specific registers for Control
 * OUT EP 0, in order to receive a setup packet:
 *
 * - DOEPTSIZ0.Packet Count = 3 (To receive up to 3 back to back setup packets)
 *
 * - DOEPTSIZE0.Transfer Size = 24 Bytes (To receive up to 3 back to back setup
 * packets)
 *
 * In DMA mode, DOEPDMA0 Register with a memory address to store any setup
 * packets received
 */
static void ep0_out_start(void)
{
	u32 temp;

	/* program transfer size*/
	temp = 8 * 3;
	/* program packet count*/
	temp |= PKTCNT;
	/* program setup packet count */
	temp |= (3 << SUPCNT_SHIFT);
	writel(temp, &dev_if->out_ep_regs[0]->doeptsiz);
}

/* should be called after set address is received */
void udc_set_address_controller(u32 address)
{
	u32 dcfg;

	dcfg = readl(&dev_if->dev_global_regs->dcfg);
	dcfg &= ~DEVADDRMSK;
	dcfg |= address << DEVADDR_SHIFT;
	writel(dcfg, &dev_if->dev_global_regs->dcfg);

	usbd_device_event_irq(udc_device, DEVICE_ADDRESS_ASSIGNED, 0);
}

/* should be called after set configuration is received */
static void dwc_otg_bulk_out_activate(void)
{
	struct device_out_ep_regs *out_regs =
		dev_if->out_ep_regs[UDC_OUT_ENDPOINT];
	struct device_global_regs *dev_global_regs
		= dev_if->dev_global_regs;
	u32 doepctl, doeptsiz, daint;

	daint = readl(&dev_global_regs->daintmsk);
	daint |= 1 << (UDC_OUT_ENDPOINT + DAINTMASK_OUT_SHIFT);
	writel(daint, &dev_global_regs->daintmsk);
	doeptsiz = CONFIG_USBD_SERIAL_BULK_PKTSIZE;
	doeptsiz |= (1 << PKTCNT_SHIFT);
	writel(doeptsiz, &out_regs->doeptsiz);
	doepctl = readl(&out_regs->doepctl);
	doepctl &= ~DOEPCTL_MPSMSK;
	doepctl &= ~EPTYPEMSK;
	doepctl |= (CONFIG_USBD_SERIAL_BULK_PKTSIZE |
			CNAK | EPENA | USBACTEP | DATA0PID
			| (EPTYPE_BULK << EPTYPE_SHIFT));
	writel(doepctl, &out_regs->doepctl);
}

/* should be called after set configuration is received */
static void dwc_otg_bulk_in_activate(void)
{
	struct device_in_ep_regs *in_regs =
		dev_if->in_ep_regs[UDC_IN_ENDPOINT];
	struct device_global_regs *dev_global_regs
		= dev_if->dev_global_regs;
	u32 diepctl, daint;

	daint = readl(&dev_global_regs->daintmsk);
	daint |= 1 << (UDC_IN_ENDPOINT + DAINTMASK_IN_SHIFT);
	writel(daint, &dev_global_regs->daintmsk);

	diepctl = readl(&in_regs->diepctl);
	diepctl &= ~DIEPCTL_MPSMSK;
	diepctl &= ~EPTYPEMSK;
	diepctl |= (CONFIG_USBD_SERIAL_BULK_PKTSIZE
			| USBACTEP | DATA0PID
			| (EPTYPE_BULK << EPTYPE_SHIFT));
	writel(diepctl, &in_regs->diepctl);
}

static void dwc_otg_int_in_activate(void)
{
	struct device_in_ep_regs *in_regs =
		dev_if->in_ep_regs[UDC_INT_ENDPOINT];
	struct device_global_regs *dev_global_regs
		= dev_if->dev_global_regs;
	u32 diepctl, daint;

	daint = readl(&dev_global_regs->daintmsk);
	daint |= 1 << (UDC_INT_ENDPOINT + DAINTMASK_IN_SHIFT);
	writel(daint, &dev_global_regs->daintmsk);

	diepctl = readl(&in_regs->diepctl);
	diepctl &= ~DIEPCTL_MPSMSK;
	diepctl &= ~EPTYPEMSK;
	diepctl |= (UDC_INT_PACKET_SIZE
			| USBACTEP | DATA0PID
			| (EPTYPE_INT << EPTYPE_SHIFT));
	writel(diepctl, &in_regs->diepctl);
}

/* should be called after set configuration is received */
void udc_set_configuration_controller(u32 config)
{
	dwc_otg_bulk_out_activate();
	dwc_otg_bulk_in_activate();
	dwc_otg_int_in_activate();
	usbd_device_event_irq(udc_device, DEVICE_CONFIGURED, 0);
}

/* should be called to receive next packet */
static void dwc_otg_bulk_out_enable(void)
{
	struct device_out_ep_regs *out_regs =
		dev_if->out_ep_regs[UDC_OUT_ENDPOINT];
	u32 doepctl, doeptsiz;

	doeptsiz = CONFIG_USBD_SERIAL_BULK_PKTSIZE;
	doeptsiz |= (1 << PKTCNT_SHIFT);
	writel(doeptsiz, &out_regs->doeptsiz);
	doepctl = readl(&out_regs->doepctl);
	doepctl |= CNAK | EPENA;
	writel(doepctl, &out_regs->doepctl);
}

/* This interrupt indicates that an OUT EP has a pending Interrupt. */

static int dwc_otg_pcd_handle_out_ep_intr(void)
{
	u32 ep_intr;
	u32 doepint;
	u32 epnum = 0;
	struct dwc_ep *dwc_ep;
	struct device_out_ep_regs **out_ep_regs
		= dev_if->out_ep_regs;

	/* Read in the device interrupt bits */
	ep_intr = dwc_otg_read_dev_all_out_ep_intr();
	while (ep_intr) {
		if (ep_intr & 0x1) {
			dwc_ep = get_out_ep(epnum);
			doepint = dwc_otg_read_doep_intr(dwc_ep);

			/* Transfer complete */
			if (doepint & XFERCOMPL) {
				/* Clear xfercompl */
				writel(XFERCOMPL, &out_ep_regs[epnum]->doepint);
				if (!epnum)
					ep0_out_start();
				else if (epnum == UDC_OUT_ENDPOINT)
					dwc_otg_bulk_out_enable();
			}
			/* Setup Phase Done (control EPs) */
			if (doepint & SETUP) {
				writel(SETUP, &out_ep_regs[epnum]->doepint);
				handle_ep0(0);
			}
		}
		epnum++;
		ep_intr >>= 1;
	}
	return 1;
}

/* This interrupt indicates that an IN EP has a pending Interrupt. */

static int dwc_otg_pcd_handle_in_ep_intr(void)
{
	u32 ep_intr;
	u32 diepint;
	u32 epnum = 0;
	struct dwc_ep *dwc_ep;
	struct device_in_ep_regs **in_ep_regs
		= dev_if->in_ep_regs;

	/* Read in the device interrupt bits */
	ep_intr = dwc_otg_read_dev_all_in_ep_intr();
	while (ep_intr) {
		if (ep_intr & 0x1) {
			dwc_ep = get_in_ep(epnum);
			diepint = dwc_otg_read_diep_intr(dwc_ep);

			/* IN token received when txfifo empty */
			if (diepint & INTKNTXFEMP) {
				/* Clear xfercompl */
				writel(INTKNTXFEMP,
						&in_ep_regs[epnum]->diepint);
				if (!epnum)
					handle_ep0(1);
				else if (epnum == UDC_IN_ENDPOINT)
					dw_udc_epn_tx(dwc_ep);
			}
		}
		epnum++;
		ep_intr >>= 1;
	}
	return 1;
}

static void dwc_otg_flush_tx_fifo(const int num)
{
	struct core_global_regs *global_regs = dev_if->core_global_regs;
	u32 val = 0;
	int count = 0;

	val = readl(&global_regs->grstctl);
	val |= TXFFLSH;
	val &= ~TXFNUM;
	val |= (num << TXFNUM_SHIFT);
	writel(val, &global_regs->grstctl);

	do {
		val = readl(&global_regs->grstctl);
		if (++count > 10000)
			break;
		udelay(1);
	} while (val & TXFFLSH);

	/* Wait for 3 PHY Clocks */
	udelay(1);
}

static void dwc_otg_flush_rx_fifo(void)
{
	struct core_global_regs *global_regs = dev_if->core_global_regs;
	int count = 0;
	u32 val = 0;

	val = readl(&global_regs->grstctl);
	val |= RXFFLSH;
	writel(val, &global_regs->grstctl);

	do {
		val = readl(&global_regs->grstctl);
		if (++count > 10000)
			break;
		udelay(1);
	} while (val & RXFFLSH);

	/* Wait for 3 PHY Clocks */
	udelay(1);
}

/*
 * This interrupt occurs when a USB Reset is detected. When the USB Reset
 * Interrupt occurs the device state is set to DEFAULT and the EP0 state is set
 * to IDLE.
 *
 */
static int dwc_otg_pcd_handle_usb_reset_intr(void)
{
	u32 temp;
	u32 i;
	u32 gintmsk;
	struct device_out_ep_regs **out_ep_regs
		= dev_if->out_ep_regs;
	struct device_global_regs *dev_global_regs
		= dev_if->dev_global_regs;
	struct core_global_regs *core_global_regs
		= dev_if->core_global_regs;
	/* Set NAK for all OUT EPs */
	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		temp = readl(&out_ep_regs[i]->doepctl);
		temp |= SNAK;
		writel(temp, &out_ep_regs[i]->doepctl);
	}

	/* Flush the NP Tx FIFO */
	dwc_otg_flush_tx_fifo(DWC_GRSTCTL_TXFNUM_ALL);
	dwc_otg_flush_rx_fifo();
	writel((1 << (DAINTMASK_IN_SHIFT + 0))
				| (1 << (DAINTMASK_OUT_SHIFT + 0)),
				&dev_global_regs->daintmsk);

	writel(SETUPMSK | XFERCOMPLMSK | AHBERRMSK | EPDISABLEDMSK,
			&dev_global_regs->doepmsk);

	writel(INTKNTXFEMP, &dev_global_regs->diepmsk);

	gintmsk = readl(&core_global_regs->gintmsk);
	gintmsk |= GOUTNAKEFF;
	writel(gintmsk, &core_global_regs->gintmsk);

	/* program fifo size for ep0 */

	writel(0x200, &core_global_regs->grxfsiz);

	temp = readl(&dev_if->in_ep_regs[0]->diepctl);
	temp &= 0xFFC3FFFF; /* TxFNumBF = 0, bits 25:22 */
	writel(temp, &dev_if->in_ep_regs[0]->diepctl);

	temp = readl(&core_global_regs->gnptxfsiz);

	writel(0x2000000, &core_global_regs->gnptxfsiz);

	/* Reset Device Address */
	temp = readl(&dev_global_regs->dcfg);
	temp &= ~DEVADDRMSK;
	writel(temp, &dev_global_regs->dcfg);

	/* setup EP0 to receive SETUP packets */
	ep0_out_start();

	/* Clear interrupt */
	writel(USBRESET, &core_global_regs->gintsts);

	UDCDBG("device reset in progess");
	usbd_device_event_irq(udc_device, DEVICE_HUB_CONFIGURED, 0);

	return 1;
}

/*
 * This function enables EP0 OUT to receive SETUP packets and configures EP0
 * IN for transmitting packets. It is normally called when the "Enumeration
 * Done" interrupt occurs.
 */
static void dwc_otg_ep0_activate(void)
{
	u32 temp;
	struct device_in_ep_regs *in_ep_regs = dev_if->in_ep_regs[0];
	struct device_out_ep_regs *out_ep_regs = dev_if->out_ep_regs[0];

	/* Read the Device Status and Endpoint 0 Control registers */
	temp = readl(&in_ep_regs->diepctl);
	temp &= ~MPSMSK0;
	temp |= DWC_DEP0CTL_MPS_64;
	writel(temp, &in_ep_regs->diepctl);

	temp = readl(&out_ep_regs->doepctl);
	/* Enable OUT EP for receive */
	temp |= EPENA;
	writel(temp, &out_ep_regs->doepctl);
}

/*
 * Read the device status register and set the device speed in the
 * data structure.
 * Set up EP0 to receive SETUP packets by calling dwc_ep0_activate.
 */
static int dwc_otg_pcd_handle_enum_done_intr(void)
{
	u32 gusbcfg;
	struct core_global_regs *global_regs = dev_if->core_global_regs;
	dwc_otg_ep0_activate();

	gusbcfg = readl(&global_regs->gusbcfg);
	gusbcfg &= ~USBTRDTIMMSK;
	gusbcfg |= PHYIF_16BIT;
	gusbcfg |= (9 << USBTRDTIM_SHIFT);
	writel(gusbcfg, &global_regs->gusbcfg);
	/* Clear interrupt */
	writel(ENUMDONE, &global_regs->gintsts);
	usbd_device_event_irq(udc_device, DEVICE_RESET, 0);
	return 1;
}

static u32 dwc_otg_read_core_intr(void)
{
	return readl(&dev_if->core_global_regs->gintsts) &
		readl(&dev_if->core_global_regs->gintmsk);
}

static void dwc_otg_init(const void *reg_base)
{
	struct dwc_pcd *pcd = &dev_if->pcd;
	u32 offset;
	u32 i;

	dev_if->core_global_regs = (struct core_global_regs *) reg_base;
	dev_if->dev_global_regs = (struct device_global_regs *) ((u32)reg_base +
			DWC_DEV_GLOBAL_REG_OFFSET);

	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		offset = i * DWC_EP_REG_OFFSET;

		dev_if->in_ep_regs[i] = (struct device_in_ep_regs *)
			((u32)reg_base + DWC_DEV_IN_EP_REG_OFFSET + offset);

		dev_if->out_ep_regs[i] = (struct device_out_ep_regs *)
			((u32)reg_base + DWC_DEV_OUT_EP_REG_OFFSET + offset);
	}

	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		dev_if->data_fifo[i] =
			(u32 *) ((u32)reg_base + DWC_OTG_DATA_FIFO_OFFSET +
					(i * DWC_OTG_DATA_FIFO_SIZE));
	}

	dev_if->speed = 0;	/* unknown */
	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		pcd->in_ep[i].num = i;
		pcd->out_ep[i].num = i;
	}
}

/*
 * This function initializes the DWC_otg controller registers and prepares the
 * core for device mode
 */
static void dwc_otg_core_init(void)
{
	struct core_global_regs *global_regs = dev_if->core_global_regs;
	u32 ahbcfg, gintmsk, usbcfg;
	/* Step 1: Program the GAHBCFG Register. */
	ahbcfg = DWC_NPTXEMPTYLVL_EMPTY | DWC_PTXEMPTYLVL_EMPTY;
	writel(ahbcfg, &global_regs->gahbcfg);

	/* Step 2: write usbcfg regs*/
	usbcfg = readl(&global_regs->gusbcfg);
	usbcfg |= SRPCAP | HNPCAP;
	writel(usbcfg, &global_regs->gusbcfg);

	/* step3: write int_msk reg*/
	gintmsk = USBRESET | ENUMDONE | RXSTSQLVL | OUTEPINTR | INEPINTR;
	writel(gintmsk, &global_regs->gintmsk);
}

/* Switch on the UDC */
static void usbotg_init(void)
{
	udc_device = NULL;
#ifdef APG_BOARD
	dwc_otg_init((void *)0x11000000);
#else
	dwc_otg_init((void *)CONFIG_SYS_USBD_BASE);
#endif

	/* Initialize the DWC_otg core.	*/
	dwc_otg_core_init();

}

void udc_irq(void)
{
	u32 status;

	status = dwc_otg_read_core_intr();
	while (status) {
		if (status & USBRESET)
			dwc_otg_pcd_handle_usb_reset_intr();
		if (status & ENUMDONE)
			dwc_otg_pcd_handle_enum_done_intr();
		if (status & RXSTSQLVL)
			dwc_otg_pcd_handle_rx_status_q_level_intr();
		if (status & OUTEPINTR)
			dwc_otg_pcd_handle_out_ep_intr();
		if (status & INEPINTR)
			dwc_otg_pcd_handle_in_ep_intr();
		status = dwc_otg_read_core_intr();
	}

}

void udc_set_nak(int epid)
{
	writel(readl(&dev_if->out_ep_regs[epid]->doepctl) | SNAK,
			&dev_if->out_ep_regs[epid]->doepctl);
	writel(readl(&dev_if->in_ep_regs[epid]->diepctl) | SNAK,
			&dev_if->in_ep_regs[epid]->diepctl);
}

void udc_unset_nak(int epid)
{
	writel(readl(&dev_if->out_ep_regs[epid]->doepctl) | CNAK,
			&dev_if->out_ep_regs[epid]->doepctl);
	writel(readl(&dev_if->in_ep_regs[epid]->diepctl) | CNAK,
			&dev_if->in_ep_regs[epid]->diepctl);
}

int udc_endpoint_write(struct usb_endpoint_instance *endpoint)
{
	udc_unset_nak(endpoint->endpoint_address & USB_ENDPOINT_NUMBER_MASK);
	return 0;
}

static void udc_enable(struct usb_device_instance *device)
{
	struct dwc_pcd *pcd = &dev_if->pcd;

	UDCDBGA("enable device %p, status %d", device, device->status);

	/* Save the device structure pointer */
	udc_device = device;

	/* Setup ep0 urb */
	if (!ep0_urb) {
		ep0_urb =
			usbd_alloc_urb(udc_device,
					udc_device->bus->endpoint_array);
		pcd->req =
			(struct usb_device_request *)&ep0_urb->device_request;
		pcd->ep0.xfer_buff = (u8 *)ep0_urb->buffer;
	} else {
#ifndef APG_BOARD
		serial_printf("udc_enable: ep0_urb already allocated %p\n",
				ep0_urb);
#endif
	}
}

void udc_connect(void)
{
	struct device_global_regs *dev_regs = dev_if->dev_global_regs;
	u32 dcfg;

	/* remove soft disconnect */
	dcfg = readl(&dev_regs->dctl);
	dcfg &= ~SFTDISCON;
	writel(dcfg, &dev_regs->dctl);
}

void udc_disconnect(void)
{
	struct device_global_regs *dev_regs = dev_if->dev_global_regs;
	u32 dcfg;

	/* soft disconnect */
	dcfg = readl(&dev_regs->dctl);
	dcfg |= SFTDISCON;
	writel(dcfg, &dev_regs->dctl);
	udelay(150);
}

void udc_startup_events(struct usb_device_instance *device)
{
	/* The DEVICE_INIT event puts the USB device in the state STATE_INIT. */
	usbd_device_event_irq(device, DEVICE_INIT, 0);

	/*
	 * The DEVICE_CREATE event puts the USB device in the state
	 * STATE_ATTACHED.
	 */
	usbd_device_event_irq(device, DEVICE_CREATE, 0);

	/*
	 * Some USB controller driver implementations signal
	 * DEVICE_HUB_CONFIGURED and DEVICE_RESET events here.
	 * DEVICE_HUB_CONFIGURED causes a transition to the state STATE_POWERED,
	 * and DEVICE_RESET causes a transition to the state STATE_DEFAULT.
	 * The DW USB client controller has the capability to detect when the
	 * USB cable is connected to a powered USB bus, so we will defer the
	 * DEVICE_HUB_CONFIGURED and DEVICE_RESET events until later.
	 */

	udc_enable(device);

}

void udc_setup_ep(struct usb_device_instance *device, unsigned int ep,
		struct usb_endpoint_instance *endpoint)
{
	/*
	 * Nothing to do here. Hob of this function has laready been
	 * done during init.
	 */
}

int is_usbd_high_speed(void)
{
	struct device_global_regs *dev_regs = dev_if->dev_global_regs;
	u32 dsts;

	dsts = readl(&dev_regs->dsts);
	dsts &= ENUMSPDMSK;
	if (dsts == DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ)
		return 1;
	else
		return 0;
}

int udc_init(void)
{
	phy_init();
	udc_disconnect();
	usbotg_init();
	return 0;
}
