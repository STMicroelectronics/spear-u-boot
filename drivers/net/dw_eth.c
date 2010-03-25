/*
 * (C) Copyright 2009
 * Vipin Kumar, ST Micoelectronics, vipin.kumar@st.com.
 *
 * (C) Copyright 2008
 * Deepak Sikri, ST Micoelectronics, deepak.sikri@st.com.
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

/*
 * Designware ethernet IP driver for u-boot
 */

#include <common.h>
#include <miiphy.h>
#include <malloc.h>
#include <linux/err.h>
#include <asm/io.h>
#include "dw_eth.h"

static void tx_descs_init(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	struct dmamacdescr *desc_table_p = &priv->tx_mac_descrtable[0];
	char *txbuffs = &priv->txbuffs[0];
	struct dmamacdescr *desc_p;

	u32 idx;

	for (idx = 0; idx < CONFIG_TX_DESCR_NUM; idx++) {

		desc_p = &desc_table_p[idx];
		desc_p->dmamac_addr = &txbuffs[idx * CONFIG_ETH_BUFSIZE];
		desc_p->dmamac_next = &desc_table_p[idx + 1];

#if defined(CONFIG_DW_ALTDESC)
		desc_p->txrx_status &= ~(DESC_TXSTS_TXINT | DESC_TXSTS_TXLAST |
				DESC_TXSTS_TXFIRST | DESC_TXSTS_TXCRCDIS | \
				DESC_TXSTS_TXCHECKINSCTRL | \
				DESC_TXSTS_TXRINGEND | DESC_TXSTS_TXPADDIS);

		desc_p->txrx_status |= DESC_TXSTS_TXCHAIN;
		desc_p->dmamac_cntl = 0;
		desc_p->txrx_status &= ~(DESC_TXSTS_MSK | DESC_TXSTS_OWNBYDMA);
#else
		desc_p->dmamac_cntl = DESC_TXCTRL_TXCHAIN;
		desc_p->txrx_status = 0;
#endif
	}

	/* Correcting the last pointer of the chain */
	desc_p->dmamac_next = &desc_table_p[0];

	writel((ulong)&desc_table_p[0], &dma_p->txdesclistaddr);
}

static void rx_descs_init(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	struct dmamacdescr *desc_table_p = &priv->rx_mac_descrtable[0];
	char *rxbuffs = &priv->rxbuffs[0];
	struct dmamacdescr *desc_p;

	u32 idx;

	for (idx = 0; idx < CONFIG_RX_DESCR_NUM; idx++) {

		desc_p = &desc_table_p[idx];
		desc_p->dmamac_addr = &rxbuffs[idx * CONFIG_ETH_BUFSIZE];
		desc_p->dmamac_next = &desc_table_p[idx + 1];

		desc_p->dmamac_cntl =
			(MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | \
				      DESC_RXCTRL_RXCHAIN;

		desc_p->txrx_status = DESC_RXSTS_OWNBYDMA;
	}

	/* Correcting the last pointer of the chain */
	desc_p->dmamac_next = &desc_table_p[0];

	writel((ulong)&desc_table_p[0], &dma_p->rxdesclistaddr);
}

static void descs_init(struct eth_device *dev)
{
	tx_descs_init(dev);
	rx_descs_init(dev);
}

static void mac_reset(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;

	u32 timeout = CONFIG_MACRESET_TIMEOUT;

	writel(DMAMAC_SRST, &dma_p->busmode);
	writel(MII_PORTSELECT, &mac_p->conf);

	do {
		if (!(readl(&dma_p->busmode) & DMAMAC_SRST))
			break;
		udelay(1000);
	} while (timeout--);
}

static int dw_eth_init(struct eth_device *dev, bd_t *bis)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;

	u8 *mac_id = &dev->enetaddr[0];
	u32 conf, macid_lo, macid_hi;

	/* Reset ethernet hardware */
	mac_reset(dev);

	macid_lo = mac_id[0] + (mac_id[1] << 8) + \
		   (mac_id[2] << 16) + (mac_id[3] << 24);
	macid_hi = mac_id[4] + (mac_id[5] << 8);

	writel(macid_hi, &mac_p->macaddr0hi);
	writel(macid_lo, &mac_p->macaddr0lo);

	writel(FIXEDBURST | PRIORXTX_41 | BURST_16,
			&dma_p->busmode);

	writel(FLUSHTXFIFO | readl(&dma_p->opmode), &dma_p->opmode);
	writel(STOREFORWARD | TXSECONDFRAME, &dma_p->opmode);

	conf = FRAMEBURSTENABLE | DISABLERXOWN;

	if (priv->speed != SPEED_1000M)
		conf |= MII_PORTSELECT;

	if (priv->duplex == FULL_DUPLEX)
		conf |= FULLDPLXMODE;

	writel(conf, &mac_p->conf);

	descs_init(dev);

	/*
	 * Start/Enable xfer at dma as well as mac level
	 */
	writel(readl(&dma_p->opmode) | RXSTART, &dma_p->opmode);
	writel(readl(&dma_p->opmode) | TXSTART, &dma_p->opmode);

	writel(readl(&mac_p->conf) | RXENABLE, &mac_p->conf);
	writel(readl(&mac_p->conf) | TXENABLE, &mac_p->conf);

	return 0;
}

static int dw_eth_send(struct eth_device *dev, volatile void *packet,
		int length)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	u32 desc_num = priv->tx_currdescnum;
	struct dmamacdescr *desc_p = &priv->tx_mac_descrtable[desc_num];

	/* Check if the descriptor is owned by CPU */
	if (desc_p->txrx_status & DESC_TXSTS_OWNBYDMA) {
		printf("eth_send : CPU not owner of tx frame\n");
		return -1;
	}

	memcpy((void *)desc_p->dmamac_addr, (void *)packet, length);

#if defined(CONFIG_DW_ALTDESC)
	desc_p->txrx_status |= DESC_TXSTS_TXFIRST | DESC_TXSTS_TXLAST;
	desc_p->dmamac_cntl |= (length << DESC_TXCTRL_SIZE1SHFT) & \
			       DESC_TXCTRL_SIZE1MASK;

	desc_p->txrx_status &= ~(DESC_TXSTS_MSK);
	desc_p->txrx_status |= DESC_TXSTS_OWNBYDMA;
#else
	desc_p->dmamac_cntl |= ((length << DESC_TXCTRL_SIZE1SHFT) & \
			       DESC_TXCTRL_SIZE1MASK) | DESC_TXCTRL_TXLAST | \
			       DESC_TXCTRL_TXFIRST;

	desc_p->txrx_status = DESC_TXSTS_OWNBYDMA;
#endif

	/* Test the wrap-around condition. */
	if (++desc_num >= CONFIG_TX_DESCR_NUM)
		desc_num = 0;

	priv->tx_currdescnum = desc_num;

	/* Start the transmission */
	writel(POLL_DATA, &dma_p->txpolldemand);

	return 0;
}

static int dw_eth_recv(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	u32 desc_num = priv->rx_currdescnum;
	struct dmamacdescr *desc_p = &priv->rx_mac_descrtable[desc_num];

	u32 status = desc_p->txrx_status;
	int length = 0;

	/* Check  if the owner is the CPU */
	if (!(status & DESC_RXSTS_OWNBYDMA)) {

		length = (status & DESC_RXSTS_FRMLENMSK) >> \
			 DESC_RXSTS_FRMLENSHFT;

		NetReceive(desc_p->dmamac_addr, length);

		/*
		 * Make the current descriptor valid again and go to
		 * the next one
		 */
		desc_p->txrx_status |= DESC_RXSTS_OWNBYDMA;

		/* Test the wrap-around condition. */
		if (++desc_num >= CONFIG_RX_DESCR_NUM)
			desc_num = 0;
	}

	priv->rx_currdescnum = desc_num;

	return length;
}

static void dw_eth_halt(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;

	mac_reset(dev);
	priv->tx_currdescnum = priv->rx_currdescnum = 0;
}

static int eth_mdio_read(char *devname, u8 addr, u8 reg, u16 *val)
{
	struct eth_device *dev = container_of(devname, struct eth_device,
							name[0]);
	struct dw_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	u32 miiaddr;
	u32 timeout = CONFIG_MDIO_TIMEOUT;

	miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) | \
		  ((reg << MIIREGSHIFT) & MII_REGMSK);

	writel(miiaddr | MII_CLKRANGE_150_250M | MII_BUSY, &mac_p->miiaddr);

	do {
		if (!(readl(&mac_p->miiaddr) & MII_BUSY))
			break;
		udelay(1000);
	} while (timeout--);

	*val = readl(&mac_p->miidata);

	return 0;
}

static int eth_mdio_write(char *devname, u8 addr, u8 reg, u16 val)
{
	struct eth_device *dev = container_of(devname, struct eth_device,
							name[0]);
	struct dw_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	u32 miiaddr;
	u32 timeout = CONFIG_MDIO_TIMEOUT;
	u16 value;

	writel(val, &mac_p->miidata);
	miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) | \
		  ((reg << MIIREGSHIFT) & MII_REGMSK) | MII_WRITE;

	writel(miiaddr | MII_CLKRANGE_150_250M | MII_BUSY, &mac_p->miiaddr);

	do {
		if (!(readl(&mac_p->miiaddr) & MII_BUSY))
			break;
		udelay(1000);
	} while (timeout--);

	/* Needed as a fix for ST-Phy */
	eth_mdio_read(devname, addr, reg, &value);

	return 0;
}

static u8 find_phy(struct eth_device *dev)
{
	u8 phy_addr = 0;
	u16 ctrl, oldctrl;

	do {
		eth_mdio_read(dev->name, phy_addr, PHY_BMCR, &ctrl);
		oldctrl = ctrl & PHY_BMCR_AUTON;

		ctrl ^= PHY_BMCR_AUTON;
		eth_mdio_write(dev->name, phy_addr, PHY_BMCR, ctrl);
		eth_mdio_read(dev->name, phy_addr, PHY_BMCR, &ctrl);
		ctrl &= PHY_BMCR_AUTON;

		if (ctrl == oldctrl) {
			phy_addr++;
		} else {
			ctrl ^= PHY_BMCR_AUTON;
			eth_mdio_write(dev->name, phy_addr, PHY_BMCR, ctrl);
			break;
		}
	} while (phy_addr < 32);

	return phy_addr;
}

static void dw_reset_phy(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	u16 ctrl;
	u32 timeout = CONFIG_PHYRESET_TIMEOUT;
	u32 phy_addr = priv->address;

	eth_mdio_write(dev->name, phy_addr, PHY_BMCR, PHY_BMCR_RESET);
	do {
		eth_mdio_read(dev->name, phy_addr, PHY_BMCR, &ctrl);
		if (!(ctrl & PHY_BMCR_RESET))
			break;
		udelay(1000);
	} while (timeout--);

#ifdef CONFIG_PHY_RESET_DELAY
	udelay(CONFIG_PHY_RESET_DELAY);
#endif
}

/*
void reset_phy(void)
{
	dw_reset_phy(netdev);
}
*/

static void configure_phy(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	u8 phy_addr;
	u16 bmcr, ctrl;
#if defined(CONFIG_DW_AUTONEG)
	u16 bmsr;
	u32 timeout;
	u16 anlpar, btsr;
#endif
	priv->address = find_phy(dev);
	phy_addr = priv->address;

	dw_reset_phy(dev);

#if defined(CONFIG_DW_AUTONEG)
	bmcr = PHY_BMCR_AUTON | PHY_BMCR_RST_NEG | PHY_BMCR_100MB | \
	       PHY_BMCR_DPLX | PHY_BMCR_1000_MBPS;
#else
	bmcr = PHY_BMCR_100MB | PHY_BMCR_DPLX;

#if defined(CONFIG_DW_SPEED10M)
	bmcr &= ~PHY_BMCR_100MB;
#endif
#if defined(CONFIG_DW_DUPLEXHALF)
	bmcr &= ~PHY_BMCR_DPLX;
#endif
#endif
	eth_mdio_write(dev->name, phy_addr, PHY_BMCR, bmcr);

	/* Read the phy status register and populate priv structure */
#if defined(CONFIG_DW_AUTONEG)
	timeout = CONFIG_AUTONEG_TIMEOUT;
	do {
		eth_mdio_read(dev->name, phy_addr, PHY_BMSR, &bmsr);
		if (bmsr & PHY_BMSR_AUTN_COMP)
			break;
		udelay(1000);
	} while (timeout--);

	eth_mdio_read(dev->name, phy_addr, PHY_ANLPAR, &anlpar);
	eth_mdio_read(dev->name, phy_addr, PHY_1000BTSR, &btsr);

	if (btsr & (PHY_1000BTSR_1000FD | PHY_1000BTSR_1000HD)) {
		priv->speed = SPEED_1000M;
		if (btsr & PHY_1000BTSR_1000FD)
			priv->duplex = FULL_DUPLEX;
		else
			priv->duplex = HALF_DUPLEX;
	} else {
		if (anlpar & PHY_ANLPAR_100)
			priv->speed = SPEED_100M;
		else
			priv->speed = SPEED_10M;

		if (anlpar & (PHY_ANLPAR_10FD | PHY_ANLPAR_TXFD))
			priv->duplex = FULL_DUPLEX;
		else
			priv->duplex = HALF_DUPLEX;
	}
#else
	eth_mdio_read(dev->name, phy_addr, PHY_BMCR, &ctrl);

	if (ctrl & PHY_BMCR_DPLX)
		priv->duplex = FULL_DUPLEX;
	else
		priv->duplex = HALF_DUPLEX;

	if (ctrl & PHY_BMCR_1000_MBPS)
		priv->speed = SPEED_1000M;
	else if (ctrl & PHY_BMCR_100_MBPS)
		priv->speed = SPEED_100M;
	else
		priv->speed = SPEED_10M;
#endif
}

int dw_mii_initialize(u32 id, ulong base_addr)
{
	struct eth_device *netdev;
	struct dw_eth_dev *priv;

	netdev = (struct eth_device *) malloc(sizeof(struct eth_device));
	if (!netdev)
		return -ENOMEM;

	/*
	 * Since the priv structure contains the descriptors which need a strict
	 * buswidth alignment, memalign is used to allocate memory
	 */
	priv = (struct dw_eth_dev *) memalign(16, sizeof(struct dw_eth_dev));
	if (!priv) {
		free(netdev);
		return -ENOMEM;
	}

	memset(netdev, 0, sizeof(struct eth_device));
	memset(priv, 0, sizeof(struct dw_eth_dev));

	sprintf(netdev->name, "mii%d", id);
	netdev->iobase = (int)base_addr;
	netdev->priv = priv;

	eth_getenv_enetaddr_by_index(id, &netdev->enetaddr[0]);

	priv->dev = netdev;
	priv->mac_regs_p = (struct eth_mac_regs *)base_addr;
	priv->dma_regs_p = (struct eth_dma_regs *)(base_addr +
			DW_DMA_BASE_OFFSET);

	mac_reset(netdev);
	configure_phy(netdev);

	netdev->init = dw_eth_init;
	netdev->send = dw_eth_send;
	netdev->recv = dw_eth_recv;
	netdev->halt = dw_eth_halt;

	eth_register(netdev);

	return 0;
}
