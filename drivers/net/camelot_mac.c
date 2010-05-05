/*
 * Donald Chan, donald@plasternetworks.com
 * Copyright (C) 2010 Plaster Networks, LLC.  All rights reserved.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */
#include <asm/camelot/camelot.h>
#include <asm/camelot/camelot_mac.h>
#include <asm/delay.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>

MODULE_AUTHOR("Donald Chan <donald@plasternetworks.com>");
MODULE_DESCRIPTION("Camelot ethernet driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:camelot_mac");

#define DRV_NAME 			"camelot_mac"
#define CAMELOT_MAC_DEFAULT_ADDRESS	{ 0x00, 0x24, 0xe7, 0x00, 0x00, 0x01 }
#define CAMELOT_MAC_PHY_BUS		"0"
#define CAMELOT_MAC_PHY_ID		5
#define CAMELOT_MAC_SKB_SIZE		1700
#define CAMELOT_OFFSET			0

static int debug_level = 8;
module_param(debug_level, int, 0444);
MODULE_PARM_DESC(debug_level, "Number of NETIF_MSG bits to enable");

static int mdio_irqs[PHY_MAX_ADDR]= { PHY_POLL, };
static struct mii_bus *camelot_mac_mii;

#define MDROPS_INIT	0x1e0100fb	
#define MRXCR_INIT	0x8201004 //defulat=0x8321004
#define ML3_INIT	0x071505dc

#define	NUM_MBUF	 	(NUM_RX_DESC+24+2)

static void cml_eth_macatable_wr(unsigned char *mac, char fid, char num, unsigned short info, unsigned short flag);

static int camelot_mac_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	u32 value = 0;

	value = MAC_MDIO_START;
	value |= (phy_id << MAC_MDIO_PHY_SHIFT);
	value |= (reg << MAC_MDIO_REG_SHIFT);

	writel(value, MAC_MDIO_REG);

	udelay(1);

	while (1) {
		if (MAC_MDIO_START & (value = readl(MAC_MDIO_REG)))
			break;
		cpu_relax();
	}

	return MAC_MDIO_VALUE(value);
}

static int camelot_mac_mdio_write(struct mii_bus *bus, int phy_id,
							int reg, u16 val)
{
	u32 value = 0;

	value = MAC_MDIO_START | MAC_MDIO_WRITE;
	value |= (phy_id << MAC_MDIO_PHY_SHIFT);
	value |= (reg << MAC_MDIO_REG_SHIFT);
	value |= MAC_MDIO_VALUE(val);

	writel(value, MAC_MDIO_REG);

	udelay(1);

	while (1) {
		if (MAC_MDIO_START & (value = readl(MAC_MDIO_REG)))
			break;
		cpu_relax();
	}

	return 0;
}

static int camelot_mac_mdio_reset(struct mii_bus *bus)
{
	/* Nothing to do */
	return 0;
}

static void dump_data(const char *in, int size)
{
	int i = 0;
	int remaining = 0;
	char buffer[128];
	char s[8];
	char s1[16 + 1];

	if (!in)
		return;

	snprintf(buffer, sizeof(buffer), "0x%04x:  ", i);
	memset(s, 0, sizeof(s));
	memset(s1, 0, sizeof(s1));

	for (i = 0; i < size; i += 2) {
		if (!(i & 0xf) && i) {
			strncat(buffer, s1,
				sizeof(buffer) - strlen(buffer) - 1);
			printk(KERN_INFO "%s\n", buffer);
			snprintf(buffer, sizeof(buffer), "0x%04x:  ", i);
			memset(s1, 0, sizeof(s1));
		}

		/* Add first byte */
		snprintf(s, sizeof(s), "%02x", (u_int8_t)in[i]);

		if (in[i] < ' ' || in[i] > '~') {
			s1[i & 0xf] = '.';
		} else {
			s1[i & 0xf] = (u_int8_t)in[i];
		}

		strncat(buffer, s, sizeof(buffer) - strlen(buffer) - 1);

		/* More byte? */
		if ((i + 1) >= size)
			continue;

		snprintf(s, sizeof(s), "%02x ", (u_int8_t)in[i + 1]);

		if (in[i + 1] < ' ' || in[i + 1] > '~') {
			s1[(i + 1) & 0xf] = '.';
		} else {
			s1[(i + 1) & 0xf] = (u_int8_t)in[i + 1];
		}

		strncat(buffer, s, sizeof(buffer) - strlen(buffer) - 1);
	}

	if (i & 0x0f) {
		/* Pad it with space */
		remaining = 49 - strlen(buffer);
		for (i = 0; i < remaining; i++)
			strncat(buffer, " ",
				sizeof(buffer) - strlen(buffer) - 1);
	}

	strncat(buffer, s1, sizeof(buffer) - strlen(buffer) - 1);
	printk(KERN_INFO "%s\n", buffer);
}

static void camelot_mac_switch_preset(void)
{
	/* Clear all settings except MDIO clock */
	writel(MAC_CTRL_INIT & 0xff000000, MAC_CTRL_REG);

	/* TODO */
	camelot_mac_mdio_write(camelot_mac_mii, 20, 6, 0x0f3f);
}

struct phy_config
{
	char phy;
	char reg; 
	unsigned short val;
};

#define VLAN_MASK(m0, m1, m2, m3, m4, m5) \
		(((m0) << 0) | ((m1) << 1) | ((m2) << 2) | \
		((m3) << 3) | ((m4) << 4) | ((m5) << 5))

#define VLAN_ENTRY(id, fid) \
		((id) << 0) | ((fid) << 12)

#define VLAN_VALID	VLAN_MASK
#define VLAN_GRP	VLAN_MASK
#define VLAN_ADD_TAG	VLAN_MASK
#define VLAN_DEL_TAG	VLAN_MASK
#define VLAN_TAG_VLAN	VLAN_MASK

#define VLAN0(x)	(x)
#define VLAN1(x)	((x) << 8)

static struct phy_config repeating_config[] =
{
	/*
	 * Address Table Structure: 1K for unicast and 1K for multicast
	 * 
	 * VLAN FID is used to create a hash key
	 */
	{ 20, 13, (1 << 3) | 1 },

	/* VLAN type settings: tag-based VLANs on all ports */
	{ 22, 0, VLAN_TAG_VLAN(1, 1, 1, 1, 1, 1) },

	/*
	 * VLAN entries 0 and 1 are valid
	 */
	{ 22, 10, VLAN_VALID(1, 1, 0, 0, 0, 0) },	

	/*
	 * VLAN IDs:
	 *
	 * VLAN0: 1
	 * VLAN1: 2
	 */
	{ 22, 14, VLAN_ENTRY(1, 1) },
	{ 22, 15, VLAN_ENTRY(2, 2) },

	/* Port 0 default VLAN ID: 1 */
	{ 22, 4, 1 },

	/* Port 1 default VLAN ID: 2 */
	{ 22, 5, 2 },

	/* Port 2 default VLAN ID: 2 */
	{ 22, 6, 2 },

	/*
	 * VLAN memberships:
	 *
	 * VLAN0: Port 0 (HPAV) and Port 5 (CPU)
	 * VLAN1: Port 1, 2 (LAN) and Port 5 (CPU)
	 */
	{ 23, 0, VLAN0(VLAN_GRP(1, 0, 0, 0, 0, 1))
			| VLAN1(VLAN_GRP(0, 1, 1, 0, 0, 1)) },

	/*
	 * VLAN tag insertion
	 *
	 * VLAN0: Add tag on Port 5 (CPU)
	 * VLAN1: Add tag on Port 5 (CPU)
	 */
	{ 23, 8, VLAN0(VLAN_ADD_TAG(0, 0, 0, 0, 0, 1))
			| VLAN1(VLAN_ADD_TAG(0, 0, 0, 0, 0, 1)) },

	/*
	 * VLAN tag removal
	 *
	 * VLAN0: Remove tag on Port 0 (HPAV)
	 * VLAN1: Remove tag on Port 1 and 2 (LAN)
	 */
	{ 23, 16, VLAN0(VLAN_DEL_TAG(1, 0, 0, 0, 0, 0))
			| VLAN1(VLAN_DEL_TAG(0, 1, 1, 0, 0, 0)) },

	/* Enable port forwarding all on ports */
	{ 20, 6, 0x3f3f },

	{ -1, -1, -1 },
};

static struct phy_config switch_config[] =
{
	/*
	 * Address Table Structure: 1K for unicast and 1K for multicast
	 * 
	 * VLAN FID is used to create a hash key
	 */
	{ 20, 13, (1 << 3) | 1 },

	/* VLAN type settings: tag-based VLANs on all ports */
	{ 22, 0, VLAN_TAG_VLAN(1, 1, 1, 1, 1, 1) },

	/*
	 * VLAN entries 0 and 1 are valid
	 */
	{ 22, 10, VLAN_VALID(1, 0, 0, 0, 0, 0) },	

	/*
	 * VLAN IDs:
	 *
	 * VLAN0: 1
	 */
	{ 22, 14, VLAN_ENTRY(1, 1) },

	/* Port 0 default VLAN ID: 1 */
	{ 22, 4, 1 },

	/* Port 1 default VLAN ID: 1 */
	{ 22, 5, 1 },

	/* Port 2 default VLAN ID: 1 */
	{ 22, 6, 1 },

	/*
	 * VLAN memberships:
	 *
	 * VLAN0: Port 0 (HPAV) and Port 5 (CPU)
	 * VLAN1: Port 1, 2 (LAN) and Port 5 (CPU)
	 */
	{ 23, 0, VLAN0(VLAN_GRP(1, 1, 1, 0, 0, 1)) },

	/*
	 * VLAN tag removal
	 *
	 * VLAN0: Remove tag on Port 0, 1, 2, and 5
	 */
	{ 23, 16, VLAN0(VLAN_DEL_TAG(1, 1, 1, 0, 0, 0)) },

	/* Enable port forwarding all on ports */
	{ 20, 6, 0x3f3f },

	{ -1, -1, -1 },
};

static void camelot_switch_init(void)
{
	struct phy_config *reg_config = switch_config;
	unsigned short val = 0;

	while (reg_config->phy != -1) {
		camelot_mac_mdio_write(camelot_mac_mii, reg_config->phy,
				reg_config->reg, reg_config->val);
		reg_config++;
	}
	/* Broadcast Storm Protection enable */
	{
		val = camelot_mac_mdio_read(camelot_mac_mii, 20, 16);
		camelot_mac_mdio_write(camelot_mac_mii, 20, 16 , val | 0x1f00);
		camelot_mac_mdio_write(camelot_mac_mii, 20, 17 , 0xffff);
		camelot_mac_mdio_write(camelot_mac_mii, 20, 18 , 0xffff);

		val = camelot_mac_mdio_read(camelot_mac_mii, 20, 19);
		camelot_mac_mdio_write(camelot_mac_mii, 20, 19, val | 0x0ff);
	}

}

static void camelot_mac_set_macaddr(u8 mac[6])
{
	char fid = 1;
	u32 addr0 = (mac[0] << 8) | mac[1];
	u32 addr1 = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];

	writel(addr0, MAC_ADDR0_REG);
	writel(addr1, MAC_ADDR1_REG);
	/* configure myself's mac address in switch to avoid mac address land attack */
	cml_eth_macatable_wr((char *)mac, fid, 0, (0x6<<3)|(fid<<6), 0x2);

}

/*=============================================================*
 *  cml_eth_macatable_rd:
 *=============================================================*/
/*!
 * \brief Get the information from switch address table.
 *
 * \param mac: Pointer to a 6-byte array which contain the address
 * \param fid: the number of fid
 * \param num: entry number
 * \param databuf: data pointer for storing entry information
 *
 * \return None
 */
static void cml_eth_macatable_rd(unsigned char *mac, char fid, char num, unsigned short *databuf)
{
	unsigned short idx;

	if(camelot_mac_mdio_read(camelot_mac_mii, 20, 13) & 0x8)
	{
		idx = fid ^ mac[0] ^ mac[1] ^ mac[2] ^ mac[3] ^ mac[4] ^ mac[5];
		if(mac[0] & 1)
			idx = idx<<2|num|(1<<11);
		else
			idx = idx<<2|num;
	}
	else
	{
		idx =	(fid<<3 | (mac[0]>>5)) ^ 
		((((short)mac[0]&0x1f) << 4)|((((short)mac[1]) >> 4))) ^  
		((((short)mac[1]&0xf) << 5)|((((short)mac[2]) >> 3))) ^
		((((short)mac[2]&0x7) << 6)|((((short)mac[3]) >> 2))) ^
		((((short)mac[3]&0x3) << 7)|((((short)mac[4]) >> 1))) ^
		((((short)mac[4]&0x1) << 8)|(((short)mac[5])));

		idx = idx<<2|num;
	}

	camelot_mac_mdio_write(camelot_mac_mii, 21, 14, idx|(2<<11)|(1<<15));
	while((camelot_mac_mdio_read(camelot_mac_mii, 21, 14) & 0x2000) == 0);

	databuf[0] = camelot_mac_mdio_read(camelot_mac_mii, 21, 15);
	databuf[1] = camelot_mac_mdio_read(camelot_mac_mii, 21, 16);
	databuf[2] = camelot_mac_mdio_read(camelot_mac_mii, 21, 17);
	databuf[3] = camelot_mac_mdio_read(camelot_mac_mii, 21, 18);
	databuf[4] = camelot_mac_mdio_read(camelot_mac_mii, 21, 19);
}

/*=============================================================*
 *  cml_eth_macatable_wr:
 *=============================================================*/
/*!
 * \brief Set the information to switch address table.
 *
 * \param mac: Pointer to a 6-byte array which contain the address
 * \param fid: the number of fid
 * \param num: entry number
 * \param info: reg 21.18
 * \param flag: reg 21.19
 *
 * \return None
 */
static void cml_eth_macatable_wr(unsigned char *mac, char fid, char num, unsigned short info, unsigned short flag)
{
	unsigned short idx;
	if(camelot_mac_mdio_read(camelot_mac_mii, 20, 13) & 0x8)
	{
		idx = fid ^ mac[0] ^ mac[1] ^ mac[2] ^ mac[3] ^ mac[4] ^ mac[5];
		if(mac[0] & 1)
			idx = idx<<2|num|(1<<11);
		else
			idx = idx<<2|num;
	}
	else
	{
		idx =	(fid<<3 | (mac[0]>>5)) ^ 
		((((short)mac[0]&0x1f) << 4)|((((short)mac[1]) >> 4))) ^  
		((((short)mac[1]&0xf) << 5)|((((short)mac[2]) >> 3))) ^
		((((short)mac[2]&0x7) << 6)|((((short)mac[3]) >> 2))) ^
		((((short)mac[3]&0x3) << 7)|((((short)mac[4]) >> 1))) ^
		((((short)mac[4]&0x1) << 8)|(((short)mac[5])));

		idx = idx<<2|num;
	}
	camelot_mac_mdio_write(camelot_mac_mii, 21, 15, *(unsigned short *)&mac[4]);
	camelot_mac_mdio_write(camelot_mac_mii, 21, 16, *(unsigned short *)&mac[2]);
	camelot_mac_mdio_write(camelot_mac_mii, 21, 17, *(unsigned short *)&mac[0]);
	camelot_mac_mdio_write(camelot_mac_mii, 21, 18, info);
	camelot_mac_mdio_write(camelot_mac_mii, 21, 19, flag);

	camelot_mac_mdio_write(camelot_mac_mii, 21, 14, idx|(1<<11)|(1<<15));
}

static void camelot_mac_hw_reset(void)
{
	writel(0xfff, MAC_RESET_REG);
	writel(0x0, MAC_RESET_REG);
}

static void camelot_mac_tx_desc_init(struct camelot_mac_tx_ctrl *ptxc)
{
	/* Clear TX descriptors */
	memset(ptxc->ptxd, 0, sizeof(*ptxc->ptxd) * NUM_TX_DESC);

	/* Mark end of ring */
	ptxc->ptxd[NUM_TX_DESC - 1].w0 = W0_END_OF_RING;

	ptxc->num_free_txd = NUM_TX_DESC;
	ptxc->tx_head = ptxc->tx_tail = 0;
}

static struct sk_buff *camelot_mac_get_skb(u32 buffer)
{
	struct sk_buff *skb = NULL;

	/*
	 * The SKB address is hidden 4 bytes
	 * behind the buffer address rounded
	 * down to 64 bytes
	 */
	skb = (struct sk_buff *)
		(*((u32 *)phys_to_virt((buffer & 0xffffffc0) - 4)));

	return skb;
}

static char *camelot_mac_get_buffer(struct net_device *dev)
{
	struct sk_buff *skb = NULL;

	skb = __netdev_alloc_skb(dev, CAMELOT_MAC_SKB_SIZE,
				GFP_ATOMIC | GFP_DMA);

	if (!skb) {
		printk(KERN_ERR "%s: out of memory for buffer\n", dev->name);
		return NULL;
	}

	/*
	 * skb->data is aligned on 32-byte by default, we add another
	 * 32 bytes so that they are aligned on 64-byte boundary
	 */
	skb_reserve(skb, 32);

	/* We hide the skb's address 4 bytes behind of skb->data */
	*((unsigned int *)(skb->data - 4)) = (unsigned int)skb;

	return skb->data;
}

static void camelot_mac_rx_desc_init(struct net_device *dev,
					struct camelot_mac_rx_ctrl *prxc)
{
	int i = 0;
	struct camelot_desc *prxd = prxc->prxd;

	for (i = 0; i < NUM_RX_DESC; i++, prxd++) {
		u8 *ptr = camelot_mac_get_buffer(dev);

		BUG_ON(!ptr);

		/*
		 * Tell DMA to write after the 4 bytes + NET_IP_ALIGN
		 * (The first 4 bytes contain the SKB address)
		 */
		prxd->w1 = dma_map_single(&dev->dev, ptr,
				CAMELOT_MAC_SKB_SIZE, DMA_FROM_DEVICE)
				+ CAMELOT_OFFSET + NET_IP_ALIGN;

		prxd->w0 = W0_RX_OWN;

		if (i == (NUM_RX_DESC - 1))
			prxd->w0 |= W0_END_OF_RING;
	}

	prxc->rx_head = 0;
}

static void camelot_mac_tx_desc_free(struct camelot_mac_tx_ctrl *ptxc)
{
	int i = 0;

	for (i = 0; i < NUM_TX_DESC; i++)
		if (ptxc->skb[i]) {
			dev_kfree_skb(ptxc->skb[i]);
			ptxc->skb[i] = NULL;
		}
}

static inline void camelot_mac_rx_start(void)
{
	writel(readl(MAC_CTRL_REG) | MAC_RX_ENABLE | MAC_RX_DMA_ENABLE,
		MAC_CTRL_REG);
}

static inline void camelot_mac_tx_start(void)
{
	writel(readl(MAC_CTRL_REG) | MAC_TX_ENABLE | MAC_TX_DMA_ENABLE,
		MAC_CTRL_REG);
}

static int camelot_mac_hw_buffer_init(struct net_device *dev)
{
	int i = 0, j = 0;
	u32 buffer_list[NUM_MBUF - 1];
	struct sk_buff *skb = NULL;
	void *current_buff = 0, *next_buff = 0;

	if (!(current_buff = camelot_mac_get_buffer(dev)))
		goto no_buffer;

	writel(virt_to_phys(current_buff) + CAMELOT_OFFSET,
		MAC_FIRST_BUFFER_REG);

	memset(buffer_list, 0, sizeof(buffer_list));

	for (i = 0; i < NUM_MBUF - 1; i++) {
		/* Save the current buffer */
		buffer_list[i] = (u32)current_buff;

		if (!(next_buff = camelot_mac_get_buffer(dev)))
			goto free;

		/*
		 * First 4 bytes of each area points to the address of the
		 * next buffer + 4 bytes (the SKB address)
		 */
#define TO_UNCAC(x)	(virt_to_phys((x)) | UNCAC_BASE)
		*(u32 *)(TO_UNCAC(current_buff) + CAMELOT_OFFSET)
			= virt_to_phys(next_buff) + CAMELOT_OFFSET;

		current_buff = next_buff;
	}

	/* Terminate the list */
	*(u32 *)(TO_UNCAC(current_buff) + CAMELOT_OFFSET) = 0;

	writel(virt_to_phys(current_buff) + CAMELOT_OFFSET,
		MAC_LAST_BUFFER_REG);

	return 0;
free:
	for (j = 0; j < i; j++) {
		if (buffer_list[j] == 0)
			break;

		skb = camelot_mac_get_skb(buffer_list[j]);
		dev_kfree_skb(skb);
	}
no_buffer:
	return -ENOMEM;
}

static int camelot_mac_hw_init(struct net_device *dev)
{
	struct camelot_mac_priv *priv = netdev_priv(dev);

	/* Preset switch */
	camelot_mac_switch_preset();

	/* Reset MAC */
	camelot_mac_hw_reset();

	/* Allocate memory for DMA access */
	priv->dma_ring = dma_alloc_coherent(&dev->dev,
				sizeof(*priv->rx.prxd) * (NUM_TOTAL_DESC),
				&priv->dma_handle, GFP_KERNEL);

	priv->tx.ptxd = (struct camelot_desc *)priv->dma_ring;
	priv->rx.prxd = (struct camelot_desc *)(priv->tx.ptxd + NUM_TX_DESC);

	if (camelot_mac_hw_buffer_init(dev))
		goto fail;

	/* Init all descriptors */	
	camelot_mac_tx_desc_init(&priv->tx);
	camelot_mac_rx_desc_init(dev, &priv->rx);

	writel((u32)priv->tx.ptxd, MAC_TX_DESC_REG);
	writel((u32)priv->rx.prxd, MAC_RX_DESC_REG);

	writel(MAC_CTRL_INIT, MAC_CTRL_REG);
	writel(MDROPS_INIT, MAC_DROPS_REG);
	
	/* Clear all pending interrupts */
	writel(readl(MAC_IRQ_STATUS_REG), MAC_IRQ_CLEAR_REG);

	/* Initialize switch */
	camelot_switch_init();	

	camelot_mac_mdio_write(camelot_mac_mii, 4, 5, 0x480);

	return 0;
fail:
	dma_free_coherent(&dev->dev,
		sizeof(*priv->rx.prxd) * (NUM_TOTAL_DESC),
		priv->dma_ring, priv->dma_handle);

	return -1;
}

static void camelot_mac_rx_stop(struct net_device *dev,
					struct camelot_mac_rx_ctrl *prxc)
{
	int i = 0;
	struct camelot_desc *prxd = NULL;
	struct sk_buff *skb = NULL;

	BUG_ON(!dev || !prxc);

	/* Disable RX MAC and DMA */
	writel(readl(MAC_CTRL_REG) & ~(MAC_RX_ENABLE | MAC_RX_DMA_ENABLE),
		MAC_CTRL_REG);

  	for (i = 0, prxd = prxc->prxd; i < NUM_RX_DESC; i++, prxd++)
		if (prxd->w0 & (W0_RX_OWN | W0_TX_OWN)) {
    			prxd->w0 &= ~(W0_RX_OWN | W0_TX_OWN);
      			prxd->w0 |= W0_RX_DROPPED;

			if (!(prxd->w1 & 0xffffff))
				/* Skip NULL addresses */
				continue;

			/*
			 * The original address that stores
			 * the SKB's address is the KSEG0
			 * value of the lower 24 bits of w1
			 * rounded down to the 64 bytes
			 */
			dma_unmap_single(&dev->dev, prxd->w1 & 0xffffc0,
					CAMELOT_MAC_SKB_SIZE, DMA_FROM_DEVICE);

			skb = camelot_mac_get_skb(prxd->w1 & 0xffffff);
			dev_kfree_skb(skb);
    		}
}

static void camelot_mac_tx_stop(struct camelot_mac_tx_ctrl *ptxc)
{
	int i = 0;
	struct camelot_desc *ptxd = ptxc->ptxd;

	/* Disable TX MAC and DMA */
	writel(readl(MAC_CTRL_REG) & ~(MAC_TX_ENABLE | MAC_TX_DMA_ENABLE),
		MAC_CTRL_REG);

  	for (i = 0; i < NUM_TX_DESC; i++, ptxd++)
		if (ptxd->w0 & (W0_RX_OWN | W0_TX_OWN)) {
    			ptxd->w0 &= ~(W0_RX_OWN | W0_TX_OWN);
      			ptxd->w0 |= W0_RX_DROPPED;
    		}
}	

static void camelot_mac_rx_irq(struct net_device *dev)
{
	int i = 0;
	u8 *ptr = NULL;
	struct sk_buff *skb = NULL;
	register u32 w0 = 0, w1 = 0;
	struct camelot_mac_priv *priv = netdev_priv(dev);
	struct camelot_mac_rx_ctrl *prxc = &priv->rx;
	struct camelot_desc *prxd = NULL;

	/* Get to the first buffer descriptor */
	prxd = &prxc->prxd[prxc->rx_head];
	w0 = prxd->w0;
	w1 = prxd->w1;

	while (!(w0 & (W0_RX_OWN)) && (i++ < NUM_RX_DESC)) {
		if (w0 & W0_RX_DROPPED) {
			dev->stats.rx_errors++;

			/* Check if they are crc errors */
			if (w0 & (W0_IP_CSUM_ERR | W0_L4_CSUM_ERR | W0_CRC_ERR))
				dev->stats.rx_crc_errors++;

			goto next;
		}

		dev->stats.rx_bytes += W0_FROM_PKT_LEN(w0);
	
		/* Check for multicast packets */	
		if (w0 & W0_MCAST_PKT) {
			dev->stats.multicast++;
		}

		dev->stats.rx_packets++;

		if ((ptr = camelot_mac_get_buffer(dev))) {
			if (!phys_to_virt(w1 & 0xffffff))
				goto next;

			/*
			 * Tell DMA to write after the 4 bytes + NET_IP_ALIGN
			 * (The first 4 bytes contain the SKB address)
			 */
			prxd->w1 = dma_map_single(&dev->dev, ptr,
					CAMELOT_MAC_SKB_SIZE, DMA_FROM_DEVICE)
					+ CAMELOT_OFFSET + NET_IP_ALIGN;

			/*
			 * The original address that stores
			 * the SKB's address is the KSEG0
			 * value of the lower 24 bits of w1
			 * rounded down to the 64 bytes
			 */
			dma_unmap_single(&dev->dev, w1 & 0xffffc0,
					CAMELOT_MAC_SKB_SIZE, DMA_FROM_DEVICE);

			skb = camelot_mac_get_skb(w1 & 0xffffff);

			/*
			 * The lower 24 bits of W1 has the physical address
			 * of the packet. Convert it to an address in KSEG0
			 * it to an address in KSEG0
			 */
			skb->data = skb->tail = phys_to_virt(w1 & 0xffffff);
			skb_put(skb, W0_FROM_PKT_LEN(w0));

#if 1
			/* OPTIMIZE */
			struct sk_buff *new_skb = netdev_alloc_skb(dev, 1518);

			BUG_ON(!new_skb);
			skb_reserve(new_skb, 2);
			memcpy(skb_put(new_skb, W0_FROM_PKT_LEN(w0)),
				skb->data, W0_FROM_PKT_LEN(w0));
			dev_kfree_skb_irq(skb);
			skb = new_skb;
#endif

			/* Setup the rest of the parameters */
			skb->dev = dev;
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			skb->protocol = eth_type_trans(skb, dev);

			if (unlikely(netif_msg_pktdata(priv))) {
				printk(KERN_DEBUG "%s: received pkt:\n",
					dev->name);
				dump_data(skb->data, skb->len);
			}

			/* Pass it up */
			netif_rx(skb);
		} else {
			/* Out of memory */
			if (netif_msg_rx_err(priv) && net_ratelimit())
				printk(KERN_WARNING "%s: low on memory, "
					"dropping packet\n", dev->name);

			dev->stats.rx_dropped++;
		}
next:
		/* Set ownership and end of ring bits */
		prxc->rx_head = (prxc->rx_head + 1) & (NUM_RX_DESC - 1);

		prxd->w0 = W0_RX_OWN;
		prxd->w0 |= (prxc->rx_head ? 0 : W0_END_OF_RING);

		/* Next buffer descriptor */
		prxd = &prxc->prxd[prxc->rx_head];
		w0 = prxd->w0;
		w1 = prxd->w1;
	}
}

static void camelot_mac_tx_irq(struct net_device *dev)
{
	struct camelot_mac_priv *priv = netdev_priv(dev);
	struct camelot_mac_tx_ctrl *ptxc = &priv->tx;
	struct camelot_desc *ptxd = ptxc->ptxd;
	
	while (!(ptxd[ptxc->tx_tail].w0 & W0_TX_OWN)
				&& (ptxc->num_free_txd < NUM_TX_DESC)) {
		if (ptxc->skb[ptxc->tx_tail]) {
			dev->stats.tx_packets++;
			dev->stats.tx_bytes += ptxc->skb[ptxc->tx_tail]->len;

			dma_unmap_single(&dev->dev,
				ptxd[ptxc->tx_tail].w1 & 0xffffff,
				CAMELOT_MAC_SKB_SIZE, DMA_TO_DEVICE);

			dev_kfree_skb_irq(ptxc->skb[ptxc->tx_tail]);
			ptxc->skb[ptxc->tx_tail] = NULL;
		}

		ptxc->num_free_txd++;
		ptxc->tx_tail = (ptxc->tx_tail + 1) & (NUM_TX_DESC - 1);
	}
}

static irqreturn_t camelot_mac_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	u32 status;

	/* Mask IRQ */
	writel(readl(MAC_IRQ_MASK_REG) | MAC_IRQ_MASK_ENABLE,
		MAC_IRQ_MASK_REG);

	/* Acknowledge all IRQs */
	status = readl(MAC_IRQ_STATUS_REG);
	writel(status, MAC_IRQ_CLEAR_REG);

	if (status & (MAC_IRQ_TX))
		camelot_mac_tx_irq(dev);

	if (status & (MAC_IRQ_RX))
		camelot_mac_rx_irq(dev);

	if (status & (1 << 4))
		printk(KERN_INFO "BE IRQ\n");

	if (status & (1 << 2))
		printk(KERN_INFO "SDE IRQ\n");

	/* Un-mask IRQ */
	writel(readl(MAC_IRQ_MASK_REG) & ~MAC_IRQ_MASK_ENABLE,
		MAC_IRQ_MASK_REG);

	return IRQ_HANDLED;
}

static int camelot_mac_open(struct net_device *dev)
{
	/* Initialize Camelot MAC hardware */
	if (camelot_mac_hw_init(dev)) {
		printk(KERN_ERR "%s: Failed to init hw\n", dev->name);
		return -ENODEV;
	}

	/* Request IRQ */
	if (request_irq(IRQ_ETHERNET, camelot_mac_isr,
			IRQF_DISABLED | IRQF_SAMPLE_RANDOM,
			"camelot_mac", dev)) {
		printk(KERN_ERR "%s: Failed to request irq\n", dev->name);
		return -ENODEV;
	}

	camelot_mac_set_macaddr(dev->dev_addr);

	camelot_mac_rx_start();
	camelot_mac_tx_start();

	/* Un-mask IRQ */
	writel(readl(MAC_IRQ_MASK_REG) & ~MAC_IRQ_MASK_ENABLE,
		MAC_IRQ_MASK_REG);

	netif_start_queue(dev);

	return 0;
}

static int camelot_mac_stop(struct net_device *dev)
{
	struct camelot_mac_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);

	/* Mask IRQ */
	writel(readl(MAC_IRQ_MASK_REG) | MAC_IRQ_MASK_ENABLE,
		MAC_IRQ_MASK_REG);

	camelot_mac_tx_stop(&priv->tx);
	camelot_mac_rx_stop(dev, &priv->rx);
	camelot_mac_tx_desc_free(&priv->tx);

	writel(0, MAC_TX_DESC_REG);
	writel(0, MAC_RX_DESC_REG);

	/* Free IRQ */
	free_irq(dev->irq, dev);

	/* Free DMA ring */
	dma_free_coherent(&dev->dev,
		sizeof(*priv->rx.prxd) * (NUM_TOTAL_DESC),
		priv->dma_ring, priv->dma_handle);
	
	return 0;
}

static int camelot_mac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int len = 0;
	struct sk_buff *new_skb = NULL;
	struct camelot_mac_priv *priv = netdev_priv(dev);
	struct camelot_mac_tx_ctrl *ptxc = &priv->tx;
	struct camelot_desc *ptxd = NULL;

	if (unlikely(skb_padto(skb, ETH_ZLEN)))
		return NETDEV_TX_OK;

	len = max(skb->len, (unsigned int)ETH_ZLEN);

	if (ptxc->num_free_txd == 0) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			printk(KERN_WARNING "%s: tx dma ring full\n",
				dev->name);
		return NETDEV_TX_BUSY;
	}

	if ((virt_to_phys(skb->data) + skb->len) >= 0x1000000) {
		/*
		 * FIXME: The Ethernet DMA can only address up to 24 bits
		 * (= 16 MB) so we need to do a skb_copy here to remap it to
		 * a memory region the DMA can address
		 */
		if (!(new_skb = skb_copy(skb, GFP_KERNEL | GFP_DMA))) {
			if (netif_msg_tx_err(priv) && net_ratelimit())
				printk(KERN_WARNING "%s: unable to copy skb "
					" into dma region\n", dev->name);
			return NETDEV_TX_BUSY;
		}

		dev_kfree_skb(skb);
		skb = new_skb;
	}

	if (unlikely(netif_msg_pktdata(priv)))
		dump_data(skb->data, len);

	dev->trans_start = jiffies;

	/* Look for a free slot */	
	ptxd = &ptxc->ptxd[ptxc->tx_head];

	ptxd->w1 = dma_map_single(&dev->dev, skb->data, len,
				DMA_TO_DEVICE);

	ptxd->w0 &= W0_END_OF_RING;
	ptxd->w0 |= W0_TO_PKT_LEN(len) | W0_TX_OWN | W0_TX_DIRECT;

	/* Start transmission */
	writel(0x01, MAC_START_TX_REG);

	/* Remember the SKB so we can free it later */
	ptxc->skb[ptxc->tx_head] = skb;
	ptxc->num_free_txd--;

	/* Advance TX head */
	ptxc->tx_head = (ptxc->tx_head + 1) & (NUM_TX_DESC - 1);
	
	return NETDEV_TX_OK;
}

static int camelot_mac_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct mii_ioctl_data *ptr = NULL;

	ptr = (struct mii_ioctl_data *)(&ifr->ifr_data);

	switch (cmd) {
	case SIOCGMIIPHY:
		ptr->phy_id = CAMELOT_MAC_PHY_ID;
		break;
	case SIOCGMIIREG:
		ptr->val_out = camelot_mac_mdio_read(camelot_mac_mii,
				ptr->phy_id, ptr->reg_num);
		break;
	case SIOCSMIIREG:
		camelot_mac_mdio_write(camelot_mac_mii, ptr->phy_id,
				ptr->reg_num, ptr->val_in);
		break;
        default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void camelot_mac_set_multicast_list(struct net_device *dev)
{
	if (dev->flags & IFF_PROMISC) {
		writel(readl(MAC_DROPS_REG) & ~(1 << 7), MAC_DROPS_REG);
	} else {
		writel(readl(MAC_DROPS_REG) | (1 << 7), MAC_DROPS_REG);
	}
}

static const struct net_device_ops camelot_mac_netdev_ops = {
	.ndo_open		= camelot_mac_open,
	.ndo_stop		= camelot_mac_stop,
	.ndo_start_xmit		= camelot_mac_start_xmit,
	.ndo_do_ioctl		= camelot_mac_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_multicast_list	= camelot_mac_set_multicast_list
};

static int __devinit camelot_mac_probe(struct platform_device *pdev)
{
	int rc = -1;
	struct camelot_mac_priv *priv = NULL;
	struct net_device *dev = NULL;
	const u8 mac_addr[6] = CAMELOT_MAC_DEFAULT_ADDRESS;

	/* Allocate net_device */
	if (!(dev = alloc_etherdev(sizeof(*priv)))) {
		printk(KERN_INFO "%s: Unable to allocate net_device\n",
							DRV_NAME);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, dev);
	priv = netdev_priv(dev);

	/* Set up IO base, IRQ, and netdev_ops */
	/* TODO: base_addr and IRQ should be supplied from platform device */
	dev->base_addr = (unsigned long)MAC_BASE;
	dev->irq = IRQ_ETHERNET;
	dev->netdev_ops	= &camelot_mac_netdev_ops;

	/* Register net_device */
	if ((rc = register_netdev(dev))) {
		printk(KERN_ERR "%s: Error registering device %s",
					DRV_NAME, dev->name);
		goto fail;
	}

	/* Setup priv structure */
	priv->dev = dev;

	priv->msg_enable = netif_msg_init(debug_level, 0xff);

	/* Use a default MAC */
	memcpy(dev->dev_addr, mac_addr, ETH_ALEN);

	if (netif_msg_probe(priv))
		printk(KERN_INFO "%s: device %s (mac: %pM)\n",
			DRV_NAME, dev->name, dev->dev_addr);

	return 0;
fail:
	free_netdev(dev);

	return rc;
}

static int __devexit camelot_mac_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	
	unregister_netdev(dev);
	free_netdev(dev);

	return 0;
}

static struct platform_driver camelot_mac_driver = {
	.driver.name = "camelot_mac",
	.driver.owner = THIS_MODULE,
	.probe = camelot_mac_probe,
	.remove = __devexit_p(camelot_mac_remove)
};

static int __init camelot_mac_init(void)
{
	int res = 0, i = 0;

	if (!(camelot_mac_mii = mdiobus_alloc()))
		return -ENOMEM;

	camelot_mac_mii->name = "camelot_mac_mii";
	camelot_mac_mii->read = camelot_mac_mdio_read;
	camelot_mac_mii->write = camelot_mac_mdio_write;
	camelot_mac_mii->reset = camelot_mac_mdio_reset;
	camelot_mac_mii->irq = mdio_irqs;

	camelot_mac_mii->phy_mask = 0xffffffff;

	/* Ports 0 through 5 are valid */
	for (i = 0; i <= 5; i++)
		camelot_mac_mii->phy_mask &= ~(1 << i);

	snprintf(camelot_mac_mii->id, MII_BUS_ID_SIZE, CAMELOT_MAC_PHY_BUS);
	
	res = mdiobus_register(camelot_mac_mii);

	if (res)
		return res;

	res = platform_driver_register(&camelot_mac_driver);

	if (res)
		return res;

	return 0;
}

static void __devexit camelot_mac_exit(void)
{
	platform_driver_unregister(&camelot_mac_driver);
}

module_init(camelot_mac_init);
module_exit(camelot_mac_exit);
