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
#ifndef __CAMELOT_ETHERNET_H
#define __CAMELOT_ETHERNET_H

#include <linux/if_ether.h>
#include <linux/phy.h>

#define NUM_TX_DESC		64	/* TX Buffer Size */
#define NUM_RX_DESC		64	/* RX Buffer Size */

#define NUM_TOTAL_DESC		(NUM_TX_DESC + NUM_RX_DESC)

/* TX and RX descriptor format */
struct camelot_desc {
	u32 w0;
	u32 w1;
} __attribute__((__packed__));

/* Receive control structure */
struct camelot_mac_rx_ctrl {
	struct camelot_desc *prxd;
	unsigned short rx_head;
	struct sk_buff *skb[NUM_RX_DESC];
} __attribute__((__packed__));

/* Transmit control structure */
struct camelot_mac_tx_ctrl {
	struct camelot_desc *ptxd;
	unsigned short num_free_txd;
	unsigned short tx_head;
	unsigned short tx_tail;
	struct sk_buff *skb[NUM_TX_DESC];
} __attribute__((__packed__));

struct camelot_mac_priv {
	struct net_device *dev;

	u32 msg_enable;

	struct camelot_mac_tx_ctrl tx;
	struct camelot_mac_rx_ctrl rx;
	void *dma_ring;
	dma_addr_t dma_handle;
} __attribute__((__packed__));

#define W0_RX_OWN		(1 << 31)
#define W0_TX_OWN		(1 << 30)
#define W0_END_OF_RING		(1 << 29)
#define W0_RX_DROPPED		(1 << 27)
#define W0_TX_DIRECT		(1 << 27)	/* Send without ARP lookup */
#define W0_BCAST_PKT		(1 << 11)	/* Broadcast packet */
#define W0_MCAST_PKT		(1 << 10)	/* Multicast packet */
#define W0_CRC_ERR		(1 << 3)	/* Ethernet FCS error */
#define W0_L4_CSUM_ERR		(1 << 2)	/* TCP/UDP checksum error */
#define W0_IP_CSUM_ERR		(1 << 1)	/* IP checksum error */
#define W0_FROM_PKT_LEN(x)	(((x) >> 16) & 0x7ff)
#define W0_TO_PKT_LEN(x)	((x) << 16)

#endif
