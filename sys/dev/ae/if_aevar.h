/*-
 * Copyright (c) 2008 Stanislav Sedov <stas@FreeBSD.org>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: releng/11.1/sys/dev/ae/if_aevar.h 216925 2011-01-03 18:28:30Z jhb $
 */
/*
 * Copyright (c) 2018 Henning Matyschok
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IF_AEVAR_H
#define IF_AEVAR_H

#include "opt_netgraph.h"

#ifdef NETGRAPH
#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_parse.h>
#endif /* NETGRAPH */

/*
 * Supported chips identifiers.
*/
#define	VENDORID_ATTANSIC	0x1969
#define	DEVICEID_ATTANSIC_L2	0x2048

/* How much to wait for reset to complete (10 microsecond units). */
#define	AE_RESET_TIMEOUT	100

/* How much to wait for device to enter idle state (100 microsecond units). */
#define	AE_IDLE_TIMEOUT		100

/* How much to wait for MDIO to do the work (2 microsecond units). */
#define	AE_MDIO_TIMEOUT		10

/* How much to wait for VPD reading operation to complete (2 ms units). */
#define AE_VPD_TIMEOUT		10

/* How much to wait for send operation to complete (HZ units). */
#define	AE_TX_TIMEOUT		5

/* Default PHY address. */
#define	AE_PHYADDR_DEFAULT	0

/* Tx packet descriptor header format. */
typedef struct ae_txd {
	uint16_t	ae_len;
	uint16_t	ae_vlan;
} __packed ae_txd_t;

/* Tx status descriptor format. */
typedef struct ae_txs {
	uint16_t	ae_len;
	uint16_t	ae_flags;
} __packed ae_txs_t;

/* Rx packet descriptor format. */
typedef struct ae_rxd {
	uint16_t	ae_len;
	uint16_t	ae_flags;
	uint16_t	ae_vlan;
	uint16_t	__pad;
	uint8_t		ae_data[1528];
} __packed ae_rxd_t;

/* Statistics. */
typedef struct ae_stats {
	uint32_t	ae_rx_bcast;
	uint32_t	ae_rx_mcast;
	uint32_t	ae_rx_pause;
	uint32_t	ae_rx_ctrl;
	uint32_t	ae_rx_crcerr;
	uint32_t	ae_rx_codeerr;
	uint32_t	ae_rx_runt;
	uint32_t	ae_rx_frag;
	uint32_t	ae_rx_trunc;
	uint32_t	ae_rx_align;
	uint32_t	ae_tx_bcast;
	uint32_t	ae_tx_mcast;
	uint32_t	ae_tx_pause;
	uint32_t	ae_tx_ctrl;
	uint32_t	ae_tx_defer;
	uint32_t	ae_tx_excdefer;
	uint32_t	ae_tx_singlecol;
	uint32_t	ae_tx_multicol;
	uint32_t	ae_tx_latecol;
	uint32_t	ae_tx_abortcol;
	uint32_t	ae_tx_underrun;
} ae_stats_t;

/* Software state structure. */
typedef struct ae_softc	{
	struct ifnet		*ae_ifp;
	device_t		ae_dev;
	device_t		ae_miibus;
	struct resource		*ae_mem[1];
	struct resource_spec	*ae_spec_mem;
	struct resource		*ae_irq[1];
	struct resource_spec	*ae_spec_irq;
	void			*ae_intrhand;

	struct mtx		ae_mtx;

	uint8_t			ae_eaddr[ETHER_ADDR_LEN];
	uint8_t			ae_flags;
	int			ae_if_flags;

	struct callout		ae_tick_ch;

	/* Tasks. */
	struct task		ae_int_task;
	struct task		ae_link_task;
	struct taskqueue	*ae_tq;
	
	/* DMA tags. */
	bus_dma_tag_t		ae_dma_parent_tag;
	bus_dma_tag_t		ae_dma_rxd_tag;
	bus_dma_tag_t		ae_dma_txd_tag;
	bus_dma_tag_t		ae_dma_txs_tag;
	bus_dmamap_t		ae_dma_rxd_map;
	bus_dmamap_t		ae_dma_txd_map;
	bus_dmamap_t		ae_dma_txs_map;

	bus_addr_t		ae_dma_rxd_busaddr;
	bus_addr_t		ae_dma_txd_busaddr;
	bus_addr_t		ae_dma_txs_busaddr;
	
	char			*ae_rxd_base_dma;	/* Start of allocated area. */
	ae_rxd_t		*ae_rxd_base;	/* Start of RxD ring. */
	char			*ae_txd_base;	/* Start of TxD ring. */
	ae_txs_t		*ae_txs_base;	/* Start of TxS ring. */

	/* Ring pointers. */
	unsigned int		ae_rxd_cur;
	unsigned int		ae_txd_cur;
	unsigned int		ae_txs_cur;
	unsigned int		ae_txs_ack;
	unsigned int		ae_txd_ack;

	int			ae_tx_inproc;	/* Active Tx frames in ring. */
	int			ae_wd_timer;

	ae_stats_t		ae_stats;
	
#ifdef NETGRAPH
	node_p 	ae_tap_node;
	hook_p 	ae_tap_hook;
#endif /* NETGRAPH */		
} ae_softc_t;

#define	AE_LOCK(_sc)		mtx_lock(&(_sc)->ae_mtx)
#define	AE_UNLOCK(_sc)		mtx_unlock(&(_sc)->ae_mtx)
#define	AE_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->ae_mtx, MA_OWNED)

#define	BUS_ADDR_LO(x)		((uint64_t) (x) & 0xFFFFFFFF)
#define	BUS_ADDR_HI(x)		((uint64_t) (x) >> 32)

#define	AE_FLAG_LINK		0x01	/* Has link. */
#define	AE_FLAG_DETACH		0x02	/* Is detaching. */
#define	AE_FLAG_TXAVAIL		0x04	/* Tx'es available. */
#define	AE_FLAG_MSI		0x08	/* Using MSI. */
#define	AE_FLAG_PMG		0x10	/* Supports PCI power management. */

#endif	/* IF_AEVAR_H */
