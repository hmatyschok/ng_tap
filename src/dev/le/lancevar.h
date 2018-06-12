/*	$NetBSD: lancevar.h,v 1.10 2005/12/11 12:21:27 christos Exp $	*/

/*-
 * Copyright (c) 1997, 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Charles M. Hannum and by Jason R. Thorpe of the Numerical Aerospace
 * Simulation Facility, NASA Ames Research Center.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "opt_netgraph.h"

#ifdef NETGRAPH
#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_parse.h>
#endif

/* $FreeBSD: releng/11.1/sys/dev/le/lancevar.h 204646 2010-03-03 17:55:51Z joel $ */

#ifndef _DEV_LE_LANCEVAR_H_
#define	_DEV_LE_LANCEVAR_H_

extern devclass_t le_devclass;

struct lance_softc {
	struct ifnet	*le_ifp;
	struct ifmedia	le_media;
	struct mtx	le_mtx;
	struct callout	le_wdog_ch;
	int		le_wdog_timer;

	/*
	 * Memory functions:
	 *
	 *	copy to/from descriptor
	 *	copy to/from buffer
	 *	zero bytes in buffer
	 */
	void	(*le_copytodesc)(struct lance_softc *, void *, int, int);
	void	(*le_copyfromdesc)(struct lance_softc *, void *, int, int);
	void	(*le_copytobuf)(struct lance_softc *, void *, int, int);
	void	(*le_copyfrombuf)(struct lance_softc *, void *, int, int);
	void	(*le_zerobuf)(struct lance_softc *, int, int);

	/*
	 * Machine-dependent functions:
	 *
	 *	read/write CSR
	 *	hardware reset hook - may be NULL
	 *	hardware init hook - may be NULL
	 *	no carrier hook - may be NULL
	 *	media change hook - may be NULL
	 */
	uint16_t	(*le_rdcsr)(struct lance_softc *, uint16_t);
	void	(*le_wrcsr)(struct lance_softc *, uint16_t, uint16_t);
	void	(*le_hwreset)(struct lance_softc *);
	void	(*le_hwinit)(struct lance_softc *);
	int	(*le_hwintr)(struct lance_softc *);
	void	(*le_nocarrier)(struct lance_softc *);
	int	(*le_mediachange)(struct lance_softc *);
	void	(*le_mediastatus)(struct lance_softc *, struct ifmediareq *);

	/*
	 * Media-supported by this interface.  If this is NULL,
	 * the only supported media is assumed to be "manual".
	 */
	const int	*le_supmedia;
	int	le_nsupmedia;
	int	le_defaultmedia;

	uint16_t	le_conf3;	/* CSR3 value */

	void	*le_mem;		/* base address of RAM - CPU's view */
	bus_addr_t	le_addr;	/* base address of RAM - LANCE's view */

	bus_size_t	le_memsize;	/* size of RAM */

	int	le_nrbuf;	/* number of receive buffers */
	int	le_ntbuf;	/* number of transmit buffers */
	int	le_last_rd;
	int	le_first_td;
	int	le_last_td;
	int	le_no_td;

	int	le_initaddr;
	int	le_rmdaddr;
	int	le_tmdaddr;
	int	le_rbufaddr;
	int	le_tbufaddr;

	uint8_t	le_enaddr[ETHER_ADDR_LEN];

	void	(*le_meminit)(struct lance_softc *);
	void	(*le_start_locked)(struct lance_softc *);

	int	le_flags;
#define	LE_ALLMULTI	(1 << 0)
#define	LE_BSWAP	(1 << 1)
#define	LE_CARRIER	(1 << 2)
#define	LE_DEBUG	(1 << 3)
#define	LE_PROMISC	(1 << 4)

#ifdef NETGRAPH
	node_p 	le_tap_node;
	hook_p 	le_tap_hook;
#endif /* NETGRAPH */
};

#define	LE_LOCK_INIT(_sc, _name)					\
	mtx_init(&(_sc)->le_mtx, _name, MTX_NETWORK_LOCK, MTX_DEF)
#define	LE_LOCK_INITIALIZED(_sc)	mtx_initialized(&(_sc)->le_mtx)
#define	LE_LOCK(_sc)			mtx_lock(&(_sc)->le_mtx)
#define	LE_UNLOCK(_sc)			mtx_unlock(&(_sc)->le_mtx)
#define	LE_LOCK_ASSERT(_sc, _what)	mtx_assert(&(_sc)->le_mtx, (_what))
#define	LE_LOCK_DESTROY(_sc)		mtx_destroy(&(_sc)->le_mtx)

/*
 * Unfortunately, manual byte swapping is only necessary for the PCnet-PCI
 * variants but not for the original LANCE or ILACC so we cannot do this
 * with #ifdefs resolved at compile time.
 */
#define	LE_HTOLE16(v)	(((sc)->le_flags & LE_BSWAP) ? htole16(v) : (v))
#define	LE_HTOLE32(v)	(((sc)->le_flags & LE_BSWAP) ? htole32(v) : (v))
#define	LE_LE16TOH(v)	(((sc)->le_flags & LE_BSWAP) ? le16toh(v) : (v))
#define	LE_LE32TOH(v)	(((sc)->le_flags & LE_BSWAP) ? le32toh(v) : (v))

int lance_config(struct lance_softc *, const char*, int);
int lance_attach(struct lance_softc *);
void lance_detach(struct lance_softc *);
void lance_suspend(struct lance_softc *);
void lance_resume(struct lance_softc *);
void lance_init_locked(struct lance_softc *);
int lance_put(struct lance_softc *, int, struct mbuf *);
struct mbuf *lance_get(struct lance_softc *, int, int);
void lance_setladrf(struct lance_softc *, u_int16_t *);

/*
 * The following functions are only useful on certain CPU/bus
 * combinations.  They should be written in assembly language for
 * maximum efficiency, but machine-independent versions are provided
 * for drivers that have not yet been optimized.
 */
void lance_copytobuf_contig(struct lance_softc *, void *, int, int);
void lance_copyfrombuf_contig(struct lance_softc *, void *, int, int);
void lance_zerobuf_contig(struct lance_softc *, int, int);

#if 0	/* Example only - see lance.c */
void lance_copytobuf_gap2(struct lance_softc *, void *, int, int);
void lance_copyfrombuf_gap2(struct lance_softc *, void *, int, int);
void lance_zerobuf_gap2(struct lance_softc *, int, int);

void lance_copytobuf_gap16(struct lance_softc *, void *, int, int);
void lance_copyfrombuf_gap16(struct lance_softc *, void *, int, int);
void lance_zerobuf_gap16(struct lance_softc *, int, int);
#endif /* Example only */

/*
 * Compare two Ether/802 addresses for equality, inlined and
 * unrolled for speed.  Use this like memcmp().
 *
 * XXX: Add <machine/inlines.h> for stuff like this?
 * XXX: or maybe add it to libkern.h instead?
 *
 * "I'd love to have an inline assembler version of this."
 * XXX: Who wanted that? mycroft?  I wrote one, but this
 * version in C is as good as hand-coded assembly. -gwr
 *
 * Please do NOT tweak this without looking at the actual
 * assembly code generated before and after your tweaks!
 */
static inline uint16_t
ether_cmp(void *one, void *two)
{
	uint16_t *a = (u_short *)one;
	uint16_t *b = (u_short *)two;
	uint16_t diff;

#ifdef	m68k
	/*
	 * The post-increment-pointer form produces the best
	 * machine code for m68k.  This was carefully tuned
	 * so it compiles to just 8 short (2-byte) op-codes!
	 */
	diff  = *a++ - *b++;
	diff |= *a++ - *b++;
	diff |= *a++ - *b++;
#else
	/*
	 * Most modern CPUs do better with a single expresion.
	 * Note that short-cut evaluation is NOT helpful here,
	 * because it just makes the code longer, not faster!
	 */
	diff = (a[0] - b[0]) | (a[1] - b[1]) | (a[2] - b[2]);
#endif

	return (diff);
}

#endif /* _DEV_LE_LANCEVAR_H_ */
