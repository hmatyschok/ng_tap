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

/*
 * Tihs netgraph(4) node operates in conjunction with ng_tap(4) node.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>

#include <net/ethernet.h>

#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_fcs.h>

/* Private data */
struct ng_fcs_priv {
	node_p	nfp_node;		/* back pointer to node */
	hook_p 	nfp_raw; 		/* Service Access Point */
	hook_p 	nfp_log;
};
typedef struct ng_fcs_priv *nfp_p;

/* Private methods */
static struct mbuf *	ng_fcs_get_trailer(struct mbuf *m0); 
static struct mbuf * 	ng_fcs_prepend_crc(struct mbuf *m0, 
	struct mbuf *m);
static struct mbuf * 	ng_fcs_prepend_eh(struct mbuf *m0, 
	struct mbuf *m);

static int	ng_fcs_rcv_raw(hook_p node, item_p item);
static int	ng_fcs_rcv_log(hook_p node, item_p item);

/* Public methods */
static ng_constructor_t 	ng_fcs_constructor;
static ng_newhook_t 	ng_fcs_newhook;
static ng_rcvmsg_t 	ng_fcs_rcvmsg;
static ng_rcvdata_t 	ng_fcs_rcvdata;
static ng_disconnect_t 	ng_fcs_disconnect;
static ng_shutdown_t 	ng_fcs_shutdown;

/* Netgraph type */
static struct ng_type ng_fcs_type = {
	.version =	NG_ABI_VERSION,
	.name =		NG_FCS_NODE_TYPE,
	.constructor =	ng_fcs_constructor,
	.newhook =	ng_fcs_newhook, 
	.rcvmsg =	ng_fcs_rcvmsg,
	.rcvdata =	ng_fcs_rcvdata,
	.disconnect =	ng_fcs_disconnect,
	.shutdown = 	ng_fcs_shutdown,
};
NETGRAPH_INIT(fcs, &ng_fcs_type);

/*
 * Ctor.
 */
static int
ng_fcs_constructor(node_p node)
{
	nfp_p nfp;

	nfp = malloc(sizeof(*nfp), M_NETGRAPH, M_WAITOK|M_ZERO);

	NG_NODE_SET_PRIVATE(node, nfp);
	nfp->nfp_node = node;

	return (0);
}

/*
 * Interconnect with peer node by referring its hook denotes SAP.
 * 
 *  (a) Accessing the raw data stream from peered ng_tap(4) node.
 * 
 *  (b) Forwarding from raw data stream collected information. 
 */
static int
ng_fcs_newhook(node_p node, hook_p hook, const char *name)
{
	const nfp_p nfp = NG_NODE_PRIVATE(node);
	hook_p *hp;
	int error;
	
	if (strcmp(name, NG_FCS_HOOK_RAW) == 0) {
		hp = &nfp->nfp_raw;
		NG_HOOK_SET_RCVDATA(hook, ng_fcs_rcv_raw);
	} else if (strcmp(name, NG_FCS_HOOK_LOG) == 0) {	
		hp = &nfp->nfp_log;
		NG_HOOK_SET_RCVDATA(hook, ng_fcs_rcv_log);
	} else {
		error = EPFNOSUPPORT;
		goto out;
	}	
		
	if (*hp != NULL)
		error = EISCONN;
	else {
		*hp = hook;
		error = 0;
	}
out:
	return (error);
}

/*
 * Receive control message and bounce it back as a reply.
 */
static int
ng_fcs_rcvmsg(node_p node, item_p item, hook_p lasthook)
{
	struct ng_mesg *msg;
	int error;

	NGI_GET_MSG(item, msg);
	msg->header.flags |= NGF_RESP;
	NG_RESPOND_MSG(error, node, item, msg);

	return (0);
}

/*
 * Generic data sink, because each hook has its own service primitive.
 */
static int
ng_fcs_rcvdata(hook_p hook, item_p item)
{
	NG_FREE_ITEM(item);
	
	return (0);
}

/*
 * Removal of the last link destroys the node.
 */
static int
ng_fcs_disconnect(hook_p hook)
{
	const nfp_p nfp = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));

	if (nfp->nfp_raw == hook)
		nfp->nfp_raw = NULL;
	else if (nfp->nfp_log == hook)
		nfp->nfp_log = NULL;
	
	if ((NG_NODE_NUMHOOKS(NG_HOOK_NODE(hook)) == 0)
	&& (NG_NODE_IS_VALID(NG_HOOK_NODE(hook)))) {
		ng_rmnode_self(NG_HOOK_NODE(hook));
	}
	return (0);
}

/*
 * Do local shutdown processing..
 */
static int
ng_fcs_shutdown(node_p node)
{
	const nfp_p nfp = NG_NODE_PRIVATE(node);

	NG_NODE_SET_PRIVATE(node, NULL);
	NG_NODE_UNREF(nfp->nfp_node);
	free(nfp, M_NETGRAPH);

	return (0);
}

/*
 * Fetch trailer.
 */
static struct mbuf *
ng_fcs_get_trailer(struct mbuf *m0)
{
	struct mbuf *t, *m;
	int off; 
	
	t = m = m0;

	off = m0->m_pkthdr.len - ETHER_CRC_LEN;
	
	if ((m = m_split(m0, off, M_NOWAIT)) != NULL) {
/*
 * Ensure that FCS won't share same cluster mbuf(9), if any.
 */
 		if (!M_WRITABLE(m)) { 
			t = m_dup(m, M_NOWAIT);
			m_freem(m);
		} else
			t = m;
	} else
		t = m;
	
	if (t != NULL)
		m0->m_flags &= ~M_HASFCS;
		
	return (t);
}

/*
 * Append re-calculated CRC-32 based FCS.
 */
static struct mbuf * 
ng_fcs_prepend_crc(struct mbuf *m0, struct mbuf *m)
{
	char buf[ETHER_MAX_LEN_JUMBO];
	uint32_t val;
	uint32_t *crc;
	
	m_copydata(m, 0, m->m_pkthdr.len, buf);
	
	val = ether_crc32_le(buf, m->m_pkthdr.len);
	val = ~val;

	M_PREPEND(m0, ETHER_CRC_LEN, M_NOWAIT);
	if (m0 != NULL) {
		crc = mtod(m0, uint32_t *);
		(void)memcpy(crc, &val, ETHER_CRC_LEN); 
	}
	return (m0);
}

/*
 * Append Ethernet Protocol Control Information.
 */
static struct mbuf * 
ng_fcs_prepend_eh(struct mbuf *m0, struct mbuf *m)
{
	struct ether_header *eh;
	
	M_PREPEND(m0, sizeof(struct ether_header), M_NOWAIT);
	if (m0 != NULL) {
		eh = mtod(m0, struct ether_header *);
		m_copydata(m, 0, sizeof(struct ether_header), (caddr_t)eh);
	}
	return (m0);
}

/*
 * Receive data, unlink its FCS from trailer and re-inject upstream.
 */
static int
ng_fcs_rcv_raw(hook_p hook, item_p item)
{
	const nfp_p nfp = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));
	struct mbuf *m, *n;
	int error;

	NGI_GET_M(item, m);
    
	if ((m->m_flags & M_HASFCS) == 0) {
		error = EINVAL;
		goto bad1;	
	}
       
	if (m->m_pkthdr.len > ETHER_MAX_LEN_JUMBO) {
		error = EINVAL;
		goto bad1;
	}   
        
	if (m->m_pkthdr.len < sizeof(struct ether_header)) {
		error = EINVAL;
		goto bad1;
	}
 
	if (m->m_len < sizeof(struct ether_header)) {
		m = m_pullup(m, sizeof(struct ether_header));
		if (m == NULL) {
			error = ENOBUFS;
			goto bad;
		}
	}
/*
 * Collect neccessary data 
 * 
 *  t := ( ether_header, fcs1, fcs0 )
 * 
 * where
 * 
 *  (a) ether_header := ( ether_dst, ether_src, ether_type ) 
 * 
 *  (b) fcs1 := recalculated CRC-32 based FCS
 * 
 *  (c) fcs0 := maps to trailer  
 * 
 * and tx for further processing.
 */	
	if ((n = ng_fcs_get_trailer(m)) == NULL) {
		error = ENOBUFS;
		goto bad1;
	}
	
	if ((n = ng_fcs_prepend_crc(n, m)) == NULL) {
		error = ENOBUFS;
		goto bad2;
	}
	
	if ((n = ng_fcs_prepend_eh(n, m)) == NULL) {
		error = ENOBUFS;
		goto bad2;
	}
	NG_SEND_DATA_ONLY(error, nfp->nfp_log, n);
/*
 * Re-inject upstream.
 */		
	NG_FWD_NEW_DATA(error, item, nfp->nfp_raw, m);
out:	
	return (error);
bad2:
	NG_FREE_M(n);
bad1:
	NG_FREE_M(m);
bad:
	NG_FREE_ITEM(item);
	goto out;
}

/*
 * Data sink.
 */
static int
ng_fcs_rcv_log(hook_p hook, item_p item)
{
	NG_FREE_ITEM(item);
	
	return (0);
}
