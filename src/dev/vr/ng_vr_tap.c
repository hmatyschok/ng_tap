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
 
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/socket.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>

#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_parse.h>
#include <dev/vr/if_vrreg.h>
#include <dev/vr/ng_vr_tap.h>

static const struct ng_cmdlist vr_tap_cmdlist[] = {
	{
	  NGM_VR_TAP_COOKIE,
	  NGM_VR_TAP_GET_IFNAME,
	  "getifname",
	  NULL,
	  &ng_parse_string_type
	},
	{ 0 }
};

/*
 * Declarations for netgraph(9) methods.
 */

static ng_constructor_t	vr_tap_constructor;
static ng_rcvmsg_t	vr_tap_rcvmsg;
static ng_shutdown_t	vr_tap_shutdown;
static ng_newhook_t	vr_tap_newhook;
static ng_rcvdata_t	vr_tap_rcvdata;	 
static ng_connect_t	vr_tap_connect;
static ng_disconnect_t	vr_tap_disconnect;

/*
 * Implements netgraph(4) node types. 
 */

static struct ng_type vr_tap_type = {
	.version =	NG_ABI_VERSION,
	.name =		NG_VR_TAP_NODE_TYPE,
	.mod_event =	NULL,
	.constructor =	vr_tap_constructor,
	.rcvmsg =	vr_tap_rcvmsg,
	.shutdown =	vr_tap_shutdown,
	.newhook =	vr_tap_newhook,
	.rcvdata =	vr_tap_rcvdata,
	.connect = 	vr_tap_connect,
	.disconnect =	vr_tap_disconnect,
	.cmdlist =	vr_tap_cmdlist,
};

/* 
 * Instances are constructed by vr_tap_attach(9) are persistent. 
 */
static int 
vr_tap_constructor(node_p node) 
{
	return (EINVAL); 
}

/* 
 * This is a persistent netgraph(4) node.
 */
static int 
vr_tap_shutdown(node_p node)
{
	if ((node->nd_flags & NGF_REALLY_DIE) == 0)
		node->nd_flags &= ~NGF_INVALID; 
 
	return (0);
}

/*
 * Inverse element for vr_tap_disconnect(9).
 */
static int 
vr_tap_newhook(node_p node, hook_p hook, const char *name)
{
	struct vr_softc *sc = NG_NODE_PRIVATE(node);
	struct ifnet *ifp = sc->vr_ifp;
		
	if (strcmp(name, NG_VR_TAP_HOOK_RAW) != 0) 
		return (EPFNOSUPPORT);
	
	if (sc->vr_tap_hook != NULL) 
		return (EISCONN);

	VR_LOCK(sc);
	sc->vr_tap_hook = hook;
	VR_UNLOCK(sc);
	
	return (ifpromisc(ifp, 1));
}

/*
 * Fallback mechanism for rejecting the connection request.
 */
static int 
vr_tap_connect(hook_p hook)
{
	NG_HOOK_FORCE_QUEUE(NG_HOOK_PEER(hook));

	return (0);
}

/* 
 * Receive data encapsulated by mbuf(9) message 
 * primitives from another netgraph(4) peer node, 
 * pass into layer above and demultiplex. 
 */
static int 
vr_tap_rcvdata(hook_p hook, item_p item)
{
	const node_p node = NG_HOOK_NODE(hook);
	struct vr_softc *sc = NG_NODE_PRIVATE(node);
 	struct ifnet *ifp = sc->vr_ifp;
	struct mbuf *m;
 	
	NGI_GET_M(item, m);
	NG_FREE_ITEM(item);	

	if (m->m_pkthdr.rcvif != ifp) {
		m_freem(m);
		return (EINVAL);
	}

	if (m->m_pkthdr.len < sizeof(struct ether_header)) {
		m_freem(m);
		return (EINVAL);
	}

	if (m->m_len < sizeof(struct ether_header)) {
		m = m_pullup(m, sizeof(struct ether_header));
		if (m == NULL)
			return (ENOBUFS);	
	}

	return (if_input(ifp, m));	
}

/*
 * Process control message.
 */
static int
vr_tap_rcvmsg(node_p node, item_p item, hook_p lasthook)
{
	struct vr_softc *sc = NG_NODE_PRIVATE(node);
	struct ng_mesg *resp = NULL, *msg;
	int error;

	NGI_GET_MSG(item, msg);

	switch (msg->header.typecookie) {
	case NGM_VR_TAP_COOKIE:
		switch (msg->header.cmd) {
		case NGM_VR_TAP_GET_IFNAME:
			NG_MKRESPONSE(resp, msg, IFNAMSIZ, M_NOWAIT);

			if (resp == NULL) {
				error = ENOMEM;
				break;
			}
	
			(void)snprintf(resp->data, IFNAMSIZ, 
				"%s", sc->vr_ifp->if_xname);

			error = 0;
			break;
		default:
			error = EINVAL;
			break;
		}
	default:
		error = EINVAL;
		break;
	}
	NG_RESPOND_MSG(error, node, item, resp);
	NG_FREE_MSG(msg);

	return (error);
}

/*
 * Inverse element for vr_tap_connect(9).
 */
static int 
vr_tap_disconnect(hook_p hook)
{
	const node_p node = NG_HOOK_NODE(hook);
	struct vr_softc *sc = NG_NODE_PRIVATE(node);
	struct ifnet *ifp = sc->vr_ifp;

	VR_LOCK(sc);
	sc->vr_tap_hook = NULL;
	VR_UNLOCK(sc);
	
	return (ifpromisc(ifp, 0));
}

/* Service primitives. */

/* 
 * Attach instance of vr(4) with netgraph(4) node.
 * 
 * It is called once for each physical card during 
 * vr_attach(9). 
 * 
 * This is effectively vr_tap_constructor(9).
 */
int 
vr_tap_attach(struct vr_softc *sc)
{
	char name[IFNAMSIZ];
	struct ifnet *ifp;
	int error;

	if (vr_tap_type.refs == 0) {
		if ((error = ng_newtype(&vr_tap_type)) != 0) {
			(void)printf("%s: ng_newtype() failed; "
				"error %d\n", __func__, error);
			goto out;
		}
	} else 
		atomic_add_int(&vr_tap_type.refs, 1);
		
	error = ng_make_node_common(&vr_tap_type, &sc->vr_tap_node);
	if (error != 0) {
		(void)printf("%s: ng_make_node_common() failed; "
				"error %d\n", __func__, error);
		goto bad;
	}
	ifp = sc->vr_ifp;
	
	(void)snprintf(name, IFNAMSIZ, "%s%d", 
		vr_tap_type.name, ifp->if_index);
	
	if ((error = ng_name_node(sc->vr_tap_node, name)) != 0) {
		(void)printf("%s: ng_name_node() failed; "
				"error %d\n", __func__, error);
		goto bad1;
	}
	NG_NODE_SET_PRIVATE(sc->vr_tap_node, sc);
out:		
	return (error);
	
bad1:
	NG_NODE_UNREF(sc->vr_tap_node);
bad:	
	if (vr_tap_type.refs == 1)
		ng_rmtype(&vr_tap_type);
	else 
		atomic_subtract_int(&vr_tap_type.refs, 1);
		
	goto out;
}

/* 
 * Detach from the netgraph(4) domain(9).
 * 
 * It is called once for each physical card 
 * during vr_detach(9) or during rl_attach(9) 
 * as exeception handling, if something went 
 * wrong. 
 * 
 * This is effectively vr_tap_destructor(9).
 */
void 
vr_tap_detach(struct vr_softc *sc)
{	
	if (NG_NODE_PRIVATE(sc->vr_tap_node)) {
		ng_rmnode_self(sc->vr_tap_node);
		NG_NODE_UNREF(sc->vr_tap_node);
			
		if (vr_tap_type.refs == 1)
			ng_rmtype(&vr_tap_type);
		else 
			atomic_subtract_int(&vr_tap_type.refs, 1);
	}
}

/*
 * Cut off the trailer from Ethernet frame, encapsulate FCS 
 * by newly allocated mbuf(9), append and forward the chain 
 * to the netgraph(4) protocol domain(9).
 */
void
vr_tap_input(hook_p hook, struct mbuf **mp)
{
	struct mbuf *t, *m;
	int off, error;

	m = *mp; 
	off = m->m_pkthdr.len - ETHER_CRC_LEN;
	
	if ((t = m_split(m, off, M_NOWAIT)) != NULL) {
		while (m->m_next != NULL)
			m = m->m_next;
		
		m->m_next = t;
/* 
 * Sets *mp = NULL.
 */			
		NG_SEND_DATA_ONLY(error, hook, *mp);
	} else {
		m_freem(*mp);
		*mp = NULL;
	}
}
