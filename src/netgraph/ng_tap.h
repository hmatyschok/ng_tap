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

#ifndef _NETGRAPH_NG_TAP_H_
#define _NETGRAPH_NG_TAP_H_

/* Magic cookie */
#define NGM_TAP_COOKIE		 1524849212	/* date -u +'%s' */

/* Hook names */
#define NG_TAP_HOOK_UPSTREAM 	"upstream" /* upstream data flow */

/* Generic netgraph(4) control messages */
enum {
	NGM_TAP_GET_IFNAME = 1,
};

#ifdef _KERNEL

/*
 * Set of generic commands.
 */
#define NG_TAP_CMDLIST_DECLARE(device)                                \
static const struct ng_cmdlist ng_##device##_tap_cmdlist[] = {        \
	{                                                                 \
	  NGM_TAP_COOKIE,                                                 \
	  NGM_TAP_GET_IFNAME,                                             \
	  "getifname",                                                    \
	  NULL,                                                           \
	  &ng_parse_string_type,                                          \
	},                                                                \
	{ 0 } 														      \
}; 																      \

/* 
 * By ng_xxx_tap_attach(9) instantiated nodes are persistent. 
 */
#define NG_TAP_CONSTRUCTOR_DECLARE(device)                            \
static int                                                            \
ng_##device##_tap_constructor(node_p node)                            \
{                                                                     \
	return (EINVAL);                                                  \
}

/* 
 * This is a persistent netgraph(4) node.
 */
#define NG_TAP_SHUTDOWN_DECLARE(device)                               \
static int                                                            \
ng_##device##_tap_shutdown(node_p node)                               \
{                                                                     \
	if ((node->nd_flags & NGF_REALLY_DIE) == 0)                       \
		node->nd_flags &= ~NGF_INVALID;                               \
                                                                      \
	return (0);                                                       \
}

/*
 * Inverse element for ng_xxx_tap_disconnect(9).
 */
#define NG_TAP_NEWHOOK_DECLARE(pfx, device, ctx)                      \
static int                                                            \
ng_##device##_tap_newhook(node_p node, hook_p hook, const char *name) \
{                                                                     \
	struct ctx *sc = NG_NODE_PRIVATE(node);                           \
	struct ifnet *ifp = sc->device##_ifp;                             \
                                                                      \
	if (strcmp(name, NG_TAP_HOOK_UPSTREAM) != 0)                      \
		return (EPFNOSUPPORT);                                        \
                                                                      \
	if (sc->device##_tap_hook != NULL)                                \
		return (EISCONN);                                             \
                                                                      \
	pfx##_LOCK(sc);                                                   \
	sc->device##_tap_hook = hook;                                     \
	pfx##_UNLOCK(sc);                                                 \
                                                                      \
	return (ifpromisc(ifp, 1));                                       \
}

/*
 * Fallback mechanism for rejecting the connection request.
 */
#define NG_TAP_CONNECT_DECLARE(device)                                \
static int                                                            \
ng_##device##_tap_connect(hook_p hook)                                \
{                                                                     \
	NG_HOOK_FORCE_QUEUE(NG_HOOK_PEER(hook));                          \
                                                                      \
	return (0);                                                       \
}

/* 
 * Receive data encapsulated by mbuf(9) message primitives from 
 * netgraph(4) peer node, reinject upstream and demultiplex by 
 * layer above or forward by e. g.  if_bridge(4). 
 */
#define NG_TAP_RCVDATA_DECLARE(device, ctx)                           \
static int                                                            \
ng_##device##_tap_rcvdata(hook_p hook, item_p item)                   \
{                                                                     \
	const node_p node = NG_HOOK_NODE(hook);                           \
	struct ctx *sc = NG_NODE_PRIVATE(node);                           \
 	struct ifnet *ifp = sc->device##_ifp;                             \
	struct mbuf *m;                                                   \
                                                                      \
	NGI_GET_M(item, m);                                               \
	NG_FREE_ITEM(item);                                               \
                                                                      \
	if (m->m_pkthdr.rcvif != ifp) {                                   \
		m_freem(m);                                                   \
		return (EINVAL);                                              \
	}                                                                 \
                                                                      \
	if (m->m_pkthdr.len < sizeof(struct ether_header)) {              \
		m_freem(m);                                                   \
		return (EINVAL);                                              \
	}                                                                 \
                                                                      \
	if (m->m_len < sizeof(struct ether_header)) {                     \
		m = m_pullup(m, sizeof(struct ether_header));                 \
		if (m == NULL)                                                \
			return (ENOBUFS);                                         \
	}                                                                 \
                                                                      \
	return (if_input(ifp, m));                                        \
}

/*
 * Process control messages, if any.
 */
#define NG_TAP_RCVMSG_DECLARE(device, ctx)                            \
static int                                                            \
ng_##device##_tap_rcvmsg(node_p node, item_p item, hook_p lasthook)   \
{                                                                     \
	struct ctx *sc = NG_NODE_PRIVATE(node);                           \
	struct ng_mesg *resp = NULL, *msg;                                \
	int error;                                                        \
                                                                      \
	NGI_GET_MSG(item, msg);                                           \
                                                                      \
	switch (msg->header.typecookie) {                                 \
	case NGM_TAP_COOKIE:                                              \
		switch (msg->header.cmd) {                                    \
		case NGM_TAP_GET_IFNAME:                                      \
			NG_MKRESPONSE(resp, msg, IFNAMSIZ, M_NOWAIT);             \
                                                                      \
			if (resp == NULL) {                                       \
				error = ENOMEM;                                       \
				break;                                                \
			}                                                         \
                                                                      \
			(void)snprintf(resp->data, IFNAMSIZ,                      \
				"%s", sc->device##_ifp->if_xname);                    \
                                                                      \
			error = 0;                                                \
			break;                                                    \
		default:                                                      \
			error = EINVAL;                                           \
			break;                                                    \
		}                                                             \
	default:                                                          \
		error = EINVAL;                                               \
		break;                                                        \
	}                                                                 \
	NG_RESPOND_MSG(error, node, item, resp);                          \
	NG_FREE_MSG(msg);                                                 \
                                                                      \
	return (error);                                                   \
}

/*
 * Inverse element for ng_xxx_tap_connect(9).
 */
#define NG_TAP_DISCONNECT_DECLARE(pfx, device, ctx)                   \
static int                                                            \
ng_##device##_tap_disconnect(hook_p hook)                             \
{                                                                     \
	const node_p node = NG_HOOK_NODE(hook);                           \
	struct ctx *sc = NG_NODE_PRIVATE(node);                           \
	struct ifnet *ifp = sc->device##_ifp;                             \
                                                                      \
	pfx##_LOCK(sc);                                                   \
	sc->device##_tap_hook = NULL;                                     \
	pfx##_UNLOCK(sc);                                                 \
                                                                      \
	return (ifpromisc(ifp, 0));                                       \
}

/*
 * Implements ng_xxx_tap(4) node type. 
 */
#define NG_TAP_TYPE_DECLARE(device, str)                              \
static struct ng_type ng_##device##_tap_type = {                      \
 	.version =	NG_ABI_VERSION,                                       \
	.name =		(str),                                                \
	.mod_event =	NULL,                                             \
	.constructor =	ng_##device##_tap_constructor,                    \
	.rcvmsg =	ng_##device##_tap_rcvmsg,                             \
	.shutdown =	ng_##device##_tap_shutdown,                           \
	.newhook =	ng_##device##_tap_newhook,                            \
	.rcvdata =	ng_##device##_tap_rcvdata,                            \
	.connect = 	ng_##device##_tap_connect,                            \
	.disconnect =	ng_##device##_tap_disconnect,                     \
	.cmdlist =	ng_##device##_tap_cmdlist,                            \
}; 

/* Service primitives. */

/* 
 * Attach instance of xxx(4) NIC with netgraph(4) node.
 * 
 * It is called once for each physical card during device_attach(9). 
 * 
 * This is effectively ng_xxx_tap_constructor(9).
 */
#define NG_TAP_ATTACH_DECLARE(device, ctx)                            \
static int                                                            \
ng_##device##_tap_attach(struct ctx *sc)                              \
{                                                                     \
	char name[IFNAMSIZ];                                              \
	struct ifnet *ifp;                                                \
	int error; 													\
																\
	if (ng_##device##_tap_type.refs == 0) { 					\
		error = ng_newtype(&ng_##device##_tap_type); 			\
		if (error != 0) { 										\
			(void)printf("%s: ng_newtype() failed; " 			\
				"error %d\n", __func__, error); 				\
			goto out; 											\
		} 														\
	} else 														\
		atomic_add_int(&ng_##device##_tap_type.refs, 1); 		\
																\
	error = ng_make_node_common(&ng_##device##_tap_type, 		\
		&sc->device##_tap_node); 								\
	if (error != 0) { 											\
		(void)printf("%s: ng_make_node_common() failed; " 		\
				"error %d\n", __func__, error); 				\
		goto bad; 												\
	} 															\
	ifp = sc->device##_ifp; 									\
																\
	(void)snprintf(name, IFNAMSIZ, "%s%d", 						\
		ng_##device##_tap_type.name, ifp->if_index); 			\
																\
	error = ng_name_node(sc->device##_tap_node, name); 			\
	if (error != 0) { 											\
		(void)printf("%s: ng_name_node() failed; " 				\
				"error %d\n", __func__, error); 				\
		goto bad1; 												\
	} 															\
	NG_NODE_SET_PRIVATE(sc->device##_tap_node, sc); 			\
out: 															\
	return (error); 											\
																\
bad1: 															\
	NG_NODE_UNREF(sc->device##_tap_node); 						\
bad: 															\
	if (ng_##device##_tap_type.refs == 1) 						\
		ng_rmtype(&ng_##device##_tap_type); 					\
	else 														\
		atomic_subtract_int(&ng_##device##_tap_type.refs, 1); 	\
																\
	goto out; 													\
}

/* 
 * Detach from the netgraph(4) domain(9).
 * 
 * It is called once for each physical card during device_detach(9) 
 * or during device_attach(9) as exeception handling, if something 
 * went wrong. This is effectively ng_device_tap_destructor(9).
 */
#define NG_TAP_DETACH_DECLARE(device, ctx) 						\
static void 													\
ng_##device##_tap_detach(struct ctx *sc) 						\
{ 																\
	if (NG_NODE_PRIVATE(sc->device##_tap_node)) { 				\
		ng_rmnode_self(sc->device##_tap_node); 					\
		NG_NODE_UNREF(sc->device##_tap_node); 					\
																\
		if (ng_##device##_tap_type.refs == 1) 					\
			ng_rmtype(&ng_##device##_tap_type); 				\
		else 													\
			atomic_subtract_int(&ng_##device##_tap_type.refs, 1); \
	} 															\
}

/*
 * Cut off the trailer from Ethernet frame, encapsulate FCS 
 * by newly allocated mbuf(9), append and forward the chain 
 * to the netgraph(4) protocol domain(9).
 */
#define NG_TAP_INPUT_DECLARE(device) 							\
static void 													\
ng_##device##_tap_input(hook_p hook, struct mbuf **mp) 			\
{ 																\
	struct mbuf *t, *m; 										\
	int off, error; 						 					\
																\
	m = *mp; 													\
	off = m->m_pkthdr.len - ETHER_CRC_LEN; 						\
																\
	if ((t = m_split(m, off, M_NOWAIT)) != NULL) { 				\
		while (m->m_next != NULL) 								\
			m = m->m_next; 										\
																\
		m->m_next = t; 											\
/* 																\
 * Sets *mp = NULL. 											\
 */																\
		NG_SEND_DATA_ONLY(error, hook, *mp); 					\
	} else { 													\
		m_freem(*mp); 											\
		*mp = NULL; 											\
	} 															\
}

/*
 * Put everything together.
 */
#define NG_TAP_MODULE(pfx, device, ctx, str) 					\
	NG_TAP_CMDLIST_DECLARE(device) 								\
	NG_TAP_CONSTRUCTOR_DECLARE(device) 							\
	NG_TAP_SHUTDOWN_DECLARE(device)								\
	NG_TAP_NEWHOOK_DECLARE(pfx, device, ctx) 					\
	NG_TAP_CONNECT_DECLARE(device)								\
	NG_TAP_RCVDATA_DECLARE(device, ctx)							\
	NG_TAP_RCVMSG_DECLARE(device, ctx) 							\
	NG_TAP_DISCONNECT_DECLARE(pfx, device, ctx) 				\
	NG_TAP_TYPE_DECLARE(device, str) 							\
	NG_TAP_ATTACH_DECLARE(device, ctx) 							\
	NG_TAP_DETACH_DECLARE(device, ctx)							\
	NG_TAP_INPUT_DECLARE(device)
#endif /* _KERNEL */
#endif /* _NETGRAPH_NG_TAP_H_ */
