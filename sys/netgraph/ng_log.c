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
 * This netgraph(4) node operates in conjunction 
 * with ng_tap(4) and ng_log(4) node as data sink.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/syslog.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>

#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_log.h>

/* 
 * Tuple holds necessary information 
 * 
 *  t := ( fcs0, fcs1, ether_header )
 * 
 * where
 * 
 *  (a) fcs0 := maps to trailer 
 * 
 *  (b) fcs1 := recalculated CRC-32 based FCS
 * 
 *  (c) ether_header := ( ether_dst, ether_src, ether_type )  
 * 
 * for further processing by syslogd(8). 
 */
struct ng_log_msg {
	uint32_t 	nlm_fcs0;
	uint32_t 	nlm_fcs1;
	struct ether_header 	nlm_eh;
} __packed;

/* Private data */
struct ng_log_priv {
	node_p	nlp_node;		/* back pointer to node */
	hook_p 	nlp_ingress; 		/* incomming messages */
	hook_p 	nlp_egress; 		/* forwarding messages */
	int (*nlp_eval)(struct ng_log_msg *nlm);
};
typedef struct ng_log_priv *nlp_p;

/* Private methods */
static int 	ng_log_all(struct ng_log_msg *nlm);
static int 	ng_log_err(struct ng_log_msg *nlm);
static int 	ng_log_none(struct ng_log_msg *nlm);

static int	ng_log_rcv_ingress(hook_p node, item_p item);
static int	ng_log_rcv_egress(hook_p node, item_p item);

/* Public methods */
static ng_constructor_t 	ng_log_constructor;
static ng_newhook_t 	ng_log_newhook;
static ng_rcvmsg_t 	ng_log_rcvmsg;
static ng_rcvdata_t 	ng_log_rcvdata;
static ng_disconnect_t 	ng_log_disconnect;
static ng_shutdown_t 	ng_log_shutdown;

/* List of commands and how to convert arguments to/from ASCII */
static const struct ng_cmdlist ng_log_cmdlist[] = {
	{
	  NGM_LOG_COOKIE,
	  NGM_LOG_ALL,
	  "logall",
	  NULL,
	  NULL
	},
	{
	  NGM_LOG_COOKIE,
	  NGM_LOG_ERR,
	  "logerr",
	  NULL,
	  NULL
	},
	{
	  NGM_LOG_COOKIE,
	  NGM_LOG_NONE,
	  "lognone",
	  NULL,
	  NULL
	},
	{ 0 }
};

/* Netgraph type */
static struct ng_type ng_log_type = {
	.version =	NG_ABI_VERSION,
	.name =		NG_LOG_NODE_TYPE,
	.constructor =	ng_log_constructor,
	.newhook =	ng_log_newhook,
	.rcvmsg =	ng_log_rcvmsg,
	.rcvdata =	ng_log_rcvdata,
	.disconnect =	ng_log_disconnect,
	.shutdown = 	ng_log_shutdown,
	.cmdlist =	ng_log_cmdlist,
};
NETGRAPH_INIT(syslog, &ng_log_type);

/*
 * Ctor.
 */
static int
ng_log_constructor(node_p node)
{
	nlp_p nlp;

	nlp = malloc(sizeof(*nlp), M_NETGRAPH, M_WAITOK|M_ZERO);
	
	NG_NODE_SET_PRIVATE(node, nlp);
	nlp->nlp_node = node;
	nlp->nlp_eval = ng_log_none;

	return (0);
}

/*
 * Interconnect with peer node by referring its hook denotes SAP.
 */
static int
ng_log_newhook(node_p node, hook_p hook, const char *name)
{
	const nlp_p nlp = NG_NODE_PRIVATE(node);
	hook_p *hp;
	int error;
	
	if (strcmp(name, NG_LOG_HOOK_INGRESS) == 0) {
		hp = &nlp->nlp_ingress;
		NG_HOOK_SET_RCVDATA(hook, ng_log_rcv_ingress);
	} else if (strcmp(name, NG_LOG_HOOK_EGRESS) == 0) {	
		hp = &nlp->nlp_egress;
		NG_HOOK_SET_RCVDATA(hook, ng_log_rcv_egress);
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
ng_log_rcvmsg(node_p node, item_p item, hook_p lasthook)
{
	const nlp_p nlp = NG_NODE_PRIVATE(node);
	struct ng_mesg *resp = NULL;
	int error = 0;
	struct ng_mesg *msg;

	NGI_GET_MSG(item, msg);
	switch (msg->header.typecookie) {
	case NGM_LOG_COOKIE:
		switch (msg->header.cmd) {
		case NGM_LOG_ALL:
			nlp->nlp_eval = ng_log_all;
			break;
		case NGM_LOG_ERR:
			nlp->nlp_eval = ng_log_err;
			break;
		case NGM_LOG_NONE:
			nlp->nlp_eval = ng_log_none;
			break;	
		default:
			error = EINVAL;
			break;
		}
		break;
	default:
		error = EINVAL;
		break;
	}
	NG_RESPOND_MSG(error, node, item, resp);
	NG_FREE_MSG(msg);
	
	return (error);
}

/*
 * Discarding data sink.
 */
static int
ng_log_rcvdata(hook_p hook, item_p item)
{
	NG_FREE_ITEM(item);
	
	return (0);
}

/*
 * Removal of the last link destroys the node.
 */
static int
ng_log_disconnect(hook_p hook)
{
	const nlp_p nlp = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));

	if (nlp->nlp_ingress == hook)
		nlp->nlp_ingress = NULL;
	else if (nlp->nlp_egress == hook)
		nlp->nlp_egress = NULL;
	
	if ((NG_NODE_NUMHOOKS(NG_HOOK_NODE(hook)) == 0)
	&& (NG_NODE_IS_VALID(NG_HOOK_NODE(hook)))) {
		ng_rmnode_self(NG_HOOK_NODE(hook));
	}
	return (0);
}

/*
 * Do local shutdown processing.
 */
static int
ng_log_shutdown(node_p node)
{
	const nlp_p nlp = NG_NODE_PRIVATE(node);

	NG_NODE_SET_PRIVATE(node, NULL);
	NG_NODE_UNREF(nlp->nlp_node);
	free(nlp, M_NETGRAPH);

	return (0);
}

/*
 * Extract information from MPI, pass it to syslogd(8) and forward MPI 
 * to peer node in netgraph(4) communication domain(9), if requested.
 */
static int
ng_log_rcv_ingress(hook_p hook, item_p item)
{
	const nlp_p nlp = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));
	int error = 0;
	struct mbuf *m;
	struct ifnet *ifp;
	struct ng_log_msg *nlm;

	NGI_GET_M(item, m);
	NG_FREE_ITEM(item);
    
	if ((ifp = m->m_pkthdr.rcvif) == NULL) {
		error = EINVAL;
		goto bad;
	}
    
	if (m->m_pkthdr.len < sizeof(struct ng_log_msg)) {
		error = EINVAL;
		goto bad;
	}
 
	if (m->m_len < sizeof(struct ng_log_msg)) {
		m = m_pullup(m, sizeof(struct ng_log_msg));
		if (m == NULL) {
			error = ENOBUFS;
			goto out;
		}
	}
	nlm = mtod(m, struct ng_log_msg *);
	
	if ((nlp->nlp_eval)(nlm)) { 
		log(LOG_NOTICE, "%s: ether_dst: %6D, ether_src: %6D,"
			" ether_type: 0x%04x, fcs1: 0x%08x, fcs: 0x%08x\n", 
			ifp->if_xname, nlm->nlm_eh.ether_dhost, ":", 
			nlm->nlm_eh.ether_shost, ":", 
			ntohs(nlm->nlm_eh.ether_type),
			ntohl(nlm->nlm_fcs1), ntohl(nlm->nlm_fcs0));

		if (nlp->nlp_egress != NULL)
			NG_SEND_DATA_ONLY(error, nlp->nlp_egress, m); 
		else
			NG_FREE_M(m);
	} else
		NG_FREE_M(m);
out:	
	return (error);
bad:
	NG_FREE_M(m);	
	goto out;
}

/*
 * Data sink, discards any incoming MPIs.
 */
static int
ng_log_rcv_egress(hook_p hook, item_p item)
{
	NG_FREE_ITEM(item);
	
	return (0);
}

/*
 * Pass all.
 */
static int
ng_log_all(struct ng_log_msg *nlm __unused)
{
	return (1);
}

/*
 * Pass Ethernet PCI from errornous frames.
 */
static int
ng_log_err(struct ng_log_msg *nlm)
{
	return (nlm->nlm_fcs0 != nlm->nlm_fcs1);
}

/*
 * Discard all.
 */
static int
ng_log_none(struct ng_log_msg *nlm __unused)
{
	return (0);
}
