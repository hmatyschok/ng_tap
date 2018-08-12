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
 * with ng_tap(4) and ng_fcs(4) node as data sink.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/syslog.h>

#include <net/ethernet.h>

#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_log.h>

/* 
 * Tuple holds necessary information 
 * 
 *  t := ( fcs0 , fcs1, ether_header )
 * 
 * where
 * 
 *  (a) fcs0 := maps to trailer 
 * 
 *  (b) fcs1 := recalculated CRC-32 based FCS
 * 
 *  (c) ether_header := ( ether_dst, ether_src, ether_type )  
 * 
 * for further processing by syslog(9). 
 */
struct ng_log_msg {
	uint32_t 	nlm_fcs0;
	uint32_t 	nlm_fcs1;
	struct ether_header 	nlm_eh;
};

/* Private data */
struct ng_log_priv {
	node_p	nlp_node;		/* back pointer to node */
	hook_p 	nlp_log;
};
typedef struct ng_log_priv *nlp_p;

/* Public methods */
static ng_constructor_t 	ng_log_constructor;
static ng_rcvmsg_t 	ng_log_rcvmsg;
static ng_rcvdata_t 	ng_log_rcvdata;
static ng_disconnect_t 	ng_log_disconnect;
static ng_shutdown_t 	ng_log_shutdown;

/* Netgraph type */
static struct ng_type ng_log_type = {
	.version =	NG_ABI_VERSION,
	.name =		NG_LOG_NODE_TYPE,
	.constructor =	ng_log_constructor,
	.rcvmsg =	ng_log_rcvmsg,
	.rcvdata =	ng_log_rcvdata,
	.disconnect =	ng_log_disconnect,
	.shutdown = 	ng_log_shutdown,
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

	return (0);
}

static int
ng_log_newhook(node_p node, hook_p hook, const char *name)
{
	const nlp_p nlp = NG_NODE_PRIVATE(node);
	
	if (nlp->nlp_log != NULL)
		return (EISCONN);
	
	if (strcmp(name, NG_LOG_HOOK_LOG) != 0) 	
		nlp->nlp_log = hook;
	else 
		return (EPFNOSUPPORT);
		
	return (0);
}

/*
 * Receive control message. 
 * 
 * But there is nothing to do here, we just bounce it back as a reply.
 */
static int
ng_log_rcvmsg(node_p node, item_p item, hook_p lasthook)
{
	struct ng_mesg *msg;
	int error;

	NGI_GET_MSG(item, msg);
	msg->header.flags |= NGF_RESP;
	NG_RESPOND_MSG(error, node, item, msg);

	return (0);
}

/*
 * Extract information from message primitive and pass it to syslog(9).
 */
static int
ng_log_rcvdata(hook_p hook, item_p item)
{
	int error = 0;
	struct mbuf *m;
	struct ng_log_msg *nlm;

	NGI_GET_M(item, m);
    
	if (m->m_pkthdr.len < sizeof(struct ng_log_msg)) {
		error = EINVAL;
		goto out1;
	}
 
	if (m->m_len < sizeof(struct ng_log_msg)) {
		m = m_pullup(m, sizeof(struct ng_log_msg));
		if (m == NULL) {
			error = ENOBUFS;
			goto out;
		}
	}
	
	nlm = mtod(m, struct ng_log_msg *);
	
	log(LOG_NOTICE, "fcs0: 0x%08x, fcs1: 0x%08x, ether_src: %6D\n", 
		nlm->nlm_fcs0, nlm->nlm_fcs1, nlm->nlm_eh.ether_shost, ":");	
out1:
	NG_FREE_M(m);
out:
	NG_FREE_ITEM(item);
	return (error);
}

/*
 * Removal of the last link destroys the node.
 */
static int
ng_log_disconnect(hook_p hook)
{
	const nlp_p nlp = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));

	if (nlp->nlp_log == hook)
		nlp->nlp_log = NULL;
	
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
ng_log_shutdown(node_p node)
{
	const nlp_p nlp = NG_NODE_PRIVATE(node);

	NG_NODE_SET_PRIVATE(node, NULL);
	NG_NODE_UNREF(nlp->nlp_node);
	free(nlp, M_NETGRAPH);

	return (0);
}
