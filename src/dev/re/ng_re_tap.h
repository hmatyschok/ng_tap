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

#ifndef _RE_NG_RE_TAP_H_
#define _RE_NG_RE_TAP_H_

/* Node name for ng_re_tap(4) */
#define NG_RE_TAP_NODE_TYPE 	"re_tap"
#define NGM_RE_TAP_COOKIE 		1523982355	/* date -u +'%s' */

/* Hook names */
#define NG_RE_TAP_HOOK_RAW 	"raw" /* connection to raw device */

/* Netgraph control messages */
enum {
	NGM_RE_TAP_GET_IFNAME = 1,
};

/* Service primitives */
#ifdef _KERNEL
int 	re_tap_attach(struct rl_softc *);
void 	re_tap_detach(struct rl_softc *);
void 	re_tap_input(hook_p, struct mbuf **);
#endif /* _KERNEL */
#endif /* _RE_NG_RE_TAP_H_ */
