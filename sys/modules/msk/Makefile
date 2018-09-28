# $FreeBSD: releng/11.1/sys/modules/msk/Makefile 319182 2017-05-30 04:11:12Z ngie $

.PATH: ${SRCTOP}/sys/dev/msk

KMOD=	if_msk
SRCS=	if_msk.c device_if.h bus_if.h pci_if.h
SRCS+= 	ng_msk_tap.h
SRCS+=	miibus_if.h
SRCS+= 	opt_netgraph.h

opt_netgraph.h:
	echo "#define NETGRAPH 1" > ${.TARGET}
	
.include <bsd.kmod.mk>
