# $FreeBSD: releng/11.1/sys/modules/my/Makefile 319182 2017-05-30 04:11:12Z ngie $

.PATH: ${SRCTOP}/sys/dev/my

KMOD=	if_my
SRCS=	if_my.c device_if.h bus_if.h pci_if.h
SRCS+= 	opt_netgraph.h

opt_netgraph.h:
	echo "#define NETGRAPH 1" > ${.TARGET}

.include <bsd.kmod.mk>
