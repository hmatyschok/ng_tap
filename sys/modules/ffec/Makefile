
.PATH: ${SRCTOP}/sys/dev/ffec

KMOD=	if_ffec
SRCS=	if_ffec.c miibus_if.h device_if.h bus_if.h pci_if.h 
SRCS+= 	ofw_bus_if.h
SRCS+= 	opt_netgraph.h

opt_netgraph.h:
	echo "#define NETGRAPH 1" > ${.TARGET}

.include <bsd.kmod.mk>
