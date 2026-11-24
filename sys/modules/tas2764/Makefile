.PATH: ${SRCTOP}/sys/arm64/apple

KMOD=	tas2764
SRCS=	tas2764.c
SRCS+=	bus_if.h device_if.h gpio_if.h gpiobus_if.h iicbus_if.h ofw_bus_if.h \
	opt_platform.h

.include <bsd.kmod.mk>
