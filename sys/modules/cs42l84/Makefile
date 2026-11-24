.PATH: ${SRCTOP}/sys/arm64/apple

KMOD=	cs42l84
SRCS=	cs42l84.c
SRCS+=	bus_if.h device_if.h gpio_if.h gpiobus_if.h iicbus_if.h ofw_bus_if.h \
	opt_platform.h

.include <bsd.kmod.mk>
