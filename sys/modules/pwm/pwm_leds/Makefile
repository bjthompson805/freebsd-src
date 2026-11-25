.PATH:	${SRCTOP}/sys/dev/pwm
KMOD=	pwm_leds
SRCS=	pwm_leds.c

SRCS+=	\
	backlight_if.h \
	bus_if.h \
	device_if.h \
	opt_platform.h \
	pwmbus_if.h \
	ofw_bus_if.h

.include <bsd.kmod.mk>
