.PATH: ${SRCTOP}/sys/arm64/apple

KMOD=	apple_macaudio
SRCS=	apple_macaudio.c apple_macaudio_spk.c
SRCS+=	bus_if.h channel_if.h device_if.h mixer_if.h ofw_bus_if.h opt_platform.h opt_snd.h

.include <bsd.kmod.mk>
