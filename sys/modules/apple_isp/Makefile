.PATH: ${SRCTOP}/sys/arm64/apple

KMOD=	apple_isp
SRCS=	apple_isp.c apple_isp_cam.c apple_isp_cmd.c apple_isp_fw.c apple_isp_ipc.c
SRCS+=	bus_if.h device_if.h ofw_bus_if.h opt_platform.h

.include <bsd.kmod.mk>
