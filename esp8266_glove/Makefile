PROGRAM=motionglove

#Enable debugging
#EXTRA_CFLAGS+=-DLWIP_DEBUG=1 -DHTTPD_DEBUG=LWIP_DBG_ON
LIBS = hal gcc c m

EXTRA_COMPONENTS=extras/i2c

PROGRAM_SRC_DIR+=. ./include

include ../esp-open-rtos/common.mk
