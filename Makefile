TARGET = usb_gamepad

EXTRA_FLAGS = -DMAX_PACKET_SIZE=64

C_FILES = \
	usb_gamepad.c \
	usb_host.c \
	debug.c

pre-flash:


include Makefile.include
