ifeq ($(CONFIG_USB_ANDROID_QCT_DIAG),y)
obj-$(CONFIG_DIAG_CHAR) := diagchar.o
ifeq ($(CONFIG_ARCH_MSM8X60_LTE),y)
diagchar-objs := diagchar_core.o diagchar_hdlc.o diagfwd.o diagmem.o diagfwd_sdio.o
else
diagchar-objs := diagchar_core.o diagchar_hdlc.o diagfwd.o diagmem.o
endif
endif