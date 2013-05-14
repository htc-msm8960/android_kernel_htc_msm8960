obj-$(CONFIG_MACH_VILLE) += board-ville.o
obj-$(CONFIG_MACH_VILLE) += board-ville-regulator.o
obj-$(CONFIG_MACH_VILLE) += board-ville-gpiomux.o
obj-$(CONFIG_MACH_VILLE) += board-ville-storage.o
obj-$(CONFIG_MACH_VILLE) += board-ville-audio.o
obj-$(CONFIG_MACH_VILLE) += board-ville-pmic.o
obj-$(CONFIG_MACH_VILLE) += board-ville-keypad.o
obj-$(CONFIG_MACH_VILLE) += board-ville-gpu.o
ifeq ($(CONFIG_MSM_CAMERA),y)
obj-$(CONFIG_MACH_VILLE) += board-ville-camera.o
ifeq ($(CONFIG_RAWCHIP),y)
obj-$(CONFIG_MACH_VILLE) += board-ville-rawchip.o
endif
endif
obj-$(CONFIG_MACH_VILLE) += display/
CFLAGS_board-ville-display.o += -Idrivers/video
