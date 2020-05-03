#
# Makefile for the linux kernel.
#

# Common support (must be linked before board specific support)
obj-y := arch.o irq.o time.o ciobuf.o dma_alloc.o mm.o \
	argus_uart.o axisflashmap.o boot_info.o argus_serial.o \
	argusksyms.o

obj-m :=
obj-n :=
obj-  :=

ifdef CONFIG_ARGUS_3
obj-y += arg3dma.o
else
obj-y += argdma.o
endif

obj-$(CONFIG_ARGUS_ETHERNET) += eth.o
obj-$(CONFIG_ARGUS_SSI) += ssi.o
# obj-$(CONFIG_ARGUS_CF) += cf.o
obj-$(CONFIG_ARGUS_GPIO) += gpio.o


