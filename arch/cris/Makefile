# $Id: Makefile,v 1.32 2005/09/09 08:09:10 karljope Exp $
# cris/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies. Remember to do have actions
# for "archclean" and "archdep" for cleaning up and making dependencies for
# this architecture
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.

# A bug in ld prevents us from having a (constant-value) symbol in a
# "ORIGIN =" or "LENGTH =" expression.

arch-y := v10
arch-$(CONFIG_ETRAX_ARCH_V10) := v10
arch-$(CONFIG_ETRAX_ARCH_V32) := v32

# No config avaiable for make clean etc
ifneq ($(arch-y),)
SARCH := arch-$(arch-y)
else
SARCH :=
endif

LD = $(CROSS_COMPILE)ld -mcrislinux

OBJCOPYFLAGS := -O binary -R .note -R .comment -S

CPPFLAGS_vmlinux.lds = -DDRAM_VIRTUAL_BASE=0x$(CONFIG_ETRAX_DRAM_VIRTUAL_BASE)
AFLAGS += -mlinux

CFLAGS := $(CFLAGS) -mlinux -march=$(arch-y) -pipe

ifdef CONFIG_FRAME_POINTER
CFLAGS := $(subst -fomit-frame-pointer,,$(CFLAGS)) -g
CFLAGS += -fno-omit-frame-pointer
endif

head-y := arch/$(ARCH)/$(SARCH)/kernel/head.o

LIBGCC = $(shell $(CC) $(CFLAGS) -print-file-name=libgcc.a)

core-y		+= arch/$(ARCH)/kernel/ arch/$(ARCH)/mm/
core-y		+= arch/$(ARCH)/$(SARCH)/kernel/ arch/$(ARCH)/$(SARCH)/mm/
drivers-y	+= arch/$(ARCH)/$(SARCH)/drivers/
libs-y		+= arch/$(ARCH)/$(SARCH)/lib/ $(LIBGCC)

# cris source path
SRC_ARCH              = $(srctree)/arch/$(ARCH)
# cris object files path
OBJ_ARCH              = $(objtree)/arch/$(ARCH)

target_boot_arch_dir  = $(OBJ_ARCH)/$(SARCH)/boot
target_boot_dir       = $(OBJ_ARCH)/boot
src_boot_dir          = $(SRC_ARCH)/boot
target_compressed_dir = $(OBJ_ARCH)/boot/compressed
src_compressed_dir    = $(SRC_ARCH)/boot/compressed
target_rescue_dir     = $(OBJ_ARCH)/boot/rescue
src_rescue_dir        = $(SRC_ARCH)/boot/rescue

export target_boot_arch_dir target_boot_dir src_boot_dir target_compressed_dir src_compressed_dir target_rescue_dir src_rescue_dir

vmlinux.bin: vmlinux
	$(OBJCOPY) $(OBJCOPYFLAGS) vmlinux vmlinux.bin

timage: vmlinux.bin
	cat vmlinux.bin cramfs.img >timage

simimage: timage
	cp vmlinux.bin simvmlinux.bin

# the following will remake timage without compiling the kernel
# it does of course require that all object files exist...

cramfs:
## cramfs      - Creates a cramfs image
	mkcramfs -b 8192 -m romfs_meta.txt root cramfs.img
	cat vmlinux.bin cramfs.img >timage

clinux: vmlinux.bin decompress.bin rescue.bin

decompress.bin: $(target_boot_dir)
	@$(MAKE) -f $(src_compressed_dir)/Makefile $(target_compressed_dir)/decompress.bin

.PHONY: $(target_rescue_dir)/rescue.bin
$(target_rescue_dir)/rescue.bin: $(target_boot_dir)
	@$(MAKE) -f $(src_rescue_dir)/Makefile $(target_rescue_dir)/rescue.bin

zImage: $(target_boot_dir) vmlinux.bin $(target_rescue_dir)/rescue.bin
## zImage     - Compressed kernel (gzip)
	@$(MAKE) -f $(src_boot_dir)/Makefile zImage

$(target_boot_dir): $(target_boot_arch_dir)
	ln -sfn $(SARCH)/boot $@

$(target_boot_arch_dir):
	mkdir -p $@

compressed: zImage

archmrproper:
archclean:
	@if [ -d arch/$(ARCH)/boot ]; then \
		$(MAKE) $(clean)=arch/$(ARCH)/boot ; \
	fi
	rm -f timage vmlinux.bin decompress.bin rescue.bin cramfs.img
	rm -rf $(LD_SCRIPT).tmp

prepare: $(SRC_ARCH)/.links $(srctree)/include/asm-$(ARCH)/.arch \
	 include/asm-$(ARCH)/$(SARCH)/offset.h

# Create some links to make all tools happy
$(SRC_ARCH)/.links:
	@rm -rf $(SRC_ARCH)/drivers
	@ln -sfn $(SARCH)/drivers $(SRC_ARCH)/drivers
	@rm -rf $(SRC_ARCH)/boot
	@ln -sfn $(SARCH)/boot $(SRC_ARCH)/boot
	@rm -rf $(SRC_ARCH)/lib
	@ln -sfn $(SARCH)/lib $(SRC_ARCH)/lib
	@ln -sfn $(SARCH) arch
	@ln -sfn ../$(SARCH)/vmlinux.lds.S $(SRC_ARCH)/kernel/vmlinux.lds.S
	@touch $@

# Create link to sub arch includes
$(srctree)/include/asm-$(ARCH)/.arch: $(wildcard include/config/arch/*.h)
	@echo '  Making $(srctree)/include/asm-$(ARCH)/arch -> $(srctree)/include/asm-$(ARCH)/$(SARCH) symlink'
	@rm -f include/asm-$(ARCH)/arch
	@ln -sf $(SARCH) $(srctree)/include/asm-$(ARCH)/arch
	@touch $@

arch/$(ARCH)/$(SARCH)/kernel/asm-offsets.s: include/asm include/linux/version.h \
					include/config/MARKER

include/asm-$(ARCH)/$(SARCH)/offset.h: arch/$(ARCH)/$(SARCH)/kernel/asm-offsets.s
	$(call filechk,gen-asm-offsets)
