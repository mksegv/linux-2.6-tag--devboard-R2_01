#
# arch/cris/boot/Makefile
#
target = $(target_boot_dir)
src    = $(src_boot_dir)

zImage: compressed/vmlinuz Image
	cp $(src)/compressed/vmlinuz $(src)/zImage

Image: vmlinux
	cp vmlinux $(src)/Image

compressed/vmlinuz:
	@$(MAKE) -f $(src)/compressed/Makefile $(target_compressed_dir)/vmlinuz

clean:
	@$(MAKE) -f $(src)/compressed/Makefile clean
