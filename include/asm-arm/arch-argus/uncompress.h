/*
 * linux/include/asm-arm/arch-argus/uncompress.h
 */

// In arch/arm/boot/compressed/setup-argus.c
// (other machs define it inline here)
extern void putstr(const char *s);

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
