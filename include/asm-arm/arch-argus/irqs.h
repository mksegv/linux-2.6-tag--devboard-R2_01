/*
 * linux/include/asm-arm/arch-argus/irqs.h
 */

// Really 21 for Argus2 and 22 for Argus3 but it is ok
// to use 22 for A2.

#define NR_IRQS			22

// SOC_IRQ.MASK uses the same bit positions for enabling FIQ's as
// for normal IRQ's
// (the SOC_IRQ.LEVEL bitmask controls IRQ/FIQ selection instead)

#define FIQ_START               0

