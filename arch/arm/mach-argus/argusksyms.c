/*
 *  Copyright (C) 2005 Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>

#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/system.h>

// Needed for DMA-using modules
EXPORT_SYMBOL(ct200_dma_inv_range);
// This does not seem to be exported by the kernel itself
EXPORT_SYMBOL(do_posix_clock_monotonic_gettime);
// For modules using the Video RAM
extern int argus_vram_base;
EXPORT_SYMBOL(argus_vram_base);
