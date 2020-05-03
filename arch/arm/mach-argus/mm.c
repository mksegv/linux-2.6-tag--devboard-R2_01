/*
 *  linux/arch/arm/mach-argus/mm.c
 *
 *  Copyright (C) 2003,2004 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mm.h>
#include <linux/init.h>

#include <asm/mach/map.h>

#include <asm/arch/argus_addr.h>

// Table of initial global kernel mappings. The mm stuff will handle the
// difference between page mappings and section mappings and use the latter
// where applicable, so it is more efficient to map whole 1 MB areas.
//
// The flash is mapped both cached and uncached.

static struct map_desc argus_io_desc[] __initdata = {
  /* virtual     physical    length      type */
  { EXT0_NOCACHE_V,       EXT0_BASE_P,         0x02000000, MT_DEVICE }, /* CS0, Flash 32 MB */
  { EXT0_CACHE_V,         EXT0_BASE_P,         0x02000000, MT_MEMORY }, /* CS0, Flash 32 MB */
  { APB_NOCACHE,          APB_BASE_P,          0x00200000, MT_DEVICE }, /* APB 2 MB       */
  { OCRAM_BASE_V,         OCRAM_BASE_P,        0x00100000, MT_DEVICE }, /* OCRAM 1 MB */
  { L2CACHE_CTRL_NOCACHE, L2CACHE_CTRL_BASE_P, 0x00100000, MT_DEVICE }, /* L2 CTRL 1MB */
  { L2CACHE_STAT_BASE_V,  L2CACHE_STAT_BASE_P, 0x00100000, MT_DEVICE }, /* L2 1MB */
  { DMA_NOCACHE,          DMA_BASE_P,          0x00100000, MT_DEVICE }, /* DMA in Argus3 1 MB */
  { EXT2_NOCACHE_V,       EXT2_BASE_P,         0x00100000, MT_DEVICE }, /* EXT2 nocache 1MB */
  /* The following weird mappings are for the 206W CF-card interface */
  { EXT5_NOCACHE1_V,      EXT5_BASE_P + 0x800000, 0x00100000, MT_DEVICE }, /* EXT5 nocache */
  { EXT5_NOCACHE2_V,      EXT5_BASE_P + 0x500000, 0x00100000, MT_DEVICE }, /* EXT5 nocache */
  { EXT5_NOCACHE3_V,      EXT5_BASE_P + 0x400000, 0x00100000, MT_DEVICE }, /* EXT5 nocache */
};

void __init 
argus_map_io(void)
{
        iotable_init(argus_io_desc, ARRAY_SIZE(argus_io_desc));
}
