#ifndef __ARGUS2_ADDR_H__
#define __ARGUS2_ADDR_H__

/* Physical map */

#define BOOT_ROM_BASE_P     0x00000000    /* 4 kB */
#define OCRAM_BASE_P        0x10000000    /* 32 or 64kB */
#define APB_BASE_P          0x20000000    /* 2 Mb */
#define ZEROS_BASE_P        0x30000000    /* 256 Mb */
#define L2CACHE_CTRL_BASE_P 0x40000000    /* 4 kB */
#define L2CACHE_STAT_BASE_P 0x50000000    /* 128 kB in Argus2, 24 kB in Argus3 */
#define DMA_BASE_P          0x60000000    /* 8 kB */
#define EXT0_BASE_P         0x80000000    /* 256 Mb */
#define EXT1_BASE_P         0x90000000    /* 256 Mb */
#define EXT2_BASE_P         0xa0000000    /* 256 Mb */
#define EXT3_BASE_P         0xb0000000    /* 256 Mb */
#define EXT4_BASE_P         0xc0000000    /* 256 Mb */
#define EXT5_BASE_P         0xd0000000    /* 256 Mb */

/* Virtual map (not all things are mapped in reality -
 * see arch/arm/mach-argus/mm.c). The sizes refer to the map
 * sizes, not the actual underlying accessible memory size.
 */

#ifndef PHYSICAL_ADDRESSING
#define EXT0_NOCACHE_V        0xe0000000    /* 32 Mb */
#define EXT0_CACHE_V          0xe2000000    /* 32 Mb */
#define APB_NOCACHE           0xe4000000    /* 2 Mb */
#define OCRAM_BASE_V          0xe4200000    /* 1 Mb */
#define L2CACHE_CTRL_NOCACHE  0xe4300000    /* 1 Mb */
#define L2CACHE_STAT_BASE_V   0xe4400000    /* 1 Mb */
#define EXT2_NOCACHE_V        0xe4500000    /* 1 Mb */
#define EXT5_NOCACHE1_V       0xe4600000    /* 1 Mb */
#define EXT5_NOCACHE2_V       0xe4700000    /* 1 Mb */
#define EXT5_NOCACHE3_V       0xe4800000    /* 1 Mb */
#define DMA_NOCACHE           0xe4900000    /* 1 Mb */
/*#define ZEROS_BASE_V        0x30000000*/  /* 256 Mb */
#else
#define EXT0_NOCACHE_V        EXT0_BASE_P            /* 32 Mb */
#define APB_NOCACHE           APB_BASE_P             /* 2 Mb */
#define OCRAM_BASE_V          OCRAM_BASE_P           /* 1 Mb */
#define L2CACHE_CTRL_NOCACHE  L2CACHE_CTRL_BASE_P    /* 1 Mb */
#define L2CACHE_STAT_BASE_V   L2CACHE_STAT_BASE_P    /* 1 Mb */
#endif
#endif /* __ARGUS2_ADDR_H__ */
