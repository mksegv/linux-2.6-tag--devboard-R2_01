 /*
 * L2-cache helper functions
 *
 * Copyright (C) 1999-2005 Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __L2CACHE_H__
#define __L2CACHE_H__

#include <asm/arch/argus.h>

void l2cache_init(int nRams, int nIxRed);

void l2cache_LockRange(void *base, int size);

static __inline void l2cache_FlushAll(void)
{
  SOC_L2CACHE.FLUSH_ALL= 0;
}

static __inline void l2cache_FlushSingle(int physaddr)
{
  SOC_L2CACHE.FLUSH_SINGLE= physaddr;
}

static __inline void l2cache_CleanAll(void)
{
  SOC_L2CACHE.CLEAN_ALL= 0;
}

static __inline void l2cache_CleanSingle(int physaddr)
{
  SOC_L2CACHE.CLEAN_SINGLE= physaddr;
}

static __inline void l2cache_DrainWB(void)
{
  SOC_L2CACHE.DRAIN_WB= 0;
}

static __inline void l2cache_FlushStream(void)
{
  SOC_L2CACHE.FLUSH_STREAM= 0;
}

#endif
