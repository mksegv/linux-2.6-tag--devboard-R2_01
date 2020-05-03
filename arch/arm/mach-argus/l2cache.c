/*
 * L2-cache helper functions
 *
 * Copyright (C) 1999-2005 Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/arch/argus.h>
#include "l2cache.h"

//----------------------------------------------------------------------------
// L1-cache control (Argus RTOS style functions)

#define CP15_CTRL_MMU         (1<<0)
#define CP15_CTRL_ALIGN       (1<<1)
#define CP15_CTRL_DFILL       (1<<2)
#define CP15_CTRL_WB          (1<<3)
#define CP15_CTRL_MAC_DISABLE (1<<4)
#define CP15_CTRL_PTAB_CACHE  (1<<7)
#define CP15_CTRL_SYSTEM      (1<<8)
#define CP15_CTRL_ROM         (1<<9)
#define CP15_CTRL_BUSLOCK     (1<<10)               
#define CP15_CTRL_IFILL       (1<<12)
#define CP15_CTRL_HIVECS      (1<<13)
#define CP15_CTRL_EXT_PERM    (1<<15)               

static __inline int cp15_GetControl(void)
{
  int ctrl;
  __asm__ __volatile__("mrc p15,0,%0,c1,c0,0" : "=r" (ctrl));
  return ctrl;
}

static __inline void cp15_SetControl(int ctrl)
{
  __asm__ __volatile__("mrc p15,0,%0,c1,c0,0" : : "r" (ctrl) : "memory");
}

static __inline void cp15_FlushDCache(void)
{
  __asm__ __volatile__("mcr p15,0,r0,c7,c6,0" : : : "memory");
}

static __inline void cp15_DrainWB(void)
{
  __asm__ __volatile__("mcr p15,0,r0,c7,c10,0" : : : "memory");
}

//----------------------------------------------------------------------------
// IRQ (Argus RTOS style functions)

static __inline int irq_off(void)
{
  int old_cpsr;
  __asm__ __volatile__(
    "mrs     %0,CPSR\n"
    "orr     r2,%0,#0x80\n"
    "msr     CPSR_c,r2\n"  : "=r" (old_cpsr) : : "r2");
  return old_cpsr;
}

static __inline void irq_set(int old_cpsr)
{
  __asm__ __volatile__( "msr     CPSR_c,%0\n" : : "r" (old_cpsr) );
}

//----------------------------------------------------------------------------

void l2cache_init(int nRams, int nIxRed)
{
  int old_irqs= irq_off();

  if ( !GET_L2CACHE_MODE_BYPASS(SOC_L2CACHE.MODE) )
    // Clean the cache if already enabled
  	l2cache_CleanAll();
	
	l2cache_FlushAll();
	SOC_L2CACHE.MODE=
    SET_L2CACHE_MODE_IXRED(nIxRed) | 
    SET_L2CACHE_MODE_ASSRED(4-nRams) |  
    SET_L2CACHE_MODE_IFILL(1) | 
    SET_L2CACHE_MODE_LOCK(0) |
    SET_L2CACHE_MODE_WB(1) |    
    SET_L2CACHE_MODE_DFILL(1);
  irq_set(old_irqs);  
}

// Lock specified memory range into the l2cache
// Calling with (0,0) releases the locking
void l2cache_LockRange(void *base, int size)
{
  int old_irqs= irq_off();
  int old_mode= SOC_L2CACHE.MODE;
  int old_ctrl= cp15_GetControl();

  // Disable first-level D-fills
  cp15_SetControl(old_ctrl & ~CP15_CTRL_DFILL);
  cp15_FlushDCache();

  // Disable second level I-fills
	SOC_L2CACHE.MODE= old_mode & ~SET_L2CACHE_MODE_IFILL(1);

  // Start by cleaning and flushing
  l2cache_CleanAll();
  l2cache_FlushAll();

  // Pull it into the L2 cache
  if (size)
  {
    int i;
    
    // Enter locking mode
    SOC_L2CACHE.MODE |= SET_L2CACHE_MODE_LOCK(1);

    for (i= 0; i<size+31; i+=32)
      (void) ((volatile char *)base) [i]; // dummy read 
  }

  // Restore second and first level cache settings
	SOC_L2CACHE.MODE= old_mode; // leave locking mode
  cp15_SetControl(old_ctrl);
  irq_set(old_irqs);
}

