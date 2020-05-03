/*
 * linux/include/asm-arm/arch-argus/system.h
 *
 * Copyright (C) 2003 Axis AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/io.h>
#include <asm/arch/argus.h>

static void arch_reset(char mode)
{
	// reset the machine
	SOC_RESET.SWRESET = 1;
}

static inline void arch_idle(void)
{
	// put the cpu into sleep-mode until an interrupt occurs
	cpu_do_idle();
}

#endif
