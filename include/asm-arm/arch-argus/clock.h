/*
 *  Copyright (C) 2003-2004 Axis AB and Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARGUS_CLOCK_H
#define ARGUS_CLOCK_H

#include <asm/arch/argus.h>

static inline void
clock_enable(int clknbr, int on)
{
	// TODO: this function might want to do some sort of
	// enable reference counting.
	if(on)
		SOC_CLK.ENABLE = clknbr;
	else
		SOC_CLK.DISABLE = clknbr;
}

#endif
