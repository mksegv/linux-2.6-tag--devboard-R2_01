/*
 *  linux/arch/arm/mach-argus/dma.c
 *
 *  Copyright (C) 2003 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/sched.h>
#include <linux/init.h>

#include <asm/dma.h>
#include <asm/io.h>

#include <asm/mach/dma.h>

void __init 
arch_dma_init(dma_t *dma)
{
	// do any dma initialization... 
}
