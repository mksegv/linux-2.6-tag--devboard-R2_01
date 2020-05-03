/*
 *  Copyright (C) 2003,2004 Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __BOOTINFO_H__
#define __BOOTINFO_H__

#include <asm/arch/argus.h>

#define NBR_CSS 6

struct bootinfo_header
{
	int magic; 
	int chip_id;
	int dummy;
	int start_address;
	int preskip;
	int vmm_info;
	int memconfig[NBR_CSS];
	int sdctrl1;
	int sdctrl2;
	int sdrfsh;
	int sdmode;
	char gpio_config[NUM_GPIO];
	int debug_port;
};

#endif
