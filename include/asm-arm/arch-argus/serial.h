/*
 * linux/include/asm-arm/arch-argus/serial.h
 *
 *  Copyright (C) 2003 Axis AB and Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H

/*
 * This assumes you have a 1.8432 MHz clock for your UART.
 *
 * It'd be nice if someone built a serial card with a 24.576 MHz
 * clock, since the 16550A is capable of handling a top speed of 1.5
 * megabits/second; but this requires the faster clock.
 */
#define BASE_BAUD (1843200 / 16)

#define STD_COM_FLAGS (ASYNC_SKIP_TEST)

#define RS_TABLE_SIZE 2

     /* UART CLK        PORT  IRQ     FLAGS        */
#define STD_SERIAL_PORT_DEFNS \
	{ 0, BASE_BAUD, -1, 0, STD_COM_FLAGS },	/* ttyS0 */	\
	{ 0, BASE_BAUD, -1, 0, STD_COM_FLAGS },	/* ttyS0 */

#define EXTRA_SERIAL_PORT_DEFNS

#endif

