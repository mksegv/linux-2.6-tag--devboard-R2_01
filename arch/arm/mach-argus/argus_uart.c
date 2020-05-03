/*
 *  linux/arch/arm/mach-argus/argus_uart.c
 *
 *  This file contains a basic serial port output driver
 *  used for debugging output. It registers an initial console.
 *
 * Copyright (C) 1999-2004 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/major.h>

#include <asm/arch/argus.h>
#include <asm/arch/hardware.h>

// If this is defined, the input pins (RX, CTS) are configured
// as input pins if the serial port is enabled. If it is undefined,
// it is up to the kernel config to set the right mode for the pins
// but this can be necessary if for example there are no pull-ups on
// the inputs.. 

#ifdef CONFIG_ARGUS_SERIAL_FORCE_RX_ENABLE
#define FORCE_ENABLE_RX_PINS
#endif

#if defined(CONFIG_ARGUS_DEBUG_PORT0) || defined(CONFIG_ARGUS_DEBUG_PORT_NULL)
#define DEBUG_PORT_IDX 0
#else
#define DEBUG_PORT_IDX 1
#endif

#if defined(CONFIG_ARGUS_DEBUG_PORT_BAUD)
#define DEBUG_PORT_BAUD CONFIG_ARGUS_DEBUG_PORT_BAUD
#else
#define DEBUG_PORT_BAUD 115200
#endif

typedef struct _argus_uart_descr
{
	volatile tUART	                *uart;
	int				timeout;
	int				tat;
} argus_uart_descr;


static argus_uart_descr g_argus_uart_descr;

#if 0
// TODO: We skip support for incoming bytes for now.
static int 
poll_get_char()
{
	unsigned int t0;
	volatile tUART *uart = g_argus_uart_descr.uart;

	// wait for incoming byte or timeout
	t0 = SOC_TIMER.COUNTER;
	while(!GET_USR_RFNE(uart->USR) && (APB.TIMER.TCR - t0) < 1000 * g_argus_uart_descr.timeout);
	if (GET_USR_RFNE(uart->USR))
		return *(g_argus_uart_descr.uart_read);
	else
		return -1; // timeout
}
#endif

static int 
poll_put_char(int ch)
{
	volatile tUART *uart = g_argus_uart_descr.uart;
	// wait until tx_fifo is not full
	while(!(GET_UART_TXNFULL(uart->STATUS))); 
	// write byte
	uart->DATA = ch;
	// wait until tx_fifo is empty
	while(!(GET_UART_TXEMPTY(uart->STATUS)));
	return 0;
}

#if 0
static void 
poll_put_string(char *str)
{
	int	i = 0;
	
	while (str[i] != 0) {
		poll_put_char(str[i]);
		i++;
	}
}
#endif

static void 
uart_setup(volatile tUART *uart, int baudrate)
{
	// Enable UART clocks

	SOC_CLK.ENABLE = CLK_UART0 | CLK_UART1;

	// Init UART here
	uart->CTRL = SET_UART_MODE(0) |
		SET_UART_AUTOFLOW(0) |
		SET_UART_TXPOL(0) |
		SET_UART_RXPOL(0) |
		SET_UART_STOPBITS(0) |
		SET_UART_LOOPBACK(0);

	switch(baudrate) {
		case 9600:
			uart->BAUD = SET_UART_BAUD(UART_BAUD_9600);
			uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(6)       |
				SET_UART_RXEND(5)         |
				SET_UART_IRDARXLEN(15);
			break;
		case 115200:
			uart->BAUD = SET_UART_BAUD(UART_BAUD_115200);
			uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(6)       |
				SET_UART_RXEND(5)         |
				SET_UART_IRDARXLEN(15);
			break;
		case 230400:
			uart->BAUD = SET_UART_BAUD(UART_BAUD_230400);
			uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(6)       |
				SET_UART_RXEND(5)         |
				SET_UART_IRDARXLEN(15);
			break;
		case 460800:
			uart->BAUD = SET_UART_BAUD(UART_BAUD_460800);
			uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(4)       |
				SET_UART_RXEND(3)         |
				SET_UART_IRDARXLEN(1);
			break;
	}

	// For now, enable only transmission.

	uart->ENABLE = SET_UART_RXEN(0) | SET_UART_TXEN(1);

}

static void 
argus_uart_init(int uart_no, int baudrate)
{
	int i;
	g_argus_uart_descr.timeout = 185;	// ms
	g_argus_uart_descr.tat = 0;		// ms . No turnaround when RS232
	if (uart_no == 0) {
		g_argus_uart_descr.uart = &SOC_UART[0];
#ifdef FORCE_ENABLE_RX_PINS
                SOC_GPIO.PIN[GPIO_UART0_RXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
#endif
                SOC_GPIO.PIN[GPIO_UART0_TXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
	} else {
		g_argus_uart_descr.uart = &SOC_UART[1];
#ifdef FORCE_ENABLE_RX_PINS
		SOC_GPIO.PIN[GPIO_UART1_RXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
#endif
                SOC_GPIO.PIN[GPIO_UART1_TXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
	}

	uart_setup(g_argus_uart_descr.uart, baudrate);
	for(i = 0; i < 5000000; i++) ;
}

// This function is called from the kernel core just before the init thread and
// usermode console starts, to switch over printks from using the low-level debug
// console output functions to the actual serial driver. We should return the
// same driver we set up in argus_serial.c.

extern struct tty_driver *serial_driver;

static struct tty_driver *
argus_console_device(struct console *c, int *index)
{
	*index = c->index;
         return serial_driver;
}

static int __init 
console_setup(struct console *co, char *options)
{
#ifndef CONFIG_ARGUS_DEBUG_PORT_NULL
	argus_uart_init(DEBUG_PORT_IDX, DEBUG_PORT_BAUD);
#endif
        return 0;
}

static void 
console_write(struct console *co, const char *buf, unsigned int len)
{
	int i;

#ifdef CONFIG_ARGUS_DEBUG_PORT_NULL
        return;
#endif
        
	for(i = 0; i < len; i++)
		poll_put_char(buf[i]);
}

static struct console sercons = {
        name : "ttyS",
        write: console_write,
        read : NULL,
        device : argus_console_device,
	unblank : NULL,
	setup : console_setup,
	flags : CON_PRINTBUFFER,
	index : DEBUG_PORT_IDX,
	cflag : 0,
	next : NULL
};

/*
 *      Register console (for printk's etc)
 *      Called by arch.c
 */

void __init 
init_argus_debug(void)
{
	/* At this point the virtual APB address is not mapped. */
	register_console(&sercons);
}
