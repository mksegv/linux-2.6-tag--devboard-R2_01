/*
 * serial.h: Arch-dep definitions for the Argus serial driver.
 *
 * Copyright (C) 1999-2004 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ARGUS_UART_H
#define _ARGUS_UART_H

#include <linux/config.h>
#include <linux/circ_buf.h>
#include <asm/termios.h>

/* Software state per channel */

#ifdef __KERNEL__
/*
 * This is our internal structure for each serial port's state.
 * 
 * Many fields are paralleled by the structure used by the serial_struct
 * structure.
 *
 * For definitions of the flags field, see tty.h
 */

#define SERIAL_RECV_DESCRIPTORS 8

struct argus_recv_buffer {
	struct argus_recv_buffer *next;
	unsigned short length;
	unsigned char error;
	unsigned char pad;

	unsigned char buffer[0];
};

struct argus_serial {
	int			baud;

	tUART                   *uart;  // The APB SOC_UART pointer
	unsigned int            IRQEN_shadow;

	int			flags;	/* defined in tty.h */

	int			enabled; /* Set to 1 if the port is enabled in HW config */
  
	/* end of fields defined in rs_table[] in .c-file */

	int			uses_dma; /* Set to 1 if DMA should be used */
	unsigned char           forced_eop; /* a fifo eop has been forced */

	//struct etrax_dma_descr	tr_descr;
	//struct etrax_dma_descr	rec_descr[SERIAL_RECV_DESCRIPTORS];
	int			cur_rec_descr;

	volatile int		tr_running; /* 1 if output is running */

	struct tty_struct	*tty;
	int			read_status_mask;
	int			ignore_status_mask;
	int			x_char;	/* xon/xoff character */
	int			close_delay;
	unsigned short		closing_wait;
	unsigned short		closing_wait2;
	unsigned long		event;
	unsigned long		last_active;
	int			line;
	int			type;  /* PORT_ARGUS */
	int			count;	    /* # of fd on device */
	int			blocked_open; /* # of blocked opens */
	struct circ_buf		xmit;
	struct argus_recv_buffer *first_recv_buffer;
	struct argus_recv_buffer *last_recv_buffer;
	unsigned int		recv_cnt;
	unsigned int		max_recv_cnt;

	struct work_struct	work;
	struct async_icount	icount;   /* error-statistics etc.*/
	struct termios		normal_termios;
#ifdef DECLARE_WAITQUEUE
	wait_queue_head_t	open_wait;
	wait_queue_head_t	close_wait;
#else
	struct wait_queue	*open_wait;
	struct wait_queue	*close_wait;
#endif  

	unsigned long		char_time_usec;       /* The time for 1 char, in usecs */
	unsigned long		last_tx_active_usec;  /* Last tx usec in the jiffies */
	unsigned long		last_tx_active;       /* Last tx time in jiffies */
	unsigned long		last_rx_active_usec;  /* Last rx usec in the jiffies */
	unsigned long		last_rx_active;       /* Last rx time in jiffies */

	int			break_detected_cnt;
	int			errorcode;

};

/* this PORT is not in the standard serial.h. it's not actually used for
 * anything since we only have one type of async serial-port anyway in this
 * system.
 */

#define PORT_ARGUS 1

/*
 * Events are used to schedule things to happen at timer-interrupt
 * time, instead of at rs interrupt time.
 */
#define RS_EVENT_WRITE_WAKEUP	0

#endif /* __KERNEL__ */

#endif /* !_ARGUS_UART_H */
