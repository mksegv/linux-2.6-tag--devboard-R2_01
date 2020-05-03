/* $Id: crisv32.c,v 1.55 2005/07/20 11:51:40 starvik Exp $
 *
 * Serial port driver for the ETRAX FS chip
 *
 *    Copyright (C) 1998-2005  Axis Communications AB
 *
 *    Many, many authors. Based once upon a time on serial.c for 16x50.
 *
 *    Johan Adolfsson - port to ETRAX FS
 *    Mikael Starvik - port to serial_core framework
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/uaccess.h>

#include <asm/arch/dma.h>
#include <asm/arch/system.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/hwregs/dma.h>
#include <asm/arch/hwregs/reg_rdwr.h>
#include <asm/arch/hwregs/ser_defs.h>
#include <asm/arch/hwregs/dma_defs.h>
#include <asm/arch/hwregs/gio_defs.h>
#include <asm/arch/hwregs/intr_vect_defs.h>
#include <asm/arch/hwregs/reg_map.h>

#define UART_NR 5 /* 4 ports + dummy port */
#define SERIAL_RECV_DESCRIPTORS 8
#define SERIAL_TX_DESCRIPTORS 8
#define ETRAX_SER_FIFO_SIZE 1 /* Or perhaps 2 */
#define SERIAL_DESCR_BUF_SIZE 256
#define regi_NULL 0
#define DMA_WAIT_UNTIL_RESET(inst) do {  reg_dma_rw_stat r; do { r = REG_RD(dma, (inst), rw_stat); }while(r.mode != regk_dma_rst); }while(0)

/* Macro to set up control lines for a port. */
#define SETUP_PINS(port) \
	if (serial_cris_ports[port].used) { \
	if (strcmp(CONFIG_ETRAX_SER##port##_DTR_BIT, "")) \
		crisv32_io_get_name(&serial_cris_ports[port].dtr_pin, CONFIG_ETRAX_SER##port##_DTR_BIT); \
	else \
		serial_cris_ports[port].dtr_pin = dummy_pin; \
	if (strcmp(CONFIG_ETRAX_SER##port##_DSR_BIT, "")) \
		crisv32_io_get_name(&serial_cris_ports[port].dsr_pin, CONFIG_ETRAX_SER##port##_DSR_BIT); \
	else \
		serial_cris_ports[port].dsr_pin = dummy_pin; \
	if (strcmp(CONFIG_ETRAX_SER##port##_RI_BIT, "")) \
		crisv32_io_get_name(&serial_cris_ports[port].ri_pin, CONFIG_ETRAX_SER##port##_RI_BIT); \
	else \
		serial_cris_ports[port].ri_pin = dummy_pin; \
	if (strcmp(CONFIG_ETRAX_SER##port##_CD_BIT, "")) \
		crisv32_io_get_name(&serial_cris_ports[port].cd_pin, CONFIG_ETRAX_SER##port##_CD_BIT); \
	else \
		serial_cris_ports[port].cd_pin = dummy_pin; \
	}

/* Set a serial port register if anything has changed. */
#define MODIFY_REG(instance, reg, var) \
  if (REG_RD_INT(ser, instance, reg) != REG_TYPE_CONV(int, reg_ser_##reg, var)) \
      REG_WR(ser, instance, reg, var);

struct etrax_recv_buffer {
	struct etrax_recv_buffer *next;
	unsigned short length;
	unsigned char error;
	unsigned char pad;

	unsigned char buffer[0];
};

struct uart_cris_port {
	struct uart_port	port;

	int initialized;
	int used;
	int irq;

	reg_scope_instances	regi_ser; /* Used to check if port enabled as well */
	reg_scope_instances	regi_dmain;
	reg_scope_instances	regi_dmaout;

	struct crisv32_iopin    dtr_pin;
	struct crisv32_iopin    dsr_pin;
	struct crisv32_iopin    ri_pin;
	struct crisv32_iopin    cd_pin;

	struct dma_descr_context tr_context_descr __attribute__ ((__aligned__(32)));
	struct dma_descr_data	 tr_descr[SERIAL_TX_DESCRIPTORS] __attribute__ ((__aligned__(32)));
  	struct dma_descr_context rec_context_descr __attribute__ ((__aligned__(32)));
	struct dma_descr_data	 rec_descr[SERIAL_RECV_DESCRIPTORS] __attribute__ ((__aligned__(32)));
	struct dma_descr_data*   first_tx_descr;
	struct dma_descr_data*   last_tx_descr;
	int			 tx_started; 
	int			 cur_rec_descr;
	struct etrax_recv_buffer *first_recv_buffer;
	struct etrax_recv_buffer *last_recv_buffer;

	unsigned int		recv_cnt;
	unsigned int		max_recv_cnt;

	unsigned long		char_time_usec;       /* The time for 1 char, in usecs */
	unsigned long		last_tx_active_usec;  /* Last tx usec in the jiffies */
	unsigned long		last_tx_active;       /* Last tx time in jiffies */
	unsigned long		last_rx_active_usec;  /* Last rx usec in the jiffies */
	unsigned long		last_rx_active;       /* Last rx time in jiffies */
	unsigned long		transmitter_running;
	
#ifdef CONFIG_ETRAX_RS485
	struct rs485_control	rs485;  /* RS-485 support */
#endif
};

extern struct uart_driver serial_cris_driver;

static struct uart_cris_port serial_cris_ports[UART_NR] = {
{
#ifdef CONFIG_ETRAX_SERIAL_PORT0
	.used        = 1,
	.irq         = SER0_INTR_VECT,
	.regi_ser    = regi_ser0,
	/* We initilise the dma stuff like this to get a compiler error
	 * if a CONFIG is missing 
	 */
	.regi_dmain  = 
#  ifdef CONFIG_ETRAX_SERIAL_PORT0_DMA7_IN
	               regi_dma7,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT0_NO_DMA_IN
	               regi_NULL,
#  endif

	.regi_dmaout =
#  ifdef CONFIG_ETRAX_SERIAL_PORT0_DMA6_OUT
	               regi_dma6,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT0_NO_DMA_OUT
	               regi_NULL,
#  endif
#else
	.regi_ser    = regi_NULL,
	.regi_dmain  = regi_NULL,
	.regi_dmaout = regi_NULL,
#endif
},  /* ttyS0 */
{ 
#ifdef CONFIG_ETRAX_SERIAL_PORT1
	.used        = 1,
	.irq         = SER1_INTR_VECT,
	.regi_ser    = regi_ser1,
	.regi_dmain  = 
#  ifdef CONFIG_ETRAX_SERIAL_PORT1_DMA5_IN
	               regi_dma5,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT1_NO_DMA_IN
	               regi_NULL,
#  endif

	 .regi_dmaout = 
#  ifdef CONFIG_ETRAX_SERIAL_PORT1_DMA4_OUT
	               regi_dma4,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT1_NO_DMA_OUT
	               regi_NULL,
#  endif
#else
	.regi_ser    = regi_NULL,
	.regi_dmain  = regi_NULL,
	.regi_dmaout = regi_NULL,
#endif
},  /* ttyS1 */
{ 
#ifdef CONFIG_ETRAX_SERIAL_PORT2
	.used       = 1,
	.irq        = SER2_INTR_VECT,
	.regi_ser    = regi_ser2,
	.regi_dmain  = 
#  ifdef CONFIG_ETRAX_SERIAL_PORT2_DMA3_IN
	               regi_dma3,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT2_NO_DMA_IN
	               regi_NULL,
#  endif

	 .regi_dmaout = 
#  ifdef CONFIG_ETRAX_SERIAL_PORT2_DMA2_OUT
	               regi_dma2,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT2_NO_DMA_OUT
	               regi_NULL,
#  endif
#else
	.regi_ser    = regi_NULL,
	.regi_dmain  = regi_NULL,
	.regi_dmaout = regi_NULL,
#endif
},  /* ttyS2 */
{ 
#ifdef CONFIG_ETRAX_SERIAL_PORT3
	.used       = 1,
	.irq        = SER3_INTR_VECT,
	.regi_ser    = regi_ser3,
	.regi_dmain  = 
#  ifdef CONFIG_ETRAX_SERIAL_PORT3_DMA9_IN
	               regi_dma9,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT3_NO_DMA_IN
	               regi_NULL,
#  endif

	 .regi_dmaout = 
#  ifdef CONFIG_ETRAX_SERIAL_PORT3_DMA8_OUT
	               regi_dma8,
#  endif
#  ifdef CONFIG_ETRAX_SERIAL_PORT3_NO_DMA_OUT
	               regi_NULL,
#  endif
#else
	.regi_ser    = regi_NULL,
	.regi_dmain  = regi_NULL,
	.regi_dmaout = regi_NULL,
#endif
},  /* ttyS3 */
{
#ifdef CONFIG_ETRAX_DEBUG_PORT_NULL
	.used        = 1,
#endif
	.regi_ser    = regi_NULL
}   /* Dummy console port */

};

/* Dummy pin used for unused CD, DSR, DTR and RI signals */
static unsigned long io_dummy;
static struct crisv32_ioport dummy_port =
{
	&io_dummy,
	&io_dummy,
	&io_dummy,
	18
};
static struct crisv32_iopin dummy_pin =
{
	&dummy_port,
	0
};

static int selected_console = 
#if defined(CONFIG_ETRAX_DEBUG_PORT0)
0;
#elif defined(CONFIG_ETRAX_DEBUG_PORT1)
1;
#elif defined(CONFIG_ETRAX_DEBUG_PORT2)
2;
#elif defined(CONFIG_ETRAX_DEBUG_PORT3)
3;
#else
4;
#endif

/*
 * Interrupts are disabled on entering
 */
static void
cris_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_cris_port *up;
	int i;
	reg_ser_r_stat_din stat;
	reg_ser_rw_tr_dma_en tr_dma_en, old;

	up = &serial_cris_ports[selected_console];
	if (!up->regi_ser)
		return;

	/* Switch to manual mode */
	tr_dma_en = old = REG_RD (ser, up->regi_ser, rw_tr_dma_en);
	if (tr_dma_en.en == regk_ser_yes) {
		tr_dma_en.en = regk_ser_no;
		REG_WR(ser, up->regi_ser, rw_tr_dma_en, tr_dma_en);
	}

	/* Send data */
	for (i = 0; i < count; i++) {
		/* LF -> CRLF */
		if (s[i] == '\n') {
			do {
				stat = REG_RD (ser, up->regi_ser, r_stat_din);
			} while (!stat.tr_rdy);
			REG_WR_INT (ser, up->regi_ser, rw_dout, '\r'); 
		}
		/* Wait until transmitter is ready and send.*/
		do {
			stat = REG_RD (ser, up->regi_ser, r_stat_din);
		} while (!stat.tr_rdy);
		REG_WR_INT (ser, up->regi_ser, rw_dout, s[i]);
	}

	/* Restore mode */
	if (tr_dma_en.en != old.en)
		REG_WR(ser, up->regi_ser, rw_tr_dma_en, old);
}

static void cris_serial_port_init(struct uart_port *port, int line);
static int __init
cris_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= UART_NR)
		co->index = 0;
	if (options)
		selected_console = co->index;     
	port = &serial_cris_ports[selected_console].port;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	cris_serial_port_init(port, selected_console);
	co->index = port->line;
	uart_set_options(port, co, baud, parity, bits, flow);

	return 0;
}

static struct tty_driver*
cris_console_device(struct console* co, int *index)
{
	struct uart_driver *p = co->data;
	*index = selected_console;
	return p->tty_driver;
}

static struct console cris_console = {
	.name		= "ttyS",
	.write		= cris_console_write,
	.device		= cris_console_device,
	.setup		= cris_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_cris_driver,
};

#define SERIAL_CRIS_CONSOLE	&cris_console

struct uart_driver serial_cris_driver = {
	.owner			= THIS_MODULE,
	.driver_name		= "serial",
	.devfs_name		= "tts/",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 64,
	.nr			= UART_NR,
	.cons			= SERIAL_CRIS_CONSOLE,
};

static int inline crisv32_serial_get_rts(struct uart_cris_port *up)
{
	reg_scope_instances regi_ser = up->regi_ser;
	if (regi_ser) {
		/* Return what the user has controlled rts to or 
		 * what the pin is? (if auto_rts is used it differs during tx)
		 */
		reg_ser_r_stat_din rstat;
		rstat = REG_RD(ser, regi_ser, r_stat_din);
		return !(rstat.rts_n == regk_ser_active);
	}
	return 1;
}

/* set = 0 means 3.3V on the pin, bitvalue: 0=active, 1=inactive  
 *                                          0=0V    , 1=3.3V
 */
static inline void crisv32_serial_set_rts(struct uart_cris_port *up, int set)
{
	reg_scope_instances regi_ser = up->regi_ser;
	if (regi_ser) {
		unsigned long flags;
		reg_ser_rw_rec_ctrl rec_ctrl;
		local_irq_save(flags);
		rec_ctrl = REG_RD(ser, regi_ser, rw_rec_ctrl);
		if (set)
			rec_ctrl.rts_n = regk_ser_active;
		else
			rec_ctrl.rts_n = regk_ser_inactive;
		REG_WR(ser, regi_ser, rw_rec_ctrl, rec_ctrl);
		local_irq_restore(flags);
	}	
}

/* Input */
static int inline crisv32_serial_get_cts(struct uart_cris_port *up)
{
	reg_scope_instances regi_ser = up->regi_ser;
	if (regi_ser) {
		reg_ser_r_stat_din rstat;
		rstat = REG_RD(ser, regi_ser, r_stat_din);
		return !(rstat.cts_n == regk_ser_active);
	}
	return 1;
}

static void transmit_chars_dma(struct uart_cris_port *up);
static void serial_cris_start_tx(struct uart_port *port, unsigned int tty_start)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	reg_scope_instances regi_ser = up->regi_ser;
	unsigned long flags;

	if (regi_ser) {
		reg_ser_rw_tr_ctrl tr_ctrl = REG_RD(ser, regi_ser, rw_tr_ctrl);
		tr_ctrl.stop = regk_ser_no;
		REG_WR(ser, regi_ser, rw_tr_ctrl, tr_ctrl);
		if (!up->regi_dmaout) {
			reg_ser_rw_intr_mask intr_mask = REG_RD(ser, regi_ser, rw_intr_mask);
			REG_WR(ser, regi_ser, rw_tr_ctrl, tr_ctrl);
			intr_mask.tr_rdy = regk_ser_yes;
			REG_WR(ser, regi_ser, rw_intr_mask, intr_mask);
		} else {
			spin_lock_irqsave(&port->lock, flags);	
			transmit_chars_dma(up);
			spin_unlock_irqrestore(&port->lock, flags);
		}
	} else {
		/* Discard characters */
		struct circ_buf *xmit = &up->port.info->xmit;
		xmit->tail = xmit->head;
	}
}

static void serial_cris_stop_tx(struct uart_port *port, unsigned int tty_stop)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	reg_scope_instances regi_ser = up->regi_ser;
	reg_ser_rw_tr_ctrl tr_ctrl;
	reg_ser_r_stat_din rstat;
	reg_ser_rw_intr_mask intr_mask;
	unsigned long flags;

	if (!regi_ser || up->regi_dmaout)
		return;

	tr_ctrl = REG_RD(ser, regi_ser, rw_tr_ctrl);
	intr_mask = REG_RD(ser, regi_ser, rw_intr_mask);

	spin_lock_irqsave(&up->port.lock, flags);
	intr_mask.tr_rdy = regk_ser_no;
	REG_WR(ser, regi_ser, rw_intr_mask, intr_mask);

	tr_ctrl.stop = 1;
	REG_WR(ser, up->regi_ser, rw_tr_ctrl, tr_ctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/* Wait for current data sent so we don't cripple it */
	do {
		rstat = REG_RD(ser, up->regi_ser, r_stat_din);
	}while(!rstat.tr_empty);

	/* Go to manual mode and set the txd pin to 0 */
	spin_lock_irqsave(&up->port.lock, flags);
	tr_ctrl = REG_RD(ser, up->regi_ser, rw_tr_ctrl);
	tr_ctrl.stop = 1;
	tr_ctrl.txd = 0;

	REG_WR(ser, up->regi_ser, rw_tr_ctrl, tr_ctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void serial_cris_stop_rx(struct uart_port *port)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	reg_scope_instances regi_ser = up->regi_ser;

	if (regi_ser) {
		reg_ser_rw_rec_ctrl rec_ctrl = REG_RD(ser, regi_ser, rw_rec_ctrl);
		rec_ctrl.en = regk_ser_no;
		REG_WR(ser, regi_ser, rw_rec_ctrl, rec_ctrl);
	}
}

static void serial_cris_enable_ms(struct uart_port *port)
{
}

static void check_modem_status(struct uart_cris_port *up)
{
}

static unsigned int serial_cris_tx_empty(struct uart_port *port)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	struct circ_buf *xmit = &up->port.info->xmit;
	unsigned long flags;
	unsigned int ret;
	reg_ser_r_stat_din rstat;
	rstat = REG_RD(ser, up->regi_ser, r_stat_din);

	if (!up->regi_ser)
		return 1;

	/* tr_empty is only valid when DMA has stopped */
	spin_lock_irqsave(&up->port.lock, flags);
	ret = rstat.tr_empty;
	if (ret && up->regi_dmaout) {
		if (CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE))	
			ret = 0;
        }
        
	spin_unlock_irqrestore(&up->port.lock, flags);
	
	return ret;
}
static unsigned int serial_cris_get_mctrl(struct uart_port *port)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	unsigned int ret;

	if (!up->regi_ser)
		return 0;

	ret = 0;
        if (crisv32_serial_get_rts(up))
		ret |= TIOCM_RTS;
	if (crisv32_io_rd(&up->dtr_pin))
		ret |= TIOCM_DTR;
	if (crisv32_io_rd(&up->ri_pin))
		ret |= TIOCM_RNG;
	if (crisv32_io_rd(&up->dsr_pin))
		ret |= TIOCM_DSR;
	if (crisv32_serial_get_cts(up))
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_cris_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;	

	if (!up->regi_ser)
		return;

	crisv32_serial_set_rts(up, mctrl & TIOCM_RTS ? 1 : 0);
	crisv32_io_set(&up->dtr_pin, mctrl & TIOCM_DTR ? 1 : 0);
	crisv32_io_set(&up->ri_pin, mctrl & TIOCM_RNG ? 1 : 0);
	crisv32_io_set(&up->cd_pin, mctrl & TIOCM_CD ? 1 : 0);
}

static void serial_cris_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	unsigned long flags;
	reg_ser_rw_tr_ctrl tr_ctrl;
	reg_ser_rw_tr_dma_en tr_dma_en;
	reg_ser_r_stat_din rstat;

	if (!up->regi_ser)
		return;

	spin_lock_irqsave(&up->port.lock, flags);

	if (break_state == -1) { /* Send break */
		/* Stop transmission */
		tr_ctrl = REG_RD(ser, up->regi_ser, rw_tr_ctrl);
		tr_dma_en = REG_RD(ser, up->regi_ser, rw_tr_dma_en);
		tr_ctrl.stop = 1;
		/* Disable DMA if used or tr_rdy interrupts if no DMA */
		if (up->regi_dmaout) {
			tr_dma_en.en = 0;
		} else{
			/* Disable tr_rdy interrupt so we don't have 
			 * to wait until all sent 
			 */
			reg_ser_rw_intr_mask intr_mask = REG_RD(ser, up->regi_ser, rw_intr_mask);
			intr_mask.tr_rdy = regk_ser_no;
			REG_WR(ser, up->regi_ser, rw_intr_mask, intr_mask);
		}
		REG_WR(ser, up->regi_ser, rw_tr_ctrl, tr_ctrl);
		REG_WR(ser, up->regi_ser, rw_tr_dma_en, tr_dma_en);

		spin_unlock_irqrestore(&up->port.lock, flags);

		/* Wait for current data sent so we don't cripple it */
		do {
			rstat = REG_RD(ser, up->regi_ser, r_stat_din);
		}while(!rstat.tr_empty);

		/* Go to manual mode and set the txd pin to 0 */
		spin_lock_irqsave(&up->port.lock, flags);
		tr_ctrl = REG_RD(ser, up->regi_ser, rw_tr_ctrl);
		tr_ctrl.stop = 1;
		tr_ctrl.txd = 0;
	} else {
		tr_ctrl = REG_RD(ser, up->regi_ser, rw_tr_ctrl);
		tr_dma_en = REG_RD(ser, up->regi_ser, rw_tr_dma_en);
		if (up->regi_dmaout) {
			tr_dma_en.en = 1;
		} else {
			reg_ser_rw_intr_mask intr_mask = REG_RD(ser, up->regi_ser, rw_intr_mask);
			intr_mask.tr_rdy = regk_ser_no;
			REG_WR(ser, up->regi_ser, rw_intr_mask, intr_mask);
		}
		 
		tr_ctrl.stop = 0;
		tr_ctrl.txd = 1;
	}
	REG_WR(ser, up->regi_ser, rw_tr_ctrl, tr_ctrl);
	REG_WR(ser, up->regi_ser, rw_tr_dma_en, tr_dma_en);

	spin_unlock_irqrestore(&up->port.lock, flags);
}

/* The output DMA channel is free - use it to send as many chars as possible
 * NOTES:
 *   We don't pay attention to up->port.x_char, which means if the TTY wants to
 *   use XON/XOFF it will set up->port.x_char but we won't send any X char!
 * 
 *   To implement this, we'd just start a DMA send of 1 byte pointing at a
 *   buffer containing the X char, and skip updating xmit. We'd also have to
 *   check if the last sent char was the X char when we enter this function
 *   the next time, to avoid updating xmit with the sent X value.
 */


static void
transmit_chars_dma(struct uart_cris_port *up)
{
	struct dma_descr_data *descr, *prev, *dmapos;
	struct circ_buf *xmit = &up->port.info->xmit;
	unsigned int c, sentl = 0;
	reg_dma_rw_ack_intr ack_intr = { .data = regk_dma_yes };
	reg_dma_rw_stat status;
	reg_scope_instances regi_dmaout = up->regi_dmaout;

	if (up->transmitter_running)
		return;
	
	/* acknowledge dma data descriptor irq */
	REG_WR(dma, regi_dmaout, rw_ack_intr, ack_intr);

	/* first get the amount of bytes sent during the last DMA transfer,
	   and update xmit accordingly */
	status = REG_RD(dma, regi_dmaout, rw_stat);
	if (status.list_state == regk_dma_data_at_eol || !up->tx_started)
		dmapos = phys_to_virt((int)up->last_tx_descr->next);
	else
		dmapos = phys_to_virt(REG_RD_INT(dma, regi_dmaout, rw_data));

	descr = up->first_tx_descr;
	while (descr != dmapos) {
		sentl += descr->after - descr->buf;
		descr->after = descr->buf = NULL;
		descr = phys_to_virt((int)descr->next);
        }

	up->first_tx_descr = descr;        
	descr = up->last_tx_descr;

	/* update stats */
	up->port.icount.tx += sentl;

	/* update xmit buffer */
	xmit->tail = (xmit->tail + sentl) & (UART_XMIT_SIZE - 1);

	/* find out the largest amount of consecutive bytes we want to send now */

	c = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (c <= 0) {
		/* our job here is done, don't schedule any new DMA transfer */
		serial_cris_stop_tx(&up->port, 0);
		return;
	}

	/* ok we can schedule a dma send of c chars starting at up->port.info->xmit.tail */
	/* set up the descriptor correctly for output */
	prev = descr;
	descr = phys_to_virt((int)descr->next);
	up->last_tx_descr = descr;
	descr->buf = (void*)virt_to_phys(xmit->buf + xmit->tail);
	descr->after = descr->buf + c;
	descr->eol  = 1;
	descr->out_eop = 0;
	descr->intr = 1;
	descr->wait = 0;
	descr->in_eop = 0;
	descr->md = 0;
	prev->eol = 0;
	up->transmitter_running = 1;
	if (!up->tx_started)
	{
		up->tx_started = 1;
		up->tr_context_descr.next = 0;
		up->tr_context_descr.saved_data = (dma_descr_data*)virt_to_phys(descr);
		up->tr_context_descr.saved_data_buf = descr->buf;
		DMA_START_CONTEXT(regi_dmaout, virt_to_phys(&up->tr_context_descr));
        }
        else
        {
		reg_dma_rw_cmd cmd = {.cont_data = regk_dma_yes};
		REG_WR(dma, regi_dmaout, rw_cmd, cmd);
        }

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);
        
	/* DMA is now running (hopefully) */
}

static void
transmit_chars_no_dma(struct uart_cris_port *up)
{
	int count;
	struct circ_buf *xmit = &up->port.info->xmit;

	reg_scope_instances regi_ser = up->regi_ser;
	reg_ser_r_stat_din rstat;
	reg_ser_rw_ack_intr ack_intr = { .tr_rdy = regk_ser_yes };

	if (up->port.x_char) {
		reg_ser_rw_dout dout = { .data = up->port.x_char };
		REG_WR(ser, regi_ser, rw_dout, dout);
		REG_WR(ser, regi_ser, rw_ack_intr, ack_intr);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		/* No more to send, disable int */
		reg_ser_rw_intr_mask intr_mask;
		intr_mask = REG_RD(ser, regi_ser, rw_intr_mask);
		intr_mask.tr_rdy = 0;
		intr_mask.tr_empty = 0;
		REG_WR(ser, regi_ser, rw_intr_mask, intr_mask);
		return;
	}

	count = ETRAX_SER_FIFO_SIZE;
	do {
		reg_ser_rw_dout dout = { .data = xmit->buf[xmit->tail] };
		REG_WR(ser, regi_ser, rw_dout, dout);
		REG_WR(ser, regi_ser, rw_ack_intr, ack_intr);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
		up->port.icount.tx++;
		if (xmit->head == xmit->tail)
			break;
		rstat = REG_RD(ser, regi_ser, r_stat_din);
	} while ((--count > 0) && rstat.tr_rdy);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);
} /* transmit_chars_no_dma */

static struct etrax_recv_buffer *
alloc_recv_buffer(unsigned int size)
{
	struct etrax_recv_buffer *buffer;

	if (!(buffer = kmalloc(sizeof *buffer + size, GFP_ATOMIC)))
		return NULL;

	buffer->next = NULL;
	buffer->length = 0;
	buffer->error = TTY_NORMAL;

	return buffer;
}

static void
append_recv_buffer(struct uart_cris_port *up, struct etrax_recv_buffer *buffer)
{
	unsigned long flags;

	local_irq_save(flags);

	if (!up->first_recv_buffer)
		up->first_recv_buffer = buffer;
	else
		up->last_recv_buffer->next = buffer;

	up->last_recv_buffer = buffer;

	up->recv_cnt += buffer->length;
	if (up->recv_cnt > up->max_recv_cnt)
		up->max_recv_cnt = up->recv_cnt;

	local_irq_restore(flags);
}

static int
add_char_and_flag(struct uart_cris_port *up, unsigned char data, unsigned char flag)
{
	struct etrax_recv_buffer *buffer;

	if (!(buffer = alloc_recv_buffer(4)))
		return 0;

	buffer->length = 1;
	buffer->error = flag;
	buffer->buffer[0] = data;
	
	append_recv_buffer(up, buffer);

	up->port.icount.rx++;

	return 1;
}

static void
flush_to_flip_buffer(struct uart_cris_port *up)
{
	struct tty_struct *tty;
	struct etrax_recv_buffer *buffer;
	unsigned int length;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);

	tty = up->port.info->tty;
	if (!up->first_recv_buffer || !tty) {
		spin_unlock_irqrestore(&up->port.lock, flags);
		return;
	}

	length = tty->flip.count;

	while ((buffer = up->first_recv_buffer) && length < TTY_FLIPBUF_SIZE) {
		unsigned int count = buffer->length;

		if (length + count > TTY_FLIPBUF_SIZE)
			count = TTY_FLIPBUF_SIZE - length;

		memcpy(tty->flip.char_buf_ptr + length, buffer->buffer, count);
		memset(tty->flip.flag_buf_ptr + length, TTY_NORMAL, count);
		tty->flip.flag_buf_ptr[length] = buffer->error;

		length += count;
		up->recv_cnt -= count;

		if (count == buffer->length) {
			up->first_recv_buffer = buffer->next;
			kfree(buffer);
		} else {
			buffer->length -= count;
			memmove(buffer->buffer, buffer->buffer + count, buffer->length);
			buffer->error = TTY_NORMAL;
		}
	}

	if (!up->first_recv_buffer)
		up->last_recv_buffer = NULL;

	tty->flip.count = length;

	spin_unlock_irqrestore(&up->port.lock, flags);

	/* this includes a check for low-latency */
	tty_flip_buffer_push(tty);
}

static unsigned int
handle_descr_data(struct uart_cris_port *up, struct dma_descr_data *descr, unsigned int recvl)
{
	struct etrax_recv_buffer *buffer = phys_to_virt((unsigned long)descr->buf) - sizeof *buffer;

	if (up->recv_cnt + recvl > 65536) {
		printk("%s Too much pending incoming serial data! Dropping %u bytes.\n", __FUNCTION__, recvl);
		return 0;
	}

	buffer->length = recvl;

	append_recv_buffer(up, buffer);

	flush_to_flip_buffer(up);

	if (!(buffer = alloc_recv_buffer(SERIAL_DESCR_BUF_SIZE)))
		panic("%s: Failed to allocate memory for receive buffer!\n", __FUNCTION__);

	descr->buf = (void*)virt_to_phys(buffer->buffer);
	
	return recvl;
}

static unsigned int
handle_all_descr_data(struct uart_cris_port *up)
{
	struct dma_descr_data *descr;
	unsigned int recvl;
	unsigned int ret = 0;
	reg_scope_instances regi_dmain = up->regi_dmain;

	while (1)
	{
		descr = &up->rec_descr[up->cur_rec_descr];

		if (descr == phys_to_virt(REG_RD(dma, regi_dmain, rw_data)))
			break;

		if (++up->cur_rec_descr == SERIAL_RECV_DESCRIPTORS)
			up->cur_rec_descr = 0;
	
		/* find out how many bytes were read */
		recvl = descr->after - descr->buf;

		/* update stats */
		up->port.icount.rx += recvl;

		ret += handle_descr_data(up, descr, recvl);
	}

	return ret;
}

static void 
receive_chars_dma(struct uart_cris_port *up)
{
	reg_ser_r_stat_din rstat;
	reg_dma_rw_ack_intr ack_intr = {0};
	
	/* Acknowledge both dma_descr and dma_eop irq */
	ack_intr.data = 1;
	ack_intr.in_eop = 1;
	REG_WR(dma, up->regi_dmain, rw_ack_intr, ack_intr);
	
	handle_all_descr_data(up);

	/* Read the status register to detect errors */
	rstat = REG_RD(ser, up->regi_ser, r_stat_din);

	if (rstat.framing_err | rstat.par_err | rstat.orun) {
		/* If we got an error, we must reset it by reading the
		 * rs_stat_din register and put the data in buffer manually.
		 */
		reg_ser_rs_stat_din stat_din;
                stat_din = REG_RD(ser, up->regi_ser, rs_stat_din);

		if (stat_din.par_err)
			add_char_and_flag(up, stat_din.data, TTY_PARITY);
		else if (stat_din.orun)
			add_char_and_flag(up, stat_din.data, TTY_OVERRUN);
		else if (stat_din.framing_err)
			add_char_and_flag(up, stat_din.data, TTY_FRAME);
	}


	/* Restart the receiving DMA */
	DMA_CONTINUE_DATA(up->regi_dmain);
}

void receive_chars_no_dma(struct uart_cris_port *up)
{
	reg_ser_rs_stat_din stat_din;
	reg_ser_r_stat_din rstat;
	struct tty_struct *tty;
	struct uart_icount *icount;
	int max_count = 16;
	reg_ser_rw_ack_intr ack_intr = { 0 };
	
	rstat = REG_RD(ser, up->regi_ser, r_stat_din);
	up->last_rx_active_usec = GET_JIFFIES_USEC();
	up->last_rx_active = jiffies;
	icount = &up->port.icount;
	tty = up->port.info->tty;

	do {
		stat_din = REG_RD(ser, up->regi_ser, rs_stat_din);
	
		ack_intr.dav = 1;
		REG_WR(ser, up->regi_ser, rw_ack_intr, ack_intr);

		*tty->flip.char_buf_ptr = stat_din.data;
		icount->rx++;

		if (stat_din.framing_err | 
                    stat_din.par_err | 
                    stat_din.orun) {
			if (stat_din.data == 0x00 && 
                            stat_din.framing_err) {
				/* Most likely a break. */
				*tty->flip.flag_buf_ptr = TTY_BREAK;
				icount->brk++;
			} else if (stat_din.par_err) {
				*tty->flip.flag_buf_ptr = TTY_PARITY;
				icount->parity++;
			} else if (stat_din.orun) {
				*tty->flip.flag_buf_ptr = TTY_OVERRUN;
				icount->overrun++;
			} else if (stat_din.framing_err) {
				*tty->flip.flag_buf_ptr = TTY_FRAME;
				icount->frame++;			
			}
		} else {
			*tty->flip.flag_buf_ptr = 0;
		}
		tty->flip.flag_buf_ptr++;
		tty->flip.char_buf_ptr++;
		tty->flip.count++;
		rstat = REG_RD(ser, up->regi_ser, r_stat_din);
	} while (rstat.dav && (max_count-- > 0));
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
} /* receive_chars_no_dma */

/* dma output channel interrupt handler
   this interrupt is called from DMA2(ser2), DMA4(ser3), DMA6(ser0) or
   DMA8(ser1) when they have finished a descriptor with the intr flag set.
*/

static irqreturn_t 
dma_tr_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct uart_cris_port *up = (struct uart_cris_port *)dev_id;
	reg_dma_r_masked_intr masked_intr;
	reg_scope_instances regi_dmaout;
	int handled = 0;

	spin_lock(&up->port.lock);
	regi_dmaout = up->regi_dmaout;
	if (!regi_dmaout) {
		spin_unlock(&up->port.lock);
		return IRQ_NONE;
	} 
	/* check for dma_descr (don't need to check for dma_eop in
	   output dma for serial */
	masked_intr = REG_RD(dma, regi_dmaout, r_masked_intr);
		
	if (masked_intr.data) {
		/* we can send a new dma bunch. make it so. */

		/* Read jiffies_usec first, 
		 * we want this time to be as late as possible
		 */
		up->last_tx_active_usec = GET_JIFFIES_USEC();
		up->last_tx_active = jiffies;
		up->transmitter_running = 0;
		transmit_chars_dma(up);
		handled = 1;
	}
	check_modem_status(up);
	spin_unlock(&up->port.lock);
	return IRQ_RETVAL(handled);
}

/* dma input channel interrupt handler */

static irqreturn_t
dma_rec_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct uart_cris_port *up = (struct uart_cris_port *)dev_id;
	reg_dma_r_masked_intr masked_intr;
	reg_scope_instances regi_dmain;
	int handled = 0;

	spin_lock(&up->port.lock);
	regi_dmain = up->regi_dmain;
	if (!regi_dmain) {
		spin_unlock(&up->port.lock);
		return IRQ_NONE;
	}

	/* check for both dma_eop and dma_descr for the input dma channel */
	masked_intr = REG_RD(dma, regi_dmain, r_masked_intr);
	if (masked_intr.data || masked_intr.in_eop) {
		/* we have received something */
		receive_chars_dma(up);
		handled = 1;
	}
	check_modem_status(up);
	spin_unlock(&up->port.lock);
	return IRQ_RETVAL(handled);
}

/* "Normal" serial port interrupt handler - both rx and tx */
static irqreturn_t
ser_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_cris_port *up = (struct uart_cris_port *)dev_id;
	reg_scope_instances regi_ser;
	int handled = 0;

	spin_lock(&up->port.lock);
	if (up->regi_dmain && up->regi_dmaout) {
		spin_unlock(&up->port.lock);
		return IRQ_NONE;
	}
		
	regi_ser = up->regi_ser;
		
	if (regi_ser) {
		reg_ser_r_masked_intr masked_intr;
		masked_intr = REG_RD(ser, regi_ser, r_masked_intr);
		/* Check what interrupts are active before taking 
		 * actions. If DMA is used the interrupt shouldn't 
		 * be enabled.
		 */
		if (masked_intr.dav) { 
			receive_chars_no_dma(up);
			handled = 1;
		}
		check_modem_status(up);
		
		if (masked_intr.tr_rdy) { 
			transmit_chars_no_dma(up);
			handled = 1;
		}
	}
	spin_unlock(&up->port.lock);
	return IRQ_RETVAL(handled);
} /* ser_interrupt */

static int start_recv_dma(struct uart_cris_port *up)
{
	struct dma_descr_data *descr = up->rec_descr;
	struct etrax_recv_buffer *buffer;
	int i;

	/* Set up the receiving descriptors */
	for (i = 0; i < SERIAL_RECV_DESCRIPTORS; i++) {
		if (!(buffer = alloc_recv_buffer(SERIAL_DESCR_BUF_SIZE)))
			panic("%s: Failed to allocate memory for receive buffer!\n", __FUNCTION__ );
		descr[i].next = (void*)virt_to_phys(&descr[i+1]);
		descr[i].buf = (void*)virt_to_phys(buffer->buffer);
		descr[i].after = descr[i].buf + SERIAL_DESCR_BUF_SIZE;
		descr[i].eol = 0;
		descr[i].out_eop = 0;
		descr[i].intr = 1;
		descr[i].wait = 0;
		descr[i].in_eop = 0;
		descr[i].md = 0;

	}

	/* Link the last descriptor to the first */
	descr[i-1].next = (void*)virt_to_phys(&descr[0]);

	/* Start with the first descriptor in the list */
	up->cur_rec_descr = 0;
	up->rec_context_descr.next = 0;
	up->rec_context_descr.saved_data = (dma_descr_data *)virt_to_phys(&descr[up->cur_rec_descr]);
	up->rec_context_descr.saved_data_buf = descr[up->cur_rec_descr].buf;
        
	/* Start the DMA */
	DMA_START_CONTEXT(up->regi_dmain, virt_to_phys(&up->rec_context_descr));

	/* Input DMA should be running now */
	return 1;
}


static void start_receive(struct uart_cris_port *up)
{
	reg_scope_instances regi_dmain = up->regi_dmain;
	if (regi_dmain) {
		start_recv_dma(up);
	}
}


static void start_transmitter(struct uart_cris_port *up)
{
	int i;
	reg_scope_instances regi_dmaout = up->regi_dmaout;
	if (regi_dmaout) {
		for (i = 0; i < SERIAL_TX_DESCRIPTORS; i++)
		{
			memset(&up->tr_descr[i], 0, sizeof(up->tr_descr[i]));
			up->tr_descr[i].eol = 1;
			up->tr_descr[i].intr = 1;			
			up->tr_descr[i].next = (dma_descr_data*)virt_to_phys(&up->tr_descr[i+1]);
		}
		up->tr_descr[i-1].next = (dma_descr_data*)virt_to_phys(&up->tr_descr[0]);
		up->first_tx_descr = up->last_tx_descr = &up->tr_descr[0];
		up->tx_started = 0;
	}
}

static int serial_cris_startup(struct uart_port *port)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	unsigned long flags;
	reg_intr_vect_rw_mask intr_mask;	
	reg_ser_rw_intr_mask ser_intr_mask = {0};
	reg_dma_rw_intr_mask dmain_intr_mask = {0};
	reg_dma_rw_intr_mask dmaout_intr_mask = {0};
	reg_dma_rw_cfg cfg = {.en = 1};
	reg_scope_instances regi_dma;

	if (!up->regi_ser)
		return 0;

	spin_lock_irqsave(&up->port.lock, flags);

	intr_mask = REG_RD(intr_vect, regi_irq, rw_mask);

	dmain_intr_mask.data = dmain_intr_mask.in_eop = regk_dma_yes;
	dmaout_intr_mask.data = regk_dma_yes;
	if (!up->regi_dmain)
		ser_intr_mask.dav = regk_ser_yes;

	if (port->line == 0) {
		if (request_irq(SER0_INTR_VECT, ser_interrupt, SA_SHIRQ | SA_INTERRUPT, "ser0", &serial_cris_ports[0]))
			panic("irq ser0");
		/* enable the ser0 irq in global config */
		intr_mask.ser0 = 1;
		/* ser0 can use dma6 for tx and dma7 for rx */
#ifdef CONFIG_ETRAX_SERIAL_PORT0_DMA6_OUT
		if (request_irq(DMA6_INTR_VECT, dma_tr_interrupt, SA_INTERRUPT, "serial 0 dma tr", &serial_cris_ports[0]))
			panic("irq ser0txdma");
		crisv32_request_dma(6, "ser0", DMA_PANIC_ON_ERROR, 0, dma_ser0);
		/* enable the dma6 irq in global config */
		intr_mask.dma6 = 1;
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT0_DMA7_IN
		if (request_irq(DMA7_INTR_VECT, dma_rec_interrupt, SA_INTERRUPT, "serial 0 dma rec", &serial_cris_ports[0]))
			panic("irq ser0rxdma");
		crisv32_request_dma(7, "ser0", DMA_PANIC_ON_ERROR, 0, dma_ser0);
		/* enable the dma7 irq in global config */
		intr_mask.dma7 = 1;
#endif
	} else if (port->line == 1) {
		crisv32_pinmux_alloc_fixed(pinmux_ser1);
		if (request_irq(SER1_INTR_VECT, ser_interrupt, SA_SHIRQ | SA_INTERRUPT, "ser1", &serial_cris_ports[1]))
			panic("irq ser1");
		/* enable the ser1 irq in global config */
		intr_mask.ser1 = 1;

		/* ser1 can use dma4 for tx and dma5 for rx */
#ifdef CONFIG_ETRAX_SERIAL_PORT1_DMA4_OUT
		if (request_irq(DMA4_INTR_VECT, dma_tr_interrupt, SA_INTERRUPT, "serial 1 dma tr", &serial_cris_ports[1]))
			panic("irq ser1txdma");
		crisv32_request_dma(4, "ser1", DMA_PANIC_ON_ERROR, 0, dma_ser1);
		/* enable the dma4 irq in global config */
		intr_mask.dma4 = 1;
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT1_DMA5_IN
		if (request_irq(DMA5_INTR_VECT, dma_rec_interrupt, SA_INTERRUPT, "serial 1 dma rec", &serial_cris_ports[1]))
			panic("irq ser1rxdma");
		crisv32_request_dma(5, "ser1", DMA_PANIC_ON_ERROR, 0, dma_ser1);
		/* enable the dma5 irq in global config */
		intr_mask.dma5 = 1;
#endif
	} else if (port->line == 2) {
		crisv32_pinmux_alloc_fixed(pinmux_ser2);
		if (request_irq(SER2_INTR_VECT, ser_interrupt, SA_SHIRQ | SA_INTERRUPT, "ser2", &serial_cris_ports[2]))
			panic("irq ser2");
		/* enable the ser2 irq in global config */
		intr_mask.ser2 = 1;

		/* ser2 can use dma2 for tx and dma3 for rx */
#ifdef CONFIG_ETRAX_SERIAL_PORT2_DMA2_OUT
		if (request_irq(DMA2_INTR_VECT, dma_tr_interrupt, SA_INTERRUPT, "serial 2 dma tr", &serial_cris_ports[2]))
			panic("irq ser2txdma");
		crisv32_request_dma(2, "ser2", DMA_PANIC_ON_ERROR, 0, dma_ser2);
		/* enable the dma2 irq in global config */
		intr_mask.dma2 = 1;
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT2_DMA3_IN
		if (request_irq(DMA3_INTR_VECT, dma_rec_interrupt, SA_INTERRUPT, "serial 2 dma rec", &serial_cris_ports[2]))
			panic("irq ser2rxdma");
			crisv32_request_dma(3, "ser2", DMA_PANIC_ON_ERROR, 0, dma_ser2);
		/* enable the dma3 irq in global config */
		intr_mask.dma3 = 1;
#endif
	} else if (port->line == 3) {
		crisv32_pinmux_alloc_fixed(pinmux_ser3);
		if (request_irq(SER3_INTR_VECT, ser_interrupt, SA_SHIRQ | SA_INTERRUPT, "ser3", &serial_cris_ports[3]))
			panic("irq ser3" );
		/* enable the ser3 irq in global config */
		intr_mask.ser3 = 1;

		/* ser3 can use dma8 for tx and dma9 for rx */
#ifdef CONFIG_ETRAX_SERIAL_PORT3_DMA8_OUT
		if (request_irq(DMA8_INTR_VECT, dma_tr_interrupt, SA_INTERRUPT, "serial 3 dma tr", &serial_cris_ports[3]))
			panic("irq ser3txdma");
		crisv32_request_dma(8, "ser3", DMA_PANIC_ON_ERROR, 0, dma_ser3);
		/* enable the dma2 irq in global config */
		intr_mask.dma8 = 1;
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT3_DMA9_IN
		if (request_irq(DMA9_INTR_VECT, dma_rec_interrupt, SA_INTERRUPT, "serial 3 dma rec", &serial_cris_ports[3]))
			panic("irq ser3rxdma");
		crisv32_request_dma(9, "ser3", DMA_PANIC_ON_ERROR, 0, dma_ser3);
		/* enable the dma3 irq in global config */
		intr_mask.dma9 = 1;
#endif
	}

	/*
	 * Reset the DMA channels and make sure their interrupts are cleared
	 */

	regi_dma = up->regi_dmain;
	if (regi_dma) {
		reg_dma_rw_ack_intr ack_intr = { 0 };
		DMA_RESET(regi_dma);
		/* Wait until reset cycle is complete */
		DMA_WAIT_UNTIL_RESET(regi_dma);
		REG_WR(dma, regi_dma, rw_cfg, cfg);
		/* Make sure the irqs are cleared */
		ack_intr.group = 1;
		ack_intr.ctxt = 1;
		ack_intr.data = 1;
		ack_intr.in_eop = 1;
		ack_intr.stream_cmd = 1;
		REG_WR(dma, regi_dma, rw_ack_intr, ack_intr);
	}
	regi_dma = up->regi_dmaout;
	if (regi_dma) {
		reg_dma_rw_ack_intr ack_intr = { 0 };		
		DMA_RESET(regi_dma);
		/* Wait until reset cycle is complete */
		DMA_WAIT_UNTIL_RESET(regi_dma);
		REG_WR(dma, regi_dma, rw_cfg, cfg);
		/* Make sure the irqs are cleared */
		ack_intr.group = 1;
		ack_intr.ctxt = 1;
		ack_intr.data = 1;
		ack_intr.in_eop = 1;
		ack_intr.stream_cmd = 1;
		REG_WR(dma, regi_dma, rw_ack_intr, ack_intr);
	}

	REG_WR(intr_vect, regi_irq, rw_mask, intr_mask);
	REG_WR(ser, up->regi_ser, rw_intr_mask, ser_intr_mask);
	if (up->regi_dmain)
		REG_WR(dma, up->regi_dmain, rw_intr_mask, dmain_intr_mask);
	if (up->regi_dmaout)
		REG_WR(dma, up->regi_dmaout, rw_intr_mask, dmaout_intr_mask);

	start_receive(up);
	start_transmitter(up);

	serial_cris_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	return 0;
}

static void serial_cris_shutdown(struct uart_port *port)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	unsigned long flags;
	reg_intr_vect_rw_mask intr_mask;
	
	spin_lock_irqsave(&up->port.lock, flags);

	intr_mask = REG_RD(intr_vect, regi_irq, rw_mask);
	
	if (port->line == 0) {
		intr_mask.ser0 = 0;
		free_irq(SER0_INTR_VECT, &serial_cris_ports[0]);
#ifdef CONFIG_ETRAX_SERIAL_PORT0_DMA6_OUT
		intr_mask.dma6 = 0;
		crisv32_free_dma(6);
		free_irq(DMA6_INTR_VECT, &serial_cris_ports[0]);
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT0_DMA7_IN
		intr_mask.dma7 = 0;
		crisv32_free_dma(7);
		free_irq(DMA7_INTR_VECT, &serial_cris_ports[0]);
#endif
	} else if (port->line == 1) {
		crisv32_pinmux_dealloc_fixed(pinmux_ser1);
		intr_mask.ser1 = 0;
		free_irq(SER1_INTR_VECT, &serial_cris_ports[1]);
#ifdef CONFIG_ETRAX_SERIAL_PORT1_DMA4_OUT
		intr_mask.dma4 = 0;
		crisv32_free_dma(4);
		free_irq(DMA4_INTR_VECT, &serial_cris_ports[1]);
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT1_DMA5_IN
		intr_mask.dma5 = 0;
		crisv32_free_dma(5);
		free_irq(DMA5_INTR_VECT, &serial_cris_ports[1]);
#endif
	} else if (port->line == 2) {
		crisv32_pinmux_dealloc_fixed(pinmux_ser2);
		intr_mask.ser2 = 0;
		free_irq(SER2_INTR_VECT, &serial_cris_ports[2]);
#ifdef CONFIG_ETRAX_SERIAL_PORT2_DMA2_OUT
		intr_mask.dma2 = 0;
		crisv32_free_dma(2);
		free_irq(DMA2_INTR_VECT, &serial_cris_ports[2]);
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT2_DMA3_IN
		intr_mask.dma3 = 0;
		crisv32_free_dma(3);
		free_irq(DMA3_INTR_VECT, &serial_cris_ports[2]);
#endif
	} else if (port->line == 3) {
		crisv32_pinmux_dealloc_fixed(pinmux_ser3);
		intr_mask.ser3 = 0;
		free_irq(SER3_INTR_VECT, &serial_cris_ports[3]);
#ifdef CONFIG_ETRAX_SERIAL_PORT3_DMA8_OUT
		intr_mask.dma8 = 0;
		crisv32_free_dma(8);
		free_irq(DMA8_INTR_VECT, &serial_cris_ports[3]);
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT3_DMA9_IN
		intr_mask.dma9 = 0;
		crisv32_free_dma(9);
		free_irq(DMA9_INTR_VECT, &serial_cris_ports[3]);
#endif
	}

	REG_WR(intr_vect, regi_irq, rw_mask, intr_mask);

	serial_cris_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

#if defined(CONFIG_ETRAX_RS485)
	if (up->rs485.enabled) {
		up->rs485.enabled = 0;
	}
#endif
}

static void
serial_cris_set_termios(struct uart_port *port, struct termios *termios,
		        struct termios *old)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	unsigned long flags;
	reg_ser_rw_xoff xoff;
	reg_ser_rw_xoff_clr xoff_clr = {0};
	reg_ser_rw_tr_ctrl tx_ctrl = {0};
	reg_ser_rw_tr_dma_en tx_dma_en = {0};
	reg_ser_rw_rec_ctrl rx_ctrl = {0};
	reg_ser_rw_tr_baud_div tx_baud_div = {0};
	reg_ser_rw_rec_baud_div rx_baud_div = {0};
        
	if (!up->regi_ser)
		return;

	if (old && 
            termios->c_cflag == old->c_cflag &&
	    termios->c_iflag == old->c_iflag)
		return;

	/* start with default settings and then fill in changes */

	/* tx: 8 bit, no/even parity, 1 stop bit, no cts */
	tx_ctrl.base_freq = regk_ser_f29_493;
	tx_ctrl.en = 0;
	if (up->regi_dmaout) 
		tx_dma_en.en = 1;
	tx_ctrl.stop = 0;
	tx_ctrl.auto_rts = 0;
	tx_ctrl.txd = 1;
	tx_ctrl.auto_cts = 0;
	/* rx: 8 bit, no/even parity */
	if (up->regi_dmain) {
		rx_ctrl.dma_mode = 1;
		rx_ctrl.auto_eop = 1;
	}
	rx_ctrl.dma_err = regk_ser_stop;
	rx_ctrl.sampling = regk_ser_majority;
	rx_ctrl.timeout = 1;
	rx_ctrl.rts_n = regk_ser_inactive;	

	/* Common for tx and rx: 8N1 */
	tx_ctrl.data_bits = regk_ser_bits8;
	rx_ctrl.data_bits = regk_ser_bits8;
	tx_ctrl.par = regk_ser_even;
	rx_ctrl.par = regk_ser_even;
	tx_ctrl.par_en = regk_ser_no;
	rx_ctrl.par_en = regk_ser_no;

	tx_ctrl.stop_bits = regk_ser_bits1;


	/* change baud-rate and write it to the hardware */
	
	/* baud_clock = base_freq / (divisor*8)
	 * divisor = base_freq / (baud_clock * 8)
	 * base_freq is either:
	 * off, ext, 29.493MHz, 32.000 MHz, 32.768 MHz or 100 MHz
	 * 20.493MHz is used for standard baudrates
	 */

	tx_baud_div.div = 29493000 / port->uartclk;
	/* RX uses same as TX */
	rx_baud_div.div = tx_baud_div.div;
	rx_ctrl.base_freq = tx_ctrl.base_freq;

	if ((termios->c_cflag & CSIZE) == CS7) {
		/* set 7 bit mode */
		tx_ctrl.data_bits = regk_ser_bits7;
		rx_ctrl.data_bits = regk_ser_bits7;
	}

	if (termios->c_cflag & CSTOPB) {
		/* set 2 stop bit mode */
		tx_ctrl.stop_bits = regk_ser_bits2;
	}	  
	
	if (termios->c_cflag & PARENB) {
		/* enable parity */
		tx_ctrl.par_en = regk_ser_yes;
		rx_ctrl.par_en = regk_ser_yes;
	}

	if (termios->c_cflag & CMSPAR) {
		if (termios->c_cflag & PARODD) {
			/* set mark parity if PARODD and CMSPAR */
			tx_ctrl.par = regk_ser_mark;
			rx_ctrl.par = regk_ser_mark;
		} else {
			tx_ctrl.par = regk_ser_space;
			rx_ctrl.par = regk_ser_space;
		}
	} else {
		if (termios->c_cflag & PARODD) {
			/* set odd parity */
		       tx_ctrl.par = regk_ser_odd;
		       rx_ctrl.par = regk_ser_odd;
		}
	}

	if (termios->c_cflag & CRTSCTS) {
		/* enable automatic CTS handling */
		tx_ctrl.auto_cts = regk_ser_yes;
	}
	
	/* make sure the tx and rx are enabled */
	tx_ctrl.en = regk_ser_yes;
	rx_ctrl.en = regk_ser_yes;	

	/* actually write the control regs (if modified) to the hardware */
        
	spin_lock_irqsave(&up->port.lock, flags);
	uart_update_timeout(port, termios->c_cflag, port->uartclk/8);
	MODIFY_REG(up->regi_ser, rw_rec_baud_div, rx_baud_div);
	MODIFY_REG(up->regi_ser, rw_rec_ctrl, rx_ctrl);

	MODIFY_REG(up->regi_ser, rw_tr_baud_div, tx_baud_div);
	MODIFY_REG(up->regi_ser, rw_tr_ctrl, tx_ctrl);
	MODIFY_REG(up->regi_ser, rw_tr_dma_en, tx_dma_en);

	xoff = REG_RD(ser, up->regi_ser, rw_xoff);

	xoff_clr.clr = regk_ser_yes;
	if (up->port.info && (up->port.info->tty->termios->c_iflag & IXON)) {
		xoff.chr = STOP_CHAR(up->port.info->tty);
		xoff.automatic = regk_ser_yes;
	}
	else
		xoff.automatic = regk_ser_no;

	MODIFY_REG(up->regi_ser, rw_xoff_clr, xoff_clr);
	MODIFY_REG(up->regi_ser, rw_xoff, xoff);

	serial_cris_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static const char *
serial_cris_type(struct uart_port *port)
{
	return "CRISv32";
}

static void serial_cris_release_port(struct uart_port *port)
{
}

static int serial_cris_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_cris_config_port(struct uart_port *port, int flags)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	up->port.type = PORT_CRIS;
}

#if defined(CONFIG_ETRAX_RS485)
/* Enable RS-485 mode on selected port. This is UGLY. */
static int
cris_enable_rs485(struct uart_cris_port* up, struct rs485_control *r)
{
	reg_scope_instances regi_ser = up->regi_ser;

	up->rs485.rts_on_send = 0x01 & r->rts_on_send;
	up->rs485.rts_after_sent = 0x01 & r->rts_after_sent;
	up->rs485.delay_rts_before_send = r->delay_rts_before_send;
	up->rs485.enabled = r->enabled;
	if (regi_ser) {
		unsigned long flags;
		reg_ser_rw_tr_ctrl tr_ctrl;
                reg_ser_rw_rec_ctrl rec_ctrl;

		local_irq_save(flags);
		tr_ctrl = REG_RD(ser, regi_ser, rw_tr_ctrl);
                rec_ctrl = REG_RD(ser, regi_ser, rw_rec_ctrl);
		if (r->enabled) {
			tr_ctrl.auto_rts = regk_ser_yes;
			rec_ctrl.rts_n = r->rts_after_sent ?
				regk_ser_active : regk_ser_inactive;
			
		} else {
			tr_ctrl.auto_rts = regk_ser_no;
			rec_ctrl.rts_n = r->rts_after_sent ?
				regk_ser_active : regk_ser_inactive;
		}

		REG_WR(ser, regi_ser, rw_tr_ctrl, tr_ctrl);
                REG_WR(ser, regi_ser, rw_rec_ctrl, rec_ctrl);
#ifdef CONFIG_ETRAX_RS485_DISABLE_RECEIVER
		{
			reg_ser_rw_rec_ctrl rec_ctrl;
			rec_ctrl = REG_RD(ser, regi_ser, rw_rec_ctrl);
			rec_ctrl.half_duplex = r->enabled;
			REG_WR(ser, regi_ser, rw_rec_ctrl, rec_ctrl);
		}
#endif /* CONFIG_ETRAX_RS485_DISABLE_RECEIVER */
		local_irq_restore(flags);
	}
	return 0;
}

static int
cris_write_rs485(struct uart_cris_port* up, const unsigned char *buf, int count)
{
	int old_enabled = up->rs485.enabled;
	reg_scope_instances regi_ser = up->regi_ser;

	/* rs485 is always implicitly enabled if we're using the ioctl() 
	 * but it doesn't have to be set in the rs485_control
	 * (to be backward compatible with old apps)
	 * So we store, set and restore it.
	 */
	up->rs485.enabled = 1;
	/* rs_write now deals with RS485 if enabled */
	if (regi_ser) {
		unsigned long flags;
		reg_ser_rw_tr_ctrl tr_ctrl;
                reg_ser_rw_rec_ctrl rec_ctrl;

		local_irq_save(flags);
		tr_ctrl = REG_RD(ser, regi_ser, rw_tr_ctrl);
                rec_ctrl = REG_RD(ser, regi_ser, rw_rec_ctrl);
		tr_ctrl.auto_rts = regk_ser_yes;
		rec_ctrl.rts_n = up->rs485.rts_after_sent ?
			regk_ser_active : regk_ser_inactive;

#ifdef CONFIG_ETRAX_RS485_DISABLE_RECEIVER
		rec_ctrl.half_duplex = regk_ser_yes;
#endif /* CONFIG_ETRAX_RS485_DISABLE_RECEIVER */
		REG_WR(ser, regi_ser, rw_tr_ctrl, tr_ctrl);
                REG_WR(ser, regi_ser, rw_rec_ctrl, rec_ctrl);
		local_irq_restore(flags);
	}	
	count = serial_cris_driver.tty_driver->write(up->port.info->tty, buf, count);

	up->rs485.enabled = old_enabled;
	if (regi_ser && !old_enabled) {
		unsigned long flags;
		reg_ser_rw_tr_ctrl tr_ctrl;
		local_irq_save(flags);
		tr_ctrl = REG_RD(ser, regi_ser, rw_tr_ctrl);
		tr_ctrl.auto_rts = regk_ser_no;
		REG_WR(ser, regi_ser, rw_tr_ctrl, tr_ctrl);
#ifdef CONFIG_ETRAX_RS485_DISABLE_RECEIVER
		{
			reg_ser_rw_rec_ctrl rec_ctrl;
			rec_ctrl = REG_RD(ser, regi_ser, rw_rec_ctrl);
			rec_ctrl.half_duplex = regk_ser_no;
			REG_WR(ser, regi_ser, rw_rec_ctrl, rec_ctrl);
		}
#endif /* CONFIG_ETRAX_RS485_DISABLE_RECEIVER */
		local_irq_restore(flags);
	}
	return count;
}

#endif /* CONFIG_ETRAX_RS485 */

static int serial_cris_ioctl(struct uart_port *port, unsigned int cmd, 
                             unsigned long arg)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;

	switch (cmd) {
#if defined(CONFIG_ETRAX_RS485)
	       case TIOCSERSETRS485:
		{
			struct rs485_control rs485ctrl;
			if (copy_from_user(&rs485ctrl, (struct rs485_control*)arg, sizeof(rs485ctrl)))
				return -EFAULT;

			return cris_enable_rs485(up, &rs485ctrl);
		}

		case TIOCSERWRRS485:
		{
			struct rs485_write rs485wr;
			if (copy_from_user(&rs485wr, (struct rs485_write*)arg, sizeof(rs485wr)))
				return -EFAULT;

			return cris_write_rs485(up, rs485wr.outc, rs485wr.outc_size);
		}
#endif
		default:
			return -ENOIOCTLCMD;
	}

	return 0;
}

static struct uart_ops serial_cris_pops = {
	.tx_empty	= serial_cris_tx_empty,
	.set_mctrl	= serial_cris_set_mctrl,
	.get_mctrl	= serial_cris_get_mctrl,
	.stop_tx	= serial_cris_stop_tx,
	.start_tx	= serial_cris_start_tx,
	.stop_rx	= serial_cris_stop_rx,
	.enable_ms	= serial_cris_enable_ms,
	.break_ctl	= serial_cris_break_ctl,
	.startup	= serial_cris_startup,
	.shutdown	= serial_cris_shutdown,
	.set_termios	= serial_cris_set_termios,
	.type		= serial_cris_type,
	.release_port	= serial_cris_release_port,
	.request_port	= serial_cris_request_port,
	.config_port	= serial_cris_config_port,
	.ioctl		= serial_cris_ioctl,
};

static void cris_serial_port_init(struct uart_port *port, int line)
{
	struct uart_cris_port *up = (struct uart_cris_port *)port;
	static int first = 1;
	
	if (up->initialized)
		return;
	up->initialized = 1;
	port->line = line;
	spin_lock_init(port->lock);
	port->ops = &serial_cris_pops;
	port->irq = up->irq;
	port->iobase = up->regi_ser ? up->regi_ser : 1;
	port->uartclk = 115200 * 8;
	port->fifosize = 64;
	port->flags = UPF_BOOT_AUTOCONF;
#if defined(CONFIG_ETRAX_RS485)
	/* Set sane defaults */
	up->rs485.rts_on_send = 0;
	up->rs485.rts_after_sent = 1;
	up->rs485.delay_rts_before_send = 0;
	up->rs485.enabled = 0;
#endif

	if (first) {
		first = 0;
#ifdef CONFIG_ETRAX_SERIAL_PORT0
		SETUP_PINS(0);
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT1
		SETUP_PINS(1);
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT2
		SETUP_PINS(2);
#endif
#ifdef CONFIG_ETRAX_SERIAL_PORT3
		SETUP_PINS(3);
#endif
	}
}

static int __init serial_cris_init(void)
{
	int ret, i;

	printk(KERN_INFO "Serial: CRISv32 driver $Revision: 1.55 $ ");

	ret = uart_register_driver(&serial_cris_driver);
	if (ret)
		goto out;

	for (i = 0; i < UART_NR; i++) {
		if (serial_cris_ports[i].used) {
			struct uart_port *port = &serial_cris_ports[i].port;
			cris_console.index = i;
			cris_serial_port_init(port, i);
			uart_add_one_port(&serial_cris_driver, port);
		}
	}

out:
	return ret;
}

static void __exit serial_cris_exit(void)
{
	int i;
	for (i = 0; i < UART_NR; i++)
	    uart_remove_one_port(&serial_cris_driver, &serial_cris_ports[i].port);
	uart_unregister_driver(&serial_cris_driver);
}

module_init(serial_cris_init);
module_exit(serial_cris_exit);

