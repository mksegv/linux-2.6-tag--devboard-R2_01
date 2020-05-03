/*
 *  linux/arch/arm/mach-argus/argus_serial.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/config.h>
#include <linux/version.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/arch/hardware.h>
#include <asm/arch/argus.h>

#include <asm/mach-types.h>

/* non-arch dependant serial structures are in linux/serial.h */
#include <linux/serial.h>

#include "argus_uart.h"

#ifndef CONFIG_ARGUS_DEBUG_PORT_BAUD
#define CONFIG_ARGUS_DEBUG_PORT_BAUD	115200
#endif

// If this is defined, the input pins (RX, CTS) are configured
// as input pins if the serial port is enabled. If it is undefined,
// it is up to the kernel config to set the right mode for the pins
// but this can be necessary if for example there are no pull-ups on
// the inputs.. 

#ifdef CONFIG_ARGUS_SERIAL_FORCE_RX_ENABLE
#define FORCE_ENABLE_RX_PINS
#endif

// What device majors to use
// 
// If this is the only serial port in the system, it makes sense
// to let it use the normal tty majors - but if for example this is
// only used for console I/O and we have "real" serial ports externally
// like a GPRS modem or something, we need to have another major because
// the standard kernel driver serial.c will want to hook TTY_MAJOR.
//

#ifdef CONFIG_ARGUS_SERIAL_ALTMAJOR
#define ARGUS_PORT_MAJOR 160
#define ARGUS_PORTAUX_MAJOR 161
#else
#define ARGUS_PORT_MAJOR TTY_MAJOR
#define ARGUS_PORTAUX_MAJOR TTYAUX_MAJOR
#endif

// Not static, because argus_uart uses this to tell the kernel about
// this driver as a debugport driver in late booting (just before init)
struct tty_driver *serial_driver;

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

#define BASE_BAUD (115200)
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST)

#define NR_PORTS		2

//#define SERIAL_DEBUG_OPEN  1

#define UART_OK			0
#define UART_TIMEOUT		-1
#define UART_INVALID_UART	-2
#define UART_OVERFLOW		-3

#define DISABLE_RX_PARAM		0x0001
#define DISABLE_TX_PARAM		0x0002
#define ENABLE_RX_PARAM			0x0004
#define ENABLE_TX_PARAM			0x0008
#define FORCE_LOW_TX_PARAM		0x0010
#define FORCE_HIGH_TX_PARAM		0x0020
#define FORCE_LOW_RTS_PARAM		0x0040
#define FORCE_HIGH_RTS_PARAM		0x0080
#define FORCE_LOW_RX_PARAM		0x0100
#define FORCE_HIGH_RX_PARAM		0x0200
#define FORCE_LOW_CTS_PARAM		0x0400
#define FORCE_HIGH_CTS_PARAM		0x0800
#define POLARITY_IR_PARAM		0x1000
#define POLARITY_RS232_PARAM		0x2000
#define ENABLE_FLOW_CONTROL		0x4000
#define DISABLE_FLOW_CONTROL		0x8000

static struct argus_serial rs_table[NR_PORTS] = {
	{ CONFIG_ARGUS_DEBUG_PORT_BAUD,
	  &SOC_UART[0],
	  0,
	  0,
	  1   // enabled
	},
	{ CONFIG_ARGUS_DEBUG_PORT_BAUD,
	  &SOC_UART[1],
	  0,
	  0,
	  1  // enabled
	}
};

static struct termios *serial_termios[NR_PORTS];
static struct termios *serial_termios_locked[NR_PORTS];

#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

// Some forwarddeclarations

static void rs_wait_until_sent(struct tty_struct *tty, int timeout);
static void rs_start(struct tty_struct *tty);

static void
argus_enable_tx_irq(struct argus_serial *info)
{
	// This uses the TXHEMPTY interrupt, that is, we get the
	// interrupt when the FIFO is half empty (< 4 bytes).
	// Might also use TXEMPTY if we want..
	info->uart->IRQEN = (info->IRQEN_shadow |= SET_UART_TXHEMPTY(1));
}

static void
argus_disable_tx_irq(struct argus_serial *info)
{
	// This uses the TXHEMPTY interrupt, that is, we get the
	// interrupt when the FIFO is half empty (< 4 bytes).
	// Might also use TXEMPTY if we want..
	info->uart->IRQEN = (info->IRQEN_shadow &= ~SET_UART_TXHEMPTY(1));
}

// TODO: see the uart_interrupt routine about RXNEMPTY.

static void
argus_enable_rx_irq(struct argus_serial *info)
{
	info->uart->IRQEN = (info->IRQEN_shadow |= SET_UART_RXNEMPTY(1));
}

static void
argus_disable_rx_irq(struct argus_serial *info)
{
	info->uart->IRQEN = (info->IRQEN_shadow &= ~SET_UART_RXNEMPTY(1));
}

static void
argus_enable_tx(struct argus_serial *info)
{
	info->uart->ENABLE |= SET_UART_TXEN(1);
}

static void
argus_disable_tx(struct argus_serial *info)
{
	info->uart->ENABLE &= ~SET_UART_TXEN(1);
}

static void
argus_enable_rx(struct argus_serial *info)
{
	info->uart->ENABLE |= SET_UART_RXEN(1);
}

static void
argus_disable_rx(struct argus_serial *info)
{
	info->uart->ENABLE &= ~SET_UART_RXEN(1);
}


/*
 * This function maps from the Bxxxx defines in asm/termbits.h into real
 * baud rates.
 */

static int 
cflag_to_baud(unsigned int cflag)
{
	static int baud_table[] = {
		0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400,
		4800, 9600, 19200, 38400 };

	static int ext_baud_table[] = {
		0, 57600, 115200, 230400, 460800, 921600, 1843200, 6250000,
                0, 0, 0, 0, 0, 0, 0, 0 };

	if (cflag & CBAUDEX)
		return ext_baud_table[(cflag & CBAUD) & ~CBAUDEX];
	else 
		return baud_table[cflag & CBAUD];
}

// change baud rate and other assorted parameters

static void 
change_speed(struct argus_serial *info)
{
	unsigned int cflag;

	// first some safety checks
	
	if (!info->tty || !info->tty->termios)
		return;
	if (!info->uart)
		return;
	
	cflag = info->tty->termios->c_cflag;
	
	// possibly, the tx/rx should be disabled first to do this safely
	
	// change baud-rate and write it to the hardware
	
	info->baud = cflag_to_baud(cflag);

	// Check for port features to use - autoflowcontrol, 2 stop bit
	// mode. Argus2 does not support 7-bit mode or parity.

	info->uart->CTRL = SET_UART_MODE(0) |
		SET_UART_AUTOFLOW((cflag & CRTSCTS) ? 1 : 0) |
		SET_UART_TXPOL(0) |
		SET_UART_RXPOL(0) |
		SET_UART_STOPBITS((cflag & CSTOPB) ? 1 : 0) |
		SET_UART_LOOPBACK(0);

	// TODO: change this into a calculation or something so we can
	// support other baudrates easier.
	//
	// The formula for the BAUD divider register is:
	//
	// divider = 12 MHz / (13 * baudrate)

	switch(info->baud) {
		case 9600:
			info->uart->BAUD = SET_UART_BAUD(UART_BAUD_9600);
			info->uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(6)       |
				SET_UART_RXEND(5)         |
				SET_UART_IRDARXLEN(15);
			break;
		case 115200:
			info->uart->BAUD = SET_UART_BAUD(UART_BAUD_115200);
			info->uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(6)       |
				SET_UART_RXEND(5)         |
				SET_UART_IRDARXLEN(15);
			break;
		case 230400:
			info->uart->BAUD = SET_UART_BAUD(UART_BAUD_230400);
			info->uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(6)       |
				SET_UART_RXEND(5)         |
				SET_UART_IRDARXLEN(15);
			break;
		case 460800:
			info->uart->BAUD = SET_UART_BAUD(UART_BAUD_460800);
			info->uart->CONFIG = SET_UART_IRDATXLEN(0)     |
				SET_UART_RXSTART(4)       |
				SET_UART_RXEND(3)         |
				SET_UART_IRDARXLEN(1);
			break;
	}

	argus_enable_tx(info);
	argus_enable_rx(info);
}

/*
 * This routine is used by the interrupt handler to schedule
 * processing in the software interrupt portion of the driver.
 */

static void 
rs_sched_event(struct argus_serial *info,
	       int event)
{
	if (info->event & (1 << event))
		return;
	info->event |= 1 << event;
	schedule_work(&info->work);
}

// Called from the interrupt handler signalling that the
// TX FIFO is half empty or less.

static void
transmit_chars(struct argus_serial *info)
{
	// TODO: check info->x_char

	if (info->xmit.head == info->xmit.tail
            || info->tty->stopped
            || info->tty->hw_stopped) {
                argus_disable_tx_irq(info);
                return;
        }

	// Try to fill the transmit FIFO.

        while (GET_UART_TXNFULL(info->uart->STATUS)) {
                info->uart->DATA = info->xmit.buf[info->xmit.tail];
                info->xmit.tail = (info->xmit.tail + 1) & (SERIAL_XMIT_SIZE - 1);
                //port->icount.tx++;   statistics
                if (info->xmit.head == info->xmit.tail)
                        break;
        }

        if (CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE) <
	    WAKEUP_CHARS)
                rs_sched_event(info, RS_EVENT_WRITE_WAKEUP);

        if (info->xmit.head == info->xmit.tail)
		argus_disable_tx_irq(info);
}

/*
 * tmp_buf is used as a temporary buffer by serial_write.  We need to
 * lock it in case the copy_from_user blocks while swapping in a page,
 * and some other program tries to do a serial write at the same time.
 * Since the lock will only come under contention when the system is
 * swapping and available memory is low, it makes sense to share one
 * buffer across all the serial ports, since it significantly saves
 * memory if large numbers of serial ports are open.
 */
static unsigned char *tmp_buf;
static DECLARE_MUTEX(tmp_buf_sem);

static void 
receive_chars(struct argus_serial *info)
{
	struct tty_struct *tty = info->tty;
	unsigned int status, ch, flg, ignored = 0;
	//struct	async_icount *icount;

	// Loop and grab all chars from the rx FIFO that we can
	// get our dirty hands on.

	status = info->uart->STATUS;

	while (GET_UART_RXNEMPTY(status)) {
		ch = info->uart->DATA;

		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
			goto ignore_char;
		flg = TTY_NORMAL;

		/*
		 * note that the error handling code is
		 * out of the main execution path
		 */
		if (GET_UART_OVERFLOW(status))  // TODO: more errors?
			goto handle_error;

		//if (uart_handle_sysrq_char(info, ch, regs))
		//	goto ignore_char;

	error_return:
		*tty->flip.flag_buf_ptr++ = flg;
		*tty->flip.char_buf_ptr++ = ch;
		tty->flip.count++;
	ignore_char:
		status = info->uart->STATUS;
	}
 out:
	tty_flip_buffer_push(tty);
	return;

handle_error:
#if 0
	if (GET_UART_OVERFLOW(status))
		port->icount.overrun++;
#endif
	if (status & info->ignore_status_mask) {
		if (++ignored > 100)
			goto out;
		goto ignore_char;
	}

	status &= info->read_status_mask;

	// TODO: check that it is enough to simply read
	// data from the UART to clear the UART overflow
	// bit!

	if (GET_UART_OVERFLOW(status)) {
		/*
		 * overrun does *not* affect the character
		 * we read from the FIFO
		 */
		*tty->flip.flag_buf_ptr++ = flg;
		*tty->flip.char_buf_ptr++ = ch;
		tty->flip.count++;
		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
			goto ignore_char;
		ch = 0;
		flg = TTY_OVERRUN;
	}
#ifdef SUPPORT_SYSRQ
	info->sysrq = 0;
#endif
	goto error_return;

}

static irqreturn_t
argus_uart_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct argus_serial *info;
	int i;

	// figure out what in the uarts caused the interrupt

	for (i = 0; i < NR_PORTS; i++) {
		info = rs_table + i;

		if(GET_UART_TXHEMPTY(info->uart->STATUS &
		                     info->IRQEN_shadow))
			transmit_chars(info);

		// Since we haven't implemented the timeout
		// functionality yet, we use RXNEMPTY to be able
		// to receive single chars, for now. Should use
		// RXHFULL instead later. TODO.

		if(GET_UART_RXNEMPTY(info->uart->STATUS &
		                     info->IRQEN_shadow))
			receive_chars(info);

		// No other interrupts can be enabled for now so
		// we don't check for them either.
	}

	// We return handled even if we did not touch anything, because
	// this interrupt cannot be shared with anything else anyway
	return IRQ_HANDLED;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void 
shutdown(struct argus_serial * info)
{
	unsigned long flags;

	argus_disable_rx(info);
	argus_disable_tx(info);
	argus_disable_rx_irq(info);
	argus_disable_tx_irq(info);
	
	if (!(info->flags & ASYNC_INITIALIZED))
		return;
	
	local_irq_save(flags);
	
	if (info->xmit.buf) {
		free_page((unsigned long)info->xmit.buf);
		info->xmit.buf = NULL;
	}
	
	if (!info->tty || (info->tty->termios->c_cflag & HUPCL)) {
		/* hang up DTR and RTS if HUPCL is enabled */
		//e100_dtr(info, 0);
		//e100_rts(info, 0); /* could check CRTSCTS before doing this */
	}

	if (info->tty)
		set_bit(TTY_IO_ERROR, &info->tty->flags);
	
	info->flags &= ~ASYNC_INITIALIZED;
	local_irq_restore(flags);
}

static void 
rs_flush_chars(struct tty_struct *tty)
{
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;
	unsigned long flags;

	if (info->tr_running ||
	    info->xmit.head == info->xmit.tail ||
	    tty->stopped ||
	    tty->hw_stopped ||
	    !info->xmit.buf)
		return;

#ifdef SERIAL_DEBUG_FLOW
	printk("rs_flush_chars\n");
#endif
	
	/* this protection might not exactly be necessary here */
	/* (there is an op in enable_tx_irq which needs to be atomic though) */
	local_irq_save(flags);
	argus_enable_tx_irq(info);
	local_irq_restore(flags);
}

/* how much space is available in the xmit buffer? */

static int 
rs_write_room(struct tty_struct *tty)
{
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;

	return CIRC_SPACE(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

/* How many chars are in the xmit buffer?
 * This does not include any chars in the transmitter FIFO.
 * Use wait_until_sent for waiting for FIFO drain.
 */

static int 
rs_chars_in_buffer(struct tty_struct *tty)
{
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;

	return CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

static void 
rs_flush_buffer(struct tty_struct *tty)
{
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;
	unsigned long flags;
	
	local_irq_save(flags);
	info->xmit.head = info->xmit.tail = 0;
	local_irq_restore(flags);

	wake_up_interruptible(&tty->write_wait);

	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void 
rs_send_xchar(struct tty_struct *tty, char ch)
{
	printk(" DUMMY rs_send_xchar: '%c'\n", ch);
}

/*
 * ------------------------------------------------------------
 * rs_throttle()
 * 
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void rs_throttle(struct tty_struct * tty)
{
//	printk(" DUMMY rs_throttle\n");
}

static void rs_unthrottle(struct tty_struct * tty)
{
//	printk(" DUMMY rs_unthrottle\n");
}

/*
 * ------------------------------------------------------------
 * rs_ioctl() and friends
 * ------------------------------------------------------------
 */
/*
 * rs_break() --- routine which turns the break handling on or off
 */
static void rs_break(struct tty_struct *tty, int break_state)
{
//	printk(" DUMMY rs_break  State: %i\n", break_state);
}

static int 
rs_ioctl(struct tty_struct *tty, struct file * file,
	 unsigned int cmd, unsigned long arg)
{
	// No implemented ioctl's yet.
	return -ENOIOCTLCMD;
}

static void 
rs_set_termios(struct tty_struct *tty, struct termios *old_termios)
{
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;

	if (tty->termios->c_cflag == old_termios->c_cflag &&
	    tty->termios->c_iflag == old_termios->c_iflag)
		return;

	change_speed(info);

	if ((old_termios->c_cflag & CRTSCTS) &&
	    !(tty->termios->c_cflag & CRTSCTS)) {
		tty->hw_stopped = 0;
		rs_start(tty);
	}
}

/*
 * ------------------------------------------------------------
 * rs_close()
 * 
 * This routine is called when the serial port gets closed.  First, we
 * wait for the last remaining data to be sent.  Then, we unlink its
 * async structure from the interrupt chain if necessary, and we free
 * that IRQ if nothing is left in the chain.
 * ------------------------------------------------------------
 */
static void 
rs_close(struct tty_struct *tty, struct file * filp)
{
	struct argus_serial * info = (struct argus_serial *)tty->driver_data;
	unsigned long flags;

	if (!info)
		return;
  
	/* interrupts are disabled for this entire function */
  
	local_irq_save(flags); 
  
	if (tty_hung_up_p(filp)) {
		local_irq_restore(flags);
		return;
	}
  
#ifdef SERIAL_DEBUG_OPEN
	printk("[%d] rs_close ttyS%d, count = %d\n", current->pid, 
	       info->line, info->count);
#endif

	if ((tty->count == 1) && (info->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  Info->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk(KERN_CRIT 
		       "rs_close: bad serial port count; tty->count is 1, "
		       "info->count is %d\n", info->count);
		info->count = 1;
	}
	if (--info->count < 0) {
		printk(KERN_CRIT "rs_close: bad serial port count for ttyS%d: %d\n",
		       info->line, info->count);
		info->count = 0;
	}
	if (info->count) {
		local_irq_restore(flags);
		return;
	}
	info->flags |= ASYNC_CLOSING;
	/*
	 * Save the termios structure, since this port may have
	 * separate termios for callout and dialin.
	 */
	if (info->flags & ASYNC_NORMAL_ACTIVE)
		info->normal_termios = *tty->termios;
	/*
	 * Now we wait for the transmit buffer to clear; and we notify 
	 * the line discipline to only process XON/XOFF characters.
	 */
	tty->closing = 1;
	if (info->closing_wait != ASYNC_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, info->closing_wait);
	/*
	 * At this point we stop accepting input.  To do this, we
	 * disable the serial receiver and the receive interrupt.
	 */
	argus_disable_rx_irq(info);
	argus_disable_rx(info);

	if (info->flags & ASYNC_INITIALIZED) {
		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important as we have a transmit FIFO!
		 */
		rs_wait_until_sent(tty, HZ);
	}

	shutdown(info);

	if (tty->driver->flush_buffer)
		tty->driver->flush_buffer(tty);
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);
	tty->closing = 0;
	info->event = 0;
	info->tty = 0;
	if (info->blocked_open) {
		if (info->close_delay) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(info->close_delay);
		}
		wake_up_interruptible(&info->open_wait);
	}
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CLOSING);
	wake_up_interruptible(&info->close_wait);
	local_irq_restore(flags);
}

/*
 * rs_wait_until_sent() --- wait until the transmitter is empty
 */
static void 
rs_wait_until_sent(struct tty_struct *tty, int timeout)
{
	unsigned long orig_jiffies;
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;
	unsigned long curr_time = jiffies;
	//unsigned long curr_time_usec = GET_JIFFIES_USEC();
	//	long elapsed_usec = 
	//	(curr_time - info->last_tx_active) * (1000000/HZ) + 
	//	curr_time_usec - info->last_tx_active_usec;

	orig_jiffies = jiffies;

	while (info->xmit.head != info->xmit.tail || /* More in send queue */
	       !GET_UART_TXIDLE(info->uart->STATUS))  /* more in FIFO */
	       // || (elapsed_usec < 2*info->char_time_usec))
		{
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(1);
		if (signal_pending(current))
			break;
		if (timeout && time_after(jiffies, orig_jiffies + timeout))
			break;
		curr_time = jiffies;
		//curr_time_usec = GET_JIFFIES_USEC();
		//elapsed_usec = 
		//	(curr_time - info->last_tx_active) * (1000000/HZ) + 
		//	curr_time_usec - info->last_tx_active_usec;
	}
	set_current_state(TASK_RUNNING);
}

/*
 * rs_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void 
rs_hangup(struct tty_struct *tty)
{
	struct argus_serial * info = (struct argus_serial *)tty->driver_data;
	
	rs_flush_buffer(tty);
	shutdown(info);
	info->event = 0;
	info->count = 0;
	info->flags &= ~ASYNC_NORMAL_ACTIVE;
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
}

/*
 * This routine is used to handle the "bottom half" processing for the
 * serial driver, known also the "software interrupt" processing.
 * This processing is done at the kernel interrupt level, after the
 * rs_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using rs_sched_event(), and they get done here.
 */

static void 
do_softint(void *private_)
{
	struct argus_serial	*info = (struct argus_serial *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	if (test_and_clear_bit(RS_EVENT_WRITE_WAKEUP, &info->event)) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.write_wakeup)
			(tty->ldisc.write_wakeup)(tty);
		wake_up_interruptible(&tty->write_wait);
	}
}

/*
 * ---------------------------------------------------------------
 * Low level utility subroutines for the serial driver:  routines to
 * figure out the appropriate timeout for an interrupt chain, routines
 * to initialize and startup a serial port, and routines to shutdown a
 * serial port.  Useful stuff like that.
 * ---------------------------------------------------------------
 */

static int 
startup(struct argus_serial * info)
{
	unsigned long flags;
	unsigned long xmit_page;

	xmit_page = get_zeroed_page(GFP_KERNEL);
	if (!xmit_page)
		return -ENOMEM;

	local_irq_save(flags);

	// if it was already initialized, skip this 

	if (info->flags & ASYNC_INITIALIZED) {
		local_irq_restore(flags);
		free_page(xmit_page);
		return 0;
	}

	if (info->xmit.buf)
		free_page(xmit_page);
	else
		info->xmit.buf = (unsigned char *) xmit_page;

#ifdef SERIAL_DEBUG_OPEN
	printk("starting up ttyS%d (xmit_buf 0x%p, recv_buf 0x%p)...\n",
	       info->line, info->xmit.buf, 0);
#endif

	if (info->tty)
		clear_bit(TTY_IO_ERROR, &info->tty->flags);

	info->xmit.head = info->xmit.tail = 0;
	info->first_recv_buffer = info->last_recv_buffer = NULL;
	info->recv_cnt = info->max_recv_cnt = 0;

	// TODO: Make sure the UART is flushed and ready here ?

	change_speed(info);

	/*
	 * Finally, enable interrupts.
	 * (no point in enabling tx irq since we dont have anything
	 *  to send yet though)
	 */

	argus_enable_rx_irq(info);
	
	info->flags |= ASYNC_INITIALIZED;

	local_irq_restore(flags);
	return 0;
}

static int 
block_til_ready(struct tty_struct *tty, struct file * filp,
		struct argus_serial *info)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long	flags;
	int		retval;
	int		do_clocal = 0, extra_count = 0;
	
	/*
	 * If the device is in the middle of being closed, then block
	 * until it's done, and then try again.
	 */
	if (tty_hung_up_p(filp) ||
	    (info->flags & ASYNC_CLOSING)) {
		if (info->flags & ASYNC_CLOSING)
			interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
		if (info->flags & ASYNC_HUP_NOTIFY)
			return -EAGAIN;
		else
			return -ERESTARTSYS;
#else
		return -EAGAIN;
#endif
	}

	/*
	 * If non-blocking mode is set, or the port is not enabled,
	 * then make the check up front and then exit.
	 */
	if ((filp->f_flags & O_NONBLOCK) ||
	    (tty->flags & (1 << TTY_IO_ERROR))) {
		info->flags |= ASYNC_NORMAL_ACTIVE;
		return 0;
	}
	
	if (tty->termios->c_cflag & CLOCAL)
		do_clocal = 1;
	
	/*
	 * Block waiting for the carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, info->count is dropped by one, so that
	 * rs_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */
	retval = 0;
	add_wait_queue(&info->open_wait, &wait);
#ifdef SERIAL_DEBUG_OPEN
	printk("block_til_ready before block: ttyS%d, count = %d\n",
	       info->line, info->count);
#endif
	local_irq_save(flags); 
	if (!tty_hung_up_p(filp)) {
		extra_count++;
		info->count--;
	}
	local_irq_restore(flags);
	info->blocked_open++;
	while (1) {
		//local_irq_save(flags);
		/* TODO: assert RTS and DTR */
		//e100_rts(info, 1);
		//e100_dtr(info, 1);
		//local_irq_restore(flags);
		set_current_state(TASK_INTERRUPTIBLE);
		if (tty_hung_up_p(filp) ||
		    !(info->flags & ASYNC_INITIALIZED)) {
#ifdef SERIAL_DO_RESTART
			if (info->flags & ASYNC_HUP_NOTIFY)
				retval = -EAGAIN;
			else
				retval = -ERESTARTSYS;	
#else
			retval = -EAGAIN;
#endif
			break;
		}
		if (!(info->flags & ASYNC_CLOSING) && do_clocal)
			/* && (do_clocal || DCD_IS_ASSERTED) */
			break;
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
#ifdef SERIAL_DEBUG_OPEN
		printk("block_til_ready blocking: ttyS%d, count = %d\n",
		       info->line, info->count);
#endif
		schedule();
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&info->open_wait, &wait);
	if (extra_count)
		info->count++;
	info->blocked_open--;
#ifdef SERIAL_DEBUG_OPEN
	printk("block_til_ready after blocking: ttyS%d, count = %d\n",
	       info->line, info->count);
#endif
	if (retval)
		return retval;
	info->flags |= ASYNC_NORMAL_ACTIVE;
	return 0;
}	

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int 
rs_open(struct tty_struct *tty, struct file * filp)
{
	struct argus_serial	*info = 0;
	int 			retval, line;
	unsigned long		page;

	/* find which port we want to open */

	line = tty->index;

	if (line < 0 || line >= NR_PORTS)
		return -ENODEV;

	info = rs_table + line;

	/* don't allow the open of ports that are not enabled in the HW config */
	if (!info->enabled)
		return -ENODEV; 

#ifdef SERIAL_DEBUG_OPEN
	printk("[%d] rs_open %s%d, count = %d\n", current->pid,
	       tty->driver.name, info->line,
	       info->count);
#endif
	info->count++;
	tty->driver_data = info;
	info->tty = tty;

	info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

	if (!tmp_buf) {
		page = get_zeroed_page(GFP_KERNEL);
		if (!page)
			return -ENOMEM;
		if (tmp_buf)
			free_page(page);
		else
			tmp_buf = (unsigned char *) page;
	}

	/*
	 * If the port is the middle of closing, bail out now
	 */

	if (tty_hung_up_p(filp) ||
	    (info->flags & ASYNC_CLOSING)) {
		if (info->flags & ASYNC_CLOSING)
			interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
		return ((info->flags & ASYNC_HUP_NOTIFY) ?
			-EAGAIN : -ERESTARTSYS);
#else
		return -EAGAIN;
#endif
	}

	/*
	 * Start up serial port
	 */

	retval = startup(info);

	if (retval)
		return retval;

	retval = block_til_ready(tty, filp, info);
	if (retval) {
#ifdef SERIAL_DEBUG_OPEN
		printk("rs_open returning after block_til_ready with %d\n",
		       retval);
#endif
		return retval;
	}

	if ((info->count == 1) &&
	    (info->flags & ASYNC_SPLIT_TERMIOS)) {
		*tty->termios = info->normal_termios;
		change_speed(info);
	}

#ifdef SERIAL_DEBUG_OPEN
	printk("rs_open ttys%d successful...\n", info->line);
#endif
	return 0;
}

/*
 * ------------------------------------------------------------
 * rs_stop() and rs_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 * ------------------------------------------------------------
 */
static void 
rs_stop(struct tty_struct *tty)
{
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;
	unsigned long flags;

	local_irq_save(flags);
	argus_disable_tx_irq(info);
	local_irq_restore(flags);
}

static void 
rs_start(struct tty_struct *tty)
{
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;
	unsigned long flags;

	local_irq_save(flags);
	// If there are more chars to send, enable the tx interrupt
	// so we start again.
	if (info->xmit.head != info->xmit.tail
	    && info->xmit.buf)
		argus_enable_tx_irq(info);
	local_irq_restore(flags);
}

static int 
rs_write(struct tty_struct * tty,
	 const unsigned char *buf, int count)
{
	int c, ret = 0;
	struct argus_serial *info = (struct argus_serial *)tty->driver_data;
	unsigned long flags;
				
	if (!tty || !info->xmit.buf || !tmp_buf)
		return 0;

	/* the local_irq_disable/restore_flags pairs below are needed because the
	 * DMA interrupt handler moves the info->xmit values. the memcpy
	 * needs to be in the critical region unfortunately, because we
	 * need to read xmit values, memcpy, write xmit values in one
	 * atomic operation... this could perhaps be avoided by more clever
	 * design.
	 */

	local_irq_save(flags);

	while (count) {
		c = CIRC_SPACE_TO_END(info->xmit.head,
				      info->xmit.tail,
				      SERIAL_XMIT_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;

		memcpy(info->xmit.buf + info->xmit.head, buf, c);
		info->xmit.head = ((info->xmit.head + c) &
				   (SERIAL_XMIT_SIZE-1));
		buf += c;
		count -= c;
		ret += c;
	}

	local_irq_restore(flags);

	// Enable the transmitter interrupts if we have data to send.

	if (info->xmit.head != info->xmit.tail
	    && !tty->stopped
	    && !tty->hw_stopped)
		argus_enable_tx_irq(info);

	return ret;
}

static inline int 
baud_to_c(int baudrate)
{
	switch(baudrate) {
	    case 1200:   return B1200;
	    case 2400:   return B2400;
	    case 4800:   return B4800;
	    case 9600:   return B9600;
	    case 19200:  return B19200;
	    case 38400:  return B38400;
	    case 57600:  return B57600;
	    case 115200: return B115200;
	    case 230400: return B230400;
	    default:     return B9600;
	}
}
 
static struct tty_operations rs_ops = {
	.open = rs_open,
	.close = rs_close,
	.write = rs_write,
	.flush_chars = rs_flush_chars,
	.write_room = rs_write_room,
	.chars_in_buffer = rs_chars_in_buffer,
	.flush_buffer = rs_flush_buffer,
	.ioctl = rs_ioctl,
	.throttle = rs_throttle,
        .unthrottle = rs_unthrottle,
	.set_termios = rs_set_termios,
	.stop = rs_stop,
	.start = rs_start,
	.hangup = rs_hangup,
	.break_ctl = rs_break,
	.send_xchar = rs_send_xchar,
	.wait_until_sent = rs_wait_until_sent,
	.read_proc = 0  //rs_read_proc
};

/*
 * The serial driver boot-time initialization code!
 */

static int __init 
rs_init(void)
{
	int i;
	struct argus_serial *info;

	serial_driver = alloc_tty_driver(NR_PORTS);

	if (!serial_driver)
		return -ENOMEM;

	printk("Argus serial port driver initializing\n");

	/* Initialize the tty_driver structure */

	serial_driver->driver_name = "Argus serial";
	serial_driver->name = "ttyS";
	serial_driver->major = ARGUS_PORT_MAJOR;
	serial_driver->minor_start = 64;
	serial_driver->type = TTY_DRIVER_TYPE_SERIAL;
	serial_driver->subtype = SERIAL_TYPE_NORMAL;
	serial_driver->init_termios = tty_std_termios;
	serial_driver->init_termios.c_cflag = 
		(baud_to_c(CONFIG_ARGUS_DEBUG_PORT_BAUD) | 
		 CS8 | CREAD | HUPCL | CLOCAL);
	serial_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_NO_DEVFS;
	serial_driver->termios = serial_termios;
	serial_driver->termios_locked = serial_termios_locked;

	tty_set_operations(serial_driver, &rs_ops);
	
	if (tty_register_driver(serial_driver))
		panic("Couldn't register serial driver\n");

	/* do some initializing for the separate ports */
  
	for (i = 0, info = rs_table; i < NR_PORTS; i++,info++) {
		info->uses_dma = 0;   
		info->line = i;
		info->tty = 0;
		info->type = PORT_ARGUS;
		info->tr_running = 0;
		info->forced_eop = 0;
		info->flags = 0;
		info->close_delay = 5*HZ/10;
		info->closing_wait = 30*HZ;
		info->x_char = 0;
		info->event = 0;
		info->count = 0;
		info->blocked_open = 0;
		info->normal_termios = serial_driver->init_termios;
		init_waitqueue_head(&info->open_wait);
		init_waitqueue_head(&info->close_wait);
		info->xmit.buf = NULL;
		info->xmit.tail = info->xmit.head = 0;
		info->first_recv_buffer = info->last_recv_buffer = NULL;
		info->recv_cnt = info->max_recv_cnt = 0;
		info->last_tx_active_usec = 0;
		info->last_tx_active = 0;

		INIT_WORK(&info->work, do_softint, info);

		if (info->enabled) {
			static const char * const irqname[2] = { "uart0", 
								 "uart1" };
#ifdef FORCE_ENABLE_RX_PINS
			static const int rxpin[2] = { GPIO_UART0_RXD,
						      GPIO_UART1_RXD};
#endif
			static const int txpin[2] = { GPIO_UART0_TXD,
						      GPIO_UART1_TXD};
			printk(KERN_INFO "%s%d at 0x%p is a builtin UART%d\n",
			       serial_driver->name, info->line, info->uart, i);
			
			// Need to clear the uart interrupts to avoid a burst
			// of things when we request_irq below.
			info->uart->IRQEN = info->IRQEN_shadow = 0;
			
			// Enable UART clock
			SOC_CLK.ENABLE = CLK_UART0 << i;
			
			// Configure pins
#ifdef FORCE_ENABLE_RX_PINS
			SOC_GPIO.PIN[rxpin[i]] = 
				SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
#endif
			SOC_GPIO.PIN[txpin[i]] =
				SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
			
			if(i == 0) {
				// We only have CTS/RTS on Uart 0 on Argus2.
				// Possibly the use of these GPIO pins as 
				// CTS/RTS should be configurable.
#ifdef FORCE_ENABLE_RX_PINS
				SOC_GPIO.PIN[GPIO_UART0_CTS] = 
					SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
#endif
//				SOC_GPIO.PIN[GPIO_UART0_RTS] = 
//					SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
			}

			// We use the same interrupt handler for both UART's.
			// It figures out who caused the irq for itself.

			request_irq(IRQ_UART0 + i, argus_uart_interrupt,
				    SA_INTERRUPT, irqname[i], NULL);
		}
	}
	
	return 0;
}

/* this makes sure that rs_init is called during kernel boot */

module_init(rs_init);

/*
 * register_serial and unregister_serial allows for serial ports to be
 * configured at run-time, to support PCMCIA modems.
 */
int 
register_serial(struct serial_struct *req)
{
	return -1;
}

void unregister_serial(int line)
{
}

