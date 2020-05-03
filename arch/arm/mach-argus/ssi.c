// SSI driver for Argus2/3
//
// Copyright (c) 2004,2005 Axis Communications AB
//
// Authors:   Bjorn Wesen

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/timer.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/pgalloc.h>
#include <asm/arch/argus.h>
#include <asm/arch/argdma.h>
#include <asm/arch/dma_alloc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/ssi.h>
#include <asm/arch/sys_time.h>

#define DBG(x)
#define DBG2(x)

#ifdef CONFIG_ARGUS_3
#define BASE_CLOCK 75000000 /* HCLK is 150/2 MHz */
#else
#define BASE_CLOCK 97500000 /* HCLK is 195/2 MHz */
#endif

#define SYNC_SERIAL_MAJOR 125

#define NUM_IN_BUFFERS 4
#define IN_BUFFER_SIZE 2048

#define NUM_OUT_BUFFERS 4
#define OUT_BUFFER_SIZE 2048

#define NUM_DMA_SLOTS 2

enum bufstate {
	BUF_IDLE,        // not used - can be issued to DMA
	BUF_RUNNING,     // issued to the DMA
	BUF_DONE,        // DMA is done, has valid data
	BUF_COPYING      // usermode is copying the contents
};

struct ssi_buffer {
	char *buffer;

	volatile enum bufstate state;
	int copy_pos;
	struct timeval timestamp;  // set when grabbed
};

struct ssi_port {
	struct ssi_buffer inbuf[NUM_IN_BUFFERS];
	struct ssi_buffer outbuf[NUM_OUT_BUFFERS];

	volatile int cur_in, cur_out;  // Buffer currently filled by DMA

	int dma_in_ch, dma_out_ch;     // Allocated DMA channel numbers
	volatile int dma_in_running, dma_out_running; // Set if DMA is running

	wait_queue_head_t wq_in, wq_out;

	int skip_start;
	int skip_end;
	int skip_total;

	//	int inited:1;
};

static int ssi_start_dma_in(struct ssi_port *p);

// Get a timeval that is not affected by date-sets, which we can use in timestamps

extern int argus_gettimeoffset(void);  // included from asm/arch/time.h

static void
ssi_get_monotonic_time(struct timeval *tv)
{
#ifdef CLOCK_MONOTONIC
	struct timespec tp;

	do_posix_clock_monotonic_gettime(&tp);
	tv->tv_sec = tp.tv_sec;
	tv->tv_usec = tp.tv_nsec / 1000;
#else
	static const struct timeval wrap_time = {
		.tv_sec = (ULONG_MAX+1ULL) / HZ,
		.tv_usec = ((ULONG_MAX+1ULL) % HZ) * (1000000 / HZ)
	};
	static struct timeval total_wrap_time;
	static unsigned long last;
	int flags;

	local_irq_save(flags);

	tv->tv_sec = jiffies / HZ;
	tv->tv_usec = argus_gettimeoffset() + (jiffies % HZ) * (1000000 / HZ);

	if (tv->tv_usec > 1000000) {
		tv->tv_sec++;
		tv->tv_usec -= 1000000;
	}

	/* Check if jiffies has wrapped. */
	if (jiffies < last) {
		timeradd(&total_wrap_time, &wrap_time, &total_wrap_time);
	}
	last = jiffies;
	local_irq_restore(flags);

	timeradd(tv, &total_wrap_time, tv);
#endif
}

static int
setup_ssi_buffer(struct ssi_buffer *buf)
{
	if((buf->buffer = kmalloc(IN_BUFFER_SIZE, GFP_KERNEL)) == 0) {
		printk("cant alloc buffer\n");
		return -ENOMEM;
	}
	return 0;
}

static void
free_ssi_buffer(struct ssi_buffer *buf)
{
	kfree(buf->buffer);
}

static struct ssi_port port0;

/* Copy to userspace */
static int copy(char* dest, char* src, int len, struct ssi_port *p)
{
	if ((p->skip_start == 0) && (p->skip_end == 0))
		return copy_to_user(dest, src, len);
	else
	{
		int i;
		int user_len = 4 - p->skip_total;
		if (user_len == 2)
			for (i = 0; i < len; i+= 2)
				put_user(*(unsigned short*)(src + p->skip_start + i*2), 
					 (unsigned short*)(dest + i)); 
	} 
	return 0;
}
// Read from the input. Wait until enough data is available to
// satisfy the read length aligned to a buffer size.
//
// If the port input DMA is not running, it is started and then we
// wait until the first buffer is ready.
//
// If we cannot keep up with the DMA, the DMA will shut off and
// purge all non-read buffers, to avoid reading non-sequential
// data upon the next read.

static ssize_t
ssi_common_read(struct ssi_port *p, char *buf, size_t len, 
		struct timeval *timestamp)
{
	unsigned long flags;
	int bidx, numread, error;
	const int blen = IN_BUFFER_SIZE;
	struct ssi_buffer *b;

	numread = 0;

	while(len > 0) {
		int copied; 
			
		local_irq_save(flags);

		// Find the oldest non-running buffer with active
		// data.
		
		bidx = (p->cur_in + 1) % NUM_IN_BUFFERS; // Oldest buffer
		while(bidx != p->cur_in && 
		      p->inbuf[bidx].state != BUF_DONE)
			bidx = (bidx + 1) % NUM_IN_BUFFERS;
		
		// If we have no data, block and try again.
		
		if(bidx == p->cur_in) {
			// Do we need to start the DMA ?

			if(!p->dma_in_running) {
				DBG(printk("SSI: starting DMA\n"));
				SOC_CLK.ENABLE = CLK_SSI;
				ssi_start_dma_in(p);
			}

			// The interrupts need to be disabled up until after this
			// sleep, to avoid the race where the irq routine wakes
			// the queue before we have time to sleep.

			interruptible_sleep_on(&p->wq_in);

			local_irq_restore(flags);

			// Might not always get data here, could be a signal
			// as well, then we bail out immediately.

			if(signal_pending(current)) {
				DBG(printk("ssi_read: bailing due to signal\n"));
				return -EAGAIN;
			}

			// Go back and get the new data.
			continue;
		}
		
		// Lock it, restore interrupts and copy the data.

		b = &p->inbuf[bidx];

		b->state = BUF_COPYING;

		local_irq_restore(flags);

		if (len * 4 / (4 - p->skip_total) >= blen - b->copy_pos) {
			copied = (blen - b->copy_pos) / 4 * (4 - p->skip_total);
			error = copy(buf, b->buffer + b->copy_pos, blen - b->copy_pos, p);
			b->state = BUF_IDLE;
			b->copy_pos = 0;
		} else {
			copied = len;
			error = copy(buf, b->buffer + b->copy_pos, len, p);
			b->state = BUF_DONE;
			b->copy_pos += len * 4 / ( 4 - p->skip_total);
		}

		if(error) {
			DBG(printk("ssi_read: user buffer faulted\n"));
			return -EFAULT;
		}

		// Copy out the timestamp of the buffer. NOTE: if the read covers many
		// buffers we will then get the last timestamp.
		
		if(timestamp)
			*timestamp = b->timestamp;

		buf += copied;
		len -= copied;
		numread += copied;
	}

	return numread;
}

// Read operation on the fd - blocking read of a buffer

static ssize_t
ssi_read(struct file *filp, char *buf, size_t len, loff_t *ppos)
{
	struct ssi_port *p = (struct ssi_port *)filp->private_data;

	return ssi_common_read(p, buf, len, NULL);
}

// Read operation on the fd through an ioctl, we get a struct describing the
// destination of the data, and where we can put the timestamp of the buffer.

static int
ssi_extended_read(struct ssi_port *p, unsigned long arg)
{
	struct ssi_request req;
	int read_bytes, ret;

	// Copy the request from user-mode
	
	ret = copy_from_user(&req,
			     (struct ssi_request *)arg,
                             sizeof(struct ssi_request));
        
        if(ret)
                return -EFAULT;
	
	// Do the actual read

	read_bytes = ssi_common_read(p, req.buf, req.length, &req.timestamp);

	if(read_bytes < 0)
		return read_bytes;

	// Copy back the request

	ret = copy_to_user((struct ssi_request *) arg,
			   &req,
			   sizeof(struct ssi_request)); 

	if(ret)
		return -EFAULT;

	// Return the number of bytes read

	return read_bytes;
}

static int 
ssi_ioctl(struct inode *inode,
	  struct file *file,
	  unsigned int cmd, 
	  unsigned long arg)
{
	struct ssi_port *p = (struct ssi_port *)file->private_data;

	switch(cmd)
	{
		case SSI_SKIP_START:
			if (arg > 4)
				return -EINVAL;
			//p->skip_start = arg;
			//p->skip_total = p->skip_start + p->skip_end;
			break;
		case SSI_SKIP_END:
			if (arg > 4)
				return -EINVAL;
			//p->skip_end = arg;
			//p->skip_total = p->skip_start + p->skip_end;
			break;

		case SSI_GET_BUFFER:
			// This is analogous to read(), but a struct is passed
			// in that can get metadata like the timestamp. It is still
			// blocking though.
			return ssi_extended_read(p, arg);
	}	
	return 0;
}

// Should be called with irq's disabled.
//
// Returns the number of new issued buffers, if this is 0
// and we call from a DMA done context, it means that the
// we have caught up with the users datareading.

static int
ssi_start_dma_in(struct ssi_port *p)
{
	int started = 0;

	// Enable the input DMA channel

	dma_enable(p->dma_in_ch, 1, 1);
	dma_irq_enable(p->dma_in_ch, 1);
	p->dma_in_running = 1;

	// Start the input DMA on the buffer pointed
	// to by the cur_in index. Issue as many unused buffers
	// as will fit in the DMA queue.

	while(p->inbuf[p->cur_in].state == BUF_IDLE &&
	      dma_buffer_status(p->dma_in_ch, 0) < NUM_DMA_SLOTS) {

		DBG2(printk("adding buf at 0x%p\n", p->inbuf[p->cur_in].buffer));

		ct200_dma_inv_range((int)p->inbuf[p->cur_in].buffer, 
				    (int)p->inbuf[p->cur_in].buffer + IN_BUFFER_SIZE);

		dma_add_buffer(p->dma_in_ch,
			       virt_to_phys(p->inbuf[p->cur_in].buffer),
			       IN_BUFFER_SIZE,
			       DMA_FLAGS_CACHEABLE | DMA_FLAGS_BUFFERABLE);
		started++;

		p->inbuf[p->cur_in].state = BUF_RUNNING;
		p->inbuf[p->cur_in].copy_pos = 0;

		p->cur_in = (p->cur_in + 1) % NUM_IN_BUFFERS;
	}
	
	return started;
}

// Called when a buffer is completely read. Mark it done.
// Wake up waiters and issue new buffers.
// If there are no free buffers, halt and
// clear everything since we do not want the user
// to read data with gaps in it.

static void
ssi_in_callback(int ch)
{
	int i, bidx;

#ifdef CONFIG_ARGUS_3
        // Ack
        SOC_DMA.CHANNEL[ch].CHACTRL = SET_DMA_CHACTRL_IRQ(1);
#endif

	// No need to safetycheck the channel number since we're
	// the only callback for the input DMA channel.

	// Find the buffer that completed - it is the oldest
	// running buffer.

	bidx = (port0.cur_in + 1) % NUM_IN_BUFFERS; // Oldest buffer
	while(bidx != port0.cur_in && 
	      port0.inbuf[bidx].state != BUF_RUNNING)
		bidx = (bidx + 1) % NUM_IN_BUFFERS;

	if(bidx == port0.cur_in) // Should always have something here.
		printk("SSI: BUG, bogus completed buffer %d\n", bidx);

	// Mark it done and set its timestamp

	port0.inbuf[bidx].state = BUF_DONE;
		
	ssi_get_monotonic_time(&port0.inbuf[bidx].timestamp);

	// Wake up any waiters

	wake_up_interruptible(&port0.wq_in);

	// Issue more buffers to the DMA

	if(!ssi_start_dma_in(&port0)) {
		// Halt and clear all done flags
		dma_enable(ch, 0, 0);
		dma_irq_enable(ch, 0);
		port0.dma_in_running = 0;
		for(i = 0; i < NUM_IN_BUFFERS; i++) {
			// Leave the buffers that are already
			// copying to usermode.
			if(port0.inbuf[i].state != BUF_COPYING)
				port0.inbuf[i].state = BUF_IDLE;
		}
		// Turn off the SSI-clock to save power
		SOC_CLK.DISABLE = CLK_SSI;
		DBG(printk("SSI: stopped and flushed\n"));
	}
}


// Open the port.
// 
// Allocate buffers and DMA channels

static int
ssi_open(struct inode *inode, struct file *filp)
{
	int i;

	memset(&port0, 0, sizeof(struct ssi_port));

        init_waitqueue_head(&port0.wq_in);
        init_waitqueue_head(&port0.wq_out);
        port0.skip_start = 0; //2;
        port0.skip_total = 0; //2;
        
	// Probably only need a small FIFO so request the smallest
	// available.

	if((port0.dma_in_ch = dma_alloc(ssi_in_callback, 0)) < 0) {
		printk("Dma alloc error");
		return -ENOMEM;
	}

	DBG(printk("ssi open: got dma in channel %d\n", port0.dma_in_ch));

	for(i = 0; i < NUM_IN_BUFFERS; i++) {
		if(setup_ssi_buffer(&port0.inbuf[i]) < 0) {
			dma_free(port0.dma_in_ch);
			return -ENOMEM;
		}
	}
  
	// Make sure the DMA channel is disabled but
	// correctly configured. dma_setup will leave the
	// channel in a flushed/reset state.
	//
	// SSI input is read from the SSI_OUT port!

	dma_setup(port0.dma_in_ch,
		  DMA_PORT_SSI_OUT, 1, 1, 
		  DMA_SIZE_SHORT, DMA_DEV2MEM); // DMA_SIZE_WORD for stereo
	
        filp->private_data = &port0;

	return 0;
}

// Close the port. 

static int 
ssi_release(struct inode *inode, struct file *filp)
{
	int i;
	struct ssi_port *p = (struct ssi_port *)filp->private_data;

	for(i = 0; i < NUM_IN_BUFFERS; i++)
		free_ssi_buffer(&p->inbuf[i]);

	dma_free(p->dma_in_ch);

	return 0;
}

static struct file_operations ssi_fops = {
	.owner   = THIS_MODULE,
	//.write   = ssi_write,
	.read    = ssi_read,
	//.poll    = ssi_poll,
	.ioctl   = ssi_ioctl,
	.open    = ssi_open,
	.release = ssi_release
};

#if 0
static unsigned int sqwave[] = {
	0x40004000, 0x40004000, 0x40004000, 0x40004000,
	0xc000c000, 0xc000c000, 0xc000c000, 0xc000c000 
};

static char *starry = "                                                               *";

static unsigned short buffy[256];
#endif

static int __init 
ssi_init(void)
{
	//int bufpos, i;

	if (register_chrdev(SYNC_SERIAL_MAJOR, "ssi", 
			    &ssi_fops) <0 ) {
		printk("ssi: no major!\n");
		return -EBUSY;
	}

#ifndef CONFIG_ARGUS_3
	// Setup the clock generator output pin, CLKGEN1.

	// For 16 khz codec operation, we need a clock
	// which is 250 times that value for AIC23B. 
	//
	// 16000*250=4 mhz
	//
	// With a system clock of 195 mhz in Argus2, we need to
	// divide by 48.75. So we set it at 49 = 24 + 25.
	
	gpio_set_mode(GPIO_CLKGEN, GPIO_MODE_ALT_FUNC, 0);

	SOC_CLKGEN[1].CTRL = SET_CLKGEN_LO(23) | SET_CLKGEN_HI(24); // 24.375 mhz

	SOC_CLK.ENABLE = CLK_CLKGEN1 | CLK_SSI;
#else
	SOC_CLK.ENABLE = CLK_SSI;
#endif

	// Setup the SSI interface for AIC23B "DSP-mode" format,
	// 16 bit stereo operation, 16 khz. 
	//
	// The TRIG value + 1 is the number of HCLK cycles between
	// each frame. A frame in DSP-mode is 2*16 bits (one stereo sample pair).
	// Thus TRIG + 1 = HCLK / 16000 => TRIG = 4687 (Argus-3).
	//
	// PERIOD controls the clock speed of the actual serial data.
	// It must be slower than 20 mhz according to the AIC23B datasheet
	// and it must be fast enough so that 33*tperiod > 1/fs.
	// (The number of data bits is 32, plus 1 cycle for the frame pulse)
	//
	// If we set PERIOD = 9, the cycle time will be 2*(9+1)/HCLK = 267 ns (Argus-3)
	// well above the 50 ns constraint, 
	// giving a total frame transfer time of 33 * 267 ns = 8.8 us.
	// 1/fs = 1 / 16000 = 62.5 us, so we're well below that constraint.

	SOC_SSI.SSIA = 
		SET_SSIA_TRIG(BASE_CLOCK / 16000 - 1) |
		SET_SSIA_PERIOD(9) |
		SET_SSIA_WIDTH(15); // mono operation read only half the data. 31 for stereo
		
	// The internal CLK of the SSI block in Argus divides each frame
	// transfer into cycles with 2 phases in each, phase 0 and phase 1.
	// Phase 0 is while CLK is positive and phase 1 when it is negative.
	// Incoming data is sampled on the positive edge of CLK.
	//
	// AIC23B in slave-mode samples SSI_FRM/SSI_TX on the positive
	// edge of SSI_CLK, with a setup/hold requirement of 10 ns each,
	// and outputs to SSI_RX with the beginning of a negative slope
	// on SSI_CLK (it seems).
	//
	// We need to invert the SSI_CLK compared to the internal CLK
	// to satisfy the setup/hold requirement on SSI_FRM. However
	// to satisfy our own hold requirements on the returned data
	// on SSI_RX we need to revert the SSI_CLK during the dataphase
	// because the codec outputs with start on the negative edge
	// of SSI_CLK so if we had been inverted, we would sample
	// on that edge with possible hold violations since they
	// specify a propagation delay of 0 min and 20 max.
	//
	// We can let SSI_CLK idle while we idle to save some power.
	//
	// The DSP-mode needs a positive SSI_FRM for one cycle before
	// the data.

	SOC_SSI.SSIB =
		SET_SSIB_TXEN(0)  |  // Enable transmitter
		SET_SSIB_RXEN(1)  |  // Enable receiver
		SET_SSIB_IFRM(0)  |  // Idle FRM inactive
		SET_SSIB_ICLK0(0) |  // Idle CLK inactive
		SET_SSIB_ICLK1(0) |  // -=-
		SET_SSIB_FFRM(1)  |  // FRM active high during frame sync
		SET_SSIB_FCLK0(0) |  // 
		SET_SSIB_FCLK1(1) |  // -=-
		SET_SSIB_DFRM(0)  |  // FRM inactive during data transfer
		SET_SSIB_DCLK0(0) |  // 
		SET_SSIB_DCLK1(1) |  // -=-
		SET_SSIB_LBM(0)   |  // Disable loopback
		SET_SSIB_FINV(0);    // Don't toggle FRM each frame (I2S).

	// Finally activate the I/O pins (they might already be
	// activated by the kernelconfig at boot).
	
	gpio_set_mode(GPIO_SSI_TX,  GPIO_MODE_ALT_FUNC, 0);
	gpio_set_mode(GPIO_SSI_RX,  GPIO_MODE_ALT_FUNC, 0);
	gpio_set_mode(GPIO_SSI_FRM, GPIO_MODE_ALT_FUNC, 0);
	gpio_set_mode(GPIO_SSI_CLK, GPIO_MODE_ALT_FUNC, 0);
			
	printk("Argus SSI driver v1.0\n");
#if 0
	do {
		bufpos = 0;
		while(bufpos < 128) {
			volatile unsigned int val;
			short a, b;
			if(GET_SSIB_DREQ_OUT(SOC_SSI.SSIB)) {
				val = SOC_SSI.DATA;
				a = (short)(val >> 16);
				b = (short)(val & 0xffff);
				buffy[bufpos++] = b;
			}
		}
#if 0
		printk("------------------------------------------------\n");
		for(i = 0; i < 128; i++) {
			printk("%07d: %s\n", buffy[i],
			       starry + 32 + (buffy[i] >> 11));
		}
#endif
	} while(1);
#endif
#if 0
	bufpos = 0;
	while(1) {
		if(GET_SSIB_DREQ_IN(SOC_SSI.SSIB)) {
			SOC_SSI.DATA = sqwave[bufpos];
			bufpos = (bufpos + 1) % 8;
		}
	}
#endif

	// Disable the SSI clock until used to save power

	SOC_CLK.DISABLE = CLK_SSI;

	return 0;
}

// Call init at boot or at module insertion.

module_init(ssi_init);
