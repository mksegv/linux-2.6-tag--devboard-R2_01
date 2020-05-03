/*
 *  Copyright (C) 2003,2004,2005 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/interrupt.h>

#include <asm/arch/ciobuf.h>
#include <asm/system.h>
#include <asm/page.h>
#include <asm/arch/argus.h>
#include <asm/arch/dma_alloc.h>
#include <asm/arch/argdma.h>

#define ASSERT(x) do { if(!(x)) BUG(); } while(0)

#define DDEBUG(x)
// Debug lowlevel irq refill handler - this is not really stable
// since the printk's themselves will probably cause the DMA source
// to overflow due to the added latencies.
#define DDEBUG2(x)

#define DDEBUGC(x)

// Argus-3 specific possibility to save power by gating the EXTMEM clock 
// while it is not used (since it needs to be enabled continously otherwise
// due to a bug)
#define USE_EXTMEM_GATING 1

// Some chip-specific DMA parameters.
// Try to put them here to avoid including config.h in too many .h files.

#ifdef CONFIG_ARGUS_3
// Argus 3
#define ARGUS_NR_DMA 10
static unsigned char fifo_size_flags[ARGUS_NR_DMA] = { 
	DMA_ALLOC_FIFO256,
	DMA_ALLOC_FIFO128,
	DMA_ALLOC_FIFO128,
	DMA_ALLOC_FIFO128,
	DMA_ALLOC_FIFO128,
	DMA_ALLOC_FIFO64,
	DMA_ALLOC_FIFO64,
	DMA_ALLOC_FIFO64,
	DMA_ALLOC_FIFO64,
	DMA_ALLOC_FIFO64 };
#else
// Argus 2
#define ARGUS_NR_DMA 8
static unsigned char fifo_size_flags[ARGUS_NR_DMA] = { 
	DMA_ALLOC_FIFO128,
	DMA_ALLOC_FIFO64,
	DMA_ALLOC_FIFO32,
	DMA_ALLOC_FIFO32,
	DMA_ALLOC_FIFO32,
	DMA_ALLOC_FIFO32,
	DMA_ALLOC_FIFO32,
	DMA_ALLOC_FIFO32 };
#endif

// Channel allocation status
static unsigned short g_channel_allocation;

struct channelinfo {
	dma_irq_callback g_channel_callback; // irq callback, if any
	dma_irq_callback kbuf_callback; // callback when kbuf is done, if any
	struct ciobuf *kbuf; // ciobuf mapping, if any
	int left;            // number of bytes left to transfer
	int last_end_addr;   // physical end address of the last buffer
	int kbuf_pagenbr;    // which page we're going to use next
	int headersize;      // starting offset into the ciobuf
	int xahb;            // flag set if using the XAHB bus (only on Argus-3)
};

static struct channelinfo channels[ARGUS_NR_DMA];

// Used for initialization check
static int g_initialized = 0;

#ifdef CONFIG_ARGUS_3
// Code to gate the EXTMEM clock in Argus-3 to save idle power
// (the automatic gating does not work with XAHB transfers)
//
// Note about this: the gating is done on a dma_alloc/dma_free basis
// not on the actual DMA channel usage basis, because the latter can
// get complicated with channel aborts, errors etc. It is easier to
// keep track of just the channel allocation and freeing, but it means
// the code that uses the DMA channels should be somewhat sane about
// releasing non-used DMA channels.

static int extmem_clock_users;

void
extmem_clock_enable(void)
{
#if USE_EXTMEM_GATING
	SOC_CLK.ENABLE = CLK_EXTMEM;	
	extmem_clock_users++;
#endif
}

void
extmem_clock_disable(void)
{
#if USE_EXTMEM_GATING
	// Only disable for real if we're the last user
	if(!--extmem_clock_users)
		SOC_CLK.DISABLE = CLK_EXTMEM;	

	if(extmem_clock_users < 0 || extmem_clock_users > 50)
		printk("BUG: extmem %d!\n", extmem_clock_users);
#endif
}
#endif

// TODO: check if this handler really needs SA_INTERRUPT

static irqreturn_t 
dma_irq_handler(int irqno, void *dev_id, struct pt_regs * regs)
{
	int cha = dma_get_irq();

	// dma_get_irq returns lowest channel number signalling irq
	// or -1 if none.

	if (cha >= 0) {
		ASSERT(channels[cha].g_channel_callback);
		channels[cha].g_channel_callback(cha);
	}
	return IRQ_HANDLED;
}

// This is called from the generic ARM code. It is too early, don't
// do anything here.

void __init 
arch_dma_init(void)
{
	// Do nothing here.
}

// This is called when initializing devices using the initcall
// mechanism. Just be sure it's linked in front of any drivers that
// need DMA support...

static int __init
dma_init(void)
{
	printk("Setting up DMA...\n");

#ifdef CONFIG_ARGUS_3
	// Clean up DMA state from boot rom
	// Reset DMA channel 0
	// (otherwise the DMA can get a hanging IRQ from DMA ch 0)
	dma_abort(0);

#if !USE_EXTMEM_GATING
	// If we do not use the EXTMEM gating, enable it here permanently
	SOC_CLK.ENABLE = CLK_EXTMEM;
#endif
#endif

	// Install dma irq handler
  
	if(request_irq(IRQ_DMA, dma_irq_handler, SA_INTERRUPT,
		       "dma", NULL) < 0)
		panic("cant install dma irq handler");
  
	// Setup global state
	g_channel_allocation = 0;
	g_initialized = 1;

	return 0;
}

__initcall(dma_init);

int 
dma_alloc(dma_irq_callback callback, int flags)
{
	int cha, size_requirement;
	unsigned long irqflags;

	ASSERT(g_initialized);
	
	DDEBUG(printk("dma_alloc flags 0x%x, callback 0x%x: ",
		      flags, callback));
	
	size_requirement = flags & 7;
	
	local_irq_save(irqflags);
  
	// Loop until we find a free channel that fulfill the fifo size
	// requirement
	for(cha = (ARGUS_NR_DMA - 1); cha >= 0; cha--)
		if (!(g_channel_allocation & (1 << cha)) &&
		     fifo_size_flags[cha] >= size_requirement)
			break;
	
	// Allocate channel and remember callback
	if (cha >= 0) {
		g_channel_allocation |= (1 << cha);
		channels[cha].g_channel_callback = callback;
#ifdef CONFIG_ARGUS_3
		// If the channel uses the XAHB bus, we need to do a bugfix for
		// Argus-3 where the EXTMEM block is not clocked automatically
		// for those transfers
		if(flags & DMA_ALLOC_XAHB) {
			channels[cha].xahb = 1;
			extmem_clock_enable();
		} else
			channels[cha].xahb = 0;
#endif
	}
	
	local_irq_restore(irqflags);
	
	// Reset the channel to avoid any surprises (who knows who
	// touched it before!)

	if(cha >= 0)
		dma_abort(cha);

	DDEBUG(printk("%d\r\n", cha));

	// -1 is returned in case of allocation failure
	return cha;
}

// Called as a callback from the DMA interrupt. The semantics of when
// the interrupt comes are slightly different between Argus-2 and Argus-3.
// In Argus-2, the irq typically comes when a new space in the descriptor
// buffer is available. In Argus-3, it comes whenever a descriptor with the
// IRQ bit set is completed, which is more useful but needs slightly
// different handling at the end.
//
// In either case, we load up a new page from the ciobuf if there are any
// left, and in the Argus-3 case we call the kbuf_callback if everything
// is done (not implemented for Argus-2 though!).

static void
dma_next_ciobuf(int cha)
{
	struct channelinfo *c = &channels[cha];

	ASSERT(c->kbuf);
	ASSERT(c->kbuf->pages);

#ifdef CONFIG_ARGUS_3
	// Acknowledge interrupt
	SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_IRQ(1);

	// If we're all done, call the kbuf callback if registred
	if(!c->left && c->kbuf_callback && !dma_buffer_status(cha, 0)) {
		c->kbuf_callback(cha);
		return;
	}
#endif

	DDEBUGC(printk("dma_next_ciobuf c->kbuf = 0x%x, c->left = %d\n",
		       c->kbuf, c->left));

	// As long as we have bytes to transfer and there is a slot in the
	// DMA buffer queue, issue a DMA buffer.

	while(c->left > 0 && dma_buffer_status(cha, 0) < 2) {

		int offset = 0;

		// If this is the first buffer, we need to take the possible
		// header space into consideration. The DMA should not put
		// anything there, but we need to advance the ciobuf start to
		// a position after the header.

		if(c->kbuf_pagenbr == 0) {
			int left_in_page;

			// Start with the ciobuf first-page offset

			offset = c->kbuf->offset;

			// Fit the header first.

			// how much of the first page is usable

			left_in_page = PAGE_SIZE - offset;

			if(c->headersize < left_in_page) {
				// header fits in buffer and we don't need 
				// to advance to the next ciobuf mappage
				offset += c->headersize;
			} else {
				// header fits exactly or not at all, 
				// need to advance to next page
				offset = c->headersize - left_in_page;
				c->kbuf_pagenbr++;
			}

			c->left -= c->headersize;
		}

		// If we still have room for some actual DMA data,
		// add a DMA buffer.

		if(c->left > 0) {
			int len;
			unsigned long physbuf =
				virt_to_phys(page_address(
                                       c->kbuf->pages[c->kbuf_pagenbr]));
			// max we can write in this page
			len = c->left > PAGE_SIZE ? 
				PAGE_SIZE : c->left;
			// Check if we have an offset (can only happen in the
			// first DMA buffer page)
			if(offset) {
				physbuf += offset;
				if(len > (PAGE_SIZE - offset))
					len = PAGE_SIZE - offset;
			}

			DDEBUG2(printk("len %d, left %d, offs %d, buf 0x%x\n",
				       len, c->left, offset, physbuf));

			// TODO: select flag DMA_FLAGS_XAHB 
			DDEBUGC(printk(" add buf 0x%x len %d\n", physbuf, len));
			dma_add_buffer(cha, physbuf, len,
				       DMA_FLAGS_CACHEABLE | DMA_FLAGS_BUFFERABLE);
			
			c->last_end_addr = physbuf + len;
			c->left -= len;     // decrease bytes to go
			c->kbuf_pagenbr++;  // next ciobuf mappage
		}
        }

	// Check if we need to enable the DMA queue interrupts. For Argus-3,
	// the IRQ bit in the descriptor will cause another interrupt when it
	// is completed (and thus when there are no more descriptors added to
	// complete, the interrupts will cease).

#ifndef CONFIG_ARGUS_3
	dma_irq_enable(cha, c->left > 0);
#endif
}

// Add a ciobuf mapped virtual user buffer to the queue, and set up
// irq callbacks to automatically traverse the pages inside the ciobuf.
// This cannot be called while the channel is already executing something
// so we don't need locking from IRQ's inside this function.
//
// Optionally a callback can be specified which is run when the transfer
// is finished and drained.
//
// The ciobuf has these fields among others:
//    length    - length in bytes of the total mapping
//    nr_pages  - how many individual pages are in the mapping
//    offset    - offset into the first page, where to start

void
dma_start_ciobuf( int cha, struct ciobuf *kbuf, int headersize, int tailsize,
		  dma_irq_callback callback)
{
	DDEBUG(printk("dma_start_ciobuf on cha %d, kbuf 0x%x, hsize %d\n",
		      cha, kbuf, headersize));

	// Note, if we had a DMA abort before, some of the channels[cha]
	// fields will still contain values so we can't check them for
	// incorrect channel reuse. We might want to fix that later though.

	// Remember the ciobuf for later use 

	channels[cha].kbuf = kbuf;
	channels[cha].kbuf_pagenbr = 0;
	channels[cha].headersize = headersize;
	channels[cha].left = kbuf->length - tailsize;

	// Override the callback with our ciobuf traversal function

	channels[cha].g_channel_callback = dma_next_ciobuf;
	
	// but remember the final ciobuf callback (if any)

	channels[cha].kbuf_callback = callback;
	
#ifdef CONFIG_ARGUS_3
	// Enable descriptor-completion interrupts (so we can load up new
	// descriptors in the queue as they complete)
	dma_irq_enable(cha, 1);
#endif
	
	// Start up the buffer queueing (this will enable further
	// queueing by interrupt if the buffer descriptors don't fit
	// in the HW queue).

	dma_next_ciobuf(cha);
}

// Figure out the final transfer size for the given channel.
// This needs to take ciobuf'ed transfers into account, and that
// there might be several buffers (up to two) in the queue with
// different start addresses.

unsigned int
dma_final_size_ciobuf(int cha)
{
	unsigned int end_addr;
	int num_bytes;
	int bufleft;
	unsigned long flags;
	unsigned long physbuf;
	struct channelinfo *c = &channels[cha];

	// To avoid races with the DMA buffer refill system,
	// we need to temporarily disable interrupts to make
	// sure that the amounts of buffers left in the channel
	// matches the ciobuf page numbering.
	//
	// Normally, this should not be necessary as this function
	// will be called between dma_drain and the next DMA session.

	local_irq_save(flags);

	ASSERT(c->kbuf);
	ASSERT(c->kbuf->pages);
	
	bufleft = dma_buffer_status(cha, &end_addr);

	// We need to relate end_addr to the buffer that the
	// DMA was working on, and the only way to do that is
	// to figure out what page in the ciobuf we started in.
	// This works differently depending on what buffers  
	// already were inside the DMA at the point we stopped.

	// The following code temporarily ignores that we do not really
	// DMA all of the first page, so we need to subsequently compensate
	// for this.
	
	switch(bufleft) {
		case 0:
			// If there are 0 buffers left, it means we used
			// all of the latest buffer added and all pages up to
			// that buffer. Since there is no on-chip end_addr
			// in this case, we used the saved one.

			physbuf = virt_to_phys(page_address(
				 c->kbuf->pages[c->kbuf_pagenbr - 1]));

			num_bytes = PAGE_SIZE * (c->kbuf_pagenbr - 1) + 
				(c->last_end_addr - physbuf);

			break;

		case 1:
			// If there is 1 buffer left, it means we used zero or
			// more but not all of the latest buffer added, and
			// all pages up to that buffer.

			physbuf = virt_to_phys(page_address(
				 c->kbuf->pages[c->kbuf_pagenbr - 1]));

			num_bytes = PAGE_SIZE * (c->kbuf_pagenbr - 1) + 
				(end_addr - physbuf);

			break;

		case 2:
			// If there are 2 buffers left, it means we used zero or
			// more of the next latest buffer added. Similar to the
			// case above, we also used all the pages leading up to
			// that buffer. 

			physbuf = virt_to_phys(page_address(
				 c->kbuf->pages[c->kbuf_pagenbr - 2]));
			
			num_bytes = PAGE_SIZE * (c->kbuf_pagenbr - 2) + 
				(end_addr - physbuf);

			break;
		default:
			BUG();
			num_bytes = 0;
			break;
	}
			
	// Now, subtract the ciobuf offset and the headersize since these
	// got included by the rough page-based calculation above.

	num_bytes -= c->headersize + c->kbuf->offset;

	local_irq_restore(flags);
	
	// Some sanity checking to catch errors as early as possible

	ASSERT(num_bytes >= 0);

	return num_bytes;
}

// Start the copying of a contiguous block of memory to a ciobuf. src is
// specified virtually. 
//
// XAHB bus and channel is selectable
//
// The given callback is called when the transfer is done. The channel has
// to be reset (not drained), important!
//

void 
dma_memcpy_to_ciobuf(struct ciobuf *dest, char *src, unsigned int size,
		     int cha, int use_xahb, 
		     dma_irq_callback callback)
{
        // stay safe

        if(size > dest->length) {
                printk("dma_memcpy_to_ciobuf: size %d > len %d !\n",
                       size, dest->length);
		if(callback)
			callback(cha);
                return;
        }

	DDEBUGC(printk("dma_memcpy %d bytes from 0x%p on cha %d\n",
		       size, src, cha));

	// Use 2^4-byte burst-sizes for efficiency on the MDEV
	// (does this work for non-XAHB?)
	dma_setup_mdev(cha, src, use_xahb, 4, DMA_DEV2MEM);

	dma_enable(cha, 1, 1);

	// Start the ciobuf-copying
	//
	// We use the tailsize argument to narrow the ciobuf copying
	// to size (otherwise it will try to copy to the entire kbuf range)

	dma_start_ciobuf(cha, dest, 0, dest->length - size, callback);
}


void 
dma_free(int cha)
{
	unsigned long flags;
	
	ASSERT((cha<ARGUS_NR_DMA)&&(cha>=0)&&(g_channel_allocation&(1<<cha)));
	
	DDEBUG(printk("dma_free %d\r\n", cha));
	
	local_irq_save(flags);
	// Safeguard - make sure a freed channel is actually in the disabled state
	dma_enable(cha, 0, 0);
	g_channel_allocation &= ~(1 << cha);
#ifdef CONFIG_ARGUS_3
	// See if we can turn off the XAHB clock again to save power
	if(channels[cha].xahb)
		extmem_clock_disable();
#endif
	local_irq_restore(flags);
}

// Show some channel status for debug purposes

void
dma_show_status(int cha)
{
	unsigned int f, c, s;

	if(cha < 0)
		return;  // not allocated

	printk("channel %d:\n", cha);
	f = SOC_DMA.CHANNEL[cha].CHACONFIG;
	c = SOC_DMA.CHANNEL[cha].CHACTRL;
	s = SOC_DMA.CHANNEL[cha].CHASTATUS;
	
	printk("   CHACONFIG 0x%x  CHACTRL 0x%x  CHASTATUS 0x%x\n", f, c, s);
	printk("CONF: port %d prio %d size %d dir %d mdev %d fifothres %d\n",
	       GET_DMA_CHACONFIG_PORT(f),
	       GET_DMA_CHACONFIG_PRIO(f),
	       GET_DMA_CHACONFIG_SIZE(f),
	       GET_DMA_CHACONFIG_DIR(f),
	       GET_DMA_CHACONFIG_MDEV(f),
	       GET_DMA_CHACONFIG_FIFOTHRES(f));
	printk("CTRL: deven %d bufen %d rst %d drain %d irq stat %d " 
	       "eof %d poll %d fifostall %d\n",
	       GET_DMA_CHACTRL_DEVEN(c),
	       GET_DMA_CHACTRL_BUFEN(c),
	       GET_DMA_CHACTRL_RESET(c),
	       GET_DMA_CHACTRL_DRAIN(c),
	       GET_DMA_CHACTRL_IRQ(c),
	       GET_DMA_CHACTRL_EOF(c),
	       GET_DMA_CHACTRL_POLL(c),
	       GET_DMA_CHACTRL_FIFOSTALL(c));
	printk("STAT: descpos %d, desccnt %d, bufbusy %d, devbusy %d, fifo size %d\n",
	       GET_DMA_CHASTATUS_DESCPOS(s),
	       GET_DMA_CHASTATUS_DESCCNT(s),
	       GET_DMA_CHASTATUS_BUFBUSY(s),
	       GET_DMA_CHASTATUS_DEVBUSY(s),
	       GET_DMA_CHASTATUS_FIFOFILL(s));

	printk("BUFADDR 0x%x, BUFSIZE %d, REMAINSIZE %d\n",
	       SOC_DMA.CHANNEL[cha].DESCQUEUE[0].DESC0,
	       GET_DMA_DESC1_BUFSIZE(SOC_DMA.CHANNEL[cha].DESCQUEUE[0].DESC1),
	       SOC_DMA.CHANNEL[cha].DESCQUEUE[0].DESC2);

	printk("MDEVADDR 0x%x, MDEVCONFIG 0x%x\n",
	       SOC_DMA.CHANNEL[cha].MDEVADDR,
	       SOC_DMA.CHANNEL[cha].MDEVCONFIG);
	       
}

EXPORT_SYMBOL(dma_enable);
EXPORT_SYMBOL(dma_alloc);
EXPORT_SYMBOL(dma_free);
EXPORT_SYMBOL(dma_show_status);
EXPORT_SYMBOL(dma_start_ciobuf);
EXPORT_SYMBOL(dma_memcpy_to_ciobuf);
