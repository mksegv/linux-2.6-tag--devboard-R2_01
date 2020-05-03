/*
 *  Copyright (C) 2003,2004,2005 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARGDMA_H__
#define __ARGDMA_H__

#define DMA_FLAGS_XAHB       8    // use XAHB rather than AHB
#define DMA_FLAGS_RELOAD     4    // flags to dma_add_buffer
#define DMA_FLAGS_CACHEABLE  2    // flags to dma_add_buffer
#define DMA_FLAGS_BUFFERABLE 1    // flags to dma_add_buffer

#define DMA_GDFLAGS_RECOVER0    (0<<0)    // recover time 0 cycles
#define DMA_GDFLAGS_RECOVER4    (1<<0)    // recover time 4 cycles
#define DMA_GDFLAGS_RECOVER8    (2<<0)    // recover time 8 cycles
#define DMA_GDFLAGS_RECOVER12   (3<<0)    // recover time 12 cycles
#define DMA_GDFLAGS_POL         (1<<2)    // neg polarity on DREQ pin
#define DMA_GDFLAGS_CONT        (1<<3)    // force continues DREQ
#define DMA_GDFLAGS_BURST       (1<<4)    // always allow bursts
#define DMA_GDFLAGS_INCR        (1<<5)    // increment address
#define DMA_GDFLAGS_INFINIT     (1<<6)    // infinit transfer length (len does not matter, no irq)
#define DMA_GDFLAGS_BUFFERABLE  (1<<7)    // bufferable bit
#define DMA_GDFLAGS_CACHEABLE   (1<<8)    // cacheable bit


// Setup the DMA channel to a given port and priority.
// The channel will be disabled and irq will be turned off.
// The irq mode will be set to buffer queue interrupt.
// The fifo and the buffer queue will be flushed and the channel
// is initialized.
// cha  - channel (0-7)
// port - device port id
// busprio - 0 low, 1 high
// chaprio - 0 low, 1 high
// size    - device access size 0=byte, 1=short, 2=word, 3=illegal, 4=illegal, 5=4-burst
// dir     - 0 memory=>device, 1 device=>memory {DMA_MEM2DEV, DMA_DEV2MEM}
void dma_setup( int cha, int port, int busprio, int chaprio, int size, int dir);

void dma_setup_mdev( int cha, void *mdevaddr,
		     int use_xahb, int size, int dir);

// Enable/disable the DMA channel. It is possible to 
// enable the memory and device side of the dma
// FIFO independently.
void dma_enable( int cha, int memory, int device);

// Enable/disable the interrupt of a DMA channel. 
// Return previous state.
int dma_irq_enable( int cha, int enable);

// Set IRQ mode
// 0 - buffer queue interrupt (normal case)
// 1 - memory to device transfer finish mode
void dma_irq_mode( int cha, int mode);

// Add a physical memory buffer to the channel queue of buffers.
// The queue has room for up to two buffers.
void dma_add_buffer( int cha, unsigned int phys_addr, int size, int flags);
// Same thing but possibility to specify strides
void dma_add_buffer_stride( int cha, unsigned int phys_addr, int size, int flags,
			    int stride, int width);

// Flush buffer queue
void dma_flush_buffer( int cha);

// Drain channel fifo.
// return 1 if successful drain
// return 0 if out of buffers in the buffer queue
int dma_drain( int cha);

// Get buffer queue status.
// *phys_addr = current address pointer in buffer. The
// address is only valid if the buffer count is 1 or 2.
// Return number of buffers in the queue (0,1 or 2)
int dma_buffer_status( int cha, unsigned int *phys_addr);

// Return number of bytes currently in the channel fifo
int dma_fifo_status( int cha);

// Flush fifo
void dma_flush_fifo( int cha);

// Abort the dma transfer on the given channel.
// The fifo and the buffer queue is not flushed.
// This must be done using the dma_flush_fifo and
// the dma_flush_buffer commands.
void dma_abort( int cha);

// Returns the lowest channel generating interrupt.
// Returns -1 if no dma interrupt is generated.
int dma_get_irq(void);

#endif
