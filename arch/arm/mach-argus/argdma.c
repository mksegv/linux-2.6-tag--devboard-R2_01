/*
 *  Copyright (C) 1999-2004 Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/system.h>
#include <asm/arch/argus2.h>
#include <asm/arch/argdma.h>

// Setup the DMA channel to a given port and priority.
// The channel will be disabled and irq is off. Irq mode is 0.
void 
dma_setup( int cha, int port, int busprio, int chaprio, int size, int dir)
{
	//// "0000 0010 0101 0101 0000 0000 0000 1101"
	//const unsigned int dir_lut = 0x0255000b;
	SOC_DMA.CONFIG[cha] = port | (busprio<<10) | (chaprio<<11) | 
		(dir<<12) | (size<<13);
	// The buffer queue and the fifo must be flushed whenever 
	// the port configuration changes.
	dma_flush_fifo(cha);
	dma_flush_buffer(cha);
}

// Enable/disable the DMA channel. It is possible to 
// enable the memory and device side of the internal dma
// FIFO independently.
void 
dma_enable( int cha, int memory, int device)
{
	SOC_DMA.CONFIG[cha] = (SOC_DMA.CONFIG[cha]&~0xc0) |
		(memory<<6) | (device<<7);
}

// Enable/disable the interrupt of a DMA channel. 
// Return previous setting.
int 
dma_irq_enable( int cha, int enable)
{
	int cfg = SOC_DMA.CONFIG[cha];
	SOC_DMA.CONFIG[cha] = (cfg&~0x100) | (enable<<8);
	return (cfg>>8)&1;
}

// Set IRQ mode
// 0 - buffer queue interrupt (normal case)
// 1 - memory to device transfer finish mode
void 
dma_irq_mode( int cha, int mode)
{
	int cfg = SOC_DMA.CONFIG[cha];
	SOC_DMA.CONFIG[cha] = (cfg&~(1<<16)) | (mode<<16);
}

// Add a physical memory buffer to the channel queue of buffers.
// The queue has room for up to two buffers.
void 
dma_add_buffer( int cha, unsigned int phys_addr, int size, int flags)
{
	//	printf("cha=%d, phys_addr=0x%x, size=0x%x, flags=%x\n", cha,phys_addr,size,flags); //Debug
	unsigned int irq;
	local_irq_save(irq);
	SOC_DMA.ARG0 = phys_addr;
	SOC_DMA.ARG1 = (flags<<24) | size;
	SOC_DMA.CMD = (cha<<4) | DMA_CMD_ADDBUF;
	while(SOC_DMA.CMD&7)
		;
	local_irq_restore(irq);
}

// Flush buffer queue
void
dma_flush_buffer( int cha)
{
	unsigned int irq;
	local_irq_save(irq);
	
	SOC_DMA.CMD = (cha<<4) | DMA_CMD_FLUSHBUF;
	while(SOC_DMA.CMD&7)
		;
	local_irq_restore(irq);
}

// Drain dma channel fifo.
// return 1 if successful drain
// return 0 if out of buffers in the buffer queue
int 
dma_drain( int cha)
{
	unsigned int irq, cfg;
	local_irq_save(irq);
	cfg = SOC_DMA.CONFIG[cha];
	// set drain mode and high busprio  
	SOC_DMA.CONFIG[cha] = cfg | (1<<9) | (1<<10);  
	// wait until fifo is empty or out of buffers
	while( (SOC_DMA.STATUS[cha]&3)==0)
		;
	// restore old configuration
	SOC_DMA.CONFIG[cha] = cfg;
	local_irq_restore(irq);
	return SOC_DMA.STATUS[cha]&1;		// STATUS_FIFOEMPTY
}

// Get buffer queue status.
// *phys_addr = current address pointer in buffer. The
// address is only valid if the buffer count is 1 or 2.
// Return number of buffers in the queue (0,1 or 2)
int 
dma_buffer_status( int cha, unsigned int *phys_addr)
{
	unsigned int irq;
	local_irq_save(irq);
	SOC_DMA.CMD = (cha<<4) | DMA_CMD_GETBUF;
	while(SOC_DMA.CMD&7)
		;
	if (phys_addr)
		*phys_addr = SOC_DMA.ARG0;
	local_irq_restore(irq);
	return SOC_DMA.ARG1&3;
}

// Return number of bytes currently in the channel fifo
int 
dma_fifo_status( int cha)
{
	unsigned int irq;
	local_irq_save(irq);
	
	SOC_DMA.CMD = (cha<<4) | DMA_CMD_GETFIFO;
	while(SOC_DMA.CMD&7)
		;
	local_irq_restore(irq);
	return SOC_DMA.ARG0&0xff;
}  

// Flush fifo
void 
dma_flush_fifo( int cha)
{
	unsigned int irq;
	local_irq_save(irq);
	
	SOC_DMA.CMD = (cha<<4) | DMA_CMD_FLUSHFIFO;
	while(SOC_DMA.CMD&7)
		;
	local_irq_restore(irq);
}


// Abort the dma transfer on the given channel.
// The fifo and the buffer queue is not flushed.
// This must be done using the dma_flush_fifo and
// the dma_flush_buffer commands.
// The channel irq is disabled.
void 
dma_abort( int cha)
{ 
	unsigned int irq;
	
	dma_enable( cha, 0, 0);  
	local_irq_save(irq);
	SOC_DMA.CMD = (cha<<4) | DMA_CMD_CLEARPIPE;
	while(SOC_DMA.CMD&7)
		;
	local_irq_restore(irq);
	dma_irq_enable( cha, 0);
}

// Returns the lowest dma channel generating interrupt.
// Returns -1 if no interrupt generated.
int 
dma_get_irq(void)
{
	int irq;
	irq = SOC_DMA.IRQ ;
	if((irq & 255) != 0)
		return ((irq >> 24) & 7);
	else
		return -1;
}

void 
dma_gendev_setup( int nbr, int flags)
{
	SOC_DMA.GENDEV[nbr] = flags;
}

// Don't enable the dma channel before this procedure.
void 
dma_gendev_transfer( int nbr, unsigned int phys_addr, int len)
{
	unsigned int irq;
	local_irq_save(irq);
	SOC_DMA.ARG0 = phys_addr;
	SOC_DMA.ARG1 = len;
	SOC_DMA.CMD = (nbr<<4) | DMA_CMD_SETGENDEV;
	while(SOC_DMA.CMD&7)
		;
	local_irq_restore(irq);
}

int 
dma_gendev_irqen( int nbr, int enable)
{
	int old = SOC_DMA.GENDEV[nbr];
	SOC_DMA.GENDEV[nbr] = (old&~(1<<9)) | (enable<<9);
	return (old>>9)&1;
}

int
dma_gendev_status( int nbr, unsigned int *phys_addr)
{
	unsigned int irq;
	local_irq_save(irq);
	SOC_DMA.CMD = (nbr<<4) | DMA_CMD_GETGENDEV;
	while(SOC_DMA.CMD&7)
		;
	if (phys_addr)
		*phys_addr = SOC_DMA.ARG0;
	local_irq_restore(irq);
	return SOC_DMA.ARG1;
}
