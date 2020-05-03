/*
 *  Copyright (C) 1999-2004 Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

// TODO: Check for adding irq locks

#include <linux/module.h>
#include <asm/system.h>
#include <asm/arch/argus3.h>
#include <asm/arch/argdma.h>

#define ASSERT(x) printk("Asserted...\n")

// MDEV data address for each port. 0 means FDEV or not available.
static const unsigned int MDEVADDR[] =
{
  0,                  // DMA_PORT_AVP_BAYER_OUT 0
  0,                  // DMA_PORT_AVP_Y         1
  0,                  // DMA_PORT_AVP_U         2
  0,                  // DMA_PORT_AVP_V         3
  0,                  // DMA_PORT_AVP_ANALYZE   4
  0,                  // DMA_PORT_AVP_BAYER_IN  5
  0,                  // DMA_PORT_ETHRX         6
  0,                  // DMA_PORT_ETHTX         7
  0,                  // N/A                    8
  0,                  // N/A                    9
  0,                  // N/A                   10
  0,                  // N/A                   11
  0,                  // N/A                   12
  0,                  // N/A                   13
  0,                  // N/A                   14
  0,                  // N/A                   15
  0x200d0040,         // DMA_PORT_USB_EP2_RD   16
  0x200d0044,         // DMA_PORT_USB_EP2_WR   17
  0x200d0048,         // DMA_PORT_USB_EP3_RD   18
  0x200d004c,         // DMA_PORT_USB_EP3_WR   19
  0x200b0000,         // DMA_PORT_UART0_RD     20
  0x200b0000,         // DMA_PORT_UART0_WR     21
  0x200c0000,         // DMA_PORT_UART1_RD     22
  0x200c0000,         // DMA_PORT_UART1_WR     23
  0x200f0008,         // DMA_PORT_SSI_IN       24
  0x200f0008,         // DMA_PORT_SSI_OUT      25
  0,                  // N/A                   26
  0,                  // N/A                   27
  0,                  // N/A                   28
  0,                  // N/A                   29
  0,                  // N/A                   30
  0                   // DMA_PORT_ALWAYS       31
};

static char g_busprio[10];
static char g_irqen[10];

// Setup the DMA channel to a given port and priority.
// The channel will be disabled and irq is off. Irq mode is 0.
void dma_setup( int cha, int port, int busprio, int chaprio, int size, int dir)
{
  int mdev=MDEVADDR[port]?1:0;
  g_busprio[cha]=busprio; // needed in addbuf
  g_irqen[cha]=0;
  
  SOC_DMA.CHANNEL[cha].CHACONFIG = 
    SET_DMA_CHACONFIG_PORT(port)              |
    SET_DMA_CHACONFIG_PRIO(chaprio)           |
    SET_DMA_CHACONFIG_SIZE(size)              |
    SET_DMA_CHACONFIG_DIR(dir)                |
    SET_DMA_CHACONFIG_MDEV(mdev)|
    SET_DMA_CHACONFIG_FIFOTHRES(1);
  
  if (mdev) {
    SOC_DMA.CHANNEL[cha].MDEVADDR = MDEVADDR[port];
    SOC_DMA.CHANNEL[cha].MDEVCONFIG = 
      SET_DMA_MDEVCONFIG_SIZE(0)      |
      SET_DMA_MDEVCONFIG_PROT(15)     |
      SET_DMA_MDEVCONFIG_BUSSEL(0)    |
      SET_DMA_MDEVCONFIG_PRIO(1)      |
      SET_DMA_MDEVCONFIG_INCRADDR(0)  |
      SET_DMA_MDEVCONFIG_USESIZE(0);
  }

  // Channel must be reset *after* CHACONFIG is set up
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_RESET(1);
  while(SOC_DMA.CHANNEL[cha].CHACTRL & SET_DMA_CHACTRL_RESET(1))
    ;
}

void 
dma_setup_mdev( int cha, void *mdevaddr,
		int use_xahb, int size, int dir)
{
	g_busprio[cha] = 0; // needed in addbuf
	g_irqen[cha] = 0;

	// Configure the source-side as an MDEV which always has words
	// to read

	SOC_DMA.CHANNEL[cha].CHACONFIG = 
		SET_DMA_CHACONFIG_PORT(DMA_PORT_ALWAYS)  |
		SET_DMA_CHACONFIG_PRIO(0)                |
		SET_DMA_CHACONFIG_SIZE(size)             |
		SET_DMA_CHACONFIG_DIR(dir)               |
		SET_DMA_CHACONFIG_MDEV(1)                |
		SET_DMA_CHACONFIG_FIFOTHRES(1);

	// Channel must be reset *after* CHACONFIG is set up to make
	// sure all states are correctly initialized

	SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_RESET(1);
	while(SOC_DMA.CHANNEL[cha].CHACTRL & SET_DMA_CHACTRL_RESET(1))
		;

	SOC_DMA.CHANNEL[cha].MDEVADDR = virt_to_phys(mdevaddr);

	// Auto-increment the address, and don't use the size
	// constraint mechanism because it only allows 16-bit
	// lengths. Let it run, and the destination side will
	// take care of the length control (side-effect: the source
	// might be read up to a FIFO-length too much).

	SOC_DMA.CHANNEL[cha].MDEVCONFIG = 
		SET_DMA_MDEVCONFIG_SIZE(0)           |
		SET_DMA_MDEVCONFIG_PROT(15)          |
		SET_DMA_MDEVCONFIG_BUSSEL(use_xahb)  |
		SET_DMA_MDEVCONFIG_PRIO(0)           |
		SET_DMA_MDEVCONFIG_INCRADDR(1)       |
		SET_DMA_MDEVCONFIG_USESIZE(0);
}

// Enable/disable the DMA channel. It is possible to 
// enable the memory and device side of the internal dma
// FIFO independently.
void dma_enable( int cha, int memory, int device)
{
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_PAUSE(1);
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_BUFEN(memory);
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_DEVEN(device);
}

// Enable/disable the interrupt of a DMA channel. 
// Return previous setting.
int dma_irq_enable( int cha, int enable)
{
  // No direct correspondance in Argus III. Save and use in dma_addbuf
  int prev=g_irqen[cha];
  g_irqen[cha]=enable;
  return prev;
}

// Set IRQ mode
// 0 - buffer queue interrupt (normal case)
// 1 - memory to device transfer finish mode
void dma_irq_mode( int cha, int mode)
{
  // No correspondance in Argus III - DMA
  ASSERT(0);
}

// Add a physical memory buffer to the channel queue of buffers.
// The queue has room for up to two buffers.
void dma_add_buffer( int cha, unsigned int phys_addr, int size, int flags)
{
  int insert_pos=GET_DMA_CHASTATUS_DESCPOS(SOC_DMA.CHANNEL[cha].CHASTATUS);
  // TODO: shouldn't flags be used for setting the PROT mask ?
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC0 = phys_addr;
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC1 = 
    SET_DMA_DESC1_BUFSIZE(0)     |
    SET_DMA_DESC1_PROT(15) |
    SET_DMA_DESC1_BUSSEL(flags & DMA_FLAGS_XAHB ? 1 : 0) |
    SET_DMA_DESC1_PRIO(g_busprio[cha]) |
    SET_DMA_DESC1_IRQ(g_irqen[cha]) |
    SET_DMA_DESC1_OPTION(DMA_DESC1_OPTION_REFILL) |
    SET_DMA_DESC1_EOF(0);
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC2 = size;
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC3 = 0;
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_ADDBUF(1);
}

void dma_add_buffer_stride( int cha, unsigned int phys_addr, int size, int flags, int skip, int refill)
{
  int insert_pos=GET_DMA_CHASTATUS_DESCPOS(SOC_DMA.CHANNEL[cha].CHASTATUS);
  // TODO: shouldn't flags be used for setting the PROT mask ?
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC0 = phys_addr;
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC1 = 
    SET_DMA_DESC1_BUFSIZE(refill)     |
    SET_DMA_DESC1_PROT(15) |
    SET_DMA_DESC1_BUSSEL(flags & DMA_FLAGS_XAHB ? 1 : 0) |
    SET_DMA_DESC1_PRIO(g_busprio[cha]) |
    SET_DMA_DESC1_IRQ(g_irqen[cha]) |
    SET_DMA_DESC1_OPTION(DMA_DESC1_OPTION_REFILL) |
    SET_DMA_DESC1_EOF(0);
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC2 = size;
  SOC_DMA.CHANNEL[cha].DESCQUEUE[insert_pos].DESC3 = 
	  SET_DMA_DESC3_REFILLSIZE(refill) |
	  SET_DMA_DESC3_SKIPSIZE(skip);
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_ADDBUF(1);
}

// Flush buffer queue
void dma_flush_buffer( int cha)
{
  // Reset channel. Not possible to do flush only.
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_RESET(1);
  while(SOC_DMA.CHANNEL[cha].CHACTRL & SET_DMA_CHACTRL_RESET(1))
    ;
}

// Drain dma channel fifo.
// return 1 if successful drain
// return 0 if out of buffers in the buffer queue
int dma_drain( int cha)
{
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_DRAIN(1);
  while((SOC_DMA.CHANNEL[cha].CHACTRL & SET_DMA_CHACTRL_DRAIN(1)) && 
        GET_DMA_CHASTATUS_DESCCNT(SOC_DMA.CHANNEL[cha].CHASTATUS)>0)
    ;
  return !(SOC_DMA.CHANNEL[cha].CHACTRL & SET_DMA_CHACTRL_DRAIN(1));
}

// Get buffer queue status.
// *phys_addr = current address pointer in buffer. The
// address is only valid if the buffer count is 1 or 2.
// Return number of buffers in the queue (0,1 or 2)
int dma_buffer_status( int cha, unsigned int *phys_addr)
{
  int status=SOC_DMA.CHANNEL[cha].CHASTATUS;
  int pos = GET_DMA_CHASTATUS_DESCPOS(status);
  int cnt = GET_DMA_CHASTATUS_DESCCNT(status);
  int cur = (2+pos-cnt)&1;
  if (phys_addr)
    *phys_addr=SOC_DMA.CHANNEL[cha].DESCQUEUE[cur].DESC0;
  return cnt;
}

// Return number of bytes currently in the channel fifo
int dma_fifo_status( int cha)
{
  int status=SOC_DMA.CHANNEL[cha].CHASTATUS;
  return GET_DMA_CHASTATUS_FIFOFILL(status);
}  

// Flush fifo
void dma_flush_fifo( int cha)
{
  // Reset channel. Not possible to do flush only.
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_RESET(1);
  while(SOC_DMA.CHANNEL[cha].CHACTRL & SET_DMA_CHACTRL_RESET(1))
    ;
}

// Abort the dma transfer on the given channel.
// The fifo and the buffer queue is not flushed.
// This must be done using the dma_flush_fifo and
// the dma_flush_buffer commands.
// The channel irq is disabled.
void dma_abort( int cha)
{ 
  // Reset channel. //TODO: maybe we would like to keep some of the state. Use PAUSE instead?
  SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_RESET(1);
  while(SOC_DMA.CHANNEL[cha].CHACTRL & SET_DMA_CHACTRL_RESET(1))
    ;
}

// Returns the lowest dma channel generating interrupt.
// Returns -1 if no interrupt generated.
int dma_get_irq(void)
{
	int irq = SOC_DMA.IRQ;
	if(irq)
		return GET_DMA_IRQ_CHANNEL(irq);
	else
		return -1;
}

EXPORT_SYMBOL(dma_add_buffer);
EXPORT_SYMBOL(dma_add_buffer_stride);
EXPORT_SYMBOL(dma_buffer_status);
EXPORT_SYMBOL(dma_setup);
EXPORT_SYMBOL(dma_drain);
EXPORT_SYMBOL(dma_abort);

