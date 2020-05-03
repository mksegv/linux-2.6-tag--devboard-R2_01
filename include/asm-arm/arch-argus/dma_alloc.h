/*
 *  Copyright (C) 1999-2004 Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DMA_ALLOC_H__
#define __DMA_ALLOC_H__

#include <asm/arch/ciobuf.h>

// dma_alloc flags
#define DMA_ALLOC_FIFO32    (0)
#define DMA_ALLOC_FIFO64    (1)
#define DMA_ALLOC_FIFO128   (2)
#define DMA_ALLOC_FIFO256   (3)
// channel will use the XAHB bus
#define DMA_ALLOC_XAHB       16

// Used defined callback called on DMA interrupt. It is the
// responsibility of the callback to ACK the interrupt.
typedef void (*dma_irq_callback)(int cha);

// Allocs a dma channel. Returns the number of the DMA channel, or -1 if fail.
// * The callback is called on channel irqs. The callback may be 0. In
//   this case channel irqs will cause a system failure. (so if callback
//   is set to 0 then dma irq must not be used on that channel)
// * flags may be used to request special hardware fifo sizes. If there is 
//   no special requirement 0 should be used. If it is not possible to meet
//   the requirement, -1 is returned (always check for this error).

int dma_alloc(dma_irq_callback callback, int flags);

void dma_start_ciobuf( int cha, struct ciobuf *kbuf, int headersize, 
		       int tailsize, dma_irq_callback callback);

void dma_memcpy_to_ciobuf(struct ciobuf *dest, char *src, unsigned int size,
			  int cha, int use_xahb, 
			  dma_irq_callback callback);

unsigned int dma_final_size_ciobuf(int cha);

// Return the dma channel to the pool
void dma_free(int cha);

#endif
