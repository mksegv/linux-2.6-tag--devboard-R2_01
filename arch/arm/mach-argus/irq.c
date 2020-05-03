/*
 * linux/arch/arm/mach-argus/irq.c
 *
 *  Copyright (C) 2003-2004 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>    /* Pick up NR_IRQS */
#include <asm/arch/argus.h>

static void 
argus_mask_irq(unsigned int irq)
{
	SOC_IRQ.MASK &= ~(1 << irq);
}

static void 
argus_unmask_irq(unsigned int irq)
{
	SOC_IRQ.MASK |= (1 << irq);
}

static struct irqchip argus_normal_chip = {
        .ack            = argus_mask_irq,
        .mask           = argus_mask_irq,
        .unmask         = argus_unmask_irq,
};

void __init 
argus_init_irq(void)
{
	int irq;

	/* disable all IRQs */
	SOC_IRQ.MASK = 0;

	/* all IRQs are IRQ, not FIQ */
	SOC_IRQ.LEVEL = 0;

	for (irq = 0; irq < NR_IRQS; irq++) {
		set_irq_chip(irq, &argus_normal_chip);
		set_irq_flags(irq, IRQF_VALID);
		set_irq_handler(irq, do_level_IRQ);
	}

}
