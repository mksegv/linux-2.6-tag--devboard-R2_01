/*
 * linux/include/asm-arm/arch-argus/time.h
 *
 *  Copyright (C) 2003, 2004 Axis Communications AB and Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/signal.h>
 
#include <asm/mach/time.h>
#include <asm/hardware.h>

#include <asm/arch/argus.h>

#ifdef CONFIG_ARGUS_FPGA
// Runs on FPGA and has another ratio between timers and host cycles
#define TICK_INTERVAL (2000*1000)
#else
#define TICK_INTERVAL (10*1000)
#endif
#ifdef CONFIG_ARGUS_WATCHOG_EXTENDED
#define WATCHDOG_INTERVAL 100*TICK_INTERVAL
#else
#define WATCHDOG_INTERVAL 10*TICK_INTERVAL
#endif

/*
 * We will be entered with IRQs enabled. (NO! SA_INTERRUPT handler...)
 *
 * Loop until we get ahead of the free running timer.
 * This ensures an exact clock tick count and time accuracy.
 * IRQs are disabled inside the loop to ensure coherence between
 * lost_ticks (updated in do_timer()) and the match reg value, so we
 * can use do_gettimeofday() from interrupt handlers.
 */

static irqreturn_t
argus_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int next_match;

	write_seqlock(&xtime_lock);

	do {
		timer_tick(regs);
		/* Clear match on timer 0 */
		SOC_TIMER.STATUS = (1<<0);
		/* Advance the match to the next tick */
		next_match = (SOC_TIMER.MATCH[0] += TICK_INTERVAL);
	} while ((signed long)(next_match - SOC_TIMER.COUNTER) <= 0);

#ifdef CONFIG_ARGUS_WATCHDOG
	// Restart watchdog timer
	SOC_TIMER.STATUS = (1<<3);
	SOC_TIMER.MATCH[3] = SOC_TIMER.COUNTER + WATCHDOG_INTERVAL;
#endif
        
	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

// IRQs are disabled before entering here from do_gettimeofday()
unsigned long 
argus_gettimeoffset(void)
{
        unsigned long ticks_to_match, elapsed;

        // Get ticks before next timer match
        ticks_to_match = SOC_TIMER.MATCH[0] - SOC_TIMER.COUNTER;

        // We need elapsed ticks since last match
        elapsed = TICK_INTERVAL - ticks_to_match;

	// Since the timer runs at 1 MHZ, the number of ticks equals the
	// number of usecs so we can return the value directly

        return elapsed;
}

static struct irqaction argus_timer_irq = {
	.name		= "Argus Timer Tick",
	.flags		= SA_INTERRUPT,
	.handler	= argus_timer_interrupt
};

static void __init
argus_timer_init(void)
{
        // Clear status of timer 0 and 3
	SOC_TIMER.STATUS = (1 << 0) | (1 << 3);
        
	// Trigger first match in 10 ms and first watchdog in 100ms
	SOC_TIMER.MATCH[0] = SOC_TIMER.COUNTER + TICK_INTERVAL;
	SOC_TIMER.MATCH[3] = SOC_TIMER.COUNTER + WATCHDOG_INTERVAL;

#ifdef CONFIG_ARGUS_WATCHDOG
	// Enable watchdog 
	SOC_RESET.WATCHDOG = 1;
	SOC_IRQ.MASK |= IRQ_TIMER3_MASK;
#endif
        
	// Register interrupt handler
	setup_irq(IRQ_TIMER0, &argus_timer_irq);
}

struct sys_timer argus_timer = {
	.init           = argus_timer_init,
	.suspend        = NULL,
	.resume         = NULL,
	.offset         = argus_gettimeoffset,
};
