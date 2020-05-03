/*
 * linux/include/asm-arm/arch-argus/time.h
 *
 *  Copyright (C) 2003, 2004 Axis Communications AB and Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/arch/argus.h>
#include <linux/config.h>

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

// Run with interrupts disabled already. But to be on the safe side
// we have explicit IRQ disabling anyway...

static void 
argus_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned long flags;
	unsigned long didbump = 0;
	unsigned long timer, time = SOC_TIMER.MATCH[0] + TICK_INTERVAL;

	// Ack the timer irq and restart the timer for another 10 ms

	SOC_TIMER.STATUS = (1<<0);

	local_irq_save(flags);

	SOC_TIMER.MATCH[0] = time;
  
	// Already elapsed?
	timer = SOC_TIMER.COUNTER;
	
	while ((int) (timer - time) >= 0) {
		// The bump value needs to be large enough to avoid
		// a scenario where the CPU can't catch up with itself.
		didbump++;
		time = timer + 200;
		SOC_TIMER.MATCH[0] = time;
		timer = SOC_TIMER.COUNTER;
	}
	local_irq_restore(flags);

#ifdef CONFIG_ARGUS_WATCHDOG
	// Restart watchdog timer
	SOC_TIMER.STATUS = (1<<3);
	SOC_TIMER.MATCH[3] = SOC_TIMER.COUNTER + WATCHDOG_INTERVAL;
#endif
        
	do_timer(regs);
	do_profile(regs);

	//if(didbump)
	//	printk("did bump %d times\n", didbump);
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

static void 
setup_timer(void)
{
	timer_irq.handler = argus_timer_interrupt;
	gettimeoffset = argus_gettimeoffset;

        // Clear status of timer 0 and 3
	SOC_TIMER.STATUS = 1 << 0;
	SOC_TIMER.STATUS = 1 << 3;
        
	// Trigger first match in 10 ms and first watchdog in 100ms
	SOC_TIMER.MATCH[0] = SOC_TIMER.COUNTER + TICK_INTERVAL;
	SOC_TIMER.MATCH[3] = SOC_TIMER.COUNTER + WATCHDOG_INTERVAL;

#ifdef CONFIG_ARGUS_WATCHDOG
	// Enable watchdog 
	SOC_RESET.WATCHDOG = 1;
	SOC_IRQ.MASK |= IRQ_TIMER3_MASK;
#endif
        
	// Reister interrupt handler
	timer_irq.flags = SA_INTERRUPT; /* is this necessary? */
	setup_arm_irq(IRQ_TIMER0, &timer_irq);
}
