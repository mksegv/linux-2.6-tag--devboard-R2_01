/*
 * linux/arch/arm/mach-argus/leds.c
 *
 *  Copyright (C) 2003 Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/timer.h>

#include <asm/leds.h>
#include <asm/mach-types.h>

#include <asm/arch/argus2.h>
#include <asm/arch/gpio.h>

static struct timer_list kitt;

static int row2map[] = { 15, 16, 17, 25, 26, 63 };

static void 
argus_flash_leds(unsigned long y)
{
	static int nbr = 0;
	static int dir = 1;

	olympus2_led(9 + nbr, dir == 1 ? 0 : 1);
	olympus2_led(row2map[5-nbr], dir == 1 ? 1 : 0);

	nbr += dir;
	if(nbr == 6) {
		nbr = 5;
		dir = -dir;
	} else if(nbr == -1) {
		nbr = 0;
		dir = -dir;
	}
		
	mod_timer(&kitt, jiffies + 6);
}


static int __init
argus_leds_init(void)
{
	kitt.function = argus_flash_leds;
	init_timer(&kitt);
	mod_timer(&kitt, jiffies + 5);

	gpio_set_mode(GPIO_SSI_TX, GPIO_MODE_OUT, 0);  // RESET low

	// set_mode has to be done first for now since it mucks around
	// with the other fields. fix this. TODO
	gpio_set_mode(GPIO_SSI_FRM, GPIO_MODE_IN_IRQ, 0);  // IRQ 
	gpio_set_edge_detect_polarity(GPIO_SSI_FRM, 0);
	gpio_clear_trig(GPIO_SSI_FRM);

	gpio_set_mode(GPIO_nCS2, GPIO_MODE_ALT_FUNC, 0);  // CS2 used as chip select

	gpio_set_mode(GPIO_nWAIT, GPIO_MODE_ALT_FUNC, 0);  // WAIT needed
	
	SOC_EXTMEM.CONFIG[2] = SET_EXTCONFIG_WIDTH(1) |
		SET_EXTCONFIG_WAIT(4) | SET_EXTCONFIG_EXTWAIT(1);

#if 0
	printk("trying to detect ne2000\n");
	{
/*  Register accessed at EN_CMD, the 8390 base addr.  */
#define E8390_STOP	0x01	/* Stop and reset the chip */
#define E8390_START	0x02	/* Start the chip, clear reset */
#define E8390_TRANS	0x04	/* Transmit a frame */
#define E8390_RREAD	0x08	/* Remote read */
#define E8390_RWRITE	0x10	/* Remote write  */
#define E8390_NODMA	0x20	/* Remote DMA */
#define E8390_PAGE0	0x00	/* Select page chip registers */
#define E8390_PAGE1	0x40	/* using the two high-order bits */
#define E8390_PAGE2	0x80	/* Page 3 is invalid. */
		volatile unsigned char *base = (unsigned char *)(EXT2_NOCACHE_V + 0x300);
		int i;
		base[0] = E8390_STOP | E8390_NODMA | E8390_PAGE1;
		for(i = 0; i < 6; i++)
			printk("phys addr %d: 0x%x\n", i, base[1+i]);
		for(i = 0; i < 6; i++)
			base[i+1] = 0x33+i;
		for(i = 0; i < 6; i++)
			printk("2nd phys addr %d: 0x%x\n", i, base[1+i]);
		
		base[0] = E8390_STOP | E8390_NODMA | E8390_PAGE0;
		
		for(i = 0; i < 0x1f; i++)
			printk("phys addr 0x%x: 0x%x\n", i, base[i]);

		printk("io reg read as 0x%x\n", base[0]);

		//base[0] = E8390_START | E8390_NODMA | E8390_PAGE0;
		printk("done.\n");
	}
#endif
	return 0;
}

__initcall(argus_leds_init);

void
olympus2_led(int nbr, int enable)
{
	// Exceptions for the above ALT_FUNC things... braindead.
	if(nbr == 17 || nbr == 15 || nbr == 63)
		return;
	gpio_set_mode(nbr, GPIO_MODE_OUT, enable);
}
