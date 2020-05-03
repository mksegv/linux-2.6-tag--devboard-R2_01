/*
 *  linux/arch/arm/mach-argus/arch.c
 *
 *  Architecture specific stuff.
 *
 *  Copyright (C) 1999-2005 Anoto Group AB and Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/tty.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/config.h>

#include <asm/elf.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/delay.h>

#include <asm/mach/arch.h>
#include <asm/arch/argus.h>
#include "boot_info.h"

// Different PLL's and clockspeeds in Argus-2 and Argus-3
#ifdef CONFIG_ARGUS_2
static const int pll_p[2][3] = {{ 4, 4, 4}, { 6, 6, 6}};
static const int pll_m[2][3] = {{22,19,22}, {28,25,28}};
static const int pll_s[2][3] = {{ 0, 0, 1}, { 0, 0, 1}};
static const int clkdivper[2][3] = {{96,80,32}, {98,85,33}};
#else
// NOTICE: for PLL_OSC_13 this table is just duplicated, it wont work. Dont use.
static const int pll_p[2][2] = {{ 1, 1}, { 1, 1}};
static const int pll_m[2][2] = {{67,76}, {67,76}};
static const int pll_s[2][2] = {{ 0, 0}, { 0, 0}};
static const int clkdivper[2][3] = {{68,80}, {68,80}};
#endif

#define PLL_OSC_12      0
#define PLL_OSC_13      1

#define PLL_A2_MODE_200    0
#define PLL_A2_MODE_175    1
#define PLL_A2_MODE_100    2

#define PLL_A3_MODE_150    0
#define PLL_A3_MODE_168    1


/********************************************************************************
 * Enables the pll. The pll output will be stable after 150 us, then PLL mode can 
 * be entered. 
 *
 * Argus2 table:
 *
 * input parameter osc should be set to 1 if a 13 MHz osc is used, and 0 for a 12 MHz Osc
 * mode param = (0..2) will give the following pll frequencys
 * 
 *  osc          mode   cpu-freq(cdiv=0)      FIN FOUT  PLL_P  PLL_M  PLL_S  CLKDIVPER
 *  0 (12 MHz)    0       ~200 MHz             12  384   4      22     0      96
 *  0 (12 MHz)    1       ~175 MHz             12  336   4      19     0      80  
 *  0 (12 MHz)    2       ~100 MHz             12  192   4      22     1      32
 *  1 (13 MHz)    0       ~200 MHz             13  390   6      28     0      98
 *  1 (13 MHz)    1       ~175 MHz             13  351   6      25     0      85
 *  1 (13 MHz)    2       ~100 MHz             13  195   6      28     1      33
 * 
 *  PLL:    FOUT = FIN*8*(M+2)/((P+2)*2^S))
 *
 *  For Argus-3, the PLL formula is:
 *        FOUT = FIN*(M+8)/((P+2)*2^S))
 *
 *  We always use a 12 mhz xtal. Some frequencies become:
 * 
 *  0             0        150 MHz             12 300    1      67     0      68
 *  0             1        168 MHz             12 336    1      76     0      80  breaks ETH
 *
 *  USB:    CLKDIVPER = (FOUT*16)/48 - 32
 */
static void 
clock_enable_pll(int osc, int mode)
{
	// Turn on PLL but do not use it
	SOC_CLK.PLL = SET_PLL_E(1)  |
                SET_PLL_P(pll_p[osc][mode])  |
                SET_PLL_S(pll_s[osc][mode])  |
                SET_PLL_M(pll_m[osc][mode]);

	// Set oscillator type in clock control register
	SOC_CLK.CTRL = CHG_CLK_13MHZ(SOC_CLK.CTRL, osc);
	
	// Prepare for generation of correct USB clock
	SOC_CLK.CLKDIVPER = clkdivper[osc][mode];
}

/********************************************************************************
 * Leaves PLL mode, and disables the pll. 
 */
static void 
clock_disable_pll(void)
{ 
	// Turn off PLL usage
	SOC_CLK.CTRL = CHG_CLK_HISPD(SOC_CLK.CTRL, 0);
	
	// Wait until PLL not actually used 
	while (GET_CLK_HISPD_STATUS(SOC_CLK.CTRL) == 1)
		;
	
	// Now, PLL generated clock is not used so it is safe to turn off PLL
	SOC_CLK.PLL = SET_PLL_E(0)  |
                SET_PLL_P(0)  |
                SET_PLL_S(0)  |
                SET_PLL_M(0);            
}
/********************************************************************************
 * Enable PLL-mode. (PLL should be enabled first)
 * (will be effective at next MHz-clock edge)
 */
static void 
clock_enable_use_pll(void)
{
	// Enable PLL usage 
	SOC_CLK.CTRL = CHG_CLK_HISPD(SOC_CLK.CTRL, 1); 
}                

/********************************************************************************
 * Returns info about which crysta is used
 * return value PLL_OSC_12 or PLL_OSC_12 (as defined in clock.h)
 */
static int 
clock_get_osc(void)
{
	if (GET_CLK_13MHZ(SOC_CLK.CTRL))
		return PLL_OSC_13;
	else
		return PLL_OSC_12;
}

/********************************************************************************
 * Leave PLL-mode. 
 * (will be effective at next MHz-clock edge)
 */
/********************************************************************************
 * set clock division factors for the frequency domains in ArgusII (set Argus II ref man)
 * cdiv - sets CPU freq.            cpu_freq   = FOUT/(cdiv+1)
 * hdiv - sets system frequency     sys_freq   = cpu_freq/(hdiv+1)
 * pdiv - sets peripheral frequency perip_freq = sys_freq/(pdiv+1)
 */

static void 
clock_set_dividers(int cdiv, int hdiv, int pdiv)
{
	int clk_ctrl = SOC_CLK.CTRL;
	SOC_CLK.CTRL = CHG_CLK_PDIV(CHG_CLK_HDIV(CHG_CLK_CDIV(clk_ctrl, 
							      cdiv), hdiv), pdiv);
}
/********************************************************************************
 * Easy-to-use all-in-one clock setup
 * Sets dividers and enables PLL, aware of PLL locking time
 * see clkEnabloePll for osc and mode definition
 * Replaces the good old enable_pll_and_wait
 */

static void __init 
argus_clock_setup()
{
	if (GET_CLK_HISPD_STATUS(SOC_CLK.CTRL) == 1)
		clock_disable_pll();
	
	// Set divider so that HCLK speed <= 100 MHz, PCLK speed <= 50 MHz
	clock_set_dividers(0, 1, 1);

#ifdef CONFIG_ARGUS_2
#ifdef CONFIG_ARGUS_PLL_LOW	
	clock_enable_pll(clock_get_osc(), PLL_A2_MODE_175);
#else
	clock_enable_pll(clock_get_osc(), PLL_A2_MODE_200);
#endif
#else
	clock_enable_pll(clock_get_osc(), PLL_A3_MODE_150);
#endif
	__delay(2000); // PLL has 150 us locking time
	clock_enable_use_pll();
	
	// Don't leave this function until PLL is actually used
	while (GET_CLK_HISPD_STATUS(SOC_CLK.CTRL) == 0)
		;

#ifdef CONFIG_ARGUS_3
	// Disable a couple of clocks we don't need 100% enabled on Argus-3

	SOC_CLK.DISABLE = CLK_CPU | CLK_AVP | CLK_SSI | CLK_AVE | CLK_CLKGEN1 | CLK_USB | CLK_EXTMEM;
#endif
}

void 
l2cache_FlushAll(void)
{
	SOC_L2CACHE.FLUSH_ALL = 0;
}

void 
l2cache_CleanAll(void)
{
	SOC_L2CACHE.CLEAN_ALL = 0;
}

static void 
l2cache_init(int nRams, int nIxRed)
{
	if ( !GET_L2CACHE_MODE_BYPASS(SOC_L2CACHE.MODE) )
		// Clean the cache if already enabled
		l2cache_CleanAll();
	
	l2cache_FlushAll();

	// To disable the L2cache, use IFILL/DFILL = 0 and BYPASS = 1

	SOC_L2CACHE.MODE =
		SET_L2CACHE_MODE_IXRED(nIxRed) | 
		SET_L2CACHE_MODE_ASSRED(4-nRams) |  
		SET_L2CACHE_MODE_IFILL(1) | 
		SET_L2CACHE_MODE_LOCK(0) |
		SET_L2CACHE_MODE_WB(1) |    
		SET_L2CACHE_MODE_BYPASS(0) |
		SET_L2CACHE_MODE_DFILL(1);
}

extern unsigned long romfs_start, romfs_length, romfs_in_flash;

// The fixup routine is called at boot, here we could "fix things up" 
// that need to be done before anything else. 

extern struct bootinfo_header argus_boot_header;

static void __init
argus_fixup(struct machine_desc *desc, struct tag *tags,
	    char **cmdline, struct meminfo *mi)
{
	int i;

	// Setup parameters for the video RAM allocator (size)
	//vram_init_parameters();

	// Initialize GPIOs
	for (i = 0; i < NUM_GPIO; i++) {
		if (argus_boot_header.gpio_config[i] != 0xff)
			SOC_GPIO.PIN[i] = argus_boot_header.gpio_config[i];
	}

#ifdef CONFIG_ARGUS_L2CACHE_SMALL
	// Use a slightly smaller L2 cache. The freed memory can be used
	// by the image block instead. This is required to handle megapixel
	// images when analyzing, jpeg and filtering is enabled simulatenously.
       	l2cache_init(3,2);
#else
	// Full L2 cache size.
#ifdef CONFIG_ARGUS_3
	// 24 kB cache only
       	l2cache_init(3,0);  // Argus3
#else
       	l2cache_init(4,0);
#endif
#endif

#ifndef CONFIG_ARGUS_FPGA
	// This does not work on the Argus3 FPGA.
	argus_clock_setup();  // warp speed!
#endif
	init_argus_debug();

	if (SOC_RESET.REASON == RESET_REASON_WD)
		printk("Restart forced by watchdog\n");
}

extern void argus_map_io(void);
extern void __init argus_init_irq(void);
extern struct sys_timer argus_timer;

MACHINE_START(ARGUS, "Argus")
     MAINTAINER("Bjorn Wesen")
     BOOT_MEM(0xc0000000, 0x40000000, 0xe0000000)
     BOOT_PARAMS(0xc0003000)
     FIXUP(argus_fixup)
     MAPIO(argus_map_io)
     INITIRQ(argus_init_irq)
     .timer = &argus_timer,
MACHINE_END
