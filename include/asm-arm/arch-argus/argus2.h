/*
 *  Copyright (C) 1999-2004 Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARGUS2_H
#define ARGUS2_H

#ifdef PHYSICAL_ADDRESSING
#include <asm/arch/argus2_physaddr.h>
#else
#include <asm/arch/argus2_addr.h>
#endif

#define REG32 volatile unsigned int

//--------------------------------------------------------------------------------
// Use definitions below to access SOC hardware
//--------------------------------------------------------------------------------

#define SOC_RESET     (*(tRESET*)(APB_NOCACHE + 0x00000)   )    // Reset controller,
#define SOC_CLK       (*(tCLK*)(APB_NOCACHE + 0x10000)     )    // Clock controller,
#define SOC_IRQ       (*(tIRQ*)(APB_NOCACHE + 0x20000)     )    // IRQ controller
#define SOC_TIMER     (*(tTIMER*)(APB_NOCACHE + 0x30000)   )    // Timer,
#define SOC_GPIO      (*(tGPIO*)(APB_NOCACHE + 0x40000)    )    // General purpose I/O,
#define SOC_PWM       ( (tPWM*)(APB_NOCACHE + 0x50000)     )    // Pulse wave modulators (2),
#define SOC_IMAGE     (*(tIMAGE*)(APB_NOCACHE + 0x70000)   )    // Image block,
#define SOC_MONTY     (*(tMONTY*)(APB_NOCACHE + 0x80000)   )    // Montgomety multiplier,
#define SOC_DMA       (*(tDMA*)(APB_NOCACHE + 0x90000)     )    // DMA controller,
#define SOC_EXTMEM    (*(tEXTMEM*)(APB_NOCACHE + 0xa0000)  )    // External memory interface,
#define SOC_UART      ( (tUART*)(APB_NOCACHE + 0xb0000)    )    // UART's (2),
#define SOC_USB       (*(tUSB*)(APB_NOCACHE + 0xd0000)     )    // USB slave
#define SOC_RTC       (*(tRTC*)(APB_NOCACHE + 0xe0000)     )    // Real Time Clock
#define SOC_SSI       (*(tSSI*)(APB_NOCACHE + 0xf0000)     )    // Synchronous Serial Interface
#define SOC_JPEG      (*(tJPEG*)(APB_NOCACHE + 0x100000)   )    // JPEG compressor,
#define SOC_BT        (*(tBT*)(APB_NOCACHE + 0x110000)     )    // BT baseband,
#define SOC_CHIPID    (*(tCHIPID*)(APB_NOCACHE + 0x120000) )    // Chip ID,
#define SOC_CLKGEN    ( (tCLKGEN*)(APB_NOCACHE + 0x130000) )    // Clock generators (2),
#define SOC_MATCH     (*(tMATCH*)(APB_NOCACHE + 0x150000)  )    // Image Matcher
#define SOC_L2CACHE   (*(tL2CACHE*)(L2CACHE_CTRL_NOCACHE)  )    // L2-Cache Control Interface

//--------------------------------------------------------------------------------
// Type definitions of hardware interface structures
//--------------------------------------------------------------------------------

//-----------------------------
// Reset controller
//-----------------------------

typedef struct _tRESET
{
  REG32 SWRESET;
  REG32 WATCHDOG;
  REG32 REASON;
                #define RESET_REASON_HW         0
                #define RESET_REASON_SW         1
                #define RESET_REASON_WD         2
} tRESET;

//-----------------------------
// Clock controller
//-----------------------------

typedef struct _tCLK
{
  REG32 CTRL;                           // Clock Controller Control
    #define GET_CLK_HISPD(x) ((x>>0)&0x1)             // set for high speed (pll) mode
    #define CHG_CLK_HISPD(x,upd) (x&~0x1 | (upd<<0))
    #define SET_CLK_HISPD(upd) (upd<<0)
    #define GET_CLK_CDIV(x) ((x>>1)&0x3)              // CPU_CLK divider
    #define CHG_CLK_CDIV(x,upd) (x&~0x6 | (upd<<1))
    #define SET_CLK_CDIV(upd) (upd<<1)
    #define GET_CLK_HDIV(x) ((x>>3)&0x3)              // HCLK divider (PHYSCACHE,IMAGE,JPEG,MATCH,MONTY...)
    #define CHG_CLK_HDIV(x,upd) (x&~0x18 | (upd<<3))
    #define SET_CLK_HDIV(upd) (upd<<3)
    #define GET_CLK_PDIV(x) ((x>>5)&0x3)              // PCLK divider
    #define CHG_CLK_PDIV(x,upd) (x&~0x60 | (upd<<5))
    #define SET_CLK_PDIV(upd) (upd<<5)
    #define GET_CLK_13MHZ(x) ((x>>7)&0x1)             // set for 13 MHz oscilator, clear for 12 MHz.
    #define CHG_CLK_13MHZ(x,upd) (x&~0x80 | (upd<<7))
    #define SET_CLK_13MHZ(upd) (upd<<7)
    #define GET_CLK_OSCTIME(x) ((x>>8)&0x1f)          // settling time for the oscilator when leaving sleep mode (osc disabled),
                                                      // 0-17, 17 => ~10 ms, 16 => ~5 ms ...
    #define CHG_CLK_OSCTIME(x,upd) (x&~0x1f00 | (upd<<8))
    #define SET_CLK_OSCTIME(upd) (upd<<8)
    #define GET_CLK_EXT_OSC_CTRL(x) ((x>>13)&0x1)     // Mux OSCEN to BT_SYS_CLK_REQ pin
    #define SET_CLK_EXT_OSC_CTRL(upd) (upd<<13)
    #define GET_CLK_HISPD_STATUS(x) ((x>>16)&0x1) // read only bit, used for checking the status of a HISPD change

  REG32 PLL;                            // PLL Control
    #define GET_PLL_E(x) ((x>>0)&0x1)
    #define CHG_PLL_E(x,upd) (x&~0x1 | (upd<<0))
    #define SET_PLL_E(upd) (upd<<0)
    #define GET_PLL_P(x) ((x>>1)&0x3f)
    #define CHG_PLL_P(x,upd) (x&~0x7e | (upd<<1))
    #define SET_PLL_P(upd) (upd<<1)
    #define GET_PLL_S(x) ((x>>7)&0x3)
    #define CHG_PLL_S(x,upd) (x&~0x180 | (upd<<7))
    #define SET_PLL_S(upd) (upd<<7)
    #define GET_PLL_M(x) ((x>>9)&0x3f)
    #define CHG_PLL_M(x,upd) (x&~0x7e00 | (upd<<9))
    #define SET_PLL_M(upd) (upd<<9)
  REG32 ENABLE;                          // Clock Gating read and enable (equal to CCG)
  REG32 DISABLE;                         // Clock Gating Disable
    #define CLK_CPU      (1<<0)          // CPU Clock
    #define CLK_AHB      (1<<1)          // AHB Clock
    #define CLK_APB      (1<<2)          // APB Clock
    #define CLK_IMAGE    (1<<3)          // Image Processor Clock
    #define CLK_JPEG     (1<<4)          // JPEG Clock
    #define CLK_MONTY    (1<<5)          // Montgomery Multiplier Clock
    #define CLK_MATCH    (1<<6)          // Matcher Clock
    #define CLK_APB_CONT (1<<7)          // PCLK_CONT  
    #define CLK_OSC      (1<<8)          // Oscillator Enable
    #define CLK_UART0    (1<<9)          // UART 0 Clock
    #define CLK_UART1    (1<<10)         // UART 1 Clock
    #define CLK_CLKGEN0  (1<<11)         // Clock Generator 0 Clock
    #define CLK_CLKGEN1  (1<<12)         // Clock Generator 1 Clock
    #define CLK_SSI      (1<<13)         // Serial Synchronous Interface (SSI) Clock
    #define CLK_TLB      (1<<14)         // TLB Clock
    #define CLK_EXTMEM   (1<<15)         // External Memory Clock
    #define CLK_USB      (1<<16)         // Normal USB clock
    #define CLK_USB_TEST (1<<17)         // USB prod test mode. NOTE: Both CLK_USB and CLK_USB_TEST should not be enabled!!
    #define CLK_BT_OSC   (1<<18)         // BT clock from Nexus oscillator
    #define CLK_BT_SYS   (1<<19)         // BT clock from BT_SYS_CLK pin NOTE: Both CLK_BT_SYS and CLK_BT_OSC should not be enabled.
    #define CLK_DMA      (1<<20)         // DMA clock    
  REG32 CLKDIVPER;
  REG32 CLKREQ;
    #define SET_CLKREQ_ASYNC_DMA(upd) (upd<<0)
    #define GET_CLKREQ_ASYNC_DMA(x) ((x>>0)&0x1)
    #define SET_CLKREQ_ASYNC_GPIO(upd) (upd<<1)
    #define GET_CLKREQ_ASYNC_GPIO(x) ((x>>1)&0x1)
    #define SET_CLKREQ_ASYNC_USB(upd) (upd<<2)
    #define GET_CLKREQ_ASYNC_USB(x) ((x>>2)&0x1)
    #define SET_CLKREQ_ASYNC_RTC(upd) (upd<<3)
    #define GET_CLKREQ_ASYNC_RTC(x) ((x>>3)&0x1)
    #define SET_CLKREQ_CCLK(upd) (upd<<4)
    #define GET_CLKREQ_CCLK(x) ((x>>4)&0x1)
    #define SET_CLKREQ_PCLK_BT(upd) (upd<<5)
    #define GET_CLKREQ_PCLK_BT(x) ((x>>5)&0x1)
    #define SET_CLKREQ_PCLK_USB(upd) (upd<<6)
    #define GET_CLKREQ_PCLK_USB(x) ((x>>6)&0x1)
    #define SET_CLKREQ_ASYNC_BT(upd) (upd<<7)
    #define GET_CLKREQ_ASYNC_BT(x) ((x>>7)&0x1)
    #define SET_CLKREQ_TLBCLK(upd) (upd<<8)
    #define GET_CLKREQ_TLBCLK(x) ((x>>8)&0x1)
    #define SET_CLKREQ_EXTCLK(upd) (upd<<9)
    #define GET_CLKREQ_EXTCLK(x) ((x>>9)&0x1)
    #define SET_CLKREQ_PCLK_RESET(upd) (upd<<10)
    #define GET_CLKREQ_PCLK_RESET(x) ((x>>10)&0x1)
    #define SET_CLKREQ_PCLK_BRIDGE(upd) (upd<<11)
    #define GET_CLKREQ_PCLK_BRIDGE(x) ((x>>11)&0x1)
    #define SET_CLKREQ_PCLK_PWM0(upd) (upd<<12)
    #define GET_CLKREQ_PCLK_PWM0(x) ((x>>12)&0x1)
    #define SET_CLKREQ_PCLK_PWM1(upd) (upd<<13)
    #define GET_CLKREQ_PCLK_PWM1(x) ((x>>13)&0x1)
    #define SET_CLKREQ_PCLK_TIMER(upd) (upd<<14)
    #define GET_CLKREQ_PCLK_TIMER(x) ((x>>14)&0x1)
    #define SET_CLKREQ_PCLK_GPIO(upd) (upd<<15)
    #define GET_CLKREQ_PCLK_GPIO(x) ((x>>15)&0x1)
    #define SET_CLKREQ_PCLK_RTC(upd) (upd<<16)
    #define GET_CLKREQ_PCLK_RTC(x) ((x>>16)&0x1)
    #define SET_CLKREQ_L2CACHE(upd) (upd<<17)
    #define GET_CLKREQ_L2CACHE(x) ((x>>17)&0x1)
    #define SET_CLKREQ_AHB(upd) (upd<<18)
    #define GET_CLKREQ_AHB(x) ((x>>18)&0x1)
    #define SET_CLKREQ_DMA(upd) (upd<<19)
    #define GET_CLKREQ_DMA(x) ((x>>19)&0x1)
    #define SET_CLKREQ_SDRAM(upd) (upd<<20)
    #define GET_CLKREQ_SDRAM(x) ((x>>20)&0x1)
    #define SET_CLKREQ_APB(upd) (upd<<21)
    #define GET_CLKREQ_APB(x) ((x>>21)&0x1)
    #define SET_CLKREQ_OCRAM(upd) (upd<<22)
    #define GET_CLKREQ_OCRAM(x) ((x>>22)&0x1)
    #define SET_CLKREQ_AHB_LOWPOWER(upd) (upd<<23)
    #define GET_CLKREQ_AHB_LOWPOWER(x) ((x>>23)&0x1)
    #define SET_CLKREQ_ASYNC_EXTMEM(upd) (upd<<24)
    #define GET_CLKREQ_ASYNC_EXTMEM(x) ((x>>24)&0x1)

} tCLK;


//-----------------------------
// Interrupt controller
//-----------------------------

typedef struct _tIRQ
{
  REG32 IRQ_SRCS;     //IQIP;            // Irq Pending (after mask and level)
  REG32 MASK;         //IQMR;            // Mask Register
    #define IRQ_UART0_MASK    0x00000001
    #define IRQ_UART1_MASK    0x00000002
    #define IRQ_TIMER0_MASK   0x00000004
    #define IRQ_TIMER1_MASK   0x00000008
    #define IRQ_TIMER2_MASK   0x00000010
    #define IRQ_TIMER3_MASK   0x00000020
    #define IRQ_GPIO_IRQ_MASK 0x00000040
    #define IRQ_GPIO_FIQ_MASK 0x00000080
    #define IRQ_IMAGE_FE_MASK 0x00000100
    #define IRQ_IMAGE_LE_MASK 0x00000200
    #define IRQ_IMAGE_OF_MASK 0x00000400
    #define IRQ_JPEG_MASK     0x00000800
    #define IRQ_DMA_MASK      0x00001000
    #define IRQ_MATCH_MASK    0x00002000
    #define IRQ_USB_MASK      0x00004000
    #define IRQ_RTC_MASK      0x00008000
    #define IRQ_MONTY_MASK    0x00010000
    #define IRQ_BT_MASK       0x00020000
    #define IRQ_SSI_MASK      0x00040000
    #define IRQ_COMMRX_MASK   0x00080000
    #define IRQ_COMMTX_MASK   0x00100000

    #define IRQ_UART0    (0)
    #define IRQ_UART1    (1)
    #define IRQ_TIMER0   (2)
    #define IRQ_TIMER1   (3)
    #define IRQ_TIMER2   (4)
    #define IRQ_TIMER3   (5)
    #define IRQ_GPIO_IRQ (6)
    #define IRQ_GPIO_FIQ (7)
    #define IRQ_IMAGE_FE (8)
    #define IRQ_IMAGE_LE (9)
    #define IRQ_IMAGE_OF (10)
    #define IRQ_JPEG     (11)
    #define IRQ_DMA      (12)
    #define IRQ_MATCH    (13)
    #define IRQ_USB      (14)
    #define IRQ_RTC      (15)
    #define IRQ_MONTY    (16)
    #define IRQ_BT       (17)
    #define IRQ_SSI      (18)
    #define IRQ_COMMRX   (19)
    #define IRQ_COMMTX   (20)
  REG32 LEVEL;      //IQLR;              // Level Register 0=>IRQ, 1=>FIQ
  REG32 FIQ_SRCS;   //IQFP;              // Fiq Pending (after mask and level)
  REG32 SRCS;     //IQPR;              // Pending Register (before mask)
} tIRQ;


//-----------------------------
// Timer
//-----------------------------

typedef struct _tTIMER
{
  REG32 MATCH[4];                         // Match register 0-3
  REG32 COUNTER;                          // Free running clock (1MHz)
  REG32 STATUS;                           // Match status flags (bit 0-3)
} tTIMER;


//-----------------------------
// GPIO Controller
//-----------------------------

typedef struct _tGPIO
{
  #define NUM_GPIO 64    // Number of GPIO pins

  REG32 PIN[64];
    #define GET_GPIO_MODE(x) (((x)>>0)&0x7)
    #define SET_GPIO_MODE(upd) ((upd)<<0)
      #define GPIO_MODE_IN        0
      #define GPIO_MODE_IN_IRQ    1
      #define GPIO_MODE_IN_FIQ    2
      #define GPIO_MODE_IN_PU     3
      #define GPIO_MODE_IN_PU_IRQ 4
      #define GPIO_MODE_IN_PU_FIQ 5
      #define GPIO_MODE_OUT       6
      #define GPIO_MODE_ALT_FUNC  7

    #define GET_GPIO_LEVEL(x) (((x)>>3)&0x1)
    #define SET_GPIO_LEVEL(upd) ((upd)<<3)

    #define GET_GPIO_TRIG(x) (((x)>>4)&0x1)
    #define SET_GPIO_TRIG(upd) ((upd)<<4)

    #define GET_GPIO_EDP(x) (((x)>>5)&0x1)
    #define SET_GPIO_EDP(upd) ((upd)<<5)
  REG32 PRIO;
    #define GET_GPIO_PRIO_IRQ_NBR(x) (((x)>>0)&0x3f)
    #define GET_GPIO_PRIO_IRQ(x) (((x)>>6)&0x1)
    #define GET_GPIO_PRIO_FIQ_NBR(x) (((x)>>7)&0x3f)
    #define GET_GPIO_PRIO_FIQ(x) (((x)>>13)&0x1)
} tGPIO;


//-----------------------------
// Pulse Wave Modulators
//-----------------------------

typedef struct _tPWM
{
  REG32 PRESCALE;
    #define PWM_PRESCALE_1   0
    #define PWM_PRESCALE_16  1
    #define PWM_PRESCALE_256 2
    #define PWM_OFF 3
  REG32 FORCE_OUTPUT;
  REG32 PERIOD;
    #define SET_PWM_LO(x) ((x)&0xffff)
    #define GET_PWM_LO(x) ((x)&0xffff)
    #define SET_PWM_HI(x) (((x)&0xffff)<<16)
    #define GET_PWM_HI(x) (((x)>>16)&0xffff)
  REG32 COUNTER;
  REG32 _PAD[(0x10000-4*4)/4];
} tPWM;


//-----------------------------
// Image
//-----------------------------

typedef struct _tIMAGE
{
  REG32 ENABLE;    // offset 0x0
    #define GET_ENABLE_EN(x) (((x)>>0)&0x1)
    #define SET_ENABLE_EN(upd) ((upd)<<0)
  REG32 SYNC;    // offset 0x4
    #define GET_SYNC_10BIT(x) (((x)>>0)&0x1)
    #define SET_SYNC_10BIT(upd) ((upd)<<0)
    #define GET_SYNC_FST_EDGE(x) (((x)>>1)&0x1)
    #define SET_SYNC_FST_EDGE(upd) ((upd)<<1)
    #define GET_SYNC_CLK_EDGE(x) (((x)>>2)&0x1)
    #define SET_SYNC_CLK_EDGE(upd) ((upd)<<2)
    #define GET_SYNC_PVAL(x) (((x)>>3)&0x1)
    #define SET_SYNC_PVAL(upd) ((upd)<<3)
    #define GET_SYNC_DMAM(x) (((x)>>4)&0x1)
    #define SET_SYNC_DMAM(upd) ((upd)<<4)
    #define GET_SYNC_SWFST(x) (((x)>>5)&0x1)
    #define SET_SYNC_SWFST(upd) ((upd)<<5)
  REG32 CUT_SETUP;    // offset 0x8
    #define GET_CUT_SETUP_IMSIZE_X(x) (((x)>>0)&0xfff)
    #define SET_CUT_SETUP_IMSIZE_X(upd) ((upd)<<0)
    #define GET_CUT_SETUP_EN_FIX(x) (((x)>>12)&0x1)
    #define SET_CUT_SETUP_EN_FIX(upd) ((upd)<<12)
    #define GET_CUT_SETUP_FIX_BAYER(x) (((x)>>13)&0x1)
    #define SET_CUT_SETUP_FIX_BAYER(upd) ((upd)<<13)
    #define GET_CUT_SETUP_BASE_ADDR(x) (((x)>>14)&0xffff)
    #define SET_CUT_SETUP_BASE_ADDR(upd) ((upd)<<14)
  REG32 CUT_START;    // offset 0xc
    #define GET_CUT_START_X(x) (((x)>>0)&0xfff)
    #define SET_CUT_START_X(upd) ((upd)<<0)
    #define GET_CUT_START_Y(x) (((x)>>12)&0xfff)
    #define SET_CUT_START_Y(upd) ((upd)<<12)
  REG32 CUT_END;    // offset 0x10
    #define GET_CUT_END_X(x) (((x)>>0)&0xfff)
    #define SET_CUT_END_X(upd) ((upd)<<0)
    #define GET_CUT_END_Y(x) (((x)>>12)&0xfff)
    #define SET_CUT_END_Y(upd) ((upd)<<12)
  REG32 CUT_SUBS;    // offset 0x14
    #define GET_CUT_SUBS_EN_X(x) (((x)>>0)&0x1)
    #define SET_CUT_SUBS_EN_X(upd) ((upd)<<0)
    #define GET_CUT_SUBS_EN_Y(x) (((x)>>1)&0x1)
    #define SET_CUT_SUBS_EN_Y(upd) ((upd)<<1)
    #define GET_CUT_SUBS_EXCL_X(x) (((x)>>2)&0x1f)
    #define SET_CUT_SUBS_EXCL_X(upd) ((upd)<<2)
    #define GET_CUT_SUBS_INCL_X(x) (((x)>>7)&0x7)
    #define SET_CUT_SUBS_INCL_X(upd) ((upd)<<7)
    #define GET_CUT_SUBS_EXCL_Y(x) (((x)>>10)&0x1f)
    #define SET_CUT_SUBS_EXCL_Y(upd) ((upd)<<10)
    #define GET_CUT_SUBS_INCL_Y(x) (((x)>>15)&0x7)
    #define SET_CUT_SUBS_INCL_Y(upd) ((upd)<<15)
  REG32 THRES;    // offset 0x18
    #define GET_THRES_EN_TM(x) (((x)>>0)&0x1)
    #define SET_THRES_EN_TM(upd) ((upd)<<0)
    #define GET_THRES_SIZEX(x) (((x)>>1)&0xff)
    #define SET_THRES_SIZEX(upd) ((upd)<<1)
    #define GET_THRES_SIZEY(x) (((x)>>9)&0xff)
    #define SET_THRES_SIZEY(upd) ((upd)<<9)
    #define GET_THRES_INV(x) (((x)>>17)&0x1)
    #define SET_THRES_INV(upd) ((upd)<<17)
  REG32 THRES_IMMEM;    // offset 0x1c
    #define GET_THRES_IMMEM_BASE(x) (((x)>>0)&0xffff)
    #define SET_THRES_IMMEM_BASE(upd) ((upd)<<0)
  REG32 ANALYZE;    // offset 0x20
    #define GET_ANALYZE_EN(x) (((x)>>0)&0x1)
    #define SET_ANALYZE_EN(upd) ((upd)<<0)
    #define GET_ANALYZE_SIZEX(x) (((x)>>1)&0xff)
    #define SET_ANALYZE_SIZEX(upd) ((upd)<<1)
    #define GET_ANALYZE_SIZEY(x) (((x)>>9)&0xff)
    #define SET_ANALYZE_SIZEY(upd) ((upd)<<9)
    #define GET_ANALYZE_COLEN(x) (((x)>>17)&0x1)
    #define SET_ANALYZE_COLEN(upd) ((upd)<<17)
  REG32 ANALYZE_IMMEM0;    // offset 0x24
    #define GET_ANALYZE_IMMEM0_BASE(x) (((x)>>0)&0xffff)
    #define SET_ANALYZE_IMMEM0_BASE(upd) ((upd)<<0)
  REG32 ANALYZE_IMMEM1;    // offset 0x28
    #define GET_ANALYZE_IMMEM1_BASE(x) (((x)>>0)&0xffff)
    #define SET_ANALYZE_IMMEM1_BASE(upd) ((upd)<<0)
  REG32 BFILT_FILT1;    // offset 0x2c
    #define GET_BFILT_FILT1_1(x) (((x)>>0)&0x0)
    #define SET_BFILT_FILT1_1(upd) ((upd)<<0)
  REG32 BFILT_FILT2;    // offset 0x30
    #define GET_BFILT_FILT2_2(x) (((x)>>0)&0x0)
    #define SET_BFILT_FILT2_2(upd) ((upd)<<0)
  REG32 BFILT_AFILT1;    // offset 0x34
    #define GET_BFILT_AFILT1_1(x) (((x)>>0)&0x0)
    #define SET_BFILT_AFILT1_1(upd) ((upd)<<0)
  REG32 BFILT_AFILT2;    // offset 0x38
    #define GET_BFILT_AFILT2_2(x) (((x)>>0)&0x0)
    #define SET_BFILT_AFILT2_2(upd) ((upd)<<0)
  REG32 BFILT_LIMIT;    // offset 0x3c
    #define GET_BFILT_LIMIT_LO(x) (((x)>>0)&0x7f)
    #define SET_BFILT_LIMIT_LO(upd) ((upd)<<0)
    #define GET_BFILT_LIMIT_HI(x) (((x)>>7)&0x7f)
    #define SET_BFILT_LIMIT_HI(upd) ((upd)<<7)
  REG32 BFILT;    // offset 0x40
    #define GET_BFILT_EN(x) (((x)>>0)&0x1)
    #define SET_BFILT_EN(upd) ((upd)<<0)
  REG32 BFILT_IMMEM;    // offset 0x44
    #define GET_BFILT_IMMEM_BASE(x) (((x)>>0)&0xffff)
    #define SET_BFILT_IMMEM_BASE(upd) ((upd)<<0)
    #define GET_BFILT_IMMEM_END(x) (((x)>>16)&0xffff)
    #define SET_BFILT_IMMEM_END(upd) ((upd)<<16)
  REG32 SCALE;    // offset 0x48
    #define GET_SCALE_EN(x) (((x)>>0)&0x1)
    #define SET_SCALE_EN(upd) ((upd)<<0)
    #define GET_SCALE_X(x) (((x)>>1)&0x7)
    #define SET_SCALE_X(upd) ((upd)<<1)
    #define GET_SCALE_Y(x) (((x)>>4)&0x7)
    #define SET_SCALE_Y(upd) ((upd)<<4)
  REG32 SCALE_IMMEM;    // offset 0x4c
    #define GET_SCALE_IMMEM_BASE(x) (((x)>>0)&0xffff)
    #define SET_SCALE_IMMEM_BASE(upd) ((upd)<<0)
  REG32 LED1;    // offset 0x50
    #define GET_LED1_ACTL(x) (((x)>>0)&0x1)
    #define SET_LED1_ACTL(upd) ((upd)<<0)
    #define GET_LED1_ONT(x) (((x)>>1)&0xfff)
    #define SET_LED1_ONT(upd) ((upd)<<1)
    #define GET_LED1_ONP(x) (((x)>>13)&0xfff)
    #define SET_LED1_ONP(upd) ((upd)<<13)
  REG32 LED2;    // offset 0x54
    #define GET_LED2_EN(x) (((x)>>0)&0x1)
    #define SET_LED2_EN(upd) ((upd)<<0)
    #define GET_LED2_PRESCALE(x) (((x)>>1)&0xfff)
    #define SET_LED2_PRESCALE(upd) ((upd)<<1)
  REG32 RLE;    // offset 0x58
    #define GET_RLE_EN_GRAY(x) (((x)>>0)&0x1)
    #define SET_RLE_EN_GRAY(upd) ((upd)<<0)
    #define GET_RLE_EN_BIN(x) (((x)>>1)&0x1)
    #define SET_RLE_EN_BIN(upd) ((upd)<<1)
    #define GET_RLE_WV(x) (((x)>>2)&0xf)
    #define SET_RLE_WV(upd) ((upd)<<2)
  REG32 PIXMODE;    // offset 0x5c
    #define GET_PIXMODE_OUTMUX_INPUT(x) (((x)>>0)&0x7)
    #define SET_PIXMODE_OUTMUX_INPUT(upd) ((upd)<<0)
    #define PIXMODE_OUTMUX_INPUT_10BIT 0
    #define PIXMODE_OUTMUX_INPUT_COMPR 1
    #define PIXMODE_OUTMUX_INPUT_IMPROC 2
    #define PIXMODE_OUTMUX_INPUT_SCALE_RLE 3
    #define PIXMODE_OUTMUX_INPUT_SCALE 4
    #define PIXMODE_OUTMUX_INPUT_BINRLE 5
    #define GET_PIXMODE_OUTMUX_OUTPUT(x) (((x)>>3)&0x3)
    #define SET_PIXMODE_OUTMUX_OUTPUT(upd) ((upd)<<3)
    #define PIXMODE_OUTMUX_OUTPUT_DISABLE 0
    #define PIXMODE_OUTMUX_OUTPUT_DMA 1
    #define PIXMODE_OUTMUX_OUTPUT_JPEG 2
    #define GET_PIXMODE_THRESMUX(x) (((x)>>5)&0x3)
    #define SET_PIXMODE_THRESMUX(upd) ((upd)<<5)
    #define PIXMODE_THRESMUX_DISABLE 0
    #define PIXMODE_THRESMUX_COMPR 1
    #define PIXMODE_THRESMUX_GAMMA 2
    #define GET_PIXMODE_DMMUX(x) (((x)>>7)&0x1)
    #define SET_PIXMODE_DMMUX(upd) ((upd)<<7)
    #define PIXMODE_DMMUX_DISABLE 0
    #define PIXMODE_DMMUX_COMPR 1
  REG32 PERSP1;    // offset 0x60
    #define GET_PERSP1_EN(x) (((x)>>0)&0x1)
    #define SET_PERSP1_EN(upd) ((upd)<<0)
    #define GET_PERSP1_EDGE(x) (((x)>>1)&0x3ff)
    #define SET_PERSP1_EDGE(upd) ((upd)<<1)
    #define GET_PERSP1_FIRST(x) (((x)>>11)&0x3ff)
    #define SET_PERSP1_FIRST(upd) ((upd)<<11)
    #define GET_PERSP1_NORM(x) (((x)>>21)&0x3ff)
    #define SET_PERSP1_NORM(upd) ((upd)<<21)
  REG32 PERSP2;    // offset 0x64
    #define GET_PERSP2_LAST(x) (((x)>>0)&0x3ff)
    #define SET_PERSP2_LAST(upd) ((upd)<<0)
    #define GET_PERSP2_NBRN(x) (((x)>>10)&0x3ff)
    #define SET_PERSP2_NBRN(upd) ((upd)<<10)
    #define GET_PERSP2_ENDCNT(x) (((x)>>20)&0x3ff)
    #define SET_PERSP2_ENDCNT(upd) ((upd)<<20)
    #define GET_PERSP2_ENDSTATE(x) (((x)>>30)&0x3)
    #define SET_PERSP2_ENDSTATE(upd) ((upd)<<30)
  REG32 PERSP3;    // offset 0x68
    #define GET_PERSP3_ROWN(x) (((x)>>0)&0x3ff)
    #define SET_PERSP3_ROWN(upd) ((upd)<<0)
    #define GET_PERSP3_ROWF(x) (((x)>>10)&0x3ff)
    #define SET_PERSP3_ROWF(upd) ((upd)<<10)
    #define GET_PERSP3_ENDLINE(x) (((x)>>20)&0x3ff)
    #define SET_PERSP3_ENDLINE(upd) ((upd)<<20)
  REG32 SENSCLK;    // offset 0x6c
    #define GET_SENSCLK_EN(x) (((x)>>0)&0x1)
    #define SET_SENSCLK_EN(upd) ((upd)<<0)
    #define GET_SENSCLK_DLHC(x) (((x)>>1)&0x1)
    #define SET_SENSCLK_DLHC(upd) ((upd)<<1)
    #define GET_SENSCLK_LOW(x) (((x)>>2)&0xff)
    #define SET_SENSCLK_LOW(upd) ((upd)<<2)
    #define GET_SENSCLK_HIGH(x) (((x)>>10)&0xff)
    #define SET_SENSCLK_HIGH(upd) ((upd)<<10)
    #define GET_SENSCLK_COUNTER(x) (((x)>>18)&0xff)
    #define SET_SENSCLK_COUNTER(upd) ((upd)<<18)
  REG32 STATUS;    // offset 0x70
    #define GET_STATUS_IRQ_OF(x) (((x)>>0)&0x1)
    #define SET_STATUS_IRQ_OF(upd) ((upd)<<0)
    #define GET_STATUS_IRQ_FE(x) (((x)>>1)&0x1)
    #define SET_STATUS_IRQ_FE(upd) ((upd)<<1)
    #define GET_STATUS_IRQ_LE(x) (((x)>>2)&0x1)
    #define SET_STATUS_IRQ_LE(upd) ((upd)<<2)
    #define GET_STATUS_FST(x) (((x)>>3)&0x1)
    #define SET_STATUS_FST(upd) ((upd)<<3)
  REG32 COMPR1;    // offset 0x74
    #define GET_COMPR1_M1(x) (((x)>>0)&0x3ff)
    #define SET_COMPR1_M1(upd) ((upd)<<0)
    #define GET_COMPR1_K1(x) (((x)>>10)&0xff)
    #define SET_COMPR1_K1(upd) ((upd)<<10)
    #define GET_COMPR1_EN(x) (((x)>>18)&0x1)
    #define SET_COMPR1_EN(upd) ((upd)<<18)
  REG32 COMPR2;    // offset 0x78
    #define GET_COMPR2_M2(x) (((x)>>0)&0x3ff)
    #define SET_COMPR2_M2(upd) ((upd)<<0)
    #define GET_COMPR2_K2(x) (((x)>>10)&0xff)
    #define SET_COMPR2_K2(upd) ((upd)<<10)
  REG32 COMPR3;    // offset 0x7c
    #define GET_COMPR3_M3(x) (((x)>>0)&0x3ff)
    #define SET_COMPR3_M3(upd) ((upd)<<0)
    #define GET_COMPR3_K3(x) (((x)>>10)&0xff)
    #define SET_COMPR3_K3(upd) ((upd)<<10)
  REG32 COMPR4;    // offset 0x80
    #define GET_COMPR4_M4(x) (((x)>>0)&0x3ff)
    #define SET_COMPR4_M4(upd) ((upd)<<0)
    #define GET_COMPR4_K4(x) (((x)>>10)&0xff)
    #define SET_COMPR4_K4(upd) ((upd)<<10)
  REG32 DEMOSAIC;    // offset 0x84
    #define GET_DEMOSAIC_ENABLE(x) (((x)>>0)&0x1)
    #define SET_DEMOSAIC_ENABLE(upd) ((upd)<<0)
    #define GET_DEMOSAIC_RGBMODE(x) (((x)>>1)&0x1)
    #define SET_DEMOSAIC_RGBMODE(upd) ((upd)<<1)
    #define GET_DEMOSAIC_SIZE(x) (((x)>>2)&0xfff)
    #define SET_DEMOSAIC_SIZE(upd) ((upd)<<2)
  REG32 DEMOSAIC_IMMEM;    // offset 0x88
    #define GET_DEMOSAIC_IMMEM_BASE(x) (((x)>>0)&0xffff)
    #define SET_DEMOSAIC_IMMEM_BASE(upd) ((upd)<<0)
    #define GET_DEMOSAIC_IMMEM_END(x) (((x)>>16)&0xffff)
    #define SET_DEMOSAIC_IMMEM_END(upd) ((upd)<<16)
  REG32 COLOR_CORR1;    // offset 0x8c
    #define GET_COLOR_CORR1_X11(x) (((x)>>0)&0xff)
    #define SET_COLOR_CORR1_X11(upd) ((upd)<<0)
    #define GET_COLOR_CORR1_X12(x) (((x)>>8)&0xff)
    #define SET_COLOR_CORR1_X12(upd) ((upd)<<8)
    #define GET_COLOR_CORR1_X13(x) (((x)>>16)&0xff)
    #define SET_COLOR_CORR1_X13(upd) ((upd)<<16)
    #define GET_COLOR_CORR1_K1(x) (((x)>>24)&0xff)
    #define SET_COLOR_CORR1_K1(upd) ((upd)<<24)
  REG32 COLOR_CORR2;    // offset 0x90
    #define GET_COLOR_CORR2_X21(x) (((x)>>0)&0xff)
    #define SET_COLOR_CORR2_X21(upd) ((upd)<<0)
    #define GET_COLOR_CORR2_X22(x) (((x)>>8)&0xff)
    #define SET_COLOR_CORR2_X22(upd) ((upd)<<8)
    #define GET_COLOR_CORR2_X23(x) (((x)>>16)&0xff)
    #define SET_COLOR_CORR2_X23(upd) ((upd)<<16)
    #define GET_COLOR_CORR2_K2(x) (((x)>>24)&0xff)
    #define SET_COLOR_CORR2_K2(upd) ((upd)<<24)
  REG32 COLOR_CORR3;    // offset 0x94
    #define GET_COLOR_CORR3_X31(x) (((x)>>0)&0xff)
    #define SET_COLOR_CORR3_X31(upd) ((upd)<<0)
    #define GET_COLOR_CORR3_X32(x) (((x)>>8)&0xff)
    #define SET_COLOR_CORR3_X32(upd) ((upd)<<8)
    #define GET_COLOR_CORR3_X33(x) (((x)>>16)&0xff)
    #define SET_COLOR_CORR3_X33(upd) ((upd)<<16)
    #define GET_COLOR_CORR3_K3(x) (((x)>>24)&0xff)
    #define SET_COLOR_CORR3_K3(upd) ((upd)<<24)
  REG32 COLOR_CORR4;    // offset 0x98
    #define GET_COLOR_CORR4_ENABLE(x) (((x)>>0)&0x1)
    #define SET_COLOR_CORR4_ENABLE(upd) ((upd)<<0)
  REG32 COLOR_CONV;    // offset 0x9c
    #define GET_COLOR_CONV_ENABLE(x) (((x)>>0)&0x1)
    #define SET_COLOR_CONV_ENABLE(upd) ((upd)<<0)
  REG32 CONV_FILTER1;    // offset 0xa0
    #define GET_CONV_FILTER_C00(x) (((x)>>0)&0xff)
    #define SET_CONV_FILTER_C00(upd) ((upd)<<0)
    #define GET_CONV_FILTER_C10(x) (((x)>>8)&0xff)
    #define SET_CONV_FILTER_C10(upd) ((upd)<<8)
    #define GET_CONV_FILTER_C20(x) (((x)>>16)&0xff)
    #define SET_CONV_FILTER_C20(upd) ((upd)<<16)
    #define GET_CONV_FILTER_C30(x) (((x)>>24)&0xff)
    #define SET_CONV_FILTER_C30(upd) ((upd)<<24)
  REG32 CONV_FILTER2;    // offset 0xa4
    #define GET_CONV_FILTER_C40(x) (((x)>>0)&0xff)
    #define SET_CONV_FILTER_C40(upd) ((upd)<<0)
    #define GET_CONV_FILTER_C01(x) (((x)>>8)&0xff)
    #define SET_CONV_FILTER_C01(upd) ((upd)<<8)
    #define GET_CONV_FILTER_C11(x) (((x)>>16)&0xff)
    #define SET_CONV_FILTER_C11(upd) ((upd)<<16)
    #define GET_CONV_FILTER_C21(x) (((x)>>24)&0xff)
    #define SET_CONV_FILTER_C21(upd) ((upd)<<24)
  REG32 CONV_FILTER3;    // offset 0xa8
    #define GET_CONV_FILTER_C31(x) (((x)>>0)&0xff)
    #define SET_CONV_FILTER_C31(upd) ((upd)<<0)
    #define GET_CONV_FILTER_C41(x) (((x)>>8)&0xff)
    #define SET_CONV_FILTER_C41(upd) ((upd)<<8)
    #define GET_CONV_FILTER_C02(x) (((x)>>16)&0xff)
    #define SET_CONV_FILTER_C02(upd) ((upd)<<16)
    #define GET_CONV_FILTER_C12(x) (((x)>>24)&0xff)
    #define SET_CONV_FILTER_C12(upd) ((upd)<<24)
  REG32 CONV_FILTER4;    // offset 0xac
    #define GET_CONV_FILTER_C22(x) (((x)>>0)&0xff)
    #define SET_CONV_FILTER_C22(upd) ((upd)<<0)
    #define GET_CONV_FILTER_C32(x) (((x)>>8)&0xff)
    #define SET_CONV_FILTER_C32(upd) ((upd)<<8)
    #define GET_CONV_FILTER_C42(x) (((x)>>16)&0xff)
    #define SET_CONV_FILTER_C42(upd) ((upd)<<16)
    #define GET_CONV_FILTER_C03(x) (((x)>>24)&0xff)
    #define SET_CONV_FILTER_C03(upd) ((upd)<<24)
  REG32 CONV_FILTER5;    // offset 0xb0
    #define GET_CONV_FILTER_C13(x) (((x)>>0)&0xff)
    #define SET_CONV_FILTER_C13(upd) ((upd)<<0)
    #define GET_CONV_FILTER_C23(x) (((x)>>8)&0xff)
    #define SET_CONV_FILTER_C23(upd) ((upd)<<8)
    #define GET_CONV_FILTER_C33(x) (((x)>>16)&0xff)
    #define SET_CONV_FILTER_C33(upd) ((upd)<<16)
    #define GET_CONV_FILTER_C43(x) (((x)>>24)&0xff)
    #define SET_CONV_FILTER_C43(upd) ((upd)<<24)
  REG32 CONV_FILTER6;    // offset 0xb4
    #define GET_CONV_FILTER_C04(x) (((x)>>0)&0xff)
    #define SET_CONV_FILTER_C04(upd) ((upd)<<0)
    #define GET_CONV_FILTER_C14(x) (((x)>>8)&0xff)
    #define SET_CONV_FILTER_C14(upd) ((upd)<<8)
    #define GET_CONV_FILTER_C24(x) (((x)>>16)&0xff)
    #define SET_CONV_FILTER_C24(upd) ((upd)<<16)
    #define GET_CONV_FILTER_C34(x) (((x)>>24)&0xff)
    #define SET_CONV_FILTER_C34(upd) ((upd)<<24)
  REG32 CONV_FILTER7;    // offset 0xb8
    #define GET_CONV_FILTER_C44(x) (((x)>>0)&0xff)
    #define SET_CONV_FILTER_C44(upd) ((upd)<<0)
    #define GET_CONV_FILTER_ENABLE(x) (((x)>>8)&0x1)
    #define SET_CONV_FILTER_ENABLE(upd) ((upd)<<8)
    #define GET_CONV_FILTER_5X3_FILT(x) (((x)>>9)&0x1)
    #define SET_CONV_FILTER_5X3_FILT(upd) ((upd)<<9)
    #define GET_CONV_FILTER_SIZE(x) (((x)>>10)&0xfff)
    #define SET_CONV_FILTER_SIZE(upd) ((upd)<<10)
    #define GET_CONV_FILTER_SHIFT(x) (((x)>>22)&0x7)
    #define SET_CONV_FILTER_SHIFT(upd) ((upd)<<22)
  REG32 CONV_FILTER_IMMEM;    // offset 0xbc
    #define GET_CONV_FILTER_IMMEM_BASE(x) (((x)>>0)&0xffff)
    #define SET_CONV_FILTER_IMMEM_BASE(upd) ((upd)<<0)
    #define GET_CONV_FILTER_IMMEM_END(x) (((x)>>16)&0xffff)
    #define SET_CONV_FILTER_IMMEM_END(upd) ((upd)<<16)
  REG32 SUBS;    // offset 0xc0
    #define GET_SUBS_ENABLE(x) (((x)>>0)&0x1)
    #define SET_SUBS_ENABLE(upd) ((upd)<<0)
    #define GET_SUBS_X(x) (((x)>>1)&0x1)
    #define SET_SUBS_X(upd) ((upd)<<1)
    #define GET_SUBS_Y(x) (((x)>>2)&0x1)
    #define SET_SUBS_Y(upd) ((upd)<<2)
  REG32 SUBS_IMMEM;    // offset 0xc4
    #define GET_SUBS_IMMEM_BASE(x) (((x)>>0)&0xffff)
    #define SET_SUBS_IMMEM_BASE(upd) ((upd)<<0)
  REG32 SER;    // offset 0xc8
    #define GET_SER_MONO(x) (((x)>>0)&0x1)
    #define SET_SER_MONO(upd) ((upd)<<0)
    #define GET_SER_DELAY_C(x) (((x)>>1)&0x1)
    #define SET_SER_DELAY_C(upd) ((upd)<<1)
  REG32 SER_IMMEM;    // offset 0xcc
    #define GET_SER_IMMEM_BASE(x) (((x)>>0)&0xffff)
    #define SET_SER_IMMEM_BASE(upd) ((upd)<<0)
    #define GET_SER_IMMEM_END(x) (((x)>>16)&0xffff)
    #define SET_SER_IMMEM_END(upd) ((upd)<<16)
  REG32 GAMMA1;    // offset 0xd0
    #define GET_GAMMA1_START0(x) (((x)>>0)&0xff)
    #define SET_GAMMA1_START0(upd) ((upd)<<0)
    #define GET_GAMMA1_START1(x) (((x)>>8)&0xff)
    #define SET_GAMMA1_START1(upd) ((upd)<<8)
    #define GET_GAMMA1_START2(x) (((x)>>16)&0xff)
    #define SET_GAMMA1_START2(upd) ((upd)<<16)
    #define GET_GAMMA1_START3(x) (((x)>>24)&0xff)
    #define SET_GAMMA1_START3(upd) ((upd)<<24)
  REG32 GAMMA2;    // offset 0xd4
    #define GET_GAMMA2_START4(x) (((x)>>0)&0xff)
    #define SET_GAMMA2_START4(upd) ((upd)<<0)
    #define GET_GAMMA2_START5(x) (((x)>>8)&0xff)
    #define SET_GAMMA2_START5(upd) ((upd)<<8)
    #define GET_GAMMA2_START6(x) (((x)>>16)&0xff)
    #define SET_GAMMA2_START6(upd) ((upd)<<16)
    #define GET_GAMMA2_START7(x) (((x)>>24)&0xff)
    #define SET_GAMMA2_START7(upd) ((upd)<<24)
  REG32 GAMMA3;    // offset 0xd8
    #define GET_GAMMA3_START8(x) (((x)>>0)&0xff)
    #define SET_GAMMA3_START8(upd) ((upd)<<0)
    #define GET_GAMMA3_SLOPE0(x) (((x)>>8)&0x7f)
    #define SET_GAMMA3_SLOPE0(upd) ((upd)<<8)
    #define GET_GAMMA3_SLOPE1(x) (((x)>>15)&0x7f)
    #define SET_GAMMA3_SLOPE1(upd) ((upd)<<15)
    #define GET_GAMMA3_SLOPE2(x) (((x)>>22)&0x7f)
    #define SET_GAMMA3_SLOPE2(upd) ((upd)<<22)
  REG32 GAMMA4;    // offset 0xdc
    #define GET_GAMMA4_SLOPE3(x) (((x)>>0)&0x7f)
    #define SET_GAMMA4_SLOPE3(upd) ((upd)<<0)
    #define GET_GAMMA4_SLOPE4(x) (((x)>>7)&0x7f)
    #define SET_GAMMA4_SLOPE4(upd) ((upd)<<7)
    #define GET_GAMMA4_SLOPE5(x) (((x)>>14)&0x7f)
    #define SET_GAMMA4_SLOPE5(upd) ((upd)<<14)
    #define GET_GAMMA4_SLOPE6(x) (((x)>>21)&0x7f)
    #define SET_GAMMA4_SLOPE6(upd) ((upd)<<21)
  REG32 GAMMA5;    // offset 0xe0
    #define GET_GAMMA5_SLOPE7(x) (((x)>>0)&0x7f)
    #define SET_GAMMA5_SLOPE7(upd) ((upd)<<0)
    #define GET_GAMMA5_SLOPE8(x) (((x)>>7)&0x7f)
    #define SET_GAMMA5_SLOPE8(upd) ((upd)<<7)
    #define GET_GAMMA5_ENABLE(x) (((x)>>14)&0x1)
    #define SET_GAMMA5_ENABLE(upd) ((upd)<<14)
} tIMAGE;


//-----------------------------
// Montgomery multiplier
//-----------------------------

typedef struct _tMONTY
{
  REG32 SETUP;
    #define SET_MONTY_PINV(x)       ((x)&0xffff)
    #define SET_MONTY_MSB_BASE(x)   (((x)&0xf)<<16)
  REG32 FACTORS;
    #define SET_MONTY_BASE_A(x)     ((x)&0x1ff)
    #define SET_MONTY_BASE_B(x)     (((x)&0x1ff)<<16)
  REG32 PRODUCTS;
    #define SET_MONTY_BASE_DST1(x)  ((x)&0x1ff)
    #define SET_MONTY_BASE_DST2(x)  (((x)&0x1ff)<<16)
  REG32 SIZE;
    #define SET_MONTY_BASE_MOD(x)   ((x)&0x1ff)
    #define SET_MONTY_SIZE(x)       (((x)&0x3f)<<16)
  REG32 STATUS;
    #define MONTY_CLEAR_IRQ         1
    #define GET_MONTY_IRQ(x)        ((x)&0x1)
    #define GET_MONTY_DEST_BUF(x)   (((x)&0x2)>>1)
    #define GET_MONTY_BUSY(x)       (((x)&0x4)>>2)
} tMONTY;


//-----------------------------
// DMA controller
//-----------------------------

typedef struct _tDMA
{
  REG32 CMD;
    #define DMA_CMD_ADDBUF    1
    #define DMA_CMD_GETBUF    2
    #define DMA_CMD_FLUSHBUF  3
    #define DMA_CMD_GETFIFO   4
    #define DMA_CMD_FLUSHFIFO 5
    #define DMA_CMD_CLEARPIPE 6
    #define DMA_CMD_SETGENDEV 8
    #define DMA_CMD_GETGENDEV 9

  REG32 ARG0;
  REG32 ARG1;
  REG32 IRQ;
        #define GET_DMA_IRQ(x) (1<<(x))
  REG32 MAGIC;
  REG32 __pad0[11];
  REG32 CONFIG[8];
  REG32 __pad1[8];
  REG32 STATUS[8];
  REG32 __pad2[8];
  REG32 GENDEV[2];

} tDMA;

#define DMA_SIZE_BYTE         0
#define DMA_SIZE_SHORT        1
#define DMA_SIZE_WORD         2
                              
#define DMA_MEM2DEV           0
#define DMA_DEV2MEM           1
                              
#define DMA_PORT_IMAGE_OUT    0
#define DMA_PORT_IMAGE_IN     1
#define DMA_PORT_ANALYZE      2
#define DMA_PORT_JPEG_OUT     3
#define DMA_PORT_JPEG_IN      4
#define DMA_PORT_MATCH        5
#define DMA_PORT_USB_EP2_RD  16
#define DMA_PORT_USB_EP2_WR  17
#define DMA_PORT_USB_EP3_RD  18
#define DMA_PORT_USB_EP3_WR  19
#define DMA_PORT_UART0_RD    20
#define DMA_PORT_UART0_WR    21
#define DMA_PORT_UART1_RD    22
#define DMA_PORT_UART1_WR    23
#define DMA_PORT_SSI_IN      24
#define DMA_PORT_SSI_OUT     25
#define DMA_PORT_GENDEV0     28
#define DMA_PORT_GENDEV1     29

//-----------------------------
// L2-Cache Control Interface
//-----------------------------

typedef struct _tL2CACHE
{
  REG32 TYPE;         // 0x00: Cache type register
  REG32 MODE;         // 0x04: Mode
    #define SET_L2CACHE_MODE_ASSRED(x)  (((x)& 0x7)<<20) // Assocativity Reduction
    #define SET_L2CACHE_MODE_IXRED(x)   (((x)& 0x3)<<16) // Index Reduction
    #define SET_L2CACHE_MODE_IFILL(x)   (((x)& 0x1)<<12) // Instruction Cacheline Fill Enable
    #define SET_L2CACHE_MODE_LOCK(x)    (((x)& 0x1)<< 6) // Locking Mode
    #define SET_L2CACHE_MODE_BYPASS(x)  (((x)& 0x1)<< 4) // Bypass Mode
    #define SET_L2CACHE_MODE_WB(x)      (((x)& 0x1)<< 3) // Write Buffer Enable
    #define SET_L2CACHE_MODE_DFILL(x)   (((x)& 0x1)<< 2) // Data Cacheline Fill Enable 

    #define GET_L2CACHE_MODE_ASSRED(x)  (((x)>>20)& 0x7) // Assocativity Reduction
    #define GET_L2CACHE_MODE_IXRED(x)   (((x)>>16)& 0x3) // Index Reduction
    #define GET_L2CACHE_MODE_IFILL(x)   (((x)>>12)& 0x1) // Instruction Cacheline Fill Enable
    #define GET_L2CACHE_MODE_LOCK(x)    (((x)>> 6)& 0x1) // Locking Mode
    #define GET_L2CACHE_MODE_BYPASS(x)  (((x)>> 4)& 0x1) // Bypass Mode
    #define GET_L2CACHE_MODE_WB(x)      (((x)>> 3)& 0x1) // Write Buffer Enable
    #define GET_L2CACHE_MODE_DFILL(x)   (((x)>> 2)& 0x1) // Data Cacheline Fill Enable

  REG32 FLUSH_ALL;    // 0x08: Flush entire cache
  REG32 FLUSH_SINGLE; // 0x0c: Flush single cache line
  REG32 CLEAN_ALL;    // 0x10: Clean entire cache
  REG32 CLEAN_SINGLE; // 0x14: Clean single cache line
  REG32 DRAIN_WB;     // 0x18: Drain write buffer
  REG32 FLUSH_STREAM; // 0x1c: Flush stream
} tL2CACHE;

#define L2CACHE_BLOCK_SIZE  32

//-----------------------------
// External Memory Interface
//-----------------------------

typedef struct _tEXTMEM
{
  REG32 CONFIG[6];
      #define SET_EXTCONFIG_WIDTH(upd)   ((upd)<<0)   // 0=>byte, 1=>hword, 2=>word
      #define SET_EXTCONFIG_SETUP(upd)   ((upd)<<2)   // Setup cycles (0=>1 cycle etc)
      #define SET_EXTCONFIG_WAIT(upd)    ((upd)<<4)   // Waitstate cycles (0=>1 cycle etc)
      #define SET_EXTCONFIG_HOLD(upd)    ((upd)<<8)   // Hold cycles
      #define SET_EXTCONFIG_BWAIT(upd)   ((upd)<<10)  // burst waitstates
      #define SET_EXTCONFIG_BTYPE(upd)   ((upd)<<14)  // burst type (0=>no burst, 1=>wrap2, 2=>wrap4, 3=>wrap8, 4=>wrap16)
      #define SET_EXTCONFIG_TOFF(upd)    ((upd)<<17)  // Toff time=1+TOFF cycles
      #define SET_EXTCONFIG_WRBM(upd)    ((upd)<<19)  // 0=>normal, 1=>nBM[3:0] is write enable
      #define SET_EXTCONFIG_SLOW(upd)    ((upd)<<20)  // 0=>normal, 1=>slow mode (wait cycles *4)
      #define SET_EXTCONFIG_EXTWAIT(upd) ((upd)<<21)  // use nWAIT pin
      #define SET_EXTCONFIG_NREMODE(upd) ((upd)<<22)  // 0: normal mode, 1: nRE not active during HOLD cycles
  REG32 __pad[2];
  REG32 SDCTRL1;
      #define SET_EXTSDCTRL1_ENABLE(upd) ((upd)<<0)   // Enable sdram controler on nCS4
      #define SET_EXTSDCTRL1_BSHIFT(upd) ((upd)<<1)   // Bank address position
      #define SET_EXTSDCTRL1_BWIDTH(upd) ((upd)<<5)   // Numbre of bank address bits
      #define SET_EXTSDCTRL1_RSHIFT(upd) ((upd)<<6)   // Row address position
      #define SET_EXTSDCTRL1_RWIDTH(upd) ((upd)<<9)   // Number of row address bits

  REG32 SDCTRL2;
      #define SET_EXTSDCTRL2_TRASMIN(upd) ((upd)<<0)  // tRASMIN in sdclk cycles minus one    ACTIVATE  -> PRECHARGE
      #define SET_EXTSDCTRL2_TRASMAX(upd) ((upd)<<4)  // tRASMAX in 8us units                 ACTIVATE  -> PRECHARGE
      #define SET_EXTSDCTRL2_TRRD(upd) ((upd)<<10)    // tRRD in sdclk cycles minus one       ACTIVATE  -> ACTIVATE
      #define SET_EXTSDCTRL2_TRCD(upd) ((upd)<<12)    // tRCD in sdclk cycles minus one       ACTIVATE  -> READ/WRITE
      #define SET_EXTSDCTRL2_TRP(upd) ((upd)<<14)     // tRP in sdclk cycles minus one        PRECHARGE -> any
      #define SET_EXTSDCTRL2_TRC(upd) ((upd)<<16)     // tRC in sdclk cycles minus one        REFRESH   -> any
      #define SET_EXTSDCTRL2_TRDL(upd) ((upd)<<20)    // tRDL in sdclk cycles minus one       WRITE     -> PRECHARGE
      #define SET_EXTSDCTRL2_CASL(upd) ((upd)<<22)    // CAS latency, 0=>2, 1=>3
      #define SET_EXTSDCTRL2_CLKDIV(upd) ((upd)<<23)  // SDCLK speed. HCLK/(CLKDIV+1)
      #define SET_EXTSDCTRL2_SDCLKFORCE(upd) ((upd)<<25) // Force SDCLK enable

  REG32 SDRFSH;
      #define SET_EXTSDRFSH_ENABLE(upd)  ((upd)<<0)   // Refresh enable
      #define SET_EXTSDRFSH_SELF(upd)    ((upd)<<1)   // Self refresh
      #define SET_EXTSDRFSH_BURST(upd)   ((upd)<<2)   // Refresh burst length burst_len=2^BURST
      #define SET_EXTSDRFSH_PERIOD(upd)  ((upd)<<5)   // Refresh period is PERIOD+1 microsec

  REG32 SDMODE;
      #define SET_EXTSDMODE_ADDR(upd)  ((upd)<<0)     // SDRAM mode register access ADDR-pins
      #define SET_EXTSDMODE_BANK(upd)  ((upd)<<14)    // SDRAM mode register access BANK-pins

  REG32 XARB;
      #define SET_EXTXARB_ENABLE(upd)   ((upd)<<0)    // External arbitration enabled
      #define SET_EXTXARB_REQPOL(upd)   ((upd)<<1)    // Polarity of XREQ
      #define SET_EXTXARB_GRANTPOL(upd) ((upd)<<2)    // Polarity of XGRANT
      #define SET_EXTXARB_WIDTH(upd)    ((upd)<<3)    // Arbitrated data bus width (0,1,2)

  REG32 NANDCLEAR;                                    // Write-only. Clear ecc registers when written
  REG32 NANDECC0;                                     // Read-only. 24-bits smartmedia ECC for page0
  REG32 NANDECC1;                                     // Read-only. 24-bits smartmedia ECC for page1

} tEXTMEM;

// SDRAM address parameters
//
// +--------+---------------------+
// |        |  BANK ADDRESS BITS  |
// | BSHIFT +----------+----------+
// |        | BWIDTH=0 | BWIDTH=1 |
// +--------+----------+----------+
// |    0   |     8    |   9:8    |
// |    1   |     9    |  10:9    |
// |    2   |     10   |  11:10   |
// |    3   |     11   |  12:11   |
// |    4   |     12   |  13:12   |
// |    5   |     20   |  21:20   |
// |    6   |     21   |  22:21   |
// |    7   |     22   |  23:22   |
// |    8   |     23   |  24:23   |
// |    9   |     24   |  25:24   |
// |    10  |     25   |  26:25   |
// |    11  |     26   |  27:26   |
// +--------+----------+----------+
//
// +--------+-------------------------------------------+
// |        |               ROW ADDRESS BITS            |
// | RSHIFT +---------------------+----------+----------+
// |        | RWIDTH=0 | RWIDTH=1 | RWIDTH=2 | RWIDTH=3 |
// +--------+----------+----------+----------+----------+
// |    0   |    18:8  |    19:8  |    20:8  |    21:8  |
// |    1   |    19:9  |    20:9  |    21:9  |    22:9  |
// |    2   |    20:10 |    21:10 |    22:10 |    23:10 |
// |    3   |    21:11 |    22:11 |    23:11 |    24:11 |
// |    4   |    22:12 |    23:12 |    24:12 |    25:12 |
// |    5   |    23:13 |    24:13 |    25:13 |    26:13 |
// |    6   |    24:14 |    25:14 |    26:14 |    27:14 |
// +--------+----------+----------+----------+----------+
//
// Formulas:
//   d - sdram databus width (8=>0, 16=>1, 32=>2)
//   c - number of column address bits of the sdram
//   b - number of bank address bits of the sdram
//   r - number of row address bits of the sdram
//
// * SDRAM size is 2^(d+c+b+r) bytes
//
// * ROW|BANK|COL|OFFSET configuration (normal)
//     CONFIG.WIDTH   = d
//     SDCTRL1.BSHIFT = d+c-8
//     SDCTRL1.BWDITH = b-1
//     SDCTRL1.RSHIFT = d+c+b-8
//     SDCTRL1.RWIDTH = r-11
//
// * BANK|ROW|COL|OFFSET configuration (useful in mobile sdrams, where bank shutdown is possible)
//     CONFIG.WIDTH   = d
//     SDCTRL1.BSHIFT = d+c+r-15
//     SDCTRL1.BWDITH = b-1
//     SDCTRL1.RSHIFT = d+c-8
//     SDCTRL1.RWIDTH = r-11
//
#define CALC_SDCTRL1(d,c,b,r) ( SET_EXTSDCTRL1_ENABLE(1) |  \
                                SET_EXTSDCTRL1_BSHIFT((d)+(c)-8) |  \
                                SET_EXTSDCTRL1_BWIDTH((b)-1) |  \
                                SET_EXTSDCTRL1_RSHIFT((d)+(c)+(b)-8) |  \
                                SET_EXTSDCTRL1_RWIDTH((r)-11))



//-----------------------------
// UART
//-----------------------------

typedef struct _tUART
{
  REG32 DATA;
  REG32 ENABLE;
    #define GET_UART_RXEN(x) (((x)>>0)&0x1)        // Receiver enable, clear fifo when disabled
    #define SET_UART_RXEN(upd) ((upd)<<0)
    #define GET_UART_TXEN(x) (((x)>>1)&0x1)        // Transmitter enable, clear fifo when disabled
    #define SET_UART_TXEN(upd) ((upd)<<1)
  REG32 CTRL;
    #define GET_UART_MODE(x) (((x)>>0)&0x1)        // 0 = RS232, 1 = IrDA
    #define SET_UART_MODE(upd) ((upd)<<0)
    #define GET_UART_AUTOFLOW(x) (((x)>>1)&0x1)    // 1 = Automatic RTS/CTS on
    #define SET_UART_AUTOFLOW(upd) ((upd)<<1)
    #define GET_UART_TXPOL(x) (((x)>>2)&0x1)       // 1 = Invert TX
    #define SET_UART_TXPOL(upd) ((upd)<<2)
    #define GET_UART_RXPOL(x) (((x)>>3)&0x1)       // 1 = Invert RX
    #define SET_UART_RXPOL(upd) ((upd)<<3)
    #define GET_UART_STOPBITS(x) (((x)>>4)&0x1)    // 0 = One stopbit, 1 = Two stopbits
    #define SET_UART_STOPBITS(upd) ((upd)<<4)
    #define GET_UART_LOOPBACK(x) (((x)>>5)&0x1)    // 1 = Loopback
    #define SET_UART_LOOPBACK(upd) ((upd)<<5)
  REG32 BAUD;                                      // Baudrate
    #define GET_UART_BAUD(x) (((x)>>0)&0x3ff)
    #define SET_UART_BAUD(upd) ((upd)<<0)
    #define UART_BAUD_9600   0x0000005f
    #define UART_BAUD_115200 0x00000007
    #define UART_BAUD_230400 0x00000003
    #define UART_BAUD_460800 0x00000001
    #define UART_BAUD_921600 0x00000000
  REG32 IRQEN;                                      // Interrupt enable register
  REG32 STATUS;                                     // Status register
    #define GET_UART_TIMEOUT(x) (((x)>>0)&0x1)      // receive timeout
    #define SET_UART_TIMEOUT(upd) ((upd)<<0)
    #define GET_UART_RXHFULL(x) (((x)>>1)&0x1)      // 1: >= 4 bytes in receive fifo
    #define SET_UART_RXHFULL(upd) ((upd)<<1)
    #define GET_UART_RXNEMPTY(x) (((x)>>2)&0x1)     // 1: Receive fifo not empty
    #define SET_UART_RXNEMPTY(upd) ((upd)<<2)
    #define GET_UART_TXHEMPTY(x) (((x)>>3)&0x1)     // 1: <= 4 bytes in transmit fifo
    #define SET_UART_TXHEMPTY(upd) ((upd)<<3)
    #define GET_UART_TXEMPTY(x) (((x)>>4)&0x1)      // 1: Transmit fifo empty
    #define SET_UART_TXEMPTY(upd) ((upd)<<4)
    #define GET_UART_TXNFULL(x) (((x)>>5)&0x1)      // 1: Transmit fifo not full
    #define SET_UART_TXNFULL(upd) ((upd)<<5)
    #define GET_UART_TXIDLE(x) (((x)>>6)&0x1)       // 1: Uart is idle
    #define SET_UART_TXIDLE(upd) ((upd)<<6)
    #define GET_UART_OVERFLOW(x) (((x)>>7)&0x1)     // 1: Recieve fifo overflow
    #define SET_UART_OVERFLOW(upd) ((upd)<<7)
  REG32 CONFIG;                                     // Uart size register
    #define GET_UART_RXSTART(x) (((x)>>0)&0xf)      // receive window start
    #define SET_UART_RXSTART(upd) ((upd)<<0)
    #define GET_UART_RXEND(x) (((x)>>4)&0xf)        // receive window end
    #define SET_UART_RXEND(upd) ((upd)<<4)
    #define GET_UART_IRDATXLEN(x) (((x)>>8)&0x3)    // IrDA tx pulselength 00:1.67us, 01:3.33us, 10:4.83us, 11:19.50us
    #define SET_UART_IRDATXLEN(upd) ((upd)<<8)
    #define GET_UART_IRDARXLEN(x) (((x)>>10)&0xf)   // IrDA rx detect length
    #define SET_UART_IRDARXLEN(upd) ((upd)<<10)
  REG32 _PAD[(0x10000-7*4)/4];
} tUART;


//-----------------------------
// Standard Synchronous Interface
//-----------------------------

typedef struct _tSSI
{
  REG32 SSIA;
    #define GET_SSIA_TRIG(x) (((x)>>0)&0xffff)
    #define SET_SSIA_TRIG(upd) ((upd)<<0)
    #define GET_SSIA_PERIOD(x) (((x)>>16)&0xff)
    #define SET_SSIA_PERIOD(upd) ((upd)<<16)
    #define GET_SSIA_WIDTH(x) (((x)>>24)&0xf)
    #define SET_SSIA_WIDTH(upd) ((upd)<<24)
  REG32 SSIB;
    #define GET_SSIB_TXEN(x) (((x)>>0)&0x1)
    #define SET_SSIB_TXEN(upd) ((upd)<<0)
    #define GET_SSIB_RXEN(x) (((x)>>1)&0x1)
    #define SET_SSIB_RXEN(upd) ((upd)<<1)
    #define GET_SSIB_IFRM(x) (((x)>>2)&0x1)
    #define SET_SSIB_IFRM(upd) ((upd)<<2)
    #define GET_SSIB_ICLK0(x) (((x)>>3)&0x1)
    #define SET_SSIB_ICLK0(upd) ((upd)<<3)
    #define GET_SSIB_ICLK1(x) (((x)>>4)&0x1)
    #define SET_SSIB_ICLK1(upd) ((upd)<<4)
    #define GET_SSIB_FFRM(x) (((x)>>5)&0x1)
    #define SET_SSIB_FFRM(upd) ((upd)<<5)
    #define GET_SSIB_FCLK0(x) (((x)>>6)&0x1)
    #define SET_SSIB_FCLK0(upd) ((upd)<<6)
    #define GET_SSIB_FCLK1(x) (((x)>>7)&0x1)
    #define SET_SSIB_FCLK1(upd) ((upd)<<7)
    #define GET_SSIB_DFRM(x) (((x)>>8)&0x1)
    #define SET_SSIB_DFRM(upd) ((upd)<<8)
    #define GET_SSIB_DCLK0(x) (((x)>>9)&0x1)
    #define SET_SSIB_DCLK0(upd) ((upd)<<9)
    #define GET_SSIB_DCLK1(x) (((x)>>10)&0x1)
    #define SET_SSIB_DCLK1(upd) ((upd)<<10)
    #define GET_SSIB_LBM(x) (((x)>>11)&0x1)
    #define SET_SSIB_LBM(upd) ((upd)<<11)
    #define GET_SSIB_FINV(x) (((x)>>12)&0x1)
    #define SET_SSIB_FINV(upd) ((upd)<<12)
    #define GET_SSIB_DREQ_IN(x) (((x)>>13)&0x1)
    #define SET_SSIB_DREQ_IN(upd) ((upd)<<13)
    #define GET_SSIB_DREQ_OUT(x) (((x)>>14)&0x1)
    #define SET_SSIB_DREQ_OUT(upd) ((upd)<<14)
        REG32 DATA;
}       tSSI;


//-----------------------------
// JPEG Compressor
//-----------------------------

typedef struct _tJPEG
{
  REG32 QUANT[2][64];
  REG32 HUFF[384];
    #define HUFF_AC0_OFFSET   0
    #define HUFF_DC0_OFFSET   162
    #define HUFF_AC1_OFFSET   174
    #define HUFF_DC1_OFFSET   336
  REG32 BASE[3];
  REG32 WIDTH;
  REG32 HEIGHT;
  REG32 MCU_WIDTH;
  REG32 MCU_NUM;
  REG32 DC_RESTART;
  REG32 CTRL;
    #define SET_JPEG_QMAP(x)  (x&0x7)
    #define GET_JPEG_QMAP(x)  (x&0x7)
    #define SET_JPEG_HMAP(x)  ((x&0x7)<<3)
    #define GET_JPEG_HMAP(x)  ((x>>3)&0x7)
    #define SET_JPEG_SAMP(x)  ((x>>6)&0x3)
    #define GET_JPEG_SAMP(x)  ((x>>6)&0x3)
      #define JPEG_SAMP_444   0
      #define JPEG_SAMP_422   3
    #define SET_JPEG_MONO(x)  ((x&0x1)<<8)
    #define GET_JPEG_MONO(x)  ((x>>8)&0x1)
    #define SET_JPEG_DMA(x)   ((x&0x1)<<9)
    #define GET_JPEG_DMA(x)   ((x>>9)&0x1)
      #define JPEG_DMA_INTERN 0
      #define JPEG_DMA_EXTERN 1
  REG32 ENABLE;
} tJPEG;


//-----------------------------
// Bluetooth baseband
//-----------------------------

typedef struct _tBT
{
  REG32 TIME;           // offset 0x000
  REG32 ALARM;          // offset 0x004
  REG32 PDIFF;          // offset 0x008
  REG32 ACTIME;         // offset 0x00c
  REG32 ACCORR;         // offset 0x010
  REG32 ENDCMD;         // offset 0x014
  REG32 CURCMD;         // offset 0x018
  REG32 HEADER;         // offset 0x01c
  REG32 PAYLOAD;        // offset 0x020
  REG32 RFREG;          // offset 0x024
  REG32 IRQEN;          // offset 0x028
  REG32 IRQ;            // offset 0x02c
  REG32 CONFIG;         // offset 0x030
  REG32 RESET;          // offset 0x034
  REG32 TXERR;          // offset 0x038
  REG32 VERSION;        // offset 0x03c
  REG32 SETIRQ;         // offset 0x040
  REG32 TXPAYLOAD;      // offset 0x044
  REG32 DBGMODE;        // offset 0x048
  REG32 DBGADDR;        // offset 0x04c
  REG32 DBGTRIG;        // offset 0x050
  REG32 LPDRIFT;        // offset 0x054
  REG32 LPPERIOD;       // offset 0x058
  REG32 LPENABLE;       // offset 0x05c
  REG32 MAGIC;          // offset 0x060
  REG32 CFGSAMPLE;      // offset 0x064
  char __pad0[0x100-26*sizeof(int)];
  REG32 PIREG;          // offset 0x100
  char __pad1[0x100-1*sizeof(int)];
  REG32 CMDBUF[0x40];   // offset 0x200
  char __pad2[0x100];   // offset 0x300
  REG32 RAM[0x100];     // offset 0x400
} tBT;


//-----------------------------
// Chip ID
//-----------------------------

typedef struct _tCHIPID
{
  REG32 ID;
  REG32 READY;
} tCHIPID;


//-----------------------------
// Clock Generator
//-----------------------------

typedef struct _tCLKGEN
{
  REG32 CTRL;
    #define SET_CLKGEN_LO(x) ((x)&0xff)
    #define GET_CLKGEN_LO(x) ((x)&0xff)
    #define SET_CLKGEN_HI(x) (((x)&0xff)<<8)
    #define GET_CLKGEN_HI(x) (((x)>>8)&0xff)
  REG32 _PAD[(0x10000-1*4)/4];
} tCLKGEN;


//-----------------------------
// Image Matcher
//-----------------------------

typedef struct _tMATCH
{
  REG32 CMD;
    // NOTE: Nexus matcher programs are not binary compatible to Argus
    //       matcher programs. Nexus supports 16-bits addresses. Argus
    //       13-bits addresses.
                #define GET_MAT_CMD(x) (((x)>>29)&0x7)
                #define SET_MAT_CMD(upd) ((upd)<<29)
                #define MAT_CMD_A1 ((unsigned int)0x00000000)
                #define GET_MAT_A_ADDR(x) (((x)>>0)&0xffff)
                #define SET_MAT_A_ADDR(upd) ((upd)<<0)
                #define GET_MAT_A_INCR(x) (((x)>>16)&0x1f)
                #define SET_MAT_A_INCR(upd) ((upd)<<16)
                #define GET_MAT_A_COLS(x) (((x)>>21)&0x7)
                #define SET_MAT_A_COLS(upd) ((upd)<<21)
                #define MAT_CMD_A2 ((unsigned int)0x00000001)
                #define GET_MAT_A_LEFT(x) (((x)>>0)&0x3f)
                #define SET_MAT_A_LEFT(upd) ((upd)<<0)
                #define GET_MAT_A_RIGHT(x) (((x)>>6)&0x3f)
                #define SET_MAT_A_RIGHT(upd) ((upd)<<6)
                #define GET_MAT_A_PRE(x) (((x)>>12)&0x7)
                #define SET_MAT_A_PRE(upd) ((upd)<<12)
                #define GET_MAT_A_POST(x) (((x)>>15)&0x7)
                #define SET_MAT_A_POST(upd) ((upd)<<15)
                #define GET_MAT_A_POST_IM(x) (((x)>>18)&0x1)
                #define SET_MAT_A_POST_IM(upd) ((upd)<<18)
                #define GET_MAT_B_CNT(x) (((x)>>19)&0xff)
                #define SET_MAT_B_CNT(upd) ((upd)<<19)
                #define MAT_CMD_B ((unsigned int)0x00000002)
                #define GET_MAT_B_ADDR(x) (((x)>>0)&0xffff)
                #define SET_MAT_B_ADDR(upd) ((upd)<<0)
                #define GET_MAT_B_DECR(x) (((x)>>16)&0x1f)
                #define SET_MAT_B_DECR(upd) ((upd)<<16)
                #define GET_MAT_B_PRE(x) (((x)>>21)&0x7)
                #define SET_MAT_B_PRE(upd) ((upd)<<21)
                #define GET_MAT_B_POST(x) (((x)>>24)&0x7)
                #define SET_MAT_B_POST(upd) ((upd)<<24)
                #define MAT_CMD_SC ((unsigned int)0x00000003)
                #define GET_MAT_SC_ADDR(x) (((x)>>0)&0x7fff)
                #define SET_MAT_SC_ADDR(upd) ((upd)<<0)
                #define GET_MAT_SC_INCR(x) (((x)>>15)&0xff)
                #define SET_MAT_SC_INCR(upd) ((upd)<<15)
                #define GET_MAT_SC_IRQ_EN(x) (((x)>>23)&0x1)
                #define SET_MAT_SC_IRQ_EN(upd) ((upd)<<23)
                #define GET_MAT_SC_DREQ_EN(x) (((x)>>24)&0x1)
                #define SET_MAT_SC_DREQ_EN(upd) ((upd)<<24)
                #define MAT_CMD_ABORT ((unsigned int)0x00000007)
                #define GET_MAT_ABORT_IRQ_EN(x) (((x)>>0)&0x1)
                #define SET_MAT_ABORT_IRQ_EN(upd) ((upd)<<0)
                #define GET_MAT_ABORT_DREQ_EN(x) (((x)>>1)&0x1)
                #define SET_MAT_ABORT_DREQ_EN(upd) ((upd)<<1)

                #define MAT_CMD_R 29
                #define MAT_A_ADDR_R 0
                #define MAT_A_INCR_R 16
                #define MAT_A_COLS_R 21
                #define MAT_A_LEFT_R 0
                #define MAT_A_RIGHT_R 6
                #define MAT_A_PRE_R 12
                #define MAT_A_POST_R 15
                #define MAT_A_POST_IM_R 18
                #define MAT_B_CNT_R 19
                #define MAT_B_ADDR_R 0
                #define MAT_B_DECR_R 16
                #define MAT_B_PRE_R 21
                #define MAT_B_POST_R 24
                #define MAT_SC_ADDR_R 0
                #define MAT_SC_INCR_R 15
                #define MAT_SC_IRQ_EN_R 23
                #define MAT_SC_DREQ_EN_R 24
                #define MAT_ABORT_IRQ_EN_R 0
                #define MAT_ABORT_DREQ_EN_R 1
} tMATCH;


//-----------------------------
// USB slave
//-----------------------------

typedef struct _tUSB
{
  REG32 DEVICE_ADDRESS_ENABLE;
    #define SET_DEVICE_ADDRESS(x)       (x&0x7f)        // Device address 0-127
    #define GET_DEVICE_ADDRESS(x)       (x&0x7f)
    #define SET_FUNCTION_ENABLE(x)      ((x&0x1)<<7)    // Enable USB slave
    #define GET_FUNCTION_ENABLE(x)      ((x>>7)&0x1)
  REG32 ENDPOINT_ENABLE;                                // Enable generic/isochronous endpoints
    #define GET_BUSRESET_STATE(x)       (x&0x1)
    #define GET_SUSPENDED(x)            ((x>>1)&0x1)
    //#define SET_GEN_ISO_EP_ENABLE(x)    (x&0x1)
    //#define GET_GEN_ISO_EP_ENABLE(x)    (x&0x1)
  REG32 MODE_0;                                         // MODE byte 0
    #define SET_INTERRUPT_MODE(x)       ((x&0x1)<<3)    // IRQ on ERROR/NAK
    #define GET_INTERRUPT_MODE(x)       ((x>>3)&0x1)
    #define SET_ENDPOINT_MODE(x)        ((x&0x3)<<6)    // Endpoint configuration
    #define GET_ENDPOINT_MODE(x)        ((x>>6)&0x3)
  REG32 MODE_1;                                         // MODE byte 1
    #define SET_ISO_MODE(x)             ((x&0x3)<<4)    // Isochronous mode
    #define GET_ISO_MODE(x)             ((x>>4)&0x3)
    #define SET_SOF_ONLY_IRQ_MODE(x)    ((x&0x1)<<7)    // IRQ on SOF only
    #define GET_SOF_ONLY_IRQ_MODE(x)    ((x>>7)&0x1)
  REG32 SET_DMA_0;
    #define SET_DMA_EP2OUT_DEOT_ENABLE(x) ((x&0x1)<<0)
    #define GET_DMA_EP2OUT_DEOT_ENABLE(x) ((x>>0)&0x1)
    #define SET_DMA_EP2IN_DEOT_ENABLE(x)  ((x&0x1)<<1)
    #define GET_DMA_EP2IN_DEOT_ENABLE(x)  ((x>>1)&0x1)
    #define SET_DMA_EP2OUT_ENABLE(x)    ((x&0x1)<<2)
    #define GET_DMA_EP2OUT_ENABLE(x)    ((x>>2)&0x1)
    #define SET_DMA_EP2IN_ENABLE(x)     ((x&0x1)<<3)
    #define GET_DMA_EP2IN_ENABLE(x)     ((x>>3)&0x1)
    #define SET_SOF_IRQ_ENABLE(x)       ((x&0x1)<<5)
    #define GET_SOF_IRQ_ENABLE(x)       ((x>>5)&0x1)
    #define SET_EP2OUT_IRQ_ENABLE(x)    ((x&0x1)<<6)
    #define GET_EP2OUT_IRQ_ENABLE(x)    ((x>>6)&0x1)
    #define SET_EP2IN_IRQ_ENABLE(x)     ((x&0x1)<<7)
    #define GET_EP2IN_IRQ_ENABLE(x)     ((x>>7)&0x1)
  REG32 SET_DMA_1;
    #define SET_DMA_EP3OUT_DEOT_ENABLE(x) ((x&0x1)<<0)
    #define GET_DMA_EP3OUT_DEOT_ENABLE(x) ((x>>0)&0x1)
    #define SET_DMA_EP3IN_DEOT_ENABLE(x)  ((x&0x1)<<1)
    #define GET_DMA_EP3IN_DEOT_ENABLE(x)  ((x>>1)&0x1)
    #define SET_DMA_EP3OUT_ENABLE(x)    ((x&0x1)<<2)
    #define GET_DMA_EP3OUT_ENABLE(x)    ((x>>2)&0x1)
    #define SET_DMA_EP3IN_ENABLE(x)     ((x&0x1)<<3)
    #define GET_DMA_EP3IN_ENABLE(x)     ((x>>3)&0x1)
    #define SET_EP3OUT_IRQ_ENABLE(x)    ((x&0x1)<<6)
    #define GET_EP3OUT_IRQ_ENABLE(x)    ((x>>6)&0x1)
    #define SET_EP3IN_IRQ_ENABLE(x)     ((x&0x1)<<7)
    #define GET_EP3IN_IRQ_ENABLE(x)     ((x>>7)&0x1)
  REG32 INTERRUPT_STATUS_0;
    #define GET_EP0OUT_IRQ(x)           ((x>>0)&0x1)
    #define GET_EP0IN_IRQ(x)            ((x>>1)&0x1)
    #define GET_EP1OUT_IRQ(x)           ((x>>2)&0x1)
    #define GET_EP1IN_IRQ(x)            ((x>>3)&0x1)
    #define GET_EP2OUT_IRQ(x)           ((x>>4)&0x1)
    #define GET_EP2IN_IRQ(x)            ((x>>5)&0x1)
    #define GET_BUS_RESET_IRQ(x)        ((x>>6)&0x1)
    #define GET_SUSPEND_CHANGE_IRQ(x)   ((x>>7)&0x1)
  REG32 INTERRUPT_STATUS_1;
    #define GET_EP2OUT_DEOT_IRQ(x)      ((x>>0)&0x1)
    #define GET_EP2IN_DEOT_IRQ(x)       ((x>>1)&0x1)
    #define GET_EP3OUT_DEOT_IRQ(x)      ((x>>2)&0x1)
    #define GET_EP3IN_DEOT_IRQ(x)       ((x>>3)&0x1)
    #define GET_EP3OUT_IRQ(x)           ((x>>4)&0x1)
    #define GET_EP3IN_IRQ(x)            ((x>>5)&0x1)
    #define GET_SOF_IRQ(x)              ((x>>6)&0x1)

  REG32 READ_BUFFER;
  REG32 WRITE_BUFFER;
  REG32 ACK_SETUP;
  REG32 CLEAR_BUFFER;
  REG32 VALIDATE_BUFFER;
  REG32 SEND_RESUME;
  REG32 FRAME_NUMBER_LO;
  REG32 FRAME_NUMBER_HI;
    #define GET_FRAME_NUMBER_HI(x)      (x&0x7)
  REG32 DMA_EP2_OUT;
  REG32 DMA_EP2_IN;
  REG32 DMA_EP3_OUT;
  REG32 DMA_EP3_IN;
  REG32 TEST_REG;

  // Pad up to 0x007c
  //REG32 __PAD__[12];                  // offset 0x0050-0x007c
  // with TEST_REG
  REG32 __PAD__[11];                    // offset 0x0054-0x007c

  // from 0x0080 to 0x00fc
  struct ENDPOINT
  {
    REG32 SELECT_ENDPOINT;
      #define GET_FULL_EMPTY(x)         ((x>>0)&0x1)
      #define GET_STALL(x)              ((x>>1)&0x1)
    REG32 LAST_TRANSACTION_STATUS;
      #define GET_DATA_RXTX_SUCCESS(x)  ((x>>0)&0x1)
      #define GET_ERROR_CODE(x)         ((x>>1)&0xf)
      #define GET_SETUP_PACKET(x)       ((x>>5)&0x1)
      #define GET_DATA_01_PACKET(x)     ((x>>6)&0x1)
      #define GET_PREV_STAT_NOT_READ(x) ((x>>7)&0x1)
    REG32 ENDPOINT_STATUS;
      #define SET_STALL(x)              (x&0x1)
    REG32 __EP_PAD__;
  } EP[8];
} tUSB;

//-----------------------------
// Real Time Clock
//-----------------------------

typedef struct _tRTC
{
  REG32 TIME_HI;
  REG32 TIME_LO;
  REG32 MATCH_HI;
  REG32 MATCH_LO;
  REG32 PERIOD;
  REG32 IRQ;
  REG32 IRQ_EN;
  REG32 WAKEUP;
    #define SET_WAKEUP_EN(x)   ((x&0x1)<<0)    // WAKEUP enable bit
    #define SET_WAKEUP_POL(x)  ((x&0x1)<<1)    // WAKEUP polarity,
} tRTC;

//-----------------------------
// GPIO pin numbers
//-----------------------------

#define GPIO_ADDR23         0  // IN_PU    
#define GPIO_ADDR24         1  // IN_PU    
#define GPIO_nCS0           2  // ALT_FUNC 
#define GPIO_nCS1           3  // ALT_FUNC 
#define GPIO_nCS2           4  // ALT_FUNC 
#define GPIO_nCS3           5  // ALT_FUNC 
#define GPIO_nCS4           6  // ALT_FUNC 
#define GPIO_nCS5           7  // ALT_FUNC 
#define GPIO_DREQ0          8  // IN_PU    
#define GPIO_DREQ1          9  // IN_PU    
#define GPIO_XREQ           10 // IN_PU    
#define GPIO_XGRANT         11 // IN_PU    
#define GPIO_PWM0           12 // IN_PU    
#define GPIO_PWM1           13 // IN_PU    
#define GPIO_SSI_CLK        14 // IN_PU    
#define GPIO_SSI_FRM        15 // IN_PU    
#define GPIO_SSI_RX         16 // IN_PU    
#define GPIO_SSI_TX         17 // IN_PU    
#define GPIO_nRESET_OUT     18 // ALT_FUNC 
#define GPIO_UART0_RXD      19 // IN_PU    
#define GPIO_UART0_TXD      20 // IN_PU    
#define GPIO_UART0_CTS      21 // IN_PU    
#define GPIO_UART0_RTS      22 // IN_PU    
#define GPIO_UART1_RXD      23 // IN_PU    
#define GPIO_UART1_TXD      24 // IN_PU    
#define GPIO_GPIO_B25       25 // IN_PU    
#define GPIO_GPIO_B26       26 // IN_PU    
#define GPIO_LED            27 // IN_PU    
#define GPIO_CLKGEN         28 // IN_PU    
#define GPIO_VFST           29 // IN_PU    
#define GPIO_VAD0           30 // IN_PU    
#define GPIO_VAD1           31 // IN_PU    
#define GPIO_VAD2           32 // IN_PU         
#define GPIO_VAD3           33 // IN_PU         
#define GPIO_VAD4           34 // IN_PU         
#define GPIO_VPIX           35 // IN_PU         
#define GPIO_VCLK           36 // IN_PU         
#define GPIO_VCLK_OUT       37 // IN_PU         
#define GPIO_VAD5           38 // IN_PU         
#define GPIO_VAD6           39 // IN_PU         
#define GPIO_VAD7           40 // IN_PU         
#define GPIO_VAD8           41 // IN_PU         
#define GPIO_VAD9           42 // IN_PU         
#define GPIO_D43            43 // IN_PU         
#define GPIO_D44            44 // IN_PU         
#define GPIO_D45            45 // IN_PU         
#define GPIO_D46            46 // IN_PU         
#define GPIO_BT_SYS_CLK_REQ 47 // ALT_FUNC      
#define GPIO_BT_TX_CLK      48 // IN_PU         
#define GPIO_BT_RX_DATA     49 // IN_PU         
#define GPIO_BT_TX_DATA     50 // ALT_FUNC      
#define GPIO_BT_SI_CDI      51 // ALT_FUNC      
#define GPIO_BT_SI_CDO      52 // IN_PU         
#define GPIO_BT_SYS_CLK     53 // IN_PU         
#define GPIO_BT_SI_CMS      54 // ALT_FUNC      
#define GPIO_BT_SI_CLK      55 // ALT_FUNC      
#define GPIO_BT_PHD_OFF     56 // ALT_FUNC      
#define GPIO_BT_PX_ON       57 // ALT_FUNC      
#define GPIO_BT_TX_ON       58 // ALT_FUNC      
#define GPIO_BT_RX_ON       59 // ALT_FUNC      
#define GPIO_BT_SYNT_ON     60 // ALT_FUNC      
#define GPIO_NAND_nWE       61 // IN_PU         
#define GPIO_NAND_nRE       62 // IN_PU         
#define GPIO_nWAIT          63 // IN_PU
            
#endif // ARGUS2_H
