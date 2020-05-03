/*****************************************************************************
 * CONFIDENTIAL AND PROPRIETARY INFORMATION. The entire contents of this file
 * is Copyright © Anoto AB 1999-2002, All Rights Reserved.  This document is 
 * an unpublished trade secret of Anoto AB and may not be viewed, copied or 
 * distributed by anyone, without the specific, written permission of Anoto AB.
 *
 * @archive: 			\projects\sw\hal\argus2\argus2.h
 * @version: 			\main\8
 * @date modified: 	Sun Nov 17 16:44:47 2002
 * @modified by: 		olah
 *
 * Description:     Argus III Register Definintions
 *
 * Original author: Niclas Bengtsson
 *
 ************************************************************************V10*/

// Revision history:
//    030918  OH Corrected GET_SSIA_WIDTH from 0xf to 0x1f



#ifndef ARGUS3_H
#define ARGUS3_H

#include <asm/arch/argus_addr.h>

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
#define SOC_AVP       (*(tAVP*)(APB_NOCACHE + 0x70000)     )    // Video Processor block,
#define SOC_DMA        (*(tDMA*)(DMA_NOCACHE))                  // DMA controller
#define SOC_EXTMEM    (*(tEXTMEM*)(APB_NOCACHE + 0xa0000)  )    // External memory interface,
#define SOC_UART      ( (tUART*)(APB_NOCACHE + 0xb0000)    )    // UART's (2),
#define SOC_USB       (*(tUSB*)(APB_NOCACHE + 0xd0000)     )    // USB slave
#define SOC_SSI       (*(tSSI*)(APB_NOCACHE + 0xf0000)     )    // Synchronous Serial Interface
#define SOC_CHIPID    (*(tCHIPID*)(APB_NOCACHE + 0x120000) )    // Chip ID,
#define SOC_CLKGEN    ( (tCLKGEN*)(APB_NOCACHE + 0x130000) )    // Clock generators (3),
#define SOC_ETHERNET  (*(tETHERNET*)(APB_NOCACHE + 0x160000) )  // Ethernet MAC
#define SOC_EXTDMA    (*(tEXTDMA*)(APB_NOCACHE + 0x170000) )    // Ethernet MAC
#define SOC_XARB      (*(tXARB*)(APB_NOCACHE + 0x180000) )      // XARB
#define SOC_L2CACHE   (*(tL2CACHE*)(L2CACHE_CTRL_NOCACHE)  )    // L2-Cache Control Interface
#define SOC_FPGA      (*(tFPGA*)(APB_NOCACHE + 0x1b0000)  )     // FPGA specific registers
#define SOC_AVE       (*(t_ave *)(AVE_BASE_V) )                 // AVE

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
    #define GET_PLL_M(x) ((x>>9)&0xff)
    #define CHG_PLL_M(x,upd) (x&~0x1fe00 | (upd<<9))
    #define SET_PLL_M(upd) (upd<<9)
  REG32 ENABLE;                          // Clock Gating read and enable (equal to CCG)
  REG32 DISABLE;                         // Clock Gating Disable
    #define CLK_CPU      (1<<0)          // CPU Clock
    #define CLK_AHB      (1<<1)          // AHB Clock
    #define CLK_APB      (1<<2)          // APB Clock
//    #define CLK_IMAGE    (1<<3)          // Image Processor Clock
//    #define CLK_JPEG     (1<<4)          // JPEG Clock
    #define CLK_AVP      (1<<3)          // Image Processor Clock
    #define CLK_AVE      (1<<4)          // Video engine Clock
//    #define CLK_MONTY    (1<<5)          // Montgomery Multiplier Clock
//    #define CLK_MATCH    (1<<6)          // Matcher Clock
    #define CLK_ETH      (1<<5)          // Ethernet Clock
    #define CLK_APB_CONT (1<<6)          // PCLK_CONT  
    #define CLK_OSC      (1<<7)          // Oscillator Enable
    #define CLK_UART0    (1<<8)          // UART 0 Clock
    #define CLK_UART1    (1<<9)          // UART 1 Clock
    #define CLK_CLKGEN0  (1<<10)         // Clock Generator 0 Clock
    #define CLK_CLKGEN1  (1<<11)         // Clock Generator 1 Clock
    #define CLK_CLKGEN2  (1<<12)         // Clock Generator 2 Clock
    #define CLK_SSI      (1<<13)         // Serial Synchronous Interface (SSI) Clock
    #define CLK_TLB      (1<<14)         // TLB Clock
    #define CLK_EXTMEM   (1<<15)         // External Memory Clock
    #define CLK_USB      (1<<16)         // Normal USB clock
    #define CLK_USB_TEST (1<<17)         // USB prod test mode. NOTE: Both CLK_USB and CLK_USB_TEST should not be enabled!!
    #define CLK_DMA      (1<<20)         // DMA clock    
  REG32 CLKDIVPER;
  REG32 CLKREQ;
    #define SET_CLKREQ_ASYNC_DMA(upd) (upd<<0)
    #define GET_CLKREQ_ASYNC_DMA(x) ((x>>0)&0x1)
    #define SET_CLKREQ_ASYNC_GPIO(upd) (upd<<1)
    #define GET_CLKREQ_ASYNC_GPIO(x) ((x>>1)&0x1)
    #define SET_CLKREQ_ASYNC_USB(upd) (upd<<2)
    #define GET_CLKREQ_ASYNC_USB(x) ((x>>2)&0x1)
//    #define SET_CLKREQ_ASYNC_RTC(upd) (upd<<3)
//    #define GET_CLKREQ_ASYNC_RTC(x) ((x>>3)&0x1)
    #define SET_CLKREQ_CCLK(upd) (upd<<4)
    #define GET_CLKREQ_CCLK(x) ((x>>4)&0x1)
//    #define SET_CLKREQ_PCLK_BT(upd) (upd<<5)
//    #define GET_CLKREQ_PCLK_BT(x) ((x>>5)&0x1)
    #define SET_CLKREQ_PCLK_USB(upd) (upd<<6)
    #define GET_CLKREQ_PCLK_USB(x) ((x>>6)&0x1)
//    #define SET_CLKREQ_ASYNC_BT(upd) (upd<<7)
//    #define GET_CLKREQ_ASYNC_BT(x) ((x>>7)&0x1)
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
    #define SET_CLKREQ_PCLK_AVP(upd) (upd<<16)
    #define GET_CLKREQ_PCLK_AVP(x) ((x>>16)&0x1)
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
    #define IRQ_AVP_MASK      0x00000100
    #define IRQ_AVE0_MASK     0x00000200
    #define IRQ_AVE1_MASK     0x00000400
    #define IRQ_DMA_MASK      0x00001000
    #define IRQ_ETHERNET_MASK 0x00002000
    #define IRQ_USB_MASK      0x00004000
    #define IRQ_SSI_MASK      0x00040000
    #define IRQ_COMMRX_MASK   0x00080000
    #define IRQ_COMMTX_MASK   0x00100000
    #define IRQ_FORCE_MASK    0x00200000

    #define IRQ_UART0    (0)
    #define IRQ_UART1    (1)
    #define IRQ_TIMER0   (2)
    #define IRQ_TIMER1   (3)
    #define IRQ_TIMER2   (4)
    #define IRQ_TIMER3   (5)
    #define IRQ_GPIO_IRQ (6)
    #define IRQ_GPIO_FIQ (7)
    #define IRQ_AVP      (8)
    #define IRQ_AVE0     (9)
    #define IRQ_AVE1     (10)
    #define IRQ_DMA      (12)
    #define IRQ_ETHERNET (13)
    #define IRQ_USB      (14)
    #define IRQ_SSI      (18)
    #define IRQ_COMMRX   (19)
    #define IRQ_COMMTX   (20)
    #define IRQ_FORCE    (21)
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
  REG32 FORCE;                            // Match force regiser (bit 0-3)
} tTIMER;

//-----------------------------
// GPIO Controller
//-----------------------------

typedef struct _tGPIO
{
  #define NUM_GPIO 92    // Number of GPIO pins

  REG32 PIN[NUM_GPIO];
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

  REG32 __gpio_pad[128-NUM_GPIO];
  REG32 PRIO;
    #define GET_GPIO_PRIO_IRQ_NBR(x) (((x)>>0)&0x7f)
    #define GET_GPIO_PRIO_IRQ(x) (((x)>>7)&0x1)
    #define GET_GPIO_PRIO_FIQ_NBR(x) (((x)>>8)&0x7f)
    #define GET_GPIO_PRIO_FIQ(x) (((x)>>15)&0x1)
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
// Video Processor
//-----------------------------

typedef struct _tAVP
{
  REG32 STATUS;           // 0x0000 : Common status register, clear bits by writing '1' 
  REG32 IRQ_MASK;         // 0x0004 : Mask for interrupt generation based on status register
    #define AVP_ALWAYS_MASK (1 << 11) // Always true condition for forcing software interrupt                   
    #define AVP_FST_MASK    (1 << 10) // External frame sync is received on video input interface               
    #define AVP_FEND_MASK   (1 <<  9) // Whole VCUT image has been delivered through video input interface           
    #define AVP_FERR_MASK   (1 <<  8) // Unexpected external frame sync (within frame or when input is disabled)
    #define AVP_RASTER_MASK (1 <<  7) // Set when video interface line counter matches RASTER register          
    #define AVP_DONE_MASK   (1 <<  6) // Full image has been processed by image processing                      
    #define AVP_RATE_MASK   (1 <<  5) // Too high pixel frequency or too short horizontal blanking              
    #define AVP_OF4_MASK    (1 <<  4) // Overflow has occurred in DMA channel 4 (analyze output)
    #define AVP_OF3_MASK    (1 <<  3) // Overflow has occurred in DMA channel 3 (image V output)
    #define AVP_OF2_MASK    (1 <<  2) // Overflow has occurred in DMA channel 2 (image U output)
    #define AVP_OF1_MASK    (1 <<  1) // Overflow has occurred in DMA channel 1 (image Y output)
    #define AVP_OF0_MASK    (1 <<  0) // Overflow has occurred in DMA channel 0 (bayer output)
                             
  REG32 DMA_IN;           // 0x0008: Format of DMA input stream for image processing
    #define SET_AVP_DMA_IN_YSIZE(x)    (((x)& 0x3fff)<<15)  // height in pixels - 1
    #define SET_AVP_DMA_IN_XSIZE(x)    (((x)& 0x3fff)<< 1)  // width in pixels - 1
    #define SET_AVP_DMA_IN_10BIT(x)    (((x)&    0x3)<< 0)  // Bits per value: 0=8-bit, 1=10-bit

  REG32 DPC;              // 0x000c: Defect pixel correction
    #define SET_AVP_DPC_H(x)           (((x)&  0x3)<<11)  // Correction hardness
    #define SET_AVP_DPC_K(x)           (((x)&  0x3)<< 9)  // Amplitude coefficient
    #define SET_AVP_DPC_D(x)           (((x)& 0xff)<< 1)  // Defect offset
    #define SET_AVP_DPC_STATIC(x)      (((x)&  0x1)<< 0)  // Enable static DPC list

  REG32 HCUT;             // 0x0010: Horizontal cut of image processing input stream
    #define SET_AVP_HCUT_XSTART(x)     (((x)& 0x3fff)<< 9)  // No. of pixels to skip
    #define SET_AVP_HCUT_XSIZE(x)      (((x)&  0x1ff)<< 0)  // width in pixels / 4

  REG32 DEMOSAIC;         // 0x0014: Bayer to RGB interpolation control, YUV decoding
    #define SET_AVP_DEMOSAIC_LE(x)     (((x)& 0x1) << 15)  // 1 = Discard 2 output pixels at left edge
    #define SET_AVP_DEMOSAIC_RE(x)     (((x)& 0x1) << 14)  // 1 = Discard 2 output pixels at right edge
    #define SET_AVP_DEMOSAIC_TE(x)     (((x)& 0x1) << 13)  // 1 = Discard 2 output pixels at top edge
    #define SET_AVP_DEMOSAIC_BE(x)     (((x)& 0x1) << 12)  // 1 = Discard 2 output pixels at bottom edge
    #define SET_AVP_DEMOSAIC_DY(x)     (((x)& 0x1) << 11)  // 0 = First line is G1/R, 1 = first line is B/G2
    #define SET_AVP_DEMOSAIC_DX(x)     (((x)& 0x1) << 10)  // 0 = First column is G1/B, 1 = first column is R/G2
    #define SET_AVP_DEMOSAIC_V(x)      (((x)& 0x3) <<  8)  // ordering of V in YYUV input stream
    #define SET_AVP_DEMOSAIC_U(x)      (((x)& 0x3) <<  6)  // ordering of U in YYUV input stream
    #define SET_AVP_DEMOSAIC_Y1(x)     (((x)& 0x3) <<  4)  // ordering of Y1 in YYUV input stream
    #define SET_AVP_DEMOSAIC_Y0(x)     (((x)& 0x3) <<  2)  // ordering of Y0 in YYUV input stream
    #define SET_AVP_DEMOSAIC_FORMAT(x) (((x)& 0x3) <<  0)  // 0=bayer 1=YY/UV 2=YYUV
    #define GET_AVP_DEMOSAIC_LE(x)     (((x) >> 15) & 0x1)
    #define GET_AVP_DEMOSAIC_RE(x)     (((x) >> 14) & 0x1)
    #define GET_AVP_DEMOSAIC_TE(x)     (((x) >> 13) & 0x1)
    #define GET_AVP_DEMOSAIC_BE(x)     (((x) >> 12) & 0x1)
    #define GET_AVP_DEMOSAIC_DY(x)     (((x) >> 11) & 0x1)
    #define GET_AVP_DEMOSAIC_DX(x)     (((x) >> 10) & 0x1)
    #define GET_AVP_DEMOSAIC_V(x)      (((x) >>  8) & 0x3)
    #define GET_AVP_DEMOSAIC_U(x)      (((x) >>  6) & 0x3)
    #define GET_AVP_DEMOSAIC_Y1(x)     (((x) >>  4) & 0x3)
    #define GET_AVP_DEMOSAIC_Y0(x)     (((x) >>  2) & 0x3)
    #define GET_AVP_DEMOSAIC_FORMAT(x) (((x) >>  0) & 0x3)

  REG32 BIAS;             // 0x0018: Filtering of G1/G2 bias artifacts, digital red/blue) gain
    #define SET_AVP_BIAS_BGAIN(x)    (((x)& 0xf)  << 28) // Digital gain of blue channel
                                                         // [-8 .. -1] maps to factor [0.5 .. 0.9375 ]
                                                         // [ 0 ..  7] maps to factor [1.0 .. 1.875  ]
    #define SET_AVP_BIAS_RGAIN(x)    (((x)& 0xf)  << 24) // Digital gain of red channel (see BGAIN)
    #define SET_AVP_BIAS_THRESH(x)   (((x)& 0xff) << 16) // Threshold for dynamic G1/G2 adjustment (unsigned)
    #define SET_AVP_BIAS_G2(x)       (((x)& 0xff) <<  8) // Static G2 adjustment (signed)
    #define SET_AVP_BIAS_G1(x)       (((x)& 0xff) <<  0) // Static G1 adjustment (signed)

  REG32 ANALYZE;          // 0x001c: Block-based statistics and histogram control
    #define SET_AVP_ANALYZE_SIZEY(x) (((x)& 0x7) << 6)  // Block height (0=8, 1=16, 2=24, 3=32 .. 7=64)
    #define SET_AVP_ANALYZE_SIZEX(x) (((x)& 0x7) << 3)  // Block width  (0=8, 1=16, 2=24, 3=32 .. 7=64)
    #define SET_AVP_ANALYZE_MINMAX(x)(((x)& 0x1) << 2)  // Enable min/max values in block-based statistics
    #define SET_AVP_ANALYZE_BLOCK(x) (((x)& 0x1) << 1)  // Enable block-based statistics
    #define SET_AVP_ANALYZE_HIST(x)  (((x)& 0x1) << 0)  // Enable histogram 

  REG32 SHARP;            // 0x0020: Sharpening filter configuration
    #define SET_AVP_SHARP_K3(x)     (((x)& 0x3f) << 26)  // Coefficient for G2 pixels
    #define SET_AVP_SHARP_K2(x)     (((x)& 0x3f) << 20)  // Coefficient for B pixels
    #define SET_AVP_SHARP_K1(x)     (((x)& 0x3f) << 14)  // Coefficient for R pixels
    #define SET_AVP_SHARP_K0(x)     (((x)& 0x3f) <<  8)  // Coefficient for G1 pixels
    #define SET_AVP_SHARP_THRESH(x) (((x)& 0xff) <<  0)  // Threshold for sharpening term

  REG32 RADIAL_X0;        // 0x0024: Location of first input pixel with respect to optical axis, 14-bit signed
  REG32 RADIAL_Y0;        // 0x0028: Location of first input pixel with respect to optical axis, 14-bit signed
  REG32 RADIAL_X02;       // 0x002c: Square of RADIAL_X0, 26-bit unsigned
  REG32 RADIAL_Y02;       // 0x0030: Square of RADIAL_Y0, 26-bit unsigned
  REG32 RADIAL_SHIFT;     // 0x0034: Exponent for radial illumination correction, 3-bit unsigned
  REG32 RADIAL_COEFF[6];  // 0x0038: Radial corrrection coefficents, 6-bit unsigned
                          //         Order is kxr kyr kxg kyg kxb kyb

  REG32 CLIP;             // 0x0050: Truncation of RGB values before color correction
    #define SET_AVP_CLIP_BMAX(x) (((x)& 0x3ff) << 20)  // Upper limit for blue channel
    #define SET_AVP_CLIP_GMAX(x) (((x)& 0x3ff) << 10)  // Upper limit for green channel
    #define SET_AVP_CLIP_RMAX(x) (((x)& 0x3ff) <<  0)  // Upper limit for red channel

  REG32 COLOR_CORR[12];   // 0x0054: Color correction coefficients, signed 14-bit
                          //         Order is C11, C12, C13, C21, C22, C23, C21, C22, C23, D1, D2, D3

  REG32 GAMMA;            // 0x0084: Gamma correction enable bit, 1=enabled

  REG32 COLOR_CONV[12];   // 0x0088: Color conversion coefficients, signed 9-bit
                          //         Order is C11, C12, C13, C21, C22, C23, C21, C22, C23, D1, D2, D3

  REG32 HSCALE;           // 0x00b8: Horizontal downscale
    #define SET_AVP_HSCALE_EXP(x)   (((x)& 0x07) << 20)  // Denominator exponent part
    #define SET_AVP_HSCALE_FRAC(x)  (((x)& 0xff) << 12)  // Denominator fraction part
    #define SET_AVP_HSCALE_SKIP(x)  (((x)& 0x0f) <<  8)  // Initial number of pixels to skip
    #define SET_AVP_HSCALE_FPOS(x)  (((x)& 0xff) <<  0)  // Initial sub-pixel offset
    #define GET_AVP_HSCALE_EXP(x)   (((x)>> 20) & 0x07)
    #define GET_AVP_HSCALE_FRAC(x)  (((x)>> 12) & 0xff)
    #define GET_AVP_HSCALE_SKIP(x)  (((x)>>  8) & 0x0f)
    #define GET_AVP_HSCALE_FPOS(x)  (((x)>>  0) & 0xff)

  REG32 VSCALE;           // 0x00bc: Vertical downscale
    #define SET_AVP_VSCALE_UVDIV(x) (((x)& 0x01) << 23)  // Extra div by 2 for UV channels
    #define SET_AVP_VSCALE_EXP(x)   (((x)& 0x07) << 20)  // Denominator exponent part
    #define SET_AVP_VSCALE_FRAC(x)  (((x)& 0xff) << 12)  // Denominator fraction part
    #define SET_AVP_VSCALE_SKIP(x)  (((x)& 0x0f) <<  8)  // Initial number of pixels to skip
    #define SET_AVP_VSCALE_FPOS(x)  (((x)& 0xff) <<  0)  // Initial sub-pixel offset
    #define GET_AVP_VSCALE_UVDIV(x) (((x) >> 23) & 0x01) // Extra div by 2 for UV channels
    #define GET_AVP_VSCALE_EXP(x)   (((x) >> 20) & 0x07) // Denominator exponent part
    #define GET_AVP_VSCALE_FRAC(x)  (((x) >> 12) & 0xff) // Denominator fraction part
    #define GET_AVP_VSCALE_SKIP(x)  (((x) >>  8) & 0x0f) // Initial number of pixels to skip
    #define GET_AVP_VSCALE_FPOS(x)  (((x) >>  0) & 0xff) // Initial sub-pixel offset
  
  REG32 OUTPUT;           // 0x00c0: Format of DMA image output streams, YUV encoding
    #define SET_AVP_OUTPUT_RGB(x)    (((x)& 0x01) << 6)  // Bypass YUV conversion
    #define SET_AVP_OUTPUT_FORMAT(x) (((x)& 0x03) << 4)  // Encoding: 0=YY+U+V 1=YY+UV 2=YYUV 3=YUYV
    #define SET_AVP_OUTPUT_10BIT(x)  (((x)& 0x01) << 3)  // Bits per value: 0=8-bit, 1=10-bit
    #define SET_AVP_OUTPUT_V(x)      (((x)& 0x01) << 2)  // Enable V (ternary) output channel (DMA3)
    #define SET_AVP_OUTPUT_U(x)      (((x)& 0x01) << 1)  // Enable U (second) output channel  (DMA2)
    #define SET_AVP_OUTPUT_Y(x)      (((x)& 0x01) << 0)  // Enable Y (primary) output channel (DMA1)

  REG32 IMAGE_CTRL;       // 0x00c4: Main control register for image processing
    #define SET_AVP_IMAGE_CTRL_NOSLEEP(x)  (((x)& 0x01) << 2) // 1= disable clock sleep during blanking
    #define SET_AVP_IMAGE_CTRL_DMA_MODE(x) (((x)& 0x01) << 1) // Input selection: 0=sensor 1=dma
    #define SET_AVP_IMAGE_CTRL_ENABLE(x)   (((x)& 0x01) << 0) // Enable image processing
                             
  REG32 VCUT;             // 0x00c8: Vertical cut of video input image. Height is YEND-YSTART+1 pixels
    #define SET_AVP_VCUT_YSTART(x)  (((x)& 0x3fff) << 14) // First line to include
    #define SET_AVP_VCUT_YEND(x)    (((x)& 0x3fff) <<  0) // Last line to include. Height is YEND-YSTART+1 pixels

  REG32 BAYER_OUT;        // 0x00cc: DMA output control from video input interface, horizontal cut
    #define SET_AVP_BAYER_OUT_XSTART(x) (((x)& 0x3fff) << 16) // First column to include
    #define SET_AVP_BAYER_OUT_XEND(x)   (((x)& 0x3fff) <<  2) // Last column to include. Width is XEND-XSTART+1 pixels
    #define SET_AVP_BAYER_OUT_10BIT(x)  (((x)&  0x01) <<  1) // Bits per value: 0=8-bit, 1=10-bit
    #define SET_AVP_BAYER_OUT_ENABLE(x) (((x)&  0x01) <<  0) // Enable bayer output channel (DMA0)

  REG32 RASTER;           // 0x00d0: Vertical position for raster interrupt, 14-bit unsigned
  
  REG32 VIDIN_CTRL;       // 0x00d4: Main control register for video input interface
    #define SET_AVP_VIDIN_CTRL_IMSIZE_X(x) (((x)& 0x3fff) << 7) // Width of input image (no of pixels - 1)
    #define SET_AVP_VIDIN_CTRL_YSUB(x)     (((x)&   0x01) << 6) // 1=enable vertical subsampling
    #define SET_AVP_VIDIN_CTRL_XSUB(x)     (((x)&   0x01) << 5) // 1=enable horizontal subsampling
    #define SET_AVP_VIDIN_CTRL_10BIT(x)    (((x)&   0x01) << 4) // 0=8-bit VAD, 1=10-bit VAD
    #define SET_AVP_VIDIN_CTRL_FST_EDGE(x) (((x)&   0x01) << 3) // 0=negative VFST edge, 1=positive
    #define SET_AVP_VIDIN_CTRL_CLK_EDGE(x) (((x)&   0x01) << 2) // 0=negative VCLK edge, 1=positive
    #define SET_AVP_VIDIN_CTRL_PIX_VAL(x)  (((x)&   0x01) << 1) // 0=VPIX active low, 1=active high 
    #define SET_AVP_VIDIN_CTRL_ENABLE(x)   (((x)&   0x01) << 0) // Enable video input 
  
  REG32 LED1;             // 0x00d8: LED strobe timing and active level
    #define SET_LED1_ON_PERIOD(x)    (((x)& 0xfff) << 13) // Duration of LED strobe
    #define SET_LED1_ON_TIME(x)      (((x)& 0xfff) <<  1) // Delay from frame sync to LED activation
    #define SET_LED1_ACTIVE_LEVEL(x) (((x)&  0x01) <<  0) // 0=active low, 1=active high

  REG32 LED2;             // 0x00dc: LED strobe clock prescale and enable bit
    #define SET_LED2_PRESCALE(x)     (((x)& 0xfff) <<  1) // Clock pre-scale factor for LED timing
    #define SET_LED2_ENABLE(x)       (((x)&  0x01) <<  0) // 0=LED disabled, 1=enabled
  
  REG32 pad0[1024-56];
  REG32 GAMMA_R[64];      // 0x1000: Gamma table for R channel SLOPE[7:0] START[9:0]
  REG32 GAMMA_G[64];      // 0x1100: Gamma table for G channel
  REG32 GAMMA_B[64];      // 0x1200: Gamma table for B channel
  REG32 pad1[1024-192];

  REG32 SCALE_RAM[2*640]; // 0x2000: Scale RAM 640*(2*20)  (low0, high0, low1, high1, ...)
  REG32 pad3[2048-2*640];

  REG32 BAYER_RAM[2*1664];// 0x4000: Bayer and analysis temp storage 1824*(2*20) (low0, high0, low1, high1, ...)
  REG32 HIST_G1[64];
  REG32 HIST_R[64];
  REG32 HIST_B[64];
  REG32 HIST_G2[64];
  REG32 DPC_LIST[2*32];

} tAVP;

//-----------------------------
// DMA controller
//-----------------------------

typedef struct _tDMA
{
  struct tDMA_CHANNEL
  {
    REG32 CHACONFIG;
    REG32 CHACTRL;
    REG32 CHASTATUS;
    REG32 __pad0;
    REG32 MDEVADDR;
    REG32 MDEVCONFIG;
    REG32 __pad1[2];
    struct tDMA_DESCQUEUE
    {
  	  REG32 DESC0;
  	  REG32 DESC1;
  	  REG32 DESC2;
  	  REG32 DESC3;
    } DESCQUEUE[2];
  } CHANNEL[10];
  REG32 __pad3[(16-10)*64/4];
  REG32 IRQ;
  REG32 COMMON;
  REG32 __pad4[(4096-16*64-8)/4];
  REG32 RAM[1536/4];
} tDMA;

// CHACONFIG read/write configuration register, normally static. Not changed by the hardware.
#define SET_DMA_CHACONFIG_PORT(x)      ((x)<<0)     // 5  device port select 0-31
#define SET_DMA_CHACONFIG_PRIO(x)      ((x)<<5)     // 2  priority group, 0: lowest prio, 3:highest prio, round robin policy in group 0
#define SET_DMA_CHACONFIG_SIZE(x)      ((x)<<7)     // 3  device size, 0:8bit, 1:16bit, 2:32bit etc
#define SET_DMA_CHACONFIG_DIR(x)       ((x)<<10)    // 1  0: mem=>dev, 1: dev=>mem
#define SET_DMA_CHACONFIG_MDEV(x)      ((x)<<11)    // 1  0: Fast device, 1: Memory mapped AHB device
#define SET_DMA_CHACONFIG_FIFOTHRES(x) ((x)<<12)    // 2  Fifo threshold 0:16 byte, 1:32 byte, 2:64 byte, 3:128 byte

#define GET_DMA_CHACONFIG_PORT(x)      (((x)>>0)&0x1f)
#define GET_DMA_CHACONFIG_PRIO(x)      (((x)>>5)&0x3)
#define GET_DMA_CHACONFIG_SIZE(x)      (((x)>>7)&0x7)
#define GET_DMA_CHACONFIG_DIR(x)       (((x)>>10)&1)
#define GET_DMA_CHACONFIG_MDEV(x)      (((x)>>11)&1)
#define GET_DMA_CHACONFIG_FIFOTHRES(x) (((x)>>12)&0x3)

// CHACTRL read/write control
#define SET_DMA_CHACTRL_DEVEN(x)  ((x)<<0)          // RW Write 1 to enable device
#define GET_DMA_CHACTRL_DEVEN(x)  (((x)>>0)&0x1)
#define SET_DMA_CHACTRL_BUFEN(x)  ((x)<<1)          // RW Write 1 to enable buffer
#define GET_DMA_CHACTRL_BUFEN(x)  (((x)>>1)&0x1)
#define SET_DMA_CHACTRL_RESET(x)  ((x)<<2)          // RW Write 1 to reset the channel. Read to 0 when reset is finished.
#define GET_DMA_CHACTRL_RESET(x)  (((x)>>2)&0x1)
#define SET_DMA_CHACTRL_DRAIN(x)  ((x)<<3)          // RW Write 1 to drain/fill the channel fifo. Read to 0 when drain is finished.
#define GET_DMA_CHACTRL_DRAIN(x)  (((x)>>3)&0x1)
#define SET_DMA_CHACTRL_IRQ(x)    ((x)<<4)          // RW Write 1 to ack irq. Read channel IRQ status.
#define GET_DMA_CHACTRL_IRQ(x)    (((x)>>4)&0x1)
#define SET_DMA_CHACTRL_SETIRQ(x) ((x)<<5)          //  W Write 1 to manually set IRQ.
#define SET_DMA_CHACTRL_EOF(x)    ((x)<<6)          // RW Write 1 to clear EOF condition. Read channel EOF status.
#define GET_DMA_CHACTRL_EOF(x)    (((x)>>6)&0x1)
#define SET_DMA_CHACTRL_SETEOF(x) ((x)<<7)          //  W Write 1 to manually generate EOF condition. Cleared when handled.
#define SET_DMA_CHACTRL_ADDBUF(x) ((x)<<8)          //  W Write 1 to add a descriptor to the descriptor queue.
#define SET_DMA_CHACTRL_PAUSE(x)  ((x)<<9)          //  W Write 1 to clear DEVEN and BUFEN
#define SET_DMA_CHACTRL_POLL(x)   ((x)<<10)         // RW Write 1 to set POLL condition. 
#define GET_DMA_CHACTRL_POLL(x)   (((x)>>10)&0x1)
#define SET_DMA_CHACTRL_FIFOSTALL(x)((x)<<11)       // RW Write 1 to clear FIFOSTALL flag
#define GET_DMA_CHACTRL_FIFOSTALL(x)(((x)>>11)&0x1)
#define SET_DMA_CHACTRL_CLEARQUEUE(x)((x)<<12)      // RW Write 1 to clear the descriptor queue (only for debug purpose)
#define GET_DMA_CHACTRL_CLEARQUEUE(x)(((x)>>12)&0x1)

// CHASTATUS is a read-only register
#define GET_DMA_CHASTATUS_DESCPOS(x) (((x)>>0)&1)     // 1  Insertion position in the desc queue
#define GET_DMA_CHASTATUS_DESCCNT(x) (((x)>>1)&3)     // 2  Number of desc in the queue (0,1,2)
#define GET_DMA_CHASTATUS_BUFBUSY(x) (((x)>>3)&1)     // 1  Set if a buffer operation is ongoing
#define GET_DMA_CHASTATUS_DEVBUSY(x) (((x)>>4)&1)     // 1  Set if a device operation is ongoing
#define GET_DMA_CHASTATUS_FIFOFILL(x)(((x)>>8)&0x1ff) // 9  Number of bytes currently in the data fifo

// IRQ is a read-only register
#define GET_DMA_IRQ_CHANNEL(x) (((x)>>16)&0xf)      // Lowest channel signalling IRQ. 0 if no channel is signalling.
#define SET_DMA_IRQ_CHANNEL(x) ((x)<<16)

// COMMON is read/write register containing global settings
#define SET_DMA_COMMON_AHBBURSTMASK(x)  ((x)<<0)            // Allowed burst types on AHB bus {WRAP4,WRAP8,WRAP16,WRAP32,WRAP64} Normally 0x03
#define GET_DMA_COMMON_AHBBURSTMASK(x)  (((x)>>0)&0x1f)
#define SET_DMA_COMMON_XAHBBURSTMASK(x) ((x)<<5)            // Allowed burst types on XAHB bus {WRAP4,WRAP8,WRAP16,WRAP32,WRAP64} Normally 0x1f
#define GET_DMA_COMMON_XAHBBURSTMASK(x) (((x)>>5)&0x1f)

#define SET_DMA_MDEVCONFIG_SIZE(x)     ((x)<<0)     // 16  Remaining size of buffer in bytes. Only used if USESIZE is set.
#define SET_DMA_MDEVCONFIG_PROT(x)     ((x)<<16)    // 4   HPROT (cacheable, bufferable)
#define SET_DMA_MDEVCONFIG_BUSSEL(x)   ((x)<<20)    // 1   0: AHB, 1: XAHB
#define SET_DMA_MDEVCONFIG_PRIO(x)     ((x)<<21)    // 1   AHB priority
#define SET_DMA_MDEVCONFIG_INCRADDR(x) ((x)<<23)    // 1   0: Fixed address, 1: Increment address
#define SET_DMA_MDEVCONFIG_USESIZE(x)  ((x)<<24)    // 1   0: Unlimited size 1: Decrement size

#define SET_DMA_DESC0_BUFADDR(x)    ((x)<<0)        // 32  Physical address of buffer. Incremented
#define SET_DMA_DESC1_BUFSIZE(x)    ((x)<<0)        // 16  Remaining size of buffer in bytes. Decremented.
#define GET_DMA_DESC1_BUFSIZE(x)    (((x)>>0)&0xffff)
#define SET_DMA_DESC1_PROT(x)       ((x)<<16)       // 4   HPROT (cacheable, bufferable)
#define SET_DMA_DESC1_BUSSEL(x)     ((x)<<20)       // 1   0: AHB, 1: XAHB
#define SET_DMA_DESC1_PRIO(x)       ((x)<<21)       // 1   AHB priority
#define SET_DMA_DESC1_IRQ(x)        ((x)<<22)       // 1   1: Generate IRQ when this descriptor has finished
#define SET_DMA_DESC1_EOF(x)        ((x)<<23)       // 1   1: Wait for EOF before finishing this descriptor
#define SET_DMA_DESC1_OPTION(x)     ((x)<<24)       // 2   DESC2/3 usage. 0: None. 1: Framereg. 2:Refill
#define SET_DMA_DESC1_STATE(x)      ((x)<<26)       // 2   0:START, 1:DATA, 2:STATUS, 3:DONE
#define GET_DMA_DESC1_STATE(x)      (((x)>>26)&3)
#define SET_DMA_DESC1_INITFIFO(x) ((x)<<28)         // 1   Prefetch fifo up to threshold before accepting first DREQ from device (useful in mem->dev transfer)
#define SET_DMA_DESC2_REMAINSIZE(x) ((x)<<0)        // 32  Remaining size of the buffer in byte
#define SET_DMA_DESC3_REFILLSIZE(x) ((x)<<0)        // 16  Refill amount in byte. Unsigned 16 bit integer
#define SET_DMA_DESC3_SKIPSIZE(x)   ((x)<<16)       // 16  Skip amount in byte. Signed 16 bit integer
#define SET_DMA_DESC2_FRAMEADDR(x)  ((x)<<0)        // 32  Physical address of frame status register
#define SET_DMA_DESC3_FRAMEDATA(x)  ((x)<<0)        // 32  Data read/write from/to frame status register

#define DMA_DESC1_STATE_START   (0)
#define DMA_DESC1_STATE_DATA    (1)
#define DMA_DESC1_STATE_STATUS  (2)
#define DMA_DESC1_STATE_DONE    (3)

#define DMA_DESC1_OPTION_NONE     (0)
#define DMA_DESC1_OPTION_FRAMEREG (1)
#define DMA_DESC1_OPTION_REFILL   (2)


// SiZE constants
#define DMA_SIZE_BYTE         0
#define DMA_SIZE_SHORT        1
#define DMA_SIZE_WORD         2

// DIR constants
#define DMA_MEM2DEV           0
#define DMA_DEV2MEM           1
                              
#define DMA_PORT_AVP_BAYER_OUT 0
#define DMA_PORT_AVP_Y         1
#define DMA_PORT_AVP_U         2
#define DMA_PORT_AVP_V         3
#define DMA_PORT_AVP_ANALYZE   4
#define DMA_PORT_AVP_BAYER_IN  5
#define DMA_PORT_ETHRX         6
#define DMA_PORT_ETHTX         7
#define DMA_PORT_USB_EP2_RD   16
#define DMA_PORT_USB_EP2_WR   17
#define DMA_PORT_USB_EP3_RD   18
#define DMA_PORT_USB_EP3_WR   19
#define DMA_PORT_UART0_RD     20
#define DMA_PORT_UART0_WR     21
#define DMA_PORT_UART1_RD     22
#define DMA_PORT_UART1_WR     23
#define DMA_PORT_SSI_IN       24
#define DMA_PORT_SSI_OUT      25
#define DMA_PORT_EXT0         28
#define DMA_PORT_EXT1         29
#define DMA_PORT_ALWAYS (31)                      // Special always requesting port

//-----------------------------
// EXT DMA (External DREQ)
//-----------------------------

typedef struct _tEXTDMA
{
  struct tPORT
  {
    REG32 ENABLE;
    REG32 POLARITY;  // 1 - Inverts polarity
  } PORT[2];

} tEXTDMA;

//-----------------------------
// XARB 
//-----------------------------

typedef struct _tXARB
{
  REG32 AVEPRIO;    // AVE prio mode
    #define XARB_AVEPRIO_LOW        0 
    #define XARB_AVEPRIO_HIGH       1
    #define XARB_AVEPRIO_ROUNDROBIN 2 // default
      
  REG32 PERFCTRL;
    #define SET_XARB_PERFCTRL_ENABLE(x) ((x)<<0)  // NOTE: when perfcnt is enabled, EXTCLK is always on (higher power)
    #define SET_XARB_PERFCTRL_BUS(x)    ((x)<<1)
      #define XARB_BUS_EXTMEM 0
      #define XARB_BUS_PCA    1
      #define XARB_BUS_DMA    2
      #define XARB_BUS_AVE    3
    #define SET_XARB_PERFCTRL_EVENT(x)  ((x)<<4)
      #define XARB_EVENT_NOGRANT  0   // number of no grant cycles  Unit: cycle
      #define XARB_EVENT_CYCLE    1   // number of access cycles    Unit: cycle
      #define XARB_EVENT_ACCESS   2   // number of accesses         Unit: access
      #define XARB_EVENT_WRITE    3   // number of write acceses    Unit: access
      #define XARB_EVENT_SEQ      4   // number of seq accesses     Unit: access
      #define XARB_EVENT_WAIT     5   // number of wait cycles      Unit: cycle
    
  REG32 PERFCNT;    // 32-bits event counter
  
} tXARB;

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
      #define SET_EXTSDCTRL2_PCHGMODE(upd) ((upd)<<26)   // 1: Eager precharge mode
      #define SET_EXTSDCTRL2_CKEMODE(upd) ((upd)<<27)    // 1: Low-latency CKE mode

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
    #define UART_BAUD_2400   0x0000017f
    #define UART_BAUD_4800   0x000000bf
    #define UART_BAUD_9600   0x0000005f
    #define UART_BAUD_14400  0x0000003f
    #define UART_BAUD_28800  0x0000001f
    #define UART_BAUD_57600  0x0000000f
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
    #define GET_SSIA_WIDTH(x) (((x)>>24)&0x1f)
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

    #define CLKGEN_VCLK_OUT   0
    #define CLKGEN_PHYCLK_OUT 1
    #define CLKGEN_CLKGEN     2

  REG32 _PAD[(0x10000-1*4)/4];
} tCLKGEN;


typedef struct _tETHERNET
{
  // MAC registers
  // NOTE: HW need to convert address before sending it to the MAC
  // MAC_ADDR = (APB_ADDR>>2) | 0x10
/*
  // Defines according to the MAC manual
  REG32 MAC_CTL;     // 0x000  10
  REG32 CAM_CTL;     // 0x004  11
  REG32 TX_CTL;      // 0x008  12
  REG32 TX_STAT;     // 0x00c  13
  REG32 RX_CTL;      // 0x010  14
  REG32 RX_STAT;     // 0x014  15
  REG32 MD_DATA;     // 0x018  16
  REG32 MD_CA;       // 0x01c  17
  REG32 MDC_DIV;     // 0x020  18
  REG32 CAM_ENA;     // 0x028
  REG32 PROM_CTL;    // 0x02c  TO BE REMOVED
  REG32 PROM_DATA;   // 0x030  TO BE REMOVED
  REG32 ALIGN_CNT;   // 0x034
  REG32 CRC_CNT;     // 0x038
  REG32 MISS_CNT;    // 0x03c
*/

  // TODO: Check register map
  // Defines according to Ethernet testbench
  REG32 MAC_CTL;       // 0x000
    #define SET_MAC_CTL_HALTREQ(upd) (upd<<0)
    #define GET_MAC_CTL_HALTREQ(x) ((x>>0)&0x1)
    #define SET_MAC_CTL_HALTIMM(upd) (upd<<1)
    #define GET_MAC_CTL_HALTIMM(x) ((x>>1)&0x1)
    #define SET_MAC_CTL_RESET(upd) (upd<<2)
    #define GET_MAC_CTL_RESET(x) ((x>>2)&0x1)
    #define SET_MAC_CTL_FULLDUP(upd) (upd<<3)
    #define GET_MAC_CTL_FULLDUP(x) ((x>>3)&0x1)
    #define SET_MAC_CTL_MACLOOP(upd) (upd<<4)
    #define GET_MAC_CTL_MACLOOP(x) ((x>>4)&0x1)
    #define SET_MAC_CTL_CONN(upd) (upd<<5)
    #define GET_MAC_CTL_CONN(x) ((x>>5)&0x3)
    #define SET_MAC_CTL_LOOP10(upd) (upd<<7)
    #define GET_MAC_CTL_LOOP10(x) ((x>>7)&0x1)
    #define GET_MAC_CTL_MISSROLL(x) ((x>>10)&0x1)
    #define SET_MAC_CTL_ENMISSROLL(upd) (upd<<13)
    #define GET_MAC_CTL_ENMISSROLL(x) ((x>>13)&0x1)
    #define GET_MAC_CTL_LINK10(x) ((x>>15)&0x1)
  REG32 CAM_CTL;       // 0x004
    #define SET_CAM_CTL_STATIONACC(upd) (upd<<0)
    #define GET_CAM_CTL_STATIONACC(x) ((x>>0)&0x1)
    #define SET_CAM_CTL_GROUPACC(upd) (upd<<1)
    #define GET_CAM_CTL_GROUPACC(x) ((x>>1)&0x1)
    #define SET_CAM_CTL_BROADACC(upd) (upd<<2)
    #define GET_CAM_CTL_BROADACC(x) ((x>>2)&0x1)
    #define SET_CAM_CTL_NEGCAM(upd) (upd<<3)
    #define GET_CAM_CTL_NEGCAM(x) ((x>>3)&0x1)
    #define SET_CAM_CTL_COMPEN(upd) (upd<<4)
    #define GET_CAM_CTL_COMPEN(x) ((x>>4)&0x1)
  REG32 TX_CTL;        // 0x008
    #define SET_TX_CTL_TXEN(upd) (upd<<0)
    #define GET_TX_CTL_TXEN(x) ((x>>0)&0x1)
    #define SET_TX_CTL_TXHALT(upd) (upd<<1)
    #define GET_TX_CTL_TXHALT(x) ((x>>1)&0x1)
    #define SET_TX_CTL_NOPAD(upd) (upd<<2)
    #define GET_TX_CTL_NOPAD(x) ((x>>2)&0x1)
    #define SET_TX_CTL_NOCRC(upd) (upd<<3)
    #define GET_TX_CTL_NOCRC(x) ((x>>3)&0x1)
    #define SET_TX_CTL_FBACK(upd) (upd<<4)
    #define GET_TX_CTL_FBACK(x) ((x>>4)&0x1)
    #define SET_TX_CTL_NOEXDEF(upd) (upd<<5)
    #define GET_TX_CTL_NOEXDEF(x) ((x>>5)&0x1)
    #define SET_TX_CTL_SDPAUSE(upd) (upd<<6)
    #define GET_TX_CTL_SDPAUSE(x) ((x>>6)&0x1)
    #define SET_TX_CTL_MII10(upd) (upd<<7)
    #define GET_TX_CTL_MII10(x) ((x>>7)&0x1)
    #define SET_TX_CTL_ENUNDER(upd) (upd<<8)
    #define GET_TX_CTL_ENUNDER(x) ((x>>8)&0x1)
    #define SET_TX_CTL_ENEXDEFER(upd) (upd<<9)
    #define GET_TX_CTL_ENEXDEFER(x) ((x>>9)&0x1)
    #define SET_TX_CTL_ENLCARR(upd) (upd<<10)
    #define GET_TX_CTL_ENLCARR(x) ((x>>10)&0x1)
    #define SET_TX_CTL_ENEXCOLL(upd) (upd<<11)
    #define GET_TX_CTL_ENEXCOLL(x) ((x>>11)&0x1)
    #define SET_TX_CTL_ENLATECOLL(upd) (upd<<12)
    #define GET_TX_CTL_ENLATECOLL(x) ((x>>12)&0x1)
    #define SET_TX_CTL_ENTXPAR(upd) (upd<<13)
    #define GET_TX_CTL_ENTXPAR(x) ((x>>13)&0x1)
    #define SET_TX_CTL_ENCOMP(upd) (upd<<14)
    #define GET_TX_CTL_ENCOMP(x) ((x>>14)&0x1)

  REG32 TX_STAT;       // 0x00c
    // Bitfields are the same as in TX_STATUS
  REG32 RX_CTL;        // 0x010
    #define SET_RX_CTL_RXEN(upd) (upd<<0)
    #define GET_RX_CTL_RXEN(x) ((x>>0)&0x1)
    #define SET_RX_CTL_RXHALT(upd) (upd<<1)
    #define GET_RX_CTL_RXHALT(x) ((x>>1)&0x1)
    #define SET_RX_CTL_LONGEN(upd) (upd<<2)
    #define GET_RX_CTL_LONGEN(x) ((x>>2)&0x1)
    #define SET_RX_CTL_SHORTEN(upd) (upd<<3)
    #define GET_RX_CTL_SHORTEN(x) ((x>>3)&0x1)
    #define SET_RX_CTL_STRIPCRC(upd) (upd<<4)
    #define GET_RX_CTL_STRIPCRC(x) ((x>>4)&0x1)
    #define SET_RX_CTL_PASSCTL(upd) (upd<<5)
    #define GET_RX_CTL_PASSCTL(x) ((x>>5)&0x1)
    #define SET_RX_CTL_IGNORECRC(upd) (upd<<6)
    #define GET_RX_CTL_IGNORECRC(x) ((x>>6)&0x1)
    #define SET_RX_CTL_ENALIGN(upd) (upd<<8)
    #define GET_RX_CTL_ENALIGN(x) ((x>>8)&0x1)
    #define SET_RX_CTL_ENCRCERR(upd) (upd<<9)
    #define GET_RX_CTL_ENCRCERR(x) ((x>>9)&0x1)
    #define SET_RX_CTL_ENOVER(upd) (upd<<10)
    #define GET_RX_CTL_ENOVER(x) ((x>>10)&0x1)
    #define SET_RX_CTL_ENLONGERR(upd) (upd<<11)
    #define GET_RX_CTL_ENLONGERR(x) ((x>>11)&0x1)
    #define SET_RX_CTL_ENRXPAR(upd) (upd<<13)
    #define GET_RX_CTL_ENRXPAR(x) ((x>>13)&0x1)
    #define SET_RX_CTL_ENGOOD(upd) (upd<<14)
    #define GET_RX_CTL_ENGOOD(x) ((x>>14)&0x1)
  REG32 RX_STAT;       // 0x014
    // Bitfields are the same as in RX_STATUS
  REG32 MD_DATA;       // 0x018
  REG32 MD_CA;         // 0x01c
    #define SET_MD_CA_ADDR(upd) (upd<<0)
    #define GET_MD_CA_ADDR(x) ((x>>0)&0x1f)
    #define SET_MD_CA_PHY(upd) (upd<<5)
    #define GET_MD_CA_PHY(x) ((x>>5)&0x1f)
    #define SET_MD_CA_WR(upd) (upd<<10)
    #define GET_MD_CA_WR(x) ((x>>10)&0x1)
    #define SET_MD_CA_BUSY(upd) (upd<<11)
    #define GET_MD_CA_BUSY(x) ((x>>11)&0x1)
    #define SET_MD_CA_PRESUP(upd) (upd<<12)
    #define GET_MD_CA_PRESUP(x) ((x>>12)&0x1)

  // CAM_ADDR in testbench
  REG32 MDC_DIV;       // 0x020

  REG32 CAM_DATA;      // 0x024
  REG32 CAM_ENA;       // 0x028
  REG32 PROM_CTL_DATA; // 0x02c  TO BE REMOVED
  REG32 RESERVED;      // 0x030
  REG32 ALIGN_CNT;     // 0x034
  REG32 CRC_CNT;       // 0x038
  REG32 MISS_CNT;      // 0x03c

  REG32 __pad1[48];

  // ETHERNET MAC IF
  REG32 TX_DATA;      // 0x100
  REG32 TX_STATUS;    // 0x104
    // All MAC TXSTAT bits
    #define GET_TX_STATUS_STATUS(x) ((x>>0)&0x1ffff)

    // MAC TXSTAT bits
    #define GET_TX_STATUS_TXCOLL(x) ((x>>0)&0xf)
    #define GET_TX_STATUS_EXCOLL(x) ((x>>4)&0x1)
    #define GET_TX_STATUS_TXDEFER(x) ((x>>5)&0x1)
    #define GET_TX_STATUS_PAUSED(x) ((x>>6)&0x1)
    #define SET_TX_STATUS_INTTX(upd) (upd<<7)
    #define GET_TX_STATUS_INTTX(x) ((x>>7)&0x1)
    #define GET_TX_STATUS_UNDER(x) ((x>>8)&0x1)
    #define GET_TX_STATUS_EXDEFER(x) ((x>>9)&0x1)
    #define GET_TX_STATUS_LOSTCRS(x) ((x>>10)&0x1)
    #define GET_TX_STATUS_TX10STAT(x) ((x>>11)&0x1)
    #define GET_TX_STATUS_LATECOLL(x) ((x>>12)&0x1)
    #define GET_TX_STATUS_TXPAR(x) ((x>>13)&0x1)
    #define SET_TX_STATUS_COMP(upd) (upd<<14)
    #define GET_TX_STATUS_COMP(x) ((x>>14)&0x1)
    #define GET_TX_STATUS_TXHALTED(x) ((x>>15)&0x1)
    #define GET_TX_STATUS_SQERR(x) ((x>>16)&0x1)

    // MAC IF bits
    #define SET_TX_STATUS_TXLENGTH(upd) (upd<<17)
    #define GET_TX_STATUS_TXLENGTH(x) ((x>>17)&0x7ff)
    #define SET_TX_STATUS_TXFRMACTIVE(upd) (upd<<28)
    #define GET_TX_STATUS_TXFRMACTIVE(x) ((x>>28)&0x1)
    #define GET_TX_STATUS_TXDATARDY(x) ((x>>29)&0x1)

  REG32 TXCTL_STATUS; // 0x108
    #define GET_TXCTL_STATUS_STATUS(x) ((x>>0)&0x1ffff)
   
    // Bitfields are the same as in TX_STATUS

    #define GET_TXCTL_STATUS_LAUNCHEN(x) ((x>>17)&0x1)
    #define SET_TXCTL_STATUS_LAUNCHEN(upd) (upd<<17)

  REG32 RX_DATA;      // 0x10c
  REG32 RX_STATUS;    // 0x110
    #define GET_RX_STATUS_STATUS(x) ((x>>0)&0xffff)

    // MAC RXSTAT bits
    #define SET_RX_STATUS_CTLRECD(upd) (upd<<5)
    #define GET_RX_STATUS_CTLRECD(x) ((x>>5)&0x1)
    #define SET_RX_STATUS_INTRX(upd) (upd<<6)
    #define GET_RX_STATUS_INTRX(x) ((x>>6)&0x1)
    #define GET_RX_STATUS_RX10STAT(x) ((x>>7)&0x1)
    #define GET_RX_STATUS_ALIGNERR(x) ((x>>8)&0x1)
    #define GET_RX_STATUS_CRCERR(x) ((x>>9)&0x1)
    #define GET_RX_STATUS_OVERFLOW(x) ((x>>10)&0x1)
    #define GET_RX_STATUS_LONGERR(x) ((x>>11)&0x1)
    #define GET_RX_STATUS_PXPAR(x) ((x>>13)&0x1)
    #define SET_RX_STATUS_GOOD(upd) (upd<<14)
    #define GET_RX_STATUS_GOOD(x) ((x>>14)&0x1)
    #define GET_RX_STATUS_RXHALTED(x) ((x>>15)&0x1)

    #define SET_RX_STATUS_RXLENGTH(upd) (upd<<16)
    #define GET_RX_STATUS_RXLENGTH(x) ((x>>16)&0x7ff)
    #define SET_RX_STATUS_RXFRMACTIVE(upd) (upd<<27)
    #define GET_RX_STATUS_RXFRMACTIVE(x) ((x>>27)&0x1)
    #define GET_RX_STATUS_RXDATARDY(x) ((x>>28)&0x1)
  REG32 RX_DETECT_LEN; // 0x114
    #define GET_RX_DETECT_LEN(x) ((x>>0)&0x3f)
    #define SET_RX_DETECT_LEN(upd) (upd<<0)

  // TODO: Add remaining bits
  // RX_DREQ, RX_EOF
  // RX_DACK

  REG32 __pad2[58];

  // CAM (index 0,1,2,27,28 are valid, all others reads to 0)
  REG32 CAM[32];       // 0x200

  REG32 __pad3[32];

  // Testbench
  REG32 DMA_CTL;       // 0x300
    #define DMA_RX_EMPTY (1<<0)
    #define DMA_TX_EMPTY (1<<1)
    #define DMA_PAR_ERR  (1<<2)  // 1 to enable
    #define CPU_BUS_REQ  (1<<3)  // 1 to enable cpu bus request
  REG32 PHY_CRS;       // 0x304
    #define SET_PHY_CRS_PHY_EN(upd) (upd<<0)
    #define SET_PHY_CRS_PHY_10EN(upd) (upd<<1)
  REG32 TXD_XO;
    #define GET_TXD_XO(x) ((x>>0)&0xf)
  REG32 VERSION;
  
} tETHERNET;

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

typedef struct _tAVE
{
  // RAM0
  REG32 VMEM[1728];
  REG32 MBDSCR[64];
  REG32 BSDBUF[128];
  REG32 MESCR[64];
  REG32 HITMAP[8];
  REG32 MVLIST[8];
  REG32 STAT[8];
  REG32 _PAD0[2048-1728-2*64-128-3*8];
  
  // RAM1
  REG32 BLOCK[6][64];
  REG32 TRANSPOSE[64];
  REG32 ZRL[6][4];
  REG32 _PAD1[32-24];
  REG32 H263QINV[2][2];
  REG32 H263Q[2][2];
  REG32 DCPRED[6];
  REG32 _PAD2[2];
  REG32 DCLAST[3];
  REG32 _PAD3[32-19];
  REG32 JPEGQ[2][64];
  REG32 HUFFMAN[344];  
  REG32 _PAD4[512-472];
  
  // ME registers
  REG32 MEINSTR;
  REG32 MESAD;
  REG32 MEMIN;
  REG32 MECTRL;
  REG32 _PAD5[8-4];

  // BP registers
  REG32 BPINSTR;
  REG32 BPSTATUS;
  REG32 BPCTRL;
  REG32 BPMCUWI;
  REG32 _PAD6[8-4];

  // MBDMA registers
  REG32 MBDCTRL;
  REG32 MBDSCRIPT;
  REG32 MBDINSTR0;
  REG32 MBDINSTR1;
  REG32 _PAD7[4];

  // BSDMA registers
  REG32 BSDADDR;
  REG32 BSDINIT;
  REG32 BSDVLC;
  REG32 BSDCTRL;
  REG32 _PAD8[1];
 
  // AVE general registers
  REG32 BMUCTRL;
  REG32 VERSION;
  REG32 IRQCTRL;
} tAVE;

//-----------------------------
// FPGA specific registers
//-----------------------------

typedef struct _tFPGA
{
  REG32 VERSION;
  REG32 MAGIC;
} tFPGA;

//-----------------------------
// GPIO pin numbers
//-----------------------------

// TODO: Change
#define GPIO_UART0_RXD      0  // IN_PU    
#define GPIO_UART1_RXD      1  // IN_PU    
#define GPIO_UART0_TXD      2  // IN_PU    
#define GPIO_UART1_TXD      3  // IN_PU    
#define GPIO_UART0_CTS      4  // IN_PU    
#define GPIO_UART0_RTS      5  // IN_PU    
#define GPIO_6              6  // IN_PU    
#define GPIO_7              7  // IN_PU    
#define GPIO_8              8  // IN_PU    
#define GPIO_SSI_CLK        9  // IN_PU    
#define GPIO_SSI_FRM        10 // IN_PU    
#define GPIO_SSI_TX         11 // IN_PU    
#define GPIO_SSI_RX         12 // IN_PU    
#define GPIO_VAD0           13 // IN_PU    
#define GPIO_VAD1           14 // IN_PU    
#define GPIO_VAD2           15 // IN_PU         
#define GPIO_VAD3           16 // IN_PU         
#define GPIO_VAD4           17 // IN_PU         
#define GPIO_VAD5           18 // IN_PU         
#define GPIO_VAD6           19 // IN_PU         
#define GPIO_VAD7           20 // IN_PU         
#define GPIO_VCLK_OUT       21 // IN_PU         
#define GPIO_VCLK           22 // IN_PU         
#define GPIO_VAD8           23 // IN_PU         
#define GPIO_VAD9           24 // IN_PU         
#define GPIO_VFST           25 // IN_PU    
#define GPIO_VPIX           26 // IN_PU         
#define GPIO_VIDEO0         27 // IN_PU         
#define GPIO_VIDEO1         28 // IN_PU         
#define GPIO_VIDEO2         29 // IN_PU         
#define GPIO_VIDEO3         30 // IN_PU         
#define GPIO_DATA16         31 // ALT_FUNC
#define GPIO_DATA17         32 // ALT_FUNC
#define GPIO_DATA18         33 // ALT_FUNC
#define GPIO_DATA19         34 // ALT_FUNC
#define GPIO_DATA20         35 // ALT_FUNC
#define GPIO_DATA21         36 // ALT_FUNC
#define GPIO_DATA22         37 // ALT_FUNC
#define GPIO_DATA23         38 // ALT_FUNC
#define GPIO_DATA24         39 // ALT_FUNC
#define GPIO_DATA25         40 // ALT_FUNC
#define GPIO_DATA26         41 // ALT_FUNC
#define GPIO_DATA27         42 // ALT_FUNC
#define GPIO_DATA28         43 // ALT_FUNC
#define GPIO_DATA29         44 // ALT_FUNC
#define GPIO_DATA30         45 // ALT_FUNC
#define GPIO_DATA31         46 // ALT_FUNC
#define GPIO_ADDR2          47 // ALT_FUNC 
#define GPIO_ADDR3          48 // ALT_FUNC 
#define GPIO_ADDR4          49 // ALT_FUNC 
#define GPIO_ADDR5          50 // ALT_FUNC 
#define GPIO_ADDR22         51 // ALT_FUNC    
#define GPIO_ADDR23         52 // IN_PU    
#define GPIO_ADDR24         53 // IN_PU    
#define GPIO_SDCKE          54 // ALT_FUNC
#define GPIO_NAND_nRE       55 // IN_PU         
#define GPIO_NAND_nWE       56 // IN_PU         
#define GPIO_nCS0           57 // ALT_FUNC 
#define GPIO_nCS1           58 // ALT_FUNC 
#define GPIO_nCS2           59 // ALT_FUNC 
#define GPIO_nCS3           60 // ALT_FUNC 
#define GPIO_nCS4           61 // ALT_FUNC 
#define GPIO_nCS5           62 // ALT_FUNC 
#define GPIO_nWAIT          63 // IN_PU         
#define GPIO_XREQ           64 // IN_PU    
#define GPIO_XGRANT         65 // IN_PU    
#define GPIO_CLKGEN         66 // IN_PU    
#define GPIO_DREQ0          67 // IN_PU    
#define GPIO_DREQ1          68 // IN_PU    
#define GPIO_PWM0           69 // IN_PU    
#define GPIO_PWM1           70 // IN_PU    
#define GPIO_nRESET_OUT     71 // ALT_FUNC 
#define GPIO_LED            72 // IN_PU    
#define GPIO_XI_COL         73 // IN_PU    
#define GPIO_TX_CLK_MII     74 // IN_PU         
#define GPIO_TXEN_XO        75 // IN_PU                       
#define GPIO_TXER_XO        76 // IN_PU                        
#define GPIO_XI_CRS_MII     77 // IN_PU                      
#define GPIO_TXD_XO0        78 // IN_PU                       
#define GPIO_TXD_XO1        79 // IN_PU                       
#define GPIO_TXD_XO2        80 // IN_PU                        
#define GPIO_TXD_XO3        81 // IN_PU                      
#define GPIO_RX_CLK_MII     82 // IN_PU                     
#define GPIO_PHYCLK_OUT     83 // IN_PU                     
#define GPIO_RXD_MII0       84 // IN_PU             
#define GPIO_RXD_MII1       85 // IN_PU             
#define GPIO_RXD_MII2       86 // IN_PU             
#define GPIO_RXD_MII3       87 // IN_PU             
#define GPIO_RX_DV_MII      88 // IN_PU                    
#define GPIO_RX_ER_MII      89 // IN_PU                      
#define GPIO_MDC            90 // IN_PU         
#define GPIO_MDIO           91 // IN_PU

#endif // ARGUS3_H
