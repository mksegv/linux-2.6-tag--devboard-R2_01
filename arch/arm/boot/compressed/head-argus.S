/*
 *  linux/arch/arm/boot/compressed/head-argus.S
 *
 *  Copyright (C) 2003 Axis Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_ARGUS_3
#include <asm/arch/argus3_physaddr.h>
#else
#include <asm/arch/argus2_physaddr.h>
#endif
#include <linux/config.h>

#define CS0_ADDR    (APB_NOCACHE + 0xa0000)
#define SDCTRL_ADDR (APB_NOCACHE + 0xa0000 + 32)
#define GPIO_ADDR   (APB_NOCACHE + 0x40000)

#ifdef CONFIG_ARGUS_DEBUG_PORT0
#define DEBUG_PORT 0
#elif defined(CONFIG_ARGUS_DEBUG_PORT1)
#define DEBUG_PORT 1
#else 
#define DEBUG_PORT -1
#endif
		
	.globl	gpio

	.text
start:
	b	code
	
	/* Header used by the bootstrapper to set up registers etc.
	 * Header must be at offset 0x14 to be detected by on-chip
	 * boot code.
	 * Never change the format of this header!
	 */
	.org	0x14
	.long 0xfedebeda @ magic for on chip boot ROM
	.long 0xa11a011e @ magic where argusboot fins parameters
#ifdef CONFIG_ARGUS_3
	.long 0x43023000
#else
	.long 0x43022000
#endif	
	.long 0x00000000
	.long 0xc0008000 @ start address
	.long 0x00000000 @ binary start
	.long 0x00000000 @ vmm_info
	.long CONFIG_ARGUS_CS0_CONFIG 
	.long CONFIG_ARGUS_CS1_CONFIG 
	.long CONFIG_ARGUS_CS2_CONFIG 
	.long CONFIG_ARGUS_CS3_CONFIG
	.long CONFIG_ARGUS_CS4_CONFIG
	.long CONFIG_ARGUS_CS5_CONFIG
	.long CONFIG_ARGUS_SDCTRL1
	.long CONFIG_ARGUS_SDCTRL2
	.long CONFIG_ARGUS_SDRFSH
	.long CONFIG_ARGUS_SDMODE
gpio:	
	.byte CONFIG_ARGUS_GPIO_0
	.byte CONFIG_ARGUS_GPIO_1
	.byte CONFIG_ARGUS_GPIO_2
	.byte CONFIG_ARGUS_GPIO_3
	.byte CONFIG_ARGUS_GPIO_4
	.byte CONFIG_ARGUS_GPIO_5
	.byte CONFIG_ARGUS_GPIO_6
	.byte CONFIG_ARGUS_GPIO_7
	.byte CONFIG_ARGUS_GPIO_8
	.byte CONFIG_ARGUS_GPIO_9
	.byte CONFIG_ARGUS_GPIO_10
	.byte CONFIG_ARGUS_GPIO_11
	.byte CONFIG_ARGUS_GPIO_12
	.byte CONFIG_ARGUS_GPIO_13
	.byte CONFIG_ARGUS_GPIO_14
	.byte CONFIG_ARGUS_GPIO_15	
	.byte CONFIG_ARGUS_GPIO_16
	.byte CONFIG_ARGUS_GPIO_17
	.byte CONFIG_ARGUS_GPIO_18
	.byte CONFIG_ARGUS_GPIO_19
	.byte CONFIG_ARGUS_GPIO_20
	.byte CONFIG_ARGUS_GPIO_21
	.byte CONFIG_ARGUS_GPIO_22
	.byte CONFIG_ARGUS_GPIO_23
	.byte CONFIG_ARGUS_GPIO_24
	.byte CONFIG_ARGUS_GPIO_25
	.byte CONFIG_ARGUS_GPIO_26
	.byte CONFIG_ARGUS_GPIO_27
	.byte CONFIG_ARGUS_GPIO_28
	.byte CONFIG_ARGUS_GPIO_29
	.byte CONFIG_ARGUS_GPIO_30
	.byte CONFIG_ARGUS_GPIO_31
	.byte CONFIG_ARGUS_GPIO_32
	.byte CONFIG_ARGUS_GPIO_33
	.byte CONFIG_ARGUS_GPIO_34
	.byte CONFIG_ARGUS_GPIO_35
	.byte CONFIG_ARGUS_GPIO_36
	.byte CONFIG_ARGUS_GPIO_37
	.byte CONFIG_ARGUS_GPIO_38
	.byte CONFIG_ARGUS_GPIO_39
	.byte CONFIG_ARGUS_GPIO_40
	.byte CONFIG_ARGUS_GPIO_41
	.byte CONFIG_ARGUS_GPIO_42
	.byte CONFIG_ARGUS_GPIO_43
	.byte CONFIG_ARGUS_GPIO_44
	.byte CONFIG_ARGUS_GPIO_45
	.byte CONFIG_ARGUS_GPIO_46
	.byte CONFIG_ARGUS_GPIO_47	
	.byte CONFIG_ARGUS_GPIO_48
	.byte CONFIG_ARGUS_GPIO_49
	.byte CONFIG_ARGUS_GPIO_50
	.byte CONFIG_ARGUS_GPIO_51
	.byte CONFIG_ARGUS_GPIO_52
	.byte CONFIG_ARGUS_GPIO_53
	.byte CONFIG_ARGUS_GPIO_54
	.byte CONFIG_ARGUS_GPIO_55
	.byte CONFIG_ARGUS_GPIO_56
	.byte CONFIG_ARGUS_GPIO_57
	.byte CONFIG_ARGUS_GPIO_58
	.byte CONFIG_ARGUS_GPIO_59
	.byte CONFIG_ARGUS_GPIO_60
	.byte CONFIG_ARGUS_GPIO_61
	.byte CONFIG_ARGUS_GPIO_62
	.byte CONFIG_ARGUS_GPIO_63
#ifdef CONFIG_ARGUS_3	
	.byte CONFIG_ARGUS_GPIO_64
	.byte CONFIG_ARGUS_GPIO_65	
	.byte CONFIG_ARGUS_GPIO_66
	.byte CONFIG_ARGUS_GPIO_67
	.byte CONFIG_ARGUS_GPIO_68
	.byte CONFIG_ARGUS_GPIO_69
	.byte CONFIG_ARGUS_GPIO_70
	.byte CONFIG_ARGUS_GPIO_71
	.byte CONFIG_ARGUS_GPIO_72
	.byte CONFIG_ARGUS_GPIO_73
	.byte CONFIG_ARGUS_GPIO_74
	.byte CONFIG_ARGUS_GPIO_75
	.byte CONFIG_ARGUS_GPIO_76
	.byte CONFIG_ARGUS_GPIO_77
	.byte CONFIG_ARGUS_GPIO_78
	.byte CONFIG_ARGUS_GPIO_79
	.byte CONFIG_ARGUS_GPIO_80
	.byte CONFIG_ARGUS_GPIO_81
	.byte CONFIG_ARGUS_GPIO_82
	.byte CONFIG_ARGUS_GPIO_83
	.byte CONFIG_ARGUS_GPIO_84
	.byte CONFIG_ARGUS_GPIO_85
	.byte CONFIG_ARGUS_GPIO_86
	.byte CONFIG_ARGUS_GPIO_87
	.byte CONFIG_ARGUS_GPIO_88
	.byte CONFIG_ARGUS_GPIO_89
	.byte CONFIG_ARGUS_GPIO_90
	.byte CONFIG_ARGUS_GPIO_91
#endif	
	.long DEBUG_PORT
header_end:	

code:
	/* Disable MMU */
	mrc	p15, 0, r3, c1, c0
	bic	r3, r3, #0xC			@ Write Buffer and DCache
	bic	r3, r3, #0x1000			@ ICache
	mcr	p15, 0, r3, c1, c0		@ disabled

	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		@ flush I,D caches on v4
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer on v4
	mcr	p15, 0, r0, c8, c7		@ flush I,D TLBs on v4

	bic	r3, r3, #0x1			@ MMU
	mcr	p15, 0, r3, c1, c0		@ disabled
		
	/* Set up memory configuration */
	ldr	r1, =CS0_ADDR
	ldr	r0, =CONFIG_ARGUS_CS0_CONFIG
	str	r0, [r1, #0x00]
	ldr	r0, =CONFIG_ARGUS_CS1_CONFIG
	str	r0, [r1, #0x04]
	ldr	r0, =CONFIG_ARGUS_CS2_CONFIG
	str	r0, [r1, #0x08]
	ldr	r0, =CONFIG_ARGUS_CS3_CONFIG
	str	r0, [r1, #0x0c]
	ldr	r0, =CONFIG_ARGUS_CS4_CONFIG
	str	r0, [r1, #0x10]
	ldr	r0, =CONFIG_ARGUS_CS5_CONFIG
	str	r0, [r1, #0x14]
	ldr	r1, =SDCTRL_ADDR
	ldr	r0, =CONFIG_ARGUS_SDCTRL1
	str	r0, [r1, #0x00]
	ldr	r0, =CONFIG_ARGUS_SDCTRL2
	str	r0, [r1, #0x04]
	ldr	r0, =CONFIG_ARGUS_SDRFSH
	str	r0, [r1, #0x08]
	ldr	r0, =CONFIG_ARGUS_SDMODE
	str	r0, [r1, #0x0c]
		
	/* Set up stack */
	ldr	sp, =user_stack

	/* Note that various scripts may have added headers */
	/* before this code so we may not be at the address */
	/* you would expect. The code below calculates the  */ 
	/* offset between current and expected address */
off:	mov	r4, pc
	sub	r4, r4, #8	@ Compensate for mov
	ldr	r5, =off
	ldr	r3, =0xffff
	and	r5, r5, r3      @ Strip off any chip selects etc
	ldr	r0, =EXT0_BASE
	add	r5, r5, r0	@ 'off' in flash
	sub	r5, r4, r5	@ Calculate the offset

	/* Copy text + data to DRAM */
	add	r0, r0, r5	@ Source address for copy
	ldr	r1, =_start	@ Destination
	ldr	r2, =_edata	@ End of destination
1:	ldr	r3, [r0], #4
	str	r3, [r1], #4
	cmp	r1, r2
	blo	1b

	/* Save the calculated offset (must be done after data copy) */
	ldr	r6, =offset
	str	r5, [r6]
	
	/* Zero BSS */
	ldr	r0, =__bss_start
	ldr	r1, =_end
	mov	r2, #0
1:	str	r2, [r0], #4
	cmp	r0, r1
	blo	1b
	
	/* Initialize hardware */
	bl	hw_init
	
	/* Do the decompression */
	ldr	r0, =0xc0008000		@ decompression dest address
	mov	r1, sp			@ malloc space above stack
	add	r2, r1, #0x10000	@ 64k max
	mov	r3, #0			@ arch id
	bl	decompress_kernel

	/* Store location of piggy-back filesystem for kernel */
	/* Assumptions:	 */
	/* 1:	_edata is the last symbol in the actual image */
	ldr	r0, =offset
	ldr	r0, [r0]
	ldr	r2, =EXT0_BASE
	ldr	r3, =start
	ldr	r11, =_edata
	add	r11, r11, r0
	sub	r11, r11, r3
	add	r11, r11, r2	
	
	/* Call kernel */
	ldr	r0, =0xc0008000
	mov	pc, r0

	.align
	.data
offset:	
	.long 0
		
	/* The stack */
	.align
	.section ".stack", "aw"
	.space 4096
user_stack:
	