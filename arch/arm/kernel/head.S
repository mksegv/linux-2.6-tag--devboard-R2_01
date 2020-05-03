/*
 *  linux/arch/arm/kernel/head.S
 *
 *  Copyright (C) 1994-2002 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Kernel startup code for all 32-bit CPUs
 */
#include <linux/config.h>
#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/domain.h>
#include <asm/mach-types.h>
#include <asm/procinfo.h>
#include <asm/ptrace.h>
#include <asm/constants.h>
#include <asm/thread_info.h>
#include <asm/system.h>

#define PROCINFO_MMUFLAGS	8
#define PROCINFO_INITFUNC	12

#define MACHINFO_TYPE		0
#define MACHINFO_PHYSRAM	4
#define MACHINFO_PHYSIO		8
#define MACHINFO_PGOFFIO	12
#define MACHINFO_NAME		16

#ifndef CONFIG_XIP_KERNEL
/*
 * We place the page tables 16K below TEXTADDR.  Therefore, we must make sure
 * that TEXTADDR is correctly set.  Currently, we expect the least significant
 * 16 bits to be 0x8000, but we could probably relax this restriction to
 * TEXTADDR >= PAGE_OFFSET + 0x4000
 *
 * Note that swapper_pg_dir is the virtual address of the page tables, and
 * pgtbl gives us a position-independent reference to these tables.  We can
 * do this because stext == TEXTADDR
 */
#if (TEXTADDR & 0xffff) != 0x8000
#error TEXTADDR must start at 0xXXXX8000
#endif

	.globl	swapper_pg_dir
	.equ	swapper_pg_dir, TEXTADDR - 0x4000

	.macro	pgtbl, rd, phys
	adr	\rd, stext
	sub	\rd, \rd, #0x4000
	.endm
#else
/*
 * XIP Kernel:
 *
 * We place the page tables 16K below DATAADDR.  Therefore, we must make sure
 * that DATAADDR is correctly set.  Currently, we expect the least significant
 * 16 bits to be 0x8000, but we could probably relax this restriction to
 * DATAADDR >= PAGE_OFFSET + 0x4000
 *
 * Note that pgtbl is meant to return the physical address of swapper_pg_dir.
 * We can't make it relative to the kernel position in this case since
 * the kernel can physically be anywhere.
 */
#if (DATAADDR & 0xffff) != 0x8000
#error DATAADDR must start at 0xXXXX8000
#endif

	.globl	swapper_pg_dir
	.equ	swapper_pg_dir, DATAADDR - 0x4000

	.macro	pgtbl, rd, phys
	ldr	\rd, =((DATAADDR - 0x4000) - VIRT_OFFSET)
	add	\rd, \rd, \phys
	.endm
#endif

/*
 * Kernel startup entry point.
 * ---------------------------
 *
 * This is normally called from the decompressor code.  The requirements
 * are: MMU = off, D-cache = off, I-cache = dont care, r0 = 0,
 * r1 = machine nr.
 *
 * This code is mostly position independent, so if you link the kernel at
 * 0xc0008000, you call this at __pa(0xc0008000).
 *
 * See linux/arch/arm/tools/mach-types for the complete list of machine
 * numbers for r1.
 *
 * We're trying to keep crap to a minimum; DO NOT add any machine specific
 * crap here - that's what the boot loader (or in extreme, well justified
 * circumstances, zImage) is for.
 */
	__INIT
	.type	stext, %function
ENTRY(stext)
#if defined(CONFIG_ARCH_ARGUS)
                mov     r12, r0

                @ Assume that there is no ROM filesystem in flash
                adr     r8, __romfs_data
                mov     r4, #0
                ldr     r5, [r8,#8] @ romfs in flash
                str     r4, [r5]
                mov     r1, #MACH_TYPE_ARGUS
                mov     r0, #0

                @ Search for ROM filesystem in flash
                @ location is provided by decompressor in r11
                adr     r8, __romfs_data
                ldr     r6, [r8, #12]   @ cramfs magic
                ldr     r2, [r11]       
                cmp     r6, r2          @ Check magic
                bne     _no_romfs_in_flash
                ldr     r3, [r11,#4]
                ldr     r4, [r8,#4]
                str     r3, [r4]        @ romfs length
                ldr     r4, [r8]
                str     r11, [r4]       @ romfs start
                mov     r3, #1
                ldr     r4, [r8,#8]
                str     r3, [r4]        @ romfs in flash = 1
_no_romfs_in_flash:     
#endif	
	msr	cpsr_c, #PSR_F_BIT | PSR_I_BIT | MODE_SVC @ ensure svc mode
						@ and irqs disabled
	bl	__lookup_processor_type		@ r5=procinfo r9=cpuid
	movs	r10, r5				@ invalid processor (r5=0)?
	beq	__error_p				@ yes, error 'p'
	bl	__lookup_machine_type		@ r5=machinfo
	movs	r8, r5				@ invalid machine (r5=0)?
	beq	__error_a			@ yes, error 'a'
	bl	__create_page_tables

	/*
	 * The following calls CPU specific code in a position independent
	 * manner.  See arch/arm/mm/proc-*.S for details.  r10 = base of
	 * xxx_proc_info structure selected by __lookup_machine_type
	 * above.  On return, the CPU will be ready for the MMU to be
	 * turned on, and r0 will hold the CPU control register value.
	 */
	ldr	r13, __switch_data		@ address to jump to after
						@ mmu has been enabled
	adr	lr, __enable_mmu		@ return (PIC) address
	add	pc, r10, #PROCINFO_INITFUNC

	.type	__switch_data, %object
__switch_data:
	.long	__mmap_switched
	.long	__data_loc			@ r4
	.long	__data_start			@ r5
	.long	__bss_start			@ r6
	.long	_end				@ r7
	.long	processor_id			@ r4
	.long	__machine_arch_type		@ r5
	.long	cr_alignment			@ r6
	.long	init_thread_union + THREAD_START_SP @ sp

#ifdef CONFIG_ARCH_ARGUS

#define CRAMFS_MAGIC 0x28cd3d45

                .type   __romfs_data, %object
__romfs_data:
                .long   romfs_start
                .long   romfs_length
                .long   romfs_in_flash
                .long   CRAMFS_MAGIC
#endif

/*
 * The following fragment of code is executed with the MMU on, and uses
 * absolute addresses; this is not position independent.
 *
 *  r0  = cp#15 control register
 *  r1  = machine ID
 *  r9  = processor ID
 */
	.type	__mmap_switched, %function
__mmap_switched:
#ifdef CONFIG_ARCH_ARGUS
	@ From the butt-ugly department
	@ (can we insert this somewhere else, really ? Some pseudo-
	@  bootloader?)
	adr	r3, __switch_data + 4
	ldmia	r3, {r4, r5, r6, r7}

        @ NOTE! The following code uses r6 and r7, but trashes
        @ r4-r5, so keep r3 intact and reload the above information
        @ at the end of the routine.

        adr     r8, __romfs_data
        
        @ Check if romfs in flash has been detected
        ldr     r4, [r8,#8] @ romfs in flash
        ldr     r5, [r4]
        cmp     r5, #0
        bne     _start_it

        @ Default if there is no romfs concatenated is 0 in romfs_length
        mov     r5, #0
        ldr     r4, [r8, #4]     @ _romfs_length
        str     r5, [r4]

        @ Check if there is a cramfs (magic value).
        @ Notice that we check for cramfs magic value - which is
        @ the rom fs we will possibly use in 2.4 if not JFFS (which does
        @ not need this mechanism anyway)

        @ r6 bss_start
        @ r7 _end
        
        @ the image will be after the vmlinux end address, in r6 (bss_start)
        ldr     r5, [r6]        @ cramfs assumes same endian on host/target
        ldr     r4, [r8, #12]   @ CRAMFS_MAGIC
        cmp     r5, r4          @ magic value in cramfs superblock
        bne     2f

        @ Ok. What is its size ? 
        
        ldr     r5, [r6, #4]    @ cramfs_super.size (again, no need to swapwb)

        @ We want to copy it to the end of the BSS

        @ Remember values so cramfs and setup can find this info
        ldr     r4, [r8]                @ romfs_start
        str     r7, [r4]                @ store r7(_end) in romfs_start
        ldr     r4, [r8, #4]            @ romfs_length
        str     r5, [r4]                @ store the length in romfs_length
         
        @ We need to copy it backwards, since they can be overlapping
        add     r5, r5,#3
        mov     r5, r5, lsr #2
        
        add     r6, r6, r5, lsl #2      @  src
        add     r7, r7, r5, lsl #2      @  dst

        @ Go ahead. Make my loop.

1:      ldr     r4, [r6,#-4]!
        str     r4, [r7,#-4]!
        subs    r5, r5, #1
        bne     1b              
         
2:      
        @ Dont worry that the BSS is tainted. It will be cleared later.

        mov     r5, #0
        ldr     r4, [r8, #8]            @ romfs_in_flash
        str     r5, [r4]

_start_it:      
        @ Reload the registers we used and continue.
#endif
	adr	r3, __switch_data + 4

	ldmia	r3!, {r4, r5, r6, r7}
	cmp	r4, r5				@ Copy data segment if needed
1:	cmpne	r5, r6
	ldrne	fp, [r4], #4
	strne	fp, [r5], #4
	bne	1b

	mov	fp, #0				@ Clear BSS (and zero fp)
1:	cmp	r6, r7
	strcc	fp, [r6],#4
	bcc	1b

	ldmia	r3, {r4, r5, r6, sp}
	str	r9, [r4]			@ Save processor ID
	str	r1, [r5]			@ Save machine type
	bic	r4, r0, #CR_A			@ Clear 'A' bit
	stmia	r6, {r0, r4}			@ Save control register values
	b	start_kernel



/*
 * Setup common bits before finally enabling the MMU.  Essentially
 * this is just loading the page table pointer and domain access
 * registers.
 */
	.type	__enable_mmu, %function
__enable_mmu:
#ifdef CONFIG_ALIGNMENT_TRAP
	orr	r0, r0, #CR_A
#else
	bic	r0, r0, #CR_A
#endif
#ifdef CONFIG_CPU_DCACHE_DISABLE
	bic	r0, r0, #CR_C
#endif
#ifdef CONFIG_CPU_BPREDICT_DISABLE
	bic	r0, r0, #CR_Z
#endif
#ifdef CONFIG_CPU_ICACHE_DISABLE
	bic	r0, r0, #CR_I
#endif
	mov	r5, #(domain_val(DOMAIN_USER, DOMAIN_MANAGER) | \
		      domain_val(DOMAIN_KERNEL, DOMAIN_MANAGER) | \
		      domain_val(DOMAIN_TABLE, DOMAIN_MANAGER) | \
		      domain_val(DOMAIN_IO, DOMAIN_CLIENT))
	mcr	p15, 0, r5, c3, c0, 0		@ load domain access register
	mcr	p15, 0, r4, c2, c0, 0		@ load page table pointer
	b	__turn_mmu_on

/*
 * Enable the MMU.  This completely changes the structure of the visible
 * memory space.  You will not be able to trace execution through this.
 * If you have an enquiry about this, *please* check the linux-arm-kernel
 * mailing list archives BEFORE sending another post to the list.
 *
 *  r0  = cp#15 control register
 *  r13 = *virtual* address to jump to upon completion
 *
 * other registers depend on the function called upon completion
 */
	.align	5
	.type	__turn_mmu_on, %function
__turn_mmu_on:
	mov	r0, r0
	mcr	p15, 0, r0, c1, c0, 0		@ write control reg
	mrc	p15, 0, r3, c0, c0, 0		@ read id reg
	mov	r3, r3
	mov	r3, r3
	mov	pc, r13



/*
 * Setup the initial page tables.  We only setup the barest
 * amount which are required to get the kernel running, which
 * generally means mapping in the kernel code.
 *
 * r8  = machinfo
 * r9  = cpuid
 * r10 = procinfo
 *
 * Returns:
 *  r0, r3, r5, r6, r7 corrupted
 *  r4 = physical page table address
 */
	.type	__create_page_tables, %function
__create_page_tables:
	ldr	r5, [r8, #MACHINFO_PHYSRAM]	@ physram
	pgtbl	r4, r5				@ page table address

	/*
	 * Clear the 16K level 1 swapper page table
	 */
	mov	r0, r4
	mov	r3, #0
	add	r6, r0, #0x4000
1:	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	teq	r0, r6
	bne	1b

	ldr	r7, [r10, #PROCINFO_MMUFLAGS]	@ mmuflags

	/*
	 * Create identity mapping for first MB of kernel to
	 * cater for the MMU enable.  This identity mapping
	 * will be removed by paging_init().  We use our current program
	 * counter to determine corresponding section base address.
	 */
	mov	r6, pc, lsr #20			@ start of kernel section
	orr	r3, r7, r6, lsl #20		@ flags + kernel base
	str	r3, [r4, r6, lsl #2]		@ identity mapping
#ifdef CONFIG_ARCH_ARGUS
        /* TODO:         remove! iospace. */
	/* probably looks better if we can move this further below
	 * where the other machs define their iospace mapping
	 */
        mov     r6, #0x20000000
	add     r0, r6, #0x3c00
	add     r0, r0, #0x12
        str     r0, [r4, r6, lsr #18]           @ identity mapping
        mov     r6, #0xe4000000
        str     r0, [r4, r6, lsr #18]           @ "real" mapping...
                
        mov     r6, #0x40000000
        add     r0, r6, #0x3c00
        add     r0, r0, #0x12
        mov     r6, #0xe4000000
        add     r6, r6, #0x00300000
        str     r0, [r4, r6, lsr #18]           @ L2CACHE_CTRL.
#endif /* CONFIG_ARCH_ARGUS */
	
	/*
	 * Now setup the pagetables for our kernel direct
	 * mapped region.  We round TEXTADDR down to the
	 * nearest megabyte boundary.  It is assumed that
	 * the kernel fits within 4 contigous 1MB sections.
	 */
	add	r0, r4,  #(TEXTADDR & 0xff000000) >> 18	@ start of kernel
	str	r3, [r0, #(TEXTADDR & 0x00f00000) >> 18]!
#ifndef CONFIG_ARCH_ARGUS
	add	r3, r3, #1 << 20
	str	r3, [r0, #4]!			@ KERNEL + 1MB
	add	r3, r3, #1 << 20
	str	r3, [r0, #4]!			@ KERNEL + 2MB
	add	r3, r3, #1 << 20
	str	r3, [r0, #4]!			@ KERNEL + 3MB
#else
	/* Our kernel-images + romfs_in_ram + VRAM can be quite large */
	mov	r6, #12
1:	add	r3, r3, #1 << 20
	str	r3, [r0, #4]!
	subs	r6, r6, #1
	bne	1b
#endif
	/*
	 * Then map first 1MB of ram in case it contains our boot params.
	 */
	add	r0, r4, #VIRT_OFFSET >> 18
	orr	r6, r5, r7
	str	r6, [r0]

#ifdef CONFIG_XIP_KERNEL
	/*
	 * Map some ram to cover our .data and .bss areas.
	 * Mapping 3MB should be plenty.
	 */
	sub	r3, r4, r5
	mov	r3, r3, lsr #20
	add	r0, r0, r3, lsl #2
	add	r6, r6, r3, lsl #20
	str	r6, [r0], #4
	add	r6, r6, #(1 << 20)
	str	r6, [r0], #4
	add	r6, r6, #(1 << 20)
	str	r6, [r0]
#endif

	bic	r7, r7, #0x0c			@ turn off cacheable
						@ and bufferable bits
#ifdef CONFIG_DEBUG_LL
	/*
	 * Map in IO space for serial debugging.
	 * This allows debug messages to be output
	 * via a serial console before paging_init.
	 */
	ldr	r3, [r8, #MACHINFO_PGOFFIO]
	add	r0, r4, r3
	rsb	r3, r3, #0x4000			@ PTRS_PER_PGD*sizeof(long)
	cmp	r3, #0x0800			@ limit to 512MB
	movhi	r3, #0x0800
	add	r6, r0, r3
	ldr	r3, [r8, #MACHINFO_PHYSIO]
	orr	r3, r3, r7
1:	str	r3, [r0], #4
	add	r3, r3, #1 << 20
	teq	r0, r6
	bne	1b
#if defined(CONFIG_ARCH_NETWINDER) || defined(CONFIG_ARCH_CATS)
	/*
	 * If we're using the NetWinder, we need to map in
	 * the 16550-type serial port for the debug messages
	 */
	teq	r1, #MACH_TYPE_NETWINDER
	teqne	r1, #MACH_TYPE_CATS
	bne	1f
	add	r0, r4, #0x3fc0			@ ff000000
	mov	r3, #0x7c000000
	orr	r3, r3, r7
	str	r3, [r0], #4
	add	r3, r3, #1 << 20
	str	r3, [r0], #4
1:
#endif
#endif
#ifdef CONFIG_ARCH_RPC
	/*
	 * Map in screen at 0x02000000 & SCREEN2_BASE
	 * Similar reasons here - for debug.  This is
	 * only for Acorn RiscPC architectures.
	 */
	add	r0, r4, #0x80			@ 02000000
	mov	r3, #0x02000000
	orr	r3, r3, r7
	str	r3, [r0]
	add	r0, r4, #0x3600			@ d8000000
	str	r3, [r0]
#endif
	mov	pc, lr
	.ltorg



/*
 * Exception handling.  Something went wrong and we can't proceed.  We
 * ought to tell the user, but since we don't have any guarantee that
 * we're even running on the right architecture, we do virtually nothing.
 *
 * If CONFIG_DEBUG_LL is set we try to print out something about the error
 * and hope for the best (useful if bootloader fails to pass a proper
 * machine ID for example).
 */

	.type	__error_p, %function
__error_p:
#ifdef CONFIG_DEBUG_LL
	adr	r0, str_p1
	bl	printascii
	b	__error
str_p1:	.asciz	"\nError: unrecognized/unsupported processor variant.\n"
	.align
#endif

	.type	__error_a, %function
__error_a:
#ifdef CONFIG_DEBUG_LL
	mov	r4, r1				@ preserve machine ID
	adr	r0, str_a1
	bl	printascii
	mov	r0, r4
	bl	printhex8
	adr	r0, str_a2
	bl	printascii
	adr	r3, 3f
	ldmia	r3, {r4, r5, r6}		@ get machine desc list
	sub	r4, r3, r4			@ get offset between virt&phys
	add	r5, r5, r4			@ convert virt addresses to
	add	r6, r6, r4			@ physical address space
1:	ldr	r0, [r5, #MACHINFO_TYPE]	@ get machine type
	bl	printhex8
	mov	r0, #'\t'
	bl	printch
	ldr     r0, [r5, #MACHINFO_NAME]	@ get machine name
	add	r0, r0, r4
	bl	printascii
	mov	r0, #'\n'
	bl	printch
	add	r5, r5, #SIZEOF_MACHINE_DESC	@ next machine_desc
	cmp	r5, r6
	blo	1b
	adr	r0, str_a3
	bl	printascii
	b	__error
str_a1:	.asciz	"\nError: unrecognized/unsupported machine ID (r1 = 0x"
str_a2:	.asciz	").\n\nAvailable machine support:\n\nID (hex)\tNAME\n"
str_a3:	.asciz	"\nPlease check your kernel config and/or bootloader.\n"
	.align
#endif

	.type	__error, %function
__error:
#ifdef CONFIG_ARCH_RPC
/*
 * Turn the screen red on a error - RiscPC only.
 */
	mov	r0, #0x02000000
	mov	r3, #0x11
	orr	r3, r3, r3, lsl #8
	orr	r3, r3, r3, lsl #16
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
#endif
1:	mov	r0, r0
	b	1b


/*
 * Read processor ID register (CP#15, CR0), and look up in the linker-built
 * supported processor list.  Note that we can't use the absolute addresses
 * for the __proc_info lists since we aren't running with the MMU on
 * (and therefore, we are not in the correct address space).  We have to
 * calculate the offset.
 *
 * Returns:
 *	r3, r4, r6 corrupted
 *	r5 = proc_info pointer in physical address space
 *	r9 = cpuid
 */
	.type	__lookup_processor_type, %function
__lookup_processor_type:
	adr	r3, 3f
	ldmda	r3, {r5, r6, r9}
	sub	r3, r3, r9			@ get offset between virt&phys
	add	r5, r5, r3			@ convert virt addresses to
	add	r6, r6, r3			@ physical address space
	mrc	p15, 0, r9, c0, c0		@ get processor id
1:	ldmia	r5, {r3, r4}			@ value, mask
	and	r4, r4, r9			@ mask wanted bits
	teq	r3, r4
	beq	2f
	add	r5, r5, #PROC_INFO_SZ		@ sizeof(proc_info_list)
	cmp	r5, r6
	blo	1b
	mov	r5, #0				@ unknown processor
2:	mov	pc, lr

/*
 * This provides a C-API version of the above function.
 */
ENTRY(lookup_processor_type)
	stmfd	sp!, {r4 - r6, r9, lr}
	bl	__lookup_processor_type
	mov	r0, r5
	ldmfd	sp!, {r4 - r6, r9, pc}

/*
 * Look in include/asm-arm/procinfo.h and arch/arm/kernel/arch.[ch] for
 * more information about the __proc_info and __arch_info structures.
 */
	.long	__proc_info_begin
	.long	__proc_info_end
3:	.long	.
	.long	__arch_info_begin
	.long	__arch_info_end

/*
 * Lookup machine architecture in the linker-build list of architectures.
 * Note that we can't use the absolute addresses for the __arch_info
 * lists since we aren't running with the MMU on (and therefore, we are
 * not in the correct address space).  We have to calculate the offset.
 *
 *  r1 = machine architecture number
 * Returns:
 *  r3, r4, r6 corrupted
 *  r5 = mach_info pointer in physical address space
 */
	.type	__lookup_machine_type, %function
__lookup_machine_type:
	adr	r3, 3b
	ldmia	r3, {r4, r5, r6}
	sub	r3, r3, r4			@ get offset between virt&phys
	add	r5, r5, r3			@ convert virt addresses to
	add	r6, r6, r3			@ physical address space
1:	ldr	r3, [r5, #MACHINFO_TYPE]	@ get machine type
	teq	r3, r1				@ matches loader number?
	beq	2f				@ found
	add	r5, r5, #SIZEOF_MACHINE_DESC	@ next machine_desc
	cmp	r5, r6
	blo	1b
	mov	r5, #0				@ unknown machine
2:	mov	pc, lr

/*
 * This provides a C-API version of the above function.
 */
ENTRY(lookup_machine_type)
	stmfd	sp!, {r4 - r6, lr}
	mov	r1, r0
	bl	__lookup_machine_type
	mov	r0, r5
	ldmfd	sp!, {r4 - r6, pc}
