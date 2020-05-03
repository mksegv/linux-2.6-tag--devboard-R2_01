/*
 *  linux/arch/arm/mm/ct200.S: MMU functions for ct200
 *
 *  Copyright (C) 1999,2000 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * These are the low level assembler for performing cache and TLB
 * functions on the ct200.
 */
#include <linux/linkage.h>
#include <linux/config.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/constants.h>
#include <asm/procinfo.h>
#include <asm/hardware.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/tlbflush.h>
#include "proc-macros.S"

/*

TODO:

- Se över användningen av B&C bitar (STREAM etc) (antagligen fixat!)

*/

/* The cpu idling should be disabled while debugging with the ICE because
   the ICE cannot break during the wait-for-irq instruction.
*/
	
#define CONFIG_CPU_CT200_CPU_IDLE
	
#define CONFIG_CPU_CT200_I_CACHE_ON
#define CONFIG_CPU_CT200_D_CACHE_ON
#define CONFIG_CPU_CT200_WB_ON

//#define CONFIG_CPU_CT200_TLB_OFF
//#define CONFIG_CPU_CT200_DISPATCH_DISABLE

/*
 * The size of one data cache line.
 */
#define CACHE_DLINESIZE	16
	
/*
 * This is the maximum size of an area which will be invalidated
 * using the single invalidate entry instructions.  Anything larger
 * than this, and we go for the whole cache.
 *
 * This value should be chosen such that we choose the cheapest
 * alternative.
 *
 * Since the cache is direct-mapped in Argus-3 and an entry-flush does not
 * make a difference of whether the entry is matching or not, if we loop
 * over 4 kbyte we will in effect invalidate the entire cache anyway. Thus
 * we should only use the single-entry invalidation for really small ranges.
 */
#define CACHE_DLIMIT	256

	.text
	
/*
 * ct200_proc_init()
 */
ENTRY(ct200_proc_init)
	mov	pc, lr

/*
 * ct200_proc_fin()
 */
ENTRY(ct200_proc_fin)
	stmfd	sp!, {lr}
	mov	ip, #PSR_F_BIT | PSR_I_BIT | SVC_MODE
	msr	cpsr_c, ip
	bl	ct200_flush_kern_cache_all	@ clean caches NOT NEEDED FOR WT RIGHT?
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1000			@ ...i............
	bic	r0, r0, #0x000e			@ ............wca.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldmfd	sp!, {pc}

/*
 * cpu_ct200_reset(loc)
 *
 * Perform a soft reset of the system.  Put the CPU into the
 * same state as it would be if it had been reset, and branch
 * to what would be the reset vector.
 *
 * loc: location to jump to for soft reset
 */
	.align	5
ENTRY(cpu_ct200_reset)
1000:	b       1000b   @ TODO:	 remove
	mov	ip, #0
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I,D caches
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
	mrc	p15, 0, ip, c1, c0, 0		@ ctrl register
	bic	ip, ip, #0x000f			@ ............wcam
	bic	ip, ip, #0x1100			@ ...i...s........
	mcr	p15, 0, ip, c1, c0, 0		@ ctrl register
	mov	pc, r0

/*
 * ct200_do_idle()
 */
	.align	5
ENTRY(ct200_do_idle)
#if defined(CONFIG_CPU_CT200_CPU_IDLE)
	mcr	p15, 0, r0, c15, c8, 2		@ Wait for interrupt
#endif
	mov	pc, lr

/* ================================= CACHE ================================ */

/*
 *	flush_user_cache_all()
 *
 *	Invalidate all cache entries in a particular address
 *	space.
 */
ENTRY(ct200_flush_user_cache_all)
	/* FALLTHROUGH */
/*
 *	flush_kern_cache_all()
 *
 *	Clean and invalidate the entire cache.
 */
ENTRY(ct200_flush_kern_cache_all)
	mov	r2, #VM_EXEC
	mov	ip, #0
__flush_whole_cache:
	mcr	p15, 0, ip, c7, c6, 0		@ invalidate D cache
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	flush_user_cache_range(start, end, flags)
 *
 *	Clean and invalidate a range of cache entries in the specified
 *	address space.
 *
 *	- start - start address (inclusive, page aligned)
 *	- end	- end address (exclusive, page aligned)
 *	- flags	- vma_area_struct flags describing address space
 */
ENTRY(ct200_flush_user_cache_range)
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #CACHE_DLIMIT
	bhs	__flush_whole_cache

1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	tst	r2, #VM_EXEC
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r1, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	coherent_kern_range(start, end)
 *
 *	Ensure coherency between the Icache and the Dcache in the
 *	region described by start.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
ENTRY(ct200_coherent_kern_range)
	/* FALLTHROUGH */

/*
 *	coherent_user_range(start, end)
 *
 *	Ensure coherency between the Icache and the Dcache in the
 *	region described by start.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
ENTRY(ct200_coherent_user_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
	/* TODO: check threshold and invalidate the entire I-cache possibly */
1:	mcr	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	pc, lr

/*
 *	flush_kern_dcache_page(void *page)
 *
 *	Ensure no D cache aliasing occurs, either with itself or
 *	the I cache
 *
 *	- addr	- page aligned address
 */
ENTRY(ct200_flush_kern_dcache_page)
	mov	r2, #0
	mcr	p15, 0, r2, c7, c5, 0		@ invalidate I cache
	add	r1, r0, #PAGE_SZ
	/* fallthrough */

/*
 *	dma_inv_range(start, end)
 *
 *	Invalidate (discard) the specified virtual address range.
 *	May not write back any entries.  If 'start' or 'end'
 *	are not cache line aligned, those lines must be written
 *	back.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
ENTRY(ct200_dma_inv_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
	@ check if we should invalidate the entire cache instead
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #CACHE_DLIMIT
	blo	1f
        mcr     p15, 0, ip, c7, c6, 0           @ invalidate D cache
	mcr	p15, 0, r1, c7, c10, 4		@ drain WB
	mov	pc, lr		
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	/* FALLTHROUGH */

/*
 *	dma_clean_range(start, end)
 *
 *	Clean the specified virtual address range.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
ENTRY(ct200_dma_clean_range)
	mcr	p15, 0, r1, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	dma_flush_range(start, end)
 *
 *	Clean and invalidate the specified virtual address range.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
	.globl	ct200_dma_flush_range
	.equ	ct200_dma_flush_range, ct200_dma_inv_range

/*
 * Perform necessary cache operations to ensure that the TLB will
 * see data written in the specified area.
 *
 * Writethrough cache so nothing to clean, just drain the WB.
 */

ENTRY(ct200_dcache_clean_area)
	mcr	p15, 0, r1, c7, c10, 4		@ drain WB
	mov	pc, lr

/* =============================== PageTable ============================== */

/*
 * ct200_switch_mm(pgd, mm)
 *
 * Set the translation base pointer to be as described by pgd.
 *
 * pgd: new page tables
 * mm: 
 */
	.align	5
ENTRY(ct200_switch_mm)
	mov	ip, #0
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I&D cache
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
	mcr	p15, 0, r0, c2, c0, 0		@ load page table pointer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
	mov	pc, lr

/*
 * cpu_ct200_set_pmd(pmdp, pmd)
 *
 * Set a level 1 translation table entry. We do our AP->EP translation here.
 *
 * pmdp: pointer to PMD entry
 * pmd:  PMD value to store
 */
	.align	5
ENTRY(ct200_set_pmd)

	and     r3, r1, #3
	cmp   	r3, #2
	bne	1f	@ jump if not section mapping

/*
  
  AP          |            SR SW UR UW
  ------------+-----------------------
  00  R       |  R         1000	 	(0x2000)
  01  RW      |  RW        1100		(0x3000)
  10  RW  R   |  R    R    1010		(0x2800)
  11  RW  RW  |  RW   RW   1111		(0x3c00)
  
*/ 		
  	mov     r3, r1, lsr #10
  	and     r3, r3, #3
  	bic	r1, r1, #0x3c00
  	cmp     r3, #0	@ AP==0
	orreq	r1, r1, #0x2000
  	cmp     r3, #1	@ AP==1
	orreq	r1, r1, #0x3000
  	cmp     r3, #2	@ AP==2
  	orreq	r1, r1, #0x2800
  	cmp     r3, #3	@ AP==3
  	orreq	r1, r1, #0x3c00
1:	
	
/* TODO: For now, avoid STREAM and BUFFER mapping. Convert to CACHE. */
	tst     r1, #0xc
	orrne	r1, r1, #0xc

	str	r1, [r0]
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 * ct200_set_pte(ptep, pte)
 *
 * Set a PTE and flush it out
 */
	.align	5
ENTRY(ct200_set_pte)
	str	r1, [r0], #-2048		@ linux version

	eor	r1, r1, #L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_WRITE | L_PTE_DIRTY

	bic	r2, r1, #PTE_SMALL_AP_MASK
	bic	r2, r2, #PTE_TYPE_MASK
	orr	r2, r2, #PTE_TYPE_SMALL

	/* TODO: check why arm920 doesnt use L_PTE_EXEC here! */
	tst	r1, #L_PTE_USER | L_PTE_EXEC	@ User or Exec?
	orrne	r2, r2, #PTE_SMALL_AP_URO_SRW

	tst	r1, #L_PTE_WRITE | L_PTE_DIRTY	@ Write and Dirty?
	orreq	r2, r2, #PTE_SMALL_AP_UNO_SRW

/*
  Translate from the v4 PTE AP bits to the extended permissions in
  Argus. This is transparent to the rest of the ARM-code.
	
  AP          |            SR SW UR UW
  ------------+-----------------------
  00  R       |  R         1000		0x80
  01  RW      |  RW        1100		0xc0
  10  RW  R   |  R    R    1010		0xa0
  11  RW  RW  |  RW   RW   1111		0xf0
  
*/
  	mov     r3, r2, lsr #4
  	and     r3, r3, #3
  	bic	r2, r2, #0xff0
  	cmp     r3, #0
  	/*orreq	r2, r2, #0xa0   @ should be #0x80*/
	orreq	r2, r2, #0x80
  	cmp     r3, #1
  	/*orreq	r2, r2, #0xf0   @ should be #0xc0*/
	orreq	r2, r2, #0xc0
  	cmp     r3, #2
  	orreq	r2, r2, #0xa0
  	cmp     r3, #3
  	orreq	r2, r2, #0xf0

/* TODO: For now, avoid STREAM and BUFFER mapping. Convert to CACHE. */
	tst	r2, #0xc
	orrne	r2, r2, #0xc /* always nocache or cache */

	tst	r1, #L_PTE_PRESENT | L_PTE_YOUNG	@ Present and Young?
	movne	r2, #0

	str	r2, [r0]			@ hardware version
	mov	r0, r0
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr



	.section ".text.init", #alloc, #execinstr

__ct200_setup:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		@ invalidate I,D caches on v4
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer on v4
	mcr	p15, 0, r0, c8, c7		@ invalidate I,D TLBs on v4
#if 0
	/* this is done in head.S */
	mcr	p15, 0, r4, c2, c0		@ load page table pointer

	mov	r0, #0x1f			@ Domains 0, 1 = client
	mcr	p15, 0, r0, c3, c0		@ load domain access register
#endif
	/* mrc	p15, 0, r0, c1, c0		@ get control register v4 */
/*
 * Clear out 'unwanted' bits (then put them in if we need them)
 * (we simply clear all, to avoid surprises from previous bootstrap code)
 */
	mov     r0, #0
						@   VI ZFRS BLDP WCAM
/*
 * Turn on what we want
 */
	orr	r0, r0, #0x0081
	orr	r0, r0, #0x2100			@ ..1. ...1 1... ...1   /* bit7: pagetable 2nd level cacheable */

#ifdef CONFIG_CPU_CT200_DISPATCH_DISABLE
	orr     r0, r0, #0x4000			@ uTLB miss to wb disable
#endif
#ifdef CONFIG_ALIGNMENT_TRAP
	orr	r0, r0, #0x0002                 @ alignment trap enable
#endif
#ifdef CONFIG_CPU_CT200_TLB_OFF
	orr     r0, r0, #0x0010			@ tlb disable
#endif
	orr     r0, r0, #0x8000			@ extended perm
#ifdef CONFIG_CPU_CT200_D_CACHE_ON
	orr	r0, r0, #0x0004			@ enable d cache
#endif
#ifdef CONFIG_CPU_CT200_I_CACHE_ON
	orr	r0, r0, #0x1000			@ enable i cache
#endif
#ifdef CONFIG_CPU_CT200_WB_ON
	orr	r0, r0, #0x0008			@ enable writebuffer
#endif

	mov	pc, lr

	.text

/* Debug function to dump the TLB contents into an array passed as
 * first argument.
 */
	
ENTRY(cpu_tlb_dump)
            mrc     p15,0,r1,c8,c0,0  @ r1= save index
            mov     r2,#0

1:          mcr     p15,0,r2,c8,c0,0  @ cp15.index= i
            mrc     p15,0,r3,c8,c0,1  @ r3= get_tag()
            str     r3,[r0],#4
            mrc     p15,0,r3,c8,c0,2  @ r3= get_data()            
            str     r3,[r0],#4
            add     r2,r2,#1
            cmp     r2,#32
            bne     1b
            
            mcr     p15,0,r1,c8,c0,0  @ restore index

            str     r1,[r0],#4        @ dump index

            mrc     p15,0,r1,c6,c0,0  @ FAR
            str     r1,[r0],#4        @ dump

            mrc     p15,0,r1,c5,c0,0  @ FSR
            str     r1,[r0],#4        @ dump
            
            mov     pc, lr

/*
 *	ct200_flush_user_tlb_range(start, end, mm)
 *
 *	Invalidate a range of TLB entries in the specified user address space.
 *
 *	- start - range start address
 *	- end   - range end address
 *	- mm    - mm_struct describing address space
 *
 * BW:	 since we don't have separate I/D TLB invalidate instructions we just
 *	 flush the entire TLB if we are running the same mm as the one that is
 *       flushed
 */
	.align	5
ENTRY(ct200_flush_user_tlb_range)
	vma_vm_mm ip, r2
	act_mm	r3				@ get current->active_mm
	eors	r3, ip, r3				@ == mm ?
	movne	pc, lr				@ no, we dont do anything
.ct200_flush_kern_tlb_range:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mcr	p15, 0, r0, c8, c7, 0		@ invalidate I & D TLBs
	/* since permission bits are cached in the L1 cache along with
	 * data, if a page table entry is changed, the I&D caches need
	 * to be invalidated too (this is Argus-specific behaviour)
	 */
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I&D cache

	mov	pc, lr

/*
 *	ct200_flush_kern_tlb_range(start, end)
 *
 *	Invalidate a range of TLB entries in the specified kernel
 *	address range.
 *
 *	- start - virtual address (may not be aligned)
 *	- end   - virtual address (may not be aligned)
 */
.globl ct200_flush_kern_tlb_range
.equ ct200_flush_kern_tlb_range, .ct200_flush_kern_tlb_range

	__INITDATA
#if 0
/* tlb functions */
	
	.type	ct200_tlb_fns, #object
ENTRY(ct200_tlb_fns)
	.long	v4uwt_flush_user_tlb_range
	.long	v4uwt_flush_kern_tlb_range
	.long	v4uwt_tlb_flags
	.size	v4uwt_tlb_fns, . - v4uwt_tlb_fns
#endif
/* cache functions */	
	
	.type	ct200_cache_fns, #object
ENTRY(ct200_cache_fns)
	.long	ct200_flush_kern_cache_all
	.long	ct200_flush_user_cache_all
	.long	ct200_flush_user_cache_range
	.long	ct200_coherent_kern_range
	.long	ct200_coherent_user_range
	.long	ct200_flush_kern_dcache_page
	.long	ct200_dma_inv_range
	.long	ct200_dma_clean_range
	.long	ct200_dma_flush_range
	.size	ct200_cache_fns, . - ct200_cache_fns
	
/*
 * Purpose : Function pointers used to access above functions - all calls
 *	     come through these
 */
	.type	ct200_processor_functions, #object
ct200_processor_functions:
	.word	v4t_early_abort
	.word	ct200_proc_init
	.word	ct200_proc_fin
	.word	cpu_ct200_reset
	.word   ct200_do_idle
	.word	ct200_dcache_clean_area
	.word	ct200_switch_mm
	.word	ct200_set_pte
	.size	ct200_processor_functions, . - ct200_processor_functions

	.section ".rodata"

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv4t"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v4"
	.size	cpu_elf_name, . - cpu_elf_name

	.type	cpu_ct200_name, #object
cpu_ct200_name:
	.ascii	"ARM/ANOTO CT200(wt)"
#if defined(CONFIG_CPU_CT200_CPU_IDLE)
	.ascii	"s"
#endif
#if defined(CONFIG_CPU_CT200_I_CACHE_ON)
	.ascii	"i"
#endif
#if defined(CONFIG_CPU_CT200_D_CACHE_ON)
	.ascii	"d"
#endif
	.ascii	"\0"
	.size	cpu_ct200_name, . - cpu_ct200_name
		
	.align

	.section ".proc.info", #alloc, #execinstr

	.type	__ct200_proc_info,#object
__ct200_proc_info:
	.long	0x43002000
	.long	0xff00eff0
	.long	0x00003c1e			@ mmuflags    /* section entry */
	b	__ct200_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP | HWCAP_HALF
	.long	cpu_ct200_name
	.long	ct200_processor_functions
	.long	0    /* ct200_tlb_fns, not used through the table */
	.long	v4wb_user_fns
	.long	v4wt_cache_fns
	.size	__ct200_proc_info, . - __ct200_proc_info
