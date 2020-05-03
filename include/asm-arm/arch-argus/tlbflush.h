/* Argus-specific tlb handling
 * Needed because Argus caches permission bits in the L1 cache so the cache
 * needs to be invalidated after TLB changes (which is different from normal v4)
 */

#ifndef ARCH_TLBFLUSH_H
#define ARCH_TLBFLUSH_H

#ifndef __ASSEMBLY__

static inline void flush_tlb_all(void)
{
	const int zero = 0;

	asm("mcr p15, 0, %0, c7, c10, 4" : : "r" (zero));  /* Drain WB */
	asm("mcr p15, 0, %0, c8, c7, 0" : : "r" (zero));   /* Invalidate TLB */
	asm("mcr p15, 0, %0, c7, c7, 0" : : "r" (zero));   /* Invalidate I/D cache */
}

static inline void flush_tlb_mm(struct mm_struct *mm)
{
	const int zero = 0;

	asm("mcr p15, 0, %0, c7, c10, 4" : : "r" (zero)); /* Drain WB */

	if (cpu_isset(smp_processor_id(), mm->cpu_vm_mask)) {
		asm("mcr%? p15, 0, %0, c8, c7, 0" : : "r" (zero)); /* Inval TLB */
		asm("mcr%? p15, 0, %0, c7, c7, 0" : : "r" (zero)); /* Inval I/D cache */
	}

}

static inline void
flush_tlb_page(struct vm_area_struct *vma, unsigned long uaddr)
{
	const int zero = 0;

	asm("mcr p15, 0, %0, c7, c10, 4" : : "r" (zero));  /* Drain WB */

	if (cpu_isset(smp_processor_id(), vma->vm_mm->cpu_vm_mask)) {
		asm("mcr%? p15, 0, %0, c8, c7, 0" : : "r" (zero)); /* Inval TLB */
		asm("mcr%? p15, 0, %0, c7, c7, 0" : : "r" (zero)); /* Inval I/D cache */
	}
}

static inline void flush_tlb_kernel_page(unsigned long kaddr)
{
	const int zero = 0;

	asm("mcr p15, 0, %0, c7, c10, 4" : : "r" (zero));  /* Drain WB */

	asm("mcr p15, 0, %0, c8, c7, 0" : : "r" (zero)); /* Inval TLB */
	asm("mcr p15, 0, %0, c7, c7, 0" : : "r" (zero)); /* Inval I/D cache */
}

/*
 *	flush_pmd_entry
 *
 *	Flush a PMD entry (word aligned, or double-word aligned) to
 *	RAM if the TLB for the CPU we are running on requires this.
 *	This is typically used when we are creating PMD entries.
 *
 *	clean_pmd_entry
 *
 *	Clean (but don't drain the write buffer) if the CPU requires
 *	these operations.  This is typically used when we are removing
 *	PMD entries.
 */

static inline void flush_pmd_entry(pmd_t *pmd)
{
	const unsigned int zero = 0;
	/* Just drain the wb */
	asm("mcr	p15, 0, %0, c7, c10, 4	@ flush_pmd"
	    : : "r" (zero));
}

static inline void clean_pmd_entry(pmd_t *pmd)
{
}

/* These two are in proc-ct200.S 
*/

extern void ct200_flush_user_tlb_range(unsigned long, unsigned long, struct vm_area_struct *);
extern void ct200_flush_kern_tlb_range(unsigned long, unsigned long);

#define flush_tlb_range(vma,start,end)	ct200_flush_user_tlb_range(start,end,vma)
#define flush_tlb_kernel_range(s,e)	ct200_flush_kern_tlb_range(s,e)

#endif /* __ASSEMBLY__ */

#endif
