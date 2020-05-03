/*
 * Copyright (C) 2003, Axis Communications AB.
 */

#include <linux/config.h>
#include <linux/ptrace.h>
#include <asm/uaccess.h>

#include <asm/arch/hwregs/supp_reg.h>

extern void reset_watchdog(void);
extern void stop_watchdog(void);

void
show_registers(struct pt_regs *regs)
{
	/*
	 * It's possible to use either the USP register or current->thread.usp.
	 * USP might not correspond to the current proccess for all cases this
	 * function is called, and current->thread.usp isn't up to date for the
	 * current proccess. Experience shows that using USP is the way to go.
	 */
	unsigned long usp;
	unsigned long d_mmu_cause;
	unsigned long i_mmu_cause;

	usp = rdusp();

	printk("CPU: %d\n", smp_processor_id());
        
	printk("ERP: %08lx SRP: %08lx  CCS: %08lx USP: %08lx MOF: %08lx\n",
		regs->erp, regs->srp, regs->ccs, usp, regs->mof);
	
	printk(" r0: %08lx  r1: %08lx   r2: %08lx  r3: %08lx\n",
		regs->r0, regs->r1, regs->r2, regs->r3);
	
	printk(" r4: %08lx  r5: %08lx   r6: %08lx  r7: %08lx\n",
		regs->r4, regs->r5, regs->r6, regs->r7);
	
	printk(" r8: %08lx  r9: %08lx  r10: %08lx r11: %08lx\n",
		regs->r8, regs->r9, regs->r10, regs->r11);

	printk("r12: %08lx r13: %08lx oR10: %08lx acr: %08lx\n",
		regs->r12, regs->r13, regs->orig_r10, regs->acr);

	printk("sp: %08lx\n", (unsigned long)regs);

	SUPP_BANK_SEL(BANK_IM);
	SUPP_REG_RD(RW_MM_CAUSE, i_mmu_cause);

	SUPP_BANK_SEL(BANK_DM);
	SUPP_REG_RD(RW_MM_CAUSE, d_mmu_cause);

	printk("       Data MMU Cause: %08lx\n", d_mmu_cause);
	printk("Instruction MMU Cause: %08lx\n", i_mmu_cause);

	printk("Process %s (pid: %d, stackpage: %08lx)\n",
		current->comm, current->pid, (unsigned long) current);

	/* Show additional info if in kernel-mode. */
	if (!user_mode(regs)) {
		int i;
		unsigned char c;

		show_stack(NULL, (unsigned long *) usp);
		
		/*
		 * If the previous stack-dump wasn't a kernel one, dump the
		 * kernel stack now.
		 */
		if (usp != 0)
			show_stack(NULL, NULL);

		printk("\nCode: ");

		if (regs->erp < PAGE_OFFSET)
			goto bad_value;

		/*
		 * Quite often the value at regs->erp doesn't point to the
		 * interesting instruction, which often is the previous
		 * instruction. So dump at an offset large enough that the
		 * instruction decoding should be in sync at the interesting
		 * point, but small enough to fit on a row. The regs->erp
		 * location is pointed out in a ksymoops-friendly way by
		 * wrapping the byte for that address in parenthesis.
		 */
		for (i = -12; i < 12; i++) {
			if (__get_user(c, &((unsigned char *) regs->erp)[i])) {
bad_value:
				printk(" Bad IP value.");
				break;
			}

			if (i == 0)
				printk("(%02x) ", c);
			else
				printk("%02x ", c);
		}

		printk("\n");
	}
}



void arch_enable_nmi(void)
{
	unsigned long flags;
	local_save_flags(flags);
	flags |= (1<<30); /* NMI M flag is at bit 30 */
	local_irq_restore(flags);
}
