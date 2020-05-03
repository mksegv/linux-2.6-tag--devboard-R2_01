 /* ------------------------------------------------------------------ */

 /*  broskfix.s */

 /*  Erik Persson, Anoto AB, 2003 */

 /* ------------------------------------------------------------------ */

 /*  Description: */
 /*    Low-level TLB miss handler that is executed before the ordinary */
 /*    abort handlers */

 /*  Usage: */
 /*    * Use CP15.CTRL as usual, however only EXT_PERM=1 is supported */
 /*    * Use broskfix_set_ttb instead of modifying CP15.TTB */
 /*    * Use broskfix_get_ttb instead of reading CP15.TTB */
 /*    * Use broskfix_flush_tlb instead of cp15 FlushTlb */
 /*    * Prefetch Abort currently trashes FAR and FSR */
 /*    * VASM must be set to 0xffffffff */

	.global broskfix_init
	.global broskfix_set_ttb
	.global broskfix_get_ttb
	.global broskfix_flush_tlb
	.global broskfix_data_abort
	.global broskfix_prefetch_abort

	.global broskfix_confirm_prefetch
	.global broskfix_data_check
	.global broskfix_prefetch_check
	.global broskfix_chain_cleanup

	.global load_page
	.global load_section
	.global miss_count
	.global physical_load

	.global last_fsr
	.global last_far
			
	.text

	.equ COUNT_MISSES,        1
	.equ SP_NO_MAGIC,         0x00000000
	.equ SP_CHAIN_MAGIC,      0x10000001
	.equ SP_CONFIRM_MAGIC,    0x20000002
	.equ SECTION_TRANS_FAULT, 0x5
	.equ PAGE_TRANS_FAULT,    0x7
	.equ ZEROS_BASE,          0x30000000

 /* ------------------------------------------------------------------ */
 /*  broskfix_init */
 /* ------------------------------------------------------------------ */

broskfix_init:  ldr     r1,=backup_regs

		mrc     p15,0,r0,c2,c0,0      /*  read translation table base */
		str     r0,[r1, #table_base-backup_regs]

		mrc     p15,0,r0,c6,c0,0      /*  Copy FAR */
		str     r0,[r1, #last_far-backup_regs]

		mrc     p15,0,r0,c5,c0,0      /*  Copy FSR */
		str     r0,[r1, #last_fsr-backup_regs]

		mov     r0,#0xffffffff
		mcr     p15,0,r0,c6,c0,0      /*  Set VASM */

		/*  Reset TLB miss handler state in SP_Abt */
		mov     r0,#0xd7
		msr     cpsr_c,r0            /*  MODE= ABT */
		mov     sp,#SP_NO_MAGIC      /*  SP_abt= SP_NO_MAGIC */
		mov     r0,#0xd3
		msr     cpsr_c,r0            /*  MODE= SVC                 */

		b       broskfix_load_tlb    /*  load tlb and enter manual-fetch mode */

/* ------------------------------------------------------------------ */
 /*  broskfix_set_ttb */
 /* ------------------------------------------------------------------ */
 /*  Used this instead of setting cp15.SetTTB() */
 /*  r0 - Table base (physical address) */

broskfix_set_ttb:       
		ldr     r1,=table_base
		str     r0,[r1]
				
		mrc     p15,0,r1,c2,c0,0  
		cmp     r1,#ZEROS_BASE
		mcrne   p15,0,r0,c2,c0,0     /*  fix disabled => set hardware TTB */
		movne   pc,lr                   /*  fix disabled => return */

		 /*  fix enabled => Fall through into broskfix_flush_tlb */
		
 /* ------------------------------------------------------------------ */
 /*  broskfix_flush_tlb */
 /* ------------------------------------------------------------------ */
 /*  Used this instead of cp15.FlushTlb() */

broskfix_flush_tlb:     
		mrc     p15,0,r1,c2,c0,0  
		cmp     r1,#ZEROS_BASE       /*  fix enabled? */
		mcrne   p15,0,r1,c8,c7,0     /*  no => cp15.FlushTlb() */
		movne   pc,lr                /*  no => return */

		mrs     r3,cpsr              /*  r3= cpsr */
		orr     r0,r3,#0xc0          /*  irqfiq_off */
		msr     cpsr_c,r0                

		 /*  Avoid sending TLB-miss Cache-hit writes to /dev/null */
		mcr     p15,0,r0,c7,c10,4   /*  cp15.DrainWb() */
		mcr     p15,0,r0,c7,c6,0    /*  cp15.FlushDCache */

		 /*  Clear TLB. 14*29/2 = 203 cycles */
		ldr     r2,=base_index
		ldr     r0,[r2]              /*  r0=base_index */
		mov     r1,#0
		tst     r0,#1
		bne     02f

01:             mcr     p15,0,r0,c8,c0,0     /*  Write TLB index */
		mcr     p15,0,r1,c8,c0,1     /*  Write TLB tag */
		add     r0,r0,#1            
		
02:             mcr     p15,0,r0,c8,c0,0     /*  Write TLB index */
		mcr     p15,0,r1,c8,c0,1     /*  Write TLB tag */
		add     r0,r0,#1                        
		
		cmp     r0,#32
		bne     01b

		msr     cpsr_c, r3           /*  irqfiq_set */
		mov     pc,lr                /*  Return                  */

 /* ------------------------------------------------------------------ */
 /*  broskfix_get_ttb */
 /* ------------------------------------------------------------------ */
 /*  Used this instead of setting cp15.GetTTB() */
 /*  Returns table base (physical address) */

broskfix_get_ttb:
		mrc     p15,0,r0,c2,c0,0
		cmp     r0,#ZEROS_BASE       /*  fix enabled? */
		ldreq   r0,=table_base
		ldreq   r0,[r0]              /*  => read global variable instead */
		mov     pc,lr

 /* ------------------------------------------------------------------ */
 /*  broskfix_load_tlb */
 /* ------------------------------------------------------------------ */

broskfix_load_tlb:
		mrs     r3,cpsr              /*  r3= cpsr */
		orr     r0,r3,#0xc0          /*  irqfiq_off */
		msr     cpsr_c,r0
		mrc     p15,0,r1,c1,c0,0     /*  Save ctrl in r1 */
		orr     r2,r1,#0x1           /*  MMU */
		bic     r2,r2,#0x4           /*  ~DFILL */
		orr     r2,r2,#0x400         /*  BUSLOCK */

		 /*  Enable hardware TLB fetches by setting TTB */
		ldr     r12,=table_base      /*  r12: &table_base */
		ldr     r0,[r12]
		mcr     p15,0,r0,c2,c0,0     /*  cp15.TTB= FirstLevelTable */

		 /*  Flush cacheline+TLB and perform data segment fetch */
		sub     r12,r12,#0x400       /*  1k margin back */
		bic     r12,r12,#0x0ff       /*  r12: data, align 4k */
		bic     r12,r12,#0xf00
		mcr     p15,0,r12,c7,c10,4   /*  cp15.DrainWb()                 */
		 /* mcr     p15,0,r12,c7,c6,1   ; cp15.FlushDCacheSingle( line 0 ) */
		mcr     p15,0,r12,c7,c6,0    /*  cp15.FlushDCache */
		mcr     p15,0,r2,c1,c0,0     /*  buslock, mmu, no dfill */
		mcr     p15,0,r1,c8,c7,0     /*  cp15.FlushTlb() */
		ldrb    r0,[r12],#0xffc      /*  tlb fetch */
		mcr     p15,0,r1,c1,c0,0     /*  restore ctrl */
		mcr     p15,0,r2,c1,c0,0     /*  buslock, mmu, no dfill */
		ldrb    r0,[r12,#4]          /*  tlb fetch + 4k */
		mcr     p15,0,r1,c1,c0,0     /*  restore ctrl */

		 /*  Fetch kernel code segment */
		sub     r12,pc,#0x400        /*  1k margin back */
		bic     r12,r12,#0x0ff       /*  r12: code, align 4k */
		bic     r12,r12,#0xf00
		mcr     p15,0,r2,c1,c0,0     /*  buslock, mmu, no dfill */
		ldrb    r0,[r12],#0xffc      /*  tlb fetch */
		mcr     p15,0,r1,c1,c0,0     /*  restore ctrl */
		mcr     p15,0,r2,c1,c0,0     /*  buslock, mmu, no dfill */
		ldrb    r0,[r12,#4]          /*  tlb fetch + 4k */
		mcr     p15,0,r1,c1,c0,0     /*  restore ctrl */

		 /*  Fetch exception vector segment */
		ands    r12,r1,#0x2000       /*  r12= hivecs? 0xffff0000 : 0x00000000 */
		ldrne   r12,=0xffff0000
		mcr     p15,0,r2,c1,c0,0     /*  buslock, mmu, no dfill */
		ldrb    r0,[r12]             /*  tlb fetch */
		mcr     p15,0,r1,c1,c0,0     /*  restore ctrl */

		 /*  Restore TTB and save index */
		mov     r0,#ZEROS_BASE
		mcr     p15,0,r0,c2,c0,0     /*  cp15.TTB= ZEROS_BASE */
		ldr     r0,=base_index
		mrc     p15,0,r1,c8,c0,0     /*  cp15.GetIndex()                 */
		str     r1,[r0]
		msr     cpsr_c, r3           /*  irqfiq_set */
		mov     pc,lr                /*  Return                  */


 /* ------------------------------------------------------------------ */
 /*  broskfix_data_check */
 /* ------------------------------------------------------------------ */

broskfix_data_check:     /*  Calculate address of aborted instruction */
		sub     lr,lr,#8

		 /*  Save registers in temporary area */
		ldr     sp,=backup_regs        /*  Load sp here to remove magic and allow next TLB miss          */
		stmia   sp,{r0-r4,lr}

		 /*  Also save SPSR to be restored by chain handler */
		mrs     r0,spsr
		str     r0,[sp, #backup_spsr-backup_regs]
		
		 /*  Indicate that the TLB miss concerns data rather than prefetch */
		mov     r0,#1
		str     r0,[sp, #check_data-backup_regs]
		
		b       tlb_miss_main

 /* ------------------------------------------------------------------ */
 /*  broskfix_prefetch_check */
 /* ------------------------------------------------------------------ */

broskfix_prefetch_check:
		 /*  Calculate address of aborted instruction */
		sub     lr,lr,#4

		 /*  Save registers in temporary area */
		ldr     sp,=backup_regs          /*  Load sp here to remove magic and allow next TLB miss  */
		stmia   sp,{r0-r4,lr}

		 /*  Also save SPSR to be restored by confirm or chain handler */
		mrs     r0,spsr
		str     r0,[sp, #backup_spsr-backup_regs]

		 /*  Section Translation fault => Confirm TLB Miss                 */
		 /*  Note: FAR and FSR are trashed by the data abort */
		mov     sp,#SP_CONFIRM_MAGIC
		ldrb    r1,[lr]

		 /*  No abort => Not a tlb miss */
		ldr     sp,=backup_regs
		b       cause_abort

 /* ------------------------------------------------------------------ */
 /*  broskfix_chain_cleanup */
 /* ------------------------------------------------------------------ */

 /*  Clean up trashed registers before calling ordinary abort handlers */
 /*  * FAR */
 /*  * FSR */
 /*  * SPSR */

broskfix_chain_cleanup:  /*  Fix FSR if necessary and jump to data_abort_handler */
		ldr     sp,=backup_regs          /*  Load sp here to remove magic and allow next TLB miss          */
		str     r0,[sp]                  /*  reserve temp register */
		ldr     r0,[sp, #last_far-backup_regs]
		mcr     p15,0,r0,c6,c0,0         /*  restore far */
		ldr     r0,[sp, #last_fsr-backup_regs]
		mcr     p15,0,r0,c5,c0,0         /*  restore fsr */
		ldr     r0,[sp, #backup_spsr-backup_regs]
		msr     spsr_csxf,r0             /*  restore spsr */
		ldr     r0,[sp]                  /*  restore temp register */
		mov     pc,lr

 /* ------------------------------------------------------------------ */
 /*  Confirm TLB Miss upon prefetch */
 /* ------------------------------------------------------------------ */

broskfix_confirm_prefetch:
		 /*  Restore SPSR from last saved abort */
		ldr     sp,=backup_regs
		ldr     r0,[sp, #backup_spsr-backup_regs]
		msr     spsr_cxsf,r0

		 /*  Indicate that the TLB miss concerns prefetch rather than data */
		mov     r0,#0
		str     r0,[sp, #check_data-backup_regs]
		
		 /*  Proceed directly into tlb_miss_main below */

 /* ------------------------------------------------------------------ */
 /*  TLB Miss Main Handler */
 /* ------------------------------------------------------------------ */

tlb_miss_main:   /*  This is executed in a Data Abort (might be prefetch confirmation) */
		 /*  sp= backup_regs */

		 /*  Check Fault Status Register */
		mrc     p15,0,r0,c5,c0,0         /*  r0= CP15.FSR */
		cmp     r0,#SECTION_TRANS_FAULT
		bne     cause_abort              /*  Not translation fault => cause ordinary abort */

		 /*  Soft 1st Level Lookup */
		mrc     p15,0,r4,c6,c0,0                 /*  r4= CP15.FAR */
		mov     r1,r4,lsr #20                    /*  r1= Section number */
		ldr     r0,[sp,#table_base-backup_regs]  /*  r0= table_base */
		add     r0,r0,r1,lsl #2                  /*  add offset for entry */
		bl      physical_load                    /*  r0=physical_load(r0,r13) */

		 /*  Decode 1st level */
		 /*  r0[1:0] : 00 - Fault */
		 /*            01 - Page Table */
		 /*            10 - Section */
		 /*            11 - UNPREDICTABLE */
		and     r1,r0,#3                 /*  Check block type */
		cmp     r1,#2
		beq     load_section             /*  2 => Section */
		cmp     r1,#1
		bne     cause_abort

		 /*  Soft 2nd Level Lookup */
		 /*  r0[31:10] : Page table base address */
		bic     r1,r0,#0x00000300        /*  Get 2nd level table base */
		bic     r1,r1,#0x000000ff        /*  Get 2nd level table base */
		and     r2,r4,#0x000ff000        /*  A[19:12] */
		add     r0,r1,r2,lsr #10         /*  add offset for entry */
		bl      physical_load            /*  r0=physical_load(r0,r13) */

		 /*  Decode 2nd level */
		 /*  r0[31:12] : Page base address */
		 /*  r0[7:4]   : Permissions */
		 /*  r0[3:2]   : CB */
		 /*  r0[1:0]   : Type (10 = Page) */

		 /*  Cancel fetch if not a valid page */
		and     r1,r0,#3                 /*  Check block type */
		cmp     r1,#2
		bne     cause_page_trans

		 /*  Proceed directly into load_page below */

 /* ------------------------------------------------------------------ */
 /*  Load page */
 /* ------------------------------------------------------------------ */

load_page:       /*  r0 - Page Descriptor */
		 /*    r0[31:12] : Page base address */
		 /*    r0[7:4]   : Permissions */
		 /*    r0[3:2]   : CB */
		 /*    r0[1:0]   : Type (10 = Page) */
		 /*  r4: VADDR */
		 /*  sp: backup_regs */

		 /*  Reformat into TLB native format */
		mov     r1,r4,lsr #12
		mov     r1,r1,lsl #12                    /*  r1= virtual high bits */
		orr     r1,r1,#0x3                       /*  tag[2:0]=3 (Small Page)                 */

		and     r2,r4,#0x000ff000                /*  combination bits 19:12 */
		eor     r2,r2,r0                         /*  phys bits 31:12, perm 7:4, cb 3:2 */

		 /*  Fall through into write_and_exit */

 /* ------------------------------------------------------------------ */
 /*  write_and_exit */
 /* ------------------------------------------------------------------ */

write_and_exit:  /*  r1: TLB tag */
		 /*  r2: TLB data */
		 /*  sp: backup_regs */

		 /*  Increase Index register,  */
		mcr     p15,0,r1,c8,c0,1                 /*  Write TLB tag */
		mcr     p15,0,r2,c8,c0,2                 /*  Write TLB data */
		mrc     p15,0,r0,c8,c0,0                 /*  Read TLB index */
		add     r0,r0,#1
		cmp     r0,#32                           /*  wrap to base_index */
		ldreq   r0,[sp,#base_index-backup_regs]
		mcr     p15,0,r0,c8,c0,0                 /*  Write TLB index */

		 /*  Avoid sending TLB-miss Cache-hit writes to /dev/null */
		mcr     p15,0,r12,c7,c6,0    /*  cp15.FlushDCache */

		 /*  Restore FAR,FSR so the TLB miss abort goes unnoticed */
		ldr     r0,[sp, #last_far-backup_regs]
		mcr     p15,0,r0,c6,c0,0         /*  restore fsr */
		ldr     r0,[sp, #last_fsr-backup_regs]
		mcr     p15,0,r0,c5,c0,0         /*  restore fsr */

	.ifdef COUNT_MISSES
		ldr     r1,[sp,#miss_count-backup_regs]
		add     r1,r1,#1
		str     r1,[sp,#miss_count-backup_regs]
	.endif

		 /*  Restore registers from temporary area,  */
		 /*  restore CPSR from SPSR, and return from exception */
		 /*  (Without setting SP to any magic) */
01:     ldmia   sp,{r0-r4,pc}^

 /* ------------------------------------------------------------------ */
 /*  Load section */
 /* ------------------------------------------------------------------ */

load_section:    /*  r0: Section descriptor */
		 /*      r0[31:20] : Section physical base address */
		 /*      r0[13:10] : PERM */
		 /*      r0[3:2]   : CB */
		 /*      r0[1:0]   : Type (10 = Section) */
		 /*  r4: VADDR */
		 /*  sp: backup_regs */

		 /*  Reformat into TLB native format */
		mov     r1,r4,lsr #20
		mov     r1,r1,lsl #20                    /*  r1= virtual high bits           */
		orr     r1,r1,#0x1                       /*  tag[2:0]=1 (Section)                 */

		mov     r2,r0,lsr #20
		mov     r2,r2,lsl #20                    /*  r2= (phys xor virt) high bits */
		and     r3,r0,#0xc                       /*  Extract CB bits */
		orr     r2,r2,r3                         /*  data[3:2] = CB */
		and     r3,r0,#0x3c00                    /*  Extract PERM bits */
		orr     r2,r2,r3,lsr #6                  /*  data[7:4] = PERM */

		b       write_and_exit

 /* ------------------------------------------------------------------ */
 /*  cause_abort */
 /* ------------------------------------------------------------------ */

cause_abort:     /*  Chain to ordinary abort handler by causing a new exception */
		 /*  sp: backup_regs */

		 /*  Get true FSR */
		mrc     p15,0,r0,c5,c0,0         /*  r0= CP15.FSR */
		b       return_chain

 /* ------------------------------------------------------------------ */
 /*  cause_page_trans */
 /* ------------------------------------------------------------------ */

cause_page_trans:        /*  Chain to ordinary abort handler by causing a new exception */
		 /*  Emulate page translation fault */
		 /*  sp: backup_regs */

		 /*  Save emulataed FSR */
		mov     r0,#PAGE_TRANS_FAULT     /*  emulate page translation fault */

 /* ------------------------------------------------------------------ */
 /*  return_chain */
 /* ------------------------------------------------------------------ */

return_chain:    /*  r0 - FSR value to emulate for data abort */
		 /*  sp: backup_regs */

		 /*  Conditionally update last_fsr, last_far */
		ldr     r1,[sp, #check_data-backup_regs]
		cmp     r1,#0                           /*  if (check_data) */
		strne   r0,[sp, #last_fsr-backup_regs]  /*  => save fsr */
		mrcne   p15,0,r0,c6,c0,0               
		strne   r0,[sp, #last_far-backup_regs]  /*  => save FAR */
		
		mrs     r0,spsr                /*  Save SPSR */
		str     r0,[sp, #backup_spsr-backup_regs]
		orr     r0,r0,#0xc0            /*  Interrupts disabled during chaining */
		msr     spsr_cxsf,r0
		ldmia   sp,{r0-r4,lr}          /*  Restore registers from temporary area */
		mov     sp,#SP_CHAIN_MAGIC     /*  Set chain magic in SP_abt */

		 /*  Restore CPSR from SPSR, and return from exception (Cause a new one)                 */
		movs    pc,lr

 /* ------------------------------------------------------------------ */
 /*  physical_load */
 /* ------------------------------------------------------------------ */
 /*  Load word from physical address */

physical_load:   /*  r0 - PHADDR */
		 /*  r1 - scratch */
		 /*  r2 - scratch */
		 /*  r3 - scratch */
		 /*  sp - used as barrier */

		mrc     p15,0,r1,c1,c0,0         /*  r3= cp15.GetControl() */
		bic     r2,r1,#0x0004            /*  ~DFILL */
		orr     r2,r2,#0x1000            /*  r2= IFILL, ~DFILL */
		mrc     p15,0,r2,c1,c0,0         /*  dcache off, icache on                                                                                             */

		bic     r2,r2,#0x0001            /*  r2= IFILL, ~DFILL. ~MMU */
		mcr     p15,0,r2,c7,c10,4        /*  cp15.DrainWb()                 */

		mcr     p15,0,r0,c7,c6,1         /*  cp15.FlushDCacheSingle(tlb)                 */
		mcr     p15,0,sp,c7,c6,1         /*  cp15.FlushDCacheSingle(barrier) */
		cmp     r1,r2                    /*  Z=0 */

01:             ldreq   r3,[sp]                  /*  => ICACHE barrier */
		mcreq   p15,0,r2,c1,c0,0         /*  => translate off */
		ldreq   r0,[r0]                  /*  => load physical word */
		mcreq   p15,0,r1,c1,c0,0         /*  => restore ctrl */
		moveq   pc,lr                    /*  => return */

		cmp     r1,r1                    /*  Z=1 */
		b       01b

 /* ------------------------------------------------------------------ */
 /*  locked_fetch */
 /* ------------------------------------------------------------------ */

 /*  Perform locked fetch and set base pointer to wrapped position */
 /*  r0 - address to fetch. */
 /*  r1 - scratch */
 /*  r2 - scratch */
 /*  r3 - scratch */

locked_fetch:    /*  Enable hardware TLB fetches by setting TTB */
		ldr     r3,=table_base        /*  r3 = &table_base */
		ldr     r1,[r3]
		mcr     p15,0,r1,c2,c0,0      /*  cp15.TTB= table_base */

		 /*  Perform locked load */
		mrc     p15,0,r1,c1,c0,0      /*  Save ctrl in r1 */
		orr     r2,r1,#0x400          /*  BUSLOCK */
		mcr     p15,0,r2,c1,c0,0      /*  buslock on */
		ldrb    r0,[r0]
		mcr     p15,0,r1,c1,c0,0      /*  restore ctrl */

		 /*  Restore TTB */
		mov     r0,#ZEROS_BASE
		mcr     p15,0,r0,c2,c0,0      /*  cp15.TTB= ZEROS_BASE */

		 /*  Wrap index and return */
		mrc     p15,0,r0,c8,c0,0      /*  cp15.GetIndex() */
		cmp     r0,#0
		ldreq   r0,[r3,#base_index-table_base]
		mcreq   p15,0,r0,c8,c0,0      /*  cp15.SetIndex() */
		mov     pc,lr                 /*  return */

 /* ------------------------------------------------------------------ */
 /*  Debug */
 /* ------------------------------------------------------------------ */

debug:          ldr     r0,=debug_index
		mrc     p15,0,r3,c8,c0,0
		str     r3,[r0]

		mov     r2,#0
		ldr     r0,=debug_tag

01:             mcr     p15,0,r2,c8,c0,0     /*  set index */
		mrc     p15,0,r1,c8,c0,1     /*  read tag */
		str     r1,[r0],#4
		add     r2,r2,#1
		cmp     r2,#32
		bne     01b

		mov     r2,#0
		ldr     r0,=debug_data

01:             mcr     p15,0,r2,c8,c0,0     /*  set index */
		mrc     p15,0,r1,c8,c0,2     /*  read data */
		str     r1,[r0],#4
		add     r2,r2,#1
		cmp     r2,#32
		bne     01b

		mcr     p15,0,r3,c8,c0,0     /*  restore index                 */

01:
		b       01b

 /* ------------------------------------------------------------------ */
 /*  Data */
 /* ------------------------------------------------------------------ */

	.bss

data_start:
backup_regs:    .skip   16*4
backup_spsr:    .skip   4

table_base:     .int 0

base_index:     .int 0


check_data:     .skip   4
last_far:       .skip   4
last_fsr:       .skip   4

miss_count:     .int 0


	.global debug_tag
	.global debug_data
	.global debug_index

debug_tag:      .skip   32*4
debug_data:     .skip   32*4
debug_index:    .skip   4


	/* END */
