#include <asm/arch/argus_addr.h>
	
		.equ    irq_base_addr, APB_NOCACHE + 0x20000 

		.macro  disable_fiq
		.endm

		.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr     \irqstat, =irq_base_addr        @ Virt addr for SOC_IRQ
		ldr     \irqstat, [\irqstat, #0]        @ get IRQ_SRCS
		mov     \irqnr, #0
1001:		tst     \irqstat, #1
		bne     1002f
		add     \irqnr, \irqnr, #1
		mov     \irqstat, \irqstat, lsr #1
		cmp     \irqnr, #21
		bne     1001b
1002:
		.endm


