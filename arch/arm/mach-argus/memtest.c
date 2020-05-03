#define IN_KERNEL

#ifdef IN_KERNEL
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/stddef.h>
#include <linux/param.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/slab.h>

#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/pgalloc.h>
#include <asm/arch/argus.h>
#include <asm/arch/argus_addr.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>

#define printf printk

#else
#include <stdio.h>
#include <sys/mman.h>
#endif

// iter, p1, 1, 0x80000000, 0, i
// iter, ~p1, 0xfffffffe, 0x7fffffff, 1, i

static volatile unsigned short *ioarea;
static volatile unsigned long *flasharea;


static void
memerror(unsigned long *p, unsigned long pat, unsigned long bad, int i)
{
	printf("mem failed at 0x%p, pat 0x%x was 0x%x, test %d\n",
	       p, pat, bad, i);
}

#define AMASK 0x001fffff
// 21 bits
// swap bits 0-9 with 10-19
// 0x0020 0000

#define AFUNC(x) (((x&0x3ff) << 10) | ((x>>10)&0x3ff) | (x&0x100000))
 
static void
movinv32(unsigned long addr, unsigned long length, int iter, 
	 unsigned long p1, 
	 unsigned long lb, unsigned long hb, unsigned long sval, int off)
{
	unsigned long *start = (unsigned long *)(addr);
	unsigned long *stop = (unsigned long *)(addr + length);
	volatile unsigned long *p;
	unsigned long pat, p3, bad;
	int k = 0, n = 0;
	volatile int a;
	int i;

	length = length >> 2;

	p3 = sval << 31;

	// Fill memory with the pattern

	k = off;
	pat = p1;
	p = start;
	i = 0;
	while(i < length) {
		p[AFUNC(i)] = pat;
		if(++k >= 32) {
			pat = lb;
			k = 0;
		} else {
			pat = pat << 1;
			pat |= sval;
		}
		i++;
	}

	while (iter > 0) {

		// Check that the pattern has not changed, and
		// write the inverse of the pattern at each location
		// as we move along.
		
		// Start from the bottom up
		
		k = off;
		pat = p1;
		p = start;
		i = 0;

		while(i < length) {
			if((bad = p[AFUNC(i)]) != pat) {
				memerror(p, pat, bad, 1);
			}
			p[AFUNC(i)] = ~pat;
			if(++k >= 32) {
				pat = lb;
				k = 0;
			} else {
				pat = pat << 1;
				pat |= sval;
			}
			i++;
		}

		if(--k < 0)
			k = 31;
		for(pat = lb, n = 0; n < k; n++) {
			pat = pat << 1;
			pat |= sval;
		}
		k++;

		// Start from the top down
		
		p = start;
		i = length - 1;

		do {
			if((bad = p[AFUNC(i)]) != ~pat) {
				memerror(p, pat, bad, 2);
			}
			p[AFUNC(i)] = pat;
			if(--k <= 0) {
				pat = hb;
				k = 32;
			} else {
				pat = pat >> 1;
				pat |= p3;
			}
			i--;
		} while(i >= 0);

		iter--;
	}

}
#ifdef IN_KERNEL


static void 
do_memtest(void)
{
	int i;
	unsigned long p1;
	unsigned long *p = (unsigned long *)0xc0200000;
	int len = 0x00800000;

	printk("memtest_init at 0x%x, len 0x%x\n", p, len);

	cli();

	do {
		for(i = 0, p1 = 1; p1; p1 = p1 << 1, i++) {
			printk("Doing moving inversion test %d\n", i);
			movinv32(p, len, 8, p1, 1, 0x80000000, 0, i);
			movinv32(p, len, 8, ~p1, 0xfffffffe, 0x7fffffff, 1, i);
		}
		printk("All passed ok.\n");
	} while(1);

}

static void __init memtest_init(void)
{
	gpio_set_mode(GPIO_nCS2, GPIO_MODE_ALT_FUNC, 0);  // CS2 used as chip select
	
	gpio_set_mode(GPIO_nWAIT, GPIO_MODE_ALT_FUNC, 0);  // WAIT needed
	
	SOC_EXTMEM.CONFIG[2] = SET_EXTCONFIG_WIDTH(1) |
		SET_EXTCONFIG_SETUP(0) |
		SET_EXTCONFIG_HOLD(0) |
		SET_EXTCONFIG_WAIT(4) | SET_EXTCONFIG_EXTWAIT(1);

	flasharea = EXT0_CACHE_V;
	ioarea = EXT2_NOCACHE_V + 0x300;

	__asm__ __volatile__ ("mov sp, %0" : : "r" (0xc1000000) : "memory");
	do_memtest();
}

module_init(memtest_init);
#else
int
main()
{
	int i;
	unsigned long p1;
	unsigned long *p;
	int len = 0x00800000;

	p = mmap(NULL, len, PROT_READ|PROT_WRITE,
		 MAP_PRIVATE|MAP_ANONYMOUS, 0, 0); 

	for(i = 0, p1 = 1; p1; p1 = p1 << 1, i++) {
		printf("Doing moving inversion test %d\n", i);
		movinv32(p, len, 8, p1, 1, 0x80000000, 0, i);
		movinv32(p, len, 8, ~p1, 0xfffffffe, 0x7fffffff, 1, i);
	}
	printf("All passed ok.\n");
}
#endif
