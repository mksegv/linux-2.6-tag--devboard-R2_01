// Dump diagnostic info onto the debug port regarding bus interface stuff

#include <linux/config.h>
#include <linux/init.h>
#include <asm/system.h>
#include <asm/arch/argus3.h>

#define PRINTF printk

#define GETFIELD(x,pos,width) (((x)>>pos)&((1<<width)-1))

/*
void argus3_print_fpga_version(void)
{
  PRINTF("FPGA_MAGIC: 0x%08x\n", SOC_FPGA.MAGIC);  
  PRINTF("FPGA_VERSION: %d\n", SOC_FPGA.VERSION);  
  nop();
  nop();
}
*/

static void __init
dump_extmem_regs(void)
{
  unsigned int reg, i;
  // Each chip select
  for(i=0; i<6; i++)
  {
    reg=SOC_EXTMEM.CONFIG[i];
    PRINTF("SOC_EXTMEM.CONFIG[%d]=0x%08x\n", i, reg);
    PRINTF("  WIDTH   = %d\n", GETFIELD(reg,0 ,2));
    PRINTF("  SETUP   = %d\n", GETFIELD(reg,2 ,4));
    PRINTF("  WAIT    = %d\n", GETFIELD(reg,4 ,2));
    PRINTF("  HOLD    = %d\n", GETFIELD(reg,8 ,4));
    PRINTF("  BWAIT   = %d\n", GETFIELD(reg,10,3));
    PRINTF("  BTYPE   = %d\n", GETFIELD(reg,14,2));
    PRINTF("  TOFF    = %d\n", GETFIELD(reg,17,1));
    PRINTF("  WRBM    = %d\n", GETFIELD(reg,19,1));
    PRINTF("  SLOW    = %d\n", GETFIELD(reg,20,1));
    PRINTF("  EXTWAIT = %d\n", GETFIELD(reg,21,1));
    PRINTF("  NREMODE = %d\n", GETFIELD(reg,22,1));
  }
  
  // SDRAM controller
  reg=SOC_EXTMEM.SDCTRL1;
  PRINTF("SOC_EXTMEM.SDCTRL1=0x%08x\n", reg);
  PRINTF("  ENABLE  = %d\n", GETFIELD(reg,0,1));
  PRINTF("  BSHIFT  = %d\n", GETFIELD(reg,1,4));
  PRINTF("  BWIDTH  = %d\n", GETFIELD(reg,5,1));
  PRINTF("  RSHIFT  = %d\n", GETFIELD(reg,6,3));
  PRINTF("  RWIDTH  = %d\n", GETFIELD(reg,9,2));
  {
    // Convert to readable format
    int WIDTH=GETFIELD(SOC_EXTMEM.CONFIG[4],0,2);
    int BSHIFT=GETFIELD(reg,1,4);
    int BWIDTH=GETFIELD(reg,5,1);
    int RSHIFT=GETFIELD(reg,6,3);
    int RWIDTH=GETFIELD(reg,9,2);
    int c=WIDTH;
    int b=BSHIFT<=4 ? BSHIFT+8 : BSHIFT+15;
    int r=RSHIFT+8;
    int bwidth=BWIDTH+1;
    int rwidth=RWIDTH+11;
    PRINTF("SDRAM address split\n");
    if (WIDTH)
      PRINTF("  [%2d:%2d] - byte select\n", c-1,0);
    if (b<r)
    {
      PRINTF("  [%2d:%2d] - col\n", b-1,c);
      PRINTF("  [%2d:%2d] - bank\n", b+bwidth-1,b);
      PRINTF("  [%2d:%2d] - row\n", r+rwidth-1,r);
    }
    else
    {
      PRINTF("  [%2d:%2d] - col\n", r-1,c);
      PRINTF("  [%2d:%2d] - row\n", r+rwidth-1,r);
      PRINTF("  [%2d:%2d] - bank\n", b+bwidth-1,b);
    }
  }
  reg=SOC_EXTMEM.SDCTRL2;
  PRINTF("SOC_EXTMEM.SDCTRL2=0x%08x\n", reg);
  PRINTF("  TRASMIN    = %d\n", GETFIELD(reg, 0 , 4));
  PRINTF("  TRASMAX    = %d\n", GETFIELD(reg, 4 , 6));
  PRINTF("  TRRD       = %d\n", GETFIELD(reg, 10, 2));
  PRINTF("  TRCD       = %d\n", GETFIELD(reg, 12, 2));
  PRINTF("  TRP        = %d\n", GETFIELD(reg, 14, 2));
  PRINTF("  TRC        = %d\n", GETFIELD(reg, 16, 4));
  PRINTF("  TRDL       = %d\n", GETFIELD(reg, 20, 2));
  PRINTF("  CASL       = %d\n", GETFIELD(reg, 22, 1));
  PRINTF("  CLKDIV     = %d\n", GETFIELD(reg, 23, 2));
  PRINTF("  SDCLKFORCE = %d\n", GETFIELD(reg, 25, 1));
  PRINTF("  PCHGMODE   = %d\n", GETFIELD(reg, 26, 1));
  PRINTF("  CKEMODE    = %d\n", GETFIELD(reg, 27, 1));
  
  reg=SOC_EXTMEM.SDRFSH;
  PRINTF("SOC_EXTMEM.SDRFSH=0x%08x\n", reg);
  PRINTF("  ENABLE = %d\n", GETFIELD(reg,  0, 1 ));
  PRINTF("  SELF   = %d\n", GETFIELD(reg,  1, 1 ));
  PRINTF("  BURST  = %d\n", GETFIELD(reg,  2, 3 ));
  PRINTF("  PERIOD = %d\n", GETFIELD(reg,  5, 16));
  
  // Arbitration
  reg=SOC_EXTMEM.XARB;
  PRINTF("SOC_EXTMEM.XARB=0x%08x\n", reg);
  PRINTF("  ENABLE   = %d\n", GETFIELD(reg, 0, 1));
  PRINTF("  REQPOL   = %d\n", GETFIELD(reg, 1, 1));
  PRINTF("  GRANTPOL = %d\n", GETFIELD(reg, 2, 1));
  PRINTF("  WIDTH    = %d\n", GETFIELD(reg, 3, 2));
}


module_init(dump_extmem_regs);
