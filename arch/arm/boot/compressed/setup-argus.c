#include <asm/arch/argus.h>
#include <asm/arch/uncompress.h>
#include <linux/config.h>

#define DONT_TOUCH_GPIO 0xff

static volatile tUART *uart; 
extern char* gpio; /* GPIO settings */

void argus_clock_setup();
void l2cache_init(int nRams, int nIxRed);

void hw_init(int test)
{
  int i;
  char* gpio_configs = &gpio;
  
  /* Make sure IRQs are off */
  SOC_IRQ.LEVEL = 0;
  SOC_IRQ.MASK = 0;

  /* Set up cache and clock */
#ifdef CONFIG_ARGUS_3
  l2cache_init(3, 0);  // 24 kB cache
#else
  l2cache_init(4, 0);
#endif

#ifndef CONFIG_ARGUS_3 
  // We don't know correct values for Argus-3 yet and the boot-ROM
  // sets sane values.
  argus_clock_setup();
#endif

  /* Initialize GPIOs */
  for (i = 0; i < NUM_GPIO; i++) {
    if (gpio_configs[i] != 0xff) 
      SOC_GPIO.PIN[i] = gpio_configs[i];
  }

  /* Set up serial port */
#ifndef CONFIG_ARGUS_DEBUG_PORT_NULL
#ifdef CONFIG_ARGUS_DEBUG_PORT0
  uart = &SOC_UART[0];
  SOC_GPIO.PIN[GPIO_UART0_RXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
  SOC_GPIO.PIN[GPIO_UART0_TXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
  SOC_GPIO.PIN[GPIO_UART0_CTS] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
  SOC_GPIO.PIN[GPIO_UART0_RTS] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
#elif defined(CONFIG_ARGUS_DEBUG_PORT1)
  uart = &SOC_UART[1];
  SOC_GPIO.PIN[GPIO_UART1_RXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
  SOC_GPIO.PIN[GPIO_UART1_TXD] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);
#endif
  SOC_CLK.ENABLE = CLK_UART0 | CLK_UART1;
  uart->CTRL = SET_UART_MODE(0) |
    SET_UART_AUTOFLOW(0) |
    SET_UART_TXPOL(0) |
    SET_UART_RXPOL(0) |
    SET_UART_STOPBITS(0) |
    SET_UART_LOOPBACK(0);
  uart->BAUD = SET_UART_BAUD(UART_BAUD_115200);
  uart->IRQEN = 0;
  uart->CONFIG = SET_UART_IRDATXLEN(0)     |
    SET_UART_RXSTART(6)       |
    SET_UART_RXEND(5)         |
    SET_UART_IRDARXLEN(15);
  uart->ENABLE = SET_UART_TXEN(1);
  // Seem to need to wait a bit to avoid getting corrupted serial output
  for (i = 0; i < 5000000; i++);
#endif
}

static void 
putc(char character)
{
#ifndef CONFIG_ARGUS_DEBUG_PORT_NULL
  // wait until tx_fifo is not full
  while(!(GET_UART_TXNFULL(uart->STATUS))); 
  // write byte
  uart->DATA = character;
  // wait until tx_fifo is empty
  while(!(GET_UART_TXEMPTY(uart->STATUS)));
#endif
}

// This function is normally inlined in include/asm-arm/mach-*/uncompress.h

void 
putstr(const char* string)
{
  while (*string)
    putc(*string++);
} 

/* Clock setup stuff */
static const int pll_p[2][3] = {{ 4, 4, 4}, { 6, 6, 6}};
static const int pll_m[2][3] = {{22,19,22}, {28,25,28}};
static const int pll_s[2][3] = {{ 0, 0, 1}, { 0, 0, 1}};
static const int clkdivper[2][3] = {{96,80,32}, {98,85,33}};

#define PLL_OSC_12      0
#define PLL_OSC_13      1

#define PLL_MODE_200    0
#define PLL_MODE_175    1
#define PLL_MODE_100    2
#define PLL_MODE_MAX    -1

/********************************************************************************
 * Enables the pll. The pll output will be stable after 150 us, then PLL mode can 
 * be entered. 
 * input parameter osc should be set to 1 if a 13 MHz osc is used, and 0 for a 12 MHz Osc
 * mode param = (0..2) will give the following pll frequencys
 * 
 *  osc          mode   cpu-freq(cdiv=0)      FIN FOUT  PLL_P  PLL_M  PLL_S  CLKDIVPER
 *  0 (12 MHz)    0       ~200 MHz             12  384   4      22     0      96
 *  0 (12 MHz)    1       ~175 MHz             12  336   4      19     0      80  
 *  0 (12 MHz)    2       ~100 MHz             12  192   4      22     1      32
 *  1 (13 MHz)    0       ~200 MHz             13  390   6      28     0      98
 *  1 (13 MHz)    1       ~175 MHz             13  351   6      25     0      85
 *  1 (13 MHz)    2       ~100 MHz             13  195   6      28     1      33
 * 
 *  PLL:    FOUT = FIN*8*(M+2)/((P+2)*2^S))
 *  USB:    CLKDIVPER = (FOUT*16)/48 - 32
 */
static void clock_enable_pll(int osc, int mode)
{
  // Turn on PLL but do not use it
  SOC_CLK.PLL = SET_PLL_E(1)  |
                SET_PLL_P(pll_p[osc][mode])  |
                SET_PLL_S(pll_s[osc][mode])  |
                SET_PLL_M(pll_m[osc][mode]);
  // Set oscillator type in clock control register
  SOC_CLK.CTRL = CHG_CLK_13MHZ(SOC_CLK.CTRL, osc);
  
  // Prepare for generation of correct USB clock
  SOC_CLK.CLKDIVPER = clkdivper[osc][mode];
}

/********************************************************************************
 * Leaves PLL mode, and disables the pll. 
 */
static void clock_disable_pll(void)
{ 
  // Turn off PLL usage
  SOC_CLK.CTRL = CHG_CLK_HISPD(SOC_CLK.CTRL, 0);

  // Wait until PLL not actually used 
  while (GET_CLK_HISPD_STATUS(SOC_CLK.CTRL) == 1)
    ;

  // Now, PLL generated clock is not used so it is safe to turn off PLL
  SOC_CLK.PLL = SET_PLL_E(0)  |
                SET_PLL_P(0)  |
                SET_PLL_S(0)  |
                SET_PLL_M(0);            
}
/********************************************************************************
 * Enable PLL-mode. (PLL should be enabled first)
 * (will be effective at next MHz-clock edge)
 */
static void clock_enable_use_pll(void)
{
  // Enable PLL usage 
  SOC_CLK.CTRL = CHG_CLK_HISPD(SOC_CLK.CTRL, 1); 
}                

/********************************************************************************
 * Returns info about which crysta is used
 * return value PLL_OSC_12 or PLL_OSC_12 (as defined in clock.h)
 */
int clock_get_osc(void)
{
  int i;

 if (GET_CLK_13MHZ(SOC_CLK.CTRL))
 {
   return PLL_OSC_13;
 }
 
 else
 {
   return PLL_OSC_12;
 }
 
}

/********************************************************************************
 * Leave PLL-mode. 
 * (will be effective at next MHz-clock edge)
 */
/********************************************************************************
 * set clock division factors for the frequency domains in ArgusII (set Argus II ref man)
 * cdiv - sets CPU freq.            cpu_freq   = FOUT/(cdiv+1)
 * hdiv - sets system frequency     sys_freq   = cpu_freq/(hdiv+1)
 * pdiv - sets peripheral frequency perip_freq = sys_freq/(pdiv+1)
 */

static void clock_set_dividers(int cdiv, int hdiv, int pdiv)
{
  int clk_ctrl = SOC_CLK.CTRL;
  SOC_CLK.CTRL = CHG_CLK_PDIV(CHG_CLK_HDIV(CHG_CLK_CDIV(clk_ctrl, cdiv), hdiv), pdiv);
}
/********************************************************************************
 * Easy-to-use all-in-one clock setup
 * Sets dividers and enables PLL, aware of PLL locking time
 * see clkEnabloePll for osc and mode definition
 * Replaces the good old enable_pll_and_wait
 */
void argus_clock_setup()
{
  int i;

  if (GET_CLK_HISPD_STATUS(SOC_CLK.CTRL) == 1)
	clock_disable_pll();

  // Set divider so that HCLK speed <= 100 MHz, PCLK speed <= 50 MHz
  clock_set_dividers(0, 1, 1);

  clock_enable_pll(clock_get_osc(), PLL_MODE_200);

  for (i = 0; i < 1000000; i++); // PLL has 150 us locking time

  clock_enable_use_pll();

  // Don't leave this function until PLL is actually used
  while (GET_CLK_HISPD_STATUS(SOC_CLK.CTRL) == 0)
    ;
}

/* Cache setup stuff */
void l2cache_FlushAll(void)
{
  SOC_L2CACHE.FLUSH_ALL= 0;
}

void l2cache_CleanAll(void)
{
  SOC_L2CACHE.CLEAN_ALL= 0;
}

void l2cache_init(int nRams, int nIxRed)
{
  if ( !GET_L2CACHE_MODE_BYPASS(SOC_L2CACHE.MODE) ) {
    // Clean the cache if already enabled
    l2cache_CleanAll();
  }

  l2cache_FlushAll();
  SOC_L2CACHE.MODE=
    SET_L2CACHE_MODE_IXRED(nIxRed) | 
    SET_L2CACHE_MODE_ASSRED(4-nRams) |  
    SET_L2CACHE_MODE_IFILL(1) | 
    SET_L2CACHE_MODE_LOCK(0) |
    SET_L2CACHE_MODE_WB(1) |    
    SET_L2CACHE_MODE_DFILL(1);
}
