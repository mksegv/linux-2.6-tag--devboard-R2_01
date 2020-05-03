/*
 * Argus3 ethernet driver
 *
 * Copyright (c) 2004,2005   Axis Communications AB (http://www.axis.com)
 *
 * Authors:    Bjorn Wesen, Ola Hugosson
 */

// TODO:
// * Needs reset av ETHMACIF vid abort
// * Implement eth_txtimeout API function
// * Optimize the RX_COPYBREAK number...
// * in argus_eth_stop, should we free skb's ?
// * During open/stop, should the PHY be reset / brought into isolate ?
// * Try to enable interrupts in the rx/tx dma handlers, without
//   getting bitten by reentrancy (??)

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/arch/hardware.h>
#include <asm/arch/argus3.h>
#include <asm/arch/gpio.h>
#include <asm/arch/dma_alloc.h>
#include <asm/arch/argdma.h>

#include <asm/system.h>

#define __DRIVER_NAME    "Argus3 ethernet driver v2.0 (c) 2005 Axis Communications AB"

#define D(x)

#define _SDEBUG
#ifdef _SDEBUG
#  define _DPRINTK(format, args...)  \
          printk (KERN_INFO "%s():%05d "format".\n" , __FUNCTION__ , __LINE__ , ## args);
#else
#  define _DPRINTK(format, args...)
#endif

#define _EPRINTK(format, args...)  \
          printk (KERN_ERR "%s():%05d "format".\n" , __FUNCTION__ , __LINE__ , ## args);

// Global eth_priv pointer, used by the irq callbacks
static struct eth_priv *g_priv;

struct transceiver_ops
{
	unsigned int oui;
	void (*check_speed)(struct net_device* dev);
	void (*check_duplex)(struct net_device* dev);
};

struct transceiver_ops* transceiver;

static unsigned int mdio_phy_addr; /* Transceiver address. */

/* Duplex settings */
enum duplex
{
	half,
	full,
	autoneg
};

/* Network speed indication. */
static struct timer_list speed_timer = TIMER_INITIALIZER(NULL, 0, 0);
static struct timer_list clear_led_timer = TIMER_INITIALIZER(NULL, 0, 0);
static int running;
static int current_speed; /* Speed read from tranceiver */
static int current_speed_selection; /* Speed selected by user */
static unsigned long led_next_time;
static int led_active;

/* Duplex */
static struct timer_list duplex_timer = TIMER_INITIALIZER(NULL, 0, 0);
static int full_duplex;
static enum duplex current_duplex;

/* Broadcom specific */
#define MDIO_AUX_CTRL_STATUS_REG           0x18
#define MDIO_BC_FULL_DUPLEX_IND             0x1
#define MDIO_BC_SPEED                       0x2

/* TDK specific */
#define MDIO_TDK_DIAGNOSTIC_REG              18
#define MDIO_TDK_DIAGNOSTIC_RATE          0x400
#define MDIO_TDK_DIAGNOSTIC_DPLX          0x800

/* RealTek 8201 specific */

/* National Semiconductors DP83846A specific */
#define MDIO_NATSEMI_PHYSTS_REG             0x10
#define MDIO_NATSEMI_PHYSTS_LINK          (1<<0)
#define MDIO_NATSEMI_PHYSTS_SPEED         (1<<1)
#define MDIO_NATSEMI_PHYSTS_DPLX          (1<<2)

/* Network flash constants */
#define NET_FLASH_TIME                  (HZ/50) /* 20 ms */
#define NET_FLASH_PAUSE                (HZ/100) /* 10 ms */
#define NET_LINK_UP_CHECK_INTERVAL        (2*HZ) /* 2 s   */
#define NET_DUPLEX_CHECK_INTERVAL         (2*HZ) /* 2 s   */

#define NO_NETWORK_ACTIVITY 0
#define NETWORK_ACTIVITY    1

// The PHYRESET pin is the same as the UART0 RTS pin
#define GPIO_PHYRESET GPIO_UART0_RTS

// Media length including CRC and alignment of length to 32 bit boundary.
#define ETH_MEDIA_LEN 1520
// Lenght of the ethernet header, used for aligning the payload correctly.
#define ETHER_HEAD_LEN 14
// Number of hardware descriptors in the DMA channels
#define NUM_HW_DESCS 2
// Taken out of the thin air.. breakeven between copying a packet and wasting
// memory by sending up the entire 1520 byte buffer.
#define RX_COPYBREAK 96

struct dmaframe {
	struct sk_buff *skb;  // remember the skb used
};

struct eth_priv {
	int dma_rx, dma_tx;  // Allocated channel numbers for rx/tx
	
	struct dmaframe txframes[NUM_HW_DESCS];
	struct dmaframe rxframes[NUM_HW_DESCS];

	int first_issued_tx, first_issued_rx; // idx of last issued frame for rx/tx
	int cnt_issued_tx, cnt_issued_rx; // number of issued frames for rx/tx
  
	struct net_device_stats stats;        	// statistics
	struct net_device *dev;                 // needed in DMA callbacks
	spinlock_t lock;
};

void dump_packet(char *buf, int len)
{
	int i;
	for(i=0;i<len;i++)
		printk("%02x%c", buf[i], (i+1)%16==0 || i==len-1?'\n':' ');
}

// Some prototypes

static void rxdma_handler(int cha);
static void txdma_handler(int cha);
static void clear_network_leds(unsigned long dummy);
static void set_network_leds(int active);
static void check_speed(unsigned long dummy);
static void set_speed(struct net_device* dev, unsigned long speed);
static void check_duplex(unsigned long dummy);
static void set_duplex(struct net_device* dev, enum duplex);

// Transceiver specific funtions
static void tdk_check_speed(struct net_device* dev);
static void tdk_check_duplex(struct net_device* dev);
static void realtek8201_check_speed(struct net_device* dev);
static void realtek8201_check_duplex(struct net_device* dev);
static void natsemi_check_speed(struct net_device* dev);
static void natsemi_check_duplex(struct net_device* dev);
static void generic_check_speed(struct net_device* dev);
static void generic_check_duplex(struct net_device* dev);

struct transceiver_ops transceivers[] = 
{
	{0xC039, tdk_check_speed, tdk_check_duplex},            /* TDK 2120 */
	{0x039C, tdk_check_speed, tdk_check_duplex},            /* TDK 2120C */
	{0x0020, realtek8201_check_speed, realtek8201_check_duplex}, /* RealTek 8201 (RealTek cheats with OUI to MII PHY id mapping) */
	{0x80017, natsemi_check_speed, natsemi_check_duplex },  /* National Semiconductors DP83846A. */
	{0x0000, generic_check_speed, generic_check_duplex}     /* Generic, must be last */
};

// Support routines for handling MII communications with the PHY.
// 
// The MAC handles the serial protocol, we just need to write to
// some registers and wait for completion.
//
// TODO: The write routine does not wait for completion - this
// could upset code that relies on synchronous updates of the PHY
// state (check).

static void 
set_mdio_reg(int addr, int data)
{
	D(printk("set_mdio_reg: addr=%x data=%x\n", addr, data));
	// Wait for BUSY flag to deassert
	while(GET_MD_CA_BUSY(SOC_ETHERNET.MD_CA) == 1)
		;
	SOC_ETHERNET.MD_DATA = data;
	SOC_ETHERNET.MD_CA =
		SET_MD_CA_ADDR(addr) |
		SET_MD_CA_PHY(mdio_phy_addr) |
		SET_MD_CA_BUSY(1) |
		SET_MD_CA_WR(1);
}

static int
get_mdio_reg(int addr)
{
	int data;
	// Wait for BUSY flag to deassert
	while(GET_MD_CA_BUSY(SOC_ETHERNET.MD_CA) == 1)
		;
	SOC_ETHERNET.MD_CA = 
		SET_MD_CA_ADDR(addr) |
		SET_MD_CA_PHY(mdio_phy_addr) |
		SET_MD_CA_BUSY(1) |
		SET_MD_CA_WR(0);
	
	// Wait for BUSY flag to deassert
	while(GET_MD_CA_BUSY(SOC_ETHERNET.MD_CA) == 1)
		;
 	data = SOC_ETHERNET.MD_DATA;
 	D(printk("get_mdio_reg: addr=%x data=%x\n", addr, data));
	return data;
}

// Some PHY startup code

#if 0
// Not used currently, because.. well.. it is not needed I guess...

static void 
phy_reset(void)
{
	int readval,i;
	readval = get_mdio_reg(MII_BMCR);
	readval = readval | BMCR_RESET;
	set_mdio_reg(readval, MII_BMCR);
	i = 0;
	while(get_mdio_reg(MII_BMCR) & BMCR_RESET)
		if (++i > 100) {
			printk("ERROR: resetting PHY");
			break;
		}
}

// Setup PHY with some initial values (turn it on etc).

static void
setup_phy(void)
{
	int readval;

	phy_reset();
	readval = get_mdio_reg(MII_BMCR);
	readval = readval & ~BMCR_ISOLATE; // turn off isolate mode
#ifdef CONFIG_ARGUS_FPGA
	// Lock the FPGA to a slow mode, it can't handle 100 mbit.
	readval = readval & ~BMCR_ANENABLE; // turn off auto negotiate
	readval = readval & ~BMCR_SPEED100;   // 10MBit
	readval = readval | BMCR_FULLDPLX;    // full duplex
#endif
	set_mdio_reg(readval, MII_BMCR);
	readval = get_mdio_reg(MII_BMCR);
}
#endif

static void
phy_negotiate(struct net_device *dev)
{
	unsigned short data = get_mdio_reg(MII_ADVERTISE);

	/* Discard old speed and duplex settings */
	data &= ~(ADVERTISE_100HALF | ADVERTISE_100FULL | 
		  ADVERTISE_10FULL | ADVERTISE_10HALF);
  
	switch (current_speed_selection) {
		case 10 :
			if (current_duplex == full)
				data |= ADVERTISE_10FULL;
			else if (current_duplex == half)
				data |= ADVERTISE_10HALF;
			else
				data |= ADVERTISE_10HALF | ADVERTISE_10FULL;
			break;

		case 100 :
			 if (current_duplex == full)
				data |= ADVERTISE_100FULL;
			else if (current_duplex == half)
				data |= ADVERTISE_100HALF;
			else
				data |= ADVERTISE_100HALF | ADVERTISE_100FULL;
			break;

		case 0 : /* Auto */
			 if (current_duplex == full)
				data |= ADVERTISE_100FULL | ADVERTISE_10FULL;
			else if (current_duplex == half)
				data |= ADVERTISE_100HALF | ADVERTISE_10HALF;
			else
				data |= ADVERTISE_100HALF | ADVERTISE_100FULL | 
					ADVERTISE_10FULL | ADVERTISE_10HALF;
			break;

		default : /* assume autoneg speed and duplex */
			data |= ADVERTISE_100HALF | ADVERTISE_100FULL | 
				ADVERTISE_10FULL | ADVERTISE_10HALF;

	}

	set_mdio_reg(MII_ADVERTISE, data); 

	/* Renegotiate with link partner */
	data = get_mdio_reg(MII_BMCR);
	data |= BMCR_ANRESTART;
	set_mdio_reg(MII_BMCR, data); 
}

static void
generic_check_speed(struct net_device *dev)
{
	unsigned long data;
	data = get_mdio_reg(MII_ADVERTISE);
	if ((data & ADVERTISE_100FULL) ||
	    (data & ADVERTISE_100HALF))
		current_speed = 100;
	else
		current_speed = 10;
}

static void
tdk_check_speed(struct net_device *dev)
{
	unsigned long data;
	data = get_mdio_reg(MDIO_TDK_DIAGNOSTIC_REG);
	current_speed = (data & MDIO_TDK_DIAGNOSTIC_RATE ? 100 : 10);
}

static void
realtek8201_check_speed(struct net_device *dev)
{
	unsigned long data;
	data = get_mdio_reg(MII_BMCR);
	current_speed = (data & BMCR_SPEED100) ? 100 : 10;
}

static void
natsemi_check_speed(struct net_device *dev)
{
	unsigned long data;
	data = get_mdio_reg(MDIO_NATSEMI_PHYSTS_REG);
	current_speed = (data & MDIO_NATSEMI_PHYSTS_SPEED) ? 10 : 100;
}

static void
check_speed(unsigned long d)
{
	unsigned long data;
	int old_speed;

	if(running) {
		old_speed = current_speed;

		data = get_mdio_reg(MII_BMSR);
		
		if (!(data & BMSR_LSTATUS)) {
			current_speed = 0;
		} else {
			transceiver->check_speed((struct net_device*)d);
		}
		
		if (old_speed != current_speed)
			set_network_leds(NO_NETWORK_ACTIVITY);
	}

	/* Reinitialize the timer. */
	speed_timer.expires = jiffies + NET_LINK_UP_CHECK_INTERVAL;
	add_timer(&speed_timer);
}

static void
set_speed(struct net_device* dev, unsigned long speed)
{
	current_speed_selection = speed;
	phy_negotiate(dev);
}

static void
generic_check_duplex(struct net_device* dev)
{
	unsigned long data;
	data = get_mdio_reg(MII_ADVERTISE);
	if ((data & ADVERTISE_10FULL) ||
	    (data & ADVERTISE_100FULL))
		full_duplex = 1;
	else
		full_duplex = 0;
}

static void
tdk_check_duplex(struct net_device* dev)
{
	unsigned long data;
	data = get_mdio_reg(MDIO_TDK_DIAGNOSTIC_REG);
	full_duplex = (data & MDIO_TDK_DIAGNOSTIC_DPLX) ? 1 : 0;
}

static void
realtek8201_check_duplex(struct net_device* dev)
{
	unsigned long data;
	data = get_mdio_reg(MII_BMCR);
	full_duplex = (data & BMCR_FULLDPLX) ? 1 : 0;
}

static void
natsemi_check_duplex(struct net_device *dev)
{
	unsigned long data;
	data = get_mdio_reg(MDIO_NATSEMI_PHYSTS_REG);
	full_duplex = (data & MDIO_NATSEMI_PHYSTS_DPLX) ? 1 : 0;}

static void
check_duplex(unsigned long d)
{
	unsigned long data;

	if(running) {
		int old_duplex = full_duplex;

		transceiver->check_duplex((struct net_device*)d);
		if (old_duplex != full_duplex) {
			if (full_duplex) {
				/* Duplex changed to full. */
				SOC_ETHERNET.MAC_CTL = SOC_ETHERNET.MAC_CTL | 
					SET_MAC_CTL_FULLDUP(1);
			} else {
				/* Duplex changed to half. */
				SOC_ETHERNET.MAC_CTL = SOC_ETHERNET.MAC_CTL & 
					~SET_MAC_CTL_FULLDUP(1);
			}		
		}
	}

	/* Reinitialize the timer. */
	duplex_timer.expires = jiffies + NET_DUPLEX_CHECK_INTERVAL;
	add_timer(&duplex_timer);
}

static int 
probe_transceiver(struct net_device* dev)
{
	unsigned int phyid_high;
	unsigned int phyid_low;
	unsigned int oui;
	struct transceiver_ops* ops = NULL;

	/* Probe MDIO physical address */
	for (mdio_phy_addr = 0; mdio_phy_addr <= 31; mdio_phy_addr++) {
		if (get_mdio_reg(MII_BMSR) != 0xffff)
			break;
	}
	printk(KERN_INFO "PHY MII address %d\n", mdio_phy_addr);
	if (mdio_phy_addr == 32)
		return -ENODEV;

	/* Get manufacturer */
	phyid_high = get_mdio_reg(MII_PHYSID1);
	phyid_low = get_mdio_reg(MII_PHYSID2);
	oui = (phyid_high << 6) | (phyid_low >> 10);

	for (ops = &transceivers[0]; ops->oui; ops++) {
		if (ops->oui == oui)
			break;
	}
	D(printk(KERN_INFO "transceiver ID1=%x ID2=%x, OUI=%x\n", phyid_high, phyid_low, oui));
	transceiver = ops;

	return 0;
}

static void 
set_duplex(struct net_device* dev, enum duplex new_duplex)
{
	current_duplex = new_duplex;
	phy_negotiate(dev);
}

static void
clear_network_leds(unsigned long dummy)
{
	if (led_active && time_after(jiffies, led_next_time)) {
		set_network_leds(NO_NETWORK_ACTIVITY);

		/* Set the earliest time we may set the LED */
		led_next_time = jiffies + NET_FLASH_PAUSE;
		led_active = 0;
	}
}

static void
set_network_leds(int active)
{
	int light_leds = (active == NO_NETWORK_ACTIVITY);

	if (!current_speed) {
		/* LED off, link is down */
		LED_NETWORK_SET(LED_OFF);
	}
	else if (light_leds) {
		if (current_speed == 10) {
			LED_NETWORK_SET(LED_ORANGE);
		} else {
			LED_NETWORK_SET(LED_GREEN);
		}
	}
	else {
		LED_NETWORK_SET(LED_OFF);
	}
}

static void
led_flash(void)
{
  	if (!led_active && time_after(jiffies, led_next_time)) {
		/* light the network leds depending on the current speed. */
		set_network_leds(NETWORK_ACTIVITY);

		/* Set the earliest time we may clear the LED */
		led_next_time = jiffies + NET_FLASH_TIME;
		led_active = 1;
		mod_timer(&clear_led_timer, jiffies + HZ/10);
	}
}

// DMA support routines

// When the DMA accesses the ethernet MAC's status registers it needs the 
// physical addresses, so we need to do some ugly translation here..

#define RXSTATUS_PHYSADDR ((((unsigned int)&SOC_ETHERNET.RX_STATUS) - \
                            APB_NOCACHE) + APB_BASE_P)

#define TXSTATUS_PHYSADDR ((((unsigned int)&SOC_ETHERNET.TX_STATUS) - \
                            APB_NOCACHE) + APB_BASE_P)

// The status ends up in DESC3 in the descriptor

#define GETSTATUS(cha,desc) (SOC_DMA.CHANNEL[cha].DESCQUEUE[desc].DESC3)

#define GET_DMA_DESCCNT(cha) (GET_DMA_CHASTATUS_DESCCNT(SOC_DMA.CHANNEL[cha].CHASTATUS))

static void
eth_setup_dma(struct eth_priv *priv)
{
	printk("  Setting up ethernet DMA..\n");

	priv->dma_rx = dma_alloc(rxdma_handler, DMA_ALLOC_FIFO128);
	priv->dma_tx = dma_alloc(txdma_handler, DMA_ALLOC_FIFO128);
	
	if(priv->dma_rx < 0 || priv->dma_tx < 0)
		panic("Can't alloc eth dma chs");

	// This is done instead of calling dma_setup, because ethernet 
	// uses special flags not implemented there yet.
	
	// The RX channel can use a FIFO threshold of 32 bytes - the DMA
	// will wait to write to the memory from the FIFO until it can
	// use a burst of 32 bytes. 

	SOC_DMA.CHANNEL[priv->dma_rx].CHACONFIG = 
		SET_DMA_CHACONFIG_PORT(DMA_PORT_ETHRX) |
		SET_DMA_CHACONFIG_PRIO(1)              |
		SET_DMA_CHACONFIG_SIZE(DMA_SIZE_WORD)  |
		SET_DMA_CHACONFIG_DIR(DMA_DEV2MEM)     |
		SET_DMA_CHACONFIG_MDEV(0)              |
		SET_DMA_CHACONFIG_FIFOTHRES(1);
	
	// TX DMA channel using a FIFO threshold of 32 bytes meaning that
	// the DMA will wait to update the FIFO until it can fit a new
	// burst of 32 bytes. This should give a good compromise between
	// performance and FIFO latency.

	SOC_DMA.CHANNEL[priv->dma_tx].CHACONFIG = 
		SET_DMA_CHACONFIG_PORT(DMA_PORT_ETHTX) |
		SET_DMA_CHACONFIG_PRIO(1)              |
		SET_DMA_CHACONFIG_SIZE(DMA_SIZE_WORD)  |
		SET_DMA_CHACONFIG_DIR(DMA_MEM2DEV)     |
		SET_DMA_CHACONFIG_MDEV(0)              |
		SET_DMA_CHACONFIG_FIFOTHRES(1);
	
	// Reset channels

	dma_abort(priv->dma_rx);
	dma_abort(priv->dma_tx);
	
	// Enable channels

	SOC_DMA.CHANNEL[priv->dma_rx].CHACTRL = 
		SET_DMA_CHACTRL_DEVEN(1) |
		SET_DMA_CHACTRL_BUFEN(1);
	
	SOC_DMA.CHANNEL[priv->dma_tx].CHACTRL = 
		SET_DMA_CHACTRL_DEVEN(1) |
		SET_DMA_CHACTRL_BUFEN(1);
}

static void
eth_release_dma(struct eth_priv *priv)
{
	// Reset channels, TODO check if this is the right way

	dma_abort(priv->dma_rx);
	dma_abort(priv->dma_tx);

	// Free them.

	dma_free(priv->dma_rx);
	dma_free(priv->dma_tx);
}


// Setup a descriptor and add to the hardware queue
//
// bufaddr needs to be a physical address (virt_to_phys for a kernel buf)
// bufsize must be long-word aligned upwards and fit a max packet size
// including the CRC field (1518 + align = 1520)

static void
eth_launch_rx(int cha, int bufaddr, int bufsize, int desc_index)
{
	// Setup the descriptor

	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC0 = bufaddr;

	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC1 = 
		SET_DMA_DESC1_BUFSIZE(bufsize) |
		SET_DMA_DESC1_PROT(15)    |   // cacheable & bufferable
		SET_DMA_DESC1_PRIO(1)     |   // DMA higher prio than CPU
		SET_DMA_DESC1_OPTION(DMA_DESC1_OPTION_FRAMEREG) | // frame reg in DESC2 and DESC3
		SET_DMA_DESC1_IRQ(1)      |   // IRQ when descriptor has completed
		SET_DMA_DESC1_EOF(1);         // wait for EOF from device

	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC2 = RXSTATUS_PHYSADDR;

	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC3 = SET_RX_STATUS_RXFRMACTIVE(1);

	// Go go go!

	SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_ADDBUF(1);
}

// Setup a descriptor and add to the hardware queue
//
// bufaddr needs to be a physical address.

static void 
eth_launch_tx(int cha, int bufaddr, int bufsize, int desc_index)
{
	// Setup the descriptor

	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC0 = bufaddr;
	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC1 = 
		SET_DMA_DESC1_BUFSIZE((bufsize+3)&~3) |   // round up to 32 bit alignment
		SET_DMA_DESC1_PROT(15)    |   // cacheable & bufferable
		SET_DMA_DESC1_PRIO(1)     |   // DMA higher prio than CPU
		SET_DMA_DESC1_OPTION(DMA_DESC1_OPTION_FRAMEREG) | // frame reg in DESC2 and DESC3
		SET_DMA_DESC1_IRQ(1)      |   // IRQ when descriptor has completed
		SET_DMA_DESC1_EOF(1)      |   // wait for EOF from device
		SET_DMA_DESC1_INITFIFO(1);    // prefetch fifo data before starting to send

	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC2 = TXSTATUS_PHYSADDR;

	SOC_DMA.CHANNEL[cha].DESCQUEUE[desc_index].DESC3 = 
		SET_TX_STATUS_TXFRMACTIVE(1) | 
		SET_TX_STATUS_TXLENGTH(bufsize);  // exact size here

	// Go go go!

	SOC_DMA.CHANNEL[cha].CHACTRL = SET_DMA_CHACTRL_ADDBUF(1);
}

// Allocate and launch a new skb
static struct sk_buff *
launch_new_rx_skb(struct eth_priv *priv)
{
	struct sk_buff *skb;
	// Allocate a new skb to issue immediately
	// (remember to align the length to a 32-bit boundary)
	skb = dev_alloc_skb(1520);
	
	if (skb) {
		unsigned char *skb_data_ptr;
		int index = (priv->first_issued_rx + priv->cnt_issued_rx) % NUM_HW_DESCS;
		skb->dev = priv->dev;
		//printk("allocated new rx skb: %08x at index %d\n", skb, index);

		// The ethernet header is 14 bytes. We want to make sure that the 
		// following IP header is aligned to a 32-bit boundary, so we back up
		// the 32-bit aligned skbuf data start by 14 bytes before sending the
		// pointer to the DMA. dev_alloc_skb always allocates at least 16 
		// bytes headroom in the beginning for this.

		skb_data_ptr = skb_push(skb, ETHER_HEAD_LEN);

		eth_launch_rx(priv->dma_rx, virt_to_phys(skb_data_ptr), 1520, index);
		if(priv->rxframes[index].skb)
				printk("launch: occupied skb slot %d, 0x%p\n", 
				       index, priv->rxframes[index].skb);
		priv->rxframes[index].skb = skb;
		priv->cnt_issued_rx++;
	} else {
		priv->stats.rx_errors++;
		printk(KERN_NOTICE "Memory squeeze, dropping packet.\n");
		// TODO: If this happen we loose one buffer in the DMA. How to get it back?
	}
	return skb;
}			


// Update rx stats based on the status from the MAC
static void
update_rx_error_stats(struct eth_priv *priv, int status)
{
	priv->stats.rx_errors++;
	if(GET_RX_STATUS_OVERFLOW(status))
		priv->stats.rx_fifo_errors++;
	if(GET_RX_STATUS_OVERFLOW(status))
		priv->stats.rx_length_errors++;
	if(GET_RX_STATUS_CRCERR(status))
		priv->stats.rx_crc_errors++;
	if(GET_RX_STATUS_LONGERR(status))
		priv->stats.rx_length_errors++;
	if(GET_RX_STATUS_ALIGNERR(status))
		priv->stats.rx_frame_errors++;
	// TODO: merge in some of the other errors here as well
	// like PXPAR is a frame err I guess.
}

// These handlers are called as callbacks from the DMA channels,
// with irq's off.

static void
rxdma_handler(int cha)
{
	struct eth_priv *priv = g_priv;
	
	// Flash with the network LED
	led_flash();

	// Ack
	SOC_DMA.CHANNEL[priv->dma_rx].CHACTRL = SET_DMA_CHACTRL_IRQ(1);
	
	// Loop as long as there are completed descriptors in the DMA queue

	while(priv->cnt_issued_rx > GET_DMA_DESCCNT(priv->dma_rx)) {
		int status = GETSTATUS(priv->dma_rx, priv->first_issued_rx);
		struct sk_buff *skb = priv->rxframes[priv->first_issued_rx].skb;

		priv->rxframes[priv->first_issued_rx].skb = 0;
		priv->first_issued_rx = (priv->first_issued_rx + 1) % NUM_HW_DESCS;
		priv->cnt_issued_rx--;
		
		//printk("rx status= 0x%08x skb=%08x\n", status, skb);

		if (GET_RX_STATUS_GOOD(status)) {
			int length;

			length = GET_RX_STATUS_RXLENGTH(status);
                        	
			// If the packet is small, we allocate a new buffer and
			// copy it upwards in the stack to save memory. If it is
			// big, we simply send up the entire buffer even if it is
			// not filled completely. The breakeven value needs to be
			// tuned.
	
			if(length < RX_COPYBREAK) {
				struct sk_buff *small_skb;
				unsigned char *small_data;
				int index;

				small_skb = dev_alloc_skb(length - ETHER_HEAD_LEN);
				if (!small_skb) {
					priv->stats.rx_errors++;
					printk(KERN_NOTICE "%s: Memory squeeze, dropping packet.\n", 
					       skb->dev->name);
					// Ouch. If this happens, restart the skb and drop the
					// currently received packet.
					goto relaunch;
				}
				small_skb->dev = skb->dev;

				// The following put/push's are done so as to align the
				// ethernet payload to the original skb allocation 
				// alignment boundary.
				
				// allocate room for the packet body
				skb_put(small_skb, length - ETHER_HEAD_LEN);
				// allocate room for the header
				small_data = skb_push(small_skb, ETHER_HEAD_LEN);
				
				// Flush skb data buffer from the dcache
				dmac_inv_range((int)skb->data, 
					       ((int)skb->data) + length);
				// Copy the packet
				memcpy(small_data, skb->data, length);

				// Restart the DMA to the original skb

				index = (priv->first_issued_rx + 
					 priv->cnt_issued_rx) % NUM_HW_DESCS;

				eth_launch_rx(priv->dma_rx, virt_to_phys(skb->data),
					      ETH_MEDIA_LEN, index);

				priv->rxframes[index].skb = skb;
				priv->cnt_issued_rx++;

				small_skb->protocol = eth_type_trans(small_skb, 
								     small_skb->dev);
				
				// Update stats
				priv->stats.rx_packets++;
				priv->stats.rx_bytes += length;
				
				netif_rx(small_skb);
			
			} else {
				// Long packet - pass it up and allocate a new one.

				// Immediately allocate and launch a new skb
				launch_new_rx_skb(priv);
			
				// Flush skb data buffer from the dcache
				dmac_inv_range((int)skb->data, 
					       ((int)skb->data) + length);
			
				// Update the skb's length and protocol
				//
				// Since we pushed the ethernet header before, we need
				// to subtract it from the added length here.
				
				skb_put(skb, length - ETHER_HEAD_LEN);				
			
				skb->protocol = eth_type_trans(skb, skb->dev);
				
				// Update stats
				priv->stats.rx_packets++;
				priv->stats.rx_bytes += length;
				
				// Send it up to the higher levels - this concludes our 
				// responsibility for the allocated sk_buff.
				
				netif_rx(skb);
			}
		} else {
			// RX packet failed in some way
			// Relaunch same skb again

			int index;

		relaunch:
			
			index = (priv->first_issued_rx + 
				 priv->cnt_issued_rx) % NUM_HW_DESCS;

			eth_launch_rx(priv->dma_rx, 
				      virt_to_phys(skb->data), 
				      ETH_MEDIA_LEN, 
				      index);

			priv->rxframes[index].skb = skb;

			priv->cnt_issued_rx++;

			priv->stats.rx_packets++;
			update_rx_error_stats(priv,status);
		}
	}
	
	// Make sure we fill the DMA queue
	while(priv->cnt_issued_rx < NUM_HW_DESCS )
		if (!launch_new_rx_skb(priv))
			break;
}

// Update tx stats based on the status from the MAC
static void
update_tx_error_stats(struct eth_priv *priv, int status)
{
	if(!GET_TX_STATUS_COMP(status)) {
		printk("update_tx_error_stats called with Comp 0, status 0x%x\n",
		       status);
		return;
	}
	if(GET_TX_STATUS_EXCOLL(status)) {
		// Set if 16 collisions occur in the same packet.
		// Transmission skipped.
		_EPRINTK("TX excessive collisions detected");
		priv->stats.tx_errors++;
		priv->stats.collisions++;
	}
	if(GET_TX_STATUS_UNDER(status)) {
		// MAC transmit FIFO becomes empty during transmission
		_EPRINTK("TX Underrun detected");
		priv->stats.tx_errors++;
		priv->stats.tx_fifo_errors++;
	}
	if(GET_TX_STATUS_LOSTCRS(status) && !full_duplex) {
		// Carrier sense is not detected or is dropped during the
		// transmission of a packet (from the SFD to the CRC)
		_EPRINTK("TX no carrier detected");
		priv->stats.tx_errors++;
		priv->stats.tx_carrier_errors++;
	}
	if(GET_TX_STATUS_LATECOLL(status)) {
		// A collision occurs after 512-bit times (64-byte times)
		_EPRINTK("TX late collision detected");
		priv->stats.tx_errors++;
		priv->stats.tx_window_errors++;
	}
	if(GET_TX_STATUS_TXPAR(status)) {
		// MAC transmit FIFO has detected a parity error
		_EPRINTK("TX parity error detected");
		priv->stats.tx_errors++;
		priv->stats.tx_aborted_errors++;
	}
}

// Called by the DMA irq scheduler. IRQs are off.
static void
txdma_handler(int cha)
{
	struct eth_priv *priv = g_priv;
	int original_issued_tx = priv->cnt_issued_tx;
	
	// Ack
	SOC_DMA.CHANNEL[priv->dma_tx].CHACTRL = SET_DMA_CHACTRL_IRQ(1);
	
	// Loop as long as there are completed descriptors in the DMA queue
	while(priv->cnt_issued_tx > GET_DMA_DESCCNT(priv->dma_tx)) {
		int status = GETSTATUS(priv->dma_tx, priv->first_issued_tx);
		struct sk_buff *skb = priv->txframes[priv->first_issued_tx].skb;
		priv->stats.tx_packets++;
		priv->stats.tx_bytes += skb->len;
		update_tx_error_stats(priv, status);
		dev_kfree_skb_irq(skb);
		priv->txframes[priv->first_issued_tx].skb = 0;
		priv->first_issued_tx = (priv->first_issued_tx + 1)%NUM_HW_DESCS;
		priv->cnt_issued_tx--;
	}
	
	if (original_issued_tx == NUM_HW_DESCS)
		// We are currently stopped
		if (priv->cnt_issued_tx < NUM_HW_DESCS )
			// Wakeup since the DMA has room for more packets
    			netif_wake_queue(priv->dev);

}

// Called by the stack to send a packet. If we notice that we fill
// up the queue slots in the TX DMA, we call the netif_stop_queue to
// block further attempts until a slot is freed.

static int 
__argus_eth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct eth_priv *priv = (struct eth_priv *) dev->priv;
	int index;
	unsigned long flags;
 	
	// Protect against interrupts coming for example after the launch
	// but before we got time to remember the slot used.

	local_irq_save(flags);

	// Safety check.

	if (priv->cnt_issued_tx >= NUM_HW_DESCS) {
		// Can't issue more buffers. Busy. This should not happen.
		local_irq_restore(flags);
		printk("start_xmit called while busy.\n");
		return -EBUSY;
	}
		
//	printk("txlen: %d, txaddr: 0x%08x\n", skb->len, skb->data);

	// Actually insert the packet into the hardware packet queue. If it
	// is empty, this will trigger the sending as well.

	dev->trans_start = jiffies;
  
	index = (priv->first_issued_tx + priv->cnt_issued_tx) % NUM_HW_DESCS; 

	eth_launch_tx(priv->dma_tx, virt_to_phys(skb->data), skb->len, index);

	priv->cnt_issued_tx++;
  
	// Remember the skb slot used
	priv->txframes[index].skb = skb;

	// Stop queue if the hardware transmit queue is full
	if (priv->cnt_issued_tx == NUM_HW_DESCS)
		netif_stop_queue(dev);
	
	// Flash with the network LED
	led_flash();
	
	local_irq_restore(flags);

	return 0;
}

static int 
__argus_eth_open(struct net_device *dev)
{
	struct eth_priv *priv = (struct eth_priv *) dev->priv;
	
	D(printk("eth_open\n"));
	
	// Setup dma
	eth_setup_dma(priv);

	// Reset and set up phy
	// (Already reset and setup)
	//setup_phy();

	// Reset MAC and wait for completion

	SOC_ETHERNET.MAC_CTL = SET_MAC_CTL_RESET(1);
	while(GET_MAC_CTL_RESET(SOC_ETHERNET.MAC_CTL) == 1)
		/* nothing */ ;

#ifdef CONFIG_ARGUS_FPGA
	// On the FPGA, the PHY is locked to full duplex and we set it here.
	// We should not do this on the sharp version, because we initialize the
	// full_duplex variable to 0 and use it to check if we need to change
	// anything after hub negotiation.
	SOC_ETHERNET.MAC_CTL = SOC_ETHERNET.MAC_CTL | SET_MAC_CTL_FULLDUP(1);
#endif

	// Setup our station address

	SOC_ETHERNET.CAM[0] = 
		(dev->dev_addr[0] << 24) |
		(dev->dev_addr[1] << 16) |
		(dev->dev_addr[2] <<  8) |
		(dev->dev_addr[3] <<  0);
	SOC_ETHERNET.CAM[1] = 
		(dev->dev_addr[4] << 24) |
		(dev->dev_addr[5] << 16);

	SOC_ETHERNET.CAM_ENA = (1<<0);	// Enable CAM entry 0
	//SOC_ETHERNET.CAM_ENA = 0;
	
	SOC_ETHERNET.CAM_CTL = 
		SET_CAM_CTL_COMPEN(1) |  // enable compare mode (check against the CAM)
		//SET_CAM_CTL_STATIONACC(1) | // accept any unicast packets (Promisc)
		SET_CAM_CTL_GROUPACC(0) | // accept any multicast packets
		SET_CAM_CTL_BROADACC(1); // accept broadcast packets

        // Maximum rx detect len (=>long latency)
	// TODO: Obsolete register, can probably be removed!

        SOC_ETHERNET.RX_DETECT_LEN = SET_RX_DETECT_LEN(32);

	// Setup TX/RX parts of the MAC

	// We don't need the interrupt bits set since we get the status through
	// the DMA.

	SOC_ETHERNET.TX_CTL = 
		SET_TX_CTL_TXEN(1) |       // MAC transmit enable
		SET_TX_CTL_ENUNDER(0) |    // irq on underrun
		SET_TX_CTL_ENLCARR(0) |    // irq on no carrier
		SET_TX_CTL_ENEXCOLL(0) |   // irq on excessive collisions
		SET_TX_CTL_ENLATECOLL(0) | // irq if collision occurs after 512 bit times(64 bytes times)
		SET_TX_CTL_ENTXPAR(0) |    // irq on parity error in the tx FIFO
		SET_TX_CTL_ENCOMP(0);      // irq when the mac transmits or discards a packet

	SOC_ETHERNET.RX_CTL = 
		SET_RX_CTL_RXEN(1) |      // enable MAC RX
		SET_RX_CTL_STRIPCRC(0) |  // strip CRC
		SET_RX_CTL_ENCRCERR(0) |  // interrupt on CRC error
		SET_RX_CTL_ENOVER(0) |    // interrupt on overflow error
		SET_RX_CTL_ENLONGERR(0) | // interrupt on long frame error
		SET_RX_CTL_ENRXPAR(0);    // interrupt on MAC FIFO parity error

	priv->cnt_issued_tx = 0;
	priv->cnt_issued_tx = 0;
	priv->first_issued_rx = 0;
	priv->first_issued_rx = 0;
  
	// Allow transmit
	netif_start_queue(dev);

	// Force RX irq. This will launch RX skbs the normal way
	SOC_DMA.CHANNEL[priv->dma_rx].CHACTRL = SET_DMA_CHACTRL_SETIRQ(1);

	// Mark us as running so the timer functions can
	// check speed and duplex with the PHY

	running++;
	
	return 0;
}

static int 
__argus_eth_stop(struct net_device *dev)
{
	struct eth_priv *priv = (struct eth_priv *) dev->priv;

	running = 0;
  	
  	D(printk("eth_stop\n"));
  
	netif_stop_queue(dev);

	// TODO: Do we need to explicit release allocated skbs?
	// TODO: check if there is something more to reset.

	// Stop the MAC

	SOC_ETHERNET.TX_CTL = 0;
	SOC_ETHERNET.RX_CTL = 0;

	// Stop the DMA

	eth_release_dma(priv);

	// Stop MAC clock
	SOC_CLK.DISABLE = CLK_ETH;
	
	// Stop PHY clock
	SOC_CLK.DISABLE = CLK_CLKGEN1;
	
	return 0;
}

struct net_device_stats *
__argus_eth_get_stats(struct net_device *dev)
{
	D(printk("eth_get_stats\n"));
	return &((struct eth_priv *)dev->priv)->stats;
}

// set MAC address of the interface. called from the core after a
// SIOCSIFADDR ioctl, and from the bootup.

static int
__argus_set_mac_address(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;
	int i;
	unsigned long cam_tmp = (unsigned long)SOC_ETHERNET.CAM[1];

	// Remember it

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	// Write it to the hardware	

	SOC_ETHERNET.CAM[0] = 
		(dev->dev_addr[0] << 24) |
		(dev->dev_addr[1] << 16) |
		(dev->dev_addr[2] << 24) |
		(dev->dev_addr[3] << 16);
	SOC_ETHERNET.CAM[1] = 
		(dev->dev_addr[4] << 24) |
		(dev->dev_addr[5] << 16) |
		(cam_tmp & 0xffff);

	// show it in the log as well

	printk(KERN_INFO "%s: changed MAC to ", dev->name);
	for (i = 0; i < 5; i++)
		printk("%02X:", dev->dev_addr[i]);
	printk("%02X\n", dev->dev_addr[i]);

	return 0;
}

// The following ioctl copied from drivers/net/ax88796.c, I haven't
// tested it.

static int
argus_ethtool_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct ethtool_cmd ecmd;

	if (copy_from_user(&ecmd, ifr->ifr_data, sizeof (ecmd)))
		return -EFAULT;

	switch (ecmd.cmd) {
		case ETHTOOL_GSET:
		{
			memset((void *) &ecmd, 0, sizeof (ecmd));
			ecmd.supported = 
			  SUPPORTED_Autoneg | SUPPORTED_TP | SUPPORTED_MII |
			  SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full | 
			  SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full;
			ecmd.port = PORT_TP;
			ecmd.transceiver = XCVR_EXTERNAL;
			ecmd.phy_address = mdio_phy_addr;
			ecmd.speed = (current_speed == 100) ? SPEED_100 : (current_speed ? SPEED_10 : 0);
			ecmd.duplex = full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
			ecmd.advertising = ADVERTISED_TP;
			if (current_duplex == autoneg && current_speed_selection == 0)
				ecmd.advertising |= ADVERTISED_Autoneg;
			else {
				ecmd.advertising |= 
				  ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
				  ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full;
				if (current_speed_selection == 10)
					ecmd.advertising &= ~(ADVERTISED_100baseT_Half |
							      ADVERTISED_100baseT_Full);
				else if (current_speed_selection == 100)
					ecmd.advertising &= ~(ADVERTISED_10baseT_Half | 
							      ADVERTISED_10baseT_Full);
				if (current_duplex == half)
					ecmd.advertising &= ~(ADVERTISED_10baseT_Full |
							      ADVERTISED_100baseT_Full);
				else if (current_duplex == full)
					ecmd.advertising &= ~(ADVERTISED_10baseT_Half |
							      ADVERTISED_100baseT_Half);
			}
			ecmd.autoneg = AUTONEG_ENABLE;
			if (copy_to_user(ifr->ifr_data, &ecmd, sizeof (ecmd)))
				return -EFAULT;
		}
		break;
		case ETHTOOL_SSET:
		{
			if (!capable(CAP_NET_ADMIN)) {
				return -EPERM;
			}
			if (ecmd.autoneg == AUTONEG_ENABLE) {
				set_duplex(dev, autoneg);
				set_speed(dev, 0);
			} else {
				set_duplex(dev, ecmd.duplex == DUPLEX_HALF ? half : full);
				set_speed(dev, ecmd.speed == SPEED_10 ? 10: 100);
			}
		}
		break;
		case ETHTOOL_GDRVINFO:
		{
			struct ethtool_drvinfo info;
			memset((void *) &info, 0, sizeof (info));
			strncpy(info.driver, "Samsung MAC", sizeof(info.driver) - 1);
			strncpy(info.version, "$Rev$", sizeof(info.version) - 1);
			strncpy(info.fw_version, "N/A", sizeof(info.fw_version) - 1);
			strncpy(info.bus_info, "N/A", sizeof(info.bus_info) - 1);
			info.regdump_len = 0;
			info.eedump_len = 0;
			info.testinfo_len = 0;
			if (copy_to_user(ifr->ifr_data, &info, sizeof (info)))
				return -EFAULT;
		}
		break;
		case ETHTOOL_NWAY_RST:
			if (current_duplex == autoneg && current_speed_selection == 0)
				phy_negotiate(dev);
		break;
		case ETHTOOL_GLINK:
		{
			struct ethtool_value edata = { ETHTOOL_GLINK };
			unsigned long data;
			data = get_mdio_reg(MII_BMSR);
			edata.data = (data & BMSR_LSTATUS) ? 1 : 0;
			if (copy_to_user(ifr->ifr_data, &edata, sizeof(edata)))
				return -EFAULT;
		}
		break;
		default:
			return -EOPNOTSUPP;
		break;
	}
	return 0;
}


// Set up address filtering.
// IFF_PROMISC    	Promiscuous mode, receive all packets
// IFF_MULTI      	Normal mode, receive all multicast packets
// num_addrs == 0	Normal mode, clear multicast list
// num_addrs > 0	Multicast mode, receive normal and MC packets,
//			and do best-effort filtering.
// ARGUS-3 only has three entries in its address filter so we can only filter
// on two group addresses, and go to promiscuous group address filtering
// if more addresses are enabled on the interface.

static void
__argus_eth_set_multicast_list(struct net_device *dev)
{
	if (dev->flags & IFF_PROMISC) {
		SOC_ETHERNET.CAM_CTL = 
			SET_CAM_CTL_COMPEN(1) |  // enable compare mode (check against the CAM)
			SET_CAM_CTL_STATIONACC(1) | // accept any unicast packets (Promisc)
			SET_CAM_CTL_GROUPACC(1) | // accept any multicast packets
			SET_CAM_CTL_BROADACC(1); // accept broadcast packets

		SOC_ETHERNET.CAM_ENA = (1<<0); // Enable only station address CAM entry.
	} else if (dev->flags & IFF_ALLMULTI) {
		/* Enable all multicasts */
		SOC_ETHERNET.CAM_CTL = 
			SET_CAM_CTL_COMPEN(1) |  // enable compare mode (check against the CAM)
			SET_CAM_CTL_STATIONACC(0) | // accept any unicast packets (Promisc)
			SET_CAM_CTL_GROUPACC(1) | // accept any multicast packets
			SET_CAM_CTL_BROADACC(1); // accept broadcast packtes

		SOC_ETHERNET.CAM_ENA = (1<<0); // Enable only station address CAM entry.
	} else if (dev->mc_count == 0) {
		/* Normal, no group addresses being received. */
		SOC_ETHERNET.CAM_CTL = 
			SET_CAM_CTL_COMPEN(1) |  // enable compare mode (check against the CAM)
			SET_CAM_CTL_STATIONACC(0) | // accept any unicast packets (Promisc)
			SET_CAM_CTL_GROUPACC(0) | // accept any multicast packets
			SET_CAM_CTL_BROADACC(1); // accept broadcast packtes

		SOC_ETHERNET.CAM_ENA = (1<<0); // Enable only station address CAM entry.
	} else {
		/* MC mode, receive normal and MC packets */

		if (2 < dev->mc_count) {
			/* Above CAM address entry capacity, enable
			 * allmulti and let the stack handle
			 * filtering. */
			SOC_ETHERNET.CAM_CTL =
				SET_CAM_CTL_COMPEN(1) |  // enable compare mode (check against the CAM)
				SET_CAM_CTL_STATIONACC(0) | // accept any unicast packets (Promisc)
				SET_CAM_CTL_GROUPACC(1) | // accept any multicast packets
				SET_CAM_CTL_BROADACC(1); // accept broadcast packets

			SOC_ETHERNET.CAM_ENA = (1<<0); // Enable only station address CAM entry.
		} else {
			unsigned int cam_ena = (1<<0);  // The first CAM entry is always the station address.
			unsigned long cam_tmp = (unsigned long)SOC_ETHERNET.CAM[1];
			struct dev_mc_list *dmi = dev->mc_list;

			switch (dev->mc_count) {
			case 2:
				cam_ena |= (1<<18);
				SOC_ETHERNET.CAM[27] =
					(dmi->dmi_addr[0] << 24) |
					(dmi->dmi_addr[1] << 16) |
					(dmi->dmi_addr[2] <<  8) |
					(dmi->dmi_addr[3] <<  0);
				SOC_ETHERNET.CAM[28] =
					(dmi->dmi_addr[4] << 24) |
					(dmi->dmi_addr[5] << 16);

				dmi = dmi->next;
				/* fall through */
			case 1:
				cam_ena |= (1<<1);
				SOC_ETHERNET.CAM[1] =
					(cam_tmp & 0xffff0000) |
					(dmi->dmi_addr[0] <<  8) |
					(dmi->dmi_addr[1] <<  0);
				SOC_ETHERNET.CAM[2] =
					(dmi->dmi_addr[2] << 24) |
					(dmi->dmi_addr[3] << 16) |
					(dmi->dmi_addr[4] <<  8) |
					(dmi->dmi_addr[5] <<  0);

				SOC_ETHERNET.CAM_ENA = cam_ena;
				break;
			default:
				BUG_ON(1);
			}
			SOC_ETHERNET.CAM_CTL =
				SET_CAM_CTL_COMPEN(1) |  // enable compare mode (check against the CAM)
				SET_CAM_CTL_STATIONACC(0) | // accept any unicast packets (Promisc)
				SET_CAM_CTL_GROUPACC(0) | // accept any multicast packets
				SET_CAM_CTL_BROADACC(1); // accept broadcast packets
		}
	}
}


// IOCTL handler for ETHTOOL and MII register read/write

static int
__argus_eth_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct mii_ioctl_data *data = (struct mii_ioctl_data *)&ifr->ifr_data;

	switch (cmd) {
		case SIOCETHTOOL:
			return argus_ethtool_ioctl(dev,ifr);
		case SIOCGMIIPHY: /* Get PHY address */
			data->phy_id = mdio_phy_addr;
			break;
		case SIOCGMIIREG: /* Read MII register */
			data->val_out = get_mdio_reg(data->reg_num);
			break;
		case SIOCSMIIREG: /* Write MII register */
			set_mdio_reg(data->reg_num, data->val_in);
			break;
		default:
			return -EINVAL;	
	}
        return 0;
}

// This seems to be an alternative interface to ethtool for setting
// things like duplex and speed.

static int
__argus_set_config(struct net_device *dev, struct ifmap *map)
{
        struct eth_priv *priv = (struct eth_priv *) dev->priv;

	spin_lock(&priv->lock); // Preempt protection

	switch(map->port) {
		case IF_PORT_UNKNOWN:
			/* Use autoneg */
			set_speed(dev, 0);
			set_duplex(dev, autoneg);
			break;
		case IF_PORT_10BASET:
			set_speed(dev, 10);
			set_duplex(dev, autoneg);
			break;
		case IF_PORT_100BASET:
		case IF_PORT_100BASETX:
			set_speed(dev, 100);
			set_duplex(dev, autoneg);
			break;
		case IF_PORT_100BASEFX:
		case IF_PORT_10BASE2:
		case IF_PORT_AUI:
			spin_unlock(&priv->lock);
			return -EOPNOTSUPP;
			break;
		default:
			printk(KERN_ERR "%s: Invalid media selected", dev->name);
			spin_unlock(&priv->lock);
			return -EINVAL;
	}
	spin_unlock(&priv->lock);
	return 0;
}

static int __init
__argus_ethernet_init(void)
{
	struct net_device *dev;
	struct eth_priv *priv;
	unsigned long i;
	int err;

	printk(KERN_INFO "%s\n", __DRIVER_NAME);

	dev = alloc_etherdev(sizeof(struct eth_priv));

	if (!dev)
		return -ENOMEM;

	priv = dev->priv;
	// Remember in a global variable for the irq handlers
	g_priv = priv;

	/* assign net_device methods */
	dev->open = __argus_eth_open;
	dev->stop = __argus_eth_stop;
	dev->do_ioctl = __argus_eth_ioctl;
	dev->get_stats = __argus_eth_get_stats;
	dev->set_multicast_list = __argus_eth_set_multicast_list;
//	dev->tx_timeout = __argus_eth_tx_timeout;
	dev->hard_start_xmit = __argus_eth_start_xmit;
	dev->set_mac_address = __argus_set_mac_address;
        dev->set_config = __argus_set_config;

	dev->irq = IRQ_ETHERNET;  // TODO: What is it used for?
	dev->tx_queue_len = 2;    // ?? TODO: What is it used for?
	dev->dma = 0;
	dev->watchdog_timeo = HZ;

	/* set a default MAC address */
	dev->dev_addr[0] = 0x00;
	dev->dev_addr[1] = 0x40;
	dev->dev_addr[2] = 0x8c;
	dev->dev_addr[3] = 0xcd;
	dev->dev_addr[4] = 0x01; // 0x00
	dev->dev_addr[5] = 0x00;
	
	memset(priv, 0, sizeof(struct eth_priv));
	priv->dev = dev;

	SET_MODULE_OWNER(dev);

	spin_lock_init(&priv->lock);
	
	// Set all Ethernet pins as alternative functions
	// The ethernet pins are ranging from GPIO_XI_COL (73) to GPIO_MDIO (91)
	for(i = GPIO_XI_COL; i < GPIO_MDIO + 1; i++)
		SOC_GPIO.PIN[i] = SET_GPIO_MODE(GPIO_MODE_ALT_FUNC);

        // Start MAC and PHY
	SOC_CLK.ENABLE = CLK_ETH;

	// Set PHY to electrical reset (active low)
	gpio_set_mode(GPIO_PHYRESET, GPIO_MODE_OUT, 0);

	// Enable PHY clock: 150MHz/6=25MHz clock, ==> hi=2, lo=2
	// TODO: we can remove this I guess, since we use a xtal anyway now
	SOC_CLKGEN[CLKGEN_PHYCLK_OUT].CTRL = SET_CLKGEN_LO(2) | SET_CLKGEN_HI(2);
	SOC_CLK.ENABLE = CLK_CLKGEN1;

	// The clock needs to be stable when we release reset on the
	// PHY so we wait a bit.
	i = jiffies + 10;
	while(time_before(jiffies, i)) /* nothing */ ;

	// Release PHY reset
	gpio_set_mode(GPIO_PHYRESET, GPIO_MODE_OUT, 1);

	udelay(200);

	// Probe PHY
	if (probe_transceiver(dev)) {
		free_netdev(dev);
		return -EAGAIN;
	}

	/* Register device */
	err = register_netdev(dev);
	if (err) {
		free_netdev(dev);
		return err;
	}

	// Initialize speed indicator stuff

	current_speed = 10;
	current_speed_selection = 0; /* Auto */
	speed_timer.expires = jiffies + NET_LINK_UP_CHECK_INTERVAL;
	speed_timer.function = check_speed;
	speed_timer.data = (unsigned long)dev;
	add_timer(&speed_timer);
        
        clear_led_timer.function = clear_network_leds;

	full_duplex = 0;
	current_duplex = autoneg;
	duplex_timer.expires = jiffies + NET_DUPLEX_CHECK_INTERVAL;		
	duplex_timer.function = check_duplex;
	duplex_timer.data = (unsigned long)dev;
	add_timer(&duplex_timer);

	phy_negotiate(dev);

	return 0;
}

// Call at boot/module initialization
module_init(__argus_ethernet_init);

MODULE_DESCRIPTION("Argus3 ethernet driver (Samsung's MAC block)");
MODULE_AUTHOR("Bjorn Wesen <bjornw@axis.com>");
MODULE_LICENSE("GPL");
