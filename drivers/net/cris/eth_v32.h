/*
 * Definitions for ETRAX FS ethernet driver.
 *
 * Copyright (C) 2003, 2004, 2005 Axis Communications.
 */

#ifndef _ETRAX_ETHERNET_H_
#define _ETRAX_ETHERNET_H_

#include <asm/arch/hwregs/dma.h>

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
#include <asm/arch/cryptocop.h>
#endif

#define MAX_MEDIA_DATA_SIZE 1522	/* Max packet size. */

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
#define NBR_RX_DMA_DESC 64
#define NBR_RX_DESC 192			/* Number of RX descriptors. */
#define NBR_TX_DESC 284			/* Number of TX descriptors. */
#else
#define NBR_RX_DESC 64			/* Number of RX descriptors. */
#define NBR_TX_DESC 16			/* Number of TX descriptors. */
#endif

/* Large packets are sent directly to upper layers while small packets
 * are copied (to reduce memory waste).  The following constant
 * decides the breakpoint.
 */
#define RX_COPYBREAK        (256)

#define ETHER_HEAD_LEN      (14)

/* 
** MDIO constants.
*/
#define MDIO_START                          0x1
#define MDIO_READ                           0x2
#define MDIO_WRITE                          0x1
#define MDIO_PREAMBLE              0xfffffffful

/* Broadcom specific */
#define MDIO_AUX_CTRL_STATUS_REG           0x18
#define MDIO_BC_FULL_DUPLEX_IND             0x1
#define MDIO_BC_SPEED                       0x2

/* TDK specific */
#define MDIO_TDK_DIAGNOSTIC_REG              18
#define MDIO_TDK_DIAGNOSTIC_RATE          0x400
#define MDIO_TDK_DIAGNOSTIC_DPLX          0x800

/*Intel LXT972A specific*/
#define MDIO_INT_STATUS_REG_2		   0x0011
#define MDIO_INT_FULL_DUPLEX_IND ( 0x0001 << 9  )
#define MDIO_INT_SPEED		 ( 0x0001 << 14 )


/* Network flash constants */
#define NET_FLASH_TIME                  (HZ/50) /* 20 ms */
#define NET_FLASH_PAUSE                 (HZ/100) /* 10 ms */
#define NET_LINK_UP_CHECK_INTERVAL	(2*HZ)	/* 2 seconds. */
#define NET_DUPLEX_CHECK_INTERVAL	(2*HZ)	/* 2 seconds. */

#define NO_NETWORK_ACTIVITY 0
#define NETWORK_ACTIVITY    1

/* Duplex settings. */
enum duplex {
	half,
	full,
	autoneg
};

/* Some transceivers requires special handling. */
struct transceiver_ops {
	unsigned int oui;
	void (*check_speed) (struct net_device * dev);
	void (*check_duplex) (struct net_device * dev);
};

typedef struct crisv32_eth_descr {
	dma_descr_data descr __attribute__ ((__aligned__(32)));
	struct sk_buff *skb;
	unsigned char *linearized_packet;
} crisv32_eth_descr;


#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
struct crisv32_eth_csum_queue_entry {
	struct crisv32_eth_csum_queue_entry *next;
	struct sk_buff *skb;
	unsigned char *linearized_packet;
	unsigned char use_hw:1;
	unsigned char csum_done:1;
	unsigned char csum_failed:1;
};

#define MAX_NBR_TX_CSUM (64)


struct crisv32_eth_tx_strcop_data {
	dma_descr_data ddesc_out[MAX_SKB_FRAGS+3] __attribute__ ((__aligned__(32)));
	/* Descriptors to be passed to the strcop for
	   csum/sg processing.  MAX_SKB_FRAGS (skbuff.h) + 3 long
	   to make room for the max amount of fragments
	   (1 + MAX_SKB_FRAGS), the fragment split between two descriptors
	   when checksumming starts, and a possible 1 byte pad to make
	   the length of the csum indata a multiple of 2. */
	dma_descr_data ddesc_in[3] __attribute__ ((__aligned__(32)));
	/* Descriptors to be passed to the strcop to receive result
	   of csum/sg processing.  3 long to make room for the copy
	   results (two descriptors) and csum output. */

	struct cryptocop_operation cop;
	unsigned int cop_flags;
	/* COP_ACTIVE 0x01
	   Flag for marking the cryptocop_operation as used, i.e.
	   a TX csum operation is queued to the cryptocop. */

	unsigned char *discard_data;
	/* Pointer to data to free for a dropped operation. */
	struct sk_buff *discard_skb;
	/* Pointer to skb to free for a dropped (due to csum FIFO
	   flush) operation. */

	/* The csum queue is full if fifo_start == fifo_end and
	 * fifo_len > 0.  Entrys are inserted at fifo_end and removed
	 * at fifo_start.  */
	unsigned short int fifo_len;
	struct crisv32_eth_csum_queue_entry *fifo_start;
	struct crisv32_eth_csum_queue_entry *fifo_end;
	/* The packet currently being processed by the cryptocop.  */
	struct crisv32_eth_csum_queue_entry *fifo_active;
	struct crisv32_eth_csum_queue_entry csum_queue[MAX_NBR_TX_CSUM];

	/* Storage and postprocessing info for HW csum result. */
	unsigned short int csum_result;
	unsigned short int csum_fixup;
	unsigned short *csum_pos;
};

struct crisv32_eth_rx_strcop_data {
	dma_descr_data ddesc_out[2] __attribute__ ((__aligned__(32))); 
	/* Descriptors to be passed to the strcop for RX
	   csum operations.  2 long to make room for the
	   RX receive data and a possible 1 byte pad to
	   make the length of the csum indata a multiple
	   of 2. */
	dma_descr_data ddesc_in[1] __attribute__ ((__aligned__(32)));
	/* Descriptors to be passed to the strcop to receive
	   result of RX csum.  1 long to make room for the
	   csum output.	*/

	struct cryptocop_operation cop;
	unsigned int cop_flags;
	/* COP_ACTIVE 0x01
	   Flag for marking the cryptocop_operation as used, i.e.
	   a TX csum operation is queued to the cryptocop. */

	struct sk_buff *discard_skb;
	/* Pointer to skb to free for a dropped (due to csum FIFO
	   flush) operation. */

	/* Storage and postprocessing info for HW csum result. */
	unsigned short int csum_result;
	unsigned short int csum_fixup;
};
#endif


/* Information that need to be kept for each device. */
struct crisv32_ethernet_local {
	dma_descr_context ctxt_in __attribute__ ((__aligned__(32)));
	dma_descr_context ctxt_out __attribute__ ((__aligned__(32)));

	crisv32_eth_descr *active_rx_desc;
	crisv32_eth_descr *prev_rx_desc;
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	crisv32_eth_descr *last_rx_desc;
	crisv32_eth_descr *csum_queue_rx_desc;

	size_t nbr_tx_list_free;
#endif

	crisv32_eth_descr *active_tx_desc;
	crisv32_eth_descr *prev_tx_desc;
	crisv32_eth_descr *catch_tx_desc;

	crisv32_eth_descr dma_rx_descr_list[NBR_RX_DESC];
	crisv32_eth_descr dma_tx_descr_list[NBR_TX_DESC];

	/* DMA and ethernet registers for the device. */
	int eth_inst;
	int dma_in_inst;
	int dma_out_inst;

	/* Network speed indication. */
	struct timer_list speed_timer;
	struct timer_list clear_led_timer;
	int current_speed;		/* Speed read from tranceiver */
	int current_speed_selection;	/* Speed selected by user */
	int led_active;
	unsigned long led_next_time;
	int sender_started;

	/* Duplex. */
	struct timer_list duplex_timer;
	int full_duplex;
	enum duplex current_duplex;

	struct net_device_stats stats;

	/* Transciever address. */
	unsigned int mdio_phy_addr;

	struct transceiver_ops *transceiver;

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	cryptocop_session_id sid;
	struct crisv32_eth_tx_strcop_data tx_strcop_data;
	struct crisv32_eth_rx_strcop_data rx_strcop_data;
#endif

	/* 
	 * TX control lock. This protects the transmit buffer ring state along
	 * with the "tx full" state of the driver.  This means all netif_queue
	 * flow control actions are protected by this lock as well.
	 */
	spinlock_t lock;

	spinlock_t led_lock; /* Protect LED state */
	spinlock_t transceiver_lock; /* Protect transceiver state. */

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	/* 
	 * RX control lock. This protects the receive buffer ring state during
	 * calls from the ethernet RX interrupt and cryptocop callback.
	 */
	spinlock_t rx_lock;
#endif	
};

/* Function prototypes. */
static int crisv32_ethernet_init(void);
static int crisv32_ethernet_device_init(struct net_device* dev);
static int crisv32_eth_open(struct net_device *dev);
static int crisv32_eth_close(struct net_device *dev);
static int crisv32_eth_set_mac_address(struct net_device *dev, void *vpntr);
static irqreturn_t crisv32rxtx_eth_interrupt(int irq, void *dev_id,
					     struct pt_regs *regs);
static irqreturn_t crisv32nw_eth_interrupt(int irq, void *dev_id,
					   struct pt_regs *regs);
static void crisv32_eth_receive_packet(struct net_device *dev);
#ifndef CONFIG_ETRAX_ETHERNET_HW_CSUM
static int crisv32_eth_send_packet(struct sk_buff *skb, struct net_device *dev);
static void crisv32_eth_hw_send_packet(unsigned char *buf, int length,
				       void *priv);
#else
static void crisv32_eth_hw_send_packet(unsigned char *buf, int length,
				       int phys_addr, void *priv);
#endif
static void crisv32_eth_tx_timeout(struct net_device *dev);
static void crisv32_eth_set_multicast_list(struct net_device *dev);
static int crisv32_eth_ioctl(struct net_device *dev, struct ifreq *ifr,
			     int cmd);
static int crisv32_ethtool_ioctl(struct net_device *dev, struct ifreq *ifr);

static void crisv32_eth_negotiate(struct net_device *dev);
static void crisv32_eth_check_speed(unsigned long idev);
static void crisv32_eth_set_speed(struct net_device *dev, unsigned long speed);
static void crisv32_eth_check_duplex(unsigned long idev);
static void crisv32_eth_set_duplex(struct net_device *dev, enum duplex);
static int crisv32_eth_probe_transceiver(struct net_device *dev);

static void generic_check_speed(struct net_device *dev);
static void generic_check_duplex(struct net_device *dev);
static void broadcom_check_speed(struct net_device *dev);
static void broadcom_check_duplex(struct net_device *dev);
static void tdk_check_speed(struct net_device *dev);
static void tdk_check_duplex(struct net_device *dev);
static void intel_check_speed(struct net_device* dev);
static void intel_check_duplex(struct net_device *dev);

#ifdef CONFIG_NET_POLL_CONTROLLER
static void crisv32_netpoll(struct net_device* dev);
#endif

static void crisv32_clear_network_leds(unsigned long dummy);
static void crisv32_set_network_leds(int active, struct net_device* dev);

static void crisv32_eth_reset_tranceiver(struct net_device *dev);
static unsigned short crisv32_eth_get_mdio_reg(struct net_device *dev,
					       unsigned char reg_num);
static void crisv32_eth_set_mdio_reg(struct net_device *dev,
                                     unsigned char reg_num,
                                     int val);
static void crisv32_eth_send_mdio_cmd(struct net_device *dev,
				      unsigned short cmd, int write_cmd);
static void crisv32_eth_send_mdio_bit(struct net_device *dev,
				      unsigned char bit);
static unsigned char crisv32_eth_receive_mdio_bit(struct net_device *dev);

static struct net_device_stats *crisv32_get_stats(struct net_device *dev);
static void crisv32_start_dma_out(struct crisv32_ethernet_local* np);

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
static int crisv32_ethernet_skb_checksum(struct sk_buff *skb);

static void crisv32_eth_tx_hw_csum_callback(struct cryptocop_operation *cop,
					    void *data);

static void crisv32_eth_setup_tx_cryptocop_csum(struct net_device *dev);

static void crisv32_eth_tx_csum_queue_push(struct net_device *dev);

static void crisv32_eth_rx_hw_csum_callback(struct cryptocop_operation *cop,
					    void *data);

static void crisv32_eth_setup_rx_cryptocop_csum(struct net_device *dev);

static void crisv32_eth_rx_flush_csum_queue(struct net_device *dev);

static int crisv32_eth_send_packet_sg_csum(struct sk_buff *skb, struct net_device *dev);

static int crisv32_eth_hw_send_packet_skb(struct sk_buff *skb, void *priv);
#endif

#endif /* _ETRAX_ETHERNET_H_ */
