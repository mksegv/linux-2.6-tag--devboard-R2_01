/*
 * Driver for the ETRAX FS network controller.
 *
 * Copyright (c) 2003-2004 Axis Communications AB.
 */

#include <linux/module.h>

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/init.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/mii.h>

#include <asm/io.h>		/* LED_* I/O functions */
#include <asm/irq.h>
#include <asm/arch/hwregs/reg_map.h>
#include <asm/arch/hwregs/reg_rdwr.h>
#include <asm/arch/hwregs/dma.h>
#include <asm/arch/hwregs/eth_defs.h>
#include <asm/arch/hwregs/config_defs.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/ethernet.h>
#include <asm/arch/dma.h>
#include <asm/arch/intmem.h>
#include <asm/arch/pinmux.h>

#include "eth_v32.h"

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
#include <asm/arch/hwregs/strcop.h>
#include <asm/arch/cryptocop.h>
#endif

#define DEBUG(x)
#define GET_BIT(bit,val)   (((val) >> (bit)) & 0x01)

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
/*
 * Flag values for crisv32_eth_{rx,tx}_strcop_data:cop_flags.
 */
#define COP_ACTIVE			(1<<0)

#define ETH_CRYPTOCOP_CSUM_TID (1)

/* FIXME This value needs tweaking. */
#define TX_HW_CSUM_BREAK      (300)

/* FIXME This value needs tweaking. */
#define RX_HW_CSUM_BREAK      (300)

#endif



static void update_rx_stats(struct crisv32_ethernet_local *);
static void update_tx_stats(struct crisv32_ethernet_local *); 

/*
 * The name of the card. Is used for messages and in the requests for
 * io regions, irqs and dma channels.
 */
static const char *cardname = "ETRAX FS built-in ethernet controller";

/* Some chipset needs special care. */
struct transceiver_ops transceivers[] = {
	{0x1018, broadcom_check_speed, broadcom_check_duplex},	/* Broadcom. */
	{0xC039, tdk_check_speed, tdk_check_duplex},		/* TDK 2120 */
        {0x039C, tdk_check_speed, tdk_check_duplex},		/* TDK 2120C */
  	{0x04de, intel_check_speed, intel_check_duplex},     	/* Intel LXT972A*/
	{0x0000, generic_check_speed, generic_check_duplex}	/* Generic, must be last. */
};

static int __init
crisv32_ethernet_init(void)
{
	struct net_device *dev[2];
	struct crisv32_ethernet_local *np;
	int ret = 0;

	printk("ETRAX FS 10/100MBit ethernet v0.01 (c) 2003 Axis Communications AB\n");

#ifdef CONFIG_ETRAX_ETHERNET_IFACE0
{
	int iface0 = 0;
	/* Default MAC address for interface 0. The real one will be set later. */
	static struct sockaddr default_mac_iface0 = 
		{0, {0x00, 0x40, 0x8C, 0xCD, 0x00, 0x00}};

	if ((dev[iface0] = alloc_etherdev(sizeof(struct crisv32_ethernet_local))) == NULL)
		return -ENOMEM;
	
	ret |= crisv32_ethernet_device_init(dev[iface0]);

	np = (struct crisv32_ethernet_local *) dev[iface0]->priv;
	np->eth_inst = regi_eth0;
	np->dma_out_inst = regi_dma0;
	np->dma_in_inst = regi_dma1;

	register_netdev(dev[iface0]);

        /* Set up default MAC address */
	memcpy(dev[iface0]->dev_addr, default_mac_iface0.sa_data, 6);
	crisv32_eth_set_mac_address(dev[iface0], &default_mac_iface0);
	
        /* Get IRQs and DMAs */
	if (request_irq(DMA0_INTR_VECT, crisv32rxtx_eth_interrupt, 
			SA_SAMPLE_RANDOM, cardname, (void *)dev[iface0])) {
		return -EAGAIN;
	}

	if (request_irq(DMA1_INTR_VECT, crisv32rxtx_eth_interrupt, 0,
			cardname, (void *)dev[iface0])) {
		goto err0_1; 
	}

	if (crisv32_request_dma(0, cardname, DMA_VERBOSE_ON_ERROR, 12500000, dma_eth0))
		goto err0_2;

	if (crisv32_request_dma(1, cardname, DMA_VERBOSE_ON_ERROR, 12500000, dma_eth0))
		goto err0_3;
	
	if (request_irq(ETH0_INTR_VECT, crisv32nw_eth_interrupt, 0,
			cardname, (void *)dev[iface0])) {
		crisv32_free_dma(1);
	err0_3:
		crisv32_free_dma(0);
	err0_2:
		free_irq(DMA1_INTR_VECT, dev[iface0]);
	err0_1:
		free_irq(DMA0_INTR_VECT, dev[iface0]);
		return -EAGAIN;
	}
}
#endif /* CONFIG_ETRAX_ETHERNET_IFACE0 */

#ifdef CONFIG_ETRAX_ETHERNET_IFACE1
{
	int iface1 = 0;
	/* Default MAC address for interface 1. The real one will be set later. */
	static struct sockaddr default_mac_iface1 = 
		{0, {0x00, 0x40, 0x8C, 0xCD, 0x00, 0x01}};

	if (crisv32_pinmux_alloc_fixed(pinmux_eth1))
		panic("Eth pinmux\n");

	/* Increase index to device array if interface 0 is enabled as well.*/
#ifdef CONFIG_ETRAX_ETHERNET_IFACE0
	iface1++;
#endif
	if ((dev[iface1] = alloc_etherdev(sizeof(struct crisv32_ethernet_local))) == NULL)
		return -ENOMEM;
	
	ret |= crisv32_ethernet_device_init(dev[iface1]);

	np = (struct crisv32_ethernet_local *) dev[iface1]->priv;
	np->eth_inst = regi_eth1;
	np->dma_out_inst = regi_dma6;
	np->dma_in_inst = regi_dma7;
	
	register_netdev(dev[iface1]);

	/* Set up default MAC address */
	memcpy(dev[iface1]->dev_addr, default_mac_iface1.sa_data, 6);
	crisv32_eth_set_mac_address(dev[iface1], &default_mac_iface1);
	
	/* Get IRQs and DMAs */
	if (request_irq(DMA6_INTR_VECT, crisv32rxtx_eth_interrupt, 0,
			cardname, (void *)dev[iface1]))
		return -EAGAIN;
	
	if (request_irq(DMA7_INTR_VECT, crisv32rxtx_eth_interrupt, 0,
			cardname, (void *)dev[iface1]))
		goto err1_1;
        
	if (crisv32_request_dma(6, cardname, DMA_VERBOSE_ON_ERROR, 0, dma_eth1))
		goto err1_2;

	if (crisv32_request_dma(7, cardname, DMA_VERBOSE_ON_ERROR, 0, dma_eth1))
		goto err1_3;

	if (request_irq(ETH1_INTR_VECT, crisv32nw_eth_interrupt, 0,
			cardname, (void *)dev[iface1])) {
		crisv32_free_dma(7);
	err1_3:
		crisv32_free_dma(6);
	err1_2:
		free_irq(DMA7_INTR_VECT, dev[iface1]);
	err1_1:
		free_irq(DMA6_INTR_VECT, dev[iface1]);
		return -EAGAIN;
	}
}
#endif /* CONFIG_ETRAX_ETHERNET_IFACE1 */

	return ret;
}

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
static void
crisv32_ethernet_tx_strcop_data_init(struct crisv32_eth_tx_strcop_data *tx_data)
{
	int i;

	/* Setup the csum queue. */
	for (i = 0; i < MAX_NBR_TX_CSUM; i++) {
		tx_data->csum_queue[i].next = &tx_data->csum_queue[i+1];
		tx_data->csum_queue[i].skb = NULL;
		tx_data->csum_queue[i].linearized_packet = NULL;
		tx_data->csum_queue[i].use_hw = 0;
		tx_data->csum_queue[i].csum_done = 0;
		tx_data->csum_queue[i].csum_failed = 0;
	}
	tx_data->csum_queue[MAX_NBR_TX_CSUM - 1].next = &tx_data->csum_queue[0];

	tx_data->fifo_start = &tx_data->csum_queue[0];
	tx_data->fifo_end = &tx_data->csum_queue[0];
	tx_data->fifo_active = NULL;
	tx_data->fifo_len = 0;

	/* Setup the cryptocop DMA lists. */
	for (i = 0; i < MAX_SKB_FRAGS+3; i++){
		tx_data->ddesc_out[i].intr = 0;
		tx_data->ddesc_out[i].wait = 0;
		tx_data->ddesc_out[i].out_eop = 0;
		tx_data->ddesc_out[i].in_eop = 0;
		tx_data->ddesc_out[i].eol = 0;
		tx_data->ddesc_out[i].md = 0;
		tx_data->ddesc_out[i].next = (dma_descr_data*)virt_to_phys(&(tx_data->ddesc_out[i+1]));
	}
	--i;
	tx_data->ddesc_out[i].eol = 1;
	tx_data->ddesc_out[i].next = 0;

	for (i = 0; i < 3; i++){
		tx_data->ddesc_in[i].intr = 0;
		tx_data->ddesc_in[i].wait = 0;
		tx_data->ddesc_in[i].out_eop = 0;
		tx_data->ddesc_in[i].in_eop = 0;
		tx_data->ddesc_in[i].eol = 0;
		tx_data->ddesc_in[i].md = 0;
		tx_data->ddesc_in[i].next = (dma_descr_data*)virt_to_phys(&(tx_data->ddesc_in[i+1]));
	}
	--i;
	tx_data->ddesc_in[i].intr = 1;
	tx_data->ddesc_in[i].eol = 1;
	tx_data->ddesc_in[i].next = 0;

	tx_data->cop_flags = 0;
	tx_data->discard_data = NULL;
	tx_data->discard_skb = NULL;
}

static void
crisv32_ethernet_rx_strcop_data_init(struct crisv32_eth_rx_strcop_data *rx_data)
{
	int i;

	/* Setup the cryptocop DMA lists. */
	for (i = 0; i < 2; i++){
		rx_data->ddesc_out[i].intr = 0;
		rx_data->ddesc_out[i].wait = 0;
		rx_data->ddesc_out[i].out_eop = 0;
		rx_data->ddesc_out[i].in_eop = 0;
		rx_data->ddesc_out[i].eol = 0;
		rx_data->ddesc_out[i].md = 0;
		rx_data->ddesc_out[i].next = (dma_descr_data*)virt_to_phys(&(rx_data->ddesc_out[i+1]));
	}
	--i;
	rx_data->ddesc_out[i].eol = 1;
	rx_data->ddesc_out[i].next = 0;

	for (i = 0; i < 1; i++){
		rx_data->ddesc_in[i].intr = 0;
		rx_data->ddesc_in[i].wait = 0;
		rx_data->ddesc_in[i].out_eop = 0;
		rx_data->ddesc_in[i].in_eop = 0;
		rx_data->ddesc_in[i].eol = 0;
		rx_data->ddesc_in[i].md = 0;
		rx_data->ddesc_in[i].next = (dma_descr_data*)virt_to_phys(&(rx_data->ddesc_in[i+1]));
	}
	--i;
	rx_data->ddesc_in[i].intr = 1;
	rx_data->ddesc_in[i].eol = 1;
	rx_data->ddesc_in[i].next = 0;

	rx_data->cop_flags = 0;
	rx_data->discard_skb = NULL;
}

#endif


static int __init
crisv32_ethernet_device_init(struct net_device* dev)
{
	struct timer_list timer_init = TIMER_INITIALIZER(NULL, 0, 0);
	int i;
	struct crisv32_ethernet_local *np;
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	struct cryptocop_transform_init tfrm_init = {
		.alg = cryptocop_alg_csum,
		.keylen = 0,
		.csum_mode = cryptocop_csum_le,
		.tid = ETH_CRYPTOCOP_CSUM_TID,
		.next = NULL
	};
#endif

	dev->base_addr = 0;	/* Just to have something to show. */

	/* We use several IRQs and DMAs so just report 0 here. */
	dev->irq = 0;
	dev->dma = 0;

	/* 
	 * Fill in our handlers so the network layer can talk to us in the
	 * future. 
	 */
	dev->open = crisv32_eth_open;
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	dev->hard_start_xmit = crisv32_eth_send_packet_sg_csum;
#else
	dev->hard_start_xmit = crisv32_eth_send_packet;
#endif
	dev->stop = crisv32_eth_close;
	dev->get_stats = crisv32_get_stats;
	dev->set_multicast_list = crisv32_eth_set_multicast_list;
	dev->set_mac_address = crisv32_eth_set_mac_address;
	dev->do_ioctl = crisv32_eth_ioctl;
	dev->tx_timeout = crisv32_eth_tx_timeout;
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = crisv32_netpoll;
#endif

	/* Allocate space for private data. */
	dev->priv = kmalloc(sizeof (struct crisv32_ethernet_local), GFP_KERNEL);
	
	if (dev->priv == NULL)
		return -ENOMEM;
	
	memset(dev->priv, 0, sizeof (struct crisv32_ethernet_local));
	np = dev->priv;

	spin_lock_init(&np->lock);
	spin_lock_init(&np->led_lock);
	spin_lock_init(&np->transceiver_lock);

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	spin_lock_init(&np->rx_lock);
	if (0 == cryptocop_new_session(&(np->sid), &tfrm_init, GFP_KERNEL))
		dev->features |= NETIF_F_SG | NETIF_F_HW_CSUM;
	else
		panic("crisv32_ethernet_device_init: failed to get cryptocop session.\n");
#endif

	/* Initialise receive descriptors for interface. */
	for (i = 0; i < NBR_RX_DESC; i++) {
		struct sk_buff *skb = dev_alloc_skb(MAX_MEDIA_DATA_SIZE);

		np->dma_rx_descr_list[i].skb = skb;
		np->dma_rx_descr_list[i].descr.buf = (char*)virt_to_phys(skb->data);
		np->dma_rx_descr_list[i].descr.after =
		    (char*)virt_to_phys(skb->data + MAX_MEDIA_DATA_SIZE);

		np->dma_rx_descr_list[i].descr.eol = 0;
		np->dma_rx_descr_list[i].descr.in_eop = 0;
		np->dma_rx_descr_list[i].descr.next =
		    (void *) virt_to_phys(&np->dma_rx_descr_list[i + 1].descr);
	}
	
	np->dma_rx_descr_list[NBR_RX_DESC - 1].descr.eol = 1;
	np->dma_rx_descr_list[NBR_RX_DESC - 1].descr.next =
	    (dma_descr_data *) virt_to_phys(&np->dma_rx_descr_list[0].descr);

	/* Initialise initial receive pointers. */
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	if (np->sid != CRYPTOCOP_SESSION_ID_NONE) {
		np->active_rx_desc = &np->dma_rx_descr_list[0];
		np->prev_rx_desc = &np->dma_rx_descr_list[NBR_RX_DESC - 1];
		np->last_rx_desc = &np->dma_rx_descr_list[NBR_RX_DMA_DESC - 1];
		np->csum_queue_rx_desc = &np->dma_rx_descr_list[0];
	} else {
#else
	{
#endif
		np->active_rx_desc = &np->dma_rx_descr_list[0];
		np->prev_rx_desc = &np->dma_rx_descr_list[NBR_RX_DESC - 1];
	}

	/* Initialise transmit descriptors. */
	for (i = 0; i < NBR_TX_DESC; i++) {
		np->dma_tx_descr_list[i].descr.wait = 1;
		np->dma_tx_descr_list[i].descr.eol = 0;
		np->dma_tx_descr_list[i].descr.out_eop = 0;
		np->dma_tx_descr_list[i].descr.next =
			(dma_descr_data *) virt_to_phys(&np->dma_tx_descr_list[i + 1].descr);
	}
	
	np->dma_tx_descr_list[0].descr.buf = np->dma_tx_descr_list[0].descr.after = 0;
	np->dma_tx_descr_list[0].descr.eol = 1;
	np->dma_tx_descr_list[NBR_TX_DESC - 1].descr.next =
		(dma_descr_data *) virt_to_phys(&np->dma_tx_descr_list[0].descr);
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	np->nbr_tx_list_free = NBR_TX_DESC;
#endif

	/*
	 * Initialise initial transmit pointers. active_tx_desc is initiated
	 * to the second element in the list since the dma will pass the first
	 * element when crisv32_eth_open is called.
	 */
	np->active_tx_desc = &np->dma_tx_descr_list[0];
	np->prev_tx_desc = &np->dma_tx_descr_list[NBR_TX_DESC - 1];
	np->catch_tx_desc = &np->dma_tx_descr_list[0];

	/* Fill context descriptors. */
	np->ctxt_in.next = 0;        
	np->ctxt_in.saved_data =
		(dma_descr_data *) virt_to_phys(&np->dma_rx_descr_list[0].descr);
	np->ctxt_in.saved_data_buf = 
		(char*)virt_to_phys(np->dma_rx_descr_list[0].descr.buf);

	np->ctxt_out.next = 0;
	np->ctxt_out.saved_data =
		(dma_descr_data *) virt_to_phys(&np->dma_tx_descr_list[0].descr);

	/* Initialize speed indicator stuff. */
	np->current_speed = 10;
	np->current_speed_selection = 0;	/* Auto. */
	np->speed_timer = timer_init;
	np->speed_timer.expires = jiffies + NET_LINK_UP_CHECK_INTERVAL;
	np->speed_timer.data = (unsigned long) dev;
	np->speed_timer.function = crisv32_eth_check_speed;

	np->clear_led_timer = timer_init;
	np->clear_led_timer.function = crisv32_clear_network_leds;
	np->clear_led_timer.data = (unsigned long) dev;

	np->full_duplex = 0;
	np->current_duplex = autoneg;
	np->duplex_timer = timer_init;
	np->duplex_timer.expires = jiffies + NET_DUPLEX_CHECK_INTERVAL;
	np->duplex_timer.data = (unsigned long) dev;
	np->duplex_timer.function = crisv32_eth_check_duplex;

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	crisv32_ethernet_tx_strcop_data_init(&np->tx_strcop_data);
	crisv32_ethernet_rx_strcop_data_init(&np->rx_strcop_data);
#endif
	
	return 0;
}

static int
crisv32_eth_open(struct net_device *dev)
{
	unsigned long flags;
	struct sockaddr mac_addr;
	reg_dma_rw_intr_mask intr_mask_out = { .data = regk_dma_yes };
	reg_dma_rw_intr_mask intr_mask_in = { .in_eop = regk_dma_yes };
	int intr_mask_nw = { 0x1eff }; /* All except mdio and carrier loss */
	reg_dma_rw_ack_intr ack_intr = { .data = 1,.in_eop = 1 };
	reg_dma_rw_cfg dma_cfg = { .en = 1 };
	reg_eth_rw_clr_err clr_err = {.clr = regk_eth_yes};
	int eth_ack_intr = {0xffff};

	reg_config_rw_pad_ctrl pad_ctrl;
        
	reg_eth_rw_rec_ctrl rec_ctrl = {
		.ma0 = regk_eth_yes,
		.broadcast = regk_eth_yes,
		.max_size = regk_eth_size1522
	};
	
	reg_eth_rw_gen_ctrl gen_ctrl = {
		.phy = regk_eth_mii_clk
	};
	
	reg_eth_rw_tr_ctrl tr_ctrl = {
		.retry = regk_eth_yes,
		.pad = regk_eth_yes,
		.crc = regk_eth_yes
	};
	
	reg_eth_rw_ga_lo ga_lo = { 0 };
	reg_eth_rw_ga_hi ga_hi = { 0 };

	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	/* Configure receiver. */
	REG_WR(eth, np->eth_inst, rw_rec_ctrl, rec_ctrl);

	/* Give the hardware an idea of what MAC address we want. */
	memcpy(mac_addr.sa_data, dev->dev_addr, dev->addr_len);
	crisv32_eth_set_mac_address(dev, &mac_addr);

	/* 
	 * Initialize group address registers to make sure that no
	 * unwanted addresses are matched.
	 */
	REG_WR(eth, np->eth_inst, rw_ga_lo, ga_lo);
	REG_WR(eth, np->eth_inst, rw_ga_hi, ga_hi);

	/* Configure transmitter. */
	REG_WR(eth, np->eth_inst, rw_tr_ctrl, tr_ctrl);

	/* Enable ethernet controller. */
	REG_WR(eth, np->eth_inst, rw_gen_ctrl, gen_ctrl);
	gen_ctrl.en = regk_eth_yes;
	REG_WR(eth, np->eth_inst, rw_gen_ctrl, gen_ctrl);

	/* Release transceiver reset (must be done after MII clk is started) */
	pad_ctrl = REG_RD(config, regi_config, rw_pad_ctrl);
	pad_ctrl.phyrst_n = 1;
	REG_WR(config, regi_config, rw_pad_ctrl, pad_ctrl);

	REG_WR(eth, np->eth_inst, rw_clr_err, clr_err);
	REG_WR_INT(eth, np->eth_inst, rw_ack_intr, eth_ack_intr);
	REG_WR_INT(eth, np->eth_inst, rw_intr_mask, intr_mask_nw);
        
	local_irq_save(flags);

	/* Enable irq and make sure that the irqs are cleared. */
	REG_WR(dma, np->dma_out_inst, rw_intr_mask, intr_mask_out);
	REG_WR(dma, np->dma_out_inst, rw_ack_intr, ack_intr);

	REG_WR(dma, np->dma_in_inst, rw_intr_mask, intr_mask_in);
	REG_WR(dma, np->dma_in_inst, rw_ack_intr, ack_intr);

	/* Start input DMA. */
	REG_WR(dma, np->dma_in_inst, rw_cfg, dma_cfg);
	REG_WR(dma, np->dma_in_inst, rw_group_down, (int) virt_to_phys(&np->ctxt_in));
	DMA_WR_CMD(np->dma_in_inst, regk_dma_load_c);
	DMA_WR_CMD(np->dma_in_inst, regk_dma_load_d | regk_dma_burst);

	/* Prepare output DMA. */
	REG_WR(dma, np->dma_out_inst, rw_cfg, dma_cfg);

	local_irq_restore(flags);

	/* Probe for transceiver. */
	crisv32_eth_probe_transceiver(dev);

	/* Start duplex/speed timers */
	add_timer(&np->speed_timer);
	add_timer(&np->duplex_timer);

	/* 
	 * We are now ready to accept transmit requeusts from the queueing
	 * layer of the networking.
	 */
	netif_start_queue(dev);
	netif_carrier_on(dev);
        
	return 0;
}

static int
crisv32_eth_close(struct net_device *dev)
{
	reg_dma_rw_intr_mask intr_mask_out = {0};
	reg_dma_rw_intr_mask intr_mask_in = {0};
	reg_dma_rw_ack_intr ack_intr = {0};
        reg_dma_rw_cfg cfg = {0};

	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	printk(KERN_INFO "Closing %s.\n", dev->name);

	netif_stop_queue(dev);

	/* Stop the receiver and the transmitter. */
	cfg.stop = regk_dma_yes;
	REG_WR(dma, np->dma_in_inst, rw_cfg, cfg);
	REG_WR(dma, np->dma_out_inst, rw_cfg, cfg);

	/* Disable irq and make sure that the irqs are cleared. */
	intr_mask_out.data = regk_dma_no;
	REG_WR(dma, np->dma_out_inst, rw_intr_mask, intr_mask_out);
	ack_intr.data = 1;
	REG_WR(dma, np->dma_out_inst, rw_ack_intr, ack_intr);

	intr_mask_in.in_eop = regk_dma_no;
	REG_WR(dma, np->dma_in_inst, rw_intr_mask, intr_mask_in);
	ack_intr.in_eop = 1;
	REG_WR(dma, np->dma_in_inst, rw_ack_intr, ack_intr);

	/* Deallocate IRQs and DMAs. */
	if (np->eth_inst == regi_eth0) {
		free_irq(DMA0_INTR_VECT, (void *)dev);
		free_irq(DMA1_INTR_VECT, (void *)dev);
		free_irq(ETH0_INTR_VECT, (void *)dev);
		crisv32_free_dma(0);
		crisv32_free_dma(1);
	} else {
		free_irq(DMA6_INTR_VECT, (void *)dev);
		free_irq(DMA7_INTR_VECT, (void *)dev);
		free_irq(ETH1_INTR_VECT, (void *)dev);
		crisv32_free_dma(6);
		crisv32_free_dma(7);
	}

	/* Update the statistics. */
	update_rx_stats(np);
	update_tx_stats(np);

	/* Stop speed/duplex timers */
	del_timer(&np->speed_timer);
	del_timer(&np->duplex_timer);

	return 0;
}

static int
crisv32_eth_set_mac_address(struct net_device *dev, void *vpntr)
{
	int i;
	unsigned char *addr = ((struct sockaddr*)vpntr)->sa_data;
	
	reg_eth_rw_ma0_lo ma0_lo =
	  { addr[0] | (addr[1] << 8) | (addr[2] << 16) | (addr[3] << 24)};
	
	reg_eth_rw_ma0_hi ma0_hi = { addr[4] | (addr[5] << 8) };

	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	/* Remember the address. */
	memcpy(dev->dev_addr, addr, dev->addr_len);

	/* 
	 * Write the address to the hardware.
	 * Note the way the address is wrapped:
	 * ma0_l0 = a0_0 | (a0_1 << 8) | (a0_2 << 16) | (a0_3 << 24);
	 * ma0_hi = a0_4 | (a0_5 << 8);
	 */
	REG_WR(eth, np->eth_inst, rw_ma0_lo, ma0_lo);
	REG_WR(eth, np->eth_inst, rw_ma0_hi, ma0_hi);

	printk(KERN_INFO "%s: changed MAC to ", dev->name);

	for (i = 0; i < 5; i++)
		printk("%02X:", dev->dev_addr[i]);

	printk("%02X\n", dev->dev_addr[i]);

	return 0;
}


#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
/*
 * Checksums the skb in place, e.g. without doing linearization first.
 * May also copy the skb if it is shared and/or cloned.
 */
static int
crisv32_ethernet_skb_checksum(struct sk_buff *skb)
{
	int ret = skb_checksum_help(skb, 0);
	if (unlikely(ret != 0)) 
		printk(KERN_NOTICE "ETRAX FS ethernet: failed checksum.");
	return ret;
}



static void
crisv32_eth_setup_tx_cryptocop_csum(struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *)dev->priv;
	struct sk_buff *skb = np->tx_strcop_data.fifo_active->skb;
	int no_odesc = skb_shinfo(skb)->nr_frags + 1;
	size_t csum_length = 0;
	size_t csum_offset;
	int i;
	struct strcop_meta_out mo = { 0	};
	struct strcop_meta_in mi = { 0 };
	int last_odesc_len = 0;
	unsigned char last_byte = 0;

	DEBUG(printk("%s: crisv32_eth_setup_tx_cryptocop_csum\n", dev->name));

	dma_descr_data *odesc = &(np->tx_strcop_data.ddesc_out[0]);
	dma_descr_data *prev_odesc = NULL;
	dma_descr_data *idesc = &(np->tx_strcop_data.ddesc_in[0]);

	/*
	 * Setup out list.
	 */
	if (unlikely((skb->h.raw < skb->data) || (skb->h.raw > skb->tail)))
		panic("crisv32_eth_setup_tx_cryptocop_csum: h.raw outside skb data\n");

	csum_offset = skb->h.raw - skb->data;

	if ((np->tx_strcop_data.fifo_active->linearized_packet) && (csum_offset > 0)) {
		/* Copy to linearized packet.  Map the first fragment. */
		odesc->buf = (char*)virt_to_phys(skb->data);
		odesc->after = (char*)virt_to_phys(skb->h.raw);
		odesc->intr = 0;
		odesc->wait = 1;
		odesc->out_eop = 1;
		odesc->in_eop = 0;
		odesc->eol = 0;
		mo.csumsel = src_none;
		odesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_out, mo);
		odesc = (dma_descr_data*)phys_to_virt((unsigned int)odesc->next);
		--no_odesc;
		prev_odesc = odesc;
	}

	/* Start checksumming on csum_offset. */
	csum_length += skb->tail - skb->h.raw;
	last_odesc_len = skb->tail - skb->h.raw;
	odesc->buf = (char*)virt_to_phys(skb->h.raw);
	odesc->after = (char*)virt_to_phys(skb->tail);
	odesc->intr = 0;
	odesc->wait = 0;
	odesc->out_eop = 0;
	odesc->in_eop = 0;
	odesc->eol = 0;
	mo.csumsel = src_dma;
	odesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_out, mo);
	last_byte = *((char*)skb->tail - 1);

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		prev_odesc = odesc;
		csum_length += frag->size;
		last_odesc_len = frag->size;
		odesc = (dma_descr_data*)phys_to_virt((unsigned int)odesc->next);
		--no_odesc;

		odesc->intr = 0;
		odesc->wait = 0;
		odesc->out_eop = 0;
		odesc->in_eop = 0;
		odesc->eol = 0;
		odesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_out, mo);
		odesc->buf = (char*)virt_to_phys(page_address(frag->page) + frag->page_offset);
		odesc->after = odesc->buf + frag->size;
		last_byte = *(char*)(page_address(frag->page) + frag->page_offset + frag->size - 1);
	}

	if (csum_length & 1) {
		/* If the csum length is odd run csum_length - 1 bytes
		 * in HW and add the remaining byte afterwards.  This
		 * should be more efficient than using an extra descriptor with
		 * a pad byte.
		 */
		np->tx_strcop_data.csum_fixup = last_byte;
		if (1 == last_odesc_len) {
			++no_odesc;
			odesc = prev_odesc;
		} else {
			odesc->after -= 1;
		}
	} else {
		np->tx_strcop_data.csum_fixup = 0;
        }

	/* Terminate out list. */
	odesc->out_eop = 1;
	odesc->eol = 1;

	/*
	 * Setup in list.
	 */
	if (np->tx_strcop_data.fifo_active->linearized_packet){
		int csum_start_offset = skb->h.raw - skb->data;
		/* Descriptor 1.  The assembled packet part 1. */
		idesc->intr = 0;
		idesc->wait = 0;
		idesc->out_eop = 0;
		idesc->in_eop = 0;
		idesc->eol = 0;
		mi.sync = 1;
		mi.dmasel = src_dma;
		idesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_in, mi);
		idesc->buf = np->tx_strcop_data.fifo_active->linearized_packet;
		idesc->after = idesc->buf + csum_start_offset;
		idesc = (dma_descr_data*)phys_to_virt((unsigned int)idesc->next);

		/* Descriptor 2.  The assembled packet part 2. */
		idesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_in, mi);
		idesc->buf = np->tx_strcop_data.fifo_active->linearized_packet + csum_start_offset;
		idesc->after = np->tx_strcop_data.fifo_active->linearized_packet + skb->len;
		idesc = (dma_descr_data*)phys_to_virt((unsigned int)idesc->next);
		if (csum_length & 1) {
			/* Copy the last byte manually if the csum
			 * length is odd since we don't process the
			 * last byte in the strcop.
			 */
			idesc->after -= 1;
			*(unsigned char*)(crisv32_intmem_phys_to_virt(np->tx_strcop_data.fifo_active->linearized_packet) + skb->len - 1) = last_byte;
		}
	}

	/* Descriptor 3.  Checksum result. */
	idesc->wait = 0;
	idesc->out_eop = 0;
	idesc->in_eop = 0;
	mi.sync = 1;
	mi.dmasel = src_csum;
	idesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_in, mi);

	if (np->tx_strcop_data.fifo_active->linearized_packet) {
		np->tx_strcop_data.csum_pos = (unsigned short*)(crisv32_intmem_phys_to_virt(np->tx_strcop_data.fifo_active->linearized_packet) + csum_offset + skb->csum);
	} else
		np->tx_strcop_data.csum_pos = (unsigned short*)(skb->h.raw + skb->csum);

	idesc->buf = (char*)virt_to_phys(&(np->tx_strcop_data.csum_result));
	idesc->after = idesc->buf + 2;

	/* Terminate in list. */
	idesc->eol = 1;
	idesc->intr = 1;

	/*
	 * Setup the cryptocop operation.
	 */
	np->tx_strcop_data.cop.cb = crisv32_eth_tx_hw_csum_callback;
	np->tx_strcop_data.cop.cb_data = dev;
	np->tx_strcop_data.cop.sid = np->sid;

	np->tx_strcop_data.cop.use_dmalists = 1;
	np->tx_strcop_data.cop.in_interrupt = 1; /* We may be in interrupt context here(?) */
	np->tx_strcop_data.cop.fast_callback = 1;

	np->tx_strcop_data.cop.list_op.outlist = (dma_descr_data*)virt_to_phys(&(np->tx_strcop_data.ddesc_out[0]));
	np->tx_strcop_data.cop.list_op.out_data_buf = (char*)virt_to_phys(np->tx_strcop_data.ddesc_out[0].buf);

	np->tx_strcop_data.cop.list_op.inlist = (dma_descr_data*)virt_to_phys(&(np->tx_strcop_data.ddesc_in[0]));
	np->tx_strcop_data.cop.list_op.in_data_buf = (char*)virt_to_phys(np->tx_strcop_data.ddesc_in[0].buf);
	np->tx_strcop_data.cop.list_op.tdes_mode = cryptocop_3des_ede;
	np->tx_strcop_data.cop.list_op.csum_mode = cryptocop_csum_le;

	if (cryptocop_job_queue_insert_csum(&np->tx_strcop_data.cop)) {
		panic("%s: failed TX checksum enqueue", dev->name);
	}
}


static void
crisv32_eth_tx_hw_csum_callback(struct cryptocop_operation *cop, void *data)
{
	struct net_device *dev = (struct net_device *) data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	unsigned long int flags;

	DEBUG(printk("%s: crisv32_eth_tx_hw_csum_callback\n", dev->name));

	/* This callback will be called from the cryptocop interrupt
	 * handler which is a slow handler.  So we need to handle the
	 * possibility of getting an ethernet TX/RX interrupt while
	 * the callback is being executed by using
	 * spin_lock_irqsave. */
	spin_lock_irqsave(&np->lock, flags);

	if (!(np->tx_strcop_data.cop_flags & COP_ACTIVE))
		panic("crisv32 eth: got csum done callback when not active.");

	if (np->tx_strcop_data.discard_skb) {
		printk(KERN_INFO "%s: reactivate TX HW checksumming\n", dev->name);
		dev_kfree_skb_irq(np->tx_strcop_data.discard_skb);
		if (np->tx_strcop_data.discard_data)
			crisv32_intmem_free(crisv32_intmem_phys_to_virt((unsigned long)np->tx_strcop_data.discard_data));
		np->tx_strcop_data.discard_skb = NULL;
		np->tx_strcop_data.discard_data = NULL;
		np->tx_strcop_data.cop_flags &= ~COP_ACTIVE;
	} else {
		if (cop->operation_status == 0) {
			/*
			 * Post process the HW csum output by adding
			 * the stored fixup byte to the checksum.  We
			 * need to convert between ones- and twos-
			 * complement representation to do this.
			 */
			unsigned int csum = ~np->tx_strcop_data.csum_result + np->tx_strcop_data.csum_fixup;
			while (csum >> 16)
				csum = (csum & 0xffff) + (csum >> 16);
			*(np->tx_strcop_data.csum_pos) = ~csum;
		} else {
			/* HW operation failed, fall back to software. */
			if (crisv32_ethernet_skb_checksum(np->tx_strcop_data.fifo_active->skb))
				np->tx_strcop_data.fifo_active->csum_failed = 1;
		}
		np->tx_strcop_data.fifo_active->csum_done = 1;
		np->tx_strcop_data.fifo_active->use_hw = 0;
		np->tx_strcop_data.fifo_active = NULL;
		np->tx_strcop_data.cop_flags &= ~COP_ACTIVE;

		/* Push checksummed packets to eth DMA and next queued operation to cryptocop. */
		crisv32_eth_tx_csum_queue_push(dev);
	}
	spin_unlock_irqrestore(&np->lock, flags);
}


static void
crisv32_eth_setup_rx_cryptocop_csum(struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *)dev->priv;
	struct sk_buff *skb = np->csum_queue_rx_desc->skb;
	struct strcop_meta_out mo = { 0	};
	struct strcop_meta_in mi = { 0 };
	dma_descr_data *odesc = &(np->rx_strcop_data.ddesc_out[0]);
	dma_descr_data *idesc = &(np->rx_strcop_data.ddesc_in[0]);

	DEBUG(printk("%s: crisv32_eth_setup_rx_cryptocop_csum\n", dev->name));

	/*
	 * Setup out list.
	 */

	/* Map the first fragment. */
	odesc->buf = (char*)virt_to_phys(skb->data);
	odesc->after = (char*)virt_to_phys(skb->tail);
	odesc->intr = 0;
	odesc->wait = 0;
	odesc->in_eop = 0;
	odesc->out_eop = 1;
	odesc->eol = 1;
	mo.csumsel = src_dma;
	odesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_out, mo);

	if (skb->len & 1) {
		/* If the csum length is odd run csum_length - 1 bytes
		 * in HW and add the remaining byte afterwards.  This
		 * should be more efficient than using an extra descriptor with
		 * a pad byte.
		 */
		np->rx_strcop_data.csum_fixup = *(skb->tail - 1);
		odesc->after -= 1;
	} else {
		np->rx_strcop_data.csum_fixup = 0;
	}

	/*
	 * Setup in list.
	 */

	/* Checksum result. */
	idesc->wait = 0;
	idesc->out_eop = 0;
	idesc->in_eop = 0;
	mi.sync = 1;
	mi.dmasel = src_csum;
	idesc->md = REG_TYPE_CONV(unsigned short int, struct strcop_meta_in, mi);
	idesc->buf = (char*)virt_to_phys(&(np->rx_strcop_data.csum_result));
	idesc->after = idesc->buf + 2;

	/* Terminate in list. */
	idesc->eol = 1;
	idesc->intr = 1;

	/*
	 * Setup the cryptocop operation.
	 */
	np->rx_strcop_data.cop.cb = crisv32_eth_rx_hw_csum_callback;
	np->rx_strcop_data.cop.cb_data = dev;
	np->rx_strcop_data.cop.sid = np->sid;

	np->rx_strcop_data.cop.use_dmalists = 1;
	np->rx_strcop_data.cop.in_interrupt = 1; /* We may be in interrupt context here(?) */
	np->rx_strcop_data.cop.fast_callback = 1;

	np->rx_strcop_data.cop.list_op.outlist = (dma_descr_data*)virt_to_phys(&(np->rx_strcop_data.ddesc_out[0]));
	np->rx_strcop_data.cop.list_op.out_data_buf = (char*)virt_to_phys(np->rx_strcop_data.ddesc_out[0].buf);

	np->rx_strcop_data.cop.list_op.inlist = (dma_descr_data*)virt_to_phys(&(np->rx_strcop_data.ddesc_in[0]));
	np->rx_strcop_data.cop.list_op.in_data_buf = (char*)virt_to_phys(np->rx_strcop_data.ddesc_in[0].buf);
	np->rx_strcop_data.cop.list_op.tdes_mode = cryptocop_3des_ede;
	np->rx_strcop_data.cop.list_op.csum_mode = cryptocop_csum_le;

	if (cryptocop_job_queue_insert_csum(&np->rx_strcop_data.cop)) {
		panic("%s: failed RX checksum enqueue", dev->name);
	}
}


static void
crisv32_eth_rx_hw_csum_callback(struct cryptocop_operation *cop, void *data)
{
	struct net_device *dev = (struct net_device *) data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	unsigned long int flags;

	DEBUG(printk("%s: crisv32_eth_rx_hw_csum_callback\n", dev->name));

	/* This callback will be called from the cryptocop interrupt
	 * handler which is a slow handler.  So we need to handle the
	 * possibility of getting an ethernet TX/RX interrupt while
	 * the callback is being executed by using
	 * spin_lock_irqsave. */
	spin_lock_irqsave(&np->rx_lock, flags);

	if (!(np->rx_strcop_data.cop_flags & COP_ACTIVE))
		panic("crisv32 eth: got RX csum done callback when not active.");

	if (unlikely(np->rx_strcop_data.discard_skb)) {
		/* The skb was flushed while stuck in the strcop.  We drop it now. */
		dev_kfree_skb_irq(np->rx_strcop_data.discard_skb);
		np->rx_strcop_data.discard_skb = NULL;
	} else {
		if (cop->operation_status == 0) {
			/*
			 * Post process the HW csum output by adding
			 * the stored fixup byte to the checksum.  We
			 * need to convert between ones- and twos-
			 * complement representation to do this.
			 */
			unsigned int csum = ~np->rx_strcop_data.csum_result + np->rx_strcop_data.csum_fixup;
			while (csum >> 16)
				csum = (csum & 0xffff) + (csum >> 16);
			np->csum_queue_rx_desc->skb->csum = csum;
			np->csum_queue_rx_desc->skb->ip_summed = CHECKSUM_HW;			
		} else {
			/* HW operation failed, fall back to software. */
			np->csum_queue_rx_desc->skb->ip_summed = CHECKSUM_NONE;
		}

		/* Deliver packet to the upper layer.
		 */
		netif_rx(np->csum_queue_rx_desc->skb);
		np->csum_queue_rx_desc =
			(crisv32_eth_descr *) phys_to_virt((int)np->csum_queue_rx_desc->descr.next);
	}

	/* Find next received packet to checksum in cryptocop and deliver
	 * other packets to the upper layer. */
	while ((np->csum_queue_rx_desc != np->active_rx_desc) &&
	       ((np->csum_queue_rx_desc->skb == NULL) ||
	        (np->csum_queue_rx_desc->skb->ip_summed != CHECKSUM_HW))) {
		if (np->csum_queue_rx_desc->skb)
			netif_rx(np->csum_queue_rx_desc->skb);
		np->csum_queue_rx_desc =
			(crisv32_eth_descr *) phys_to_virt((int)np->csum_queue_rx_desc->descr.next);
	}

	/* Send next queued operation to cryptocop. */
	if (np->csum_queue_rx_desc != np->active_rx_desc) {
		crisv32_eth_setup_rx_cryptocop_csum(dev);
	} else {
		/* No HW csum jobs in queue. */
		np->rx_strcop_data.cop_flags &= ~COP_ACTIVE;
	}

	spin_unlock_irqrestore(&np->rx_lock, flags);
}

static void 
crisv32_eth_rx_flush_csum_queue(struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *)dev->priv;

	DEBUG(printk("%s: crisv32_eth_rx_flush_csum_queue\n", dev->name));

	/* This funcion must be called with at least the cryptocop
	 * operation done interrupt disabled.
	 */

	/* Drop Active RX strcop operation (if not already flushed). */
	if ((np->rx_strcop_data.cop_flags & COP_ACTIVE) &&
	    !(np->rx_strcop_data.discard_skb)) {
		np->rx_strcop_data.discard_skb = np->csum_queue_rx_desc->skb;
		np->csum_queue_rx_desc =
			(crisv32_eth_descr *) phys_to_virt((int)np->csum_queue_rx_desc->descr.next);
	}
	/* Deliver all queued packets to the upper layer. */
	while (np->csum_queue_rx_desc != np->active_rx_desc) {
		np->csum_queue_rx_desc->skb->ip_summed = CHECKSUM_NONE;
		netif_rx(np->csum_queue_rx_desc->skb);
		np->csum_queue_rx_desc =
			(crisv32_eth_descr *) phys_to_virt((int)np->csum_queue_rx_desc->descr.next);
	}
}
#endif

static irqreturn_t
crisv32rxtx_eth_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	reg_dma_r_masked_intr masked_in;
	reg_dma_r_masked_intr masked_out;
	reg_dma_rw_intr_mask intr_mask;
	reg_dma_rw_stat stat;
	dma_descr_data *dma_pos;
	reg_dma_rw_cmd cmd = {0};
	reg_dma_rw_ack_intr ack_intr = {0};
	struct net_device *dev = (struct net_device *) dev_id;
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;
	int sent = 0;
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	int freed_tx_descr = 0;
#endif
	masked_in = REG_RD(dma, np->dma_in_inst, r_masked_intr);
	masked_out = REG_RD(dma, np->dma_out_inst, r_masked_intr);

	/* Disable RX/TX IRQs to avoid reentrancy. */
	intr_mask = REG_RD(dma, np->dma_in_inst, rw_intr_mask);
	intr_mask.in_eop = 0;
	REG_WR(dma, np->dma_in_inst, rw_intr_mask, intr_mask);

	intr_mask = REG_RD(dma, np->dma_out_inst, rw_intr_mask);
	intr_mask.data = 0;
	REG_WR(dma, np->dma_out_inst, rw_intr_mask, intr_mask);

	if (masked_in.in_eop) {
		DEBUG(printk("EOP_IN interrupt\n"));

		/* Acknowledge input dma interrupt. */
		ack_intr.in_eop = 1;
		REG_WR(dma, np->dma_in_inst, rw_ack_intr, ack_intr);

		/* Check if one or more complete packets were indeed received. */
		while (np->active_rx_desc->descr.in_eop == 1) {
			/*
			 * Take out the buffer and give it to the OS, then
			 * allocate a new buffer to put a packet in.
			 */
			crisv32_eth_receive_packet(dev);

			/* Update number of packets received. */
			np->stats.rx_packets++;

			/* Restarts input dma. */
			cmd.cont_data = 1;
			REG_WR(dma, np->dma_in_inst, rw_cmd, cmd);

			/* Acknowledge input dma interrupt. */
			REG_WR(dma, np->dma_in_inst, rw_ack_intr, ack_intr);
		}
	}

	/* Get the current output dma position. */
	stat = REG_RD(dma, np->dma_out_inst, rw_stat);
	if (stat.list_state == regk_dma_data_at_eol || !np->sender_started)
		dma_pos = &np->active_tx_desc->descr;
	else
		dma_pos = phys_to_virt(REG_RD_INT(dma, np->dma_out_inst, rw_data));

	ack_intr.data = 1;

	/* Take care of transmited dma descriptors and report sent packet. */
	while (masked_out.data && (&np->catch_tx_desc->descr != dma_pos)) {
		sent = 1;

		/* Update sent packet statistics. */
		np->stats.tx_bytes += np->catch_tx_desc->skb->len;
		np->stats.tx_packets++;

		dev_kfree_skb_irq(np->catch_tx_desc->skb);
		np->catch_tx_desc->skb = 0;
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
		++freed_tx_descr;
		if (np->catch_tx_desc->linearized_packet) {
			/* Packet was linearized, only one descriptor used. */
			crisv32_intmem_free(crisv32_intmem_phys_to_virt((unsigned long)np->catch_tx_desc->linearized_packet));
			np->catch_tx_desc->linearized_packet = NULL;
		} else {
			/* Loop over all descriptors that make up this packet. */
			while ((!np->catch_tx_desc->descr.out_eop) &&
			       ((dma_descr_data*)virt_to_phys(&np->catch_tx_desc->descr) != dma_pos)) {
				np->catch_tx_desc =
					(crisv32_eth_descr *) phys_to_virt((int)np->catch_tx_desc->descr.next);
				++freed_tx_descr;
			}
		}
		np->catch_tx_desc =
			(crisv32_eth_descr *) phys_to_virt((int)np->catch_tx_desc->descr.next);
#else
		np->catch_tx_desc =
		    (crisv32_eth_descr *) phys_to_virt((int)np->catch_tx_desc->descr.next);
#endif
		/* ACK intr */
		REG_WR(dma, np->dma_out_inst, rw_ack_intr, ack_intr);
	}

	if (sent) {
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
		/* Wake the queue only when it is certain there is
		 * room for the worst case fragmented packet. */
		np->nbr_tx_list_free -= freed_tx_descr;
		if (np->nbr_tx_list_free > (MAX_SKB_FRAGS + 1)) {
			netif_wake_queue(dev);
		}
#else
		netif_wake_queue(dev);
#endif
	}

	/* Enable RX/TX IRQs again */
	intr_mask = REG_RD(dma, np->dma_in_inst, rw_intr_mask);
	intr_mask.in_eop = 1;
	REG_WR(dma, np->dma_in_inst, rw_intr_mask, intr_mask);
	
	intr_mask = REG_RD(dma, np->dma_out_inst, rw_intr_mask);
	intr_mask.data = 1;
	REG_WR(dma, np->dma_out_inst, rw_intr_mask, intr_mask);

	return IRQ_HANDLED;
}

/* Update receive errors. */
static void
update_rx_stats(struct crisv32_ethernet_local *np)
{
	reg_eth_rs_rec_cnt r;

	r = REG_RD(eth, np->eth_inst, rs_rec_cnt);

	np->stats.rx_fifo_errors += r.congestion;
	np->stats.rx_crc_errors += r.crc_err;
	np->stats.rx_frame_errors += r.align_err;
	np->stats.rx_length_errors += r.oversize;
}

/* Update transmit errors. */
static void
update_tx_stats(struct crisv32_ethernet_local *np)
{
	reg_eth_rs_tr_cnt r;

	r = REG_RD(eth, np->eth_inst, rs_tr_cnt);

	np->stats.collisions += r.single_col + r.mult_col;
	np->stats.tx_errors += r.deferred;
}

/* Get current statistics. */
static struct net_device_stats *
crisv32_get_stats(struct net_device *dev)
{
	unsigned long flags;
	struct crisv32_ethernet_local *np = 
		(struct crisv32_ethernet_local *) dev->priv;
	
	spin_lock_irqsave(&np->lock, flags);

	update_rx_stats(np);
	update_tx_stats(np);

	spin_unlock_irqrestore(&np->lock, flags);

	return &np->stats;
}

/* Check for network errors. This acknowledge the received interrupt. */
static irqreturn_t
crisv32nw_eth_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct crisv32_ethernet_local *np = 
		(struct crisv32_ethernet_local *) dev->priv;
	reg_eth_r_masked_intr intr_mask;
	int ack_intr = {0xffff};
	reg_eth_rw_clr_err clr_err;
	
	intr_mask = REG_RD(eth, np->eth_inst, r_masked_intr);
	clr_err.clr = 1;
	
	/*
	 * Check for underrun and/or excessive collisions. Note that the
	 * rw_clr_err register clears both underrun and excessive collision
	 * errors, so there's no need to check them separately.
	 */
	if (intr_mask.urun || intr_mask.excessive_col) {
		np->stats.tx_errors++;
	}

	update_rx_stats(np);
	update_tx_stats(np);

	REG_WR(eth, np->eth_inst, rw_clr_err, clr_err);
	REG_WR_INT(eth, np->eth_inst, rw_ack_intr, ack_intr);
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
/* We have a good packet(s), get it/them out of the buffers. */
static void
crisv32_eth_receive_packet(struct net_device *dev)
{
	int length;
	struct sk_buff *skb;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	crisv32_eth_descr *next_rx_desc = (crisv32_eth_descr *) phys_to_virt((unsigned int)np->last_rx_desc->descr.next);
	unsigned long int flags;

	DEBUG(printk("crisv32_receive_packet (HW csum)\n"));

	/* All fiddling with the RX lists must be done within
	 * spin_lock and interrupt protection.  Interrupts must be
	 * disabled since the RX lists will be modified both in eth
	 * DMA RX and strcop DMA (by way of the cryptocop operation
	 * callback) interrupt context.
	 */
	spin_lock_irqsave(&np->rx_lock, flags);

	/* Are receive buffers full? */
	if (next_rx_desc == np->csum_queue_rx_desc) {
		/* Yes.  Flush the csum queue to make room for new packet. */
		crisv32_eth_rx_flush_csum_queue(dev);
	}

	next_rx_desc->descr.eol = 1;

	/* Discard CRC (4 bytes). */
	length = (np->active_rx_desc->descr.after) - 
	         (np->active_rx_desc->descr.buf) - 4;
	/* Update received packet statistics. */
	np->stats.rx_bytes += length;

	/* Do RX_COPYBREAK processing first.  If the packet is small
	 * enough to be copied HW checksumming will not be worthwhile
	 * anyway.
	 */
	if (length < RX_COPYBREAK) {
		unsigned char *skb_data_ptr;

		skb = dev_alloc_skb(length - ETHER_HEAD_LEN);
		if (!skb) {
			/* Memory squeeze.  Drop the packet. */
			np->stats.rx_errors++;
			printk(KERN_NOTICE "%s: Memory squeeze, dropping packet.",
			       dev->name);
			next_rx_desc->skb = np->active_rx_desc->skb;
			next_rx_desc->descr.buf = np->active_rx_desc->descr.buf;
			next_rx_desc->descr.after = np->active_rx_desc->descr.buf + MAX_MEDIA_DATA_SIZE;

			np->active_rx_desc->skb = NULL;
			np->active_rx_desc->descr.buf = NULL;
			np->active_rx_desc->descr.after = NULL;
			spin_unlock_irqrestore(&np->rx_lock, flags);
			return;
		}
		skb_put(skb, length - ETHER_HEAD_LEN);        /* Allocate room for the packet body. */
		skb_data_ptr = skb_push(skb, ETHER_HEAD_LEN); /* Allocate room for the header. */

		DEBUG(printk("head = 0x%p, data = 0x%p, tail = 0x%p, end = 0x%p, length=%d\n",
			     skb->head, skb->data, skb->tail, skb->end, (int)(skb->tail - skb->data)));
		DEBUG(printk("copying packet to 0x%p.\n", skb_data_ptr));

		memcpy(skb_data_ptr, phys_to_virt((unsigned int)np->active_rx_desc->descr.buf), length);

		next_rx_desc->skb = np->active_rx_desc->skb;
		next_rx_desc->descr.buf = np->active_rx_desc->descr.buf;
		next_rx_desc->descr.after = np->active_rx_desc->descr.buf + MAX_MEDIA_DATA_SIZE;

		np->active_rx_desc->skb = skb;
	} else {
		/* No copy, allocate new skb and receive buffer for next_rx_descr. */
		next_rx_desc->skb = dev_alloc_skb(MAX_MEDIA_DATA_SIZE);
		if (!next_rx_desc->skb) {
			/* Memory squeeze.  Drop the packet. */
			np->stats.rx_errors++;
			printk(KERN_NOTICE "%s: Memory squeeze, dropping packet.",
			       dev->name);

			next_rx_desc->skb = np->active_rx_desc->skb;
			next_rx_desc->descr.buf = np->active_rx_desc->descr.buf;
			next_rx_desc->descr.after = np->active_rx_desc->descr.buf + MAX_MEDIA_DATA_SIZE;

			np->active_rx_desc->skb = NULL;
			np->active_rx_desc->descr.buf = NULL;
			np->active_rx_desc->descr.after = NULL;
		} else {
			next_rx_desc->descr.buf =
				(unsigned char *) virt_to_phys(next_rx_desc->skb->data);
			next_rx_desc->descr.after =
				next_rx_desc->descr.buf + MAX_MEDIA_DATA_SIZE;

			skb_put(np->active_rx_desc->skb, length);
	
			/* Hardware checksum triage.  We use packet length only for
			 * now.  Other things to weigh into the decision may be if 
			 * the packet was multicast or broadcast, or is non-IP.
			 *
			 * If there is a dead csum operation stuck in the strcop
			 * we csum on the CPU.
			 */
			if ((length > RX_HW_CSUM_BREAK) && !(np->rx_strcop_data.discard_skb)) {
				/* Mark this descr for HW checksumming. */
				np->active_rx_desc->skb->ip_summed = CHECKSUM_HW;
			} else {
				np->active_rx_desc->skb->ip_summed = CHECKSUM_NONE;
			}
		}
	}

	if (np->active_rx_desc->skb) {
		np->active_rx_desc->skb->dev = dev;
		np->active_rx_desc->skb->protocol = eth_type_trans(np->active_rx_desc->skb, dev);
	}

	/* Move RX eol forward. */
	next_rx_desc->descr.eol = 1;
	np->last_rx_desc->descr.eol = 0;
	np->last_rx_desc = next_rx_desc;

	np->active_rx_desc->descr.in_eop = 0;
	np->active_rx_desc =
		(crisv32_eth_descr *) phys_to_virt((int)np->active_rx_desc->descr.next);
	np->prev_rx_desc->descr.eol = 0;
	np->prev_rx_desc =
		(crisv32_eth_descr *) phys_to_virt((int)np->prev_rx_desc->descr.next);

	if (!(np->rx_strcop_data.cop_flags & COP_ACTIVE)) {
		/* No packets queued for RX checksum.  Start processing the received packet. */
		if (np->csum_queue_rx_desc->skb) {
			if (np->csum_queue_rx_desc->skb->ip_summed == CHECKSUM_HW) {
				/* Submit the packet to the co-processor. */
				np->rx_strcop_data.cop_flags |= COP_ACTIVE;
				crisv32_eth_setup_rx_cryptocop_csum(dev);
			} else {
				/* Deliver to upper layer immediately.  This empties the queue.
				 */
				netif_rx(np->csum_queue_rx_desc->skb);
				np->csum_queue_rx_desc =
					(crisv32_eth_descr *) phys_to_virt((int)np->csum_queue_rx_desc->descr.next);
			}
		}
	}

	spin_unlock_irqrestore(&np->rx_lock, flags);
}
#else
/* We have a good packet(s), get it/them out of the buffers. */
static void
crisv32_eth_receive_packet(struct net_device *dev)
{
	int length;
	struct sk_buff *skb;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	struct sk_buff *tmp;
	unsigned long flags;

	DEBUG(printk("crisv32_receive_packet\n"));

	/* Activate LED */
	spin_lock_irqsave(&np->led_lock, flags);
	if (!np->led_active && time_after(jiffies, np->led_next_time)) {
		/* light the network leds depending on the current speed. */
		crisv32_set_network_leds(NETWORK_ACTIVITY, dev);

		/* Set the earliest time we may clear the LED */
		np->led_next_time = jiffies + NET_FLASH_TIME;
		np->led_active = 1;
		mod_timer(&np->clear_led_timer, jiffies + HZ/10);
	}
	spin_unlock_irqrestore(&np->led_lock, flags);

        /* Discard CRC (4 bytes). */
	length = (np->active_rx_desc->descr.after) - 
	         (np->active_rx_desc->descr.buf) - 4;

	/* Update received packet statistics. */
	np->stats.rx_bytes += length;

	/* Utilize RX_COPYBREAK to speed up communication. */
	tmp = dev_alloc_skb(MAX_MEDIA_DATA_SIZE);
	if (!tmp) {
		np->stats.rx_errors++;
		printk(KERN_NOTICE "%s: memory squeeze, dropping packet.",
			dev->name);
		return;
	}
	skb = np->active_rx_desc->skb;
	np->active_rx_desc->skb = tmp;
	skb_put(skb, length);

	np->active_rx_desc->descr.buf =
		(unsigned char *) virt_to_phys(np->active_rx_desc->skb->data);
        np->active_rx_desc->descr.after =
		np->active_rx_desc->descr.buf + MAX_MEDIA_DATA_SIZE;

	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;

	/* Send the packet to the upper layer. */
	netif_rx(skb);
	np->active_rx_desc->descr.eol = 1;
	np->active_rx_desc->descr.in_eop = 0;
	np->active_rx_desc =
		(crisv32_eth_descr *) phys_to_virt((int)np->active_rx_desc->descr.next);
	np->prev_rx_desc->descr.eol = 0;
	np->prev_rx_desc =
		(crisv32_eth_descr *) phys_to_virt((int)np->prev_rx_desc->descr.next);
}
#endif

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
static void
crisv32_eth_tx_csum_queue_push(struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	int stop = 0;

	/* Move checksummed packets to eth TX DMA. */
	while (!stop && np->tx_strcop_data.fifo_start->csum_done) {
		if (np->tx_strcop_data.fifo_start->csum_failed) {
			if (np->tx_strcop_data.fifo_start->linearized_packet) {
				crisv32_intmem_free(crisv32_intmem_phys_to_virt((unsigned long)np->tx_strcop_data.fifo_start->linearized_packet));
				np->tx_strcop_data.fifo_start->linearized_packet = NULL;
			}
			dev_kfree_skb_irq(np->tx_strcop_data.fifo_start->skb);
			np->tx_strcop_data.fifo_start->skb = NULL;
			np->tx_strcop_data.fifo_start->csum_done = 0;
			np->tx_strcop_data.fifo_start->csum_failed = 0;
			np->tx_strcop_data.fifo_start = np->tx_strcop_data.fifo_start->next;
			--np->tx_strcop_data.fifo_len;
			/* Update statistics. */
			np->stats.tx_errors++;
			continue;
		}
		np->active_tx_desc->skb = np->tx_strcop_data.fifo_start->skb;

		if (np->tx_strcop_data.fifo_start->linearized_packet) {
			np->active_tx_desc->linearized_packet = np->tx_strcop_data.fifo_start->linearized_packet;
			crisv32_eth_hw_send_packet(np->tx_strcop_data.fifo_start->linearized_packet,
						   np->tx_strcop_data.fifo_start->skb->len,
						   1,
						   dev->priv);
			if (np->active_tx_desc == np->catch_tx_desc) {
				/* Putting this packet in ETH DMA filled the descriptor list. */
				stop = 1;
			}
			np->tx_strcop_data.fifo_start->linearized_packet = NULL;
		} else {
			if (crisv32_eth_hw_send_packet_skb(np->tx_strcop_data.fifo_start->skb,
							   dev->priv) < 1) {
				/* Failed to add packet to DMA list. */
				stop = 1;
				np->active_tx_desc->skb = NULL;
				continue;
			}
		}
		np->tx_strcop_data.fifo_start->skb = NULL;
		np->tx_strcop_data.fifo_start->csum_done = 0;
		np->tx_strcop_data.fifo_start->csum_failed = 0;
		np->tx_strcop_data.fifo_start = np->tx_strcop_data.fifo_start->next;
		--np->tx_strcop_data.fifo_len;
	}

	/* If the TX queues are full stop the device. */
	if (stop ||
	    (np->tx_strcop_data.fifo_len &&
	     (np->tx_strcop_data.fifo_start == np->tx_strcop_data.fifo_end))) {
		netif_stop_queue(dev);
	}

	/* Send HW csum packets to the co-processor. */
	if (!(np->tx_strcop_data.cop_flags & COP_ACTIVE)) {
		struct crisv32_eth_csum_queue_entry *csum_packet = np->tx_strcop_data.fifo_start;
		while ((csum_packet != np->tx_strcop_data.fifo_end) &&
		       csum_packet->csum_done &&
		       !csum_packet->use_hw) {
			csum_packet = csum_packet->next;
		}
		if (csum_packet != np->tx_strcop_data.fifo_end) {
			/* Setup and queue the checksum cryptocop operation. */
			np->tx_strcop_data.fifo_active = csum_packet;
			np->tx_strcop_data.cop_flags |= COP_ACTIVE;
			crisv32_eth_setup_tx_cryptocop_csum(dev);
		}
	}
}


/*
 * This function (i.e. hard_start_xmit when the strcop is used for
 * SG and CSUM processing) is protected from concurrent calls by a
 * spinlock (xmit_lock) in the net_device structure.
 */
static int
crisv32_eth_send_packet_sg_csum(struct sk_buff *skb, struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	unsigned long flags;

	dev->trans_start = jiffies;

	spin_lock_irqsave(&np->led_lock, flags);
	if (!np->led_active && time_after(jiffies, np->led_next_time)) {
		/* light the network leds depending on the current speed. */
		crisv32_set_network_leds(NETWORK_ACTIVITY, dev);

		/* Set the earliest time we may clear the LED */
		np->led_next_time = jiffies + NET_FLASH_TIME;
		np->led_active = 1;
		mod_timer(&np->clear_led_timer, jiffies + HZ/10);
	}
	spin_unlock_irqrestore(&np->led_lock, flags);

	/*
	 * Lock between cryptocop callback and hard_start_xmit (this
	 * function).  A side effect of using spin_lock_irq is that
	 * blocking the interrupts makes sure that the cryptocop
	 * operation callback will not be called.
	 */
	spin_lock_irq(&np->lock);

	np->tx_strcop_data.fifo_end->skb = skb;
	np->tx_strcop_data.fifo_end->csum_done = 0;
	np->tx_strcop_data.fifo_end->csum_failed = 0;

	if (skb->ip_summed == CHECKSUM_HW) {
		/* Checksum in HW if packet length is above the threshold.
		 * Process on CPU if there is a TX csum operation stuck in
		 * the strcop. */
		if ((skb->len > TX_HW_CSUM_BREAK) &&
		    (NULL == np->tx_strcop_data.discard_skb)) {
			/* Do hardware csum. */
			np->tx_strcop_data.fifo_end->use_hw = 1;
			np->tx_strcop_data.fifo_end->linearized_packet = crisv32_intmem_alloc(skb->len, 1);
			
			if (np->tx_strcop_data.fifo_end->linearized_packet)
				np->tx_strcop_data.fifo_end->linearized_packet = (unsigned char *)crisv32_intmem_virt_to_phys(np->tx_strcop_data.fifo_end->linearized_packet);
			if (!np->tx_strcop_data.fifo_end->linearized_packet && ((skb_shared(skb) || skb_cloned(skb)))) {
				/* No linearization buffer and cannot write in the sbk data,
				 * the skb must be cloned.  The clone operation gives a
				 * checksum almost free, do not do HW csumming.
				 */
				np->tx_strcop_data.fifo_end->use_hw = 0;
				if (crisv32_ethernet_skb_checksum(skb))
					np->tx_strcop_data.fifo_end->csum_failed = 1;
				np->tx_strcop_data.fifo_end->csum_done = 1;
			}
		} else {
			/* Process in slow path. */
			np->tx_strcop_data.fifo_end->use_hw = 0;
			if (crisv32_ethernet_skb_checksum(skb))
				np->tx_strcop_data.fifo_end->csum_failed = 1;
			np->tx_strcop_data.fifo_end->csum_done = 1;
		}
	} else {
		np->tx_strcop_data.fifo_end->use_hw = 0;
		np->tx_strcop_data.fifo_end->csum_done = 1;
	}
	np->tx_strcop_data.fifo_end = np->tx_strcop_data.fifo_end->next;
	++np->tx_strcop_data.fifo_len;

	/* Make room in the csum queue if it fills up.  This is done
	 * by dropping the current csum operation (moving it to the
	 * discard pointers), the call to
	 * crisv32_eth_tx_csum_queue_push() below will make sure that
	 * at least one packet is moved to eth TX DMA or the device
	 * will be stopped.*/
	if (np->tx_strcop_data.fifo_end == np->tx_strcop_data.fifo_start) {
		if ((np->tx_strcop_data.cop_flags & COP_ACTIVE) &&
		    !(np->tx_strcop_data.discard_skb)) {
			if (np->tx_strcop_data.fifo_active != np->tx_strcop_data.fifo_start)
				panic("TX HW csum FIFO: active != start\n");
			/* Drop the TX csum being done in the strcop. */
			np->tx_strcop_data.discard_skb = np->tx_strcop_data.fifo_active->skb;
			np->tx_strcop_data.discard_data = np->tx_strcop_data.fifo_active->linearized_packet;

			np->tx_strcop_data.fifo_active->skb = NULL;
			np->tx_strcop_data.fifo_active->linearized_packet = NULL;
			np->tx_strcop_data.fifo_active->csum_done = 0;
			np->tx_strcop_data.fifo_active->use_hw = 0;
			np->tx_strcop_data.fifo_active->csum_failed = 0;
			np->tx_strcop_data.fifo_active = NULL;

			np->tx_strcop_data.fifo_start = np->tx_strcop_data.fifo_start->next;
			--np->tx_strcop_data.fifo_len;
		}
	}

	/* Push checksummed packets to eth DMA. */
	crisv32_eth_tx_csum_queue_push(dev);
	spin_unlock_irq(&np->lock);

	return 0;
}
#else
/* 
 * This function (i.e. hard_start_xmit) is protected from concurent calls by a
 * spinlock (xmit_lock) in the net_device structure.
 */
static int
crisv32_eth_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	unsigned char *buf = skb->data;
	unsigned long flags;

	dev->trans_start = jiffies;

	spin_lock_irqsave(&np->led_lock, flags);
	if (!np->led_active && time_after(jiffies, np->led_next_time)) {
		/* light the network leds depending on the current speed. */
		crisv32_set_network_leds(NETWORK_ACTIVITY, dev);

		/* Set the earliest time we may clear the LED */
		np->led_next_time = jiffies + NET_FLASH_TIME;
		np->led_active = 1;
		mod_timer(&np->clear_led_timer, jiffies + HZ/10);
	}
	spin_unlock_irqrestore(&np->led_lock, flags);

	/*
	 * Need to disable irq to avoid updating pointer in interrupt while sending
	 * packets.
	 */
	spin_lock_irqsave(&np->lock, flags);

	np->active_tx_desc->skb = skb;
	
	crisv32_eth_hw_send_packet(buf, skb->len, np);

	/* Stop queue if full. */
	if (np->active_tx_desc == np->catch_tx_desc)
		netif_stop_queue(dev);

	spin_unlock_irqrestore(&np->lock, flags);

	return 0;
}
#endif

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
/*
 * Returns the number of descriptors used or -1 if the was no room in
 * the TX list.
 */
static int
crisv32_eth_hw_send_packet_skb(struct sk_buff *skb, void *priv)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) priv;
	int i;
	crisv32_eth_descr *prev_active = np->active_tx_desc;
	int used_descrs = 1;

	np->active_tx_desc->descr.buf = (unsigned char *) virt_to_phys(skb->data);
	np->active_tx_desc->descr.after = (unsigned char *) virt_to_phys(skb->tail);
	np->active_tx_desc->descr.intr = 0;
	np->active_tx_desc->descr.eol = 0;
	np->active_tx_desc->descr.out_eop = 0;

	if (skb_is_nonlinear(skb)) {
		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
			np->active_tx_desc =
				(crisv32_eth_descr *) phys_to_virt((int)np->active_tx_desc->descr.next);
			if (np->active_tx_desc == np->catch_tx_desc) {
				/* Queue filled up.  Restore TX list and bail. */
				np->active_tx_desc = prev_active;
				return -1;
			}
			np->active_tx_desc->descr.buf =
				(unsigned char*)virt_to_phys(page_address(frag->page) + frag->page_offset);
			np->active_tx_desc->descr.after = np->active_tx_desc->descr.buf + frag->size;
			np->active_tx_desc->descr.intr = 0;
			np->active_tx_desc->descr.eol = 0;
			np->active_tx_desc->descr.out_eop = 0;
			++used_descrs;
		}
	}
	np->active_tx_desc->descr.intr = 1;
	np->active_tx_desc->descr.out_eop = 1;

	/* Move eol. */
	np->active_tx_desc->descr.eol = 1;
	np->prev_tx_desc->descr.eol = 0;

	np->nbr_tx_list_free += used_descrs;

	/* Update pointers. */
	np->prev_tx_desc = np->active_tx_desc;
	np->active_tx_desc =
		(crisv32_eth_descr *) phys_to_virt((int)np->active_tx_desc->descr.next);

	/* Start DMA. */
	crisv32_start_dma_out(np);
        
	return used_descrs;
}
#endif

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
static void
crisv32_eth_hw_send_packet(unsigned char *buf, int length, int phys_addr, void *priv)
#else
static void
crisv32_eth_hw_send_packet(unsigned char *buf, int length, void *priv)
#endif
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) priv;

	/* Configure the tx dma descriptor. */
#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	if (!phys_addr) {
		np->active_tx_desc->descr.buf = (unsigned char *) virt_to_phys(buf);
	} else {
		np->active_tx_desc->descr.buf = buf;
	}
#else
	np->active_tx_desc->descr.buf = (unsigned char *) virt_to_phys(buf);
#endif
	np->active_tx_desc->descr.after = np->active_tx_desc->descr.buf + length;
	np->active_tx_desc->descr.intr = 1;        
	np->active_tx_desc->descr.out_eop = 1;

	/* Move eol. */
	np->active_tx_desc->descr.eol = 1;
	np->prev_tx_desc->descr.eol = 0;

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	np->nbr_tx_list_free++;
#endif

	/* Update pointers. */
	np->prev_tx_desc = np->active_tx_desc;
	np->active_tx_desc =
		(crisv32_eth_descr *) phys_to_virt((int)np->active_tx_desc->descr.next);

	/* Start DMA. */
	crisv32_start_dma_out(np);
}

static void
crisv32_start_dma_out(struct crisv32_ethernet_local* np)
{
	if (!np->sender_started) {
		/* Start DMA for the first time. */
		np->ctxt_out.saved_data_buf = np->prev_tx_desc->descr.buf;
		REG_WR(dma, np->dma_out_inst, rw_group_down, (int) virt_to_phys(&np->ctxt_out));
		DMA_WR_CMD(np->dma_out_inst, regk_dma_load_c);
		DMA_WR_CMD(np->dma_out_inst, regk_dma_load_d | regk_dma_burst);
		np->sender_started = 1;
	} else {
		/* Restarts output dma. */
		reg_dma_rw_cmd cmd = {.cont_data = 1};
		REG_WR(dma, np->dma_out_inst, rw_cmd, cmd);
	}
}

/* 
 * Called by upper layers if they decide it took too long to complete sending
 * a packet - we need to reset and stuff.
 */
static void
crisv32_eth_tx_timeout(struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	reg_dma_rw_cfg cfg = {0};
        reg_dma_rw_stat stat = {0};

	printk(KERN_WARNING "%s: transmit timed out\n", dev->name);

	/* Update error stats. */
	np->stats.tx_errors++; 

	/* Reset the TX DMA in case it has hung on something. */
	cfg.en = 0;
	REG_WR(dma, np->dma_out_inst, rw_cfg, cfg);
	
	do {
		stat = REG_RD(dma, np->dma_out_inst, rw_stat);
	} while (stat.mode != regk_dma_rst);

	/* Reset the tranceiver. */
	crisv32_eth_reset_tranceiver(dev);

	/* Get rid of the packets that never got an interrupt. */
	while (np->catch_tx_desc != np->active_tx_desc) {
		dev_kfree_skb(np->catch_tx_desc->skb);

		np->catch_tx_desc->skb = 0;
		np->catch_tx_desc =
		  (crisv32_eth_descr *) phys_to_virt((int)np->catch_tx_desc->descr.next);
	} 

#ifdef CONFIG_ETRAX_ETHERNET_HW_CSUM
	{
		if ((np->tx_strcop_data.cop_flags & COP_ACTIVE) &&
		    !(np->tx_strcop_data.discard_skb)) {
			unsigned long int flags;

			spin_lock_irqsave(&np->lock, flags);

			np->tx_strcop_data.discard_skb =
                          np->tx_strcop_data.fifo_start->skb;
			np->tx_strcop_data.discard_data =
                          np->tx_strcop_data.fifo_start->linearized_packet;
			np->tx_strcop_data.fifo_start->skb = NULL;
			np->tx_strcop_data.fifo_start->csum_done = 0;
			np->tx_strcop_data.fifo_start->csum_failed = 0;
			np->tx_strcop_data.fifo_start->use_hw = 0;
			--np->tx_strcop_data.fifo_len;
			np->tx_strcop_data.fifo_start = np->tx_strcop_data.fifo_start->next;
			spin_unlock_irqrestore(&np->lock, flags);
		}		

		while ((np->tx_strcop_data.fifo_start != np->tx_strcop_data.fifo_end) && (np->tx_strcop_data.fifo_len != 0)) {
			if (np->tx_strcop_data.fifo_start->linearized_packet) {
				crisv32_intmem_free(crisv32_intmem_phys_to_virt((unsigned long)np->tx_strcop_data.fifo_start->linearized_packet));
				np->tx_strcop_data.fifo_start->linearized_packet = NULL;
			}
			dev_kfree_skb(np->tx_strcop_data.fifo_start->skb);
			np->tx_strcop_data.fifo_start->skb = NULL;
			np->tx_strcop_data.fifo_start->csum_done = 0;
			np->tx_strcop_data.fifo_start->csum_failed = 0;
			np->tx_strcop_data.fifo_start->use_hw = 0;

			--np->tx_strcop_data.fifo_len;
			np->tx_strcop_data.fifo_start = np->tx_strcop_data.fifo_start->next;
		}
	}
#endif

	/* Start output DMA. */
	REG_WR(dma, np->dma_out_inst, rw_group_down, (int) virt_to_phys(&np->ctxt_out));
	DMA_WR_CMD(np->dma_out_inst, regk_dma_load_c);
	DMA_WR_CMD(np->dma_out_inst, regk_dma_load_d | regk_dma_burst);

	/* Tell the upper layers we're ok again. */
	netif_wake_queue(dev);
}

/*
 * Set or clear the multicast filter for this adaptor.
 * num_addrs == -1	Promiscuous mode, receive all packets
 * num_addrs == 0	Normal mode, clear multicast list
 * num_addrs > 0	Multicast mode, receive normal and MC packets,
 *			and do best-effort filtering.
 */
static void
crisv32_eth_set_multicast_list(struct net_device *dev)
{
	int num_addr = dev->mc_count;
	unsigned long int lo_bits;
	unsigned long int hi_bits;
	reg_eth_rw_rec_ctrl rec_ctrl = {0};
	reg_eth_rw_ga_lo ga_lo = {0};
	reg_eth_rw_ga_hi ga_hi = {0};
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	if (dev->flags & IFF_PROMISC) {
		/* Promiscuous mode. */
		lo_bits = 0xfffffffful;
		hi_bits = 0xfffffffful;

		/* Enable individual receive. */
		rec_ctrl =
			(reg_eth_rw_rec_ctrl) REG_RD(eth, np->eth_inst, rw_rec_ctrl);
		rec_ctrl.individual = regk_eth_yes;
		REG_WR(eth, np->eth_inst, rw_rec_ctrl, rec_ctrl);
	} else if (dev->flags & IFF_ALLMULTI) {
		/* Enable all multicasts. */
		lo_bits = 0xfffffffful;
		hi_bits = 0xfffffffful;

		/* Disable individual receive */
		rec_ctrl =
		  (reg_eth_rw_rec_ctrl) REG_RD(eth, np->eth_inst, rw_rec_ctrl);
		rec_ctrl.individual = regk_eth_no;
		REG_WR(eth, np->eth_inst, rw_rec_ctrl, rec_ctrl);
	} else if (num_addr == 0) {
		/* Normal, clear the mc list. */
		lo_bits = 0x00000000ul;
		hi_bits = 0x00000000ul;

		/* Disable individual receive */
		rec_ctrl =
		  (reg_eth_rw_rec_ctrl) REG_RD(eth, np->eth_inst, rw_rec_ctrl);
		rec_ctrl.individual = regk_eth_no;
		REG_WR(eth, np->eth_inst, rw_rec_ctrl, rec_ctrl);
	} else {
		/* MC mode, receive normal and MC packets. */
		char hash_ix;
		struct dev_mc_list *dmi = dev->mc_list;
		int i;
		char *baddr;
		lo_bits = 0x00000000ul;
		hi_bits = 0x00000000ul;
		
		for (i = 0; i < num_addr; i++) {
			/* Calculate the hash index for the GA registers. */
			hash_ix = 0;
			baddr = dmi->dmi_addr;
			hash_ix ^= (*baddr) & 0x3f;
			hash_ix ^= ((*baddr) >> 6) & 0x03;
			++baddr;
			hash_ix ^= ((*baddr) << 2) & 0x03c;
			hash_ix ^= ((*baddr) >> 4) & 0xf;
			++baddr;
			hash_ix ^= ((*baddr) << 4) & 0x30;
			hash_ix ^= ((*baddr) >> 2) & 0x3f;
			++baddr;
			hash_ix ^= (*baddr) & 0x3f;
			hash_ix ^= ((*baddr) >> 6) & 0x03;
			++baddr;
			hash_ix ^= ((*baddr) << 2) & 0x03c;
			hash_ix ^= ((*baddr) >> 4) & 0xf;
			++baddr;
			hash_ix ^= ((*baddr) << 4) & 0x30;
			hash_ix ^= ((*baddr) >> 2) & 0x3f;

			hash_ix &= 0x3f;

			if (hash_ix > 32)
				hi_bits |= (1 << (hash_ix - 32));
			else
				lo_bits |= (1 << hash_ix);

			dmi = dmi->next;
		}
		
		/* Disable individual receive. */
		rec_ctrl =
		  (reg_eth_rw_rec_ctrl) REG_RD(eth, np->eth_inst, rw_rec_ctrl);
		rec_ctrl.individual = regk_eth_no;
		REG_WR(eth, np->eth_inst, rw_rec_ctrl, rec_ctrl);
	}

	ga_lo.table = (unsigned int) lo_bits;
	ga_hi.table = (unsigned int) hi_bits;

	REG_WR(eth, np->eth_inst, rw_ga_lo, ga_lo);
	REG_WR(eth, np->eth_inst, rw_ga_hi, ga_hi);
}

static int
crisv32_eth_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCETHTOOL:
		return crisv32_ethtool_ioctl(dev, ifr);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int
crisv32_ethtool_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct ethtool_cmd ecmd;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

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
			ecmd.phy_address = np->mdio_phy_addr;
			ecmd.speed = np->current_speed;
			ecmd.duplex =
			    np->full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
			ecmd.advertising = ADVERTISED_TP;
			
			if (np->current_duplex == autoneg
			    && np->current_speed_selection == 0) {
				ecmd.advertising |= ADVERTISED_Autoneg;
			} else {
				ecmd.advertising |=
				    ADVERTISED_10baseT_Half |
				    ADVERTISED_10baseT_Full |
				    ADVERTISED_100baseT_Half |
				    ADVERTISED_100baseT_Full;
				
				if (np->current_speed_selection == 10) {
					ecmd.advertising &=
					    ~(ADVERTISED_100baseT_Half |
					      ADVERTISED_100baseT_Full);
				} else if (np->current_speed_selection == 100) {
					ecmd.advertising &=
					    ~(ADVERTISED_10baseT_Half |
					      ADVERTISED_10baseT_Full);
				}
				
				if (np->current_duplex == half) {
					ecmd.advertising &=
					    ~(ADVERTISED_10baseT_Full |
					      ADVERTISED_100baseT_Full);
				} else if (np->current_duplex == full) {
					ecmd.advertising &=
					    ~(ADVERTISED_10baseT_Half |
					      ADVERTISED_100baseT_Half);
				}
			}
			
			ecmd.autoneg = AUTONEG_ENABLE;
			
			if (copy_to_user(ifr->ifr_data, &ecmd, sizeof (ecmd)))
				return -EFAULT;
		}
		break;
	case ETHTOOL_SSET:
		{
			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			
			if (ecmd.autoneg == AUTONEG_ENABLE) {
				crisv32_eth_set_duplex(dev, autoneg);
				crisv32_eth_set_speed(dev, 0);
			} else {
				crisv32_eth_set_duplex(dev,
						       ecmd.duplex ==
						       DUPLEX_HALF ? half :
						       full);
				crisv32_eth_set_speed(dev,
						      ecmd.speed ==
						      SPEED_10 ? 10 : 100);
			}
		}
		break;
	case ETHTOOL_GDRVINFO:
		{
			struct ethtool_drvinfo info;

			memset((void *) &info, 0, sizeof(info));
			strncpy(info.driver, "ETRAX FS", sizeof(info.driver) - 1);
			strncpy(info.version, "$Revision: 1.63 $", sizeof(info.version) - 1);
			strncpy(info.fw_version, "N/A", sizeof(info.fw_version) - 1);
			strncpy(info.bus_info, "N/A", sizeof(info.bus_info) - 1);
			info.regdump_len = 0;
			info.eedump_len = 0;
			info.testinfo_len = 0;
			
			if (copy_to_user(ifr->ifr_data, &info, sizeof(info)))
				return -EFAULT;
		}
		break;
	case ETHTOOL_NWAY_RST:
		if (np->current_duplex == autoneg
		    && np->current_speed_selection == 0)
			crisv32_eth_negotiate(dev);
		break;
	case ETHTOOL_GLINK:
		{
			struct ethtool_value edata = { ETHTOOL_GLINK };
			unsigned long data;
			data = crisv32_eth_get_mdio_reg(dev, MII_BMSR);
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

static void
crisv32_eth_negotiate(struct net_device *dev)
{
	unsigned short data =
	    crisv32_eth_get_mdio_reg(dev, MII_ADVERTISE);
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;

	switch (np->current_speed_selection) {
		case 10 :
			if (np->current_duplex == full)
				data |= ADVERTISE_10FULL;
			else if (np->current_duplex == half)
				data |= ADVERTISE_10HALF;
			else
				data |= ADVERTISE_10HALF | ADVERTISE_10FULL;
			break;

		case 100 :
			if (np->current_duplex == full)
				data |= ADVERTISE_100FULL;
			else if (np->current_duplex == half)
				data |= ADVERTISE_100HALF;
			else
				data |= ADVERTISE_100HALF | ADVERTISE_100FULL;
			break;

		case 0 : /* Auto */
			if (np->current_duplex == full)
				data |= ADVERTISE_100FULL | ADVERTISE_10FULL;
			else if (np->current_duplex == half)
				data |= ADVERTISE_100HALF | ADVERTISE_10HALF;
			else
				data |= ADVERTISE_10HALF | ADVERTISE_10FULL |
				  ADVERTISE_100HALF | ADVERTISE_100FULL;
			break;

		default : /* assume autoneg speed and duplex */
			data |= ADVERTISE_10HALF | ADVERTISE_10FULL |
				  ADVERTISE_100HALF | ADVERTISE_100FULL;
	}

	crisv32_eth_set_mdio_reg(dev, MII_ADVERTISE, data);

	/* Renegotiate with link partner */
	data = crisv32_eth_get_mdio_reg(dev, MII_BMCR);
	data |= BMCR_ANENABLE | BMCR_ANRESTART;

	crisv32_eth_set_mdio_reg(dev, MII_BMCR, data);
}
static void
crisv32_eth_check_speed(unsigned long idev)
{
  	static int led_initiated = 0;
	struct net_device *dev = (struct net_device *) idev;
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;

	unsigned long data;
	int old_speed = np->current_speed;
	unsigned long flags;

	spin_lock(&np->transceiver_lock);

	data = crisv32_eth_get_mdio_reg(dev, MII_BMSR);
	
	if (!(data & BMSR_LSTATUS))
		np->current_speed = 0;
	else
		np->transceiver->check_speed(dev);

	spin_lock_irqsave(&np->led_lock, flags);
        if ((old_speed != np->current_speed) || !led_initiated) {
		led_initiated = 1;
		crisv32_set_network_leds(NO_NETWORK_ACTIVITY, dev);
		if (np->current_speed)
			netif_carrier_on(dev);
		else
			netif_carrier_off(dev);
	}
	spin_unlock_irqrestore(&np->led_lock, flags);

	/* Reinitialize the timer. */
	np->speed_timer.expires = jiffies + NET_LINK_UP_CHECK_INTERVAL;
	add_timer(&np->speed_timer);

	spin_unlock(&np->transceiver_lock);
}

static void
crisv32_eth_set_speed(struct net_device *dev, unsigned long speed)
{
	struct crisv32_ethernet_local *np = 
	  (struct crisv32_ethernet_local *) dev->priv;

	spin_lock(&np->transceiver_lock);
	np->current_speed_selection = speed;
	crisv32_eth_negotiate(dev);
	spin_unlock(&np->transceiver_lock);
}

static void
crisv32_eth_check_duplex(unsigned long idev)
{
	struct net_device *dev = (struct net_device *) idev;
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;
	reg_eth_rw_rec_ctrl rec_ctrl;
	int old_duplex = np->full_duplex;

	np->transceiver->check_duplex(dev);
	
	if (old_duplex != np->full_duplex) {
		/* Duplex changed. */
		rec_ctrl =
			(reg_eth_rw_rec_ctrl) REG_RD(eth, np->eth_inst, rw_rec_ctrl);
		rec_ctrl.duplex = np->full_duplex;
		REG_WR(eth, np->eth_inst, rw_rec_ctrl, rec_ctrl);
	}

	/* Reinitialize the timer. */
	np->duplex_timer.expires = jiffies + NET_DUPLEX_CHECK_INTERVAL;
	add_timer(&np->duplex_timer);
}

static void
crisv32_eth_set_duplex(struct net_device *dev, enum duplex new_duplex)
{
	((struct crisv32_ethernet_local *) dev->priv)->current_duplex =
	    new_duplex;
	crisv32_eth_negotiate(dev);
}

static int
crisv32_eth_probe_transceiver(struct net_device *dev)
{
	unsigned int phyid_high;
	unsigned int phyid_low;
	unsigned int oui;
	struct transceiver_ops *ops = NULL;
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;

	/* Probe MDIO physical address. */
	for (np->mdio_phy_addr = 0; np->mdio_phy_addr <= 31; np->mdio_phy_addr++) {
		if (crisv32_eth_get_mdio_reg(dev, MII_BMSR) != 0xffff)
			break;
	}
	
	if (np->mdio_phy_addr == 32)
		return -ENODEV;

	/* Get manufacturer. */
	phyid_high = crisv32_eth_get_mdio_reg(dev, MII_PHYSID1);
	phyid_low = crisv32_eth_get_mdio_reg(dev, MII_PHYSID2);
	
	oui = (phyid_high << 6) | (phyid_low >> 10);

	for (ops = &transceivers[0]; ops->oui; ops++) {
		if (ops->oui == oui)
			break;
	}

	np->transceiver = ops;
	return 0;
}

static void
generic_check_speed(struct net_device *dev)
{
	unsigned long data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	data = crisv32_eth_get_mdio_reg(dev, MII_ADVERTISE);
	if ((data & ADVERTISE_100FULL) ||
	    (data & ADVERTISE_100HALF))
		np->current_speed = 100;
	else
		np->current_speed = 10;	
}

static void
generic_check_duplex(struct net_device *dev)
{
	unsigned long data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	
	data = crisv32_eth_get_mdio_reg(dev, MII_ADVERTISE);
	if ((data & ADVERTISE_10FULL) ||
	    (data & ADVERTISE_100FULL))
		np->full_duplex = 1;
	else
		np->full_duplex = 0;
}

static void
broadcom_check_speed(struct net_device *dev)
{
	unsigned long data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	data = crisv32_eth_get_mdio_reg(dev, MDIO_AUX_CTRL_STATUS_REG);
	np->current_speed = (data & MDIO_BC_SPEED ? 100 : 10);
}

static void
broadcom_check_duplex(struct net_device *dev)
{
	unsigned long data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	data = crisv32_eth_get_mdio_reg(dev, MDIO_AUX_CTRL_STATUS_REG);        
	np->full_duplex = (data & MDIO_BC_FULL_DUPLEX_IND) ? 1 : 0;
}

static void
tdk_check_speed(struct net_device *dev)
{
	unsigned long data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	
	data = crisv32_eth_get_mdio_reg(dev, MDIO_TDK_DIAGNOSTIC_REG);
	np->current_speed = (data & MDIO_TDK_DIAGNOSTIC_RATE ? 100 : 10);
}

static void
tdk_check_duplex(struct net_device *dev)
{
	unsigned long data;
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	data = crisv32_eth_get_mdio_reg(dev, MDIO_TDK_DIAGNOSTIC_REG);
	np->full_duplex = (data & MDIO_TDK_DIAGNOSTIC_DPLX) ? 1 : 0;

}

static void
intel_check_speed(struct net_device *dev)
{
	unsigned long data;
        struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	data = crisv32_eth_get_mdio_reg(dev, MDIO_INT_STATUS_REG_2);
	np->current_speed = (data & MDIO_INT_SPEED ? 100 : 10);
}

static void
intel_check_duplex(struct net_device *dev)
{
	unsigned long data;
        struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	
	data = crisv32_eth_get_mdio_reg(dev, MDIO_INT_STATUS_REG_2);        
	np->full_duplex = (data & MDIO_INT_FULL_DUPLEX_IND) ? 1 : 0;
}
 
static void
crisv32_eth_reset_tranceiver(struct net_device *dev)
{
	int i;
	unsigned short cmd;
	unsigned short data;
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;

	data = crisv32_eth_get_mdio_reg(dev, MII_BMCR);

	cmd =
	    (MDIO_START << 14) | (MDIO_WRITE << 12) | (np->mdio_phy_addr << 7) |
	    (MII_BMCR << 2);

	crisv32_eth_send_mdio_cmd(dev, cmd, 1);

	data |= 0x8000;

	/* Magic value is number of bits. */
	for (i = 15; i >= 0; i--)
		crisv32_eth_send_mdio_bit(dev, GET_BIT(i, data));
}

static unsigned short
crisv32_eth_get_mdio_reg(struct net_device *dev, unsigned char reg_num)
{
	int i;
	unsigned short cmd;	/* Data to be sent on MDIO port. */
	unsigned short data;	/* Data read from MDIO. */
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;

	/* Start of frame, OP Code, Physical Address, Register Address. */
	cmd = (MDIO_START << 14) | (MDIO_READ << 12) | (np->mdio_phy_addr << 7) |
	    (reg_num << 2);

	crisv32_eth_send_mdio_cmd(dev, cmd, 0);

	data = 0;

	/* Receive data. Magic value is number of bits. */
	for (i = 15; i >= 0; i--)
		data |= (crisv32_eth_receive_mdio_bit(dev) << i);

	return data;
}

static void
crisv32_eth_set_mdio_reg(struct net_device *dev, unsigned char reg, int value)
{
	int bitCounter;
	unsigned short cmd;
	struct crisv32_ethernet_local *np =
	    (struct crisv32_ethernet_local *) dev->priv;

	cmd = (MDIO_START << 14) | (MDIO_WRITE << 12) | (np->mdio_phy_addr << 7) |
	      (reg << 2);

	crisv32_eth_send_mdio_cmd(dev, cmd, 1);

	/* Data... */
	for (bitCounter=15; bitCounter>=0 ; bitCounter--) {
		crisv32_eth_send_mdio_bit(dev, GET_BIT(bitCounter, value));
	}

}

static void
crisv32_eth_send_mdio_cmd(struct net_device *dev, unsigned short cmd,
			  int write_cmd)
{
	int i;
	unsigned char data = 0x2;

	/* Preamble. Magic value is number of bits. */
	for (i = 31; i >= 0; i--)
		crisv32_eth_send_mdio_bit(dev, GET_BIT(i, MDIO_PREAMBLE));

	for (i = 15; i >= 2; i--)
		crisv32_eth_send_mdio_bit(dev, GET_BIT(i, cmd));

	/* Turnaround. */
	for (i = 1; i >= 0; i--)
		if (write_cmd)
			crisv32_eth_send_mdio_bit(dev, GET_BIT(i, data));
		else
			crisv32_eth_receive_mdio_bit(dev);
}

static void
crisv32_eth_send_mdio_bit(struct net_device *dev, unsigned char bit)
{
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;
	
	reg_eth_rw_mgm_ctrl mgm_ctrl = {
		.mdoe = regk_eth_yes,
		.mdio = bit & 1
	};

	REG_WR(eth, np->eth_inst, rw_mgm_ctrl, mgm_ctrl);

	udelay(1);

	mgm_ctrl.mdc = 1;
	REG_WR(eth, np->eth_inst, rw_mgm_ctrl, mgm_ctrl);

	udelay(1);
}

static unsigned char
crisv32_eth_receive_mdio_bit(struct net_device *dev)
{
	reg_eth_r_stat stat;
	reg_eth_rw_mgm_ctrl mgm_ctrl = {0};
	struct crisv32_ethernet_local *np =
		(struct crisv32_ethernet_local *) dev->priv;

	REG_WR(eth, np->eth_inst, rw_mgm_ctrl, mgm_ctrl);
	stat = REG_RD(eth, np->eth_inst, r_stat);

	udelay(1);

	mgm_ctrl.mdc = 1;
	REG_WR(eth, np->eth_inst, rw_mgm_ctrl, mgm_ctrl);

	udelay(1);
	return stat.mdio;
}

static void
crisv32_clear_network_leds(unsigned long priv)
{
	struct net_device *dev = (struct net_device*)priv;
	struct crisv32_ethernet_local *np =
	  (struct crisv32_ethernet_local *) dev->priv;
	unsigned long flags;

	spin_lock_irqsave(&np->led_lock, flags);
  	if (np->led_active && time_after(jiffies, np->led_next_time)) {
		crisv32_set_network_leds(NO_NETWORK_ACTIVITY, dev);

		/* Set the earliest time we may set the LED */
		np->led_next_time = jiffies + NET_FLASH_PAUSE;
		np->led_active = 0;
	}
	spin_unlock_irqrestore(&np->led_lock, flags);
}

static void
crisv32_set_network_leds(int active, struct net_device *dev)
{
	struct crisv32_ethernet_local *np =
	  (struct crisv32_ethernet_local *) dev->priv;

#if defined(CONFIG_ETRAX_NETWORK_LED_ON_WHEN_LINK)
	int light_leds = (active == NO_NETWORK_ACTIVITY);
#elif defined(CONFIG_ETRAX_NETWORK_LED_ON_WHEN_ACTIVITY)
	int light_leds = (active == NETWORK_ACTIVITY);
#else
#error "Define either CONFIG_ETRAX_NETWORK_LED_ON_WHEN_LINK or CONFIG_ETRAX_NETWORK_LED_ON_WHEN_ACTIVITY"
#endif

	if (!np->current_speed) {
		/* Make LED red, link is down */
#if defined(CONFIG_ETRAX_NETWORK_RED_ON_NO_CONNECTION)
		LED_NETWORK_SET(LED_RED);
#else		
		LED_NETWORK_SET(LED_OFF);
#endif		
	}
	else if (light_leds) {
		if (np->current_speed == 10) {
			LED_NETWORK_SET(LED_ORANGE);
		} else {
			LED_NETWORK_SET(LED_GREEN);
		}
	}
	else {
		LED_NETWORK_SET(LED_OFF);
	}
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void
crisv32_netpoll(struct net_device* netdev)
{
	crisv32rxtx_eth_interrupt(DMA0_INTR_VECT, netdev, NULL);
}
#endif

static int
crisv32_init_module(void)
{
	return crisv32_ethernet_init();
}

module_init(crisv32_init_module);
