// Copyright (c) 2003-2004 Axis Communications AB
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.
//
// Argus driver for GPIO

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <asm/arch/gpiodriver.h>
#include <asm/arch/gpio.h> /* To get dependency right */

#define GPIO_MAJOR 120  /* experimental MAJOR number */
#define PORT_BITS   32  /* Number of bits serverd by each minor deive */
#ifdef CONFIG_ARGUS_2
#define NUM_PORTS    2	/* 2*32 bits */
#else
#define NUM_PORTS    3	/* 2*32 bits + 28 bits */
#endif
#define LED_MINOR GPIO_MINOR_LEDS

#define D(x)

struct gpio_private {
       struct gpio_private* next;
       unsigned long highalarm, lowalarm;
       unsigned long changeable_bits;
       wait_queue_head_t alarm_wq;
       int base; /* First GPIO bit served by this device */
       int minor;
};

/* linked list of alarms to check for */
static struct gpio_private *alarmlist = 0;

/* Helper functions */
static inline unsigned long
gpio_read_bits(struct gpio_private *priv)
{
	int bit, val = 0;
	for (bit = 0; bit < PORT_BITS; bit++) 
		val |= gpio_get_level(priv->base + bit) << bit;	    
	return val;
}

static inline void
gpio_write_bits(struct gpio_private *priv, unsigned long data, unsigned long mask)
{
	int bit;
	for (bit = 0; bit < PORT_BITS; bit++) {
		if (mask & (1 << bit))
			gpio_set_mode(priv->base + bit, GPIO_MODE_OUT, (data & (1<<bit))>>bit);   		
	}
}

static inline void
gpio_set_dir(struct gpio_private *priv, unsigned long mask, unsigned long dir)
{
	int bit;
	for (bit = 0; bit < PORT_BITS; bit++) {
		if (mask & (1 << bit))
			gpio_set_mode(priv->base + bit, dir, 0);
	}
}

static inline unsigned long
gpio_get_dir(struct gpio_private *priv)
{
	int bit;
	unsigned long ret = 0;
	for (bit = 0; bit < PORT_BITS; bit++) {
		if (GET_GPIO_MODE(SOC_GPIO.PIN[priv->base + bit]) == GPIO_MODE_OUT)
			ret |= (1 << bit);
	}
	return ret;
}

static inline void
gpio_enable_irqs(struct gpio_private *priv, unsigned long mask, int polarity)
{
	int bit;
	for (bit = 0; bit < PORT_BITS; bit++) {
		if (mask & (1 << bit)) { 
			gpio_set_edge_detect_polarity(priv->base + bit, polarity);
			gpio_clear_trig(priv->base + bit);
			gpio_set_mode(priv->base + bit, GPIO_MODE_IN_IRQ, 0);
		}
	}	
}


static unsigned int
gpio_poll(struct file *file, poll_table *wait)
{
	struct gpio_private *priv = (struct gpio_private *)file->private_data;
	unsigned int val;

	poll_wait(file, &priv->alarm_wq, wait);
	val = gpio_read_bits(priv);

	/* Unmask GPIO interrupt */
	SOC_IRQ.MASK = SOC_IRQ.MASK | (IRQ_GPIO_IRQ_MASK);

	/* Check if alarm is active */
	if ((val & priv->highalarm) ||
	    (~val & priv->lowalarm)) {
		return POLLIN|POLLRDNORM;
	}	
	return 0;
}

static irqreturn_t
gpio_interrupt(int irq, void *dev_id, struct pt_regs* regs)
{
	struct gpio_private *priv = alarmlist;
	unsigned int val = 0;
	int bit;
	int handled = 0;

	/* Look for pending alarms */
	while (priv) {
		if (priv->highalarm || priv->lowalarm) {
			val = gpio_read_bits(priv);
			if ((val & priv->highalarm) || (~val & priv->lowalarm)) {
				wake_up_interruptible(&priv->alarm_wq);
			}
			/* ACK IRQs */
			for (bit = 0; bit < PORT_BITS; bit++) {
				if ((val & (1 << bit) & priv->highalarm) ||
				    (~val & (1 << bit) & priv->lowalarm))
					gpio_clear_trig(priv->base + bit);
			}
			handled = 1;
		}
		priv = priv->next;
	}
	return IRQ_RETVAL(handled);
}

static int
gpio_open(struct inode *inode, struct file *filp)
{
	struct gpio_private* priv;
	int minor =  MINOR(inode->i_rdev);

	D(printk("GPIO open %d\n", minor));

	if (minor > NUM_PORTS)
		return -EINVAL;

	priv = (struct gpio_private*)kmalloc(sizeof *priv, GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
        
	priv->next = alarmlist;
	priv->minor = minor;
	alarmlist = priv;
	priv->highalarm = priv->lowalarm = 0;
	init_waitqueue_head(&priv->alarm_wq);
	filp->private_data = (void *)priv;
	if (minor == 0) {
		priv->base = 0;
		priv->changeable_bits = CONFIG_ARGUS_GPIO_CHANGEABLE_BITS1;
	} else if (minor == 1) {
		priv->base = PORT_BITS;
		priv->changeable_bits = CONFIG_ARGUS_GPIO_CHANGEABLE_BITS2;
	} else if (minor == 3) {
		priv->base = PORT_BITS * 2;
		priv->changeable_bits = CONFIG_ARGUS_GPIO_CHANGEABLE_BITS3;
        }
	return 0;
}

static int
gpio_release(struct inode *inode, struct file *filp)
{
	struct gpio_private *p = alarmlist;
	struct gpio_private *todel = (struct gpio_private *)filp->private_data;
	
	/* unlink from alarmlist and free the private structure */

	if (p == todel) {
		alarmlist = todel->next;
	} else {
		while (p->next != todel)
			p = p->next;
		p->next = todel->next;
	}

	kfree(todel);
	return 0;
}

static int
gpio_leds_ioctl(unsigned int cmd, unsigned long arg);

static int 
gpio_ioctl(struct inode* inode, struct file* file,
           unsigned int cmd, unsigned long arg)
{
	int val = 0;

	struct gpio_private* priv = (struct gpio_private*)file->private_data;

	D(printk("GPIO IOCTL cmd %x arg %X\n", cmd, arg));

	switch(_IOC_NR(cmd)) {
	case IO_READBITS: /* Use IO_READ_INBITS and IO_READ_OUTBITS instead */
		return gpio_read_bits(priv);
	case IO_SETBITS:
		gpio_write_bits(priv, 0xffffffff, arg & priv->changeable_bits);
		break;
	case IO_CLRBITS:
		gpio_write_bits(priv, 0, arg & priv->changeable_bits);
		break;
	case IO_HIGHALARM:	
		// set alarm when bits with 1 in arg go high
		priv->highalarm |= arg;
		gpio_enable_irqs(priv, priv->highalarm,	1);
		break;
	case IO_LOWALARM:
		// set alarm when bits with 1 in arg go low
		priv->lowalarm |= arg;
		gpio_enable_irqs(priv, priv->lowalarm, 0);
		break;	
	case IO_CLRALARM:
		// clear alarm for bits with 1 in arg
		priv->highalarm &= ~arg;
		priv->lowalarm  &= ~arg;
		break;	
	case IO_READDIR: /* Use IO_SETGET_INPUT/OUTPUT instead! */
		gpio_get_dir(priv);
		break;	 
	case IO_SETINPUT: /* Use IO_SETGET_INPUT instead! */
		gpio_set_dir(priv, val & priv->changeable_bits, GPIO_MODE_IN);
		return gpio_read_bits(priv) & ~gpio_get_dir(priv);
	case IO_SETOUTPUT: /* Use IO_SETGET_OUTPUT instead! */
		gpio_set_dir(priv, val & priv->changeable_bits, GPIO_MODE_OUT);
		return gpio_read_bits(priv) & gpio_get_dir(priv);
	case IO_READ_INBITS: 
		if (copy_from_user(&val, (unsigned long*)arg, sizeof(val)))
			return -EFAULT;
		/* *arg is result of reading the input/output pins */
		val = gpio_read_bits(priv) & ~gpio_get_dir(priv);
		if (copy_to_user((unsigned long*)arg, &val, sizeof(val)))
			return -EFAULT;
		break;
	case IO_READ_OUTBITS:
		if (copy_from_user(&val, (unsigned long*)arg, sizeof(val)))
			return -EFAULT;
		/* *arg is result of reading the input/output pins */
		val = gpio_read_bits(priv) & gpio_get_dir(priv);
		if (copy_to_user((unsigned long*)arg, &val, sizeof(val)))
			return -EFAULT;
		break;
	case IO_SETGET_INPUT: 
		/* bits set in *arg is set to input,
		 * *arg updated with current input pins.
		 */
		if (copy_from_user(&val, (unsigned long*)arg, sizeof(val)))
			return -EFAULT;
		gpio_set_dir(priv, val, GPIO_MODE_IN);
		val = ~gpio_get_dir(priv);
		if (copy_to_user((unsigned long*)arg, &val, sizeof(val)))
			return -EFAULT;
		break;
	case IO_SETGET_OUTPUT:
		/* bits set in *arg is set to output,
		 * *arg updated with current output pins.
		 */
		if (copy_from_user(&val, (unsigned long*)arg, sizeof(val)))
			return -EFAULT;
		gpio_set_dir(priv, val, GPIO_MODE_OUT);
		val = gpio_get_dir(priv);
		if (copy_to_user((unsigned long*)arg, &val, sizeof(val)))
			return -EFAULT;
		break;
	default:
		if (priv->minor == GPIO_MINOR_LEDS)
			return gpio_leds_ioctl(cmd, arg);
		else
			return -EINVAL;
	}
	return 0;
}

static int
gpio_leds_ioctl(unsigned int cmd, unsigned long arg)
{
	unsigned char green;
	unsigned char red;

	switch (_IOC_NR(cmd)) {
	case IO_LEDACTIVE_SET:
		green = ((unsigned char) arg) & 1;
		red   = (((unsigned char) arg) >> 1) & 1;
		LED_ACTIVE_SET_G(green);
		LED_ACTIVE_SET_R(red);
		break;
	default:
		return -EINVAL;
	} /* switch */

	return 0;
}

struct file_operations gpio_fops = {
	owner:       THIS_MODULE,
	poll:        gpio_poll,
	ioctl:       gpio_ioctl,
	open:        gpio_open,
	release:     gpio_release,
};

/* Driver initialization */
static __init int
gpio_init(void)
{
	int res;
  
	res = register_chrdev(GPIO_MAJOR, "argus gpio" , &gpio_fops);
  	if (res < 0) {
		printk(KERN_ERR "gpio: couldn't get a major number.\n");
		return res;
	}

	printk("Argus GPIO driver\n");
	
	SOC_IRQ.MASK = SOC_IRQ.MASK &~ (IRQ_GPIO_IRQ_MASK);
	if (request_irq(IRQ_GPIO_IRQ, gpio_interrupt, SA_SHIRQ ,"gpio", &alarmlist)) {
		printk("err: irq for gpio\n");
	}
	return res;

}

module_init(gpio_init);
