/*
 *  Copyright (C) 2003-2004 Axis AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARGUS_GPIO_H
#define ARGUS_GPIO_H

#include <asm/arch/argus.h>

// Set both the mode and level for a GPIO pin

#define GPIO_MODE_MASK (SET_GPIO_MODE(GPIO_MODE_ALT_FUNC))
#define GPIO_LEVEL_MASK (SET_GPIO_LEVEL(1))
#define GPIO_EDP_MASK (SET_GPIO_EDP(1))
#define GPIO_MODE_LEVEL_MASK (GPIO_MODE_MASK | GPIO_LEVEL_MASK)

static inline void 
gpio_set_mode(unsigned int nbr, unsigned int mode,
	      unsigned int level) {
	SOC_GPIO.PIN[nbr] = (SOC_GPIO.PIN[nbr] & ~GPIO_MODE_LEVEL_MASK) |
	  SET_GPIO_MODE(mode) | SET_GPIO_LEVEL(level);
}

static inline void 
gpio_clear_trig(unsigned int nbr) {
	SOC_GPIO.PIN[nbr] |= SET_GPIO_TRIG(1);
}

// val = 1 means rising

static inline void 
gpio_set_edge_detect_polarity(unsigned int nbr, unsigned int val) {
	SOC_GPIO.PIN[nbr] = (SOC_GPIO.PIN[nbr] & ~GPIO_EDP_MASK) |
	  SET_GPIO_EDP(val);
}

static inline unsigned int 
gpio_get_level(unsigned int nbr) {
	return GET_GPIO_LEVEL(SOC_GPIO.PIN[nbr]);
}

static inline unsigned int 
gpio_get_trig(unsigned int nbr) {
	return GET_GPIO_TRIG(SOC_GPIO.PIN[nbr]);
}

#define LED_OFF    0x00
#define LED_GREEN  0x01
#define LED_RED    0x02
#define LED_ORANGE (LED_GREEN | LED_RED)
#define LED_LEFT_GREEN  0x04
#define LED_LEFT_RED    0x08
#define LED_LEFT_ORANGE (LED_LEFT_GREEN | LED_LEFT_RED)
#define LED_RIGHT_GREEN  0x10
#define LED_RIGHT_RED    0x20
#define LED_RIGHT_ORANGE (LED_RIGHT_GREEN | LED_RIGHT_RED)

#if CONFIG_ARGUS_LED1G == CONFIG_ARGUS_LED1R 
#define LED_NETWORK_SET(x)                          \
	do {                                        \
		LED_NETWORK_SET_G((x) & LED_GREEN); \
	} while (0)
#else
#define LED_NETWORK_SET(x)                          \
	do {                                        \
		LED_NETWORK_SET_G((x) & LED_GREEN); \
		LED_NETWORK_SET_R((x) & LED_RED);   \
	} while (0)
#endif

#if CONFIG_ARGUS_LED2G == CONFIG_ARGUS_LED2R 
#define LED_ACTIVE_SET(x)                           \
	do {                                        \
		LED_ACTIVE_SET_G((x) & LED_GREEN);  \
	} while (0)
#else
#if CONFIG_ARGUS_LED2G == CONFIG_ARGUS_LED2GR 
#define LED_ACTIVE_SET(x)                           \
	do {                                        \
		LED_ACTIVE_SET_G((x) & LED_GREEN);  \
		LED_ACTIVE_SET_R((x) & LED_RED);    \
	} while (0)
#else
/* Status indicator has two LED, one to the left and one to the right
 * of sensor.
 * 
 */
#define LED_ACTIVE_SET(x)                           \
	do {                                        \
		LED_ACTIVE_LEFT_SET_G((x) & (LED_GREEN | LED_LEFT_GREEN));  \
		LED_ACTIVE_RIGHT_SET_G((x) & (LED_GREEN | LED_RIGHT_GREEN));  \
		LED_ACTIVE_LEFT_SET_R((x) & (LED_RED | LED_LEFT_RED));    \
		LED_ACTIVE_RIGHT_SET_R((x) & (LED_RED | LED_RIGHT_RED));    \
	} while (0)
#endif
#endif

#if CONFIG_ARGUS_LED4G == CONFIG_ARGUS_LED4R 
#define LED_NETWORK_WLAN_SET(x)                          \
	do {                                             \
		LED_NETWORK_WLAN_SET_G((x) & LED_GREEN); \
	} while (0)
#else
#define LED_NETWORK_WLAN_SET(x)                          \
	do {                                             \
		LED_NETWORK_WLAN_SET_G((x) & LED_GREEN); \
		LED_NETWORK_WLAN_SET_R((x) & LED_RED);   \
	} while (0)
#endif


#define LED_NETWORK_SET_G(x) gpio_set_mode(CONFIG_ARGUS_LED1G, GPIO_MODE_OUT, x ? 0 : 1)
#define LED_NETWORK_SET_R(x) gpio_set_mode(CONFIG_ARGUS_LED1R, GPIO_MODE_OUT, x ? 0 : 1)

#define LED_NETWORK_WLAN_SET_G(x) gpio_set_mode(CONFIG_ARGUS_LED4G, GPIO_MODE_OUT, x ? 0 : 1)
#define LED_NETWORK_WLAN_SET_R(x) gpio_set_mode(CONFIG_ARGUS_LED4R, GPIO_MODE_OUT, x ? 0 : 1)

#if !defined(CONFIG_ARGUS_LED2GR) || (CONFIG_ARGUS_LED2GR == CONFIG_ARGUS_LED2G)
#define LED_ACTIVE_SET_G(x) gpio_set_mode(CONFIG_ARGUS_LED2G, GPIO_MODE_OUT, x ? 0 : 1)
#else
#define LED_ACTIVE_SET_G(x) do { \
  gpio_set_mode(CONFIG_ARGUS_LED2G, GPIO_MODE_OUT, x ? 0 : 1); \
  gpio_set_mode(CONFIG_ARGUS_LED2GR, GPIO_MODE_OUT, x ? 0 : 1); } while (0)
#endif

#if !defined(CONFIG_ARGUS_LED2RR) || (CONFIG_ARGUS_LED2RR == CONFIG_ARGUS_LED2R)
#define LED_ACTIVE_SET_R(x) gpio_set_mode(CONFIG_ARGUS_LED2R, GPIO_MODE_OUT, x ? 0 : 1)
#else
#define LED_ACTIVE_SET_R(x) do { \
  gpio_set_mode(CONFIG_ARGUS_LED2R, GPIO_MODE_OUT, x ? 0 : 1); \
  gpio_set_mode(CONFIG_ARGUS_LED2RR, GPIO_MODE_OUT, x ? 0 : 1); } while (0)
#endif

#define LED_ACTIVE_LEFT_SET_G(x) gpio_set_mode(CONFIG_ARGUS_LED2G, GPIO_MODE_OUT, x ? 0 : 1)
#define LED_ACTIVE_LEFT_SET_R(x) gpio_set_mode(CONFIG_ARGUS_LED2R, GPIO_MODE_OUT, x ? 0 : 1)
#define LED_ACTIVE_RIGHT_SET_G(x) gpio_set_mode(CONFIG_ARGUS_LED2GR, GPIO_MODE_OUT, x ? 0 : 1)
#define LED_ACTIVE_RIGHT_SET_R(x) gpio_set_mode(CONFIG_ARGUS_LED2RR, GPIO_MODE_OUT, x ? 0 : 1)

#endif
