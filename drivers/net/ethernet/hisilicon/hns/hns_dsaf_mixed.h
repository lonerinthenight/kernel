/*
 * Copyright (c) 2014-2015 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _IWARE_LED_HAL_H
#define _IWARE_LED_HAL_H

#include "hns_dsaf_mac.h"
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define CPLD_ADDR_PORT_OFFSET	0x4

#define HS_LED_ON		0xE
#define HS_LED_OFF		0xF

#define CPLD_LED_ON_VALUE	0x20
#define CPLD_LED_DEFAULT_VALUE	0x3

#define HNS_LED_FCR			0x10
#define HNS_LED_PCR			0x11
#define MAC_SFP_PORT_OFFSET	0x2

#define DSAF_LED_SPEED_S 0
#define DSAF_LED_SPEED_M (0x3 << DSAF_LED_SPEED_S)

#define DSAF_LED_LINK_B 2
#define DSAF_LED_DATA_B 4
#define DSAF_LED_ANCHOR_B 5

void hns_cpld_set_led(struct hns_mac_cb *mac_cb, int port, int link_status,
		      u16 speed, int data);
void cpld_led_reset(struct hns_mac_cb *mac_cb);
int cpld_set_led_id(struct hns_mac_cb *mac_cb,
		    enum hnae_led_state status);
int hns_mac_get_sfp_prsnt(struct hns_mac_cb *mac_cb, int *sfp_prsnt);

#endif /* _IWARE_LED_HAL_H */
