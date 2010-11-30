/* arch/arm/mach-msm/board-legend-wifi.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <linux/wifi_tiwlan.h>

extern int legend_wifi_set_carddetect(int val);
extern int legend_wifi_power(int on);
extern int legend_wifi_reset(int on);


struct wifi_platform_data legend_wifi_control = {
	.set_power		= legend_wifi_power,
	.set_reset		= legend_wifi_reset,
	.set_carddetect	= legend_wifi_set_carddetect,
	.mem_prealloc	= NULL,
};

static struct platform_device wifi_ctrl_dev = {
	.name		= "msm_wifi",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
	.dev		= {
		.platform_data = &legend_wifi_control,
	},
};

static int __init legend_wifi_init(void)
{
	int ret;

	if (!machine_is_legend())
		return 0;

	printk("%s: start\n", __func__);
	ret = platform_device_register(&wifi_ctrl_dev);
	return ret;
}


late_initcall(legend_wifi_init);

