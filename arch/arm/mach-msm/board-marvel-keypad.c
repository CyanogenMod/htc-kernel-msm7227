/* arch/arm/mach-msm/board-marvel-keypad.c
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>

#include <mach/board_htc.h>

#include "board-marvel.h"
#include "proc_comm.h"

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_marvel."

module_param_named(keycaps, keycaps, charp, 0);

static struct gpio_event_direct_entry marvel_keypad_nav_map[] = {
	{
		.gpio = MARVEL_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = MARVEL_GPIO_VOL_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = MARVEL_GPIO_VOL_UP,
		.code = KEY_VOLUMEUP,
	},
};

static void marvel_direct_inputs_gpio(void)
{
	static uint32_t matirx_inputs_gpio_table[] = {
		PCOM_GPIO_CFG(MARVEL_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(MARVEL_GPIO_VOL_DOWN, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(MARVEL_GPIO_VOL_UP, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(MARVEL_GPIO_RESET_BTN_N, 0, GPIO_INPUT,
						GPIO_NO_PULL, GPIO_2MA),
	};

	config_gpio_table(matirx_inputs_gpio_table,
		ARRAY_SIZE(matirx_inputs_gpio_table));
}


static struct gpio_event_input_info marvel_keypad_power_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = marvel_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(marvel_keypad_nav_map),
	.setup_input_gpio = marvel_direct_inputs_gpio,
};

static struct gpio_event_info *marvel_keypad_info[] = {
	&marvel_keypad_power_info.info,
};

static struct gpio_event_platform_data marvel_keypad_data = {
	.name = "marvel-keypad",
	.info = marvel_keypad_info,
	.info_count = ARRAY_SIZE(marvel_keypad_info),
};

static struct platform_device marvel_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &marvel_keypad_data,
	},
};
/*
static int marvel_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/
static struct keyreset_platform_data marvel_reset_keys_pdata = {
	/*.keys_up = marvel_reset_keys_up,*/
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

static struct platform_device marvel_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &marvel_reset_keys_pdata,
};

int __init marvel_init_keypad(void)
{
	if (platform_device_register(&marvel_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&marvel_keypad_device);
}

