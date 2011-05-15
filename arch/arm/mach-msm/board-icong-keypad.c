/* arch/arm/mach-msm/board-icong-keypad.c
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

#include "board-icong.h"
#include "proc_comm.h"

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_icong."

module_param_named(keycaps, keycaps, charp, 0);

static struct gpio_event_direct_entry icong_keypad_nav_map[] = {
	{
		.gpio = ICONG_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = ICONG_GPIO_VOL_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = ICONG_GPIO_VOL_UP,
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = ICONG_GPIO_FB_KEY,
		.code = KEY_F10,
	},
	{
		.gpio = ICONG_GPIO_CAM_STEP1,
		.code = KEY_HP,
	},
	{
		.gpio = ICONG_GPIO_CAM_STEP2,
		.code = KEY_CAMERA,
	},
};

static void icong_direct_inputs_gpio(void)
{
	static uint32_t matirx_inputs_gpio_table[] = {
		PCOM_GPIO_CFG(ICONG_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(ICONG_GPIO_VOL_DOWN, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(ICONG_GPIO_VOL_UP, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(ICONG_GPIO_FB_KEY, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(ICONG_GPIO_CAM_STEP1, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(ICONG_GPIO_CAM_STEP2, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(ICONG_GPIO_RESET_BTN_N, 0, GPIO_INPUT,
						GPIO_NO_PULL, GPIO_2MA),
	};

	config_gpio_table(matirx_inputs_gpio_table,
		ARRAY_SIZE(matirx_inputs_gpio_table));
}


static struct gpio_event_input_info icong_keypad_power_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = icong_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(icong_keypad_nav_map),
	.setup_input_gpio = icong_direct_inputs_gpio,
};

static struct gpio_event_info *icong_keypad_info[] = {
	&icong_keypad_power_info.info,
};

int icong_gpio_event_power(const struct gpio_event_platform_data *pdata, bool on)
{
       return 0;
}

static struct gpio_event_platform_data icong_keypad_data = {
	.name = "icong-keypad",
	.info = icong_keypad_info,
	.info_count = ARRAY_SIZE(icong_keypad_info),
	.power = icong_gpio_event_power,
};

static struct platform_device icong_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &icong_keypad_data,
	},
};
/*
static int icong_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/
static struct keyreset_platform_data icong_reset_keys_pdata = {
	/*.keys_up = icong_reset_keys_up,*/
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

static struct platform_device icong_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &icong_reset_keys_pdata,
};

int __init icong_init_keypad(void)
{
	if (platform_device_register(&icong_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&icong_keypad_device);
}

