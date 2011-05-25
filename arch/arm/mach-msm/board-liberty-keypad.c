/* arch/arm/mach-msm/board-liberty-keypad.c
 * Copyright (C) 2009 HTC Corporation.
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

#include "board-liberty.h"
#include "proc_comm.h"

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_liberty."

module_param_named(keycaps, keycaps, charp, 0);

static unsigned int liberty_col_gpios[] = {
	LIBERTY_GPIO_KP_MKOUT0, //35
	LIBERTY_GPIO_KP_MKOUT1, //34
	LIBERTY_GPIO_KP_MKOUT2, //33
};
static unsigned int liberty_row_gpios[] = {
	LIBERTY_GPIO_KP_MKIN0, //42
	LIBERTY_GPIO_KP_MKIN1, //41
	LIBERTY_GPIO_KP_MKIN2, //40
};

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(liberty_row_gpios) + (row))

static const unsigned short liberty_keymap[ARRAY_SIZE(liberty_col_gpios) *
					ARRAY_SIZE(liberty_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(0, 2)] = KEY_RESERVED,

	[KEYMAP_INDEX(1, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(1, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(1, 2)] = KEY_RESERVED,

	[KEYMAP_INDEX(2, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = BTN_MOUSE, /* OJ_ACTION */
};

static void liberty_matrix_inputs_gpio(void)
{
	static uint32_t matirx_inputs_gpio_table[] = {
		PCOM_GPIO_CFG(LIBERTY_GPIO_KP_MKIN0, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(LIBERTY_GPIO_KP_MKIN1, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(LIBERTY_GPIO_KP_MKIN2, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(matirx_inputs_gpio_table,
		ARRAY_SIZE(matirx_inputs_gpio_table));
}

static struct gpio_event_matrix_info liberty_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.info.oj_btn = true,
	.keymap = liberty_keymap,
	.output_gpios = liberty_col_gpios,
	.input_gpios = liberty_row_gpios,
	.noutputs = ARRAY_SIZE(liberty_col_gpios),
	.ninputs = ARRAY_SIZE(liberty_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS /*|
		   GPIOKPF_PRINT_MAPPED_KEYS */),
	.setup_ninputs_gpio = liberty_matrix_inputs_gpio,
	.detect_phone_status = 0,
};

static struct gpio_event_direct_entry liberty_keypad_nav_map[] = {
	{
		.gpio = LIBERTY_POWER_KEY,
		.code = KEY_POWER,
	},
};

static void liberty_direct_inputs_gpio(void)
{
	static uint32_t matirx_inputs_gpio_table[] = {
		PCOM_GPIO_CFG(LIBERTY_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(LIBERTY_GPIO_RESET_BTN_N, 0, GPIO_INPUT,
						GPIO_NO_PULL, GPIO_2MA),
	};

	config_gpio_table(matirx_inputs_gpio_table,
		ARRAY_SIZE(matirx_inputs_gpio_table));
}


static struct gpio_event_input_info liberty_keypad_power_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = liberty_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(liberty_keypad_nav_map),
	.setup_input_gpio = liberty_direct_inputs_gpio,
};

static struct gpio_event_info *liberty_keypad_info[] = {
	&liberty_keypad_matrix_info.info,
	&liberty_keypad_power_info.info,
};

int liberty_gpio_event_power(const struct gpio_event_platform_data *pdata, bool on)
{
       return 0;
}

static struct gpio_event_platform_data liberty_keypad_data = {
	.name = "liberty-keypad",
	.info = liberty_keypad_info,
	.info_count = ARRAY_SIZE(liberty_keypad_info),
	.power = liberty_gpio_event_power,
};

static struct platform_device liberty_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &liberty_keypad_data,
	},
};

static int liberty_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data liberty_reset_keys_pdata = {
	.keys_up = liberty_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

static struct platform_device liberty_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &liberty_reset_keys_pdata,
};

int __init liberty_init_keypad(void)
{
	char *get_cid, *get_carrier, *get_keycaps;

	board_get_cid_tag(&get_cid);
	board_get_carrier_tag(&get_carrier);
	board_get_keycaps_tag(&get_keycaps);

	if (platform_device_register(&liberty_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&liberty_keypad_device);
}

