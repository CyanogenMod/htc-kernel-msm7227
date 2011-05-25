/* arch/arm/mach-msm/board-latte-keypad.c
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

#include "board-latte.h"
#include "proc_comm.h"

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_latte."

module_param_named(keycaps, keycaps, charp, 0);

static unsigned int latte_col_gpios[] = {
	LATTE_GPIO_KP_MKOUT0, /* 35 */
	LATTE_GPIO_KP_MKOUT1, /* 34 */
	LATTE_GPIO_KP_MKOUT2, /* 33 */
	LATTE_GPIO_KP_MKOUT3, /* 120 */
	LATTE_GPIO_KP_MKOUT4, /* 119 */
	LATTE_GPIO_KP_MKOUT5, /* 118 */
	LATTE_GPIO_KP_MKOUT6, /* 117 */
	LATTE_GPIO_KP_MKOUT7, /* 116 */
};
static unsigned int latte_row_gpios[] = {
	LATTE_GPIO_KP_MKIN0, /* 42 */
	LATTE_GPIO_KP_MKIN1, /* 41 */
	LATTE_GPIO_KP_MKIN2, /* 40 */
	LATTE_GPIO_KP_MKIN3, /* 92 */
	LATTE_GPIO_KP_MKIN4, /* 17 */
	LATTE_GPIO_KP_MKIN5, /* 114 */
	LATTE_GPIO_KP_MKIN6, /* 112 */
};

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(latte_row_gpios) + (row))

static const unsigned short latte_keymap_x0[ARRAY_SIZE(latte_col_gpios) *
					ARRAY_SIZE(latte_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_Q,
	[KEYMAP_INDEX(0, 1)] = KEY_I,
	[KEYMAP_INDEX(0, 2)] = KEY_G,
	[KEYMAP_INDEX(0, 3)] = KEY_Z,
	[KEYMAP_INDEX(0, 4)] = KEY_RIGHTSHIFT,
	[KEYMAP_INDEX(0, 5)] = KEY_SEARCH,
	[KEYMAP_INDEX(0, 6)] = KEY_EMAIL, /* @ */

	[KEYMAP_INDEX(1, 0)] = KEY_W,
	[KEYMAP_INDEX(1, 1)] = KEY_O,
	[KEYMAP_INDEX(1, 2)] = KEY_H,
	[KEYMAP_INDEX(1, 3)] = KEY_X,
	[KEYMAP_INDEX(1, 4)] = KEY_ENTER, /* e-SYM */
	[KEYMAP_INDEX(1, 5)] = KEY_F13,
	[KEYMAP_INDEX(1, 6)] = KEY_TAB,

	[KEYMAP_INDEX(2, 0)] = KEY_E,
	[KEYMAP_INDEX(2, 1)] = KEY_P,
	[KEYMAP_INDEX(2, 2)] = KEY_J,
	[KEYMAP_INDEX(2, 3)] = KEY_C,
	[KEYMAP_INDEX(2, 4)] = KEY_LEFTALT, /* FN */
	[KEYMAP_INDEX(2, 5)] = KEY_RIGHTALT,
	[KEYMAP_INDEX(2, 6)] = KEY_DOT,

	[KEYMAP_INDEX(3, 0)] = KEY_R,
	[KEYMAP_INDEX(3, 1)] = KEY_A,
	[KEYMAP_INDEX(3, 2)] = KEY_K,
	[KEYMAP_INDEX(3, 3)] = KEY_V,
	[KEYMAP_INDEX(3, 4)] = KEY_QUESTION, /* ? */
	[KEYMAP_INDEX(3, 5)] = KEY_F16, /* AP1 */
	[KEYMAP_INDEX(3, 6)] = KEY_VOLUMEUP,

	[KEYMAP_INDEX(4, 0)] = KEY_T,
	[KEYMAP_INDEX(4, 1)] = KEY_S,
	[KEYMAP_INDEX(4, 2)] = KEY_L,
	[KEYMAP_INDEX(4, 3)] = KEY_B,
	[KEYMAP_INDEX(4, 4)] = KEY_COMMA, /* , */
	[KEYMAP_INDEX(4, 5)] = KEY_HOME, /* HOME (AP) */
	[KEYMAP_INDEX(4, 6)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(5, 0)] = KEY_Y,
	[KEYMAP_INDEX(5, 1)] = KEY_D,
	[KEYMAP_INDEX(5, 2)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(5, 3)] = KEY_N,
	[KEYMAP_INDEX(5, 4)] = KEY_F14, /* SYM */
	[KEYMAP_INDEX(5, 5)] = BTN_MOUSE, /* OJ-action */
	[KEYMAP_INDEX(5, 6)] = KEY_F17,

	[KEYMAP_INDEX(6, 0)] = KEY_U,
	[KEYMAP_INDEX(6, 1)] = KEY_F,
	[KEYMAP_INDEX(6, 2)] = KEY_LEFTSHIFT, /* SHIFT */
	[KEYMAP_INDEX(6, 3)] = KEY_M,
	[KEYMAP_INDEX(6, 4)] = KEY_SPACE,
	[KEYMAP_INDEX(6, 5)] = KEY_MENU, /* MENU (AP) */
	[KEYMAP_INDEX(6, 6)] = KEY_BACK, /* BACK (AP) */

	[KEYMAP_INDEX(7, 0)] = KEY_HP, /* CAM STEP1 */
	[KEYMAP_INDEX(7, 1)] = KEY_CAMERA, /* CAM STEP2 */
	[KEYMAP_INDEX(7, 2)] = KEY_F15,
	[KEYMAP_INDEX(7, 3)] = KEY_RESERVED,
	[KEYMAP_INDEX(7, 4)] = KEY_RESERVED,
	[KEYMAP_INDEX(7, 5)] = KEY_RESERVED,
	[KEYMAP_INDEX(7, 6)] = KEY_RESERVED,
};

static const unsigned short latte_keymap_x1[ARRAY_SIZE(latte_col_gpios) *
					ARRAY_SIZE(latte_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_Q,
	[KEYMAP_INDEX(0, 1)] = KEY_I,
	[KEYMAP_INDEX(0, 2)] = KEY_G,
	[KEYMAP_INDEX(0, 3)] = KEY_Z,
	[KEYMAP_INDEX(0, 4)] = KEY_LEFTALT,
	[KEYMAP_INDEX(0, 5)] = KEY_SEARCH,
	[KEYMAP_INDEX(0, 6)] = KEY_EMAIL, /* @ */

	[KEYMAP_INDEX(1, 0)] = KEY_W,
	[KEYMAP_INDEX(1, 1)] = KEY_O,
	[KEYMAP_INDEX(1, 2)] = KEY_H,
	[KEYMAP_INDEX(1, 3)] = KEY_X,
	[KEYMAP_INDEX(1, 4)] = KEY_F15, /* HOME */
	[KEYMAP_INDEX(1, 5)] = KEY_F13, /* 0-SYM */
	[KEYMAP_INDEX(1, 6)] = KEY_TAB,

	[KEYMAP_INDEX(2, 0)] = KEY_E,
	[KEYMAP_INDEX(2, 1)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(2, 2)] = KEY_ENTER,
	[KEYMAP_INDEX(2, 3)] = KEY_C,
	[KEYMAP_INDEX(2, 4)] = KEY_RIGHTSHIFT, /* SHIFT_R */
	[KEYMAP_INDEX(2, 5)] = KEY_RIGHTALT,
	[KEYMAP_INDEX(2, 6)] = KEY_DOT,

	[KEYMAP_INDEX(3, 0)] = KEY_R,
	[KEYMAP_INDEX(3, 1)] = KEY_P,
	[KEYMAP_INDEX(3, 2)] = KEY_L,
	[KEYMAP_INDEX(3, 3)] = KEY_V,
	[KEYMAP_INDEX(3, 4)] = KEY_QUESTION, /* ? */
	[KEYMAP_INDEX(3, 5)] = KEY_F14, /* SYM */
	[KEYMAP_INDEX(3, 6)] = KEY_VOLUMEUP,

	[KEYMAP_INDEX(4, 0)] = KEY_T,
	[KEYMAP_INDEX(4, 1)] = KEY_S,
	[KEYMAP_INDEX(4, 2)] = KEY_LEFTSHIFT,
	[KEYMAP_INDEX(4, 3)] = KEY_B,
	[KEYMAP_INDEX(4, 4)] = KEY_COMMA,
	[KEYMAP_INDEX(4, 5)] = KEY_HOME,
	[KEYMAP_INDEX(4, 6)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(5, 0)] = KEY_Y,
	[KEYMAP_INDEX(5, 1)] = KEY_D,
	[KEYMAP_INDEX(5, 2)] = KEY_F,
	[KEYMAP_INDEX(5, 3)] = KEY_N,
	[KEYMAP_INDEX(5, 4)] = KEY_F16, /* BACK */
	[KEYMAP_INDEX(5, 5)] = BTN_MOUSE, /* OJ-action */
	[KEYMAP_INDEX(5, 6)] = KEY_F17,

	[KEYMAP_INDEX(6, 0)] = KEY_A,
	[KEYMAP_INDEX(6, 1)] = KEY_U,
	[KEYMAP_INDEX(6, 2)] = KEY_K,
	[KEYMAP_INDEX(6, 3)] = KEY_M,
	[KEYMAP_INDEX(6, 4)] = KEY_SPACE,
	[KEYMAP_INDEX(6, 5)] = KEY_MENU, /* MENU (AP) */
	[KEYMAP_INDEX(6, 6)] = KEY_BACK, /* BACK (AP) */

	[KEYMAP_INDEX(7, 0)] = KEY_HP, /* CAM STEP1 */
	[KEYMAP_INDEX(7, 1)] = KEY_CAMERA, /* CAM STEP2 */
	[KEYMAP_INDEX(7, 2)] = KEY_J,
	[KEYMAP_INDEX(7, 3)] = KEY_RESERVED,
	[KEYMAP_INDEX(7, 4)] = KEY_RESERVED,
	[KEYMAP_INDEX(7, 5)] = KEY_RESERVED,
	[KEYMAP_INDEX(7, 6)] = KEY_RESERVED,
};

static void latte_matrix_inputs_gpio(void)
{
	static uint32_t matirx_inputs_gpio_table[] = {
		PCOM_GPIO_CFG(LATTE_GPIO_KP_MKIN0, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(LATTE_GPIO_KP_MKIN1, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(LATTE_GPIO_KP_MKIN2, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(LATTE_GPIO_KP_MKIN3, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(LATTE_GPIO_KP_MKIN4, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(LATTE_GPIO_KP_MKIN5, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(LATTE_GPIO_KP_MKIN6, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
	};

	config_gpio_table(matirx_inputs_gpio_table,
		ARRAY_SIZE(matirx_inputs_gpio_table));
}


static struct gpio_event_matrix_info latte_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.info.oj_btn = true,
	.keymap = latte_keymap_x1,
	.output_gpios = latte_col_gpios,
	.input_gpios = latte_row_gpios,
	.noutputs = ARRAY_SIZE(latte_col_gpios),
	.ninputs = ARRAY_SIZE(latte_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS |
		  GPIOKPF_PRINT_MAPPED_KEYS ),
	.setup_ninputs_gpio = latte_matrix_inputs_gpio,
};

static struct gpio_event_direct_entry latte_keypad_power_map[] = {
	{
		.gpio = LATTE_POWER_KEY,
		.code = KEY_POWER,
	},
};

static void latte_power_input_gpio(void)
{
	static uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(LATTE_POWER_KEY, 0, GPIO_INPUT,
					GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table,
		ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info latte_keypad_power_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = latte_keypad_power_map,
	.keymap_size = ARRAY_SIZE(latte_keypad_power_map),
	.setup_input_gpio = latte_power_input_gpio,
};

static struct gpio_event_direct_entry latte_sliding_switch[] = {
	{
		.gpio = LATTE_GPIO_SLIDING_DET,
		.code = SW_LID,
	},
};

static void latte_sliding_input_gpio(void)
{
	static uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(LATTE_GPIO_SLIDING_DET, 0, GPIO_INPUT,
						GPIO_NO_PULL, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table,
		ARRAY_SIZE(inputs_gpio_table));
}


static struct gpio_event_input_info latte_keypad_switch_info = {
	.info.func = gpio_event_input_func,
#if 1
	.flags = GPIOEDF_PRINT_KEYS,
#else
	.flags = GPIOKPF_ACTIVE_HIGH,
	/* FIXME: since sliding is opened on Latte barebone system,
	 * set the active direction inverse on purpose
	 * just for our convenience in current stage.
	 */
#endif
	.type = EV_SW,
	.keymap = latte_sliding_switch,
	.keymap_size = ARRAY_SIZE(latte_sliding_switch),
	.setup_input_gpio = latte_sliding_input_gpio,
};

static struct gpio_event_info *latte_keypad_info[] = {
	&latte_keypad_switch_info.info,
	&latte_keypad_matrix_info.info,
	&latte_keypad_power_info.info,
};

static struct gpio_event_platform_data latte_keypad_data = {
	.name = "latte-keypad-v2",
	.info = latte_keypad_info,
	.info_count = ARRAY_SIZE(latte_keypad_info)
};

static struct platform_device latte_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &latte_keypad_data,
	},
};

static int latte_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data latte_reset_keys_pdata = {
	.keys_up = latte_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

static struct platform_device latte_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &latte_reset_keys_pdata,
};

int __init latte_init_keypad(void)
{
	char *get_cid, *get_carrier, *get_keycaps;

	board_get_cid_tag(&get_cid);
	board_get_carrier_tag(&get_carrier);
	board_get_keycaps_tag(&get_keycaps);
	if (system_rev < 2) {
		latte_keypad_data.name = "latte-keypad-v0";
		latte_keypad_matrix_info.keymap =  latte_keymap_x0;
	}

	if (system_rev == 2)
		latte_keypad_data.name = "latte-keypad-v1";

	if (platform_device_register(&latte_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&latte_keypad_device);
}

