/* arch/arm/mach-msm/board-chacha-keypad.c
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

#include "board-chacha.h"
#include "proc_comm.h"

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_chacha."

module_param_named(keycaps, keycaps, charp, 0);

static unsigned int chacha_col_gpios[] = {
	CHACHA_GPIO_KP_MKOUT0, CHACHA_GPIO_KP_MKOUT1,
	CHACHA_GPIO_KP_MKOUT2, CHACHA_GPIO_KP_MKOUT3,
	CHACHA_GPIO_KP_MKOUT4, CHACHA_GPIO_KP_MKOUT5,
	CHACHA_GPIO_KP_MKOUT6
};
static unsigned int chacha_row_gpios[] = {
	CHACHA_GPIO_KP_MKIN0, CHACHA_GPIO_KP_MKIN1,
	CHACHA_GPIO_KP_MKIN2, CHACHA_GPIO_KP_MKIN3,
	CHACHA_GPIO_KP_MKIN4, CHACHA_GPIO_KP_MKIN5,
	CHACHA_GPIO_KP_MKIN6
};

#define KEYMAP_NUM_COLS		ARRAY_SIZE(chacha_col_gpios)
#define KEYMAP_NUM_ROWS		ARRAY_SIZE(chacha_row_gpios)
#define KEYMAP_INDEX(col, row)	(((col) * KEYMAP_NUM_ROWS) + (row))
#define KEYMAP_SIZE		(KEYMAP_NUM_ROWS * KEYMAP_NUM_COLS)

static unsigned short chacha_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_F,
	[KEYMAP_INDEX(0, 1)] = KEY_P,
	[KEYMAP_INDEX(0, 2)] = KEY_L,
	[KEYMAP_INDEX(0, 3)] = KEY_Z,
	[KEYMAP_INDEX(0, 4)] = KEY_C,
	[KEYMAP_INDEX(0, 5)] = KEY_SEND,
	[KEYMAP_INDEX(0, 6)] = KEY_RESERVED,

	[KEYMAP_INDEX(1, 0)] = KEY_S,
	[KEYMAP_INDEX(1, 1)] = KEY_LEFTALT,
	[KEYMAP_INDEX(1, 2)] = KEY_D,
	[KEYMAP_INDEX(1, 3)] = KEY_M,
	[KEYMAP_INDEX(1, 4)] = KEY_I,
	[KEYMAP_INDEX(1, 5)] = KEY_E,
	[KEYMAP_INDEX(1, 6)] = KEY_RESERVED,

	[KEYMAP_INDEX(2, 0)] = KEY_R,
	[KEYMAP_INDEX(2, 1)] = KEY_ENTER,
	[KEYMAP_INDEX(2, 2)] = KEY_END,
	[KEYMAP_INDEX(2, 3)] = KEY_X,
	[KEYMAP_INDEX(2, 4)] = KEY_U,
	[KEYMAP_INDEX(2, 5)] = KEY_J,
	[KEYMAP_INDEX(2, 6)] = KEY_RESERVED,

	[KEYMAP_INDEX(3, 0)] = KEY_W,
	[KEYMAP_INDEX(3, 1)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(3, 2)] = KEY_QUESTION,
	[KEYMAP_INDEX(3, 3)] = KEY_SPACE,
	[KEYMAP_INDEX(3, 4)] = KEY_Q,
	[KEYMAP_INDEX(3, 5)] = KEY_F13, /*sym*/
	[KEYMAP_INDEX(3, 6)] = KEY_VOLUMEUP,

	[KEYMAP_INDEX(4, 0)] = KEY_F14, /*share*/
	[KEYMAP_INDEX(4, 1)] = KEY_LEFTSHIFT,
	[KEYMAP_INDEX(4, 2)] = KEY_K,
	[KEYMAP_INDEX(4, 3)] = KEY_TAB,
	[KEYMAP_INDEX(4, 4)] = KEY_O,
	[KEYMAP_INDEX(4, 5)] = KEY_LEFT,
	[KEYMAP_INDEX(4, 6)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(5, 0)] = KEY_B,
	[KEYMAP_INDEX(5, 1)] = KEY_RIGHT,
	[KEYMAP_INDEX(5, 2)] = KEY_T,
	[KEYMAP_INDEX(5, 3)] = KEY_UP,
	[KEYMAP_INDEX(5, 4)] = KEY_Y,
	[KEYMAP_INDEX(5, 5)] = KEY_H,
	[KEYMAP_INDEX(5, 6)] = KEY_RESERVED,

	[KEYMAP_INDEX(6, 0)] = KEY_G,
	[KEYMAP_INDEX(6, 1)] = KEY_DOWN,
	[KEYMAP_INDEX(6, 2)] = KEY_N,
	[KEYMAP_INDEX(6, 3)] = KEY_COMMA,
	[KEYMAP_INDEX(6, 4)] = KEY_V,
	[KEYMAP_INDEX(6, 5)] = KEY_DOT,
	[KEYMAP_INDEX(6, 6)] = KEY_RESERVED,
};

static void chacha_matrix_inputs_gpio(void)
{
	static uint32_t matirx_inputs_gpio_table[] = {
		PCOM_GPIO_CFG(CHACHA_GPIO_KP_MKIN0, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(CHACHA_GPIO_KP_MKIN1, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(CHACHA_GPIO_KP_MKIN2, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(CHACHA_GPIO_KP_MKIN3, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(CHACHA_GPIO_KP_MKIN4, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(CHACHA_GPIO_KP_MKIN5, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
		PCOM_GPIO_CFG(CHACHA_GPIO_KP_MKIN6, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(matirx_inputs_gpio_table,
		ARRAY_SIZE(matirx_inputs_gpio_table));
}

static struct gpio_event_matrix_info chaha_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.info.oj_btn = true,
	.keymap = chacha_keymap,
	.output_gpios = chacha_col_gpios,
	.input_gpios = chacha_row_gpios,
	.noutputs = ARRAY_SIZE(chacha_col_gpios),
	.ninputs = ARRAY_SIZE(chacha_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS /*|
		   GPIOKPF_PRINT_MAPPED_KEYS */),
	.setup_ninputs_gpio = chacha_matrix_inputs_gpio,
	.detect_phone_status = 1,
};

static struct gpio_event_direct_entry chacha_keypad_nav_map[] = {
	{
		.gpio = CHACHA_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = CHACHA_GPIO_KEY_DIRECT,
		.code = KEY_A,
	},
};

static void chacha_direct_inputs_gpio(void)
{
	static uint32_t matirx_inputs_gpio_table[] = {
		PCOM_GPIO_CFG(CHACHA_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP,
								GPIO_4MA),
		PCOM_GPIO_CFG(CHACHA_GPIO_KEY_DIRECT, 0, GPIO_INPUT,
						GPIO_NO_PULL, GPIO_2MA),
	};

	config_gpio_table(matirx_inputs_gpio_table,
		ARRAY_SIZE(matirx_inputs_gpio_table));
}

static struct gpio_event_input_info chacha_keypad_power_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = chacha_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(chacha_keypad_nav_map),
	.setup_input_gpio = chacha_direct_inputs_gpio,
};

static struct gpio_event_info *chacha_keypad_info[] = {
	&chacha_keypad_power_info.info,
	&chaha_keypad_matrix_info.info,
};

int chacha_gpio_event_power(const struct gpio_event_platform_data *pdata, bool on)
{
       return 0;
}

static struct gpio_event_platform_data chacha_keypad_data = {
	.name = "chacha-keypad",
	.info = chacha_keypad_info,
	.info_count = ARRAY_SIZE(chacha_keypad_info),
	.power = chacha_gpio_event_power,
};

static struct platform_device chacha_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data	= &chacha_keypad_data,
	},
};
/*
static int chacha_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/
static struct keyreset_platform_data chacha_reset_keys_pdata = {
	/*.keys_up = chacha_reset_keys_up,*/
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

static struct platform_device chacha_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &chacha_reset_keys_pdata,
};

char *SKU_WWE = "(HTC__001),(HTC__016),(HTC__032),(VIRGI001),(HTC__E11),"\
		"(H3G__001),(O2___001),(ORANG001),(ORANG309),(ORANGB10),(T-MOB005),(VODAP001),(VODAPE17)";
char *SKU_ELL = "(VODAP006)";
char *SKU_FRA = "(HTC__203),(HTC__247),(ORANG202),(VODAP203)";
char *SKU_ARA = "(HTC__J15)";
char *SKU_ESN = "(HTC__304),(VODAP304),(VODAPD18)";
char *SKU_GER = "(HTC__102),(O2___102),(ORANG216),(VODAP102),(VODAP110),(VODAP120)";
char *SKU_ITA = "(HTC__405)";
char *SKU_RUS = "(HTC__A07)";
char *SKU_HK = "";
char *SKU_NOR = "(HTC__Y13)";
char *SKU_TUR = "(HTC__M27)";
char *SKU_BOPOMO = "(HTC_621)";
char *SKU_CN = "";
char *SKU_AW = "(CWS__001)";

int __init chacha_init_keypad(void)
{
	char *get_cid;
	char *get_carrier;
	char *get_keycaps;
	uint8_t cid_len;

	KEY_LOGD("%s\n",	__func__);
	board_get_cid_tag(&get_cid);
	board_get_carrier_tag(&get_carrier);
	board_get_keycaps_tag(&get_keycaps);
	KEY_LOGI("%s: get CID: %s\n\tCarrier: %s, Keycaps: %s\n",
		__func__, get_cid, get_carrier, get_keycaps);
	cid_len = strlen(get_cid);

	if (cid_len) {
		if (strstr(SKU_BOPOMO, get_cid) != NULL) {

			chacha_keypad_data.name = "chacha-keypad-bopomo";

		} else if (strstr(SKU_FRA, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(0, 3)] = KEY_W;
			chacha_keymap[KEYMAP_INDEX(3, 0)] = KEY_Z;
			chacha_keymap[KEYMAP_INDEX(3, 4)] = KEY_A;
			chacha_keypad_nav_map[1].code = KEY_Q;
			chacha_keypad_data.name = "chacha-keypad-fra";

		} else if (strstr(SKU_ITA, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(4, 3)] = KEY_F15;
			chacha_keypad_data.name = "chacha-keypad-ita";

		} else if (strstr(SKU_TUR, get_cid) != NULL) {

			chacha_keypad_data.name = "chacha-keypad-tur";

		} else if (strstr(SKU_ELL, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(4, 3)] = KEY_F15;
			chacha_keypad_data.name = "chacha-keypad-ell";

		} else if (strstr(SKU_GER, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(0, 3)] = KEY_Y;
			chacha_keymap[KEYMAP_INDEX(5, 4)] = KEY_Z;
			chacha_keypad_data.name = "chacha-keypad-ger";

		} else if (strstr(SKU_ARA, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(4, 3)] = KEY_F15;
			chacha_keypad_data.name = "chacha-keypad-ara";

		} else if (strstr(SKU_ESN, get_cid) != NULL) {

			chacha_keypad_data.name = "chacha-keypad-esn";

		} else if (strstr(SKU_NOR, get_cid) != NULL) {

			chacha_keypad_data.name = "chacha-keypad-nor";

		} else if (strstr(SKU_RUS, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(4, 3)] = KEY_F15;
			chacha_keymap[KEYMAP_INDEX(3, 5)] = KEY_F16;
			chacha_keypad_data.name = "chacha-keypad-rus";

		} else if (strstr(SKU_HK, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(4, 3)] = KEY_F15;
			chacha_keypad_data.name = "chacha-keypad-hk";

		} else if (strstr(SKU_CN, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(4, 3)] = KEY_F15;
			chacha_keypad_data.name = "chacha-keypad-cn";

		} else if (strstr(SKU_AW, get_cid) != NULL) {

			chacha_keymap[KEYMAP_INDEX(4, 3)] = KEY_F15;
			chacha_keypad_data.name = "chacha-keypad-aw";

		} else {

			if (strstr(SKU_WWE, get_cid) == NULL)
				KEY_LOGI("%s: CID not matched\n", __func__);

		}
	}

	if (platform_device_register(&chacha_reset_keys_device))
		KEY_LOGE("%s: register reset key fail\n", __func__);

	return platform_device_register(&chacha_keypad_device);
}

