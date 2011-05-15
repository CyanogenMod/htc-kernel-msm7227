/*
 * Copyright (C) 2009 Google, Inc.
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
 *
 */

/* Control bluetooth power for icong platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include "gpio_chip.h"
#include "proc_comm.h"
#include "board-icong.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

/* bt initial configuration */
static uint32_t icong_bt_init_table[] = {

	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_RTS, /* BT_RTS */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_CTS, /* BT_CTS */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_RX, /* BT_RX */
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_TX, /* BT_TX */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),

	PCOM_GPIO_CFG(ICONG_GPIO_BT_RESET_N, /* BT_RESET_N */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_SD_N, /* BT_SHUTDOWN_N */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),

	PCOM_GPIO_CFG(ICONG_GPIO_BT_HOST_WAKE, /* BT_HOST_WAKE */
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_4MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_CHIP_WAKE, /* BT_CHIP_WAKE */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
};

/* bt on configuration */
static uint32_t icong_bt_on_table[] = {

	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_RTS, /* BT_RTS */
				2,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_CTS, /* BT_CTS */
				2,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_RX, /* BT_RX */
				2,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_TX, /* BT_TX */
				3,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),

	PCOM_GPIO_CFG(ICONG_GPIO_BT_HOST_WAKE, /* BT_HOST_WAKE */
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_4MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_CHIP_WAKE, /* BT_CHIP_WAKE */
				0,
				GPIO_OUTPUT,
				GPIO_PULL_UP,
				GPIO_4MA),

	PCOM_GPIO_CFG(ICONG_GPIO_BT_RESET_N, /* BT_RESET_N */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_SD_N, /* BT_SHUTDOWN_N */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
};

/* bt off configuration */
static uint32_t icong_bt_off_table[] = {

	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_RTS, /* BT_RTS */
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_CTS, /* BT_CTS */
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_RX, /* BT_RX */
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_8MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_UART1_TX, /* BT_TX */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_8MA),

	PCOM_GPIO_CFG(ICONG_GPIO_BT_RESET_N, /* BT_RESET_N */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_SD_N, /* BT_SHUTDOWN_N */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),

	PCOM_GPIO_CFG(ICONG_GPIO_BT_HOST_WAKE, /* BT_HOST_WAKE */
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_4MA),
	PCOM_GPIO_CFG(ICONG_GPIO_BT_CHIP_WAKE, /* BT_CHIP_WAKE */
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_4MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for (n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void icong_config_bt_init(void)
{
	/* set bt initial configuration*/
	config_bt_table(icong_bt_init_table,
				ARRAY_SIZE(icong_bt_init_table));
	mdelay(5);

	/* BT_RTS */
	gpio_configure(ICONG_GPIO_BT_UART1_RTS,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);

	/* BT_CTS */
	gpio_configure(ICONG_GPIO_BT_UART1_CTS,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);

	/* BT_RX */

	/* BT_TX */
	gpio_configure(ICONG_GPIO_BT_UART1_TX,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);

	/* BT_RESET_N */
	gpio_configure(ICONG_GPIO_BT_RESET_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(2);
	/* BT_SHUTDOWN_N */
	gpio_configure(ICONG_GPIO_BT_SD_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(2);

	/* BT_CHIP_WAKE */
	gpio_configure(ICONG_GPIO_BT_CHIP_WAKE,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);

}

static void icong_config_bt_on(void)
{
	/* set bt on configuration*/
	config_bt_table(icong_bt_on_table,
				ARRAY_SIZE(icong_bt_on_table));
	mdelay(5);

	/* BT_RESET_N */
	gpio_configure(ICONG_GPIO_BT_RESET_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(2);
	/* BT_SHUTDOWN_N */
	gpio_configure(ICONG_GPIO_BT_SD_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(2);
}

static void icong_config_bt_off(void)
{
	/* BT_SHUTDOWN_N */
	gpio_configure(ICONG_GPIO_BT_SD_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(2);
	/* BT_RESET_N */
	gpio_configure(ICONG_GPIO_BT_RESET_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(2);

	/* set bt off configuration*/
	config_bt_table(icong_bt_off_table,
				ARRAY_SIZE(icong_bt_off_table));
	mdelay(5);

	/* BT_RTS */

	/* BT_CTS */

	/* BT_TX */
	gpio_configure(ICONG_GPIO_BT_UART1_TX,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	/* BT_RX */

	/* BT_HOST_WAKE */

	/* BT_CHIP_WAKE */
	gpio_configure(ICONG_GPIO_BT_CHIP_WAKE,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
}

#if 1
static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		icong_config_bt_on();
	else
		icong_config_bt_off();

	return 0;
}
#else
static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		icong_config_bt_on();
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		icong_config_bt_off();
		break;
	default:
		pr_err("%s: bad rfkill state %d\n", __func__, state);
	}

	return 0;
}
#endif

#if 1
static struct rfkill_ops icong_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int icong_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true; /* off */

	icong_config_bt_init();	/* bt gpio initial config */

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
						 &icong_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_reset;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_reset:
	return rc;
}
#else
static int icong_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	enum rfkill_state default_state = RFKILL_STATE_SOFT_BLOCKED;

	icong_config_bt_init();	/* bt gpio initial config */

	rfkill_set_default(RFKILL_TYPE_BLUETOOTH, default_state);
	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_rfk)
		return -ENOMEM;

	bt_rfk->name = bt_name;
	bt_rfk->state = default_state;

	/* userspace cannot take exclusive control */
	bt_rfk->user_claim_unsupported = 1;
	bt_rfk->user_claim = 0;
	bt_rfk->data = NULL;
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_free(bt_rfk);
	return rc;
}
#endif

static int icong_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	//rfkill_free(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static struct platform_driver icong_rfkill_driver = {
	.probe = icong_rfkill_probe,
	.remove = icong_rfkill_remove,
	.driver = {
		.name = "icong_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init icong_rfkill_init(void)
{
	if (!machine_is_icong())
		return 0;

	return platform_driver_register(&icong_rfkill_driver);
}

static void __exit icong_rfkill_exit(void)
{
	platform_driver_unregister(&icong_rfkill_driver);
}

module_init(icong_rfkill_init);
module_exit(icong_rfkill_exit);
MODULE_DESCRIPTION("icong rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
