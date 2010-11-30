/*
 * Copyright (C) 2009 Google, Inc.
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
 *
 */

/* Control bluetooth power for legend platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include "gpio_chip.h"
#include "proc_comm.h"
#include "board-legend.h"

extern int legend_bt_fastclock_power(int on);

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6450";

static uint32_t legend_bt_init_table[] = {
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RTS, /* BT_RTS */
			0,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_CTS, /* BT_CTS */
			0,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RX, /* BT_RX */
			0,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_TX, /* BT_TX */
			0,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),

	PCOM_GPIO_CFG(LEGEND_GPIO_BT_EN, /* BT_ENABLE */
			0,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
};

static uint32_t legend_bt_on_table[] = {
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RTS, /* BT_RTS */
			2,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_CTS, /* BT_CTS */
			2,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RX, /* BT_RX */
			2,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_TX, /* BT_TX */
			3,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
};

static uint32_t legend_bt_off_table[] = {
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RTS, /* BT_RTS */
			2,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_CTS, /* BT_CTS */
			2,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RX, /* BT_RX */
			2,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_TX, /* BT_TX */
			3,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
};

#if 0
static uint32_t legend_bt_disable_active_table[] = {
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RTS, /* BT_RTS */
			2,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_CTS, /* BT_CTS */
			2,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RX, /* BT_RX */
			2,
			GPIO_INPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_TX, /* BT_TX */
			3,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
};

static uint32_t legend_bt_disable_sleep_table[] = {
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RTS, /* O(L) */
			0,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_CTS, /* I(PU) */
			0,
			GPIO_INPUT,
			GPIO_PULL_UP,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_RX, /* I(PU) */
			0,
			GPIO_INPUT,
			GPIO_PULL_UP,
			GPIO_8MA),
	PCOM_GPIO_CFG(LEGEND_GPIO_UART1_TX, /* O(H) */
			0,
			GPIO_OUTPUT,
			GPIO_NO_PULL,
			GPIO_8MA),
};
#endif

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for (n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void legend_config_bt_init(void)
{
	config_bt_table(legend_bt_init_table,
			ARRAY_SIZE(legend_bt_init_table));
	mdelay(2);

	gpio_configure(LEGEND_GPIO_BT_EN,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(2);
}

static void legend_config_bt_on(void)
{
	config_bt_table(legend_bt_on_table,
			ARRAY_SIZE(legend_bt_on_table));
	mdelay(2);

	gpio_configure(LEGEND_GPIO_BT_EN,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(15);
	gpio_configure(LEGEND_GPIO_BT_EN,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(1);
	gpio_configure(LEGEND_GPIO_BT_EN,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(1);

	legend_bt_fastclock_power(1);
	mdelay(2);
}

static void legend_config_bt_off(void)
{
	gpio_configure(LEGEND_GPIO_BT_EN,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(2);

	legend_bt_fastclock_power(0);
	mdelay(2);

	/* set bt off configuration*/
	config_bt_table(legend_bt_off_table,
			ARRAY_SIZE(legend_bt_off_table));
	mdelay(2);
}

#if 0
void legend_config_bt_disable_active(void)
{
	config_bt_table(legend_bt_disable_active_table,
			ARRAY_SIZE(legend_bt_disable_active_table));
}

void legend_config_bt_disable_sleep(void)
{
	config_bt_table(legend_bt_disable_sleep_table,
			ARRAY_SIZE(legend_bt_disable_sleep_table));
	mdelay(5);
	gpio_configure(LEGEND_GPIO_UART1_RTS,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* O(L) */
	gpio_configure(LEGEND_GPIO_UART1_TX,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH); /* O(H) */
}

int legend_is_bluetooth_off(void)
{
	return !legend_bt_status;	/* ON:1, OFF:0 */
}
#endif

#if 1
static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		legend_config_bt_on();
	else
		legend_config_bt_off();

	return 0;
}
#else
static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		legend_config_bt_on();
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		legend_config_bt_off();
		break;
	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}
#endif

#if 1
static struct rfkill_ops legend_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int legend_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true; /* off */

	legend_config_bt_init();	/* bt gpio initial config */

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
						 &legend_rfkill_ops, NULL);
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
static int legend_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	enum rfkill_state default_state = RFKILL_STATE_SOFT_BLOCKED;  /* off */

	/* force BT on and off to do GPIO setting when initiate */
	bluetooth_set_power(NULL, RFKILL_STATE_UNBLOCKED);
	legend_config_bt_init();	/* bt gpio initial config */

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
	bt_rfk->data = NULL;  /* user data */
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_free(bt_rfk);
	return rc;
}
#endif

static int legend_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	//rfkill_free(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static struct platform_driver legend_rfkill_driver = {
	.probe = legend_rfkill_probe,
	.remove = legend_rfkill_remove,
	.driver = {
		.name = "legend_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init legend_rfkill_init(void)
{
	if (!machine_is_legend())
		return 0;
	return platform_driver_register(&legend_rfkill_driver);
}

static void __exit legend_rfkill_exit(void)
{
	platform_driver_unregister(&legend_rfkill_driver);
}

module_init(legend_rfkill_init);
module_exit(legend_rfkill_exit);
MODULE_DESCRIPTION("legend rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
