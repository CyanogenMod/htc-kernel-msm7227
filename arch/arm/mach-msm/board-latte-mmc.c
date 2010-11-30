/* linux/arch/arm/mach-msm/board-latte-mmc.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>

#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>

#include "devices.h"
#include "board-latte.h"
#include "proc_comm.h"

#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include "../../../drivers/mmc/host/msm_sdcc.h"

#include <linux/jiffies.h>

static unsigned long last_bt_enable_time=0;
#define WIFI_ENABLE_DELAY_MS 1000

/* #include <linux/irq.h> */

extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);

/* ---- SDCARD ---- */
static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_6MA), /* CLK */
	PCOM_GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_6MA), /* CMD */
	PCOM_GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_6MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_6MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_6MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_6MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
};

static uint opt_disable_sdcard;

static int __init latte_disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_espresso.disable_sdcard=", latte_disablesdcard_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

static struct mmc_vdd_xlat mmc_vdd_table[] = {
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2900 },
};

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t latte_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
		writel(MCI_PWR_OFF, MSM_SDC2_BASE + MMCIPOWER);
		mdelay(1);
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		mdelay(5);
		vreg_enable(vreg_sdslot);
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			printk(KERN_INFO "%s: Setting level to %u\n",
					__func__, mmc_vdd_table[i].level);
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
			if (!sdslot_vreg_enabled)
				break;
			else
				return 0;
		}
	}

	/* All vdd match failed */
	if (i == ARRAY_SIZE(mmc_vdd_table))
		goto out;

	if (!sdslot_vreg_enabled) {
		u32 pwr = 0;

		/* Power on MCI controller */
		mdelay(5);
		pwr = readl(MSM_SDC2_BASE + MMCIPOWER);
		writel(pwr | MCI_PWR_UP, MSM_SDC2_BASE + MMCIPOWER);
		mdelay(5);
		pwr = readl(MSM_SDC2_BASE + MMCIPOWER);
		writel(pwr | MCI_PWR_ON, MSM_SDC2_BASE + MMCIPOWER);
		mdelay(5);

		/* ..then, config GPIO */
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
		return 0;
	}

out:
	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int latte_sdslot_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) gpio_get_value(LATTE_GPIO_SDMC_CD_N);
	return (!status);
}

#define LATTE_MMC_VDD	MMC_VDD_28_29 | MMC_VDD_29_30

static unsigned int latte_sdslot_type = MMC_TYPE_SD;

static struct mmc_platform_data latte_sdslot_data = {
	.ocr_mask	= LATTE_MMC_VDD,
	.status_irq	= MSM_GPIO_TO_INT(LATTE_GPIO_SDMC_CD_N),
	.status		= latte_sdslot_status,
	.translate_vdd	= latte_sdslot_switchvdd,
	.slot_type	= &latte_sdslot_type,
};

/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* CLK */
	PCOM_GPIO_CFG(29, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(51, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(56, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(29, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static struct vreg *vreg_wifi_osc;	/* WIFI 32khz oscilator */
static int latte_wifi_cd;		/* WIFI virtual 'card detect' status */

static struct sdio_embedded_func wifi_func[2] = {
	{.f_class	= SDIO_CLASS_WLAN,
	.f_maxblksize	= 512},
	{.f_class       = SDIO_CLASS_WLAN,
	.f_maxblksize   = 512},
};

static struct embedded_sdio_data latte_wifi_emb_data = {
	.cis	= {
		.vendor		= 0x104c,
		.device		= 0x9066,
		.blksize	= 512,
		/* .max_dtr	= 24000000, */
		.max_dtr	= 25000000,
	},
	.cccr	= {
		.multi_block	= 0,
		.low_speed	= 0,
		.wide_bus	= 1,
		.high_power	= 0,
		.high_speed	= 0,
	},
	.funcs	= wifi_func,
	.num_funcs = 2,
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
latte_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int latte_wifi_status(struct device *dev)
{
	return latte_wifi_cd;
}

static struct mmc_platform_data latte_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= latte_wifi_status,
	.register_status_notify	= latte_wifi_status_register,
	.embedded_sdio		= &latte_wifi_emb_data,
#ifdef CONFIG_MMC_SUPPORT_EXTERNEL_DRIVER
	.use_ext_sdiodrv	= 1,
	.ext_sdiodrv_name	= "TIWLAN_SDIO",
#endif
};

int latte_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	latte_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(latte_wifi_set_carddetect);

int latte_wifi_power_state = 0;
int latte_bt_power_state = 0;

int latte_wifi_power(int on)
{
	int rc = 0;
	unsigned long t, v;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		t = jiffies;

		v = t - last_bt_enable_time;
		//printk(KERN_INFO "%s: v: %lu\n", __func__, v);
		if (v < HZ) {
			printk(KERN_INFO "%s: workarond for BT/WIFI initial at the same time - delay wifi enabling (msleep(%d))\n",
					__func__, WIFI_ENABLE_DELAY_MS);
			msleep(WIFI_ENABLE_DELAY_MS);
			printk(KERN_INFO "%s: finish wifi delaying\n", __func__);
		}

		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));

		rc = vreg_enable(vreg_wifi_osc);
		mdelay(100);

		gpio_set_value(LATTE_GPIO_WIFI_EN, 1);
		mdelay(50);
		gpio_set_value(LATTE_GPIO_WIFI_EN, 0);
		mdelay(1);
		gpio_set_value(LATTE_GPIO_WIFI_EN, 1);
		mdelay(200);
		htc_pwrsink_set(PWRSINK_WIFI, 70);


		if (rc)
			return rc;
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
		gpio_set_value(LATTE_GPIO_WIFI_EN, on);
		mdelay(10);

		htc_pwrsink_set(PWRSINK_WIFI, 0);

		if (!latte_bt_power_state) {
			vreg_disable(vreg_wifi_osc);
			printk(KERN_INFO "WiFi disable vreg_wifi_osc.\n");
		} else
			printk(KERN_ERR "WiFi shouldn't disable "
					"vreg_wifi_osc. BT is using it!!\n");
	}

	latte_wifi_power_state = on;
	return 0;
}
EXPORT_SYMBOL(latte_wifi_power);

/* Eenable VREG_MMC pin to turn on fastclock oscillator : colin */
int latte_bt_fastclock_power(int on)
{
	int rc;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (vreg_wifi_osc) {
		if (on) {
			rc = vreg_enable(vreg_wifi_osc);

			last_bt_enable_time = jiffies;
			//printk(KERN_INFO "%s: enable time %lu\n", __func__, last_bt_enable_time);

			if (rc) {
				printk(KERN_ERR "Error turn bt_fastclock_power"
							" rc=%d\n", rc);
				return rc;
			}
		} else {
			if (!latte_wifi_power_state)
				vreg_disable(vreg_wifi_osc);
		}
	}
	latte_bt_power_state = on;
	return 0;
}
EXPORT_SYMBOL(latte_bt_fastclock_power);

// static int latte_wifi_reset_state;
int latte_wifi_reset(int on)
{
#if 1
	printk(KERN_INFO "%s: do nothing\n", __func__);
#else
	printk(KERN_INFO "%s: %d\n", __func__, on);
	gpio_set_value(TROUT_GPIO_WIFI_PA_RESETX, !on);
	latte_wifi_reset_state = on;
	mdelay(50);
#endif
	return 0;
}

int __init latte_init_mmc(unsigned int sys_rev)
{

	uint32_t id;
	sdslot_vreg_enabled = 0;


	/* initial LEVEL_SHIT_EN */
	id = PCOM_GPIO_CFG(30, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	/* let level shifter alaways enabled,
	   and remove all GPIO30 configuration codes */
	gpio_set_value(30, 0);

	wifi_status_cb = NULL;


	printk(KERN_INFO "%s\n", __func__);

	vreg_wifi_osc = vreg_get(0, "rftx");
	if (IS_ERR(vreg_wifi_osc))
		return PTR_ERR(vreg_wifi_osc);
	vreg_set_level(vreg_wifi_osc, 1800);

	msm_add_sdcc(1, &latte_wifi_data, 0, 0);


	if (opt_disable_sdcard) {
		printk(KERN_INFO "latte: SD-Card interface disabled\n");
		goto done;
	}

	vreg_sdslot = vreg_get(0, "gp6");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	set_irq_wake(MSM_GPIO_TO_INT(LATTE_GPIO_SDMC_CD_N), 1);

	msm_add_sdcc(2, &latte_sdslot_data, MSM_GPIO_TO_INT(LATTE_GPIO_SDMC_CD_N),
			IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);
done:
	return 0;
}
