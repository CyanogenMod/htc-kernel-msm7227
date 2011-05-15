/* linux/arch/arm/mach-msm/board-legend-mmc.c
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

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>

#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>
#include <mach/msm_iomap.h>

#include "devices.h"
#include "board-legend.h"
#include "proc_comm.h"

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

static int __init legend_disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_legend.disable_sdcard=", legend_disablesdcard_setup);

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

static uint32_t legend_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;

	BUG_ON(!vreg_sdslot);
	if (vdd == sdslot_vdd)
		return 0;

	printk("%s::vdd=%08x sdslot_vdd=%08x\n",__func__,vdd,sdslot_vdd);
	sdslot_vdd = vdd;

	if (vdd == 0) {
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
		printk("off:MCI_Power=%08x\n", readl(MSM_SDC2_BASE));
		writel(readl(MSM_SDC2_BASE) & 0xfffffffc, MSM_SDC2_BASE); /* MCI_Power */
		printk("off:MCI_Power=%08x\n",readl(MSM_SDC2_BASE));
		mdelay(1);
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		mdelay(1);
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		vreg_enable(vreg_sdslot);
	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			printk(KERN_INFO "%s: Setting level to %u\n",
					__func__, mmc_vdd_table[i].level);
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
		}
	}
		mdelay(5);
		writel(readl(MSM_SDC2_BASE) | 0x2, MSM_SDC2_BASE); /* MCI_Power */
		printk("on:MCI_Power=%08x\n", readl(MSM_SDC2_BASE));
		mdelay(5);
		writel(readl(MSM_SDC2_BASE) | 0x3, MSM_SDC2_BASE); /* MCI_Power */
		printk("on:MCI_Power=%08x\n", readl(MSM_SDC2_BASE));
		mdelay(5);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			printk(KERN_INFO "%s: Setting level to %u\n",
					__func__, mmc_vdd_table[i].level);
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
			mdelay(5);
			return 0;
		}
	}
	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int legend_sdslot_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) gpio_get_value(LEGEND_GPIO_SDMC_CD_N);
	return (!status);
}

#define LEGEND_MMC_VDD	(MMC_VDD_28_29 | MMC_VDD_29_30)

static struct mmc_platform_data legend_sdslot_data = {
	.ocr_mask	= LEGEND_MMC_VDD,
	.status_irq	= MSM_GPIO_TO_INT(LEGEND_GPIO_SDMC_CD_N),
	.status		= legend_sdslot_status,
	.translate_vdd	= legend_sdslot_switchvdd,
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
static int legend_wifi_cd;		/* WIFI virtual 'card detect' status */

static struct sdio_embedded_func wifi_func[2] = {
	{.f_class	= SDIO_CLASS_WLAN,
	.f_maxblksize	= 512},
	{.f_class       = SDIO_CLASS_WLAN,
	.f_maxblksize   = 512},
};

static struct embedded_sdio_data legend_wifi_emb_data = {
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
legend_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int legend_wifi_status(struct device *dev)
{
	return legend_wifi_cd;
}

static struct mmc_platform_data legend_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= legend_wifi_status,
	.register_status_notify	= legend_wifi_status_register,
	.embedded_sdio		= &legend_wifi_emb_data,
#ifdef CONFIG_MMC_SUPPORT_EXTERNEL_DRIVER
	.use_ext_sdiodrv	= 1,
	.ext_sdiodrv_name	= "TIWLAN_SDIO",
#endif
};

int legend_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	legend_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(legend_wifi_set_carddetect);

#define ID_WIFI	0
#define ID_BT	1
#define CLK_OFF	0
#define CLK_ON	1
int latte_fast_clk_state_wifi = CLK_OFF;
int latte_fast_clk_state_bt = CLK_OFF;
static DEFINE_SPINLOCK(latte_f_slock);

static int fast_clk_ctl(int on, int id)
{
	unsigned long flags;
	int rc = 0;

	if (!vreg_wifi_osc)
		printk(KERN_DEBUG "--- %s vreg_wifi_osc==NULL ---\n", __func__);

	printk(KERN_DEBUG "--- %s ON=%d, ID=%s ---\n",
		__func__, on, id ? "BT":"WIFI");

	spin_lock_irqsave(&latte_f_slock, flags);
	if (on) {
		if ((CLK_OFF == latte_fast_clk_state_wifi)
			&& (CLK_OFF == latte_fast_clk_state_bt)) {

			rc = vreg_enable(vreg_wifi_osc);
		}

		if (id == ID_BT)
			latte_fast_clk_state_bt = CLK_ON;
		else
			latte_fast_clk_state_wifi = CLK_ON;
	} else {
		if (((id == ID_BT) && (CLK_OFF == latte_fast_clk_state_wifi))
			|| ((id == ID_WIFI)
			&& (CLK_OFF == latte_fast_clk_state_bt))) {

			vreg_disable(vreg_wifi_osc);
		} else {
			printk(KERN_DEBUG "KEEP SLEEP CLK ALIVE\n");
		}

		if (id)
			latte_fast_clk_state_bt = CLK_OFF;
		else
			latte_fast_clk_state_wifi = CLK_OFF;
	}
	spin_unlock_irqrestore(&latte_f_slock, flags);

	return 0;
}

int legend_wifi_power(int on)
{
	int rc = 0;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));

		fast_clk_ctl(1, ID_WIFI);
		mdelay(100);

		gpio_set_value(LEGEND_GPIO_WIFI_EN, 1);
		mdelay(50);
		gpio_set_value(LEGEND_GPIO_WIFI_EN, 0);
		mdelay(1);
		gpio_set_value(LEGEND_GPIO_WIFI_EN, 1);
		mdelay(200);
		htc_pwrsink_set(PWRSINK_WIFI, 70);


		if (rc)
			return rc;
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
		gpio_set_value(LEGEND_GPIO_WIFI_EN, on);
		mdelay(10);

		htc_pwrsink_set(PWRSINK_WIFI, 0);

		fast_clk_ctl(0, ID_WIFI);
	}

	return 0;
}
EXPORT_SYMBOL(legend_wifi_power);

/* Eenable VREG_MMC pin to turn on fastclock oscillator : colin */
int legend_bt_fastclock_power(int on)
{
	int rc;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (vreg_wifi_osc) {
		if (on) {
			rc = fast_clk_ctl(1, ID_BT);

			if (rc) {
				printk(KERN_ERR "Error turn bt_fastclock_power"
							" rc=%d\n", rc);
				return rc;
			}
		} else {
			fast_clk_ctl(0, ID_BT);
		}
	}
	return 0;
}
EXPORT_SYMBOL(legend_bt_fastclock_power);

// static int legend_wifi_reset_state;
int legend_wifi_reset(int on)
{
#if 1
	printk(KERN_INFO "%s: do nothing\n", __func__);
#else
	printk(KERN_INFO "%s: %d\n", __func__, on);
	gpio_set_value(TROUT_GPIO_WIFI_PA_RESETX, !on);
	legend_wifi_reset_state = on;
	mdelay(50);
#endif
	return 0;
}

int __init legend_init_mmc(unsigned int sys_rev)
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

	msm_add_sdcc(1, &legend_wifi_data, 0, 0);


	if (opt_disable_sdcard) {
		printk(KERN_INFO "legend: SD-Card interface disabled\n");
		goto done;
	}

	vreg_sdslot = vreg_get(0, "gp6");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	set_irq_wake(MSM_GPIO_TO_INT(LEGEND_GPIO_SDMC_CD_N), 1);

	msm_add_sdcc(2, &legend_sdslot_data, MSM_GPIO_TO_INT(LEGEND_GPIO_SDMC_CD_N),
			IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);

done:
	return 0;
}
