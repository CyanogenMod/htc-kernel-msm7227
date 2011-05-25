/* linux/arch/arm/mach-msm/board-latte-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
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
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
/*#include <mach/pmic.h> */

#include "devices.h"
#include "board-latte.h"
#include "proc_comm.h"

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

static struct led_trigger *latte_lcd_backlight;
static void latte_set_backlight(int on)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	if (on) {
		/* vsync back porch is about 17 ms */
		hr_msleep(40);
		led_trigger_event(latte_lcd_backlight, LED_FULL);
	} else
		led_trigger_event(latte_lcd_backlight, LED_OFF);
}

static struct vreg *vreg_lcm_2v6;
static struct vreg *vreg_lcm_2v85;

static void
latte_mddi_eid_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");
	if (on) {
		on_off = 0;
		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);
		hr_msleep(1);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);
		hr_msleep(5);

		gpio_set_value(LATTE_LCD_RSTz, 1);
		hr_msleep(15);
	} else {
		on_off = 1;
		hr_msleep(150);
		gpio_set_value(LATTE_LCD_RSTz, 0);
		hr_msleep(3);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v85);
		hr_msleep(5);

		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v6);
	}
}

static int
latte_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	latte_set_backlight(1);
	return 0;
}

static int
latte_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	latte_set_backlight(0);
	return 0;
}

#define LATTE_BR_DEF_USER_TOP           102
#define LATTE_BR_MIN_USER_TOP           30
#define LATTE_BR_MAX_USER_TOP           255
#define LATTE_BR_DEF_PANEL_TOP          122
#define LATTE_BR_MIN_PANEL_TOP          8
#define LATTE_BR_MAX_PANEL_TOP          250

#define LATTE_BR_DEF_USER_SHARP         102
#define LATTE_BR_MIN_USER_SHARP         30
#define LATTE_BR_MAX_USER_SHARP         255
#define LATTE_BR_DEF_PANEL_SHARP        122
#define LATTE_BR_MIN_PANEL_SHARP        8
#define LATTE_BR_MAX_PANEL_SHARP        255

static int
latte_panel_shrink_top(int brightness)
{
        if (brightness <= LATTE_BR_DEF_USER_TOP) {
                if (brightness <= LATTE_BR_MIN_USER_TOP)
                        brightness = LATTE_BR_MIN_PANEL_TOP;
                else
                        brightness = (LATTE_BR_DEF_PANEL_TOP - LATTE_BR_MIN_PANEL_TOP) *
                                (brightness - LATTE_BR_MIN_USER_TOP) /
                                (LATTE_BR_DEF_USER_TOP - LATTE_BR_MIN_USER_TOP) +
                                LATTE_BR_MIN_PANEL_TOP;
                } else
                        brightness = (LATTE_BR_MAX_PANEL_TOP - LATTE_BR_DEF_PANEL_TOP) *
                                (brightness - LATTE_BR_DEF_USER_TOP) /
                                (LATTE_BR_MAX_USER_TOP - LATTE_BR_DEF_USER_TOP) +
                                LATTE_BR_DEF_PANEL_TOP;

        return brightness;
}

static int
latte_panel_shrink_sharp(int brightness)
{
        if (brightness <= LATTE_BR_DEF_USER_SHARP) {
                if (brightness <= LATTE_BR_MIN_USER_SHARP)
                        brightness = LATTE_BR_MIN_PANEL_SHARP;
                else
                        brightness = (LATTE_BR_DEF_PANEL_SHARP - LATTE_BR_MIN_PANEL_SHARP) *
                                (brightness - LATTE_BR_MIN_USER_SHARP) /
                                (LATTE_BR_DEF_USER_SHARP - LATTE_BR_MIN_USER_SHARP) +
                                LATTE_BR_MIN_PANEL_SHARP;
                } else
                        brightness = (LATTE_BR_MAX_PANEL_SHARP - LATTE_BR_DEF_PANEL_SHARP) *
                                (brightness - LATTE_BR_DEF_USER_SHARP) /
                                (LATTE_BR_MAX_USER_SHARP - LATTE_BR_DEF_USER_SHARP) +
                                LATTE_BR_DEF_PANEL_SHARP;

        return brightness;
}

static void panel_eid_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	B("%s: enter.\n", __func__);
	*mfr_name = 0x0101;
	*product_code = 0x0;
}

static u8 pwm_eid[10] = {8, 16, 34, 61, 96, 138, 167, 195, 227, 255};

static struct msm_mddi_bridge_platform_data eid_client_data = {
	.blank = latte_panel_blank,
	.unblank = latte_panel_unblank,
	.fb_data = {
		.xres = 320,
		.yres = 480,
		.output_format = 0,
	},
};

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_mddi_platform_data latte_pdata = {
	.clk_rate = 106000000,
	.power_client = latte_mddi_eid_power,
	.fixup = panel_eid_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0x0101 << 16 | 0),
			.name = "mddi_c_0101_0000",
			.id = 1,
			.client_data = &eid_client_data,
			.clk_rate = 0,
		},
	},
};

/*
 * In boot loader, mddi is powered on already.
 *
 * So, we just detect panel here, setting different
 * power function for each panel.
 *
 * Then we did not have to detect panel in each time
 * mddi_client_power or panel_power is called.
 *
 * jay: Nov 20, 08'
 */
int __init latte_init_panel(void)
{
	int rc;
	int panel_type = 0;
	int panel_id = -1;
	int gpio_lcd_id0, gpio_lcd_id1;
	uint32_t config;
	struct panel_data *panel_data = &eid_client_data.panel_conf;

	if (!machine_is_latte())
		return -1;

	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcm_2v6 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_2v6))
		return PTR_ERR(vreg_lcm_2v6);

	vreg_lcm_2v85 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	gpio_lcd_id0 =  PCOM_GPIO_CFG(LATTE_GPIO_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	gpio_lcd_id1 =  PCOM_GPIO_CFG(LATTE_GPIO_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id0, 0);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id1, 0);

	panel_id = gpio_get_value(LATTE_GPIO_LCD_ID0) |
		(gpio_get_value(LATTE_GPIO_LCD_ID1) << 1);

	B(KERN_INFO "%s: panel_id: %d\n", __func__, panel_id);

	switch(panel_id) {
		case 0:
			panel_type = 8;
			panel_data->shrink_br = latte_panel_shrink_sharp;
		break;
		case 1:
			panel_type = 7;
			panel_data->shrink_br = latte_panel_shrink_top;
		break;
		default:
			return -1;
		break;
        }
	panel_data->panel_id = panel_type;
	panel_data->caps = MSMFB_CAP_CABC;
	panel_data->pwm = pwm_eid;
        panel_data->shrink = 1;
	panel_data->default_br = 66;

	config = PCOM_GPIO_CFG(LATTE_GPIO_LCD_VSYNC, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &latte_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	led_trigger_register_simple("lcd-backlight-gate", &latte_lcd_backlight);
	if (IS_ERR(latte_lcd_backlight))
		printk(KERN_ERR "%s: backlight registration failed!\n",
			__func__);
	return 0;

}
device_initcall(latte_init_panel);

