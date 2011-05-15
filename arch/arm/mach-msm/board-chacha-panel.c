/* linux/arch/arm/mach-msm/board-chacha-panel.c
 *
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


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/leds.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#include <mach/atmega_microp.h>

#include "devices.h"
#include "board-chacha.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk("[DISP] " s)
#else
#define B(s...) do {} while (0)
#endif

#define DEFAULT_BRIGHTNESS 100

#define PWM_USER_DEF 142
#define PWM_USER_MIN 30
#define PWM_USER_MAX 255

static struct renesas_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} renesas;

static atomic_t lcm_init_done = ATOMIC_INIT(1);
static struct led_trigger *chacha_lcd_backlight;

static void chacha_set_backlight(int on)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	if (on) {
		/* vsync back porch is about 17 ms */
		msleep(40);
		led_trigger_event(chacha_lcd_backlight, LED_FULL);
	} else
		led_trigger_event(chacha_lcd_backlight, LED_OFF);
}

static struct vreg *vreg_lcm_2v6;
static struct vreg *vreg_lcm_2v85;

static void
chacha_mddi_renesas_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");
	if (on) {
		on_off = 0;

		gpio_set_value(CHACHA_LCD_RSTz, 1);
		mdelay(5);

		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);

		mdelay(1);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);

		mdelay(16);
		gpio_set_value(CHACHA_LCD_RSTz, 0);
		mdelay(16);
		gpio_set_value(CHACHA_LCD_RSTz, 1);
		msleep(100);
	} else {
		on_off = 1;

		mdelay(60);
		gpio_set_value(CHACHA_LCD_RSTz, 0);
		mdelay(2);

		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v6);

		mdelay(1);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v85);
	}
}

struct mddi_cmd {
	u8 cmd;
	unsigned delay;
	u8 *vals;
	unsigned len;
};

#define LCM_CMD(_cmd, _delay, ...)                              \
{                                                               \
	.cmd = _cmd,                                            \
	.delay = _delay,                                        \
	.vals = (u8 []){__VA_ARGS__},                           \
	.len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}

static const struct mddi_cmd renesas_init_cmd_table[] = {
	LCM_CMD(0xb0, 0, 0x04, 0x00, 0x00, 0x00),
	LCM_CMD(0x36, 0, 0x08, 0x00, 0x00, 0x00),
	LCM_CMD(0x3a, 0, 0x07, 0x00, 0x00, 0x00),
	LCM_CMD(0x35, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xb8, 0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,
		0x0f, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00,
		0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
		0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x85, 0x00, 0x00, 0x00,
		0x0a, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xc0, 0, 0x01, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x40, 0x00,
		0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xc1, 0, 0x07, 0x00, 0x00, 0x00, 0x2d, 0x00, 0x00, 0x00, 0x05, 0x00,
		0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xc4, 0, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
		0x00, 0x00, 0x02, 0x00, 0x00, 0x00),
	LCM_CMD(0xc8, 0, 0x04, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00,
		0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x30, 0x00,
		0x00, 0x00, 0x59, 0x00, 0x00, 0x00, 0x4c, 0x00, 0x00, 0x00, 0x40, 0x00,
		0x00, 0x00, 0x35, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x1a, 0x00,
		0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00,
		0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x30, 0x00,
		0x00, 0x00, 0x59, 0x00, 0x00, 0x00, 0x4c, 0x00, 0x00, 0x00, 0x40, 0x00,
		0x00, 0x00, 0x35, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x1a, 0x00,
		0x00, 0x00),
	LCM_CMD(0xc9, 0, 0x0c, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x13, 0x00,
		0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x34, 0x00,
		0x00, 0x00, 0x58, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x38, 0x00,
		0x00, 0x00, 0x2b, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x1a, 0x00,
		0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x13, 0x00,
		0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x34, 0x00,
		0x00, 0x00, 0x57, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x38, 0x00,
		0x00, 0x00, 0x2b, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x1a, 0x00,
		0x00, 0x00),
	LCM_CMD(0xca, 0, 0x04, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x14, 0x00,
		0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x37, 0x00,
		0x00, 0x00, 0x56, 0x00, 0x00, 0x00, 0x46, 0x00, 0x00, 0x00, 0x3a, 0x00,
		0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x2d, 0x00, 0x00, 0x00, 0x1a, 0x00,
		0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x14, 0x00,
		0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x37, 0x00,
		0x00, 0x00, 0x56, 0x00, 0x00, 0x00, 0x46, 0x00, 0x00, 0x00, 0x3a, 0x00,
		0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x2d, 0x00, 0x00, 0x00, 0x1a, 0x00,
		0x00, 0x00),
	LCM_CMD(0xd0, 0, 0x29, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x0e, 0x00,
		0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0x04, 0x00,
		0x00, 0x00, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
		0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
		0x00, 0x00, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x00, 0x00, 0x00),
	LCM_CMD(0xd1, 0, 0x02, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x35, 0x00,
		0x00, 0x00, 0x69, 0x00, 0x00, 0x00),
	LCM_CMD(0xe1, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00),
	LCM_CMD(0xe2, 0, 0x80, 0x00, 0x00, 0x00),
	LCM_CMD(0xfd, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x00, 0x00, 0x00, 0x31, 0x00,
		0x00, 0x00, 0x37, 0x00, 0x00, 0x00,	0x70, 0x00, 0x00, 0x00, 0x32, 0x00,
		0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x60, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x2a, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x00, 0x00, 0xdf, 0x00, 0x00, 0x00),
	LCM_CMD(0x2b, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x00, 0x00, 0x3f, 0x00, 0x00, 0x00),
	LCM_CMD(0x11, 130, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x29, 0, 0x00, 0x00, 0x00, 0x00),
};

static struct mddi_cmd renesas_backlight_cmd_table[] = {
	LCM_CMD(0xb9, 0, 0x01, 0x00, 0x00, 0x00,
		0xff, 0x00, 0x00, 0x00,
		0x03, 0x00, 0x00, 0x00,
		0x18, 0x00, 0x00, 0x00),
};

static const struct mddi_cmd renesas_uninit_cmd_table[] = {
	LCM_CMD(0x28, 50, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x10, 130, 0x00, 0x00, 0x00, 0x00),
};

enum led_brightness chacha_brightness_value = DEFAULT_BRIGHTNESS;

static int
chacha_panel_shrink(int brightness)
{
	unsigned int panel_def, panel_min, panel_max;

	panel_def = 126;
	panel_min = 10;
	panel_max = 255;

	if (brightness <= PWM_USER_DEF) {
		if (brightness == 0)
			return 0;
		else if (brightness <= PWM_USER_MIN)
			brightness = panel_min;
		else
			brightness = (panel_def - panel_min) * (brightness - PWM_USER_MIN)
					/ (PWM_USER_DEF - PWM_USER_MIN) + panel_min;
	} else {
		brightness = (panel_max - panel_def) * (brightness - PWM_USER_DEF)
				/ (PWM_USER_MAX - PWM_USER_DEF) + panel_def;
	}
	return brightness;
}

static void
chacha_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness val)
{
	struct msm_mddi_client_data *client = renesas.client_data;
	unsigned int shrink_bl = chacha_panel_shrink(val);
	struct mddi_cmd *pcmd = renesas_backlight_cmd_table;

	uint8_t data[4] = {     /* PWM setting of microp, see p.8 */
			0x05,           /* Fading time; suggested: 5/10/15/20/25 */
			val,            /* Duty Cycle */
			0x00,           /* Channel H byte */
			0x01,           /* Channel L byte */
			};

	if (atomic_read(&lcm_init_done) == 0) {
		return;
	}

	pcmd->vals[4] = shrink_bl;

	mutex_lock(&renesas.lock);

	client->remote_write_vals(client, pcmd->vals, pcmd->cmd, pcmd->len);
	microp_i2c_write(0x25, data, sizeof(data));

	chacha_brightness_value = val;
	mutex_unlock(&renesas.lock);
}

static enum led_brightness
chacha_brightness_get(struct led_classdev *led_cdev)
{
	return chacha_brightness_value;
}

static void
chacha_backlight_switch(struct msm_mddi_client_data *client_data, int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "[BKL] turn on backlight\n");

		val = renesas.lcd_backlight.brightness;
		printk(KERN_DEBUG "[BKL] chacha_brightness_set = %d\n",val);

		chacha_brightness_set(&renesas.lcd_backlight, val);
	} else {
		printk(KERN_DEBUG "[BKL] turn off backlight\n");
		printk(KERN_DEBUG "[BKL] chacha_brightness_set = 0\n");
		chacha_brightness_set(&renesas.lcd_backlight, 0);
	}
}

static int
chacha_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0;
	B(KERN_DEBUG "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(renesas_init_cmd_table); i++) {
		client_data->remote_write_vals(client_data,
				renesas_init_cmd_table[i].vals,
				renesas_init_cmd_table[i].cmd,
				renesas_init_cmd_table[i].len);
		if (renesas_init_cmd_table[i].delay)
			hr_msleep(renesas_init_cmd_table[i].delay);
	}
	B(KERN_DEBUG "%s end\n", __func__);
	return 0;
}

static int
chacha_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	int i = 0;
	B(KERN_DEBUG "%s: enter.\n", __func__);

	for (i = 0; i < ARRAY_SIZE(renesas_uninit_cmd_table); i++) {
		client_data->remote_write_vals(client_data,
				renesas_uninit_cmd_table[i].vals,
				renesas_uninit_cmd_table[i].cmd,
				renesas_uninit_cmd_table[i].len);
		if (renesas_uninit_cmd_table[i].delay)
			hr_msleep(renesas_uninit_cmd_table[i].delay);
	}
	return 0;
}

static int
chacha_AUO_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);

	atomic_set(&lcm_init_done, 1);
	mdelay(50);
	chacha_backlight_switch(client_data, LED_FULL);

	return 0;
}

static int
chacha_AUO_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);

	atomic_set(&lcm_init_done, 0);
	chacha_backlight_switch(client_data, LED_OFF);

	return 0;
}

static int
chacha_liberty_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	chacha_set_backlight(1);
	return 0;
}

static int
chacha_liberty_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	chacha_set_backlight(0);
	return 0;
}


static int
chacha_panel_shutdown(
                struct msm_mddi_bridge_platform_data *bridge_data,
                struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	chacha_mddi_renesas_power(client_data, 0);
	return 0;
}

static void panel_renesas_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	int panel_id = -1;
	B(KERN_DEBUG "%s: enter.\n", __func__);

	panel_id = gpio_get_value(CHACHA_GPIO_LCD_ID0) |
		(gpio_get_value(CHACHA_GPIO_LCD_ID1) << 1);

	if (panel_id == 0) {
		*mfr_name = 0x0101;
		*product_code = 0x0;
	} else {
		*mfr_name = 0x0;
		*product_code = 0x1531;
	}
}

static u8 pwm_renesas[10] = {8, 16, 34, 61, 96, 138, 167, 195, 227, 255};

static struct msm_mddi_bridge_platform_data renesas_liberty_client_data = {
	.blank = chacha_liberty_panel_blank,
	.unblank = chacha_liberty_panel_unblank,
	.shutdown = chacha_panel_shutdown,
	.fb_data = {
		.xres = 320,
		.yres = 480,
		.output_format = 0,
	},
};

static struct msm_mddi_bridge_platform_data renesas_AUO_client_data = {
	.init = chacha_mddi_init,
	.uninit = chacha_mddi_uninit,
	.blank = chacha_AUO_panel_blank,
	.unblank = chacha_AUO_panel_unblank,
	.shutdown = chacha_panel_shutdown,
	.fb_data = {
		.xres = 480,
		.yres = 320,
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

static struct msm_mdp_platform_data chacha_mdp_pdata = {
	.color_format = MSM_MDP_OUT_IF_FMT_RGB888,
};

static struct msm_mddi_platform_data chacha_pdata = {
	.clk_rate = 160000000,
	.power_client = chacha_mddi_renesas_power,
	.fixup = panel_renesas_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			.product_id = (0x0101 << 16 | 0),
			.name = "mddi_c_0101_0000",
			.id = 1,
			.client_data = &renesas_liberty_client_data,
			.clk_rate = 0,
		},
		{
			.product_id = 0x00001531,
			.name = "mddi_renesas_0000_1531",
			.id = 1,
			.client_data = &renesas_AUO_client_data,
			.clk_rate = 0,
		},
	},
};

static int chacha_backlight_probe(struct platform_device *pdev)
{
	mutex_init(&renesas.lock);
	renesas.client_data = pdev->dev.platform_data;
	renesas.lcd_backlight.name = "lcd-backlight";
	renesas.lcd_backlight.brightness_set = chacha_brightness_set;
	renesas.lcd_backlight.brightness_get = chacha_brightness_get;
	led_classdev_register(&pdev->dev, &renesas.lcd_backlight);

	return 0;
}
static struct platform_driver chacha_backlight_pdrv = {
	.probe          = chacha_backlight_probe,
	.driver         = {
		.name   = "chacha-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init chacha_backlight_init(void)
{
	return platform_driver_register(&chacha_backlight_pdrv);
}

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
int __init chacha_init_panel(void)
{
	int rc;
	int panel_type = 0;
	int panel_id = -1;
	int gpio_lcd_id0, gpio_lcd_id1;
	uint32_t config;
	struct panel_data *panel_data = &renesas_AUO_client_data.panel_conf;
	struct msm_mddi_client_data *client = renesas.client_data;

	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcm_2v6 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_2v6))
		return PTR_ERR(vreg_lcm_2v6);

	vreg_lcm_2v85 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	gpio_lcd_id0 =  PCOM_GPIO_CFG(CHACHA_GPIO_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	gpio_lcd_id1 =  PCOM_GPIO_CFG(CHACHA_GPIO_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id0, 0);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id1, 0);

	panel_id = gpio_get_value(CHACHA_GPIO_LCD_ID0) |
	           (gpio_get_value(CHACHA_GPIO_LCD_ID1) << 1);

	B(KERN_INFO "%s: panel_id:%d.\n", __func__,panel_id);

	switch(panel_id) {
		case 0:
			panel_type = 10;
			panel_data = &renesas_liberty_client_data.panel_conf;
		break;

		case 1:
			panel_type = 13;
			panel_data = &renesas_AUO_client_data.panel_conf;
		break;
		default:
			chacha_mddi_renesas_power(client, 0);
			gpio_lcd_id0 =  PCOM_GPIO_CFG(CHACHA_GPIO_LCD_ID0, 0,
						GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
			gpio_lcd_id1 =  PCOM_GPIO_CFG(CHACHA_GPIO_LCD_ID1, 0,
						GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
			msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id0, 0);
			msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id1, 0);
		break;
	}

	panel_data->panel_id = panel_type;
	panel_data->caps = MSMFB_CAP_CABC;
	panel_data->pwm = pwm_renesas;
	panel_data->shrink = 1;
	panel_data->shrink_br = chacha_panel_shrink;
	panel_data->default_br = 83;

	config = PCOM_GPIO_CFG(CHACHA_GPIO_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

	msm_device_mdp.dev.platform_data = &chacha_mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &chacha_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	led_trigger_register_simple("lcd-backlight-gate", &chacha_lcd_backlight);
	if (IS_ERR(chacha_lcd_backlight))
		printk(KERN_ERR "[BKL] %s: backlight registration failed!\n",
			__func__);
	return 0;
}
module_init(chacha_backlight_init);

