/* linux/arch/arm/mach-msm/board-icong-panel.c
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

#include "devices.h"
#include "board-icong.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk("[DISP] " s)
#else
#define B(s...) do {} while (0)
#endif

#define DEFAULT_BRIGHTNESS 100

static struct samsung_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} samsung;

static struct led_trigger *icong_lcd_backlight;
static atomic_t lcm_init_done = ATOMIC_INIT(1);

static struct vreg *vreg_lcm_2v6;
static struct vreg *vreg_lcm_2v85;

static void
icong_mddi_samsung_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");
	if (on) {
		on_off = 0;

		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);

		mdelay(1);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);
		mdelay(5);
		gpio_set_value(ICONG_LCD_RSTz, 1);
		msleep(15);
	} else {
		on_off = 1;
		mdelay(50);
		gpio_set_value(ICONG_LCD_RSTz, 0);
		mdelay(3);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v85);

		mdelay(5);

		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v6);
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

static const struct mddi_cmd sharp_init_cmd_table[] = {
	LCM_CMD(0xef, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xf2, 0, 0x14, 0x14, 0x0f, 0x1f,
		0x03, 0x1f, 0x03, 0x12, 0x00, 0x18, 0x18, 0x00),
	LCM_CMD(0xf6, 0, 0x80, 0x10, 0x09, 0x00),
	LCM_CMD(0xfd, 0, 0x33, 0x3b, 0x00, 0x00),
	LCM_CMD(0x2a, 0, 0x00, 0x00, 0x01, 0x3f),
	LCM_CMD(0x2b, 0, 0x00, 0x00, 0x01, 0xdf),
	LCM_CMD(0x36, 0, 0x40, 0x00, 0x00, 0x00),
	LCM_CMD(0x3a, 0, 0x55, 0x00, 0x00, 0x00),
	LCM_CMD(0xe0, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xe1, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x35, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xf7, 0, 0x08, 0x2e, 0x00, 0x2a,
		0x38, 0x3a, 0x35, 0x36, 0x07, 0x05, 0x06, 0x0c, 0x00,
		0x14, 0x14, 0x00),
	LCM_CMD(0xf8, 0, 0x17, 0x13, 0x00, 0x0b,
		0x13, 0x1e, 0x23, 0x25, 0x18, 0x1f, 0x2a, 0x28, 0x00,
		0x14, 0x14, 0x00),
	LCM_CMD(0xf3, 0, 0x01, 0x00, 0x26, 0x26,
		0x08, 0x33, 0x6D, 0x6D, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xf4, 0, 0x4d, 0x4d, 0x5e, 0x5e, 0x77,
		0x00, 0x00, 0x00),
	LCM_CMD(0xf5, 0, 0x03, 0x11, 0x06, 0x00, 0x00,
		0x1f, 0x00, 0x00),
	LCM_CMD(0xcd, 0, 0x66, 0x15, 0x00, 0x00),
	LCM_CMD(0x11, 160, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x29, 0, 0x00, 0x00, 0x00, 0x00),
};

static struct mddi_cmd tpo_init_cmd_table[] = {
	LCM_CMD(0xf3, 0, 0x01, 0x00, 0x2A, 0x00, 0x09, 0x33, 0x75, 0x75, 0x00),
	LCM_CMD(0xf2, 0, 0x16, 0x16, 0x0f, 0x1f, 0x03, 0x1f,
		0x03, 0x10, 0x00, 0x16, 0x16, 0x00),
	LCM_CMD(0xf4, 0, 0x50, 0x50, 0x7f, 0x7f, 0x77, 0x00, 0x00, 0x00),
	LCM_CMD(0xf5, 0, 0x12, 0x00, 0x03, 0x01, 0x01, 0x1d, 0x00, 0x00),
	LCM_CMD(0xfd, 0, 0x11, 0x3b, 0x00, 0x00),
	LCM_CMD(0x35, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x36, 0, 0x48, 0x00, 0x00, 0x00),
	LCM_CMD(0x3a, 0, 0x55, 0x00, 0x00, 0x00),
	LCM_CMD(0xf7, 0, 0x80, 0x0a, 0x00, 0x18, 0x32, 0x2f,
		0x2a, 0x28, 0x13, 0x12, 0x14, 0x18, 0x06, 0x22, 0x22),
	LCM_CMD(0xf8, 0, 0x80, 0x00, 0x00, 0x18, 0x28, 0x24,
		0x21, 0x20, 0x19, 0x17, 0x13, 0x17, 0x05, 0x22, 0x22),
	LCM_CMD(0xf9, 0, 0x94, 0x0a, 0x00, 0x1c, 0x3b, 0x3d,
		0x3b, 0x39, 0x03, 0x03, 0x08, 0x11, 0x04, 0x22, 0x22),
	LCM_CMD(0xfa, 0, 0x80, 0x14, 0x00, 0x22, 0x3b, 0x38,
		0x35, 0x36, 0x03, 0x03, 0x08, 0x10, 0x03, 0x22, 0x22),
	LCM_CMD(0xfb, 0, 0x80, 0x0a, 0x00, 0x29, 0x36, 0x2c,
		0x26, 0x23, 0x1b, 0x22, 0x2c, 0x2c, 0x0f, 0x22, 0x22),
	LCM_CMD(0xfc, 0, 0x80, 0x00, 0x00, 0x23, 0x2b, 0x21,
		0x1d, 0x1b, 0x22, 0x25, 0x2a, 0x29, 0x0d, 0x22, 0x22),
	LCM_CMD(0xcb, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x55, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x2a, 0, 0x00, 0x00, 0x01, 0x3f),
	LCM_CMD(0x2b, 0, 0x00, 0x00, 0x01, 0xdf),
	LCM_CMD(0xef, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x2c, 40, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x53, 0, 0x24, 0x00, 0x00, 0x00),
	LCM_CMD(0xcd, 0, 0x66, 0x15, 0x00, 0x00),
	LCM_CMD(0x11, 120, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x29, 0, 0x00, 0x00, 0x00, 0x00),
};

static const struct mddi_cmd sharp_uninit_cmd_table[] = {
	LCM_CMD(0x28, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x10, 120, 0x00, 0x00, 0x00, 0x00),
};

static const struct mddi_cmd tpo_uninit_cmd_table[] = {
	LCM_CMD(0xef, 45, 0x06, 0x00, 0x00, 0x00),
	LCM_CMD(0xef, 30, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0xf3, 0, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x33, 0x41,
		0x41, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x10, 210, 0x00, 0x00, 0x00, 0x00),
};

enum led_brightness icong_brightness_value = DEFAULT_BRIGHTNESS;

static int
icong_panel_shrink(int brightness)
{
	int panel_id = -1;
	panel_id = gpio_get_value(ICONG_GPIO_LCD_ID0) |
	           (gpio_get_value(ICONG_GPIO_LCD_ID1) << 1);
	if (panel_id == 0) {
		if (brightness <= 143) {
			if (brightness == 0)
				brightness = 0;
			else if (brightness <= 30)
				brightness = 9;
			else
				brightness = 123 * (brightness - 30) / 113 + 9;
		} else
			brightness = 123 * (brightness - 143) / 112 + 132;
	} else {
		if (brightness <= 143) {
			if (brightness == 0)
				brightness = 0;
			else if (brightness <= 30)
				brightness = 9;
			else
				brightness = 106 * (brightness - 30) / 113 + 9;
		} else
			brightness = 115 * (brightness - 143) / 112 + 115;
	}

	return brightness;
}

static void icong_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness val)
{
	struct msm_mddi_client_data *client = samsung.client_data;
	unsigned int shrink_bl = icong_panel_shrink(val);
	if (atomic_read(&lcm_init_done) == 0) {
		return;
	}

	mutex_lock(&samsung.lock);
	client->remote_write(client, 0x01, 0xcb);
	client->remote_write(client, shrink_bl, 0x51);
	client->remote_write(client, 0x0, 0x55);
	client->remote_write(client, 0x24, 0x53);
	client->remote_write(client, 0x0, 0xef);

	mutex_unlock(&samsung.lock);

	icong_brightness_value = val;
}

static enum led_brightness
icong_brightness_get(struct led_classdev *led_cdev)
{
	return icong_brightness_value;
}

static void
icong_backlight_switch(struct msm_mddi_client_data *client_data, int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "[BKL] turn on backlight\n");

		val = samsung.lcd_backlight.brightness;
		printk(KERN_DEBUG "[BKL] icong_brightness_set = %d\n",val);

		icong_brightness_set(&samsung.lcd_backlight, val);
	} else {
		printk(KERN_DEBUG "[BKL] turn off backlight\n");
		printk(KERN_DEBUG "[BKL] icong_brightness_set = 0\n");
		icong_brightness_set(&samsung.lcd_backlight, 0);
	}
}

static int
icong_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	int i = 0;
	int panel_id = -1;
	B(KERN_DEBUG "%s\n", __func__);

	panel_id = gpio_get_value(ICONG_GPIO_LCD_ID0) |
		(gpio_get_value(ICONG_GPIO_LCD_ID1) << 1);

	if (panel_id == 0) {
		for (i = 0; i < ARRAY_SIZE(sharp_init_cmd_table); i++) {
			client_data->remote_write_vals(client_data,
					sharp_init_cmd_table[i].vals,
					sharp_init_cmd_table[i].cmd,
					sharp_init_cmd_table[i].len);
			if (sharp_init_cmd_table[i].delay)
				hr_msleep(sharp_init_cmd_table[i].delay);
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(tpo_init_cmd_table); i++) {
			client_data->remote_write_vals(client_data,
					tpo_init_cmd_table[i].vals,
					tpo_init_cmd_table[i].cmd,
					tpo_init_cmd_table[i].len);
			if (tpo_init_cmd_table[i].delay)
				hr_msleep(tpo_init_cmd_table[i].delay);
		}
	}

	return 0;
}

static int
icong_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
		  struct msm_mddi_client_data *client_data)
{
	int i = 0;
	int panel_id = -1;
	B(KERN_DEBUG "%s\n", __func__);

	panel_id = gpio_get_value(ICONG_GPIO_LCD_ID0) |
		(gpio_get_value(ICONG_GPIO_LCD_ID1) << 1);

	if (panel_id == 0) {
		for (i = 0; i < ARRAY_SIZE(sharp_uninit_cmd_table); i++) {
			client_data->remote_write_vals(client_data,
					sharp_uninit_cmd_table[i].vals,
					sharp_uninit_cmd_table[i].cmd,
					sharp_uninit_cmd_table[i].len);
			if (sharp_uninit_cmd_table[i].delay)
				hr_msleep(sharp_uninit_cmd_table[i].delay);
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(tpo_uninit_cmd_table); i++) {
			client_data->remote_write_vals(client_data,
					tpo_uninit_cmd_table[i].vals,
					tpo_uninit_cmd_table[i].cmd,
					tpo_uninit_cmd_table[i].len);
			if (tpo_uninit_cmd_table[i].delay)
				hr_msleep(tpo_uninit_cmd_table[i].delay);
		}
	}

	return 0;
}

static int
icong_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);

	atomic_set(&lcm_init_done, 1);
	mdelay(50);
	icong_backlight_switch(client_data, LED_FULL);
	return 0;
}

static int
icong_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);

	atomic_set(&lcm_init_done, 0);
	icong_backlight_switch(client_data, LED_OFF);
	return 0;
}

static int icong_panel_shutdown(
                struct msm_mddi_bridge_platform_data *bridge_data,
                struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	icong_mddi_samsung_power(client_data, 0);
	return 0;
}

static void panel_samsung_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	*mfr_name = 0x0101;
	*product_code = 0x05a0;
}

static u8 pwm_samsung[10] = {8, 16, 34, 61, 96, 138, 167, 195, 227, 255};

static struct msm_mddi_bridge_platform_data samsung_client_data = {
	.init = icong_mddi_init,
	.uninit = icong_mddi_uninit,
	.blank = icong_panel_blank,
	.unblank = icong_panel_unblank,
	.shutdown = icong_panel_shutdown,
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

static struct msm_mdp_platform_data icong_mdp_pdata = {
	.tearing_check = 0,
};

static struct msm_mddi_platform_data icong_pdata = {
	.clk_rate = 106000000,
	.power_client = icong_mddi_samsung_power,
	.fixup = panel_samsung_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = 0x010105a0,
			.name = "mddi_c_0101_05a0",
			.id = 1,
			.client_data = &samsung_client_data,
			.clk_rate = 0,
		},
	},
};

static int icong_backlight_probe(struct platform_device *pdev)
{
	mutex_init(&samsung.lock);
	samsung.client_data = pdev->dev.platform_data;
	samsung.lcd_backlight.name = "lcd-backlight";
	samsung.lcd_backlight.brightness_set = icong_brightness_set;
	samsung.lcd_backlight.brightness_get = icong_brightness_get;
	led_classdev_register(&pdev->dev, &samsung.lcd_backlight);

	return 0;
}
static struct platform_driver icong_backlight_pdrv = {
	.probe          = icong_backlight_probe,
	.driver         = {
		.name   = "icong-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init icong_backlight_init(void)
{
	return platform_driver_register(&icong_backlight_pdrv);
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
int __init icong_init_panel(void)
{
	int rc;
	int panel_type = 0;
	int panel_id = -1;
	int gpio_lcd_id0, gpio_lcd_id1;
	uint32_t config;
	struct panel_data *panel_data = &samsung_client_data.panel_conf;
	struct msm_mddi_client_data *client = samsung.client_data;

	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcm_2v6 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_2v6))
		return PTR_ERR(vreg_lcm_2v6);

	vreg_lcm_2v85 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	gpio_lcd_id0 =  PCOM_GPIO_CFG(ICONG_GPIO_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	gpio_lcd_id1 =  PCOM_GPIO_CFG(ICONG_GPIO_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id0, 0);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id1, 0);

	panel_id = gpio_get_value(ICONG_GPIO_LCD_ID0) |
	           (gpio_get_value(ICONG_GPIO_LCD_ID1) << 1);

	B(KERN_INFO "%s: panel_id:%d.\n", __func__,panel_id);

	switch(panel_id) {
	case 0:
		panel_type = 0;
		break;

	case 1:
		panel_type = 1;
		break;
	default:
		icong_mddi_samsung_power(client, 0);
		gpio_lcd_id0 =  PCOM_GPIO_CFG(ICONG_GPIO_LCD_ID0, 0,
					GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
		gpio_lcd_id1 =  PCOM_GPIO_CFG(ICONG_GPIO_LCD_ID1, 0,
					GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id1, 0);
		break;
	}

	panel_data->panel_id = panel_type;
	panel_data->caps = MSMFB_CAP_CABC;
	panel_data->pwm = pwm_samsung;
	panel_data->shrink = 1;
	panel_data->shrink_br = icong_panel_shrink;
	panel_data->default_br = 83;

	config = PCOM_GPIO_CFG(ICONG_GPIO_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

	msm_device_mdp.dev.platform_data = &icong_mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &icong_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	led_trigger_register_simple("lcd-backlight-gate", &icong_lcd_backlight);
	if (IS_ERR(icong_lcd_backlight))
		printk(KERN_ERR "[BKL] %s: backlight registration failed!\n",
			__func__);
	return 0;
}
module_init(icong_backlight_init);
