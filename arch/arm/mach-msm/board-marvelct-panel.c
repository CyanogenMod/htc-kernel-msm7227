/* linux/arch/arm/mach-msm/board-marvelct-panel.c
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
#include "board-marvelct.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk("[DISP] " s)
#else
#define B(s...) do {} while (0)
#endif

#define DEFAULT_BRIGHTNESS 100

static struct novatek_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} novatek;

static struct led_trigger *marvelct_lcd_backlight;

static struct vreg *vreg_lcm_2v6;
static struct vreg *vreg_lcm_2v85;

static atomic_t lcm_init_done = ATOMIC_INIT(1);

static void
marvelct_mddi_novatek_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");
	if (on) {
		on_off = 0;
		/* 2V8(pmic rftx) */
		id = PM_VREG_PDOWN_RFTX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);

		mdelay(20);

		/* 2V6(pmic gp1) */
		id = PM_VREG_PDOWN_CAM_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);

		gpio_set_value(MARVELCT_LCD_RSTz, 0);
		mdelay(10);

		gpio_set_value(MARVELCT_LCD_RSTz, 1);
		msleep(2);
	} else {
		on_off = 1;
		gpio_set_value(MARVELCT_LCD_RSTz, 0);
		mdelay(120);

		/* 2V8(pmic rftx) */
		id = PM_VREG_PDOWN_RFTX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v85);

		/* 2V6(pmic gp1) */
		id = PM_VREG_PDOWN_CAM_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v6);
	}
}

#define REG_WAIT (0xffff)
struct nov_regs {
	unsigned reg;
	unsigned val;
};

static struct nov_regs  marvelct_auo_panel_cmd[] = {
	{REG_WAIT, 60},
	{0xF000, 0xAA},
	{0xF001, 0x55},
	{0xF002, 0x52},

	{0x1100, 0x00},
	{REG_WAIT, 120},

	{0xFD02, 0x05},

	{0xB100, 0x65},
	{0xB101, 0x01},
	{0xB102, 0x28},
	{0xB200, 0x65},
	{0xB201, 0x01},
	{0xB202, 0x28},
	{0xB300, 0x65},
	{0xB301, 0x01},
	{0xB302, 0x28},

	{0xB400, 0x03},
	{0xB401, 0x03},
	{0xB402, 0x03},
	{0xB800, 0xEF},
	{0xBF00, 0x40},

	{0xC000, 0x78},
	{0xC001, 0x78},
	{0xC002, 0x00},
	{0xC003, 0x00},
	{0xC100, 0x46},
	{0xC101, 0x46},
	{0xC102, 0x46},
	{0xC104, 0x02},
	{0xC200, 0x01},
	{0xC202, 0x01},

	{0x2600, 0x10},

	{0xE000, 0x45},
	{0xE001, 0x48},
	{0xE002, 0x51},
	{0xE003, 0x5A},
	{0xE004, 0x17},
	{0xE005, 0x29},
	{0xE006, 0x5A},
	{0xE007, 0x3D},
	{0xE008, 0x1E},
	{0xE009, 0x28},
	{0xE00A, 0x7E},
	{0xE00B, 0x19},
	{0xE00C, 0x3E},
	{0xE00D, 0x4C},
	{0xE00E, 0x66},
	{0xE00F, 0x99},
	{0xE010, 0x30},
	{0xE011, 0x71},

	{0xE100, 0x45},
	{0xE101, 0x48},
	{0xE102, 0x51},
	{0xE103, 0x5A},
	{0xE104, 0x17},
	{0xE105, 0x29},
	{0xE106, 0x5A},
	{0xE107, 0x3D},
	{0xE108, 0x1E},
	{0xE109, 0x28},
	{0xE10A, 0x7E},
	{0xE10B, 0x19},
	{0xE10C, 0x3E},
	{0xE10D, 0x4C},
	{0xE10E, 0x66},
	{0xE10F, 0x99},
	{0xE110, 0x30},
	{0xE111, 0x71},

	{0xE200, 0x21},
	{0xE201, 0x27},
	{0xE202, 0x39},
	{0xE203, 0x49},
	{0xE204, 0x20},
	{0xE205, 0x34},
	{0xE206, 0x61},
	{0xE207, 0x42},
	{0xE208, 0x1D},
	{0xE209, 0x27},
	{0xE20A, 0x82},
	{0xE20B, 0x1C},
	{0xE20C, 0x46},
	{0xE20D, 0x55},
	{0xE20E, 0x64},
	{0xE20F, 0x89},
	{0xE210, 0x2A},
	{0xE211, 0x71},

	{0xE300, 0x21},
	{0xE301, 0x27},
	{0xE302, 0x39},
	{0xE303, 0x49},
	{0xE304, 0x20},
	{0xE305, 0x34},
	{0xE306, 0x61},
	{0xE307, 0x42},
	{0xE308, 0x1D},
	{0xE309, 0x27},
	{0xE30A, 0x82},
	{0xE30B, 0x1C},
	{0xE30C, 0x46},
	{0xE30D, 0x55},
	{0xE30E, 0x64},
	{0xE30F, 0x89},
	{0xE310, 0x2A},
	{0xE311, 0x71},

	{0xE400, 0x40},
	{0xE401, 0x52},
	{0xE402, 0x6A},
	{0xE403, 0x77},
	{0xE404, 0x20},
	{0xE405, 0x33},
	{0xE406, 0x60},
	{0xE407, 0x50},
	{0xE408, 0x1C},
	{0xE409, 0x24},
	{0xE40A, 0x86},
	{0xE40B, 0x0D},
	{0xE40C, 0x29},
	{0xE40D, 0x35},
	{0xE40E, 0xF4},
	{0xE40F, 0xF0},
	{0xE410, 0x70},
	{0xE411, 0x7F},

	{0xE500, 0x40},
	{0xE501, 0x52},
	{0xE502, 0x6A},
	{0xE503, 0x77},
	{0xE504, 0x20},
	{0xE505, 0x33},
	{0xE506, 0x60},
	{0xE507, 0x50},
	{0xE508, 0x1C},
	{0xE509, 0x24},
	{0xE50A, 0x86},
	{0xE50B, 0x0D},
	{0xE50C, 0x29},
	{0xE50D, 0x35},
	{0xE50E, 0xF4},
	{0xE50F, 0xF0},
	{0xE510, 0x70},
	{0xE511, 0x7F},

	{REG_WAIT, 30},
	{0xFE00, 0x08},
	{REG_WAIT, 20},

	{0xCB00, 0x80},
	{0xCB01, 0x02},
	{0x5300, 0x24},
	{0x3500, 0x00},

	{0x2900, 0x00},
};

static struct nov_regs  marvelct_lgd_panel_cmd[] = {
	{REG_WAIT, 60},
	{0xF000, 0x00AA},
	{0xF001, 0x0055},
	{0xF002, 0x0052},

	{0x1100, 0x0},
	{REG_WAIT, 120},

	{0xFD02, 0x05},

	{0xB100, 0x65},
	{0xB101, 0x01},
	{0xB102, 0x28},
	{0xB200, 0x65},
	{0xB201, 0x01},
	{0xB202, 0x28},
	{0xB300, 0x65},
	{0xB301, 0x01},
	{0xB302, 0x28},

	{0xB400, 0x02},
	{0xB401, 0x02},
	{0xB402, 0x02},
	{0xB800, 0x2F},
	{0xBF00, 0x40},

	{0xC000, 0x82},
	{0xC001, 0x82},
	{0xC100, 0x44},
	{0xC101, 0x44},
	{0xC102, 0x44},
	{0xC103, 0xFF},
	{0xC104, 0x03},
	{0xC700, 0x44},

	{0x2600, 0x10},
	{0x3600, 0x08},

	{0xE000, 0x00},
	{0xE001, 0x17},
	{0xE002, 0x3E},
	{0xE003, 0x54},
	{0xE004, 0x1E},
	{0xE005, 0x32},
	{0xE006, 0x62},
	{0xE007, 0x7E},
	{0xE008, 0x1B},
	{0xE009, 0x21},
	{0xE00A, 0xE4},
	{0xE00B, 0x2B},
	{0xE00C, 0x5B},
	{0xE00D, 0x73},
	{0xE00E, 0xE9},
	{0xE00F, 0xF7},
	{0xE010, 0x7E},
	{0xE011, 0x7F},

	{0xE100, 0x00},
	{0xE101, 0x17},
	{0xE102, 0x3E},
	{0xE103, 0x54},
	{0xE104, 0x1E},
	{0xE105, 0x32},
	{0xE106, 0x62},
	{0xE107, 0x7E},
	{0xE108, 0x1B},
	{0xE109, 0x21},
	{0xE10A, 0xE4},
	{0xE10B, 0x2B},
	{0xE10C, 0x5B},
	{0xE10D, 0x73},
	{0xE10E, 0xE9},
	{0xE10F, 0xF7},
	{0xE110, 0x7E},
	{0xE111, 0x7F},

	{0xE200, 0x00},
	{0xE201, 0x17},
	{0xE202, 0x3E},
	{0xE203, 0x54},
	{0xE204, 0x1E},
	{0xE205, 0x32},
	{0xE206, 0x62},
	{0xE207, 0x7E},
	{0xE208, 0x1B},
	{0xE209, 0x21},
	{0xE20A, 0xE4},
	{0xE20B, 0x2B},
	{0xE20C, 0x5B},
	{0xE20D, 0x73},
	{0xE20E, 0xE9},
	{0xE20F, 0xF7},
	{0xE210, 0x7E},
	{0xE211, 0x7F},

	{0xE300, 0x00},
	{0xE301, 0x17},
	{0xE302, 0x3E},
	{0xE303, 0x54},
	{0xE304, 0x1E},
	{0xE305, 0x32},
	{0xE306, 0x62},
	{0xE307, 0x7E},
	{0xE308, 0x1B},
	{0xE309, 0x21},
	{0xE30A, 0xE4},
	{0xE30B, 0x2B},
	{0xE30C, 0x5B},
	{0xE30D, 0x73},
	{0xE30E, 0xE9},
	{0xE30F, 0xF7},
	{0xE310, 0x7E},
	{0xE311, 0x7F},

	{0xE400, 0x00},
	{0xE401, 0x17},
	{0xE402, 0x3E},
	{0xE403, 0x54},
	{0xE404, 0x1E},
	{0xE405, 0x32},
	{0xE406, 0x62},
	{0xE407, 0x7E},
	{0xE408, 0x1B},
	{0xE409, 0x21},
	{0xE40A, 0xE4},
	{0xE40B, 0x2B},
	{0xE40C, 0x5B},
	{0xE40D, 0x73},
	{0xE40E, 0xE9},
	{0xE40F, 0xF7},
	{0xE410, 0x7E},
	{0xE411, 0x7F},

	{0xE500, 0x00},
	{0xE501, 0x17},
	{0xE502, 0x3E},
	{0xE503, 0x54},
	{0xE504, 0x1E},
	{0xE505, 0x32},
	{0xE506, 0x62},
	{0xE507, 0x7E},
	{0xE508, 0x1B},
	{0xE509, 0x21},
	{0xE50A, 0xE4},
	{0xE50B, 0x2B},
	{0xE50C, 0x5B},
	{0xE50D, 0x73},
	{0xE50E, 0xE9},
	{0xE50F, 0xF7},
	{0xE510, 0x7E},
	{0xE511, 0x7F},

	{REG_WAIT, 30},
	{0xFE00, 0x08},
	{REG_WAIT, 20},

	{0xCB00, 0x80},
	{0xCB01, 0x02},
	{0x5300, 0x24},
	{0x3500, 0x00},

	{0x2900, 0x0},
};

enum led_brightness marvelct_brightness_value = DEFAULT_BRIGHTNESS;

static int
marvelct_panel_shrink(int brightness)
{
	int panel_id = -1;

	panel_id = gpio_get_value(MARVELCT_GPIO_LCD_ID0) |
		(gpio_get_value(MARVELCT_GPIO_LCD_ID1) << 1);

	switch (panel_id) {
	case 0:
		if (brightness <= 142) {
			if (brightness == 0)
				return  0;
			else if (brightness <= 30)
				brightness = 10;
			else
				brightness = 91 * (brightness - 30) / 112 + 10;
		} else
			brightness = 93 * (brightness - 142) / 113 + 101;
	break;

	case 1:
		if (brightness <= 142) {
			if (brightness == 0)
				return  0;
			else if (brightness <= 30)
				brightness = 10;
			else
				brightness = 124 * (brightness - 30) / 112 + 10;
		} else
			brightness = 121 * (brightness - 142) / 113 + 134;
	break;

	default:
		return -1;
	break;
	}

	return brightness;
}

static void marvelct_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness val)
{
	struct msm_mddi_client_data *client = novatek.client_data;
	unsigned int shrink_bl = marvelct_panel_shrink(val);

	if(atomic_read(&lcm_init_done) == 0)
	{
		return;
	}

	mutex_lock(&novatek.lock);
	client->remote_write(client, shrink_bl, 0x5100);

	mutex_unlock(&novatek.lock);
	marvelct_brightness_value = val;
}

static enum led_brightness
marvelct_brightness_get(struct led_classdev *led_cdev)
{
	return marvelct_brightness_value;
}

static void
marvelct_backlight_switch(struct msm_mddi_client_data *client_data, int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "[BKL] turn on backlight\n");

		val = novatek.lcd_backlight.brightness;
		printk(KERN_DEBUG "[BKL] marvelct_brightness_set = %d\n", val);
		marvelct_brightness_set(&novatek.lcd_backlight, val);
	} else {
		printk(KERN_DEBUG "[BKL] turn off backlight\n");
		printk(KERN_DEBUG "[BKL] marvelct_brightness_set = 0\n");
		marvelct_brightness_set(&novatek.lcd_backlight, 0);
	}
}

static int
marvelct_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size;
	int panel_id=-1;
	unsigned reg, val;
	struct nov_regs *init_seq;

	panel_id = gpio_get_value(MARVELCT_GPIO_LCD_ID0) |
	           (gpio_get_value(MARVELCT_GPIO_LCD_ID1) << 1);

	switch(panel_id) {
		case 0:
			init_seq = marvelct_lgd_panel_cmd;
			array_size = ARRAY_SIZE(marvelct_lgd_panel_cmd);

		break;

		case 1:
			init_seq = marvelct_auo_panel_cmd;
			array_size = ARRAY_SIZE(marvelct_auo_panel_cmd);

		break;

		default:
			return -1;
		break;
	}

	for (i = 0; i < array_size; i++) {
		reg = cpu_to_le32(init_seq[i].reg);
		val = cpu_to_le32(init_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
	}

	return 0;
}

static int
marvelct_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	return 0;
}

static int
marvelct_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	atomic_set(&lcm_init_done, 1);
	mdelay(50);
	marvelct_backlight_switch(client_data, LED_FULL);

	return 0;
}

static int
marvelct_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	atomic_set(&lcm_init_done, 0);
	marvelct_backlight_switch(client_data, LED_OFF);

	return 0;
}

static int marvelct_panel_shutdown(
                struct msm_mddi_bridge_platform_data *bridge_data,
                struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	marvelct_mddi_novatek_power(client_data, 0);
	return 0;
}

static void panel_novatek_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	*mfr_name = 0xb9f6;
	*product_code = 0x5410;
}

static u8 pwm_novatek[10] = {8, 16, 34, 61, 96, 138, 167, 195, 227, 255};

static struct msm_mddi_bridge_platform_data novatek_client_data = {
	.init = marvelct_mddi_init,
	.uninit = marvelct_mddi_uninit,
	.blank = marvelct_panel_blank,
	.unblank = marvelct_panel_unblank,
	.shutdown = marvelct_panel_shutdown,
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

static struct msm_mdp_platform_data marvelct_mdp_pdata = {
	.color_format = MSM_MDP_OUT_IF_FMT_RGB888,
};

static struct msm_mddi_platform_data marvelct_pdata = {
	.clk_rate = 160000000,
	.power_client = marvelct_mddi_novatek_power,
	.fixup = panel_novatek_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = 0xb9f65410,
			.name = "mddi_c_b9f6_5410",
			.id = 1,
			.client_data = &novatek_client_data,
			.clk_rate = 0,
		},
	},
};

static int marvelct_backlight_probe(struct platform_device *pdev)
{
	mutex_init(&novatek.lock);
	novatek.client_data = pdev->dev.platform_data;
	novatek.lcd_backlight.name = "lcd-backlight";
	novatek.lcd_backlight.brightness_set = marvelct_brightness_set;
	novatek.lcd_backlight.brightness_get = marvelct_brightness_get;
	led_classdev_register(&pdev->dev, &novatek.lcd_backlight);

	return 0;
}
static struct platform_driver marvelct_backlight_pdrv = {
	.probe          = marvelct_backlight_probe,
	.driver         = {
		.name   = "marvelct-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init marvelct_backlight_init(void)
{
	return platform_driver_register(&marvelct_backlight_pdrv);
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
int __init marvelct_init_panel(void)
{
	int rc;
	int panel_type = 0;
	int panel_id = -1;
	int gpio_lcd_id0, gpio_lcd_id1;
	uint32_t config;
	struct panel_data *panel_data = &novatek_client_data.panel_conf;
	struct msm_mddi_client_data *client = novatek.client_data;

	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcm_2v6 = vreg_get(0, "gp1");
	if (IS_ERR(vreg_lcm_2v6))
		return PTR_ERR(vreg_lcm_2v6);

	vreg_lcm_2v85 = vreg_get(0, "rftx");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	gpio_lcd_id0 =  PCOM_GPIO_CFG(MARVELCT_GPIO_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	gpio_lcd_id1 =  PCOM_GPIO_CFG(MARVELCT_GPIO_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id0, 0);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id1, 0);

	panel_id = gpio_get_value(MARVELCT_GPIO_LCD_ID0) |
	           (gpio_get_value(MARVELCT_GPIO_LCD_ID1) << 1);

	B(KERN_INFO "%s: panel_id:%d.\n", __func__,panel_id);

	switch(panel_id) {
		case 0:
			panel_type = 10;
		break;

		case 1:
			panel_type = 13;
		break;

		default:
			marvelct_mddi_novatek_power(client, 0);
			gpio_lcd_id0 =  PCOM_GPIO_CFG(MARVELCT_GPIO_LCD_ID0, 0,
						GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
			gpio_lcd_id1 =  PCOM_GPIO_CFG(MARVELCT_GPIO_LCD_ID1, 0,
						GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
			msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id0, 0);
			msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &gpio_lcd_id1, 0);
		break;
	}

	panel_data->panel_id = panel_type;
	panel_data->caps = MSMFB_CAP_CABC;
	panel_data->pwm = pwm_novatek;
	panel_data->shrink = 1;
	panel_data->shrink_br = marvelct_panel_shrink;
	panel_data->default_br = 83;

	config = PCOM_GPIO_CFG(MARVELCT_GPIO_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

	msm_device_mdp.dev.platform_data = &marvelct_mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &marvelct_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	led_trigger_register_simple("lcd-backlight-gate", &marvelct_lcd_backlight);
	if (IS_ERR(marvelct_lcd_backlight))
		printk(KERN_ERR "[BKL] %s: backlight registration failed!\n",
			__func__);
	return 0;
}
module_init(marvelct_backlight_init);
