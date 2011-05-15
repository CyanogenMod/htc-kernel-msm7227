/* arch/arm/mach-msm/board-icong.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/cy8c_tma_ts.h>
#include <linux/akm8975.h>
#include <linux/bma150.h>
#include <linux/cm3628.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/msm_flashlight.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>
#include <asm/mach/mmc.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/vreg.h>
/* #include <mach/gpio_chip.h> */
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>
#include <mach/atmega_microp.h>
#include <mach/htc_battery.h>
#include <linux/tps65200.h>
#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>
#include <mach/camera.h>
#include <mach/msm_serial_debugger.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/usbdiag.h>
#include <mach/htc_usb.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>
#include <mach/socinfo.h>

#include "devices.h"
#include "board-icong.h"
#include "proc_comm.h"
#include "../../../drivers/staging/android/timed_gpio.h"

static unsigned engineerid;

void msm_init_irq(void);
void msm_init_gpio(void);
void config_icong_camera_on_gpios(void);
void config_icong_camera_off_gpios(void);


#ifdef CONFIG_MICROP_COMMON
void __init icong_microp_init(void);
#endif

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= ICONG_GPIO_35MM_HEADSET_DET,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_MICROP Driver */
static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.remote_int		= 1 << 13,
	.remote_irq		= MSM_uP_TO_INT(13),
	.remote_enable_pin	= 0,
	.adc_channel		= 0x01,
	.adc_remote		= {0, 33, 38, 82, 95, 167},
};

static struct htc_headset_microp_platform_data htc_headset_microp_data_xc = {
	.remote_int		= 1 << 13,
	.remote_irq		= MSM_uP_TO_INT(13),
	.remote_enable_pin	= 0,
	.adc_channel		= 0x01,
	.adc_remote 	= {0, 33, 38, 102, 133, 204},
};

static struct platform_device htc_headset_microp = {
	.name	= "HTC_HEADSET_MICROP",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_microp_data,
	},
};

static struct platform_device htc_headset_microp_xc = {
	.name	= "HTC_HEADSET_MICROP",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_microp_data_xc,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_microp,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct platform_device *headset_devices_xc[] = {
	&htc_headset_microp_xc,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_MODEM,
	.charger = SWITCH_CHARGER_TPS65200,
	.m2a_cable_detect = 1,
	.int_data.chg_int = MSM_GPIO_TO_INT(ICONG_GPIO_CHG_INT),
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};


static struct microp_function_config microp_functions[] = {
	{
		.name   = "microp_intrrupt",
		.category = MICROP_FUNCTION_INTR,
	},
	{
		.name   = "reset-int",
		.category = MICROP_FUNCTION_RESET_INT,
		.int_pin = 1 << 8,
	},
};

static struct microp_led_config led_config[] = {
	{
		.name   = "amber",
		.type = LED_RGB,
	},
	{
		.name   = "green",
		.type = LED_RGB,
	},
	{
		.name = "button-backlight",
		.type = LED_PWM,
		.led_pin = 1 << 5,
		.init_value = 0x0A,
		.fade_time = 5,
	},
	{
		.name = "sharekey",
		.type = LED_SKEY,
		.led_pin = 1 << 1,
		.init_value = 0xFF,
		.fade_time = 5,
	},
};

static void microp_special_led_setting(void)
{
	uint8_t data[3] = {0x19, 0x19, 0xA0};
	microp_i2c_write(MICROP_I2C_WCMD_FACEBOOK_LED_TIMESET, data, 3);
}

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(led_config),
	.led_config	= led_config,
	.special_led_setting	= microp_special_led_setting,
};


static struct bma150_platform_data icong_g_sensor_pdata = {
	.microp_new_cmd = 1,
	.chip_layout = 0,
};

static struct platform_device microp_devices[] = {
	{
		.name		= "leds-microp",
		.id		= -1,
		.dev		= {
			.platform_data	= &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &icong_g_sensor_pdata,
		},
	},
	{
		.name	= "HTC_HEADSET_MGR",
		.id	= -1,
		.dev	= {
			.platform_data	= &htc_headset_mgr_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions   = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = ICONG_GPIO_UP_RESET_N,
	.spi_devices = SPI_GSENSOR,
};

static int icong_ts_cy8c_power(int on)
{
	printk(KERN_INFO "%s():\n", __func__);
	if (on) {
		gpio_set_value(ICONG_V_TP_3V3_EN, 1);
		msleep(2);
		gpio_set_value(ICONG_GPIO_TP_RST_N, 0);
		msleep(2);
		gpio_set_value(ICONG_GPIO_TP_RST_N, 1);
	}
	return 0;
}

static int icong_ts_cy8c_wake(void)
{
	printk(KERN_INFO "%s():\n", __func__);
	gpio_direction_output(ICONG_GPIO_TP_ATT_N, 0);
	udelay(500);
	gpio_direction_input(ICONG_GPIO_TP_ATT_N);

	return 0;
}

static int icong_ts_cy8c_reset(void)
{
	printk(KERN_INFO "%s():\n", __func__);
	gpio_direction_output(ICONG_GPIO_TP_RST_N,0);
	mdelay(100);
	gpio_direction_output(ICONG_GPIO_TP_RST_N,1);
	return 0;
}


struct cy8c_i2c_platform_data icong_ts_cy8c_data[] = {
	{
		.version = 0x09,
		.abs_x_min = 16,
		.abs_x_max = 1007,
		.abs_y_min = 10,
		.abs_y_max = 949,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 512,
		.power = icong_ts_cy8c_power,
		.wake = icong_ts_cy8c_wake,
		.gpio_irq = ICONG_GPIO_TP_ATT_N,
	},
	{
		.version = 0x00,
		.abs_x_min = 16,
		.abs_x_max = 1007,
		.abs_y_min = 10,
		.abs_y_max = 949,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 512,
		.power = icong_ts_cy8c_power,
		.gpio_irq = ICONG_GPIO_TP_ATT_N,
	},
};

static int icong_phy_init_seq[] =
{
	0x2C, 0x31,
	0x08, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1
};

static void icong_phy_reset(void)
{
	int ret;
	printk(KERN_INFO "msm_hsusb_phy_reset\n");
	ret = msm_proc_comm(PCOM_MSM_HSUSB_PHY_RESET,
			NULL, NULL);
	if (ret)
		printk(KERN_INFO "%s failed\n", __func__);
}

static void icong_disable_usb_charger(void)
{
	printk(KERN_INFO "%s\n", __func__);
	htc_battery_charger_disable();
}

#ifdef CONFIG_USB_ANDROID

static uint32_t usb_ID_PIN_ouput_table[] = {
	PCOM_GPIO_CFG(ICONG_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t usb_ID_PIN_input_table[] = {
	PCOM_GPIO_CFG(ICONG_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
};

void config_icong_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table,
			ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(ICONG_GPIO_USB_ID_PIN, 1);
	} else
		config_gpio_table(usb_ID_PIN_input_table,
			ARRAY_SIZE(usb_ID_PIN_input_table));
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= icong_phy_init_seq,
	.phy_reset		= icong_phy_reset,
	.usb_id_pin_gpio	= ICONG_GPIO_USB_ID_PIN,
	.disable_usb_charger	= icong_disable_usb_charger,
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0cb6,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static struct akm8975_platform_data compass_platform_data = {
	.layouts = ICONG_LAYOUTS,
};

static int __capella_cm3628_power(int on)
{
	int rc;
	struct vreg *vreg = vreg_get(0, "wlan");

	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	rc = vreg_set_level(vreg, 2800);

	printk(KERN_DEBUG "%s: Turn the capella_cm3628 power %s\n",
		__func__, (on) ? "on" : "off");

	if (on) {
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
	}

	return rc;
}

static DEFINE_MUTEX(capella_cm3628_lock);
static int als_power_control;

static int capella_cm3628_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3628_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3628_power(1);
	else if (!on)
		ret = __capella_cm3628_power(0);

	mutex_unlock(&capella_cm3628_lock);
	return ret;
}

static struct cm3628_platform_data cm3628_pdata = {
	.intr = ICONG_GPIO_PROXIMITY_INT,
	.levels = { 1, 3, 5, 88, 175, 1380,
			2390, 2919, 3448, 0xFFFF},
	.golden_adc = 0x6BC,
	.power = capella_cm3628_power,
	.ALS_slave_address = 0xC0>>1,
	.PS_slave_address = 0xC2>>1,
	.check_interrupt_add = 0x2C>>1	,
	.is_cmd = CM3628_ALS_IT_400ms | CM3628_ALS_PERS_4,
	.ps_thd_set = 0x3,
	.ps_thd_no_cal = 0x7,
	.ps_thd_with_cal = 0x3,
	.ps_adc_offset = 0x3,
	.ps_calibration_rule = 1,/*sync saga calibration rule*/
	.ps_conf2_val = 0,
	.ps_conf1_val = CM3628_PS_DR_1_320 |CM3628_PS_IT_1_6T,
};

static struct tps65200_platform_data tps65200_data = {
	.charger_check = 0,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(CYPRESS_TMA_NAME, 0x67),
		.platform_data = &icong_ts_cy8c_data,
		.irq = MSM_GPIO_TO_INT(ICONG_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = ICONG_GPIO_TO_INT(ICONG_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
		.platform_data = &tps65200_data,
	},
	{
		I2C_BOARD_INFO(CM3628_I2C_NAME, 0x30 >> 1),
		.platform_data = &cm3628_pdata,
		.irq = MSM_GPIO_TO_INT(ICONG_GPIO_PROXIMITY_INT),
	},
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data,
		.irq = ICONG_GPIO_TO_INT(ICONG_GPIO_COMPASS_RDY),
	},
};

static struct pwr_sink icong_pwrsink_table[] = {
	{
		.id     = PWRSINK_AUDIO,
		.ua_max = 100000,
	},
	{
		.id     = PWRSINK_BACKLIGHT,
		.ua_max = 125000,
	},
	{
		.id     = PWRSINK_LED_BUTTON,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_LED_KEYBOARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_GP_CLK,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_BLUETOOTH,
		.ua_max = 15000,
	},
	{
		.id     = PWRSINK_CAMERA,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_SDCARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_VIDEO,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id     = PWRSINK_SYSTEM_LOAD,
		.ua_max = 100000,
		.percent_util = 38,
	},
};

static int icong_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void icong_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void icong_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int icong_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data icong_pwrsink_data = {
	.num_sinks      = ARRAY_SIZE(icong_pwrsink_table),
	.sinks          = icong_pwrsink_table,
	.suspend_late	= icong_pwrsink_suspend_late,
	.resume_early	= icong_pwrsink_resume_early,
	.suspend_early	= icong_pwrsink_suspend_early,
	.resume_late	= icong_pwrsink_resume_late,
};

static struct platform_device icong_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev    = {
		.platform_data = &icong_pwrsink_data,
	},
};

static struct msm_pmem_setting pmem_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
        .kgsl_start = MSM_GPU_MEM_BASE,
        .kgsl_size = MSM_GPU_MEM_SIZE,
};


#ifdef CONFIG_MSM_CAMERA

static int camera_power_on_init(void)
{
	int rc=0;

//	printk(KERN_INFO "%s():\n", __func__);

	gpio_request(ICONG_GPIO_VCM_PD, "cam_pwr_on");
	gpio_direction_output(ICONG_GPIO_VCM_PD, 0);
	gpio_free(ICONG_GPIO_VCM_PD);

	gpio_request(ICONG_GPIO_CAM2_RST_N, "cam_pwr_on");
	gpio_direction_output(ICONG_GPIO_CAM2_RST_N, 0);
	gpio_free(ICONG_GPIO_CAM2_RST_N);

	return rc;
}

static int flashlight_control(int mode)
{
	return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

static struct resource msm_camera_resources[] = {
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_icong_camera_on_gpios,
	.camera_gpio_off = config_icong_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};


static void glacier_s5k4e1gx_clk_switch(void){
	int rc = 0;
	pr_info("[CAM]doing clk switch (s5k4e1gx)\n");
	rc = gpio_request(ICONG_GPIO_CAM_SEL, "s5k4e1gx");
	if (rc < 0)
		pr_err("[CAM]GPIO (%d) request fail\n", ICONG_GPIO_CAM_SEL);
	else
		gpio_direction_output(ICONG_GPIO_CAM_SEL, 0);
	gpio_free(ICONG_GPIO_CAM_SEL);

	return;
}

static void glacier_mt9v113_clk_switch(void){
	int rc = 0;
	pr_info("[CAM]doing clk switch (mt9v113)\n");
	rc = gpio_request(ICONG_GPIO_CAM_SEL, "mt9v113");
	if (rc < 0)
		pr_err("[CAM]GPIO (%d) request fail\n", ICONG_GPIO_CAM_SEL);
	else
		gpio_direction_output(ICONG_GPIO_CAM_SEL, 1);
	gpio_free(ICONG_GPIO_CAM_SEL);

	return;
}


static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "s5k4e1gx",
	.sensor_reset   = ICONG_GPIO_CAM_RST_N,
	.vcm_pwd        = ICONG_GPIO_VCM_PD,
	.camera_clk_switch	= glacier_s5k4e1gx_clk_switch,
	.camera_power_on = camera_power_on_init,
	.pdata          = &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED,
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name      = "msm_camera_s5k4e1gx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};



static struct msm_camera_sensor_info msm_camera_sensor_mt9v113_data = {
	.sensor_name	= "mt9v113",
	.sensor_reset	= ICONG_GPIO_CAM2_RST_N,
	.vcm_pwd		= ICONG_GPIO_CAM2_PWDN,
	.camera_clk_switch	= glacier_mt9v113_clk_switch,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_NONE,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.cam_select_pin = ICONG_GPIO_CAM_SEL,
	.mirror_mode = true, /* for sensor upside down */
};

static struct platform_device msm_camera_sensor_mt9v113 = {
	.name	   = "msm_camera_mt9v113",
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9v113_data,
	},
};


#endif

static struct platform_device icong_rfkill = {
	.name = "icong_rfkill",
	.id = -1,
};

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 384000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_4MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	if(tp_type == 1)
		msm_i2c_pdata.reset_slave = icong_ts_cy8c_reset;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static uint32_t fl_gpio_table[] = {
	PCOM_GPIO_CFG(ICONG_GPIO_FLASHLIGHT_TORCH, 0,
					GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(ICONG_GPIO_FLASHLIGHT_FLASH, 0,
					GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

};

static void config_icong_flashlight_gpios(void)
{
	config_gpio_table(fl_gpio_table, ARRAY_SIZE(fl_gpio_table));
}

static struct flashlight_platform_data icong_flashlight_data = {
	.gpio_init		= config_icong_flashlight_gpios,
	.torch			= ICONG_GPIO_FLASHLIGHT_TORCH,
	.flash			= ICONG_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms	= 600,
	.led_count 		= 0,
	.chip_model 		= AAT1271,
};

static struct platform_device icong_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev		= {
		.platform_data	= &icong_flashlight_data,
	},
};

static struct platform_device *devices[] __initdata = {
	&msm_device_i2c,
	&htc_battery_pdev,
	&msm_camera_sensor_s5k4e1gx,
	&icong_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&icong_pwr_sink,
#endif
	&msm_camera_sensor_mt9v113, /* 2nd CAM */
	&icong_flashlight_device,
};

extern struct sys_timer msm_timer;

static void __init icong_init_irq(void)
{
	printk("icong_init_irq()\n");
	msm_init_irq();
}

static uint cpld_iset;
static uint cpld_charger_en;
static uint cpld_usb_h2w_sw;
static uint opt_disable_uart3;
static char *keycaps = "";

module_param_named(iset, cpld_iset, uint, 0);
module_param_named(charger_en, cpld_charger_en, uint, 0);
module_param_named(usb_h2w_sw, cpld_usb_h2w_sw, uint, 0);
module_param_named(disable_uart3, opt_disable_uart3, uint, 0);
module_param_named(keycaps, keycaps, charp, 0);

static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void icong_reset(void)
{
	gpio_set_value(ICONG_GPIO_MSM_PS_HOLD, 0);
}

static struct i2c_board_info i2c_camera_devices[] = {
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x20 >> 1),/*5M samsung bayer sensor driver*/
	},
	{
		I2C_BOARD_INFO("mt9v113", 0x3C), /* 0x78: w, 0x79 :r */
	},
};

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */

	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* MCLK */

/* PCOM_GPIO_CFG(27, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),//CAM_I2C_SDA */
/* PCOM_GPIO_CFG(49, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),//CAM_I2C_SCL */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

void config_icong_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_icong_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

/* for bcm */
static char bdaddress[20];
extern unsigned char *get_bt_bd_ram(void);

static void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x",
		cTemp[0], cTemp[1], cTemp[2], cTemp[3], cTemp[4], cTemp[5]);
	printk(KERN_INFO "YoYo--BD_ADDRESS=%s\n", bdaddress);
}

module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");

static uint32_t icong_serial_debug_table[] = {
	/* config as serial debug uart */
	PCOM_GPIO_CFG(ICONG_GPIO_UART3_RX, 1,
			GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* UART3 RX */
	PCOM_GPIO_CFG(ICONG_GPIO_UART3_TX, 1,
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* UART3 TX */
};

static void icong_config_serial_debug_gpios(void)
{
	config_gpio_table(icong_serial_debug_table,
			ARRAY_SIZE(icong_serial_debug_table));
}

static void __init config_gpios(void)
{
	icong_config_serial_debug_gpios();
	config_icong_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data icong_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 200000,
};

static unsigned icong_perf_acpu_table[] = {
	245760000,
	480000000,
	600000000,
};
/* 7x27 turbo CPU table support */
static struct msm_acpu_clock_platform_data icong_turbo_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 400000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 200000,
};
static unsigned icong_turbo_perf_acpu_table[] = {
	245760000,
	480000000,
	800000000,
};

static struct perflock_platform_data icong_perflock_data = {
	.perf_acpu_table = icong_perf_acpu_table,
	.table_size = ARRAY_SIZE(icong_perf_acpu_table),
};
static struct perflock_platform_data icong_turbo_perflock_data = {
	.perf_acpu_table = icong_turbo_perf_acpu_table,
	.table_size = ARRAY_SIZE(icong_turbo_perf_acpu_table),
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(ICONG_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = ICONG_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = ICONG_GPIO_BT_HOST_WAKE,

};
#endif

static ssize_t icong_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)	    ":40:510:60:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":120:510:60:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":200:510:60:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":280:510:60:60"
		"\n");
}

static struct kobj_attribute icong_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.cy8c-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &icong_virtual_keys_show,
};

static struct attribute *icong_properties_attrs[] = {
	&icong_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group icong_properties_attr_group = {
	.attrs = icong_properties_attrs,
};

static void __init icong_init(void)
{
	int rc;
	char *cid = NULL;
	struct kobject *properties_kobj;

	printk(KERN_INFO "icong_init() revision = 0x%X, engineerid = 0x%X\n", system_rev, engineerid);
	msm_clock_init();
	board_get_cid_tag(&cid);

	/* for bcm */
	bt_export_bd_address();

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	/* We need to set this pin to 0 only once on power-up; we will
	 * not actually enable the chip until we apply power to it via
	 * vreg.
	 */
	gpio_request(ICONG_GPIO_LS_EN, "ls_en");
	gpio_direction_output(ICONG_GPIO_LS_EN, 0);


	msm_hw_reset_hook = icong_reset;

	if (socinfo_init() < 0)
		BUG();

	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) > 1)
		|| ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1)
		&& (SOCINFO_VERSION_MINOR(socinfo_get_version()) >= 3)))
		{
			msm_acpu_clock_init(&icong_turbo_clock_data);
			perflock_init(&icong_turbo_perflock_data);
		}else{
			msm_acpu_clock_init(&icong_clock_data);
			perflock_init(&icong_perflock_data);
		}

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(ICONG_GPIO_UART3_RX));
#endif

	msm_add_devices();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_uart_dm1.name = "msm_serial_hs_bcm";	/* for bcm */
	msm_add_serial_devices(3);
#else
	msm_add_serial_devices(0);
#endif

	msm_add_serial_devices(2);
	/*
	msm_change_usb_id(0x0bb4, 0x0c10);
	*/
#ifdef CONFIG_USB_FUNCTION
	msm_add_usb_id_pin_gpio(ICONG_GPIO_USB_ID_PIN);
	msm_add_usb_devices(icong_phy_reset, NULL);
#endif

#ifdef CONFIG_USB_ANDROID
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	config_icong_usb_id_gpios(0);
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
#endif
	msm_add_mem_devices(&pmem_setting);

#ifdef CONFIG_MICROP_COMMON
	icong_microp_init();
#endif

	rc = icong_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
						&icong_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	/* probe camera driver */
	i2c_register_board_info(0, i2c_camera_devices, ARRAY_SIZE(i2c_camera_devices));

	/* Update XB03, XC headset ADC table for iphone headset*/
	if (engineerid == 0x0E) {
		htc_headset_mgr_data.headset_devices_num = ARRAY_SIZE(
							   headset_devices_xc);
		htc_headset_mgr_data.headset_devices = headset_devices_xc;
	}

	msm_device_i2c_init();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	icong_init_panel();

	icong_init_keypad();

	icong_wifi_init();

	msm_init_pmic_vibrator(3000);
}

static void __init icong_fixup(struct machine_desc *desc, struct tag *tags,
                               char **cmdline, struct meminfo *mi)
{
	engineerid = parse_tag_engineerid(tags);

	mi->nr_banks=1;
	mi->bank[0].start = MSM_LINUX_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE);
	mi->bank[0].size = MSM_LINUX_SIZE;
}

static void __init icong_map_io(void)
{
	printk("icong_init_map_io()\n");
	msm_map_common_io();
	if (socinfo_init() < 0)
		BUG();
#ifdef CONFIG_CACHE_L2X0
	/* 7x27 has 256KB L2 cache:
	64Kb/Way and 4-Way Associativity;
	evmon/parity/share disabled. */
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) > 1)
		|| ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1)
		&& (SOCINFO_VERSION_MINOR(socinfo_get_version()) >= 3)))
		{
			/* R/W latency: 4 cycles; */
			l2x0_init(MSM_L2CC_BASE, 0x0006801B, 0xfe000000);
			printk("Icong_init L2-cache latency 4 cyc for 7x27-t\n");
		}else{
		/* R/W latency: 3 cycles; */
	l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
			printk("Icong_init L2-cache latency 3 cyc for 7x27\n");
		}
#endif
}

MACHINE_START(ICONG, "icong")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x12C00100,
	.fixup          = icong_fixup,
	.map_io         = icong_map_io,
	.init_irq       = icong_init_irq,
	.init_machine   = icong_init,
	.timer          = &msm_timer,
MACHINE_END
