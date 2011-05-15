/* arch/arm/mach-msm/board-legend.c
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
#include <linux/synaptics_i2c_rmi.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>
#include <linux/curcial_oj.h>
#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/delay.h>
#include <linux/atmel_qt602240.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>
#include <asm/mach/mmc.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <mach/system.h>
#include <mach/vreg.h>
/* #include <mach/gpio_chip.h> */
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>
#include <mach/atmega_microp.h>
/* #include <mach/msm_tssc.h> */
#include <mach/htc_battery.h>
#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>
#include <mach/camera.h>
#include <mach/msm_flashlight.h>
#include <mach/msm_serial_debugger.h>
#include <mach/hardware.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/htc_usb.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>

#include "devices.h"
#include "board-legend.h"
#include "proc_comm.h"

void msm_init_irq(void);
void msm_init_gpio(void);
void config_legend_camera_on_gpios(void);
void config_legend_camera_off_gpios(void);
#ifdef CONFIG_MICROP_COMMON
void __init legend_microp_init(void);
#endif
void config_legend_proximity_gpios(int on);
static int legend_phy_init_seq[] = {0x20, 0x31, 0x1, 0x0D, 0x1, 0x10, -1};

static void legend_phy_reset(void)
{
	int ret;
	printk(KERN_INFO "%s\n", __func__);
	ret = msm_proc_comm(PCOM_MSM_HSUSB_PHY_RESET,
			NULL, NULL);
	if (ret)
		printk(KERN_INFO "%s failed\n", __func__);
}

#ifdef CONFIG_USB_ANDROID
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= legend_phy_init_seq,
	.phy_reset		= legend_phy_reset,
	.usb_id_pin_gpio =  LEGEND_GPIO_USB_ID_PIN,
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
	.product_id	= 0x0c97,
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

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= LEGEND_GPIO_35MM_HEADSET_DET,
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
	.remote_int		= 1 << 5,
	.remote_irq		= MSM_uP_TO_INT(5),
	.remote_enable_pin	= 0,
	.adc_channel		= 0x01,
	.adc_remote		= {0, 33, 38, 82, 95, 167},
};

static struct platform_device htc_headset_microp = {
	.name	= "HTC_HEADSET_MICROP",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_microp_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_microp,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_MODEM,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

static int capella_cm3602_power(int pwr_device, uint8_t enable);

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
	{
		.name   = "oj",
		.category = MICROP_FUNCTION_OJ,
		.int_pin = 1 << 12,
	},
};

static struct microp_function_config microp_lightsensor = {
	.name = "light_sensor",
	.category = MICROP_FUNCTION_LSENSOR,
	.levels = { 0, 0x21, 0x4D, 0xDC, 0x134, 0x18D, 0x1E5, 0x3FF, 0x3FF, 0x3FF },
	.channel = 3,
	.int_pin = 1 << 9,
	.golden_adc = 0xC0,
	.ls_power = capella_cm3602_power,
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_lightsensor,
	.irq = MSM_uP_TO_INT(9),
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
		.name   = "button-backlight",
		.type = LED_GPO,
		.mask_w = {0x00, 0x00, 0x08},
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(led_config),
	.led_config	= led_config,
};

static struct bma150_platform_data legend_g_sensor_pdata = {
	.microp_new_cmd = 1,
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	 {
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &legend_g_sensor_pdata,
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
	.gpio_reset = LEGEND_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_LCM | SPI_GSENSOR,
	.spi_devices_init = SPI_LCM,
};

static struct synaptics_i2c_rmi_platform_data legend_ts_rmi_data[] = {
	{
		.version = 0x0100,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -1 * 0x10000 / 320,
		.inactive_right = -1 * 0x10000 / 320,
		.inactive_top = -1 * 0x10000 / 480,
		.inactive_bottom = -1 * 0x10000 / 480,
		.display_width = 320,
		.display_height = 480,
		.dup_threshold = 10,
	}
};

static struct synaptics_i2c_rmi_platform_data legend_ts_t1021_data[] = {
	{
		.version = 0x0100,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -10 * 0x10000 / 1808,
		.inactive_right = -10 * 0x10000 / 1808,
		.inactive_top = -20 * 0x10000 / 2707,
		.inactive_bottom = -20 * 0x10000 / 2707,
		.snap_left_on = 10 * 0x10000 / 1808,
		.snap_left_off = 20 * 0x10000 / 1808,
		.snap_right_on = 10 * 0x10000 / 1808,
		.snap_right_off = 20 * 0x10000 / 1808,
		.snap_top_on = 20 * 0x10000 / 2707,
		.snap_top_off = 30 * 0x10000 / 2707,
		.snap_bottom_on = 20 * 0x10000 / 2707,
		.snap_bottom_off = 30 * 0x10000 / 2707,
	}
};

static int legend_ts_atmel_power(int on)
{
	printk(KERN_INFO "%s():\n", __func__);
	if (on) {
		gpio_set_value(LEGEND_TP_5V_EN, 1);
		msleep(2);
		gpio_set_value(LEGEND_GPIO_TP_RST, 1);
	} else {
		gpio_set_value(LEGEND_TP_5V_EN, 0);
		msleep(2);
	}
	return 0;
}

struct atmel_i2c_platform_data legend_ts_atmel_data[] = {
	{
		.version = 0x016,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1023,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = LEGEND_GPIO_TP_ATT_N,
		.power = legend_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {7, 0, 10, 10, 0, 0, 10, 15},
		.config_T9 = {139, 0, 0, 14, 10, 0, 16, 40, 3, 5, 10, 10, 5, 15, 2, 10, 20, 0, 0, 0, 0, 0, 254, 0, 30, 34, 152, 60, 142, 89, 40},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {7, 0, 0, 0, 0, 0, 0, 35, 20, 4, 15, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 16, 0, 1, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8, 30},
		.object_crc = {0x86, 0xE4, 0x42},
		.cable_config = {30, 30, 8, 16},
	},
	{
		.version = 0x0015,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1023,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = LEGEND_GPIO_TP_ATT_N,
		.power = legend_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 20, 10, 0, 0, 5, 25},
		.config_T9 = {139, 0, 0, 14, 10, 0, 16, 35, 2, 5, 0, 2, 2, 14, 2, 10, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 153, 60, 152, 89},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {19, 0, 0, 5, 5, 0, 0, 35, 20, 4, 15, 0},
		.config_T22 = {13, 0, 0, 25, 0, 231, 255, 4, 20, 0, 1, 10, 15, 20, 255, 255, 4},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 248, 42, 88, 27, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8, 60},
		.object_crc = {0x50, 0x8A, 0xD1},
	},
	{
		.version = 0x0014,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1023,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = LEGEND_GPIO_TP_ATT_N,
		.power = legend_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 20, 20, 0, 0, 10, 15},
		.config_T9 = {139, 0, 0, 14, 10, 0, 48, 35, 2, 5, 0, 1, 1, 0, 2, 10, 25, 0, 0, 0, 0, 0, 5, 10, 30, 30, 153, 60, 152, 89},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {5, 0, 0, 25, 0, -25, 255, 4, 30, 0, 1, 10, 15, 20, 10, 10, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8},
		.object_crc = {0x59, 0xB6, 0x5E},
	}
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = LEGEND_LAYOUTS,
	.project_name = LEGEND_PROJECT_NAME,
	.reset = LEGEND_GPIO_COMPASS_RST_N,
	.intr = LEGEND_GPIO_COMPASS_INT_N,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.platform_data = &legend_ts_rmi_data,
		.irq = LEGEND_GPIO_TO_INT(LEGEND_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(SYNAPTICS_T1021_NAME, 0x21),
		.platform_data = &legend_ts_t1021_data,
		.irq = LEGEND_GPIO_TO_INT(LEGEND_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(ATMEL_QT602240_NAME, 0x94 >> 1),
		.platform_data = &legend_ts_atmel_data,
		.irq = MSM_GPIO_TO_INT(LEGEND_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = LEGEND_GPIO_TO_INT(LEGEND_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = LEGEND_GPIO_TO_INT(LEGEND_GPIO_COMPASS_INT_N),
	},
};

static struct pwr_sink legend_pwrsink_table[] = {
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

static int legend_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void legend_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void legend_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int legend_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data legend_pwrsink_data = {
	.num_sinks      = ARRAY_SIZE(legend_pwrsink_table),
	.sinks          = legend_pwrsink_table,
	.suspend_late	= legend_pwrsink_suspend_late,
	.resume_early	= legend_pwrsink_resume_early,
	.suspend_early	= legend_pwrsink_suspend_early,
	.resume_late	= legend_pwrsink_resume_late,
};

static struct platform_device legend_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev    = {
		.platform_data = &legend_pwrsink_data,
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
	.camera_gpio_on  = config_legend_camera_on_gpios,
	.camera_gpio_off = config_legend_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

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

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "s5k4e1gx",
	.sensor_reset   = LEGEND_GPIO_CAM1_RST_N,
	.vcm_pwd        = LEGEND_GPIO_VCM_PWDN,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name      = "msm_camera_s5k4e1gx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};
#endif

static struct platform_device legend_rfkill = {
	.name = "legend_rfkill",
	.id = -1,
};

/* Proximity Sensor (Capella_CM3602)*/
static int __capella_cm3602_power(int on)
{
	int rc;
	struct vreg *vreg = vreg_get(0, "wlan");
	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	rc = vreg_set_level(vreg, 2800);

	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		config_legend_proximity_gpios(1);
		gpio_direction_output(LEGEND_GPIO_PROXIMITY_EN, 1);
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
		gpio_direction_output(LEGEND_GPIO_PROXIMITY_EN, 0);
		config_legend_proximity_gpios(0);
	}

	return rc;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.p_out = LEGEND_GPIO_PROXIMITY_INT,
	.p_en = LEGEND_GPIO_PROXIMITY_EN,
	.power = capella_cm3602_power,
	.irq = MSM_GPIO_TO_INT(LEGEND_GPIO_PROXIMITY_INT),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};
/* End Proximity Sensor (Capella_CM3602)*/
#define CURCIAL_OJ_POWER            85
static void curcial_oj_shutdown (int	enable)
{
	uint8_t	cmd[3];
	memset(cmd, 0x00, sizeof(uint8_t)*3);

	cmd[2] = 0x80;
	if (enable)
		microp_i2c_write(0x90, cmd,	3);
	else
		microp_i2c_write(0x91, cmd,	3);
}
static int curcial_oj_poweron(int on)
{
	gpio_set_value(CURCIAL_OJ_POWER, on);

	if (gpio_get_value(CURCIAL_OJ_POWER) != on) {
		printk(KERN_ERR "%s:OJ:power status fail \n", __func__);
		return 0;
	}
		printk(KERN_ERR "%s:OJ:power status ok \n", __func__);
	return 1;
}
static void curcial_oj_adjust_xy(uint8_t *data, int16_t *mSumDeltaX, int16_t *mSumDeltaY)
{
	int8_t 	deltaX;
	int8_t 	deltaY;


	if (data[2] == 0x80)
		data[2] = 0x81;
	if (data[1] == 0x80)
		data[1] = 0x81;
	if (0) {
		deltaX = (1)*((int8_t) data[2]); /*X=2*/
		deltaY = (1)*((int8_t) data[1]); /*Y=1*/
	} else {
		deltaX = (1)*((int8_t) data[1]);
		deltaY = (1)*((int8_t) data[2]);
	}
	*mSumDeltaX += -((int16_t)deltaX);
	*mSumDeltaY += -((int16_t)deltaY);
}
#define LED_MICROP_VER	0x08

static struct curcial_oj_platform_data legend_oj_data = {
	.oj_poweron = curcial_oj_poweron,
	.oj_shutdown = curcial_oj_shutdown,
	.oj_adjust_xy = curcial_oj_adjust_xy,
	.microp_version = LED_MICROP_VER,
	.mdelay_time = 0,
	.normal_th = 8,
	.xy_ratio = 15,
	.interval = 20,
	.swap = true,
	.x = 1,
	.y = 1,
	.share_power = true,
	.debugflag = 0,
	.ap_code = false,
	.sht_tbl = {0, 1000, 1250, 1500, 1750, 2000, 3000},
	.pxsum_tbl = {0, 0, 90, 100, 110, 120, 130},
	.degree = 7,
	.Xsteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 10, 10, 10, 10, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.Ysteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 10, 10, 10, 10, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.irq = MSM_uP_TO_INT(12),
	.device_id = 0x0D,
};

static struct platform_device legend_oj = {
	.name = CURCIAL_OJ_NAME,
	.id = -1,
	.dev = {
		.platform_data	= &legend_oj_data,
	}
};

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 400000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_4MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct platform_device *devices[] __initdata = {
	&msm_device_i2c,
	&htc_battery_pdev,
	&msm_camera_sensor_s5k4e1gx,
	&legend_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&legend_pwr_sink,
#endif
	&legend_oj,
	&capella_cm3602,
};

extern struct sys_timer msm_timer;

static void __init legend_init_irq(void)
{
	printk("legend_init_irq()\n");
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

static void legend_reset(void)
{
	gpio_set_value(LEGEND_GPIO_PS_HOLD, 0);
}

static uint32_t proximity_on_gpio_table[] = {
	PCOM_GPIO_CFG(21, 0, GPIO_INPUT, GPIO_NO_PULL, 0), /* PS_VOUT */
};

static uint32_t proximity_off_gpio_table[] = {
	PCOM_GPIO_CFG(21, 0, GPIO_INPUT, GPIO_PULL_DOWN, 0) /* PS_VOUT */
};

static struct i2c_board_info i2c_camera_devices[] = {
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x20 >> 1),/*5M samsung bayer sensor driver*/
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

void config_legend_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_legend_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

void config_legend_proximity_gpios(int on)
{
	if (on)
		config_gpio_table(proximity_on_gpio_table,
			ARRAY_SIZE(proximity_on_gpio_table));
	else
		config_gpio_table(proximity_off_gpio_table,
			ARRAY_SIZE(proximity_off_gpio_table));
}

static uint32_t legend_serial_debug_table[] = {
	/* config as serial debug uart */
	PCOM_GPIO_CFG(LEGEND_GPIO_UART3_RX, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA)	/* UART3 RX */,
	PCOM_GPIO_CFG(LEGEND_GPIO_UART3_TX, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* UART3 TX */
};

static void legend_config_serial_debug_gpios(void)
{
	config_gpio_table(legend_serial_debug_table, ARRAY_SIZE(legend_serial_debug_table));
}

static void __init config_gpios(void)
{
	legend_config_serial_debug_gpios();
	config_legend_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data legend_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 200000,
};

static unsigned legend_perf_acpu_table[] = {
	245760000,
	480000000,
	600000000,
};

static struct perflock_platform_data legend_perflock_data = {
	.perf_acpu_table = legend_perf_acpu_table,
	.table_size = ARRAY_SIZE(legend_perf_acpu_table),
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(LEGEND_GPIO_UART1_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
	.cpu_lock_supported = 1,
};
#endif

static void config_legend_flashlight_gpios(void)
{
	uint32_t flashlight_gpio_table[] = {
		PCOM_GPIO_CFG(LEGEND_GPIO_TORCH_EN, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
		PCOM_GPIO_CFG(LEGEND_GPIO_FLASH_EN, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	};
	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data legend_flashlight_data = {
	.gpio_init = config_legend_flashlight_gpios,
	.torch = LEGEND_GPIO_TORCH_EN,
	.flash = LEGEND_GPIO_FLASH_EN,
	.flash_duration_ms = 600,
};

static struct platform_device legend_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev = {
		.platform_data  = &legend_flashlight_data,
	},
};

static void __init legend_init(void)
{
	int rc;
	char *cid = NULL;
	printk("legend_init() revision = 0x%X\n", system_rev);

	/* Must set msm_hw_reset_hook before first proc comm */
	msm_hw_reset_hook = legend_reset;

	board_get_cid_tag(&cid);

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	/* We need to set this pin to 0 only once on power-up; we will
	 * not actually enable the chip until we apply power to it via
	 * vreg.
	 */
	gpio_request(LEGEND_GPIO_LS_EN, "ls_en");
	gpio_direction_output(LEGEND_GPIO_LS_EN, 0);
	/* disable power for cm3602 chip */
	/* __capella_cm3602_power(0); */

	msm_acpu_clock_init(&legend_clock_data);
	perflock_init(&legend_perflock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(LEGEND_GPIO_UART3_RX));
#endif

	msm_add_devices();

	#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_uart_dm1.name = "msm_serial_hs_ti"; /* for ti */
	msm_add_serial_devices(3);
	#else
	msm_add_serial_devices(0);
	#endif

	msm_add_serial_devices(2);

#ifdef CONFIG_USB_FUNCTION
	msm_register_usb_phy_init_seq(legend_phy_init_seq);
	msm_add_usb_id_pin_gpio(LEGEND_GPIO_USB_ID_PIN);
	msm_add_usb_devices(legend_phy_reset, NULL);
#endif

#ifdef CONFIG_USB_ANDROID
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
#endif
	msm_add_mem_devices(&pmem_setting);

	msm_init_pmic_vibrator(3000);
#ifdef CONFIG_MICROP_COMMON
	legend_microp_init();
#endif

	rc = legend_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	/* register flashlight at new-XA above */
	platform_device_register(&legend_flashlight_device);

	/* probe camera driver */
	i2c_register_board_info(0, i2c_camera_devices, ARRAY_SIZE(i2c_camera_devices));

	msm_device_i2c_init();

	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	legend_init_panel();

	legend_init_keypad();
}

static void __init legend_fixup(struct machine_desc *desc, struct tag *tags,
					char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_LINUX_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE);
	mi->bank[0].size = MSM_LINUX_SIZE;
}

static void __init legend_map_io(void)
{
	printk("legend_init_map_io()\n");
	msm_map_common_io();
	msm_clock_init();
#ifdef CONFIG_CACHE_L2X0
	/* 7x27 has 256KB L2 cache:
	* 64Kb/Way and 4-Way Associativity;
	* R/W latency: 3 cycles;
	* evmon/parity/share disabled.
	*/
	l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

MACHINE_START(LEGEND, "legend")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x12C00100,
	.fixup          = legend_fixup,
	.map_io         = legend_map_io,
	.init_irq       = legend_init_irq,
	.init_machine   = legend_init,
	.timer          = &msm_timer,
MACHINE_END
