/* arch/arm/mach-msm/board-liberty.c
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
#include <linux/atmel_qt602240.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>
#include <linux/curcial_oj.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
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

#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/vreg.h>
/* #include <mach/gpio_chip.h> */
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
#include <mach/bcm_bt_lpm.h>
#endif

#include <mach/atmega_microp.h>
/* #include <mach/msm_tssc.h> */
#include <mach/htc_battery.h>
#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>
#include <mach/camera.h>
#include <mach/msm_serial_debugger.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/htc_usb.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>

#include "devices.h"
#include "board-liberty.h"
#include "proc_comm.h"
#include "../../../drivers/staging/android/timed_gpio.h"


void msm_init_irq(void);
void msm_init_gpio(void);
void config_liberty_camera_on_gpios(void);
void config_liberty_camera_off_gpios(void);

void config_liberty_proximity_gpios(int on);

#ifdef CONFIG_MICROP_COMMON
void __init liberty_microp_init(void);
#endif

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= LIBERTY_GPIO_35MM_HEADSET_DET,
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


static struct bma150_platform_data liberty_g_sensor_pdata = {
	.microp_new_cmd = 1,
	.chip_layout = 1,
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev		= {
			.platform_data	= &lightsensor_data,
		},
	},
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
			.platform_data = &liberty_g_sensor_pdata,
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
	.gpio_reset = LIBERTY_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static struct synaptics_i2c_rmi_platform_data liberty_ts_t1007_data[] = {
	{
		.version = 0x0100,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -10 * 0x10000 / 3214,
		.inactive_right = -10 * 0x10000 / 3214,
		.inactive_top = -20 * 0x10000 / 5414,
		.inactive_bottom = -20 * 0x10000 / 5414,
		.snap_left_on = 10 * 0x10000 / 3214,
		.snap_left_off = 20 * 0x10000 / 3214,
		.snap_right_on = 10 * 0x10000 / 3214,
		.snap_right_off = 20 * 0x10000 / 3214,
		.snap_top_on = 20 * 0x10000 / 5414,
		.snap_top_off = 30 * 0x10000 / 5414,
		.snap_bottom_on = 20 * 0x10000 / 5414,
		.snap_bottom_off = 30 * 0x10000 / 5414,
	}
};

static struct synaptics_i2c_rmi_platform_data liberty_ts_t1021_data[] = {
	{
		.version = 0x0100,
		.inactive_left = -1 * 0x10000 / 320,
		.inactive_right = -1 * 0x10000 / 320,
		.inactive_top = -1 * 0x10000 / 480,
		.inactive_bottom = -50 * 0x10000 / 480,
	}
};

static int liberty_ts_atmel_power(int on)
{
	printk(KERN_INFO "%s():\n", __func__);
	if (on) {
		gpio_set_value(LIBERTY_TP_5V_EN, 1);
		msleep(2);
		gpio_set_value(LIBERTY_GPIO_TP_RST, 1);
	} else {
		gpio_set_value(LIBERTY_TP_5V_EN, 0);
		msleep(2);
	}
	return 0;
}

struct atmel_i2c_platform_data liberty_ts_atmel_data[] = {
	{
		.version = 0x016,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 915,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = LIBERTY_GPIO_TP_ATT_N,
		.power = liberty_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {7, 0, 10, 10, 0, 0, 10, 15},
		.config_T9 = {139, 0, 0, 16, 10, 0, 16, 40, 3, 1, 10, 10, 5, 15, 3, 10, 20, 0, 0, 0, 0, 0, 254, 2, 42, 36, 154, 54, 142, 89, 40},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {7, 0, 0, 0, 0, 0, 0, 35, 20, 4, 15, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 16, 0, 1, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8, 60},
		.object_crc = {0xDC, 0x1F, 0x50},
		.cable_config = {30, 30, 8, 16},
	},
	{
		.version = 0x0015,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 915,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = LIBERTY_GPIO_TP_ATT_N,
		.power = liberty_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 20, 10, 0, 0, 5, 25},
		.config_T9 = {139, 0, 0, 16, 10, 0, 16, 35, 2, 1, 0, 5, 2, 14, 2, 10, 20, 0, 0, 0, 0, 0, 1, 1, 20, 20, 153, 54, 153, 89},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {19, 0, 0, 5, 5, 0, 0, 35, 20, 4, 15, 0},
		.config_T22 = {13, 0, 0, 25, 0, -25, 255, 4, 20, 0, 1, 10, 15, 20, 25, 30, 4},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 200, 50, 64, 31, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8, 60},
		.object_crc = {0x62, 0xAE, 0x09},
	},
	{
		.version = 0x0014,
		.abs_x_min = 13,
		.abs_x_max = 1009,
		.abs_y_min = 10,
		.abs_y_max = 930,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = LIBERTY_GPIO_TP_ATT_N,
		.power = liberty_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {32, 16, 50},
		.config_T8 = {8, 0, 20, 20, 0, 0, 10, 15},
		.config_T9 = {131, 0, 0, 16, 10, 0, 48, 35, 2, 1, 0, 1, 1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {5, 0, 0, 25, 0, -25, 255, 4, 50, 0, 1, 10, 15, 20, 10, 10, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 224, 46, 88, 27, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8},
	}
};

static int liberty_phy_init_seq[] = { 0x1D, 0x0D, 0x1D, 0x10, -1 };
static void liberty_phy_reset(void)
{
	int ret;
	printk(KERN_INFO "msm_hsusb_phy_reset\n");
	ret = msm_proc_comm(PCOM_MSM_HSUSB_PHY_RESET,
			NULL, NULL);
	if (ret)
		printk(KERN_INFO "%s failed\n", __func__);
}

#ifdef CONFIG_USB_ANDROID
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= liberty_phy_init_seq,
	.phy_reset		= liberty_phy_reset,
	.usb_id_pin_gpio =  LIBERTY_GPIO_USB_ID_PIN,
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

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x18d1,
	.vendorDescr	= "Google, Inc.",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c92,
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
static struct akm8973_platform_data compass_platform_data = {
	.layouts = LIBERTY_LAYOUTS,
	.project_name = LIBERTY_PROJECT_NAME,
	.reset = LIBERTY_GPIO_COMPASS_RST_N,
	.intr = LIBERTY_GPIO_COMPASS_INT_N,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_T1007_NAME, 0x20),
		.platform_data = &liberty_ts_t1007_data,
		.irq = LIBERTY_GPIO_TO_INT(LIBERTY_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(SYNAPTICS_T1021_NAME, 0x21),
		.platform_data = &liberty_ts_t1021_data,
		.irq = LIBERTY_GPIO_TO_INT(LIBERTY_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(ATMEL_QT602240_NAME, 0x94 >> 1),
		.platform_data = &liberty_ts_atmel_data,
		.irq = MSM_GPIO_TO_INT(LIBERTY_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = LIBERTY_GPIO_TO_INT(LIBERTY_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = LIBERTY_GPIO_TO_INT(LIBERTY_GPIO_COMPASS_INT_N),
	},
};

static struct pwr_sink liberty_pwrsink_table[] = {
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

static int liberty_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void liberty_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void liberty_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int liberty_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data liberty_pwrsink_data = {
	.num_sinks      = ARRAY_SIZE(liberty_pwrsink_table),
	.sinks          = liberty_pwrsink_table,
	.suspend_late	= liberty_pwrsink_suspend_late,
	.resume_early	= liberty_pwrsink_resume_early,
	.suspend_early	= liberty_pwrsink_suspend_early,
	.resume_late	= liberty_pwrsink_resume_late,
};

static struct platform_device liberty_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev    = {
		.platform_data = &liberty_pwrsink_data,
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
	.camera_gpio_on  = config_liberty_camera_on_gpios,
	.camera_gpio_off = config_liberty_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "s5k4e1gx",
	.sensor_reset   = LIBERTY_GPIO_CAM1_RST_N,
	.vcm_pwd        = LIBERTY_GPIO_VCM_PWDN,
	.pdata          = &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name      = "msm_camera_s5k4e1gx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};
#endif

static struct platform_device liberty_rfkill = {
	.name = "liberty_rfkill",
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
		config_liberty_proximity_gpios(1);
		gpio_direction_output(LIBERTY_GPIO_PROXIMITY_EN, 1);
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
		gpio_direction_output(LIBERTY_GPIO_PROXIMITY_EN, 0);
		config_liberty_proximity_gpios(0);
	}

	return rc;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static unsigned int als_power_control;

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
	.p_out = LIBERTY_GPIO_PROXIMITY_INT,
	.p_en = LIBERTY_GPIO_PROXIMITY_EN,
	.power = capella_cm3602_power,
	.irq = MSM_GPIO_TO_INT(LIBERTY_GPIO_PROXIMITY_INT),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};
/* End Proximity Sensor (Capella_CM3602)*/

static void curcial_oj_shutdown (int	enable)
{
	uint8_t cmd[3];
	memset(cmd, 0, sizeof(uint8_t)*3);

	cmd[2] = 0x80;
	if (enable)
		microp_i2c_write(0x91, cmd, 3);
	else
		microp_i2c_write(0x90, cmd, 3);
}
static int curcial_oj_poweron(int on)
{
	struct vreg	*oj_power = vreg_get(0, "rftx");
	if (IS_ERR(oj_power)) {
		printk(KERN_ERR"%s:Error power domain\n",__func__);
		return 0;
	}

	if (on) {
		vreg_set_level(oj_power, 2850);
		vreg_enable(oj_power);
		printk(KERN_ERR "%s:OJ	power	enable(%d)\n", __func__, on);
	} else {
		vreg_disable(oj_power);
		printk(KERN_ERR "%s:OJ	power	enable(%d)\n", __func__, on);
		}
	return 1;
}
#define LIB_MICROP_VER	0x02
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
static struct curcial_oj_platform_data liberty_oj_data = {
	.oj_poweron = curcial_oj_poweron,
	.oj_shutdown = curcial_oj_shutdown,
	.oj_adjust_xy = curcial_oj_adjust_xy,
	.microp_version = LIB_MICROP_VER,
	.mdelay_time = 0,
	.normal_th = 8,
	.xy_ratio = 15,
	.interval = 20,
	.swap = true,
	.x = 1,
	.y = 1,
	.share_power = false,
	.debugflag = 0,
	.ap_code = true,
	.sht_tbl = {0, 2000, 2250, 2500, 2750, 3000},
	.pxsum_tbl = {0, 0, 40, 50, 60, 70},
	.degree = 6,
	.Xsteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 10, 10, 10, 10, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.Ysteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 10, 10, 10, 10, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.irq = MSM_uP_TO_INT(12),
};

static struct platform_device liberty_oj = {
	.name = CURCIAL_OJ_NAME,
	.id = -1,
	.dev = {
		.platform_data	= &liberty_oj_data,
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

static struct timed_gpio liberty_timed_gpios_str[] = {
	{
		.name = "vibrator",
		.gpio = LIBERTY_GPIO_VIB_3V_EN,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data liberty_timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(liberty_timed_gpios_str),
	.gpios		= liberty_timed_gpios_str,
};

static struct platform_device liberty_timed_gpios = {
	.name		= TIMED_GPIO_NAME,
	.id		= -1,
	.dev		= {
		.platform_data	= &liberty_timed_gpio_data,
	},
};

#if defined(CONFIG_SERIAL_MSM_HS) && defined(CONFIG_SERIAL_MSM_HS_PURE_ANDROID)
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
        .rx_wakeup_irq = -1,
        .inject_rx_on_wakeup = 0,
        .exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
        .gpio_wake = LIBERTY_GPIO_BT_CHIP_WAKE,
        .gpio_host_wake = LIBERTY_GPIO_BT_HOST_WAKE,
        .request_clock_off_locked = msm_hs_request_clock_off_locked,
        .request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
        .name = "bcm_bt_lpm",
        .id = 0,
        .dev = {
                .platform_data = &bcm_bt_lpm_pdata,
        },
};

#define ATAG_BDADDR 0x43294329  /* mahimahi bluetooth address tag */
#define ATAG_BDADDR_SIZE 4
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];
extern unsigned char *get_bt_bd_ram(void);

static void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	sprintf(bdaddr, "%02x:%02x:%02x:%02x:%02x:%02x",
		cTemp[0], cTemp[1], cTemp[2], cTemp[3], cTemp[4], cTemp[5]);
	printk(KERN_INFO "BT HW address=%s\n", bdaddr);
}

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

#elif defined(CONFIG_SERIAL_MSM_HS)
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(LIBERTY_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = LIBERTY_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = LIBERTY_GPIO_BT_HOST_WAKE,

};
#endif


static struct platform_device *devices[] __initdata = {
	&msm_device_i2c,
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
        &bcm_bt_lpm_device,
#endif
	&htc_battery_pdev,
	&msm_camera_sensor_s5k4e1gx,
	&liberty_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&liberty_pwr_sink,
#endif
	&liberty_oj,
	&capella_cm3602,
	&liberty_timed_gpios,
};

extern struct sys_timer msm_timer;

static void __init liberty_init_irq(void)
{
	printk("liberty_init_irq()\n");
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

static void liberty_reset(void)
{
	gpio_set_value(LIBERTY_GPIO_PS_HOLD, 0);
}

static uint32_t proximity_on_gpio_table[] = {
	PCOM_GPIO_CFG(LIBERTY_GPIO_PROXIMITY_INT,
		0, GPIO_INPUT, GPIO_NO_PULL, 0), /* PS_VOUT */
};

static uint32_t proximity_off_gpio_table[] = {
	PCOM_GPIO_CFG(LIBERTY_GPIO_PROXIMITY_INT,
		0, GPIO_INPUT, GPIO_PULL_DOWN, 0) /* PS_VOUT */
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

void config_liberty_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_liberty_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

void config_liberty_proximity_gpios(int on)
{
	if (on)
		config_gpio_table(proximity_on_gpio_table,
			ARRAY_SIZE(proximity_on_gpio_table));
	else
		config_gpio_table(proximity_off_gpio_table,
			ARRAY_SIZE(proximity_off_gpio_table));
}

#ifndef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
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
#endif

static uint32_t liberty_serial_debug_table[] = {
	/* config as serial debug uart */
	PCOM_GPIO_CFG(LIBERTY_GPIO_UART3_RX, 1,
			GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* UART3 RX */
	PCOM_GPIO_CFG(LIBERTY_GPIO_UART3_TX, 1,
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* UART3 TX */
};

static void liberty_config_serial_debug_gpios(void)
{
	config_gpio_table(liberty_serial_debug_table,
			ARRAY_SIZE(liberty_serial_debug_table));
}

static void __init config_gpios(void)
{
	liberty_config_serial_debug_gpios();
	config_liberty_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data liberty_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 200000,
};

static unsigned liberty_perf_acpu_table[] = {
	245760000,
	480000000,
	600000000,
};

static struct perflock_platform_data liberty_perflock_data = {
	.perf_acpu_table = liberty_perf_acpu_table,
	.table_size = ARRAY_SIZE(liberty_perf_acpu_table),
};

static ssize_t liberty_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	if (system_rev > 1) {
			/* center: x: home: 25, menu: 85, back: 235, search 295, y: 510 */
		return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOME)	    ":25:510:50:55"
			":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":85:510:50:55"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":235:510:50:55"
			":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":295:510:50:55"
			"\n");
	} else {

		/* center: x: home: 30, menu: 110, back: 205, search 285, y: 510 */
		return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":30:510:60:55"
			":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":110:510:100:55"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":205:510:90:55"
			":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":285:510:70:55"
			"\n");
	}
}

static struct kobj_attribute liberty_synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &liberty_virtual_keys_show,
};

static struct kobj_attribute liberty_atmel_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &liberty_virtual_keys_show,
};

static struct attribute *liberty_properties_attrs[] = {
	&liberty_synaptics_virtual_keys_attr.attr,
	&liberty_atmel_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group liberty_properties_attr_group = {
	.attrs = liberty_properties_attrs,
};

static void __init liberty_init(void)
{
	int rc;
	char *cid = NULL;
	struct kobject *properties_kobj;

	printk("liberty_init() revision = 0x%X\n", system_rev);
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
	gpio_request(LIBERTY_GPIO_LS_EN, "ls_en");
	gpio_direction_output(LIBERTY_GPIO_LS_EN, 0);
	/* disable power for cm3602 chip */
	/* __capella_cm3602_power(0); */

	msm_hw_reset_hook = liberty_reset;

	msm_acpu_clock_init(&liberty_clock_data);
	perflock_init(&liberty_perflock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(LIBERTY_GPIO_UART3_RX));
#endif

	msm_add_devices();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#ifndef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
	msm_device_uart_dm1.name = "msm_serial_hs_bcm";	/* for bcm */
#endif
	msm_add_serial_devices(3);
#else
	msm_add_serial_devices(0);
#endif

	msm_add_serial_devices(2);
	/*
	msm_change_usb_id(0x0bb4, 0x0c10);
	*/
#ifdef CONFIG_USB_FUNCTION
	msm_add_usb_id_pin_gpio(LIBERTY_GPIO_USB_ID_PIN);
	msm_add_usb_devices(liberty_phy_reset, NULL);
#endif

#ifdef CONFIG_USB_ANDROID
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_device_register(&msm_device_hsusb);
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
#endif
	msm_add_mem_devices(&pmem_setting);

#ifdef CONFIG_MICROP_COMMON
	liberty_microp_init();
#endif

	rc = liberty_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
						&liberty_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	/* probe camera driver */
	i2c_register_board_info(0, i2c_camera_devices, ARRAY_SIZE(i2c_camera_devices));

	msm_device_i2c_init();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	liberty_init_panel();

	liberty_init_keypad();
}

static void __init liberty_fixup(struct machine_desc *desc, struct tag *tags,
                               char **cmdline, struct meminfo *mi)
{
	mi->nr_banks=1;
	mi->bank[0].start = MSM_LINUX_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE);
	mi->bank[0].size = MSM_LINUX_SIZE;
}

static void __init liberty_map_io(void)
{
	printk("liberty_init_map_io()\n");
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

MACHINE_START(LIBERTY, "liberty")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x12C00100,
	.fixup          = liberty_fixup,
	.map_io         = liberty_map_io,
	.init_irq       = liberty_init_irq,
	.init_machine   = liberty_init,
	.timer          = &msm_timer,
MACHINE_END
