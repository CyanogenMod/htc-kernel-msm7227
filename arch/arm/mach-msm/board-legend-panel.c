/* linux/arch/arm/mach-msm/board-legend-panel.c
 *
 * Copyright (c) 2009 Google Inc.
 * Author: Dima Zavin <dima@android.com>
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/vreg.h>
#include <mach/msm_fb.h>
/* #include <mach/pmic.h> */

#include "devices.h"
#include "board-legend.h"
#include "proc_comm.h"

#ifdef CONFIG_MICROP_COMMON
#include <mach/atmega_microp.h>
#endif

#if 0
#define D(fmt, args...) printk(KERN_INFO "Panel: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

static struct wake_lock backlight_wakelock;
static struct work_struct work_set_brightness;
static int g_brightness_level;
static DEFINE_MUTEX(legend_backlight_lock);
static atomic_t gate;
extern int panel_type;
extern int microp_spi_vote_enable(int spi_device, uint8_t enable);
#define SPI_LCM                         (1 << 1)

#define GAMMA_LEVEL_MAX_SAMSUNG 27
#define GAMMA_LEVEL_MIN_SAMSUNG 0
#define GAMMA_LEVEL_DEFAULT_SAMSUNG 11
#define LED_VALUE_MAX_SAMSUNG LED_FULL
#define LED_VALUE_MIN_SAMSUNG 30
#define LED_VALUE_DEFAULT_SAMSUNG 100

#define GAMMA_LEVEL_MAX_TPO 15
#define GAMMA_LEVEL_MIN_TPO 0
#define GAMMA_LEVEL_DEFAULT_TPO 10
#define LED_VALUE_MAX_TPO LED_FULL
#define LED_VALUE_MIN_TPO 30
#define LED_VALUE_DEFAULT_TPO 100

void config_legend_display_on_gpios(void);
void config_legend_display_off_gpios(void);

typedef struct lcd_init_table {
	u8 reg;
	u8 val;
	int delay;
} LCD_INIT_TABLE;

static const LCD_INIT_TABLE OLED_POWER_SEQUENCE[] = {
	{0x01, 0x00, 0},
	{0x21, 0x33, 0},
	{0x22, 0x08, 0},
	{0x23, 0x00, 0},
	{0x24, 0x33, 0},
	{0x25, 0x33, 0},
	{0x26, 0x02, 0},
	{0x27, 0x42, 0},
	{0x2F, 0x02, 0},

	{0x20, 0x01, 10},
	{0x20, 0x11, 20},
	{0x20, 0x31, 60},
	{0x20, 0x71, 60},
	{0x20, 0x73, 20},
	{0x20, 0x77, 10},

	{0x04, 0x01, 10},
};

static const LCD_INIT_TABLE OLED_INIT_SEQUENCE[] = {
	{0x06, 0x44, 0},
	{0x07, 0x04, 0},
	{0x08, 0x01, 0},
	{0x09, 0x06, 0},
	{0x0a, 0x21, 0},
	{0x0c, 0x00, 0},
	{0x0d, 0x14, 0},
	{0x0e, 0x00, 0},
	{0x0f, 0x1E, 0},
	{0x10, 0x02, 0},	// 0:RGB888 2:RGB565
	{0x1c, 0x08, 0},
	{0x1d, 0x05, 0},
	{0x1f, 0x00, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_10_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x0D, 0},
		{0x34, 0x11, 0},
		{0x35, 0x14, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x1D, 0},
		{0x38, 0x1D, 0},
		{0x39, 0x27, 0},
		{0x3A, 0x15, 0},
		{0x3B, 0x25, 0},
		{0x3C, 0x3F, 0},
		{0x3D, 0x00, 0},
		{0x3E, 0x31, 0},
		{0x3F, 0x3F, 0},
		{0x40, 0x00, 0},
		{0x41, 0x3F, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_40_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x20, 0},
		{0x34, 0x27, 0},
		{0x35, 0x2B, 0},
		{0x36, 0x20, 0},
		{0x37, 0x1F, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2B, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x2B, 0},
		{0x3C, 0x1A, 0},
		{0x3D, 0x0A, 0},
		{0x3E, 0x13, 0},
		{0x3F, 0x3F, 0},
		{0x40, 0x00, 0},
		{0x41, 0x3E, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_50_TABLE[] = {
		/* 50 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x24, 0},
		{0x34, 0x2B, 0},
		{0x35, 0x30, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x20, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2B, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2B, 0},
		{0x3C, 0x1B, 0},
		{0x3D, 0x14, 0},
		{0x3E, 0x16, 0},
		{0x3F, 0x3F, 0},
		{0x40, 0x00, 0},
		{0x41, 0x3B, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_60_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x28, 0},
		{0x34, 0x2F, 0},
		{0x35, 0x34, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x20, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2B, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2B, 0},
		{0x3C, 0x1C, 0},
		{0x3D, 0x18, 0},
		{0x3E, 0x18, 0},
		{0x3F, 0x3F, 0},
		{0x40, 0x00, 0},
		{0x41, 0x38, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_70_TABLE[] = {
		/* 70 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x2B, 0},
		{0x34, 0x33, 0},
		{0x35, 0x38, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x20, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2B, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2B, 0},
		{0x3C, 0x1E, 0},
		{0x3D, 0x1D, 0},
		{0x3E, 0x1A, 0},
		{0x3F, 0x3F, 0},
		{0x40, 0x00, 0},
		{0x41, 0x35, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_80_TABLE[] = {
		/* 80 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x2E, 0},
		{0x34, 0x36, 0},
		{0x35, 0x3B, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x20, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2B, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2B, 0},
		{0x3C, 0x1F, 0},
		{0x3D, 0x21, 0},
		{0x3E, 0x1C, 0},
		{0x3F, 0x3F, 0},
		{0x40, 0x00, 0},
		{0x41, 0x32, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_90_TABLE[] = {
		/* 90 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x31, 0},
		{0x34, 0x39, 0},
		{0x35, 0x3E, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x1F, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2B, 0},
		{0x3A, 0x25, 0},
		{0x3B, 0x2B, 0},
		{0x3C, 0x20, 0},
		{0x3D, 0x23, 0},
		{0x3E, 0x1D, 0},
		{0x3F, 0x3E, 0},
		{0x40, 0x00, 0},
		{0x41, 0x31, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_100_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x33, 0},
		{0x34, 0x3C, 0},
		{0x35, 0x41, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x1F, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2B, 0},
		{0x3A, 0x25, 0},
		{0x3B, 0x2B, 0},
		{0x3C, 0x20, 0},
		{0x3D, 0x25, 0},
		{0x3E, 0x1E, 0},
		{0x3F, 0x3E, 0},
		{0x40, 0x00, 0},
		{0x41, 0x30, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_110_TABLE[] = {
		/* 110 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x35, 0},
		{0x34, 0x3F, 0},
		{0x35, 0x44, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x1F, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x21, 0},
		{0x3D, 0x26, 0},
		{0x3E, 0x1F, 0},
		{0x3F, 0x3C, 0},
		{0x40, 0x00, 0},
		{0x41, 0x2E, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_120_TABLE[] = {
		/* 120 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x37, 0},
		{0x34, 0x41, 0},
		{0x35, 0x46, 0},
		{0x36, 0x1F, 0},
		{0x37, 0x1F, 0},
		{0x38, 0x1E, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x22, 0},
		{0x3D, 0x27, 0},
		{0x3E, 0x20, 0},
		{0x3F, 0x3A, 0},
		{0x40, 0x00, 0},
		{0x41, 0x2C, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_130_TABLE[] = {
		/* 130 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x3A, 0},
		{0x34, 0x44, 0},
		{0x35, 0x49, 0},
		{0x36, 0x1D, 0},
		{0x37, 0x1E, 0},
		{0x38, 0x1D, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x29, 0},
		{0x3E, 0x21, 0},
		{0x3F, 0x39, 0},
		{0x40, 0x04, 0},
		{0x41, 0x2C, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_140_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x3C, 0},
		{0x34, 0x46, 0},
		{0x35, 0x4C, 0},
		{0x36, 0x1D, 0},
		{0x37, 0x1E, 0},
		{0x38, 0x1C, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x24, 0},
		{0x3D, 0x2A, 0},
		{0x3E, 0x22, 0},
		{0x3F, 0x38, 0},
		{0x40, 0x07, 0},
		{0x41, 0x2C, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_150_TABLE[] = {
		/* 150 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x3E, 0},
		{0x34, 0x48, 0},
		{0x35, 0x4E, 0},
		{0x36, 0x1E, 0},
		{0x37, 0x1E, 0},
		{0x38, 0x1C, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2A, 0},
		{0x3E, 0x22, 0},
		{0x3F, 0x38, 0},
		{0x40, 0x09, 0},
		{0x41, 0x2C, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_160_TABLE[] = {
		/* 160 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x3F, 0},
		{0x34, 0x4A, 0},
		{0x35, 0x50, 0},
		{0x36, 0x1E, 0},
		{0x37, 0x1E, 0},
		{0x38, 0x1C, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2A, 0},
		{0x3E, 0x22, 0},
		{0x3F, 0x38, 0},
		{0x40, 0x0B, 0},
		{0x41, 0x2C, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_170_TABLE[] = {
		/* 170 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x41, 0},
		{0x34, 0x4C, 0},
		{0x35, 0x52, 0},
		{0x36, 0x1D, 0},
		{0x37, 0x1E, 0},
		{0x38, 0x1C, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2A, 0},
		{0x3E, 0x22, 0},
		{0x3F, 0x39, 0},
		{0x40, 0x11, 0},
		{0x41, 0x2E, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_180_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x43, 0},
		{0x34, 0x4E, 0},
		{0x35, 0x54, 0},
		{0x36, 0x1D, 0},
		{0x37, 0x1E, 0},
		{0x38, 0x1C, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x2A, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2A, 0},
		{0x3E, 0x22, 0},
		{0x3F, 0x39, 0},
		{0x40, 0x13, 0},
		{0x41, 0x2F, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_190_TABLE[] = {
		/* 190 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x45, 0},
		{0x34, 0x50, 0},
		{0x35, 0x56, 0},
		{0x36, 0x1D, 0},
		{0x37, 0x1D, 0},
		{0x38, 0x1C, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x29, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2B, 0},
		{0x3E, 0x23, 0},
		{0x3F, 0x38, 0},
		{0x40, 0x14, 0},
		{0x41, 0x2E, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_200_TABLE[] = {
		/* 200 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x46, 0},
		{0x34, 0x52, 0},
		{0x35, 0x58, 0},
		{0x36, 0x1D, 0},
		{0x37, 0x1D, 0},
		{0x38, 0x1C, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x29, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2B, 0},
		{0x3E, 0x23, 0},
		{0x3F, 0x38, 0},
		{0x40, 0x15, 0},
		{0x41, 0x2E, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_210_TABLE[] = {
		/* 210 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x48, 0},
		{0x34, 0x54, 0},
		{0x35, 0x5B, 0},
		{0x36, 0x1C, 0},
		{0x37, 0x1C, 0},
		{0x38, 0x1B, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x29, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2B, 0},
		{0x3E, 0x23, 0},
		{0x3F, 0x3A, 0},
		{0x40, 0x18, 0},
		{0x41, 0x30, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_220_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x4A, 0},
		{0x34, 0x56, 0},
		{0x35, 0x5D, 0},
		{0x36, 0x1B, 0},
		{0x37, 0x1C, 0},
		{0x38, 0x1A, 0},
		{0x39, 0x2A, 0},
		{0x3A, 0x24, 0},
		{0x3B, 0x29, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2B, 0},
		{0x3E, 0x23, 0},
		{0x3F, 0x3B, 0},
		{0x40, 0x1B, 0},
		{0x41, 0x32, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_230_TABLE[] = {
		/* 230 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x4B, 0},
		{0x34, 0x58, 0},
		{0x35, 0x5F, 0},
		{0x36, 0x1C, 0},
		{0x37, 0x1D, 0},
		{0x38, 0x1B, 0},
		{0x39, 0x29, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x28, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2B, 0},
		{0x3E, 0x24, 0},
		{0x3F, 0x3B, 0},
		{0x40, 0x1C, 0},
		{0x41, 0x33, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_240_TABLE[] = {
		/* 240 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x4C, 0},
		{0x34, 0x59, 0},
		{0x35, 0x60, 0},
		{0x36, 0x1D, 0},
		{0x37, 0x1D, 0},
		{0x38, 0x1B, 0},
		{0x39, 0x29, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x28, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x2B, 0},
		{0x3E, 0x24, 0},
		{0x3F, 0x3B, 0},
		{0x40, 0x1D, 0},
		{0x41, 0x33, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_250_TABLE[] = {
		/* 250 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x4E, 0},
		{0x34, 0x5B, 0},
		{0x35, 0x62, 0},
		{0x36, 0x1C, 0},
		{0x37, 0x1C, 0},
		{0x38, 0x1A, 0},
		{0x39, 0x29, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x28, 0},
		{0x3C, 0x24, 0},
		{0x3D, 0x2C, 0},
		{0x3E, 0x25, 0},
		{0x3F, 0x3C, 0},
		{0x40, 0x1F, 0},
		{0x41, 0x34, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_260_TABLE[] = {
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x50, 0},
		{0x34, 0x5D, 0},
		{0x35, 0x64, 0},
		{0x36, 0x1B, 0},
		{0x37, 0x1C, 0},
		{0x38, 0x1A, 0},
		{0x39, 0x29, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x28, 0},
		{0x3C, 0x25, 0},
		{0x3D, 0x2C, 0},
		{0x3E, 0x25, 0},
		{0x3F, 0x3C, 0},
		{0x40, 0x21, 0},
		{0x41, 0x34, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_270_TABLE[] = {
		/* 270 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x51, 0},
		{0x34, 0x5F, 0},
		{0x35, 0x66, 0},
		{0x36, 0x1C, 0},
		{0x37, 0x1C, 0},
		{0x38, 0x1A, 0},
		{0x39, 0x29, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x28, 0},
		{0x3C, 0x25, 0},
		{0x3D, 0x2D, 0},
		{0x3E, 0x25, 0},
		{0x3F, 0x3D, 0},
		{0x40, 0x22, 0},
		{0x41, 0x35, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_280_TABLE[] = {
		/* 280 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x52, 0},
		{0x34, 0x60, 0},
		{0x35, 0x67, 0},
		{0x36, 0x1C, 0},
		{0x37, 0x1C, 0},
		{0x38, 0x1A, 0},
		{0x39, 0x29, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x28, 0},
		{0x3C, 0x25, 0},
		{0x3D, 0x2D, 0},
		{0x3E, 0x25, 0},
		{0x3F, 0x3D, 0},
		{0x40, 0x23, 0},
		{0x41, 0x36, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_290_TABLE[] = {
		/* 290 nits*/
		{0x30, 0x3A, 0},
		{0x31, 0x3A, 0},
		{0x32, 0x3A, 0},
		{0x33, 0x54, 0},
		{0x34, 0x62, 0},
		{0x35, 0x69, 0},
		{0x36, 0x1B, 0},
		{0x37, 0x1B, 0},
		{0x38, 0x1A, 0},
		{0x39, 0x29, 0},
		{0x3A, 0x23, 0},
		{0x3B, 0x28, 0},
		{0x3C, 0x25, 0},
		{0x3D, 0x2E, 0},
		{0x3E, 0x25, 0},
		{0x3F, 0x3E, 0},
		{0x40, 0x25, 0},
		{0x41, 0x37, 0},
};

static const LCD_INIT_TABLE LCD_GAMMA_300_TABLE[] = {
		{0x30, 0x3a, 0},
		{0x31, 0x3a, 0},
		{0x32, 0x3a, 0},
		{0x33, 0x53, 0},
		{0x34, 0x62, 0},
		{0x35, 0x6e, 0},
		{0x36, 0x18, 0},
		{0x37, 0x1b, 0},
		{0x38, 0x16, 0},
		{0x39, 0x2e, 0},
		{0x3A, 0x26, 0},
		{0x3B, 0x2c, 0},
		{0x3C, 0x23, 0},
		{0x3D, 0x31, 0},
		{0x3E, 0x29, 0},
		{0x3F, 0x30, 0},
		{0x40, 0x28, 0},
		{0x41, 0x3f, 0},
};

static const LCD_INIT_TABLE *gamma_level_table[] = {
	LCD_GAMMA_10_TABLE,
	LCD_GAMMA_40_TABLE,
	LCD_GAMMA_50_TABLE,
	LCD_GAMMA_60_TABLE,
	LCD_GAMMA_70_TABLE,
	LCD_GAMMA_80_TABLE,
	LCD_GAMMA_90_TABLE,
	LCD_GAMMA_100_TABLE,
	LCD_GAMMA_110_TABLE,
	LCD_GAMMA_120_TABLE,
	LCD_GAMMA_130_TABLE,
	LCD_GAMMA_140_TABLE,
	LCD_GAMMA_150_TABLE,
	LCD_GAMMA_160_TABLE,
	LCD_GAMMA_170_TABLE,
	LCD_GAMMA_180_TABLE,
	LCD_GAMMA_190_TABLE,
	LCD_GAMMA_200_TABLE,
	LCD_GAMMA_210_TABLE,
	LCD_GAMMA_220_TABLE,
	LCD_GAMMA_230_TABLE,
	LCD_GAMMA_240_TABLE,
	LCD_GAMMA_250_TABLE,
	LCD_GAMMA_260_TABLE,
	LCD_GAMMA_270_TABLE,
	LCD_GAMMA_280_TABLE,
	LCD_GAMMA_290_TABLE,
	LCD_GAMMA_300_TABLE
};

static const LCD_INIT_TABLE TPO_10_TABLE[] = {
		{0x51, 0x8, 0},
};

static const LCD_INIT_TABLE TPO_40_TABLE[] = {
		{0x51, 0x10, 0},
};

static const LCD_INIT_TABLE TPO_50_TABLE[] = {
		{0x51, 0x14, 0},
};

static const LCD_INIT_TABLE TPO_60_TABLE[] = {
		{0x51, 0x1C, 0},
};

static const LCD_INIT_TABLE TPO_80_TABLE[] = {
		{0x51, 0x20, 0},
};

static const LCD_INIT_TABLE TPO_100_TABLE[] = {
		{0x51, 0x2C, 0},
};

static const LCD_INIT_TABLE TPO_120_TABLE[] = {
		{0x51, 0x34, 0},
};

static const LCD_INIT_TABLE TPO_140_TABLE[] = {
		{0x51, 0x40, 0},
};

static const LCD_INIT_TABLE TPO_160_TABLE[] = {
		{0x51, 0x4C, 0},
};

static const LCD_INIT_TABLE TPO_180_TABLE[] = {
		{0x51, 0x55, 0},
};

static const LCD_INIT_TABLE TPO_200_TABLE[] = {
		{0x51, 0x6D, 0},
};

static const LCD_INIT_TABLE TPO_220_TABLE[] = {
		{0x51, 0xAA, 0},
};

static const LCD_INIT_TABLE TPO_240_TABLE[] = {
		{0x51, 0xBB, 0},
};

static const LCD_INIT_TABLE TPO_260_TABLE[] = {
		{0x51, 0xDD, 0},
};

static const LCD_INIT_TABLE TPO_280_TABLE[] = {
		{0x51, 0xEE, 0},
};

static const LCD_INIT_TABLE TPO_300_TABLE[] = {
		{0x51, 0xFF, 0},
};

static const LCD_INIT_TABLE *tpo_level_table[] = {
	TPO_10_TABLE,
	TPO_40_TABLE,
	TPO_50_TABLE,
	TPO_60_TABLE,
	TPO_80_TABLE,
	TPO_100_TABLE,
	TPO_120_TABLE,
	TPO_140_TABLE,
	TPO_160_TABLE,
	TPO_180_TABLE,
	TPO_200_TABLE,
	TPO_220_TABLE,
	TPO_240_TABLE,
	TPO_260_TABLE,
	TPO_280_TABLE,
	TPO_300_TABLE,
};

static struct vreg *vreg_lcm_1v8;
static struct vreg *vreg_lcm_2v8;

static uint32_t display_on_gpio_table[] = {
	/* Display */
	PCOM_GPIO_CFG(LEGEND_LCD_G5, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_DE, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_HSYNC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_VSYNC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B0, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B1, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),

	PCOM_GPIO_CFG(LEGEND_LCD_G1, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G0, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B4, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B3, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B2, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G4, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G3, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G2, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),

};

static uint32_t display_off_gpio_table[] = {
	/* Display */
	PCOM_GPIO_CFG(LEGEND_LCD_G5, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_DE, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_HSYNC, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_VSYNC, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B0, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B1, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),

	PCOM_GPIO_CFG(LEGEND_LCD_G1, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G0, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B4, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B3, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_B2, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G4, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G3, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
	PCOM_GPIO_CFG(LEGEND_LCD_G2, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),
};

void config_legend_display_on_gpios(void)
{
	D( "%s: enter.\n", __func__);
	config_gpio_table(display_on_gpio_table,
		ARRAY_SIZE(display_on_gpio_table));
}

void config_legend_display_off_gpios(void)
{
	D( "%s: enter.\n", __func__);
	config_gpio_table(display_off_gpio_table,
		ARRAY_SIZE(display_off_gpio_table));
}

static int lcm_spi_write(unsigned char add, unsigned char val)
{
	uint8_t buf[3] = {0, add, val};
	int ret = 0;

	if (!panel_type){
	        ret = microp_i2c_write(MICROP_I2C_WCMD_LCM_REGISTER, buf, 3);
	}
	else {
	        ret = microp_i2c_write(0x6F, buf, 3);
	}
	if (ret < 0) {
		D("%s: microp_spi_write fail\n", __func__);
		return ret;
	}
	return ret;
}

static int spi_write_tb(const LCD_INIT_TABLE init_table[], unsigned size)
{
	int i;

	for (i = 0; i < size; i++) {
		/* microp need 1 ms to finish its task */
		lcm_spi_write(init_table[i].reg, init_table[i].val);

		if (init_table[i].delay > 20)
			hr_msleep(init_table[i].delay);
		else if (init_table[i].delay)
			mdelay(init_table[i].delay);
	}
	return 0;
}

static int spi_write_gmtb(const LCD_INIT_TABLE init_table[], unsigned size)
{
	int i, ret = -1;
	uint8_t buf[18];

	for (i = 0; i < size; i++)
		buf[i] = init_table[i].val;

#ifdef CONFIG_MICROP_COMMON
	ret = microp_i2c_write(MICROP_I2C_WCMD_LCM_BURST, buf, 18);
	if (ret < 0) {
		D("%s: write gamma table failed\n", __func__);
		return ret;
	}
	buf[0] = 0x00;
	buf[1] = 0x01;
	ret = microp_i2c_write(MICROP_I2C_WCMD_LCM_BURST_EN, buf, 2);
	if (ret < 0) {
		D("%s: burst data enable failed\n", __func__);
		return ret;
	}
#endif

	return ret;
}


void set_backlight_level(int level)
{
	mutex_lock(&legend_backlight_lock);


	printk("set_backlight_level:%d\n", level);
	if (!panel_type) {
                if (level < 0 || level > GAMMA_LEVEL_MAX_SAMSUNG)
                        goto finish;
                spi_write_gmtb(gamma_level_table[level], ARRAY_SIZE(LCD_GAMMA_40_TABLE));
	} else {
                if (level < 0 || level > GAMMA_LEVEL_MAX_TPO)
	                goto finish;
                spi_write_tb(tpo_level_table[level], ARRAY_SIZE(TPO_100_TABLE));
	}
finish:
	mutex_unlock(&legend_backlight_lock);
}

static void set_brightness_work_func(struct work_struct *work)
{
	if (atomic_read(&gate) == 0)
		return;

	set_backlight_level(g_brightness_level);
}

static void legend_brightness_set(struct led_classdev *led_cdev,
				  enum led_brightness value)
{
	static uint8_t last_level = -1;
	uint8_t level;

	/* LED_VALUE_DEFAULT ~ LED_VALUE_MAX map to
	 GAMMA_LEVEL_DEFAULT ~ GAMMA_LEVEL_MAX */
	/* LED_VALUE_MIN ~ LED_VALUE_DEFAULT map to
	 0 ~ GAMMA_LEVEL_DEFAULT */
        if (!panel_type) {
                if (value > LED_VALUE_DEFAULT_SAMSUNG)
		        level = ((value - LED_VALUE_DEFAULT_SAMSUNG) *
			(GAMMA_LEVEL_MAX_SAMSUNG - GAMMA_LEVEL_DEFAULT_SAMSUNG) /
			(LED_VALUE_MAX_SAMSUNG  - LED_VALUE_DEFAULT_SAMSUNG)) +
                        GAMMA_LEVEL_DEFAULT_SAMSUNG;
                else if (value <= LED_VALUE_MIN_SAMSUNG)
                        level = GAMMA_LEVEL_MIN_SAMSUNG;
	else
		level = (value - LED_VALUE_MIN_SAMSUNG) *
			(GAMMA_LEVEL_DEFAULT_SAMSUNG - GAMMA_LEVEL_MIN_SAMSUNG)/
			(LED_VALUE_DEFAULT_SAMSUNG - LED_VALUE_MIN_SAMSUNG +1) + 1;
        } else {
                if (value > LED_VALUE_DEFAULT_TPO)
		        level = ((value - LED_VALUE_DEFAULT_TPO) *
			(GAMMA_LEVEL_MAX_TPO - GAMMA_LEVEL_DEFAULT_TPO) /
			(LED_VALUE_MAX_TPO  - LED_VALUE_DEFAULT_TPO)) +
                        GAMMA_LEVEL_DEFAULT_TPO;
                else if (value <= LED_VALUE_MIN_TPO)
                        level = GAMMA_LEVEL_MIN_TPO;
                else
		        level = (value - LED_VALUE_MIN_TPO) *
			(GAMMA_LEVEL_DEFAULT_TPO - GAMMA_LEVEL_MIN_TPO)/
			(LED_VALUE_DEFAULT_TPO - LED_VALUE_MIN_TPO +1) + 1;
	}
	D("%s:%d level:%d\n", __func__, value, level);
	mutex_lock(&legend_backlight_lock);

	if (level == last_level) {
		mutex_unlock(&legend_backlight_lock);
		return;
	}
	g_brightness_level = level;
	last_level = level;

	mutex_unlock(&legend_backlight_lock);

	schedule_work(&work_set_brightness);
}

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};


static int oled_panel_init(struct msm_lcdc_panel_ops *ops)
{
	unsigned id, on = 1;

	printk(KERN_DEBUG "%s: enter.\n", __func__);

	/* check RST pin value, 1 means already on, 0 means off */
	if(gpio_get_value(LEGEND_GPIO_LCD_RST_N))
	{
		/* delay and check again to filter ripple */
		mdelay(1);
		if(gpio_get_value(LEGEND_GPIO_LCD_RST_N))
		{
			printk(KERN_DEBUG "%s already init\n", __func__);
			return 0;
		}
	}


        if (!panel_type) {
	/* power on sequence */
	id = PM_VREG_PDOWN_AUX_ID;
	msm_proc_comm(PCOM_VREG_PULLDOWN, &on, &id);
	vreg_enable(vreg_lcm_1v8);
	hr_msleep(10);

	id = PM_VREG_PDOWN_RFRX2_ID;
	msm_proc_comm(PCOM_VREG_PULLDOWN, &on, &id);
	vreg_enable(vreg_lcm_2v8);
	hr_msleep(15);

	gpio_set_value(LEGEND_GPIO_LCD_RST_N, 1);
	hr_msleep(10);

	/* config LCDC pin mux to RGB function */
	config_legend_display_on_gpios();

        } else {
                gpio_set_value(LEGEND_GPIO_EL_EN, 0);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                gpio_set_value(LEGEND_GPIO_LCD_RST_N, 0);
                id = PM_VREG_PDOWN_AUX_ID;
                msm_proc_comm(PCOM_VREG_PULLDOWN, &on, &id);
                vreg_enable(vreg_lcm_1v8);
                hr_msleep(20);
                id = PM_VREG_PDOWN_RFRX2_ID;
                msm_proc_comm(PCOM_VREG_PULLDOWN, &on, &id);
                vreg_enable(vreg_lcm_2v8);
                hr_msleep(20);

                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                hr_msleep(3);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                hr_msleep(300);
                gpio_set_value(LEGEND_GPIO_LCD_RST_N, 1);
                hr_msleep(5);

		/* config LCDC pin mux to RGB function */
		config_legend_display_on_gpios();

        }

	return 0;
}

static int oled_panel_uninit(struct msm_lcdc_panel_ops *ops)
{
	unsigned id, off = 1;

	printk(KERN_DEBUG "%s: enter.\n", __func__);
	atomic_set(&gate, 0);
	if (!panel_type) {
	/* start power off sequence */
	/*Reset pull low*/
	gpio_set_value(LEGEND_GPIO_LCD_RST_N, 0);
	hr_msleep(100);

	/* config LCDC pin mux to gpio function */
	config_legend_display_off_gpios();
	gpio_set_value(LEGEND_LCD_B2,0);

	mdelay(15);
	/* turn off VCI */
	id = PM_VREG_PDOWN_RFRX2_ID;
	msm_proc_comm(PCOM_VREG_PULLDOWN, &off, &id);
	vreg_disable(vreg_lcm_2v8);

	/* turn off VDD3 */
	id = PM_VREG_PDOWN_AUX_ID;
	msm_proc_comm(PCOM_VREG_PULLDOWN, &off, &id);
	vreg_disable(vreg_lcm_1v8);
	mdelay(10);
	} else {
	        id = PM_VREG_PDOWN_RFRX2_ID;
                msm_proc_comm(PCOM_VREG_PULLDOWN, &off, &id);
	        vreg_disable(vreg_lcm_2v8);
                hr_msleep(20);
	        id = PM_VREG_PDOWN_AUX_ID;
	        msm_proc_comm(PCOM_VREG_PULLDOWN, &off, &id);
                vreg_disable(vreg_lcm_1v8);
		hr_msleep(20);

	}
	microp_spi_vote_enable(SPI_LCM, 0);
	return 0;
}

static int oled_panel_blank(struct msm_lcdc_panel_ops *op)
{
	printk(KERN_DEBUG "%s\n", __func__);
	atomic_set(&gate, 0);
	if (!panel_type) {
	/* start display off sequence */
	lcm_spi_write(0x4, 0x3);
	hr_msleep(100);
	lcm_spi_write(0x4, 0x1);
	hr_msleep(60);
	lcm_spi_write(0x4, 0x0);
	hr_msleep(100);
	lcm_spi_write(0x20, 0x0);
	} else {
		/* config LCDC pin mux to gpio function */
		config_legend_display_off_gpios();
		gpio_set_value(LEGEND_LCD_VSYNC, 0);
                gpio_set_value(LEGEND_LCD_HSYNC, 0);
                gpio_set_value(LEGEND_LCD_DE, 0);
                gpio_set_value(LEGEND_GPIO_EL_EN, 0);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                gpio_set_value(LEGEND_GPIO_LCD_RST_N, 0);
		hr_msleep(10);
		lcm_spi_write(0x28, 0x00);
		lcm_spi_write(0x10, 0x00);
		hr_msleep(120);
	}
	microp_spi_vote_enable(SPI_LCM, 0);
	return 0;
}

static int oled_panel_unblank(struct msm_lcdc_panel_ops *op)
{
	int ret;

	printk(KERN_DEBUG "%s type %d\n", __func__, panel_type);

	if (atomic_read(&gate)) {
		printk(KERN_DEBUG "already unblank\n");
		return 0;
	}

        if (!panel_type) {
		ret = microp_spi_vote_enable(SPI_LCM, 1);
		if (ret)
			printk(KERN_ERR "%s: enable SPI fail\n", __func__);

		spi_write_tb(OLED_POWER_SEQUENCE, ARRAY_SIZE(OLED_POWER_SEQUENCE));
		spi_write_tb(OLED_INIT_SEQUENCE, ARRAY_SIZE(OLED_INIT_SEQUENCE));

		/* display on sequence */
		lcm_spi_write(0x4, 0x5);
		hr_msleep(20);
		lcm_spi_write(0x4, 0x7);
		mutex_lock(&legend_backlight_lock);
		spi_write_gmtb(gamma_level_table[g_brightness_level], ARRAY_SIZE(LCD_GAMMA_40_TABLE));
		mutex_unlock(&legend_backlight_lock);
        } else {
		ret = microp_spi_vote_enable(SPI_LCM, 1);
		if (ret)
			printk(KERN_ERR "%s: enable SPI fail\n", __func__);
		lcm_spi_write(0x01, 0x00);
		hr_msleep(5);
		lcm_spi_write(0x3a, 0x55);
		lcm_spi_write(0x51, 0xff);
		lcm_spi_write(0x53, 0x34);
		lcm_spi_write(0x11, 0x00);
		hr_msleep(120);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                gpio_set_value(LEGEND_GPIO_EL_EN, 0);
                hr_msleep(3);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                hr_msleep(3);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 0);
                udelay(20);
                gpio_set_value(LEGEND_GPIO_EL_ADJ, 1);
                udelay(20);
		lcm_spi_write(0x29, 0x00);

		mutex_lock(&legend_backlight_lock);
                spi_write_tb(tpo_level_table[g_brightness_level], ARRAY_SIZE(TPO_100_TABLE));
		mutex_unlock(&legend_backlight_lock);
        }
	atomic_set(&gate, 1);


	return 0;
}

static struct msm_lcdc_panel_ops legend_lcdc_panel_ops = {
	.init = oled_panel_init,
	.uninit = oled_panel_uninit,
	.blank = oled_panel_blank,
	.unblank = oled_panel_unblank,
};

#define CLK_NS_TO_RATE(ns)	(1000000000UL / (ns))
static struct msm_lcdc_timing legend_lcdc_timing = {
	.clk_rate = CLK_NS_TO_RATE(75),
	.hsync_pulse_width = 4,
	.hsync_back_porch = 60,
	.hsync_front_porch = 64,
	.hsync_skew = 0,
	.vsync_pulse_width = 4,
	.vsync_back_porch = 4,
	.vsync_front_porch = 8,
	.vsync_act_low = 1,
	.hsync_act_low = 1,
	.den_act_low = 1,
};

static struct msm_fb_data legend_lcdc_fb_data = {
	.xres = 320,
	.yres = 480,
	.width = 67,
	.height = 45,
	.output_format = 0,
};

static struct msm_lcdc_platform_data legend_lcdc_platform_data = {
	.panel_ops = &legend_lcdc_panel_ops,
	.timing = &legend_lcdc_timing,
	.fb_id = 0,
	.fb_data = &legend_lcdc_fb_data,
	.fb_resource = &resources_msm_fb[0],
};

static struct platform_device legend_lcdc_device = {
	.name = "msm_mdp_lcdc",
	.id = -1,
	.dev = {
		.platform_data = &legend_lcdc_platform_data,
		},
};

static struct msm_mdp_platform_data mdp_pdata = {
       .overrides = MSM_MDP_DMA_PACK_ALIGN_LSB,
};

static struct led_classdev legend_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = legend_brightness_set,
};

static int legend_backlight_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &legend_backlight_led);
	if (rc) {
		printk("backlight: failure on register led_classdev\n");
	}
	INIT_WORK(&work_set_brightness, set_brightness_work_func);

	return 0;
}

static int legend_backlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&legend_backlight_led);
	return 0;
}

static struct platform_device legend_backlight = {
	.name = "legend-backlight",
};

static struct platform_driver legend_backlight_driver = {
	.probe = legend_backlight_probe,
	.remove = legend_backlight_remove,
	.driver = {
		   .name = "legend-backlight",
		   .owner = THIS_MODULE,
	},
};

int __init legend_init_backlight(void)
{
	int ret;
	ret = platform_driver_register(&legend_backlight_driver);
	if (ret)
		return ret;
	atomic_set(&gate, 1);
	return 0;
}

int __init legend_init_panel(void)
{
	int ret;

	vreg_lcm_1v8 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_1v8))
		return PTR_ERR(vreg_lcm_1v8);

	vreg_lcm_2v8 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcm_2v8))
		return PTR_ERR(vreg_lcm_2v8);

	msm_device_mdp.dev.platform_data = &mdp_pdata;

	ret = platform_device_register(&msm_device_mdp);
	if (ret != 0)
		return ret;

	wake_lock_init(&backlight_wakelock, WAKE_LOCK_SUSPEND,
		       "backlight_present");

	printk("panel_type %d\n", panel_type);

        /* FIXME: This is a lcdc timing highjack for different panel */
        if (panel_type)
		((struct msm_lcdc_platform_data *)
                legend_lcdc_device.dev.platform_data)->timing->den_act_low = 0;

	ret = platform_device_register(&legend_lcdc_device);
	if (ret != 0)
		return ret;

	platform_device_register(&legend_backlight);

	return 0;
}

device_initcall(legend_init_backlight);
