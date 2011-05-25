/* linux/arch/arm/mach-msm/board-liberty.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_LIBERTY_H
#define __ARCH_ARM_MACH_MSM_BOARD_LIBERTY_H

#include <mach/board.h>

#define MSM_MEM_BASE		0x10000000
#define MSM_MEM_SIZE		0x18000000

#define MSM_LINUX_BASE_OFFSET	0x02C00000

#define MSM_MM_HEAP_SIZE        0x02800000

#define MSM_LINUX_BASE          (MSM_MEM_BASE + MSM_LINUX_BASE_OFFSET) /* 2MB alignment */
#define MSM_LINUX_SIZE          (MSM_MEM_SIZE - MSM_LINUX_BASE_OFFSET - MSM_MM_HEAP_SIZE)

#define MSM_FB_BASE             (MSM_MEM_BASE + 0x02A00000)
#define MSM_FB_SIZE             0x00200000

#define MSM_GPU_MEM_BASE        (MSM_MEM_BASE + MSM_MEM_SIZE - MSM_MM_HEAP_SIZE)
#define MSM_GPU_MEM_SIZE        0x00300000

#define MSM_PMEM_MDP_BASE       MSM_GPU_MEM_BASE + MSM_GPU_MEM_SIZE
#define MSM_PMEM_MDP_SIZE       0x01000000

#define MSM_PMEM_ADSP_BASE      MSM_PMEM_MDP_BASE + MSM_PMEM_MDP_SIZE
#define MSM_PMEM_ADSP_SIZE      0x00C1B000

#define MSM_PMEM_CAMERA_BASE    MSM_PMEM_ADSP_BASE + MSM_PMEM_ADSP_SIZE
#define MSM_PMEM_CAMERA_SIZE    0x00800000

/* TODO: To save space, we can move RAM_CONSOLE to 0x00000000 */
#define MSM_RAM_CONSOLE_BASE    MSM_PMEM_CAMERA_BASE + MSM_PMEM_CAMERA_SIZE
#define MSM_RAM_CONSOLE_SIZE    128 * SZ_1K

#define LIBERTY_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */

#define LIBERTY_GPIO_UP_INT2_N           (18)
#define LIBERTY_GPIO_USB_ID_PIN          (19)
#define LIBERTY_POWER_KEY                (20)
#define LIBERTY_GPIO_PS_HOLD             (25)
#define LIBERTY_GPIO_CAM_I2C_SDA         (27)
#define LIBERTY_GPIO_WIFI_IRQ1           (29)
#define LIBERTY_GPIO_USB_ISET            (30)
#define LIBERTY_GPIO_RESET_BTN_N         (36)
#define LIBERTY_GPIO_COMPASS_INT_N       (37)
#define LIBERTY_GPIO_SDMC_CD_N           (38)
#define LIBERTY_GPIO_UP_INT_N            (39)
#define LIBERTY_GPIO_CAM_I2C_SCL         (49)

/* WLAN SD data */
#define LIBERTY_GPIO_SD_D3               (51)
#define LIBERTY_GPIO_SD_D2               (52)
#define LIBERTY_GPIO_SD_D1               (53)
#define LIBERTY_GPIO_SD_D0               (54)
#define LIBERTY_GPIO_SD_CMD              (55)
#define LIBERTY_GPIO_SD_CLK_0            (56)

/* I2C */
#define LIBERTY_GPIO_I2C_SCL             (60)
#define LIBERTY_GPIO_I2C_SDA             (61)

/* MicroSD */
#define LIBERTY_GPIO_SDMC_CLK_0          (62)
#define LIBERTY_GPIO_SDMC_CMD            (63)
#define LIBERTY_GPIO_SDMC_D3             (64)
#define LIBERTY_GPIO_SDMC_D2             (65)
#define LIBERTY_GPIO_SDMC_D1             (66)
#define LIBERTY_GPIO_SDMC_D0             (67)

/* BT PCM */
#define LIBERTY_GPIO_AUD_PCM_DO          (68)
#define LIBERTY_GPIO_AUD_PCM_DI          (69)
#define LIBERTY_GPIO_AUD_PCM_SYNC        (70)
#define LIBERTY_GPIO_AUD_PCM_CLK         (71)

#define LIBERTY_GPIO_UP_RESET_N          (76)
#define LIBERTY_GPIO_UART3_RX            (86)
#define LIBERTY_GPIO_UART3_TX            (87)
#define LIBERTY_GPIO_VIB_3V_EN           (92)
#define LIBERTY_GPIO_LS_EN               (93)
#define LIBERTY_GPIO_COMPASS_RST_N       (107)
#define LIBERTY_GPIO_WIFI_EN             (108)
#define LIBERTY_GPIO_USBPHY_3V3_EN       (109)

#define LIBERTY_PROJECT_NAME		"liberty"
#define LIBERTY_LAYOUTS			{ \
		{ { -1,  0, 0}, {  0, -1, 0}, {0, 0, 1} },  \
		{ { 0, -1, 0}, { 1,  0, 0}, {0, 0, -1} },  \
		{ { 0, -1, 0}, {  1, 0, 0}, {0, 0, 1} },  \
		{ { -1,  0, 0}, {  0,  0, -1}, {0, 1,  0} }   \
					}

/* Proximity  */
#define LIBERTY_GPIO_PROXIMITY_INT       (21)
#define LIBERTY_GPIO_PROXIMITY_EN        (119)

/* Navi key output/input matrix */
#define LIBERTY_GPIO_KP_MKOUT2           (33)
#define LIBERTY_GPIO_KP_MKOUT1           (34)
#define LIBERTY_GPIO_KP_MKOUT0           (35)
#define LIBERTY_GPIO_KP_MKIN2            (40)
#define LIBERTY_GPIO_KP_MKIN1            (41)
#define LIBERTY_GPIO_KP_MKIN0            (42)

/* BT */
#define LIBERTY_GPIO_BT_UART1_RTS        (43)
#define LIBERTY_GPIO_BT_UART1_CTS        (44)
#define LIBERTY_GPIO_BT_UART1_RX         (45)
#define LIBERTY_GPIO_BT_UART1_TX         (46)
#define LIBERTY_GPIO_BT_RESET_N          (90)
#define LIBERTY_GPIO_BT_HOST_WAKE        (112)
#define LIBERTY_GPIO_BT_CHIP_WAKE        (122)
#define LIBERTY_GPIO_BT_SHUTDOWN_N       (123)

/* Touch Panel */
#define LIBERTY_TP_5V_EN                 (31)
#define LIBERTY_TP_LS_EN                 (91)
#define LIBERTY_GPIO_TP_ATT_N            (94)
#define LIBERTY_LCD_RSTz		 (118)
#define LIBERTY_GPIO_TP_RST              (120)

/* 35mm headset */
#define LIBERTY_GPIO_35MM_HEADSET_DET    (83)

/*Camera AF VCM POWER*/
#define LIBERTY_GPIO_VCM_PWDN            (82)
#define LIBERTY_GPIO_CAM1_RST_N          (121)

/*Display*/
#define LIBERTY_GPIO_LCD_ID0             (57)
#define LIBERTY_GPIO_LCD_ID1             (58)
#define LIBERTY_GPIO_LCD_VSYNC           (97)
#define LIBERTY_GPIO_LCD_RST_N           (118)

int __init liberty_init_keypad(void);
int liberty_init_mmc(unsigned int sys_rev);
int __init liberty_init_panel(void);
#endif /* GUARD */

