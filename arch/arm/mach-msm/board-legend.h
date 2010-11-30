/* linux/arch/arm/mach-msm/board-legend.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_LEGEND_H
#define __ARCH_ARM_MACH_MSM_BOARD_LEGEND_H

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

#define LEGEND_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */

#define LEGEND_GPIO_USB_ID_PIN          (19) /*for legend USB ID pin*/

#define LEGEND_POWER_KEY                (20)
#define LEGEND_GPIO_WIFI_EN             (108)
#define LEGEND_GPIO_SDMC_CD_N           (38)
#define LEGEND_GPIO_UART3_RX            (86)
#define LEGEND_GPIO_UART3_TX            (87)
#define LEGEND_GPIO_PS_HOLD             (25)
#define LEGEND_GPIO_UP_RESET_N          (76)
#define LEGEND_GPIO_CAP_LED_EN          (121)
#define LEGEND_GPIO_USBPHY_3V3_EN       (109)
#define LEGEND_GPIO_FUNC_LED_EN         (0)
#define LEGEND_GPIO_LS_EN               (93)
#define LEGEND_GPIO_UP_INT_N            (39)
#define LEGEND_GPIO_COMPASS_INT_N       (36)
#define LEGEND_GPIO_COMPASS_RST_N       (92)
#define LEGEND_PROJECT_NAME		"legend"
#define LEGEND_LAYOUTS			{ \
		{ {-1,  0, 0}, {  0,  1, 0}, {0, 0, -1} },  \
		{ { 0, -1, 0}, { -1,  0, 0}, {0, 0,  1} },  \
		{ { 0,  1, 0}, {  1,  0, 0}, {0, 0, -1} },  \
		{ { 1,  0, 0}, {  0,  0, 1}, {0, 1,  0} }   \
					}

/* Proximity  */
#define LEGEND_GPIO_PROXIMITY_INT       (21)
#define LEGEND_GPIO_PROXIMITY_EN        (58)

/* Navi key output/input matrix */
#define LEGEND_GPIO_KP_MKOUT2           (33)
#define LEGEND_GPIO_KP_MKOUT1           (34)
#define LEGEND_GPIO_KP_MKOUT0           (35)
#define LEGEND_GPIO_KP_MKIN2            (40)
#define LEGEND_GPIO_KP_MKIN1            (41)
#define LEGEND_GPIO_KP_MKIN0            (42)

/* BT */
#define LEGEND_GPIO_UART1_RTS           (43)
#define LEGEND_GPIO_UART1_CTS           (44)
#define LEGEND_GPIO_UART1_RX            (45)
#define LEGEND_GPIO_UART1_TX            (46)
#define LEGEND_GPIO_BT_EN               (90)

/* Touch Panel */
#define LEGEND_TP_5V_EN                 (31)
#define LEGEND_GPIO_TP_RST              (57)
#define LEGEND_TP_LS_EN                 (91)
#define LEGEND_GPIO_TP_ATT_N            (94)

/* 35mm headset */
#define LEGEND_GPIO_35MM_HEADSET_DET    (83)

/*Camera AF VCM POWER*/
#define LEGEND_GPIO_VCM_PWDN            (82)
#define LEGEND_GPIO_CAM1_RST_N          (97)

/*Display*/

#define LEGEND_LCD_G1                   (111)
#define LEGEND_LCD_G0                   (112)

#define LEGEND_LCD_B4                   (116)
#define LEGEND_LCD_B3                   (117)
#define LEGEND_LCD_B2                   (118)
#define LEGEND_LCD_G4                   (119)
#define LEGEND_LCD_G3                   (120)
#define LEGEND_LCD_G2                   (121)

#define LEGEND_LCD_B1                   (125)
#define LEGEND_LCD_B0                   (126)
#define LEGEND_LCD_VSYNC                (127)
#define LEGEND_LCD_HSYNC                (128)
#define LEGEND_LCD_DE                   (129)
#define LEGEND_LCD_G5                   (130)
#define LEGEND_LCD_RSTz_ID1             (131)
#define LEGEND_LCD_ID0                  (132)

/* for new XA board */
#define LEGEND_GPIO_BAT_DET_N           (17)
#define LEGEND_GPIO_MOT_INT_CPU         (18)
#define LEGEND_GPIO_CAM_I2C_SDA         (27)
#define LEGEND_GPIO_UP_INT2_N           (37)
#define LEGEND_GPIO_CAM_I2C_SCL         (49)
#define LEGEND_GPIO_EL_ADJ              (84)
#define LEGEND_GPIO_FLASH_EN            (122)
#define LEGEND_GPIO_TORCH_EN            (123)
#define LEGEND_GPIO_EL_EN               (124)
#define LEGEND_GPIO_LCD_RST_N           (131)

int __init legend_init_keypad(void);
int legend_init_mmc(unsigned int sys_rev);
int __init legend_init_panel(void);
#endif /* GUARD */

