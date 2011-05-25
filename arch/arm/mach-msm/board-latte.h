/* linux/arch/arm/mach-msm/board-latte.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_LATTE_H
#define __ARCH_ARM_MACH_MSM_BOARD_LATTE_H

#include <mach/board.h>

#define MSM_MEM_BASE		0x10000000
#define MSM_MEM_SIZE		0x20000000

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

#define LATTE_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */
#define LATTE_GPIO_USB_ID_PIN          (19)
#define LATTE_POWER_KEY                (20)
#define LATTE_GPIO_WIFI_EN             (108)
#define LATTE_GPIO_SDMC_CD_N           (38)
#define LATTE_GPIO_UART3_RX            (86)
#define LATTE_GPIO_UART3_TX            (87)
#define LATTE_GPIO_PS_HOLD             (25)
#define LATTE_GPIO_UP_RESET_N          (76)
#define LATTE_GPIO_CAP_LED_EN          (121)
#define LATTE_GPIO_USBPHY_3V3_EN       (109)
#define LATTE_GPIO_FUNC_LED_EN         (0)
#define LATTE_GPIO_LS_EN               (93)
#define LATTE_GPIO_UP_INT_N            (39)
#define LATTE_GPIO_COMPASS_INT_N       (36)
#define LATTE_GPIO_COMPASS_RST_N       (84)
#define LATTE_PROJECT_NAME		"latte"
#define LATTE_LAYOUTS			{ \
		{ { -1,  0, 0}, {  0, -1, 0}, {0, 0, 1} },  \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0, 1} },  \
		{ {  0, -1, 0}, {  1,  0, 0}, {0, 0, 1} },  \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1, 0} }   \
					}

/* Proximity  */
#define LATTE_GPIO_PROXIMITY_INT       (21)
#define LATTE_GPIO_PROXIMITY_EN        (58)

/* Navi key output/input matrix */
#define LATTE_GPIO_KP_MKOUT7           (116)
#define LATTE_GPIO_KP_MKOUT6           (117)
#define LATTE_GPIO_KP_MKOUT5           (118)
#define LATTE_GPIO_KP_MKOUT4           (119)
#define LATTE_GPIO_KP_MKOUT3           (120)
#define LATTE_GPIO_KP_MKOUT2           (33)
#define LATTE_GPIO_KP_MKOUT1           (34)
#define LATTE_GPIO_KP_MKOUT0           (35)
#define LATTE_GPIO_KP_MKIN6            (112)
#define LATTE_GPIO_KP_MKIN5            (114)
#define LATTE_GPIO_KP_MKIN4            (17)
#define LATTE_GPIO_KP_MKIN3            (92)
#define LATTE_GPIO_KP_MKIN2            (40)
#define LATTE_GPIO_KP_MKIN1            (41)
#define LATTE_GPIO_KP_MKIN0            (42)

/* sliding keyboard detect */
#define LATTE_GPIO_SLIDING_DET         (49)

/* BT */
#define LATTE_GPIO_UART1_RTS        (43)
#define LATTE_GPIO_UART1_CTS        (44)
#define LATTE_GPIO_UART1_RX         (45)
#define LATTE_GPIO_UART1_TX         (46)
#define LATTE_GPIO_BT_EN               (90)

/* Touch Panel */
#define LATTE_TP_5V_EN                 (31)
#define LATTE_GPIO_TP_RST              (57)
#define LATTE_TP_LS_EN                 (91)
#define LATTE_GPIO_TP_ATT_N            (94)

/* 35mm headset */
#define LATTE_GPIO_35MM_HEADSET_DET    (83)

/* Camera AF VCM POWER*/
#define LATTE_GPIO_VCM_PWDN            (82)
#define LATTE_GPIO_CAM1_RST_N          (115)

/* Other */
#define LATTE_GPIO_BAT_DET_N           (28)
#define LATTE_GPIO_MOT_INT_CPU         (18)
#define LATTE_GPIO_CAM_I2C_SDA         (27)
#define LATTE_GPIO_UP_INT2_N           (37)
#define LATTE_GPIO_CAM_I2C_SCL         (124)
#define LATTE_GPIO_FLASH_EN            (122)
#define LATTE_GPIO_TORCH_EN            (123)

/* Display */
#define LATTE_GPIO_LCD_VSYNC           (97)
#define LATTE_GPIO_LCD_ID1             (130)
#define LATTE_LCD_RSTz                 (131)
#define LATTE_GPIO_LCD_ID0             (132)

/* LED */
#define LATTE_GPIO_LED_FN_LED_EN       (128)
#define LATTE_GPIO_LED_CAP_LED_EN      (129)

int __init latte_init_keypad(void);
int latte_init_mmc(unsigned int sys_rev);
#endif /* GUARD */

