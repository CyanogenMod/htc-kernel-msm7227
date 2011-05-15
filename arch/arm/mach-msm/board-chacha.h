/* linux/arch/arm/mach-msm/board-chacha.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_CHACHA_H
#define __ARCH_ARM_MACH_MSM_BOARD_CHACHA_H

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

#define CHACHA_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */

#define CHACHA_GPIO_USB_ID_PIN          (19)
#define CHACHA_GPIO_MSM_PS_HOLD         (25)
#define CHACHA_GPIO_WIFI_IRQ            (29)
#define CHACHA_GPIO_SDMC_CD_N           (38)
#define CHACHA_GPIO_UP_INT_N            (39)
#define CHACHA_GPIO_CHG_INT		(40)

/* WLAN SD data */
#define CHACHA_GPIO_SD_D3               (51)
#define CHACHA_GPIO_SD_D2               (52)
#define CHACHA_GPIO_SD_D1               (53)
#define CHACHA_GPIO_SD_D0               (54)
#define CHACHA_GPIO_SD_CMD              (55)
#define CHACHA_GPIO_SD_CLK_0            (56)

/* I2C */
#define CHACHA_GPIO_I2C_SCL             (60)
#define CHACHA_GPIO_I2C_SDA             (61)

/* MicroSD */
#define CHACHA_GPIO_SDMC_CLK_1          (62)
#define CHACHA_GPIO_SDMC_CMD            (63)
#define CHACHA_GPIO_SDMC_D3             (64)
#define CHACHA_GPIO_SDMC_D2             (65)
#define CHACHA_GPIO_SDMC_D1             (66)
#define CHACHA_GPIO_SDMC_D0             (67)

/* BT PCM */
#define CHACHA_GPIO_AUD_PCM_DO          (68)
#define CHACHA_GPIO_AUD_PCM_DI          (69)
#define CHACHA_GPIO_AUD_PCM_SYNC        (70)
#define CHACHA_GPIO_AUD_PCM_CLK         (71)

#define CHACHA_GPIO_UP_RESET_N          (76)
#define CHACHA_GPIO_UART3_RX            (86)
#define CHACHA_GPIO_UART3_TX            (87)
#define CHACHA_GPIO_LS_EN               (93)
#define CHACHA_GPIO_WIFI_EN             (108)
#define CHACHA_GPIO_V_USBPHY_3V3_EN     (109)
#define CHACHA_GPIO_VIBRATOR_ON         (116)

/* Compass  */
#define CHACHA_GPIO_COMPASS_RDY         (37)
#define CHACHA_LAYOUTS_XA		{ \
		{ {  0,  1, 0}, { -1,  0,  0}, {0, 0,  1} }, \
		{ {  0, -1, 0}, {  1,  0,  0}, {0, 0, -1} }, \
		{ { -1,  0, 0}, {  0, -1,  0}, {0, 0,  1} }, \
		{ { -1,  0, 0}, {  0,  0, -1}, {0, 1,  0} }  \
					}
#define CHACHA_LAYOUTS_XB		{ \
		{ {  0,  1, 0}, { -1,  0, 0}, {0, 0,  1} }, \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0,  1} }, \
		{ { -1,  0, 0}, {  0, -1, 0}, {0, 0,  1} }, \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1,  0} }  \
					}

/* Proximity  */
#define CHACHA_GPIO_PROXIMITY_INT       (21)
#define CHACHA_GPIO_PROXIMITY_EN        (119)

/* BT */
#define CHACHA_GPIO_BT_UART1_RTS        (43)
#define CHACHA_GPIO_BT_UART1_CTS        (44)
#define CHACHA_GPIO_BT_UART1_RX         (45)
#define CHACHA_GPIO_BT_UART1_TX         (46)
#define CHACHA_GPIO_BT_RESET_N          (90)
#define CHACHA_GPIO_BT_HOST_WAKE        (112)
#define CHACHA_GPIO_BT_CHIP_WAKE        (122)
#define CHACHA_GPIO_BT_SD_N             (123)

/* Touch Panel */
#define CHACHA_GPIO_TP_ATT_N            (18)
#define CHACHA_V_TP_3V3_EN              (31)
#define CHACHA_LCD_RSTz		        (118)
#define CHACHA_GPIO_TP_RST_N            (120)

/* 35mm headset */
#define CHACHA_GPIO_35MM_HEADSET_DET    (83)

/*Camera AF VCM POWER*/
#define CHACHA_GPIO_VCM_PD              (126)
#define CHACHA_GPIO_CAM_RST_N           (125)
#define CHACHA_GPIO_CAM_SEL             (127)
#define CHACHA_GPIO_CAM2_PWDN           (128)
#define CHACHA_GPIO_CAM2_RST_N          (129)

/*Display*/
#define CHACHA_GPIO_LCD_ID0             (34)
#define CHACHA_GPIO_LCD_ID1             (35)
#define CHACHA_GPIO_MDDI_TE             (97)
#define CHACHA_GPIO_LCD_RST_N           (118)

/* keypad */
#define CHACHA_POWER_KEY                (20)
#define CHACHA_GPIO_KEY_DIRECT          (36)
#define CHACHA_GPIO_KP_MKIN0            (94)
#define CHACHA_GPIO_KP_MKIN1            (41)
#define CHACHA_GPIO_KP_MKIN2            (27)
#define CHACHA_GPIO_KP_MKIN3            (92)
#define CHACHA_GPIO_KP_MKIN4            (17)
#define CHACHA_GPIO_KP_MKIN5            (114)
#define CHACHA_GPIO_KP_MKIN6            (49)
#define CHACHA_GPIO_KP_MKOUT0           (32)
#define CHACHA_GPIO_KP_MKOUT1           (115)
#define CHACHA_GPIO_KP_MKOUT2           (33)
#define CHACHA_GPIO_KP_MKOUT3           (121)
#define CHACHA_GPIO_KP_MKOUT4           (111)
#define CHACHA_GPIO_KP_MKOUT5           (131)
#define CHACHA_GPIO_KP_MKOUT6           (117)

/* Flashlight */
#define CHACHA_GPIO_FLASHLIGHT          (85)

int __init chacha_init_keypad(void);
int chacha_init_mmc(unsigned int sys_rev);
int __init chacha_init_panel(void);
int __init chacha_wifi_init(void);
#endif /* GUARD */

