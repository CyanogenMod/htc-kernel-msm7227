/* linux/arch/arm/mach-msm/board-marvel.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_MARVEL_H
#define __ARCH_ARM_MACH_MSM_BOARD_MARVEL_H

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

#define MARVEL_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */

#define MARVEL_GPIO_USB_ID_PIN          (19)
#define MARVEL_POWER_KEY                (20)
#define MARVEL_GPIO_MSM_PS_HOLD         (25)
#define MARVEL_GPIO_WIFI_IRQ            (29)
#define MARVEL_GPIO_RESET_BTN_N         (36)
#define MARVEL_GPIO_SDMC_CD_N           (38)
#define MARVEL_GPIO_UP_INT_N            (39)
#define MARVEL_GPIO_CHG_INT		(40)
#define MARVEL_GPIO_VOL_DOWN            (49)

/* WLAN SD data */
#define MARVEL_GPIO_SD_D3               (51)
#define MARVEL_GPIO_SD_D2               (52)
#define MARVEL_GPIO_SD_D1               (53)
#define MARVEL_GPIO_SD_D0               (54)
#define MARVEL_GPIO_SD_CMD              (55)
#define MARVEL_GPIO_SD_CLK_0            (56)

/* I2C */
#define MARVEL_GPIO_I2C_SCL             (60)
#define MARVEL_GPIO_I2C_SDA             (61)

/* MicroSD */
#define MARVEL_GPIO_SDMC_CLK_1          (62)
#define MARVEL_GPIO_SDMC_CMD            (63)
#define MARVEL_GPIO_SDMC_D3             (64)
#define MARVEL_GPIO_SDMC_D2             (65)
#define MARVEL_GPIO_SDMC_D1             (66)
#define MARVEL_GPIO_SDMC_D0             (67)

/* BT PCM */
#define MARVEL_GPIO_AUD_PCM_DO          (68)
#define MARVEL_GPIO_AUD_PCM_DI          (69)
#define MARVEL_GPIO_AUD_PCM_SYNC        (70)
#define MARVEL_GPIO_AUD_PCM_CLK         (71)

#define MARVEL_GPIO_UP_RESET_N          (76)
#define MARVEL_GPIO_FLASHLIGHT          (85)
#define MARVEL_GPIO_UART3_RX            (86)
#define MARVEL_GPIO_UART3_TX            (87)
#define MARVEL_GPIO_VOL_UP              (92)
#define MARVEL_GPIO_LS_EN               (93)
#define MARVEL_GPIO_WIFI_EN             (108)
#define MARVEL_GPIO_V_USBPHY_3V3_EN     (109)
#define MARVEL_GPIO_VIBRATOR_ON         (116)

/* Compass  */
#define MARVEL_GPIO_COMPASS_RDY         (37)
#define MARVEL_LAYOUTS			{ \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0, -1} }, \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0,  1} }, \
		{ { -1,  0, 0}, {  0,  1, 0}, {0, 0, -1} }, \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1,  0} }  \
					}

/* Proximity  */
#define MARVEL_GPIO_PROXIMITY_INT       (21)
#define MARVEL_GPIO_PROXIMITY_EN        (119)

/* BT */
#define MARVEL_GPIO_BT_UART1_RTS        (43)
#define MARVEL_GPIO_BT_UART1_CTS        (44)
#define MARVEL_GPIO_BT_UART1_RX         (45)
#define MARVEL_GPIO_BT_UART1_TX         (46)
#define MARVEL_GPIO_BT_RESET_N          (90)
#define MARVEL_GPIO_BT_HOST_WAKE        (112)
#define MARVEL_GPIO_BT_CHIP_WAKE        (122)
#define MARVEL_GPIO_BT_SD_N             (123)

/* Touch Panel */
#define MARVEL_GPIO_TP_ATT_N            (18)
#define MARVEL_V_TP_3V3_EN              (31)
#define MARVEL_LCD_RSTz		        (118)
#define MARVEL_GPIO_TP_RST_N            (120)

/* 35mm headset */
#define MARVEL_GPIO_35MM_HEADSET_DET    (83)

/*Camera AF VCM POWER*/
#define MARVEL_GPIO_VCM_PD              (126)
#define MARVEL_GPIO_CAM_RST_N           (125)

/*Display*/
#define MARVEL_GPIO_LCD_ID0             (34)
#define MARVEL_GPIO_LCD_ID1             (35)
#define MARVEL_GPIO_MDDI_TE             (97)
#define MARVEL_GPIO_LCD_RST_N           (118)

int __init marvel_init_keypad(void);
int marvel_init_mmc(unsigned int sys_rev);
int __init marvel_init_panel(void);
int __init marvel_wifi_init(void);
#endif /* GUARD */

