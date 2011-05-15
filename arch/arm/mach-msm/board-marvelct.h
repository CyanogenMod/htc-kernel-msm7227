/* linux/arch/arm/mach-msm/board-marvelct.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_MARVELCT_H
#define __ARCH_ARM_MACH_MSM_BOARD_MARVELCT_H

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

#define MARVELCT_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */

#define MARVELCT_GPIO_USB_ID_PIN          (19)
#define MARVELCT_POWER_KEY                (20)
#define MARVELCT_GPIO_MSM_PS_HOLD         (25)
#define MARVELCT_GPIO_WIFI_IRQ            (29)
#define MARVELCT_GPIO_RESET_BTN_N         (36)
#define MARVELCT_GPIO_SDMC_CD_N           (38)
#define MARVELCT_GPIO_UP_INT_N            (39)
#define MARVELCT_GPIO_CHG_INT		(40)
#define MARVELCT_GPIO_VOL_DOWN            (49)

/* WLAN SD data */
#define MARVELCT_GPIO_SD_D3               (51)
#define MARVELCT_GPIO_SD_D2               (52)
#define MARVELCT_GPIO_SD_D1               (53)
#define MARVELCT_GPIO_SD_D0               (54)
#define MARVELCT_GPIO_SD_CMD              (55)
#define MARVELCT_GPIO_SD_CLK_0            (56)

/* I2C */
#define MARVELCT_GPIO_I2C_SCL             (60)
#define MARVELCT_GPIO_I2C_SDA             (61)

/* MicroSD */
#define MARVELCT_GPIO_SDMC_CLK_1          (62)
#define MARVELCT_GPIO_SDMC_CMD            (63)
#define MARVELCT_GPIO_SDMC_D3             (64)
#define MARVELCT_GPIO_SDMC_D2             (65)
#define MARVELCT_GPIO_SDMC_D1             (66)
#define MARVELCT_GPIO_SDMC_D0             (67)

/* BT PCM */
#define MARVELCT_GPIO_AUD_PCM_DO          (68)
#define MARVELCT_GPIO_AUD_PCM_DI          (69)
#define MARVELCT_GPIO_AUD_PCM_SYNC        (70)
#define MARVELCT_GPIO_AUD_PCM_CLK         (71)

#define MARVELCT_GPIO_UP_RESET_N          (76)
#define MARVELCT_GPIO_FLASHLIGHT          (85)
#define MARVELCT_GPIO_UART3_RX            (86)
#define MARVELCT_GPIO_UART3_TX            (87)
#define MARVELCT_GPIO_VOL_UP              (92)
#define MARVELCT_GPIO_LS_EN               (93)
#define MARVELCT_GPIO_WIFI_EN             (108)
#define MARVELCT_GPIO_V_USBPHY_3V3_EN     (109)
#define MARVELCT_GPIO_VIBRATOR_ON         (116)

/* Compass  */
#define MARVELCT_GPIO_COMPASS_RDY         (37)
#define MARVELCT_LAYOUTS			{ \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0, -1} }, \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0,  1} }, \
		{ { -1,  0, 0}, {  0,  1, 0}, {0, 0, -1} }, \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1,  0} }  \
					}

/* Proximity  */
#define MARVELCT_GPIO_PROXIMITY_INT       (21)
#define MARVELCT_GPIO_PROXIMITY_EN        (119)

/* BT */
#define MARVELCT_GPIO_BT_UART1_RTS        (43)
#define MARVELCT_GPIO_BT_UART1_CTS        (44)
#define MARVELCT_GPIO_BT_UART1_RX         (45)
#define MARVELCT_GPIO_BT_UART1_TX         (46)
#define MARVELCT_GPIO_BT_RESET_N          (90)
#define MARVELCT_GPIO_BT_HOST_WAKE        (112)
#define MARVELCT_GPIO_BT_CHIP_WAKE        (122)
#define MARVELCT_GPIO_BT_SD_N             (123)

/* Touch Panel */
#define MARVELCT_GPIO_TP_ATT_N            (18)
#define MARVELCT_V_TP_3V3_EN              (31)
#define MARVELCT_LCD_RSTz		        (118)
#define MARVELCT_GPIO_TP_RST_N            (120)

/* 35mm headset */
#define MARVELCT_GPIO_35MM_HEADSET_DET    (83)

/*Camera AF VCM POWER*/
#define MARVELCT_GPIO_VCM_PD              (126)
#define MARVELCT_GPIO_CAM_RST_N           (125)

/*Display*/
#define MARVELCT_GPIO_LCD_ID0             (34)
#define MARVELCT_GPIO_LCD_ID1             (35)
#define MARVELCT_GPIO_MDDI_TE             (97)
#define MARVELCT_GPIO_LCD_RST_N           (118)

int __init marvelct_init_keypad(void);
int marvelct_init_mmc(unsigned int sys_rev);
int __init marvelct_init_panel(void);
int __init marvelct_wifi_init(void);
#endif /* GUARD */

