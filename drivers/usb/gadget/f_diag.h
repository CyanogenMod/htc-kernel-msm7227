/* drivers/usb/gadget/f_diag.h
 *
 * Diag Function Device - Route DIAG frames between SMD and USB
 *
 * Copyright (C) 2008-2009 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#ifndef __F_DIAG_H
#define __F_DIAG_H

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>


#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
extern	int sdio_diag_init_enable;
#endif

int checkcmd_modem_epst(unsigned char *buf);
int modem_to_userspace(void *buf, int r, int cmdtype, int is9k);

void msm_sdio_diag_write(void *data, int len);
void sdio_diag_read_data(struct work_struct *work);
void diag_sdio_mdm_send_req(int context);
#endif /* __F_DIAG_H */

