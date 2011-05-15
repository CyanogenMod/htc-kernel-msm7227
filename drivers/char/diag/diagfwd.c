/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/diagchar.h>

#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#include "../../usb/gadget/f_diag.h"
#endif
#include <mach/msm_smd.h>
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagchar_hdlc.h"
#include <linux/pm_runtime.h>
#if defined(CONFIG_MACH_MECHA)
#include "../../../arch/arm/mach-msm/lte/sdio_diag.h"
#endif
#if defined(CONFIG_ARCH_MSM8X60_LTE)
#include "diagfwd_sdio.h"
#endif

#ifdef CONFIG_BTPORT
#include "../../btport/btport.h"
#endif

static int diag7k_debug_mask;
module_param_named(debug_mask, diag7k_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

MODULE_DESCRIPTION("Diag Char Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");

int diag_debug_buf_idx;
unsigned char diag_debug_buf[1024];
static unsigned int buf_tbl_size = 8; /*Number of entries in table of buffers */
static int diag_smd_function_mode;
static int diag_ch_sdio;  /* 0 for DIAG_MDM, 1 for DIAG_LEGACY*/

#ifdef CONFIG_DIAG_NO_MODEM

struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };

#define ENCODE_RSP_AND_SEND(buf_length)				\
do {									\
	send.state = DIAG_STATE_START;					\
	send.pkt = driver->apps_rsp_buf;				\
	send.last = (void *)(driver->apps_rsp_buf + buf_length);	\
	send.terminate = 1;						\
	if (!driver->in_busy_1) {					\
		enc.dest = driver->buf_in_1;				\
		enc.dest_last = (void *)(driver->buf_in_1 + 499);	\
		diag_hdlc_encode(&send, &enc);				\
		driver->write_ptr_1->buf = driver->buf_in_1;		\
		driver->write_ptr_1->length = buf_length + 4 +          \
		*(int *)(enc.dest - buf_length);               \
		usb_diag_write(driver->legacy_ch, driver->write_ptr_1);	\
	}								\
} while (0)
#endif
#define CHK_OVERFLOW(bufStart, start, end, length) \
((bufStart <= start) && (end - start >= length)) ? 1 : 0

void __diag_smd_send_req(void)
{
	void *buf = NULL;
	int *in_busy_ptr = NULL;
	struct diag_request *write_ptr_modem = NULL;
#if  DIAG_XPST
	int type;
#endif

	if (!driver->in_busy_1) {
		buf = driver->buf_in_1;
		write_ptr_modem = driver->write_ptr_1;
		in_busy_ptr = &(driver->in_busy_1);
	} else if (!driver->in_busy_2) {
		buf = driver->buf_in_2;
		write_ptr_modem = driver->write_ptr_2;
		in_busy_ptr = &(driver->in_busy_2);
	}

	if (driver->ch && buf) {
		int r = smd_read_avail(driver->ch);

		if (r > IN_BUF_SIZE) {
			if (r < MAX_IN_BUF_SIZE) {
				DIAGFWD_INFO("\n diag: SMD sending in "
						   "packets upto %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				DIAGFWD_ERR("\n diag: SMD sending in "
				"packets more than %d bytes", MAX_IN_BUF_SIZE);
				return;
			}
		}
		if (r > 0) {
			if (!buf)
				DIAGFWD_INFO("Out of diagmem for a9\n");
			else {
				APPEND_DEBUG('i');
				smd_read(driver->ch, buf, r);

				if (diag7k_debug_mask) {
					switch (diag7k_debug_mask) {
					case 1:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K(first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						break;
					case 2:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K(first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K(last 16 bytes) ", 16, 1, DUMP_PREFIX_ADDRESS, buf+r-16, 16, 1);
						break;
					default:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K ", 16, 1, DUMP_PREFIX_ADDRESS, buf, r, 1);

					}
				}

#if  DIAG_XPST

		type = checkcmd_modem_epst(buf);
		if (type) {
			modem_to_userspace(buf, r, type, 0);
			return;
		}

#endif


				APPEND_DEBUG('j');
				write_ptr_modem->length = r;
				*in_busy_ptr = 1;
				diag_device_write(buf, MODEM_DATA,
							 write_ptr_modem);
			}
		}
	}
}

int diag_device_write(void *buf, int proc_num, struct diag_request *write_ptr)
{
	int i, err = 0;

	if (driver->logging_mode == MEMORY_DEVICE_MODE) {
		int data_type = 0;
		if (proc_num == APPS_DATA) {
			data_type = MEMORY_DEVICE_LOG_TYPE;
			for (i = 0; i < driver->poolsize_write_struct; i++)
				if (driver->buf_tbl[i].length == 0) {
					driver->buf_tbl[i].buf = buf;
					driver->buf_tbl[i].length =
								 driver->used;
#ifdef DIAG_DEBUG
					printk(KERN_INFO "\n ENQUEUE buf ptr"
						   " and length is %x , %d\n",
			(unsigned int)(driver->buf_tbl[i].buf), driver->buf_tbl[i].length);
#endif
					break;
				}
#if defined(CONFIG_ARCH_MSM8X60_LTE)
		} else if (proc_num == MODEM_DATA || proc_num == QDSP_DATA || proc_num == SDIO_DATA) {
#else
		} else if (proc_num == MODEM_DATA || proc_num == QDSP_DATA) {
#endif
			data_type = USERMODE_DIAGFWD;
#ifdef DIAG_DEBUG
			pr_info("%s#%d: Modem Data buf=%p, len=%d\n", __func__, __LINE__, buf, write_ptr->length);
#endif
#if defined(CONFIG_ARCH_MSM8X60_LTE)
		if (proc_num == SDIO_DATA) {

			for (i = 0; i < driver->num_mdmclients; i++)
				if (driver->mdmclient_map[i].pid ==
						 driver->logging_process_id)
				break;

			if (i < driver->num_mdmclients) {
				driver->mdmdata_ready[i] |= data_type;
				wake_up_interruptible(&driver->mdmwait_q);

				return err;
			} else
				return -EINVAL;
		}
#endif
		}
#if defined(CONFIG_MACH_MECHA)
		else if (proc_num == SDIO_DATA) {

			for (i = 0; i < driver->num_mdmclients; i++)
				if (driver->mdmclient_map[i].pid ==
						 driver->logging_process_id)
				break;

			if (i <= driver->num_mdmclients) {
				driver->mdmdata_ready[i] |= MEMORY_DEVICE_LOG_TYPE;
				wake_up_interruptible(&driver->mdmwait_q);
				return err;
			} else
				return -EINVAL;
		}
#endif

		for (i = 0; i < driver->num_clients; i++)
			if (driver->client_map[i].pid == driver->logging_process_id)
				break;
		if (i < driver->num_clients) {
			driver->data_ready[i] |= data_type;
			wake_up_interruptible(&driver->wait_q);
		} else
			return -EINVAL;
	} else if (driver->logging_mode == NO_LOGGING_MODE) {
		if (proc_num == MODEM_DATA) {
			driver->in_busy_1 = 0;
			driver->in_busy_2 = 0;
			queue_work(driver->diag_wq, &(driver->
							diag_read_smd_work));
		} else if (proc_num == QDSP_DATA) {
			driver->in_busy_qdsp_1 = 0;
			driver->in_busy_qdsp_2 = 0;
			queue_work(driver->diag_wq, &(driver->
						diag_read_smd_qdsp_work));
		}
		err = -1;
	}
#ifdef CONFIG_DIAG_OVER_USB
	else if (driver->logging_mode == USB_MODE) {
		if (proc_num == APPS_DATA) {
			driver->write_ptr_svc = (struct diag_request *)
			(diagmem_alloc(driver, sizeof(struct diag_request),
				 POOL_TYPE_WRITE_STRUCT));
			if (driver->write_ptr_svc) {
				driver->write_ptr_svc->length = driver->used;
				driver->write_ptr_svc->buf = buf;
				err = usb_diag_write(driver->legacy_ch,
						driver->write_ptr_svc);
			} else
				err = -1;
		} else if (proc_num == MODEM_DATA) {
			write_ptr->buf = buf;
#ifdef DIAG_DEBUG
				printk(KERN_INFO "writing data to USB,"
				"pkt length %d\n", write_ptr->length);
			print_hex_dump(KERN_DEBUG, "Written Packet Data to"
					   " USB: ", 16, 1, DUMP_PREFIX_ADDRESS,
					    buf, write_ptr->length, 1);
#endif /* DIAG DEBUG */
			driver->diag_smd_count += write_ptr->length;
			err = usb_diag_write(driver->legacy_ch, write_ptr);
		} else if (proc_num == QDSP_DATA) {
			write_ptr->buf = buf;
			driver->diag_qdsp_count += write_ptr->length;
			err = usb_diag_write(driver->legacy_ch, write_ptr);
		}
#if defined(CONFIG_ARCH_MSM8X60_LTE)//|| defined(CONFIG_MACH_MECHA)
		else if (proc_num == SDIO_DATA) {
			write_ptr->buf = buf;
			if (diag_ch_sdio) {
			err = usb_diag_write(driver->legacy_ch, write_ptr);
			} else {
			err = usb_diag_write(driver->mdm_ch, write_ptr);
			}
		}
#endif
		APPEND_DEBUG('d');
	}
#endif /* DIAG OVER USB */
   return err;
}

void __diag_smd_qdsp_send_req(void)
{
	void *buf = NULL;
	int *in_busy_qdsp_ptr = NULL;
	struct diag_request *write_ptr_qdsp = NULL;
#if  DIAG_XPST
	int type;
#endif
	if (!driver->in_busy_qdsp_1) {
		buf = driver->buf_in_qdsp_1;
		write_ptr_qdsp = driver->write_ptr_qdsp_1;
		in_busy_qdsp_ptr = &(driver->in_busy_qdsp_1);
	} else if (!driver->in_busy_qdsp_2) {
		buf = driver->buf_in_qdsp_2;
		write_ptr_qdsp = driver->write_ptr_qdsp_2;
		in_busy_qdsp_ptr = &(driver->in_busy_qdsp_2);
	}

	if (driver->chqdsp && buf) {
		int r = smd_read_avail(driver->chqdsp);

		if (r > IN_BUF_SIZE) {
			if (r < MAX_IN_BUF_SIZE) {
				DIAGFWD_INFO("\n diag: SMD sending in "
						   "packets upto %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				DIAGFWD_ERR("\n diag: SMD sending in "
				"packets more than %d bytes", MAX_IN_BUF_SIZE);
			return;
		}
		}
		if (r > 0) {
			if (!buf)
				DIAGFWD_INFO("Out of diagmem for QDSP\n");
			else {
				APPEND_DEBUG('i');
					smd_read(driver->chqdsp, buf, r);

				if (diag7k_debug_mask) {
					switch (diag7k_debug_mask) {
					case 1:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from qdsp(first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						break;
					case 2:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from qdsp(first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from qdsp(last 16 bytes) ", 16, 1, DUMP_PREFIX_ADDRESS, buf+r-16, 16, 1);
						break;
					default:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from qdsp ", 16, 1, DUMP_PREFIX_ADDRESS, buf, r, 1);

					}
				}
#if  DIAG_XPST

		type = checkcmd_modem_epst(buf);
		if (type) {
			modem_to_userspace(buf, r, type, 0);
			return;
		}

#endif
				APPEND_DEBUG('j');
				write_ptr_qdsp->length = r;
				*in_busy_qdsp_ptr = 1;
				diag_device_write(buf, QDSP_DATA,
							 write_ptr_qdsp);
			}
		}
	}
}

static void diag_print_mask_table(void)
{
/* Enable this to print mask table when updated */
#ifdef MASK_DEBUG
	int first;
	int last;
	uint8_t *ptr = driver->msg_masks;
	int i = 0;

	while (*(uint32_t *)(ptr + 4)) {
		first = *(uint32_t *)ptr;
		ptr += 4;
		last = *(uint32_t *)ptr;
		ptr += 4;
		printk(KERN_INFO "SSID %d - %d\n", first, last);
		for (i = 0 ; i <= last - first ; i++)
			printk(KERN_INFO "MASK:%x\n", *((uint32_t *)ptr + i));
		ptr += ((last - first) + 1)*4;

	}
#endif
}

static void diag_update_msg_mask(int start, int end , uint8_t *buf)
{
	int found = 0;
	int first;
	int last;
	uint8_t *ptr = driver->msg_masks;
	uint8_t *ptr_buffer_start = &(*(driver->msg_masks));
	uint8_t *ptr_buffer_end = &(*(driver->msg_masks)) + MSG_MASK_SIZE;
	unsigned long flags;

	/*mutex_lock(&driver->diagchar_mutex);*/
	spin_lock_irqsave(&driver->diagchar_lock, flags);
	/* First SSID can be zero : So check that last is non-zero */

	while (*(uint32_t *)(ptr + 4)) {
		first = *(uint32_t *)ptr;
		ptr += 4;
		last = *(uint32_t *)ptr;
		ptr += 4;
		if (start >= first && start <= last) {
			ptr += (start - first)*4;
			if (end <= last)
				if (CHK_OVERFLOW(ptr_buffer_start, ptr,
						  ptr_buffer_end,
						  (((end - start)+1)*4)))
						memcpy(ptr, buf , ((end - start)+1)*4);
					else
					DIAGFWD_ERR("Not enough"
							 " buffer space for"
							 " MSG_MASK\n");
			else
				printk(KERN_INFO "Unable to copy"
						 " mask change\n");

			found = 1;
			break;
		} else {
			ptr += ((last - first) + 1)*4;
		}
	}
	/* Entry was not found - add new table */
	if (!found) {
		if (CHK_OVERFLOW(ptr_buffer_start, ptr, ptr_buffer_end,
				  8 + ((end - start) + 1)*4)) {
			memcpy(ptr, &(start) , 4);
			ptr += 4;
			memcpy(ptr, &(end), 4);
			ptr += 4;
			memcpy(ptr, buf , ((end - start) + 1)*4);
		} else
			DIAGFWD_ERR(" Not enough buffer"
					 " space for MSG_MASK\n");
	}

	/*mutex_unlock(&driver->diagchar_mutex);*/
	spin_unlock_irqrestore(&driver->diagchar_lock, flags);

	diag_print_mask_table();

}

static void diag_update_event_mask(uint8_t *buf, int toggle, int num_bits)
{
	uint8_t *ptr = driver->event_masks;
	uint8_t *temp = buf + 2;

	mutex_lock(&driver->diagchar_mutex);
	if (!toggle)
		memset(ptr, 0 , EVENT_MASK_SIZE);
	else
		if (CHK_OVERFLOW(ptr, ptr,
				 ptr+EVENT_MASK_SIZE,
				  num_bits/8 + 1))
			memcpy(ptr, temp , num_bits/8 + 1);
		else
			DIAGFWD_ERR("Not enough buffer space "
					 "for EVENT_MASK\n");
	mutex_unlock(&driver->diagchar_mutex);
}

static void diag_update_log_mask(int equip_id, uint8_t *buf, int num_items)
{
	uint8_t *temp = buf;
	struct mask_info {
		int equip_id;
		int index;
	};
	int i = 0;
	unsigned char *ptr_data;
	int offset = 8*MAX_EQUIP_ID;
	struct mask_info *ptr = (struct mask_info *)driver->log_masks;

	mutex_lock(&driver->diagchar_mutex);
	/* Check if we already know index of this equipment ID */
	for (i = 0; i < MAX_EQUIP_ID; i++) {
		if ((ptr->equip_id == equip_id) && (ptr->index != 0)) {
			offset = ptr->index;
			break;
		}
		if ((ptr->equip_id == 0) && (ptr->index == 0)) {
			/*Reached a null entry */
			ptr->equip_id = equip_id;
			ptr->index = driver->log_masks_length;
			offset = driver->log_masks_length;
			driver->log_masks_length += ((num_items+7)/8);
			break;
		}
		ptr++;
	}
	ptr_data = driver->log_masks + offset;
	if (CHK_OVERFLOW(ptr_data, ptr_data, ptr_data + LOG_MASK_SIZE,
				  (num_items+7)/8))
		memcpy(ptr_data, temp , (num_items+7)/8);
	else
		DIAGFWD_ERR(" Not enough buffer space for LOG_MASK\n");
	mutex_unlock(&driver->diagchar_mutex);
}

static void diag_update_pkt_buffer(unsigned char *buf)
{
	unsigned char *ptr = driver->pkt_buf;
	unsigned char *temp = buf;

	mutex_lock(&driver->diagchar_mutex);
	if (CHK_OVERFLOW(ptr, ptr, ptr + PKT_SIZE, driver->pkt_length))
		memcpy(ptr, temp , driver->pkt_length);
	else
		DIAGFWD_ERR(" Not enough buffer space for PKT_RESP\n");
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_userspace_clients(unsigned int type)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid != 0)
			driver->data_ready[i] |= type;
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_sleeping_process(int process_id)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid == process_id) {
			driver->data_ready[i] |= PKT_TYPE;
			break;
		}
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

static int diag_process_apps_pkt(unsigned char *buf, int len)
{
	uint16_t start;
	uint16_t end, subsys_cmd_code;
	int i, cmd_code, subsys_id;
	int packet_type = 1;
	unsigned char *temp = buf;

	/* event mask */
	if ((*buf == 0x60) && (*(++buf) == 0x0)) {
		diag_update_event_mask(buf, 0, 0);
		diag_update_userspace_clients(EVENT_MASKS_TYPE);
	}
	/* check for set event mask */
	else if (*buf == 0x82) {
		buf += 4;
		diag_update_event_mask(buf, 1, *(uint16_t *)buf);
		diag_update_userspace_clients(
		EVENT_MASKS_TYPE);
	}
	/* log mask */
	else if (*buf == 0x73) {
		buf += 4;
		if (*(int *)buf == 3) {
			buf += 4;
			/* Read Equip ID and pass as first param below*/
			diag_update_log_mask(*(int *)buf, buf+8,
							 *(int *)(buf+4));
			diag_update_userspace_clients(LOG_MASKS_TYPE);
		}
	}
	/* Check for set message mask  */
	else if ((*buf == 0x7d) && (*(++buf) == 0x4)) {
		buf++;
			start = *(uint16_t *)buf;
			buf += 2;
			end = *(uint16_t *)buf;
			buf += 4;
		diag_update_msg_mask((uint32_t)start, (uint32_t)end , buf);
			diag_update_userspace_clients(MSG_MASKS_TYPE);
		}
	/* Set all run-time masks
	if ((*buf == 0x7d) && (*(++buf) == 0x5)) {
		TO DO
	} */
#if defined(CONFIG_DIAG_NO_MODEM) && defined(CONFIG_DIAG_OVER_USB)
	/* Respond to polling for Apps only DIAG */
	else if ((*buf == 0x4b) && (*(buf+1) == 0x32) && (*(buf+2) == 0x03)) {
		for (i = 0; i < 3; i++)
			driver->apps_rsp_buf[i] = *(buf+i);
		for (i = 0; i < 13; i++)
			driver->apps_rsp_buf[i+3] = 0;

		ENCODE_RSP_AND_SEND(15);
		return 0;
	}
	/* respond to 0x0 command */
	else if (*buf == 0x00) {
		for (i = 0; i < 55; i++)
			driver->apps_rsp_buf[i] = 0;

		ENCODE_RSP_AND_SEND(54);
		return 0;
	}
	/* respond to 0x7c command */
	else if (*buf == 0x7c) {
		driver->apps_rsp_buf[0] = 0x7c;
		for (i = 1; i < 8; i++)
			driver->apps_rsp_buf[i] = 0;

		*(int *)(driver->apps_rsp_buf + 8) = 4062; /* ID for APQ 8060 */
		*(unsigned char *)(driver->apps_rsp_buf + 12) = '\0';
		*(unsigned char *)(driver->apps_rsp_buf + 13) = '\0';
		ENCODE_RSP_AND_SEND(13);
		return 0;
	}
#endif /* DIAG over USB & NO MODEM present */
	/* Check for registered clients and forward packet to user-space */
	else{
		cmd_code = (int)(*(char *)buf);
		temp++;
		subsys_id = (int)(*(char *)temp);
		temp++;
		subsys_cmd_code = *(uint16_t *)temp;
		temp += 2;

		for (i = 0; i < diag_max_registration; i++) {
			if (driver->table[i].process_id != 0) {
				if (driver->table[i].cmd_code ==
				     cmd_code && driver->table[i].subsys_id ==
				     subsys_id &&
				    driver->table[i].cmd_code_lo <=
				     subsys_cmd_code &&
					  driver->table[i].cmd_code_hi >=
				     subsys_cmd_code){
					driver->pkt_length = len;
					diag_update_pkt_buffer(buf);
					diag_update_sleeping_process(
						driver->table[i].process_id);
						return 0;
				    } /* end of if */
				else if (driver->table[i].cmd_code == 255
					  && cmd_code == 75) {
					if (driver->table[i].subsys_id ==
					    subsys_id &&
					   driver->table[i].cmd_code_lo <=
					    subsys_cmd_code &&
					     driver->table[i].cmd_code_hi >=
					    subsys_cmd_code){
						driver->pkt_length = len;
						diag_update_pkt_buffer(buf);
						diag_update_sleeping_process(
							driver->table[i].
							process_id);
						return 0;
					}
				} /* end of else-if */
				else if (driver->table[i].cmd_code == 255 &&
					  driver->table[i].subsys_id == 255) {
					if (driver->table[i].cmd_code_lo <=
							 cmd_code &&
						     driver->table[i].
						    cmd_code_hi >= cmd_code){
						driver->pkt_length = len;
						diag_update_pkt_buffer(buf);
						diag_update_sleeping_process
							(driver->table[i].
							 process_id);
						return 0;
					}
				} /* end of else-if */
			} /* if(driver->table[i].process_id != 0) */
		}  /* for (i = 0; i < diag_max_registration; i++) */
	} /* else */
		return packet_type;
}

void diag_process_hdlc(void *data, unsigned len)
{
	struct diag_hdlc_decode_type hdlc;
	int ret, type = 0;
#if HPST_FUN
	unsigned char *buf_9k = NULL;
	int path;
#endif
#ifdef DIAG_DEBUG
	int i;
	DIAGFWD_INFO("\n HDLC decode function, len of data  %d\n", len);
#endif
	hdlc.dest_ptr = driver->hdlc_buf;
	hdlc.dest_size = USB_MAX_OUT_BUF;
	hdlc.src_ptr = data;
	hdlc.src_size = len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = diag_hdlc_decode(&hdlc);

	if (ret)
		type = diag_process_apps_pkt(driver->hdlc_buf,
							  hdlc.dest_idx - 3);
	else if (driver->debug_flag) {
		DIAGFWD_ERR("Packet dropped due to bad HDLC coding/CRC"
				" errors or partial packet received, packet"
				" length = %d\n", len);
		print_hex_dump(KERN_DEBUG, "Dropped Packet Data: ", 16, 1,
					   DUMP_PREFIX_ADDRESS, data, len, 1);
		driver->debug_flag = 0;
	}
#ifdef CONFIG_DIAG_NO_MODEM
	if (type == 1) { /* implies this packet is NOT meant for apps */
		if (driver->chqdsp)
			smd_write(driver->chqdsp, driver->hdlc_buf,
							 hdlc.dest_idx - 3);
		type = 0;
	}
#endif /* NO MODEM present */

#ifdef DIAG_DEBUG
	printk(KERN_INFO "\n hdlc.dest_idx = %d", hdlc.dest_idx);
	for (i = 0; i < hdlc.dest_idx; i++)
		printk(KERN_DEBUG "\t%x", *(((unsigned char *)
							driver->hdlc_buf)+i));
#endif /* DIAG DEBUG */
	/* ignore 2 bytes for CRC, one for 7E and send */
	if ((driver->ch) && (ret) && (type) && (hdlc.dest_idx > 3)) {
		APPEND_DEBUG('g');
#if defined(CONFIG_ARCH_MSM8X60_LTE) || defined(CONFIG_ARCH_MSM8X60)
		smd_write(driver->ch, driver->hdlc_buf, hdlc.dest_idx - 3);
#else //defined(CONFIG_MACH_MECHA)
		smd_write(driver->ch, data, len);
#endif
		if (diag7k_debug_mask)
		print_hex_dump(KERN_DEBUG, "Written Packet Data to SMD: ", 16,
			       1, DUMP_PREFIX_ADDRESS, data, len, 1);
		APPEND_DEBUG('h');
#ifdef DIAG_DEBUG
		printk(KERN_INFO "writing data to SMD, pkt length %d \n", len);
		print_hex_dump(KERN_DEBUG, "Written Packet Data to SMD: ", 16,
			       1, DUMP_PREFIX_ADDRESS, data, len, 1);
#endif /* DIAG DEBUG */
	}

}

#ifdef CONFIG_DIAG_OVER_USB
#define N_LEGACY_WRITE	(driver->poolsize + 5) /* 2+1 for modem ; 2 for q6 */
#define N_LEGACY_READ	15
int diagfwd_connect(void)
{
	int err;

	DIAGFWD_INFO("%s\n", __func__);
	if (!driver->usb_connected) {
		err = usb_diag_alloc_req(driver->legacy_ch, N_LEGACY_WRITE,
			N_LEGACY_READ);
		if (err)
			DIAGFWD_ERR("diag: unable to allocate USB requests");
	}
	driver->usb_connected = 1;
	driver->in_busy_1 = 0;
	driver->in_busy_2 = 0;
	driver->in_busy_qdsp_1 = 0;
	driver->in_busy_qdsp_2 = 0;
#if defined(CONFIG_MACH_MECHA)
	driver->in_busy_mdm_1 = 0;
	driver->in_busy_mdm_2 = 0;
#endif
	/* Poll SMD channels to check for data*/
	queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));

#if defined(CONFIG_ARCH_MSM8X60_LTE)
	driver->in_busy_sdio = 0;
	if (diag_ch_sdio) {
		if (driver->legacy_ch && !IS_ERR(driver->legacy_ch))
			diagfwd_connect_sdio();
		else
			DIAGFWD_INFO("diag:No data from SDIO without  USB LEGACY ch");

	} else {
		if ((!driver->usb_connected) && driver->mdm_ch && !IS_ERR(driver->mdm_ch))
			diagfwd_connect_sdio();
		else
			DIAGFWD_INFO("diag:No data from SDIO without  USB MDM ch");

		queue_work(driver->diag_sdio_wq, &(driver->diag_read_mdm_work));
	}

#endif
	/* Poll USB channel to check for data*/
	queue_work(driver->diag_wq, &(driver->diag_read_work));

	return 0;
}

int diagfwd_disconnect(void)
{

	DIAGFWD_INFO("%s\n", __func__);
	driver->in_busy_1 = 1;
	driver->in_busy_2 = 1;
	driver->in_busy_qdsp_1 = 1;
	driver->in_busy_qdsp_2 = 1;
	driver->debug_flag = 1;
	if (driver->usb_connected)
		usb_diag_free_req(driver->legacy_ch);
	driver->usb_connected = 0;
#if defined(CONFIG_MACH_MECHA)
	driver->in_busy_mdm_1 = 1;
	driver->in_busy_mdm_2 = 1;
#endif
#if defined(CONFIG_ARCH_MSM8X60_LTE)
	if (diag_ch_sdio) {
		if (driver->legacy_ch && !IS_ERR(driver->legacy_ch))
			diagfwd_disconnect_sdio();
	} else {
		if (driver->usb_connected && driver->mdm_ch && !IS_ERR(driver->mdm_ch))
			diagfwd_disconnect_sdio();
	}
#endif
	/* TBD - notify and flow control SMD */
	return 0;
}

int diagfwd_write_complete(struct diag_request *diag_write_ptr)
{
	unsigned char *buf = diag_write_ptr->buf;
	/*Determine if the write complete is for data from arm9/apps/q6 */
	/* Need a context variable here instead */
	if (buf == (void *)driver->buf_in_1) {
		driver->in_busy_1 = 0;
		APPEND_DEBUG('o');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	} else if (buf == (void *)driver->buf_in_2) {
		driver->in_busy_2 = 0;
		APPEND_DEBUG('O');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	} else if (buf == (void *)driver->buf_in_qdsp_1) {
		driver->in_busy_qdsp_1 = 0;
		APPEND_DEBUG('p');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	} else if (buf == (void *)driver->buf_in_qdsp_2) {
		driver->in_busy_qdsp_2 = 0;
		APPEND_DEBUG('P');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	} else if (driver->in_busy_dmrounter == 1) {
		driver->in_busy_dmrounter = 0;
	}
#if defined(CONFIG_ARCH_MSM8X60_LTE)
	else if (buf == (void *)driver->buf_in_sdio) {
		diagfwd_write_complete_sdio();
	}
#endif
#if defined(CONFIG_MACH_MECHA)
	else if (buf == (void *)driver->buf_in_mdm_1) {
		driver->in_busy_mdm_1 = 0;
		queue_work(driver->mdm_diag_workqueue, &(driver->diag_read_smd_mdm_work));
	} else if (buf == (void *)driver->buf_in_mdm_2) {
		driver->in_busy_mdm_2 = 0;
		queue_work(driver->mdm_diag_workqueue, &(driver->diag_read_smd_mdm_work));
	}
#endif
	else {
		diagmem_free(driver, (unsigned char *)buf, POOL_TYPE_HDLC);
		diagmem_free(driver, (unsigned char *)diag_write_ptr,
						 POOL_TYPE_WRITE_STRUCT);
		APPEND_DEBUG('q');
	}
	return 0;
}

int diagfwd_read_complete(struct diag_request *diag_read_ptr)
{
	int status = diag_read_ptr->status;
	unsigned char *buf = diag_read_ptr->buf;

	/* Determine if the read complete is for data on legacy/mdm ch */
	if (buf == (void *)driver->usb_buf_out) {
		driver->read_len_legacy = diag_read_ptr->actual;
		APPEND_DEBUG('s');
#ifdef DIAG_DEBUG
	printk(KERN_INFO "read data from USB, pkt length %d \n",
		    diag_read_ptr->actual);
	print_hex_dump(KERN_DEBUG, "Read Packet Data from USB: ", 16, 1,
		       DUMP_PREFIX_ADDRESS, diag_read_ptr->buf,
		       diag_read_ptr->actual, 1);
#endif /* DIAG DEBUG */
	if (driver->nohdlc) {
		driver->usb_read_ptr->buf = driver->usb_buf_out;
		driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
		usb_diag_read(driver->legacy_ch, driver->usb_read_ptr);
		return 0;
	}
	if (driver->logging_mode == USB_MODE) {
#ifdef CONFIG_ARCH_MSM8X60_LTE
		if (diag_ch_sdio) {
			driver->read_len_mdm = diag_read_ptr->actual;
			diagfwd_read_complete_sdio();
		} else
#endif
			{
			if (status != -ECONNRESET && status != -ESHUTDOWN)
				queue_work(driver->diag_wq,
					&(driver->diag_proc_hdlc_work));
			else
					queue_work(driver->diag_wq,
						 &(driver->diag_read_work));
			}
		}
	}
#if defined(CONFIG_ARCH_MSM8X60_LTE)
	else if (buf == (void *)driver->usb_buf_mdm_out) {
		driver->read_len_mdm = diag_read_ptr->actual;
		diagfwd_read_complete_sdio();
	}
#endif
	else
		DIAGFWD_ERR("diag: Unknown buffer ptr from USB");

	return 0;
}

void diag_read_work_fn(struct work_struct *work)
{

	APPEND_DEBUG('d');
	driver->usb_read_ptr->buf = driver->usb_buf_out;
	driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
	usb_diag_read(driver->legacy_ch, driver->usb_read_ptr);
	APPEND_DEBUG('e');
}

void diag_process_hdlc_fn(struct work_struct *work)
{

	APPEND_DEBUG('D');
	diag_process_hdlc(driver->usb_buf_out, driver->read_len_legacy);
	diag_read_work_fn(work);
	APPEND_DEBUG('E');
}

void diag_usb_legacy_notifier(void *priv, unsigned event,
			struct diag_request *d_req)
{
	switch (event) {
	case USB_DIAG_CONNECT:
		diagfwd_connect();
		break;
	case USB_DIAG_DISCONNECT:
		diagfwd_disconnect();
		break;
	case USB_DIAG_READ_DONE:
		diagfwd_read_complete(d_req);
		break;
	case USB_DIAG_WRITE_DONE:
		diagfwd_write_complete(d_req);
		break;
	default:
		DIAGFWD_ERR("Unknown event from USB diag\n");
		break;
	}
}

#endif /* DIAG OVER USB */

static void diag_smd_notify(void *ctxt, unsigned event)
{
	/*printk(KERN_INFO "%s:\n", __func__);*/
	switch (diag_smd_function_mode) {
#ifdef CONFIG_BTPORT
	case SMD_FUNC_OPEN_BT:
				/*BT DUN funciton*/
				DIAGFWD_INFO("%s:bt check me\n", __func__);
				bt_smd_diag_notify(ctxt, event);
				break;
#endif
	case SMD_FUNC_OPEN_DIAG:
				queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
				break;
	case SMD_FUNC_CLOSE:
	default:
				DIAGFWD_INFO("%s:diag smd is closed \n", __func__);
				break;
	}

}

#if defined(CONFIG_MSM_N_WAY_SMD)
static void diag_smd_qdsp_notify(void *ctxt, unsigned event)
{
	queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
}
#endif

void diag_smd_enable(smd_channel_t *ch, char *src, int mode)
{
	int r = 0;
	static smd_channel_t *_ch = NULL;
	DIAGFWD_INFO("smd_try_open(%s): mode=%d\n", src, mode);

	mutex_lock(&driver->smd_lock);
	diag_smd_function_mode = mode;
	if (mode) {
		if (!driver->ch) {
			r = smd_open(SMDDIAG_NAME, &driver->ch, driver, diag_smd_notify);
			if (!r)
				_ch = driver->ch;
				DIAGFWD_INFO("_ch=%x: driver->ch=%xn", (unsigned int)_ch, (unsigned int)driver->ch);
		}
	} else {
		if (driver->ch) {
			r = smd_close(driver->ch);
			driver->ch = NULL;
			if (!r)
				_ch = driver->ch;
				DIAGFWD_INFO("_ch=%x: driver->ch=%xn", (unsigned int)_ch, (unsigned int)driver->ch);
		}
	}
	ch = _ch;
	mutex_unlock(&driver->smd_lock);
	DIAGFWD_INFO("smd_try_open(%s): r=%d _ch=%x\n", src, r, (unsigned int)_ch);
}
static int diag_smd_probe(struct platform_device *pdev)
{
	int r = 0;

	if (pdev->id == 0) {
		r = smd_open(SMDDIAG_NAME, &driver->ch, driver, diag_smd_notify);
		if (!r) {
			diag_smd_function_mode = SMD_FUNC_OPEN_DIAG;
		}
	}
#if defined(CONFIG_MSM_N_WAY_SMD)
	if (pdev->id == 1)
#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_ARCH_MSM8X60_LTE) || defined(CONFIG_ARCH_MSM8X60)
				r = smd_named_open_on_edge("DIAG", SMD_APPS_QDSP
			, &driver->chqdsp, driver, diag_smd_qdsp_notify);
#else
				r = smd_open("DSP_DIAG", &driver->chqdsp, driver, diag_smd_qdsp_notify);
#endif
#endif
/*	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);*/
//err:
	printk(KERN_INFO "diag opened SMD port pdev->id=%d; r = %d ch=%x\n", pdev->id, r, (unsigned int)driver->ch);

	return 0;
}
/*
static int diagfwd_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_dev_pm_ops = {
	.runtime_suspend = diagfwd_runtime_suspend,
	.runtime_resume = diagfwd_runtime_resume,
};
*/

static struct platform_driver msm_smd_ch1_driver = {

	.probe = diag_smd_probe,
	.driver = {
		   .name = SMDDIAG_NAME,
		   .owner = THIS_MODULE,
		   /*.pm   = &diagfwd_dev_pm_ops,*/
		   },
};

int diagfwd_init(void)
{
	diag_debug_buf_idx = 0;
	driver->read_len_legacy = 0;
	if (driver->buf_in_1 == NULL)
		driver->buf_in_1 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_1 == NULL)
			goto err;
	if (driver->buf_in_2 == NULL)
		driver->buf_in_2 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_2 == NULL)
			goto err;
	if (driver->buf_in_qdsp_1 == NULL)
		driver->buf_in_qdsp_1 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_qdsp_1 == NULL)
			goto err;
	if (driver->buf_in_qdsp_2 == NULL)
		driver->buf_in_qdsp_2 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_qdsp_2 == NULL)
			goto err;
	if (driver->usb_buf_out  == NULL &&
	     (driver->usb_buf_out = kzalloc(USB_MAX_OUT_BUF,
					 GFP_KERNEL)) == NULL)
		goto err;
	if (driver->hdlc_buf == NULL
	    && (driver->hdlc_buf = kzalloc(HDLC_MAX, GFP_KERNEL)) == NULL)
		goto err;
	if (driver->msg_masks == NULL
	    && (driver->msg_masks = kzalloc(MSG_MASK_SIZE,
					     GFP_KERNEL)) == NULL)
		goto err;
	if (driver->log_masks == NULL &&
	    (driver->log_masks = kzalloc(LOG_MASK_SIZE, GFP_KERNEL)) == NULL)
		goto err;
	driver->log_masks_length = 8*MAX_EQUIP_ID;
	if (driver->event_masks == NULL &&
	    (driver->event_masks = kzalloc(EVENT_MASK_SIZE,
					    GFP_KERNEL)) == NULL)
		goto err;
	if (driver->client_map == NULL &&
	    (driver->client_map = kzalloc
	     ((driver->num_clients) * sizeof(struct diag_client_map),
		   GFP_KERNEL)) == NULL)
		goto err;
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	if (driver->buf_in_mdm_1 == NULL)
		driver->buf_in_mdm_1 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_mdm_1 == NULL)
			goto err;
	if (driver->buf_in_mdm_2 == NULL)
		driver->buf_in_mdm_2 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_mdm_2 == NULL)
			goto err;
#endif
#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_ARCH_MSM8X60_LTE)
	if (driver->mdmclient_map == NULL &&
	    (driver->mdmclient_map = kzalloc
	     ((driver->num_mdmclients) * sizeof(struct diag_client_map),
		   GFP_KERNEL)) == NULL)
		goto err;
#endif
	if (driver->buf_tbl == NULL)
			driver->buf_tbl = kzalloc(buf_tbl_size *
			  sizeof(struct diag_write_device), GFP_KERNEL);
	if (driver->buf_tbl == NULL)
		goto err;
#if 0//defined(CONFIG_MACH_MECHA) || defined(CONFIG_ARCH_MSM8X60_LTE)
	if (driver->mdmbuf_tbl == NULL)
			driver->mdmbuf_tbl = kzalloc(buf_tbl_size *
			  sizeof(struct diag_write_device), GFP_KERNEL);
	if (driver->mdmbuf_tbl == NULL)
		goto err;
#endif
	if (driver->data_ready == NULL &&
	     (driver->data_ready = kzalloc(driver->num_clients * sizeof(int)
					 , GFP_KERNEL)) == NULL)
		goto err;
#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_ARCH_MSM8X60_LTE)
	if (driver->mdmdata_ready == NULL &&
	     (driver->mdmdata_ready = kzalloc(driver->num_mdmclients * sizeof(struct
					 diag_client_map), GFP_KERNEL)) == NULL)
		goto err;
#endif
	if (driver->table == NULL &&
	     (driver->table = kzalloc(diag_max_registration*
				      sizeof(struct diag_master_table),
				       GFP_KERNEL)) == NULL)
		goto err;
	if (driver->write_ptr_1 == NULL)
		driver->write_ptr_1 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_1 == NULL)
					goto err;
	if (driver->write_ptr_2 == NULL)
		driver->write_ptr_2 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_2 == NULL)
			goto err;
	if (driver->write_ptr_qdsp_1 == NULL)
		driver->write_ptr_qdsp_1 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_qdsp_1 == NULL)
			goto err;
	if (driver->write_ptr_qdsp_2 == NULL)
		driver->write_ptr_qdsp_2 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_qdsp_2 == NULL)
			goto err;
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	if (driver->write_ptr_mdm_1 == NULL)
			driver->write_ptr_mdm_1 = kzalloc(
				sizeof(struct diag_request), GFP_KERNEL);
			if (driver->write_ptr_mdm_1 == NULL)
					goto err;
	if (driver->write_ptr_mdm_2 == NULL)
			driver->write_ptr_mdm_2 = kzalloc(
				sizeof(struct diag_request), GFP_KERNEL);
			if (driver->write_ptr_mdm_2 == NULL)
					goto err;
#endif
	if (driver->usb_read_ptr == NULL)
			driver->usb_read_ptr = kzalloc(
				sizeof(struct diag_request), GFP_KERNEL);
			if (driver->usb_read_ptr == NULL)
				goto err;
	if (driver->pkt_buf == NULL &&
	     (driver->pkt_buf = kzalloc(PKT_SIZE,
					 GFP_KERNEL)) == NULL)
		goto err;
#ifdef CONFIG_DIAG_NO_MODEM
	if (driver->apps_rsp_buf == NULL)
			driver->apps_rsp_buf = kzalloc(150, GFP_KERNEL);
		if (driver->apps_rsp_buf == NULL)
			goto err;
#endif
	driver->diag_wq = create_singlethread_workqueue("diag_wq");
#ifdef CONFIG_DIAG_OVER_USB
	INIT_WORK(&(driver->diag_proc_hdlc_work), diag_process_hdlc_fn);
	INIT_WORK(&(driver->diag_read_work), diag_read_work_fn);
	diag_setup();
#ifndef CONFIG_ARCH_MSM8X60_LTE
	driver->legacy_ch = usb_diag_open(DIAG_LEGACY, driver,
			diag_usb_legacy_notifier);
	if (IS_ERR(driver->legacy_ch)) {
		DIAGFWD_ERR("Unable to open USB diag legacy channel\n");
		goto err;
	}
#endif
#endif
	mutex_init(&driver->smd_lock);
#ifdef CONFIG_ARCH_MSM8X60_LTE
#if defined(CONFIG_USB_ANDROID_LTE_DIAG)
	diag_ch_sdio = 0;
	diagfwd_sdio_init(DIAG_MDM);
#else
	diag_ch_sdio = 1;
	diagfwd_sdio_init(DIAG_LEGACY);
#endif
#endif
	platform_driver_register(&msm_smd_ch1_driver);

#if defined(CONFIG_MACH_MECHA)
	if (sdio_diag_init_enable)
		sdio_diag_init();
#endif
	return diag_ch_sdio;
err:
		DIAGFWD_INFO("\n Could not initialize diag buffers\n");
		kfree(driver->buf_in_1);
		kfree(driver->buf_in_2);
		kfree(driver->buf_in_qdsp_1);
		kfree(driver->buf_in_qdsp_2);
		kfree(driver->usb_buf_out);
		kfree(driver->hdlc_buf);
		kfree(driver->msg_masks);
		kfree(driver->log_masks);
		kfree(driver->event_masks);
		kfree(driver->client_map);
		kfree(driver->buf_tbl);
		kfree(driver->data_ready);
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
		kfree(driver->mdmclient_map);
		//kfree(driver->mdmbuf_tbl);
		kfree(driver->mdmdata_ready);
#endif
		kfree(driver->table);
		kfree(driver->pkt_buf);
		kfree(driver->write_ptr_1);
		kfree(driver->write_ptr_2);
		kfree(driver->write_ptr_qdsp_1);
		kfree(driver->write_ptr_qdsp_2);
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
		kfree(driver->write_ptr_mdm_1);
		kfree(driver->write_ptr_mdm_2);
#endif
		kfree(driver->usb_read_ptr);
#ifdef CONFIG_DIAG_NO_MODEM
		kfree(driver->apps_rsp_buf);
#endif
		if (driver->diag_wq)
			destroy_workqueue(driver->diag_wq);
		return 0;
}

void diagfwd_exit(void)
{
	smd_close(driver->ch);
	smd_close(driver->chqdsp);
	driver->ch = 0;		/*SMD can make this NULL */
	driver->chqdsp = 0;
#ifdef CONFIG_DIAG_OVER_USB
	if (driver->usb_connected)
		usb_diag_free_req(driver->legacy_ch);
#endif
	platform_driver_unregister(&msm_smd_ch1_driver);
#ifdef CONFIG_DIAG_OVER_USB
	usb_diag_close(driver->legacy_ch);
#endif

	kfree(driver->buf_in_1);
	kfree(driver->buf_in_2);
	kfree(driver->buf_in_qdsp_1);
	kfree(driver->buf_in_qdsp_2);
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	kfree(driver->buf_in_mdm_1);
	kfree(driver->buf_in_mdm_2);
#endif
	kfree(driver->usb_buf_out);
	kfree(driver->hdlc_buf);
	kfree(driver->msg_masks);
	kfree(driver->log_masks);
	kfree(driver->event_masks);
	kfree(driver->client_map);
	kfree(driver->buf_tbl);
	kfree(driver->data_ready);
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	kfree(driver->mdmclient_map);
	//kfree(driver->mdmbuf_tbl);
	kfree(driver->mdmdata_ready);
#endif
	kfree(driver->table);
	kfree(driver->pkt_buf);
	kfree(driver->write_ptr_1);
	kfree(driver->write_ptr_2);
	kfree(driver->write_ptr_qdsp_1);
	kfree(driver->write_ptr_qdsp_2);
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	kfree(driver->write_ptr_mdm_1);
	kfree(driver->write_ptr_mdm_2);
#endif
	kfree(driver->usb_read_ptr);
#ifdef CONFIG_DIAG_NO_MODEM
	kfree(driver->apps_rsp_buf);
#endif
	destroy_workqueue(driver->diag_wq);
}
