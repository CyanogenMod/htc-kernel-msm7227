/* drivers/usb/gadget/f_diag.c
 * Diag Function Device - Route ARM9 and ARM11 DIAG messages
 * between HOST and DEVICE.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/usbdiag.h>
#include <mach/rpc_hsusb.h>

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android_composite.h>
#include <linux/workqueue.h>

#include <mach/usbdiag.h>

#if defined(CONFIG_MACH_MECHA)
#include <mach/smsc251x.h>
#endif
/*#define HTC_DIAG_DEBUG*/
#if DIAG_XPST
#include <mach/lte/msm_smd.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include "../../char/diag/diagchar.h"
#include "../../char/diag/diagfwd.h"
#include "../../char/diag/diagmem.h"
#include "../../char/diag/diagchar_hdlc.h"
#include "../../../arch/arm/mach-msm/lte/sdio_diag.h"

#define USB_DIAG_IOC_MAGIC 0xFF
#define USB_DIAG_FUNC_IOC_ENABLE_SET	_IOW(USB_DIAG_IOC_MAGIC, 1, int)
#define USB_DIAG_FUNC_IOC_ENABLE_GET	_IOR(USB_DIAG_IOC_MAGIC, 2, int)
#define USB_DIAG_FUNC_IOC_REGISTER_SET  _IOW(USB_DIAG_IOC_MAGIC, 3, char *)
#define USB_DIAG_FUNC_IOC_AMR_SET	_IOW(USB_DIAG_IOC_MAGIC, 4, int)

#define USB_DIAG_NV_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 1, uint16_t *)
#define USB_DIAG_NV_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 2, uint16_t *)
#define USB_DIAG_NV_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 3, uint16_t *)
#define USB_DIAG_NV_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 4, uint16_t *)
/*
#define USB_DIAG_RC9_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 5, uint16_t *)
#define USB_DIAG_RC9_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 6, uint16_t *)
#define USB_DIAG_RC9_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 7, uint16_t *)
#define USB_DIAG_RC9_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 8, uint16_t *)
*/
#define USB_DIAG_PRL_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 9, uint16_t *)
#define USB_DIAG_PRL_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 10, uint16_t *)
#define USB_DIAG_PRL_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 11, uint16_t *)
#define USB_DIAG_PRL_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 12, uint16_t *)
#define USB_DIAG_M29_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 13, uint16_t *)
#define USB_DIAG_M29_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 14, uint16_t *)
#define USB_DIAG_M29_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 15, uint16_t *)
#define USB_DIAG_M29_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 16, uint16_t *)
#define SMD_MAX 8192
#define NV_TABLE_SZ  128
#define M29_TABLE_SZ  10
#define PRL_TABLE_SZ  10

#define EPST_PREFIX 0xC8
#define HPST_PREFIX 0xF1


#define NO_PST 0
#define NO_DEF_ID 1
#define DM7K9K  2
#define DM7KONLY  3
#define DM9KONLY  4
#define DM7K9KDIFF  5
#define NO_DEF_ITEM  0xff

#define MAX(x, y) (x > y ? x : y)
#endif

#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
int sdio_diag_init_enable;
#endif

#if defined(CONFIG_ARCH_MSM8X60_LTE)
int diag_init_enabled_state = 0;
#endif

int diag_configured;

static DEFINE_SPINLOCK(ch_lock);
static LIST_HEAD(usb_diag_ch_list);

static struct usb_interface_descriptor intf_desc = {
	.bLength            =	sizeof intf_desc,
	.bDescriptorType    =	USB_DT_INTERFACE,
	.bNumEndpoints      =	2,
	.bInterfaceClass    =	0xFF,
	.bInterfaceSubClass =	0xFF,
	.bInterfaceProtocol =	0xFF,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength 			=	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType 	=	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes 		=	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize 	=	__constant_cpu_to_le16(512),
	.bInterval 			=	0,
};
static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(512),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_descriptor_header *fs_diag_desc[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	NULL,
	};
static struct usb_descriptor_header *hs_diag_desc[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	NULL,
};

/**
 * struct diag_context - USB diag function driver private structure
 * @android_function: Used for registering with Android composite driver
 * @function: function structure for USB interface
 * @out: USB OUT endpoint struct
 * @in: USB IN endpoint struct
 * @in_desc: USB IN endpoint descriptor struct
 * @out_desc: USB OUT endpoint descriptor struct
 * @read_pool: List of requests used for Rx (OUT ep)
 * @write_pool: List of requests used for Tx (IN ep)
 * @config_work: Work item schedule after interface is configured to notify
 *               CONNECT event to diag char driver and updating product id
 *               and serial number to MODEM/IMEM.
 * @lock: Spinlock to proctect read_pool, write_pool lists
 * @cdev: USB composite device struct
 * @pdata: Platform data for this driver
 * @ch: USB diag channel
 *
 */
struct diag_context {
	struct android_usb_function android_function;
	struct usb_function function;
	struct usb_ep *out;
	struct usb_ep *in;
	struct usb_endpoint_descriptor  *in_desc;
	struct usb_endpoint_descriptor  *out_desc;
	struct list_head read_pool;
	struct list_head write_pool;
	struct work_struct config_work;
	spinlock_t lock;
	unsigned configured;
	struct usb_composite_dev *cdev;
	struct  usb_diag_platform_data *pdata;
	struct usb_diag_ch ch;


	unsigned char i_serial_number;
	unsigned short  product_id;
	int function_enable;

#if DIAG_XPST
	spinlock_t req_lock;

	struct mutex user_lock;
#define ID_TABLE_SZ 20 /* keep this small */
	struct list_head rx_req_idle;
	struct list_head rx_req_user;
	wait_queue_head_t read_wq;
	char *user_read_buf;
	uint32_t user_read_len;
	char *user_readp;
	bool opened;
	/* list of registered command ids to be routed to userspace */
	unsigned char id_table[ID_TABLE_SZ];

	//smd_channel_t *ch;
        int online;
	int error;
/* for slate test */
	struct list_head rx_arm9_idle;
	struct list_head rx_arm9_done;
	struct mutex diag2arm9_lock;
	struct mutex diag2arm9_read_lock;
	struct mutex diag2arm9_write_lock;
	bool diag2arm9_opened;
	unsigned char toARM9_buf[SMD_MAX];
	unsigned read_arm9_count;
	unsigned char *read_arm9_buf;
	wait_queue_head_t read_arm9_wq;
	struct usb_request *read_arm9_req;
	u64 tx_count; /* to smd */
	u64 rx_count; /* from smd */
	u64 usb_in_count; /* to pc */
	u64 usb_out_count; /* from pc */
#endif
};

struct diag_context _context;
static struct usb_diag_ch *legacych;
static  struct diag_context *legacyctxt;

#if defined(CONFIG_USB_ANDROID_LTE_DIAG)
static struct usb_diag_ch *mdmch;
#endif
static  struct diag_context *mdmctxt;



#if DIAG_XPST
struct device diag_device;
static char *htc_write_buf_copy;
static struct diag_request htc_w_diag_req;
static struct diag_request *htc_write_diag_req;
#define TRX_REQ_BUF_SZ 8192

#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
static	uint16_t nv7K9K_table[NV_TABLE_SZ] = {82,
0, 4, 5, 20, 21, 37, 258, 318, 460, 461,
462, 463, 464, 465, 466, 546, 707, 714, 854, 1943,
2825, 2953, 30000, 30001, 30002, 30003, 30004, 30005, 30006, 30007,
30008, 30010, 30013, 30014, 30015, 30018, 30019, 30021, 30031, 30032,
30033, 6, 8, 53, 54, 161, 162, 40, 42, 43,
57, 71, 82, 85, 168, 169, 170, 191, 192, 197,
219, 231, 240, 241, 256, 297, 298, 300, 423, 424,
429, 442, 450, 459, 495, 835, 855, 910, 933, 941,
945, 3634};

static	uint16_t nv7Konly_table[NV_TABLE_SZ] = {43,
25, 27, 29, 32, 33, 34, 35, 36, 176, 177,
259, 260, 261, 262, 263, 264, 265, 296, 319, 374,
375, 67, 69, 70, 74, 178, 215, 179, 255, 285,
401, 403, 405, 409, 426, 452, 1018, 4102, 906, 30026,
30027, 30028, 30029};

static	uint16_t nv7K9Kdiff_table[NV_TABLE_SZ] = {14,
11, 10, 441, 946, 30016, 30030, 562, 32768+11, 32768+10, 32768+441,
32768+946, 32768+30016, 32768+30030, 32768+562};

static	uint16_t nv9Konly_table[NV_TABLE_SZ] = {7, 579, 580, 1192, 1194, 4204, 4964, 818};

/*mode reset */
static	uint16_t M297K9K_table[M29_TABLE_SZ] = {4, 1, 2, 4, 5};
static	uint16_t M297Konly_table[M29_TABLE_SZ];
static	uint16_t M297K9Kdiff_table[M29_TABLE_SZ];
static	uint16_t M299Konly_table[M29_TABLE_SZ];

/*PRL read/write*/
static	uint16_t PRL7K9K_table[PRL_TABLE_SZ] = {1, 0};
static	uint16_t PRL7Konly_table[PRL_TABLE_SZ];
static	uint16_t PRL7K9Kdiff_table[PRL_TABLE_SZ];
static	uint16_t PRL9Konly_table[PRL_TABLE_SZ];
#endif


static struct usb_request *diag_req_new(unsigned len)
{
	struct usb_request *req;
	if (len > SMD_MAX)
		return NULL;
	req = kmalloc(sizeof(struct usb_request), GFP_KERNEL);
	if (!req)
		return NULL;
	req->buf = kmalloc(len, GFP_KERNEL);
	if (!req->buf) {
		kfree(req);
		return NULL;
	}
	return req;
}

static void diag_req_free(struct usb_request *req)
{
	if (!req)
		return;

	if (req->buf) {
		kfree(req->buf);
		req->buf = 0;
	}
	kfree(req);
	req = 0;
}





/* add a request to the tail of a list */
static void req_put(struct diag_context *ctxt, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->req_lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct diag_context *ctxt,
		struct list_head *head)
{
	struct usb_request *req = 0;
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	if (!list_empty(head)) {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return req;
}

#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
int decode_encode_hdlc(void*data, int *len, unsigned char *buf_hdlc, int remove, int pos)
{
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	struct diag_hdlc_decode_type hdlc;
  unsigned char *buf_9k = NULL;
  int ret;


	buf_9k = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
	if (!buf_9k) {
		DIAG_INFO("%s:out of memory\n", __func__);
		return -ENOMEM;
	}

	hdlc.dest_ptr = buf_9k;
	hdlc.dest_size = USB_MAX_OUT_BUF;
	hdlc.src_ptr = data;
	hdlc.src_size = *len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = diag_hdlc_decode(&hdlc);
	if (!ret) {
		DIAG_INFO("Packet dropped due to bad HDLC coding/CRC\n");
		kfree(buf_9k);
		return -EINVAL;
	}
	if (remove)
		*((char *)buf_9k+pos) = (*((char *)buf_9k+pos) ^ 0x80);
	else
		*((char *)buf_9k+pos) = (*((char *)buf_9k+pos) | 0x80);


	send.state = DIAG_STATE_START;
	send.pkt = hdlc.dest_ptr;
	send.last = (void *)(hdlc.dest_ptr + hdlc.dest_idx - 4);
	send.terminate = 1;
	enc.dest = buf_hdlc;
	enc.dest_last = (void *)(buf_hdlc + 2*hdlc.dest_idx  - 3);
	diag_hdlc_encode(&send, &enc);

	print_hex_dump(KERN_DEBUG, "encode Data"
	, 16, 1, DUMP_PREFIX_ADDRESS, buf_hdlc, hdlc.dest_idx, 1);

	*len = hdlc.dest_idx;

	kfree(buf_9k);
	return 0;


}
#endif
int checkcmd_modem_epst(unsigned char *buf)
{
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	int j;
	uint16_t nv_num;
	uint16_t max_item;

		if (*buf == EPST_PREFIX) {
			if (*(buf+1) == 0x26 || *(buf+1) == 0x27) {
				max_item = MAX(MAX(nv7K9K_table[0], nv7Konly_table[0]),
				 MAX(nv9Konly_table[0], nv7K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < NV_TABLE_SZ; j++) {
					if (j <= nv7K9K_table[0] && nv7K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= nv7Konly_table[0] && nv7Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= nv9Konly_table[0]  && nv9Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= nv7K9Kdiff_table[0]  && nv7K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0x48 || *(buf+1) == 0x49) {
				max_item = MAX(MAX(PRL7K9K_table[0], PRL7Konly_table[0]),
				 MAX(PRL9Konly_table[0], PRL7K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < PRL_TABLE_SZ; j++) {
					if (j <= PRL7K9K_table[0] && PRL7K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= PRL7Konly_table[0] && PRL7Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= PRL9Konly_table[0]  && PRL9Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= PRL7K9Kdiff_table[0]  && PRL7K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0xC9) {
				nv_num = *(buf+2);
				DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				if (*(buf+2) == 0x01 || *(buf+2) == 0x11)
					return  DM7K9K;
				else
					return  NO_DEF_ITEM;

			} else if (*(buf+1) == 0x29) {
				max_item = MAX(MAX(M297K9K_table[0], M297Konly_table[0]),
				 MAX(M299Konly_table[0], M297K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < M29_TABLE_SZ; j++) {
					if (j <= M297K9K_table[0] && M297K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= M297Konly_table[0] && M297Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= M299Konly_table[0]  && M299Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= M297K9Kdiff_table[0]  && M297K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0x41 || *(buf+1) == 0x0C || *(buf+1) == 0x40) {
				return  DM7K9K;
			} else if (*(buf+1) == 0x00 || *(buf+1) == 0xCD || *(buf+1) == 0xD8
				|| *(buf+1) == 0x35 || *(buf+1) == 0x36) {
				return  DM7KONLY;
			} else if (*(buf+1) == 0xDF) {
				return  DM9KONLY;
			} else if (*(buf+1) == 0x4B && *(buf+2) == 0x0D) {
				return  DM7KONLY;
			} else
				DIAG_INFO("%s:id = 0x%x no default routing path\n", __func__, *(buf+1));
				return NO_DEF_ID;
		} else {
				/*DIAG_INFO("%s: not EPST_PREFIX id = 0x%x route to USB!!!\n", __func__, *buf);*/
				return NO_PST;
		}

#else
	if (_context.diag2arm9_opened)
		return  DM7KONLY;
	else
		return NO_PST;
#endif

}
int modem_to_userspace(void *buf, int r, int type, int is9k)
{

	struct diag_context *ctxt = &_context;
	struct usb_request *req;
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	unsigned char value;
#endif

	if (!ctxt->diag2arm9_opened)
		return 0;
	req = req_get(ctxt, &ctxt->rx_arm9_idle);
	if (!req) {
		DIAG_INFO("There is no enough request to ARM11!!\n");
		return 0;
	}
	memcpy(req->buf, buf, r);

#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	if (type == DM7K9KDIFF) {
		value = *((uint8_t *)req->buf+1);
		if ((value == 0x27) || (value == 0x26)) {
			if (is9k == 1) {
				decode_encode_hdlc(buf, &r, req->buf, 0, 3);
			}
		}
	} else if (type == NO_DEF_ID) {
	/*in this case, cmd may reply error message*/
		value = *((uint8_t *)req->buf+2);
		DIAG_INFO("%s:check error cmd=0x%x message=ox%x\n", __func__
		, value, *((uint8_t *)req->buf+1));
		if ((value == 0x27) || (value == 0x26)) {
			if (is9k == 1) {
				decode_encode_hdlc(buf, &r, req->buf, 0, 4);
			}
		}
	}
#endif

	if (is9k == 1)
		print_hex_dump(KERN_DEBUG, "DM Read Packet Data"
					       " from 9k radio (first 16 Bytes): ", 16, 1, DUMP_PREFIX_ADDRESS, req->buf, 16, 1);
	else
		print_hex_dump(KERN_DEBUG, "DM Read Packet Data"
					       " from 7k radio (first 16 Bytes): ", 16, 1, DUMP_PREFIX_ADDRESS, req->buf, 16, 1);
//	ctxt->rx_count += r;
	req->actual = r;
	req_put(ctxt, &ctxt->rx_arm9_done, req);
	wake_up(&ctxt->read_arm9_wq);
	return 1;
}





static long htc_diag_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	int tmp_value;
	unsigned long flags;
	unsigned char temp_id_table[ID_TABLE_SZ];

	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);


	if (_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case USB_DIAG_FUNC_IOC_ENABLE_SET:
		if (copy_from_user(&tmp_value, argp, sizeof(int)))
			return -EFAULT;
		DIAG_INFO("diag: enable %d\n", tmp_value);
		android_enable_function(&_context.function, tmp_value);

		diag_smd_enable(driver->ch, "diag_ioctl", tmp_value);
#if defined(CONFIG_MACH_MECHA)
		/* internal hub*/
		smsc251x_mdm_port_sw(tmp_value);
#endif
		/* force diag_read to return error when disable diag */
		if (tmp_value == 0)
			ctxt->error = 1;
		wake_up(&ctxt->read_wq);
	break;
	case USB_DIAG_FUNC_IOC_ENABLE_GET:
		tmp_value = !_context.function.hidden;
		if (copy_to_user(argp, &tmp_value, sizeof(tmp_value)))
			return -EFAULT;
	break;

	case USB_DIAG_FUNC_IOC_REGISTER_SET:
		if (copy_from_user(temp_id_table, (unsigned char *)argp, ID_TABLE_SZ))
			return -EFAULT;
		spin_lock_irqsave(&ctxt->req_lock, flags);
		memcpy(ctxt->id_table, temp_id_table, ID_TABLE_SZ);
		print_hex_dump(KERN_DEBUG, "ID_TABLE_SZ Data: ", 16, 1,
		DUMP_PREFIX_ADDRESS, temp_id_table, ID_TABLE_SZ, 1);
		spin_unlock_irqrestore(&ctxt->req_lock, flags);
		break;

	case USB_DIAG_FUNC_IOC_AMR_SET:
	/*	if (copy_from_user(&ctxt->is2ARM11, argp, sizeof(int)))
			return -EFAULT;*/
		DIAG_INFO("diag: fix me USB_DIAG_FUNC_IOC_AMR_SET\n");
		break;
	default:
		return -ENOTTY;
	}


	return 0;
}

static ssize_t htc_diag_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req = 0;
	int ret = 0;


	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);

	/* we will block until we're online */
	if (!ctxt->online) {
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0 || ctxt->error)
			return -EFAULT;
	}

	mutex_lock(&ctxt->user_lock);

	if (ctxt->user_read_len && ctxt->user_readp) {
		if (count > ctxt->user_read_len)
			count = ctxt->user_read_len;
		if (copy_to_user(buf, ctxt->user_readp, count))
			ret = -EFAULT;
		else {
			ctxt->user_readp += count;
			ctxt->user_read_len -= count;
			ret = count;
		}
		goto end;
	}

	mutex_unlock(&ctxt->user_lock);
	ret = wait_event_interruptible(ctxt->read_wq,
		(req = req_get(ctxt, &ctxt->rx_req_user)) || !ctxt->online);
	mutex_lock(&ctxt->user_lock);

	if (ret < 0) {
		DIAG_INFO("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}

	if (!ctxt->online) {
		 DIAG_INFO("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}

	if (req) {
		if (req->actual == 0) {
			DIAG_INFO("%s: no data\n", __func__);
			goto end;
		}
		if (count > req->actual)
			count = req->actual;
		if (copy_to_user(buf, req->buf, count)) {
			ret = -EFAULT;
			goto end;
		}
		req->actual -= count;
		if (req->actual) {
			memcpy(ctxt->user_read_buf, req->buf + count, req->actual);
			ctxt->user_read_len = req->actual;
			ctxt->user_readp = ctxt->user_read_buf;
		}
		ret = count;
	}
end:
	if (req)
		req_put(ctxt, &ctxt->rx_req_idle, req);

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static ssize_t htc_diag_write(struct file *fp, const char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	int ret = 0;


	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);


	mutex_lock(&ctxt->user_lock);

	if (ret < 0) {
		DIAG_INFO("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}

	if (!ctxt->online) {
		DIAG_INFO("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}

	if (count > TRX_REQ_BUF_SZ)
		count = TRX_REQ_BUF_SZ;

	if (!htc_write_buf_copy || !htc_write_diag_req) {
		ret = -EIO;
		goto end;
	}

	if (copy_from_user(htc_write_buf_copy, buf, count)) {
		ret = -EFAULT;
		DIAG_INFO("%s:EFAULT\n", __func__);
		goto end;
	}

	htc_write_diag_req->buf = htc_write_buf_copy;
	htc_write_diag_req->length = count;

	driver->in_busy_dmrounter = 1;

	ret = usb_diag_write(driver->legacy_ch, htc_write_diag_req);

	if (ret < 0) {
		DIAG_INFO("%s: usb_diag_write error %d\n", __func__, ret);
		goto end;
	}
		ret = count;

end:

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static int htc_diag_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	int rc = 0;
	int n;
	struct usb_request *req;


	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);

	mutex_lock(&ctxt->user_lock);

	if (ctxt->opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}


	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (!ctxt->user_read_buf) {
		ctxt->user_read_buf = kmalloc(TRX_REQ_BUF_SZ, GFP_KERNEL);
		if (!ctxt->user_read_buf) {
			rc = -ENOMEM;
			goto done;
		}
	}

	if (!htc_write_buf_copy) {
		htc_write_buf_copy = (char *)kmalloc(TRX_REQ_BUF_SZ, GFP_KERNEL);
		if (!htc_write_buf_copy) {
			rc = -ENOMEM;
			kfree(ctxt->user_read_buf);
			goto done;
		}
	}

	if (!htc_write_diag_req) {
		htc_write_diag_req = &htc_w_diag_req;
		if (!htc_write_diag_req) {
			kfree(ctxt->user_read_buf);
			kfree(htc_write_buf_copy);
			rc = -ENOMEM;
			goto done;
		}
	}

	/* clear pending data if any */
	while ((req = req_get(ctxt, &ctxt->rx_req_idle)))
		diag_req_free(req);

	for (n = 0; n < 10; n++) {
		req = diag_req_new(SMD_MAX);
		if (!req) {
			while ((req = req_get(ctxt, &ctxt->rx_req_idle)))
				diag_req_free(req);
			rc = -EFAULT;
			goto done;
		}
		req_put(ctxt, &ctxt->rx_req_idle, req);
	}

	ctxt->opened = true;
	/* clear the error latch */
	ctxt->error = 0;

done:
	mutex_unlock(&ctxt->user_lock);

	return rc;
}

static int htc_diag_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;


	DIAG_INFO("%s: \n", __func__);


	mutex_lock(&ctxt->user_lock);
	ctxt->opened = false;
	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (ctxt->user_read_buf) {
		kfree(ctxt->user_read_buf);
		ctxt->user_read_buf = 0;
	}
	if (!htc_write_buf_copy) {
		kfree(htc_write_buf_copy);
		htc_write_buf_copy = 0;
	}
	if (!htc_write_diag_req) {
		kfree(htc_write_diag_req);
		htc_write_diag_req = 0;
	}
	while ((req = req_get(ctxt, &ctxt->rx_req_idle)))
		diag_req_free(req);
	while ((req = req_get(ctxt, &ctxt->rx_req_user)))
		diag_req_free(req);
	mutex_unlock(&ctxt->user_lock);

	return 0;
}

static struct file_operations htc_diag_fops = {
	.owner =   THIS_MODULE,
	.read =    htc_diag_read,
	.write =   htc_diag_write,
	.open =    htc_diag_open,
	.release = htc_diag_release,
	.unlocked_ioctl = htc_diag_ioctl,
};

static struct miscdevice htc_diag_device_fops = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htcdiag",
	.fops = &htc_diag_fops,
};


static int if_route_to_userspace(struct diag_context *ctxt, unsigned int cmd)
{
	unsigned long flags;
	int i;
	unsigned short tmp;
	unsigned char cmd_id, cmd_num;

	tmp = (unsigned short)cmd;

	cmd_num = (unsigned char)(tmp >> 8);
	cmd_id = (unsigned char)(tmp & 0x00ff);

	if (!ctxt->opened || cmd_id == 0)
		return 0;
	/* command ids 0xfb..0xff are not used by msm diag; we steal these ids
	 * for communication between userspace tool and host test tool.
	 */
	 /*printk("cmd_num=%d cmd_id=%d\n", cmd_num, cmd_id);*/
	if (cmd_id >= 0xfb && cmd_id <= 0xff)
		return 1;


	spin_lock_irqsave(&ctxt->req_lock, flags);
	for (i = 0; i < ARRAY_SIZE(ctxt->id_table); i = i+2)
		if (ctxt->id_table[i] == cmd_id) {
			/* if the command id equals to any of registered ids,
			 * route to userspace to handle.
			 */
			if (ctxt->id_table[i+1] == cmd_num || ctxt->id_table[i+1] == 0xff) {
				spin_unlock_irqrestore(&ctxt->req_lock, flags);
				return 1;
			}
		}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return 0;
}
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
static long diag2arm9_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	unsigned long flags;
	uint16_t temp_nv_table[NV_TABLE_SZ];
	int table_size;
	uint16_t *table_ptr;

	if (_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC)
		return -ENOTTY;

	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);

	switch (cmd) {

	case USB_DIAG_NV_7K9K_SET:
		DIAG_INFO("USB_DIAG_NV_7K9K_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7K9K_table;
		break;
	case USB_DIAG_NV_7KONLY_SET:
		DIAG_INFO("USB_DIAG_NV_7KONLY_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7Konly_table;
		break;
	case USB_DIAG_NV_9KONLY_SET:
		DIAG_INFO("USB_DIAG_NV_9KONLY_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv9Konly_table;
		break;
	case USB_DIAG_NV_7K9KDIFF_SET:
		DIAG_INFO("USB_DIAG_NV_7K9KDIFF_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7K9Kdiff_table;
		break;

	case USB_DIAG_PRL_7K9K_SET:
		DIAG_INFO("USB_DIAG_PRL_7K9K_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7K9K_table;
		break;
	case USB_DIAG_PRL_7KONLY_SET:
		DIAG_INFO("USB_DIAG_PRL_7KONLY_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7Konly_table;
		break;
	case USB_DIAG_PRL_9KONLY_SET:
		DIAG_INFO("USB_DIAG_PRL_9KONLY_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL9Konly_table;
		break;
	case USB_DIAG_PRL_7K9KDIFF_SET:
		DIAG_INFO("USB_DIAG_PRL_7K9KDIFF_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7K9Kdiff_table;
		break;

	case USB_DIAG_M29_7K9K_SET:
		DIAG_INFO("USB_DIAG_M29_7K9K_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297K9K_table;
		break;

	case USB_DIAG_M29_7KONLY_SET:
		DIAG_INFO("USB_DIAG_M29_7KONLY_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297Konly_table;
		break;
	case USB_DIAG_M29_9KONLY_SET:
		DIAG_INFO("USB_DIAG_M29_9KONLY_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M299Konly_table;
		break;
	case USB_DIAG_M29_7K9KDIFF_SET:
		DIAG_INFO("USB_DIAG_M29_7K9KDIFF_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297K9Kdiff_table;
		break;

	default:
		return -ENOTTY;
	}

		if (copy_from_user(temp_nv_table, (uint8_t *)argp, (table_size*2)))
			return -EFAULT;
		DIAG_INFO("%s:input %d item\n", __func__, temp_nv_table[0]);
		if (temp_nv_table[0] > table_size)
			return -EFAULT;

		spin_lock_irqsave(&ctxt->req_lock, flags);
		memcpy((uint8_t *)table_ptr, (uint8_t *)&temp_nv_table[0], (temp_nv_table[0]+1)*2);
		print_hex_dump(KERN_DEBUG, "TABLE Data: ", 16, 1,
		DUMP_PREFIX_ADDRESS, table_ptr, (*table_ptr+1)*2, 1);
		spin_unlock_irqrestore(&ctxt->req_lock, flags);



	return 0;
}
#endif
static int diag2arm9_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	int rc = 0;
	int n;
	DIAG_INFO("%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_lock);
	if (ctxt->diag2arm9_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	/* clear pending data if any */
	while ((req = req_get(ctxt, &ctxt->rx_arm9_done)))
		diag_req_free(req);

	for (n = 0; n < 4; n++) {
		req = diag_req_new(SMD_MAX);
		if (!req) {
			while ((req = req_get(ctxt, &ctxt->rx_arm9_idle)))
				diag_req_free(req);
			rc = -EFAULT;
			goto done;
		}
		req_put(ctxt, &ctxt->rx_arm9_idle, req);
	}
	ctxt->read_arm9_count = 0;
	ctxt->read_arm9_buf = 0;
	ctxt->read_arm9_req = 0;
	ctxt->diag2arm9_opened = true;

	diag_smd_enable(driver->ch, "diag2arm9_open", SMD_FUNC_OPEN_DIAG);

done:
	mutex_unlock(&ctxt->diag2arm9_lock);
	return rc;
}

static int diag2arm9_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;

	DIAG_INFO("%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_lock);
	ctxt->diag2arm9_opened = false;
	wake_up(&ctxt->read_arm9_wq);
	mutex_lock(&ctxt->diag2arm9_read_lock);
	while ((req = req_get(ctxt, &ctxt->rx_arm9_idle)))
		diag_req_free(req);
	while ((req = req_get(ctxt, &ctxt->rx_arm9_done)))
		diag_req_free(req);
	if (ctxt->read_arm9_req)
		diag_req_free(ctxt->read_arm9_req);
	mutex_unlock(&ctxt->diag2arm9_read_lock);

	/*************************************
	* If smd is closed, it will  lead to slate can't be tested.
	* slate will open it for one time
	* but close it for several times and never open
	*************************************/
	/*smd_diag_enable("diag2arm9_release", 0);*/
	mutex_unlock(&ctxt->diag2arm9_lock);

	return 0;
}

static ssize_t diag2arm9_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	int r = count;
	int writed = 0;
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	int path;
	unsigned char *buf_9k = NULL;
#endif
	DIAG_INFO("%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_write_lock);

	while (count > 0) {
		writed = count > SMD_MAX ? SMD_MAX : count;
		if (copy_from_user(ctxt->toARM9_buf, buf, writed)) {
			r = -EFAULT;
			break;
		}
		if (driver->ch == NULL) {
			DIAG_INFO("%s: driver->ch == NULL", __func__);
			r = -EFAULT;
			break;
		} else if (ctxt->toARM9_buf == NULL) {
			DIAG_INFO("%s: ctxt->toARM9_buf == NULL", __func__);
			r = -EFAULT;
			break;
		}

#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
		path = checkcmd_modem_epst(ctxt->toARM9_buf);

		switch (path) {
		case DM7K9K:
				DIAG_INFO("%s:DM7K9K sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				smd_write(driver->ch, ctxt->toARM9_buf, writed);

				if (sdio_diag_initialized) {
					buf_9k = kzalloc(writed, GFP_KERNEL);
					if (!buf_9k) {
						DIAG_INFO("%s:out of memory\n", __func__);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -ENOMEM;
					}
					memcpy(buf_9k, ctxt->toARM9_buf, writed);
					msm_sdio_diag_write((void *)buf_9k, writed);
					buf_9k = NULL;
				}
				break;
		case DM9KONLY:
				DIAG_INFO("%s:DM9KONLY sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				if (sdio_diag_initialized) {
					buf_9k = kzalloc(writed, GFP_KERNEL);
					if (!buf_9k) {
						DIAG_INFO("%s:out of memory\n", __func__);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -ENOMEM;
					}
					memcpy(buf_9k, ctxt->toARM9_buf, writed);
					msm_sdio_diag_write((void *)buf_9k, writed);
					buf_9k = NULL;
				}
				break;
		case DM7K9KDIFF:
				DIAG_INFO("%s:DM7K9KDIFF sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				if (((ctxt->toARM9_buf[3] & 0x80) == 0x80) && sdio_diag_initialized) {
					DIAG_INFO("%s:DM7K9KDIFF to 9K\n", __func__);

					buf_9k = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
					if (!buf_9k) {
						DIAG_INFO("%s:out of memory\n", __func__);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -ENOMEM;
					}
					if (decode_encode_hdlc(ctxt->toARM9_buf, &writed, buf_9k, 1, 3)) {
						kfree(buf_9k);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -EINVAL;
					}
					msm_sdio_diag_write((void *)buf_9k, writed);
					buf_9k = NULL;

				} else {
					DIAG_INFO("%s:DM7K9KDIFF to 7K\n", __func__);
					smd_write(driver->ch, ctxt->toARM9_buf, writed);
				}
				break;

		case DM7KONLY:
				//printk(KERN_INFO "%s:DM7KONLY sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				smd_write(driver->ch, ctxt->toARM9_buf, writed);
				break;
		case NO_DEF_ID:
		case NO_DEF_ITEM:
		default:
				DIAG_INFO("%s:no default routing path\n", __func__);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
		}
#else

	DIAG_INFO("%s: fix me\n", __func__);

				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio 7k ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
	//			smd_write(driver->ch, ctxt->toARM9_buf, writed);

#endif
//		ctxt->tx_count += writed;
		buf += writed;
		count -= writed;

	}

	mutex_unlock(&ctxt->diag2arm9_write_lock);

	return r;

}

static ssize_t diag2arm9_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	int r = 0, xfer;
	int ret;
	DIAG_INFO("%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_read_lock);

	/* if we have data pending, give it to userspace */
	if (ctxt->read_arm9_count > 0)
		req = ctxt->read_arm9_req;
	else {
retry:
	/* get data from done queue */
		req = 0;
		ret = wait_event_interruptible(ctxt->read_arm9_wq,
				((req = req_get(ctxt, &ctxt->rx_arm9_done)) ||
				!ctxt->diag2arm9_opened));
		if (!ctxt->diag2arm9_opened) {
			if (req)
				req_put(ctxt, &ctxt->rx_arm9_idle, req);
			goto done;
		}
		if (ret < 0 || req == 0)
			goto done;

		if (req->actual == 0) {
			req_put(ctxt, &ctxt->rx_arm9_idle, req);
			goto retry;
		}
		ctxt->read_arm9_req = req;
		ctxt->read_arm9_count = req->actual;
		ctxt->read_arm9_buf = req->buf;
	}
	xfer = (ctxt->read_arm9_count < count) ? ctxt->read_arm9_count : count;
	if (copy_to_user(buf, ctxt->read_arm9_buf, xfer)) {
		DIAG_INFO("diag: copy_to_user fail\n");
		r = -EFAULT;
		goto done;
	}
	ctxt->read_arm9_buf += xfer;
	ctxt->read_arm9_count -= xfer;
	r += xfer;
	/* if we've emptied the buffer, release the request */
	if (ctxt->read_arm9_count == 0) {
		print_hex_dump(KERN_DEBUG, "DM Packet Data"
		" read from radio ", 16, 1, DUMP_PREFIX_ADDRESS, req->buf, req->actual, 1);
		req_put(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
		ctxt->read_arm9_req = 0;
	}
done:
	mutex_unlock(&ctxt->diag2arm9_read_lock);
	return r;
}
static struct file_operations diag2arm9_fops = {
	.owner =   THIS_MODULE,
	.open =    diag2arm9_open,
	.release = diag2arm9_release,
	.write = diag2arm9_write,
	.read = diag2arm9_read,
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
	.unlocked_ioctl = diag2arm9_ioctl,
#endif
};

static struct miscdevice diag2arm9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag_arm9",
	.fops = &diag2arm9_fops,
};

#endif




static inline struct diag_context *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct diag_context, function);
}

static void usb_config_work_func(struct work_struct *work)
{
	struct diag_context *ctxt = container_of(work,
			struct diag_context, config_work);
#if 0
	struct usb_composite_dev *cdev = ctxt->cdev;
	struct usb_gadget_strings *table;
	struct usb_string *s;
#endif

	DIAG_INFO("%s: dev=%s\n", __func__, (ctxt == mdmctxt)?DIAG_MDM:DIAG_LEGACY);

	ctxt->tx_count = ctxt->rx_count = 0;
	ctxt->usb_in_count = ctxt->usb_out_count = 0;
	driver->diag_smd_count = driver->diag_qdsp_count = 0;
	if (ctxt->ch.notify && (!driver->usb_connected))
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_CONNECT, NULL);

#if 0
	if (!ctxt->pdata)
		return;

	/* pass on product id and serial number to dload */
	if (!cdev->desc.iSerialNumber) {
		ctxt->pdata->update_pid_and_serial_num(
					cdev->desc.idProduct, 0);
		return;
	}

	/*
	 * Serial number is filled by the composite driver. So
	 * it is fair enough to assume that it will always be
	 * found at first table of strings.
	 */
	table = *(cdev->driver->strings);
	for (s = table->strings; s && s->s; s++)
		if (s->id == cdev->desc.iSerialNumber) {
			ctxt->pdata->update_pid_and_serial_num(
					cdev->desc.idProduct, s->s);
			break;
		}
#endif
}

static void diag_write_complete(struct usb_ep *ep ,
		struct usb_request *req)
{
	struct diag_context *ctxt = ep->driver_data;
	struct diag_request *d_req = req->context;
	unsigned long flags;

	if (ctxt == NULL) {
		DIAG_INFO("%s: requesting"
				"NULL device pointer\n", __func__);
		return;
	}
	if (!req->status) {
		if ((req->length >= ep->maxpacket) &&
				((req->length % ep->maxpacket) == 0)) {
			req->length = 0;
			d_req->actual = req->actual;
			d_req->status = req->status;
			/* Queue zero length packet */
			usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
			return;
		}
	}

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->write_pool);
	if (req->length != 0) {
		d_req->actual = req->actual;
		d_req->status = req->status;
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);

	if (ctxt->ch.notify)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_WRITE_DONE, d_req);
}

static void diag_read_complete(struct usb_ep *ep ,
		struct usb_request *req)
{
	struct diag_context *ctxt = ep->driver_data;
	struct diag_request *d_req = req->context;
	struct usb_request *xpst_req;
	unsigned long flags;
#if DIAG_XPST
	unsigned int cmd_id;
#endif

	d_req->actual = req->actual;
	d_req->status = req->status;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->read_pool);
	spin_unlock_irqrestore(&ctxt->lock, flags);


#if DIAG_XPST

#ifdef HTC_DIAG_DEBUG
	print_hex_dump(KERN_DEBUG, "from PC: ", 16, 1,
	 DUMP_PREFIX_ADDRESS, req->buf, req->actual, 1);
#endif

		cmd_id = *((unsigned short *)req->buf);
		if (if_route_to_userspace(ctxt, cmd_id)) {
			xpst_req = req_get(ctxt, &ctxt->rx_req_idle);
			if (xpst_req) {
				xpst_req->actual = req->actual;
				xpst_req->status = req->status;
				memcpy(xpst_req->buf, req->buf, req->actual);
				req_put(ctxt, &ctxt->rx_req_user, xpst_req);
				wake_up(&ctxt->read_wq);
				driver->nohdlc = 1;
			} else
				DIAG_INFO("%s No enough xpst_req \n", __func__);
		} else {
			driver->nohdlc = 0;
			ctxt->tx_count += req->actual;
		}
#endif
			ctxt->usb_out_count += req->actual;
	if (ctxt->ch.notify)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_READ_DONE, d_req);


}

/**
 * usb_diag_open() - Open a diag channel over USB
 * @name: Name of the channel
 * @priv: Private structure pointer which will be passed in notify()
 * @notify: Callback function to receive notifications
 *
 * This function iterates overs the available channels and returns
 * the channel handler if the name matches. The notify callback is called
 * for CONNECT, DISCONNECT, READ_DONE and WRITE_DONE events.
 *
 */
struct usb_diag_ch *usb_diag_open(const char *name, void *priv,
		void (*notify)(void *, unsigned, struct diag_request *))
{
	struct usb_diag_ch *ch;
	unsigned long flags;
	int found = 0;

	DIAG_DBUG("%s: \n", __func__);


	spin_lock_irqsave(&ch_lock, flags);

	list_for_each_entry(ch, &usb_diag_ch_list, list) {
		if (!strcmp(name, ch->name)) {
			found = 1;
			break;
		}
	}

	if (!found) {
		ch =  ERR_PTR(-ENOENT);
		goto out;
	}

	if (ch->priv) {
		ch = ERR_PTR(-EBUSY);
		goto out;
	}

	ch->priv = priv;
	ch->notify = notify;
out:
	spin_unlock_irqrestore(&ch_lock, flags);
	return ch;
}
EXPORT_SYMBOL(usb_diag_open);

/**
 * usb_diag_close() - Close a diag channel over USB
 * @ch: Channel handler
 *
 * This function closes the diag channel.
 *
 */
void usb_diag_close(struct usb_diag_ch *ch)
{
	unsigned long flags;


	DIAG_DBUG("%s: \n", __func__);

	spin_lock_irqsave(&ch_lock, flags);
	ch->priv = NULL;
	ch->notify = NULL;
	spin_unlock_irqrestore(&ch_lock, flags);
}
EXPORT_SYMBOL(usb_diag_close);

/**
 * usb_diag_free_req() - Free USB requests
 * @ch: Channel handler
 *
 * This function free read and write USB requests for the interface
 * associated with this channel.
 *
 */
void usb_diag_free_req(struct usb_diag_ch *ch)
{
	struct diag_context *ctxt = ch->priv_usb;
	struct usb_request *req;
	struct list_head *act, *tmp;

	DIAG_INFO("%s: dev=%s\n", __func__, (ctxt == mdmctxt)?DIAG_MDM:DIAG_LEGACY);

	if (!ctxt)
		return;

	list_for_each_safe(act, tmp, &ctxt->write_pool) {
		req = list_entry(act, struct usb_request, list);
		list_del(&req->list);
		usb_ep_free_request(ctxt->in, req);
	}

	list_for_each_safe(act, tmp, &ctxt->read_pool) {
		req = list_entry(act, struct usb_request, list);
		list_del(&req->list);
		usb_ep_free_request(ctxt->out, req);
	}
}
EXPORT_SYMBOL(usb_diag_free_req);

/**
 * usb_diag_alloc_req() - Allocate USB requests
 * @ch: Channel handler
 * @n_write: Number of requests for Tx
 * @n_read: Number of requests for Rx
 *
 * This function allocate read and write USB requests for the interface
 * associated with this channel. The actual buffer is not allocated.
 * The buffer is passed by diag char driver.
 *
 */
int usb_diag_alloc_req(struct usb_diag_ch *ch, int n_write, int n_read)
{
	struct diag_context *ctxt = ch->priv_usb;
	struct usb_request *req;
	int i;

	DIAG_INFO("%s: dev=%s\n", __func__, (ctxt == mdmctxt)?DIAG_MDM:DIAG_LEGACY);

	if (!ctxt)
		return -ENODEV;

	for (i = 0; i < n_write; i++) {
		req = usb_ep_alloc_request(ctxt->in, GFP_ATOMIC);
		if (!req)
			goto fail;
		req->complete = diag_write_complete;
		list_add_tail(&req->list, &ctxt->write_pool);
	}

	for (i = 0; i < n_read; i++) {
		req = usb_ep_alloc_request(ctxt->out, GFP_ATOMIC);
		if (!req)
			goto fail;
		req->complete = diag_read_complete;
		list_add_tail(&req->list, &ctxt->read_pool);
	}

	return 0;

fail:
	usb_diag_free_req(ch);
	return -ENOMEM;

}
EXPORT_SYMBOL(usb_diag_alloc_req);

/**
 * usb_diag_read() - Read data from USB diag channel
 * @ch: Channel handler
 * @d_req: Diag request struct
 *
 * Enqueue a request on OUT endpoint of the interface corresponding to this
 * channel. This function returns proper error code when interface is not
 * in configured state, no Rx requests available and ep queue is failed.
 *
 * This function operates asynchronously. READ_DONE event is notified after
 * completion of OUT request.
 *
 */
int usb_diag_read(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	struct diag_context *ctxt = ch->priv_usb;
	unsigned long flags;
	struct usb_request *req = NULL;

#ifdef HTC_DIAG_DEBUG
	DIAG_DBUG("%s: \n", __func__);
#endif

	spin_lock_irqsave(&ctxt->lock, flags);

	if (!ctxt->configured) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return -EIO;
	}

	if (list_empty(&ctxt->read_pool)) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: no requests available\n", __func__);
		return -EAGAIN;
	}

	req = list_first_entry(&ctxt->read_pool, struct usb_request, list);
	list_del(&req->list);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	req->buf = d_req->buf;
	req->length = d_req->length;
	req->context = d_req;
	if (usb_ep_queue(ctxt->out, req, GFP_ATOMIC)) {
		/* If error add the link to linked list again*/
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->read_pool);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: cannot queue"
				" read request\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(usb_diag_read);

/**
 * usb_diag_write() - Write data from USB diag channel
 * @ch: Channel handler
 * @d_req: Diag request struct
 *
 * Enqueue a request on IN endpoint of the interface corresponding to this
 * channel. This function returns proper error code when interface is not
 * in configured state, no Tx requests available and ep queue is failed.
 *
 * This function operates asynchronously. WRITE_DONE event is notified after
 * completion of IN request.
 *
 */
int usb_diag_write(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	struct diag_context *ctxt = ch->priv_usb;
	unsigned long flags;
	struct usb_request *req = NULL;

#ifdef HTC_DIAG_DEBUG
	DIAG_DBUG("%s: \n", __func__);
#endif

	spin_lock_irqsave(&ctxt->lock, flags);

	if (!ctxt->configured) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return -EIO;
	}

	if (list_empty(&ctxt->write_pool)) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: no requests available\n", __func__);
		return -EAGAIN;
	}

	req = list_first_entry(&ctxt->write_pool, struct usb_request, list);
	list_del(&req->list);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	req->buf = d_req->buf;
	req->length = d_req->length;
	req->context = d_req;

	if (usb_ep_queue(ctxt->in, req, GFP_ATOMIC)) {
		/* If error add the link to linked list again*/
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->write_pool);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: cannot queue"
				" read request\n", __func__);
		return -EIO;
	}
	ctxt->usb_in_count += d_req->length;
	return 0;
}
EXPORT_SYMBOL(usb_diag_write);

struct usb_diag_ch *diag_setup(void)
{

	unsigned long flags;

#ifdef HTC_DIAG_DEBUG
	DIAG_DBUG("%s: \n", __func__);
#endif

/*	ctxt = kzalloc(sizeof(*ctxt), GFP_KERNEL);*/
	legacyctxt = &_context;
	legacych = &legacyctxt->ch;

	spin_lock_irqsave(&ch_lock, flags);
	legacych->name = DIAG_LEGACY;
	list_add_tail(&legacych->list, &usb_diag_ch_list);
	spin_unlock_irqrestore(&ch_lock, flags);

#if defined(CONFIG_USB_ANDROID_LTE_DIAG)
	mdmctxt = kzalloc(sizeof(*mdmctxt), GFP_KERNEL);
	if (!mdmctxt)
		return ERR_PTR(-ENOMEM);
	mdmch	 = &mdmctxt->ch;

	spin_lock_irqsave(&ch_lock, flags);
	mdmch->name = DIAG_MDM;
	list_add_tail(&mdmch->list, &usb_diag_ch_list);
	spin_unlock_irqrestore(&ch_lock, flags);
#endif

	return legacych;
}
EXPORT_SYMBOL(diag_setup);
static void diag_function_disable(struct usb_function *f)
{
	struct diag_context  *dev = func_to_dev(f);
	unsigned long flags;


	DIAG_INFO("%s: dev=%s\n", __func__, (dev == mdmctxt)?DIAG_MDM:DIAG_LEGACY);


	spin_lock_irqsave(&dev->lock, flags);
	diag_configured = dev->configured = 0;

	spin_unlock_irqrestore(&dev->lock, flags);

	if (dev->ch.notify && (driver->usb_connected))
		dev->ch.notify(dev->ch.priv, USB_DIAG_DISCONNECT, NULL);

		usb_ep_disable(dev->in);
		dev->in->driver_data = NULL;

		usb_ep_disable(dev->out);
		dev->out->driver_data = NULL;

#if DIAG_XPST
	if (dev == legacyctxt) {
		dev->online = 0;
		wake_up(&dev->read_wq);
	}
#endif


}

static int diag_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct diag_context  *dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	unsigned long flags;
	int rc = 0;
#if DIAG_XPST
	struct usb_request *req;
#endif


	DIAG_INFO("%s: dev=%s\n", __func__, (dev == mdmctxt)?DIAG_MDM:DIAG_LEGACY);

	if (!dev)
		return -ENODEV;

	dev->in_desc = ep_choose(cdev->gadget,
			&hs_bulk_in_desc, &fs_bulk_in_desc);
	dev->out_desc = ep_choose(cdev->gadget,
			&hs_bulk_out_desc, &fs_bulk_in_desc);
	dev->in->driver_data = dev;
	rc = usb_ep_enable(dev->in, dev->in_desc);
	if (rc) {
		ERROR(dev->cdev, "can't enable %s, result %d\n",
						dev->in->name, rc);
		return rc;
	}
	dev->out->driver_data = dev;
	rc = usb_ep_enable(dev->out, dev->out_desc);
	if (rc) {
		ERROR(dev->cdev, "can't enable %s, result %d\n",
						dev->out->name, rc);
		usb_ep_disable(dev->in);
		return rc;
	}
	dev->i_serial_number = cdev->desc.iSerialNumber;
	dev->product_id   = cdev->desc.idProduct;
	schedule_work(&dev->config_work);

	spin_lock_irqsave(&dev->lock, flags);
	diag_configured = dev->configured = 1;
#if DIAG_XPST
	if (dev == legacyctxt) {
		while ((req = req_get(dev, &dev->rx_req_user)))
			req_put(dev, &dev->rx_req_idle, req);
		dev->online = !dev->function.hidden;
		wake_up(&dev->read_wq);
	}
#endif
	spin_unlock_irqrestore(&dev->lock, flags);

	return rc;
}

static void diag_function_unbind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct diag_context *ctxt = func_to_dev(f);

	DIAG_INFO("%s: dev=%s\n", __func__, (ctxt == mdmctxt)?DIAG_MDM:DIAG_LEGACY);

	if (!ctxt)
		return;
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);

	usb_free_descriptors(f->descriptors);
	ctxt->ch.priv_usb = NULL;
#if DIAG_XPST
	if (ctxt == legacyctxt) {
		misc_deregister(&htc_diag_device_fops);
		misc_deregister(&diag2arm9_device);
		ctxt->tx_count = ctxt->rx_count = 0;
		ctxt->usb_in_count = ctxt->usb_out_count = 0;
		driver->diag_smd_count = driver->diag_qdsp_count = 0;
	}
#endif
}

static int diag_function_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct diag_context *ctxt = func_to_dev(f);
	struct usb_ep      *ep;
	int status = -ENODEV;


	DIAG_INFO("%s: dev=%s\n", __func__, (ctxt == mdmctxt)?DIAG_MDM:DIAG_LEGACY);


	if (!ctxt)
		return status;

	ctxt->cdev = cdev;
	intf_desc.bInterfaceNumber =  usb_interface_id(c, f);

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_in_desc);
	if (!ep)
		goto fail;
	ctxt->in = ep;
	ep->driver_data = ctxt;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_out_desc);
	if (!ep)
		goto fail;
	ctxt->out = ep;
	ep->driver_data = ctxt;

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(fs_diag_desc);
	if (!f->descriptors)
		goto fail;

	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hs_bulk_in_desc.bEndpointAddress =
				fs_bulk_in_desc.bEndpointAddress;
		hs_bulk_out_desc.bEndpointAddress =
				fs_bulk_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(hs_diag_desc);
	}
#if DIAG_XPST
	if (ctxt == legacyctxt) {
		misc_register(&htc_diag_device_fops);
/*DMrounter*/
		misc_register(&diag2arm9_device);
		ctxt->usb_in_count = ctxt->usb_out_count = 0;
		ctxt->tx_count = ctxt->rx_count = 0;
		driver->diag_smd_count = driver->diag_qdsp_count = 0;
	}
#endif
	return 0;
fail:
	if (ctxt->out)
		ctxt->out->driver_data = NULL;
	if (ctxt->in)
		ctxt->in->driver_data = NULL;
	return status;

}
static struct usb_string diag_string_defs[] = {
	[0].s = "HTC DIAG",
	[1].s = "HTC 9K DIAG",
	{  } /* end of list */
};

static struct usb_gadget_strings diag_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		diag_string_defs,
};

static struct usb_gadget_strings *diag_strings[] = {
	&diag_string_table,
	NULL,
};

int diag_function_add(struct usb_configuration *c)
{
	struct diag_context *dev;
	struct usb_diag_ch *_ch;
	int found = 0, ret;


	list_for_each_entry(_ch, &usb_diag_ch_list, list) {
		/* Find an unused channel */
		if (!_ch->priv_usb) {
			found = 1;
			break;
		}
	}
	if (!found) {
		ERROR(c->cdev, "unable to get diag usb channel\n");
		return -ENODEV;
	}

	dev = container_of(_ch, struct diag_context, ch);
	/* claim the channel for this USB interface */
	_ch->priv_usb = dev;


	ret = usb_string_id(c->cdev);
	DIAG_DBUG("%s: ret=%d\n", __func__, ret);
	if (ret < 0)
		return ret;
	if (dev == legacyctxt)
		diag_string_defs[0].id = ret;
	else
		diag_string_defs[1].id = ret;
	intf_desc.iInterface = ret;

	dev->cdev = c->cdev;
	dev->function.name = _ch->name;
	dev->function.strings = diag_strings;
	dev->function.descriptors = fs_diag_desc;
	dev->function.hs_descriptors = hs_diag_desc;
	dev->function.bind = diag_function_bind;
	dev->function.unbind = diag_function_unbind;
	dev->function.set_alt = diag_function_set_alt;
	dev->function.disable = diag_function_disable;
	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->read_pool);
	INIT_LIST_HEAD(&dev->write_pool);
	INIT_WORK(&dev->config_work, usb_config_work_func);

	dev->function.hidden = !_context.function_enable;

#if defined(CONFIG_MACH_MECHA)
	/*for internal hub*/
	smsc251x_set_diag_boot_flag(_context.function_enable);
#endif

#if DIAG_XPST
	if (dev == legacyctxt) {

		spin_lock_init(&dev->req_lock);
		mutex_init(&dev->user_lock);
		INIT_LIST_HEAD(&dev->rx_req_user);
		INIT_LIST_HEAD(&dev->rx_req_idle);
		init_waitqueue_head(&dev->read_wq);
		INIT_LIST_HEAD(&dev->rx_arm9_idle);
		INIT_LIST_HEAD(&dev->rx_arm9_done);
		init_waitqueue_head(&dev->read_arm9_wq);
		mutex_init(&dev->diag2arm9_lock);
		mutex_init(&dev->diag2arm9_read_lock);
		mutex_init(&dev->diag2arm9_write_lock);
	}
#endif
DIAG_DBUG("%s: dev->function.hidden =%d\n", __func__, dev->function.hidden );
	ret = usb_add_function(c, &dev->function);
	if (ret) {
		INFO(c->cdev, "usb_add_function failed\n");
		_ch->priv_usb = NULL;
	}

	return ret;
}

/*
static struct android_usb_function diag_function = {
	.name = DIAG_LEGACY,
	.bind_config = diag_function_add,
};*/
static int diag2sd_probe(struct platform_device *pdev)
{
	struct diag2sd_platform_data *pdata = pdev->dev.platform_data;
	DIAG_DBUG("%s: \n", __func__);
	if (pdata->enable_sd_log)
		driver->enable_sd_log = pdata->enable_sd_log;

	return 0;
}
static struct platform_driver diag2sd_platform_driver = {
	.driver = {
		.name = "diag2sd",
		},
	.probe  = diag2sd_probe,
};

static int __init usb_diag_init(void)
{
	struct diag_context *dev;
	struct android_usb_function *func;

	DIAG_DBUG("%s: \n", __func__);

	dev = container_of(legacych, struct diag_context, ch);

	func = &dev->android_function;
	func->name = DIAG_LEGACY;
	func->bind_config = diag_function_add;

	android_register_function(func);

#if defined(CONFIG_USB_ANDROID_LTE_DIAG)
	dev = container_of(mdmch, struct diag_context, ch);

	func = &dev->android_function;
	func->name = DIAG_MDM;
	func->bind_config = diag_function_add;
	android_register_function(func);

#endif
	platform_driver_register(&diag2sd_platform_driver);
	return 0;
}

module_init(usb_diag_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("usb diag gadget driver");
MODULE_VERSION("2.00");


static int diag_set_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	DIAG_INFO("%s: %d\n", __func__, enabled);

	if (_context.cdev) {
#if defined(CONFIG_USB_ANDROID_LTE_DIAG)
		android_enable_function(&mdmctxt->function, enabled);
#endif
		android_enable_function(&_context.function, enabled);
		diag_smd_enable(driver->ch, "diag_set_enabled", !!enabled);

	}
	else {
#if defined(CONFIG_MACH_MECHA) //|| defined(CONFIG_ARCH_MSM8X60_LTE)
		sdio_diag_init_enable = !enabled;
		DIAG_INFO("%s: sdio_diag_init_enable=%d\n", __func__, enabled);
#elif defined(CONFIG_ARCH_MSM8X60_LTE)
		diag_init_enabled_state = !!enabled;
#endif
	}
	_context.function_enable = !!enabled;

	return 0;
}

static int diag_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !_context.function.hidden;
	return 1;
}
module_param_call(enabled, diag_set_enabled, diag_get_enabled, NULL, 0664);
static int show_diag_xfer_count(char *buffer, struct kernel_param *kp)
{
	struct diag_context *ctxt = &_context;
	ctxt->rx_count = driver->diag_smd_count + driver->diag_qdsp_count;
/*	return sprintf(buffer, "tx: %llu bytes, rx: %llu bytes",
	ctxt->tx_count, ctxt->rx_count);*/
	return  sprintf(buffer, "tx_count: %llu, rx_count: %llu\n",
		ctxt->tx_count, ctxt->rx_count);
}
module_param_call(diag_xfer_count, NULL, show_diag_xfer_count, NULL, 0444);
static int diag_get_usb_inout_count(char *buffer, struct kernel_param *kp)
{
	struct diag_context *ctxt = &_context;

	return sprintf(buffer, "FromPC: %llu bytes, ToPC: %llu bytes",
	ctxt->usb_out_count, ctxt->usb_in_count);
}
module_param_call(usb_inout_count, NULL, diag_get_usb_inout_count, NULL, 0444);
