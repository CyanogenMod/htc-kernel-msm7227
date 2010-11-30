/* arch/arm/mach-msm/qdsp5_comp/vocpcm.c
 *
 * pcm audio input device
 *
 * Copyright (C) 2008 HTC Corporation
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>

#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/msm_adsp.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_rpc_version.h>

#define VOC_PCM_CLIENT_OUTPUT_PTR 2
#define VOC_PCM_CLIENT_INPUT_PTR 3

#define RPC_SND_PROG	0x30000002
#define RPC_SND_CB_PROG	0x31000002

#define FRAME_NUM	(12)
#define FRAME_SIZE	(160)
#define BUFFER_NUM	(2)
#define BUFFER_SIZE	(FRAME_NUM * FRAME_SIZE)

struct task_struct *create_snd_rpc_kthread(void);
struct msm_rpc_endpoint *create_rpc_connect(uint32_t, uint32_t, unsigned);

struct buffer {
	uint16_t data[BUFFER_SIZE];
	uint32_t index;
};

struct voc_ctxt {
	struct miscdevice misc;
	struct buffer buf[BUFFER_NUM];
	struct mutex lock;

	spinlock_t dsp_lock;
	wait_queue_head_t wait;
	wait_queue_head_t last_write_wait;

	uint32_t head;
	uint32_t tail;
	uint32_t count;
	int opened;
	int intr;
	int client;
	int final_input;
	uint8_t *s_ptr;
};

struct voc_rpc {
	struct msm_rpc_endpoint *ept;
	struct task_struct *task;
	struct mutex lock;
	int inited;
};

#define VOC_PCM_INTERFACE_RX_OUTPUT    (0)
#define VOC_PCM_INTERFACE_RX_INPUT     (1)
#define VOC_PCM_INTERFACE_TX_OUTPUT    (2)
#define VOC_PCM_INTERFACE_TX_INPUT     (3)

#define VOC_PCM_DATA_STATUS_AVAILABLE     (0)/* Data available for PCM input */
#define VOC_PCM_DATA_STATUS_UNAVAILABLE   (1)  /* Data not available        */
#define VOC_PCM_DATA_STATUS_MAX           (2)


struct vocpcm_register_client_msg {
	struct rpc_request_hdr hdr;
	uint32_t intr;
	uint32_t client_func;
};

struct vocpcm_input_callback_msg {
	uint32_t rep[6];
	uint32_t type;
	uint32_t size;
	uint32_t buf[160];
};

static struct voc_rpc the_voc_proc;
static struct voc_ctxt *voc_minor_to_ctxt(unsigned n);
static struct voc_ctxt *voc_intr_to_ctxt(unsigned n);
static struct vocpcm_input_callback_msg msg;

void put_vocpcm_data(uint32_t *pcm_data, uint32_t cb_id, uint32_t len,
		struct msm_rpc_endpoint *ept, uint32_t xid)
{
	struct voc_ctxt *ctxt = voc_intr_to_ctxt(cb_id);

	unsigned long flags;
	uint32_t buf_index;
	uint32_t data_index;
	struct buffer *frame;
	uint16_t *data;
	int i = 0;

	memset(msg.buf , 0 , FRAME_SIZE);

	if (ctxt == NULL)
		return;

	if (ctxt->count > 0) {

		if (len != FRAME_SIZE) {
			pr_err("len error\n");
			return;
		}
		spin_lock_irqsave(&ctxt->dsp_lock, flags);

		buf_index = ctxt->tail;
		frame = &ctxt->buf[buf_index];
		data_index = frame->index * FRAME_SIZE;
		data = &frame->data[data_index];

		for (i = 0; i < FRAME_SIZE; i++)
			msg.buf[i] = cpu_to_be32(*(data+i));

		if (++frame->index >= FRAME_NUM) {
			frame->index = 0;
			ctxt->tail = (ctxt->tail + 1) & (BUFFER_NUM - 1);
			if (ctxt->head == ctxt->tail) {
				ctxt->head = (ctxt->head + 1) &
						(BUFFER_NUM - 1);
				if (!ctxt->final_input) {
					pr_err(" when head=tail,"
						" head= %d, tail=%d \n",
						ctxt->head, ctxt->tail);
				} else {
					ctxt->count--;
				}
			} else
				ctxt->count--;

			spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
			wake_up(&ctxt->wait);
			wake_up(&ctxt->last_write_wait);
		} else
			spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
	} else {
		pr_err("rpc miss data and return \n");
	}

}

void get_vocpcm_data(const uint32_t *pcm_data, uint32_t cb_id, uint32_t len)
{
	struct voc_ctxt *ctxt = voc_intr_to_ctxt(cb_id);
	unsigned long flags;
	uint32_t buf_index;
	uint32_t data_index;
	struct buffer *frame;
	uint16_t *data;
	int i = 0;

	if (ctxt == NULL)
		return;

	buf_index = ctxt->head;
	frame = &ctxt->buf[buf_index];

	if (len != FRAME_SIZE) {
		pr_err("len error\n");
		return;
	}
	spin_lock_irqsave(&ctxt->dsp_lock, flags);
	data_index = frame->index * FRAME_SIZE;
	data = &frame->data[data_index];

	for (i = 0; i < FRAME_SIZE; i++)
		*(data+i) = be32_to_cpu(*(pcm_data+i));
	if (++frame->index == FRAME_NUM) {
		frame->index = 0;
		ctxt->head = (ctxt->head + 1) & (BUFFER_NUM - 1);
		if (ctxt->head == ctxt->tail) {
			ctxt->tail = (ctxt->tail + 1) & (BUFFER_NUM - 1);
			pr_err(" when head=tail, head= %d, tail=%d \n",
						ctxt->head, ctxt->tail);
		}
		else
			ctxt->count++;
		spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
		wake_up(&ctxt->wait);
	} else
		spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
}

void rpc_in_ack(struct msm_rpc_endpoint *ept, uint32_t xid)
{
	int rc;
	msg.rep[0] = cpu_to_be32(xid);
	msg.rep[1] = cpu_to_be32(1);
	msg.rep[2] = cpu_to_be32(RPCMSG_REPLYSTAT_ACCEPTED);
	msg.rep[3] = cpu_to_be32(RPC_ACCEPTSTAT_SUCCESS);
	msg.rep[4] = cpu_to_be32(0);
	msg.rep[5] = cpu_to_be32(0);
	msg.type = cpu_to_be32(0);
	msg.size = cpu_to_be32(160);

	rc = msm_rpc_write(ept, &msg, sizeof(msg));
	if (rc < 0)
		pr_info("rpc_in_ack %d %d\n", rc, sizeof(msg));
}

#define RPC_TYPE_REQUEST 0
#define RPC_TYPE_REPLY 1

#define RPC_VERSION 2

#define RPC_COMMON_HDR_SZ  (sizeof(uint32_t) * 2)
#define RPC_REQUEST_HDR_SZ (sizeof(struct rpc_request_hdr))
#define RPC_REPLY_HDR_SZ   (sizeof(uint32_t) * 3)
#define RPC_REPLY_SZ       (sizeof(uint32_t) * 6)

static int voc_register_client(struct voc_ctxt *ctxt)
{
	struct voc_rpc *voc_rpc = &the_voc_proc;
	int rc;

	struct vocpcm_register_client_msg msg;
	msg.intr = cpu_to_be32(ctxt->intr);
	msg.client_func = cpu_to_be32(ctxt->intr);

	if (ctxt->intr % 2) {
		msm_rpc_setup_req(&msg.hdr, RPC_SND_PROG, RPC_SND_VERS,
				VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC);
	} else {
		msm_rpc_setup_req(&msg.hdr, RPC_SND_PROG, RPC_SND_VERS,
				VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC);
	}

	rc = msm_rpc_write(voc_rpc->ept, &msg, sizeof(msg));
	if (rc < 0) {
		pr_err("%s: %d failed\n", __func__, ctxt->intr);
	} else {
		pr_info("%s: %d success\n", __func__, ctxt->intr);
		ctxt->client = 1;
	}

	return rc;
}

static int voc_unregister_client(struct voc_ctxt *ctxt)
{
	struct voc_rpc *voc_rpc = &the_voc_proc;
	int rc;

	struct vocpcm_register_client_msg msg;
	msg.intr = cpu_to_be32(ctxt->intr);
	msg.client_func = cpu_to_be32(0xffffffff);

	if (ctxt->intr % 2) {
		msm_rpc_setup_req(&msg.hdr, RPC_SND_PROG, RPC_SND_VERS,
				VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC);
	} else {
		msm_rpc_setup_req(&msg.hdr, RPC_SND_PROG, RPC_SND_VERS,
				VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC);
	}

	rc = msm_rpc_write(voc_rpc->ept, &msg, sizeof(msg));
	if (rc < 0) {
		pr_err("%s: %d failed\n", __func__, ctxt->intr);
	} else {
		pr_info("%s: %d success\n", __func__, ctxt->intr);
		ctxt->client = 0;
	}
	return rc;
}

/* ------------------- device --------------------- */
#define VOCPCM_IOCTL_MAGIC 'v'

#define VOCPCM_REGISTER_CLIENT          _IOW(VOCPCM_IOCTL_MAGIC, 0, unsigned)
#define VOCPCM_UNREGISTER_CLIENT        _IOW(VOCPCM_IOCTL_MAGIC, 1, unsigned)

static long vocpcm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct voc_ctxt *ctxt = file->private_data;
	struct buffer *frame;
	unsigned long flags;
	uint32_t index;
	uint32_t data_index;
	uint32_t len = 0;
	uint8_t *dest;
	int rc = 0;

	mutex_lock(&ctxt->lock);
	switch (cmd) {
	case VOCPCM_REGISTER_CLIENT:
		rc = voc_register_client(ctxt);
		break;

	case VOCPCM_UNREGISTER_CLIENT:
		if (ctxt->intr % 2) {
			if (ctxt->s_ptr) {
				index = ctxt->head;
				frame = &ctxt->buf[index];
				data_index = FRAME_NUM * FRAME_SIZE - 1;
				dest = (uint8_t *)&frame->data[data_index] + 1;
				len = dest - ctxt->s_ptr + 1;

				memset(ctxt->s_ptr, 0, len);

				spin_lock_irqsave(&ctxt->dsp_lock, flags);
				frame->index = 0;
				ctxt->head = (ctxt->head + 1) &
						 (BUFFER_NUM - 1);
				ctxt->count++;
				ctxt->final_input = 1;
				spin_unlock_irqrestore(&ctxt->dsp_lock, flags);

				rc = wait_event_interruptible_timeout(
						ctxt->last_write_wait,
						ctxt->count == 0,
						5 * HZ);
				if (rc < 0)
					break;
			}
		}

		if (ctxt->client)
			rc = voc_unregister_client(ctxt);
		else
			pr_err("no %d client to unregister.", ctxt->intr);
		break;

	default:
		pr_err("unknown command.\n");
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&ctxt->lock);

	return rc;
}

static ssize_t vocpcm_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	struct voc_ctxt *ctxt = file->private_data;
	struct buffer *frame;
	const char __user *start = buf;
	unsigned long flags;
	uint32_t index;
	uint32_t data_index;
	uint32_t len = 0;
	uint32_t actual_len = 0;
	uint8_t *dest;
	int rc = 0;

	if (ctxt->intr % 2) {
		return -EFAULT;
	}

	while (count > 0) {
		rc = wait_event_interruptible_timeout(ctxt->wait,
						 ctxt->count > 0,
						 5 * HZ);
		if (!rc)
			rc = -ETIMEDOUT;

		if (rc < 0)
			break;

		index = ctxt->tail;
		frame = &ctxt->buf[index];

		if (ctxt->s_ptr == NULL)
			ctxt->s_ptr = (uint8_t *)&frame->data;

		data_index = FRAME_NUM * FRAME_SIZE - 1;
		dest = (uint8_t *)&frame->data[data_index] + 1;

		len = dest - ctxt->s_ptr + 1;

		actual_len = (len > count) ? count : len;
		if (copy_to_user(buf, ctxt->s_ptr, actual_len)) {
		rc = -EFAULT;
		break;
		}

		spin_lock_irqsave(&ctxt->dsp_lock, flags);
		if (index != ctxt->tail) {
			/* overrun -- data is invalid
			    and we need to retry */
			spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
			continue;
		}

		if (len > count) {
			ctxt->s_ptr += count;
		} else {
			frame->index = 0;
			ctxt->tail = (ctxt->tail + 1) & (BUFFER_NUM - 1);
			ctxt->count--;
			ctxt->s_ptr = 0;
		}
		spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
		buf += actual_len;
		count -= actual_len;
	}

	if (buf > start)
		return buf - start;

	return rc;
}

static ssize_t vocpcm_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *pos)
{
	struct voc_ctxt *ctxt = file->private_data;
	struct buffer *frame;
	const char __user *start = buf;
	unsigned long flags;
	uint32_t index;
	uint32_t data_index;
	uint32_t len = 0;
	uint32_t actual_len = 0;
	uint8_t *dest;
	int rc = 0;

	if ((ctxt->intr % 2) == 0) {
		return -EFAULT;
	}

	while (count > 0) {
		rc = wait_event_interruptible_timeout(ctxt->wait,
					ctxt->count < BUFFER_NUM,
					5 * HZ);
		if (!rc)
			rc = -ETIMEDOUT;

		if (rc < 0)
			break;

		index = ctxt->head;
		frame = &ctxt->buf[index];

		if (ctxt->s_ptr == NULL)
			ctxt->s_ptr = (uint8_t *)&frame->data;

		data_index = FRAME_NUM * FRAME_SIZE - 1;
		dest = (uint8_t *)&frame->data[data_index] + 1;

		len = dest - ctxt->s_ptr + 1;

		actual_len = (len > count) ? count : len;
		if (copy_from_user(ctxt->s_ptr, buf, actual_len)) {
		rc = -EFAULT;
		break;
		}

		spin_lock_irqsave(&ctxt->dsp_lock, flags);
		if (index != ctxt->head) {
			/* overrun -- data is invalid
			    and we need to retry */
			spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
			continue;
		}

		if (len > count) {
			ctxt->s_ptr += count;
		} else {
			frame->index = 0;
			ctxt->head = (ctxt->head + 1) & (BUFFER_NUM - 1);
			ctxt->count++;
			ctxt->s_ptr = 0;
		}
		spin_unlock_irqrestore(&ctxt->dsp_lock, flags);
		buf += actual_len;
		count -= actual_len;
	}

	if (buf > start)
		return buf - start;

	return rc;
}

static int vocpcm_release(struct inode *inode, struct file *file)
{
	struct voc_ctxt *ctxt = file->private_data;

	mutex_lock(&ctxt->lock);
	if (ctxt->client)
		voc_unregister_client(ctxt);
	ctxt->opened = 0;
	mutex_unlock(&ctxt->lock);
	return 0;
}

static int vocpcm_open(struct inode *inode, struct file *file)
{
	struct voc_rpc *voc_rpc = &the_voc_proc;
	struct voc_ctxt *ctxt = voc_minor_to_ctxt(MINOR(inode->i_rdev));
	int rc = 0;

	if (!ctxt) {
		pr_err("unknown voc misc %d\n", MINOR(inode->i_rdev));
		return -ENODEV;
	}

	mutex_lock(&voc_rpc->lock);
	if (voc_rpc->inited == 0) {
		voc_rpc->ept = create_rpc_connect(RPC_SND_PROG, RPC_SND_VERS,
					MSM_RPC_UNINTERRUPTIBLE |
					MSM_RPC_ENABLE_RECEIVE);
		if (IS_ERR(voc_rpc->ept)) {
			rc = PTR_ERR(voc_rpc->ept);
			voc_rpc->ept = NULL;
			pr_err("vocpcm: failed to connect snd svc\n");
			return rc;
		}

		voc_rpc->task = create_snd_rpc_kthread();
		if (IS_ERR(voc_rpc->task)) {
			rc = PTR_ERR(voc_rpc->task);
			voc_rpc->task = NULL;
			msm_rpc_close(voc_rpc->ept);
			voc_rpc->ept = NULL;
			return rc;
		}
		voc_rpc->inited = 1;
	}
	mutex_unlock(&voc_rpc->lock);

	mutex_lock(&ctxt->lock);
	if (ctxt->opened) {
		pr_err("vocpcm already opened.\n");
		rc = -EBUSY;
		goto err;
	}

	file->private_data = ctxt;
	ctxt->head = 0;
	ctxt->tail = 0;
	ctxt->client = 0;

	memset(ctxt->buf[0].data, 0, sizeof(ctxt->buf[0].data));
	memset(ctxt->buf[1].data, 0, sizeof(ctxt->buf[1].data));
	ctxt->buf[0].index = 0;
	ctxt->buf[1].index = 0;
	ctxt->opened = 1;
	ctxt->count = 0;
	ctxt->s_ptr = 0;
	ctxt->final_input = 0;

err:
	mutex_unlock(&ctxt->lock);
	return rc;
}

static struct file_operations vocpcm_fops = {
	.owner		= THIS_MODULE,
	.open		= vocpcm_open,
	.release	= vocpcm_release,
	.read		= vocpcm_read,
	.write		= vocpcm_write,
	.unlocked_ioctl	= vocpcm_ioctl,
};

static struct voc_ctxt vocpcm3 = {
	.misc = {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= "voc_tx_playback",
		.fops	= &vocpcm_fops,
	}
};

static struct voc_ctxt vocpcm2 = {
	.misc = {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= "voc_tx_record",
		.fops	= &vocpcm_fops,
	}
};

static struct voc_ctxt vocpcm1 = {
	.misc = {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= "voc_rx_playback",
		.fops	= &vocpcm_fops,
	}
};

static struct voc_ctxt vocpcm0 = {
	.misc = {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= "voc_rx_record",
		.fops	= &vocpcm_fops,
	}
};

void voc_ctxt_init(struct voc_ctxt *ctxt, unsigned n)
{
	mutex_init(&ctxt->lock);
	spin_lock_init(&ctxt->dsp_lock);
	init_waitqueue_head(&ctxt->wait);
	init_waitqueue_head(&ctxt->last_write_wait);

	ctxt->intr = n;
}

static struct voc_ctxt *voc_intr_to_ctxt(unsigned n)
{
	if (n == vocpcm0.intr)
		return &vocpcm0;
	if (n == vocpcm1.intr)
		return &vocpcm1;
	if (n == vocpcm2.intr)
		return &vocpcm2;
	if (n == vocpcm3.intr)
		return &vocpcm3;
	return 0;
}

static struct voc_ctxt *voc_minor_to_ctxt(unsigned n)
{
	if (n == vocpcm0.misc.minor)
		return &vocpcm0;
	if (n == vocpcm1.misc.minor)
		return &vocpcm1;
	if (n == vocpcm2.misc.minor)
		return &vocpcm2;
	if (n == vocpcm3.misc.minor)
		return &vocpcm3;
	return 0;
}

static int __init vocpcm_init(void)
{
	struct voc_rpc *voc_rpc = &the_voc_proc;
	int rc;

	voc_ctxt_init(&vocpcm0, 0);
	voc_ctxt_init(&vocpcm1, 1);
	voc_ctxt_init(&vocpcm2, 2);
	voc_ctxt_init(&vocpcm3, 3);

	mutex_init(&voc_rpc->lock);
	voc_rpc->inited = 0;
	rc = misc_register(&vocpcm0.misc);
	if (rc == 0)
		rc = misc_register(&vocpcm1.misc);
	if (rc == 0)
		rc = misc_register(&vocpcm2.misc);
	if (rc == 0)
		rc = misc_register(&vocpcm3.misc);
	return rc;
}

device_initcall(vocpcm_init);
