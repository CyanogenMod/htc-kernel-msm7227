/* arch/arm/mach-msm/qdsp5_comp/snd.c
 *
 * interface to "snd" service on the baseband cpu
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
#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_rpc_version.h>

#include <mach/htc_pwrsink.h>

#include "snd.h"

static struct snd_ctxt the_snd;
static struct msm_rpc_endpoint *snd_ept;
static struct mutex rpc_lock;
static struct task_struct *task;
static unsigned kcount;

static void rpc_ack(struct msm_rpc_endpoint *ept, uint32_t xid)
{
	uint32_t rep[6];

	rep[0] = cpu_to_be32(xid);
	rep[1] = cpu_to_be32(1);
	rep[2] = cpu_to_be32(RPCMSG_REPLYSTAT_ACCEPTED);
	rep[3] = cpu_to_be32(RPC_ACCEPTSTAT_SUCCESS);
	rep[4] = 0;
	rep[5] = 0;

	msm_rpc_write(ept, rep, sizeof(rep));
}

static void process_rpc_request(uint32_t proc, uint32_t xid,
				void *data, int len)
{
	uint32_t *x = data;
	uint32_t cb_id, data_len, pointer;

	if (0) {
		int n = len / 4;
		pr_info("rpc_call proc %d:", proc);
		while (n--)
			pr_info(" %08x", be32_to_cpu(*x++));
		pr_info("\n");
	}

	if (proc == VOC_PCM_CLIENT_INPUT_PTR) {
		cb_id = be32_to_cpu(*x++);
		pointer = be32_to_cpu(*x++);
		data_len = be32_to_cpu(*x++);
		put_vocpcm_data(x, cb_id, data_len, snd_ept, xid);
		rpc_in_ack(snd_ept, xid);
	} else if (proc == VOC_PCM_CLIENT_OUTPUT_PTR) {
		cb_id = be32_to_cpu(*x++);
		data_len = be32_to_cpu(*x++);
		get_vocpcm_data(x, cb_id, data_len);
		rpc_ack(snd_ept, xid);
	} else if (proc == SND_CB_FUNC_PTR_TYPE_PROC) {
		rpc_ack(snd_ept, xid);
	} else {
		pr_err("snd_client: unknown rpc proc %d\n", proc);
		rpc_ack(snd_ept, xid);
	}
}

static int snd_rpc_thread(void *d)
{
	struct rpc_request_hdr *hdr = NULL;
	uint32_t type;
	int len;

	pr_info("snd_rpc_thread() start\n");
	while (!kthread_should_stop()) {
		if (hdr != NULL) {
			kfree(hdr);
			hdr = NULL;
		}
		len = msm_rpc_read(snd_ept, (void **) &hdr, -1, -1);
		if (len < 0) {
			pr_err("snd: rpc read failed (%d)\n", len);
			break;
		}
		if (len < RPC_COMMON_HDR_SZ)
			continue;

		type = be32_to_cpu(hdr->type);
		if (type == RPC_TYPE_REPLY) {
			struct rpc_reply_hdr *rep = (void *) hdr;
			uint32_t status;
			if (len < RPC_REPLY_HDR_SZ)
				continue;
			status = be32_to_cpu(rep->reply_stat);
			if (status == RPCMSG_REPLYSTAT_ACCEPTED) {
				status = be32_to_cpu(rep->
						data.acc_hdr.accept_stat);
				pr_info("snd: rpc_reply status %d\n", status);
			} else {
				pr_info("snd: rpc_reply denied!\n");
			}
			/* process reply */
			continue;
		}

		if (len < RPC_REQUEST_HDR_SZ)
			continue;

		process_rpc_request(be32_to_cpu(hdr->procedure),
				    be32_to_cpu(hdr->xid),
				    (void *) (hdr + 1),
				    len - sizeof(*hdr));

	}

	pr_info("snd_rpc_thread() exit\n");
	if (hdr != NULL) {
		kfree(hdr);
		hdr = NULL;
	}
	return 0;
}

/* ------------------- device --------------------- */

static long snd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct snd_set_device_msg dmsg;
	struct snd_set_volume_msg vmsg;
	struct snd_device_config dev;
	struct snd_volume_config vol;
	struct snd_ctxt *snd = file->private_data;
	int rc = 0;

	mutex_lock(&snd->lock);
	switch (cmd) {
	case SND_SET_DEVICE:
		if (copy_from_user(&dev, (void *) arg, sizeof(dev))) {
			rc = -EFAULT;
			break;
		}

		dmsg.args.device = cpu_to_be32(dev.device);
		dmsg.args.ear_mute = cpu_to_be32(dev.ear_mute);
		dmsg.args.mic_mute = cpu_to_be32(dev.mic_mute);
		dmsg.args.cb_func = cpu_to_be32(0x11111111);
		dmsg.args.client_data = cpu_to_be32(0x11223344);

		pr_info("snd_set_device %d %d %d\n", dev.device,
						 dev.ear_mute, dev.mic_mute);

		msm_rpc_setup_req(&dmsg.hdr, RPC_SND_PROG, RPC_SND_VERS,
			  SND_SET_DEVICE_PROC);

		rc = msm_rpc_write(snd->ept, &dmsg, sizeof(dmsg));
		htc_pwrsink_audio_path_set(dmsg.args.device);
		break;

	case SND_SET_VOLUME:
		if (copy_from_user(&vol, (void *) arg, sizeof(vol))) {
			rc = -EFAULT;
			break;
		}

		vmsg.args.device = cpu_to_be32(vol.device);
		vmsg.args.method = cpu_to_be32(vol.method);
		vmsg.args.volume = cpu_to_be32(vol.volume);
		vmsg.args.cb_func = cpu_to_be32(0x11111111);
		vmsg.args.client_data = cpu_to_be32(0x11223344);

		pr_info("snd_set_volume %d %d %d\n", vol.device,
						vol.method, vol.volume);

		msm_rpc_setup_req(&vmsg.hdr, RPC_SND_PROG, RPC_SND_VERS,
			  SND_SET_VOLUME_PROC);

		rc = msm_rpc_write(snd->ept, &vmsg, sizeof(vmsg));
		break;

	default:
		pr_err("snd_ioctl unknown command.\n");
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&snd->lock);

	return rc;
}

static ssize_t snd_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos)
{
	return -EINVAL;
}

static ssize_t snd_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	return -EINVAL;
}

static int snd_release(struct inode *inode, struct file *file)
{
	struct snd_ctxt *snd = file->private_data;

	mutex_lock(&snd->lock);
	snd->opened = 0;
	mutex_unlock(&snd->lock);
	return 0;
}

struct msm_rpc_endpoint *create_rpc_connect(uint32_t prog,
					uint32_t vers, unsigned flags)
{
	mutex_lock(&rpc_lock);
	if (snd_ept == NULL) {
		snd_ept = msm_rpc_connect(prog, vers, flags);
		if (IS_ERR(snd_ept))
			pr_err("snd: failed to connect snd svc\n");
	}
	mutex_unlock(&rpc_lock);
	return snd_ept;
}
EXPORT_SYMBOL(create_rpc_connect);

struct task_struct *create_snd_rpc_kthread(void)
{
	mutex_lock(&rpc_lock);
	if (kcount == 0) {
		task = kthread_run(snd_rpc_thread, NULL, "snd_rpc");
		if (IS_ERR(task))
			pr_err("snd: failed to create kthread\n");
		else
			kcount = 1;
	}
	mutex_unlock(&rpc_lock);
	return task;
}
EXPORT_SYMBOL(create_snd_rpc_kthread);

static int snd_open(struct inode *inode, struct file *file)
{
	struct snd_ctxt *snd = &the_snd;
	int rc = 0;

	mutex_lock(&snd->lock);
	if (snd->inited == 0) {
		snd->ept = create_rpc_connect(RPC_SND_PROG, RPC_SND_VERS,
					MSM_RPC_UNINTERRUPTIBLE |
					MSM_RPC_ENABLE_RECEIVE);
		if (IS_ERR(snd->ept)) {
			rc = PTR_ERR(snd->ept);
			snd->ept = NULL;
			pr_err("snd: failed to connect snd svc\n");
			goto err;
		}

		snd->task = create_snd_rpc_kthread();
		if (IS_ERR(snd->task)) {
			rc = PTR_ERR(snd->task);
			snd->task = NULL;
			msm_rpc_close(snd->ept);
			snd->ept = NULL;
			goto err;
		}
		snd->inited = 1;
	}

	if (snd->opened) {
		pr_err("snd already opened.\n");
		rc = -EBUSY;
		goto err;
	}

	file->private_data = snd;
	snd->opened = 1;
err:
	mutex_unlock(&snd->lock);
	return rc;
}

static struct file_operations snd_fops = {
	.owner		= THIS_MODULE,
	.open		= snd_open,
	.release	= snd_release,
	.read		= snd_read,
	.write		= snd_write,
	.unlocked_ioctl	= snd_ioctl,
};

struct miscdevice snd_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_snd",
	.fops	= &snd_fops,
};

static int __init snd_init(void)
{
	struct snd_ctxt *snd = &the_snd;

	mutex_init(&snd->lock);
	mutex_init(&rpc_lock);
	snd->inited = 0;
	return misc_register(&snd_misc);
}

device_initcall(snd_init);
