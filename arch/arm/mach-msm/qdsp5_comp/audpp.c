/* arch/arm/mach-msm/qdsp5_comp/audpp.c
 *
 * common code to deal with the AUDPP dsp task (audio postproc)
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include <linux/msm_audio.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/msm_adsp.h>

#include "audmgr.h"

#include <mach/qdsp5/qdsp5audppcmdi.h>
#include <mach/qdsp5/qdsp5audppmsg.h>

/* for queue ids - should be relative to module number*/
#include "adsp.h"

#include "evlog.h"


enum {
	EV_NULL,
	EV_ENABLE,
	EV_DISABLE,
	EV_EVENT,
	EV_DATA,
};

static const char *dsp_log_strings[] = {
	"NULL",
	"ENABLE",
	"DISABLE",
	"EVENT",
	"DATA",
};

DECLARE_LOG(dsp_log, 64, dsp_log_strings);

static int __init _dsp_log_init(void)
{
	return ev_log_init(&dsp_log);
}
module_init(_dsp_log_init);
#define LOG(id, arg) ev_log_write(&dsp_log, id, arg)

static DEFINE_MUTEX(audpp_lock);

#define CH_COUNT 5
#define AUDPP_CLNT_MAX_COUNT 7

#define AUDPP_CMD_CFG_OBJ_UPDATE 0x8000
#define AUDPP_CMD_EQ_FLAG_DIS	0x0000
#define AUDPP_CMD_EQ_FLAG_ENA	-1
#define AUDPP_CMD_IIR_FLAG_DIS	  0x0000
#define AUDPP_CMD_IIR_FLAG_ENA	  -1

#define AUDPP_CMD_IIR_TUNING_FILTER  1
#define AUDPP_CMD_EQUALIZER	2
#define AUDPP_CMD_ADRC	3

#define ADRC_ENABLE  0x0001
#define EQ_ENABLE    0x0002
#define IIR_ENABLE   0x0004

struct adrc_filter {
	uint16_t compression_th;
	uint16_t compression_slope;
	uint16_t rms_time;
	uint16_t attack_const_lsw;
	uint16_t attack_const_msw;
	uint16_t release_const_lsw;
	uint16_t release_const_msw;
	uint16_t adrc_system_delay;
};

struct eqalizer {
	uint16_t num_bands;
	uint16_t eq_params[132];
};

struct rx_iir_filter {
	uint16_t num_bands;
	uint16_t iir_params[48];
};

struct audpp_cmd_cfg_object_params_eq {
	audpp_cmd_cfg_object_params_common common;
	uint16_t eq_flag;
	uint16_t num_bands;
	uint16_t eq_params[132];
};

struct audpp_cmd_cfg_object_params_rx_iir {
	audpp_cmd_cfg_object_params_common common;
	uint16_t active_flag;
	uint16_t num_bands;
	uint16_t iir_params[48];
};

struct audpp_state {
	struct msm_adsp_module *mod;
	audpp_event_func func[AUDPP_CLNT_MAX_COUNT];
	audpp_modem_event_func mfunc[AUDPP_CLNT_MAX_COUNT];
	void *private[AUDPP_CLNT_MAX_COUNT];
	struct mutex *lock;
	unsigned open_count;
	unsigned enabled;

	/* which channels are actually enabled */
	unsigned avsync_mask;

	/* flags, 48 bits sample/bytes counter per channel */
	uint16_t avsync[CH_COUNT * AUDPP_CLNT_MAX_COUNT + 1];

	int adrc_enable;
	struct adrc_filter adrc;

	int eq_enable;
	struct eqalizer eq;

	int rx_iir_enable;
	struct rx_iir_filter iir;
};

struct audpp_state the_audpp_state = {
	.lock = &audpp_lock,
};

int audpp_send_queue1(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audpp_state.mod,
			      QDSP_uPAudPPCmd1Queue, cmd, len);
}

int audpp_send_queue2(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audpp_state.mod,
			      QDSP_uPAudPPCmd2Queue, cmd, len);
}

int audpp_send_queue3(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audpp_state.mod,
			      QDSP_uPAudPPCmd3Queue, cmd, len);
}

static int audpp_dsp_config(int enable)
{
	audpp_cmd_cfg cmd;

	cmd.cmd_id = AUDPP_CMD_CFG;
	cmd.cfg = enable ? AUDPP_CMD_CFG_ENABLE : AUDPP_CMD_CFG_SLEEP;

	return audpp_send_queue1(&cmd, sizeof(cmd));
}

static int audpp_dsp_set_adrc(void)
{
	struct audpp_state *audpp = &the_audpp_state;
	audpp_cmd_cfg_object_params_adrc cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.comman_cfg = AUDPP_CMD_CFG_OBJ_UPDATE;
	cmd.common.command_type = AUDPP_CMD_ADRC;

	if (audpp->adrc_enable) {
		cmd.adrc_flag = AUDPP_CMD_ADRC_FLAG_ENA;
		cmd.compression_th = audpp->adrc.compression_th;
		cmd.compression_slope = audpp->adrc.compression_slope;
		cmd.rms_time = audpp->adrc.rms_time;
		cmd.attack_const_lsw = audpp->adrc.attack_const_lsw;
		cmd.attack_const_msw = audpp->adrc.attack_const_msw;
		cmd.release_const_lsw = audpp->adrc.release_const_lsw;
		cmd.release_const_msw = audpp->adrc.release_const_msw;
		cmd.adrc_system_delay = audpp->adrc.adrc_system_delay;
	} else {
		cmd.adrc_flag = AUDPP_CMD_ADRC_FLAG_DIS;
	}
	return audpp_send_queue3(&cmd, sizeof(cmd));
}

static int audpp_dsp_set_eq(void)
{
	struct audpp_state *audpp = &the_audpp_state;
	struct audpp_cmd_cfg_object_params_eq cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.comman_cfg = AUDPP_CMD_CFG_OBJ_UPDATE;
	cmd.common.command_type = AUDPP_CMD_EQUALIZER;

	if (audpp->eq_enable) {
		cmd.eq_flag = AUDPP_CMD_EQ_FLAG_ENA;
		cmd.num_bands = audpp->eq.num_bands;
		memcpy(&cmd.eq_params, audpp->eq.eq_params,
				sizeof(audpp->eq.eq_params));
	} else {
		cmd.eq_flag = AUDPP_CMD_EQ_FLAG_DIS;
	}
	return audpp_send_queue3(&cmd, sizeof(cmd));
}

static int audpp_dsp_set_rx_iir(void)
{
	struct audpp_state *audpp = &the_audpp_state;
	struct audpp_cmd_cfg_object_params_rx_iir cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.comman_cfg = AUDPP_CMD_CFG_OBJ_UPDATE;
	cmd.common.command_type = AUDPP_CMD_IIR_TUNING_FILTER;

	if (audpp->rx_iir_enable) {
		cmd.active_flag = AUDPP_CMD_IIR_FLAG_ENA;
		cmd.num_bands = audpp->iir.num_bands;
		memcpy(&cmd.iir_params, audpp->iir.iir_params,
				sizeof(audpp->iir.iir_params));
	} else {
		cmd.active_flag = AUDPP_CMD_IIR_FLAG_DIS;
	}

	return audpp_send_queue3(&cmd, sizeof(cmd));
}

static void audpp_broadcast(struct audpp_state *audpp,
		unsigned id, uint16_t *msg)
{
	unsigned n;
	for (n = 0; n < 6; n++) {
		if (audpp->func[n])
			audpp->func[n](audpp->private[n], id, msg);
	}
}

static void audpp_status_broadcast(struct audpp_state *audpp,
		unsigned image_swap)
{
	unsigned n;
	for (n = 0; n < 6; n++) {
		if (audpp->mfunc[n])
			audpp->mfunc[n](audpp->private[n], image_swap);
	}
}

static void audpp_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len))
{
	struct audpp_state *audpp = data;
	uint16_t msg[8];

	if (id == AUDPP_MSG_AVSYNC_MSG) {
		getevent(audpp->avsync, sizeof(audpp->avsync));

		/* mask off any channels we're not watching to avoid
		 * cases where we might get one last update after
		 * disabling avsync and end up in an odd state when
		 * we next read...
		 */
		audpp->avsync[0] &= audpp->avsync_mask;
		audpp_broadcast(audpp, id, msg);
		return;
	}

	getevent(msg, sizeof(msg));

	LOG(EV_EVENT, (id << 16) | msg[0]);
	LOG(EV_DATA, (msg[1] << 16) | msg[2]);

	switch (id) {
	case AUDPP_MSG_STATUS_MSG: {
		unsigned cid = msg[0];
		pr_info("audpp: status %d %d %d\n", cid, msg[1], msg[2]);
		if ((cid < 5) && audpp->func[cid])
			audpp->func[cid](audpp->private[cid], id, msg);
		break;
	}
	case AUDPP_MSG_HOST_PCM_INTF_MSG:
		if (audpp->func[5])
			audpp->func[5](audpp->private[5], id, msg);
		break;
	case AUDPP_MSG_PCMDMAMISSED: {
		pr_info("audpp: DMA missed\n");
		audpp_broadcast(audpp, id, msg);
		break;
	}
	case AUDPP_MSG_CFG_MSG:
		if (msg[0] == AUDPP_MSG_ENA_ENA) {
			pr_info("audpp: ENABLE\n");
			audpp->enabled = 1;
			audpp_dsp_set_adrc();
			audpp_dsp_set_eq();
			audpp_dsp_set_rx_iir();
			audpp_broadcast(audpp, id, msg);
		} else if (msg[0] == AUDPP_MSG_ENA_DIS) {
			pr_info("audpp: DISABLE\n");
			audpp->enabled = 0;
			audpp_broadcast(audpp, id, msg);
		} else
			pr_err("audpp: invalid config msg %d\n", msg[0]);
		break;
	case ADSP_MESSAGE_ID:
		pr_info("audpp: module enabled\n");
		break;
	default:
		pr_info("audpp: unhandled msg id %x\n", id);
	}
}

static void audpp_modem_event(void *data, uint32_t image)
{
	struct audpp_state *audpp = data;
	struct msm_adsp_module *module = audpp->mod;
	if (module->state == ADSP_STATE_DISABLED && audpp->enabled) {
		/* ignore audpp disabled event during image swap */
		module->state = ADSP_STATE_ENABLED;
		audpp_status_broadcast(audpp, 1);
	} else
		audpp_status_broadcast(audpp, 0);
}

static struct msm_adsp_ops adsp_ops = {
	.event = audpp_dsp_event,
	.modem_event = audpp_modem_event,
};

static void audpp_fake_event(struct audpp_state *audpp, int id,
			     unsigned event, unsigned arg)
{
	uint16_t msg[1];
	msg[0] = arg;
	audpp->func[id](audpp->private[id], event, msg);
}

int audpp_enable(int id, audpp_event_func func,
		 audpp_modem_event_func mfunc, void *private)
{
	struct audpp_state *audpp = &the_audpp_state;
	int res = 0;

	if (id < -1 || id > 4)
		return -EINVAL;

	if (id == -1)
		id = 5;

	mutex_lock(audpp->lock);
	if (audpp->func[id] || audpp->mfunc[id]) {
		res = -EBUSY;
		goto out;
	}

	audpp->func[id] = func;
	audpp->mfunc[id] = mfunc;
	audpp->private[id] = private;

	LOG(EV_ENABLE, 1);
	if (audpp->open_count++ == 0) {
		pr_info("audpp: enable\n");
		res = msm_adsp_get("AUDPPTASK", &audpp->mod, &adsp_ops, audpp);
		if (res < 0) {
			pr_err("audpp: cannot open AUDPPTASK\n");
			audpp->open_count = 0;
			audpp->func[id] = NULL;
			audpp->mfunc[id] = NULL;
			audpp->private[id] = NULL;
			goto out;
		}
		LOG(EV_ENABLE, 2);
		msm_adsp_enable(audpp->mod);
		audpp_dsp_config(1);
	} else {
		unsigned long flags;
		local_irq_save(flags);
		if (audpp->enabled)
			audpp_fake_event(audpp, id,
					 AUDPP_MSG_CFG_MSG,
					 AUDPP_MSG_ENA_ENA);
		local_irq_restore(flags);
	}

	res = 0;
out:
	mutex_unlock(audpp->lock);
	return res;
}

void audpp_disable(int id, void *private)
{
	struct audpp_state *audpp = &the_audpp_state;
	unsigned long flags;

	if (id < -1 || id > 4)
		return;

	if (id == -1)
		id = 5;

	mutex_lock(audpp->lock);
	LOG(EV_DISABLE, 1);
	if (!audpp->func[id])
		goto out;
	if (!audpp->mfunc[id])
		goto out;
	if (audpp->private[id] != private)
		goto out;

	local_irq_save(flags);
	audpp_fake_event(audpp, id, AUDPP_MSG_CFG_MSG, AUDPP_MSG_ENA_DIS);
	audpp->func[id] = NULL;
	audpp->mfunc[id] = NULL;
	audpp->private[id] = NULL;
	local_irq_restore(flags);

	if (--audpp->open_count == 0) {
		pr_info("audpp: disable\n");
		LOG(EV_DISABLE, 2);
		audpp_dsp_config(0);
		msm_adsp_disable(audpp->mod);
		msm_adsp_put(audpp->mod);
		audpp->mod = NULL;
	}
out:
	mutex_unlock(audpp->lock);
}


#define BAD_ID(id) ((id < 0) || (id >= CH_COUNT))

void audpp_avsync(int id, unsigned rate)
{
	unsigned long flags;
	audpp_cmd_avsync cmd;

	if (BAD_ID(id))
		return;

	local_irq_save(flags);
	if (rate)
		the_audpp_state.avsync_mask |= (1 << id);
	else
		the_audpp_state.avsync_mask &= (~(1 << id));
	the_audpp_state.avsync[0] &= the_audpp_state.avsync_mask;
	local_irq_restore(flags);

	cmd.cmd_id = AUDPP_CMD_AVSYNC;
	cmd.object_number = id;
	cmd.interrupt_interval_lsw = rate;
	cmd.interrupt_interval_msw = rate >> 16;
	audpp_send_queue1(&cmd, sizeof(cmd));
}

unsigned audpp_avsync_sample_count(int id)
{
	uint16_t *avsync = the_audpp_state.avsync;
	unsigned val;
	unsigned long flags;
	unsigned mask;

	if (BAD_ID(id))
		return 0;

	mask = 1 << id;
	id = id * 6 + 2;
	local_irq_save(flags);
	if (avsync[0] & mask)
		val = (avsync[id] << 16) | avsync[id];
	else
		val = 0;
	local_irq_restore(flags);

	return val;
}

unsigned audpp_avsync_byte_count(int id)
{
	uint16_t *avsync = the_audpp_state.avsync;
	unsigned val;
	unsigned long flags;
	unsigned mask;

	if (BAD_ID(id))
		return 0;

	mask = 1 << id;
	id = id * AUDPP_CLNT_MAX_COUNT + 5;
	local_irq_save(flags);
	if (avsync[0] & mask)
		val = (avsync[id] << 16) | avsync[id + 1];
	else
		val = 0;
	local_irq_restore(flags);

	return val;
}

#define AUDPP_CMD_CFG_OBJ_UPDATE 0x8000
#define AUDPP_CMD_VOLUME_PAN 0

int audpp_set_volume_and_pan(unsigned id, unsigned volume, int pan)
{
	/* cmd, obj_cfg[7], cmd_type, volume, pan */
	uint16_t cmd[11];

	if (id > 6)
		return -EINVAL;

	memset(cmd, 0, sizeof(cmd));
	cmd[0] = AUDPP_CMD_CFG_OBJECT_PARAMS;
	cmd[1 + id] = AUDPP_CMD_CFG_OBJ_UPDATE;
	cmd[8] = AUDPP_CMD_VOLUME_PAN;
	cmd[9] = volume;
	cmd[10] = pan;

	return audpp_send_queue3(cmd, sizeof(cmd));
}

int audpp_dec_ctrl(audpp_cmd_dec_ctrl *cmd_ptr)
{
	uint16_t cmd[6];

	memset(cmd, 0, sizeof(cmd));
	cmd[0] = AUDPP_CMD_DEC_CTRL;
	cmd[1] = cmd_ptr->dec0_ctrl;
	cmd[2] = cmd_ptr->dec1_ctrl;
	cmd[3] = cmd_ptr->dec2_ctrl;
	cmd[4] = cmd_ptr->dec3_ctrl;
	cmd[5] = cmd_ptr->dec4_ctrl;

	return audpp_send_queue1(cmd, sizeof(cmd));
}

static int audpp_enable_adrc(struct audpp_state *audpp, int enable)
{
	if (audpp->adrc_enable != enable) {
		audpp->adrc_enable = enable;
		if (audpp->enabled)
			audpp_dsp_set_adrc();
	}
	return 0;
}

static int audpp_enable_eq(struct audpp_state *audpp, int enable)
{
	if (audpp->eq_enable != enable) {
		audpp->eq_enable = enable;
		if (audpp->enabled)
			audpp_dsp_set_eq();
	}
	return 0;
}

static int audpp_enable_rx_iir(struct audpp_state *audpp, int enable)
{
	if (audpp->rx_iir_enable != enable) {
		audpp->rx_iir_enable = enable;
		if (audpp->enabled)
			audpp_dsp_set_rx_iir();
	}
	return 0;
}

static long audpp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct audpp_state *audpp = file->private_data;
	int rc = 0, enable;
	uint16_t enable_mask;
	/* int i; */

	mutex_lock(audpp->lock);
	switch (cmd) {
	case AUDIO_ENABLE_AUDPP:
		if (copy_from_user(&enable_mask, (void *) arg,
				sizeof(enable_mask)))
			goto out_fault;

		enable = (enable_mask & ADRC_ENABLE)? 1 : 0;
		audpp_enable_adrc(audpp, enable);
		enable = (enable_mask & EQ_ENABLE)? 1 : 0;
		audpp_enable_eq(audpp, enable);
		enable = (enable_mask & IIR_ENABLE)? 1 : 0;
		audpp_enable_rx_iir(audpp, enable);
		break;

	case AUDIO_SET_ADRC:
		if (copy_from_user(&audpp->adrc, (void *) arg,
				sizeof(audpp->adrc)))
			goto out_fault;
#if 0
		pr_info("set adrc_filter\n");
		pr_info("0x%04x\n", audpp->adrc.compression_th);
		pr_info("0x%04x\n", audpp->adrc.compression_slope);
		pr_info("0x%04x\n", audpp->adrc.rms_time);
		pr_info("0x%04x\n", audpp->adrc.attack_const_lsw);
		pr_info("0x%04x\n", audpp->adrc.attack_const_msw);
		pr_info("0x%04x\n", audpp->adrc.release_const_lsw);
		pr_info("0x%04x\n", audpp->adrc.release_const_msw);
		pr_info("0x%04x\n", audpp->adrc.adrc_system_delay);
#endif
		break;

	case AUDIO_SET_EQ:
		if (copy_from_user(&audpp->eq, (void *) arg, sizeof(audpp->eq)))
			goto out_fault;
#if 0
		pr_info("set eq\n");
		pr_info("eq.num_bands = 0x%04x\n", audpp->eq.num_bands);
		for (i = 0; i < 132; i++) \
			pr_info("eq_params[%d] =0x%04x\n", i,
					audpp->eq.eq_params[i]);
#endif
		break;

	case AUDIO_SET_RX_IIR:
		if (copy_from_user(&audpp->iir, (void *) arg,
				sizeof(audpp->iir)))
			goto out_fault;
#if 0
			pr_info("set rx iir\n");
			pr_info("iir.num_bands = 0x%04x\n",
				audpp->iir.num_bands);
			for (i = 0; i < 48; i++) \
				pr_info("iir_params[%d] = 0x%04x\n",
					i, audpp->iir.iir_params[i]);
#endif
		break;

	default:
		rc = -EINVAL;
	}

	goto out;

 out_fault:
	rc = -EFAULT;
 out:
	mutex_unlock(audpp->lock);
	return rc;
}

static int audpp_open(struct inode *inode, struct file *file)
{
	struct audpp_state *audpp = &the_audpp_state;

	file->private_data = audpp;
	return 0;
}

static struct file_operations audpp_fops = {
	.owner		= THIS_MODULE,
	.open		= audpp_open,
	.unlocked_ioctl	= audpp_ioctl,
};

struct miscdevice audpp_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_ctl",
	.fops	= &audpp_fops,
};

static int __init audpp_init(void)
{
	return misc_register(&audpp_misc);
}

device_initcall(audpp_init);
