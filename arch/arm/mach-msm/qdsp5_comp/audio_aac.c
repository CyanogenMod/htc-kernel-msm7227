/* arch/arm/mach-msm/qdsp5_comp/audio_aac.c
 *
 * aac audio output device
 *
 * Copyright (C) 2008 Google, Inc.
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
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/msm_adsp.h>
#include <mach/msm_iomap.h>
#include <mach/debug_mm.h>

#include <linux/msm_audio.h>

#include "audmgr.h"

#include <mach/qdsp5/qdsp5audppcmdi.h>
#include <mach/qdsp5/qdsp5audppmsg.h>
#include <mach/qdsp5/qdsp5audplaymsg.h>

#include <linux/wakelock.h>

#include <mach/htc_pwrsink.h>

/* for queue ids - should be relative to module number*/
#include "adsp.h"
#include "audplay.h"

#define AUDDEC_DEC_AACLC 5

/* Decoder status received from AUDPPTASK */
enum audpp_dec_status_type{
	AUDPP_DEC_STATUS_SLEEP,
	AUDPP_DEC_STATUS_INIT,
	AUDPP_DEC_STATUS_CFG,
	AUDPP_DEC_STATUS_PLAY
};

static void audio_prevent_sleep(struct audio *audio)
{
	wake_lock(&audio->wakelock);
}

static void audio_allow_sleep(struct audio *audio)
{
	wake_unlock(&audio->wakelock);
}

static int auddec_dsp_config(struct audio *audio, int enable);
static int auddec_flush_decoder(struct audio *audio);
static int auddec_pause_decoder(struct audio *audio);
static int auddec_resume_decoder(struct audio *audio);
static void audpp_cmd_cfg_adec_params(struct audio *audio);
static void audplay_aac_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len));
static void audplay_aac_modem_event(void *data, uint32_t image);
static void audio_dsp_event(void *private, unsigned id, uint16_t *msg);
static void audio_modem_event(void *private, unsigned image_swap);
static void audio_aac_audmgr_cb(void);

static uint32_t discard_bytes;

struct msm_adsp_ops audplay_aac_adsp_ops = {
	.event = audplay_aac_dsp_event,
	.modem_event = audplay_aac_modem_event,
};

/* must be called with audio->lock held */
static int audio_enable(struct audio *audio)
{
	struct audmgr_config cfg;
	int rc;

	MM_AUD_INFO("audio_aac_enable()\n");

	if (audio->enabled)
		return 0;

	audio->out_tail = 0;
	audio->out_needed = 0;

	cfg.tx_rate = RPC_AUD_DEF_SAMPLE_RATE_NONE;
	cfg.rx_rate = RPC_AUD_DEF_SAMPLE_RATE_48000;
	cfg.def_method = RPC_AUD_DEF_METHOD_PLAYBACK;
	cfg.codec = RPC_AUD_DEF_CODEC_AAC;
	cfg.snd_method = RPC_SND_METHOD_MIDI;

	audio_prevent_sleep(audio);
	audio->audmgr.cb = audio_aac_audmgr_cb;
	rc = audmgr_enable(&audio->audmgr, &cfg);
	if (rc < 0) {
		MM_AUD_ERR("audio_aac: audmgr_enable() failed\n");
		audio_allow_sleep(audio);
		return rc;
	}

	rc = msm_adsp_get("AUDPLAY0TASK", &audio->audplay,
			&audplay_aac_adsp_ops, audio);
	if (rc) {
		MM_AUD_ERR("audio_aac:"
				" failed to get audplay0 dsp module\n");
		goto err_get_adsp;
	}

	if (msm_adsp_enable(audio->audplay)) {
		MM_AUD_ERR("audio_aac:"
				" msm_adsp_enable(audplay) failed\n");
		goto err_enable_adsp;
	}

	if (audpp_enable(audio->dec_id, audio_dsp_event,
			audio_modem_event, audio)) {
		MM_AUD_ERR("audio_aac: audpp_enable() failed\n");
		goto err_enable_audpp;
	}

	atomic_set(&audio->image_swap, 0);
	audio->enabled = 1;
	htc_pwrsink_audio_set(PWRSINK_AUDIO_AAC, 100);
	return 0;

err_enable_audpp:
	msm_adsp_disable(audio->audplay);
err_enable_adsp:
	msm_adsp_put(audio->audplay);
err_get_adsp:
	audmgr_disable(&audio->audmgr);
	audio_allow_sleep(audio);
	return -ENODEV;
}

/* must be called with audio->lock held */
static int audio_disable(struct audio *audio)
{
	if (audio->enabled) {
		MM_AUD_INFO("audio_aac_disable()\n");
		audio->enabled = 0;
		auddec_dsp_config(audio, 0);
		wake_up(&audio->wait);
		audpp_disable(audio->dec_id, audio);
		msm_adsp_disable(audio->audplay);
		msm_adsp_put(audio->audplay);
		audmgr_disable(&audio->audmgr);
		atomic_set(&audio->image_swap, 0);
		audio->sent_bytes = 0;
		audio->consumed_bytes = 0;
		audio->total_consumed_bytes = 0;
		audio->out_needed = 0;
		audio->paused = 0;
		audio_allow_sleep(audio);
		htc_pwrsink_audio_set(PWRSINK_AUDIO_AAC, 0);
	}
	return 0;
}

/* ------------------- dsp --------------------- */

static void audplay_aac_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len))
{
	struct audio *audio = data;
	uint32_t msg[28];
	getevent(msg, sizeof(msg));

	switch (id) {
	case AUDPLAY_MSG_STREAM_INFO:
		MM_AUD_INFO("aac:channel_mode = %d, sample_rate = %d\n", \
			msg[1], msg[2]);
		break;
	case AUDPLAY_MSG_DEC_NEEDS_DATA:
		if (!atomic_read(&audio->curr_img)) {
			audplay_send_data(audio, 1);
		} else {
			audio->dsp_free_len = msg[3] - 2;
			audio->dsp_write_ptr = (uint16_t *)
				adsp_rtos_phy_to_vir(msg[4], MSM_AD5_BASE);
			audio->dsp_start_ptr = (uint16_t *)
				adsp_rtos_phy_to_vir(msg[5], MSM_AD5_BASE);
			audio->dsp_buf_size = msg[6];
			audplay_send_lp_data(audio, 1);
		}
		break;
	case ADSP_MESSAGE_ID:
		MM_AUD_INFO("audplay: module enabled\n");
		break;
	default:
		MM_AUD_ERR("aac: unexpected message from decoder\n");
		break;
	}
}

static void audplay_aac_modem_event(void *data, uint32_t image)
{
	struct audio *audio = data;
	struct msm_adsp_module *module = audio->audplay;
	if (module->state == ADSP_STATE_DISABLED && audio->enabled) {
		atomic_set(&audio->image_swap, 1);
		schedule_work(&audio->work);
	} else if (module->state == ADSP_STATE_ENABLED) {
		atomic_set(&audio->curr_img, image);
	}
}

static void audio_dsp_event(void *private, unsigned id, uint16_t *msg)
{
	struct audio *audio = private;

	switch (id) {
	case AUDPP_MSG_STATUS_MSG: {
		unsigned id = msg[0];
		unsigned status = msg[1];

		if (id != audio->dec_id) {
			MM_AUD_ERR("aac:bogus id\n");
			break;
		}
		switch (status) {
		case AUDPP_DEC_STATUS_SLEEP:
			MM_AUD_INFO("aac:decoder status: sleep \n");
			break;
		case AUDPP_DEC_STATUS_INIT: {
			MM_AUD_INFO("aac:decoder status: init \n");
			audpp_cmd_cfg_adec_params(audio);
			audplay_cmd_set_aac_dual_mono_mode(audio);
			break;
		}
		case AUDPP_DEC_STATUS_CFG:
			MM_AUD_INFO("aac:decoder status: cfg \n");
			break;
		case AUDPP_DEC_STATUS_PLAY:
			MM_AUD_INFO("aac:decoder status: play \n");
			break;
		default:
			MM_AUD_ERR("aac:unknown decoder status \n");
			break;
		}
		break;
	}
	case AUDPP_MSG_CFG_MSG:
		if (msg[0] == AUDPP_MSG_ENA_ENA) {
			MM_AUD_INFO("audio_aac_dsp_event: CFG_MSG ENABLE\n");
			auddec_dsp_config(audio, 1);
			audio->out_needed = 0;
			audio->running = 1;
			audpp_set_volume_and_pan(audio->dec_id,
					audio->volume, 0);
			audpp_avsync(audio->dec_id, 22050);
		} else if (msg[0] == AUDPP_MSG_ENA_DIS) {
			MM_AUD_INFO("audio_aac_dsp_event: CFG_MSG DISABLE\n");
			audpp_avsync(audio->dec_id, 0);
			audio->running = 0;
		} else {
			MM_AUD_ERR("audio_aac_dsp_event: CFG_MSG %d?\n", msg[0]);
		}
		break;
	case AUDPP_MSG_AVSYNC_MSG: {
		uint32_t byte_count;
		uint32_t sample_count;
		byte_count = audpp_avsync_byte_count(audio->dec_id);
		sample_count = audpp_avsync_sample_count(audio->dec_id);
		if (atomic_read(&audio->image_swap) && byte_count == 0)
			audio->total_consumed_bytes += audio->consumed_bytes;
		audio->consumed_bytes = byte_count;
		break;
	}
	case AUDPP_MSG_PCMDMAMISSED:
		if (atomic_read(&audio->wait_adsp_done)) {
			if (audio->adsp_done == 0) {
				audio->adsp_done = 1;
				wake_up(&audio->adsp_wait);
			}
		}
		break;
	default:
		MM_AUD_ERR("audio_aac_dsp_event: UNKNOWN (%d)\n", id);
	}

}

static void audio_modem_event(void *private, unsigned image_swap)
{
	struct audio *audio = private;
	atomic_set(&audio->audpp_notify, image_swap);
	wake_up(&audio->audpp_wait);
}

static int auddec_dsp_config(struct audio *audio, int enable)
{
	audpp_cmd_cfg_dec_type cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDPP_CMD_CFG_DEC_TYPE;
	if (enable)
		cmd.dec0_cfg = AUDPP_CMD_UPDATDE_CFG_DEC |
			       AUDPP_CMD_ENA_DEC_V |
			       AUDDEC_DEC_AACLC;
	else
		cmd.dec0_cfg = AUDPP_CMD_UPDATDE_CFG_DEC |
			       AUDPP_CMD_DIS_DEC_V;

	return audpp_send_queue1(&cmd, sizeof(cmd));
}

static int auddec_flush_decoder(struct audio *audio)
{
	audpp_cmd_dec_ctrl cmd;
	uint16_t *cmd_ptr = &(cmd.dec0_ctrl);

	memset(&cmd, 0, sizeof(cmd));
	cmd_ptr[audio->dec_id] = AUDPP_CMD_UPDATE_V |
				 AUDPP_CMD_FLUSH_V;
	return audpp_dec_ctrl(&cmd);
}

static int auddec_pause_decoder(struct audio *audio)
{
	audpp_cmd_dec_ctrl cmd;
	uint16_t *cmd_ptr = &(cmd.dec0_ctrl);

	memset(&cmd, 0, sizeof(cmd));
	cmd_ptr[audio->dec_id] = AUDPP_CMD_UPDATE_V |
				 AUDPP_CMD_PAUSE_V;
	return audpp_dec_ctrl(&cmd);
}

static int auddec_resume_decoder(struct audio *audio)
{
	audpp_cmd_dec_ctrl cmd;
	uint16_t *cmd_ptr = &(cmd.dec0_ctrl);

	memset(&cmd, 0, sizeof(cmd));
	cmd_ptr[audio->dec_id] = AUDPP_CMD_UPDATE_V |
				 AUDPP_CMD_RESUME_V;
	return audpp_dec_ctrl(&cmd);
}

static void audpp_cmd_cfg_adec_params(struct audio *audio)
{
	audpp_cmd_cfg_adec_params_aac cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.cmd_id =
			AUDPP_CMD_CFG_ADEC_PARAMS;
	cmd.common.length =
			AUDPP_CMD_CFG_ADEC_PARAMS_AAC_LEN;
	cmd.common.dec_id = audio->dec_id;
	cmd.common.input_sampling_frequency =
			audio->out_sample_rate;
	cmd.format = audio->format;
	cmd.audio_object = audio->audio_object;
	cmd.ep_config = audio->ep_config;
	cmd.aac_section_data_resilience_flag =
			audio->aac_section_data_resilience_flag;
	cmd.aac_scalefactor_data_resilience_flag =
			audio->aac_scalefactor_data_resilience_flag;
	cmd.aac_spectral_data_resilience_flag =
			audio->aac_spectral_data_resilience_flag;
	cmd.sbr_on_flag = audio->sbr_on_flag;
	cmd.sbr_ps_on_flag = audio->sbr_ps_on_flag;
	audpp_send_queue2(&cmd, sizeof(cmd));
}
/* ------------------- device --------------------- */

static void audio_flush(struct audio *audio)
{
	int n;

	if (audio->enabled) {
		MM_AUD_INFO("audio_flush: flush decoder\n");
		auddec_flush_decoder(audio);
		audio->sent_bytes = 0;
		audio->total_consumed_bytes = 0;
	}

	for (n = 0; n < BUFCNT; n++) {
		audio->out[n].used = 0;
		audio->out[n].offset = 0;
	}
	audio->out_head = 0;
	audio->out_tail = 0;
	audio->stopped = 0;
	audio->flushed = 0;
	atomic_set(&audio->out_bytes, 0);
}

static long audio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct audio *audio = file->private_data;
	int rc;

	if (cmd == AUDIO_GET_STATS) {
		struct msm_audio_stats stats;
		stats.byte_count = audpp_avsync_byte_count(audio->dec_id);
		stats.sample_count = audpp_avsync_sample_count(audio->dec_id);
		if (copy_to_user((void *) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}
	if (cmd == AUDIO_SET_VOLUME) {
		unsigned long flags;
		spin_lock_irqsave(&audio->dsp_lock, flags);
		audio->volume = arg;
		if (audio->running) {
			audpp_set_volume_and_pan(audio->dec_id, arg, 0);
			htc_pwrsink_audio_volume_set(PWRSINK_AUDIO_AAC,
					audio->volume);
		}
		spin_unlock_irqrestore(&audio->dsp_lock, flags);
		return 0;
	}
	if (cmd == AUDIO_ADSP_PAUSE) {
		unsigned long flags;
		spin_lock_irqsave(&audio->dsp_lock, flags);
		if (audio->running) {
			audio->paused = 1;
			auddec_pause_decoder(audio);
			wake_up(&audio->adsp_wait);
		}
		spin_unlock_irqrestore(&audio->dsp_lock, flags);
		return 0;
	}

	if (cmd == AUDIO_ADSP_RESUME) {
		unsigned long flags;
		spin_lock_irqsave(&audio->dsp_lock, flags);
		if (audio->running) {
			audio->paused = 0;
			auddec_resume_decoder(audio);
		}
		spin_unlock_irqrestore(&audio->dsp_lock, flags);
		return 0;
	}

	mutex_lock(&audio->lock);
	switch (cmd) {
	case AUDIO_START:
		rc = audio_enable(audio);
		break;
	case AUDIO_STOP:
		rc = audio_disable(audio);
		audio->stopped = 1;
		break;
	case AUDIO_FLUSH:
		if (audio->enabled)
			audio->flushed = 1;
		wake_up(&audio->wait);
		mutex_lock(&audio->write_lock);
		audio_flush(audio);
		mutex_unlock(&audio->write_lock);
		rc = 0;
		break;
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		if (copy_from_user(&config, (void *) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
		if (config.channel_count == 1) {
			config.channel_count = AUDPP_CMD_PCM_INTF_MONO_V;
		} else if (config.channel_count == 2) {
			config.channel_count = AUDPP_CMD_PCM_INTF_STEREO_V;
		} else {
			rc = -EINVAL;
			break;
		}
		audio->out_sample_rate = config.sample_rate;
		audio->out_channel_mode = config.channel_count;
		rc = 0;
		break;
	}
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = BUFSZ;
		config.buffer_count = 2;
		config.sample_rate = audio->out_sample_rate;
		if (audio->out_channel_mode ==
				AUDPP_CMD_PCM_INTF_MONO_V) {
			config.channel_count = 1;
		} else {
			config.channel_count = 2;
		}
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		if (copy_to_user((void *) arg, &config,
				sizeof(config))) {
			rc = -EFAULT;
		} else {
			rc = 0;
		}
		break;
	}
	case AUDIO_SET_AAC_CONFIG: {
		struct msm_audio_aac_config aac_config;
		if (copy_from_user(&aac_config, (void *) arg,
				sizeof(aac_config))) {
			rc = -EFAULT;
			break;
		}
		audio->format = aac_config.format;
		audio->audio_object = aac_config.audio_object;
		audio->ep_config = aac_config.ep_config;
		audio->aac_section_data_resilience_flag =
				aac_config.aac_section_data_resilience_flag;
		audio->aac_scalefactor_data_resilience_flag =
				aac_config.aac_scalefactor_data_resilience_flag;
		audio->aac_spectral_data_resilience_flag =
				aac_config.aac_spectral_data_resilience_flag;
		audio->sbr_on_flag = aac_config.sbr_on_flag;
		audio->sbr_ps_on_flag = aac_config.sbr_ps_on_flag;
		audio->dual_mono_mode = aac_config.dual_mono_mode;
		audio->channel_configuration = aac_config.channel_configuration;
		rc = 0;
		break;
	}
	case AUDIO_WAIT_ADSP_DONE: {
		int timeout;
		atomic_set(&audio->wait_adsp_done, 1);
		timeout = wait_event_interruptible_timeout(audio->adsp_wait,
			(audio->adsp_done || audio->paused), 15 * HZ);
		rc = (timeout == 0) ?
			-ETIMEDOUT : ((audio->paused) ? -EPERM : 0);
		audio->adsp_done = 0;
		atomic_set(&audio->wait_adsp_done, 0);
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&audio->lock);
	return rc;
}

static ssize_t audio_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos)
{
	return -EINVAL;
}

static inline int rt_policy(int policy)
{
	if (unlikely(policy == SCHED_FIFO) || unlikely(policy == SCHED_RR))
		return 1;
	return 0;
}

static inline int task_has_rt_policy(struct task_struct *p)
{
	return rt_policy(p->policy);
}

static ssize_t audio_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct sched_param s = { .sched_priority = 1 };
	struct audio *audio = file->private_data;
	const char __user *start = buf;
	struct buffer *frame;
	size_t xfer;
	int old_prio = current->rt_priority;
	int old_policy = current->policy;
	int cap_nice = cap_raised(current_cap(), CAP_SYS_NICE);
	int rc = 0;

	/* just for this write, set us real-time */
	if (!task_has_rt_policy(current)) {
		struct cred *new = prepare_creds();
		if (new != NULL) {
		    cap_raise(new->cap_effective, CAP_SYS_NICE);
		    commit_creds(new);
		    sched_setscheduler(current, SCHED_RR, &s);
		}
	}

	if (count & 1)
		return -EINVAL;

	mutex_lock(&audio->write_lock);
	while (count > 0) {
		frame = audio->out + audio->out_head;
		rc = wait_event_interruptible(
				audio->wait,
				(frame->used == 0) ||
				(audio->stopped) ||
				(audio->flushed));
		if (rc < 0)
			break;
		if (audio->stopped || audio->flushed) {
			rc = -EBUSY;
			break;
		}
		xfer = (count > frame->size) ? frame->size : count;
		if (copy_from_user(frame->data, buf, xfer)) {
			rc = -EFAULT;
			break;
		}

		frame->used = xfer;
		audio->out_head = incr(audio->out_head);
		count -= xfer;
		buf += xfer;

		if (atomic_read(&audio->curr_img))
			audplay_send_lp_data(audio, 0);
		else
			audplay_send_data(audio, 0);

	}
	mutex_unlock(&audio->write_lock);

	/* restore scheduling policy and priority */
	if (!rt_policy(old_policy)) {
		struct sched_param v = { .sched_priority = old_prio };
		sched_setscheduler(current, old_policy, &v);
		if (likely(!cap_nice)) {
			struct cred *new = prepare_creds();
			if (new != NULL) {
			    cap_lower(new->cap_effective, CAP_SYS_NICE);
			    commit_creds(new);
			}
		}
	}

	if (buf > start)
		return buf - start;
	return rc;
}

static int audio_release(struct inode *inode, struct file *file)
{
	struct audio *audio = file->private_data;

	mutex_lock(&audio->lock);
	audio_disable(audio);
	audio_flush(audio);
	audio->audplay = NULL;
	audio->opened = 0;
	mutex_unlock(&audio->lock);
	return 0;
}

struct audio the_aac_audio;
static void audio_aac_audmgr_cb(void)
{
	struct audio *audio = &the_aac_audio;
	struct audmgr *audmgr = &audio->audmgr;

	if (audmgr->state == STATE_DISABLING && audio->enabled) {
		if (audmgr_disable_event_rsp(audmgr))
			MM_AUD_ERR("audio_aac_audmgr_cb:"
					" audmgr_disable_event_rsp() failed\n");
	} else if (audmgr->state == STATE_ENABLED) {
		if (atomic_read(&audio->image_swap) == 1) {
			adjust_data_offset(audio, discard_bytes);
			atomic_set(&audio->image_swap, 0);
			if (audpp_enable(audio->dec_id, audio_dsp_event,
					 audio_modem_event, audio))
				MM_AUD_ERR("audio_aac_audmgr_cb:"
						" audpp_enable() failed\n");
		}
	}
}

static void audio_aac_work_func(struct work_struct *work)
{
	struct audio *audio = &the_aac_audio;
	struct msm_adsp_module *module = audio->audplay;
	int rc;
	if (module->state == ADSP_STATE_DISABLED && audio->enabled) {
		auddec_pause_decoder(audio);
		auddec_flush_decoder(audio);
		discard_bytes = audio->sent_bytes -
				(audio->consumed_bytes +
						audio->total_consumed_bytes);
		MM_AUD_INFO("audio_aac_work_func: %d bytes discarded.\n",
				discard_bytes);
		auddec_dsp_config(audio, 0);
		rc = wait_event_interruptible(audio->audpp_wait,
					atomic_read(&audio->audpp_notify));
		if (rc < 0) {
			MM_AUD_ERR("audio_aac_work_func: wait_event failed\n");
			return;
		}
		audpp_disable(audio->dec_id, audio);
		rc = msm_adsp_disable_event_rsp(module);
		if (rc < 0)
			MM_AUD_ERR("audio_aac_work_func:"
					" disable_event_rsp() failed\n");
	}
}

static int audio_open(struct inode *inode, struct file *file)
{
	struct audio *audio = &the_aac_audio;
	int rc, n;

	mutex_lock(&audio->lock);
	if (audio->opened) {
		MM_AUD_ERR("audio_aac: busy\n");
		rc = -EBUSY;
		goto done;
	}

	rc = audmgr_open(&audio->audmgr);
	if (rc)
		goto done;

	audio->audmgr.handle = 0xFFFF;
	audio->out_sample_rate = 44100;
	audio->out_channel_mode = AUDPP_CMD_PCM_INTF_STEREO_V;
	audio->dec_id = 0;

	/* set default aac parameters */
	audio->format = AUDPP_CMD_AAC_FORMAT_ADTS;
	audio->audio_object = AUDPP_CMD_AAC_AUDIO_OBJECT_LC;
	audio->ep_config = 0;
	audio->aac_section_data_resilience_flag = 0;
	audio->aac_scalefactor_data_resilience_flag = 0;
	audio->aac_spectral_data_resilience_flag = 0;
	audio->sbr_on_flag = AUDPP_CMD_AAC_SBR_ON_FLAG_ON;
	audio->sbr_ps_on_flag = AUDPP_CMD_AAC_SBR_PS_ON_FLAG_ON;
	audio->dual_mono_mode = 3;
	audio->channel_configuration = 2;

	audio->dsp_free_len = 0;
	audio->dsp_write_ptr = 0;
	audio->dsp_start_ptr = 0;
	audio->dsp_buf_size = 0;

	for (n = 0; n < BUFCNT; n++) {
		audio->out[n].data = audio->data + (BUFSZ * n);
		audio->out[n].addr = audio->phys + (BUFSZ * n);
		audio->out[n].size = BUFSZ;
	}

	audio->volume = 0x2000; /* Q13 1.0 */

	audio_flush(audio);

	file->private_data = audio;
	audio->opened = 1;
	rc = 0;
done:
	mutex_unlock(&audio->lock);
	return rc;
}

static struct file_operations audio_aac_fops = {
	.owner		= THIS_MODULE,
	.open		= audio_open,
	.release	= audio_release,
	.read		= audio_read,
	.write		= audio_write,
	.unlocked_ioctl	= audio_ioctl,
};

struct miscdevice audio_aac_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_aac",
	.fops	= &audio_aac_fops,
};

static int __init audio_init(void)
{
	the_aac_audio.data = dma_alloc_coherent(NULL, DMASZ,
					 &the_aac_audio.phys, GFP_KERNEL);
	if (!the_aac_audio.data) {
		MM_AUD_ERR("audio_aac: could not allocate DMA buffers\n");
		return -ENOMEM;
	}

	mutex_init(&the_aac_audio.lock);
	mutex_init(&the_aac_audio.write_lock);
	spin_lock_init(&the_aac_audio.dsp_lock);
	init_waitqueue_head(&the_aac_audio.wait);
	init_waitqueue_head(&the_aac_audio.audpp_wait);
	init_waitqueue_head(&the_aac_audio.adsp_wait);
	INIT_WORK(&the_aac_audio.work, audio_aac_work_func);
	wake_lock_init(&the_aac_audio.wakelock, WAKE_LOCK_SUSPEND, "audio_aac");
	return misc_register(&audio_aac_misc);
}

device_initcall(audio_init);
