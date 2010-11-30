/* arch/arm/mach-msm/qdsp5_comp/snd.h
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

#ifndef _SND_H_
#define _SND_H_

#include <mach/msm_rpc_version.h>

#define SND_CB_FUNC_PTR_TYPE_PROC 1
#define VOC_PCM_CLIENT_OUTPUT_PTR 2
#define VOC_PCM_CLIENT_INPUT_PTR 3
#define SND_SET_DEVICE_PROC 2
#define SND_SET_VOLUME_PROC 3

#define RPC_SND_PROG	0x30000002
#define RPC_SND_CB_PROG	0x31000002

struct snd_ctxt {
	struct mutex lock;
	int opened;

	struct msm_rpc_endpoint *ept;
	struct task_struct *task;
	int inited;
};

#define RPC_TYPE_REQUEST 0
#define RPC_TYPE_REPLY 1

#define RPC_VERSION 2

#define RPC_COMMON_HDR_SZ  (sizeof(uint32_t) * 2)
#define RPC_REQUEST_HDR_SZ (sizeof(struct rpc_request_hdr))
#define RPC_REPLY_HDR_SZ   (sizeof(uint32_t) * 3)
#define RPC_REPLY_SZ       (sizeof(uint32_t) * 6)

#define SND_IOCTL_MAGIC 's'

#define SND_SET_DEVICE		_IOW(SND_IOCTL_MAGIC, 2, unsigned)
#define SND_SET_VOLUME		_IOW(SND_IOCTL_MAGIC, 3, unsigned)

struct snd_device_config {
	uint32_t device;
	uint32_t ear_mute;
	uint32_t mic_mute;
};

struct snd_volume_config {
	uint32_t device;
	uint32_t method;
	uint32_t volume;
};

struct rpc_snd_set_device_args {
	uint32_t device;
	uint32_t ear_mute;
	uint32_t mic_mute;

	uint32_t cb_func;
	uint32_t client_data;
};

struct rpc_snd_set_volume_args {
	uint32_t device;
	uint32_t method;
	uint32_t volume;

	uint32_t cb_func;
	uint32_t client_data;
};

struct snd_set_device_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_set_device_args args;
};

struct snd_set_volume_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_set_volume_args args;
};

void put_vocpcm_data(uint32_t *, uint32_t, uint32_t,
		struct msm_rpc_endpoint *, uint32_t);
void get_vocpcm_data(const uint32_t *, uint32_t , uint32_t);
void rpc_in_ack(struct msm_rpc_endpoint *, uint32_t);
#endif /*_SND_H_*/

