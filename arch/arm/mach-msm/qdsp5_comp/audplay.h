/* arch/arm/mach-msm/qdsp5_comp/audplay.h
 *
 * Copyright (C) 2008 HTC, Inc.
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

#ifndef _AUDPLAY_H_
#define _AUDPLAY_H_

#define adsp_rtos_phy_to_vir(phy_addr, virt_base_addr) \
		(phy_addr - MSM_AD5_PHYS + virt_base_addr)
#define adsp_rtos_vir_to_phy(virt_addr) \
		((void __force __iomem *)virt_addr - \
				MSM_AD5_BASE + MSM_AD5_PHYS)
#define MIN(a, b) (a <= b)? a : b

#define BUFSZ 40960
#define BUFCNT 3
#define DMASZ (BUFSZ * BUFCNT)

#define incr(index) \
	((index == BUFCNT - 1) ? 0 : (index + 1))
#define decr(index) \
	((index == 0) ? (BUFCNT - 1) : (index - 1))

struct buffer {
	void *data;
	unsigned size;
	unsigned used;
	unsigned addr;
	unsigned offset;
};

struct audio {
	struct buffer out[BUFCNT];

	spinlock_t dsp_lock;

	uint8_t out_head;
	uint8_t out_tail;
	uint8_t out_needed; /* number of buffers the dsp is waiting for */

	atomic_t out_bytes;

	struct mutex lock;
	struct mutex write_lock;
	wait_queue_head_t wait;

	struct msm_adsp_module *audplay;

	/* configuration to use on next enable */
	uint32_t out_sample_rate;
	uint32_t out_channel_mode;

	struct audmgr audmgr;

	/* data allocated for various buffers */
	char *data;
	dma_addr_t phys;

	int opened;
	int enabled;
	int running;
	int stopped; /* set when stopped, cleared on flush */
	int flushed;
	int paused;
	unsigned volume;

	struct wake_lock wakelock;

	uint16_t dec_id;

	/* variables to support low power playback */
	uint32_t dsp_free_len;
	uint16_t *dsp_write_ptr;
	uint16_t *dsp_start_ptr;
	uint32_t dsp_buf_size;

	atomic_t curr_img;

	/* variables to support image swap */
	atomic_t image_swap;
	atomic_t audpp_notify;
	struct work_struct work;
	wait_queue_head_t audpp_wait;

	uint32_t sent_bytes;
	uint32_t consumed_bytes;
	uint32_t total_consumed_bytes;

	/* aac configuration */
	signed short format;
	unsigned short audio_object;
	unsigned short ep_config;
	unsigned short aac_section_data_resilience_flag;
	unsigned short aac_scalefactor_data_resilience_flag;
	unsigned short aac_spectral_data_resilience_flag;
	unsigned short sbr_on_flag;
	unsigned short sbr_ps_on_flag;
	unsigned short dual_mono_mode;
	unsigned short channel_configuration;

	wait_queue_head_t adsp_wait;
	int adsp_done;
	atomic_t wait_adsp_done;
};

int audplay_dsp_send_data_avail(struct audio *audio, unsigned idx,
					unsigned len, unsigned offset);
void audplay_send_data(struct audio *audio, unsigned needed);
void audplay_adsp_write(uint16_t *dest, uint16_t *src, uint32_t wlen);
void audplay_send_lp_data(struct audio *audio, unsigned needed);
int audplay_cmd_set_aac_dual_mono_mode(struct audio *audio);
void adjust_data_offset(struct audio *audio, uint32_t discard_bytes);
#endif
