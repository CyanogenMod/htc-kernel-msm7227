/* arch/arm/mach-msm/qdsp5_comp/audplay.c
 *
 * interface to aDSP audplay module
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
#include <linux/wait.h>

#include <asm/atomic.h>
#include <mach/msm_adsp.h>
#include <mach/msm_iomap.h>
#include <mach/qdsp5/qdsp5audplaycmdi.h>

#include <linux/wakelock.h>
#include <linux/kthread.h>

/* for queue ids - should be relative to module number*/
#include "adsp.h"
#include "audmgr.h"
#include "audplay.h"

#define audplay_send_queue0(audio, cmd, len) \
	msm_adsp_write(audio->audplay, \
			QDSP_uPAudPlay0BitStreamCtrlQueue, cmd, len)

void dump_buffer_status(struct audio *audio)
{
	int ptr;
	struct buffer *frame;

	pr_info("dump_buffer_status: head = %d, tail = %d\n",
		audio->out_head, audio->out_tail);
	for (ptr = 0; ptr < BUFCNT; ptr++) {
		frame = audio->out + ptr;
		pr_info("buffer %d: used = %d, offset = %d\n",
			ptr, frame->used, frame->offset);
	}
}

int audplay_dsp_send_data_avail(struct audio *audio, unsigned idx,
					unsigned len, unsigned offset)
{
	audplay_cmd_bitstream_data_avail cmd;

	cmd.cmd_id		= AUDPLAY_CMD_BITSTREAM_DATA_AVAIL;
	cmd.decoder_id		= audio->dec_id;
	cmd.buf_ptr		= audio->out[idx].addr + offset;
	cmd.buf_size		= len/2;
	cmd.partition_number	= 0;

	if (atomic_read(&audio->curr_img))
		cmd.dsp_write_phy_addr = (uint32_t) audio->dsp_write_ptr;

	return audplay_send_queue0(audio, &cmd, sizeof(cmd));
}

void audplay_send_data(struct audio *audio, unsigned needed)
{
	struct buffer *frame;
	unsigned long flags;

	spin_lock_irqsave(&audio->dsp_lock, flags);
	if (!audio->running)
		goto done;

	if (needed) {
		/* We were called from the callback because the DSP
		 * requested more data.  Note that the DSP does want
		 * more data, and if a buffer was in-flight, mark it
		 * as available (since the DSP must now be done with
		 * it).
		 */
		audio->out_needed = 1;
		frame = audio->out + audio->out_tail;
		if (frame->used == 0xffffffff) {
			frame->used = 0;
			frame->offset = 0;
			audio->out_tail = incr(audio->out_tail);
			wake_up(&audio->wait);
		}
	}

	if (audio->out_needed) {
		/* If the DSP currently wants data and we have a
		 * buffer available, we will send it and reset
		 * the needed flag.  We'll mark the buffer as in-flight
		 * so that it won't be recycled until the next buffer
		 * is requested
		 */

		frame = audio->out + audio->out_tail;
		if (frame->used) {
			BUG_ON(frame->used == 0xffffffff);
			audplay_dsp_send_data_avail(audio, audio->out_tail,\
					frame->used, frame->offset);
			frame->offset += frame->used;
			audio->sent_bytes += frame->used;
			audio->out_needed = 0;
			frame->used = 0xffffffff;
		}
	}
done:
	spin_unlock_irqrestore(&audio->dsp_lock, flags);
}

void audplay_adsp_write(uint16_t *dest, uint16_t *src, uint32_t wlen)
{
	uint16_t  tword;

	BUG_ON((dest == NULL) || (src == NULL));
	BUG_ON(((uint32_t)dest & 1) && ((uint32_t)src & 1));

	/* Prepare for loops */
	wlen++;

	if (!(((uint32_t)dest & 1) || ((uint32_t)src & 1))) {
		/* Dest and Src are aligned */
		while (--wlen) {
			*dest++ = (*src << 8) | (*src >> 8);
			src++;
		}
	} else {
		if ((uint32_t)dest & 1) {
			/* Dest is misaligned */
			pr_info("Odd alignment on dest pointer\n");
			while (--wlen) {
				tword = *src++;
				((uint8_t *)dest)[0] = (uint8_t)(tword >> 8);
				((uint8_t *)dest)[1] = (uint8_t)tword;
				dest++;
			}
		} else {
			/* Src is misaligned */
			pr_info("Odd alignment on src pointer\n");
			while (--wlen) {
				tword = (((uint16_t)((uint8_t *)src)[0]) << 8)|
						((uint16_t)((uint8_t *)src)[1]);
				*dest++ = tword;
				src++;
			}
		}
	}
}

void audplay_send_lp_data(struct audio *audio, unsigned needed)
{
	struct buffer *frame, *frame2;
	unsigned long flags;

	uint32_t dsp_free_len, dsp_write_len, dsp_offset;
	uint32_t num_write, dsp_free_len_copy, num_used = 0;
	uint16_t *arm_next_write;
	uint16_t *adec_data_buf = (uint16_t *)audio->dsp_start_ptr;
	uint8_t odd_buffer[2];
	uint32_t frame2_used_old, ptr;

	if (atomic_read(&audio->image_swap)) {
		pr_err("audplay_send_lp_data: image swap\n");
		return;
	}

	spin_lock_irqsave(&audio->dsp_lock, flags);
	if (needed)
		audio->out_needed = 1;
	if (audio->out_needed) {
		frame = audio->out + audio->out_tail;
		frame2 = audio->out + incr(audio->out_tail);
		frame2_used_old = frame2->used;

		if (frame->used) {
		BUG_ON(frame->used == 0xffffffff);
		if (audio->dsp_free_len > 0) {
			if (audio->dsp_write_ptr == NULL ||
					audio->dsp_start_ptr == NULL)
				goto out;
			dsp_free_len = audio->dsp_free_len;
			dsp_free_len_copy = dsp_free_len;
			arm_next_write = (uint16_t *)audio->dsp_write_ptr;

			/* Calculate offset (in words) we wrote to last time */
			dsp_offset = arm_next_write -
					(uint16_t *)audio->dsp_start_ptr;
			dsp_write_len = MIN(dsp_free_len ,
					audio->dsp_buf_size - dsp_offset);

			num_write = MIN((frame->used >> 1) , dsp_write_len);
			if (num_write != 0) {
				audplay_adsp_write(
						adec_data_buf + dsp_offset,
						(uint16_t *)(frame->data +
								frame->offset),
						num_write);
				frame->offset += 2 * num_write;
				frame->used -= 2 * num_write;
				dsp_offset += num_write;
				dsp_free_len -= num_write;
				dsp_write_len -= num_write;
				arm_next_write += num_write;
			}

			if (dsp_write_len) {
				/* frame->out_tail completly written and some
				more space left in DSP buffer on which data can
				be written.When writing above only even
				number of data is written,so now check
				if any data is still left in frame->out_tail.
				If left then it is odd byte */
				if ((frame->used != 0) && (frame2->used != 0)) {
					odd_buffer[0] = *((uint8_t *)
						(frame->data + frame->offset));
					odd_buffer[1] = *((uint8_t *)
						(frame2->data + frame2->offset));
					audplay_adsp_write(
						adec_data_buf + dsp_offset,
						(uint16_t *)odd_buffer,
						1);
					frame->offset++;
					frame2->offset++;
					frame->used--;
					frame2->used--;
					dsp_offset++;
					dsp_free_len--;
					dsp_write_len--;
					arm_next_write++;
				}
			}

			/* If still empty space left in DSP buffer before
			writing from start of circular buffer then fill this
			empty space with buffer2. */
			num_write = MIN(frame2->used >> 1, dsp_write_len);
			if (num_write != 0) {
				audplay_adsp_write(
						adec_data_buf + dsp_offset,
						(uint16_t *)(frame2->data +
								frame2->offset),
						num_write);
				frame2->offset += 2 * num_write;
				frame2->used -= 2 * num_write;
				dsp_offset += num_write;
				dsp_free_len -= num_write;
				arm_next_write += num_write;
			}

			/* If below condition is false that means space is
			still left at end of circular buffer and we don't
			have enough data(buffer1+buffer2) to fill.
			So we should not touch start of circular buffer. */
			if (audio->dsp_buf_size == dsp_offset) {
				dsp_offset = 0;
				arm_next_write =
					(uint16_t *) audio->dsp_start_ptr;

				num_write = MIN((frame->used >> 1) ,
						dsp_free_len);
				if (num_write != 0) {
					audplay_adsp_write(
						adec_data_buf + dsp_offset,
						(uint16_t *)(frame->data +
						frame->offset),
						num_write);
					frame->offset += 2 * num_write;
					frame->used -= 2 * num_write;
					dsp_offset += num_write;
					dsp_free_len -= num_write;
					arm_next_write += num_write;
				}

				if (dsp_free_len) {
					if ((frame->used != 0) &&
							(frame2->used != 0)) {
						odd_buffer[0] = *((uint8_t *)
							(frame->data +
								frame->offset));
						odd_buffer[1] = *((uint8_t *)
							(frame2->data +
								frame2->offset));
						audplay_adsp_write(
						  adec_data_buf + dsp_offset,
						  (uint16_t *)odd_buffer, 1);
						frame->offset++;
						frame2->offset++;
						frame->used--;
						frame2->used--;
						dsp_offset++;
						dsp_free_len--;
						arm_next_write++;
					}
				}

				num_write = MIN(frame2->used >> 1,
						dsp_free_len);
				if (num_write != 0) {
					audplay_adsp_write(
						adec_data_buf + dsp_offset,
						(uint16_t *)(frame2->data +
							frame2->offset),
						num_write);
					frame2->offset += 2 * num_write;
					frame2->used -= 2 * num_write;
					dsp_offset += num_write;
					dsp_free_len -= num_write;
					arm_next_write += num_write;
				}
			}

			if (dsp_free_len && (frame->used + frame2->used == 1)) {
				/* If there's only one byte remaining,
				just write it, but don't update any of
				our pointers - this will allow DSP to
				process the frame if it is the last byte
				of the last frame*/
				odd_buffer[0] = 0;
				if (frame->used == 1) {
					odd_buffer[1] = *((uint8_t *)
						(frame->data + frame->offset));
					frame->used = 0;
					frame->offset++;
				} else {
					odd_buffer[1] = *((uint8_t *)
						(frame2->data + frame2->offset));
					frame2->used = 0;
					frame2->offset++;
				}
				*(adec_data_buf + dsp_offset) =
						*((uint16_t *) odd_buffer);
			}

			num_used = (dsp_free_len_copy - dsp_free_len) * 2 ;
			audio->dsp_write_ptr = (uint16_t *)
				adsp_rtos_vir_to_phy(
					(uint32_t)arm_next_write);
			audplay_dsp_send_data_avail(
					audio, audio->out_tail, 0, 0);
			audio->sent_bytes += num_used;
			audio->dsp_free_len = 0;
			audio->out_needed = 0;

			/* adjust frame pointers */
			if (frame->used == 0)
				frame->used = 0xffffffff;
			if ((frame2->used == 0) && (frame2_used_old != 0))
				frame2->used = 0xffffffff;
			}
			for (ptr = 0; ptr < 2; ptr++) {
				frame = audio->out + audio->out_tail;
				if (frame->used == 0xffffffff) {
					audio->out_tail =
						incr(audio->out_tail);
					frame = audio->out +
						((audio->out_tail + 1) %
							BUFCNT);
					if (frame->used == 0xffffffff) {
						frame->used = 0;
						frame->offset = 0;
						wake_up(&audio->wait);
					}
				}
			}
		}
	}
out:
	spin_unlock_irqrestore(&audio->dsp_lock, flags);
}

int audplay_cmd_set_aac_dual_mono_mode(struct audio *audio)
{
	audplay_cmd_channel_info cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDPLAY_CMD_AUDDEC_CMD_CHANNEL_INFO;
	cmd.dual_mono_mode = audio->dual_mono_mode;
	return audplay_send_queue0(audio, &cmd, sizeof(cmd));
}

void adjust_data_offset(struct audio *audio, uint32_t discard_bytes)
{
	struct buffer *frame;
	uint32_t count;
	uint8_t idx;

	pr_info("adjust_data_offset: before adjust\n");
	dump_buffer_status(audio);

	while (1) {
		frame = audio->out + audio->out_tail;
		count = MIN(discard_bytes, frame->offset);
		if (count != 0) {
			if (frame->used == 0xffffffff)
				frame->used = count;
			else
				frame->used += count;
			frame->offset -= count;
			discard_bytes -= count;

			pr_info("recover %d bytes from buffer %d\n",
				count, audio->out_tail);
		}

		if (discard_bytes == 0) {
			pr_info("adjust_data_offset: discard_bytes = 0\n");
			break;
		} else if (audio->out_tail == audio->out_head) {
			pr_info("adjust_data_offset: tail = head\n");
			break;
		} else {
			audio->out_tail = decr(audio->out_tail);
			pr_info("adjust_data_offset: change out_tail to %d\n",
				audio->out_tail);
		}
	}

	idx = audio->out_tail;
	while (1) {
		frame = audio->out + idx;
		if (frame->used == 0xffffffff) {
			pr_info("adjust_data_offset: reset frame %d\n",
				idx);
			frame->used = 0;
			frame->offset = 0;
		}
		if (idx == audio->out_head)
			break;
		idx = decr(idx);
	}
	pr_info("adjust_data_offset: after adjust\n");
	dump_buffer_status(audio);
	if (discard_bytes)
		pr_info("adjust_data_offset: %d bytes lost\n",
			discard_bytes);
}
