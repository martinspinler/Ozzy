// SPDX-License-Identifier: MIT
/*
 * Ozzy USB Audio Driver - PCM Subsystem
 *
 * Device-agnostic ALSA PCM implementation. Handles URB lifecycle,
 * ALSA ops, and DMA ring buffer management. Actual audio encoding/
 * decoding is delegated to device ops (process_out_packet / process_in_packet).
 *
 * Copyright (C) 2024 Marcel Bierling <marcel@hackerman.art>
 */

#include <linux/slab.h>
#include <sound/pcm.h>

#include "ozzy.h"
#include "ozzy_log.h"
#include "ozzy_pcm.h"

/* ========================================================================
 * Stream State Management
 * ======================================================================== */

/*
 * ozzy_pcm_get_substream - Map an ALSA substream to our internal struct.
 */
static struct pcm_substream *ozzy_pcm_get_substream(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);

	if (alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return &rt->playback;
	else if (alsa_sub->stream == SNDRV_PCM_STREAM_CAPTURE)
		return &rt->capture;

	ozzy_pcm_err(&rt->chip->dev->dev, "Invalid PCM stream type\n");
	return NULL;
}

static void ozzy_pcm_kill_urbs(struct pcm_runtime *rt);

/*
 * ozzy_pcm_stream_stop - Transition stream to disabled state.
 * Kills all URBs so none are left running once the stream is DISABLED.
 */
static void ozzy_pcm_stream_stop(struct pcm_runtime *rt)
{
	if (rt->stream_state != STREAM_DISABLED) {
		rt->stream_state = STREAM_STOPPING;
		ozzy_pcm_kill_urbs(rt);
		rt->stream_state = STREAM_DISABLED;
	}
}

/*
 * ozzy_pcm_stream_start - Transition stream to running state.
 * Call with stream_mutex held.
 */
static int ozzy_pcm_stream_start(struct pcm_runtime *rt)
{
	if (rt->stream_state == STREAM_DISABLED) {
		rt->panic = false;
		rt->stream_state = STREAM_STARTING;
		rt->stream_state = STREAM_RUNNING;
	}
	return 0;
}

/* ========================================================================
 * URB Lifecycle
 * ======================================================================== */

/*
 * ozzy_pcm_kill_urbs - Kill all PCM URBs (waits for completion).
 */
static void ozzy_pcm_kill_urbs(struct pcm_runtime *rt)
{
	int i, time;

	for (i = 0; i < OZZY_PCM_N_URBS; i++) {
		time = usb_wait_anchor_empty_timeout(&rt->pcm_in_urbs[i].submitted, 100);
		if (!time)
			usb_kill_anchored_urbs(&rt->pcm_in_urbs[i].submitted);
		time = usb_wait_anchor_empty_timeout(&rt->pcm_out_urbs[i].submitted, 100);
		if (!time)
			usb_kill_anchored_urbs(&rt->pcm_out_urbs[i].submitted);
		usb_kill_urb(&rt->pcm_in_urbs[i].instance);
		usb_kill_urb(&rt->pcm_out_urbs[i].instance);

		if (rt->pcm_isoc_out_urbs[i].instance) {
			time = usb_wait_anchor_empty_timeout(&rt->pcm_isoc_out_urbs[i].submitted, 100);
			if (!time)
				usb_kill_anchored_urbs(&rt->pcm_isoc_out_urbs[i].submitted);
			usb_kill_urb(rt->pcm_isoc_out_urbs[i].instance);
		}
		if (rt->pcm_isoc_sync_urbs[i].instance) {
			time = usb_wait_anchor_empty_timeout(&rt->pcm_isoc_sync_urbs[i].submitted, 100);
			if (!time)
				usb_kill_anchored_urbs(&rt->pcm_isoc_sync_urbs[i].submitted);
			usb_kill_urb(rt->pcm_isoc_sync_urbs[i].instance);
		}
	}
}

/* ========================================================================
 * URB Completion Handlers
 * ======================================================================== */

/*
 * ozzy_pcm_in_urb_handler - Input URB completion handler.
 * Calls the device's process_in_packet to decode audio into the ALSA DMA area.
 */
static void ozzy_pcm_in_urb_handler(struct urb *usb_urb)
{
	struct pcm_urb *in_urb = usb_urb->context;
	struct ozzy_chip *chip = in_urb->chip;
	struct pcm_runtime *rt = chip->pcm;
	struct pcm_substream *sub;
	bool do_period_elapsed = false;
	unsigned long flags;
	unsigned int bytes;
	int ret;

	if (!rt || rt->panic || rt->stream_state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||
		     usb_urb->status == -ENODEV ||
		     usb_urb->status == -ECONNRESET ||
		     usb_urb->status == -ESHUTDOWN)) {
		/* Transient unlink: stop resubmitting but do NOT panic --
		 * expected during pre_reset/disconnect teardown. */
		return;
	}

	sub = &rt->capture;
	spin_lock_irqsave(&sub->lock, flags);
	if (sub->active) {
		struct snd_pcm_runtime *alsa_rt = sub->instance->runtime;
		unsigned int pcm_buffer_size = snd_pcm_lib_buffer_bytes(sub->instance);

		bytes = chip->ops->process_in_packet(chip, in_urb->buffer,
						     alsa_rt->dma_area,
						     sub->dma_off,
						     pcm_buffer_size);

		sub->dma_off += bytes;
		if (sub->dma_off >= pcm_buffer_size)
			sub->dma_off -= pcm_buffer_size;

		sub->period_off += bytes;
		if (sub->period_off >= alsa_rt->period_size) {
			sub->period_off %= alsa_rt->period_size;
			do_period_elapsed = true;
		}
	}
	spin_unlock_irqrestore(&sub->lock, flags);

	if (do_period_elapsed)
		snd_pcm_period_elapsed(sub->instance);

	usb_anchor_urb(&in_urb->instance, &in_urb->submitted);
	ret = usb_submit_urb(&in_urb->instance, GFP_ATOMIC);
	if (ret < 0) {
		usb_unanchor_urb(&in_urb->instance);
		goto in_fail;
	}

	return;

in_fail:
	ozzy_pcm_err(&chip->dev->dev, "PCM input URB failure\n");
	rt->panic = true;
}

/*
 * ozzy_pcm_out_urb_handler - Output URB completion handler.
 * Unified handler for both bulk and interrupt output. Calls device's
 * process_out_packet for audio and fill_midi_out for embedded MIDI.
 */
static void ozzy_pcm_out_urb_handler(struct urb *usb_urb)
{
	struct pcm_urb *out_urb = usb_urb->context;
	struct ozzy_chip *chip = out_urb->chip;
	struct pcm_runtime *rt = chip->pcm;
	struct pcm_substream *sub;
	bool do_period_elapsed = false;
	unsigned long flags;
	unsigned int bytes;
	int ret;

	if (!rt || rt->panic || rt->stream_state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||
		     usb_urb->status == -ENODEV ||
		     usb_urb->status == -ECONNRESET ||
		     usb_urb->status == -ESHUTDOWN)) {
		/* Transient unlink: stop resubmitting but do NOT panic --
		 * expected during pre_reset/disconnect teardown. */
		return;
	}

	sub = &rt->playback;
	spin_lock_irqsave(&sub->lock, flags);
	if (sub->active) {
		struct snd_pcm_runtime *alsa_rt = sub->instance->runtime;
		unsigned int pcm_buffer_size = snd_pcm_lib_buffer_bytes(sub->instance);

		bytes = chip->ops->process_out_packet(chip, out_urb->buffer,
						      alsa_rt->dma_area,
						      sub->dma_off,
						      pcm_buffer_size);

		sub->dma_off += bytes;
		if (sub->dma_off >= pcm_buffer_size)
			sub->dma_off -= pcm_buffer_size;

		sub->period_off += bytes;
		if (sub->period_off >= alsa_rt->period_size) {
			sub->period_off %= alsa_rt->period_size;
			do_period_elapsed = true;
		}
	} else {
		/* No active playback -- re-initialize silence pattern */
		chip->ops->init_out_urb(chip, out_urb->buffer);
	}
	spin_unlock_irqrestore(&sub->lock, flags);

	if (do_period_elapsed)
		snd_pcm_period_elapsed(sub->instance);

	/* Embed pending MIDI output bytes */
	if (chip->ops->fill_midi_out)
		chip->ops->fill_midi_out(chip, out_urb->buffer);

	usb_anchor_urb(&out_urb->instance, &out_urb->submitted);
	ret = usb_submit_urb(&out_urb->instance, GFP_ATOMIC);
	if (ret < 0) {
		usb_unanchor_urb(&out_urb->instance);
		goto out_fail;
	}

	return;

out_fail:
	ozzy_pcm_err(&chip->dev->dev, "PCM output URB failure\n");
	rt->panic = true;
}

/* ========================================================================
 * Isochronous Output + Sync Feedback
 *
 * Used instead of the fixed-size bulk/interrupt out path when
 * chip->info->isoc_out_packets is nonzero. Isochronous transfers have
 * no flow control, so the number of audio frames per USB packet must
 * be computed per-URB from an accumulator driven by the device's
 * current sample rate. If the device also exposes a sync (feedback)
 * endpoint, its packets report the actual USB-frame rate the device is
 * consuming at, which is averaged in to correct for clock drift.
 * ======================================================================== */

static const uint32_t OZZY_ISOC_UPPS = 8000; /* USB packets/sec (125us = USB 2.0 uframe) */

/*
 * ozzy_pcm_distribute_isoc - Fill in iso_frame_desc[] lengths for one URB.
 *
 * Call with the playback substream lock held. Consumes any pending
 * sync feedback samples, folds them into a smoothed rate estimate via
 * a simple IIR filter, then distributes ALSA frames across the URB's
 * USB packets according to that rate. Returns the total number of
 * ALSA bytes needed to fill this URB.
 */
static size_t ozzy_pcm_distribute_isoc(struct ozzy_chip *chip, struct pcm_isoc_urb *out_urb)
{
	struct pcm_runtime *rt = chip->pcm;
	struct urb *urb = out_urb->instance;
	struct pcm_substream *sub = &rt->playback;
	const struct ozzy_device_info *info = chip->info;
	unsigned int alsa_frame_bytes = info->playback_channels * info->bytes_per_sample;

	const uint32_t nominal_rate = info->rates[chip->current_rate];
	uint32_t current_frame_rate = sub->isoc_rate ? sub->isoc_rate : nominal_rate;
	size_t acc = sub->isoc_acc;
	size_t off = 0;
	int i;

	uint16_t sync_cnt = rt->sync_wr - rt->sync_rd;

	if (sync_cnt > 0) {
		size_t sfcs = 0;
		size_t valid = 0;
		const uint8_t fpms_min = nominal_rate / 1000 - 5;
		const uint8_t fpms_max = nominal_rate / 1000 + 5;

		while (rt->sync_rd != rt->sync_wr) {
			uint16_t sfc = rt->sync_frames[rt->sync_rd];

			rt->sync_rd++;
			if (sfc != 255 && sfc >= fpms_min && sfc <= fpms_max) {
				sfcs += sfc;
				valid++;
			}
		}

		if (valid > 0) {
			/* Average frames/ms -> Hz, then IIR-filter into the running
			 * rate. The 1/8 blend factor smooths quantization noise
			 * while still tracking real (ppm-level) clock drift. */
			uint32_t measured = (uint32_t)(sfcs * 1000 / valid);

			current_frame_rate = (current_frame_rate * 7 + measured) / 8;
		}

		sub->isoc_rate = current_frame_rate;
	}

	for (i = 0; i < urb->number_of_packets; i++) {
		size_t p_frames;
		size_t len;

		acc += current_frame_rate;
		p_frames = acc / OZZY_ISOC_UPPS;
		acc -= p_frames * OZZY_ISOC_UPPS;

		len = p_frames * alsa_frame_bytes;
		urb->iso_frame_desc[i].offset = off;
		urb->iso_frame_desc[i].length = len;
		off += len;
	}

	sub->isoc_acc = acc;

	return off;
}

/*
 * ozzy_pcm_isoc_sync_urb_handler - Isochronous sync (feedback) URB handler.
 * Records the frames/ms value from each feedback packet into the sync
 * ring buffer for ozzy_pcm_distribute_isoc() to consume, then resubmits.
 */
static void ozzy_pcm_isoc_sync_urb_handler(struct urb *usb_urb)
{
	struct pcm_isoc_urb *sync_urb = usb_urb->context;
	struct ozzy_chip *chip = sync_urb->chip;
	struct pcm_runtime *rt = chip->pcm;
	int i, ret;

	if (!rt || rt->panic || rt->stream_state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT || usb_urb->status == -ENODEV ||
		     usb_urb->status == -ECONNRESET || usb_urb->status == -ESHUTDOWN)) {
		/* Transient unlink: stop resubmitting but do NOT panic. */
		return;
	}

	if (unlikely(usb_urb->status))
		goto fail;

	for (i = 0; i < OZZY_ISOC_SYNC_PKTS; i++) {
		if (usb_urb->iso_frame_desc[i].actual_length != 3)
			rt->sync_frames[rt->sync_wr] = 255;
		else
			rt->sync_frames[rt->sync_wr] = sync_urb->buffer[i * 0x40 + 0];
		rt->sync_wr++;
	}

	usb_anchor_urb(sync_urb->instance, &sync_urb->submitted);
	ret = usb_submit_urb(sync_urb->instance, GFP_ATOMIC);
	if (ret < 0) {
		usb_unanchor_urb(sync_urb->instance);
		goto fail;
	}

	return;

fail:
	ozzy_pcm_err(&chip->dev->dev, "Isoc sync URB failure\n");
	rt->panic = true;
}

/*
 * ozzy_pcm_isoc_out_urb_handler - Isochronous output URB completion handler.
 * Computes this URB's packet framing via ozzy_pcm_distribute_isoc(), then
 * copies ALSA audio (or silence) directly into the URB buffer -- isoc
 * playback here is a plain interleaved copy, no device-specific encoding.
 */
static void ozzy_pcm_isoc_out_urb_handler(struct urb *usb_urb)
{
	struct pcm_isoc_urb *out_urb = usb_urb->context;
	struct ozzy_chip *chip = out_urb->chip;
	struct pcm_runtime *rt = chip->pcm;
	struct pcm_substream *sub;
	bool do_period_elapsed = false;
	unsigned long flags;
	size_t off;
	int ret;

	if (!rt || rt->panic || rt->stream_state == STREAM_STOPPING)
		return;

	sub = &rt->playback;

	if (unlikely(usb_urb->status == -ENOENT || usb_urb->status == -ENODEV ||
		     usb_urb->status == -ECONNRESET || usb_urb->status == -ESHUTDOWN)) {
		/* Transient unlink: stop resubmitting but do NOT panic. */
		return;
	}

	if (unlikely(usb_urb->status))
		goto fail;

	spin_lock_irqsave(&sub->lock, flags);

	off = ozzy_pcm_distribute_isoc(chip, out_urb);

	if (sub->active) {
		struct snd_pcm_runtime *alsa_rt = sub->instance->runtime;
		unsigned int pcm_buffer_size = snd_pcm_lib_buffer_bytes(sub->instance);

		if (sub->dma_off + off <= pcm_buffer_size) {
			memcpy(out_urb->buffer, alsa_rt->dma_area + sub->dma_off, off);
		} else {
			/* wrap around at end of ring buffer */
			size_t len1 = pcm_buffer_size - sub->dma_off;
			size_t len2 = off - len1;

			memcpy(out_urb->buffer, alsa_rt->dma_area + sub->dma_off, len1);
			memcpy(out_urb->buffer + len1, alsa_rt->dma_area, len2);
		}

		sub->dma_off += off;
		if (sub->dma_off >= pcm_buffer_size)
			sub->dma_off -= pcm_buffer_size;

		sub->period_off += off;
		if (sub->period_off >= alsa_rt->period_size) {
			sub->period_off %= alsa_rt->period_size;
			do_period_elapsed = true;
		}
	} else {
		memset(out_urb->buffer, 0, off);
	}
	spin_unlock_irqrestore(&sub->lock, flags);

	if (do_period_elapsed)
		snd_pcm_period_elapsed(sub->instance);

	usb_anchor_urb(out_urb->instance, &out_urb->submitted);
	ret = usb_submit_urb(out_urb->instance, GFP_ATOMIC);
	if (ret < 0) {
		usb_unanchor_urb(out_urb->instance);
		goto fail;
	}

	return;

fail:
	ozzy_pcm_err(&chip->dev->dev, "Isoc output URB failure\n");
	rt->panic = true;
}

/* ========================================================================
 * ALSA PCM Operations
 * ======================================================================== */

/*
 * ozzy_pcm_open - ALSA PCM open callback.
 * Builds snd_pcm_hardware dynamically from the device's info descriptor.
 */
static int ozzy_pcm_open(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct ozzy_chip *chip = rt->chip;
	const struct ozzy_device_info *info = chip->info;
	struct pcm_substream *sub;
	struct snd_pcm_runtime *alsa_rt = alsa_sub->runtime;
	unsigned int channels, frames_per_packet, alsa_frame_bytes, alsa_pkt_bytes;

	if (rt->panic)
		return -EPIPE;

	if (alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sub = &rt->playback;
		channels = info->playback_channels;
		frames_per_packet = info->frames_per_out_packet;
	} else if (alsa_sub->stream == SNDRV_PCM_STREAM_CAPTURE) {
		sub = &rt->capture;
		channels = info->capture_channels;
		frames_per_packet = info->frames_per_in_packet;
	} else {
		return -EINVAL;
	}

	/* Playback and capture channel counts (and thus packet sizing) can
	 * differ -- e.g. a device with stereo isoc playback but 8-channel
	 * capture -- so compute these per-stream rather than always from
	 * the playback topology. */
	alsa_frame_bytes = channels * info->bytes_per_sample;
	alsa_pkt_bytes = frames_per_packet * alsa_frame_bytes;

	mutex_lock(&rt->stream_mutex);

	/* Build hardware descriptor from device info */
	alsa_rt->hw.info = SNDRV_PCM_INFO_MMAP |
			   SNDRV_PCM_INFO_INTERLEAVED |
			   SNDRV_PCM_INFO_BLOCK_TRANSFER |
			   SNDRV_PCM_INFO_PAUSE |
			   SNDRV_PCM_INFO_MMAP_VALID;
	alsa_rt->hw.formats = info->alsa_format;
	alsa_rt->hw.rates = info->rates_mask;
	alsa_rt->hw.rate_min = info->rate_min;
	alsa_rt->hw.rate_max = info->rate_max;
	alsa_rt->hw.channels_min = channels;
	alsa_rt->hw.channels_max = channels;
	alsa_rt->hw.buffer_bytes_max = 2000 * alsa_pkt_bytes;
	alsa_rt->hw.period_bytes_min = 2 * alsa_pkt_bytes;
	alsa_rt->hw.period_bytes_max = 2000 * alsa_pkt_bytes;
	alsa_rt->hw.periods_min = 2;
	alsa_rt->hw.periods_max = 1024;

	sub->instance = alsa_sub;
	sub->active = false;
	mutex_unlock(&rt->stream_mutex);
	return 0;
}

/*
 * ozzy_pcm_close - ALSA PCM close callback.
 * Deactivates the substream and stops streaming if all substreams are closed.
 */
static int ozzy_pcm_close(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct ozzy_chip *chip = rt->chip;
	struct pcm_substream *sub = ozzy_pcm_get_substream(alsa_sub);
	unsigned long flags;

	if (rt->panic)
		return 0;

	mutex_lock(&rt->stream_mutex);
	if (sub) {
		spin_lock_irqsave(&sub->lock, flags);
		sub->instance = NULL;
		sub->active = false;
		spin_unlock_irqrestore(&sub->lock, flags);

		if (!rt->playback.instance && !rt->capture.instance) {
			ozzy_pcm_stream_stop(rt);
			rt->rate = chip->info->num_rates;
		}
	}
	mutex_unlock(&rt->stream_mutex);
	return 0;
}

/*
 * ozzy_pcm_set_rate - Handle sample rate changes, resetting device if needed.
 * Call with stream_mutex held.
 */
static int ozzy_pcm_set_rate(struct pcm_runtime *rt)
{
	struct ozzy_chip *chip = rt->chip;

	chip->requested_rate = rt->rate;

	if (chip->requested_rate != chip->current_rate) {
		ozzy_pcm_err(&chip->dev->dev,
			   "Resetting device for sample rate change %u -> %u\n",
			   chip->info->rates[chip->current_rate],
			   chip->info->rates[chip->requested_rate]);

		/* Set the new rate via device ops */
		chip->ops->set_rate(chip, chip->requested_rate);

		/* Reset the device -- pre_reset/post_reset handle URB lifecycle */
		mutex_unlock(&rt->stream_mutex);
		chip->ops->reset(chip);
		mutex_lock(&rt->stream_mutex);
	}

	return 0;
}

/*
 * ozzy_pcm_prepare - ALSA PCM prepare callback.
 * Validates format, sets sample rate, and starts the stream.
 */
static int ozzy_pcm_prepare(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct ozzy_chip *chip = rt->chip;
	struct pcm_substream *sub = ozzy_pcm_get_substream(alsa_sub);
	struct snd_pcm_runtime *alsa_rt = alsa_sub->runtime;
	unsigned int i;
	int ret;

	if (rt->panic)
		return -EPIPE;
	if (!sub)
		return -ENODEV;

	mutex_lock(&rt->stream_mutex);

	if (alsa_rt->format != SNDRV_PCM_FORMAT_S24_3LE) {
		mutex_unlock(&rt->stream_mutex);
		return -EINVAL;
	}

	sub->dma_off = 0;
	sub->period_off = 0;

	if (rt->stream_state == STREAM_DISABLED) {
		/* Find the rate index */
		for (i = 0; i < chip->info->num_rates; i++) {
			if (alsa_rt->rate == chip->info->rates[i])
				break;
		}
		if (i == chip->info->num_rates) {
			mutex_unlock(&rt->stream_mutex);
			ozzy_pcm_err(&chip->dev->dev, "Invalid sample rate %d\n",
				alsa_rt->rate);
			return -EINVAL;
		}

		rt->rate = i;

		ret = ozzy_pcm_set_rate(rt);
		if (ret) {
			mutex_unlock(&rt->stream_mutex);
			return ret;
		}

		ret = ozzy_pcm_stream_start(rt);
		if (ret) {
			mutex_unlock(&rt->stream_mutex);
			ozzy_pcm_err(&chip->dev->dev, "Could not start PCM stream\n");
			return ret;
		}
	}

	mutex_unlock(&rt->stream_mutex);
	return 0;
}

/*
 * ozzy_pcm_trigger - ALSA PCM trigger callback.
 * Activates or deactivates the substream for URB processing.
 */
static int ozzy_pcm_trigger(struct snd_pcm_substream *alsa_sub, int cmd)
{
	struct pcm_substream *sub = ozzy_pcm_get_substream(alsa_sub);
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);

	if (rt->panic)
		return -EPIPE;
	if (!sub)
		return -ENODEV;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irq(&sub->lock);
		sub->active = true;
		spin_unlock_irq(&sub->lock);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irq(&sub->lock);
		sub->active = false;
		spin_unlock_irq(&sub->lock);
		return 0;

	default:
		return -EINVAL;
	}
}

/*
 * ozzy_pcm_pointer - ALSA PCM pointer callback.
 * Returns the current DMA position in frames.
 */
static snd_pcm_uframes_t ozzy_pcm_pointer(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_substream *sub = ozzy_pcm_get_substream(alsa_sub);
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	unsigned long flags;
	snd_pcm_uframes_t dma_offset;

	if (rt->panic || !sub) {
		ozzy_pcm_err(&rt->chip->dev->dev, "PCM XRUN\n");
		return SNDRV_PCM_POS_XRUN;
	}

	spin_lock_irqsave(&sub->lock, flags);
	dma_offset = sub->dma_off;
	spin_unlock_irqrestore(&sub->lock, flags);

	return bytes_to_frames(alsa_sub->runtime, dma_offset);
}

static const struct snd_pcm_ops ozzy_pcm_ops = {
	.open    = ozzy_pcm_open,
	.close   = ozzy_pcm_close,
	.prepare = ozzy_pcm_prepare,
	.trigger = ozzy_pcm_trigger,
	.pointer = ozzy_pcm_pointer,
};

/* ========================================================================
 * URB Initialization
 * ======================================================================== */

/*
 * ozzy_pcm_init_out_urb - Initialize a single output URB.
 * Detects bulk vs interrupt and fills the appropriate USB pipe.
 */
static int ozzy_pcm_init_out_urb(struct pcm_urb *urb, struct ozzy_chip *chip)
{
	const struct ozzy_device_info *info = chip->info;
	unsigned int pkt_size;

	urb->chip = chip;
	usb_init_urb(&urb->instance);

	/* Get packet size (may differ for bulk vs interrupt) */
	if (chip->ops->get_out_packet_size)
		pkt_size = chip->ops->get_out_packet_size(chip, chip->is_bulk);
	else
		pkt_size = info->out_packet_size;

	urb->buffer = kzalloc(pkt_size, GFP_KERNEL);
	if (!urb->buffer)
		return -ENOMEM;

	/* Fill initial silence pattern */
	if (chip->ops->init_out_urb)
		chip->ops->init_out_urb(chip, urb->buffer);

	if (chip->is_bulk) {
		usb_fill_bulk_urb(&urb->instance, chip->dev,
				  usb_sndbulkpipe(chip->dev, info->out_ep),
				  urb->buffer, pkt_size,
				  ozzy_pcm_out_urb_handler, urb);
	} else {
		usb_fill_int_urb(&urb->instance, chip->dev,
				 usb_sndintpipe(chip->dev, info->out_ep),
				 urb->buffer, pkt_size,
				 ozzy_pcm_out_urb_handler, urb,
				 chip->dev->ep_out[info->out_ep]->desc.bInterval);
	}

	if (usb_urb_ep_type_check(&urb->instance)) {
		ozzy_pcm_err(&chip->dev->dev, "Output URB endpoint sanity check failed\n");
		return -EINVAL;
	}

	init_usb_anchor(&urb->submitted);
	return 0;
}

/*
 * ozzy_pcm_init_in_urb - Initialize a single input URB.
 * Detects bulk vs interrupt and fills the appropriate USB pipe.
 */
static int ozzy_pcm_init_in_urb(struct pcm_urb *urb, struct ozzy_chip *chip)
{
	const struct ozzy_device_info *info = chip->info;

	urb->chip = chip;
	usb_init_urb(&urb->instance);

	urb->buffer = kzalloc(info->in_packet_size, GFP_KERNEL);
	if (!urb->buffer)
		return -ENOMEM;

	if ((chip->dev->ep_in[info->in_ep]->desc.bmAttributes &
	     USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) {
		usb_fill_bulk_urb(&urb->instance, chip->dev,
				  usb_rcvbulkpipe(chip->dev, info->in_ep),
				  urb->buffer, info->in_packet_size,
				  ozzy_pcm_in_urb_handler, urb);
	} else {
		usb_fill_int_urb(&urb->instance, chip->dev,
				 usb_rcvintpipe(chip->dev, info->in_ep),
				 urb->buffer, info->in_packet_size,
				 ozzy_pcm_in_urb_handler, urb,
				 chip->dev->ep_in[info->in_ep]->desc.bInterval);
	}

	if (usb_urb_ep_type_check(&urb->instance)) {
		ozzy_pcm_err(&chip->dev->dev, "Input URB endpoint sanity check failed\n");
		return -EINVAL;
	}

	init_usb_anchor(&urb->submitted);
	return 0;
}

/*
 * ozzy_pcm_init_isoc_out_urb - Allocate and initialize one isoc output URB.
 * Buffer is sized for the worst case (max rate) since isoc packet
 * lengths vary per URB according to ozzy_pcm_distribute_isoc().
 */
static int ozzy_pcm_init_isoc_out_urb(struct pcm_isoc_urb *urb, struct ozzy_chip *chip)
{
	const struct ozzy_device_info *info = chip->info;
	unsigned int alsa_frame_bytes = info->playback_channels * info->bytes_per_sample;

	urb->chip = chip;
	urb->instance = usb_alloc_urb(info->isoc_out_packets, GFP_KERNEL);
	if (!urb->instance)
		return -ENOMEM;

	urb->len = alsa_frame_bytes * info->isoc_out_packets *
		   (info->rate_max / OZZY_ISOC_UPPS + 1);

	urb->buffer = usb_alloc_coherent(chip->dev, urb->len, GFP_KERNEL, &urb->dma);
	if (!urb->buffer) {
		usb_free_urb(urb->instance);
		urb->instance = NULL;
		return -ENOMEM;
	}
	memset(urb->buffer, 0, urb->len);

	urb->instance->number_of_packets = info->isoc_out_packets;
	urb->instance->interval = 1;
	urb->instance->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;
	urb->instance->transfer_dma = urb->dma;

	usb_fill_bulk_urb(urb->instance, chip->dev,
			  usb_sndisocpipe(chip->dev, info->isoc_out_ep),
			  urb->buffer, 0, ozzy_pcm_isoc_out_urb_handler, urb);

	if (usb_urb_ep_type_check(urb->instance)) {
		ozzy_pcm_err(&chip->dev->dev, "Isoc output URB endpoint sanity check failed\n");
		return -EINVAL;
	}

	init_usb_anchor(&urb->submitted);
	return 0;
}

/*
 * ozzy_pcm_init_isoc_sync_urb - Allocate and initialize one isoc sync URB.
 * Each of the OZZY_ISOC_SYNC_PKTS packets carries a 3-byte feedback
 * value in a fixed 0x40-byte slot.
 */
static int ozzy_pcm_init_isoc_sync_urb(struct pcm_isoc_urb *urb, struct ozzy_chip *chip)
{
	const struct ozzy_device_info *info = chip->info;
	int i;

	urb->chip = chip;
	urb->instance = usb_alloc_urb(OZZY_ISOC_SYNC_PKTS, GFP_KERNEL);
	if (!urb->instance)
		return -ENOMEM;

	urb->len = 0x40 * OZZY_ISOC_SYNC_PKTS;

	urb->buffer = usb_alloc_coherent(chip->dev, urb->len, GFP_KERNEL, &urb->dma);
	if (!urb->buffer) {
		usb_free_urb(urb->instance);
		urb->instance = NULL;
		return -ENOMEM;
	}
	memset(urb->buffer, 0, urb->len);

	for (i = 0; i < OZZY_ISOC_SYNC_PKTS; i++) {
		urb->instance->iso_frame_desc[i].offset = i * 0x40;
		urb->instance->iso_frame_desc[i].length = 0x03;
	}

	urb->instance->number_of_packets = OZZY_ISOC_SYNC_PKTS;
	urb->instance->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;
	urb->instance->interval = 8;
	urb->instance->transfer_dma = urb->dma;

	usb_fill_bulk_urb(urb->instance, chip->dev,
			  usb_rcvisocpipe(chip->dev, info->isoc_sync_ep),
			  urb->buffer, urb->len,
			  ozzy_pcm_isoc_sync_urb_handler, urb);

	if (usb_urb_ep_type_check(urb->instance)) {
		ozzy_pcm_err(&chip->dev->dev, "Isoc sync URB endpoint sanity check failed\n");
		return -EINVAL;
	}

	init_usb_anchor(&urb->submitted);
	return 0;
}

/*
 * ozzy_pcm_free_urbs - Free URB buffers and reset the buffer pointers.
 *
 * Safe to call multiple times (e.g. once from an error path and again
 * from ozzy_pcm_destroy) since kfree(NULL)/usb_free_urb(NULL) are
 * no-ops. Must be called with all URBs killed/poisoned first -- does
 * not touch anchors, only URB buffers and (for isoc) instances.
 */
static void ozzy_pcm_free_urbs(struct pcm_runtime *rt)
{
	uint8_t i;

	for (i = 0; i < OZZY_PCM_N_URBS; i++) {
		kfree(rt->pcm_out_urbs[i].buffer);
		rt->pcm_out_urbs[i].buffer = NULL;
		kfree(rt->pcm_in_urbs[i].buffer);
		rt->pcm_in_urbs[i].buffer = NULL;

		if (rt->pcm_isoc_out_urbs[i].instance) {
			usb_free_coherent(rt->chip->dev, rt->pcm_isoc_out_urbs[i].len,
					  rt->pcm_isoc_out_urbs[i].buffer,
					  rt->pcm_isoc_out_urbs[i].dma);
			usb_free_urb(rt->pcm_isoc_out_urbs[i].instance);
			rt->pcm_isoc_out_urbs[i].instance = NULL;
			rt->pcm_isoc_out_urbs[i].buffer = NULL;
		}
		if (rt->pcm_isoc_sync_urbs[i].instance) {
			usb_free_coherent(rt->chip->dev, rt->pcm_isoc_sync_urbs[i].len,
					  rt->pcm_isoc_sync_urbs[i].buffer,
					  rt->pcm_isoc_sync_urbs[i].dma);
			usb_free_urb(rt->pcm_isoc_sync_urbs[i].instance);
			rt->pcm_isoc_sync_urbs[i].instance = NULL;
			rt->pcm_isoc_sync_urbs[i].buffer = NULL;
		}
	}
}

/*
 * ozzy_pcm_init_urbs - Initialize and submit all PCM URBs.
 * Called during initial probe and after USB device reset (post_reset).
 * Frees any URBs from a previous call first, since post_reset re-runs
 * this on the same pcm_runtime without a matching teardown in between.
 */
int ozzy_pcm_init_urbs(struct ozzy_chip *chip)
{
	struct pcm_runtime *rt = chip->pcm;
	uint8_t i;
	int ret;

	rt->chip = chip;

	ozzy_pcm_free_urbs(rt);

	/* Initialize input URBs */
	for (i = 0; i < OZZY_PCM_N_URBS; i++) {
		ret = ozzy_pcm_init_in_urb(&rt->pcm_in_urbs[i], chip);
		if (ret < 0)
			goto error;
	}

	/* Initialize output URBs -- isochronous or bulk/interrupt */
	if (chip->info->isoc_out_packets) {
		rt->playback.isoc_acc = 0;
		rt->playback.isoc_rate = chip->info->rates[chip->current_rate];

		for (i = 0; i < OZZY_PCM_N_URBS; i++) {
			ret = ozzy_pcm_init_isoc_out_urb(&rt->pcm_isoc_out_urbs[i], chip);
			if (ret < 0)
				goto error;
			ret = ozzy_pcm_init_isoc_sync_urb(&rt->pcm_isoc_sync_urbs[i], chip);
			if (ret < 0)
				goto error;
		}
	} else {
		for (i = 0; i < OZZY_PCM_N_URBS; i++) {
			ret = ozzy_pcm_init_out_urb(&rt->pcm_out_urbs[i], chip);
			if (ret < 0)
				goto error;
		}
	}

	/* Submit all URBs */
	mutex_lock(&rt->stream_mutex);
	for (i = 0; i < OZZY_PCM_N_URBS; i++) {
		usb_anchor_urb(&rt->pcm_in_urbs[i].instance,
			       &rt->pcm_in_urbs[i].submitted);
		ret = usb_submit_urb(&rt->pcm_in_urbs[i].instance, GFP_ATOMIC);
		if (ret < 0) {
			ozzy_pcm_stream_stop(rt);
			ozzy_pcm_kill_urbs(rt);
			goto error_locked;
		}
	}

	if (chip->info->isoc_out_packets) {
		for (i = 0; i < OZZY_PCM_N_URBS; i++) {
			usb_anchor_urb(rt->pcm_isoc_sync_urbs[i].instance,
				       &rt->pcm_isoc_sync_urbs[i].submitted);
			ret = usb_submit_urb(rt->pcm_isoc_sync_urbs[i].instance, GFP_ATOMIC);
			if (ret < 0) {
				ozzy_pcm_stream_stop(rt);
				ozzy_pcm_kill_urbs(rt);
				goto error_locked;
			}
		}
		for (i = 0; i < OZZY_PCM_N_URBS; i++) {
			usb_anchor_urb(rt->pcm_isoc_out_urbs[i].instance,
				       &rt->pcm_isoc_out_urbs[i].submitted);
			ret = usb_submit_urb(rt->pcm_isoc_out_urbs[i].instance, GFP_ATOMIC);
			if (ret < 0) {
				ozzy_pcm_stream_stop(rt);
				ozzy_pcm_kill_urbs(rt);
				goto error_locked;
			}
		}
	} else {
		for (i = 0; i < OZZY_PCM_N_URBS; i++) {
			usb_anchor_urb(&rt->pcm_out_urbs[i].instance,
				       &rt->pcm_out_urbs[i].submitted);
			ret = usb_submit_urb(&rt->pcm_out_urbs[i].instance, GFP_ATOMIC);
			if (ret < 0) {
				ozzy_pcm_stream_stop(rt);
				ozzy_pcm_kill_urbs(rt);
				goto error_locked;
			}
		}
	}
	mutex_unlock(&rt->stream_mutex);

	return 0;

error_locked:
	mutex_unlock(&rt->stream_mutex);
error:
	ozzy_pcm_err(&chip->dev->dev, "PCM URB initialization failed\n");
	ozzy_pcm_free_urbs(rt);
	return ret;
}

/* ========================================================================
 * Init / Abort
 * ======================================================================== */

/*
 * ozzy_pcm_abort - Emergency stop all PCM activity.
 * Sets panic flag and synchronously kills all URBs so the caller
 * (disconnect/pre_reset) can rely on the hardware being fully
 * quiesced before it proceeds, regardless of stream_state.
 */
void ozzy_pcm_abort(struct ozzy_chip *chip)
{
	struct pcm_runtime *rt = chip->pcm;

	if (rt) {
		rt->panic = true;
		ozzy_pcm_kill_urbs(rt);
	}
}

/*
 * ozzy_pcm_init - Create and initialize the PCM subsystem.
 * Allocates the runtime, registers ALSA PCM, and sets up URBs.
 */
int ozzy_pcm_init(struct ozzy_chip *chip)
{
	struct snd_pcm *pcm;
	struct pcm_runtime *rt;
	int ret;

	rt = kzalloc(sizeof(struct pcm_runtime), GFP_KERNEL);
	if (!rt)
		return -ENOMEM;

	rt->chip = chip;
	rt->stream_state = STREAM_DISABLED;

	mutex_init(&rt->stream_mutex);
	spin_lock_init(&rt->playback.lock);
	spin_lock_init(&rt->capture.lock);

	ret = snd_pcm_new(chip->card, chip->dev->product, 0, 1, 1, &pcm);
	if (ret < 0) {
		kfree(rt);
		ozzy_pcm_err(&chip->dev->dev, "Cannot create PCM instance\n");
		return ret;
	}

	pcm->private_data = rt;

	strscpy(pcm->name, chip->dev->product, sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &ozzy_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &ozzy_pcm_ops);
	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL, 0, 0);

	rt->instance = pcm;
	chip->pcm = rt;

	ret = ozzy_pcm_init_urbs(chip);
	if (ret < 0) {
		ozzy_pcm_err(&chip->dev->dev, "PCM URB setup failed\n");
		chip->pcm = NULL;
		kfree(rt);
		return ret;
	}

	return 0;
}

/*
 * ozzy_pcm_destroy - Tear down the PCM subsystem.
 * Called via card->private_free. Ensures URBs are dead before freeing
 * their buffers and the pcm_runtime itself.
 */
void ozzy_pcm_destroy(struct ozzy_chip *chip)
{
	struct pcm_runtime *rt = chip->pcm;

	if (!rt)
		return;

	/* Make sure no completion handler is running before we free buffers */
	rt->panic = true;
	ozzy_pcm_kill_urbs(rt);
	ozzy_pcm_free_urbs(rt);

	chip->pcm = NULL;
	kfree(rt);
}
