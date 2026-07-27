/* SPDX-License-Identifier: MIT */
/*
 * Ozzy USB Audio Driver - PCM Subsystem
 *
 * Copyright (C) 2024 Marcel Bierling <marcel@hackerman.art>
 */

#ifndef OZZY_PCM_H
#define OZZY_PCM_H

struct ozzy_chip;

#define OZZY_PCM_N_URBS  4

struct pcm_urb {
	struct ozzy_chip *chip;
	struct urb instance;
	struct usb_anchor submitted;
	uint8_t *buffer;
};

/*
 * struct pcm_isoc_urb - Isochronous URB (variable-length, DMA-coherent).
 *
 * Unlike pcm_urb, the instance is allocated (not embedded) since
 * usb_alloc_urb(n_packets, ...) appends a variably-sized
 * iso_frame_desc[] array sized for this URB's packet count. The
 * buffer is DMA-coherent, matching what isochronous transfers expect
 * on most host controllers.
 */
struct pcm_isoc_urb {
	struct ozzy_chip *chip;
	struct urb *instance;
	struct usb_anchor submitted;
	uint8_t *buffer;
	size_t len;        /* allocated buffer length in bytes */
	dma_addr_t dma;
};

/* Number of USB packets per isochronous sync (feedback) URB */
#define OZZY_ISOC_SYNC_PKTS  5

struct pcm_substream {
	spinlock_t lock;
	struct snd_pcm_substream *instance;

	bool active;

	snd_pcm_uframes_t dma_off;     /* current position in ALSA DMA area (bytes) */
	snd_pcm_uframes_t period_off;  /* current position within current period */

	/* Isochronous playback only: rate accumulator state */
	size_t isoc_acc;                /* accumulator remainder across URBs */
	uint32_t isoc_rate;             /* smoothed frame rate from sync feedback (Hz) */
};

/* PCM streaming states */
enum {
	STREAM_DISABLED,  /* no PCM streaming */
	STREAM_STARTING,  /* PCM streaming requested, waiting to become ready */
	STREAM_RUNNING,   /* PCM streaming active */
	STREAM_STOPPING   /* PCM streaming shutting down */
};

/*
 * struct pcm_runtime - Per-device PCM subsystem state.
 */
struct pcm_runtime {
	struct ozzy_chip *chip;
	struct snd_pcm *instance;

	struct pcm_substream playback;
	struct pcm_substream capture;
	bool panic;  /* if set, driver won't do any more PCM on this device */

	struct pcm_urb pcm_out_urbs[OZZY_PCM_N_URBS];
	struct pcm_urb pcm_in_urbs[OZZY_PCM_N_URBS];

	/* Only used when chip->info->isoc_out_packets is nonzero */
	struct pcm_isoc_urb pcm_isoc_out_urbs[OZZY_PCM_N_URBS];
	struct pcm_isoc_urb pcm_isoc_sync_urbs[OZZY_PCM_N_URBS];
	uint16_t sync_frames[256 * 256];  /* ring buffer: frames/ms from sync EP */
	uint16_t sync_rd;                 /* read index into sync_frames */
	uint16_t sync_wr;                 /* write index into sync_frames */

	struct mutex stream_mutex;
	uint8_t stream_state;  /* one of STREAM_xxx */
	uint8_t rate;          /* index into device's rate table */
};

/*
 * ozzy_pcm_init - Create and initialize the PCM subsystem.
 * Allocates pcm_runtime, registers ALSA PCM, sets up URBs.
 * Returns 0 on success, negative errno on failure.
 */
int ozzy_pcm_init(struct ozzy_chip *chip);

/*
 * ozzy_pcm_init_urbs - (Re)initialize and submit PCM URBs.
 * Called during initial probe and after USB device reset.
 * Returns 0 on success, negative errno on failure.
 */
int ozzy_pcm_init_urbs(struct ozzy_chip *chip);

/*
 * ozzy_pcm_abort - Emergency stop all PCM URBs.
 * Called during disconnect and pre-reset. Sets panic flag.
 */
void ozzy_pcm_abort(struct ozzy_chip *chip);

/*
 * ozzy_pcm_destroy - Tear down the PCM subsystem.
 * Called via card->private_free. Kills URBs, frees URB buffers and the
 * pcm_runtime itself. Safe to call with chip->pcm == NULL.
 */
void ozzy_pcm_destroy(struct ozzy_chip *chip);

#endif /* OZZY_PCM_H */
