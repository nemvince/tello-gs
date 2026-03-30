#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Optional callback invoked from the video thread on every decoded frame.
 * The rgb pointer is valid only for the duration of the call.
 * The implementation must be thread-safe and return quickly
 * (e.g. copy data and signal a worker thread).
 */
typedef void (*VideoFrameCallback)(const uint8_t *rgb, int w, int h);

/* Register a callback for decoded frames (pass NULL to unregister). */
void video_set_frame_callback(VideoFrameCallback cb);

/* Launch the background video-receiver / decoder thread. */
void video_start(void);

/* Signal the thread to stop and join it. */
void video_stop(void);

/*
 * Acquire the latest decoded frame for reading.
 * Returns true and fills *rgb, *w, *h if a new frame is available.
 * The frame pointer is valid until video_unlock_frame() is called.
 * Returns false (no lock taken) if no new frame is pending.
 */
bool video_lock_frame(const uint8_t **rgb, int *w, int *h);

/* Release the frame lock and mark the frame as consumed. */
void video_unlock_frame(void);
