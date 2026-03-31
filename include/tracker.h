#pragma once

#include <stdint.h>
#include <stdbool.h>

/* One normalised landmark (coordinates in [0,1] relative to frame). */
typedef struct
{
    float x, y, z;
} Landmark;

/* 21 MediaPipe hand landmarks. valid == false when no hand detected. */
typedef struct
{
    Landmark pts[21];
    bool valid;
} HandLandmarks;

/*
 * RC override produced by a tracking algorithm.
 * When active is true the main loop uses these values instead of
 * manual input for roll/pitch/throttle/yaw.
 */
typedef struct
{
    int roll, pitch, throttle, yaw; /* -100 .. 100 */
    bool active;
} TrackerOutput;

/* Called once from main() before the main loop starts. */
void tracker_init(void);

/*
 * Called from the video thread on every decoded frame (RGB24, row-major).
 * Must be thread-safe and return quickly.
 */
void tracker_process_frame(const uint8_t *rgb, int w, int h);

/* Enable / disable tracking mode. When disabled, TrackerOutput.active is
 * always false and no RC overrides are generated. */
void tracker_set_enabled(bool enabled);
bool tracker_is_enabled(void);

/* Thread-safe snapshot of the latest RC override. */
TrackerOutput tracker_get_output(void);

/* Thread-safe snapshot of the latest detected landmarks (for HUD). */
HandLandmarks tracker_get_landmarks(void);

/* Debug info updated every frame - useful for diagnosing detection failures. */
typedef struct
{
    float palm_score;    /* confidence of winning detection (0 = none passed threshold) */
    float palm_best_raw; /* unconditional best sigmoid score this frame */
    float hand_presence; /* 0..1, landmark model hand-presence score */
    int gesture;         /* GestureID of current detected gesture */
    int lost_frames;     /* consecutive frames with no palm */
    float inference_ms;  /* wall-clock time for the inference pipeline this frame */
    /* palm bounding box in normalised [0,1] frame coords (valid when palm_score>0) */
    float box_cx, box_cy, box_w, box_h;
} TrackerDebug;
TrackerDebug tracker_get_debug(void);

/* Called once from main() during shutdown. */
void tracker_cleanup(void);
