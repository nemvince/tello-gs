/*
 * tracker.c — hand-tracking inference worker using onnxruntime C API.
 *
 * Pipeline (runs in a dedicated worker thread):
 *   1. Palm detector  (192×192 NCHW → SSD anchor decode → best box)
 *   2. Landmark model (224×224 NCHW crop → 21 3-D landmarks)
 *   3. Read-only gesture identification (never sends drone commands)
 *   4. PD controller  → TrackerOutput stored atomically
 *
 * SAFETY: This module NEVER calls drone_send(). All drone commands
 * originate exclusively from main.c user input handlers.
 *
 * Threading contract
 *   tracker_process_frame()  — video thread: copies frame, signals worker.
 *   tracker_get_output()     — main thread:  reads latest TrackerOutput.
 *   tracker_get_landmarks()  — main thread:  reads latest HandLandmarks (HUD).
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "tracker.h"
#include "gesture.h"
#include "config.h"
#include "settings.h"
#include "app.h"

#include <onnxruntime_c_api.h>

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ================================================================
 *  ONNX Runtime globals
 * ================================================================ */

static const OrtApi *ort = NULL;
static OrtEnv *ort_env = NULL;
static OrtSession *palm_session = NULL;
static OrtSession *lmark_session = NULL;
static OrtMemoryInfo *mem_info = NULL;

/* ================================================================
 *  Frame double-buffer (video thread → worker thread)
 * ================================================================ */

typedef struct
{
    uint8_t *data;
    int w, h;
} Frame;

static Frame frames[2];
static int write_idx = 0;
static pthread_mutex_t frame_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t frame_cv = PTHREAD_COND_INITIALIZER;
static bool frame_pending = false;

/* ================================================================
 *  Shared output (worker writes, main/HUD reads)
 * ================================================================ */

static pthread_mutex_t out_mtx = PTHREAD_MUTEX_INITIALIZER;
static TrackerOutput g_output = {0, 0, 0, 0, false};
static HandLandmarks g_landmarks = {0};
static TrackerDebug g_debug = {0};

/* ================================================================
 *  Enable flag — only affects PD controller output, NOT inference
 * ================================================================ */

static volatile bool g_enabled = false;

/* Worker thread handle */
static pthread_t worker_tid;

/* Previous-frame landmark state for re-tracking (avoids palm detector) */
static HandLandmarks prev_lm = {0};

/* ================================================================
 *  ORT helpers
 * ================================================================ */

#define ORT_CHECK(expr)                                                   \
    do                                                                    \
    {                                                                     \
        OrtStatus *_s = (expr);                                           \
        if (_s)                                                           \
        {                                                                 \
            fprintf(stderr, "ORT error: %s\n", ort->GetErrorMessage(_s)); \
            ort->ReleaseStatus(_s);                                       \
        }                                                                 \
    } while (0)

static OrtSession *load_session(const char *model_path)
{
    OrtSessionOptions *opts = NULL;
    ORT_CHECK(ort->CreateSessionOptions(&opts));
    ORT_CHECK(ort->SetIntraOpNumThreads(opts, 2));
    ORT_CHECK(ort->SetSessionGraphOptimizationLevel(opts, ORT_ENABLE_ALL));

    OrtSession *session = NULL;
    OrtStatus *s = ort->CreateSession(ort_env, model_path, opts, &session);
    ort->ReleaseSessionOptions(opts);
    if (s)
    {
        fprintf(stderr, "Failed to load model '%s': %s\n",
                model_path, ort->GetErrorMessage(s));
        ort->ReleaseStatus(s);
        return NULL;
    }
    return session;
}

/* ================================================================
 *  Image preprocessing
 *
 *  All functions produce NCHW float32 output normalised to [0, 1].
 *  Bilinear interpolation matches the Python reference:
 *    src_coord = (dst_idx + 0.5) * (src_size / dst_size) - 0.5
 * ================================================================ */

/*
 * resize_normalise — resize full RGB24 frame into NCHW float buffer.
 *
 * src: RGB24 row-major, sw×sh
 * dst: float[3 * dh * dw], NCHW layout, values in [0, 1]
 */
static void resize_normalise(const uint8_t *src, int sw, int sh,
                             float *dst, int dw, int dh)
{
    const float scale_x = (float)sw / dw;
    const float scale_y = (float)sh / dh;

    for (int y = 0; y < dh; y++)
    {
        float fy = (y + 0.5f) * scale_y - 0.5f;
        int y0 = (int)floorf(fy);
        int y1 = y0 + 1;
        float dy = fy - y0;
        if (y0 < 0)
        {
            y0 = 0;
            dy = 0.0f;
        }
        if (y1 >= sh)
            y1 = sh - 1;

        for (int x = 0; x < dw; x++)
        {
            float fx = (x + 0.5f) * scale_x - 0.5f;
            int x0 = (int)floorf(fx);
            int x1 = x0 + 1;
            float dx = fx - x0;
            if (x0 < 0)
            {
                x0 = 0;
                dx = 0.0f;
            }
            if (x1 >= sw)
                x1 = sw - 1;

            const uint8_t *p00 = src + (y0 * sw + x0) * 3;
            const uint8_t *p01 = src + (y0 * sw + x1) * 3;
            const uint8_t *p10 = src + (y1 * sw + x0) * 3;
            const uint8_t *p11 = src + (y1 * sw + x1) * 3;

            float w00 = (1.0f - dx) * (1.0f - dy);
            float w01 = dx * (1.0f - dy);
            float w10 = (1.0f - dx) * dy;
            float w11 = dx * dy;

            for (int c = 0; c < 3; c++)
            {
                float v = p00[c] * w00 + p01[c] * w01 + p10[c] * w10 + p11[c] * w11;
                /* NCHW: channel-plane first, normalised to [0, 1] */
                dst[c * dh * dw + y * dw + x] = v / 255.0f;
            }
        }
    }
}

/*
 * crop_resize_normalise — extract a crop from the frame and resize to NCHW.
 *
 * The crop box is specified in PIXEL coordinates (cx, cy, cw, ch) where
 * (cx, cy) is the centre. The crop is clamped to frame bounds, then the
 * CLAMPED region is resized to dw×dh NCHW float.
 */
static void crop_resize_normalise(const uint8_t *src, int sw, int sh,
                                  float *dst, int dw, int dh,
                                  float cx, float cy, float cw, float ch)
{
    int x0 = (int)(cx - cw * 0.5f);
    int y0 = (int)(cy - ch * 0.5f);
    int x1 = (int)(cx + cw * 0.5f);
    int y1 = (int)(cy + ch * 0.5f);
    if (x0 < 0)
        x0 = 0;
    if (y0 < 0)
        y0 = 0;
    if (x1 > sw)
        x1 = sw;
    if (y1 > sh)
        y1 = sh;

    int rw = x1 - x0, rh = y1 - y0;
    if (rw <= 0 || rh <= 0)
    {
        memset(dst, 0, (size_t)(dw * dh * 3) * sizeof(float));
        return;
    }

    /* Copy crop rows into a contiguous RGB24 buffer */
    uint8_t *crop = malloc((size_t)(rw * rh * 3));
    if (!crop)
    {
        memset(dst, 0, (size_t)(dw * dh * 3) * sizeof(float));
        return;
    }
    for (int row = 0; row < rh; row++)
        memcpy(crop + row * rw * 3,
               src + ((y0 + row) * sw + x0) * 3,
               (size_t)(rw * 3));

    resize_normalise(crop, rw, rh, dst, dw, dh);
    free(crop);
}

/* ================================================================
 *  SSD Anchor generation — MediaPipe BlazePalm 192×192
 *
 *  MediaPipe's SsdAnchorsCalculator config for palm_detection_lite:
 *    num_layers=4, strides=[8, 16, 16, 16]
 *    input_size=192, anchor_offset=0.5, fixed_anchor_size=true
 *
 *  This produces:
 *    stride  8 → grid 24×24, 2 anchors/cell = 1152
 *    stride 16 → grid 12×12, 6 anchors/cell =  864  (3 layers × 2 each)
 *    Total: 2016 anchors
 *
 *  Each anchor stores (cx, cy) in normalised [0,1] coords.
 *  IMPORTANT: The loops go y-outer, x-inner to match TFLite's
 *  row-major feature-map traversal order.
 * ================================================================ */

#define N_ANCHORS 2016

static float palm_anchors[N_ANCHORS][2]; /* [i][0]=cx, [i][1]=cy */

static void generate_palm_anchors(void)
{
    int idx = 0;

    /* stride=8 → 24×24, 2 anchors per cell */
    for (int y = 0; y < 24; y++)
        for (int x = 0; x < 24; x++)
            for (int a = 0; a < 2; a++, idx++)
            {
                palm_anchors[idx][0] = (x + 0.5f) / 24.0f; /* cx */
                palm_anchors[idx][1] = (y + 0.5f) / 24.0f; /* cy */
            }

    /* stride=16 → 12×12, 6 anchors per cell (3 layers × 2) */
    for (int y = 0; y < 12; y++)
        for (int x = 0; x < 12; x++)
            for (int a = 0; a < 6; a++, idx++)
            {
                palm_anchors[idx][0] = (x + 0.5f) / 12.0f;
                palm_anchors[idx][1] = (y + 0.5f) / 12.0f;
            }

    /* Verify count */
    if (idx != N_ANCHORS)
        fprintf(stderr, "[tracker] BUG: generated %d anchors, expected %d\n",
                idx, N_ANCHORS);
}

/* ================================================================
 *  Palm detection
 *
 *  Model: palm_detection_lite.onnx
 *    Input:  "input"          [1, 3, 192, 192]  NCHW float32 [-1,1]
 *    Output: "regressors"     [1, 2016, 18]
 *    Output: "classificators" [1, 2016, 1]       raw logits
 *
 *  Regressor layout per anchor (MediaPipe TensorsToDetections):
 *    [0] = y_center offset (pixels in 192-space)
 *    [1] = x_center offset
 *    [2] = h
 *    [3] = w
 *    [4..17] = 7 keypoints × (x, y)  — also in 192-pixel space
 *
 *  Decoding to normalised [0,1]:
 *    decoded_cx = anchor_cx + reg[1] / 192
 *    decoded_cy = anchor_cy + reg[0] / 192
 *    decoded_w  = reg[3] / 192
 *    decoded_h  = reg[2] / 192
 * ================================================================ */

typedef struct
{
    float cx, cy, w, h; /* normalised [0,1] frame coordinates */
    float score;        /* sigmoid confidence, 0 if no detection */
} PalmBox;

#define PALM_SCORE_THRESHOLD 0.5f
#define PALM_NMS_IOU_THRESH 0.3f

static float box_iou(float ax, float ay, float aw, float ah,
                     float bx, float by, float bw, float bh)
{
    float a_x0 = ax - aw * 0.5f, a_y0 = ay - ah * 0.5f;
    float a_x1 = ax + aw * 0.5f, a_y1 = ay + ah * 0.5f;
    float b_x0 = bx - bw * 0.5f, b_y0 = by - bh * 0.5f;
    float b_x1 = bx + bw * 0.5f, b_y1 = by + bh * 0.5f;

    float ix0 = fmaxf(a_x0, b_x0), iy0 = fmaxf(a_y0, b_y0);
    float ix1 = fminf(a_x1, b_x1), iy1 = fminf(a_y1, b_y1);
    float iw = fmaxf(0.0f, ix1 - ix0);
    float ih = fmaxf(0.0f, iy1 - iy0);
    float inter = iw * ih;
    float area_a = aw * ah, area_b = bw * bh;
    float uni = area_a + area_b - inter;
    return (uni > 0.0f) ? inter / uni : 0.0f;
}

/*
 * Run palm detector. Returns the best NMS-surviving detection,
 * or a PalmBox with score==0 if nothing found.
 * The returned box coordinates are normalised [0,1].
 */
static PalmBox run_palm_detector(const uint8_t *rgb, int fw, int fh,
                                 float *input_buf)
{
    PalmBox result = {0};
    if (!palm_session)
        return result;

    /* Preprocess: resize full frame → 192×192 NCHW [-1,1] */
    resize_normalise(rgb, fw, fh, input_buf, PALM_INPUT_W, PALM_INPUT_H);

    /* Create input tensor */
    int64_t input_shape[] = {1, 3, PALM_INPUT_H, PALM_INPUT_W};
    OrtValue *input_tensor = NULL;
    OrtStatus *s = ort->CreateTensorWithDataAsOrtValue(
        mem_info, input_buf,
        (size_t)(PALM_INPUT_W * PALM_INPUT_H * 3) * sizeof(float),
        input_shape, 4, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &input_tensor);
    if (s)
    {
        ort->ReleaseStatus(s);
        return result;
    }

    /* Run inference */
    const char *in_names[] = {"input"};
    const char *out_names[] = {"regressors", "classificators"};
    OrtValue *outputs[2] = {NULL, NULL};

    s = ort->Run(palm_session, NULL,
                 in_names, (const OrtValue *const *)&input_tensor, 1,
                 out_names, 2, outputs);
    ort->ReleaseValue(input_tensor);
    if (s)
    {
        ort->ReleaseStatus(s);
        return result;
    }

    /* Get raw output pointers: regressors [1,2016,18], classificators [1,2016,1] */
    float *regressors = NULL, *classificators = NULL;
    ORT_CHECK(ort->GetTensorMutableData(outputs[0], (void **)&regressors));
    ORT_CHECK(ort->GetTensorMutableData(outputs[1], (void **)&classificators));

    /*
     * Phase 1: Collect all detections above threshold.
     * Apply sigmoid to raw logits, decode box from anchor offsets.
     */
    typedef struct
    {
        float cx, cy, w, h, score;
    } Det;
    Det dets[64]; /* more than enough for a single hand */
    int n_dets = 0;

    float best_raw = 0.0f; /* unconditional best for debug */

    for (int i = 0; i < N_ANCHORS; i++)
    {
        float sc = 1.0f / (1.0f + expf(-classificators[i]));
        if (sc > best_raw)
            best_raw = sc;

        if (sc < PALM_SCORE_THRESHOLD)
            continue;
        if (n_dets >= 64)
            break;

        float *b = regressors + i * 18;
        float cx = palm_anchors[i][0] + b[1] / (float)PALM_INPUT_W;
        float cy = palm_anchors[i][1] + b[0] / (float)PALM_INPUT_H;
        float w = fabsf(b[3]) / (float)PALM_INPUT_W;
        float h = fabsf(b[2]) / (float)PALM_INPUT_H;

        dets[n_dets++] = (Det){cx, cy, w, h, sc};
    }

    /* Publish unconditional best score to debug */
    pthread_mutex_lock(&out_mtx);
    g_debug.palm_best_raw = best_raw;
    pthread_mutex_unlock(&out_mtx);

    /* Phase 2: Greedy NMS — sort by score descending, suppress overlaps */
    /* Simple insertion sort (n_dets is small) */
    for (int i = 1; i < n_dets; i++)
    {
        Det key = dets[i];
        int j = i - 1;
        while (j >= 0 && dets[j].score < key.score)
        {
            dets[j + 1] = dets[j];
            j--;
        }
        dets[j + 1] = key;
    }

    bool suppressed[64] = {false};
    for (int i = 0; i < n_dets; i++)
    {
        if (suppressed[i])
            continue;
        for (int j = i + 1; j < n_dets; j++)
        {
            if (suppressed[j])
                continue;
            float iou = box_iou(dets[i].cx, dets[i].cy, dets[i].w, dets[i].h,
                                dets[j].cx, dets[j].cy, dets[j].w, dets[j].h);
            if (iou > PALM_NMS_IOU_THRESH)
                suppressed[j] = true;
        }
    }

    /* Take the top surviving detection */
    for (int i = 0; i < n_dets; i++)
    {
        if (!suppressed[i])
        {
            result.cx = dets[i].cx;
            result.cy = dets[i].cy;
            result.w = dets[i].w;
            result.h = dets[i].h;
            result.score = dets[i].score;
            break;
        }
    }

    /* Store normalised box in debug */
    if (result.score > 0.0f)
    {
        pthread_mutex_lock(&out_mtx);
        g_debug.box_cx = result.cx;
        g_debug.box_cy = result.cy;
        g_debug.box_w = result.w;
        g_debug.box_h = result.h;
        pthread_mutex_unlock(&out_mtx);
    }

    ort->ReleaseValue(outputs[0]);
    ort->ReleaseValue(outputs[1]);
    return result;
}

/* ================================================================
 *  Landmark inference
 *
 *  Model: hand_landmark_lite.onnx
 *    Input:  "input"      [1, 3, 224, 224]  NCHW float32 [-1,1]
 *    Output: "Identity"   [1, 63]  = 21 landmarks × (x, y, z)
 *    Output: "Identity_1" [1, 1]   = hand presence logit
 *
 *  Landmark coordinates are in 224-pixel crop space.
 *  Projection to normalised frame coords:
 *    frame_x = (crop_x0 + lm_x / 224 * crop_w) / frame_w
 *    frame_y = (crop_y0 + lm_y / 224 * crop_h) / frame_h
 *
 *  The crop box passed in is in NORMALISED coords [0,1].
 *  We expand it by CROP_SCALE for stability before cropping.
 * ================================================================ */

#define CROP_SCALE 1.5f /* expand palm box for landmark crop */
#define PRESENCE_THRESH 0.5f
#define RETRACK_PRESENCE_THRESH 0.8f /* stricter when re-tracking to avoid ghost locks */

static bool run_landmark_model(const uint8_t *rgb, int fw, int fh,
                               const PalmBox *palm,
                               float *input_buf,
                               HandLandmarks *out_lm,
                               float thresh)
{
    out_lm->valid = false;
    if (!lmark_session)
        return false;

    /* Compute crop in pixel coordinates from normalised palm box.
     * Use a square crop (max of w,h) expanded by CROP_SCALE. */
    float side = fmaxf(palm->w, palm->h) * CROP_SCALE;
    float crop_cx_px = palm->cx * fw;
    float crop_cy_px = palm->cy * fh;
    float crop_w_px = side * fw;
    float crop_h_px = side * fh;

    crop_resize_normalise(rgb, fw, fh, input_buf,
                          LMARK_INPUT_W, LMARK_INPUT_H,
                          crop_cx_px, crop_cy_px, crop_w_px, crop_h_px);

    /* Create tensor */
    int64_t shape[] = {1, 3, LMARK_INPUT_H, LMARK_INPUT_W};
    OrtValue *input_tensor = NULL;
    OrtStatus *s = ort->CreateTensorWithDataAsOrtValue(
        mem_info, input_buf,
        (size_t)(LMARK_INPUT_W * LMARK_INPUT_H * 3) * sizeof(float),
        shape, 4, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &input_tensor);
    if (s)
    {
        ort->ReleaseStatus(s);
        return false;
    }

    /* Run */
    const char *in_names[] = {"input"};
    const char *out_names[] = {"Identity", "Identity_1"};
    OrtValue *outputs[2] = {NULL, NULL};

    s = ort->Run(lmark_session, NULL,
                 in_names, (const OrtValue *const *)&input_tensor, 1,
                 out_names, 2, outputs);
    ort->ReleaseValue(input_tensor);
    if (s)
    {
        ort->ReleaseStatus(s);
        return false;
    }

    /* Extract output data */
    float *lm_raw = NULL;   /* [1, 63] = 21 × (x, y, z) in 224-pixel space */
    float *presence = NULL; /* [1, 1]  = hand presence logit */
    ORT_CHECK(ort->GetTensorMutableData(outputs[0], (void **)&lm_raw));
    ORT_CHECK(ort->GetTensorMutableData(outputs[1], (void **)&presence));

    float hand_presence = 1.0f / (1.0f + expf(-presence[0]));

    /* Publish presence to debug */
    pthread_mutex_lock(&out_mtx);
    g_debug.hand_presence = hand_presence;
    pthread_mutex_unlock(&out_mtx);

    if (hand_presence >= thresh)
    {
        /*
         * Project landmarks from 224-pixel crop space to normalised frame coords.
         *
         * The crop_resize_normalise function clamped the crop to frame bounds.
         * We need the ACTUAL crop top-left in pixels (after clamping) to project correctly.
         */
        int crop_x0 = (int)(crop_cx_px - crop_w_px * 0.5f);
        int crop_y0 = (int)(crop_cy_px - crop_h_px * 0.5f);
        if (crop_x0 < 0)
            crop_x0 = 0;
        if (crop_y0 < 0)
            crop_y0 = 0;

        int crop_x1 = (int)(crop_cx_px + crop_w_px * 0.5f);
        int crop_y1 = (int)(crop_cy_px + crop_h_px * 0.5f);
        if (crop_x1 > fw)
            crop_x1 = fw;
        if (crop_y1 > fh)
            crop_y1 = fh;

        float actual_cw = (float)(crop_x1 - crop_x0);
        float actual_ch = (float)(crop_y1 - crop_y0);

        for (int i = 0; i < 21; i++)
        {
            float lx = lm_raw[i * 3 + 0]; /* x in 224-space */
            float ly = lm_raw[i * 3 + 1]; /* y in 224-space */
            float lz = lm_raw[i * 3 + 2];

            /* Map from 224-pixel crop to pixel-space frame coords */
            float frame_px_x = crop_x0 + (lx / (float)LMARK_INPUT_W) * actual_cw;
            float frame_px_y = crop_y0 + (ly / (float)LMARK_INPUT_H) * actual_ch;

            /* Normalise to [0,1] */
            out_lm->pts[i].x = frame_px_x / (float)fw;
            out_lm->pts[i].y = frame_px_y / (float)fh;
            out_lm->pts[i].z = lz;
        }
        out_lm->valid = true;
    }

    ort->ReleaseValue(outputs[0]);
    ort->ReleaseValue(outputs[1]);
    return out_lm->valid;
}

/* ================================================================
 *  PD controller
 * ================================================================ */

typedef struct
{
    float prev_err;
} PD;
static PD pd_roll = {0}, pd_throttle = {0}, pd_pitch = {0};

static int pd_update(PD *pd, float err, float kp, float kd)
{
    float deriv = err - pd->prev_err;
    pd->prev_err = err;
    int v = (int)(kp * err + kd * deriv);
    if (v > 100)
        v = 100;
    if (v < -100)
        v = -100;
    return v;
}

/* ================================================================
 *  Landmark-based re-tracking
 *
 *  When we already have valid landmarks from the previous frame,
 *  compute a bounding box from them and feed that directly to the
 *  landmark model — skipping the expensive palm detector entirely.
 *  This is how the real MediaPipe pipeline works and gives much
 *  more stable, lower-latency tracking.
 * ================================================================ */

#define RETRACK_PAD 1.8f /* extra padding around landmark bbox for re-crop */

/*
 * Compute a PalmBox from existing landmarks (normalised [0,1] coords).
 * Returns a box that encloses all landmarks with RETRACK_PAD expansion.
 */
static PalmBox palmbox_from_landmarks(const HandLandmarks *lm)
{
    float min_x = 1.0f, max_x = 0.0f;
    float min_y = 1.0f, max_y = 0.0f;
    for (int i = 0; i < 21; i++)
    {
        if (lm->pts[i].x < min_x)
            min_x = lm->pts[i].x;
        if (lm->pts[i].x > max_x)
            max_x = lm->pts[i].x;
        if (lm->pts[i].y < min_y)
            min_y = lm->pts[i].y;
        if (lm->pts[i].y > max_y)
            max_y = lm->pts[i].y;
    }
    float w = max_x - min_x;
    float h = max_y - min_y;
    float side = fmaxf(w, h) * RETRACK_PAD;
    PalmBox box;
    box.cx = (min_x + max_x) * 0.5f;
    box.cy = (min_y + max_y) * 0.5f;
    box.w = side;
    box.h = side;
    box.score = 1.0f; /* synthetic, not from detector */
    return box;
}

/* ================================================================
 *  EMA smoothing for landmarks
 *
 *  Exponential moving average reduces jitter between frames while
 *  keeping tracking responsive. Alpha=1 means no smoothing (raw),
 *  Alpha=0.5 means equal weight to new and previous.
 * ================================================================ */

#define LM_SMOOTH_ALPHA 0.8f /* higher = more responsive, lower = smoother */

static HandLandmarks smooth_lm = {0};

static void ema_smooth_landmarks(HandLandmarks *lm)
{
    if (!smooth_lm.valid)
    {
        /* First valid frame — initialise */
        smooth_lm = *lm;
        return;
    }
    for (int i = 0; i < 21; i++)
    {
        lm->pts[i].x = LM_SMOOTH_ALPHA * lm->pts[i].x +
                       (1.0f - LM_SMOOTH_ALPHA) * smooth_lm.pts[i].x;
        lm->pts[i].y = LM_SMOOTH_ALPHA * lm->pts[i].y +
                       (1.0f - LM_SMOOTH_ALPHA) * smooth_lm.pts[i].y;
        lm->pts[i].z = LM_SMOOTH_ALPHA * lm->pts[i].z +
                       (1.0f - LM_SMOOTH_ALPHA) * smooth_lm.pts[i].z;
    }
    smooth_lm = *lm;
}

/* ================================================================
 *  Worker thread
 * ================================================================ */

static void *worker_thread(void *arg)
{
    (void)arg;

    float *palm_buf = malloc((size_t)(PALM_INPUT_W * PALM_INPUT_H * 3) * sizeof(float));
    float *lmark_buf = malloc((size_t)(LMARK_INPUT_W * LMARK_INPUT_H * 3) * sizeof(float));
    if (!palm_buf || !lmark_buf)
    {
        fprintf(stderr, "[tracker] out of memory for inference buffers\n");
        free(palm_buf);
        free(lmark_buf);
        return NULL;
    }

    int lost_frames = 0;

    while (g_running)
    {
        /* Wait for a new frame from the video thread */
        pthread_mutex_lock(&frame_mtx);
        while (!frame_pending && g_running)
            pthread_cond_wait(&frame_cv, &frame_mtx);
        if (!g_running)
        {
            pthread_mutex_unlock(&frame_mtx);
            break;
        }
        int read_idx = 1 - write_idx;
        frame_pending = false;
        pthread_mutex_unlock(&frame_mtx);

        Frame *f = &frames[read_idx];
        if (!f->data)
            continue;

        struct timespec t_start;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        /*
         * Re-tracking strategy (matches real MediaPipe pipeline):
         *
         * If we have valid landmarks from the previous frame, compute
         * a crop from those landmarks and run ONLY the landmark model.
         * This is faster and much more stable than re-running the palm
         * detector every frame.
         *
         * Fall back to the palm detector only when:
         *   - No previous landmarks (first detection)
         *   - Landmark model says "no hand" (presence below threshold)
         *   - Lost for too many consecutive frames
         */
        PalmBox palm = {0};
        bool used_retrack = false;

        if (prev_lm.valid)
        {
            /* Re-track from previous landmarks */
            palm = palmbox_from_landmarks(&prev_lm);
            used_retrack = true;
        }
        else
        {
            /* No previous landmarks — need palm detector */
            palm = run_palm_detector(f->data, f->w, f->h, palm_buf);
        }

        if (palm.score == 0.0f)
        {
            lost_frames++;
            pthread_mutex_lock(&out_mtx);
            g_debug.palm_score = 0.0f;
            g_debug.hand_presence = 0.0f;
            g_debug.gesture = GESTURE_NONE;
            g_debug.lost_frames = lost_frames;
            if (lost_frames >= g_settings.lost_frames)
            {
                g_output = (TrackerOutput){0, 0, 0, 0, false};
                memset(&g_landmarks, 0, sizeof(g_landmarks));
            }
            pthread_mutex_unlock(&out_mtx);
            if (lost_frames >= g_settings.lost_frames)
            {
                pd_roll = pd_throttle = pd_pitch = (PD){0};
                prev_lm.valid = false;
                smooth_lm.valid = false;
            }
            continue;
        }

        /* ---- Stage 2: Landmark model ---- */
        HandLandmarks lm = {0};
        bool lm_ok = run_landmark_model(f->data, f->w, f->h,
                                        &palm, lmark_buf, &lm,
                                        used_retrack ? RETRACK_PRESENCE_THRESH
                                                     : PRESENCE_THRESH);

        if (!lm_ok && used_retrack)
        {
            /*
             * Re-track failed — the hand may have moved outside the
             * predicted crop. Fall back to palm detector for this frame.
             */
            palm = run_palm_detector(f->data, f->w, f->h, palm_buf);
            if (palm.score > 0.0f)
            {
                lm_ok = run_landmark_model(f->data, f->w, f->h,
                                           &palm, lmark_buf, &lm,
                                           PRESENCE_THRESH);
            }
        }

        if (!lm_ok)
        {
            lost_frames++;
            prev_lm.valid = false;
            pthread_mutex_lock(&out_mtx);
            g_debug.palm_score = palm.score;
            g_debug.hand_presence = g_debug.hand_presence;
            g_debug.gesture = GESTURE_NONE;
            g_debug.lost_frames = lost_frames;
            if (lost_frames >= g_settings.lost_frames)
            {
                g_output = (TrackerOutput){0, 0, 0, 0, false};
                memset(&g_landmarks, 0, sizeof(g_landmarks));
                smooth_lm.valid = false;
            }
            pthread_mutex_unlock(&out_mtx);
            if (lost_frames >= g_settings.lost_frames)
                pd_roll = pd_throttle = pd_pitch = (PD){0};
            continue;
        }

        lost_frames = 0;

        /* Save raw landmarks for next-frame re-tracking (before smoothing) */
        prev_lm = lm;

        /* ---- EMA smoothing ---- */
        ema_smooth_landmarks(&lm);

        /* ---- Stage 3: Gesture identification (READ-ONLY, no drone commands) ---- */
        GestureID gesture = gesture_identify(&lm);

        /* ---- Stage 4: PD controller (only when tracking enabled) ---- */
        TrackerOutput out = {0, 0, 0, 0, false};

        if (g_enabled && lm_ok &&
            (gesture == GESTURE_OPEN_PALM || gesture == GESTURE_NONE))
        {
            /* Palm centroid from all 21 landmarks */
            float pcx = 0.0f, pcy = 0.0f;
            for (int i = 0; i < 21; i++)
            {
                pcx += lm.pts[i].x;
                pcy += lm.pts[i].y;
            }
            pcx /= 21.0f;
            pcy /= 21.0f;

/* Track lateral (roll), vertical (throttle), and distance (pitch).
 *
 * Distance keeping uses palm size (wrist→middle-MCP distance) as
 * a proxy for depth. The gains are much lower than lateral/vertical
 * so forward/backward motion is slow and cautious.
 *
 * If the centroid is near a frame edge, zero that axis so the
 * drone doesn't keep flying in that direction after the hand
 * leaves the frame. */
#define EDGE_MARGIN 0.05f
#define PITCH_DEADZONE 0.03f /* ignore small palm-size fluctuations */
#define PITCH_MAX 30         /* cap forward/backward speed */
            float err_roll = (pcx - 0.5f);
            float err_throttle = -(pcy - 0.5f);

            if (pcx < EDGE_MARGIN || pcx > (1.0f - EDGE_MARGIN))
            {
                err_roll = 0.0f;
                pd_roll = (PD){0};
            }
            if (pcy < EDGE_MARGIN || pcy > (1.0f - EDGE_MARGIN))
            {
                err_throttle = 0.0f;
                pd_throttle = (PD){0};
            }

            /* Palm size: wrist-to-middle_MCP distance (normalised) */
            float dx = lm.pts[9].x - lm.pts[0].x;
            float dy = lm.pts[9].y - lm.pts[0].y;
            float palm_size = sqrtf(dx * dx + dy * dy);
            float err_pitch = (g_settings.target_palm_size - palm_size);

            /* Deadzone — don't chase tiny size changes */
            if (fabsf(err_pitch) < PITCH_DEADZONE)
            {
                err_pitch = 0.0f;
                pd_pitch = (PD){0};
            }

            out.roll = pd_update(&pd_roll, err_roll, g_settings.gain_roll, g_settings.deriv_roll);
            out.throttle = pd_update(&pd_throttle, err_throttle, g_settings.gain_throttle, g_settings.deriv_throttle);
            int raw_pitch = pd_update(&pd_pitch, err_pitch, g_settings.gain_pitch, g_settings.deriv_pitch);
            /* Clamp pitch to a low max so forward/backward is cautious */
            if (raw_pitch > PITCH_MAX)
                raw_pitch = PITCH_MAX;
            if (raw_pitch < -PITCH_MAX)
                raw_pitch = -PITCH_MAX;
            out.pitch = raw_pitch;
            out.yaw = 0;
            out.active = true;
        }
        else
        {
            pd_roll = pd_throttle = pd_pitch = (PD){0};
        }

        /* ---- Publish results ---- */
        struct timespec t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        float inf_ms = (float)(t_end.tv_sec - t_start.tv_sec) * 1000.0f +
                       (float)(t_end.tv_nsec - t_start.tv_nsec) / 1e6f;

        pthread_mutex_lock(&out_mtx);
        g_output = out;
        g_landmarks = lm;
        g_debug.palm_score = palm.score;
        g_debug.hand_presence = g_debug.hand_presence; /* already set by run_landmark_model */
        g_debug.gesture = (int)gesture;
        g_debug.lost_frames = lost_frames;
        g_debug.inference_ms = inf_ms;
        pthread_mutex_unlock(&out_mtx);
    }

    free(palm_buf);
    free(lmark_buf);
    return NULL;
}

/* ================================================================
 *  Public API
 * ================================================================ */

void tracker_init(void)
{
    memset(&g_landmarks, 0, sizeof(g_landmarks));
    memset(&g_output, 0, sizeof(g_output));
    memset(&g_debug, 0, sizeof(g_debug));

    ort = OrtGetApiBase()->GetApi(ORT_API_VERSION);
    ORT_CHECK(ort->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "tello_gs", &ort_env));
    ORT_CHECK(ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &mem_info));

    palm_session = load_session(PALM_MODEL);
    lmark_session = load_session(LMARK_MODEL);

    generate_palm_anchors();
    fprintf(stderr, "[tracker] %d SSD anchors generated\n", N_ANCHORS);

    if (!palm_session || !lmark_session)
        fprintf(stderr, "[tracker] model load failed — tracking disabled\n");

    frames[0] = (Frame){NULL, 0, 0};
    frames[1] = (Frame){NULL, 0, 0};

    pthread_create(&worker_tid, NULL, worker_thread, NULL);
}

void tracker_process_frame(const uint8_t *rgb, int w, int h)
{
    pthread_mutex_lock(&frame_mtx);
    Frame *f = &frames[write_idx];

    if (!f->data || f->w != w || f->h != h)
    {
        free(f->data);
        f->data = malloc((size_t)(w * h * 3));
        f->w = w;
        f->h = h;
    }
    if (f->data)
        memcpy(f->data, rgb, (size_t)(w * h * 3));

    write_idx = 1 - write_idx;
    frame_pending = true;
    pthread_cond_signal(&frame_cv);
    pthread_mutex_unlock(&frame_mtx);
}

void tracker_set_enabled(bool enabled)
{
    g_enabled = enabled;
    if (!enabled)
    {
        pthread_mutex_lock(&out_mtx);
        g_output = (TrackerOutput){0, 0, 0, 0, false};
        pthread_mutex_unlock(&out_mtx);
        pd_roll = pd_throttle = pd_pitch = (PD){0};
    }
}

bool tracker_is_enabled(void) { return g_enabled; }

TrackerOutput tracker_get_output(void)
{
    pthread_mutex_lock(&out_mtx);
    TrackerOutput o = g_output;
    pthread_mutex_unlock(&out_mtx);
    return o;
}

HandLandmarks tracker_get_landmarks(void)
{
    pthread_mutex_lock(&out_mtx);
    HandLandmarks lm = g_landmarks;
    pthread_mutex_unlock(&out_mtx);
    return lm;
}

TrackerDebug tracker_get_debug(void)
{
    pthread_mutex_lock(&out_mtx);
    TrackerDebug d = g_debug;
    pthread_mutex_unlock(&out_mtx);
    return d;
}

void tracker_cleanup(void)
{
    pthread_mutex_lock(&frame_mtx);
    frame_pending = true;
    pthread_cond_signal(&frame_cv);
    pthread_mutex_unlock(&frame_mtx);

    pthread_join(worker_tid, NULL);

    free(frames[0].data);
    free(frames[1].data);

    if (lmark_session)
        ort->ReleaseSession(lmark_session);
    if (palm_session)
        ort->ReleaseSession(palm_session);
    if (mem_info)
        ort->ReleaseMemoryInfo(mem_info);
    if (ort_env)
        ort->ReleaseEnv(ort_env);
}
