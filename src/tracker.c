/*
 * tracker.c — hand-tracking inference worker using onnxruntime C API.
 *
 * Pipeline (runs in a dedicated worker thread):
 *   1. Palm detector  (192×192 RGB → bounding-box NMS)
 *   2. Landmark model (224×224 RGB crop → 21 3-D landmarks)
 *   3. Gesture classification (gesture.h)
 *   4. PD controller  → TrackerOutput stored atomically
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
#include "app.h"

#include <onnxruntime_c_api.h>
#include <SDL2/SDL.h> /* SDL_GetTicks */

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- ORT globals ---- */

static const OrtApi *ort = NULL;
static OrtEnv *ort_env = NULL;
static OrtSession *palm_session = NULL;
static OrtSession *lmark_session = NULL;
static OrtMemoryInfo *mem_info = NULL;

/* ---- Frame double-buffer ---- */

typedef struct
{
    uint8_t *data;
    int w, h;
} Frame;

static Frame frames[2];   /* ping-pong buffer */
static int write_idx = 0; /* which slot video thread writes to */
static pthread_mutex_t frame_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t frame_cv = PTHREAD_COND_INITIALIZER;
static bool frame_pending = false;

/* ---- Shared output (worker writes, main thread reads) ---- */

static pthread_mutex_t out_mtx = PTHREAD_MUTEX_INITIALIZER;
static TrackerOutput g_output = {0, 0, 0, 0, false};
static HandLandmarks g_landmarks;

/* ---- Enable flag ---- */

static volatile bool g_enabled = false;

/* ---- Worker thread ---- */

static pthread_t worker_tid;

/* ---- ORT helpers ---- */

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

/* ---- Image preprocessing ---- */

/* Resize RGB24 src (sw×sh) into dst (dw×dh) using bilinear interpolation,
 * then normalise to [-1,1] float32. */
static void resize_normalise(const uint8_t *src, int sw, int sh,
                             float *dst, int dw, int dh)
{
    float sx = (float)sw / dw;
    float sy = (float)sh / dh;
    for (int y = 0; y < dh; y++)
    {
        for (int x = 0; x < dw; x++)
        {
            float fx = (x + 0.5f) * sx - 0.5f;
            float fy = (y + 0.5f) * sy - 0.5f;
            int x0 = (int)fx;
            if (x0 < 0)
                x0 = 0;
            int y0 = (int)fy;
            if (y0 < 0)
                y0 = 0;
            int x1 = x0 + 1;
            if (x1 >= sw)
                x1 = sw - 1;
            int y1 = y0 + 1;
            if (y1 >= sh)
                y1 = sh - 1;
            float dx = fx - x0, dy = fy - y0;
            for (int c = 0; c < 3; c++)
            {
                float v = (src[(y0 * sw + x0) * 3 + c] * (1 - dx) * (1 - dy) + src[(y0 * sw + x1) * 3 + c] * dx * (1 - dy) + src[(y1 * sw + x0) * 3 + c] * (1 - dx) * dy + src[(y1 * sw + x1) * 3 + c] * dx * dy);
                dst[c * dh * dw + y * dw + x] = v / 127.5f - 1.0f;
            }
        }
    }
}

/* Crop a ROI from src (sw×sh) and write resized float crop to dst (dw×dh).
 * cx,cy,cw,ch are the crop box in pixel coords (clamped). */
static void crop_resize_normalise(const uint8_t *src, int sw, int sh,
                                  float *dst, int dw, int dh,
                                  float cx, float cy, float cw, float ch)
{
    /* Convert to integer clip region */
    int x0 = (int)(cx - cw / 2);
    if (x0 < 0)
        x0 = 0;
    int y0 = (int)(cy - ch / 2);
    if (y0 < 0)
        y0 = 0;
    int x1 = (int)(cx + cw / 2);
    if (x1 > sw)
        x1 = sw;
    int y1 = (int)(cy + ch / 2);
    if (y1 > sh)
        y1 = sh;
    int rw = x1 - x0, rh = y1 - y0;
    if (rw <= 0 || rh <= 0)
    {
        memset(dst, 0, dw * dh * 3 * sizeof(float));
        return;
    }

    /* Allocate temp crop buffer and fill */
    uint8_t *crop = malloc((size_t)(rw * rh * 3));
    if (!crop)
    {
        memset(dst, 0, dw * dh * 3 * sizeof(float));
        return;
    }
    for (int row = 0; row < rh; row++)
        memcpy(crop + row * rw * 3, src + ((y0 + row) * sw + x0) * 3, (size_t)(rw * 3));
    resize_normalise(crop, rw, rh, dst, dw, dh);
    free(crop);
}

/* ---- Palm detection ---- */

typedef struct
{
    float cx, cy, w, h, score;
} PalmBox;

/* Run palm detector on frame. Returns best detection or score=0 on failure. */
static PalmBox run_palm_detector(const uint8_t *rgb, int fw, int fh,
                                 float *input_buf)
{
    PalmBox result = {0};
    if (!palm_session)
        return result;

    resize_normalise(rgb, fw, fh, input_buf,
                     PALM_INPUT_W, PALM_INPUT_H);

    int64_t input_shape[] = {1, 3, PALM_INPUT_H, PALM_INPUT_W};
    OrtValue *input_tensor = NULL;
    OrtStatus *s = ort->CreateTensorWithDataAsOrtValue(
        mem_info, input_buf,
        (size_t)(PALM_INPUT_W * PALM_INPUT_H * 3 * sizeof(float)),
        input_shape, 4, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &input_tensor);
    if (s)
    {
        ort->ReleaseStatus(s);
        return result;
    }

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

    /* boxes: [1, N, 18], scores: [1, N, 1] */
    float *boxes = NULL, *scores = NULL;
    OrtStatus *s2;
    s2 = ort->GetTensorMutableData(outputs[0], (void **)&boxes);
    if (s2)
        ort->ReleaseStatus(s2);
    s2 = ort->GetTensorMutableData(outputs[1], (void **)&scores);
    if (s2)
        ort->ReleaseStatus(s2);

    OrtTensorTypeAndShapeInfo *info = NULL;
    s2 = ort->GetTensorTypeAndShape(outputs[1], &info);
    if (s2)
        ort->ReleaseStatus(s2);
    size_t n_boxes = 0;
    s2 = ort->GetDimensionsCount(info, &n_boxes); /* misuse: get element count */
    if (s2)
        ort->ReleaseStatus(s2);
    {
        size_t elem_count = 0;
        s2 = ort->GetTensorShapeElementCount(info, &elem_count);
        if (s2)
            ort->ReleaseStatus(s2);
        n_boxes = elem_count;
    }
    ort->ReleaseTensorTypeAndShapeInfo(info);

    /* Sigmoid + pick best */
    float best_score = 0.5f; /* min confidence threshold */
    int best_i = -1;
    for (size_t i = 0; i < n_boxes; i++)
    {
        float sc = 1.0f / (1.0f + expf(-scores[i]));
        if (sc > best_score)
        {
            best_score = sc;
            best_i = (int)i;
        }
    }

    if (best_i >= 0)
    {
        /* boxes layout: [cy, cx, h, w, ...keypoints] in normalised [0,1] */
        float *b = boxes + best_i * 18;
        result.cy = b[0] * fh;
        result.cx = b[1] * fw;
        result.h = b[2] * fh;
        result.w = b[3] * fw;
        result.score = best_score;
        /* Expand box a little for a stable crop */
        result.w *= 1.4f;
        result.h *= 1.4f;
    }

    ort->ReleaseValue(outputs[0]);
    ort->ReleaseValue(outputs[1]);
    return result;
}

/* ---- Landmark inference ---- */

static bool run_landmark_model(const uint8_t *rgb, int fw, int fh,
                               const PalmBox *box,
                               float *input_buf,
                               HandLandmarks *out_lm)
{
    if (!lmark_session)
        return false;

    crop_resize_normalise(rgb, fw, fh, input_buf,
                          LMARK_INPUT_W, LMARK_INPUT_H,
                          box->cx, box->cy, box->w, box->h);

    int64_t shape[] = {1, 3, LMARK_INPUT_H, LMARK_INPUT_W};
    OrtValue *input_tensor = NULL;
    OrtStatus *s = ort->CreateTensorWithDataAsOrtValue(
        mem_info, input_buf,
        (size_t)(LMARK_INPUT_W * LMARK_INPUT_H * 3 * sizeof(float)),
        shape, 4, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &input_tensor);
    if (s)
    {
        ort->ReleaseStatus(s);
        return false;
    }

    const char *in_names[] = {"input"};
    const char *out_names[] = {"Identity", "Identity_1"}; /* landmarks, presence */
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

    /* landmarks: [1, 63] = 21 × xyz normalised to crop */
    float *lm_raw = NULL;
    float *presence = NULL;
    OrtStatus *s2;
    s2 = ort->GetTensorMutableData(outputs[0], (void **)&lm_raw);
    if (s2)
        ort->ReleaseStatus(s2);
    s2 = ort->GetTensorMutableData(outputs[1], (void **)&presence);
    if (s2)
        ort->ReleaseStatus(s2);

    float hand_presence = 1.0f / (1.0f + expf(-presence[0]));
    bool ok = hand_presence > 0.5f;

    if (ok)
    {
        /* Project normalised crop coords back to frame coords */
        float bx0 = box->cx - box->w / 2.0f;
        float by0 = box->cy - box->h / 2.0f;
        for (int i = 0; i < 21; i++)
        {
            out_lm->pts[i].x = (bx0 + lm_raw[i * 3 + 0] / LMARK_INPUT_W * box->w) / fw;
            out_lm->pts[i].y = (by0 + lm_raw[i * 3 + 1] / LMARK_INPUT_H * box->h) / fh;
            out_lm->pts[i].z = lm_raw[i * 3 + 2];
        }
        out_lm->valid = true;
    }
    else
    {
        out_lm->valid = false;
    }

    ort->ReleaseValue(outputs[0]);
    ort->ReleaseValue(outputs[1]);
    return ok;
}

/* ---- PD controller state ---- */

typedef struct
{
    float prev_err;
} PD;
static PD pd_roll = {0}, pd_throttle = {0}, pd_pitch = {0};

static int pd_update(PD *pd, float err, float kp, float kd)
{
    float out = kp * err + kd * (err - pd->prev_err);
    pd->prev_err = err;
    int v = (int)out;
    if (v > 100)
        v = 100;
    if (v < -100)
        v = -100;
    return v;
}

/* ---- Worker thread ---- */

static void *worker_thread(void *arg)
{
    (void)arg;

    /* Pre-allocate inference input buffers */
    float *palm_buf = malloc(PALM_INPUT_W * PALM_INPUT_H * 3 * sizeof(float));
    float *lmark_buf = malloc(LMARK_INPUT_W * LMARK_INPUT_H * 3 * sizeof(float));
    if (!palm_buf || !lmark_buf)
    {
        fprintf(stderr, "tracker: out of memory\n");
        free(palm_buf);
        free(lmark_buf);
        return NULL;
    }

    int lost_frames = 0;
    int read_idx = 0;

    while (g_running)
    {
        /* Wait for a new frame */
        pthread_mutex_lock(&frame_mtx);
        while (!frame_pending && g_running)
            pthread_cond_wait(&frame_cv, &frame_mtx);
        if (!g_running)
        {
            pthread_mutex_unlock(&frame_mtx);
            break;
        }

        read_idx = 1 - write_idx; /* consume the non-write slot */
        frame_pending = false;
        pthread_mutex_unlock(&frame_mtx);

        Frame *f = &frames[read_idx];
        if (!f->data || !g_enabled)
        {
            /* Tracking disabled: publish inactive output */
            pthread_mutex_lock(&out_mtx);
            g_output = (TrackerOutput){0, 0, 0, 0, false};
            memset(&g_landmarks, 0, sizeof(g_landmarks));
            pthread_mutex_unlock(&out_mtx);
            pd_roll = pd_throttle = pd_pitch = (PD){0};
            continue;
        }

        /* Stage 1: palm detection */
        PalmBox box = run_palm_detector(f->data, f->w, f->h, palm_buf);

        if (box.score == 0.0f)
        {
            lost_frames++;
            if (lost_frames >= TRACK_LOST_FRAMES)
            {
                pthread_mutex_lock(&out_mtx);
                g_output = (TrackerOutput){0, 0, 0, 0, false};
                memset(&g_landmarks, 0, sizeof(g_landmarks));
                pthread_mutex_unlock(&out_mtx);
                pd_roll = pd_throttle = pd_pitch = (PD){0};
            }
            continue;
        }
        lost_frames = 0;

        /* Stage 2: landmark model */
        HandLandmarks lm;
        memset(&lm, 0, sizeof(lm));
        bool ok = run_landmark_model(f->data, f->w, f->h, &box, lmark_buf, &lm);

        /* Stage 3: gesture classification */
        uint32_t now_ms = SDL_GetTicks();
        GestureID gesture = GESTURE_NONE;
        if (ok)
            gesture = gesture_classify(&lm, now_ms);

        /* Stage 4: PD controller — only output RC while open-palm */
        TrackerOutput out = {0, 0, 0, 0, false};
        if (ok && (gesture == GESTURE_OPEN_PALM || gesture == GESTURE_NONE))
        {
            /* Compute palm centroid from landmarks 0-4 (wrist + thumb) */
            float pcx = 0, pcy = 0;
            for (int i = 0; i <= 4; i++)
            {
                pcx += lm.pts[i].x;
                pcy += lm.pts[i].y;
            }
            pcx /= 5.0f;
            pcy /= 5.0f;

            /* Palm size: wrist to middle_MCP Euclidean distance (normalised) */
            float dx = lm.pts[9].x - lm.pts[0].x;
            float dy = lm.pts[9].y - lm.pts[0].y;
            float palm_size = sqrtf(dx * dx + dy * dy);

            float err_roll = (pcx - 0.5f);                          /* + = palm right  → roll right   */
            float err_throttle = -(pcy - 0.5f);                     /* + = palm up     → climb         */
            float err_pitch = (TRACK_TARGET_PALM_SIZE - palm_size); /* + = too far → advance */

            out.roll = pd_update(&pd_roll, err_roll, TRACK_GAIN_ROLL, TRACK_DERIV_ROLL);
            out.throttle = pd_update(&pd_throttle, err_throttle, TRACK_GAIN_THROTTLE, TRACK_DERIV_THROTTLE);
            out.pitch = pd_update(&pd_pitch, err_pitch, TRACK_GAIN_PITCH, TRACK_DERIV_PITCH);
            out.yaw = 0;
            out.active = true;
        }
        else
        {
            pd_roll = pd_throttle = pd_pitch = (PD){0};
        }

        pthread_mutex_lock(&out_mtx);
        g_output = out;
        g_landmarks = lm;
        pthread_mutex_unlock(&out_mtx);
    }

    free(palm_buf);
    free(lmark_buf);
    return NULL;
}

/* ---- Public API ---- */

void tracker_init(void)
{
    memset(&g_landmarks, 0, sizeof(g_landmarks));
    memset(&g_output, 0, sizeof(g_output));
    ort = OrtGetApiBase()->GetApi(ORT_API_VERSION);
    ORT_CHECK(ort->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "tello_gs", &ort_env));
    ORT_CHECK(ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &mem_info));

    palm_session = load_session(PALM_MODEL);
    lmark_session = load_session(LMARK_MODEL);

    if (!palm_session || !lmark_session)
        fprintf(stderr, "tracker: one or more models failed to load — tracking disabled\n");

    /* Allocate frame buffers */
    frames[0].data = frames[1].data = NULL;
    frames[0].w = frames[1].w = 0;
    frames[0].h = frames[1].h = 0;

    pthread_create(&worker_tid, NULL, worker_thread, NULL);
}

void tracker_process_frame(const uint8_t *rgb, int w, int h)
{
    pthread_mutex_lock(&frame_mtx);
    Frame *f = &frames[write_idx];

    /* (Re)allocate if dimensions changed */
    if (!f->data || f->w != w || f->h != h)
    {
        free(f->data);
        f->data = malloc((size_t)(w * h * 3));
        f->w = w;
        f->h = h;
    }
    if (f->data)
        memcpy(f->data, rgb, (size_t)(w * h * 3));

    write_idx = 1 - write_idx; /* flip for worker */
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
        memset(&g_landmarks, 0, sizeof(g_landmarks));
        pthread_mutex_unlock(&out_mtx);
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

void tracker_cleanup(void)
{
    pthread_mutex_lock(&frame_mtx);
    frame_pending = true; /* unblock the condvar */
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
