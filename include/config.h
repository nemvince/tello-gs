#pragma once

/* ---- Network ---- */
#define TELLO_IP "192.168.10.1"
#define CMD_PORT 8889
#define STATE_PORT 8890
#define VIDEO_PORT 11111

/* ---- Display ---- */
#define WIN_W 960
#define WIN_H 720

/* ---- Control ---- */
#define RC_RATE_MS 50  /* RC command interval (ms) */
#define DEADZONE 0.15f /* Stick deadzone (0-1) */
#define EXPO 2.0f      /* Expo curve exponent */

/* ---- Hand tracking ---- */
#ifndef MODELS_DIR
#define MODELS_DIR "./models"
#endif
#define PALM_MODEL MODELS_DIR "/palm_detection_lite.onnx"
#define LMARK_MODEL MODELS_DIR "/hand_landmark_lite.onnx"

/* Palm detector input size */
#define PALM_INPUT_W 192
#define PALM_INPUT_H 192

/* Landmark model input size */
#define LMARK_INPUT_W 224
#define LMARK_INPUT_H 224

/* PD controller gains (tune these while flying) */
#define TRACK_GAIN_ROLL 120.0f /* proportional, error in [-0.5, 0.5] */
#define TRACK_DERIV_ROLL 30.0f /* derivative */
#define TRACK_GAIN_THROTTLE 120.0f
#define TRACK_DERIV_THROTTLE 30.0f
#define TRACK_GAIN_PITCH 60.0f /* palm-size error → pitch (gentle) */
#define TRACK_DERIV_PITCH 15.0f

/* Desired palm size as fraction of frame width (tune = desired hover distance) */
#define TRACK_TARGET_PALM_SIZE 0.25f

/* Frames without detection before auto-disabling RC override */
#define TRACK_LOST_FRAMES 30

/* Gesture debounce: ms the gesture must be held before firing command */
#define GESTURE_DEBOUNCE_MS 500
