#pragma once

/*
 * Runtime-adjustable settings that mirror the compile-time defaults in
 * config.h.  Loaded from / saved to ~/.config/tello-gs/settings.conf.
 */

typedef struct
{
    /* Network */
    char tello_ip[64];
    int  cmd_port;
    int  state_port;
    int  video_port;

    /* Control */
    int   rc_rate_ms;
    float deadzone;
    float expo;

    /* Tracking PD gains */
    float gain_roll;
    float deriv_roll;
    float gain_throttle;
    float deriv_throttle;
    float gain_pitch;
    float deriv_pitch;
    float target_palm_size;
    int   lost_frames;

    /* Gesture */
    int gesture_debounce_ms;
} AppSettings;

/* Global settings instance — initialised by settings_load(). */
extern AppSettings g_settings;

/* Load from disk (creates defaults if file absent). */
void settings_load(void);

/* Write current values to disk. */
void settings_save(void);

/* Reset to compile-time defaults. */
void settings_reset_defaults(void);
