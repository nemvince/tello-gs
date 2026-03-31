#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "settings.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

AppSettings g_settings;

/* ------------------------------------------------------------------ */
/* Defaults (mirrors config.h)                                         */
/* ------------------------------------------------------------------ */

void settings_reset_defaults(void)
{
  strncpy(g_settings.tello_ip, TELLO_IP, sizeof(g_settings.tello_ip) - 1);
  g_settings.tello_ip[sizeof(g_settings.tello_ip) - 1] = '\0';

  g_settings.cmd_port = CMD_PORT;
  g_settings.state_port = STATE_PORT;
  g_settings.video_port = VIDEO_PORT;

  g_settings.rc_rate_ms = RC_RATE_MS;
  g_settings.deadzone = DEADZONE;
  g_settings.expo = EXPO;

  g_settings.gain_roll = TRACK_GAIN_ROLL;
  g_settings.deriv_roll = TRACK_DERIV_ROLL;
  g_settings.gain_throttle = TRACK_GAIN_THROTTLE;
  g_settings.deriv_throttle = TRACK_DERIV_THROTTLE;
  g_settings.gain_pitch = TRACK_GAIN_PITCH;
  g_settings.deriv_pitch = TRACK_DERIV_PITCH;
  g_settings.target_palm_size = TRACK_TARGET_PALM_SIZE;
  g_settings.lost_frames = TRACK_LOST_FRAMES;

  g_settings.gesture_debounce_ms = GESTURE_DEBOUNCE_MS;
}

/* ------------------------------------------------------------------ */
/* Config-dir / file path helpers                                      */
/* ------------------------------------------------------------------ */

static void get_config_dir(char *out, size_t n)
{
  const char *home = getenv("HOME");
  if (!home || home[0] == '\0')
    home = "/tmp";
  snprintf(out, n, "%s/.config/tello-gs", home);
}

static void get_config_path(char *out, size_t n)
{
  char dir[512];
  get_config_dir(dir, sizeof(dir));
  snprintf(out, n, "%s/settings.conf", dir);
}

/* ------------------------------------------------------------------ */
/* Load                                                                */
/* ------------------------------------------------------------------ */

void settings_load(void)
{
  settings_reset_defaults();

  char path[640];
  get_config_path(path, sizeof(path));

  FILE *f = fopen(path, "r");
  if (!f)
    return; /* first run - use defaults */

  char line[256];
  while (fgets(line, sizeof(line), f))
  {
    /* Skip comments and blank lines */
    char *p = line;
    while (*p == ' ' || *p == '\t')
      p++;
    if (*p == '#' || *p == '\n' || *p == '\0')
      continue;

    char key[64], val[192];
    if (sscanf(line, " %63[^= ] = %191[^\n]", key, val) != 2)
      continue;

    /* Trim trailing whitespace from val */
    int vlen = (int)strlen(val);
    while (vlen > 0 && (val[vlen - 1] == ' ' || val[vlen - 1] == '\r' || val[vlen - 1] == '\n'))
      val[--vlen] = '\0';

#define LOAD_STR(k, field)                                           \
  if (strcmp(key, k) == 0)                                           \
  {                                                                  \
    snprintf(g_settings.field, sizeof(g_settings.field), "%s", val); \
    continue;                                                        \
  }
#define LOAD_INT(k, field)        \
  if (strcmp(key, k) == 0)        \
  {                               \
    g_settings.field = atoi(val); \
    continue;                     \
  }
#define LOAD_FLT(k, field)               \
  if (strcmp(key, k) == 0)               \
  {                                      \
    g_settings.field = (float)atof(val); \
    continue;                            \
  }

    LOAD_STR("tello_ip", tello_ip)
    LOAD_INT("cmd_port", cmd_port)
    LOAD_INT("state_port", state_port)
    LOAD_INT("video_port", video_port)
    LOAD_INT("rc_rate_ms", rc_rate_ms)
    LOAD_FLT("deadzone", deadzone)
    LOAD_FLT("expo", expo)
    LOAD_FLT("gain_roll", gain_roll)
    LOAD_FLT("deriv_roll", deriv_roll)
    LOAD_FLT("gain_throttle", gain_throttle)
    LOAD_FLT("deriv_throttle", deriv_throttle)
    LOAD_FLT("gain_pitch", gain_pitch)
    LOAD_FLT("deriv_pitch", deriv_pitch)
    LOAD_FLT("target_palm_size", target_palm_size)
    LOAD_INT("lost_frames", lost_frames)
    LOAD_INT("gesture_debounce_ms", gesture_debounce_ms)

#undef LOAD_STR
#undef LOAD_INT
#undef LOAD_FLT
  }

  fclose(f);
}

/* ------------------------------------------------------------------ */
/* Save                                                                */
/* ------------------------------------------------------------------ */

void settings_save(void)
{
  char dir[512];
  get_config_dir(dir, sizeof(dir));

  /* Create directory (and intermediate paths) if absent */
  {
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", dir);
    for (char *s = tmp + 1; *s; s++)
    {
      if (*s == '/')
      {
        *s = '\0';
        mkdir(tmp, 0755);
        *s = '/';
      }
    }
    mkdir(tmp, 0755);
  }

  char path[640];
  get_config_path(path, sizeof(path));

  FILE *f = fopen(path, "w");
  if (!f)
  {
    fprintf(stderr, "settings: cannot write to %s\n", path);
    return;
  }

  fprintf(f, "# tello-gs settings - auto-generated, do not edit while running\n\n");

  fprintf(f, "# Network\n");
  fprintf(f, "tello_ip            = %s\n", g_settings.tello_ip);
  fprintf(f, "cmd_port            = %d\n", g_settings.cmd_port);
  fprintf(f, "state_port          = %d\n", g_settings.state_port);
  fprintf(f, "video_port          = %d\n", g_settings.video_port);

  fprintf(f, "\n# Control\n");
  fprintf(f, "rc_rate_ms          = %d\n", g_settings.rc_rate_ms);
  fprintf(f, "deadzone            = %.3f\n", g_settings.deadzone);
  fprintf(f, "expo                = %.2f\n", g_settings.expo);

  fprintf(f, "\n# Tracking PD gains\n");
  fprintf(f, "gain_roll           = %.2f\n", g_settings.gain_roll);
  fprintf(f, "deriv_roll          = %.2f\n", g_settings.deriv_roll);
  fprintf(f, "gain_throttle       = %.2f\n", g_settings.gain_throttle);
  fprintf(f, "deriv_throttle      = %.2f\n", g_settings.deriv_throttle);
  fprintf(f, "gain_pitch          = %.2f\n", g_settings.gain_pitch);
  fprintf(f, "deriv_pitch         = %.2f\n", g_settings.deriv_pitch);
  fprintf(f, "target_palm_size    = %.3f\n", g_settings.target_palm_size);
  fprintf(f, "lost_frames         = %d\n", g_settings.lost_frames);

  fprintf(f, "\n# Gesture\n");
  fprintf(f, "gesture_debounce_ms = %d\n", g_settings.gesture_debounce_ms);

  fclose(f);
}
