#include "gesture.h"
#include "config.h"

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* MediaPipe hand landmark indices */
#define LM_WRIST 0
#define LM_THUMB_CMC 1
#define LM_THUMB_MCP 2
#define LM_THUMB_IP 3
#define LM_THUMB_TIP 4
#define LM_INDEX_MCP 5
#define LM_INDEX_PIP 6
#define LM_INDEX_DIP 7
#define LM_INDEX_TIP 8
#define LM_MIDDLE_MCP 9
#define LM_MIDDLE_PIP 10
#define LM_MIDDLE_DIP 11
#define LM_MIDDLE_TIP 12
#define LM_RING_MCP 13
#define LM_RING_PIP 14
#define LM_RING_DIP 15
#define LM_RING_TIP 16
#define LM_PINKY_MCP 17
#define LM_PINKY_PIP 18
#define LM_PINKY_DIP 19
#define LM_PINKY_TIP 20

/* Returns true when a finger (not thumb) is extended.
 * Uses the angle at the PIP joint: if the DIP-PIP-MCP angle is roughly
 * straight (> ~150°), the finger is extended. This works regardless of
 * hand rotation, unlike a simple Y-comparison. */
static bool finger_extended(const HandLandmarks *lm, int mcp, int pip, int dip, int tip)
{
  (void)tip;
  /* Vector PIP→MCP */
  float ax = lm->pts[mcp].x - lm->pts[pip].x;
  float ay = lm->pts[mcp].y - lm->pts[pip].y;
  /* Vector PIP→DIP */
  float bx = lm->pts[dip].x - lm->pts[pip].x;
  float by = lm->pts[dip].y - lm->pts[pip].y;

  float dot = ax * bx + ay * by;
  float mag_a = sqrtf(ax * ax + ay * ay);
  float mag_b = sqrtf(bx * bx + by * by);
  if (mag_a < 1e-6f || mag_b < 1e-6f)
    return false;

  float cos_angle = dot / (mag_a * mag_b);
  /* cos(160°) ≈ -0.94, cos(150°) ≈ -0.87.
   * Bent finger: cos_angle is positive (small angle at PIP).
   * Straight finger: cos_angle is very negative (close to 180°).
   * Threshold: cos < -0.5 means angle > ~120° → extended. */
  return cos_angle < -0.5f;
}

/* Thumb extension: use angle at MCP joint (CMC→MCP→IP).
 * Same logic as fingers — if the joint is roughly straight, thumb is out. */
static bool thumb_extended(const HandLandmarks *lm)
{
  float ax = lm->pts[LM_THUMB_CMC].x - lm->pts[LM_THUMB_MCP].x;
  float ay = lm->pts[LM_THUMB_CMC].y - lm->pts[LM_THUMB_MCP].y;
  float bx = lm->pts[LM_THUMB_IP].x - lm->pts[LM_THUMB_MCP].x;
  float by = lm->pts[LM_THUMB_IP].y - lm->pts[LM_THUMB_MCP].y;

  float dot = ax * bx + ay * by;
  float mag_a = sqrtf(ax * ax + ay * ay);
  float mag_b = sqrtf(bx * bx + by * by);
  if (mag_a < 1e-6f || mag_b < 1e-6f)
    return false;

  float cos_angle = dot / (mag_a * mag_b);
  return cos_angle < -0.3f; /* thumb has wider natural angle, use looser threshold */
}

/* Debounce state per gesture */
typedef struct
{
  GestureID held;
  uint32_t since_ms;
  bool fired;
} DebounceState;

static DebounceState dbs = {GESTURE_NONE, 0, false};

GestureID gesture_classify(const HandLandmarks *lm, uint32_t now_ms)
{
  if (!lm->valid)
  {
    dbs.held = GESTURE_NONE;
    return GESTURE_NONE;
  }

  bool idx = finger_extended(lm, LM_INDEX_MCP, LM_INDEX_PIP, LM_INDEX_DIP, LM_INDEX_TIP);
  bool mid = finger_extended(lm, LM_MIDDLE_MCP, LM_MIDDLE_PIP, LM_MIDDLE_DIP, LM_MIDDLE_TIP);
  bool ring = finger_extended(lm, LM_RING_MCP, LM_RING_PIP, LM_RING_DIP, LM_RING_TIP);
  bool pinky = finger_extended(lm, LM_PINKY_MCP, LM_PINKY_PIP, LM_PINKY_DIP, LM_PINKY_TIP);
  bool thumb = thumb_extended(lm);

  GestureID current = GESTURE_NONE;

  if (idx && mid && ring && pinky)
  {
    current = GESTURE_OPEN_PALM; /* all four fingers extended */
  }
  else if (thumb && !idx && !mid && !ring && !pinky)
  {
    /* Thumb only — up or down based on tip vs wrist Y */
    bool thumb_up = lm->pts[LM_THUMB_TIP].y < lm->pts[LM_WRIST].y;
    current = thumb_up ? GESTURE_THUMBS_UP : GESTURE_THUMBS_DOWN;
  }

  /* Debounce for command gestures */
  if (current != dbs.held)
  {
    dbs.held = current;
    dbs.since_ms = now_ms;
    dbs.fired = false;
  }

  if (!dbs.fired &&
      (current == GESTURE_THUMBS_UP || current == GESTURE_THUMBS_DOWN) &&
      (now_ms - dbs.since_ms) >= GESTURE_DEBOUNCE_MS)
  {
    dbs.fired = true;
    /* NOTE: No drone_send() here. Command gestures are reported as return
     * values only — the caller (main loop) decides whether to act on them. */
  }

  return current;
}

/*
 * Read-only classify: same shape detection without debounce or drone_send().
 * Safe to call from any thread, any time, without side-effects.
 */
GestureID gesture_identify(const HandLandmarks *lm)
{
  if (!lm->valid)
    return GESTURE_NONE;

  bool idx = finger_extended(lm, LM_INDEX_MCP, LM_INDEX_PIP, LM_INDEX_DIP, LM_INDEX_TIP);
  bool mid = finger_extended(lm, LM_MIDDLE_MCP, LM_MIDDLE_PIP, LM_MIDDLE_DIP, LM_MIDDLE_TIP);
  bool ring = finger_extended(lm, LM_RING_MCP, LM_RING_PIP, LM_RING_DIP, LM_RING_TIP);
  bool pinky = finger_extended(lm, LM_PINKY_MCP, LM_PINKY_PIP, LM_PINKY_DIP, LM_PINKY_TIP);
  bool thumb = thumb_extended(lm);

  if (idx && mid && ring && pinky)
    return GESTURE_OPEN_PALM;
  if (thumb && !idx && !mid && !ring && !pinky)
  {
    bool thumb_up = lm->pts[LM_THUMB_TIP].y < lm->pts[LM_WRIST].y;
    return thumb_up ? GESTURE_THUMBS_UP : GESTURE_THUMBS_DOWN;
  }
  return GESTURE_NONE;
}

const char *gesture_name(GestureID g)
{
  switch (g)
  {
  case GESTURE_OPEN_PALM:
    return "OPEN PALM";
  case GESTURE_THUMBS_UP:
    return "THUMBS UP";
  case GESTURE_THUMBS_DOWN:
    return "THUMBS DOWN";
  default:
    return "";
  }
}
