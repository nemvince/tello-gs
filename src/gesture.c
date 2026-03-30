#include "gesture.h"
#include "drone.h"
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

/* Returns true when a finger (not thumb) is extended:
 * tip is higher (smaller Y) than the PIP joint. */
static bool finger_extended(const HandLandmarks *lm, int tip, int pip)
{
  return lm->pts[tip].y < lm->pts[pip].y;
}

/* Thumb extension: tip is further from the wrist in X than the MCP.
 * Use absolute X distance to handle both left and right hands. */
static bool thumb_extended(const HandLandmarks *lm)
{
  float tip_dist = fabsf(lm->pts[LM_THUMB_TIP].x - lm->pts[LM_WRIST].x);
  float mcp_dist = fabsf(lm->pts[LM_THUMB_MCP].x - lm->pts[LM_WRIST].x);
  return tip_dist > mcp_dist * 1.2f;
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

  bool idx = finger_extended(lm, LM_INDEX_TIP, LM_INDEX_PIP);
  bool mid = finger_extended(lm, LM_MIDDLE_TIP, LM_MIDDLE_PIP);
  bool ring = finger_extended(lm, LM_RING_TIP, LM_RING_PIP);
  bool pinky = finger_extended(lm, LM_PINKY_TIP, LM_PINKY_PIP);
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
    if (current == GESTURE_THUMBS_UP)
      drone_send("takeoff");
    else
      drone_send("land");
  }

  return current;
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
