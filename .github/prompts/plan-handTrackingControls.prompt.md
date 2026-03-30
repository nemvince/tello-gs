# Plan: Hand Tracking Controls with Gesture Recognition

**TL;DR:** Use the **onnxruntime C API** with MediaPipe-derived ONNX models. A dedicated worker thread runs two-stage inference (palm detector → hand landmark). A gesture module interprets 21 3D landmarks into discrete commands and continuous RC overrides. The drone **follows the palm** — centering it in frame laterally and maintaining constant perceived distance via forward/back pitch. All wired through the existing `TrackerOutput` interface.

---

## Library Choice

### Why onnxruntime over libmediapipe

| | libmediapipe | **onnxruntime (chosen)** |
|---|---|---|
| Language API | C | C |
| Install | Build from source (Bazel + OpenCV 4.7.0, ~1h) | `apt install libonnxruntime-dev` |
| Age | 3 years old, v0.8.11 | Actively maintained |
| License | GPL-3.0 | Apache 2.0 |
| Pipeline control | Black box | Full visibility / tunable |

The two-stage ONNX pipeline (palm-detection + hand-landmark) matches exactly what libmediapipe wraps internally — we just own it directly.

---

## Control Mapping (to be tuned during implementation)

```
Palm centroid X offset from frame center  →  roll   (left/right)
Palm centroid Y offset from frame center  →  throttle (up/down)
Palm bounding-box pixel area              →  pitch   (advance/retreat to maintain distance)
Yaw                                       →  manual only (controller / keyboard)
```
A PD controller on each axis prevents oscillation.

**Gestures (discrete):**
- Open palm visible → activate tracking mode
- Fist or hand lost for N frames → deactivate (drone hovers)
- Thumbs-up → `drone_send("takeoff")`
- Thumbs-down → `drone_send("land")`

---

## Steps

### Phase 1 — Build setup *(no dependencies on later phases)*
1. Add `scripts/download_models.sh`: fetches `palm_detection_lite.onnx` and `hand_landmark_lite.onnx` from PINTO0309 model zoo into `models/`. These are widely-used ONNX exports of the exact MediaPipe models.
2. Update `Makefile`: add `$(shell pkg-config --cflags onnxruntime)` / `--libs` (with fallback hardcoded paths), add `models/` to `.gitignore`.

### Phase 2 — `tracker.c` (async inference worker) *(depends on Phase 1)*
3. Extend `include/tracker.h`: add `tracker_set_enabled(bool)` and `tracker_is_enabled()`. Add `HandLandmarks` struct (21 × {x,y,z} float, normalised 0–1) and `tracker_get_landmarks()` for the HUD.
4. Replace `src/tracker.c` stub:
   - **Frame queue**: mutex + condvar double-buffer (video thread writes, worker reads). `tracker_process_frame()` just copies the frame and signals.
   - **Worker thread**: on each frame, run palm detector → if palm found, crop + run landmark model → call `gesture_classify()` → store `TrackerOutput` + raw landmarks atomically.
   - ORT session setup in `tracker_init()` (load both `.onnx` files from `MODELS_DIR`). Pre-allocate input tensors.
   - Auto-disable safety: if no palm detected for >30 frames, set `TrackerOutput.active = false`.

### Phase 3 — `gesture.c` *(depends on Phase 2 for landmark format)*
5. Create `include/gesture.h`: declare `GestureID` enum (`GESTURE_NONE`, `GESTURE_OPEN_PALM`, `GESTURE_THUMBS_UP`, `GESTURE_THUMBS_DOWN`) and `gesture_classify(HandLandmarks)`.
6. Create `src/gesture.c`: rule-based classifier using landmark geometry:
   - *Finger extended check*: tip Y < PIP Y (in image space) → finger is extended
   - *Open palm*: all 5 fingers extended
   - *Thumbs-up*: only thumb extended AND thumb tip above wrist
   - *Thumbs-down*: only thumb extended AND thumb tip below wrist
   - Returns `GestureID` + internally fires `drone_send()` for takeoff/land on rising edge (debounced — gesture must hold for 500 ms before triggering)

### Phase 4 — PD controller *(inside Phase 2 worker, depends on Phase 3)*
7. In `tracker.c` worker, after landmark model: compute palm centroid (average of landmarks 0–4) and palm size (wrist-to-middle-MCP distance in pixels). Feed into simple proportional-derivative controller per axis → clamp to [-100, 100] → store in `TrackerOutput`. Gain constants defined in `include/config.h`.

### Phase 5 — HUD overlay *(parallel with Phases 2–4)*
8. Extend `hud_draw()` signature in `include/hud.h` to accept `const HandLandmarks *lm` (NULL = not tracking). In `src/hud.c`:
   - Draw the 21 landmark dots and skeleton connections defined by MediaPipe hand topology (scaled from normalised coords to window pixels)
   - Draw tracking mode string: `TRACKING` (green) / `MANUAL` (white)
   - Show gesture name while active

### Phase 6 — Integration *(depends on all above)*
9. `src/main.c`: add `Tab` key and gamepad `Y`-button toggle calling `tracker_set_enabled()`. Pass `tracker_get_landmarks()` to `hud_draw()`. Verify `TrackerOutput.active` already gates RC override (it does — no change needed there).
10. Verify build: `make && ./tello_gs` with live drone or by pointing the video stream at a hand.

---

## Files Changed / Created

| File | Change |
|---|---|
| `include/tracker.h` | Add `tracker_set_enabled`, `tracker_is_enabled`, `HandLandmarks`, `tracker_get_landmarks` |
| `src/tracker.c` | Full replacement: ORT sessions, worker thread, double-buffer, PD controller |
| `include/gesture.h` | New: `GestureID` enum, `gesture_classify(HandLandmarks)` |
| `src/gesture.c` | New: rule-based classifier, debounced command dispatch |
| `include/config.h` | Add `TRACK_GAIN_*` PD constants and `MODELS_DIR` path macro |
| `include/hud.h` | Extend `hud_draw` signature |
| `src/hud.c` | Draw skeleton overlay, tracking indicator, gesture label |
| `src/main.c` | Tab toggle, pass landmarks to HUD |
| `Makefile` | onnxruntime link flags |
| `scripts/download_models.sh` | New: one-shot model download |
| `models/*.onnx` | Runtime data (gitignored, fetched by script) |

---

## Verification

1. `scripts/download_models.sh` → both `.onnx` files present in `models/`
2. `make clean && make` → zero warnings/errors
3. With drone stream: raise open palm → `TRACKING` indicator goes green, RC commands sent; lower hand → falls back to manual
4. Thumbs-up gesture held 500 ms → drone takes off; thumbs-down → lands
5. Move palm left/right → roll authority; up/down → throttle authority; step toward/away → pitch responds

---

## Design Decisions

- PD gains (`TRACK_GAIN_ROLL`, `TRACK_GAIN_PITCH`, `TRACK_GAIN_THROTTLE`) live in `config.h` to make tuning easy without touching logic
- Yaw is deliberately excluded from tracking to avoid spin instability; can be added later
- Debounce period (500 ms) prevents accidental takeoff/land from transient landmark noise
- `MODELS_DIR` defaults to `./models` (relative to cwd), overridable via `CFLAGS -DMODELS_DIR=\"/path\"` at compile time
