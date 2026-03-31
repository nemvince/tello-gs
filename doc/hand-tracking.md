# Hand Tracking Pipeline

The hand tracking system is the core feature of the ground station. It runs a MediaPipe-style two-stage inference pipeline entirely in C using the ONNX Runtime C API, followed by gesture classification and PD-based flight control.

## Models

Both models come from [PINTO0309's model zoo](https://github.com/PINTO0309/PINTO_model_zoo/tree/main/033_Hand_Detection_and_Tracking) and are the "lite" variants optimised for real-time inference.

### Palm Detection (`palm_detection_lite.onnx`)

| Property | Value |
|----------|-------|
| Input | `"input"` — `[1, 3, 192, 192]` NCHW float32, normalised to [0, 1] |
| Output 1 | `"regressors"` — `[1, 2016, 18]` box offsets + keypoints |
| Output 2 | `"classificators"` — `[1, 2016, 1]` raw logits |

The model produces 2,016 SSD anchor predictions. Each anchor's regressor contains:
- `[0]` Δcy, `[1]` Δcx, `[2]` h, `[3]` w — all in 192-pixel space
- `[4..17]` — 7 keypoints × (x, y), unused in the current pipeline

**Anchor layout** (matches MediaPipe's `SsdAnchorsCalculator`):
- Stride 8: 24×24 grid × 2 anchors/cell = 1,152
- Stride 16: 12×12 grid × 6 anchors/cell = 864 (3 layers × 2)
- Total: 2,016 anchors, pre-generated at startup

**Decoding:**
```
score = sigmoid(classificators[i])
if score >= 0.5:
    cx = anchor_cx + reg[1] / 192
    cy = anchor_cy + reg[0] / 192
    w  = |reg[3]| / 192
    h  = |reg[2]| / 192
```

**Post-processing:** Greedy NMS with IoU threshold 0.3. Only the best surviving detection is used.

### Hand Landmark (`hand_landmark_lite.onnx`)

| Property | Value |
|----------|-------|
| Input | `"input"` — `[1, 3, 224, 224]` NCHW float32, normalised to [0, 1] |
| Output 1 | `"Identity"` — `[1, 63]` = 21 landmarks × (x, y, z) in crop space |
| Output 2 | `"Identity_1"` — `[1, 1]` hand presence logit |

The crop is a square region centred on the palm detection, expanded by 1.5× (`CROP_SCALE`), clamped to frame bounds, and resized to 224×224.

**Projection to frame coordinates:**
```
frame_x = (clamp_x0 + lm_x / 224 * actual_crop_w) / frame_w
frame_y = (clamp_y0 + lm_y / 224 * actual_crop_h) / frame_h
```

**Presence threshold:**
- 0.5 when the detection came from the palm detector
- 0.8 when re-tracking from previous landmarks (stricter to prevent ghost locks)

## Re-Tracking (Frame-to-Frame)

Running the palm detector every frame is expensive and produces jittery bounding boxes. Instead, the pipeline uses a **re-tracking** strategy that mirrors the real MediaPipe pipeline:

1. After a successful landmark extraction, save the raw (pre-smoothing) landmarks.
2. On the next frame, compute a bounding box from those 21 points (with 1.8× padding) and feed it directly to the landmark model — skipping the palm detector entirely.
3. If the landmark model rejects the crop (hand presence below 0.8), fall back to the palm detector for that frame.
4. If the palm detector also fails, increment the lost-frame counter.

This gives:
- **Lower latency** — one model instead of two per frame
- **More stable tracking** — incremental crop updates instead of full re-detection
- **Automatic recovery** — if re-tracking fails, the palm detector kicks in

## EMA Smoothing

Raw landmark positions jitter between frames. An exponential moving average (α = 0.8) smooths each landmark's x, y, z:

```
smoothed = α × current + (1 - α) × previous_smoothed
```

α = 0.8 gives responsive tracking while filtering high-frequency noise. Smoothing is applied after landmark extraction but before gesture classification and PD control.

## Gesture Classification

Gestures are identified from the 21 landmark positions using geometric analysis — no additional ML model.

### Finger Extension (Angle-Based)

A finger is "extended" when the PIP joint is roughly straight. The system computes the angle at the PIP joint using the dot product of vectors PIP→MCP and PIP→DIP:

```
cos_angle = dot(PIP→MCP, PIP→DIP) / (|PIP→MCP| × |PIP→DIP|)
extended = cos_angle < -0.5   (angle > ~120°)
```

This is **rotation-invariant** — it works regardless of hand orientation, unlike simpler Y-coordinate comparisons.

### Thumb Extension

The thumb uses the same angle approach at the MCP joint (CMC→MCP→IP), with a looser threshold (cos < -0.3) because the thumb's natural range of motion is wider.

### Recognised Gestures

| Gesture | Condition | Action |
|---------|-----------|--------|
| **Open Palm** | All 4 fingers + thumb extended | Enables PD tracking |
| **Thumbs Up** | Only thumb extended, thumb tip above wrist | — (reserved) |
| **Thumbs Down** | Only thumb extended, thumb tip below wrist | Land drone |
| **None** | Anything else | PD tracking active if previously enabled |

**Debounce:** Command gestures (thumbs-down) must be held for 500 ms before the main loop acts on them. This prevents accidental triggers from momentary mis-classifications.

## PD Controller

When tracking is enabled and the gesture is open-palm (or none), a proportional-derivative controller generates RC commands from the landmark positions.

### Roll (Left / Right)

```
error = centroid_x - 0.5
output = Kp × error + Kd × (error - prev_error)
Kp = 120, Kd = 30
```

The centroid is the average of all 21 landmarks' x-coordinates. The drone strafes to keep the hand centred horizontally.

### Throttle (Up / Down)

```
error = -(centroid_y - 0.5)
output = Kp × error + Kd × (error - prev_error)
Kp = 120, Kd = 30
```

Inverted because pixel y increases downward. The drone climbs/descends to keep the hand centred vertically.

### Pitch (Forward / Backward — Distance Keeping)

```
palm_size = distance(wrist, middle_finger_MCP)     // normalised
error = TARGET_PALM_SIZE - palm_size
```

If the hand moves further away (palm_size shrinks), the drone flies forward. If the hand comes closer (palm_size grows), the drone backs up.

**Safeguards for pitch:**
- **Deadzone** (0.03): Small palm-size fluctuations are ignored to prevent jitter-driven lurching.
- **Speed cap** (±30 out of ±100): Forward/backward motion is much slower than lateral/vertical.
- **Lower gains** (Kp=60, Kd=15): About half the lateral gains, so distance adjustments are cautious.

### Edge Clamping

If the palm centroid is within 5% of any frame edge, that axis's error is set to zero and its PD state is reset. This prevents the drone from continuing to fly toward a hand that has left the frame.

### Auto-Disarm

If no hand is detected for 30 consecutive frames (~1.5 seconds at 20 Hz), the PD controller deactivates completely. All PD state, smoothing state, and re-tracking state are reset.

## Image Preprocessing

Both models expect NCHW (channel-first) float32 tensors normalised to [0, 1].

**Normalisation:** `pixel_value / 255.0f`

> This was a critical discovery during development — the models were initially fed [-1, 1] normalised input (v/127.5 - 1.0), which produced maximum palm confidence of only ~0.42 (below the 0.5 threshold). Switching to [0, 1] normalisation immediately raised confidence to 0.96+.

**Interpolation:** Bilinear with half-pixel centre alignment: `src = (dst + 0.5) × scale - 0.5`

The `resize_normalise()` function handles full-frame resizing, while `crop_resize_normalise()` first extracts and clamps a rectangular ROI, then resizes the clamped region.
