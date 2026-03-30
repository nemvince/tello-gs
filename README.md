# Tello Ground Station

A ground station for the DJI/Ryze Tello drone written in C.  
Supports live video, manual flight controls (keyboard and gamepad), and AI-powered hand tracking with gesture commands.

---

## Dependencies

| Library | Purpose |
|---|---|
| SDL2 | Window, input, rendering |
| SDL2_ttf | HUD text |
| FFmpeg (`libavcodec`, `libavutil`, `libswscale`) | H.264 video decode |
| onnxruntime | Hand tracking inference |

Install on Debian/Ubuntu:

```sh
sudo apt install libsdl2-dev libsdl2-ttf-dev \
                 libavcodec-dev libavutil-dev libswscale-dev \
                 libonnxruntime-dev
```

---

## Build

```sh
make
```

The binary is placed at `./tello_gs`.

---

## ONNX Models (hand tracking)

Hand tracking requires two ONNX models in `./models/`:

| File | Source |
|---|---|
| `models/palm_detection_lite.onnx` | PINTO0309 model zoo, `033_Hand_Detection_and_Tracking` |
| `models/hand_landmark_lite.onnx` | PINTO0309 model zoo, `033_Hand_Detection_and_Tracking` |

Download automatically (requires ~700 MB download):

```sh
bash scripts/download_models.sh
```

Or extract manually from:  
`033_Hand_Detection_and_Tracking/30_batchN_post-process_marged/post_process_marged.tar.gz`  
inside the Wasabi archive at  
`https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/033_Hand_Detection_and_Tracking/resources.tar.gz`

The app runs without the models — hand tracking simply stays disabled until the models are present.

---

## Running

1. Power on the Tello and connect to its Wi-Fi (`TELLO-XXXXXX`).
2. Launch the ground station:

```sh
./tello_gs
```

The app automatically sends `command` and `streamon` on startup and `land` + `streamoff` on exit.

---

## Manual Flight Controls

### Keyboard

| Key | Action |
|---|---|
| `T` | Takeoff |
| `L` | Land |
| `Space` | **Emergency stop** (kills motors immediately) |
| `Tab` | Toggle hand tracking on/off |
| `W` / `S` | Pitch forward / backward |
| `A` / `D` | Roll left / right |
| `↑` / `↓` | Throttle up / down |
| `←` / `→` | Yaw left / right |
| `Escape` | Quit |

Keyboard movement speed is fixed at 60% authority. If a gamepad is connected, keyboard movement is disabled (use the gamepad instead).

### Gamepad (Xbox layout)

| Input | Action |
|---|---|
| **A** | Takeoff |
| **B** | Land |
| **X** | Emergency stop |
| **Y** | Toggle hand tracking on/off |
| **LB** | Flip left |
| **RB** | Flip right |
| **Left stick** X | Roll left / right |
| **Left stick** Y | Pitch forward / backward |
| **Right stick** X | Yaw left / right |
| **Right stick** Y | Throttle up / down |

Sticks have a 15% deadzone and an expo curve (exponent 2.0) for fine control near centre. RC commands are sent at 20 Hz.

---

## Hand Tracking Mode

Hand tracking uses a two-stage ONNX inference pipeline:

1. **Palm detector** (`palm_detection_lite.onnx`, 192×192 input) — locates the hand in the frame.
2. **Landmark model** (`hand_landmark_lite.onnx`, 224×224 input) — produces 21 3D landmarks on the detected hand.

A PD controller converts landmark position and palm size into RC commands sent to the drone.

### Enabling tracking

Press **Tab** (keyboard) or **Y** (gamepad) to toggle. The HUD shows **TRACKING** in green when active.

When enabled, the tracker output **overrides** manual stick input completely. Toggle it off to regain manual control.

### How the drone follows your hand

| Axis | Behaviour |
|---|---|
| **Roll** | Tracks hand X position — drone moves left/right to keep the hand centred horizontally |
| **Throttle** | Tracks hand Y position — drone climbs/descends to keep the hand centred vertically |
| **Pitch** | Tracks palm size — drone moves forward/back to maintain a fixed hover distance |
| **Yaw** | Always 0 in tracking mode |

If the hand is lost for 30 consecutive frames (~1.5 s), tracking auto-disables and control returns to manual.

### Tuning tracking behaviour

Edit `include/config.h` and rebuild:

| Constant | Default | Effect |
|---|---|---|
| `TRACK_GAIN_ROLL` | `120.0` | Roll aggressiveness |
| `TRACK_DERIV_ROLL` | `30.0` | Roll damping |
| `TRACK_GAIN_THROTTLE` | `120.0` | Throttle aggressiveness |
| `TRACK_DERIV_THROTTLE` | `30.0` | Throttle damping |
| `TRACK_GAIN_PITCH` | `200.0` | Distance-hold aggressiveness |
| `TRACK_DERIV_PITCH` | `50.0` | Distance-hold damping |
| `TRACK_TARGET_PALM_SIZE` | `0.25` | Target palm width as fraction of frame — increase to hover closer |
| `TRACK_LOST_FRAMES` | `30` | Frames without detection before auto-disable |

---

## Gesture Commands

Gestures are recognised while tracking is active and trigger drone commands after being held for **500 ms**.

| Gesture | Command |
|---|---|
| **Open palm** (all 4 fingers extended) | Activates/maintains tracking mode — no command fired, just keeps the drone following |
| **Thumbs up** (thumb only, pointing up) | Takeoff |
| **Thumbs down** (thumb only, pointing down) | Land |

The HUD displays the current recognised gesture name above the wrist landmark.

---

## HUD

The on-screen display overlays the live video with:

- **Top-left** — battery %, height (cm), speed (cm/s), flight time
- **Top-right** — Wi-Fi signal strength, input mode (KEYBOARD / GAMEPAD)
- **Left edge** — **TRACKING** indicator (green when hand tracking is active)
- **Hand skeleton** — cyan bone lines + red landmark dots drawn on the detected hand
- **Gesture label** — current gesture name shown above the wrist

---

## Project Structure

```
tello_gs.c          Original single-file version (reference only)
src/
  main.c            Main loop, SDL init, event handling
  drone.c           UDP socket, command/RC send
  video.c           FFmpeg H.264 decode, frame buffer
  telemetry.c       State UDP listener, parsed telemetry
  input.c           Keyboard/gamepad RC reading
  hud.c             SDL HUD and skeleton overlay rendering
  tracker.c         ONNX inference worker, PD controller
  gesture.c         Rule-based landmark gesture classifier
include/
  config.h          All compile-time constants (network, gains, model paths)
  app.h             Shared types (TrackerOutput, etc.)
  drone.h / video.h / telemetry.h / input.h / hud.h / tracker.h / gesture.h
scripts/
  download_models.sh  Fetches ONNX models from Wasabi storage
models/             Runtime model files (not committed)
build/              Object files (generated)
```
