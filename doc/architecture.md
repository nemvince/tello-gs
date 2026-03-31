# Architecture Overview

Tello Ground Station is a real-time drone control application written in C. It streams live video from a DJI Tello drone, overlays a heads-up display, and optionally tracks a human hand to generate autonomous flight commands — all from a single binary with no scripting layer.

## Libraries & Frameworks

| Library | Version | Role |
|---------|---------|------|
| **SDL2** | 2.x | Window, rendering, input (keyboard + gamepad) |
| **SDL2_ttf** | 2.x | Font rendering for the HUD |
| **FFmpeg** (libavcodec, libavutil, libswscale) | 6.x+ | H.264 video decoding, YUV→RGB conversion |
| **ONNX Runtime** | 1.24.4 | Neural network inference (C API) |
| **pthreads** | POSIX | Threading and synchronisation |

Build toolchain: GCC (C11/GNU), Make, pkg-config for dependency resolution.

## Threading Model

The application runs **four background threads** plus the main thread:

```
 MAIN THREAD                         VIDEO THREAD
 ┌────────────────────┐              ┌─────────────────────┐
 │ SDL event loop      │              │ UDP recv (port 11111)│
 │ RC commands @ 20 Hz │              │ H.264 decode         │
 │ Gesture → commands  │              │ YUV → RGB24          │
 │ HUD rendering       │              │ Double-buffer write  │
 └────────┬───────────┘              └─────────┬───────────┘
          │ reads output                       │ signals worker
          │                                    ▼
          │               TRACKER WORKER THREAD
          │              ┌──────────────────────────┐
          │              │ Palm detection (ONNX)     │
          │              │ Landmark extraction (ONNX)│
          │              │ Gesture classification    │
          │              │ PD controller → RC values │
          │              └──────────────────────────┘
          │
          │ reads telemetry
          ▼
 TELEMETRY THREAD         RESPONSE THREAD
 ┌──────────────────┐     ┌────────────────────┐
 │ UDP recv (8890)   │     │ UDP recv (8889)     │
 │ Parse state pkt   │     │ Log "ok" / "error"  │
 │ Update struct     │     │ Set connected flag   │
 └──────────────────┘     └────────────────────┘
```

All shared state is protected by `pthread_mutex_t`. The video-to-tracker handoff uses a condition variable so the worker blocks until a frame arrives.

## Data Flow

1. **Video thread** receives UDP packets on port 11111, decodes H.264, converts to RGB24, writes into a double-buffer, and signals the tracker worker.
2. **Tracker worker** wakes up, runs the two-stage ML pipeline (palm detection → hand landmarks), classifies gestures, runs the PD controller, and publishes results under a mutex.
3. **Main thread** at 20 Hz reads the tracker output. If tracking is active, the tracker's RC values override manual input. It also checks for gesture commands (thumbs-down → land). The merged RC values are sent to the drone via `drone_rc()`.
4. **Telemetry thread** independently receives and parses state packets from the drone (battery, altitude, attitude, velocity).
5. **HUD** is drawn every frame by the main thread, compositing video texture + telemetry + hand skeleton + debug info.

## Source Layout

```
tello2/
├── include/           Header files (one per module + config + app)
│   ├── app.h          Global g_running flag
│   ├── config.h       All compile-time constants
│   ├── drone.h        Tello UDP protocol
│   ├── video.h        Video decoding
│   ├── telemetry.h    State packet parsing
│   ├── tracker.h      Hand tracking + PD controller
│   ├── gesture.h      Gesture classification
│   ├── hud.h          HUD rendering
│   └── input.h        Stick/keyboard processing
├── src/               Implementations
│   ├── main.c         Entry point, event loop, RC multiplexing
│   ├── drone.c        UDP command socket, response thread
│   ├── video.c        H.264 decode pipeline, frame callback
│   ├── telemetry.c    State packet parser
│   ├── tracker.c      ONNX inference, PD controller, re-tracking
│   ├── gesture.c      Finger angle analysis, gesture state machine
│   ├── hud.c          SDL2 draw calls, attitude indicator, skeleton
│   └── input.c        Deadzone, expo curve, gamepad/keyboard mapping
├── models/            ONNX model files
│   ├── palm_detection_lite.onnx
│   └── hand_landmark_lite.onnx
├── Makefile
└── doc/               This documentation
```

## Network Protocol

The Tello uses a simple text-over-UDP protocol:

| Port | Direction | Purpose |
|------|-----------|---------|
| **8889** | GS → Drone | SDK commands (`"command"`, `"takeoff"`, `"rc 0 0 0 0"`) |
| **8889** | Drone → GS | Responses (`"ok"`, `"error"`) |
| **8890** | Drone → GS | State telemetry (semicolon-delimited key:value) |
| **11111** | Drone → GS | H.264 video stream (raw NAL units over UDP) |

The drone lives at `192.168.10.1` on its own Wi-Fi network. The ground station binds its sockets to `0.0.0.0` on the corresponding ports.

## Safety Design

All drone commands originate **exclusively** from `main.c`. The tracker module computes RC override values but never calls `drone_send()` — this is an intentional safety boundary. The main thread decides whether to use tracking output, manual input, or issue gesture-triggered commands.

Additional safeguards:
- **Auto-disarm**: If no hand is detected for 30 consecutive frames (~1.5 s), the PD controller deactivates and RC override stops.
- **Edge clamping**: If the hand reaches the frame edge, that control axis is zeroed to prevent the drone from flying out of view.
- **Gesture debounce**: Command gestures require 500 ms continuous hold before triggering.
- **Emergency kill**: Space bar or gamepad X sends `"emergency"` — instant motor cutoff.
