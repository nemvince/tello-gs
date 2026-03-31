# Module Reference

Detailed breakdown of each module in the ground station.

---

## 1. Drone Communication (`drone.c`)

Handles all network I/O with the Tello over UDP port 8889.

**Responsibilities:**
- Send SDK text commands (`"command"`, `"takeoff"`, `"land"`, `"streamon"`, etc.)
- Send RC control packets at 20 Hz: `"rc <roll> <pitch> <throttle> <yaw>"` with values in [-100, 100]
- Run a background response thread that listens for `"ok"` / `"error"` replies
- Track connection state via `drone_connected` flag (set on first response or telemetry packet)

**API:**
```c
int  drone_init(void);              // Create socket, start response thread
void drone_send(const char *cmd);   // Fire-and-forget command
void drone_rc(int r, p, t, y);     // Send clamped RC values
void drone_cleanup(void);           // Join thread, close socket
```

**Design notes:**
- The socket is non-blocking for sends but uses `SO_RCVTIMEO=2s` for receives.
- `drone_rc()` clamps each axis to [-100, 100] before formatting the string.
- Response thread logs all replies to stderr for debugging.

---

## 2. Video Decoding (`video.c`)

Receives the Tello's H.264 video stream and decodes it to RGB24 frames.

**Pipeline:**
1. Bind UDP socket on port 11111 with 512 KB receive buffer
2. Feed raw UDP payloads into `av_parser_parse2()` to reconstruct H.264 NAL units
3. Decode complete packets via `avcodec_send_packet()` / `avcodec_receive_frame()`
4. Convert decoded YUV420P frame to RGB24 with `sws_scale()` (handles resolution changes)
5. Write to a double-buffer and call the tracker's frame callback

**Threading:**
- One background thread runs the receive → decode → convert loop.
- Main thread reads frames via `video_lock_frame()` / `video_unlock_frame()`.
- The tracker callback is invoked while the frame lock is held, ensuring a consistent snapshot.

**API:**
```c
void video_set_frame_callback(VideoFrameCallback cb);
void video_start(void);
bool video_lock_frame(const uint8_t **rgb, int *w, int *h);
void video_unlock_frame(void);
void video_stop(void);
```

**Libraries:** FFmpeg (libavcodec, libavutil, libswscale)

**Quirks:**
- The decoder uses `AV_CODEC_FLAG_LOW_DELAY` and a single thread to minimise latency.
- Resolution can change mid-stream (Tello sometimes resets the encoder); the swscale context is rebuilt on the fly.
- The Tello sends raw H.264 NAL units directly over UDP — there is no container format, RTP, or framing protocol.

---

## 3. Telemetry (`telemetry.c`)

Parses periodic state packets the drone sends on UDP port 8890.

**Data:**
```c
typedef struct {
    int bat, height, flight_time;
    int pitch, roll, yaw;
    int vgx, vgy, vgz;
    int templ, temph, tof;
    bool valid;
} Telemetry;
```

**Packet format:** semicolon-delimited, e.g.:
```
bat:85;h:120;time:45;pitch:0;roll:5;yaw:12;vgx:0;vgy:0;vgz:-10;templ:40;temph:45;tof:230
```

**Parsing:** `strtok_r()` to split on `;`, then `sscanf()` for `key:%d`.

**Threading:** Background thread writes under mutex; main thread reads a snapshot via `telemetry_get()`.

---

## 4. Input Processing (`input.c`)

Converts raw gamepad axes and keyboard state into integer RC commands.

**Stick processing:**
```
if |raw| < deadzone (0.15): output = 0
else: output = sign * ((|raw| - deadzone) / (1 - deadzone))^expo * 100
```
The expo curve (exponent 2.0) gives fine control near centre and full authority at extremes.

**Gamepad mapping:**
| Axis/Button | Action |
|-------------|--------|
| Left stick X | Roll (strafe left/right) |
| Left stick Y | Pitch (forward/backward, inverted) |
| Right stick X | Yaw (rotate, inverted) |
| Right stick Y | Throttle (up/down, inverted) |
| A | Takeoff |
| B | Land |
| X | Emergency stop |
| Y | Toggle hand tracking |
| LB / RB | Flip left / right |

**Keyboard mapping:** WASD for pitch/roll (±60%), arrow keys for throttle/yaw (±60%).

---

## 5. HUD Rendering (`hud.c`)

Draws all on-screen overlays using SDL2 renderer primitives and SDL2_ttf.

**Elements:**

| Area | Content |
|------|---------|
| Top bar | Battery % (colour-coded: green/yellow/red), flight time, temperature |
| Left panel | Velocity components (vgx, vgy, vgz) and magnitude |
| Right bar | Altitude gauge (filled bar to scale), TOF range |
| Centre | Attitude indicator (pitch/roll circle), compass arc (yaw) |
| Bottom bar | Connection status, input source, TOF readout |
| Debug panel | Tracking state, palm confidence, hand presence, gesture name |
| Hand overlay | 21 landmark dots (blue) + skeleton connections (white) |
| Palm box | Bounding rectangle — green when landmarks valid, orange otherwise |

**Font handling:** Searches system font directories for a monospace font (0xProto, DejaVu Sans Mono, Liberation Mono, Noto Sans Mono, FreeMono). Fonts are re-created on window resize so text scales properly.

**Colour coding:**
- Green = active / good
- Red = error / disconnected
- Yellow = warning / inactive
- Cyan = informational
- Orange = pending / caution

---

## 6. Main Loop (`main.c`)

The entry point and event loop. Ties everything together.

**Startup sequence:**
1. `drone_init()` → opens socket, sends `"command"` (enter SDK mode)
2. `video_start()` + `video_set_frame_callback(tracker_process_frame)`
3. `telemetry_start()`
4. `tracker_init()` → loads ONNX models
5. `drone_send("streamon")` → starts video stream
6. Create SDL window + renderer

**Event loop (runs until ESC or window close):**
1. Poll SDL events → handle keyboard/gamepad commands
2. Every 50 ms: check gesture commands (thumbs-down → land), read tracker or manual RC, send `drone_rc()`
3. Lock video frame → update SDL texture
4. Draw HUD → present

**RC multiplexing priority:**
1. Debounced gesture commands (thumbs-down → land)
2. Tracker PD output (if tracking active)
3. Gamepad input (if connected)
4. Keyboard input (fallback)

**Shutdown:** sends `"streamoff"` + `"land"`, joins all threads, frees resources.
