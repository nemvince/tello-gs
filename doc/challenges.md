# Challenges & Quirks

Things that were hard, surprising, or non-obvious during development.

## The Normalisation Bug

**The single most impactful bug in the project.**

The palm detection and landmark models were initially preprocessed with the standard MediaPipe normalisation: `v / 127.5 - 1.0` (mapping pixel values to [-1, 1]). This produced a maximum palm confidence of ~0.42 — just below the 0.5 detection threshold — so *no hand was ever detected*.

The fix was switching to `v / 255.0` (mapping to [0, 1]). This immediately raised palm confidence to 0.96+ and produced strong, reliable detections.

This was discovered by writing a Python diagnostic script that tested both normalisations against the same model and input image. The [-1, 1] variant consistently failed; [0, 1] consistently succeeded. The PINTO0309 model zoo variants expect [0, 1], which differs from the standard MediaPipe TFLite models that expect [-1, 1].

**Lesson:** Never assume normalisation conventions — validate against the actual model with known inputs.

## SSD Anchor Generation Order

MediaPipe's SSD anchor calculator uses a specific traversal order: y-outer, x-inner (row-major over the feature map grid). Getting this wrong produces anchors that don't align with the model's regressor outputs, causing all decoded boxes to be in the wrong locations even when confidence scores are correct.

The anchor configuration also has a subtle detail: there are 4 declared layers with strides [8, 16, 16, 16], but strides 16 share a single 12×12 grid — they contribute 6 anchors per cell (3 layers × 2 per layer), not 3 separate 12×12 grids.

## Regressor Coordinate Swap

The palm detection regressor layout follows MediaPipe's convention where index 0 is **y-centre** and index 1 is **x-centre** (not x, y as one might assume). The same applies to width/height: index 2 is height, index 3 is width. Getting these swapped produces detections that are transposed across the diagonal.

## Ghost Lock (Re-Tracking Feedback Loop)

When re-tracking was first implemented, the system would frequently "lock on" to empty parts of the frame and never let go. The cause was a feedback loop:

1. Previous landmarks exist → compute crop → feed to landmark model
2. Landmark model says "maybe a hand" (presence 0.55, above 0.5 threshold)
3. System accepts these landmarks as valid → saves them for next frame
4. Next frame: repeat from step 1

The fix was a **dual presence threshold**: 0.5 for fresh palm detections (where the palm detector already provides a high-confidence region) and 0.8 for re-tracked crops (where the region is based on unverified extrapolation). This breaks the feedback loop — the model must be genuinely confident the crop still contains a hand.

A second contributing factor was that the EMA-smoothed landmarks were being saved for re-tracking. The smoothing caused the crop to drift gradually, keeping the model's attention on a slowly-moving ghost region. Saving **raw** landmarks (before smoothing) fixed this.

## H.264 over Raw UDP

The Tello streams H.264 video as raw NAL units directly over UDP — no RTP, no RTSP, no container format. This means:

- There's no framing protocol — FFmpeg's `av_parser_parse2()` must reconstruct NAL boundaries from arbitrary UDP packet boundaries.
- Packet loss causes decoder errors with no recovery mechanism other than waiting for the next keyframe.
- The encoder sometimes resets mid-stream, changing resolution. The decoder pipeline handles this by detecting resolution changes and rebuilding the swscale context.
- The receive buffer is set to 512 KB to reduce packet drops during decode stalls.

## Thread Safety of the Frame Handoff

The video thread decodes a frame and needs to both (a) make it available to the main thread for rendering and (b) signal the tracker worker. The frame callback `tracker_process_frame()` is called while the video frame lock is held, ensuring the tracker gets a consistent snapshot before the video thread moves to the next frame.

The tracker uses its own double-buffer and condition variable, separate from the video module's lock, so the video thread is never blocked by slow inference.

## PD Controller Tuning

The PD gains were tuned empirically while flying. Key observations:

- **Derivative term is essential** — without it, the drone oscillates around the target position because the Tello has significant momentum. The derivative acts as damping.
- **Pitch (distance) needs to be much gentler** than roll/throttle — the palm-size signal is noisy (small changes in hand angle cause large apparent size changes), and aggressive forward/backward motion is dangerous. The pitch axis uses half the gains and is hard-capped at ±30.
- **Edge clamping is not optional** — without it, when the hand leaves the frame, the last valid centroid is near the edge, producing a large error that sends the drone flying in that direction with no hand to bring it back.
- **Deadzone on pitch** prevents the drone from constantly hunting forward/backward in hover. Small palm-size fluctuations from hand micro-movements and landmark jitter would otherwise cause visible bobbing.

## Gesture Misclassification

Early gesture detection used simple Y-coordinate comparisons (`tip.y < pip.y` means finger is extended). This worked when the hand was upright but failed completely when the hand was rotated, tilted, or viewed from an angle.

The fix was switching to **angle-based detection**: compute the actual joint angle at PIP (for fingers) or MCP (for thumb) using the dot product of bone vectors. A joint angle > 120° means the finger is straight. This is fully rotation-invariant.

The thumb required a separate, looser threshold (cos < -0.3 instead of -0.5) because the thumb's natural range of motion is much smaller — a "fully extended" thumb often only reaches ~110-120°.

## ONNX Runtime C API

The ONNX Runtime C API is functional but verbose. Every operation returns an `OrtStatus*` that must be checked and released. Tensor creation requires explicit shape arrays, memory info objects, and careful lifetime management. A typical inference call involves ~15 API calls.

The `ORT_CHECK` macro wraps this pattern to keep the code readable, but errors are logged and execution continues (best-effort) rather than aborting, since a failed inference frame can simply be skipped.

## The Tello's Wi-Fi Limitations

The Tello creates its own Wi-Fi access point. This means:
- The ground station machine loses internet connectivity while connected.
- Only one client can connect at a time.
- Wi-Fi latency adds ~20-50 ms to the video pipeline on top of encode/decode time.
- Range is limited to ~30 metres line-of-sight before packet loss becomes severe.

The `SO_RCVTIMEO=2s` timeout on all sockets prevents threads from blocking forever if the drone disconnects or goes out of range.
