#!/usr/bin/env bash
# Download MediaPipe hand-tracking ONNX models from PINTO0309's model zoo (Wasabi storage).
# Output: models/palm_detection_lite.onnx
#         models/hand_landmark_lite.onnx
#
# The models live inside a nested tarball:
#   resources.tar.gz
#     └── 033_Hand_Detection_and_Tracking/30_batchN_post-process_marged/post_process_marged.tar.gz
#           ├── onnx_parts/palm_detection_lite_192x192.onnx
#           └── hand_landmark_lite_1x3x224x224.onnx
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$SCRIPT_DIR/../models"
mkdir -p "$MODELS_DIR"

if [[ -f "$MODELS_DIR/palm_detection_lite.onnx" && -f "$MODELS_DIR/hand_landmark_lite.onnx" ]]; then
    echo "Models already present in $MODELS_DIR"
    exit 0
fi

WASABI_URL="https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/033_Hand_Detection_and_Tracking/resources.tar.gz"
INNER="033_Hand_Detection_and_Tracking/30_batchN_post-process_marged/post_process_marged.tar.gz"

echo "Downloading archive (~700 MB) ..."
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

curl -fL --retry 3 -o "$TMP/resources.tar.gz" "$WASABI_URL"

echo "Extracting models..."
# Extract the inner tarball
tar -xzf "$TMP/resources.tar.gz" -C "$TMP" "$INNER"
INNER_TGZ="$TMP/$INNER"

# Palm detector (inside onnx_parts/)
tar -xzf "$INNER_TGZ" --strip-components=1 -C "$MODELS_DIR" \
    "onnx_parts/palm_detection_lite_192x192.onnx"
mv "$MODELS_DIR/palm_detection_lite_192x192.onnx" "$MODELS_DIR/palm_detection_lite.onnx"

# Hand landmark (at root of inner tarball)
tar -xzf "$INNER_TGZ" -C "$MODELS_DIR" "hand_landmark_lite_1x3x224x224.onnx"
mv "$MODELS_DIR/hand_landmark_lite_1x3x224x224.onnx" "$MODELS_DIR/hand_landmark_lite.onnx"

echo ""
echo "Models ready:"
ls -lh "$MODELS_DIR"/*.onnx
