#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 IMAGE MODEL_DIR (authorized local files, not provided in Git)" >&2
  exit 2
fi
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT
export YOLO_CONFIG_DIR="$TMP_DIR/config" OV_CACHE_DIR="$TMP_DIR/cache"
mkdir -p "$YOLO_CONFIG_DIR" "$OV_CACHE_DIR"
"${YOLO_NODE_PYTHON:-python3}" "$WS/tools/p2_offline/yolo_rgb_sample.py" "$1" "$2" "$TMP_DIR"
