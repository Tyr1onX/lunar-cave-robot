#!/usr/bin/env bash
set -euo pipefail
PID_FILE="${TZB_SIM_PID_FILE:-${XDG_RUNTIME_DIR:-/tmp}/tzb_sim_offline_demo.pid}"
if [[ ! -f "$PID_FILE" ]]; then
  echo "No tzb_sim demo PID file at $PID_FILE"
  exit 0
fi
PID="$(cat "$PID_FILE")"
if [[ ! "$PID" =~ ^[0-9]+$ ]] || ! kill -0 "$PID" 2>/dev/null; then
  rm -f "$PID_FILE"
  echo "Removed stale tzb_sim PID file"
  exit 0
fi
CMD="$(tr '\0' ' ' < "/proc/$PID/cmdline" 2>/dev/null || true)"
if [[ "$CMD" != *"ros2 launch tzb_sim demo.launch.py"* ]]; then
  echo "Refusing to signal pid $PID because it is not the tzb_sim demo." >&2
  exit 1
fi
PGID="$(ps -o pgid= -p "$PID" 2>/dev/null | tr -d ' ')"
if [[ "$PGID" != "$PID" ]]; then
  echo "Refusing to signal pid $PID because it is not the leader of its own session." >&2
  exit 1
fi
# Stop the synthetic camera first so YOLO and pose callbacks drain before ROS teardown.
SCENE_PID="$(pgrep -g "$PGID" -f '/tzb_sim/lib/tzb_sim/blue_block_scene' | head -n 1 || true)"
if [[ "$SCENE_PID" =~ ^[0-9]+$ && "$SCENE_PID" != "$PID" ]]; then
  /bin/kill -INT "$SCENE_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "$SCENE_PID" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  sleep 0.6
fi
/bin/kill -INT -- "-$PID"
for _ in $(seq 1 40); do
  STATE="$(ps -o stat= -p "$PID" 2>/dev/null | tr -d ' ' || true)"
  if [[ -z "$STATE" || "$STATE" == Z* ]]; then
    rm -f "$PID_FILE"
    echo "Stopped tzb_sim demo process group $PGID"
    exit 0
  fi
  sleep 0.5
done
/bin/kill -TERM -- "-$PID" 2>/dev/null || true
rm -f "$PID_FILE"
echo "Sent SIGTERM to the tzb_sim process group after waiting (pgid $PGID)"
