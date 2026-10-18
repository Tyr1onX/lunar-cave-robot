#!/usr/bin/env bash
set -euo pipefail

env_or() {
  local name="$1" fallback="$2" value
  value="$(printenv "$name" 2>/dev/null || true)"
  if [[ -n "$value" ]]; then printf '%s' "$value"; else printf '%s' "$fallback"; fi
}
DOMAIN="$(env_or TZB_GZ_ROS_DOMAIN_ID 216)"
PARTITION="tzb-issue48-p2-$DOMAIN"
RUNTIME_DIR="$(env_or XDG_RUNTIME_DIR /tmp)"
PID_FILE="$(env_or TZB_GZ_PID_FILE "$RUNTIME_DIR/tzb_issue48_gazebo.pid")"
ROOT_PID=""
SIDS=()
PIDS=()

add_unique() {
  local value="$1" item
  for item in "${SIDS[@]}"; do [[ "$item" == "$value" ]] && return 0; done
  SIDS+=("$value")
}
if [[ -f "$PID_FILE" ]]; then
  candidate="$(cat "$PID_FILE")"
  if [[ "$candidate" =~ ^[0-9]+$ ]] && kill -0 "$candidate" 2>/dev/null; then
    cmd="$(tr '\0' ' ' < "/proc/$candidate/cmdline" 2>/dev/null || true)"
    if [[ "$cmd" == *"ros2 launch tzb_sim gazebo_grasp.launch.py"* ]]; then
      ROOT_PID="$candidate"
      root_sid="$(ps -o sid= -p "$candidate" 2>/dev/null | tr -d ' ')"
      [[ -n "$root_sid" ]] && add_unique "$root_sid"
    else
      echo "Ignoring stale P2 pid file entry $candidate; process identity does not match. Continuing with partition-scoped cleanup." >&2
      rm -f "$PID_FILE"
    fi
  else
    rm -f "$PID_FILE"
  fi
fi

# Gazebo Fortress starts server and GUI processes in separate process groups.
# Discover only processes carrying this simulation's unique transport partition.
for env_path in /proc/[0-9]*/environ; do
  candidate="${env_path#/proc/}"
  candidate="${candidate%/environ}"
  [[ "$candidate" =~ ^[0-9]+$ ]] || continue
  if grep -aFq "GZ_PARTITION=$PARTITION" "$env_path" 2>/dev/null; then
    cmd="$(tr '\0' ' ' < "/proc/$candidate/cmdline" 2>/dev/null || true)"
    if [[ "$cmd" == *"ign gazebo"* ]]; then
      sid="$(ps -o sid= -p "$candidate" 2>/dev/null | tr -d ' ')"
      if [[ -n "$sid" ]]; then add_unique "$sid"; fi
    fi
  fi
done
for sid in "${SIDS[@]}"; do
  while read -r pid proc_sid; do
    if [[ "$proc_sid" == "$sid" && "$pid" =~ ^[0-9]+$ ]]; then
      PIDS+=("$pid")
    fi
  done < <(ps -eo pid=,sid=)
done
if [[ -n "$ROOT_PID" ]]; then
  PIDS+=("$ROOT_PID")
fi
if [[ ${#PIDS[@]} -eq 0 ]]; then
  echo "No isolated P2 Gazebo processes remain."
  exit 0
fi

# Signal the launch group and every process in each confirmed P2 session.
if [[ -n "$ROOT_PID" ]]; then kill -INT -- "-$ROOT_PID" 2>/dev/null || true; fi
for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
sleep 4
for pid in "${PIDS[@]}"; do
  sid_now="$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "$sid_now" && " ${SIDS[*]} " == *" $sid_now "* ]]; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
sleep 3
for pid in "${PIDS[@]}"; do
  sid_now="$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "$sid_now" && " ${SIDS[*]} " == *" $sid_now "* && "$state" != Z* ]]; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
done
rm -f "$PID_FILE"
echo "Stopped isolated P2 Gazebo session(s): ${SIDS[*]:-(none)}; hardware drivers were not started."
