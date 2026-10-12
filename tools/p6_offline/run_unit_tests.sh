#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="$REPO_ROOT/Nav2_rea_ws/src/p6_autonomous_mission${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s "$REPO_ROOT/Nav2_rea_ws/src/p6_autonomous_mission/test" -v
