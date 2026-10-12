#!/usr/bin/env python3
"""Assert that cancel races a real planner response and prevents later commands."""
import json
import sys
from pathlib import Path

arm_log = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace").splitlines()
event_path = Path(sys.argv[2])
monitor_log = Path(sys.argv[3]).read_text(encoding="utf-8", errors="replace")
events = [json.loads(line) for line in event_path.read_text(encoding="utf-8").splitlines()
          if line.strip()]

solution = next((i for i, line in enumerate(arm_log)
                 if "Solution found in" in line), None)
cancel = next((i for i, line in enumerate(arm_log)
               if "P6 task cancellation barrier accepted" in line), None)
consumed = next((i for i, line in enumerate(arm_log)
                 if "move_arm_to_observe_before_perception_planned_final" in line), None)
assert solution is not None, "real planning_node did not produce an RRTConnect solution"
assert cancel is not None, "catch did not receive the P6 cancellation"
assert consumed is not None, "catch did not receive the async plan result"
assert solution < cancel < consumed, (
    "cancel did not land between planner solution and catch result handling: "
    f"solution={solution}, cancel={cancel}, result={consumed}")
assert any(event.get("event") == "ARM_CANCEL_CONFIRMED" for event in events), (
    "coordinator did not receive upstream quiescence acknowledgement")
assert "stream stayed quiet for 0.50s after acknowledgement" in monitor_log, (
    "post-cancel ROS command stream was not observed quiet")
assert "PASS cancelled worker accepted and completed a higher epoch follow-up task" in monitor_log, (
    "higher epoch follow-up task did not complete")
print("PASS real planner result/cancel race; no post-ack command and higher epoch takeover succeeded")
