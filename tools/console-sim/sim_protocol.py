"""Simulation-only contract; importable without ROS for safety regression tests."""
import math
import os

HOST, PORT, DOMAIN = "127.0.0.1", 8766, "217"
COMMAND, ODOM, MAP, SCAN = ("/console_sim/cmd_vel", "/console_sim/odom",
                            "/console_sim/map", "/console_sim/scan")
MAX_LINEAR, MAX_TURN = 0.24, 0.65


def require_isolation():
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("SIMULATION ONLY: require ROS_DOMAIN_ID=217 and ROS_LOCALHOST_ONLY=1")


def validate_command(body):
    if not isinstance(body, dict) or set(body) != {"token", "seq", "forward", "turn"}:
        raise ValueError("invalid command shape")
    token, seq, forward, turn = (body[k] for k in ("token", "seq", "forward", "turn"))
    if not isinstance(token, str) or len(token) != 32 or type(seq) is not int or not 1 <= seq <= 2**53:
        raise ValueError("invalid token or sequence")
    if type(forward) not in (float, int) or type(turn) not in (float, int):
        raise ValueError("invalid axes")
    if not (math.isfinite(forward) and math.isfinite(turn) and -1 <= forward <= 1 and -1 <= turn <= 1):
        raise ValueError("axis out of range")
    return token, seq, float(forward), float(turn)
