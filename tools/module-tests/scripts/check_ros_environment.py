#!/usr/bin/env python3
"""Read-only ROS environment check for the isolated module test runner."""

from __future__ import annotations

import importlib.util
import os
import shutil
import sys


def main() -> int:
    failures: list[str] = []
    distro = os.environ.get("ROS_DISTRO")
    print(f"ROS_DISTRO={distro or 'unset'}")
    if distro != "humble":
        failures.append("ROS_DISTRO must be humble")

    ros2 = shutil.which("ros2")
    print(f"ros2={ros2 or 'not found in PATH'}")
    if not ros2:
        failures.append("ros2 executable is not available in PATH")

    for module in ("rclpy", "ament_index_python"):
        available = importlib.util.find_spec(module) is not None
        print(f"python_module.{module}={'available' if available else 'missing'}")
        if not available:
            failures.append(f"Python module {module} is unavailable")

    local_only = os.environ.get("ROS_LOCALHOST_ONLY")
    domain_id = os.environ.get("ROS_DOMAIN_ID")
    print(f"ROS_LOCALHOST_ONLY={local_only or 'unset'}")
    print(f"ROS_DOMAIN_ID={domain_id or 'unset'}")
    if local_only != "1":
        failures.append("ROS_LOCALHOST_ONLY must be 1 for isolated tests")
    try:
        domain = int(domain_id or "")
        if not 0 <= domain <= 232:
            raise ValueError
    except ValueError:
        failures.append("ROS_DOMAIN_ID must be an integer between 0 and 232")

    print(f"python={sys.version.split()[0]}")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("ROS environment check passed (software-only, localhost isolated)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
