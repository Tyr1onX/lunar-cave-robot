#!/usr/bin/env python3
"""Run the repository's existing software-only chassis protocol audits."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CHECKS = (
    ("P5a 14-byte frame / firmware model", ROOT / "tools/p5a_offline/test_chassis_control_path.py", "-v"),
    ("P5b source/configuration contract", ROOT / "tools/p5b_validate_configuration.py"),
)


def main() -> int:
    for title, script, *arguments in CHECKS:
        print(f"=== {title} ===", flush=True)
        result = subprocess.run([sys.executable, str(script), *arguments], check=False)
        if result.returncode:
            print(f"FAIL {title}: exit {result.returncode}", flush=True)
            return result.returncode
    print("CHASSIS_PROTOCOL_CHECKS=PASS (software-only; no UART opened)", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
