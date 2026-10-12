#!/usr/bin/env python3
"""Check the archived firmware trees and the P7/P8 safety entry points."""

from __future__ import annotations

from pathlib import Path
import hashlib
import re
import sys
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

REQUIRED = {
    "arm": [
        "lunar_rover.ioc",
        "CMakeLists.txt",
        "CMakePresets.json",
        "MDK-ARM/lunar_rover.uvprojx",
        "MDK-ARM/startup_stm32h723xx.s",
        "MDK-ARM/lunar_rover/lunar_rover.sct",
        "STM32H723xG_flash.ld",
        "Core/Src/main.c",
        "Core/Src/stm32h7xx_it.c",
        "Core/Src/stm32h7xx_hal_msp.c",
        "Drivers/STM32H7xx_HAL_Driver/LICENSE.txt",
        "Middlewares/Third_Party/FreeRTOS/Source/LICENSE",
        "Middlewares/Third_Party/LibXR/CMakeLists.txt",
        "cmake/LibXR.CMake",
        "User/app_main.cpp",
    ],
    "chassis": [
        "CtrlBoard-H7_IMU.ioc",
        "CMakeLists.txt",
        "CMakePresets.json",
        "MDK-ARM/CtrlBoard-H7_IMU.uvprojx",
        "MDK-ARM/CtrlBoard-H7_IMU/CtrlBoard-H7_IMU.sct",
        "Core/Startup/startup_stm32h723xx.s",
        "STM32H723VGTX_FLASH.ld",
        "Core/Src/main.c",
        "Core/Src/stm32h7xx_it.c",
        "Core/Src/stm32h7xx_hal_msp.c",
        "User/APP/Chaiss_Communication.c",
        "User/Bsp/can_bsp.c",
        "Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a",
        "Drivers/CMSIS/DSP/Lib/ARM/arm_cortexM7lfdp_math.lib",
        "Middlewares/Third_Party/FreeRTOS/Source/LICENSE",
    ],
}

errors: list[str] = []
warnings: list[str] = []
artifact_suffixes = {
    ".elf", ".axf", ".hex", ".bin", ".map", ".o", ".obj", ".pyc", ".crf", ".dep", ".log"
}
personal_path_patterns = re.compile(
    r"(?:C:/Users/|D:/New_Start|/Users/[^/\s]+/|/home/(?:qibai|tzb)/)", re.IGNORECASE
)

for module, required in REQUIRED.items():
    tree = ROOT / "firmware" / module
    missing = [item for item in required if not (tree / item).is_file()]
    if missing:
        errors.append(f"firmware/{module}: missing {', '.join(missing)}")
        continue
    files = [path for path in tree.rglob("*") if path.is_file()]
    print(f"PASS firmware/{module}: {len(files)} files, {sum(p.stat().st_size for p in files)} bytes")
    if any(".git" in path.parts for path in files):
        errors.append(f"firmware/{module}: embedded Git metadata found")
    project_rel = "MDK-ARM/lunar_rover.uvprojx" if module == "arm" else "MDK-ARM/CtrlBoard-H7_IMU.uvprojx"
    try:
        project = ET.parse(tree / project_rel).getroot()
        unresolved = []
        for node in project.iter():
            if not node.tag.lower().endswith("filepath") or not node.text:
                continue
            source = Path(node.text.strip().replace("\\", "/"))
            candidates = ((tree / Path(project_rel).parent / source).resolve(), (tree / source).resolve())
            if not any(candidate.is_file() for candidate in candidates):
                unresolved.append(node.text.strip())
        if unresolved:
            errors.append(f"firmware/{module}: Keil project references missing files: {unresolved[:8]}")
        else:
            print(f"PASS firmware/{module}: all Keil FilePath references resolve")
    except (ET.ParseError, OSError) as error:
        errors.append(f"firmware/{module}: cannot inspect Keil project {project_rel}: {error}")
    artifacts = [path for path in files if path.suffix.lower() in artifact_suffixes]
    if artifacts:
        errors.append(f"firmware/{module}: generated artifacts found: {artifacts[:5]}")
    for path in files:
        if path.suffix.lower() not in {".cmake", ".yaml", ".yml", ".ioc", ".xml", ".md", ".txt", ".uvprojx", ".sct"}:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if personal_path_patterns.search(text):
            errors.append(f"{path.relative_to(ROOT)}: machine-local absolute path found")

for workspace, marker in [
    ("tzb_ws", "src/lite_motion_planner/package.xml"),
    ("Nav2_rea_ws", "src/p6_autonomous_mission/package.xml"),
]:
    if not (ROOT / workspace / marker).is_file():
        errors.append(f"{workspace}: expected P8 package missing: {marker}")
    else:
        print(f"PASS {workspace}: expected P8 package present")

for relative in [
    "tools/p4e_offline/test_arm_host_safety_pty.py",
    "tools/p4f_offline/validate.sh",
    "tools/p7_offline/run.sh",
    "tools/p7_offline/validate_scenarios.sh",
    "docs/P8_VALIDATION_REPORT.md",
]:
    if not (ROOT / relative).is_file():
        errors.append(f"P7/P8 validation entry missing: {relative}")
    else:
        print(f"PASS preserved P7/P8 entry: {relative}")

generated = [
    ROOT / "firmware/arm/User/xrobot_main.hpp",
    ROOT / "firmware/arm/Modules/CMakeLists.txt",
]
missing_generated = [str(path.relative_to(ROOT)) for path in generated if not path.is_file()]
if missing_generated:
    warnings.append("Arm CMake requires missing XRobot generated files: " + ", ".join(missing_generated))

manifest = ROOT / "firmware/SOURCE_SHA256SUMS"
if not manifest.is_file():
    errors.append("firmware/SOURCE_SHA256SUMS is missing")
else:
    listed: set[str] = set()
    manifest_entries: dict[str, str] = {}
    for line_number, line in enumerate(manifest.read_text(encoding="utf-8").splitlines(), 1):
        digest, separator, relative = line.partition("  ")
        if not separator or not re.fullmatch(r"[0-9a-f]{64}", digest):
            errors.append(f"firmware/SOURCE_SHA256SUMS:{line_number}: malformed entry")
            continue
        path = ROOT / relative
        listed.add(relative)
        manifest_entries[relative] = digest
        if not path.is_file():
            errors.append(f"checksum source missing: {relative}")
    actual = {
        path.relative_to(ROOT).as_posix()
        for module in ("arm", "chassis")
        for path in (ROOT / "firmware" / module).rglob("*")
        if path.is_file()
    }
    if actual != listed:
        errors.append(
            "checksum inventory differs from firmware tree: "
            f"unlisted={sorted(actual - listed)[:5]}, stale={sorted(listed - actual)[:5]}"
        )
    else:
        index = subprocess.run(
            ["git", "ls-files", "-s", "-z", "--", "firmware/arm", "firmware/chassis"],
            cwd=ROOT, check=True, stdout=subprocess.PIPE,
        ).stdout
        entries = []
        for record in index.split(b"\0"):
            if not record:
                continue
            metadata, raw_path = record.split(b"\t", 1)
            mode, object_id, stage = metadata.split()
            entries.append((raw_path.decode("utf-8"), object_id.decode("ascii")))
        blobs = subprocess.run(
            ["git", "cat-file", "--batch"], cwd=ROOT, check=True,
            input=b"".join(object_id.encode("ascii") + b"\n" for _, object_id in entries),
            stdout=subprocess.PIPE,
        ).stdout
        cursor = 0
        tracked_hashes: dict[str, str] = {}
        for relative, object_id in entries:
            header_end = blobs.index(b"\n", cursor)
            header = blobs[cursor:header_end].split()
            size = int(header[2])
            start = header_end + 1
            content = blobs[start:start + size]
            tracked_hashes[relative] = hashlib.sha256(content).hexdigest()
            cursor = start + size + 1
        for relative, digest in manifest_entries.items():
            if tracked_hashes.get(relative) != digest:
                errors.append(f"tracked checksum mismatch: {relative}")
        if set(tracked_hashes) != listed:
            errors.append("checksum inventory differs from Git index")
        if subprocess.run(
            ["git", "diff", "--quiet", "--", "firmware/arm", "firmware/chassis"],
            cwd=ROOT,
        ).returncode != 0:
            errors.append("firmware worktree differs from the Git index")
        print(f"PASS firmware SHA-256 manifest: {len(listed)} files")

if warnings:
    for item in warnings:
        print("WARN " + item)
if errors:
    for item in errors:
        print("FAIL " + item, file=sys.stderr)
    raise SystemExit(1)
print("PASS project asset structure checks")
