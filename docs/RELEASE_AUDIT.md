# Pre-publication audit — 2026-10-11

Status: **private repository import authorized; not cleared for public release**. Staging: `~/lunar-cave-robot-staging/` on ThinkBook; NUC unchanged.

## Source snapshots

- `tzb_ws/` from branch `p2b/real-vision-data`, commit `b5bd744` (P0, P1, P2, P2b fixes carried as working-tree content, not Git history).
- `Nav2_rea_ws/` from branch `p0/offline-build`, commit `081ba74`.
- `Nav2_rea_ws/src/localization/dlio/` copies the **modified working tree**, not its nested Git directory or history (upstream `b446a3d`); it includes the restored local changes.
- `p1_offline/` includes only the Python probes; outdated absolute-path P1 YAML removed; excludes run logs and transient helper scripts.
- P2/P2b scripts under `tzb_ws/tools/` retained; README reflects incomplete real-sensor verification.

## P3–P4b private archive update

The integration branch imports only the four source patches in dependency order (P3 `f91bc6a`, P3b `e402434`, P4a `67359bd`, P4b `9d48d5d`), creating new commits from the current private `main`; the original Git history is not imported. See [P3–P4b validation](P3_P4B_VALIDATION.md) for test commands, observed results, and remaining hardware boundaries. Navigation contents were not changed.

The original STM32 firmware project and any firmware ZIP remain outside Git: permission to redistribute, third-party terms, and compiled firmware artifacts have not been cleared. The upper-computer protocol tests are not hardware acceptance evidence. Existing public-release ownership and license blockers below remain unchanged.

## Intentionally excluded

- All `.git` data, virtual environments, build/install/log outputs, Python caches, PID files.
- Local RGB-D bags, private PCD/PGM maps and map scans, large upstream demonstration images and saved Orbbec sample frames.
- Model `.bin` weights (`best.bin`, `yoloe-26l-seg.bin` and other OpenVINO binaries); their `*.xml` architecture and sanitized metadata may remain.
- Bundled Orbbec SDK `SDK/lib` precompiled objects; locally installed Livox SDK `vendor/livox_sdk` binaries.
- Any files not explicitly in the source workspaces or approved offline test resources.

## Open risks before publishing

1. **Ownership / permission:** NUC-recovered team code and DLIO modifications require project-owner publication approval.
2. **Licensing:** mixed GPL-2.0, Apache-2.0, MIT, BSD-like, AGPL-3.0; verify source-level attributions and distribution obligations. The absence of a repository-wide LICENSE is intentional.
3. **Binary provenance:** Orbbec proprietary/redistributable subcomponents and Ultralytics trained models are withheld.
4. **Portability:** legacy hardware-facing YAML/C++ contains previous-machine absolute paths; P2 offline launch resolves package paths, but real robot bringup requires separate review.
5. **Completeness:** P2b has no approved real-world camera bag, TF/calibration evidence, or closed-loop arm acceptance. Missing `tzb_state_test` prevents some runtime integration.
6. **Reproducibility:** full 14+18 package builds require hardware SDK dependencies omitted from this publishable snapshot. Verify safe offline subset separately.

**Do not make this repository public before resolving ownership, licenses, model rights and legacy configuration risks.**

## Local staging verification

- Arm: 13 packages compiled; the Orbbec camera driver was excluded because its binary SDK is not distributed. The pose estimator's optional camera runtime dependency was corrected in staging only.
- Navigation: 14 packages compiled; the Livox driver, FAST-LIO, Point-LIO and pointcloud_tools were excluded pending a separately supplied Livox SDK.
- Safe P2 test: synthetic grasp, detach, placement and return to observe position completed on ROS domain 187 with localhost-only transport. The process was then intentionally interrupted.
- Python and Bash syntax checks passed. No physical robot or sensor driver was activated.
- At least 24 source files still contain prior-machine absolute path references. Hardware bringup is not portable or approved for unattended operation.
- Exact candidate inventory and checksums are stored outside Git in the ThinkBook home directory, as lunar-cave-robot-preupload-manifest.txt and lunar-cave-robot-preupload-sha256.txt.
