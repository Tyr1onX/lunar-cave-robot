# MID-360 NUC driver build record

Audit/build date: 2026-10-12

## Source and isolation

- Repository: `Tyr1onX/lunar-cave-robot`
- Baseline: `p4f/arm-trajectory-safety` at `eab6eef`; this is newer than P6 and contains the P5b and P6 history.
- Fix branch: `codex/mid360-driver-nuc-ready`
- Fix commit: `20fef12`
- NUC checkout: `/home/tzb/mid360_driver_dev/repo`
- Build, install, and logs: `/home/tzb/mid360_driver_dev/{build,install,log}`
- The existing `tzb_ws`, `Nav2_rea_ws`, and `nav_ws` were not used as build or install bases and were not changed.

The NUC could not validate GitHub's SSH host key, so the branch was fetched and
pushed from the Mac using its existing GitHub SSH configuration, then transferred
as a Git bundle. No credentials were copied. The NUC checkout's origin points to
the canonical GitHub SSH URL; NUC-side GitHub SSH access still needs its host key
to be trusted through the normal process before future fetches.

## Host and SDK

- Host: `tzb-NUC14RVS-B`, Ubuntu 22.04.5, kernel 6.8, ROS 2 Humble.
- CPU: Intel Core Ultra 9 185H; memory 14 GiB; root filesystem 916 GiB total / about 829 GiB free at audit.
- Livox SDK2 runtime API reports version `1.4.3`; headers also define 1.4.3.
- Shared library: `/usr/local/lib/liblivox_lidar_sdk_shared.so`
- Library SHA-256: `97f0663f71444b1fb6cf40baaa778f917bfc9716d187f01bc3b6b6bbdbcbaebe`
- SDK headers: `/usr/local/include/livox_lidar_api.h`, `livox_lidar_def.h`, and `livox_lidar_cfg.h`.
- SDK shared library dependencies resolve to the host's standard C/C++ runtime; no missing dependency was reported. SDK source checkout had no Git metadata, so its upstream commit/tag could not be established from the NUC.

## Build and checks

Build command used:

```bash
source /opt/ros/humble/setup.bash
cd ~/mid360_driver_dev/repo
colcon --log-base ~/mid360_driver_dev/log build \
  --base-paths Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2 \
  --packages-select livox_ros_driver2 \
  --build-base ~/mid360_driver_dev/build \
  --install-base ~/mid360_driver_dev/install
```

Result: `livox_ros_driver2` built successfully (one package; 8.62 seconds).
No system packages were installed.

- Package prefix: `/home/tzb/mid360_driver_dev/install/livox_ros_driver2`
- Executables found: `livox_ros_driver2`, `livox_ros_driver2_node`
- `ros2 launch --show-args livox_ros_driver2 mid360_static.launch.py` exposes `config_file` and its installed MID-360 default.
- `ldd` on `install/livox_ros_driver2/lib/liblivox_ros_driver2.so` resolves `liblivox_lidar_sdk_shared.so` to `/usr/local/lib`; no missing dependencies.
- The driver’s own summary/config parsers accepted both preserved NUC MID-360 JSON files (.119 and .147) and the branch's checked-in .119 configuration; each reported lidar type 8 and one lidar.
- Launch inspection and parser checks did not start a ROS driver process or access hardware.
- Build log: `/home/tzb/mid360_driver_dev/log/build_2026-10-12_09-21-58/`

## Configuration and field blockers

- Preserved `Nav2_rea_ws` MID-360 config: lidar `192.168.1.119`, host `192.168.1.50`.
- Preserved `nav_ws` MID-360 config: lidar `192.168.1.147`, host `192.168.1.50`.
- Current wired NIC `enp86s0` is down; Wi-Fi `wlo1` is `192.168.1.105/24`. The current route to `192.168.1.119` selects Wi-Fi. Tailscale is on `tailscale0`; keep it and Wi-Fi enabled during the test.
- Once the Ethernet cable is connected, verify that the actual sensor route uses `enp86s0` and the selected config's host IP. If not, stop; a narrow route change needs separate authorization.
- UFW service was active. Read-only `ufw status verbose` returned that root is required, so rules remain unverified and unchanged.
- The configs specify lidar-side UDP ports 56100, 56200, 56300, 56400, 56500 and host-side ports 56101, 56201, 56301, 56401, 56501. Confirm firewall policy and packet directions against the selected config before starting the driver.
- No real MID-360 was connected or powered, and no Livox ROS driver launch was run.

## Reproduction and live entry point

See [MID360_STATIC_ACCEPTANCE.md](MID360_STATIC_ACCEPTANCE.md) for package
checks, the driver-only launch command, route/firewall preflight, and point-cloud
plus IMU capture procedure. The launch entry point starts only
`livox_ros_driver2_node`.
