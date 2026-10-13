# V0.3a SIMULATION ONLY

Explicitly launch the **existing** P6 `cave_simulator.py` inside a dedicated isolated ROS process, never the real robot stack. Requires ROS 2 Humble on NUC. No dependency on `tzb_ws`, `Nav2_rea_ws` overlays, serial, CAN, device drivers, or the production read-only Agent.

On NUC, after copying this repository or **only** `tools/console-sim` plus `Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission`, run:

```sh
cd ~/lunar-cave-robot
bash tools/console-sim/run.sh
```

The runner sets `ROS_DOMAIN_ID=217`, `ROS_LOCALHOST_ONLY=1`, publishes to `/console_sim/cmd_vel` and subscribes to isolated `/console_sim/odom`, `/console_sim/map`. Fixed loopback `127.0.0.1:8766` only, forwarded to Mac via separate SSH `127.0.0.1:18766`. The Mac app does NOT automatically start the simulator. The normal Console read-only Agent remains unchanged on 8765.

In Mac Console choose **模拟驾驶**; click the keyboard capture surface; use WASD. Key release sends zero once; losing focus sends release; server and P6 simulator each enforce a stop after lost commands (P6: 0.25 s). SIMULATION means mathematical 2D diff-drive, not a validated physical six-wheel model. Map, position, speed and trail are from the simulator's ROS topics only. The GUI never integrates a second kinematic model.

HTTP interface (localhost only): `GET /v1/simulation/{state|map}`, `POST /v1/simulation/{claim|command|release}`. Single active random token, increasing sequence number and clamped axes; rejects unknown endpoints and bodies. No generic topic selector, shell command or physical controls.

## Safety check

```sh
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 python3 tools/console-sim/sim_server.py  # must refuse
```

The source `cave_simulator.py` defaults to /cmd_vel for unrelated P6 usage, but **this launcher overrides** all command and telemetry topic names and explicitly verifies them before accepting HTTP. This repo change does not alter the P6 implementation, firmwares, production Agent, or ROS middleware system configuration.
