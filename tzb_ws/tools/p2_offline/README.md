# Software-only P2 validation

Build `tzb_ws` as described in the repository README and run `./tools/p2_offline/run.sh` here. The launch file uses synthetic joint feedback and localhost-only `/p2_offline/*` topics, with no camera or hardware drivers. Ctrl-C stops it. This does not validate real hardware or calibration.

To test CPU model inference with files you have permission to use, activate a compatible Python environment and run `./tools/p2_offline/run_yolo_rgb_sample.sh IMAGE MODEL_DIR`. No sample frames or model weights are distributed here.
