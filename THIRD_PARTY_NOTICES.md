# Third-party provenance and licenses

Keep original notices and license files at source. This summary is not a replacement for the actual license terms.

- `tzb_ws/src/OrbbecSDK_ROS2/`: upstream Orbbec ROS 2 wrapper, Apache-2.0 at the wrapper level, with `NOTICE` and individual `SDK/licenses/` files. Original precompiled SDK libraries are withheld pending per-component redistribution review.
- `Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/`: Livox ROS Driver2 (scoped MIT license and bundled RapidJSON notice). Original local Livox SDK2 shared library is **not** redistributed; install a compatible official SDK2 separately.
- `Nav2_rea_ws/src/localization/fast_lio/`: FAST-LIO, GPL-2.0 as represented by included `LICENSE`. Keep required source and license, and review derivative/combined-work obligations before public release.
- `Nav2_rea_ws/src/localization/point_lio/`: source license includes BSD-like retention/attribution requirements; its bundled `IKFoM/LICENSE` is preserved.
- `Nav2_rea_ws/src/localization/dlio/`: upstream DLIO MIT license. This snapshot includes locally modified working-tree files and must preserve attribution and obtain project-owner release authorization.
- `tzb_ws/src/best_openvino_model/`, `tzb_ws/src/openvino/`: original metadata indicates Ultralytics AGPL-3.0; model weights excluded pending model ownership/redistribution approval and licensing review.
- Other ROS project and organization-authored files may carry independent, unstated rights. No claim is made that all code has one permissive license.
