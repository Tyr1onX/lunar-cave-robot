# Third-party provenance and licenses

Keep original notices and license files at source. This summary is not a replacement for the actual license terms.

- `tzb_ws/src/OrbbecSDK_ROS2/`: upstream Orbbec ROS 2 wrapper, Apache-2.0 at the wrapper level, with `NOTICE` and individual `SDK/licenses/` files. `orbbec_camera/CMakeLists.txt` carries a project-local SDK library path/cache discovery fix; upstream notices and source attribution are retained. Original precompiled SDK libraries are withheld pending per-component redistribution review.
- `Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/`: Livox ROS Driver2 (scoped MIT license and bundled RapidJSON notice). Original local Livox SDK2 shared library is **not** redistributed; install a compatible official SDK2 separately.
- `Nav2_rea_ws/src/localization/fast_lio/`: FAST-LIO, GPL-2.0 as represented by included `LICENSE`. Keep required source and license, and review derivative/combined-work obligations before public release.
- `Nav2_rea_ws/src/localization/point_lio/`: source license includes BSD-like retention/attribution requirements; its bundled `IKFoM/LICENSE` is preserved.
- `Nav2_rea_ws/src/localization/dlio/`: upstream DLIO MIT license. This snapshot includes locally modified working-tree files and must preserve attribution and obtain project-owner release authorization.
- `tzb_ws/src/best_openvino_model/`, `tzb_ws/src/openvino/`: original metadata indicates Ultralytics AGPL-3.0; model weights excluded pending model ownership/redistribution approval and licensing review.
- Other ROS project and organization-authored files may carry independent, unstated rights. No claim is made that all code has one permissive license.

## STM32 firmware sources archived in P9

- `firmware/arm/Drivers/CMSIS/` and `firmware/chassis/Drivers/CMSIS/`: Arm CMSIS and ST device headers with the original Apache-2.0 notice files retained.
- `firmware/arm/Drivers/STM32H7xx_HAL_Driver/` and `firmware/chassis/Drivers/STM32H7xx_HAL_Driver/`: ST STM32H7 HAL sources with the original ST license references retained.
- `firmware/arm/Middlewares/ST/STM32_USB_Device_Library/`: ST USB Device Library with its SLA0044 notice retained.
- `firmware/arm/Middlewares/Third_Party/FreeRTOS/` and `firmware/chassis/Middlewares/Third_Party/FreeRTOS/`: FreeRTOS source with the original MIT license retained.
- `firmware/arm/Middlewares/Third_Party/LibXR/`: vendored LibXR snapshot from upstream commit `783b157fb3e9e5a7801edc9083834ba3ac49801f`; its Apache-2.0 license is retained. The bundled Eigen headers retain MPL-2.0 and their BSD/Apache/MINPACK notices. The optional Doxygen CSS documentation asset was excluded; its upstream tracking token was not retained.
- `firmware/chassis/Drivers/CMSIS/DSP/Lib/`: the GCC and ARM CMSIS-DSP static libraries required by the checked-in CMake and Keil project settings are retained with CMSIS licensing. Other architecture variants and CMSIS examples are excluded.

These original notices remain beside the corresponding third-party source. This summary does not replace them or grant permission to publish the whole repository publicly. The original HM-series motor manual found inside the chassis archive was not added because the manufacturer's redistribution terms were not established; its filename and SHA-256 are recorded in `docs/PROJECT_ASSETS.md`.
