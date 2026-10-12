#pragma once

/* 主机侧 Bsp_CAN 替身（仅供 tests/ 使用）。
   用途：others/inc/obj/motor_dm.hpp 里的 `#include "bsp_can.hpp"` 会先找同目录、再按
   -I 顺序找，把本文件所在目录排在 others/inc/bsp 之前，主机测试就能用**真实的**
   motor_dm.hpp（而不是 tests/stubs 里那份会与真实头文件同目录冲突的替身），
   同时不把 HAL/FDCAN 拖进来。
   另外把 tests/stubs/main.h 拉进来，roboarm.cpp 需要的 __get_PRIMASK / __disable_irq /
   __set_PRIMASK 由它提供（真实的 bsp_can.hpp 也是靠 fdcan.h → HAL 间接带进来的）。 */

#include <stdint.h>
#include "main.h"

struct Bsp_CAN {};
