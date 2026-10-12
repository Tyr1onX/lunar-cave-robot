$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $taskRoot
try {
    New-Item -ItemType Directory -Path build/host-tests -Force | Out-Null
    # -Itests/stubs_can 必须排在 others/inc/bsp 之前：
    # others/inc/obj/motor_dm.hpp 里的 `#include "bsp_can.hpp"` 先找同目录、再按 -I 顺序找，
    # 让主机侧替身先命中，就能编译**真实的** motor_dm.hpp（而不是 tests/stubs 里那份与真实
    # 头文件同目录冲突、实际永远命中不了的替身），又不把 HAL/FDCAN 拖进来。
    # 注意 -Itests/stubs 提供 main.h（__get_PRIMASK / __disable_irq / __set_PRIMASK）。
    & g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itests/stubs_can -Itests/stubs -Iothers/inc/obj -Iothers/inc/algorithm tests/joint_stream_test.cpp others/src/obj/roboarm.cpp -o build/host-tests/joint_stream_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Streaming target test compilation failed' }
    & ./build/host-tests/joint_stream_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Streaming target tests failed' }
}
finally {
    Pop-Location
}
