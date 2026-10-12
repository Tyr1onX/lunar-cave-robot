$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $taskRoot
try {
    New-Item -ItemType Directory -Path build/host-tests -Force | Out-Null
    & g++ -std=c++17 -O2 -Wall -Wextra -Werror -Iothers/inc/algorithm tests/motion_planning_test.cpp -o build/host-tests/motion_planning_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motion planner test compilation failed' }
    & ./build/host-tests/motion_planning_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motion planner tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -Iothers/inc/app -c others/src/app/Remote.c -o build/host-tests/remote.o
    if ($LASTEXITCODE -ne 0) { throw 'Remote C compilation failed' }
    & g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itests/stubs -Iothers/inc/obj -Iothers/inc/algorithm -Iothers/inc/app tests/joint_remote_test.cpp others/src/obj/roboarm.cpp build/host-tests/remote.o -o build/host-tests/joint_remote_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Joint integration test compilation failed' }
    & ./build/host-tests/joint_remote_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Joint integration tests failed' }
}
finally {
    Pop-Location
}
