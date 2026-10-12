# USB 启动与枚举

## 启动与时钟

- 真正执行的启动任务是 `others/src/thread/task_start.cpp` 中的 `StartDefaultTask()`，它覆盖 `freertos.c` 的弱定义：按“任务初始化总表”依次做微秒时基、FDCAN3 过滤/启动、`MX_USB_DEVICE_Init()`，然后自杀。各任务体（`roboARM_task` / `Host_Task` / `Error_tasks`）不做初始化。
- `defaultTask` 优先级为 `osPriorityNormal`（高于其它任务），所以初始化必定早于其它任务开始跑；栈为 `256*4 = 1024 B`（原来是 512 B，容不下 USB 初始化调用链）。
- `Host_Task` 只等 `HOST_RX_FLAG` 并调 `Host_Poll()`，不再负责 USB 生命周期；删掉它不影响枚举，但不处理上位机数据。
- USB OTG HS 控制器当前使用内置 PHY，以全速设备运行，USB 内核时钟为 48 MHz。
- USB 使用外部晶振驱动的 PLL3Q：`24 MHz / 8 × 80 / 5 = 48 MHz`；PLL3 输入为 3 MHz，使用 `RCC_PLL3VCIRANGE_1`，VCO 为 240 MHz，使用 `RCC_PLL3VCOMEDIUM`。
- PLL1Q 仍为 120 MHz，保留现有 FDCAN 时钟及位时序。
- 默认任务栈为 1024 字节。USB 初始化调用链的静态栈用量：AC6 估算 352 字节（含间接调用为 Unknown）、GCC Debug 估算约 652 字节，原来的 512 字节不足；还需要为任务上下文留出空间。
- `lunar_rover.ioc` 已同步 USB 时钟选择、PLL3 参数和默认任务栈，重新生成时应保留这些设置。

## 协议兼容

此次修复不修改 USB 描述符、应用数据包或收发解析。保持当前下行 46 字节、上行 35 字节协议，设备标识仍为 `VID 0483 / PID 5740`。

## 验证

- GCC Debug 和 Keil 构建通过；Keil 重编译 `freertos.c` 时有 4 条原有弱任务入口的未使用参数警告。
- 最终反汇编与链接映射确认：实际 `StartDefaultTask()` 调用 `MX_USB_DEVICE_Init()`，初始化代码未被链接器移除，PLL3 配置函数也进入固件。
- USB 协议实现、描述符、主系统时钟及 FDCAN 配置与修改前的 SHA-256 一致。
- 本次未烧录；实际枚举与上位机自动识别尚需上板验证。

烧录后将数据线接到 MCU 的 USB 设备接口，系统应枚举出 `0483:5740` 的 CDC 串口。`FAED:4873` 的 Horco CMSIS-DAP 调试器附带串口不是该应用的 USB-CDC 接口。

枚举成功后，可观察 `host_link_tick` 和 `host_rx_frames`：前者表示已接收 USB 数据，后者表示已识别完整合法请求帧。上位机控制权只决定是否采纳位置命令，不阻止合法请求的状态应答。
