
# 机械臂 STM32 工程归档状态

状态：源码待授权审计；本目录当前只有本说明，不包含可编译固件。

输入包：lunar_rover.zip  
SHA-256：63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8  
包内 Git 元数据记录的工作分支头：963fd0275ddbe2c759b35250842284c6d62e90c1  
上游 URL：https://github.com/champ719/lunar_rover

## 结构检查

输入包包含 Core/User、STM32H723 的 CubeMX lunar_rover.ioc、根 CMakeLists.txt/CMakePresets.json、Keil MDK-ARM/lunar_rover.uvprojx、链接脚本和多个第三方组件。从工程配置看，源代码与常用构建入口基本齐全。

包内有 Git 历史和 LibXR 子模块、数百个 build/cache/Keil 输出、日志、object/hex/map 文件，以及一个与工程无关的可执行文件。归档候选应排除整个 .git、构建目录、缓存、运行日志、目标文件和无关文件；保留工程配置前须逐项复核机器专用路径。

## 许可与权限阻塞

工程根目录没有 LICENSE；上游 GitHub 仓库也没有声明项目许可证。包内 CMSIS、ST HAL、USB Device Library、FreeRTOS、LibXR、Eigen 和文档样式等各有不同第三方许可。它们的组件许可不能代替机械臂项目代码的授权，也不能证明团队内部共享许可。

因此目前不上传固件源码、子模块内容或二进制。取得项目权利人书面授权和各第三方组件清单后，再决定是保留许可兼容的来源文件，还是改用授权上游依赖。

## 接口摘要

源码显示控制器目标为 STM32H723；内部 FDCAN3 连接关节电机，控制周期约 2 ms。时序参数对应标称速率 1 Mbps、数据相位 2 Mbps；variables.cpp 的注释称 classic CAN，但构造参数启用了 FD/BRS，需与电机侧协议核对。主机链路由 USB OTG HS CDC 提供 46 字节下行目标帧和 35 字节上行状态帧；UART5 是 100000 baud、9 位/偶校验的仅接收遥控输入，不是 ROS 2 串口命令接口。ROS 2 驱动与 USB CDC 的设备映射和帧字段仍需实机或受控接口验证，不能据源码静态核对宣称互通。
