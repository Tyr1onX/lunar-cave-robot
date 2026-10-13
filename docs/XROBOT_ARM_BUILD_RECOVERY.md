# P9-E：机械臂 XRobot 构建恢复记录

## 输入和证据

已找到的机械臂源包为 Mac 下载目录 ~/Downloads/lunar_rover.zip，SHA-256：

**63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8**

压缩包提取出的 Git 元数据指向 https://github.com/champ719/lunar_rover.git，提交 **963fd0275ddbe2c759b35250842284c6d62e90c1**。仓库归档了 STM32H723VGT6 的 CubeMX、Keil、CMake、HAL/CMSIS、FreeRTOS、LibXR 和应用源码。该源包没有被本轮改写。

当前 main 工程存在或缺少的关键文件：

| 路径 | 状态 | 作用 |
| --- | --- | --- |
| firmware/arm/.config.yaml | 存在 | STM32/CubeMX 外设配置；它不是 XRobot 产品选择配置 |
| firmware/arm/Modules/modules.yaml | 存在但仍是 BlinkLED 示例模板 | 模块清单，不能据此推断真实机械臂模块 |
| firmware/arm/Modules/sources.yaml | 存在 | 指向 XRobot 官方模块索引 |
| firmware/arm/User/libxr_config.yaml | 存在 | LibXR 生成配置；写有 generator: 6.0.0 和 FreeRTOS 等设置 |
| firmware/arm/User/xrobot.yaml | 未找到 | 产品生成配置；工程中没有可复现的产品输入 |
| firmware/arm/Modules/CMakeLists.txt | 未找到 | XRobot setup 生成的模块依赖 CMake |
| firmware/arm/User/xrobot_main.hpp | 未找到 | 由产品配置生成的入口/依赖文件，app_main.cpp 会包含它 |
| firmware/arm/xrobot.lock | 未找到 | 没有可核验的模块版本锁定记录 |

本轮对 Mac 已知存档路径、ThinkBook 机器人项目目录及 NUC 已知工程目录做过定向搜索；没有找到这两个生成输出、产品配置或锁文件。modules.yaml 里的 BlinkLED 是模板示例，不是机械臂工程可用的产品清单。不能用空清单、样例模块或手工编写的生成结果替代。

## 生成链和恢复办法

仓库内 **firmware/arm/Middlewares/Third_Party/LibXR/cmake/XRobot.cmake** 明确要求：

- Modules/CMakeLists.txt 不存在时，执行 xrobot setup（XRobot 1.0 或更新版本）。
- User/xrobot_main.hpp 不存在或需要选定产品时，执行 xrobot gen -c <config>。
- 构建时的 freshness check 读取生成头中的 config、entry、depends 和 digest 指令，并比较输入内容；构建检查不会自行修复或生成这些文件。
- User/libxr_config.yaml 中的 generator 6.0.0 是 LibXR 生成器配置，不能据此断言 XRobot CLI 的具体版本。

XRobot 官方项目管理说明列出 Modules/modules.yaml、可选 sources.yaml、User/xrobot.yaml 和生成的 User/xrobot_main.hpp 等文件；官方 STM32 生成说明描述从 CubeMX .ioc 提取配置并生成 XRobot/LibXR 工程接合文件。参考：[XRobot 项目管理](https://xrobot-org.github.io/docs/proj_man)、[STM32 代码生成](https://xrobot-org.github.io/en/docs/code_gen/stm32)、[XRobot 项目](https://github.com/xrobot-org/XRobot)。命令参数和版本要求应以源工程实际锁定版本的文档为准；当前 LibXR CMake 明确要求 XRobot 1.0+。

可以复现构建所需的最少人工材料：

1. 原工程负责人提供真实的 User/xrobot.yaml，或能生成同一产品配置的源配置。
2. 提供机械臂实际模块列表、来源地址、ref/提交和 xrobot.lock；确认 BlinkLED 是否仅模板，或提供完整产品清单。
3. 提供生成时使用的 XRobot CLI 版本、LibXR 版本，以及模块仓库访问所需的正常团队权限。
4. 提供任何不在仓库内的 C++ runtime/板级依赖及对应版本。不能从缺失报错前推断依赖版本。
5. 如现存完整固件在设备上运行，由负责人提供其源码/产品配置来源及固件构建提交。不得把设备固件反向推测成配置。

收到输入后，在全新隔离副本里先固定工具和依赖版本，再运行官方 setup/gen；检查生成文件的模块列表、依赖和 digest，执行 XRobot freshness check，最后尝试完整 STM32 编译。将生成输入、工具版本、输出源码 SHA 和构建日志记入源码管理。不要先在归档源树上运行未知版本生成器。

## 当前构建结论

目前机械臂完整固件仍未能配置到生成阶段，不能产出 ELF。具体条件是缺少 XRobot 生成的 firmware/arm/Modules/CMakeLists.txt、firmware/arm/User/xrobot_main.hpp 及其真实产品配置/模块版本输入；匹配生成器和 runtime 版本也未确认。没有伪造文件。底盘现有构建结论不受此阻塞影响。
