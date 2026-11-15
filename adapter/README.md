# SOWA ROS Adapter

This lightweight ROS 1 adapter connects LiDAR and visual measurement updates to the SOWA binary core. It loads scheduling parameters, maintains the temporal control state, and calls the numerical routines in `libsowa_core.so`.

The installer compiles the adapter locally. `compat_api.h` contains ABI declarations for the compatibility entry point; `core.h` declares the ROS-free core interface. Both use Eigen with 16-byte maximum dynamic/static alignment. The CMake targets propagate those definitions to consumers.

The adapter and integration patch are distributed under the GPLv2 terms in `COPYING`.

## 中文

轻量 ROS 1 适配层将激光和视觉测量更新接入 SOWA 二进制核心，负责读取调度参数、维护时序控制状态并调用 `libsowa_core.so` 中的数值函数。

安装脚本在本地编译适配层。`compat_api.h` 提供兼容入口的 ABI 声明，`core.h` 提供无 ROS 核心接口。二者使用 Eigen 16 字节动态与静态最大对齐，相关定义由 CMake 目标传递给调用方。