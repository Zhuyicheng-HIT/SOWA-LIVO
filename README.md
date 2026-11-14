# SOWA-LIVO Extension

[简体中文](README.zh-CN.md)

SOWA-LIVO is distributed here as a binary extension layer for FAST-LIVO2. The package separates the direction-weighting core from the ROS 1 adapter. It does not include FAST-LIVO2 source; the deployment script fetches the pinned upstream repository and builds the integration locally.

## Release package

The `v0.1.0` Linux x86_64 asset contains `libsowa_core.so`, public interface headers, CMake package files, and license notes. It targets Ubuntu 20.04, ROS Noetic, and GCC 9.4. The ROS adapter source is included in this repository and compiled against the binary core during deployment. The deployment installs `sowa_mapping_avia.launch`, which loads the long-corridor SOWA profile before including the upstream AVIA launch.

The adapter source and FAST-LIVO2 integration patch are in this repository. The numerical implementation is in the binary core. Source code for the core is planned for release after paper acceptance.

## One-command deployment

Use Ubuntu 20.04 with ROS Noetic. The script installs FAST-LIVO2 prerequisites, checks out upstream commit `0d2c0346107b75b59934975adec9a6eeeb913c64`, downloads the extension asset, applies the integration patch, and builds the catkin workspace.

```bash
git clone https://github.com/Zhuyicheng-HIT/SOWA-LIVO.git
cd SOWA-LIVO
bash scripts/deploy_fast_livo2.sh "$HOME/sowa_fast_livo2_ws"
```

On Windows with WSL2, run the Docker-based deployment from a WSL checkout:

```bash
bash scripts/deploy_fast_livo2_docker.sh "$HOME/sowa_fast_livo2_ws"
```

The script keeps a named Noetic container so the built workspace and its ROS runtime remain available. It prints the `docker exec` launch command after the build.

The upstream project and license are at [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2). The deployment uses that repository at a fixed commit and leaves its source in the user's local workspace.

After deployment, source the workspace and start the AVIA example with `roslaunch fast_livo sowa_mapping_avia.launch`. Supply the matching LiDAR, IMU, RGB topics and camera calibration for the target sensor setup.

## Connect another estimator

The ROS-free core accepts a Jacobian, residual, covariance, and state-direction maps through `SOWA::core`. `adapter/include/doa_axis_scheduler.h` is the ROS 1 integration interface. A sensor adapter maps its estimator state and measurement block to this interface.

## Long-corridor reference run

The best recorded full L-corridor result is SOWA-LIVO v0.7.6 Dynamic Route A: XYZ RMSE `0.464388 / 0.207779 / 0.043056 m`, with 3D RMSE `0.510570 m` over 2441 samples. The result belongs to the recorded estimator replay; the extension package is versioned and hashed separately in the release asset.

The rosbag is attached to the GitHub Release as split assets. It is not stored on `main`. Windows and WSL reconstruction commands are in [`docs/bag-download.md`](docs/bag-download.md).

## Licenses

The binary core is covered by [LICENSE-BINARY.md](LICENSE-BINARY.md). The ROS adapter and FAST-LIVO2 integration patch carry the FAST-LIVO2 GPLv2 notice in [`adapter/COPYING`](adapter/COPYING). Third-party notices remain with their respective projects.

## 中文说明

本仓库发布 SOWA-LIVO 的二进制拓展层，供 FAST-LIVO2 调用。数值核心与 ROS 1 适配层分开。仓库不包含 FAST-LIVO2 源码；部署脚本会下载固定版本的上游仓库，并在本机完成集成编译。

`v0.1.0` Linux x86_64 发布包包含核心动态库、接口头文件、CMake 配置和许可说明。目标环境为 Ubuntu 20.04、ROS Noetic 和 GCC 9.4。ROS 适配源码随仓库提供，由部署脚本在本地编译并链接核心库。核心源码计划在论文录用后公开。

使用 Ubuntu 20.04 和 ROS Noetic 时，运行上面的部署命令即可安装依赖、下载上游 FAST-LIVO2 `0d2c034`、获取拓展库、应用补丁并编译 catkin 工作区。

Windows + WSL2 用户可在 WSL 中运行 `bash scripts/deploy_fast_livo2_docker.sh "$HOME/sowa_fast_livo2_ws"`，脚本会创建并保留 Noetic 容器、编译工作区，并输出启动命令。部署后使用 `roslaunch fast_livo sowa_mapping_avia.launch` 加载长走廊参数并启动 AVIA 示例。

历史完整长 L 记录中，精度最高的是 SOWA-LIVO v0.7.6 Dynamic Route A：XYZ RMSE 为 `0.464388 / 0.207779 / 0.043056 m`，2441 个样本的 3D RMSE 为 `0.510570 m`。该指标对应历史估计器回放；发布包本身使用独立版本号和哈希记录。

rosbag 作为 GitHub Release 附件分卷提供，不放入 `main`。Windows PowerShell 与 WSL 拼接、校验命令见 [`docs/bag-download.md`](docs/bag-download.md)。
