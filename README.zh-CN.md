# SOWA-LIVO

**Spatial Observability-aware Weight Adaptation**
**空间可观测性感知的自适应加权**

[English](README.md)

SOWA-LIVO 根据激光与视觉观测在不同空间方向上的约束能力，自适应调节各自的信息贡献。面向长走廊工况，保留几何约束较强方向的有效信息，并在弱约束方向利用可靠视觉提供补充。

本仓库提供独立、无 ROS 依赖的 C++ 二进制核心、轻量 ROS 1 适配层和部署脚本。核心算法源码将在相关论文录用后开源。

## 安装

Ubuntu 20.04 + ROS Noetic：

```bash
git clone https://github.com/Zhuyicheng-HIT/SOWA-LIVO.git
cd SOWA-LIVO
bash scripts/install.sh "$HOME/sowa_ws"
```

Windows + WSL2 + Docker，在 WSL 中运行：

```bash
bash scripts/install_docker.sh "$HOME/sowa_ws"
```

脚本会获取依赖、下载并校验 SOWA 二进制包、安装适配层与长走廊参数，并编译工作区。Docker 方式会保留 Noetic 容器，并在安装结束后打印启动命令。

原生安装完成后运行：

```bash
bash scripts/run.sh "$HOME/sowa_ws"
```

启动配置会加载 SOWA 长走廊参数。实际使用时，配置对应的激光雷达、IMU、RGB 话题、相机标定和传感器外参。

## 发布包与接口

`v0.1.0` 发布包面向 Linux x86_64、Ubuntu 20.04、GCC 9.4 和 Eigen 3.3.7，包含 `libsowa_core.so`、公共接口头文件、CMake 包文件和许可说明。ROS 适配源码由安装脚本在本机编译。

核心通过 `SOWA::core` 接收 Jacobian、残差、测量协方差和状态方向映射；ROS 适配层负责连接估计器的测量更新。接口与构建信息见 [`adapter/README.md`](adapter/README.md) 和 [`docs/binary-build-info.md`](docs/binary-build-info.md)。

## 长走廊参考结果

完整长 L 记录采用 SOWA-LIVO v0.7.6 Dynamic Route A，共 2441 个评价样本：

| 指标 | RMSE（m） |
| --- | ---: |
| x | 0.464388 |
| y | 0.207779 |
| z | 0.043056 |
| 3D | 0.510570 |

二进制包单独记录发布版本和 SHA-256。数据附件名称与 Windows / WSL 拼接命令见 [`docs/bag-download.md`](docs/bag-download.md)。

## 许可

SOWA 二进制核心采用 [LICENSE-BINARY.md](LICENSE-BINARY.md)。适配层与集成补丁采用 [`adapter/COPYING`](adapter/COPYING) 中的 GPLv2 条款。第三方组件保留各自许可及版权声明。