# SOWA-LIVO 独立拓展层

[English](README.md)

本仓库发布供 FAST-LIVO2 调用的 SOWA-LIVO 二进制核心。核心库不依赖 ROS；ROS 适配源码单独提供，并由部署脚本在本机编译。仓库不包含 FAST-LIVO2 源码，部署脚本会从官方仓库下载固定提交并在本机集成。

## 发布包

`v0.1.0` Linux x86_64 附件包含 `libsowa_core.so`、接口头文件、CMake 包文件和许可说明。目标环境为 Ubuntu 20.04、ROS Noetic、GCC 9.4。核心算法源码计划在论文录用后公开。

## 一键部署

Ubuntu 20.04 + ROS Noetic：

```bash
git clone https://github.com/Zhuyicheng-HIT/SOWA-LIVO.git
cd SOWA-LIVO
bash scripts/deploy_fast_livo2.sh "$HOME/sowa_fast_livo2_ws"
```

Windows + WSL2：

```bash
bash scripts/deploy_fast_livo2_docker.sh "$HOME/sowa_fast_livo2_ws"
```

脚本固定使用官方 FAST-LIVO2 提交 `0d2c0346107b75b59934975adec9a6eeeb913c64`，下载 SOWA 核心库，安装适配源码与长走廊参数，应用集成补丁并执行 `catkin_make`。WSL2 脚本会保留 Noetic Docker 容器，并在结束时打印启动命令。

部署后可运行：

```bash
roslaunch fast_livo sowa_mapping_avia.launch
```

该 launch 加载长走廊 SOWA 参数，再调用上游 AVIA 示例。实际传感器使用时，请提供对应的点云、IMU、RGB 话题和相机标定。

FAST-LIVO2 官方仓库：[hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)。

## 历史长走廊精度

历史完整长 L 记录中，最佳版本为 SOWA-LIVO v0.7.6 Dynamic Route A：XYZ RMSE 为 `0.464388 / 0.207779 / 0.043056 m`，2441 个样本的 3D RMSE 为 `0.510570 m`。发布包采用独立版本号，并提供构建信息与哈希。

## rosbag

冻结的 rich-texture seed 7 rosbag 作为 GitHub Release 附件分成两个文件，不进入 `main`。两部分下载后，Windows PowerShell、Windows 命令提示符或 WSL 的拼接命令见 [`docs/bag-download.md`](docs/bag-download.md)。拼接后的文件应为 `3002838721` 字节，SHA-256 为 `09d5d6d3d346e83a13e3687c4f17a6ffe550c53ca4b38044c93d1bb789217e0e`。

## 许可

独立二进制核心按 [LICENSE-BINARY.md](LICENSE-BINARY.md) 用于学术研究和实验复现。ROS 适配层与 FAST-LIVO2 集成补丁附带 FAST-LIVO2 GPLv2 许可文本。第三方依赖仍适用各自许可。
