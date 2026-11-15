# SOWA-LIVO

**Spatial Observability-aware Weight Adaptation**

[简体中文](README.zh-CN.md)

SOWA-LIVO adapts the information contributed by LiDAR and visual measurements according to their spatial observability. In long corridors, it preserves strong geometric constraints and uses complementary visual information along weakly constrained directions.

This repository provides an independent, ROS-free C++ binary core, a lightweight ROS 1 adapter, and deployment scripts. The core source code will be released upon acceptance of the associated paper.

## Installation

For Ubuntu 20.04 with ROS Noetic:

```bash
git clone https://github.com/Zhuyicheng-HIT/SOWA-LIVO.git
cd SOWA-LIVO
bash scripts/install.sh "$HOME/sowa_ws"
```

For Windows with WSL2 and Docker, run from a WSL checkout:

```bash
bash scripts/install_docker.sh "$HOME/sowa_ws"
```

The scripts fetch the required dependencies, download and verify the SOWA binary package, install the adapter and long-corridor profile, and build the workspace. The Docker option creates a persistent Noetic container and prints its launch command.

After a native installation:

```bash
bash scripts/run.sh "$HOME/sowa_ws"
```

The launch configuration loads the SOWA long-corridor profile. Set the LiDAR, IMU, RGB topics, camera calibration, and extrinsics for your sensor setup.

## Binary package and interfaces

Release `v0.1.0` targets Linux x86_64, Ubuntu 20.04, GCC 9.4, and Eigen 3.3.7. It contains `libsowa_core.so`, public interface headers, CMake package files, and license notes. The ROS adapter is compiled locally during installation.

The core interface accepts a Jacobian, residual, measurement covariance, and state-direction maps through `SOWA::core`. The ROS adapter connects measurement updates to that interface. See [`adapter/README.md`](adapter/README.md) and [`docs/binary-build-info.md`](docs/binary-build-info.md).

## Long-corridor reference result

The recorded full L-corridor run uses SOWA-LIVO v0.7.6 Dynamic Route A, with 2441 evaluation samples:

| Metric | RMSE (m) |
| --- | ---: |
| x | 0.464388 |
| y | 0.207779 |
| z | 0.043056 |
| 3D | 0.510570 |

The binary package has its own release version and SHA-256 checksum. Dataset asset names and Windows/WSL reconstruction commands are documented in [`docs/bag-download.md`](docs/bag-download.md).

## License

The SOWA binary core is distributed under [LICENSE-BINARY.md](LICENSE-BINARY.md). The adapter and integration patch use the GPLv2 terms included in [`adapter/COPYING`](adapter/COPYING). Third-party components retain their respective licenses and copyright notices.