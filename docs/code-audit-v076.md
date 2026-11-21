# SOWA-LIVO v0.7.6 source and result record

## Best recorded replay

The lowest recorded full-route 3D RMSE is the route-a-dynamic-v0.7.6-20261103 replay.

| Metric | Recorded value |
| --- | ---: |
| x RMSE | 0.4643875885 m |
| y RMSE | 0.2077793295 m |
| z RMSE | 0.0430562735 m |
| 3D RMSE | 0.5105701957 m |
| Evaluation samples | 2441 |

The replay used the frozen hospital L sensor bag with SHA-256 09d5d6d3d346e83a13e3687c4f17a6ffe550c53ca4b38044c93d1bb789217e0e. The parameter file hash is d558ca70c3ef4fb5cbb41fa1eca6f1c9ca03fcd8aa9da6f3385b04eaee307401; camera configuration hash is 42f24cd8a2d1d64b53edc3200a23f1094a2a0b084ce223fa89bc01ea3fc9a87; replay executable hash is 868d97838023d4c2b04da58d3ff436911a96a01fea99478559f2a1bcdb76ce2a.

## Frozen parameters

The recorded profile enables LiDAR and visual directional scheduling, visual Route A, a visual support threshold of 0.70, calibration gain 26.630767801197237, and a three-frame activation gate. The public estimator profile at estimator/config/sowa_livo_long_corridor.yaml carries these values in the SOWA parameter namespace.

## Source and build status

The public source tree contains the full ROS 1 estimator package and Eigen-only SOWA core. The metrics identify the recorded executable and its inputs; they do not indicate that this Windows environment rebuilt that executable from the current checkout.

Build verification:

- The ROS-free core Release build and CTest passed with GCC 11.4, Eigen 3, and C++14. The current checkout also built and passed CTest in Ubuntu 22.04 with GCC 11.4.
- The full ROS 1 catkin package build was not run in this environment because ROS Noetic is not installed.

The disabled core path returns before matrix operations. The disabled adapter test checks that the input measurement block is unchanged.
