# Configuration / 参数配置

`sowa_livo_long_corridor.yaml` contains the frozen SOWA long-corridor scheduling profile. The installer places it in the estimator's configuration directory. `scripts/run.sh` starts the launch configuration that loads this profile.

Sensor topics, camera calibration, LiDAR model, and extrinsics are supplied by the sensor configuration. Set `sowa.enabled: false` to disable scheduling and retain the estimator's original measurement-update path.

`sowa_livo_long_corridor.yaml` 为冻结的 SOWA 长走廊调度参数。安装脚本将其放入估计器配置目录，`scripts/run.sh` 通过启动配置加载该文件。

传感器配置负责指定话题、相机标定、激光雷达型号和外参。设置 `sowa.enabled: false` 可关闭调度，使用估计器原有的测量更新路径。