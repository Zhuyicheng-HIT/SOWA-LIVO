# SOWA-LIVO estimator

This ROS 1 catkin package contains the complete estimator and the SOWA measurement-weighting integration. The direction-aware core is kept under sowa_core/ so it can also be built and tested independently.

The sensor-specific YAML files are in config/. Select a launch file under launch/ and verify its topics and calibration against the connected platform.

ROS Noetic dependencies and build instructions are listed in the repository's docs/build.md. The corridor profile is config/sowa_livo_long_corridor.yaml; weighting is opt-in in the general configuration.
