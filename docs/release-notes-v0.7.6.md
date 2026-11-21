# SOWA-LIVO v0.7.6

This release publishes the complete estimator source associated with the current best recorded long-corridor result.

## Result

The recorded full-route replay contains 2,441 samples and reports x/y/z RMSE of 0.464388/0.207779/0.043056 m, with 3D RMSE of 0.510570 m. Run inputs and hashes are listed in docs/code-audit-v076.md.

## Included

- Full ROS 1 estimator source with the integrated SOWA scheduler.
- Standalone Eigen-based C++ core and tests.
- Frozen v0.7.6 corridor configuration.
- Rich- and low-texture L-shaped Gazebo scene, source assets, and capture tools.
- Existing long-corridor comparison data.

## Build verification

The standalone core built with GCC 11.4 and its CTest suite passed in Ubuntu 22.04. The ROS 1 catkin package requires the ROS Noetic dependencies listed in docs/build.md.

## License

Estimator and SOWA core sources use GPL-2.0. Scene assets retain the licenses and notices included beside their files.
