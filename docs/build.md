# Build requirements

## Standalone core

The reusable core requires a C++14 compiler, CMake, and Eigen 3. It has no ROS, PCL, or OpenCV dependency.

    cmake -S estimator/sowa_core -B build/sowa-core -DCMAKE_BUILD_TYPE=Release
    cmake --build build/sowa-core -j
    ctest --test-dir build/sowa-core --output-on-failure
    cmake --install build/sowa-core --prefix "$HOME/.local"

The install exports the SOWA::core CMake target.

## ROS 1 estimator

The complete estimator package targets Ubuntu 20.04 and ROS Noetic. Build dependencies are Eigen 3, PCL, OpenCV, Sophus, Vikit common/ROS libraries, and ROS messages listed in estimator/package.xml.

Place the repository's estimator/ directory in the src/ directory of a catkin workspace, then build:

    cd "$HOME/sowa_ws"
    catkin_make -DCMAKE_BUILD_TYPE=Release
    source devel/setup.bash

For the corridor profile, load config/sowa_livo_long_corridor.yaml together with calibration and sensor topic settings for the selected platform. Review the launch file before using a live sensor; topic names and calibration are platform-specific.

## Corridor simulation

Install Gazebo Sim and set GZ_SIM_RESOURCE_PATH to include simulation/scenes. Launch rich.sdf for the textured scene or low.sdf for the paired low-texture scene. The capture utilities use Python dependencies listed by their import statements; the bag converter also needs ROS 1 message definitions.
