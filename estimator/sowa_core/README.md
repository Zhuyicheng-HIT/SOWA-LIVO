# SOWA core

This Eigen-only C++14 library evaluates directional measurement information and applies synchronized transforms to a Jacobian and residual. It accepts state-direction and nuisance-state maps from its caller and does not depend on ROS, PCL, OpenCV, or Sophus.

For each requested direction, the core projects out other mapped directions and nuisance responses before measuring the remaining conditional information. The v0.7.6 visual path preserves the selected conditional forward mode and softly scales its orthogonal measurement subspace. Covariance whitening is applied before this operation.

Build and test:

    cmake -S estimator/sowa_core -B build/sowa-core -DCMAKE_BUILD_TYPE=Release
    cmake --build build/sowa-core -j
    ctest --test-dir build/sowa-core --output-on-failure

Install the library and exported SOWA::core CMake target:

    cmake --install build/sowa-core --prefix "$HOME/.local"
