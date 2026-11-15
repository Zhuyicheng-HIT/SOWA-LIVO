# SOWA Binary Build Record v0.1.0

- Target: Linux x86_64, Ubuntu 20.04, ROS Noetic adapter
- Compiler: GCC 9.4.0
- SOWA source snapshot: `efc68c81045576c69a3443dfe8dd6d9dc110f540`
- Core: ROS-free `libsowa_core.so`, built with `BUILD_SHARED_LIBS=ON`
- Interface: Eigen 3.3.7, 16-byte maximum dynamic/static alignment
- Adapter: source compiled locally as a shared library
- Core tests: passed, 1/1
- Adapter linkage and ABI tests: passed, 2/2
- Enabled update: direction gain, transformed Jacobian/residual, raw/modified information, and information-loss eigenvalue verified
- Disabled update: Jacobian and residual remain byte-for-byte equal
- Complete integrated catkin build: passed

The historical v0.7.6 full-route result is 3D RMSE `0.5105701957 m`. Release `v0.1.0` identifies the binary packaging build.