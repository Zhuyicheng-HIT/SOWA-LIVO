# Binary Build Record v0.1.0

- Target: Linux x86_64, Ubuntu 20.04, ROS Noetic
- Compiler: GCC 9.4.0
- FAST-LIVO2 base: `hku-mars/FAST-LIVO2@0d2c0346107b75b59934975adec9a6eeeb913c64`
- SOWA source snapshot: `Zhuyicheng-HIT/SOWA-LIVO_private@efc68c81045576c69a3443dfe8dd6d9dc110f540`
- Core: `libsowa_core.so`, built with `BUILD_SHARED_LIBS=ON`
- Binary interface: Eigen 3.3.7, 16-byte maximum dynamic/static alignment; the adapter sets both Eigen alignment definitions locally.
- Adapter: source compiled into a shared library in the pinned FAST-LIVO2 workspace
- Core test: passed, 1/1
- Adapter link smoke test: passed, 1/1
- Enabled binary API with a native-optimized consumer: passed; direction gain, transformed Jacobian/residual, raw/modified information, and information-loss eigenvalue agree.
- Disabled binary API: Jacobian and residual remain byte-for-byte equal.
- Full patched FAST-LIVO2 build: passed with catkin on the pinned upstream commit

The historical v0.7.6 full-route 3D RMSE is `0.5105701957 m`. That metric identifies the estimator replay and is not a measurement of the packaging build.
