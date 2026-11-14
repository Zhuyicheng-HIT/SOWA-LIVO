# FAST-LIVO2 Adapter

This ROS 1 adapter connects FAST-LIVO2 LiDAR and visual update blocks to the SOWA binary core. It contains the scheduler interface and parameter/control glue and is compiled locally by the deployment script. Numerical routines are provided by `libsowa_core.so`.

The adapter and its integration patch follow the FAST-LIVO2 GPLv2 distribution terms. `compat_api.h` exposes only ABI declarations for the binary compatibility entry point.
