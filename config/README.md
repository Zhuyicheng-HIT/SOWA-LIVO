# Configuration

`sowa_livo_long_corridor.yaml` is the frozen long-corridor profile from the private source repository. The FAST-LIVO2 patch adds the scheduler object to LiDAR and visual update owners. To enable scheduling in a launch, load the profile as ROS parameters:

```xml
<rosparam command="load" file="$(find fast_livo)/config/sowa_livo_long_corridor.yaml" />
```

Sensor topics, camera calibration, LiDAR model, and extrinsics remain platform-specific FAST-LIVO2 settings and must be supplied by the selected launch configuration.
