# `ugv_else` third-party dependency provenance

Historically every package under `src/ugv_else/` was a source copy ("vendored")
with no record of where it came from or which revision. This document restores
that provenance and explains how each dependency is now sourced.

Most packages are now pinned declaratively in [`../../ugv_else.repos`](../../ugv_else.repos)
and pulled with `vcs import` (see the build scripts). The exact upstream commit for
each was identified by comparing git **blob hashes** of the vendored files against
upstream history, so the pins reproduce the previously-shipped behaviour.

## Pulled via `ugv_else.repos` (no longer committed here)

| Package(s) | Upstream | Pin | Notes |
|---|---|---|---|
| `emcl2` | [CIT-Autonomous-Robot-Lab/emcl2_ros2](https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2) | `561ef81` | Source identical. Only the bundled `config/*.param.yaml` differed; ugv_nav passes its own emcl params, so this is irrelevant. |
| `explore_lite` | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) | `e40e857` | Source byte-identical. The 3 tuned params (commits `6772fc5`, `94f95f2`) now live in `ugv_nav/param/explore_lite.yaml`, launched via `ugv_nav/launch/explore.launch.py`. The repo's extra `map_merge` package is `COLCON_IGNORE`d by the build scripts. |
| `teb_local_planner`, `teb_msgs` | [rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) | `630a22e` | Byte-identical. |
| `costmap_converter`, `costmap_converter_msgs` | [rst-tu-dortmund/costmap_converter](https://github.com/rst-tu-dortmund/costmap_converter) | `9565858` | Byte-identical (v0.1.2). |
| `apriltag` | [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) | tag `v3.4.2` | Byte-identical. Builds as a normal colcon CMake package — the old manual `build_apriltag.sh` step is gone. |
| `apriltag_msgs`, `apriltag_ros` | [Adlink-ROS/apriltag_ros](https://github.com/Adlink-ROS/apriltag_ros) (foxy-devel) | `2941821` | Source identical. The UGV-added composable-node launch (`bringup.launch.py`) now lives first-party at `ugv_vision/launch/apriltag_bringup.launch.py`. |
| `ldlidar` → `ldlidar_stl_ros2` | [ldrobotSensorTeam/ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) | `bf668a8` | Driver source identical. The former copy was renamed and carried UGV launch config; that config now lives first-party in `ugv_bringup/launch/ldlidar/` and targets the upstream `ldlidar_stl_ros2_node`. |

## Still committed here (genuine source forks — no matching upstream commit)

These have real Waveshare source modifications, so they cannot be pulled cleanly
from upstream. They remain vendored, but their origin is now recorded so a future
maintainer can rebase the deltas or upstream them.

| Package | Closest upstream | Local modifications to preserve |
|---|---|---|
| `rf2o_laser_odometry` | [MAPIRlab/rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) (ros2) | Adds an IMU passthrough (`imu_topic` param + `sensor_msgs/Imu` publisher) and **negates the odom x/y position** (`-1.0 * translation`). Behavioural — do not drop. |
| `vizanti` (`vizanti`, `vizanti_cpp`, `vizanti_demos`, `vizanti_msgs`, `vizanti_server`) | [MoffKalast/vizanti](https://github.com/MoffKalast/vizanti) (ros2) | Modified `vizanti_cpp` (e.g. `tf_consolidator.cpp`) and an added `param_manager.cpp`; demo launch/scripts renamed. |
| `robot_pose_publisher` | [MilanMichael/robot_pose_publisher_ros2](https://github.com/MilanMichael/robot_pose_publisher_ros2) | Renamed package; default `base_frame` `base_link`→`base_footprint`; extra `tf2_ros/transform_listener.h` include; launch `is_stamped: True`. |

## Removed earlier (not dependencies anymore)

`cartographer` and `gmapping` were deleted in commit `d6f1e3d` when SLAM moved to
`slam_toolbox` (installed via apt). See `change_explanation.md`.
