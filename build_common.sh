#!/usr/bin/env bash
# Incremental build of the ugv_ws workspace.
#
# Third-party dependencies are no longer vendored in git: most are pulled from
# pinned upstream commits via ugv_else.repos (vcstool), and system/ROS deps are
# resolved with rosdep. A few genuinely-forked packages remain committed under
# src/ugv_else/ (see src/ugv_else/PROVENANCE.md).
set -e
cd /home/ws/ugv_ws

# 1) Fetch pinned third-party sources into src/ugv_else/ (idempotent).
vcs import src < ugv_else.repos

# The m-explore-ros2 repo also ships a map_merge package that this workspace
# does not use; tell colcon to skip it.
touch src/ugv_else/m-explore-ros2/map_merge/COLCON_IGNORE 2>/dev/null || true

# 2) Resolve system + ROS dependencies declared in package.xml files.
rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO:-humble}"

# 3) Build everything (package.xml dependencies determine build order).
# Release is required: without CMAKE_BUILD_TYPE the C++ packages (rf2o laser
# odometry, ugv_hardware, the lidar driver, ...) compile unoptimized and burn
# several times the CPU at runtime.
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
