#!/usr/bin/env bash
# First-time setup + build of the ugv_ws workspace.
#
# Pulls pinned third-party dependencies (ugv_else.repos / vcstool), resolves
# system + ROS dependencies (rosdep), builds, and wires up shell auto-sourcing.
# See src/ugv_else/PROVENANCE.md for how the dependencies are sourced.
#
# Usage: ./build_first.sh [--clean]
#   --clean  Remove build/ install/ log/ before building. Use this after the
#            source layout changes (e.g. the ugv_else vendoring migration),
#            otherwise stale CMakeCache paths cause "source directory does not
#            exist" and symlink "Is a directory" errors.
set -e
cd /home/ws/ugv_ws

CLEAN=0
if [ "${1:-}" = "--clean" ]; then
  CLEAN=1
fi

# Tooling needed for the declarative dependency workflow.
sudo apt-get update && sudo apt-get install -y python3-vcstool python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# 1) Fetch pinned third-party sources into src/ugv_else/.
vcs import src < ugv_else.repos
touch src/ugv_else/m-explore-ros2/map_merge/COLCON_IGNORE 2>/dev/null || true

# 2) Resolve system + ROS dependencies declared in package.xml files.
rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO:-humble}"

# 3) Build everything (package.xml dependencies determine build order).
# Release is required: without CMAKE_BUILD_TYPE the C++ packages (rf2o laser
# odometry, ugv_hardware, the lidar driver, ...) compile unoptimized and burn
# several times the CPU at runtime.
if [ "$CLEAN" -eq 1 ]; then
  rm -rf build install log
fi
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4) Convenience: auto-source ROS, the workspace, and argcomplete.
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo 'eval "$(register-python-argcomplete ros2)"' >> ~/.bashrc
echo 'eval "$(register-python-argcomplete colcon)"' >> ~/.bashrc
echo "source /home/ws/ugv_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
