#!/usr/bin/env bash
# First-time setup + build of the ugv_ws workspace.
#
# Pulls pinned third-party dependencies (ugv_else.repos / vcstool), resolves
# system + ROS dependencies (rosdep), builds, and wires up shell auto-sourcing.
# See src/ugv_else/PROVENANCE.md for how the dependencies are sourced.
set -e
cd /home/ws/ugv_ws

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
colcon build --symlink-install

# 4) Convenience: auto-source ROS, the workspace, and argcomplete.
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo 'eval "$(register-python-argcomplete ros2)"' >> ~/.bashrc
echo 'eval "$(register-python-argcomplete colcon)"' >> ~/.bashrc
echo "source /home/ws/ugv_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
