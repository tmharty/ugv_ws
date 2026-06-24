# ROS 2 Jazzy image for the ugv_ws workspace.
#
# Build:  docker build -t ugv_jazzy:latest .
FROM osrf/ros:jazzy-desktop-full

# ---- system + ROS dependencies (full stack, from README + rosdep) ----
# Gazebo: Jazzy pairs with new Gazebo (Harmonic). ros-jazzy-ros-gz pulls the
# ros_gz_sim / ros_gz_bridge / ros_gz_image stack; ros-jazzy-gz-ros2-control
# provides the gz_ros2_control/GazeboSimSystem plugin that lets the sim reuse the
# real robot's diff_drive_controller. (Gazebo Classic has no Jazzy release.)
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-argcomplete alsa-utils \
      python3-vcstool python3-rosdep \
      python3-numpy python3-dev \
      ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
      ros-jazzy-libg2o libsuitesparse-dev \
      ros-jazzy-slam-toolbox \
      ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-nav2-* \
      ros-jazzy-rosbridge-* \
      ros-jazzy-rqt-* \
      ros-jazzy-rtabmap-* \
      ros-jazzy-usb-cam \
      ros-jazzy-depthai-ros \
      ros-jazzy-apriltag ros-jazzy-apriltag-ros \
      ros-jazzy-teleop-twist-joy ros-jazzy-joy \
      libceres-dev libgoogle-glog-dev liblua5.3-dev libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- python dependencies (requirements.txt) ----
RUN pip3 install --no-cache-dir --break-system-packages pyserial flask mediapipe requests

# ---- GUI / GPU environment ----
ENV QT_X11_NO_MITSHM=1

# ---- convenience: auto-source ROS and the workspace (if built) ----
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /home/ws/ugv_ws/install/setup.bash ] && source /home/ws/ugv_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /home/ws/ugv_ws
