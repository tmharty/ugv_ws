# ROS 2 Humble image for the ugv_ws workspace.
#
# Build:  docker build -t ugv_humble:latest .
FROM osrf/ros:humble-desktop-full

# ---- system + ROS dependencies (full stack, from README + rosdep) ----
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-argcomplete alsa-utils \
      ros-humble-gazebo-* \
      ros-humble-cartographer-* \
      ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
      ros-humble-nav2-* \
      ros-humble-rosbridge-* \
      ros-humble-rqt-* \
      ros-humble-rtabmap-* \
      ros-humble-usb-cam \
      ros-humble-depthai-ros \
      ros-humble-teleop-twist-joy ros-humble-joy \
      libceres-dev libgoogle-glog-dev liblua5.3-dev libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- python dependencies (requirements.txt) ----
RUN pip3 install --no-cache-dir pyserial flask mediapipe requests

# ---- GUI / GPU environment ----
ENV QT_X11_NO_MITSHM=1
# Gazebo finds the repo's custom world/robot models once the workspace is built.
ENV GAZEBO_MODEL_PATH=/home/ws/ugv_ws/install/ugv_gazebo/share/ugv_gazebo/models:/home/ws/ugv_ws/install/ugv_description/share

# ---- convenience: auto-source ROS, the workspace (if built), and Gazebo ----
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /home/ws/ugv_ws/install/setup.bash ] && source /home/ws/ugv_ws/install/setup.bash' >> /root/.bashrc \
 && echo 'source /usr/share/gazebo/setup.sh' >> /root/.bashrc

WORKDIR /home/ws/ugv_ws
