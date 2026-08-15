# ROS 2 Humble image for the ugv_ws workspace.
#
# Build:  docker build -t ugv_humble:latest .
FROM osrf/ros:humble-desktop-full

# ---- system + ROS dependencies (full stack, from README + rosdep) ----
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-argcomplete alsa-utils \
      python3-vcstool python3-rosdep \
      python3-numpy python3-dev \
      ros-humble-libg2o libsuitesparse-dev \
      ros-humble-gazebo-* \
      ros-humble-slam-toolbox \
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

# ---- voice stack (voice_control_plan.md Phase 0): audio I/O + placeholder TTS ----
# Separate layer so adding these didn't invalidate the big apt layer above.
# robot-localization (EKF, used by bringup_ros2_control) was previously only
# hand-installed inside the persistent container — baked in here.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libportaudio2 portaudio19-dev espeak-ng \
      ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir sounddevice

# ---- voice stack (voice_control_plan.md Phase 1): wake word + ASR + TTS ----
# numpy MUST stay <2: unpinned, openwakeword's dependency tree upgrades numpy
# to 2.x, which breaks the distro scipy and with it the ROS Python stack
# (verified break + fix 2026-08-15). Models are NOT baked in — fetch them with
# scripts/fetch_voice_models.sh into the host-mounted models/ directory.
# pytest 7: faster-whisper drags in anyio, whose pytest plugin needs >=7 while
# Ubuntu 22.04 ships 6.2.5 — without the upgrade, all `colcon test` collection
# breaks with "No module named '_pytest.scope'".
RUN pip3 install --no-cache-dir 'numpy<2' 'pytest>=7,<8' \
      openwakeword faster-whisper piper-tts onnxruntime

# ---- GUI / GPU environment ----
ENV QT_X11_NO_MITSHM=1
# Gazebo finds the repo's custom world/robot models once the workspace is built.
ENV GAZEBO_MODEL_PATH=/home/ws/ugv_ws/install/ugv_gazebo/share/ugv_gazebo/models:/home/ws/ugv_ws/install/ugv_description/share

# ---- convenience: auto-source ROS, the workspace (if built), and Gazebo ----
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /home/ws/ugv_ws/install/setup.bash ] && source /home/ws/ugv_ws/install/setup.bash' >> /root/.bashrc \
 && echo 'source /usr/share/gazebo/setup.sh' >> /root/.bashrc

WORKDIR /home/ws/ugv_ws
