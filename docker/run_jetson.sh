#!/usr/bin/env bash
# Launch the ugv_ws autonomy container on the Jetson (ARM64, headless).
# Gazebo/RViz run on a remote desktop; this container runs SLAM/Nav2/teleop/web.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=ugv_jetson:latest
NAME=ugv_jetson

COMMON_ARGS=(
  -e UGV_MODEL="${UGV_MODEL:-ugv_beast}"        # match the model the desktop spawns
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"        # MUST match the desktop (see §6)
  -v "$REPO":/home/ws/ugv_ws                    # repo at the hardcoded path
  --network host                                # DDS discovery + web UI over the LAN

  # --- real-robot devices: commented out for the Gazebo test (Gazebo supplies these). ---
  --device /dev/ttyTHS1                          # ESP32 UART (Jetson)   -> ugv_hardware
  --device /dev/ttyACM0                          # LDLiDAR USB
  --device /dev/video0                           # USB pan/tilt camera
  --device /dev/snd                              # ALSA: camera mic (in) + audio board speakers (out)
  -v /dev/bus/usb:/dev/bus/usb                   # OAK-D (depthai)
  -v /dev/input:/dev/input                       # gamepad: expose the device nodes...
  "--device-cgroup-rule=c 13:* rwm"              # ...AND allow opening them (a bind mount alone is blocked by the device cgroup)
  -v /run/udev:/run/udev:ro                      # SDL2 (joy_node + pygame) enumerates joysticks via udev
  # --runtime nvidia                             # only if you need CUDA in-container
)

if [ "${1:-}" = "--persist" ]; then
  if ! docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    docker run -dit --name "$NAME" "${COMMON_ARGS[@]}" "$IMAGE" sleep infinity
  fi
  docker start "$NAME" >/dev/null
  exec docker exec -it "$NAME" bash
else
  exec docker run -it --rm --name "$NAME" "${COMMON_ARGS[@]}" "$IMAGE" bash
fi