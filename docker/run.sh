#!/usr/bin/env bash
#
# Launch the ugv_ws ROS 2 Humble container with GUI (Gazebo/RViz)
#
# The repo is mounted at /home/ws/ugv_ws so the workspace's hardcoded paths work.
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=ugv_humble:latest
NAME=ugv_humble

# Allow local X11 clients (XWayland on Wayland hosts). Scoped to root, the
# container's default user; falls back to the broader form if unavailable.
xhost +SI:localuser:root >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true

# Grant the in-container root the host's video/render GIDs so it can use
# /dev/dri (AMD radeonsi) for hardware-accelerated OpenGL.
GRP_ARGS=()
for g in video render; do
  gid="$(getent group "$g" | cut -d: -f3)"
  [ -n "$gid" ] && GRP_ARGS+=(--group-add "$gid")
done

COMMON_ARGS=(
  -e DISPLAY="$DISPLAY"                       # :0 via XWayland
  -e QT_X11_NO_MITSHM=1
  -e UGV_MODEL="${UGV_MODEL:-ugv_rover}"      # ugv_rover | ugv_beast | rasp_rover
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  --device /dev/dri                           # AMD GPU acceleration
  -v /dev/input:/dev/input                    # game controller (hotplug-friendly)
  -v "$REPO":/home/ws/ugv_ws                  # repo at the hardcoded path
  --network host                              # simplest for ROS DDS + web UI
  "${GRP_ARGS[@]}"
)

if [ "${1:-}" = "--persist" ]; then
  # Long-lived: create once if missing, then exec into it.
  if ! docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    docker run -dit --name "$NAME" "${COMMON_ARGS[@]}" "$IMAGE" sleep infinity
  fi
  docker start "$NAME" >/dev/null
  exec docker exec -it "$NAME" bash
else
  # Disposable: fresh container, nothing to clean up afterwards.
  exec docker run -it --rm --name "$NAME" "${COMMON_ARGS[@]}" "$IMAGE" bash
fi
