#!/bin/bash
# Save the 2D map built by slam_toolbox (Gazebo simulation).
# Writes map.pgm + map.yaml (for AMCL / Nav2 static costmap) and a serialized
# map.posegraph + map.data (for slam_toolbox localization mode).
cd /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps
ros2 run nav2_map_server map_saver_cli -f ./map
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map'}"
cd -
