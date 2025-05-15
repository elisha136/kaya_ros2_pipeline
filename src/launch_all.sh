#!/usr/bin/env bash
#
# launch_all.sh — bring up the full pipeline:
#   1) Camera → /cube_info
#   2) Motion Planning → /planned_path
#   3) Control → /cmd_vel → Dynamixels

set -e

# When this script exits (including via Ctrl+C), kill all background jobs
trap 'echo "Shutting down…"; kill $(jobs -p) 2>/dev/null' EXIT

# 1) Source the workspace overlay
echo "Sourcing ROS 2 workspace…"
source "$(dirname "$0")/install/setup.bash"

# 2) Launch camera+vision
echo "Launching camera vision…"
ros2 launch kaya_camera_vision cube_detection_launch.py &

# 3) Launch motion planning
echo "Launching motion planning…"
ros2 launch kaya_motion_planning motion_planning_launch.py &

# 4) Launch robot control
echo "Launching robot control…"
ros2 launch kaya_robot_control control_launch.py &

# 5) Wait on all of them
wait
