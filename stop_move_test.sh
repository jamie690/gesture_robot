#!/usr/bin/env bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 daemon stop || true
pkill -f rviz2 || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f gesture_control.gesture_pub || true
pkill -f moveit_cartesian_demo || true
pkill -f ur_control.launch.py || true
tmux kill-session -t move_test 2>/dev/null || true
ros2 daemon start
