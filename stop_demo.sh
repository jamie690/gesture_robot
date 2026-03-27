#!/usr/bin/env bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop || true
pkill -f rviz2 || true
pkill -f joint_state_publisher || true
pkill -f joint_state_publisher_gui || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f gesture_control.gesture_pub || true
pkill -f moveit_cartesian_demo || true
tmux kill-session -t gesture_demo 2>/dev/null || true
ros2 daemon start
