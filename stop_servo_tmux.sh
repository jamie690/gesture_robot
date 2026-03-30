#!/usr/bin/env bash
set -e

SESSION="servo_test"

echo "Stopping servo test session..."

tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

pkill -f rviz2 || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f servo_node_main || true
pkill -f ur_control.launch.py || true
pkill -f gesture_control.gesture_pub || true
pkill -f gesture_servo_bridge || true
pkill -f gesture_pub || true

ros2 daemon stop || true

echo "Servo test session stopped."
