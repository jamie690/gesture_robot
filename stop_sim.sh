#!/usr/bin/env bash
set -e

SESSION="sim_only"

echo "Stopping simulation..."

tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

pkill -f rviz2 || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f gesture_control.gesture_pub || true
pkill -f moveit_cartesian_demo || true

ros2 daemon stop || true

echo "Simulation stopped."
