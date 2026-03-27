#!/usr/bin/env bash
set -e

SESSION="move_test"

tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop || true
pkill -f rviz2 || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f gesture_control.gesture_pub || true
pkill -f moveit_cartesian_demo || true
pkill -f ur_control.launch.py || true
ros2 daemon start

tmux new-session -d -s $SESSION

# Create panes
P0=$(tmux display-message -p -t $SESSION:0.0 "#{pane_id}")
P1=$(tmux split-window -h -t "$P0" -P -F "#{pane_id}")
P2=$(tmux split-window -v -t "$P0" -P -F "#{pane_id}")
P3=$(tmux split-window -v -t "$P1" -P -F "#{pane_id}")

# ------------------------------------------------------------------
# Pane 0 — UR driver
# ------------------------------------------------------------------

tmux send-keys -t "$P0" "
sleep 1
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10e \
  robot_ip:=192.168.1.69 \
  initial_joint_controller:=scaled_joint_trajectory_controller
" C-m

# ------------------------------------------------------------------
# Pane 1 — MoveIt (IK service only)
# ------------------------------------------------------------------

tmux send-keys -t "$P1" "
sleep 5
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur10e \
  launch_rviz:=false
" C-m

# ------------------------------------------------------------------
# Pane 2 — controller node
# ------------------------------------------------------------------

tmux send-keys -t "$P2" "
sleep 8
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run gesture_control moveit_cartesian_demo --ros-args \
  -p base_frame:=world \
  -p ik_link:=tool0 \
  -p joint_topic:=/joint_states_gesture \
  -p use_ur_io_handshake:=true \
  -p use_real_robot_motion:=true \
  -p trajectory_action_name:=/scaled_joint_trajectory_controller/follow_joint_trajectory \
  -p pick_request_pin:=0 \
  -p pick_done_pin:=1 \
  -p hand_back_service:=/io_and_status_controller/hand_back_control \
  -p dx:=0.005 \
  -p dy:=0.005 \
  -p dz:=0.005 \
  -p microsteps:=1 \
  -p step_axis_pause_s:=0.1 \
  -p step_move_duration_s:=0.1 \
  -p debounce_s:=0.05 \
  -p manual_axis:=1 \
  -p left_sign:=-1.0 \
" C-m

# ------------------------------------------------------------------
# Pane 3 — gesture detection
# ------------------------------------------------------------------

tmux send-keys -t "$P3" "
sleep 10
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/mp_venv/bin/activate

python3 -m gesture_control.gesture_pub --ros-args \
  -p zone_x_min:=0.0 \
  -p window_width:=1000 \
  -p window_height:=650 \
  -p zone_x_max:=1.0 \
  -p zone_y_min:=0.0 \
  -p zone_y_max:=1.0 \
  -p neutral_deadzone_x:=0.09 \
  -p neutral_deadzone_y:=0.10 \
  -p gesture_hold_frames:=5 \
  -p activation_hold_frames:=15 \
  -p command_cooldown_s:=0.25 \
  -p min_palm_facing_score:=0.35
" C-m

tmux select-layout -t $SESSION tiled
tmux attach-session -t $SESSION
