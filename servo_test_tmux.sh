#!/usr/bin/env bash
set -e

SESSION="servo_test"

tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop || true
pkill -f rviz2 || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f servo_node_main || true
pkill -f ur_control.launch.py || true
pkill -f gesture_control.gesture_pub || true
pkill -f gesture_servo_bridge || true
ros2 daemon start

tmux new-session -d -s $SESSION

# Create panes
P0=$(tmux display-message -p -t $SESSION:0.0 "#{pane_id}")
P1=$(tmux split-window -h -t "$P0" -P -F "#{pane_id}")
P2=$(tmux split-window -v -t "$P0" -P -F "#{pane_id}")
P3=$(tmux split-window -v -t "$P1" -P -F "#{pane_id}")
P4=$(tmux split-window -v -t "$P2" -P -F "#{pane_id}")

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
  initial_joint_controller:=forward_position_controller \
  launch_rviz:=false
" C-m

# ------------------------------------------------------------------
# Pane 1 — MoveIt + Servo
# ------------------------------------------------------------------
tmux send-keys -t "$P1" "
sleep 5
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur10e \
  launch_rviz:=false \
  launch_servo:=true
" C-m

# ------------------------------------------------------------------
# Pane 2 — gesture_servo_bridge
# ------------------------------------------------------------------
tmux send-keys -t "$P2" "
sleep 10
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run gesture_control gesture_servo_bridge --ros-args \
  -p command_frame:=base_link \
  -p max_vx:=1.0 \
  -p max_vy:=1.0 \
  -p xy_deadzone:=0.08 \
  -p vz:=0.00 \
  -p publish_rate:=50.0 \
  -p left_sign:=-1.0 \
  -p base_frame:=world \
  -p use_ur_io_handshake:=true \
  -p pick_request_pin:=0 \
  -p cycle_done_pin:=1 \
  -p hand_back_service:=/io_and_status_controller/hand_back_control \
  -p table_enabled:=true \
  -p table_size_x:=0.6 \
  -p table_size_y:=0.8 \
  -p table_size_z:=0.01 \
  -p table_pos_x:=-0.4 \
  -p table_pos_y:=0.0 \
  -p table_top_z:=0.0 \
  -p workspace_enabled:=true \
  -p x_min:=-0.85 \
  -p x_max:=0.1 \
  -p y_min:=-0.6 \
  -p y_max:=0.6 \
  -p inner_radius_enabled:=true \
  -p inner_radius:=0.25 \
  -p inner_radius_center_x:=0.0 \
  -p inner_radius_center_y:=0.0
" C-m

# ------------------------------------------------------------------
# Pane 3 — gesture_pub
# ------------------------------------------------------------------
tmux send-keys -t "$P3" "
sleep 12
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/mp_venv/bin/activate

python3 -m gesture_control.gesture_pub --ros-args \
  -p zone_x_min:=0.05 \
  -p zone_x_max:=0.95 \
  -p zone_y_min:=0.05 \
  -p zone_y_max:=0.95 \
  -p neutral_deadzone_x:=0.09 \
  -p neutral_deadzone_y:=0.10 \
  -p xy_publish_deadzone:=0.03 \
  -p gesture_hold_frames:=5 \
  -p activation_hold_frames:=15 \
  -p command_cooldown_s:=0.25 \
  -p min_palm_facing_score:=0.35 \
  -p window_width:=1000 \
  -p window_height:=650 \
  -p publish_image:=true \
  -p show_window:=false
" C-m

# ------------------------------------------------------------------
# Pane 4 — RViz
# ------------------------------------------------------------------
tmux send-keys -t "$P4" "
sleep 14
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
rviz2 -d ~/gesture_ws/rviz/gesture_demo.rviz
" C-m

tmux select-layout -t $SESSION tiled
tmux attach-session -t $SESSION
