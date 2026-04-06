#!/usr/bin/env bash
set -e

SESSION="servo_sim"

tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop || true
pkill -f rviz2 || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f servo_node_main || true
pkill -f ur_control.launch.py || true
pkill -f gesture_control.gesture_pub || true
pkill -f gesture_servo_bridge_sim || true
ros2 daemon start

tmux new-session -d -s $SESSION

P0=$(tmux display-message -p -t $SESSION:0.0 "#{pane_id}")
P1=$(tmux split-window -h -t "$P0" -P -F "#{pane_id}")
P2=$(tmux split-window -v -t "$P0" -P -F "#{pane_id}")
P3=$(tmux split-window -v -t "$P1" -P -F "#{pane_id}")
P4=$(tmux split-window -v -t "$P2" -P -F "#{pane_id}")
P5=$(tmux split-window -v -t "$P3" -P -F "#{pane_id}")

# ------------------------------------------------------------
# Pane 0 — UR fake hardware driver
# ------------------------------------------------------------
tmux send-keys -t "$P0" "
sleep 1
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10e \
  robot_ip:=192.168.56.101 \
  use_fake_hardware:=true \
  launch_rviz:=false \
  initial_joint_controller:=joint_trajectory_controller
" C-m

# ------------------------------------------------------------
# Pane 1 — MoveIt + Servo node
# NOTE: launch_servo:=true starts the servo node, but we do NOT
# call /servo_node/start_servo until after homing + controller switch.
# ------------------------------------------------------------
tmux send-keys -t "$P1" "
sleep 5
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur10e \
  use_fake_hardware:=true \
  launch_rviz:=false \
  launch_servo:=true
" C-m

# ------------------------------------------------------------
# Pane 2 — Home robot, switch controller, seed hold pose, then start Servo
# ------------------------------------------------------------
tmux send-keys -t "$P2" "
sleep 12
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo '--- Homing with joint_trajectory_controller ---'
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
\"{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [3.1415, -1.9, 2.2, -1.8, -1.57, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}\"

sleep 5

echo '--- Switching to forward_position_controller ---'
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --activate forward_position_controller

sleep 2

echo '--- Seeding forward_position_controller with same home pose ---'
ros2 topic pub --once /forward_position_controller/commands \
std_msgs/msg/Float64MultiArray \
\"{data: [3.1415, -1.9, 2.2, -1.8, -1.57, 0.0]}\"

sleep 2

echo '--- Starting servo AFTER homing and controller switch ---'
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'
" C-m

# ------------------------------------------------------------
# Pane 3 — gesture_servo_bridge_sim
# Delayed until AFTER homing / controller switch / servo start
# ------------------------------------------------------------
tmux send-keys -t "$P3" "
sleep 24
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run gesture_control gesture_servo_bridge_sim --ros-args \
  -p sim_mode:=true \
  -p command_frame:=base_link \
  -p max_vx:=0.5 \
  -p max_vy:=0.5 \
  -p xy_deadzone:=0.08 \
  -p vz:=0.00 \
  -p publish_rate:=50.0 \
  -p left_sign:=-1.0 \
  -p base_frame:=world \
  -p use_ur_io_handshake:=false \
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

# ------------------------------------------------------------
# Pane 4 — gesture_pub
# Delayed until after robot is fully ready
# ------------------------------------------------------------
tmux send-keys -t "$P4" "
sleep 26
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

# ------------------------------------------------------------
# Pane 5 — RViz only
# ------------------------------------------------------------
tmux send-keys -t "$P5" "
sleep 16
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

rviz2 -d ~/gesture_ws/rviz/gesture_demo.rviz
" C-m

tmux select-layout -t $SESSION tiled
tmux attach-session -t $SESSION
