#!/usr/bin/env bash
set -e

SESSION="gesture_real_demo"

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

P0=$(tmux display-message -p -t $SESSION:0.0 "#{pane_id}")
P1=$(tmux split-window -h -t "$P0" -P -F "#{pane_id}")
P2=$(tmux split-window -v -t "$P0" -P -F "#{pane_id}")
P3=$(tmux split-window -v -t "$P1" -P -F "#{pane_id}")
P4=$(tmux split-window -v -t "$P2" -P -F "#{pane_id}")
P5=$(tmux split-window -v -t "$P3" -P -F "#{pane_id}")

# Pane 0: UR driver
tmux send-keys -t "$P0" "sleep 1; source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.69 launch_rviz:=false" C-m

# Pane 1: MoveIt
tmux send-keys -t "$P1" "sleep 2; source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=false" C-m

# Pane 2: robot_state_publisher for RViz model
tmux send-keys -t "$P2" "sleep 2; source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ROBOT_DESCRIPTION=\$(xacro \$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 name:=ur ur_type:=ur5e tf_prefix:=\"\"); ros2 run robot_state_publisher robot_state_publisher --ros-args -r /joint_states:=/joint_states_gesture -p robot_description:=\"\$ROBOT_DESCRIPTION\"" C-m

# Pane 3: RViz
tmux send-keys -t "$P3" "sleep 3; source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; rviz2 -d \$(ros2 pkg prefix ur_description)/share/ur_description/rviz/view_robot.rviz" C-m

# Pane 4: gesture publisher
tmux send-keys -t "$P4" "sleep 3; source /opt/ros/humble/setup.bash; source ~/gesture_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source ~/mp_venv/bin/activate; python3 -m gesture_control.gesture_pub" C-m

# Pane 5: controller node
tmux send-keys -t "$P5" "sleep 4; source /opt/ros/humble/setup.bash; source ~/gesture_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 run gesture_control moveit_cartesian_demo --ros-args -p base_frame:=world -p ik_link:=tool0 -p joint_topic:=/joint_states_gesture -p use_ur_io_handshake:=true -p pick_request_pin:=0 -p pick_done_pin:=1 -p hand_back_service:=/io_and_status_controller/hand_back_control" C-m

tmux select-layout -t $SESSION tiled
tmux attach-session -t $SESSION
