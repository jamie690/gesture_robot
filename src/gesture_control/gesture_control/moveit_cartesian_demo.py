#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class MoveItCartesianDemo(Node):
    def __init__(self):
        super().__init__("moveit_cartesian_demo")

        # ------------------------------------------------------------------
        # Core robot / MoveIt params
        # ------------------------------------------------------------------
        self.group_name = self.declare_parameter("group_name", "ur_manipulator").value
        self.base_frame = self.declare_parameter("base_frame", "world").value
        self.ik_link = self.declare_parameter("ik_link", "tool0").value
        self.joint_topic = self.declare_parameter("joint_topic", "/joint_states_gesture").value

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.tf_sync_link = self.ik_link

        # ------------------------------------------------------------------
        # Motion tuning
        # ------------------------------------------------------------------
        self.dx = float(self.declare_parameter("dx", 0.01).value)
        self.dy = float(self.declare_parameter("dy", self.dx).value)
        self.dz = float(self.declare_parameter("dz", self.dx).value)
        self.microsteps = int(self.declare_parameter("microsteps", 5).value)
        self.step_axis_pause_s = float(self.declare_parameter("step_axis_pause_s", 0.15).value)
        self.debounce_s = float(self.declare_parameter("debounce_s", 0.5).value)

        self.manual_axis = int(self.declare_parameter("manual_axis", 0).value) #x=0, y=1
        self.left_sign = float(self.declare_parameter("left_sign", -1.0).value)

        self.use_real_robot_motion = bool(self.declare_parameter("use_real_robot_motion", False).value)
        self.trajectory_action_name = str(
            self.declare_parameter(
                "trajectory_action_name",
                "/scaled_joint_trajectory_controller/follow_joint_trajectory"
            ).value
        )
        self.declare_parameter("step_move_duration_s", 0.25)
        self.step_move_duration_s = float(self.get_parameter("step_move_duration_s").value)

        # Optional IK jump guard. Set to 0.0 to disable.
        self.max_joint_step_rad = float(self.declare_parameter("max_joint_step_rad", 0.0).value)

        # ------------------------------------------------------------------
        # Workspace limits
        # ------------------------------------------------------------------
        self.x_min = float(self.declare_parameter("x_min", -0.8).value)
        self.x_max = float(self.declare_parameter("x_max", 0.04).value)
        self.y_min = float(self.declare_parameter("y_min", -0.40).value)
        self.y_max = float(self.declare_parameter("y_max", 0.40).value)
        self.z_min = float(self.declare_parameter("z_min", 0.10).value)
        self.z_max = float(self.declare_parameter("z_max", 0.75).value)

        #------------------------------------------------------------------
        # Collision object (table) params
        #------------------------------------------------------------------
        self.table_enabled = bool(self.declare_parameter("table_enabled", True).value)
        self.table_size_x = float(self.declare_parameter("table_size_x", 0.8).value)
        self.table_size_y = float(self.declare_parameter("table_size_y", 0.6).value)
        self.table_size_z = float(self.declare_parameter("table_size_z", 0.02).value)

        self.table_pos_x = float(self.declare_parameter("table_pos_x", 0.0).value)
        self.table_pos_y = float(self.declare_parameter("table_pos_y", -0.4).value)
        self.table_top_z = float(self.declare_parameter("table_top_z", 0.0).value)

        # ------------------------------------------------------------------
        # UR handshake
        # ------------------------------------------------------------------
        self.use_ur_io_handshake = bool(self.declare_parameter("use_ur_io_handshake", False).value)
        self.set_io_service = str(
            self.declare_parameter("set_io_service", "/io_and_status_controller/set_io").value
        )
        self.io_states_topic = str(
            self.declare_parameter("io_states_topic", "/io_and_status_controller/io_states").value
        )
        self.robot_program_topic = str(
            self.declare_parameter(
                "robot_program_topic",
                "/io_and_status_controller/robot_program_running"
            ).value
        )
        self.hand_back_service = str(
            self.declare_parameter(
                "hand_back_service",
                "/io_and_status_controller/hand_back_control"
            ).value
        )

        self.pick_request_pin = int(self.declare_parameter("pick_request_pin", 0).value)
        self.cycle_done_pin = int(self.declare_parameter("cycle_done_pin", 1).value)


        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.q = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.latest_real_q = None

        self.target = PoseStamped()
        self.target.header.frame_id = self.base_frame
        self.target.pose.position.x = 0.5
        self.target.pose.position.y = 0.0
        self.target.pose.position.z = 0.3
        self.target.pose.orientation.x = 1.0
        self.target.pose.orientation.y = 0.0
        self.target.pose.orientation.z = 0.0
        self.target.pose.orientation.w = 0.0

        self.state = "COARSE_PICK_GUIDE"
        self.status_text = "Guide to pickup area"
        self.object_attached = False

        self.last_gesture = "none"
        self.last_cmd_time = self.get_clock().now()

        self.ik_pending = False
        self.real_motion_busy = False
        self.set_io_pending = False
        self.hand_back_pending = False

        self.robot_program_running = False
        self.cycle_done_seen = False

        self.pending_direction = 0.0
        self.pending_microsteps = 0
        self.next_microstep_time = None

        # Visual only
        self.gripper_closed = False

        # ------------------------------------------------------------------
        # ROS I/O
        # ------------------------------------------------------------------
        self.sub_gesture = self.create_subscription(String, "/gesture", self.on_gesture, 10)
        self.sub_real_js = self.create_subscription(JointState, "/joint_states", self.on_real_joint_states, 50)

        self.pub_js = self.create_publisher(JointState, self.joint_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/rg2_markers", 10)

        self.pub_planning_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)

        if self.use_ur_io_handshake:
            self.sub_io = self.create_subscription(IOStates, self.io_states_topic, self.on_io_states, 10)
            self.sub_prog = self.create_subscription(Bool, self.robot_program_topic, self.on_robot_program, 10)

            self.set_io_cli = self.create_client(SetIO, self.set_io_service)
            self.get_logger().info(f"Waiting for {self.set_io_service} ...")
            self.set_io_cli.wait_for_service()
            self.get_logger().info(f"Connected to {self.set_io_service}")

            self.hand_back_cli = self.create_client(Trigger, self.hand_back_service)
            self.get_logger().info(f"Waiting for {self.hand_back_service} ...")
            self.hand_back_cli.wait_for_service()
            self.get_logger().info(f"Connected to {self.hand_back_service}")

        # ------------------------------------------------------------------
        # TF + IK
        # ------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for /compute_ik ...")
        self.ik_cli.wait_for_service()
        self.get_logger().info("Connected to /compute_ik")

        if self.use_real_robot_motion:
            self.traj_client = ActionClient(
                self,
                FollowJointTrajectory,
                self.trajectory_action_name,
            )
            self.get_logger().info(f"Waiting for {self.trajectory_action_name} ...")
            self.traj_client.wait_for_server()
            self.get_logger().info(f"Connected to {self.trajectory_action_name}")

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self.marker_timer = self.create_timer(0.1, self.publish_markers)
        self.state_timer = self.create_timer(0.05, self.update_state_machine)

        # Initial pose sync
        self.init_target_from_tf()
        self.request_ik("startup")

        if self.table_enabled:
            self.publish_table_collision()

        self.get_logger().info(
            f"STARTUP CONFIG: manual_axis={self.manual_axis} left_sign={self.left_sign} "
            f"dx={self.dx} microsteps={self.microsteps} step_move_duration_s={self.step_move_duration_s}"
            f"IK REQUEST: group={self.group_name} ik_link={self.ik_link} frame={self.target.header.frame_id}"
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def on_real_joint_states(self, msg: JointState) -> None:
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        if all(n in name_to_pos for n in self.joint_names):
            self.latest_real_q = [name_to_pos[n] for n in self.joint_names]

    def on_robot_program(self, msg: Bool) -> None:
        self.robot_program_running = bool(msg.data)

    def on_io_states(self, msg: IOStates) -> None:
        self.cycle_done_seen = self._read_digital_state(msg.digital_out_states, self.cycle_done_pin)

    def on_gesture(self, msg: String) -> None:
        g = msg.data.strip().lower()
        self.last_gesture = g

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt < self.debounce_s:
            return

        if g == "stop":
            self.handle_stop()
            self.last_cmd_time = now
            return

        if self.state == "STOPPED":
            return

        if self.state == "COARSE_PICK_GUIDE":
            if g == "left":
                self.manual_axis = 1   # y
                self.pending_direction = -self.left_sign
                self.pending_microsteps = self.microsteps
                self.next_microstep_time = self.get_clock().now()
                self.last_cmd_time = now
                return

            if g == "right":
                self.manual_axis = 1   # y
                self.pending_direction = self.left_sign
                self.pending_microsteps = self.microsteps
                self.next_microstep_time = self.get_clock().now()
                self.last_cmd_time = now
                return

            if g == "forward":
                self.manual_axis = 0   # x
                self.pending_direction = +1.0
                self.pending_microsteps = self.microsteps
                self.next_microstep_time = self.get_clock().now()
                self.last_cmd_time = now
                self.get_logger().warn(f"GESTURE CMD: {g} -> axis={self.manual_axis} dir={self.pending_direction:+.1f}")
                return

            if g == "back":
                self.manual_axis = 0   # x
                self.pending_direction = -1.0
                self.pending_microsteps = self.microsteps
                self.next_microstep_time = self.get_clock().now()
                self.last_cmd_time = now
                self.get_logger().warn(f"GESTURE CMD: {g} -> axis={self.manual_axis} dir={self.pending_direction:+.1f}")
                return

            if g == "fist":
                self.start_cycle_sequence()
                self.last_cmd_time = now
                return

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def handle_stop(self) -> None:
        if self.use_ur_io_handshake:
            self.set_pick_request(False)

        self.pending_microsteps = 0
        self.state = "STOPPED"
        self.status_text = "STOPPED"
        self.object_attached = False
        self.gripper_closed = False
        self.get_logger().warn("Stop gesture received. System halted.")

    def start_cycle_sequence(self) -> None:
        if self.state != "COARSE_PICK_GUIDE":
            return

        self.pending_microsteps = 0
        self.state = "WAITING_FOR_CYCLE"
        self.status_text = "Cycle requested"
        self.object_attached = True
        self.gripper_closed = True
        self.get_logger().info("Cycle sequence started.")

        if self.use_ur_io_handshake:
            self.cycle_done_seen = False
            self.set_pick_request(True)

            if not self.robot_program_running:
                self.get_logger().warn("robot_program_running is false. External Control may not be active.")

            self.hand_back_control()

    def complete_cycle_sequence(self) -> None:
        self.set_pick_request(False)

        if self.latest_real_q is not None:
            self.q = list(self.latest_real_q)

        self.state = "COARSE_PICK_GUIDE"
        self.status_text = "Cycle complete - ready"
        self.object_attached = False
        self.gripper_closed = False
        self.get_logger().info("Cycle complete. Returning to guidance mode.")

    def update_state_machine(self) -> None:
        if self.state == "WAITING_FOR_CYCLE" and self.use_ur_io_handshake and self.cycle_done_seen:
            self.complete_cycle_sequence()

        if self.state == "COARSE_PICK_GUIDE" and self.pending_microsteps > 0:
            if self.use_real_robot_motion and self.real_motion_busy:
                return

            if self.next_microstep_time is None or self.get_clock().now() >= self.next_microstep_time:
                if self.use_real_robot_motion and self.latest_real_q is None:
                    self.get_logger().warn("No real joint state received yet, cannot microstep.")
                    self.pending_microsteps = 0
                    return

                if not self.sync_target_from_tf_once():
                    self.get_logger().warn("Skipping microstep due to TF sync failure")
                    self.pending_microsteps = 0
                    return

                moved = self.step_manual_target(
                    self.pending_direction,
                    scale=1.0 / float(self.microsteps),
                )

                if not moved:
                    self.pending_microsteps = 0
                    return

                self.request_ik("microstep")
                self.pending_microsteps -= 1
                self.next_microstep_time = self.get_clock().now() + Duration(seconds=self.step_axis_pause_s)

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def step_manual_target(self, direction: float, scale: float = 1.0) -> bool:
        if self.manual_axis == 0:
            step_base = self.dx
        elif self.manual_axis == 1:
            step_base = self.dy
        elif self.manual_axis == 2:
            step_base = self.dz
        elif self.manual_axis == 3:
            step_base = self.dx
        else:
            self.get_logger().warn(f"Unknown manual_axis {self.manual_axis}")
            return False
        
        step = step_base * scale 
        
        self.get_logger().warn(
            f"STEP INPUT: axis={self.manual_axis} direction={direction:+.1f} "
            f"before x={self.target.pose.position.x:.3f} y={self.target.pose.position.y:.3f}"
        )
        
        if abs(direction) > 1.0:
            self.get_logger().warn(f"Direction {direction} out of expected range [-1, 1]")
            return False
        


        if self.manual_axis == 0:
            new_x = clamp(self.target.pose.position.x + direction * step, self.x_min, self.x_max)
            if abs(new_x - self.target.pose.position.x) < 1e-9:
                return False
            self.target.pose.position.x = new_x
            self.status_text = f"Guide pickup area: x={new_x:.3f}"
            self.get_logger().warn(
                f"STEP CMD: manual_axis={self.manual_axis} direction={direction:+.1f} "
            f"target_x={self.target.pose.position.x:.3f} "
            f"target_y={self.target.pose.position.y:.3f} "
            f"target_z={self.target.pose.position.z:.3f}"
            )
            return True

        if self.manual_axis == 2:
            new_z = clamp(self.target.pose.position.z + direction * step, self.z_min, self.z_max)
            if abs(new_z - self.target.pose.position.z) < 1e-9:
                return False
            self.target.pose.position.z = new_z
            self.status_text = f"Guide pickup area: z={new_z:.3f}"
            self.get_logger().warn(
                f"STEP CMD: manual_axis={self.manual_axis} direction={direction:+.1f} "
                f"target_x={self.target.pose.position.x:.3f} "
                f"target_y={self.target.pose.position.y:.3f} "
                f"target_z={self.target.pose.position.z:.3f}"
            )
            return True

        if self.manual_axis == 3:
            q = self.target.pose.orientation
            R = self.quat_to_rot_matrix(q.x, q.y, q.z, q.w)

            # Tool Z axis in base frame (often the most intuitive "forward/back" for a downward TCP)
            vx = R[0][1]
            vy = R[1][1]
            vz = R[2][1]
            
            self.get_logger().warn(
                f"TOOL VEC: vx={vx:.4f} vy={vy:.4f} vz={vz:.4f} step={step:.6f} dir={direction:+.1f}"
            )

            new_x = clamp(self.target.pose.position.x + direction * step * vx, self.x_min, self.x_max)
            new_y = clamp(self.target.pose.position.y + direction * step * vy, self.y_min, self.y_max)
            new_z = clamp(self.target.pose.position.z + direction * step * vz, self.z_min, self.z_max)

            moved = (
                abs(new_x - self.target.pose.position.x) > 1e-9 or
                abs(new_y - self.target.pose.position.y) > 1e-9 or
                abs(new_z - self.target.pose.position.z) > 1e-9
            )
            if not moved:
                return False

            self.target.pose.position.x = new_x
            self.target.pose.position.y = new_y
            self.target.pose.position.z = new_z
            self.status_text = f"Guide pickup area: tool-forward step"
            self.get_logger().warn(
                f"STEP CMD: manual_axis={self.manual_axis} direction={direction:+.1f} "
                f"target_x={self.target.pose.position.x:.3f} "
                f"target_y={self.target.pose.position.y:.3f} "
                f"target_z={self.target.pose.position.z:.3f}"
            )
            return True

        new_y = clamp(self.target.pose.position.y + direction * step, self.y_min, self.y_max)
        if abs(new_y - self.target.pose.position.y) < 1e-9:
            return False
        self.target.pose.position.y = new_y
        self.status_text = f"Guide pickup area: y={new_y:.3f}"
        self.get_logger().warn(
            f"STEP CMD: manual_axis={self.manual_axis} direction={direction:+.1f} "
            f"target_x={self.target.pose.position.x:.3f} "
            f"target_y={self.target.pose.position.y:.3f} "
            f"target_z={self.target.pose.position.z:.3f}"
        )

        return True

    def quat_to_rot_matrix(self, qx, qy, qz, qw):
        return [
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
        ]

    def sync_target_from_tf_once(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tf_sync_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )

            self.target.header.frame_id = self.base_frame
            self.target.pose.position.x = tf.transform.translation.x 
            self.target.pose.position.y = tf.transform.translation.y
            self.target.pose.position.z = tf.transform.translation.z

            # Preserve current real orientation instead of forcing a new one
            self.target.pose.orientation = tf.transform.rotation
            self.get_logger().warn(
                f"SYNC TF: x={self.target.pose.position.x:.3f} "
                f"y={self.target.pose.position.y:.3f} "
                f"z={self.target.pose.position.z:.3f}"
            )
            return True

        except Exception as e:
            self.get_logger().warn(f"Could not sync target from TF: {e}")
            return False


    # ------------------------------------------------------------------
    # UR I/O helpers
    # ------------------------------------------------------------------

    def _read_digital_state(self, states, pin: int) -> bool:
        for s in states:
            if int(s.pin) == pin:
                return bool(s.state)
        return False

    def set_pick_request(self, on: bool) -> None:
        if not self.use_ur_io_handshake or self.set_io_pending:
            return

        req = SetIO.Request()
        req.fun = 1
        req.pin = self.pick_request_pin
        req.state = 1.0 if on else 0.0

        self.set_io_pending = True
        future = self.set_io_cli.call_async(req)
        future.add_done_callback(lambda fut: self.on_set_io_done(fut, on))

    def on_set_io_done(self, future, on: bool) -> None:
        self.set_io_pending = False
        try:
            res = future.result()
            if res is None or not res.success:
                self.get_logger().warn(f"set_io failed while setting pick_request={on}")
            else:
                self.get_logger().info(f"pick_request set to {on}")
        except Exception as e:
            self.get_logger().warn(f"set_io call failed: {e}")

    def hand_back_control(self) -> None:
        if not self.use_ur_io_handshake or self.hand_back_pending:
            return

        req = Trigger.Request()
        self.hand_back_pending = True
        future = self.hand_back_cli.call_async(req)
        future.add_done_callback(self.on_hand_back_done)

    def on_hand_back_done(self, future) -> None:
        self.hand_back_pending = False
        try:
            res = future.result()
            if res is None or not res.success:
                self.get_logger().warn("hand_back_control failed")
            else:
                self.get_logger().info("hand_back_control sent successfully")
        except Exception as e:
            self.get_logger().warn(f"hand_back_control call failed: {e}")

    # ------------------------------------------------------------------
    # TF init
    # ------------------------------------------------------------------

    def init_target_from_tf(self) -> None:
        self.get_logger().info(f"Initializing target from TF: {self.base_frame} -> {self.ik_link}")
        for _ in range(40):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.tf_sync_link,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                self.target.header.frame_id = self.base_frame
                self.target.pose.position.x = clamp(tf.transform.translation.x, self.x_min, self.x_max)
                self.target.pose.position.y = clamp(tf.transform.translation.y, self.y_min, self.y_max)
                self.target.pose.position.z = clamp(tf.transform.translation.z, self.z_min, self.z_max)
                self.target.pose.orientation = tf.transform.rotation
                self.get_logger().info(
                    f"Target set from TF: x={self.target.pose.position.x:.3f}, "
                    f"y={self.target.pose.position.y:.3f}, z={self.target.pose.position.z:.3f}"
                )
                return
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn("Could not read TF for initial target; using hard-coded target pose.")

    # ------------------------------------------------------------------
    # IK
    # ------------------------------------------------------------------

    def request_ik(self, reason: str) -> None:
        if self.ik_pending:
            return
        self.ik_pending = True

        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link

        pose = PoseStamped()
        pose.header.frame_id = self.target.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = self.target.pose
        req.ik_request.pose_stamped = pose

        rs = RobotState()
        rs.joint_state.name = list(self.joint_names)
        rs.joint_state.position = list(self.latest_real_q) if self.latest_real_q is not None else list(self.q)
        req.ik_request.robot_state = rs

        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(0.3 * 1e9)

        future = self.ik_cli.call_async(req)
        future.add_done_callback(lambda fut: self.on_ik_result(fut, reason, pose))

    def on_ik_result(self, future, reason: str, pose_sent: PoseStamped) -> None:
        self.ik_pending = False

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f"IK service call failed ({reason}): {e}")
            return

        if res is None or res.error_code.val != res.error_code.SUCCESS:
            code = None if res is None else res.error_code.val
            self.get_logger().warn(
                f"IK no-solution ({reason}). error_code={code} "
                f"pose x={pose_sent.pose.position.x:.3f} "
                f"y={pose_sent.pose.position.y:.3f} "
                f"z={pose_sent.pose.position.z:.3f}"
            )
            return

        sol = res.solution.joint_state
        name_to_pos = {n: p for n, p in zip(sol.name, sol.position)}
        if not all(n in name_to_pos for n in self.joint_names):
            self.get_logger().warn("IK returned solution missing expected UR joint names.")
            return

        q_candidate = [name_to_pos[n] for n in self.joint_names]

        if self.max_joint_step_rad > 0.0 and self.latest_real_q is not None:
            for name, a, b in zip(self.joint_names, q_candidate, self.latest_real_q):
                if abs(a - b) > self.max_joint_step_rad:
                    self.get_logger().warn(
                        f"Rejected IK jump on {name}: {abs(a-b):.3f} rad > {self.max_joint_step_rad:.3f}"
                    )
                    return

        if self.latest_real_q is not None:
            diffs = [cand - cur for cand, cur in zip(q_candidate, self.latest_real_q)]
            self.get_logger().warn(
                "IK DELTAS: " + ", ".join(
                    f"{name}={d:+.3f}" for name, d in zip(self.joint_names, diffs)
                )
            )

        if self.latest_real_q is not None:
            diffs = [a -b for a,b in zip(q_candidate, self.latest_real_q)]
            self.get_logger().warn(
                "IK DELTAS: " + ", ".join(
                    f"{name}={d:+.3f}" for name, d in zip(self.joint_names, diffs)
                )
            )

        self.q = q_candidate

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.q
        self.pub_js.publish(js)

        self.get_logger().info(
            f"{reason}: state={self.state} "
            f"x={pose_sent.pose.position.x:.3f} "
            f"y={pose_sent.pose.position.y:.3f} "
            f"z={pose_sent.pose.position.z:.3f}"
        )

        if (
            self.use_real_robot_motion
            and self.state == "COARSE_PICK_GUIDE"
            and reason in ["microstep", "startup"]
            and self.robot_program_running
        ):
            self.send_real_joint_step(self.q)

    # ------------------------------------------------------------------
    # Real robot motion
    # ------------------------------------------------------------------

    def send_real_joint_step(self, positions) -> None:
        if not self.use_real_robot_motion:
            return
        if self.real_motion_busy:
            self.get_logger().warn("Real motion busy, ignoring new command")
            return

        goal_msg = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start.sec = int(self.step_move_duration_s)
        pt.time_from_start.nanosec = int((self.step_move_duration_s % 1.0) * 1e9)

        traj.points = [pt]
        goal_msg.trajectory = traj

        self.real_motion_busy = True
        self.get_logger().info(f"Sending real joint step: {positions}")

        send_future = self.traj_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.on_real_goal_response)

    def on_real_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            self.real_motion_busy = False
            self.get_logger().warn(f"Failed to send real motion goal: {e}")
            return

        if not goal_handle.accepted:
            self.real_motion_busy = False
            self.get_logger().warn("Real motion goal rejected")
            return

        self.get_logger().info("Real motion goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_real_goal_result)

    def on_real_goal_result(self, future) -> None:
        self.real_motion_busy = False
        try:
            result = future.result().result
            self.get_logger().info(f"Real motion finished with error_code={result.error_code}")
        except Exception as e:
            self.get_logger().warn(f"Error waiting for real motion result: {e}")

    #------------------------------------------------------------------
    # Collision objects (tbl)
    #------------------------------------------------------------------
    def publish_table_collision(self) -> None:
        scene = PlanningScene()
        scene.is_diff = True

        table = CollisionObject()
        table.header.frame_id = self.base_frame
        table.id = "table"
        table.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.table_size_x,
            self.table_size_y,
            self.table_size_z,
        ]

        pose = PoseStamped().pose
        pose.position.x = self.table_pos_x
        pose.position.y = self.table_pos_y
        pose.position.z = self.table_top_z - (self.table_size_z / 2.0)
        pose.orientation.w = 1.0

        table.primitives.append(primitive)
        table.primitive_poses.append(pose)

        scene.world.collision_objects.append(table)
        self.pub_planning_scene.publish(scene)

        self.get_logger().info(
            f"Published table collision object: "
            f"size=({self.table_size_x:.3f}, {self.table_size_y:.3f}, {self.table_size_z:.3f}) "
            f"pos=({self.table_pos_x:.3f}, {self.table_pos_y:.3f}, {pose.position.z:.3f}) "
            f"in frame {self.base_frame}"
        )   


    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------

    def publish_markers(self) -> None:
        ma = MarkerArray()

        opening = 0.015 if self.gripper_closed or self.object_attached else 0.08

        jaw_thickness = 0.012
        jaw_depth = 0.02
        jaw_height = 0.06

        for i, side in enumerate([-1.0, 1.0]):
            m = Marker()
            m.header.frame_id = self.ik_link
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "rg2"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.frame_locked = True
            m.pose.position.x = 0.0
            m.pose.position.y = side * (opening / 2.0)
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = jaw_depth
            m.scale.y = jaw_thickness
            m.scale.z = jaw_height
            m.color.r = 0.9
            m.color.g = 0.9
            m.color.b = 0.9
            m.color.a = 1.0
            ma.markers.append(m)

        obj = Marker()
        obj.header.frame_id = self.ik_link
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.ns = "grasped_object"
        obj.id = 100
        obj.frame_locked = True
        if self.object_attached:
            obj.type = Marker.CUBE
            obj.action = Marker.ADD
            obj.pose.position.x = 0.0
            obj.pose.position.y = 0.0
            obj.pose.position.z = 0.11
            obj.pose.orientation.w = 1.0
            obj.scale.x = 0.03
            obj.scale.y = 0.03
            obj.scale.z = 0.05
            obj.color.r = 0.2
            obj.color.g = 0.8
            obj.color.b = 0.2
            obj.color.a = 1.0
        else:
            obj.action = Marker.DELETE
        ma.markers.append(obj)

        text = Marker()
        text.header.frame_id = self.base_frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "status"
        text.id = 200
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.frame_locked = False
        text.pose.position.x = 0.15
        text.pose.position.y = -0.45
        text.pose.position.z = 0.75
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = (
            f"State: {self.state}\n"
            f"Gesture: {self.last_gesture}\n"
            f"Cycle active: {self.object_attached}\n"
            f"Program running: {self.robot_program_running}\n"
            f"cycle_done: {self.cycle_done_seen}\n"
            f"axis={self.manual_axis} left_sign={self.left_sign:+.1f}\n"
            f"{self.status_text}"
        )
        ma.markers.append(text)

        self.pub_markers.publish(ma)

    

def main() -> None:
    rclpy.init()
    node = MoveItCartesianDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()