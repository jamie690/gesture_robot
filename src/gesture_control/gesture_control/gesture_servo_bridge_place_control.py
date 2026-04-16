#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger

import tf2_ros


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class GestureServoBridge(Node):
    def __init__(self):
        super().__init__("gesture_servo_bridge")

        # ------------------------------------------------------------
        # Core Servo params
        # ------------------------------------------------------------
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("publish_rate", 50.0)

        self.declare_parameter("vx", 0.03)
        self.declare_parameter("vy", 0.03)
        self.declare_parameter("vz", 0.00)

        self.declare_parameter("left_sign", -1.0)
        self.declare_parameter("enable_z", False)

        self.declare_parameter("hand_xy_timeout_s", 0.15)

        # ------------------------------------------------------------
        # Table collision params
        # ------------------------------------------------------------
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("table_enabled", True)
        self.declare_parameter("table_size_x", 0.8)
        self.declare_parameter("table_size_y", 0.6)
        self.declare_parameter("table_size_z", 0.02)
        self.declare_parameter("table_pos_x", 0.0)
        self.declare_parameter("table_pos_y", -0.4)
        self.declare_parameter("table_top_z", 0.0)

        # ------------------------------------------------------------
        # Pick-cycle / handshake params
        # ------------------------------------------------------------
        self.declare_parameter("use_ur_io_handshake", True)
        self.declare_parameter("set_io_service", "/io_and_status_controller/set_io")
        self.declare_parameter("io_states_topic", "/io_and_status_controller/io_states")
        self.declare_parameter("robot_program_topic", "/io_and_status_controller/robot_program_running")
        self.declare_parameter("hand_back_service", "/io_and_status_controller/hand_back_control")
        self.declare_parameter("pick_request_pin", 0)
        self.declare_parameter("cycle_done_pin", 1)

        # ------------------------------------------------------------
        # Place-control params
        # ------------------------------------------------------------
        self.declare_parameter("enable_place_control", True)
        self.declare_parameter("place_release_frames", 4)
        self.declare_parameter("place_z", 0.10)
        self.declare_parameter("place_lift_z", 0.32)
        self.declare_parameter("place_down_speed", 0.5)
        self.declare_parameter("place_up_speed", 0.5)
        self.declare_parameter("place_z_tol", 0.01)

        # Backend options:
        #   "none"      -> demo only, just drop internal marker and lift back up
        #   "polyscope" -> hand place-open to Polyscope using request/done pins
        self.declare_parameter("gripper_backend", "polyscope")
        self.declare_parameter("place_request_pin", 2)
        self.declare_parameter("place_done_pin", 3)
        self.declare_parameter("pick_success_pin", 4)

        # ------------------------------------------------------------
        # Workspace limits
        # ------------------------------------------------------------
        self.declare_parameter("workspace_enabled", True)
        self.declare_parameter("x_min", -0.05)
        self.declare_parameter("x_max", 0.75)
        self.declare_parameter("y_min", -0.55)
        self.declare_parameter("y_max", 0.55)

        self.declare_parameter("inner_radius_enabled", True)
        self.declare_parameter("inner_radius", 0.28)
        self.declare_parameter("inner_radius_center_x", 0.0)
        self.declare_parameter("inner_radius_center_y", 0.0)

        self.declare_parameter("workspace_marker_z", 0.001)
        self.declare_parameter("workspace_marker_height", 0.002)

        # ------------------------------------------------------------
        # Diagonal motion params
        # ------------------------------------------------------------
        self.declare_parameter("max_vx", 0.4)
        self.declare_parameter("max_vy", 0.4)
        self.declare_parameter("xy_deadzone", 0.05)

        #-------------------------------------------------------------
        # Reorient gripper params
        #-------------------------------------------------------------
        self.declare_parameter("enable_post_pick_reorient", True)
        self.declare_parameter("carry_yaw_deg", 90.0)
        self.declare_parameter("carry_yaw_tol_deg", 1.0)
        self.declare_parameter("reorient_wz_max", 2.0)

        self.declare_parameter("align_down_enabled", True)
        self.declare_parameter("tilt_tol_deg", 3.0)
        self.declare_parameter("tilt_kp", 3.0)
        self.declare_parameter("tilt_w_max", 0.8)

        # ------------------------------------------------------------
        # Read params
        # ------------------------------------------------------------
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.command_frame = str(self.get_parameter("command_frame").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.vz = float(self.get_parameter("vz").value)

        self.left_sign = float(self.get_parameter("left_sign").value)
        self.enable_z = bool(self.get_parameter("enable_z").value)

        self.hand_xy_timeout_s = float(self.get_parameter("hand_xy_timeout_s").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.table_enabled = bool(self.get_parameter("table_enabled").value)
        self.table_size_x = float(self.get_parameter("table_size_x").value)
        self.table_size_y = float(self.get_parameter("table_size_y").value)
        self.table_size_z = float(self.get_parameter("table_size_z").value)
        self.table_pos_x = float(self.get_parameter("table_pos_x").value)
        self.table_pos_y = float(self.get_parameter("table_pos_y").value)
        self.table_top_z = float(self.get_parameter("table_top_z").value)

        self.use_ur_io_handshake = bool(self.get_parameter("use_ur_io_handshake").value)
        self.set_io_service = str(self.get_parameter("set_io_service").value)
        self.io_states_topic = str(self.get_parameter("io_states_topic").value)
        self.robot_program_topic = str(self.get_parameter("robot_program_topic").value)
        self.hand_back_service = str(self.get_parameter("hand_back_service").value)
        self.pick_request_pin = int(self.get_parameter("pick_request_pin").value)
        self.cycle_done_pin = int(self.get_parameter("cycle_done_pin").value)
        self.pick_success_pin = int(self.get_parameter("pick_success_pin").value)

        self.enable_place_control = bool(self.get_parameter("enable_place_control").value)
        self.place_release_frames = int(self.get_parameter("place_release_frames").value)
        self.place_z = float(self.get_parameter("place_z").value)
        self.place_lift_z = float(self.get_parameter("place_lift_z").value)
        self.place_down_speed = float(self.get_parameter("place_down_speed").value)
        self.place_up_speed = float(self.get_parameter("place_up_speed").value)
        self.place_z_tol = float(self.get_parameter("place_z_tol").value)
        self.gripper_backend = str(self.get_parameter("gripper_backend").value).strip().lower()
        self.place_request_pin = int(self.get_parameter("place_request_pin").value)
        self.place_done_pin = int(self.get_parameter("place_done_pin").value)

        self.workspace_enabled = bool(self.get_parameter("workspace_enabled").value)
        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)

        self.inner_radius_enabled = bool(self.get_parameter("inner_radius_enabled").value)
        self.inner_radius = float(self.get_parameter("inner_radius").value)
        self.inner_radius_center_x = float(self.get_parameter("inner_radius_center_x").value)
        self.inner_radius_center_y = float(self.get_parameter("inner_radius_center_y").value)

        self.workspace_marker_z = float(self.get_parameter("workspace_marker_z").value)
        self.workspace_marker_height = float(self.get_parameter("workspace_marker_height").value)

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.xy_deadzone = float(self.get_parameter("xy_deadzone").value)

        self.enable_post_pick_reorient = bool(self.get_parameter("enable_post_pick_reorient").value)
        self.carry_yaw_deg = float(self.get_parameter("carry_yaw_deg").value)
        self.carry_yaw_tol_deg = float(self.get_parameter("carry_yaw_tol_deg").value)
        self.reorient_wz_max = float(self.get_parameter("reorient_wz_max").value)

        self.align_down_enabled = bool(self.get_parameter("align_down_enabled").value)
        self.tilt_tol_deg = float(self.get_parameter("tilt_tol_deg").value)
        self.tilt_kp = float(self.get_parameter("tilt_kp").value)
        self.tilt_w_max = float(self.get_parameter("tilt_w_max").value)


        # ------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------
        self.current_gesture = "none"
        self.last_logged_gesture = None

        self.robot_program_running = False
        self.cycle_done_seen = False
        self.place_done_seen = False
        self.object_attached = False
        self.gripper_closed = False
        self.state = "COARSE_PICK_GUIDE"
        self.status_text = "Servo guidance active"

        self.set_io_pending = False
        self.hand_back_pending = False
        self.prev_cycle_done_seen = False
        self.prev_place_done_seen = False
        self.pick_success_seen = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_tcp_x = None
        self.current_tcp_y = None
        self.current_tcp_z = None

        self.hand_dx = 0.0
        self.hand_dy = 0.0
        self.last_hand_xy_time = self.get_clock().now()
        self.place_release_count = 0

        self.last_z_debug_time = self.get_clock().now()

        self.pre_pick_yaw = 0.0
        self.have_pre_pick_yaw = False


        # ------------------------------------------------------------
        # ROS I/O
        # ------------------------------------------------------------
        self.sub_hand_xy = self.create_subscription(Vector3, "/hand_xy", self.on_hand_xy, 10)
        self.sub_gesture = self.create_subscription(String, "/gesture", self.on_gesture, 10)

        self.pub_twist = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/rg2_markers", 10)
        self.pub_collision_object = self.create_publisher(CollisionObject, "/collision_object", 10)



        if self.use_ur_io_handshake:
            self.sub_io = self.create_subscription(IOStates, self.io_states_topic, self.on_io_states, 10)
            self.sub_prog = self.create_subscription(Bool, self.robot_program_topic, self.on_robot_program, 10)

            self.set_io_cli = self.create_client(SetIO, self.set_io_service)
            self.hand_back_cli = self.create_client(Trigger, self.hand_back_service)

            self.get_logger().info(f"Waiting for {self.set_io_service} ...")
            self.set_io_cli.wait_for_service()
            self.get_logger().info(f"Connected to {self.set_io_service}")

            self.get_logger().info(f"Waiting for {self.hand_back_service} ...")
            self.hand_back_cli.wait_for_service()
            self.get_logger().info(f"Connected to {self.hand_back_service}")

        self.sub_fist_pressed = self.create_subscription(
            Bool, "/fist_pressed", self.on_fist_pressed, 10
        )


        # ------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------
        self.twist_timer = self.create_timer(1.0 / self.publish_rate, self.publish_twist)
        self.marker_timer = self.create_timer(0.1, self.publish_markers)
        self.table_timer = self.create_timer(2.0, self.republish_table)

        if self.table_enabled:
            self.publish_table_collision()

        self.get_logger().info(
            f"gesture_servo_bridge started. frame={self.command_frame} "
            f"publish_rate={self.publish_rate:.1f} Hz backend={self.gripper_backend}"
        )

    # ------------------------------------------------------------
    # Gesture handling
    # ------------------------------------------------------------
    def on_gesture(self, msg: String) -> None:
        g = msg.data.strip().lower()

        if g != self.last_logged_gesture:
            self.get_logger().info(f"Gesture -> {g}")
            self.last_logged_gesture = g

        if self.state in ["COARSE_PICK_GUIDE", "COARSE_PLACE_GUIDE"]:
            self.current_gesture = g

    def on_hand_xy(self, msg: Vector3) -> None:
        self.hand_dx = float(msg.x)
        self.hand_dy = float(msg.y)
        self.last_hand_xy_time = self.get_clock().now()

    # ------------------------------------------------------------
    # Servo publishing
    # ------------------------------------------------------------
    def publish_twist(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame

        vx = 0.0
        vy = 0.0
        vz = 0.0
        wx = 0.0
        wy = 0.0
        wz = 0.0

        if self.state == "COARSE_PICK_GUIDE":
            if self.current_gesture == "fist":
                self.status_text = "Pick requested"
            else:
                vx, vy = self.manual_xy_from_hand(timeout_text="Hand signal timeout")
                self.status_text = "Servo guidance active"

        elif self.state == "COARSE_PLACE_GUIDE":
            if self.current_gesture == "fist":
                self.place_release_count = 0
                vx, vy = self.manual_xy_from_hand(timeout_text="Carry mode: hand signal timeout")
                self.status_text = "Carry mode: hold fist to move, release to place"
            else:
                vx = 0.0
                vy = 0.0
                vz = 0.0
                self.place_release_count += 1
                self.status_text = (
                    f"Carry mode: release detected "
                    f"({self.place_release_count}/{self.place_release_frames})"
                )

                if self.place_release_count >= self.place_release_frames:
                    self.state = "PLACE_DESCEND"
                    self.status_text = "Place: descending"
                    self.get_logger().info("Release confirmed. Starting ROS place sequence.")

        elif self.state == "PLACE_DESCEND":
            vz, reached = self.step_towards_z(self.place_z, self.place_down_speed)
            if self.current_tcp_z is not None:
                self.status_text = f"Place: lowering z={self.current_tcp_z:.3f} target={self.place_z:.3f} vz={vz:.3f}"
            else:
                self.status_text = "Place: lowering"
            if reached:
                self.on_place_height_reached()

        elif self.state == "PLACE_ASCEND":
            vz, reached = self.step_towards_z(self.place_lift_z, self.place_up_speed)
            self.status_text = "Place: lifting"
            if reached:
                self.finish_place_sequence()

        elif self.state == "WAITING_FOR_CYCLE":
            self.status_text = "Waiting for Polyscope pick cycle"

        elif self.state == "WAITING_FOR_PLACE":
            self.status_text = "Waiting for Polyscope gripper release"

        elif self.state == "POST_PICK_REORIENT":
            wz, yaw_ok = self.step_reorient_yaw()

            if self.align_down_enabled:
                wx, wy, tilt_ok = self.step_align_down()
            else:
                wx, wy, tilt_ok = 0.0, 0.0, True

            self.status_text = (
                f"Post-pick: reorient yaw={'ok' if yaw_ok else '...'} "
                f"tilt={'ok' if tilt_ok else '...'}"
            )

            if yaw_ok and tilt_ok:
                self.state = "COARSE_PLACE_GUIDE"
                self.current_gesture = "fist"
                self.place_release_count = 0
                self.status_text = "Carry mode: hold fist to move, release to place"
                self.get_logger().info("Post-pick reorient complete. Entered carry mode.")

        if self.state in ["COARSE_PICK_GUIDE", "COARSE_PLACE_GUIDE"] and (vx != 0.0 or vy != 0.0):
            vx, vy = self.apply_workspace_guard(vx, vy)

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self.pub_twist.publish(msg)

    def manual_xy_from_hand(self, timeout_text: str):
        age = (self.get_clock().now() - self.last_hand_xy_time).nanoseconds / 1e9
        if age > self.hand_xy_timeout_s:
            self.status_text = timeout_text
            return 0.0, 0.0

        dx = self.hand_dx
        dy = self.hand_dy
        mag = math.hypot(dx, dy)
        if mag < self.xy_deadzone:
            return 0.0, 0.0

        vx = self.max_vx * (-dy)
        vy = -self.max_vy * dx
        return vx, vy

    # ------------------------------------------------------------
    # Pick / place sequencing
    # ------------------------------------------------------------
    def start_pick_sequence(self) -> None:
        if self.state != "COARSE_PICK_GUIDE":
            return
        
        if self.set_io_pending or self.hand_back_pending:
            self.get_logger().warn("Pick start ignored: handshake still busy")
            return

        if self.update_current_tcp_pose():
            self.pre_pick_yaw = self.quat_to_yaw(
                self.current_tcp_qx,
                self.current_tcp_qy,
                self.current_tcp_qz,
                self.current_tcp_qw,
            )
            self.have_pre_pick_yaw = True
        else:
            self.have_pre_pick_yaw = False

        self.state = "WAITING_FOR_CYCLE"
        self.status_text = "Pick cycle requested"
        self.current_gesture = "none"
        self.place_release_count = 0

        self.get_logger().info("Pick sequence started.")

        if self.use_ur_io_handshake:
            self.cycle_done_seen = False
            self.prev_cycle_done_seen = False
            self.set_digital_output(self.pick_request_pin, True, label="pick_request")

            if not self.robot_program_running:
                self.get_logger().warn(
                    "robot_program_running is false. External Control may not be active."
                )

            self.hand_back_control()

    def on_robot_program(self, msg: Bool) -> None:
        self.robot_program_running = bool(msg.data)

    def on_io_states(self, msg: IOStates) -> None:
        cycle_current = self._read_digital_state(msg.digital_out_states, self.cycle_done_pin)
        cycle_rising = cycle_current and not self.prev_cycle_done_seen
        self.prev_cycle_done_seen = cycle_current
        self.cycle_done_seen = cycle_current

        place_current = self._read_digital_state(msg.digital_out_states, self.place_done_pin)
        place_rising = place_current and not self.prev_place_done_seen
        self.prev_place_done_seen = place_current
        self.place_done_seen = place_current

        pick_success_current = self._read_digital_state(msg.digital_out_states, self.pick_success_pin)
        self.pick_success_seen = pick_success_current

        if self.state == "WAITING_FOR_CYCLE" and cycle_rising:
            self.complete_pick_sequence()

        if self.state == "WAITING_FOR_PLACE" and place_rising:
            self.complete_polyscope_release()

    def _read_digital_state(self, states, pin: int) -> bool:
        for s in states:
            if int(s.pin) == pin:
                return bool(s.state)
        return False

    def complete_pick_sequence(self) -> None:
        self.set_digital_output(self.pick_request_pin, False, label="pick_request")

        if not self.enable_place_control:
            self.state = "COARSE_PICK_GUIDE"
            self.status_text = "Pick complete - ready"
            self.current_gesture = "none"
            self.get_logger().info("Pick complete. Place control disabled.")
            return

        if self.pick_success_seen:
            self.object_attached = True
            self.gripper_closed = True
            self.current_gesture = "fist"
            self.place_release_count = 0

            if self.enable_post_pick_reorient:
                self.state = "POST_PICK_REORIENT"
                self.status_text = "Post-pick: reorienting gripper"
                self.get_logger().info("Pick successful. Starting ROS post-pick reorient.")
            else:
                self.state = "COARSE_PLACE_GUIDE"
                self.status_text = "Carry mode: hold fist to move, release to place"
                self.get_logger().info("Pick successful. Entered carry mode.")
        else:
            self.object_attached = False
            self.gripper_closed = False
            self.state = "COARSE_PICK_GUIDE"
            self.current_gesture = "none"
            self.place_release_count = 0
            self.status_text = "Pick failed / no object"
            self.get_logger().info("Pick finished but no object detected. Returning to pick guidance.")

    def on_place_height_reached(self) -> None:
        self.get_logger().info("Reached place height.")

        if self.gripper_backend == "none":
            self.object_attached = False
            self.gripper_closed = False
            self.state = "PLACE_ASCEND"
            self.status_text = "Place: released (visual only), lifting"
            return

        if self.gripper_backend == "polyscope":
            self.state = "WAITING_FOR_PLACE"
            self.status_text = "Place: handover to Polyscope for gripper open"
            self.place_done_seen = False
            self.prev_place_done_seen = False
            self.set_digital_output(self.place_request_pin, True, label="place_request")
            self.hand_back_control()
            return

        self.get_logger().warn(
            f"Unsupported gripper_backend='{self.gripper_backend}'. Falling back to visual release only."
        )
        self.object_attached = False
        self.gripper_closed = False
        self.state = "PLACE_ASCEND"
        self.status_text = "Place: fallback release, lifting"

    def complete_polyscope_release(self) -> None:
        self.set_digital_output(self.place_request_pin, False, label="place_request")
        self.object_attached = False
        self.gripper_closed = False
        self.state = "PLACE_ASCEND"
        self.status_text = "Place: release done, lifting"
        self.get_logger().info("Polyscope release complete. Lifting back up.")

    def finish_place_sequence(self) -> None:
        self.state = "COARSE_PICK_GUIDE"
        self.current_gesture = "none"
        self.place_release_count = 0
        self.status_text = "Servo guidance active"
        self.get_logger().info("ROS place sequence complete. Returning to pick guidance mode.")

    def step_towards_z(self, target_z: float, max_speed: float):
        if not self.update_current_tcp_pose():
            return 0.0, False

        ez = target_z - self.current_tcp_z
        if abs(ez) < self.place_z_tol:
            return 0.0, True

        kz = 7.0
        min_speed = 1.0

        raw_vz = kz * ez
        vz = clamp(raw_vz, -max_speed, max_speed)

        if abs(vz) < min_speed:
            vz = math.copysign(min_speed, ez)

        if abs(ez) < 0.01:
            vz = clamp(vz, -1.0, 1.0)

        return vz, False
    
    def on_fist_pressed(self, msg: Bool) -> None:
        if not msg.data:
            return

        if self.state == "COARSE_PICK_GUIDE":
            self.get_logger().info("Fresh fist press -> starting pick")
            self.start_pick_sequence()
    
    def quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_to_pi(self, a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def step_reorient_yaw(self):
        if not self.update_current_tcp_pose():
            return 0.0, False

        yaw_now = self.quat_to_yaw(
            self.current_tcp_qx,
            self.current_tcp_qy,
            self.current_tcp_qz,
            self.current_tcp_qw,
        )

        if self.have_pre_pick_yaw:
            yaw_target = self.pre_pick_yaw
        else:
            yaw_target = math.radians(self.carry_yaw_deg)

        yaw_err = self.wrap_to_pi(yaw_target - yaw_now)

        tol = math.radians(self.carry_yaw_tol_deg)
        if abs(yaw_err) < tol:
            return 0.0, True

        kp = 7.5
        raw_wz = kp * yaw_err
        wz = clamp(raw_wz, -self.reorient_wz_max, self.reorient_wz_max)

        # keep it moving, but don't let it hunt too hard near the end
        min_wz = 3.0
        if abs(wz) < min_wz:
            wz = math.copysign(min_wz, yaw_err)

        if abs(yaw_err) < math.radians(2.0):
            wz = clamp(wz, -1.0, 1.0)

        return wz, False
    
    def quat_to_rotmat(self, x: float, y: float, z: float, w: float):
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z

        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
            [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
        ]


    def step_align_down(self):
        if not self.update_current_tcp_pose():
            return 0.0, 0.0, False

        R = self.quat_to_rotmat(
            self.current_tcp_qx,
            self.current_tcp_qy,
            self.current_tcp_qz,
            self.current_tcp_qw,
        )

        # tool Z axis expressed in base/world frame
        ax = R[0][2]
        ay = R[1][2]
        az = R[2][2]

        # desired "point straight down"
        dx, dy, dz = 0.0, 0.0, -1.0

        # correction axis = current × desired
        ex = ay * dz - az * dy
        ey = az * dx - ax * dz
        ez = ax * dy - ay * dx

        tilt_mag = math.sqrt(ex * ex + ey * ey + ez * ez)

        if tilt_mag < math.sin(math.radians(self.tilt_tol_deg)):
            return 0.0, 0.0, True

        wx = clamp(self.tilt_kp * ex, -self.tilt_w_max, self.tilt_w_max)
        wy = clamp(self.tilt_kp * ey, -self.tilt_w_max, self.tilt_w_max)


        return wx, wy, False

    # ------------------------------------------------------------
    # I/O helpers
    # ------------------------------------------------------------
    def set_digital_output(self, pin: int, on: bool, label: str = "io") -> None:
        if not self.use_ur_io_handshake or self.set_io_pending:
            return

        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = 1.0 if on else 0.0

        self.set_io_pending = True
        future = self.set_io_cli.call_async(req)
        future.add_done_callback(lambda fut: self.on_set_io_done(fut, label, on))

    def on_set_io_done(self, future, label: str, on: bool) -> None:
        self.set_io_pending = False
        try:
            res = future.result()
            if res is None or not res.success:
                self.get_logger().warn(f"set_io failed while setting {label}={on}")
            else:
                self.get_logger().info(f"{label} set to {on}")
        except Exception as e:
            self.get_logger().warn(f"set_io call failed for {label}: {e}")

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

    def destroy_node(self):
        try:
            if self.use_ur_io_handshake:
                self.set_digital_output(self.pick_request_pin, False, label="pick_request")
                self.set_digital_output(self.place_request_pin, False, label="place_request")
        except Exception:
            pass
        super().destroy_node()

    # ------------------------------------------------------------
    # Workspace helpers
    # ------------------------------------------------------------
    def update_current_tcp_pose(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                "tool0",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            self.current_tcp_x = tf.transform.translation.x
            self.current_tcp_y = tf.transform.translation.y
            self.current_tcp_z = tf.transform.translation.z
            self.current_tcp_qx = tf.transform.rotation.x
            self.current_tcp_qy = tf.transform.rotation.y
            self.current_tcp_qz = tf.transform.rotation.z
            self.current_tcp_qw = tf.transform.rotation.w
            return True
        except Exception:
            return False

    def candidate_pose_allowed(self, x: float, y: float) -> bool:
        if not self.workspace_enabled:
            return True

        if x < self.x_min or x > self.x_max:
            return False
        if y < self.y_min or y > self.y_max:
            return False

        if self.inner_radius_enabled:
            dx = x - self.inner_radius_center_x
            dy = y - self.inner_radius_center_y
            if math.hypot(dx, dy) < self.inner_radius:
                return False

        return True

    def workspace_violation(self, x: float, y: float) -> float:
        violation = 0.0

        if x < self.x_min:
            violation += self.x_min - x
        elif x > self.x_max:
            violation += x - self.x_max

        if y < self.y_min:
            violation += self.y_min - y
        elif y > self.y_max:
            violation += y - self.y_max

        if self.inner_radius_enabled:
            dx = x - self.inner_radius_center_x
            dy = y - self.inner_radius_center_y
            r = math.hypot(dx, dy)
            if r < self.inner_radius:
                violation += self.inner_radius - r

        return violation

    def apply_workspace_guard(self, vx: float, vy: float):
        if not self.update_current_tcp_pose():
            return vx, vy

        horizon = 0.15
        cur_x = self.current_tcp_x
        cur_y = self.current_tcp_y
        cand_x = cur_x + vx * horizon
        cand_y = cur_y + vy * horizon

        cur_ok = self.candidate_pose_allowed(cur_x, cur_y)
        cand_ok = self.candidate_pose_allowed(cand_x, cand_y)

        if cur_ok:
            if not cand_ok:
                self.get_logger().warn(
                    f"Workspace block: cur=({cur_x:.3f}, {cur_y:.3f}) "
                    f"cand=({cand_x:.3f}, {cand_y:.3f})"
                )
                self.status_text = "Workspace limit reached"
                return 0.0, 0.0
        else:
            cur_v = self.workspace_violation(cur_x, cur_y)
            cand_v = self.workspace_violation(cand_x, cand_y)
            if cand_v > (cur_v - 1e-4):
                self.status_text = "Move back toward workspace"
                return 0.0, 0.0

        return vx, vy
    


    # ------------------------------------------------------------
    # Table collision object
    # ------------------------------------------------------------
    def republish_table(self) -> None:
        if self.table_enabled:
            self.publish_table_collision()

    def publish_table_collision(self) -> None:
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

        pose = Pose()
        pose.position.x = self.table_pos_x
        pose.position.y = self.table_pos_y
        pose.position.z = self.table_top_z - (self.table_size_z / 2.0)
        pose.orientation.w = 1.0

        table.primitives.append(primitive)
        table.primitive_poses.append(pose)

        self.pub_collision_object.publish(table)

    # ------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------
    def publish_markers(self) -> None:
        ma = MarkerArray()

        opening = 0.015 if self.gripper_closed or self.object_attached else 0.08

        jaw_thickness = 0.02
        jaw_depth = 0.011
        jaw_height = 0.06

        for i, side in enumerate([-1.0, 1.0]):
            m = Marker()
            m.header.frame_id = "tool0"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "rg2"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.frame_locked = True
            m.pose.position.x = side * (opening / 2.0)
            m.pose.position.y = 0.0
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
        obj.header.frame_id = "tool0"
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

        if self.workspace_enabled and self.inner_radius_enabled:
            inner = Marker()
            inner.header.frame_id = self.base_frame
            inner.header.stamp = self.get_clock().now().to_msg()
            inner.ns = "workspace"
            inner.id = 300
            inner.type = Marker.CYLINDER
            inner.action = Marker.ADD
            inner.pose.position.x = self.inner_radius_center_x
            inner.pose.position.y = self.inner_radius_center_y
            inner.pose.position.z = self.workspace_marker_z
            inner.pose.orientation.w = 1.0
            inner.scale.x = 2.0 * self.inner_radius
            inner.scale.y = 2.0 * self.inner_radius
            inner.scale.z = self.workspace_marker_height
            inner.color.r = 1.0
            inner.color.g = 0.0
            inner.color.b = 0.0
            inner.color.a = 0.35
            ma.markers.append(inner)

        if self.workspace_enabled:
            rear = Marker()
            rear.header.frame_id = self.base_frame
            rear.header.stamp = self.get_clock().now().to_msg()
            rear.ns = "workspace"
            rear.id = 301
            rear.type = Marker.LINE_STRIP
            rear.action = Marker.ADD
            rear.pose.orientation.w = 1.0
            rear.scale.x = 0.01
            rear.color.r = 1.0
            rear.color.g = 0.4
            rear.color.b = 0.0
            rear.color.a = 1.0

            p1 = Pose().position
            p1.x = self.x_min
            p1.y = self.y_min
            p1.z = self.workspace_marker_z

            p2 = Pose().position
            p2.x = self.x_min
            p2.y = self.y_max
            p2.z = self.workspace_marker_z

            rear.points.append(p1)
            rear.points.append(p2)
            ma.markers.append(rear)

        if self.workspace_enabled:
            box = Marker()
            box.header.frame_id = self.base_frame
            box.header.stamp = self.get_clock().now().to_msg()
            box.ns = "workspace"
            box.id = 302
            box.type = Marker.LINE_STRIP
            box.action = Marker.ADD
            box.pose.orientation.w = 1.0
            box.scale.x = 0.008
            box.color.r = 0.0
            box.color.g = 0.8
            box.color.b = 1.0
            box.color.a = 1.0

            corners = [
                (self.x_min, self.y_min),
                (self.x_max, self.y_min),
                (self.x_max, self.y_max),
                (self.x_min, self.y_max),
                (self.x_min, self.y_min),
            ]

            for x, y in corners:
                p = Pose().position
                p.x = x
                p.y = y
                p.z = self.workspace_marker_z
                box.points.append(p)

            ma.markers.append(box)

        text = Marker()
        text.header.frame_id = self.base_frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "status"
        text.id = 200
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.15
        text.pose.position.y = 0.45
        text.pose.position.z = 1.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = (
            f"State: {self.state}\n"
            f"Gesture: {self.current_gesture}\n"
            f"Object attached: {self.object_attached}\n"
            f"Program running: {self.robot_program_running}\n"
            f"cycle_done: {self.cycle_done_seen}\n"
            f"place_done: {self.place_done_seen}\n"
            f"backend={self.gripper_backend}\n"
            f"{self.status_text}"
        )
        ma.markers.append(text)

        self.pub_markers.publish(ma)


def main() -> None:
    rclpy.init()
    node = GestureServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
