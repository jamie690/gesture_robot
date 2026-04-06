#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger

import tf2_ros

from geometry_msgs.msg import Vector3


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
        self.declare_parameter("use_ur_io_handshake", False)
        self.declare_parameter("set_io_service", "/io_and_status_controller/set_io")
        self.declare_parameter("io_states_topic", "/io_and_status_controller/io_states")
        self.declare_parameter("robot_program_topic", "/io_and_status_controller/robot_program_running")
        self.declare_parameter("hand_back_service", "/io_and_status_controller/hand_back_control")
        self.declare_parameter("pick_request_pin", 0)
        self.declare_parameter("cycle_done_pin", 1)

        #------------------------------------------------
        # Workspace limits
        #------------------------------------------------
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

        #--------------------------------------------
        # Diagonal motion params
        #--------------------------------------------
        self.declare_parameter("max_vx", 0.4)
        self.declare_parameter("max_vy", 0.4)
        self.declare_parameter("xy_deadzone", 0.05)

        self.declare_parameter("sim_mode", True)

        #--------------------------------------------
        # Sim pick cycle params
        #--------------------------------------------
        self.declare_parameter("sim_pick_down_speed", 0.4)
        self.declare_parameter("sim_pick_up_speed", 0.4)
        self.declare_parameter("sim_pick_drop_x", -0.1)
        self.declare_parameter("sim_pick_drop_y", -0.30)
        self.declare_parameter("sim_pick_z_pick", 0.1)
        self.declare_parameter("sim_pick_z_lift", 0.32)
        self.declare_parameter("sim_drop_xy_tol", 0.03)
        self.declare_parameter("sim_drop_z_tol", 0.02)
        self.declare_parameter("place_release_frames", 4)

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

        self.sim_mode = bool(self.get_parameter("sim_mode").value)

        self.sim_pick_down_speed = float(self.get_parameter("sim_pick_down_speed").value)
        self.sim_pick_up_speed = float(self.get_parameter("sim_pick_up_speed").value)
        self.sim_pick_drop_x = float(self.get_parameter("sim_pick_drop_x").value)
        self.sim_pick_drop_y = float(self.get_parameter("sim_pick_drop_y").value)
        self.sim_pick_z_pick = float(self.get_parameter("sim_pick_z_pick").value)
        self.sim_pick_z_lift = float(self.get_parameter("sim_pick_z_lift").value)
        self.sim_drop_xy_tol = float(self.get_parameter("sim_drop_xy_tol").value)
        self.sim_drop_z_tol = float(self.get_parameter("sim_drop_z_tol").value)
        self.place_release_frames = int(self.get_parameter("place_release_frames").value)
        

        # ------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------
        self.current_gesture = "none"
        self.last_logged_gesture = None

        self.robot_program_running = False
        self.cycle_done_seen = False
        self.object_attached = False
        self.gripper_closed = False
        self.state = "COARSE_PICK_GUIDE"
        self.status_text = "Servo guidance active"

        self.set_io_pending = False
        self.hand_back_pending = False
        self.prev_cycle_done_seen = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_tcp_x = None
        self.current_tcp_y = None
        self.current_tcp_z = None

        self.hand_dx = 0.0
        self.hand_dy = 0.0

        self.last_hand_xy_time = self.get_clock().now()

        self.place_release_count = 0

        # Subscribe to hand position updates
        self.sub_hand_xy = self.create_subscription(Vector3, "/hand_xy", self.on_hand_xy, 10)

        # ------------------------------------------------------------
        # ROS I/O
        # ------------------------------------------------------------
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
            f"vx={self.vx:.3f} vy={self.vy:.3f} vz={self.vz:.3f} "
            f"publish_rate={self.publish_rate:.1f} Hz"
        )

    # ------------------------------------------------------------
    # Gesture handling
    # ------------------------------------------------------------
    def on_gesture(self, msg: String) -> None:
        g = msg.data.strip().lower()

        if g != self.last_logged_gesture:
            self.get_logger().info(f"Gesture -> {g}")
            self.last_logged_gesture = g

        # Always track latest gesture in both manual guidance states
        if self.state in ["COARSE_PICK_GUIDE", "COARSE_PLACE_GUIDE"]:
            self.current_gesture = g

        # Empty-hand guidance: fist starts pick
        if self.state == "COARSE_PICK_GUIDE" and g == "fist":
            self.start_cycle_sequence()

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

        # default = stop
        vx = 0.0
        vy = 0.0
        vz = 0.0

        if self.state == "COARSE_PICK_GUIDE":
            if self.current_gesture == "fist":
                vx = 0.0
                vy = 0.0
            else:
                age = (self.get_clock().now() - self.last_hand_xy_time).nanoseconds / 1e9
                if age > self.hand_xy_timeout_s:
                    dx = 0.0
                    dy = 0.0
                    self.status_text = "Hand signal timeout"
                else:
                    dx = self.hand_dx
                    dy = self.hand_dy

                mag = math.hypot(dx, dy)

                if mag < self.xy_deadzone:
                    vx = 0.0
                    vy = 0.0
                else:
                    vx = self.max_vx * (-dy)
                    vy = -self.max_vy * (dx)

        elif self.state == "SIM_DESCEND_PICK":
            vz, reached = self.step_towards_z(self.sim_pick_z_pick, self.sim_pick_down_speed)
            vx = 0.0
            vy = 0.0
            self.status_text = "Sim pick: descending to object"

            if reached:
                self.object_attached = True
                self.gripper_closed = True
                self.state = "SIM_LIFT_AFTER_PICK"
                self.status_text = "Sim pick: lifting object"
                self.get_logger().info("Sim object picked.")

        elif self.state == "SIM_LIFT_AFTER_PICK":
            vz, reached = self.step_towards_z(self.sim_pick_z_lift, self.sim_pick_up_speed)
            vx = 0.0
            vy = 0.0
            self.status_text = "Sim pick: lifting after pick"

            if reached:
                self.state = "COARSE_PLACE_GUIDE"
                self.current_gesture = "fist"
                self.place_release_count = 0
                self.status_text = "Carry mode: hold fist to move, release to place"
                self.get_logger().info("Entered carry mode.")

        elif self.state == "COARSE_PLACE_GUIDE":
            age = (self.get_clock().now() - self.last_hand_xy_time).nanoseconds / 1e9
            if age > self.hand_xy_timeout_s:
                dx = 0.0
                dy = 0.0
                self.status_text = "Carry mode: hand signal timeout"
            else:
                dx = self.hand_dx
                dy = self.hand_dy

            # While fist is held, allow XY motion
            if self.current_gesture == "fist":
                self.place_release_count = 0

                mag = math.hypot(dx, dy)
                if mag < self.xy_deadzone:
                    vx = 0.0
                    vy = 0.0
                else:
                    vx = self.max_vx * (-dy)
                    vy = -self.max_vy * (dx)

                vz = 0.0
                self.status_text = "Carry mode: hold fist to move, release to place"

            else:
                vx = 0.0
                vy = 0.0
                vz = 0.0
                self.place_release_count += 1
                self.status_text = f"Carry mode: release detected ({self.place_release_count}/{self.place_release_frames})"

                if self.place_release_count >= self.place_release_frames:
                    self.state = "SIM_DESCEND_PLACE"
                    self.status_text = "Sim place: descending"
                    self.get_logger().info("Release confirmed. Starting place macro.")

        elif self.state == "SIM_DESCEND_PLACE":
            vz, reached = self.step_towards_z(self.sim_pick_z_pick, self.sim_pick_down_speed)
            vx = 0.0
            vy = 0.0
            self.status_text = "Sim place: lowering to place"

            if reached:
                self.object_attached = False
                self.gripper_closed = False
                self.state = "SIM_ASCEND_AFTER_PLACE"
                self.status_text = "Sim place: lifting after release"
                self.get_logger().info("Sim object placed.")

        elif self.state == "SIM_ASCEND_AFTER_PLACE":
            vz, reached = self.step_towards_z(self.sim_pick_z_lift, self.sim_pick_up_speed)
            vx = 0.0
            vy = 0.0
            self.status_text = "Sim place: ascending"

            if reached:
                self.state = "COARSE_PICK_GUIDE"
                self.current_gesture = "none"
                self.place_release_count = 0
                self.status_text = "Servo guidance active"
                self.get_logger().info("Sim pick/place cycle complete.")
        
        if self.state in ["COARSE_PICK_GUIDE", "COARSE_PLACE_GUIDE"] and (vx != 0.0 or vy != 0.0):
            if self.update_current_tcp_pose():
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
                        vx = 0.0
                        vy = 0.0
                        self.status_text = "Workspace limit reached"
                else:
                    cur_v = self.workspace_violation(cur_x, cur_y)
                    cand_v = self.workspace_violation(cand_x, cand_y)

                    # allow motion only if it improves the situation
                    if cand_v > (cur_v - 1e-4):
                        vx = 0.0
                        vy = 0.0
                        self.status_text = "Move back toward workspace"

        

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub_twist.publish(msg)

    # ------------------------------------------------------------
    # Pick-cycle helpers
    # ------------------------------------------------------------
    def on_robot_program(self, msg: Bool) -> None:
        self.robot_program_running = bool(msg.data)

    def on_io_states(self, msg: IOStates) -> None:
        current = self._read_digital_state(msg.digital_out_states, self.cycle_done_pin)
        rising_edge = current and not self.prev_cycle_done_seen
        self.prev_cycle_done_seen = current
        self.cycle_done_seen = current

        if self.state == "WAITING_FOR_CYCLE" and rising_edge:
            self.complete_cycle_sequence()

    def _read_digital_state(self, states, pin: int) -> bool:
        for s in states:
            if int(s.pin) == pin:
                return bool(s.state)
        return False

    def start_cycle_sequence(self) -> None:
        if self.state != "COARSE_PICK_GUIDE":
            return

        if self.sim_mode or not self.use_ur_io_handshake:
            self.state = "SIM_DESCEND_PICK"
            self.status_text = "Sim pick: descending to object"
            self.current_gesture = "fist"
            self.gripper_closed = False
            self.object_attached = False
            self.place_release_count = 0
            self.get_logger().info("Sim pick cycle started.")
            return

        # real robot path unchanged
        self.state = "WAITING_FOR_CYCLE"
        self.status_text = "Cycle requested"
        self.object_attached = True
        self.gripper_closed = True
        self.current_gesture = "none"
        self.get_logger().info("Cycle sequence started.")

        if self.use_ur_io_handshake:
            self.cycle_done_seen = False
            self.prev_cycle_done_seen = False
            self.set_pick_request(True)

            if not self.robot_program_running:
                self.get_logger().warn("robot_program_running is false. External Control may not be active.")

            self.hand_back_control()

    def step_towards_xy(self, target_x: float, target_y: float, gain: float = 1.5, max_speed: float = 0.10):
        if not self.update_current_tcp_pose():
            return 0.0, 0.0, False

        ex = target_x - self.current_tcp_x
        ey = target_y - self.current_tcp_y

        vx = clamp(gain * ex, -max_speed, max_speed)
        vy = clamp(gain * ey, -max_speed, max_speed)

        reached = math.hypot(ex, ey) < self.sim_drop_xy_tol
        return vx, vy, reached
    
    def step_towards_z(self, target_z: float, speed: float):
        if not self.update_current_tcp_pose():
            return 0.0, False

        ez = target_z - self.current_tcp_z

        if abs(ez) < self.sim_drop_z_tol:
            return 0.0, True

        vz = clamp(ez, -speed, speed)
        return vz, False

        
    def complete_cycle_sequence(self) -> None:
        self.set_pick_request(False)
        self.state = "COARSE_PICK_GUIDE"
        self.status_text = "Cycle complete - ready"
        self.object_attached = False
        self.gripper_closed = False
        self.current_gesture = "none"
        self.get_logger().info("Cycle complete. Returning to guidance mode.")

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

    def destroy_node(self):
        try:
            if self.use_ur_io_handshake:
                self.set_pick_request(False)
        except Exception:
            pass
        super().destroy_node()

    #--------------------------------------------
    # Workspace limit helpers
    #--------------------------------------------
    def update_current_tcp_pose(self) -> bool:
        try:
            import tf2_ros
            if not hasattr(self, "tf_buffer"):
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                "tool0",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            self.current_tcp_x = tf.transform.translation.x
            self.current_tcp_y = tf.transform.translation.y
            self.current_tcp_z = tf.transform.translation.z
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

        # These are still visual-only jaw markers
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

        # workspace boundary marker
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
            f"Cycle active: {self.object_attached}\n"
            f"Program running: {self.robot_program_running}\n"
            f"cycle_done: {self.cycle_done_seen}\n"
            f"frame={self.command_frame}\n"
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