#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger


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

        # ------------------------------------------------------------
        # ROS I/O
        # ------------------------------------------------------------
        self.sub_gesture = self.create_subscription(String, "/gesture", self.on_gesture, 10)
        self.pub_twist = self.create_publisher(TwistStamped, self.twist_topic, 10)

        self.pub_markers = self.create_publisher(MarkerArray, "/rg2_markers", 10)
        self.pub_planning_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)

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
        self.current_gesture = g

        if g != self.last_logged_gesture:
            self.get_logger().info(f"Gesture -> {g}")
            self.last_logged_gesture = g

        if g == "fist":
            self.start_cycle_sequence()

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
            if self.current_gesture == "left":
                vy = self.left_sign * self.vy
            elif self.current_gesture == "right":
                vy = -self.left_sign * self.vy
            elif self.current_gesture == "forward":
                vx = self.vx
            elif self.current_gesture == "back":
                vx = -self.vx
            elif self.enable_z and self.current_gesture == "up":
                vz = self.vz
            elif self.enable_z and self.current_gesture == "down":
                vz = -self.vz

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
        self.cycle_done_seen = self._read_digital_state(msg.digital_out_states, self.cycle_done_pin)

        if self.state == "WAITING_FOR_CYCLE" and self.cycle_done_seen:
            self.complete_cycle_sequence()

    def _read_digital_state(self, states, pin: int) -> bool:
        for s in states:
            if int(s.pin) == pin:
                return bool(s.state)
        return False

    def start_cycle_sequence(self) -> None:
        if self.state != "COARSE_PICK_GUIDE":
            return

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

    # ------------------------------------------------------------
    # Table collision object
    # ------------------------------------------------------------
    def republish_table(self) -> None:
        if self.table_enabled:
            self.publish_table_collision()

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

        pose = Pose()
        pose.position.x = self.table_pos_x
        pose.position.y = self.table_pos_y
        pose.position.z = self.table_top_z - (self.table_size_z / 2.0)
        pose.orientation.w = 1.0

        table.primitives.append(primitive)
        table.primitive_poses.append(pose)

        scene.world.collision_objects.append(table)
        self.pub_planning_scene.publish(scene)

    # ------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------
    def publish_markers(self) -> None:
        ma = MarkerArray()

        opening = 0.015 if self.gripper_closed or self.object_attached else 0.08

        jaw_thickness = 0.012
        jaw_depth = 0.02
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

        text = Marker()
        text.header.frame_id = self.base_frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "status"
        text.id = 200
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
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