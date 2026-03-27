import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class GestureToJointStates(Node):
    def __init__(self):
        super().__init__("gesture_to_jointstates")

        self.sub = self.create_subscription(
            String, "/gesture", self.cb, 10
        )
        self.pub = self.create_publisher(
            JointState, "/joint_states", 10
        )

        # UR5 joint names
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.q = [0.0] * 6
        self.limits = [(-3.14, 3.14)] * 6
        self.step = 0.05

        self.timer = self.create_timer(
            0.05, self.publish_joint_state
        )

        self.get_logger().info("gesture_to_jointstates started.")

    def cb(self, msg: String):
        g = msg.data.lower().strip()

        # Apply gesture → joint change
        if g == "left":
            self.q[0] += self.step
        elif g == "right":
            self.q[0] -= self.step
        elif g == "open":
            self.q[1] += self.step
        elif g == "fist":
            self.q[1] -= self.step
        elif g == "pinch":
            self.q = [0.0] * 6

        # Clamp all joints
        for i in range(6):
            lo, hi = self.limits[i]
            self.q[i] = max(lo, min(hi, self.q[i]))

        self.get_logger().info(
            f"Gesture {g} -> q0={self.q[0]:.2f}, q1={self.q[1]:.2f}"
        )

    def publish_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.q
        self.pub.publish(js)


def main():
    rclpy.init()
    node = GestureToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
