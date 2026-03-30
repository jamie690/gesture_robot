from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur10e", package_name="ur_moveit_config")
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            "/home/jamie/gesture_ws/install/gesture_control/share/gesture_control/config/servo.yaml",
        ],
    )

    return LaunchDescription([servo_node])