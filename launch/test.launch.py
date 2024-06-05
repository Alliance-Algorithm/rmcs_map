from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = [
        PathJoinSubstitution([FindPackageShare("decision_interface"), "config", "config.yaml"])
    ]

    node = Node(
        package="decision_interface",
        executable="decision_interface_test",
        parameters=params,
    )

    # Assemble the launch description
    ld = LaunchDescription([node])

    return ld
