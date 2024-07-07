from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = [
        PathJoinSubstitution([FindPackageShare("rmcs_map"), "config", "config.yaml"])
    ]

    node = Node(
        package="rmcs_map",
        executable="rmcs_map_exe",
        parameters=params,
    )

    # Assemble the launch description
    ld = LaunchDescription([node])

    return ld
