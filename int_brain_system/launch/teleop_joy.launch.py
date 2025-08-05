#!/usr/bin/python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package's share directory path
    int_brain_system_pkg_share = FindPackageShare('int_brain_system')

    teleop_joy_params = PathJoinSubstitution([int_brain_system_pkg_share, 'config', 'teleop_joy_params.yaml'])

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_joy_params],
    )

    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
    )

    nodes = [
        teleop_node, game_controller_node
    ]

    return LaunchDescription(nodes)
