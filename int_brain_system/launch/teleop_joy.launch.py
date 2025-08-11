#!/usr/bin/python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package's share directory path
    int_brain_system_pkg_share = FindPackageShare('int_brain_system')

    teleop_joy_params = PathJoinSubstitution([int_brain_system_pkg_share, 'config', 'teleop_joy_params.yaml'])
    rviz_config_file = LaunchConfiguration("rviz_config_file", 
                            default=PathJoinSubstitution([
                                int_brain_system_pkg_share, 'config', 'view.rviz'
                            ])
                        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

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

    imu_feedback_node = Node(
        package='int_brain_system',
        executable='imu_feedback_node',
        name='imu_feedback_node',
        parameters=[teleop_joy_params],
    )

    nodes = [
        teleop_node, game_controller_node, imu_feedback_node, rviz_node
    ]

    return LaunchDescription(nodes)
