#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package's share directory path
    int_brain_description_pkg_share = FindPackageShare('int_brain_description')
    int_brain_system_pkg_share = FindPackageShare('int_brain_system')

    rviz_arg = DeclareLaunchArgument("rviz", default_value="true", 
                                     description="Launch RViz2 with the robot model and controllers")

    # Launch Configurations to be used by nodes
    rviz = LaunchConfiguration("rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file", 
                            default=PathJoinSubstitution([
                                int_brain_system_pkg_share, 'config', 'view.rviz'
                            ])
                        )
    
    robot_controllers = PathJoinSubstitution(
        [
            int_brain_system_pkg_share,
            "config",
            "controllers.yaml",
        ]
    )

    # Path to the Xacro file
    xacro_path = PathJoinSubstitution([
        int_brain_description_pkg_share, 'urdf', 'int_brain.xacro'
    ])
    
    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ', xacro_path,
        ' sim:=', 'false',
        ' controllers_yaml:=', robot_controllers,
    ])
    robot_description = {"robot_description": robot_description_content}

    # Load robot controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        # arguments=["--ros-args", "--log-level", "debug"],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[robot_controllers],
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
        parameters=[robot_controllers],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Teleoperation node
    game_controller_node = Node(
        package="joy",
        executable="game_controller_node",
        name="game_controller_node",
        output="screen",
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[robot_controllers],
        remappings=[
            ("/cmd_vel", "/diff_drive_controller/cmd_vel"),
        ],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_broadcaster_spawner,
        # mecanum_drive_controller_spawner,
        diff_drive_controller_spawner,
        teleop_node, game_controller_node,
        rviz_node
    ]

    arguments = [
        rviz_arg
    ]

    return LaunchDescription(arguments+nodes)
