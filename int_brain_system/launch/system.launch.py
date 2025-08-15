#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package's share directory path
    int_brain_description_pkg_share = FindPackageShare('int_brain_description')
    int_brain_system_pkg_share = FindPackageShare('int_brain_system')

    rviz_arg = DeclareLaunchArgument("rviz", default_value="true", 
                                     description="Launch RViz2 with the robot model and controllers")
    drive_type_arg = DeclareLaunchArgument(
        "drive_type",
        default_value="mecanum",
        description="Select drive type: 'mecanum' or 'diff'"
    )
    debug_arg = DeclareLaunchArgument(
        "debug", default_value="false",
        description="Enable debug logging for nodes"
    )
    is_feedforward_arg = DeclareLaunchArgument(
        "is_feedforward", default_value="false",
        description="Enable feedforward control for the robot"
    )

    # Launch Configurations to be used by nodes
    rviz = LaunchConfiguration("rviz")
    drive_type = LaunchConfiguration("drive_type")
    is_feedforward = LaunchConfiguration("is_feedforward")
    debug = LaunchConfiguration("debug")
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

    ekf_params = PathJoinSubstitution(
        [
            int_brain_system_pkg_share, 
            "config",
            "ekf.yaml"
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
        ' is_feedforward:=', PythonExpression(["'", is_feedforward, "'"])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Load robot controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        remappings=[
            ("/mecanum_drive_controller/reference", "/cmd_vel"),
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
            ("/mecanum_drive_controller/tf_odometry", "/tf"),
            ("/mecanum_drive_controller/odometry", "/int_brain/odom"),
            ("/diff_drive_controller/odom", "/int_brain/odom"),
        ],
        arguments=[
            "--ros-args", "--log-level", 
            PythonExpression(["'debug' if '", debug, "' == 'true' else 'info'"])
        ],
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
        condition=IfCondition(PythonExpression(["'", drive_type, "' == 'mecanum'"])),
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", drive_type, "' == 'diff'"])),
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, robot_description],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_broadcaster_spawner,
        mecanum_drive_controller_spawner,
        diff_drive_controller_spawner,
        robot_localization,
        rviz_node
    ]

    arguments = [
        rviz_arg,
        drive_type_arg, is_feedforward_arg,
        debug_arg
    ]

    return LaunchDescription(arguments+nodes)
