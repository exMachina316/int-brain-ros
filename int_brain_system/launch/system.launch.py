#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package's share directory path
    int_brain_description_pkg_share = FindPackageShare('int_brain_description')
    int_brain_system_pkg_share = FindPackageShare('int_brain_system')
    rplidar_ros_pkg_share = FindPackageShare('rplidar_ros')

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
    lidar_arg = DeclareLaunchArgument(
        "lidar", default_value="true",
        description="Launch RPLidar A1"
    )
    channel_type_arg = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Specifying channel type of lidar'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Specifying usb port to connected lidar'
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Specifying usb port baudrate to connected lidar'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='lidar_frame',
        description='Specifying frame_id of lidar'
    )
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Specifying whether or not to invert scan data'
    )
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data'
    )
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Boost',
        description='Specifying scan mode of lidar'
    )
    stamp_twist_arg = DeclareLaunchArgument(
        'stamp_twist',
        default_value='false',
        description='Enable timestamping of incoming twist messages'
    )

    # Launch Configurations to be used by nodes
    rviz = LaunchConfiguration("rviz")
    drive_type = LaunchConfiguration("drive_type")
    is_feedforward = LaunchConfiguration("is_feedforward")
    debug = LaunchConfiguration("debug")
    lidar = LaunchConfiguration("lidar")
    stamp_twist = LaunchConfiguration("stamp_twist")
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

    # Incoming velocity topic based on stamping requirement
    cmd_vel_topic = PythonExpression(
        ["'/cmd_vel_stamped' if '", stamp_twist, "' == 'true' else '/cmd_vel'"]
    )

    # Load robot controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        remappings=[
            ("/mecanum_drive_controller/reference", cmd_vel_topic),
            ("/diff_drive_controller/cmd_vel", cmd_vel_topic),
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

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        remappings=[('/cmd_vel_in', '/cmd_vel'),
                    ('/cmd_vel_out', '/cmd_vel_stamped')],
        condition=IfCondition(stamp_twist)
    )

    rplidar_a1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                rplidar_ros_pkg_share,
                'launch',
                'rplidar_a1_launch.py'
            ])
        ),
        condition=IfCondition(lidar),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_broadcaster_spawner,
        mecanum_drive_controller_spawner,
        diff_drive_controller_spawner,
        twist_stamper,
        # robot_localization,
        rplidar_a1_launch,
        rviz_node
    ]

    arguments = [
        rviz_arg,
        drive_type_arg, is_feedforward_arg,
        debug_arg, lidar_arg,
        channel_type_arg,
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        inverted_arg,
        angle_compensate_arg,
        scan_mode_arg,
        stamp_twist_arg
    ]

    return LaunchDescription(arguments+nodes)
