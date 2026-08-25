#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav_pkg_share = FindPackageShare('int_brain_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    # Remove docking_server and collision_monitor from this list
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    cmd_vel_pre_stamp = remappings + [('cmd_vel', 'cmd_vel_nav_raw')]

    # FIX: Explicitly map 'yaml_filename' to the map_yaml_file argument
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'yaml_filename': map_yaml_file}, 
            convert_types=True,
        ),
        allow_substs=True,
    )

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[
            ('/cmd_vel_in', 'cmd_vel_nav_raw'),
            ('/cmd_vel_out', 'cmd_vel_nav') # Output to the mux
        ]
    )

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([nav_pkg_share, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file'
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', description='Full path to the map yaml file to load'
    )

   # Node Definitions (docking and collision removed)
    map_server_node = Node(
        package='nav2_map_server', 
        executable='map_server', 
        name='map_server', 
        output='screen', 
        parameters=[{'yaml_filename': map_yaml_file}, configured_params], # Direct param injection
        remappings=remappings
    )
    
    amcl_node = Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[configured_params], remappings=remappings)
    controller_node = Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[configured_params], remappings=cmd_vel_pre_stamp)
    behavior_node = Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[configured_params], remappings=cmd_vel_pre_stamp)
    smoother_node = Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen', parameters=[configured_params], remappings=remappings)
    planner_node = Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[configured_params], remappings=remappings)
    route_node = Node(package='nav2_route', executable='route_server', name='route_server', output='screen', parameters=[configured_params], remappings=remappings)
    bt_navigator_node = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[configured_params], remappings=remappings)
    waypoint_node = Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[configured_params], remappings=remappings)

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': lifecycle_nodes}]
    )

    ld = LaunchDescription([
        twist_stamper_node,
        stdout_linebuf_envvar,
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([nav_pkg_share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('map', description='Full path to map yaml'),
        SetParameter('use_sim_time', use_sim_time),
        map_server_node, amcl_node, controller_node, smoother_node, 
        planner_node, route_node, behavior_node, bt_navigator_node, 
        waypoint_node, lifecycle_manager
    ])

    return ld