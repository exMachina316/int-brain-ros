import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the package name
    pkg_name = 'rl_policy_runner'
    
    # Construct the path to the model file
    model_path = os.path.join(
        get_package_share_directory(pkg_name),
        'models',
        'ppo_mecanum_robot.zip' # Make sure your model file is named this
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='policy_node',
            name='policy_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'model_path': model_path},
                {'goal_x': 8.5}, # Set your desired goal x-coordinate
                {'goal_y': 8.5}, # Set your desired goal y-coordinate
                {'lidar_beams': 36} # Must match the training environment
            ]
        )
    ])