#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get the config file path
    config_dir = os.path.join(get_package_share_directory('go2_config'), 'config')
    ekf_config = os.path.join(config_dir, 'ekf_ground_truth.yaml')
    
    return LaunchDescription([
        # EKF node using ground truth odometry
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_ground_truth',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': True}
            ],
            remappings=[
                ('/odometry/filtered', '/odom/filtered'),
            ]
        ),
    ])
