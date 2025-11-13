#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    
    return LaunchDescription([
        # Use sim time
        SetParameter(name='use_sim_time', value=True),
        
        # Simple relay: remap ground truth from world frame to odom frame
        # This ensures proper TF tree: odom -> base_link
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_relay',
            output='screen',
            arguments=['/odom/ground_truth', '/odom/gt_remapped'],
            parameters=[{'use_sim_time': True}]
        ),
        
        # Transform odometry frame_id from 'world' to 'odom'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_static',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'world'],
            parameters=[{'use_sim_time': True}]
        ),
    ])
