#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get config directory
    config_dir = get_package_share_directory('go2_config')
    ekf_config = os.path.join(config_dir, 'config', 'ekf_visual_imu.yaml')
    
    # Get RViz config
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
    rviz_config = os.path.join(rtabmap_launch_dir, 'launch', 'config', 'rgbd.rviz')
    
    return LaunchDescription([
        # Use sim time
        SetParameter(name='use_sim_time', value=True),
        
        # SIMPLE APPROACH: Visual odometry only (lightest option)
        # This publishes odom->base_link directly
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='visual_odom',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,  # Publish odom->base_link
                'wait_for_transform': 0.5,
                'use_sim_time': True,
                'approx_sync': True,
                'queue_size': 10,
                
                # Optimized for accuracy AND speed
                'Odom/Strategy': '0',  # Frame-to-Map (more accurate)
                'Odom/ResetCountdown': '0',
                'OdomF2M/MaxSize': '2000',  # Larger map for stability
                'Vis/MinInliers': '12',  # Reasonable threshold
                'Vis/MaxFeatures': '400',  # More features = better accuracy
                'Vis/CorGuessWinSize': '20',
                'GFTT/MinDistance': '7',
                'GFTT/QualityLevel': '0.0001',
                'Odom/FillInfoData': 'true',
                'Odom/FilteringStrategy': '1',  # Kalman filtering
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('odom', '/odom'),
            ]
        ),
        
        # RTAB-Map SLAM (uses visual odometry)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan_cloud': True,
                'subscribe_odom_info': True,  # Use odometry quality info
                'approx_sync': True,
                'queue_size': 10,
                'use_sim_time': True,
                
                # SLAM parameters
                'Rtabmap/DetectionRate': '1.0',
                'Reg/Strategy': '1',  # ICP for loop closures
                'Reg/Force3DoF': 'true',
                
                # Grid/mapping with lidar
                'Grid/3D': 'true',
                'Grid/RayTracing': 'true',
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '20.0',
                'Grid/Sensor': '0',  # Use camera for grid
                
                # Loop closure
                'Vis/MinInliers': '15',
                'Vis/MaxFeatures': '400',
                'Kp/MaxFeatures': '400',
                'Kp/DetectorStrategy': '6',  # GFTT/BRIEF
                
                # Optimization
                'Optimizer/Strategy': '1',  # g2o
                'Optimizer/Iterations': '100',
                'RGBD/OptimizeFromGraphEnd': 'false',
                
                # Use lidar for mapping
                'Icp/VoxelSize': '0.05',
                'Icp/MaxCorrespondenceDistance': '0.1',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan_cloud', '/livox/lidar_PointCloud2'),
                ('odom', '/odom'),
            ],
            arguments=['--delete_db_on_start']
        ),
        
        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan_cloud': True,
                'subscribe_odom_info': True,
                'approx_sync': True,
                'queue_size': 10,
                'use_sim_time': True,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan_cloud', '/livox/lidar_PointCloud2'),
                ('odom', '/odom'),
            ]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
