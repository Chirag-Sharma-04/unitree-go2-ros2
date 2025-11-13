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
    ekf_config = os.path.join(config_dir, 'config', 'ekf_sensor_fusion.yaml')
    
    # Get RViz config
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
    rviz_config = os.path.join(rtabmap_launch_dir, 'launch', 'config', 'rgbd.rviz')
    
    return LaunchDescription([
        # Use sim time
        SetParameter(name='use_sim_time', value=True),
        
        # 1. RGB-D Visual Odometry (lightweight, frame-to-frame only)
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # EKF will publish the final odom->base_link
                'wait_for_transform': 0.5,
                'use_sim_time': True,
                'approx_sync': True,
                'queue_size': 10,
                
                # Lightweight visual odometry settings
                'Odom/Strategy': '0',  # 0=Frame-to-Map, 1=Frame-to-Frame (use 1 for speed)
                'Odom/ResetCountdown': '0',  # Never reset
                'OdomF2M/MaxSize': '1000',
                'Vis/MinInliers': '10',
                'Vis/MaxFeatures': '300',  # Reduce features for speed
                'Vis/CorGuessWinSize': '10',
                'GFTT/MinDistance': '5',
                'GFTT/QualityLevel': '0.001',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('odom', '/odom/visual'),
            ]
        ),
        
        # 2. ICP LiDAR Odometry (point cloud registration)
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # EKF will publish
                'wait_for_transform': 0.5,
                'use_sim_time': True,
                'queue_size': 10,
                
                # ICP settings optimized for speed
                'Odom/Strategy': '0',  # 0=Frame-to-Map
                'Odom/ResetCountdown': '0',
                'OdomF2M/ScanSubtractRadius': '0.05',
                'OdomF2M/ScanMaxSize': '5000',  # Limit point cloud size
                'Icp/VoxelSize': '0.1',  # Downsample aggressively for speed
                'Icp/MaxCorrespondenceDistance': '0.15',
                'Icp/MaxTranslation': '1.0',
                'Icp/CorrespondenceRatio': '0.3',
                'Icp/Iterations': '20',  # Reduce iterations for speed
                'Icp/Epsilon': '0.001',
            }],
            remappings=[
                ('scan_cloud', '/livox/lidar_PointCloud2'),
                ('odom', '/odom/icp'),
            ]
        ),
        
        # 3. EKF Sensor Fusion (fuse visual + ICP + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom_fusion',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/odom'),  # Output final fused odometry
            ]
        ),
        
        # 4. RTAB-Map SLAM (uses fused odometry from EKF)
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
                'subscribe_odom_info': False,
                'approx_sync': True,
                'queue_size': 10,
                'use_sim_time': True,
                
                # SLAM-only parameters (odometry comes from EKF)
                'Rtabmap/DetectionRate': '1.0',
                'Reg/Strategy': '1',  # ICP for loop closures
                'Reg/Force3DoF': 'true',
                
                # Grid/mapping
                'Grid/3D': 'true',
                'Grid/RayTracing': 'true',
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '20.0',
                
                # Loop closure
                'Vis/MinInliers': '15',
                'Vis/MaxFeatures': '400',
                'Kp/MaxFeatures': '400',
                'Kp/DetectorStrategy': '6',  # GFTT/BRIEF
                
                # Optimization
                'Optimizer/Strategy': '1',  # g2o
                'Optimizer/Iterations': '100',
                'RGBD/OptimizeFromGraphEnd': 'false',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan_cloud', '/livox/lidar_PointCloud2'),
                ('odom', '/odom'),  # Use fused odometry from EKF
            ],
            arguments=['--delete_db_on_start']
        ),
        
        # 5. RTAB-Map Visualization
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
                'subscribe_odom_info': False,
                'approx_sync': True,
                'queue_size': 10,
                'use_sim_time': True,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan_cloud', '/livox/lidar_PointCloud2'),
                ('odom', '/odom'),  # Use fused odometry
            ]
        ),
        
        # 6. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
