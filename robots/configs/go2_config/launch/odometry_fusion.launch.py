#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get the config file path
    config_dir = os.path.join(get_package_share_directory('go2_config'), 'config')
    ekf_config = os.path.join(config_dir, 'ekf_sensor_fusion.yaml')
    
    return LaunchDescription([
        # Use sim time
        SetParameter(name='use_sim_time', value=True),
        
        # RGB-D Visual Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='visual_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # Don't publish TF, let EKF handle it
                'wait_for_transform': 0.5,
                'approx_sync': True,
                'queue_size': 30,
                'use_sim_time': True,
                
                # Visual odometry parameters - more conservative
                'Odom/Strategy': '1',  # 1=Frame-to-Frame (more stable than Frame-to-Map)
                'Odom/ResetCountdown': '10',  # Higher countdown before reset
                'Odom/Holonomic': 'false',  # Robot is not holonomic
                'Vis/MaxFeatures': '300',  # Reduce features for stability
                'Vis/MinInliers': '20',  # Increase min inliers
                'Vis/InlierDistance': '0.05',  # Stricter inlier distance
                'Vis/CorGuessWinSize': '20',
                'Vis/CorFlowWinSize': '16',
                'OdomF2M/MaxSize': '1000',
                'Odom/FillInfoData': 'true',
                'Odom/GuessMotion': 'true',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('odom', '/odom/visual'),
            ]
        ),
        
        # ICP Lidar Odometry
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # Don't publish TF, let EKF handle it
                'wait_for_transform': 0.5,
                'use_sim_time': True,
                'guess_frame_id': '',
                
                # ICP parameters - more conservative
                'Icp/VoxelSize': '0.1',  # Larger voxel size for stability (10cm)
                'Icp/MaxCorrespondenceDistance': '0.2',  # 20cm max correspondence
                'Icp/MaxTranslation': '0.3',  # 30cm max translation per frame
                'Icp/CorrespondenceRatio': '0.4',  # 40% inlier requirement
                'Icp/Iterations': '30',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/PointToPlaneK': '5',
                'Icp/PointToPlaneRadius': '0.3',
                'Odom/Strategy': '1',  # 1=Frame-to-Frame (more stable)
                'Odom/ScanKeyFrameThr': '0.7',  # 70% overlap required
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '10000',  # Limit point cloud size
                'Odom/GuessMotion': 'true',
                'Odom/Holonomic': 'false',
            }],
            remappings=[
                ('scan_cloud', '/livox/lidar_PointCloud2'),
                ('odom', '/odom/icp'),
            ]
        ),
        
        # EKF Sensor Fusion: Visual + ICP + IMU
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_sensor_fusion',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': True}
            ],
            remappings=[
                ('/odometry/filtered', '/odom'),
            ]
        ),
    ])
