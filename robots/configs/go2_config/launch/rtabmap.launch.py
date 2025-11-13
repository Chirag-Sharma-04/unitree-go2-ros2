#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Declare launch arguments
    localization_arg = DeclareLaunchArgument(
        'localization', 
        default_value='false',
        description='Set to true for localization mode, false for mapping mode'
    )
    
    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='true',
        description='Delete database on start (for fresh mapping)'
    )
    
    # Get launch configurations
    localization = LaunchConfiguration('localization')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    
    # Get RViz config path
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
    rviz_config = os.path.join(rtabmap_launch_dir, 'launch', 'config', 'rgbd.rviz')
    
    return LaunchDescription([
        localization_arg,
        delete_db_arg,
        
        # Use sim time
        SetParameter(name='use_sim_time', value=True),
        
        # SIMPLE SOLUTION: Static TF to connect world->odom
        # Ground truth publishes: world -> base_link
        # RTAB-Map expects: odom -> base_link
        # This bridges them: odom -> world -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_world_bridge',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'world'],
            parameters=[{'use_sim_time': True}]
        ),
        
        # RTAB-Map node with RGB-D camera + 3D LiDAR fusion
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                # Core Settings
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'ground_truth_frame_id': '',  # Don't use ground truth frame
                'ground_truth_base_frame_id': '',
                
                # Odometry - let RTAB-Map compute it from visual+lidar
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan_cloud': True,
                'use_sim_time': True,
                'wait_for_transform': 1.0,
                'tf_tolerance': 0.1,
                
                # Use RTAB-Map's own odometry (visual+ICP fusion internally)
                'Reg/Strategy': '1',  # 1=ICP, but also uses visual if available
                'Reg/Force3DoF': 'true',  # 2D mode (ignore z, roll, pitch)
                
                # Topic remappings
                'rgb/image': '/camera/color/image_raw',
                'rgb/camera_info': '/camera/color/camera_info',
                'depth/image': '/camera/depth/image_raw',
                'scan_cloud': '/livox/lidar_PointCloud2',
                
                # Synchronization (critical for slow simulation)
                'approx_sync': True,
                'approx_sync_max_interval': 0.5,  # 500ms tolerance for 0.55x realtime
                'queue_size': 50,  # Large buffer for timing variations
                
                # Database management
                'database_path': '~/rtabmap.db',
                'Mem/IncrementalMemory': 'True',  # Always true for mapping mode
                
                # RTAB-Map Parameters - Optimized for slow simulation + sensor fusion
                'Rtabmap/DetectionRate': '1.0',        # Process at 1 Hz (stable for slow sim)
                'Rtabmap/TimeThr': '0',                 # Disable time threshold
                'Mem/InitWMWithAllNodes': 'false',
                
                # Odometry constraints
                'RGBD/LinearUpdate': '0.05',            # Update every 5cm movement
                'RGBD/AngularUpdate': '0.05',           # Update every ~3 degree rotation
                'RGBD/OptimizeFromGraphEnd': 'false',
                
                # Visual odometry (RGB-D)
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/LocalRadius': '5',
                
                # 3D LiDAR processing
                'Grid/Sensor': '0',                     # 0=RGB-D, 1=Stereo, 2=Laser scan
                'Grid/3D': 'true',                      # Enable 3D occupancy grid
                'Grid/RayTracing': 'true',              # Ray tracing for obstacles
                'Grid/CellSize': '0.05',                # 5cm resolution
                'Grid/RangeMax': '20.0',                # Max range 20m (adjust based on needs)
                'Grid/RangeMin': '0.2',                 # Min range 20cm
                'Grid/MaxObstacleHeight': '2.0',        # 2m max obstacle height
                'Grid/MaxGroundHeight': '0.05',         # Ground classification
                'Grid/NormalsSegmentation': 'false',
                'Grid/ClusterRadius': '0.1',
                'Grid/FlatObstacleDetected': 'true',
                
                # ICP (Iterative Closest Point) for LiDAR odometry
                'Icp/VoxelSize': '0.05',                # Downsample to 5cm voxels
                'Icp/MaxCorrespondenceDistance': '0.1', # 10cm max correspondence
                'Icp/MaxTranslation': '0.5',            # Max 50cm per frame
                'Icp/CorrespondenceRatio': '0.2',       # 20% inlier requirement
                'Icp/Iterations': '30',                 # ICP iterations
                'Icp/Epsilon': '0.001',
                
                # Loop closure detection (visual + geometric)
                'Kp/MaxFeatures': '400',                # Features per image
                'Kp/DetectorStrategy': '6',             # 6=GFTT/BRIEF (fast & robust)
                'Vis/MinInliers': '15',                 # Min inliers for loop closure
                'Vis/MaxFeatures': '400',
                'Vis/CorGuessWinSize': '20',
                'RGBD/OptimizeMaxError': '3.0',
                
                # Graph optimization
                'Optimizer/Strategy': '1',              # 1=g2o (most accurate)
                'Optimizer/Iterations': '100',
                'Optimizer/Robust': 'true',
                'Optimizer/VarianceIgnored': 'false',
                'RGBD/OptimizeMaxError': '1.0',
                
                # Memory management
                'Mem/RehearsalSimilarity': '0.30',
                'Mem/BadSignaturesIgnored': 'true',
                'Mem/MapLabelsAdded': 'true',
                'Mem/STMSize': '30',                    # Short-term memory
                'Mem/LocalizationDataSaved': 'true',
            }],
            arguments=[
                '--delete_db_on_start',
                '--Mem/IncrementalMemory', 'true',
                '--udebug'
            ],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan_cloud', '/livox/lidar_PointCloud2'),
            ]
        ),
        
        # RTAB-Map visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',  # Match the SLAM node
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan_cloud': True,
                'use_sim_time': True,
                'approx_sync': True,
                'queue_size': 50,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan_cloud', '/livox/lidar_PointCloud2'),
            ]
        ),
        
        # RViz2 with RTAB-Map configuration
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
