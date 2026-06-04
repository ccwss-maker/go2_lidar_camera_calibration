from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    filter_node = Node(
        package='go2_user_tools',
        executable='pointcloud_filter_forward',
        name='pointcloud_filter_forward',
        parameters=[{
            'camera_accum_frames': 0,
            'camera_voxel_leaf_size': 0.05,
            'camera_x_min': -2.0,
            'camera_x_max': 2.0,
            'camera_y_min': -0.5,
            'camera_y_max': 0.8,
            'camera_z_min': 0.0,
            'camera_z_max': 3.0,
            'lidar_accum_frames': 30,
            'lidar_voxel_leaf_size': 0.0,
            'lidar_x_min': 1.0,
            'lidar_x_max': 4.0,
            'lidar_y_min': -2.0,
            'lidar_y_max': 2.0,
            'lidar_z_min': -0.5,
            'lidar_z_max': 4.0,
        }],
        output='screen',
    )

    icp_node = Node(
        package='go2_user_tools',
        executable='icp_extrinsic_match',
        name='icp_extrinsic_match',
        parameters=[{
            'camera_cloud_topic': '/icp/filtered_camera_points',
            'lidar_cloud_topic': '/icp/filtered_lidar_points',
            'aligned_cloud_topic': '/icp/aligned_camera_points',
            'transform_topic': '/icp/extrinsic_transform',
            'target_frame': 'base_link',
            'source_frame': 'camera_depth_optical_frame',
            'initial_x': 0.182758,
            'initial_y': 0.060812 ,
            'initial_z': 0.202804,
            'initial_roll': -1.548191,
            'initial_pitch': 0.014885,
            'initial_yaw': -1.508523,
            'max_correspondence_distance': 0.5,
            'icp_camera_voxel_leaf_size': 0.08,
            'icp_lidar_voxel_leaf_size': 0.08,
            'max_iterations': 50,
            'transformation_epsilon': 1e-8,
            'euclidean_fitness_epsilon': 1e-5,
            'min_camera_points': 50,
            'min_lidar_points': 50,
            'sync_queue_size': 20,
            'sync_slop_seconds': 1.0,
            'icp_min_interval_seconds': 1.0,
            'fitness_good_threshold': 0.01,
            'fitness_bad_threshold': 0.05,
            'publish_tf': False,
        }],
        output='screen',
    )

    return LaunchDescription([
        filter_node,
        icp_node,
    ])
