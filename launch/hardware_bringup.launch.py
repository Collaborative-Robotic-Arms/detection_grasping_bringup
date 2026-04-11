#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # ---------------------------------------------------------
    # 1. Read Launch Arguments
    # ---------------------------------------------------------
    mode = LaunchConfiguration('mode').perform(context)
    is_sim = (mode == 'sim')

    nodes_to_start = []

    # ---------------------------------------------------------
    # 2. HARDWARE-ONLY INCLUDES & NODES
    # ---------------------------------------------------------
    if not is_sim:
        # RealSense Camera
        realsense_dir = get_package_share_directory('realsense2_camera')
        nodes_to_start.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_dir, 'launch', 'rs_launch.py')
                ),
                launch_arguments={
                    'align_depth.enable': 'true',
                    'pointcloud.enable': 'false', 
                }.items()
            )
        )

    # ---------------------------------------------------------
    # 3. DYNAMIC PARAMETER SETUP FOR COMMON NODES
    # ---------------------------------------------------------
    
    # Detector Parameters
    detector_params = {
        'image_topic': '/camera/camera/color/image_raw', 
        'camera_info_topic': '/camera/camera/color/camera_info',
        'camera_frame': 'camera_color_optical_frame',
        'use_sim': is_sim
    }
    
    if is_sim:
        detector_params['static_z_height'] = 0.723  # Derived from table height minus brick height
    else:
        detector_params['static_z_height'] = 0.712

    # Grasping Parameters
    grasping_params = {
        'image_topic': '/camera/camera/color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
        'camera_frame': 'camera_color_optical_frame',
        'target_topic': '/grasp/target_index',
        'use_sim': is_sim
    }
    
    if is_sim:
        grasping_params['depth_scale'] = 1.0
    else:
        grasping_params['depth_scale'] = 0.001

    # ---------------------------------------------------------
    # 4. COMMON NODES (Launch in both, behavior changed by params)
    # ---------------------------------------------------------

    # Brick Detector Node
    nodes_to_start.append(
        Node(
            package='brick_detection', 
            executable='final_detector', 
            name='final_detector',
            output='screen',
            parameters=[detector_params]
        )
    )

    # Grasping Node
    nodes_to_start.append(
        Node(
            package='brick_grasping_model',
            executable='advanced_grasping_node.py',
            output='screen',
            parameters=[grasping_params]
        )
    )

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            choices=['sim', 'real'],
            description='System mode: "sim" (default) or "real".'
        ),
        OpaqueFunction(function=launch_setup)
    ])