#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Declare Launch Arguments ---
    # Default is "sim" mode. Can be set to "real".
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='System mode: "sim" (default) or "real".'
    )
    
    mode = LaunchConfiguration('mode')

    # Boolean expressions to evaluate the current mode
    is_real = PythonExpression(["'", mode, "' == 'real'"])
    is_sim = PythonExpression(["'", mode, "' == 'sim'"])

    # --- 2. Locate the realsense2_camera package ---
    realsense_dir = get_package_share_directory('realsense2_camera')
    
    # --- 3. Define the RealSense Launch Inclusion ---
    # Condition: Only launches in 'real' mode
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false', 
        }.items(),
        condition=IfCondition(is_real)
    )

    # --- 4. Define your Brick Detection Nodes ---
    
    # Version A: SIM MODE (camera_info_topic is omitted, use_sim is True)
    brick_detector_node_sim = Node(
        package='brick_detection', 
        executable='final_detector', 
        name='final_detector',
        output='screen',
        parameters=[{
            'static_z_height': 0.712, 
            'image_topic': '/camera/camera/color/image_raw', 
            # 'camera_info_topic': '/camera/camera/color/camera_info',
            'camera_frame': 'camera_color_optical_frame',
            'use_sim': True
        }],
        condition=IfCondition(is_sim)
    )

    # Version B: REAL MODE (camera_info_topic is active, use_sim is False)
    brick_detector_node_real = Node(
        package='brick_detection', 
        executable='final_detector', 
        name='final_detector',
        output='screen',
        parameters=[{
            'static_z_height': 0.712, 
            'image_topic': '/camera/camera/color/image_raw', 
            'camera_info_topic': '/camera/camera/color/camera_info', 
            'camera_frame': 'camera_color_optical_frame',
            'use_sim': False
        }],
        condition=IfCondition(is_real)
    )

    # --- 5. Define Grasping Nodes ---

    # Version A: SIM MODE (camera_info_topic omitted, use_sim is True)
    grasping_node_sim = Node(
        package="brick_grasping_model",
        executable="advanced_grasping_node.py",
        output="screen",
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw', 
            # 'camera_info_topic': '/camera/camera/color/camera_info',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_frame': 'camera_color_optical_frame',
            'target_topic': '/grasp/target_index',
            'depth_scale': 1.0,
            'use_sim': True
        }],
        condition=IfCondition(is_sim)
    )

    # Version B: REAL MODE (camera_info_topic is active, use_sim is False)
    grasping_node_real = Node(
        package="brick_grasping_model",
        executable="advanced_grasping_node.py",
        output="screen",
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw', 
            'camera_info_topic': '/camera/camera/color/camera_info',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_frame': 'camera_color_optical_frame',
            'target_topic': '/grasp/target_index',
            'depth_scale': 0.001,
            'use_sim': False
        }],
        condition=IfCondition(is_real)
    )

    # --- 6. Define Static TF Publisher ---
    # Condition: Only launches in 'real' mode
    tf_camera_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.0546', '-0.674', '0.769', '0', '0', '0', 'base_link', 'camera_color_optical_frame'],
        condition=IfCondition(is_real)
    )

    # --- 7. Return the LaunchDescription ---
    return LaunchDescription([
        mode_arg,
        realsense_launch,
        brick_detector_node_sim,
        brick_detector_node_real,
        grasping_node_sim,
        grasping_node_real,
        tf_camera_base_link
    ])