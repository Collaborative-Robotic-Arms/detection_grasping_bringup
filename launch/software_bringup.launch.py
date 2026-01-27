import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 3. Define your Brick Detection Node
    # Make sure to update 'your_package_name' to the actual name of your package
    brick_detector_node = Node(
        package='brick_detection', # Replace with your package name
        executable='advanced_yolo', # Replace with your executable name/entry point
        name='advanced_yolo',
        output='screen',
        parameters=[{
            'static_z_height': 0.80, # The calibrated height
            'image_topic': '/environment_camera/image_raw', 
            'camera_info_topic': '/environment_camera/camera_info' ,
            'camera_frame': 'camera',
        }]
    )


    grasping_node = Node(
        package="brick_grasping_model",
        executable="advanced_grasping_node.py",
        output="screen",
        parameters=[{
            'image_topic': '/environment_camera/image_raw', 
            'camera_info_topic': '/environment_camera/camera_info' ,
            'depth_topic': '/environment_camera/depth_image',
            'camera_frame': 'camera',
            'target_topic': '/grasp/target_index',
            'depth_scale': 1.0
        }]
    )

    # 4. Return the LaunchDescription
    return LaunchDescription([
        brick_detector_node,
        grasping_node
    ])