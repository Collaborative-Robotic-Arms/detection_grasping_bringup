import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the realsense2_camera package
    realsense_dir = get_package_share_directory('realsense2_camera')
    
    # 2. Define the RealSense Launch Inclusion
    # This is equivalent to: ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false', # Optional: Disable pointcloud if unused to save CPU
            # 'initial_reset': 'true'
        }.items()
    )

    # 3. Define your Brick Detection Node
    # Make sure to update 'your_package_name' to the actual name of your package
    brick_detector_node = Node(
        package='brick_detection', # Replace with your package name
        executable='advanced_yolo', # Replace with your executable name/entry point
        name='advanced_yolo',
        output='screen',
        parameters=[{
            'static_z_height': 0.712, # The calibrated height
            'image_topic': '/camera/camera/color/image_raw', 
            # 'camera_info_topic': '/camera/camera/color/camera_info',
            'camera_frame': 'camera_color_optical_frame',
        }]
    )

    grasping_node = Node(
        package="brick_grasping_model",
        executable="advanced_grasping_node.py",
        output="screen",
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw', 
            'camera_info_topic': '/camera/camera/color/camera_info' ,
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_frame': 'camera_color_optical_frame',
            'target_topic': '/grasp/target_index',
            'depth_scale': 0.001
        }]
    )

    tf_camera_base_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments = ['0.0546', '-0.674', '0.769', '0', '0', '0', 'base_link', 'camera_color_optical_frame']
        )

    # 4. Return the LaunchDescription
    return LaunchDescription([
        realsense_launch,
        brick_detector_node,
        grasping_node,
        tf_camera_base_link
    ])