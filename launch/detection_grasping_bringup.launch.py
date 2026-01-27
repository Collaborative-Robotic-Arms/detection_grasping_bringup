#!/usr/bin/python3

import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import yaml


# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None
        
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print("absolute path: ",absolute_file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
def generate_launch_description():
    LD = LaunchDescription()
    
    grasping_node = Node(
        package="brick_grasping_model",
        executable="advanced_grasping_node.py",
        output="screen",
        parameters=[]
    )

    detection_node = Node(
        package="brick_detection",
        executable="advanced_yolo",
        output="screen",
        parameters=[]
    )

    # Add nodes to the launch description:
    LD.add_action(grasping_node)
    LD.add_action(detection_node)
    
    return LD

if __name__ == '__main__':
    ld = generate_launch_description()