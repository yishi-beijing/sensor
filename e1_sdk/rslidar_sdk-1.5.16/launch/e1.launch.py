#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
import os

def generate_launch_description():

    # Define composable nodes
    driver_node = ComposableNode(
        package='rslidar_sdk',
        plugin='robosense::lidar::NodeManager',  # Ensure this is correct
        name='rslidar_sdk_components',
        # namespace='cx'
    )


    # Define the container for composable nodes
    container = ComposableNodeContainer(
        name='component_container',
        namespace='container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[driver_node],
        output='screen'
    )

    return LaunchDescription([
        container
    ])