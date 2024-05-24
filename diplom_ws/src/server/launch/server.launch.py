#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    launch_list = [
        Node(
            package="server",
            executable="server_start",
            output="screen"
        ),
        Node(
            package="sit_awar",
            executable="sit_awar",
            output="screen"
        )
    ]

    return LaunchDescription(launch_list)