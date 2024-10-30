from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro
import os

def generate_launch_description():
    package_name = 'bumperbot_bringup'

    laser_driver = Node(
            package="ldlidar_stl_ros2",
            executable="ldlidar_stl_ros2_node",
            name="ldlidar_node",
            parameters=[os.path.join(
                get_package_share_directory("bumperbot_bringup"),
                "config",
                "ld06_lidar.yaml"
            )],
            output="screen"
    )     

    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )  

    
    return LaunchDescription([    
    #fake_odom,
    laser_driver,
    ])
