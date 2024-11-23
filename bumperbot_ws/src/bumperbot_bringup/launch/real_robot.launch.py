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

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )   
    
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

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.control.launch.py'
            )])
    )

    imu_driver_node = Node(
        package=package_name,
        executable="mpu6050_driver"
    )

    local_imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','local_localization.launch.py'
            )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([    
    hardware_interface,
    #fake_odom, 
    laser_driver,
    twist_mux,
    diff_drive_spawner,
    joint_broad_spawner,
    joystick,
    imu_driver_node,
    local_imu,
    ])
