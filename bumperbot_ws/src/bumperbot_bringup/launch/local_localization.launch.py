from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():
    package_name = 'bumperbot_bringup'

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                    description="Use simulated time")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint",
                   "--child-frame-id", "imu_link"],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        #parameters=[os.path.join(get_package_share_directory(package_name), "config", "ekf.yaml")],
        parameters=[os.path.join(get_package_share_directory(package_name), "config", "ekf.yaml"),
                   {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("/odometry/filtered", "odom")],

    )

    return LaunchDescription([
        use_sim_time_arg,
        static_transform_publisher,
        robot_localization,
    ])
