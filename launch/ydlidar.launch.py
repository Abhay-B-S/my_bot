import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode


def generate_launch_description():
    # YDLIDAR X2 driver with our parameters (config/ydlidar.yaml).
    # Unlike the vendor launch file this does NOT start a static base_link->laser_frame transform:
    # laser_frame already comes from the URDF (laser_joint), and two publishers would conflict.
    params_file = LaunchConfiguration("params_file")

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(get_package_share_directory("my_bot"), "config", "ydlidar.yaml"),
        description="Path to the YDLIDAR driver parameters file",
    )

    driver_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        node_executable="ydlidar_ros2_driver_node",
        node_name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
        node_namespace="/",
    )

    return LaunchDescription([params_declare, driver_node])
