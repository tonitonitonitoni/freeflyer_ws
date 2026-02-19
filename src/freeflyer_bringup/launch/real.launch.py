from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("freeflyer_bringup")

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "hardware.launch.py")
        )
    )

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "common.launch.py")
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return LaunchDescription([
        hardware_launch,
        common_launch,
    ])
