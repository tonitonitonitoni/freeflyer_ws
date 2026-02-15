from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mm_pkg = get_package_share_directory('marvelmind_ros2')

    marvelmind_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mm_pkg, 'launch', 'marvelmind_ros2.launch.py')
        )
    )

    # outputs 'hedgehog_imu_fusion', etc.

    mm_pose_node = Node(
        package='combined_pose_system',
        executable='marvelmind_pose',
        name='marvelmind_pose',
        output='screen'
    )

    # outputs 'marvelmind/pose'

    plotter_node = Node(
        package='combined_pose_system',
        executable='single_trajectory_plotter',
        name='mm_trajectory_plotter',
        arguments=['mm'], 
        output='screen'
    )

    return LaunchDescription([
        marvelmind_launch,
        mm_pose_node,
        #plotter_node, #Use PlotJuggler first
    ])
