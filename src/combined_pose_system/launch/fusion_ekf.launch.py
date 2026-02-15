from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------
    # Marvelmind bringup
    # -----------------------
    mm_pkg = get_package_share_directory('marvelmind_ros2')

    marvelmind_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mm_pkg, 'launch', 'marvelmind_ros2.launch.py')
        )
    )

    marvelmind_pose_node = Node(
        package='combined_pose_system',
        executable='marvelmind_pose',
        name='marvelmind_pose',
        output='screen'
    )

    mm_twist_node = Node(
        package='combined_pose_system',
        executable='pose_to_twist',
        name='mm_pose_to_twist',
        arguments=['mm'],
        output='screen'
    )

    # -----------------------
    # Vision bringup
    # -----------------------
    pose_cam_node = Node(
        package='combined_pose_system',
        executable='pose_cam',
        name='pose_cam',
        output='screen'
    )

    vision_pose_node = Node(
        package='combined_pose_system',
        executable='vision_pose',
        name='vision_pose',
        output='screen'
    )

    vision_mm_transform_node = Node(
        package='combined_pose_system',
        executable='align_vision_mm',
        name='vision_mm_transform',
        output='screen'
    )

    vision_twist_node = Node(
        package='combined_pose_system',
        executable='pose_to_twist',
        name='vision_pose_to_twist',
        arguments=['vision'],
        output='screen'
    )

    # -----------------------
    # EKF (robot_localization)
    # -----------------------
    ekf_config = os.path.join(
        get_package_share_directory('combined_pose_system'),
        'config',
        'ekf_mm_map.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    return LaunchDescription([
        
        # Sensors
        marvelmind_launch,
        pose_cam_node,

        # Pose layers
        marvelmind_pose_node,
        vision_pose_node,

        # Frame alignment
        vision_mm_transform_node,

        # Velocities
        mm_twist_node,
        vision_twist_node,

        # Fusion
        ekf_node,
    ])
