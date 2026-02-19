from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
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

    marvelmind_pose = Node(
        package='combined_pose_system',
        executable='marvelmind_pose',
        name='marvelmind_pose_node',
        output='screen',
    )

    pose_cam = Node(
        package='combined_pose_system',
        executable='gst_pose_cam',
        name='gst_pose_cam',
        output='screen'
    )

    vision_yaw = Node(
        package='combined_pose_system',
        executable='vision_yaw',
        name='vision_yaw_node',
        output='screen',
    )

    micro_ros_agent = ExecuteProcess(
        cmd=['micro_ros_agent', 'udp4', '--port', '8888'],
        output='screen',
    )

    serial_bridge = Node(
        package='freeflyer_control',
        executable='serial_bridge',
        name='ff_serial_bridge',
        output='screen',
    )

    return LaunchDescription([
        marvelmind_launch,
        marvelmind_pose,
        pose_cam,
        vision_yaw,
        micro_ros_agent,
        serial_bridge,
    ])
