from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pose_cam_node = Node(
        package='combined_pose_system',
        executable='gst_pose_cam',
        name='gst_pose_cam',
        output='screen'
    )

    # outputs 'pose_cam/image/compressed'

    vision_pose_node = Node(
        package='combined_pose_system',
        executable='vision_pose',
        name='vision_pose',
        output='screen'
    )

    # outputs 'vision_pose/raw'

    plotter_node = Node(
        package='combined_pose_system',
        executable='single_trajectory_plotter',
        name='vision_trajectory_plotter',
        arguments=['vision'],
        output='screen'
    )

    return LaunchDescription([
        pose_cam_node,
        vision_pose_node,
        #plotter_node, #Use PlotJuggler first
    ])
