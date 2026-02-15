from launch import LaunchDescription
from launch_ros.actions import Node

#This assumes everything from step 1 is working and alignment pipeline is complete.

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

    # outputs 'vision/pose_raw'
    vision_mm_transform_node = Node(
        package='combined_pose_system',
        executable='align_vision_mm',
        name='vision_mm_transform',
        output='screen'
    )

    # outputs 'vision_pose/mm_frame'

    vision_twist_node = Node(
        package='combined_pose_system',
        executable='pose_to_twist',
        name='vision_pose_to_twist',
        arguments=['vision'],
        output='screen'
    )
    
    # outputs 'vision_twist'

    return LaunchDescription([
        pose_cam_node,
        vision_pose_node,
        vision_mm_transform_node,
        vision_twist_node
    ])
