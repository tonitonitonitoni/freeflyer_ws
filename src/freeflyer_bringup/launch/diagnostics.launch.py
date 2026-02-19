from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

bringup_pkg = FindPackageShare("freeflyer_bringup").find("freeflyer_bringup")
rviz_config = os.path.join(bringup_pkg, "config", "freeflyer.rviz")

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    force_velocity_plot = Node(
        package="freeflyer_diagnostics",
        executable="force_velocity_live_plot",
        name="force_velocity_live_plot",
        output="screen",
    )

    error_plotter = Node(
            package='freeflyer_diagnostics',
            executable='error_plotter',
            name='error_plotter',
            output='screen'
        )
    
    path_publisher = Node(
        package='freeflyer_control',
        executable='path_publisher',
        name='path_publisher',
        output='screen')
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )
    return LaunchDescription([
        declare_use_sim_time,
        #force_velocity_plot,
        error_plotter,
        path_publisher,
        rviz,
    ])