from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

bringup_pkg = get_package_share_directory("freeflyer_bringup")
desc_pkg = get_package_share_directory("freeflyer_description")

ekf_yaml = os.path.join(bringup_pkg, "config", "ekf.yaml")
urdf_file = os.path.join(desc_pkg, "urdf", "freeflyer.urdf.xacro")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_file]),
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    keyboard_controller = Node(
        package="freeflyer_testing",
        executable="keyboard_teleop",
        name="keyboard_teleop",
        output="screen",
        prefix="xterm -fa 'Monospace' -fs 10 -hold -e",
    )

    radial_controller = Node(
        package="freeflyer_control",
        executable="bang_bang_radial_controller",
        name="bang_bang_radial_controller",
        parameters=[{"thruster_force": 0.5}],
        output="screen",
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace="freeflyer",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml, {"use_sim_time": use_sim_time}],
    )

    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "diagnostics.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        rsp,
        ekf,
        #keyboard_controller,
        radial_controller,
        diagnostics_launch,
    ])
