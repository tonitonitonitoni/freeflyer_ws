from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os

sim_pkg = get_package_share_directory("freeflyer_gazebo")
prefix = get_package_prefix("freeflyer_gazebo")

world_file = os.path.join(sim_pkg, "worlds", "air_table.world")
plugin_path = os.path.join(prefix, "lib")


def generate_launch_description():
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.pathsep.join([
            plugin_path,
            os.environ.get("GAZEBO_PLUGIN_PATH", "")
        ])
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world_file,
        ],
        output="screen",
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", "freeflyer",
            "-topic", "robot_description",
            "-x", "1.0",
            "-y", "0.0",
            "-z", "0.55",
            "-Y", "3.141592653589793",
        ],
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    return LaunchDescription([
        set_gazebo_plugin_path,
        gazebo,
        delayed_spawn,
    ])
