from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_prefix

bringup_pkg = FindPackageShare("freeflyer_bringup").find("freeflyer_bringup")
ekf_yaml = os.path.join(bringup_pkg, "config", "ekf_both.yaml")

sim_pkg = FindPackageShare("freeflyer_gazebo").find("freeflyer_gazebo")
desc_pkg = FindPackageShare("freeflyer_description").find("freeflyer_description")
prefix = get_package_prefix("freeflyer_gazebo")
plugin_path = os.path.join(prefix, "lib")

world_file = os.path.join(sim_pkg, "worlds", "air_table.world")
urdf_file  = os.path.join(desc_pkg, "urdf", "bang_bang.urdf.xacro")
rviz_config = os.path.join(bringup_pkg, "config", "freeflyer.rviz")

def generate_launch_description():
    # ---------------------------
    # Gazebo Classic
    # ---------------------------
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.pathsep.join([
            plugin_path,
            os.environ.get("GAZEBO_PLUGIN_PATH", "")
        ])
    )


    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', "freeflyer",
            '-topic', 'robot_description',
            "-x", "1.0",
            "-y", "0.0",
            "-z", "0.55",
            "-Y", "3.141592653589793",
        ],
        output='screen'
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    # ---------------------------
    # Robot State Publisher
    # ---------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", urdf_file])}],
        output="screen"
    )

    # ---------------------------
    # Autonomous circle controller
    # ---------------------------
    circle_controller = Node(
        package='freeflyer_control',
        executable='bang_bang_rw_controller',
        name='bang_bang_rw_controller',
        output='screen',
    )

    keyboard_thruster_controller = Node(
        package='freeflyer_control',
        executable='bang_bang_keyboard',
        name='bang_bang_keyboard',
        output='screen',
        prefix="xterm -fa 'Monospace' -fs 10 -hold -e",
        parameters=[{
            "thrust_level": 1.0,
        }]
    )

    force_velocity_plot = Node(
        package='freeflyer_diagnostics',
        executable='force_velocity_live_plot',
        name='force_velocity_live_plot',
        output='screen'
    )

    thruster_odom_predictor = Node(
        package='freeflyer_control',
        executable='thruster_rw_odom_predictor',
        name='thruster_rw_odom_predictor',
        output='screen'
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace="freeflyer",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml, {"use_sim_time": True}]
    )

    gazebo_to_pose = Node(
        package='freeflyer_control',
        executable='gazebo_to_pose',
        name='gazebo_to_pose',
        output='screen',
        parameters=[{
            "use_sim_time": True,
            "frame_id": "odom",
        }]
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
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    error_plotter = Node(
        package='freeflyer_diagnostics',
        executable='dual_error_plotter',
        name='dual_error_plotter',
        output='screen')
    

    return LaunchDescription([
        set_gazebo_plugin_path,
        gazebo,
        rsp,
        gazebo_to_pose,
        delayed_spawn,
        thruster_odom_predictor,
        ekf,
        circle_controller,
        #keyboard_thruster_controller,
        force_velocity_plot,
        path_publisher,
        rviz,
        error_plotter,
    ])
