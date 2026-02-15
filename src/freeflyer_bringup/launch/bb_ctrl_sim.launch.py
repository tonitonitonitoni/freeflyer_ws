from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    bringup_pkg = FindPackageShare("freeflyer_bringup")
    desc_pkg = FindPackageShare("freeflyer_description")
    
    sim_pkg = FindPackageShare("freeflyer_gazebo").find("freeflyer_gazebo")
    world_file = os.path.join(sim_pkg, "worlds", "air_table.world")

    # -------- Paths --------
    urdf_file = PathJoinSubstitution([
        desc_pkg,
        "urdf",
        "ctrl_bb.urdf.xacro"
    ])

    controllers_file = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "controllers.yaml"
    ])
    ekf_yaml = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "ekf_both.yaml"
    ])

    # -------- Robot Description --------
    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf_file,
            " F_max:=", "4.0",
            " rw_tau_max:=", "0.05",
            " controllers_file:=",
            controllers_file,
        ])
    }

    # -------- Robot State Publisher --------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    # -------- Gazebo --------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ),
        launch_arguments={"world": world_file}.items(),
    )

    # -------- Spawn Robot into Gazebo --------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "freeflyer",
            "-x", "1.0",
            "-y", "0.0",
            "-z", "0.55",
            "-Y", "3.141592653589793",
        ],
        output="screen"
    )

    # -------- Controller Spawner --------
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bang_bang_circle_controller",
            "--param-file",
            controllers_file
        ],
        output="screen"
    )

    gazebo_to_pose = Node(
        package="freeflyer_control",
        executable="gazebo_to_pose",
        name="gazebo_to_pose",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "odom",
        }]
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace="freeflyer",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml, {"use_sim_time": True}]
    )

    dual_error_plotter = Node(
        package='freeflyer_diagnostics',
        executable='dual_error_plotter',
        name='dual_error_plotter',
        output='screen'
    )

    # Delay controller spawn until robot is spawned
    delayed_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_robot,
            on_start=[spawn_controller],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        gazebo_to_pose,
        ekf,
        dual_error_plotter,
        spawn_robot,
        delayed_controller
    ])
