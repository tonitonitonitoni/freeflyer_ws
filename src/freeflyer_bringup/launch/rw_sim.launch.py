from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    bringup_pkg = FindPackageShare("freeflyer_bringup")
    desc_pkg = FindPackageShare("freeflyer_description")

    sim_pkg = FindPackageShare("freeflyer_gazebo").find("freeflyer_gazebo")
    world_file = os.path.join(sim_pkg, "worlds", "air_table.world")

    urdf_file = PathJoinSubstitution([
        desc_pkg,
        "urdf",
        "ctrl_rw.urdf.xacro"
    ])

    controllers_file = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "rw_rad_controller.yaml"
    ])
    ekf_yaml = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "ekf_both.yaml"
    ])

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

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

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

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "freeflyer",
            "-x", "1.2",
            "-y", "0.0",
            "-z", "0.55",
            "-Y", "3.141592653589793",
        ],
        output="screen"
    )

    spawn_radial_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "radial_controller",
            "--param-file",
            controllers_file
        ],
        output="screen"
    )

    spawn_attitude_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "attitude_controller",
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

    thruster_odom_predictor = Node(
        package='freeflyer_control',
        executable='thruster_rw_odom_predictor',
        name='thruster_rw_odom_predictor',
        output='screen'
    )

    dual_error_plotter = Node(
        package="freeflyer_diagnostics",
        executable="dual_error_plotter",
        name="dual_error_plotter",
        output="screen"
    )

    path_publisher = Node(
        package="freeflyer_control",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }]
    )

    delayed_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_robot,
            on_start=[spawn_radial_controller, spawn_attitude_controller],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        gazebo_to_pose,
        thruster_odom_predictor,
        ekf,
        dual_error_plotter,
        path_publisher,
        spawn_robot,
        delayed_controllers
    ])
