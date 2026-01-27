import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Package name
    package_name = "diff_drive_robot"

    # Path to default world
    world_path = os.path.join(
        get_package_share_directory(package_name), "worlds", "my_world.sdf"
    )

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model file to load",
    )

    declare_rviz = DeclareLaunchArgument(
        name="rviz", default_value="True", description="Opens rviz is set to True"
    )

    # Launch configurations
    world = LaunchConfiguration("world")
    rviz = LaunchConfiguration("rviz")

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(
        get_package_share_directory(package_name), "urdf", "diff_drive.urdf.xacro"
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "urdf": urdf_path}.items(),
    )

    # Launch the gazebo server to initialize the simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v1 ", world],  # GUI ON by default
            'on_exit_shutdown':'true'
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package.
    spawn_diff_bot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "diff_bot", "-z", "0.13"],
        output="screen",
    )

    delay_spawn_diff_bot = TimerAction(
        period=10.0,
        actions=[spawn_diff_bot],
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(
        get_package_share_directory(package_name), "config", "gz_bridge.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    # Launch Rviz with diff bot rviz file
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "config", "rviz", "robot.rviz"
    )
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    delay_rviz2 = TimerAction(
        period=10.0,
        actions=[rviz2],
    )

    # Launch them all!
    return LaunchDescription(
        [
            # Declare launch arguments
            declare_rviz,
            declare_world,
            # Launch the nodes
            rsp,
            ros_gz_bridge,
            gazebo,
            delay_spawn_diff_bot,
            delay_rviz2,
        ]
    )
