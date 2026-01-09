from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    pkg_name = "diff_drive_robot"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(pkg_name),
                    "urdf",
                    "diff_drive.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "config", "rviz", "robot.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(gui),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        )
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare(pkg_name),
                "config",
                "gz_bridge.yaml",
            ]
        ),
        allow_substs=True,
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[bridge_params],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    delay_spawn_entity = TimerAction(period=3.0, actions=[spawn_entity])

    delay_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node],
        condition=IfCondition(gui),
    )

    nodes = [
        robot_state_pub_node,
        delay_rviz,
        gazebo,
        delay_spawn_entity,
        ros_gz_bridge,
    ]

    return LaunchDescription(declared_arguments + nodes)
