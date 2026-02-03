from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    pose_file_arg = DeclareLaunchArgument(
        "pose_file",
        default_value="/workspaces/diff_drive_robot/src/stretch_flex/config/pose_registry.json",
        description="Path to pose registry JSON file",
    )

    pose_registry_node = Node(
        package="pose_registry",
        executable="pose_registry",
        name="pose_registry_node",
        parameters=[{"pose_file": LaunchConfiguration("pose_file")}],
    )

    stretch_flex_node = Node(
        package="stretch_flex",
        executable="stretch_flex",
        name="stretch_flex_node",
    )

    # args
    ld.add_action(pose_file_arg)

    # nodes
    ld.add_action(pose_registry_node)
    ld.add_action(stretch_flex_node)

    return ld
