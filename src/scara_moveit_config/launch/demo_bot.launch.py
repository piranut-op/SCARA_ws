"""Bring up MoveIt2 (move_group + RViz + fake controllers) and run a demo node.

Usage:
    ros2 launch scara_moveit_config demo_bot.launch.py demo:=move
    ros2 launch scara_moveit_config demo_bot.launch.py demo:=cartesian_move
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    demo_arg = DeclareLaunchArgument(
        "demo",
        default_value="pick_and_place",
        description="Executable from scara_robot to launch (move | pick_and_place)",
    )

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("scara_moveit_config"),
            "/launch/demo.launch.py",
        ]),
    )

    demo_node = Node(
        package="scara_robot",
        executable=LaunchConfiguration("demo"),
        name="scara_demo",
        output="screen",
    )

    # Give move_group a moment to come up before the client connects.
    delayed_demo = TimerAction(period=5.0, actions=[demo_node])

    return LaunchDescription([demo_arg, moveit_demo, delayed_demo])
