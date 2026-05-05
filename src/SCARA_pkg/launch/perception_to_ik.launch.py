"""Camera-driven SCARA targeting.

Pipeline:

  detect_bottle_cap   RealSense + YOLO -> /bottle_cap/workspace_position
        v
  cap_to_target       picks best detection, applies base offset -> /ik_target
        v
  ikpos               SCARA IK -> /odrive/angle_cmd

Optionally start the hardware drivers (newposition + mks_ee) with
`with_hardware:=true`. Bring CAN up first:

    sudo ip link set can0 up type can bitrate 250000

Calibration: measure the offset from the SCARA base origin to the workspace
centre (camera-derived frame), in metres, and pass via:
    base_to_workspace_x_m:=<x>  base_to_workspace_y_m:=<y>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_preview        = LaunchConfiguration("show_preview")
    class_filter        = LaunchConfiguration("class_filter")
    min_confidence      = LaunchConfiguration("min_confidence")
    base_off_x          = LaunchConfiguration("base_to_workspace_x_m")
    base_off_y          = LaunchConfiguration("base_to_workspace_y_m")
    deadband_m          = LaunchConfiguration("deadband_m")
    min_publish_period  = LaunchConfiguration("min_publish_period_s")
    with_hardware       = LaunchConfiguration("with_hardware")
    can_iface           = LaunchConfiguration("can_interface")

    return LaunchDescription([
        DeclareLaunchArgument("show_preview",          default_value="false"),
        DeclareLaunchArgument("class_filter",          default_value="bottle_cap"),
        DeclareLaunchArgument("min_confidence",        default_value="0.40"),
        DeclareLaunchArgument("base_to_workspace_x_m", default_value="0.0"),
        DeclareLaunchArgument("base_to_workspace_y_m", default_value="0.20"),
        DeclareLaunchArgument("deadband_m",            default_value="0.005"),
        DeclareLaunchArgument("min_publish_period_s",  default_value="0.20"),
        DeclareLaunchArgument("with_hardware",         default_value="false"),
        DeclareLaunchArgument("can_interface",         default_value="can0"),

        Node(
            package="SCARA_pkg",
            executable="detect_bottle_cap",
            name="bottle_cap_detector",
            output="screen",
            parameters=[{"show_preview": show_preview}],
        ),
        Node(
            package="SCARA_pkg",
            executable="cap_to_target",
            name="cap_to_target",
            output="screen",
            parameters=[{
                "class_filter":          class_filter,
                "min_confidence":        min_confidence,
                "base_to_workspace_x_m": base_off_x,
                "base_to_workspace_y_m": base_off_y,
                "deadband_m":            deadband_m,
                "min_publish_period_s":  min_publish_period,
            }],
        ),
        Node(
            package="SCARA_pkg",
            executable="ikpos",
            name="inverse_kinematics_node",
            output="screen",
        ),

        Node(
            package="SCARA_pkg",
            executable="newposition",
            name="odrive_angle_can_node",
            output="screen",
            condition=IfCondition(with_hardware),
            parameters=[{"can_interface": can_iface}],
        ),
        Node(
            package="SCARA_pkg",
            executable="mks_ee",
            name="mks_ee_can_node",
            output="screen",
            condition=IfCondition(with_hardware),
            parameters=[{"can_interface": can_iface}],
        ),
    ])
