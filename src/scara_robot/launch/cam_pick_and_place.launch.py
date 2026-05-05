"""Camera-driven SCARA pick-and-place via MoveIt.

This launch chains:

  scara_moveit_config/demo.launch.py
        starts move_group + RViz + ros2_control fake hardware
  detect_bottle_cap (SCARA_pkg)
        RealSense + YOLO -> /bottle_cap/workspace_position
  cam_pick_and_place (scara_robot)
        stability gate + MoveIt pose targets on the "arm" group

Compared to SCARA_pkg/launch/cam_pick_and_place.launch.py (which uses the
analytical IK in ikpos.py and a Python state machine), this version uses
MoveIt's KDL solver and the same MoveGroupInterface plumbing as the
existing src/pick_and_place.cpp coordinate demo.

Calibration:
    base_to_workspace_x_m / base_to_workspace_y_m  — workspace-centre
        offset in the SCARA base frame.
    z_travel_m / z_pick_m  — Cartesian z of Link_ee in base_link frame
        for travel and engaged states. Tune to your URDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_demo = os.path.join(
        get_package_share_directory("scara_moveit_config"),
        "launch", "demo.launch.py",
    )

    show_preview      = LaunchConfiguration("show_preview")
    class_filter      = LaunchConfiguration("class_filter")
    min_confidence    = LaunchConfiguration("min_confidence")
    base_off_x        = LaunchConfiguration("base_to_workspace_x_m")
    base_off_y        = LaunchConfiguration("base_to_workspace_y_m")
    stable_frames     = LaunchConfiguration("stable_frames")
    stable_deadband_m = LaunchConfiguration("stable_deadband_m")
    z_travel_m        = LaunchConfiguration("z_travel_m")
    z_pick_m          = LaunchConfiguration("z_pick_m")
    place_x_m         = LaunchConfiguration("place_x_m")
    place_y_m         = LaunchConfiguration("place_y_m")
    cooldown_s        = LaunchConfiguration("cooldown_s")
    dwell_engage_s    = LaunchConfiguration("dwell_engage_s")

    return LaunchDescription([
        DeclareLaunchArgument("show_preview",          default_value="false"),
        DeclareLaunchArgument("class_filter",          default_value="bottle_cap"),
        DeclareLaunchArgument("min_confidence",        default_value="0.50"),
        DeclareLaunchArgument("base_to_workspace_x_m", default_value="0.0"),
        DeclareLaunchArgument("base_to_workspace_y_m", default_value="0.20"),
        DeclareLaunchArgument("stable_frames",         default_value="8"),
        DeclareLaunchArgument("stable_deadband_m",     default_value="0.01"),
        DeclareLaunchArgument("z_travel_m",            default_value="0.10"),
        DeclareLaunchArgument("z_pick_m",              default_value="0.04"),
        DeclareLaunchArgument("place_x_m",             default_value="0.10"),
        DeclareLaunchArgument("place_y_m",             default_value="0.25"),
        DeclareLaunchArgument("cooldown_s",            default_value="2.0"),
        DeclareLaunchArgument("dwell_engage_s",        default_value="1.0"),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_demo)),

        Node(
            package="SCARA_pkg",
            executable="detect_bottle_cap",
            name="bottle_cap_detector",
            output="screen",
            parameters=[{"show_preview": show_preview}],
        ),
        Node(
            package="scara_robot",
            executable="cam_pick_and_place",
            name="scara_cam_pick_and_place",
            output="screen",
            parameters=[{
                "class_filter":          class_filter,
                "min_confidence":        min_confidence,
                "base_to_workspace_x_m": base_off_x,
                "base_to_workspace_y_m": base_off_y,
                "stable_frames":         stable_frames,
                "stable_deadband_m":     stable_deadband_m,
                "z_travel_m":            z_travel_m,
                "z_pick_m":              z_pick_m,
                "place_x_m":             place_x_m,
                "place_y_m":             place_y_m,
                "cooldown_s":            cooldown_s,
                "dwell_engage_s":        dwell_engage_s,
            }],
        ),
    ])
