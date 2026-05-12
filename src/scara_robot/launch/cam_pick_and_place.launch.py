"""Camera-driven SCARA pick-and-place via MoveIt.

This launch chains:

  scara_moveit_config/demo.launch.py
        starts move_group + RViz + ros2_control fake hardware
  static_transform_publisher
        base_link → camera_color_optical_frame (your measured mount pose)
  detect_bottle_cap (SCARA_pkg)
        RealSense + YOLO -> /bottle_cap/detections
        (geometry_msgs/PointStamped in camera_color_optical_frame, metres)
  cam_pick_and_place (scara_robot)
        tf2 transform → MoveIt pose targets on the "arm" group

Calibration: only the camera mount pose and pick/place heights need tuning.

  cam_x / cam_y / cam_z       — translation of camera optical centre in
                                 base_link, metres.
  cam_roll / cam_pitch / cam_yaw — rotation of the optical frame relative
                                 to base_link, radians. For a camera
                                 mounted straight down, defaults are
                                 roll=π, pitch=0, yaw=0 (optical +Z points
                                 down so it deprojects into +base_link Z
                                 below the camera). Swap roll/yaw to
                                 rotate the image axes if needed.
  z_travel_m / z_pick_m       — Cartesian z of Link_ee in base_link for
                                 travel and engaged states. Tune to URDF.
  place_x_m / place_y_m       — drop location in base_link (m).
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_demo = os.path.join(
        get_package_share_directory("scara_moveit_config"),
        "launch", "demo.launch.py",
    )
    hw_bridge = os.path.join(
        get_package_share_directory("SCARA_pkg"),
        "launch", "scara_hw_bridge.launch.py",
    )

    use_hardware     = LaunchConfiguration("use_hardware")
    show_preview     = LaunchConfiguration("show_preview")
    cam_x            = LaunchConfiguration("cam_x")
    cam_y            = LaunchConfiguration("cam_y")
    cam_z            = LaunchConfiguration("cam_z")
    cam_roll         = LaunchConfiguration("cam_roll")
    cam_pitch        = LaunchConfiguration("cam_pitch")
    cam_yaw          = LaunchConfiguration("cam_yaw")
    z_travel_m       = LaunchConfiguration("z_travel_m")
    z_pick_m         = LaunchConfiguration("z_pick_m")
    place_x_m        = LaunchConfiguration("place_x_m")
    place_y_m        = LaunchConfiguration("place_y_m")
    cooldown_s       = LaunchConfiguration("cooldown_s")
    dwell_engage_s   = LaunchConfiguration("dwell_engage_s")
    workspace_z_m    = LaunchConfiguration("workspace_z_m")
    joint1_offset_turns = LaunchConfiguration("joint1_offset_turns")
    joint2_offset_turns = LaunchConfiguration("joint2_offset_turns")
    zero_on_start    = LaunchConfiguration("zero_on_start")

    return LaunchDescription([
        DeclareLaunchArgument("use_hardware",  default_value="false",
            description="If true, also bring up scara_hw_bridge (CAN servos)."),
        DeclareLaunchArgument("show_preview",  default_value="true"),

        # Camera mount pose (base_link → camera_color_optical_frame).
        DeclareLaunchArgument("cam_x",         default_value="0.20"),
        DeclareLaunchArgument("cam_y",         default_value="0.0"),
        DeclareLaunchArgument("cam_z",         default_value="0.69"),
        # Straight-down: optical +Z points downward into the workspace.
        DeclareLaunchArgument("cam_roll",      default_value=str(math.pi)),
        DeclareLaunchArgument("cam_pitch",     default_value="0.0"),
        DeclareLaunchArgument("cam_yaw",       default_value="0.0"),

        DeclareLaunchArgument("z_travel_m",    default_value="0.18"),
        DeclareLaunchArgument("z_pick_m",      default_value="0.10"),
        DeclareLaunchArgument("place_x_m",     default_value="0.10"),
        DeclareLaunchArgument("place_y_m",     default_value="0.25"),
        DeclareLaunchArgument("cooldown_s",    default_value="2.0"),
        DeclareLaunchArgument("dwell_engage_s", default_value="1.0"),
        # Workspace surface height in base_link (m). Caps sit on this plane;
        # the ray-plane intersection projects pixels onto it.
        DeclareLaunchArgument("workspace_z_m", default_value="0.0"),
        DeclareLaunchArgument("joint1_offset_turns", default_value="-0.5",
            description="Link_1 motor turns offset for hardware calibration."),
        DeclareLaunchArgument("joint2_offset_turns", default_value="0.0",
            description="Link_2 motor turns offset for hardware calibration."),
        DeclareLaunchArgument("zero_on_start", default_value="true",
            description="Send MKS 0x92 set-zero so the third motor adopts "
                        "its current shaft position as count=0 at startup. "
                        "Set true if you back-drive by hand before power-up."),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_demo)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hw_bridge),
            condition=IfCondition(use_hardware),
            launch_arguments={
                "joint1_offset_turns": joint1_offset_turns,
                "joint2_offset_turns": joint2_offset_turns,
                "zero_on_start":       zero_on_start,
            }.items(),
        ),

        # Static TF: base_link → camera_color_optical_frame.
        # Use named flags — positional ordering is yaw-pitch-roll, easy to
        # get backwards.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_optical",
            arguments=[
                "--x",     cam_x,
                "--y",     cam_y,
                "--z",     cam_z,
                "--roll",  cam_roll,
                "--pitch", cam_pitch,
                "--yaw",   cam_yaw,
                "--frame-id",       "base_link",
                "--child-frame-id", "camera_color_optical_frame",
            ],
        ),

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
                "z_travel_m":     z_travel_m,
                "z_pick_m":       z_pick_m,
                "place_x_m":      place_x_m,
                "place_y_m":      place_y_m,
                "cooldown_s":     cooldown_s,
                "dwell_engage_s": dwell_engage_s,
                "workspace_z_m":  workspace_z_m,
            }],
        ),
    ])
