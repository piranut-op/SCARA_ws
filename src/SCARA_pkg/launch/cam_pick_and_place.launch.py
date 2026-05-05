"""One-shot camera-driven pick-and-place bring-up.

Brings up the entire stack:

  detect_bottle_cap   RealSense + YOLO -> /bottle_cap/workspace_position
  ikpos               /ik_target -> /odrive/angle_cmd
  newposition         /odrive/angle_cmd -> CAN (Link_1, Link_2)
  mks_ee              /mks/ee_cmd -> CAN (prismatic ee_joint)
  pick_and_place_cam  state machine driving /ik_target + /mks/ee_cmd

Bring CAN up first:
    sudo ip link set can0 up type can bitrate 250000

Calibration: measure the offset from the SCARA base origin to the workspace
centre (camera-derived frame), in metres, and pass via:
    base_to_workspace_x_m:=<x>  base_to_workspace_y_m:=<y>

EE heights, place / home positions, and timings are all launch arguments
(see the parameter declarations below). Tune them dry-run before going live.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_preview        = LaunchConfiguration("show_preview")
    can_iface           = LaunchConfiguration("can_interface")
    with_hardware       = LaunchConfiguration("with_hardware")

    class_filter        = LaunchConfiguration("class_filter")
    min_confidence      = LaunchConfiguration("min_confidence")

    base_off_x          = LaunchConfiguration("base_to_workspace_x_m")
    base_off_y          = LaunchConfiguration("base_to_workspace_y_m")

    stable_frames       = LaunchConfiguration("stable_frames")
    stable_deadband_m   = LaunchConfiguration("stable_deadband_m")

    ee_travel_m         = LaunchConfiguration("ee_travel_m")
    ee_pick_m           = LaunchConfiguration("ee_pick_m")
    ee_place_m          = LaunchConfiguration("ee_place_m")

    place_x_m           = LaunchConfiguration("place_x_m")
    place_y_m           = LaunchConfiguration("place_y_m")
    home_x_m            = LaunchConfiguration("home_x_m")
    home_y_m            = LaunchConfiguration("home_y_m")

    approach_s          = LaunchConfiguration("approach_s")
    descend_s           = LaunchConfiguration("descend_s")
    dwell_pick_s        = LaunchConfiguration("dwell_pick_s")
    lift_s              = LaunchConfiguration("lift_s")
    to_place_s          = LaunchConfiguration("to_place_s")
    dwell_place_s       = LaunchConfiguration("dwell_place_s")
    home_s              = LaunchConfiguration("home_s")
    cooldown_s          = LaunchConfiguration("cooldown_s")

    return LaunchDescription([
        DeclareLaunchArgument("show_preview",          default_value="false"),
        DeclareLaunchArgument("can_interface",         default_value="can0"),
        DeclareLaunchArgument("with_hardware",         default_value="true"),

        DeclareLaunchArgument("class_filter",          default_value="bottle_cap"),
        DeclareLaunchArgument("min_confidence",        default_value="0.50"),

        DeclareLaunchArgument("base_to_workspace_x_m", default_value="0.0"),
        DeclareLaunchArgument("base_to_workspace_y_m", default_value="0.20"),

        DeclareLaunchArgument("stable_frames",         default_value="8"),
        DeclareLaunchArgument("stable_deadband_m",     default_value="0.01"),

        DeclareLaunchArgument("ee_travel_m",           default_value="0.060"),
        DeclareLaunchArgument("ee_pick_m",             default_value="-0.040"),
        DeclareLaunchArgument("ee_place_m",            default_value="-0.040"),

        DeclareLaunchArgument("place_x_m",             default_value="0.10"),
        DeclareLaunchArgument("place_y_m",             default_value="0.25"),
        DeclareLaunchArgument("home_x_m",              default_value="0.0"),
        DeclareLaunchArgument("home_y_m",              default_value="0.20"),

        DeclareLaunchArgument("approach_s",            default_value="2.0"),
        DeclareLaunchArgument("descend_s",             default_value="1.5"),
        DeclareLaunchArgument("dwell_pick_s",          default_value="0.8"),
        DeclareLaunchArgument("lift_s",                default_value="1.5"),
        DeclareLaunchArgument("to_place_s",            default_value="2.5"),
        DeclareLaunchArgument("dwell_place_s",         default_value="0.8"),
        DeclareLaunchArgument("home_s",                default_value="2.0"),
        DeclareLaunchArgument("cooldown_s",            default_value="2.0"),

        Node(
            package="SCARA_pkg",
            executable="detect_bottle_cap",
            name="bottle_cap_detector",
            output="screen",
            parameters=[{"show_preview": show_preview}],
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
        Node(
            package="SCARA_pkg",
            executable="pick_and_place_cam",
            name="pick_and_place_cam",
            output="screen",
            parameters=[{
                "class_filter":          class_filter,
                "min_confidence":        min_confidence,
                "base_to_workspace_x_m": base_off_x,
                "base_to_workspace_y_m": base_off_y,
                "stable_frames":         stable_frames,
                "stable_deadband_m":     stable_deadband_m,
                "ee_travel_m":           ee_travel_m,
                "ee_pick_m":             ee_pick_m,
                "ee_place_m":            ee_place_m,
                "place_x_m":             place_x_m,
                "place_y_m":             place_y_m,
                "home_x_m":              home_x_m,
                "home_y_m":              home_y_m,
                "approach_s":            approach_s,
                "descend_s":             descend_s,
                "dwell_pick_s":          dwell_pick_s,
                "lift_s":                lift_s,
                "to_place_s":            to_place_s,
                "dwell_place_s":         dwell_place_s,
                "home_s":                home_s,
                "cooldown_s":            cooldown_s,
            }],
        ),
    ])
