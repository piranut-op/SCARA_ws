"""Bring up the quick-path hardware bridge:

  joint_states_bridge  — taps /joint_states and republishes per-joint cmds
  newposition          — ODrive CAN bridge for Link_1_joint, Link_2_joint
  mks_ee               — MKS CAN bridge for ee_joint (prismatic)

Run alongside the MoveIt2 demo from ~/scara_bot_ws/:

    # terminal A
    cd ~/scara_bot_ws && source install/setup.bash
    ros2 launch scara_moveit_config demo_bot.launch.py

    # terminal B
    cd ~/SCARA && source install/setup.bash
    ros2 launch SCARA_pkg scara_hw_bridge.launch.py

Bring CAN up first:
    sudo ip link set can0 up type can bitrate 250000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_iface = LaunchConfiguration('can_interface')
    set_zero  = LaunchConfiguration('zero_on_start')
    set_foc   = LaunchConfiguration('set_bus_foc_on_start')

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('zero_on_start', default_value='false',
                              description='Send MKS 0x92 set-zero at start.'),
        DeclareLaunchArgument('set_bus_foc_on_start', default_value='false',
                              description='Send MKS 0x82 mode=5 (Bus FOC) at start.'),

        Node(
            package='SCARA_pkg',
            executable='joint_states_bridge',
            name='joint_states_bridge',
            output='screen',
        ),
        Node(
            package='SCARA_pkg',
            executable='newposition',
            name='odrive_angle_can_node',
            output='screen',
            parameters=[{'can_interface': can_iface}],
        ),
        Node(
            package='SCARA_pkg',
            executable='mks_ee',
            name='mks_ee_can_node',
            output='screen',
            parameters=[{
                'can_interface': can_iface,
                'zero_on_start': set_zero,
                'set_bus_foc_on_start': set_foc,
            }],
        ),
    ])
