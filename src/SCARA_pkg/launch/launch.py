from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='SCARA_pkg',
            executable='ikpos',
            name='ikpos_node',
            output='screen'
        ),
        Node(
            package='SCARA_pkg',
            executable='fkpos',
            name='fkpos_node',
            output='screen'
        ),
        Node(
            package='SCARA_pkg',
            executable='newposition',
            name='newposition_node',
            output='screen'
        ),
    ])
