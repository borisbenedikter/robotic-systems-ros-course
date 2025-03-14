from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='closed_loop',
            executable='motion_command',
            name='motion_command'         
        ),
        Node(
            package='closed_loop',
            executable='wall_detector',
            name='wall_detector'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1'
        ),
    ])
