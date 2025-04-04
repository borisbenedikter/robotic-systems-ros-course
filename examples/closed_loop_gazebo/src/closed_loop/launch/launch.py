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
        # Node(
        #     package='turtlesim',
        #     executable='turtlesim_node',
        #     name='turtle1'
        # ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/rover_blue_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen',  # Optional: to see the output in the terminal
            name='bridge_cmd_vel'  # Name of the parameter bridge node
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/rover_blue/pose@geometry_msgs/msg/Pose@gz.msgs.Pose'
            ],
            output='screen',
            name='bridge_pose'
        ),
    ])
