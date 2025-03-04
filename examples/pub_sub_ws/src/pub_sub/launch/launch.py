from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pub_sub',              # package name
            executable='pub_sub_listener',  # executable name
            name='listener'                 # node name
        ),
        Node(
            package='pub_sub',
            executable='pub_sub_talker',
            name='talker'
        )
    ])
