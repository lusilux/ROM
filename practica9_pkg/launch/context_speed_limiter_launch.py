from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='practica9_pkg',
            executable='context_speed_limiter',
            name='context_speed_limiter',
            output='screen'
        )
    ])

