from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='practica9_pkg',                  # paquete
            executable='context_label_publisher',     # nodo C++
            name='context_label_publisher',
            output='screen'
        )
    ])

