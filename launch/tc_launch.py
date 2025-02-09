from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tc',
            executable='center_publisher',
            output='screen'
        ),
        Node(
            package='tc',
            executable='center_subscriber',
            output='screen'
        )
    ])