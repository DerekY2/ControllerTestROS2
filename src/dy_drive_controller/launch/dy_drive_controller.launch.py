from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dy_drive_controller',
            executable='joy_republisher',
            name='joy_republisher'
        ),
        Node(
            package='dy_drive_controller',
            executable='drive_subscriber',
            name='drive_subscriber'
        )
    ])