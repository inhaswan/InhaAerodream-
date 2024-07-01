from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='precision_landing',
            executable='detect',
            name='detect',
            output='screen'),

        Node(
            package='precision_landing',
            executable='control',
            name='control',
            output='screen'),
    ])