from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='point_cloud',  # point_cloud.py 노드를 실행
            name='point_cloud',
            output='screen',
            respawn=True,  # 노드가 죽으면 다시 시작함
            respawn_delay=0.05  # 0.05초 후에 다시 시작
        ),

        Node(
            package='object_detection',
            executable='offboard_control',  # offboard_control.py 노드를 실행
            name='offboard_control',
            output='screen',
            # respawn=True,  # 노드가 죽으면 다시 시작함
            # respawn_delay=0.05  # 0.05초 후에 다시 시작
        ),
    ])
