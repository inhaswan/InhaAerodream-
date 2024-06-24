# My_robot.launch.py
# 이 파일은 내가 생성한 URDF 파일을 Gazebo에 토픽으로 전송하는 런치 파일이다.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
"""
본 런치 파일은 내가 작성한 URDF 파일을 기준으로 robot_description으로 하는 robot_state_publisher를 생성함.
robot_state_publisher는 tf2로 robot의 state를 보내는 노드로서,
간단하게 설명하면 내가 작성한 URDF 파일을 Gazebo 상에서 시현 및 구동 가능하게 만들어주는 런치 파일임.
"""
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory("object_detection"))
    xacro_file = os.path.join(pkg_path, "urdf", "my_robot.xacro")
    robot_description = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description.toxml(), "use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
        ]
    )