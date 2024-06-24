# sim.launch.py
# 이 파일은 my_robot.launch.py, gazebo-ros-pkg의 런치 파일, gazebo에 내 URDF를
# 생성하는 런치 파일을 한번에 실행시키기 위해서 작성한 런치 파일이다.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "object_detection"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "my_robot.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    ) # 내가 만든 로봇을 gazebo로 송신하는 노드

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
    ) # gazebo-ros 연동을 통해 gazebo를 시작하는 파일

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "with_robot",
            "-x", "0", "-y", "0", "-z", "0",
            "-R", "0", "-P", "0", "-Y", "0"
        ],
        output="screen",
    )  # gazebo에 내 URDF 파일을 통해 로봇을 스폰하는 노드

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
        ]
    )
