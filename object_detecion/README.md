<개발 환경>
- Ubuntu 20.04 LTS
- ROS2 Foxy
- Gazebo classic(Gazebo 11)
- cmake 3.20.0
- setuptools 60.2.0
- python 3.8.10
- open3d 0.18.0

<테스트 해본 환경>
- Ubuntu 20.04 LTS
- ROS2 Foxy

<필요한 라이브러리>
1. Open3D
$ pip3 install open3d

ERROR: jupyter-packaging 0.12.3 has requirement setuptools>=60.2.0, but you'll have setuptools 45.2.0 which is incompatible.\
ERROR: importlib-resources 6.4.0 has requirement zipp>=3.1.0; python_version < "3.10", but you'll have zipp 1.0.0 which is incompatible.\
ERROR: nbconvert 7.16.4 has requirement jinja2>=3.0, but you'll have jinja2 2.10.1 which is incompatible.\
ERROR: jupyter-server 2.14.0 has requirement jinja2>=3.0.3, but you'll have jinja2 2.10.1 which is incompatible.\
ERROR: jupyter-server 2.14.0 has requirement packaging>=22.0, but you'll have packaging 20.3 which is incompatible.\
ERROR: jupyterlab-server 2.27.1 has requirement jinja2>=3.0.3, but you'll have jinja2 2.10.1 which is incompatible.\
ERROR: jupyterlab-server 2.27.1 has requirement packaging>=21.3, but you'll have packaging 20.3 which is incompatible.\

이런 오류들이 발생시 해결법은 간단하다.
저기서 요구하는 모든 패키지들을 업그레이드 하면 된다.
pip install --upgrade setuptools\
pip install 'setuptools>=60.2.0'\
pip install 'zipp>=3.1.0'\
pip install 'jinja2>=3.0.3'\
pip install 'packaging>=22.0'\
\
여기까지 완료했으면 
python3 -c "import open3d as o3d; print(o3d.__version__)"
open3d 버젼 확인
만약 0.18.0 버젼이 아니면
pip3 install 'open3d>=0.17'
업그레이드 필요함

2. gazebo_ros_pkg
3. sensor_msgs_py\
sudo apt-get install ros-foxy-sensor-msgs-py

<사용방법>
1번째 터미널.
cd <workspace_name>/object_detection
soruce install/local_setup.bash
ros2 launch object_detection sim.launch.py

2번째 터미널
cd <workspace_name>/object_detection
source install/loacl_setup.bash
ros2 run object_detection point_cloud

3번째 터미널
cd <workspace_name>/object_detection
source install/local_setup.bash
ros2 run object_detection offboard_control
