### 놀라운 사실
- PX4 Avoidance에서 무지성 git clone 해와서 실행하면 실행이 안됨
  
### 요구사항
- ROS Noetic (Ubuntu 20.04 LTS) & Gazebo-classic 11
  
### SITL 검증을 위헤 필요한 것
- catkin_ws에서 source ~/.bashrc와 source devel/setup.bash 필수적으로 진행할 것
- 기본적으로 세팅된 launch 파일 실행 시 다음과 같은 에러 발생
```
[ERROR] [1727524616.131229509]: PR: Unknown parameter to get: MPC_Z_VEL_MAX_DN
[ERROR] [1727524616.137825165]: PR: Unknown parameter to get: MPC_Z_VEL_MAX_UP
[ERROR] [1727524616.142827148]: PR: Unknown parameter to get: CP_DIST
[ERROR] [1727524616.148006736]: PR: Unknown parameter to get: MPC_LAND_SPEED
[ERROR] [1727524616.153743811]: PR: Unknown parameter to get: MPC_JERK_MAX
[ERROR] [1727524616.157333086]: PR: Unknown parameter to get: NAV_ACC_RAD
[ERROR] [1727524616.161014908]: PR: Unknown parameter to get: MPC_YAWRAUTO_MAX
```
- 해결하기 위해서는
```
catkin_ws/src/avoidance/avoidance/launch/avoidance_sitl_mavros.launch 파일에서
    <arg name="gui" default="true"/>
이거 원래 false로 되어있는데 true로 수정해주기
```
- 그러고 다시 roslaunch를 하면, 굉장히 오랜 시간이 기다리면 gazebo가 켜지면서 오류 메시지가 해결됨
- Github Issue를 베이스로 추정하자면, Gazebo가 켜져야 각종 tree나 파라미터를 PX4 Avoidance가 get할 수 있는 모양
