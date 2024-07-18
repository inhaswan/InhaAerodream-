로컬에서 xhost + 한 후, 

docker run -it \
   -e DISPLAY=${DISPLAY} \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   --device=/dev/dri:/dev/dri \
   --privileged \
   --name seungwon \
   rohang bash



------ PX4-Autopilot(v1.14.3)------
cd ~/PX4-Autopilot
pip install empy==3.3.4
make px4_sitl
make px4_sitl gazebo-classic

make px4_sitl할 때 아래 에러를 피하고자
AttributeError: module 'em' has no attribute 'RAW_OPT'
pip install empy==3.3.4 로 잡음
왜냐하면 v3.3.3까지는 지원됬었기에 오류남



꿀팁: make distclean

sudo apt install nano 설치
cmake version 3.20.0수정

cmake버전 수정하니 오류생김
sudo ln -s /root/cmake-3.20.0-linux-x86_64/bin/cmake /usr/bin/cmake
로 해결한줄 알았으나 추가 문제 발생
px4 지우고 Dockerfile 참고해서 재설치 


------ DDS(v2.4.2)------
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git -b v2.4.2
cd Micro-XRCE-DDS-Agent
nano CMakelists.txt
fastdds 버전을 2.12.x --> 2.12.2로 수정
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

------ px4_msgs(v1.14) ------
git clone https://github.com/PX4/px4_msgs.git -b release/1.14

------ ros2(foxy) ------
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-foxy-ros-base python3-argcomplete
sudo apt install ros-dev-tools
~/.bashrc에 source /opt/ros/foxy/setup.bash 넣어주기
