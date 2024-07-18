FROM ubuntu:20.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV \ 
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN apt update \
    && apt upgrade -y \
    && apt-get install --no-install-recommends -y \
        ca-certificates \
        locales \
        git \
        lsb-release \
        sudo \
        nano

RUN apt update \
    && apt upgrade -y \
    && apt-get install --no-install-recommends -y \
        wget \
        gnupg \
        python3 \
        python3-pip

RUN pip install empy==3.3.4

RUN \
    locale-gen en_US en_US.UTF-8\
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN \
    groupadd user \
    && useradd -ms /bin/zsh user -g user
RUN \
  echo "user ALL=NOPASSWD: ALL" >> /etc/sudoers
USER user
WORKDIR /home/user

RUN \
    git clone https://github.com/PX4/PX4-Autopilot.git -b v1.14.3 \
    && git -C PX4-Autopilot submodule update --init --recursive  \
    && PX4-Autopilot/Tools/setup/ubuntu.sh \
    && git clone https://github.com/PX4/px4_msgs.git -b release/1.14 \
    && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git -b v2.4.2

RUN \ 
    cd /home/user/PX4-Autopilot \
    && make px4_sitl


    



# ------ 빌드 후 필요한 추가작업 -------
# 1. DDS
# cd Micro-XRCE-DDS-Agent
# nano CMakelists.txt
# fastdds 버전을 2.12.x --> 2.12.2로 수정
# mkdir build
# cd build
# cmake ..
# make
# sudo make install
# sudo ldconfig /usr/local/lib/

# 2. ROS2 foxy
# sudo apt-get update
# sudo apt-get install -y software-properties-common curl gnupg2 lsb-release 
# sudo add-apt-repository universe 
# sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
# sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
# sudo apt-get update
# sudo apt-get upgrade -y
# sudo apt-get install -y ros-foxy-desktop python3-argcomplete ros-foxy-ros-base ros-dev-tools
# echo "source /opt/ros/foxy/setup.bash" >> /home/user/.bashrc

# ------ TIP ------
    # gazebo를 도커환경에서 실행하기 위해서는 로컬환경에서 'xhost +' 명령어 실행할것


# ------ IMAGE BUILD COMMAND ------
# docker build -t rohang .

# ------ CONTAINER BUILD COMMAND ------
# docker run -it \
#    -e DISPLAY=${DISPLAY} \
#    -e QT_NO_MITSHM=1 \
#    -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
#    -v /tmp/.X11-unix:/tmp/.X11-unix \
#    --device=/dev/dri:/dev/dri \
#    --privileged \
#    --name seungwon \
#    rohang bash
