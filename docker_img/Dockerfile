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
    make px4_sitl











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