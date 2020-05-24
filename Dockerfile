FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04

# Setup environment
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install basic packages
RUN apt update && apt install -y \
    curl \
    sudo \
    tmux \
    openssh-server \
    software-properties-common \
    bash-completion \
    debian-keyring \
    debian-archive-keyring \
    tzdata \
    wget

# Update apt sources
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sh -c \
    'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ros-kinetic and gazebo9
RUN apt update && \
    apt install -y \
    ros-melodic-desktop \
    gazebo9 \
    libgazebo9-dev \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control

# Install other ROS packages
RUN apt update && \
    apt install -y \
    ros-melodic-xacro \
    ros-melodic-amcl \
    ros-melodic-camera-calibration \
    ros-melodic-controller-manager \
    ros-melodic-ecl-threads \
    ros-melodic-hardware-interface \
    ros-melodic-joint-limits-interface \
    ros-melodic-move-base \
    ros-melodic-pcl-ros \
    ros-melodic-robot-localization \
    ros-melodic-transmission-interface \
    ros-melodic-yocs-controllers

# Create a user with passwordless sudo
RUN adduser -uid 1001 --gecos "Development User" --disabled-password agustin
RUN adduser agustin sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/agustin/.bashrc

RUN usermod -a -G video agustin
RUN usermod -a -G dialout agustin
USER agustin

RUN yes | sudo apt-get install ros-melodic-turtlebot3
RUN yes | sudo apt-get install ros-melodic-turtlebot3-simulations
RUN yes | sudo apt-get install vim

WORKDIR /home/agustin/padawan
ENTRYPOINT ./setup_ws.sh



