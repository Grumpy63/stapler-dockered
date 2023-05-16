# This is an auto generated Dockerfile for ros:ros1-bridge
# generated from docker_images_ros2/ros1_bridge/create_ros_ros1_bridge_image.Dockerfile.em
FROM ros:noetic-ros-base-focal

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
#RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV ROS1_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 apt-utils \
 wget \
 git \
 python3-catkin-tools \
 bash-completion \
 build-essential \
 sudo \
 curl \
 python3-catkin-tools \
 python3-pip \
 python3-rosdep \
 python3-rosinstall-generator \
 python3-matplotlib \
 python3-tk \
 python3-vcstool \
 build-essential \
 bash-completion \
 ros-noetic-rqt \
 ros-noetic-rqt-common-plugins \
 ros-noetic-navigation \
 ros-noetic-robot-localization \
 ros-noetic-robot-state-publisher \
 ros-noetic-ros-comm \
 ros-noetic-roscpp-tutorials \
 ros-noetic-rospy-tutorials \
 ros-noetic-laser-geometry \
 ros-noetic-pcl-ros \
 ros-noetic-pcl-conversions \
 ros-noetic-roslint \
 ros-noetic-xacro \
 # clear apt cache
 && rm -rf /var/lib/apt/lists/*

# install gtsam
RUN sudo apt update
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository ppa:borglab/gtsam-release-4.0 -y
RUN sudo apt install libgtsam-dev libgtsam-unstable-dev -y

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* 

# set working directory
WORKDIR /home/catkin_ws

# setup entrypoint
COPY ./ros_entrypoint.sh /home/catkin_ws/
#COPY ./catkin_ws/src /home/catkin_ws/src
RUN chmod +x /home/catkin_ws/ros_entrypoint.sh
ENTRYPOINT ./ros_entrypoint.sh

# register source for rosdep
#RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update
RUN rosdep update
