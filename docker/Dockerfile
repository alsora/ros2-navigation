FROM osrf/ros:eloquent-desktop
LABEL maintainer="alberto dot soragna at gmail dot com"

# working directory
ENV HOME /root
WORKDIR $HOME

# general utilities
RUN apt-get update && apt-get install -y \
  curl \
  wget \
  git \
  vim \
  nano \
  python3-pip

# pip
RUN pip3 install --upgrade pip

# Locale options
RUN apt-get install -y locales
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install debug tools
RUN apt-get update && apt-get install -y \
  gdb \
  netcat \
  valgrind

# install development tools and ROS tools
RUN apt-get install -y \
  build-essential \
  cmake \
  python3-catkin-pkg-modules \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# install python packages
RUN pip3 install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

# install Fast-RTPS dependencies
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# install CycloneDDS dependencies
RUN apt-get install --no-install-recommends -y \
  libcunit1-dev

# package required for building rviz plugins, but not included in base image
RUN apt-get install -y ros-eloquent-rviz-visual-testing-framework

# install rqt tools
RUN apt-get install -y \
  ros-eloquent-python-qt-binding \
  ros-eloquent-qt-gui-core \
  ros-eloquent-rqt*

# install cycloneDDS RMW implementation
RUN apt-get install -y ros-eloquent-rmw-cyclonedds-cpp

# install gazebo
RUN /bin/bash -c 'curl -sSL http://get.gazebosim.org | sh'
RUN apt-get install -y ros-eloquent-gazebo-*

# install turtlebot packages
RUN mkdir -p $HOME/turtlebot_ws/src
WORKDIR $HOME/turtlebot_ws
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
RUN vcs import src < turtlebot3.repos

RUN /bin/bash -c 'source /opt/ros/eloquent/setup.sh; \
  colcon build --cmake-args --no-warn-unused-cli --symlink-install'

# install navigation stack
RUN apt-get update && apt-get install -y ros-eloquent-navigation2 ros-eloquent-nav2-*

WORKDIR $HOME

# source everything when opening new bash
RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source /opt/ros/eloquent/setup.sh \n\
source $HOME/turtlebot_ws/install/local_setup.sh' >> $HOME/.bashrc
