FROM osrf/ros:eloquent-desktop
LABEL maintainer="alberto dot soragna at gmail dot com"

# Working directory
ENV HOME /root
WORKDIR $HOME

# General utilities
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

# Install development tools and ROS tools
RUN apt-get install -y \
  build-essential \
  cmake \
  python3-catkin-pkg-modules \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Install python packages
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

# Install Fast-RTPS dependencies
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# Install CycloneDDS dependencies
RUN apt-get install --no-install-recommends -y \
  libcunit1-dev
# Install cycloneDDS RMW implementation
RUN apt-get install -y ros-eloquent-rmw-cyclonedds-cpp

# Package required for building rviz plugins, but not included in base image
RUN apt-get install -y ros-eloquent-rviz-visual-testing-framework

# Install rqt tools
RUN apt-get install -y \
  ros-eloquent-python-qt-binding \
  ros-eloquent-qt-gui-core \
  ros-eloquent-rqt*

# Install gazebo
RUN /bin/bash -c 'curl -sSL http://get.gazebosim.org | sh'
RUN apt-get install -y ros-eloquent-gazebo-*
# Get gazebo models
RUN apt-get install unzip
RUN wget --progress=bar:force:noscroll https://bitbucket.org/osrf/gazebo_models/get/e6d645674e8a.zip \
  && unzip -q e6d645674e8a.zip \
  && rm e6d645674e8a.zip

# Install turtlebot packages
RUN mkdir -p $HOME/turtlebot_ws/src
WORKDIR $HOME/turtlebot_ws
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
RUN vcs import src < turtlebot3.repos

RUN /bin/bash -c 'source /opt/ros/eloquent/setup.sh; \
  colcon build --cmake-args --no-warn-unused-cli --symlink-install'

# Install ROS 2 navigation stack
RUN apt-get update && apt-get install -y ros-eloquent-navigation2 ros-eloquent-nav2-*

WORKDIR $HOME

# Source everything and export Gazebo models when opening new shell
RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source /opt/ros/eloquent/setup.sh \n\
source $HOME/turtlebot_ws/install/local_setup.sh \n\ 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/turtlebot_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models \n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/osrf-gazebo_models-e6d645674e8a'>> $HOME/.bashrc
