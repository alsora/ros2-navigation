# ros2-navigation

This repository allows to run ROS 2 navigation algorithms in a Gazebo simulated environment.

## Usage

Install `docker` following the [instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository).

Add your user to the docker group

    sudo groupadd docker
    sudo usermod -aG docker $USER
    sudo gpasswd -a $USER docker

Build the docker image

    bash docker/build.sh

Run the docker image

    bash docker/run.sh

Run the simulation

    ./simulation.sh
