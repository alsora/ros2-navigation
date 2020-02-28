# ros2_eloquent_dev Docker image


Build the Docker image and deploy a container

    $ bash build.sh
    $ bash run.sh


Try the image, by running nodes inside the Docker container

    # ros2 run demo_nodes_cpp listener & ros2 run demo_nodes_cpp talker

Make sure the X server is correctly shared by running an application with GUI

    # rviz2

#### Rebuild the docker image without using cache

    $ bash build.sh --force

#### Mount this repository folder to the docker container

    $ bash run.sh --dev