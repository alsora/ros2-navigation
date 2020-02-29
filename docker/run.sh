#!/bin/bash
#
# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2020

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

IMG_NAME="ros2_eloquent"

NETWORK_SETTINGS="--net=host --privileged"

XSOCK=/tmp/.X11-unix
DISPLAY_SETTINGS="-e DISPLAY=$DISPLAY -v $XSOCK:$XSOCK -v $XAUTHORITY:/root/.Xauthority"

if [[ $1 == "--dev" ]]; then
    # If developer mode, mount the whole repository
    DEV_SETTINGS="-v $THIS_DIR/..:/root/my_repo"
else
    # Else mount only the simulation script in read only mode
    DEV_SETTINGS="-v $THIS_DIR/../simulation.sh:/root/simulation.sh:ro"
fi

ENTRY_CMD="bash"

# Start Docker container
docker run -it --rm \
    $NETWORK_SETTINGS \
    $DISPLAY_SETTINGS \
    $DEV_SETTINGS \
    $IMG_NAME \
    $ENTRY_CMD
