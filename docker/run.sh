#!/bin/bash
#
# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2020

IMG_NAME="ros2_eloquent"

NETWORK_SETTINGS="--net=host --privileged"

XSOCK=/tmp/.X11-unix
DISPLAY_SETTINGS="-e DISPLAY="$DISPLAY" -v "$XSOCK":"$XSOCK" -v "$XAUTHORITY":/root/.Xauthority"

if [[ $1 == "--dev" ]]; then
    DEV_SETTINGS="-v "$PWD"/..:/root/my_repo"
else
    DEV_SETTINGS=""
fi

ENTRY_CMD="bash"

# Start Docker container
docker run -it --rm \
    $NETWORK_SETTINGS \
    $DISPLAY_SETTINGS \
    $DEV_SETTINGS \
    $IMG_NAME \
    $ENTRY_CMD
