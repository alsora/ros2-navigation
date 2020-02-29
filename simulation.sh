#!/bin/bash
#
# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2020

THIS_SCRIPT=`basename "$0"`

main() {

    parse_arguments "$@"

    # Set default values
    if [ -z "$ROBOT" ]; then
        ROBOT=waffle
    fi

    # If requested to kill, do it and exit
    if [[ $KILL == "YES" ]]; then
        kill_simulation
    fi

    # Make sure gazebo is not already running
    kill_gazebo

    export TURTLEBOT3_MODEL=${ROBOT}

    # Start gazebo
    execute "gazebo --verbose -s libgazebo_ros_init.so /opt/ros/eloquent/share/nav2_bringup/worlds/$TURTLEBOT3_MODEL.model"
    
    # Start TF publisher
    execute "sleep 15; ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True"
    
    # Start Rviz
    execute "sleep 20; rviz2 -d /opt/ros/eloquent/share/nav2_bringup/rviz/nav2_default_view.rviz"
    
    # Start ROS 2 Navigation Stack
    execute "sleep 25; ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True autostart:=True map:=/opt/ros/eloquent/share/nav2_bringup/maps/turtlebot3_world.yaml"
}

parse_arguments() {
    POSITIONAL=()
    while [[ $# -gt 0 ]]
    do
    key="$1"

    case $key in
        -r|--robot)
        ROBOT="$2"
        shift # past argument
        shift # past value
        ;;
        kill|--kill)
        KILL=YES
        shift # past argument
        ;;
        *)    # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
    esac
    done
    set -- "${POSITIONAL[@]}" # restore positional parameters
}

kill_simulation() {
    pkill -9 xterm
    kill_gazebo
    
    echo "[$THIS_SCRIPT] Killed everything!"
    exit 0
}

kill_gazebo() {
    pkill -9 gzclient
    pkill -9 gzserver
    pkill -9 gazebo
}

execute() {
    if [ "$#" -ne 1 ]; then
        echo "[$THIS_SCRIPT] execute requires exactly 1 argument"
    fi
    xterm -hold -e "$1" &
}


main "$@"; exit
