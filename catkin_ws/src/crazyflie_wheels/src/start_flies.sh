#!/bin/bash

#Usage: start_flies.sh [name1] (name2) ... (nameN)

#IMPORTANT: script assumes roscore is already running.

cf_1="$1"

roslaunch crazyflie_wheels crazyflie_server.launch &

while [ -n "$1" ]; do
	rosrun crazyflie_wheels launch_by_name.sh "$1" &
	shift
done

rosrun joy joy_node /dev/input/js0 &
rosrun topic_tools relay /joy /$cf_1/joy &

wait