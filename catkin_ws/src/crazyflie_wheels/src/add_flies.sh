#!/bin/bash

while [ -n "$1" ]; do
	rosrun crazyflie_wheels launch_by_name.sh "$1" &
	shift
done