#!/bin/bash 

assignAddress() {
	if [$1 = 'ace']; then
		roslaunch teleop_wheels.launch veh=$1 uri='radio://0/80/2M/E7E7E7E7EA'
	fi
}

assignAddress $1