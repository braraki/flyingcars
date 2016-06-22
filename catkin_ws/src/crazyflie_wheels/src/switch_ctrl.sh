#!/bin/bash

joy_relay="$(rosnode list | grep joy_relay)"

if [ -n "$joy_relay" ]; then
	rosnode kill $joy_relay
fi

rosrun topic_tools relay /joy /$1/joy &

wait