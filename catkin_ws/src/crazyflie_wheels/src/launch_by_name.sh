#!/bin/bash 

name="$1"

echo "DIR IS: $(pwd)"

param_file=$(cat $(find -name '*cf_params.csv'))

params=$(echo "$param_file" | grep "$name")

uri="$(echo $params | cut -d , -f2)"
roll_trim="$(echo $params | cut -d , -f3)"
pitch_trim="$(echo $params | cut -d , -f4)"
tf_prefix="$(echo $params | cut -d , -f5)"

uri="uri:=$uri"

if [ -n "$roll_trim" ];	then
	roll_trim="roll_trim:=$roll_trim"
fi
if [ -n "$pitch_trim" ];	then
	pitch_trim="pitch_trim:=$pitch_trim"
fi
if [ -n "$tf_prefix" ];	then
	tf_prefix="tf_prefix:=$tf_prefix"
fi

roslaunch crazyflie_wheels teleop_wheels.launch "veh:=$name" $uri $roll_trim $pitch_trim $tf_prefix

exit 0

# {
# echo $name
# read line
# while read line; do
# 	if [ "$line" = "^$name*" ]; then
# 		uri="$(echo $line | cut -d , -f2)"
# 		roll_trim="$(echo $line | cut -d , -f3)"
# 		pitch_trim="$(echo $line | cut -d , -f4)"
# 		tf_prefix="$(echo $line | cut -d , -f5)"

# 		#roslaunch teleop_wheels.launch veh=$name
# 	fi

# done
# }<cf_params.csv


# assignAddress() {
# 	if [$1 = 'ace']; then
# 		roslaunch teleop_wheels.launch veh=$1 uri='radio://0/80/2M/E7E7E7E7EA'
# 	fi
# }

# assignAddress $1