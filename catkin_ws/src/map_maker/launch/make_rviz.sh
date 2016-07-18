#!/bin/bash

cf_num=$1

current_num=0

#define rviz file as file
file=/home/crazyflier/flyingcars/catkin_ws/src/map_maker/launch/duckietown.rviz

#delete everything in file
> $file
#replace with base_world txt
cat /home/crazyflier/flyingcars/catkin_ws/src/map_maker/launch/base_world.rviz >> $file

#filename1=/home/crazyflier/flyingcars/catkin_ws/src/map_maker/launch/difference.txt

filename2=/home/crazyflier/flyingcars/catkin_ws/src/map_maker/launch/difference2.txt

while [ $current_num -lt $cf_num ]; do

	#sed -i "/#difference1/r $filename1" $file
	#sed -i -e $'s/#insertID/'$current_num$'/g' $file
	
	sed -i "/#difference2/r $filename2" $file
	sed -i -e $'s/#insertID/'$current_num$'/g' $file

	let current_num=current_num+1
done

