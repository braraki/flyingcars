# flyingcars

Crazyflie Parameter are stored in Crazyflie_wheels/src/cf_params.csv. A new flie's parameters should be added according to the header:

name,uri,roll_trim,pitch_trim,tf_prefix

Add an empty ,, if you don't want to specify a parameter. Only name and uri need to be included.

To launch a Crazyflie by name, use

rosrun crazyflie_wheels launch_by_name.sh [name]

TO DO:

-make RViz autostart for individual Crazyflies. 
