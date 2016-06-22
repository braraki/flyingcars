# flyingcars

---ADDING CRAZYFLIES TO THE CONFIG FILE---

Crazyflie Parameter are stored in Crazyflie_wheels/src/cf_params.csv. A new flie's parameters should be added according to the header:

name,uri,roll_trim,pitch_trim,tf_prefix

Add an empty ,, if you don't want to specify a parameter. Only name and uri need to be included.


=====AUTOMATIC=====

---LAUNCH SERVER & FLIES---

$ rosrun crazyflie_wheels start_flies.sh (name1))(name2) ... (nameN)

Automatically gives joystick ctrl to the first listed flie. 

---ADD ADDITIONAL FLIES---

$ rosrun crazyflie_wheels add_flies.sh (name1) (name2) ... (nameN)

---SWITCH JOYSTICK CONTROL---

$ rosrun crazyflie_wheels switch_ctrl.sh (name)

You can Ctrl-C to stop a specific flie or start a new terminal and run the command again, it will kill the previous instance. 

=====MANUAL=====

---LAUNCH A CRAZYFLIE BY NAME---

Use:

$ roslaunch crazyflie_wheels crazyflie_server.launch

and then: 

$ rosrun crazyflie_wheels launch_by_name.sh (name)

for each crazyflie you want to add. 

---GET TELEOP CONTROL/SWITCH CRAZYFLIE---

$ rosrun joy joy_node /dev/input/js0

to start joystick control.

$ rosrun topic_tools mux /(name)/joy /joy

to switch to crazyflie with (name). Ctrl-C to stop controlling that flie.

---TO DO---
-add easy removal of flies?
-make RViz/RQT_plot autostart for individual Crazyflies. 
