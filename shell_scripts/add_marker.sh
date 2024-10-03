#!/bin/sh
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch my_robot world.launch " &
sleep 3
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 3
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch add_markers add_markers.launch"
