#!/bin/sh
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch my_robot world.launch " &
sleep 3
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch my_robot gmapping.launch" &
sleep 3
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch my_robot teleop.launch"
