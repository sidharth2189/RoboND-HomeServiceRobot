#!/bin/sh
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 3
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 3
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
xterm -e " source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
