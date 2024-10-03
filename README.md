# Home Service Robot
The purpose of this repository is to program a home service robot that will autonomously map an environment and navigate to pickup and deliver objects. 

The steps are listed as [summary of tasks](task_summary.txt).

<img src="HomeServiceRobot.gif"/>

## Description
Inside the [Gazebo world](https://github.com/sidharth2189/RoboND-GazeboWorld/blob/main/images/office.png) one can identify:

* Two wheeled Robot with caster.
* Sensors (lidar and camera) mounted on the robot.

## Getting Started

### Directory structure
    .HomeServiceRobot                       # Robot SLAM and navigation
    ├── my_robot                            # my_robot package                   
    │   ├── launch                          # launch files   
    │   │   ├── robot_description.launch    # Generate urdf from xacro
    │   │   ├── world.launch                # launch Gazebo world along with robot
    │   │   ├── amcl.launch                 # launch robot localization using amcl
    │   │   ├── mapping.launch              # launch mapping node using rtabmap
    │   │   ├── localization.launch         # launch mapping with localization
    │   │   ├── teleop.launch               # launch teleop_twist_keyboard to move robot
    │   │   ├── gmapping.launch             # launch ROS gmapping for robot    
    │   ├── meshes                          # meshes folder for sensors
    │   │   ├── hokuyo.dae                  # Hokuyo lidar sensor
    │   ├── urdf                            # urdf folder for xarco files
    │   │   ├── my_robot.gazebo             # Plugin for sensor/actuator (RGBD camera/Hokuyo lidar/Differential drive)
    │   │   ├── my_robot.xacro              # Robot description
    │   ├── world                           # world folder for world files
    │   │   ├── office.world
    │   ├── CMakeLists.txt                  # compiler instructions
    │   ├── package.xml                     # package info
    │   ├── config                          # parmater for robot's navigational goal   
    │   │   ├── costmap_common_params.yaml  # rosparam for move_base package
    │   │   ├── local_costmap_params.yaml   # rosparam for move_base package
    │   │   ├── global_costmap_params.yaml  # rosparam for move_base package
    │   │   ├── base_costmap_params.yaml    # rosparam for move_base package
    │   ├── maps                            # map storage   
    │   │   ├── map.pgm                     # map generated from pgm_map_creator package
    │   │   ├── map.yaml                    # map metadata
    │   │   ├── slam_map.yaml               # map generated from SLAM testing using shell_script/test_SLAM.sh
    │   ├── rtabmap                         # database generated from mapping
    │   │   ├── rtabmap.db                  # database file
    │   ├── rviz                            # saved rviz config files
    │   │   ├── marker.rviz                 # view_navigation from turtlebot_rviz_launchers with marker added
    ├── pgm_map_creator                     # map creator package (submodule)    
    ├── teleop_twist_keyboard               # package to control robot motion through keyboard (submodule)
    ├── turtlebot_interactions              # turtlebot_rviz_launchers package for preconfigured rviz workspace
    ├── pick_objects                        # Navigational goal node
    │   ├── launch                          # launch files   
    │   │   ├── pick_objects.launch         # launch file for pick_objects node
    │   ├── src                             # source files   
    │   │   ├── pick_objects.cpp            # node for navigational goal
    ├── add_markers                         # Model virtual objects
    │   ├── launch                          # launch files   
    │   │   ├── add_markers.launch          # launch file for visualizing marker in home service robot
    │   │   ├── add_markers_test.launch     # launch file for testing marker visualization
    │   ├── src                             # source files   
    │   │   ├── add_markers.cpp             # node for visualizing marker in home service robot
    │   │   ├── add_markers_test.cpp        # node for testing marker visualization
    ├── amcl.rviz                           # visualization file for localization using amcl
    ├── shell_scripts                       # store various shell scripts
    │   ├── launch.sh                       # demo launch file for gazebo, rosmaster and rviz
    │   ├── pick_objects.sh                 # script to launch robot, amcl, rviz with navigational goal
    │   ├── test_navigation.sh              # script for localization and navigation testing
    │   ├── test_slam.sh                    # script for slam testing (gmapping)  
    │   ├── add_marker.sh                   # script to test marker visualization (robot, amcl, rviz and add_markers)
    │   ├── home_service.sh                 # script for home service robot (robot, rviz config, pick_objects and add_markers)                           
    └──                          

### Node/Topic view

* Node/Topic view for SLAM and Navigation for [my_robot](/my_robot/)
<img src="/docs/HomeServRobo_Nodes.png"/>
<img src="/docs/HomeServRobo_Nodes_Topics.png"/>

* An overview of the packages used in this project is summarized [here](/docs/Packages%20in%20the%20Home%20Service%20Robot.pdf).

### Dependencies

* Operating System — Ubuntu 16.04 LTS. ([Udacity VM Image](https://s3-us-west-1.amazonaws.com/udacity-robotics/Virtual+Machines/Lubuntu_071917/RoboVM_V2.1.0.zip))
  *  Please refer steps for usage of VM, resource allocation and first boot [here](/docs/VM.txt).
  * Alternatively, for configuring personal set up, [here](https://wiki.ros.org/kinetic/Installation/Ubuntu) is the Ubuntu installation steps for ROS Kinetic.

* [gmapping ros](https://wiki.ros.org/gmapping)
  * Please check the "Show EOL distros" and check the [gmapping github](https://github.com/ros-perception/slam_gmapping) branch appropriate for your version of ROS.
  * Alternatively, if the package is not installed for above VM (ros kinetic), please install as below.
  ```sudo apt install ros-kinetic-slam-gmapping```</brk> or ```sudo apt-get install ros-kinetic-gmapping```

* [amcl](https://github.com/sidharth2189/RoboND-WhereAmI)


### Installing
* To verify installation, run
```
gazebo
```

### How to run
* Update and upgrade the Workspace
```
sudo apt-get update && sudo apt-get upgrade -y
```
* Create a [catkin workspace](https://wiki.ros.org/catkin/conceptual_overview)
```
$ mkdir -p ~/catkin_ws/src
```
* Navigate to source directory
```
$ cd ~/catkin_ws/src
```
* Initialize the catkin workspace which will create a ```CMakeLists.txt``` file.
```
catkin_init_workspace
```
* Clone this repository and its submodules.
```
git clone https://github.com/sidharth2189/RoboND-HomeServiceRobot.git
```
```
git submodule update --init --recursive
```
* Copy [```my_robot```](/my_robot/), [```teleop_twist_keyboard```](https://github.com/ros-teleop/teleop_twist_keyboard), [```turtlebot_interactions```](https://github.com/turtlebot/turtlebot_interactions), [```pick_objects```](/pick_objects/) and[```add_markers```](/add_markers/) packages into the source folder for catkin workspace, ```/catkin_ws/src```.
* Navigate to catkin workspace.
```
cd ~/catkin_ws/
```
* Build packages.Note that the command is issued from within the top level directory (i.e., within ```catkin_ws``` NOT ```catkin_ws/src```) 
```
catkin_make
```
* Source the set up script of the workspace. 
```
source devel/setup.bash
```
* To check for missing package and install dependency if not done.
```
rosdep check <package name>
```
```
rosdep -i install <package name>
```
* Turn shell script to executable.
```
chmod +x home_service.sh
```
* Launch script.
```
./home_service.sh
```
* Note that to run the script ```./test_Navigation.sh```, the [dependency](#dependencies) for gmapping needs to be followed.

## Improvements
Some improvements to the code base can be considered as provided by project reviwer.

* Instead of hard-coding goal poses in the [```pick_objects/src/pick_objects.cpp```](/pick_objects/src/pick_objects.cpp), one can load goal locations as ```<rosparam>``` through launch file and send pick up and drop locations to [```pick_objects```](/pick_objects/) node. Use the ```<rosparam>``` tag to load the parameter to ROS master.

* In [```pick_objects/src/pick_objects.cpp```](/pick_objects/src/pick_objects.cpp), the move_base action server is coded to send goals in sequential order inside the main function. This is good for beginners. But in the longer run, a better approach will be to use functions to do these tasks. 
  * For example, one can use a function called ```send_move_base_goal(x, y)``` which will call the movebase action server. This will make the code more readable and reusable in the longer run. 
  * Also function can be called with different (x, y) values without having to duplicate the code.

* Use 2D gmapping [map](/my_robot/maps/slam_map.pgm) instead of [map](/my_robot/maps/map.pgm) from [pgm_map_creator](/https://github.com/hyfan1116/pgm_map_creator).
  * Refer [understanding gmapping and how to use the package](https://www.youtube.com/watch?v=tUm6MRGYam8)
  * Basic navigation tuning [guide](https://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide).

## Useful links
* [Duplicating a repository](https://docs.github.com/en/repositories/creating-and-managing-repositories/duplicating-a-repository)
* [Robot reference](https://github.com/sidharth2189/RoboND-WhereAmI)
* [Navigation tuning guide](https://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
* [Knowledge discussion](https://knowledge.udacity.com/questions/418425#419880) on amcl and base planner tuning.
* [Configuration space](https://www.youtube.com/watch?v=SBFwgR4K1Gk)
* [Minkowski addition](https://en.wikipedia.org/wiki/Minkowski_addition)
* [Voronoi diagram](https://en.wikipedia.org/wiki/Voronoi_diagram)
* [State value function](https://towardsdatascience.com/reinforcement-learning-rl-101-with-python-e1aa0d37d43b) in Markov decision process for probabilistic path planning
* [Uniform cost search algorithm](https://www.geeksforgeeks.org/uniform-cost-search-dijkstra-for-large-graphs/)
* [Visualize discrete path planners](https://qiao.github.io/PathFinding.js/visual/)
* [Types of graphs](https://www.geeksforgeeks.org/graph-types-and-applications/)
* Comparative [study](https://webspace.science.uu.nl/~gerae101/pdf/compare.pdf) for Probabilistic roadmap planners.
* [Path planning](https://www.cs.cmu.edu/~maxim/files/pathplanforMAV_icra13.pdf) for non-circular Micro Aerial Vehicles in constrained environment
* [Turtlebot rviz launchers](https://wiki.ros.org/turtlebot_rviz_launchers) - a package for preconfigured rviz workspace.
* [Tutorial](https://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) for sending simple navigational goals.
* [Turorial](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes) for modeling virtual objects with markers in rviz.
* [Handle ROS param through YAML](https://roboticsbackend.com/ros-param-yaml-format/). (For example instead of hard-coding goal poses in the code, one can load goal locations as ```<rosparam>``` through launch file and send pick up and drop locations to [```pick_objects```](/pick_objects/) node. Use the ```<rosparam>``` tag to load the parameter to ROS master)
* RGBD camera [openni_kinect](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#OpenniKinect) &  Laser [Hokuyo](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Laser)
* [Tune the navigation parameters](https://www.youtube.com/watch?v=s0JA9jjZi44) for better performance and understanding of the navigation software stack.
* [README formatting](https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax)
* [How various parameters of the mapping code can be tuned to improve the map accuracy](https://ieeexplore.ieee.org/document/7847825).
* [How](https://www.semanticscholar.org/paper/Design-of-an-autonomous-mobile-robot-based-on-ROS-K%C3%B6seo%C4%9Flu-%C3%87eli%CC%87k/9dd90f7b746657fe077f8cf1ea56f6a8d65ce21c#extracted) to build a real robot including how to select hardware and also a brief note on the communication protocol that can be used on a real robot.

