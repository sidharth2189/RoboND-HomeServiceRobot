# Home Service Robot
The purpose of this repository is to program a home service robot that will autonomously map an environment and navigate to pickup and deliver objects. 

The steps are listed as [summary of tasks](task_summary.txt).

## Description
Inside the [Gazebo world](https://github.com/sidharth2189/RoboND-GazeboWorld/blob/main/images/office.png) one can identify:

* Two wheeled Robot with caster.
* Sensors (lidar and camera) mounted on the robot.

## Getting Started

### Directory structure
    .HomeServiceRobot                       # Robot SLAM and navigation
    ├── my_robot                            # my_robot package                   
    │   ├── launch                          # launch folder for launch files   
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
    │   ├── rtabmap                         # database generated from mapping
    │   │   ├── rtabmap.db                  # database file
    ├── pgm_map_creator                     # map creator package (submodule)    
    ├── teleop_twist_keyboard               # package to control robot motion through keyboard (submodule)
    ├── turtlebot_interactions              # turtlebot_rviz_launchers package for preconfigured rviz workspace
    ├── pick_objects                        # Navigational goal node
    ├── amcl.rviz                           # visualization file for localization using amcl
    ├── shell_scripts                       # store various shell scripts
    │   ├── launch.sh                       # sample launch for gazebo, rosmaster and rviz
    │   ├── pick_objects.sh                 # script to launch robot, amcl, rviz and pick_objects
    │   ├── test_navigation.sh              # script for localization and navigation testing
    │   ├── test_slam.sh                    # script for slam testing (gmapping)                            
    └──                          

### Node view

Node view for SLAM and Navigation for [my_robot](/my_robot/)
<img src="/docs/HomeServiceRobo.png"/>

### Dependencies

* Operating System — Ubuntu 16.04 LTS. ([Udacity VM Image](https://s3-us-west-1.amazonaws.com/udacity-robotics/Virtual+Machines/Lubuntu_071917/RoboVM_V2.1.0.zip))
  *  Please refer steps for usage of VM, resource allocation and first boot [here](/docs/VM.txt).
  * Alternatively, for configuring personal set up, [here](https://wiki.ros.org/kinetic/Installation/Ubuntu) is the Ubuntu installation steps for ROS Kinetic.

* [gmapping ros](https://wiki.ros.org/gmapping)
  * Please check the "Show EOL distros" and check the [gmapping github](https://github.com/ros-perception/slam_gmapping) branch appropriate for your version of ROS.
  * Alternatively, if the package is not installed for above VM (ros kinetic), please install as below.
  ```sudo apt install ros-kinetic-slam-gmapping```</brk> or ```sudo apt-get install ros-kinetic-gmapping```

* [amcl](https://github.com/sidharth2189/RoboND-WhereAmI)

* Git LFS: The generated database is [stored](/my_robot/rtabmap/rtabmap.db) as an LFS file due to its size. Please [install git LFS](https://github.com/git-lfs/git-lfs/blob/main/INSTALLING.md) on system to be able to not have issues fetching this file.


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
* Copy [```my_robot```](/my_robot/), [```teleop_twist_keyboard```](https://github.com/ros-teleop/teleop_twist_keyboard), [```turtlebot_interactions```](https://github.com/turtlebot/turtlebot_interactions) and [pick_objects](/pick_objects/) packages into the source folder for catkin workspace, ```/catkin_ws/src```.
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
chmod +x pick_objects.sh
```
* Launch script.
```
./pick_objects.sh
```
* Note that to run the script ```./test_Navigation.sh, the [dependency](#dependencies) for gmapping needs to be followed.

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
* [Turorial](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes) for modeling virtual objects with markers in rviz.
