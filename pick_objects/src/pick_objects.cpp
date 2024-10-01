#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Initialize flags for pick and drop
  bool pick = false;
  bool drop = false;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  if (!pick)
  {
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -5.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
  
    // Wait an infinite time for the results
    ac.waitForResult();
  
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, reached pick up location!");
      pick = true;
      
      // Wait at pick up for 5 seconds
      ros::Duration(5.0).sleep();
    }
    else
    {
      ROS_INFO("Failed to reach pick up location!");
    }
    drop = false;
  }

  if (!drop)
  {
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = 5.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
  
    // Wait an infinite time for the results
    ac.waitForResult();
  
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, reached drop location!");
      drop = true;
    }
    else
    {
      ROS_INFO("Failed to reach drop location!");
    }
    pick = false;
  }
  

  return 0;
}
