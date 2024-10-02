#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Pick up and drop locations from navigation goal package
// Refer pick_objects/src/pick_objects.cpp
float pick_x = 2.0, pick_y = 0.0, pick_w = 1.0;
float drop_x = 0.0, drop_y = 7.0, drop_w = 1.0;

// Location flags
bool pick = false, drop = false;

// Cube dimension and tolerance
float side = 0.2;
float tol = 0.1 * side;

void checkRoboPose(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Set location flags as per odom
  if ((abs(pick_x - msg->pose.pose.position.x) < tol) && (abs(pick_y - msg->pose.pose.position.y) < tol) && (abs(pick_w - msg->pose.pose.orientation.w) < 0.02))
  {
    pick = true;
    ROS_INFO("Robot at pick up location");
  }
  
  if ((abs(drop_x - msg->pose.pose.position.x) < tol) && (abs(drop_y - msg->pose.pose.position.y) < tol) && (abs(pick_w - msg->pose.pose.orientation.w) < 0.02))
  {
    drop = true;
    ROS_INFO("Robot at drop location");
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Duration(1).sleep();
  ros::Subscriber marker_sub = n.subscribe("odom", 100, checkRoboPose);
  ros::Duration(1).sleep();

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    
    // Publish marker at pick up location
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pick_x;
    marker.pose.position.y = pick_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pick_w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = side;
    marker.scale.y = side;
    marker.scale.z = side;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish marker
    marker_pub.publish(marker);

    // Wait for Robot to reach pick up location
    while (!pick)
    {
      ros::spinOnce();
    }

    // Remove marker after robot reaches pick up location
    
    // Wait for 5 seconds
    ros::Duration(5).sleep();
      
    // Delete marker
    marker.action = visualization_msgs::Marker::DELETE;
    ROS_INFO("Pick up done!");

    // Wait for Robot to reach drop location
    while (!drop)
    {
      ros::spinOnce();
    } 

    // Publish marker at drop location when robot reaches there
    // Add marker
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = drop_x;
    marker.pose.position.y = drop_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = drop_w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = side;
    marker.scale.y = side;
    marker.scale.z = side;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish marker
    marker_pub.publish(marker);
    ROS_INFO("Drop done!");
      
    // Wait for 5 seconds
    ros::Duration(5).sleep();
    
    // ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
