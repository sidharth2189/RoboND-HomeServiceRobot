#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Pick up and drop locations from navigation goal package
// Refer pick_objects/src/pick_objects.cpp
double pick_x = 2.0, pick_y = 0.0, pick_w = 1.0;
double drop_x = 0.0, drop_y = 7.0, drop_w = 1.0;

// Location flags
bool pick = false, drop = false;

// Cube dimension and tolerance
double side = 0.4;
double tol = 0.2;

double euclideanDist(double x1, double y1, double x2, double y2)
{
  return (sqrt(pow((x1-x2), 2.0) + pow((y1-y2), 2.0)));
}

void checkRoboPose(const nav_msgs::Odometry::ConstPtr& msg)
{
  double transform_y = (-1)*msg->pose.pose.position.x;
  double transform_x = msg->pose.pose.position.y;

  // Set location flags as per odom
  if (euclideanDist(pick_x, pick_y, transform_x, transform_y) < tol)
  {
    pick = true;
    ROS_INFO("Robot at pick up location");
  }
  else
  {
    pick = false;
  }
  
  if (euclideanDist(drop_x, drop_y, transform_x, transform_y) < tol)
  {
    drop = true;
    ROS_INFO("Robot at drop location");
  }
  else
  {
    drop = false;
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
    ros::Duration(1).sleep();
    ROS_INFO("Reached pick up zone!");

    // Remove marker after robot reaches pick up location
    
    // Wait for 5 seconds
    ros::Duration(5).sleep();
      
    // Delete marker and publsh the deletion
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ROS_INFO("Pick up done!");

    // Wait for Robot to reach drop location
    while (!drop)
    {
      ros::spinOnce();
    }
    ros::Duration(1).sleep();
    ROS_INFO("Reached drop zone!"); 

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
    // r.sleep();
    return 0;
  }
}
