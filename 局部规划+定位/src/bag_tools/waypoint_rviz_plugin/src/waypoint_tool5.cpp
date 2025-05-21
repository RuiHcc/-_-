#include "waypoint_tool5.h"

namespace rviz
{
WaypointTool5::WaypointTool5()
{
  shortcut_key_ = 'w';

  topic_property_ = new StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void WaypointTool5::onInitialize()
{
  PoseTool::onInitialize();
  setName("Waypoint05");
  updateTopic();
  vehicle_z = 0;
}

void WaypointTool5::updateTopic()
{
  sub_ = nh_.subscribe<nav_msgs::Odometry> ("/ant05/state_estimation", 5, &WaypointTool5::odomHandler, this);
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/ant05/way_point", 5);
  // pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
}

void WaypointTool5::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void WaypointTool5::onPoseSet(double x, double y, double theta)
{
  // sensor_msgs::Joy joy;

  // joy.axes.push_back(0);
  // joy.axes.push_back(0);
  // joy.axes.push_back(-1.0);
  // joy.axes.push_back(0);
  // joy.axes.push_back(1.0);
  // joy.axes.push_back(1.0);
  // joy.axes.push_back(0);
  // joy.axes.push_back(0);

  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(1);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);
  // joy.buttons.push_back(0);

  // joy.header.stamp = ros::Time::now();
  // joy.header.frame_id = "waypoint_tool";
  // pub_joy_.publish(joy);

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = vehicle_z;

  pub_.publish(waypoint);
  usleep(10000);
  pub_.publish(waypoint);
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool5, rviz::Tool)
