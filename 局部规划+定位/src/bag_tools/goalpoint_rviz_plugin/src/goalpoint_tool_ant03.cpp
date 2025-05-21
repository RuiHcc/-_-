#include "goalpoint_tool3.h"

namespace rviz
{
GoalPointTool3::GoalPointTool3()
{
  shortcut_key_ = 'w';

  topic_property_ = new StringProperty("Topic", "goalpoint", "The topic on which to publish goal waypiont for dynamic route planner.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void GoalPointTool3::onInitialize()
{
  PoseTool::onInitialize();
  setName("Goalpoint03");
  updateTopic();
  vehicle_z = 0;
}

void GoalPointTool3::updateTopic()
{
  sub_ = nh_.subscribe<nav_msgs::Odometry> ("/ant03/state_estimation", 5, &GoalPointTool3::odomHandler, this);
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/ant03/goal_point", 5);
  // pub_joy_ = nh_.advertise<sensor_msgs::Joy>("joy", 5);
}

void GoalPointTool3::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void GoalPointTool3::onPoseSet(double x, double y, double theta)
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
  // joy.header.frame_id = "goalpoint_tool";
  
  geometry_msgs::PointStamped goal_point;
  goal_point.header.frame_id = "map";
  goal_point.header.stamp = ros::Time::now();
  goal_point.point.x = x;
  goal_point.point.y = y;
  goal_point.point.z = vehicle_z;

  pub_.publish(goal_point);
  usleep(10000);
  pub_.publish(goal_point);
  
  // usleep(10000); // set to 1000000us (1s) on real robot
  // pub_joy_.publish(joy);
}
}  // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::GoalPointTool3, rviz::Tool)
