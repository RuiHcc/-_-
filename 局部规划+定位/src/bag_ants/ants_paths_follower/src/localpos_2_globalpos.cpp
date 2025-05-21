// 接受定位发布的位置信息和识别发来的相对位置，发布追踪目标点的全局坐标
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

std::string antname;

nav_msgs::Path path;

double vehicleX, vehicleY, vehicleZ, vehicleYaw;
double goalX, goalY;
bool get_new_target = false;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x;
  vehicleY = odomIn->pose.pose.position.y;
  vehicleZ = odomIn->pose.pose.position.z;
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  get_new_target = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("NameSpace", antname);

  // 订阅来自WLF的相对位置估计
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/yolov5_track_location", 5, goalHandler);
  // 订阅来自定位系统的自身位置估计
  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("state_estimation", 5, odomHandler);
  // 发布追踪目标点的全局坐标
  ros::Publisher pubGlobalPos = nh.advertise<geometry_msgs::PointStamped> ("way_point", 5);

  ros::Rate rate(100);
  bool status = ros::ok();
  double last_pub_t_ = ros::Time::now().toSec();
  while (status) {
    ros::spinOnce();

    // 计算目标点的全局位置
    geometry_msgs::PointStamped goal_global;
    goal_global.header.stamp = ros::Time::now();
    goal_global.header.frame_id = "map";
    goal_global.point.x = vehicleX + goalX * cos(vehicleYaw) - goalY * sin(vehicleYaw);
    goal_global.point.y = vehicleY + goalX * sin(vehicleYaw) + goalY * cos(vehicleYaw);
    goal_global.point.z = vehicleZ;
if(get_new_target)
    {pubGlobalPos.publish(goal_global);

get_new_target = false;}

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

