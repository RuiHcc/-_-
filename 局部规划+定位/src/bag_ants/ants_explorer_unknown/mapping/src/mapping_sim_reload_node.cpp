// This ROS node is for assigning the timestamp of the pointcloud.
// As the pointcloud received from Windows Unity can not add the corresponding timestamp for Ubuntu

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher lidar_pc_pub_;
ros::Subscriber lidar_pc_sub_;


void lidar_pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msgPtr) {
  sensor_msgs::PointCloud2 msg_new = *msgPtr;
  msg_new.header.stamp = ros::Time::now();
  lidar_pc_pub_.publish(msg_new);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_vis");
  ros::NodeHandle nh("~");

  lidar_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/lidar/pcl", 1, lidar_pc_callback);

  lidar_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pcl_out", 1);

  ros::spin();
  return 0;
}
