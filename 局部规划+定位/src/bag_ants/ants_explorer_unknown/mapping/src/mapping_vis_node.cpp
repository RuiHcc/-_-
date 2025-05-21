/*
 * @Author: Chenyang Zhang && 1727326672@qq.com
 * @Date: 2024-02-28 16:22:53
 * @LastEditors: Chenyang Zhang && 1727326672@qq.com
 * @LastEditTime: 2024-03-17 13:35:42
 * @FilePath: /bit_zcy_001/src/bag_ants/ants_explorer_unknown/mapping/src/mapping_vis_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <mapping/mapping.h>
// #include <quadrotor_msgs/OccMap3d.h>
#include <msg_utils/OccMap3d.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher gridmap_vs_pub, gridmap_inflate_vs_pub;

double z_max_;

pcl::PointCloud<pcl::PointXYZ>::Ptr pcd;

void gridmap_callback(const msg_utils::OccMap3dConstPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  gridmap.from_msg(*msgPtr);
  sensor_msgs::PointCloud2 pc;
  gridmap.occ2pc(pcd);
  pcl::toROSMsg(*pcd, pc);
  pc.header.frame_id = "world";
  gridmap_vs_pub.publish(pc);
}

void gridmap_inflate_callback(const msg_utils::OccMap3dConstPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  gridmap.from_msg(*msgPtr);
  sensor_msgs::PointCloud2 pc;
  gridmap.occ2pc_z_cut(pc, z_max_);
  pc.header.frame_id = "world";
  gridmap_inflate_vs_pub.publish(pc);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_vis");
  ros::NodeHandle nh("~");
  nh.param("z_max", z_max_, 5.0);

  pcd.reset(new pcl::PointCloud<pcl::PointXYZ>);

  gridmap_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap", 1);
  gridmap_inflate_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap_inflate", 1);
  ros::Subscriber gridmap_sub = nh.subscribe<msg_utils::OccMap3d>("gridmap", 1, gridmap_callback); // has pub in mapping_nodelet.cpp
  ros::Subscriber gridmap_inflate_sub = nh.subscribe<msg_utils::OccMap3d>("gridmap_inflate", 1, gridmap_inflate_callback);

  ros::spin();
  return 0;
}
