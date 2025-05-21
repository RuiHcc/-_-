#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <algorithm>
#include <vector>
#include <random>
#include <time.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
// #include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


#include "CorridorBuilder.h"
#include "tic_toc.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "map");
  ros::NodeHandle nh("~");

  float radius;
  float precision;
  bool starornot;
  nh.getParam("radius", radius);
  nh.getParam("precision", precision); // precision of quick hull algorithm
  nh.getParam("starconvex", starornot);

  std::cout << radius << std::endl;

  float bound = 50;
  int num = 10000;
  std::default_random_engine random(time(NULL));
  std::uniform_real_distribution<double> r(-bound, bound);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  int loop_n = 0;
  while (loop_n < num) {
      float ax = r(random);
      float ay = r(random);
      float az = r(random);
      float fx1 = bound/4.0;
      float fy1 = bound/4.0;
      float fy2 = bound/4.0;
      float fz2 = bound/4.0;
      if (ax < fx1 && ax > - fx1 && ay < fy1 && ay > - fy1) continue;
      if (ay < fy2 && ay > - fy2 && az < fz2 && az > - fz2) continue;
      cloud->points.push_back(pcl::PointXYZ(ax,ay,az));
      loop_n++;
  }

  ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_map",100);
  ros::Publisher polyhedron_pub = nh.advertise<visualization_msgs::Marker> ("polyhedron",100);
  TicToc t;
  t.tic();

  FastLab::Corridor builder(cloud, radius, precision);
  std::vector<FastLab::Triangle> meshes; // triangle meshes of convex hull
  std::vector<Eigen::Vector3d> v; // vertexes of convex hull
  pcl::PointXYZ p(0,0,0); // where to generate convex hull
  builder.build(meshes, v, p, starornot);
  std::cout << t.toc() << " ms" << " " << cloud->points.size() << std::endl;

  sensor_msgs::PointCloud2 show_cloud;
  pcl::toROSMsg(*cloud, show_cloud);
  show_cloud.header.frame_id = "map";

  visualization_msgs::Marker component;
  component.header.frame_id = "map";
  component.header.stamp = ros::Time::now();
  component.ns = "";
  component.lifetime = ros::Duration();
  component.frame_locked = true;
  component.type = visualization_msgs::Marker::TRIANGLE_LIST;
  component.action = visualization_msgs::Marker::ADD;
  component.color.r = 1.0f;
  component.color.g = 0.0f;
  component.color.b = 0.0f;
  component.color.a = 0.4f;
  component.scale.x = 1.0;
  component.scale.y = 1.0;
  component.scale.z = 1.0;
  component.pose.position.x = 0;
  component.pose.position.y = 0;
  component.pose.position.z = 0;
  component.pose.orientation.x = 0.0;
  component.pose.orientation.y = 0.0;
  component.pose.orientation.z = 0.0;
  component.pose.orientation.w = 1.0;
  for (unsigned int i=0; i<meshes.size(); i++) {
      geometry_msgs::Point p1;
      p1.x = meshes[i]._a.x;
      p1.y = meshes[i]._a.y;;
      p1.z = meshes[i]._a.z;
      component.points.push_back(p1);
      geometry_msgs::Point p2;
      p2.x = meshes[i]._b.x;
      p2.y = meshes[i]._b.y;
      p2.z = meshes[i]._b.z;
      component.points.push_back(p2);
      geometry_msgs::Point p3;
      p3.x = meshes[i]._c.x;
      p3.y = meshes[i]._c.y;
      p3.z = meshes[i]._c.z;
      component.points.push_back(p3);
  }

  polyhedron_pub.publish(component);
  pointcloud_pub.publish(show_cloud);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    // ROS_INFO("running...");
    pointcloud_pub.publish(show_cloud);
    polyhedron_pub.publish(component);
    loop_rate.sleep();
  }
  return 0;
}

