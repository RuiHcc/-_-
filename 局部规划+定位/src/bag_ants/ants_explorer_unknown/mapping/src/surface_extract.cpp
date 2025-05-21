#include <mapping/mapping.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <iostream>

namespace mapping {

void OccGridMap::ExtractSurface(Eigen::Vector3d laser_p, pcl::PointCloud<pcl::PointXYZ>::Ptr pco, pcl::PointCloud<pcl::PointXYZ>::Ptr pout){
  // ROS_WARN_STREAM("ExtractSurface------------------------------");
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr extractor_kdtree_;
  extractor_kdtree_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  pout->clear();
  extractor_kdtree_->setInputCloud(pco);
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sqdist;
  // ROS_WARN_STREAM("pco_size: "<<pco->points.size());
  // ROS_WARN_STREAM("extract_radius_threshold: "<<extract_radius_threshold);


  for (int i = 0; i < pco->points.size(); i++)
  {
    pcl::PointXYZ point = pco->points[i];
    Eigen::Vector3d pt(point.x, point.y, point.z);
    Eigen::Vector3d up_p(point.x, point.y, point.z + resolution);


    if(isOccupied(up_p)) continue;
    if(point.z > laser_p(2)) continue;


    extractor_kdtree_->radiusSearch(point, extract_radius_threshold, neighbor_indices, neighbor_sqdist);
    bool is_surface = false;
    int neighbor_count = 0;
    // if(neighbor_indices.size() > 0){
    //   ROS_WARN_STREAM("up_p: "<<up_p.transpose());
    // }
    for (const auto& idx : neighbor_indices)
    {
      double z_diff = std::abs(point.z - pco->points[idx].z);
      // ROS_WARN_STREAM("z_diff: "<<z_diff);

      if (z_diff < z_diff_max_thr)
      {
        neighbor_count++;
        if (neighbor_count >= surface_neighbor_thr)
        {
          is_surface = true;
          break;
        }
      }

    }
    if (is_surface)
    {
      pcl::PointXYZ point_out;
      point_out.x = pco->points[i].x;
      point_out.y = pco->points[i].y;
      point_out.z = pco->points[i].z;
      pout->points.push_back(point_out);
    }
  }
  pout->width = pout->points.size();
  pout->height = 1;
  pout->is_dense = true;
}



}  // namespace mapping