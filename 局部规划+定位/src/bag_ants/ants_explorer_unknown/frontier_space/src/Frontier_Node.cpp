/*
 * @FilePath: /bit_zcy_001/src/bag_ants/ants_explorer_unknown/frontier_space/src/Frontier_Node.cpp
 * @Brief:
 * @Version: 1.0
 * @Date: 2020-11-26 17:08:58
 * @Author: your name
 * @Copyright: your copyright description
 * @LastEditors: Chenyang Zhang && 1727326672@qq.com
 * @LastEditTime: 2024-03-17 17:45:56
 */

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// #include "explorer_point_finder2.h"
#include "explorer_point_finder.h"
#include <msg_utils/OccMap3d.h>

ros::Subscriber gridmap_sub_, gridmap_inf_sub_;
std::shared_ptr<mapping::OccGridMap> gridmapPtr_, gridmap_noinf_Ptr_;
std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;
std::atomic_flag gridmap_inf_lock_ = ATOMIC_FLAG_INIT;


void gridmap_inf_callback(const msg_utils::OccMap3dConstPtr& msgPtr) {
    // ROS_ERROR("receiving map");
    while (gridmap_inf_lock_.test_and_set())
        ;
    gridmapPtr_->from_msg(*msgPtr);
    gridmapPtr_->recieve_cnt++;
    gridmap_inf_lock_.clear();
}

void gridmap_callback(const msg_utils::OccMap3dConstPtr& msgPtr) {
    while (gridmap_lock_.test_and_set())
        ;
    // ROS_ERROR("gridmap_callback");
    gridmap_noinf_Ptr_->from_msg(*msgPtr);
    gridmap_noinf_Ptr_->recieve_cnt++;
    gridmap_lock_.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_node");
    ros::NodeHandle nh("~");

    int robot_id;
    nh.param("client/robot_id", robot_id, -1);
    
    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    gridmap_noinf_Ptr_ = std::make_shared<mapping::OccGridMap>();
    
    Eigen::Vector3d map_size;
    double res;

    nh.getParam("grid_map/resolution", res);
    nh.getParam("grid_map/global_map_x", gridmapPtr_->global_map_size.x());
    nh.getParam("grid_map/global_map_y", gridmapPtr_->global_map_size.y());
    nh.getParam("grid_map/global_map_z", gridmapPtr_->global_map_size.z());
    nh.getParam("grid_map/global_map_origin_x", gridmapPtr_->global_map_origin.x());
    nh.getParam("grid_map/global_map_origin_y", gridmapPtr_->global_map_origin.y());
    nh.getParam("grid_map/global_map_origin_z", gridmapPtr_->global_map_origin.z());
    nh.getParam("grid_map/local_x", map_size.x());
    nh.getParam("grid_map/local_y", map_size.y());
    nh.getParam("grid_map/local_z", map_size.z());

    nh.getParam("is_on_car", gridmapPtr_->is_on_car);
    nh.getParam("grid_map/plan_roof", gridmapPtr_->plan_roof);
    nh.getParam("grid_map/plan_floor", gridmapPtr_->plan_floor);
    nh.getParam("grid_map/extract_surface_radius", gridmapPtr_->extract_radius_threshold);
    nh.getParam("grid_map/extract_surface_z_max_thr", gridmapPtr_->z_diff_max_thr);
    nh.getParam("grid_map/extract_surface_neighbor_thr", gridmapPtr_->surface_neighbor_thr);
    ROS_WARN_STREAM("extract_surface_radius" << gridmapPtr_->extract_radius_threshold);
    
    gridmapPtr_->setup(res, map_size);


    gridmap_noinf_Ptr_->global_map_size = gridmapPtr_->global_map_size;
    gridmap_noinf_Ptr_->global_map_origin = gridmapPtr_->global_map_origin;
    gridmap_noinf_Ptr_->is_on_car = gridmapPtr_->is_on_car;
    gridmap_noinf_Ptr_->plan_roof = gridmapPtr_->plan_roof;
    gridmap_noinf_Ptr_->plan_floor = gridmapPtr_->plan_floor;
    gridmap_noinf_Ptr_->extract_radius_threshold = gridmapPtr_->extract_radius_threshold;
    gridmap_noinf_Ptr_->z_diff_max_thr = gridmapPtr_->z_diff_max_thr;
    gridmap_noinf_Ptr_->surface_neighbor_thr = gridmapPtr_->surface_neighbor_thr;
    gridmap_noinf_Ptr_->setup(res, map_size);

    gridmap_inf_sub_ = nh.subscribe<msg_utils::OccMap3d>("gridmap_inflate", 1, &gridmap_inf_callback, ros::TransportHints().tcpNoDelay());
    gridmap_sub_     = nh.subscribe<msg_utils::OccMap3d>("gridmap", 1, &gridmap_callback, ros::TransportHints().tcpNoDelay());


    FrontierClusterInfo::Ptr fc_info;
    fc_info.reset(new FrontierClusterInfo(nh));
    fc_info->setMapServer(gridmap_noinf_Ptr_, gridmapPtr_);

    FrontierServer::Ptr frontier_server_ptr;

    frontier_server_ptr.reset(new FrontierServer(nh,  fc_info));

    ros::spin();

    return 0;
}
