/*
 * @Author: Chenyang Zhang && 1727326672@qq.com
 * @Date: 2024-03-25 09:32:49
 * @LastEditors: Chenyang Zhang && 1727326672@qq.com
 * @LastEditTime: 2024-03-25 09:33:13
 * @FilePath: /bit_zcy_001/src/bag_ants/ants_coordinate/task_allocation/src/task_allocation_test.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <task_allocation/task_allocation.h>
#include <ros/ros.h>

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_allocation_node");
  ros::NodeHandle nh("~");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}