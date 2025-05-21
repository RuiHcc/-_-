/*
 * @Author: Chenyang Zhang && 1727326672@qq.com
 * @Date: 2024-03-25 09:10:45
 * @LastEditors: Chenyang Zhang && 1727326672@qq.com
 * @LastEditTime: 2024-04-06 10:24:30
 * @FilePath: /bit_zcy_001/src/bag_ants/ants_coordinate/task_allocation/include/task_allocation/task_balance.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>



#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <task_allocation/DroneState.h>
#include <task_allocation/PairOpt.h>
#include <task_allocation/PairOptResponse.h>
#include <bspline/Bspline.h>

#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>

#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>
#include <active_perception/hgrid.h>

#include <traj_utils/planning_visualization.h>

#include <lkh_tsp_solver/SolveTSP.h>
#include <lkh_mtsp_solver/SolveMTSP.h>
#include <msg_utils/FreeSpaceAndFrontierInfo.h>
using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {


struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  vector<Eigen::Vector3d> start_poss;
  bspline::Bspline newest_traj_;

  // Swarm collision avoidance
  bool avoid_collision_, go_back_;
  ros::Time fsm_init_time_;
  ros::Time last_check_frontier_time_;

  Eigen::Vector3d start_pos_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second

  // Swarm
  double attempt_interval_;   // Min interval of opt attempt
  double pair_opt_interval_;  // Min interval of successful pair opt
  int repeat_send_num_;
};

struct DroneState {
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double yaw_;
  double stamp_;                // Stamp of pos,vel,yaw
  double recent_attempt_time_;  // Stamp of latest opt attempt with any drone

  vector<int> grid_ids_;         // Id of grid tour
  double recent_interact_time_;  // Stamp of latest opt with this drone
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<Vector3d> frontier_tour_;
  vector<vector<Vector3d>> other_tours_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_, kino_path_;
  Vector3d next_pos_;
  double next_yaw_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  // Swarm, other drones' state
  vector<DroneState> swarm_state_;
  vector<double> pair_opt_stamps_, pair_opt_res_stamps_;
  vector<int> ego_ids_, other_ids_;
  double pair_opt_stamp_;
  bool reallocated_, wait_response_;

  // Coverage planning
  vector<Vector3d> grid_tour_, grid_tour2_;
  // int prev_first_id_;
  vector<int> last_grid_ids_;

  int plan_num_;
};

struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_;   // resource dir of tsp solver
  string mtsp_dir_;  // resource dir of tsp solver
  double relax_time_;
  int init_plan_num_;

  // Swarm
  int drone_num_;
  int drone_id_;
};

class FastExplorationFSM {

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);
  void findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids, vector<int>& frontier2_ids);
  bool findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init = false);
  void findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices);
  void getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double>& yaws, const vector<int>& ftr_ids,
    const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat);
  void getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double> yaws, Eigen::MatrixXd& mat);
  void visualize(int content);
  void visualize(vector<vector<int>>& ego_ids);

  void allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
    vector<int>& other_ids);
  void allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<vector<int>>& ego_ids);
  void checkAllocateResult();
  
  EDTEnvironment::Ptr edt_environment_;
  shared_ptr<HGrid> hgrid_;
  shared_ptr<PlanningVisualization> visualization_;
  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  static double vm_, am_, yd_, ydd_, w_dir_;
  static shared_ptr<RayCaster> caster_;
  static shared_ptr<Astar> astar_;

  // list<Frontier> frontiers_;
  vector<list<Frontier>> frontiers_svp;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* helper functions */
  void updateFrontierStates();
  void updateFrontierStates(const msg_utils::FrontierInfo msg);
  double computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1,
    const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path);
  double searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path);
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points,
    vector<double>& yaws, vector<Eigen::Vector3d>& averages);

  
  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  // void VISCallback(const ros::TimerEvent& e);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mcarOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mcar_FreeSpaceAndFrontierInfoCallback(const msg_utils::FreeSpaceAndFrontierInfoConstPtr msg);
  double computeGridPathCost(const Eigen::Vector3d& pos,
    const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const double& w_f);

  // Swarm

  /* planning utils */
  Vector3d odom_pos_, odom_vel_;
  Eigen::Quaterniond odom_orient_;
  double  odom_yaw_;
  double min_candidate_dist_;
  vector<vector<int>> last_ego_ids;
  double prev_app1;
  ros::Time stop_time;

  
  

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_;
  // ros::Timer vis_timer_;
  ros::Subscriber odom_sub_;

  ros::Publisher grid_tour2_pub;
  // vis
  ros::Publisher vis_pub_;
  
  ros::ServiceClient tsp_client_, acvrp_client_;
  ros::Subscriber mcar_FreeSpaceAndFrontierInfo_sub;
  ros::Publisher mcar_task_allocation_pub;

  std::vector<ros::Subscriber> subscribers_;

  // Swarm state
};

}  // namespace fast_planner

#endif