

#include <task_allocation/task_allocation.h>
#include <task_allocation/HGrid.h>
#include <task_allocation/GridTour.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>

#include <fstream>

using Eigen::Vector4d;

namespace fast_planner {

shared_ptr<Astar> FastExplorationFSM::astar_;
shared_ptr<RayCaster> FastExplorationFSM::caster_;
double FastExplorationFSM::vm_;
double FastExplorationFSM::am_;
double FastExplorationFSM::yd_;
double FastExplorationFSM::ydd_;
double FastExplorationFSM::w_dir_;

void FastExplorationFSM::init(ros::NodeHandle& nh) {
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(sdf_map_);

    hgrid_.reset(new HGrid(edt_environment_, nh));

    ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);

    nh.param("exploration/refine_local", ep_->refine_local_, true);
    nh.param("exploration/refined_num", ep_->refined_num_, -1);
    nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
    nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
    nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
    nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
    nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
    nh.param("exploration/drone_num", ep_->drone_num_, 1);
    nh.param("exploration/drone_id", ep_->drone_id_, 1);
    nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);

    nh.param("exploration/vm", FastExplorationFSM::vm_, -1.0);
    nh.param("exploration/am", FastExplorationFSM::am_, -1.0);
    nh.param("exploration/yd", FastExplorationFSM::yd_, -1.0);
    nh.param("exploration/ydd", FastExplorationFSM::ydd_, -1.0);
    nh.param("exploration/w_dir", FastExplorationFSM::w_dir_, -1.0);

    nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);

    ed_->swarm_state_.resize(ep_->drone_num_);
    ed_->pair_opt_stamps_.resize(ep_->drone_num_);
    ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
    for (int i = 0; i < ep_->drone_num_; ++i) {
        ed_->swarm_state_[i].stamp_ = 0.0;
        ed_->pair_opt_stamps_[i] = 0.0;
        ed_->pair_opt_res_stamps_[i] = 0.0;
    }

    for (auto& state : ed_->swarm_state_) {
        state.stamp_ = 0.0;
        state.recent_interact_time_ = 0.0;
        state.recent_attempt_time_ = 0.0;
    }
    ed_->last_grid_ids_ = {};
    ed_->reallocated_ = true;
    ed_->pair_opt_stamp_ = 0.0;
    ed_->wait_response_ = false;
    ed_->plan_num_ = 0;

    exec_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::FSMCallback, this);
    odom_sub_ = nh.subscribe("/odom", 1, &FastExplorationFSM::odomCallback, this);
}

void FastExplorationFSM::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    Vector3d rot_x = odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    odom_yaw_ = atan2(rot_x(1), rot_x(0));
}

/**
 * hgrid 使用
 * 1、hgrid_->getNextGrid(grid_ids, grid_pos, grid_yaw)  
 *      获取到序列上网格位置、yaw
 * 2、hgrid_->getFrontiersInGrid(ego_ids, ftr_ids, ftr2_ids)
 * 
 * 3、hgrid_->getCenter(grid_ids.front())
 * 
 * 4、hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat)
 * 
 * 5、hgrid_->getUnknownCellsNum(grid_ids[i])
 * 
 * 6、hgrid_->getCostDroneToGrid(pos, grid_ids[0], first)
 * 
 * 7、hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size())
 * 
 * 8、hgrid_->inputFrontiers(ed_->averages_)
 * 
 * 9、hgrid_->updateGridData(ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids)
 *
 * 10、hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_)
 * 
 * 
*/

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
    Vector3d pos = odom_pos_;
    Vector3d vel = odom_vel_;
    Vector3d yaw = Vector3d(odom_yaw_, 0, 0);
    std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose() << std::endl;
    vector<int> grid_ids, frontier_ids, frontier2_ids;

    findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids, frontier2_ids);
}


void FastExplorationFSM::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids, vector<int>& frontier2_ids) {
  auto t1 = ros::Time::now();

  // Select nearby drones according to their states' stamp
  vector<Eigen::Vector3d> positions = { cur_pos };
  // vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0) };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // Partitioning-based tour planning
  vector<int> ego_ids;
  vector<vector<int>> other_ids;
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};
    ROS_WARN("No grid found after findGlobalTourOfGrid");
    return;
  }
  if(ego_ids.empty())
  ROS_WARN("ego is empty");
  grid_ids = ego_ids;

  double grid_time = (ros::Time::now() - t1).toSec();

  // Frontier-based single drone tour planning
  // Restrict frontier within the first visited grid
  t1 = ros::Time::now();

  vector<int> ftr_ids;
  vector<int> ftr2_ids;
  // uniform_grid_->getFrontiersInGrid(ego_ids[0], ftr_ids);
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids, ftr2_ids);
  ROS_INFO("Find frontier tour, %d involved------------", ftr_ids.size());

  if (ftr_ids.empty()) {
    frontier_ids = {};
    return;
  }

  // Consider next grid in frontier tour planning
  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = { grid_pos };
  }
  frontier2_ids = ftr2_ids;
  findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  // findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr2_ids, grid_pos_vec, frontier2_ids);
  double ftr_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

bool FastExplorationFSM::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init) {

    ROS_INFO("Find grid tour---------------");

    auto t1 = ros::Time::now();

    auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
    if (grid_ids.empty() && ed_->swarm_state_.empty()) {
        // ROS_WARN("Empty dominance1.");
        ed_->grid_tour_.clear();
        // return false;
    }
    // ROS_WARN("Empty dominance33 %i",grid_ids.size());

    // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

    vector<int> first_ids, second_ids;

    // TODO 此处需要一个函数，把前沿的数值填入ed_->averages_
    updateFrontierStates();

    hgrid_->inputFrontiers(ed_->averages_);

    hgrid_->updateGridData(
        ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);
    // ROS_WARN("Empty dominance22 %i",grid_ids.size());

    if (grid_ids.empty()) {
        ROS_WARN("Empty dominance.");
        ed_->grid_tour_.clear();
        return false;
    }

    std::cout << "Allocated grid: ";
    for (auto id : grid_ids) std::cout << id << ", ";
    std::cout << "" << std::endl;

    Eigen::MatrixXd mat;
    // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
    if (!init)
        hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
    else
        hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

    double mat_time = (ros::Time::now() - t1).toSec();

    // Find optimal path through ATSP
    t1 = ros::Time::now();
    const int dimension = mat.rows();
    const int drone_num = 1;

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
        }
        file << "\n";
    }
    file.close();

    // Create par file
    file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
    // "\n"; file << "MTSP_MAX_SIZE = "
    //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 2;
    if (!tsp_client_.call(srv)) {
        ROS_ERROR("Fail to solve ATSP.");
        return false;
    }

    double mtsp_time = (ros::Time::now() - t1).toSec();
    // std::cout << "AmTSP time: " << mtsp_time << std::endl;

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
        if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
        int id = stoi(res);
        ids.push_back(id - 1);
        if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour of grid
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
        if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
        } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
        } else {
        tour.push_back(id);
        }
    }


    others.resize(drone_num - 1);
    for (int i = 1; i < tours.size(); ++i) {
        if (tours[i][0] == 1) {
        indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
        } else {
        others[tours[i][0] - 2].insert(
            others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
        }
    }
    for (auto& id : indices) {
        id -= 1 + drone_num;
    }
    for (auto& other : others) {
        for (auto& id : other) id -= 1 + drone_num;
    }
    std::cout << "Grid tour: ";
    for (auto& id : indices) {
        id = grid_ids[id];
        std::cout << id << ", ";
    }
    std::cout << "" << std::endl;

    // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
    grid_ids = indices;
    hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

    ed_->last_grid_ids_ = grid_ids;
    ed_->reallocated_ = false;

    // hgrid_->checkFirstGrid(grid_ids.front());

    return true;
}

void FastExplorationFSM::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {

    auto t1 = ros::Time::now();

    vector<Eigen::Vector3d> positions = { cur_pos };
    vector<Eigen::Vector3d> velocities = { cur_vel };
    vector<double> yaws = { cur_yaw[0] };

    // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
    Eigen::MatrixXd mat;
    getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
    const int dimension = mat.rows();
    // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

    double mat_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("mat time: %lf", mat_time);

    // Find optimal allocation through AmTSP
    t1 = ros::Time::now();

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
        }
        file << "\n";
    }
    file.close();

    // Create par file
    const int drone_num = 1;

    file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
    file << "MTSP_MAX_SIZE = "
        << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 1;
    if (!tsp_client_.call(srv)) {
        ROS_ERROR("Fail to solve ATSP.");
        return;
    }

    double mtsp_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("AmTSP time: %lf", mtsp_time);

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
        if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
        int id = stoi(res);
        ids.push_back(id - 1);
        if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
        if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
        } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
        } else {
        tour.push_back(id);
        }
    }

    vector<vector<int>> others(drone_num - 1);
    for (int i = 1; i < tours.size(); ++i) {
        if (tours[i][0] == 1) {
        indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
        }
        // else {
        //   others[tours[i][0] - 2].insert(
        //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
        // }
    }
    for (auto& id : indices) {
        id -= 1 + drone_num;
    }
    // for (auto& other : others) {
    //   for (auto& id : other)
    //     id -= 1 + drone_num;
    // }

    if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
        indices.pop_back();
    }
    // Subset of frontier inside first grid
    for (int i = 0; i < indices.size(); ++i) {
        indices[i] = ftr_ids[indices[i]];
    }

    // Get the path of optimal tour from path matrix
    // frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
    // if (!grid_pos.empty()) {
    //     ed_->frontier_tour_.push_back(grid_pos[0]);
    // }

    // ed_->other_tours_.clear();
    // for (int i = 1; i < positions.size(); ++i) {
    //   ed_->other_tours_.push_back({});
    //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
    // }

    double parse_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
    //     parse_time, indices.size());
}

void FastExplorationFSM::updateFrontierStates() {
    // 重点是填充这个数据：list<Frontier> frontiers_;


    // ed_->averages_.clear();
    // for (auto& ftr : ed_->frontiers_) {
    //   ed_->averages_.push_back(ftr.average_);
    // }
}

void FastExplorationFSM::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double>& yaws, const vector<int>& ftr_ids,
    const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat) {

  Eigen::MatrixXd full_mat;
  getSwarmCostMatrix(positions, velocities, yaws, full_mat);

  // Get part of the full matrix according to selected frontier

  const int drone_num = positions.size();
  const int ftr_num = ftr_ids.size();
  int dimen = 1 + drone_num + ftr_num;
  if (!grid_pos.empty()) dimen += 1;

  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = -1000;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to frontiers
  for (int i = 0; i < ftr_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // Costs from drones to frontiers
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + i, 1 + drone_num + j) = full_mat(1 + i, 1 + drone_num + ftr_ids[j]);
      mat(1 + drone_num + j, 1 + i) = 0;
    }
  }
  // Costs between frontiers
  for (int i = 0; i < ftr_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + drone_num + i, 1 + drone_num + j) =
          full_mat(1 + drone_num + ftr_ids[i], 1 + drone_num + ftr_ids[j]);
    }
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }

  // Consider next grid in global tour
  if (!grid_pos.empty()) {
    // Depot, 1000, -1000
    mat(0, 1 + drone_num + ftr_num) = 1000;
    mat(1 + drone_num + ftr_num, 0) = -1000;

    // Drone
    for (int i = 0; i < drone_num; ++i) {
      mat(1 + i, 1 + drone_num + ftr_num) = 1000;
      mat(1 + drone_num + ftr_num, 1 + i) = 1000;
    }

    // Frontier
    vector<Eigen::Vector3d> points, tmps;
    vector<double> yaws;
    getTopViewpointsInfo(positions[0], points, yaws, tmps);
    Eigen::Vector3d next_grid = grid_pos[0];

    for (int i = 0; i < ftr_num; ++i) {
      double cost = computeCost(
          next_grid, points[ftr_ids[i]], 0, 0, Eigen::Vector3d(0, 0, 0), 0, tmps);
      mat(1 + drone_num + i, 1 + drone_num + ftr_num) = cost;
      mat(1 + drone_num + ftr_num, 1 + drone_num + i) = cost;
    }
  }
}

void FastExplorationFSM::getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points,
    vector<double>& yaws, vector<Eigen::Vector3d>& averages) {
  points.clear();
  yaws.clear();
  averages.clear();

  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest coverage).
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}

void FastExplorationFSM::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double> yaws, Eigen::MatrixXd& mat) {

    const int drone_num = positions.size();

//   const int ftr_num = frontiers_.size(); // 前沿点 TODO
    const int ftr_num = 0;

    const int dimen = 1 + drone_num + ftr_num;
    mat = Eigen::MatrixXd::Zero(dimen, dimen);

    // Virtual depot to drones
    for (int i = 0; i < drone_num; ++i) {
        mat(0, 1 + i) = -1000;
        mat(1 + i, 0) = 1000;
    }
    // Virtual depot to frontiers
    for (int i = 0; i < ftr_num; ++i) {
        mat(0, 1 + drone_num + i) = 1000;
        mat(1 + drone_num + i, 0) = 0;
    }
    // Costs between drones
    for (int i = 0; i < drone_num; ++i) {
        for (int j = 0; j < drone_num; ++j) {
        mat(1 + i, 1 + j) = 10000;
        }
    }

    // Costs from drones to frontiers
    for (int i = 0; i < drone_num; ++i) {
        int j = 0;
        // TODO 遍历全部的前沿点，计算代价值
        for (auto ftr : frontiers_) {
        Viewpoint vj = ftr.viewpoints_.front();
        vector<Vector3d> path;
        mat(1 + i, 1 + drone_num + j) =
            computeCost(positions[i], vj.pos_, yaws[0], vj.yaw_, velocities[i], 0.0, path);
        mat(1 + drone_num + j, 1 + i) = 0;
        ++j;
        }
    }
    // Costs between frontiers
    int i = 0, j = 0;
    for (auto ftr : frontiers_) {
        for (auto cs : ftr.costs_) {
        mat(1 + drone_num + i, 1 + drone_num + j) = cs;
        ++j;
        }
        ++i;
        j = 0;
    }
    // Diag
    for (int i = 0; i < dimen; ++i) {
        mat(i, i) = 1000;
    }

    // std::cout << "mat: " << std::endl;
    // std::cout << mat << std::endl;
}

double FastExplorationFSM::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1,
    const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {
  // Cost of position change
  double pos_cost = searchPath(p1, p2, path) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);

  // // Consider yaw rate change
  // if (fabs(yd1) > 1e-3)
  // {
  //   double diff1 = y2 - y1;
  //   while (diff1 < -M_PI)
  //     diff1 += 2 * M_PI;
  //   while (diff1 > M_PI)
  //     diff1 -= 2 * M_PI;
  //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
  // }
  // else
  // {
  // }
}

double FastExplorationFSM::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path) {
    // Try connect two points with straight line
    bool safe = true;
    Eigen::Vector3i idx;
    caster_->input(p1, p2);
    while (caster_->nextId(idx)) {
        if (sdf_map_->getInflateOccupancy(idx) == 1 || !sdf_map_->isInBox(idx)) {
        // map_->getOccupancy(idx) == SDFMap::UNKNOWN
        safe = false;
        break;
        }
    }
    if (safe) {
        path = { p1, p2 };
        return (p1 - p2).norm();
    }
    // Search a path using decreasing resolution
    vector<double> res = { 0.4 };
    for (int k = 0; k < res.size(); ++k) {
        astar_->reset();
        astar_->setResolution(res[k]);
        if (astar_->search(p1, p2) == Astar::REACH_END) {
        path = astar_->getPath();
        return astar_->pathLength(path);
        }
    }
    // Use Astar early termination cost as an estimate
    path = { p1, p2 };
    return 100;
}


}  // namespace fast_planner
