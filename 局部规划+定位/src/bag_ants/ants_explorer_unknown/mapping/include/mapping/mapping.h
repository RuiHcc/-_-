#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#ifndef _MAPPING_H_
#define _MAPPING_H_

namespace mapping {

template <typename _Datatype>
struct RingBuffer {
 public:
  double resolution;
  int size_x, size_y, size_z;
  int offset_x, offset_y, offset_z;
  std::vector<_Datatype> data;

  inline const int idx2add(int x, int N) const {  //这个函数的作用是将一个整数 x 限制在范围 [0, N) 内，并返回结果。如果 x 在该范围内，则直接返回 x 的值。如果 x 超出了该范围，则将其映射到 [0, N) 范围内的对应值。
    // return x % N >= 0 ? x % N : x % N + N;
    // NOTE this is much faster than before!! 控制x位于N的范围内
    return (x & N) >= 0 ? (x & N) : (x & N) + N;
  } 
  inline const Eigen::Vector3i idx2add(const Eigen::Vector3i& id) const {  //这个函数接受一个三维向量 id，并对其各个分量分别调用 idx2add 函数，以确保其每个分量都在合适的范围内。
    return Eigen::Vector3i(idx2add(id.x(), size_x - 1),
                           idx2add(id.y(), size_y - 1),
                           idx2add(id.z(), size_z - 1));
  }
  // NOTE dangerous!! ad should be the address in the data
  inline const _Datatype& at(const Eigen::Vector3i& ad) const { //这个函数接受一个三维向量 ad，并返回缓冲区中相应位置的数据。在内部，它通过将三维坐标映射到一维数组中的索引来实现。
    return data[(ad.z() * size_x + ad.y()) * size_y + ad.x()];
  }
  inline _Datatype& at(const Eigen::Vector3i& ad) {  
    return data[(ad.z() * size_x + ad.y()) * size_y + ad.x()];
  }
  inline _Datatype* atPtr(const Eigen::Vector3i& ad) {  //这个函数与上一个函数类似，但返回的是指向相应位置数据的指针。
    return &(data[(ad.z() * size_x + ad.y()) * size_y + ad.x()]);
  }
  inline const _Datatype& atId(const Eigen::Vector3i& id) const {  //这个函数接受一个三维向量 id，并通过调用 idx2add 函数来获取在缓冲区中对应位置的数据。
    return at(idx2add(id));
  }
  inline _Datatype& atId(const Eigen::Vector3i& id) {
    return at(idx2add(id));
  }
  inline _Datatype* atIdPtr(const Eigen::Vector3i& id) {  //这个函数与上一个函数类似，但返回的是指向相应位置数据的指针。
    return atPtr(idx2add(id));
  }
  inline const Eigen::Vector3i pos2idx(const Eigen::Vector3d& pt) const {  //这个函数接受一个三维向量 pt，表示空间中的一个点，然后将其转换为在缓冲区中的索引位置。
    return (pt / resolution).array().floor().cast<int>();
  }
  inline const Eigen::Vector3d idx2pos(const Eigen::Vector3i& id) const {  //这个函数接受一个三维向量 id，表示缓冲区中的一个位置，然后将其转换为空间中的实际坐标点。
    return (id.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * resolution;
  }
  inline const bool isInMap(const Eigen::Vector3i& id) const {  //这个函数接受一个三维向量 id，表示缓冲区中的一个位置，然后检查该位置是否在缓冲区的有效范围内。
    return !((id.x() - offset_x) & (~(size_x - 1))) &&
           !((id.y() - offset_y) & (~(size_y - 1))) &&
           !((id.z() - offset_z) & (~(size_z - 1)));
  }
  inline const bool isInMap(const Eigen::Vector3d& p) const {  //这个函数接受一个三维向量 p，表示空间中的一个点，然后将其转换为缓冲区中的索引位置，并调用上一个函数来检查其是否在有效范围内。
    return isInMap(pos2idx(p));
  }
  template <typename _Msgtype>
  inline void to_msg(_Msgtype& msg) {  //这个模板函数将 RingBuffer 结构体中的数据转换为 ROS 消息类型 _Msgtype。
    msg.resolution = resolution;
    msg.size_x = size_x;
    msg.size_y = size_y;
    msg.size_z = size_z;
    msg.offset_x = offset_x;
    msg.offset_y = offset_y;
    msg.offset_z = offset_z;
    msg.data = data;
  }
  template <typename _Msgtype>
  inline void from_msg(const _Msgtype& msg) {  //这个模板函数从 ROS 消息类型 _Msgtype 中提取数据，并将其设置到 RingBuffer 结构体中。
    resolution = msg.resolution;
    size_x = msg.size_x;
    size_y = msg.size_y;
    size_z = msg.size_z;
    offset_x = msg.offset_x;
    offset_y = msg.offset_y;
    offset_z = msg.offset_z;
    data = msg.data;
  }
};

enum VisibleState {
    OCC,
    VISIBLE,
    NARROW
};

struct OccGridMap : public RingBuffer<int8_t> {
 private:
  // std::vector<int8_t> data;  // 1 for occupied, 0 for free or known
  // NOTE only used for update map
  std::vector<int8_t> vis;     // 1 for occupied, -1 for raycasted, 0 for free or unvisited
  bool init_finished = false;  // just for initialization
  int vanished_grids;
  std::vector<Eigen::Vector2d> point_surround_step;

  const int8_t hit_score_  = 5;
  const int8_t miss_score_ = 1;
  const int8_t score_max_ = 5;
  const int8_t score_min_ = 0;

 public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_occ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_surface;

  Eigen::Vector3d global_map_size;
  Eigen::Vector3d global_map_origin;
  Eigen::Vector3d map_min_boundary_;
  Eigen::Vector3d map_max_boundary_;
  bool is_on_car;
  double plan_roof, plan_floor;
  double extract_radius_threshold;
  double z_diff_max_thr;
  int surface_neighbor_thr;
  //safety
  ros::Time last_occ_update_time_;
  double odom_depth_timeout_ = 0.05;
  bool flag_depth_odom_timeout_ = false;

  std::vector<Eigen::Vector3i> v0, v1;
  int recieve_cnt = 0;
  inline void setup(double res, Eigen::Vector3d& map_size) {
    resolution = res;
    // The idx_size of map must be 2^n, n = int(log2(map_size.i() / res))
    size_x = exp2(int(log2(map_size.x() / res)));
    size_y = exp2(int(log2(map_size.y() / res)));
    size_z = exp2(int(log2(map_size.z() / res)));
    data.resize(size_x * size_y * size_z);
    vis.resize(size_x * size_y * size_z);
    v0.reserve(size_x * size_y * size_z);
    v1.reserve(size_x * size_y * size_z);

    // Eigen::Vector3d map_origin_ = Eigen::Vector3d(-global_map_size(0) / 2.0, -global_map_size(1) / 2.0, 0.0);
    map_min_boundary_ = global_map_origin;
    map_max_boundary_ = global_map_origin + global_map_size;
    std::cout<<"map_min_boundary_: "<<map_min_boundary_.transpose()<<", map_max_boundary_: "<<map_max_boundary_.transpose()<<std::endl;

    pc_occ.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pc_surface.reset(new pcl::PointCloud<pcl::PointXYZ>);

    point_surround_step.push_back(Eigen::Vector2d(1,0));
    point_surround_step.push_back(Eigen::Vector2d(-1,0));
    point_surround_step.push_back(Eigen::Vector2d(0,1));
    point_surround_step.push_back(Eigen::Vector2d(0,-1));
  }

  inline bool isInGlobalMap(const Eigen::Vector3d& pos) const{
    if (pos(0) < map_min_boundary_(0) + 1e-4 || pos(1) < map_min_boundary_(1) + 1e-4 ||
        pos(2) < map_min_boundary_(2) + 1e-4) {
      // cout << "less than min range!" << endl;
      return false;
    }
    if (pos(0) > map_max_boundary_(0) - 1e-4 || pos(1) > map_max_boundary_(1) - 1e-4 ||
        pos(2) > map_max_boundary_(2) - 1e-4) {
      return false;
    }
    return true;
  }

  inline bool getOdomDepthTimeout(){ 
    return flag_depth_odom_timeout_; 
  }

 private:
  // NOTE x must be in map
  inline const int8_t& visted(const Eigen::Vector3i& ad) const {
    return vis[(ad.z() * size_x + ad.y()) * size_y + ad.x()];
  }
  inline int8_t& visited(const Eigen::Vector3i& ad) {
    return vis[(ad.z() * size_x + ad.y()) * size_y + ad.x()];
  }
  inline void resetVisited() {
    std::fill(vis.begin(), vis.end(), 0);
  }
  // return true if in range; id_filtered will be limited in range
  inline bool filter(const Eigen::Vector3d& sensor_p,
                     const Eigen::Vector3d& p,
                     Eigen::Vector3d& pt) const {
    Eigen::Vector3i id = pos2idx(p);
    if (isInMap(id)) {
      pt = p;
      return true;
    } else {
      Eigen::Vector3d dp = p - sensor_p;
      Eigen::Array3d v = dp.array().abs() / resolution;
      Eigen::Array3d d;
      d.x() = v.x() <= size_x / 2 - 1 ? 0 : v.x() - size_x / 2 + 1;
      d.y() = v.y() <= size_y / 2 - 1 ? 0 : v.y() - size_y / 2 + 1;
      d.z() = v.z() <= size_z / 2 - 1 ? 0 : v.z() - size_z / 2 + 1;
      double t_max = 0;
      for (int i = 0; i < 3; ++i) {
        t_max = (d[i] > 0 && d[i] / v[i] > t_max) ? d[i] / v[i] : t_max;
      }
      pt = p - dp * t_max;
      return false;
    }
  }

 public:
  // in local map & occ -> true
  inline const bool isOccupied(const Eigen::Vector3i& id) const {
    return isInMap(id) && (at(idx2add(id)) > score_min_);
  }
  inline const bool isOccupied(const Eigen::Vector3d& p) const {
    return isOccupied(pos2idx(p));
  }
  // in local map & occ -> true
  inline const bool getInflateOccupancy(const Eigen::Vector3d& p) const {
    return isOccupied(pos2idx(p));
  }
  inline const bool getInflateOccupancyRoof(const Eigen::Vector3d& p) const {
    if(p(2) > plan_roof || p(2) < plan_floor){
      return true;
    }
    return isOccupied(pos2idx(p));
  }

  inline Eigen::Vector3d getMinBound() const {
    return map_min_boundary_;
  }

  inline Eigen::Vector3d getMaxBound() const {
    return map_max_boundary_;
  }
  
  inline Eigen::Vector3d getGridCenter(Eigen::Vector3d pos) const { return idx2pos(pos2idx(pos)); };

  void updateMap(const Eigen::Vector3d& sensor_p,
                 const std::vector<Eigen::Vector3d>& pc);
  void occ2pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcout);
  void ExtractSurface(Eigen::Vector3d laser_p, pcl::PointCloud<pcl::PointXYZ>::Ptr pco, pcl::PointCloud<pcl::PointXYZ>::Ptr pout);
  void occ2pc_z_cut(sensor_msgs::PointCloud2& msg, double z_max);
  void inflate_once(const std::vector<Eigen::Vector3i>& v0, std::vector<Eigen::Vector3i>& v1);
  void inflate_xy(const std::vector<Eigen::Vector3i>& v0, std::vector<Eigen::Vector3i>& v1);
  void inflate_last();
  void inflate(int inflate_size);

  double getResolution();

  VisibleState isVisible(Eigen::Vector3d p1, Eigen::Vector3d p2, double check_res, int check_r_step, Eigen::Vector3d& occ_p);
  
};

inline double OccGridMap::getResolution() { return resolution; }


class EdtMap : public RingBuffer<float> {
 private:
  RingBuffer<float> tmp_buffer1, tmp_buffer2, laplace_buffer;
  
  double layer_height_;
  int layer_z_idx_;
  bool is_layer_mode_ = false;

 public:
  std::vector<Eigen::Vector3d> door_vec_;

  EdtMap(){}
  EdtMap(const double layer_height):layer_height_(layer_height){
    is_layer_mode_ = true;
  }

  template <typename F_get_val, typename F_set_val>
  void fill_edt(F_get_val f_get_val, F_set_val f_set_val, int size) {
    int v[size];
    double z[size + 1];

    int k = 0;
    v[0] = 0;
    z[0] = -std::numeric_limits<double>::max();
    z[0 + 1] = std::numeric_limits<double>::max();

    for (int q = 0 + 1; q < size; q++) {
      k++;
      double s;

      do {
        k--;
        s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
      } while (s <= z[k]);

      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = std::numeric_limits<double>::max();
    }

    k = 0;

    for (int q = 0; q < size; q++) {
      while (z[k + 1] < q) k++;
      double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
      f_set_val(q, val);
    }
  }
  void computeEDT(const OccGridMap& occGridMap);
  void computeEDTLayer(const OccGridMap& occGridMap, const double& layer_height);
  void computeEDTLayer(const OccGridMap& occGridMap);
  void computeDoorCue(const Eigen::Vector3d laser_p, const OccGridMap& occGridMap);

  void computeLaplace();
  bool checkIfDoor(const Eigen::Vector3i idx, const OccGridMap& occGridMap,
                   const int patch_step_half, const double cooridor_len_half, const double door_width_half);
  void occ2pc(sensor_msgs::PointCloud2& msg);
  void occ2pc_laplace(sensor_msgs::PointCloud2& msg);
  void vec2pc(std::vector<Eigen::Vector3d>& vec, sensor_msgs::PointCloud2& msg);
};

}  // namespace mapping

#endif