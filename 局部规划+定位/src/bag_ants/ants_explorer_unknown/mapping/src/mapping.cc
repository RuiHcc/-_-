#include <mapping/mapping.h>
//#include <opencv4/opencv2/opencv.hpp> // Ubuntu 20
//#include <opencv4/opencv2/ximgproc.hpp> // Ubuntu 20
 #include <opencv/cv.hpp> // Ubuntu 18

#include <iostream>
#include <Eigen/Dense>

namespace mapping {
void OccGridMap::updateMap(const Eigen::Vector3d& sensor_p,
                           const std::vector<Eigen::Vector3d>& pc) 
{
  resetVisited();
  vanished_grids = 0;

  Eigen::Vector3i sensor_idx = pos2idx(sensor_p);
  Eigen::Vector3i offset = sensor_idx - Eigen::Vector3i(size_x / 2, size_y / 2, size_z / 2);
  // TODO clear the updated part
  if (init_finished) {
    Eigen::Vector3i move = offset - Eigen::Vector3i(offset_x, offset_y, offset_z);
    Eigen::Vector3i from, to;
    from.setZero();
    to.setZero();
    from.x() = move.x() >= 0 ? 0 : move.x() + size_x;
    from.y() = move.y() >= 0 ? 0 : move.y() + size_y;
    from.z() = move.z() >= 0 ? 0 : move.z() + size_z;
    to.x() = move.x() >= 0 ? move.x() : size_x;
    to.y() = move.y() >= 0 ? move.y() : size_y;
    to.z() = move.z() >= 0 ? move.z() : size_z;
    for (int x = from.x(); x < to.x(); ++x) {
      for (int y = 0; y < size_y; ++y) {
        for (int z = 0; z < size_z; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          atId(id) = 0;
        }
      }
    }
    offset_x = offset.x();
    for (int y = from.y(); y < to.y(); ++y) {
      for (int x = 0; x < size_x; ++x) {
        for (int z = 0; z < size_z; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          atId(id) = 0;
        }
      }
    }
    offset_y = offset.y();
    for (int z = from.z(); z < to.z(); ++z) {
      for (int x = 0; x < size_x; ++x) {
        for (int y = 0; y < size_y; ++y) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          atId(id) = 0;
        }
      }
    }
    offset_z = offset.z();
  } else {
    offset_x = offset.x();
    offset_y = offset.y();
    offset_z = offset.z();
    init_finished = true;
  }


  // for (int x = 0; x < size_x; ++x) {
  //   for (int y = 0; y < size_y; ++y) {
  //     for (int z = 0; z < size_z; ++z) {
  //       Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
  //       atId(idx) = 0;
  //     }
  //   }
  // }

  // set occupied
  for (const auto& p : pc) {
// ROS_INFO("=====================");
    Eigen::Vector3d pt;
    bool inrange = filter(sensor_p, p, pt);
    // if (!inrange) continue;
    Eigen::Vector3i idx = pos2idx(pt);
    Eigen::Vector3i add = idx2add(idx);
    if (visited(add) == 1) {
      continue;
    }
    if (inrange) {
      if (visited(add) == -1) {
        vanished_grids--;
      }
      at(add) += hit_score_;       // set occupied
      // ROS_INFO_STREAM("set occ: " << (int)at(add) << ", add: " << add.transpose());
      if (at(add) > score_max_) at(add) = score_max_;
      // ROS_INFO_STREAM("set occ 2: " << (int)at(add) << ", add: " << add.transpose());

      if (pt.z() > 0.2) //TODO 高度大于0.5m多向z轴正方向膨胀
      {
        int z0 = idx.z() - 1 >= offset_z ? idx.z() - 1 : offset_z;
        int z1;
        z1 = idx.z() + 3 <= offset_z + size_z - 1 ? idx.z() + 3 : offset_z + size_z - 1;
        Eigen::Vector3i p = idx;
        for (p.z() = z0; p.z() <= z1; p.z()++) 
        {
          Eigen::Vector3i addz = idx2add(p);
          at(addz) += hit_score_;       // set occupied
        }
      }


      visited(add) = 1;  // set occupied
// ROS_INFO_STREAM("visited: " << (int)visited(add) << ", add: " << add.transpose());
    } else {
      // NOTE for fail detection
      if (at(add) > score_min_) {
        at(add) -= miss_score_;        // set free
        visited(add) = -1;  // set raycasted
        ++vanished_grids;
      }
    }
    // ray casting

    Eigen::Vector3i d_idx = sensor_idx - idx;
    Eigen::Vector3i step = d_idx.array().sign().cast<int>();
    Eigen::Vector3d delta_t;
    Eigen::Vector3d dp = sensor_p - pt;
    for (int i = 0; i < 3; ++i) {
      delta_t(i) = dp(i) == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(dp(i));
    }
    Eigen::Vector3d t_max;
    for (int i = 0; i < 3; ++i) {
      t_max(i) = step(i) > 0 ? (idx(i) + 1) - pt(i) / resolution : pt(i) / resolution - idx(i);
    }
    t_max = t_max.cwiseProduct(delta_t);
    Eigen::Vector3i rayIdx = idx;
    int cnt = 0;
    while ((rayIdx - sensor_idx).squaredNorm() != 1) {
      // find the shortest t_max
      int s_dim = 0;
      for (int i = 1; i < 3; ++i) {
        s_dim = t_max(i) < t_max(s_dim) ? i : s_dim;
      }
      rayIdx(s_dim) += step(s_dim);
      t_max(s_dim) += delta_t(s_dim);
      cnt++;
      if(cnt <= 3) continue;  // not update the closest 3 grid to occupied one (aviod 'free ring' on the ground)
      Eigen::Vector3i rayAdd = idx2add(rayIdx);
      if (visited(rayAdd) == 1) {
        continue;
      }
// ROS_INFO_STREAM("[rc] visited: " << (int)visited(rayAdd) << ", add: " << rayAdd.transpose());
      // NOTE for fail detection
      if (at(rayAdd) > score_min_) {
        at(rayAdd) -= miss_score_;        // set free
        // ROS_INFO_STREAM("set free: " << (int)at(rayAdd) << ", add: " << rayAdd.transpose());
        visited(rayAdd) = -1;  // set raycasted
        ++vanished_grids;
      }
    }

  }
}

void OccGridMap::occ2pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd) {
  pcd->clear();
  pcl::PointXYZ pt;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (atId(idx) > score_min_) {
          pt.x = (offset_x + x + 0.5) * resolution;
          pt.y = (offset_y + y + 0.5) * resolution;
          pt.z = (offset_z + z + 0.5) * resolution;
          pcd->push_back(pt);
        }
      }
    }
  }
  pcd->width = pcd->points.size();
  pcd->height = 1;
  pcd->is_dense = true;
  // pcl::toROSMsg(pcd, msg);
  // msg.header.frame_id = "world";
}

void OccGridMap::occ2pc_z_cut(sensor_msgs::PointCloud2& msg, double z_max) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (atId(idx) > score_min_) {
          pt.x = (offset_x + x + 0.5) * resolution;
          pt.y = (offset_y + y + 0.5) * resolution;
          pt.z = (offset_z + z + 0.5) * resolution;
          if(pt.z <= z_max){
            // std::cout<<"zmax: "<<z_max<<std::endl;
            pcd.push_back(pt);
          }
        }
      }
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}
 
void OccGridMap::inflate_once(const std::vector<Eigen::Vector3i>& v0, std::vector<Eigen::Vector3i>& v1) {
  static Eigen::Vector3i p;
  for (const auto& id : v0) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int z0 = id.z() - 1 >= offset_z ? id.z() - 1 : offset_z;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    Eigen::Vector3d pos = idx2pos(id);
    
    int z1;
    if (pos.z() > 0.8) //TODO 高度大于0.8m多向z轴正方向膨胀
      z1 = id.z() + 3 <= offset_z + size_z - 1 ? id.z() + 3 : offset_z + size_z - 1;
    else
      z1 = id.z() + 1 <= offset_z + size_z - 1 ? id.z() + 1 : offset_z + size_z - 1;
    
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++)
        for (p.z() = z0; p.z() <= z1; p.z()++) {
          auto ptr = atIdPtr(p);
          if (!(*ptr)) {
            *ptr = 1;
            v1.push_back(p);
          }
        }
  }
}

void OccGridMap::inflate_xy(const std::vector<Eigen::Vector3i>& v0, std::vector<Eigen::Vector3i>& v1) {
  static Eigen::Vector3i p;
  for (const auto& id : v0) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    p.z() = id.z();
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++) {
        auto ptr = atIdPtr(p);
        if (!(*ptr)) {
          *ptr = 1;
          v1.push_back(p);
        }
      }
  }
}

void OccGridMap::inflate_last() {
  static Eigen::Vector3i p;
  for (const auto& id : v1) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    // int z0 = id.z() - 1 >= offset_z ? id.z() - 1 : offset_z;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    // int z1 = id.z() + 1 <= offset_z + size_z - 1 ? id.z() + 1 : offset_z + size_z - 1;
    p.z() = id.z();
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++)
        // for (p.z() = z0; p.z() <= z1; p.z()++)
        atId(p) = 1;
  }
}

void OccGridMap::inflate(int inflate_size) {
  Eigen::Vector3i idx;
  v1.clear();
  for (idx.x() = offset_x; idx.x() < offset_x + size_x; ++idx.x())
    for (idx.y() = offset_y; idx.y() < offset_y + size_y; ++idx.y())
      for (idx.z() = offset_z; idx.z() < offset_z + size_z; ++idx.z()) {
        if (atId(idx) > score_min_) {
          v1.push_back(idx);
        }
      }
  for (int i = 0; i < inflate_size - 1; ++i) {
    std::swap(v0, v1);
    // v0 = v1;
    v1.clear();
    inflate_once(v0, v1);
  }
  std::swap(v0, v1);
  v1.clear();
  inflate_xy(v0, v1);
}


// p1->p2 -1 not visible    2 narrow   1 visible ; check_res是check的间隔长度，check_r_step是check的膨胀半径
VisibleState OccGridMap::isVisible(Eigen::Vector3d p1, Eigen::Vector3d p2, double check_res, int check_r_step, Eigen::Vector3d& occ_p)
{
  // 计算从 p1 到 p2 的向量，并获取该向量的单位向量和长度
    Eigen::Vector3d p1_2_p2 = p2 - p1;
    Eigen::Vector3d dir = p1_2_p2.normalized();
    double dis = p1_2_p2.norm();
    occ_p = p2; // 初始化 occ_p 为 p2，表示如果没有障碍物，则 p2 是可见的
    
    // 确保检查间隔不小于地图的分辨率，且不大于 p1 和 p2 之间的距离
    if(check_res < resolution) check_res = resolution;
    if(check_res >= dis) check_res = dis / 2.0;

    // 从 p1 开始沿着 p1 到 p2 的方向，以 check_res 为间隔检查每个点是否被占据
    Eigen::Vector3d pt = p1 + check_res * dir;
    for(;(pt-p1).norm() < dis; pt += check_res * dir)
    {
        if(isOccupied(pt)){  // 如果该点被占据，则返回 OCC 并设置 occ_p 为该点
            occ_p = pt;
            return OCC;
        }
    }

    if(check_r_step == 0){  // 如果没有障碍物且 check_r_step 为 0，则直接返回 VISIBLE
        return VISIBLE;
    }

    // 在p1所在平面生成与p1,p2连线垂直的4点  
    // 如果 check_r_step 不为 0，则在 p1 所在平面上，生成与 p1 和 p2 连线垂直的 4 个点
    double expand_res = check_r_step * resolution;
    double m_res = resolution;
    Eigen::Vector3d n2drone_norm_V = dir;
    std::vector<Eigen::Vector3d> tp_set;
    double maxElement = std::max(abs(n2drone_norm_V.maxCoeff()), abs(n2drone_norm_V.minCoeff()));
    if(maxElement <= 0){
        ROS_ERROR("dsafe check ERROR!");
    }
    // 如果 n2drone_norm_V 的某个分量是最大的绝对值，那么确定垂直于 p1 到 p2 方向的平面，并在该平面上生成 4 个点
    if(abs(n2drone_norm_V(0)) == maxElement){
        Eigen::Vector3d npt;
        for(int i = 0; i < 4; i++){
            npt(1) = point_surround_step[i](0);
            npt(2) = point_surround_step[i](1);
            // 通过解方程计算 npt 的第三个分量，确保 npt 在与 p1 到 p2 方向垂直的平面上
            npt(0) = -(npt(1)*n2drone_norm_V(1)+npt(2)*n2drone_norm_V(2))/n2drone_norm_V(0);
            // 将 npt 归一化并乘以扩展距离，然后添加到 tp_set 集合中
            npt = npt.normalized() * expand_res;
            tp_set.push_back(p1 + npt);
        }

    }else if(abs(n2drone_norm_V(1)) == maxElement){
        Eigen::Vector3d npt;
        for(int i = 0; i < 4; i++){
            npt(0) = point_surround_step[i](0);
            npt(2) = point_surround_step[i](1);
            npt(1) = -(npt(0)*n2drone_norm_V(0)+npt(2)*n2drone_norm_V(2))/n2drone_norm_V(1);
            npt = npt.normalized() * expand_res;
            tp_set.push_back(p1 + npt);
        }
    }else if(abs(n2drone_norm_V(2)) == maxElement){
        Eigen::Vector3d npt;
        for(int i = 0; i < 4; i++){
            npt(0) = point_surround_step[i](0);
            npt(1) = point_surround_step[i](1);
            npt(2) = -(npt(0)*n2drone_norm_V(0)+npt(1)*n2drone_norm_V(1))/n2drone_norm_V(2);
            npt = npt.normalized() * expand_res;
            tp_set.push_back(p1 + npt);
        }
    }

    std::vector<Eigen::Vector3d> vecpub;
    std::vector<Eigen::Vector3d> vecpub1;

    std::vector<bool> issafe;
    for(size_t i = 0; i < 3; i++){
        issafe.push_back(true);
    }
    int k = 0;
    // 遍历 tp_set 中的每个点，检查它们是否被占据
    for(std::vector<Eigen::Vector3d>::iterator it_p = tp_set.begin(); it_p != tp_set.end(); it_p++,k++){
        Eigen::Vector3d tp = *it_p;
        if (tp.z() < p1.z()) continue;
        Eigen::Vector3d begin_p = tp;
        tp += dir*resolution;
        // ROS_INFO_STREAM("[" << k << "] tp: " << tp.transpose());
        
        for(;(tp-begin_p).squaredNorm() < (dis- 0.2)*(dis-0.2); tp += dir*check_res){
            if(k==0||k==1) vecpub.push_back(tp);
            if(k==2||k==3) vecpub1.push_back(tp);

            if(!isInGlobalMap(tp)){
              break;
            }

            if(isOccupied(tp)){
              occ_p = p1 + dir * (tp - begin_p).norm();
              issafe[k] = false;
              break;
            }
        }
    }
    // markerpub->publishMaker(vecpub,GREEN_CUBE,"world",0.05);
    // markerpub->publishMaker(vecpub1,BLUE_CUBE,"world",0.05);

    // sleep(1);
    if(issafe[0] == false && issafe[1] == false){
        return NARROW;
    }
    if(issafe[2] == false && issafe[3] == false){
        return NARROW;
    }
    return VISIBLE;
}


void EdtMap::computeEDT(const OccGridMap& occGridMap) {
  resolution = occGridMap.resolution;
  size_x = occGridMap.size_x;
  size_y = occGridMap.size_y;
  size_z = occGridMap.size_z;
  offset_x = occGridMap.offset_x;
  offset_y = occGridMap.offset_y;
  offset_z = occGridMap.offset_z;
  data.resize(occGridMap.data.size());
  tmp_buffer1.resolution = occGridMap.resolution;
  tmp_buffer1.size_x = occGridMap.size_x;
  tmp_buffer1.size_y = occGridMap.size_y;
  tmp_buffer1.size_z = occGridMap.size_z;
  tmp_buffer1.offset_x = 0;
  tmp_buffer1.offset_y = 0;
  tmp_buffer1.offset_z = 0;
  tmp_buffer1.data.resize(occGridMap.data.size());
  tmp_buffer2.resolution = occGridMap.resolution;
  tmp_buffer2.size_x = occGridMap.size_x;
  tmp_buffer2.size_y = occGridMap.size_y;
  tmp_buffer2.size_z = occGridMap.size_z;
  tmp_buffer2.offset_x = 0;
  tmp_buffer2.offset_y = 0;
  tmp_buffer2.offset_z = 0;
  tmp_buffer2.data.resize(occGridMap.data.size());
  for (int x = 0; x < size_x; x++) {
    for (int y = 0; y < size_y; y++) {
      fill_edt([&](int z) { return occGridMap.atId(Eigen::Vector3i(x + offset_x, y + offset_y, z + offset_z)) ? 0 : std::numeric_limits<float>::max(); },
               [&](int z, float val) { tmp_buffer1.atId(Eigen::Vector3i(x, y, z)) = val; }, size_z);
    }
  }
  for (int x = 0; x < size_x; x++) {
    for (int z = 0; z < size_z; z++) {
      fill_edt([&](int y) { return tmp_buffer1.atId(Eigen::Vector3i(x, y, z)); },
               [&](int y, float val) { tmp_buffer2.atId(Eigen::Vector3i(x, y, z)) = val; }, size_y);
    }
  }
  for (int y = 0; y < size_y; y++) {
    for (int z = 0; z < size_z; z++) {
      fill_edt([&](int x) { return tmp_buffer2.atId(Eigen::Vector3i(x, y, z)); },
               [&](int x, float val) { atId(Eigen::Vector3i(x + offset_x, y + offset_y, z + offset_z)) = resolution * std::sqrt(val); }, size_x);
    }
  }
}

void EdtMap::computeEDTLayer(const OccGridMap& occGridMap){
  computeEDTLayer(occGridMap, layer_height_);
}

// Only compute the ESDF at a certain height
void EdtMap::computeEDTLayer(const OccGridMap& occGridMap, const double& layer_height) {
  layer_height_ = layer_height;
  resolution = occGridMap.resolution;
  size_x = occGridMap.size_x;
  size_y = occGridMap.size_y;
  size_z = 1;
  int occGridMap_data_size = size_x * size_y * size_z;
  offset_x = occGridMap.offset_x;
  offset_y = occGridMap.offset_y;
  offset_z = occGridMap.offset_z;
  data.resize(occGridMap_data_size);
  tmp_buffer1.resolution = occGridMap.resolution;
  tmp_buffer1.size_x = occGridMap.size_x;
  tmp_buffer1.size_y = occGridMap.size_y;
  tmp_buffer1.size_z = 1;
  tmp_buffer1.offset_x = 0;
  tmp_buffer1.offset_y = 0;
  tmp_buffer1.offset_z = 0;
  tmp_buffer1.data.resize(occGridMap_data_size);
  tmp_buffer2.resolution = occGridMap.resolution;
  tmp_buffer2.size_x = occGridMap.size_x;
  tmp_buffer2.size_y = occGridMap.size_y;
  tmp_buffer2.size_z = 1;
  tmp_buffer2.offset_x = 0;
  tmp_buffer2.offset_y = 0;
  tmp_buffer2.offset_z = 0;
  tmp_buffer2.data.resize(occGridMap_data_size);

  laplace_buffer.resolution = occGridMap.resolution;
  laplace_buffer.size_x = occGridMap.size_x;
  laplace_buffer.size_y = occGridMap.size_y;
  laplace_buffer.size_z = 1;
  laplace_buffer.offset_x = 0;
  laplace_buffer.offset_y = 0;
  laplace_buffer.offset_z = 0;
  laplace_buffer.data.resize(occGridMap_data_size);

  Eigen::Vector3d p_w(0.0, 0.0, layer_height);
  Eigen::Vector3i p_idx = occGridMap.pos2idx(p_w);
  layer_z_idx_ = p_idx.z();

  for (int x = 0; x < size_x; x++) {
    for (int y = 0; y < size_y; y++) {
      fill_edt([&](int z) { return occGridMap.atId(Eigen::Vector3i(x + offset_x, y + offset_y, layer_z_idx_)) ? 0 : std::numeric_limits<float>::max(); },
               [&](int z, float val) { tmp_buffer1.atId(Eigen::Vector3i(x, y, z)) = val; }, size_z);
    }
  }
  for (int x = 0; x < size_x; x++) {
    for (int z = 0; z < size_z; z++) {
      fill_edt([&](int y) { return tmp_buffer1.atId(Eigen::Vector3i(x, y, z)); },
               [&](int y, float val) { tmp_buffer2.atId(Eigen::Vector3i(x, y, z)) = val; }, size_y);
    }
  }
  for (int y = 0; y < size_y; y++) {
    for (int z = 0; z < size_z; z++) {
      fill_edt([&](int x) { return tmp_buffer2.atId(Eigen::Vector3i(x, y, z)); },
               [&](int x, float val) { atId(Eigen::Vector3i(x + offset_x, y + offset_y, z + offset_z)) = resolution * std::sqrt(val); }, size_x);
    }
  }
}

void EdtMap::computeLaplace() {
  cv::Mat mat(size_x, size_y, CV_64F);
  for (int x = 0; x < size_x; ++x)
    for (int y = 0; y < size_y; ++y) 
    {
      Eigen::Vector3i idx(offset_x + x, offset_y + y, layer_z_idx_);
      mat.at<double>(x, y) = atId(idx);
    }

  // Laplacian
  cv::Mat gray_lap;
  cv::Laplacian(mat, gray_lap, CV_64F, 5);
  // cv::normalize(gray_lap, gray_lap, 0.0, 1.0, cv::NORM_MINMAX);
  cv::Mat bin_mat(gray_lap.size(), CV_8U);

  double thr = -5;
  for (int i = 0; i < gray_lap.rows; ++i) {
    for (int j = 0; j < gray_lap.cols; ++j) {
        if (gray_lap.at<double>(i, j) < thr) {
            bin_mat.at<uchar>(i, j) = 255;
        }else{
            bin_mat.at<uchar>(i, j) = 0;
        }
    }
  }

  // 使用 cv::ximgproc::thinning 进行骨架化
  cv::Mat skeleton(bin_mat.size(), CV_8U);
  //cv::ximgproc::thinning(bin_mat, skeleton);

  for (int x = 0; x < size_x; ++x)
    for (int y = 0; y < size_y; ++y) 
    {
      Eigen::Vector3i idx(offset_x + x, offset_y + y, layer_z_idx_);
      laplace_buffer.atId(idx) = skeleton.at<uchar>(x, y) > 0 ? 1.0 : 0.0;
    }

}

bool EdtMap::checkIfDoor(const Eigen::Vector3i idx, const OccGridMap& occGridMap,
                         const int patch_step_half, const double cooridor_len_half, const double door_width_half)
{
  auto PCA_2d = [](Eigen::MatrixXd& X, Eigen::Vector2d& pv_1, Eigen::Vector2d& pv_2)
  {
    Eigen::MatrixXd C = X * X.transpose();
    C = C / (X.cols());

	  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(C);
	  Eigen::MatrixXd vec = eig.eigenvectors();
	  Eigen::MatrixXd val = eig.eigenvalues();
    double max_val = -9999;
    int max_inx;
    double min_val =  9999;
    int min_inx;
    for (int i = 0; i < 2; i++){
      if (val(i, 0) > max_val){
        max_val = val(i, 0);
        max_inx = i;
      }
      if (val(i, 0) < min_val){
        min_val = val(i, 0);
        min_inx = i;
      }
    }
    pv_1 << vec(0, max_inx), vec(1, max_inx);
    pv_2 << vec(0, min_inx), vec(1, min_inx);
  };

  // return false if occ
  auto CheckAlongDir = [occGridMap](const Eigen::Vector3d& pos, const Eigen::Vector3d& dir,
                              const double check_dis, const double check_res) -> bool
  {
      bool isfree = true;
      Eigen::Vector3d pt = pos;
      for(;(pt-pos).norm() < check_dis; pt += check_res * dir){
          if(occGridMap.atId(occGridMap.pos2idx(pt)) == 1){
            isfree = false;
            break;
          }
      }
      return isfree;
  };

  if (laplace_buffer.atId(idx) <= 0) return false;

  // Judge if this point is door
  if (occGridMap.atId(idx) == 1) return false; //occ

  // Find all skeleon points in the patch
  std::vector<Eigen::Vector3d> skele_vec;
  for (int i = idx.x() - patch_step_half; i <= idx.x() + patch_step_half; i++)
    for (int j = idx.y() - patch_step_half; j <= idx.y() + patch_step_half; j++)
  {
    Eigen::Vector3i idx_c(i, j, layer_z_idx_);

    if (laplace_buffer.atId(idx_c) > 0)
    {
      skele_vec.push_back(occGridMap.idx2pos(idx_c));
    }
  }

  if (skele_vec.size() < 3) return false;

  // PCA
  Eigen::Vector3d pos = occGridMap.idx2pos(idx);
  Eigen::MatrixXd X;
  X.resize(2, skele_vec.size());
  for(size_t i = 0; i < skele_vec.size(); i++){
      X.col(i) = (skele_vec[i] - pos).head(2);
  }
  Eigen::Vector2d dir_1_2d, dir_2_2d;
  PCA_2d(X, dir_1_2d, dir_2_2d);
  Eigen::Vector3d dir_1(dir_1_2d.x(), dir_1_2d.y(), pos.z());
  Eigen::Vector3d dir_2(dir_2_2d.x(), dir_2_2d.y(), pos.z());


  // if (pos.x() > 5){
  //   ROS_ERROR_STREAM("------------- pos: " << pos.transpose());
  //   ROS_INFO_STREAM("dir_1: " << dir_1.transpose() << " , dir_2: " << dir_2.transpose());
  // }

  // Check along dir_1
  if (!CheckAlongDir(pos,  dir_1, cooridor_len_half, resolution)) return false;
  if (!CheckAlongDir(pos, -dir_1, cooridor_len_half, resolution)) return false;


  // Check along dir_2
  if (door_width_half > 0)
  {
    if (CheckAlongDir(pos,  dir_2, door_width_half, resolution)) return false;
    if (CheckAlongDir(pos, -dir_2, door_width_half, resolution)) return false;
  }

  return true;
}


void EdtMap::computeDoorCue(const Eigen::Vector3d laser_p, const OccGridMap& occGridMap)
{

  const int patch_step_half = 5; 
  const double door_width_half = 1.5; 
  const double cooridor_len_half = 1.5; 

  const double judge_door_dis_thr = 5.0;
  const double door_near_dis_thr = 1.5;


  auto findMinDisInVec = [](std::vector<Eigen::Vector3d>& vec, Eigen::Vector3d& p_q)->double
  {
    double min_dis = 99999;
    for(auto p:vec){
      double dis = (p - p_q).norm();
      if (dis < min_dis) min_dis = dis;
    }
    return min_dis;
  };

  for (int y = patch_step_half; y < size_y-patch_step_half; ++y)
    for (int x = patch_step_half; x < size_x-patch_step_half; ++x) 
    {
      Eigen::Vector3i idx(offset_x + x, offset_y + y, layer_z_idx_);
      Eigen::Vector3d pos = occGridMap.idx2pos(idx);
      if ((pos - laser_p).norm() > judge_door_dis_thr) continue;

      if (checkIfDoor(idx, occGridMap, patch_step_half, cooridor_len_half, door_width_half))
      {
        if (door_vec_.empty() || findMinDisInVec(door_vec_, pos) > door_near_dis_thr){
          door_vec_.push_back(pos);
        }
      }
    }

  for (auto iter = door_vec_.begin(); iter != door_vec_.end(); )
  {
    if (((*iter) - laser_p).norm() > judge_door_dis_thr){
      iter++;
      continue;
    }
    Eigen::Vector3i idx = occGridMap.pos2idx((*iter));
    
    if (!checkIfDoor(idx, occGridMap, patch_step_half, cooridor_len_half * 0.5, -1))
    {
      door_vec_.erase(iter);
    }
    else{
      iter++;
    }
  }
} 

void EdtMap::vec2pc(std::vector<Eigen::Vector3d>& vec, sensor_msgs::PointCloud2& msg) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (size_t i = 0; i < vec.size(); i++)
  {
    pt.x = vec[i].x();
    pt.y = vec[i].y();
    pt.z = vec[i].z();
    pcd.push_back(pt);
  }

  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}

void EdtMap::occ2pc(sensor_msgs::PointCloud2& msg) {
  pcl::PointXYZI pt;
  pcl::PointCloud<pcl::PointXYZI> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (is_layer_mode_)
          Eigen::Vector3i idx(offset_x + x, offset_y + y, layer_z_idx_);

        pt.x = (offset_x + x + 0.5) * resolution;
        pt.y = (offset_y + y + 0.5) * resolution;
        
        if (is_layer_mode_){
          pt.z = layer_height_;
        }else{
          pt.z = (offset_z + z + 0.5) * resolution;
        }
        pt.intensity = atId(idx);
        pcd.push_back(pt);
      }
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}

void EdtMap::occ2pc_laplace(sensor_msgs::PointCloud2& msg) {
  pcl::PointXYZI pt;
  pcl::PointCloud<pcl::PointXYZI> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) 
    {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, layer_z_idx_);

        pt.x = (offset_x + x + 0.5) * resolution;
        pt.y = (offset_y + y + 0.5) * resolution;
        pt.z = (layer_z_idx_ + 0.5) * resolution;
        if (is_layer_mode_){
          pt.z = layer_height_ + 0.2;
        }
        pt.intensity = laplace_buffer.atId(idx);
        pcd.push_back(pt);
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}

}  // namespace mapping
