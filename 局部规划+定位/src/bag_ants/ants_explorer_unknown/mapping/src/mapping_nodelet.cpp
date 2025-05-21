#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <msg_utils/OccMap3d.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Int8.h>

#include <atomic>
#include <thread>

namespace mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    PointCloudOdomSyncPolicy;
typedef message_filters::Synchronizer<PointCloudOdomSyncPolicy> PointCloudOdomSynchronizer;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, nav_msgs::Odometry>
    PtOdomSyncPolicy;
typedef message_filters::Synchronizer<PtOdomSyncPolicy> PtOdomSynchronizer;

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  std::atomic_flag callback_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Matrix3d laser2body_R_;
  Eigen::Vector3d laser2body_p_;
  Eigen::Vector3d up2body_t_, dn2body_t_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
  message_filters::Subscriber<geometry_msgs::PointStamped> up_sub_, dn_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::shared_ptr<PointCloudOdomSynchronizer> pc_odom_sync_Ptr_;
  std::shared_ptr<PtOdomSynchronizer> up_odom_sync_Ptr_, dn_odom_sync_Ptr_;
  ros::Publisher gridmap_pub_, gridmap_inflate_pub_;
  ros::Publisher gridmap_vs_pub_, surface_pub;
  ros::Publisher global_pcl_pub_, esdf_pub_, esdf_hessian_pub_, esdf_door_pub_;
  ros::Subscriber replan_sub_;
  ros::Timer safe_check_timer_;
  bool stop_fsm_ = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_pcl_;
  OccGridMap gridmap_;
  mapping::EdtMap edtMap_;
  int inflate_size_;
  void ultrasonic_up_callback(const geometry_msgs::PointStampedConstPtr& ultrasonic_msg,
                              const nav_msgs::OdometryConstPtr& odom_msg) {
    double dz = ultrasonic_msg->point.x;
    std::cout << "up dz: " << dz << std::endl;
  }
  void ultrasonic_dn_callback(const geometry_msgs::PointStampedConstPtr& ultrasonic_msg,
                              const nav_msgs::OdometryConstPtr& odom_msg) {
    double dz = -ultrasonic_msg->point.x;
    std::cout << "dn dz: " << dz << std::endl;
  }

  void replanCallback(const std_msgs::Int8 msg)
  {
    return;
    // if(msg.data >= 2)
    // {
    //   stop_fsm_ = true;
    //   ROS_ERROR_STREAM("[Mapping] CallBack Stop!");
    // }
  }


  void global_pcl_callback(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
  {
      // 将新点云添加到全局点云
      *global_pcl_ += *inputCloud;

      // 执行体素栅格滤波
      pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(global_pcl_);
      sor.setLeafSize(0.1, 0.1, 0.1); // 设置体素大小
      sor.filter(*filteredCloud);

      // 将滤波后的点云发布出去
      sensor_msgs::PointCloud2 filteredCloudMsg;
      pcl::toROSMsg(*filteredCloud, filteredCloudMsg);
      // 设置header等信息
      filteredCloudMsg.header.stamp = ros::Time::now();
      filteredCloudMsg.header.frame_id = "world"; // 设置坐标系

      global_pcl_pub_.publish(filteredCloudMsg);
  }

  void pc_odom_check(const ros::TimerEvent& /*event*/){
    if(gridmap_.last_occ_update_time_.toSec() < 1.0 ) return;
    if((ros::Time::now() - gridmap_.last_occ_update_time_).toSec() > gridmap_.odom_depth_timeout_){
      gridmap_.flag_depth_odom_timeout_ = true;
    }
  }

  void pc_odom_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg,
                        const nav_msgs::OdometryConstPtr& odom_msg) 
  {
    if (stop_fsm_) return;
    if (callback_lock_.test_and_set()) {
      return;
    }

    // ROS_ERROR("in pc_odom_callback");
    // ros::Time t1 = ros::Time::now();
    gridmap_.last_occ_update_time_ = ros::Time::now();

    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Quaterniond laser_q = body_q * Eigen::Quaterniond(laser2body_R_);
    Eigen::Vector3d laser_p = body_p;
    std::vector<Eigen::Vector3d> pc;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*pc_msg, point_cloud);
    // if (pc_msg->header.frame_id != "world") {
    //   laser_p = body_p + laser2body_p_;
    // }
    for (const auto& pt : point_cloud.points) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      // if (pc_msg->header.frame_id != "world") {
      //   p = laser_q.toRotationMatrix() * p + laser_p;
      // }
      pc.push_back(p);
    }
    // TODO pub local map
    // ros::Time t1 = ros::Time::now();
    // ROS_ERROR("in gridmap_.updateMap(laser_p, pc)");
    gridmap_.updateMap(laser_p, pc);
    // ROS_ERROR("out gridmap_.updateMap(laser_p, pc)");

    // std::cout << gridmap_.vanished_grids * 100.0 / gridmap_.size_x / gridmap_.size_y / gridmap_.size_z << "%" << std::endl;
    // ros::Time t2 = ros::Time::now();
    // std::cout << "updatemap costs: " << (t2 - t1).toSec() * 1e3 << "ms" << std::endl;
    msg_utils::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_pub_.publish(gridmap_msg);

    OccGridMap gridmap_inflate = gridmap_;
    // ros::Time t1 = ros::Time::now();
    gridmap_inflate.inflate(inflate_size_);
    // ros::Time t2 = ros::Time::now();
    // std::cout << "inflate costs: " << (t2 - t1).toSec() * 1e3 << "ms" << std::endl;

    gridmap_inflate.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
 
    // ros::Time t2 = ros::Time::now();
    // std::cout << "mapping totally costs: " << (t2 - t1).toSec() * 1e3 << "ms" << std::endl;

    // // t1 = ros::Time::now();
    // edtMap_.computeEDTLayer(gridmap_);
    // sensor_msgs::PointCloud2 esdf_msg;
    // esdf_msg.header.stamp = ros::Time::now();
    // edtMap_.occ2pc(esdf_msg);
    // esdf_pub_.publish(esdf_msg);

    // edtMap_.computeLaplace();
    // sensor_msgs::PointCloud2 esdf_h_msg;
    // esdf_h_msg.header.stamp = ros::Time::now();
    // edtMap_.occ2pc_laplace(esdf_h_msg);
    // esdf_hessian_pub_.publish(esdf_h_msg);
    // // ROS_ERROR("in computeDoorCue");


    // edtMap_.computeDoorCue(laser_p, gridmap_inflate);
    // sensor_msgs::PointCloud2 door_msg;
    // door_msg.header.stamp = ros::Time::now();
    // edtMap_.vec2pc(edtMap_.door_vec_, door_msg);
    // esdf_door_pub_.publish(door_msg);
    
    // ROS_ERROR("in computeDoorCue done");


    // t2 = ros::Time::now();
    // std::cout << "edt costs: " << (t2 - t1).toSec() * 1e3 << "ms" << std::endl;

    gridmap_.occ2pc(gridmap_.pc_occ);
    gridmap_.ExtractSurface(laser_p, gridmap_.pc_occ, gridmap_.pc_surface);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*gridmap_.pc_occ, msg);
    msg.header.frame_id = "world";
    gridmap_vs_pub_.publish(msg);

    global_pcl_callback(gridmap_.pc_occ);

    sensor_msgs::PointCloud2 msg1;
    pcl::toROSMsg(*gridmap_.pc_occ, msg1);
    msg1.header.frame_id = "world";
    surface_pub.publish(msg1);

    // ROS_INFO("mapping callbacks down");

    callback_lock_.clear();

    // ROS_ERROR("out pc_odom_callback");
  }

  void init(ros::NodeHandle& nh) {
    // exit();
    laser2body_R_.setIdentity();
    laser2body_p_.setZero();
    std::vector<double> offsetRotV, offsetTransV;
    // set offset of sensor from body frame
    if (nh.param<std::vector<double>>("/lio_sam/offsetTrans", offsetTransV, std::vector<double>())) {
      laser2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(offsetTransV.data(), 3, 1);
    }
    if (nh.param<std::vector<double>>("/lio_sam/offsetRot", offsetRotV, std::vector<double>())) {
      laser2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(offsetRotV.data(), 3, 3);
    }
    if (nh.getParam("dypa02/offsetTransUp", offsetTransV)) {
      up2body_t_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(offsetTransV.data(), 3, 1);
    }
    if (nh.getParam("dypa02/offsetTransDn", offsetTransV)) {
      dn2body_t_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(offsetTransV.data(), 3, 1);
    }
    // set parameters of mapping
    double res;
    Eigen::Vector3d map_size;
    nh.getParam("resolution", res);
    nh.getParam("global_map_x", gridmap_.global_map_size.x());
    nh.getParam("global_map_y", gridmap_.global_map_size.y());
    nh.getParam("global_map_z", gridmap_.global_map_size.z());
    nh.getParam("global_map_origin_x", gridmap_.global_map_origin.x());
    nh.getParam("global_map_origin_y", gridmap_.global_map_origin.y());
    nh.getParam("global_map_origin_z", gridmap_.global_map_origin.z());
    nh.getParam("local_x", map_size.x());
    nh.getParam("local_y", map_size.y());
    nh.getParam("local_z", map_size.z());
    nh.getParam("inflate_size", inflate_size_);
    nh.getParam("extract_surface_radius", gridmap_.extract_radius_threshold);
    nh.getParam("extract_surface_z_max_thr", gridmap_.z_diff_max_thr);
    nh.getParam("extract_surface_neighbor_thr", gridmap_.surface_neighbor_thr);

    gridmap_.setup(res, map_size);
    gridmap_.last_occ_update_time_.fromSec(0);
    global_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    edtMap_ = mapping::EdtMap(1.0);

    gridmap_pub_ = nh.advertise<msg_utils::OccMap3d>("gridmap", 1);
    gridmap_inflate_pub_ = nh.advertise<msg_utils::OccMap3d>("gridmap_inflate", 1);

    pc_sub_.subscribe(nh, "local_pointcloud", 1);
    up_sub_.subscribe(nh, "up_distance", 1);
    dn_sub_.subscribe(nh, "dn_distance", 1);
    odom_sub_.subscribe(nh, "odom", 50);
    pc_odom_sync_Ptr_ = std::make_shared<PointCloudOdomSynchronizer>(PointCloudOdomSyncPolicy(100), pc_sub_, odom_sub_);
    pc_odom_sync_Ptr_->registerCallback(boost::bind(&Nodelet::pc_odom_callback, this, _1, _2));
    up_odom_sync_Ptr_ = std::make_shared<PtOdomSynchronizer>(PtOdomSyncPolicy(10), up_sub_, odom_sub_);
    up_odom_sync_Ptr_->registerCallback(boost::bind(&Nodelet::ultrasonic_up_callback, this, _1, _2));
    dn_odom_sync_Ptr_ = std::make_shared<PtOdomSynchronizer>(PtOdomSyncPolicy(10), dn_sub_, odom_sub_);
    dn_odom_sync_Ptr_->registerCallback(boost::bind(&Nodelet::ultrasonic_dn_callback, this, _1, _2));
    safe_check_timer_ = nh.createTimer(ros::Duration(0.05), &Nodelet::pc_odom_check, this);

    replan_sub_ = nh.subscribe("/replan", 1, &Nodelet::replanCallback, this);

    gridmap_vs_pub_   = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap", 1);
    surface_pub       = nh.advertise<sensor_msgs::PointCloud2>("surface", 1);
    global_pcl_pub_   = nh.advertise<sensor_msgs::PointCloud2>("global_pcl", 1);
    esdf_pub_         = nh.advertise<sensor_msgs::PointCloud2>("esdf", 1);
    esdf_hessian_pub_ = nh.advertise<sensor_msgs::PointCloud2>("esdf_H", 1);
    esdf_door_pub_    = nh.advertise<sensor_msgs::PointCloud2>("esdf_ddor", 1);
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapping::Nodelet, nodelet::Nodelet);