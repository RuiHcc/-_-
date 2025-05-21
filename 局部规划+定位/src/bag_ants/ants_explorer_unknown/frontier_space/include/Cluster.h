#ifndef _CLUSTER_H_
#define _CLUSTER_H_

// C++标准库
#include<iostream>
#include<algorithm>
#include <math.h>
#include <random>
#include <chrono>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
// Eigen
#include <Eigen/Dense>
// 私有库
#include <visualization.hpp>
#include <FreeSpace.h>
#include <mapping/mapping.h>

class MapServer;

using namespace Eigen;

// Viewpoint to cover a frontier cluster
enum VIEWPOINT_STATE
{
    GROUND,
    AIR
};
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  VIEWPOINT_STATE state_;
};

class FrontierCluster{
public:
    typedef std::shared_ptr<FrontierCluster> Ptr;
    CellList cell_list;
    Eigen::Vector3d center;
    Eigen::Vector3d normal; 
    Eigen::Vector3d normal_nd; 
    Eigen::Vector3d normal_rd; 

    std::vector<Viewpoint> viewpoints;

    int id;
    double d_vertical_max;
    double d_para_max;
    float intensity;

    Vector3d updateCenter();
    Vector3d updateNormal();
    void updateD();
    void addCell(Triangle& tcell);
    void clearCluster();
    void update(){
        updateCenter();
        updateNormal();
        updateD();
    }
};
inline Vector3d FrontierCluster::updateCenter(){
    Vector3d sum(0,0,0);
    for(CellList::iterator it = cell_list.begin(); it != cell_list.end(); it++){
        sum += it->_center;
    }
    center = sum/(int)cell_list.size();

    return center;
}

inline Vector3d FrontierCluster::updateNormal(){
    Vector3d sum(0,0,0);
    for(CellList::iterator it = cell_list.begin(); it != cell_list.end(); it++){
        sum += it->_normal;
    }
    normal = sum/(int)cell_list.size();

    return normal;
}

inline void FrontierCluster::updateD(){
    double dv, dp;
    d_vertical_max = 0.0;
    d_para_max = 0.0;
    for(CellList::iterator it = cell_list.begin(); it != cell_list.end(); it++){
        dv = abs((it->_center-center).dot(normal.normalized()));
        dp = (it->_center-center).cross(normal.normalized()).norm();

        if(dv > d_vertical_max) d_vertical_max = dv;
        if(dp > d_para_max) d_para_max = dp;
    }
}

inline void FrontierCluster::addCell(Triangle& tcell){
    cell_list.push_back(tcell);
}

inline void FrontierCluster::clearCluster(){
    cell_list.clear();
    d_vertical_max = 0;
    d_vertical_max = 0;
}



class SuperViewPoint{
public:
    vector<FrontierCluster> fc_list;
    pcl::PointXYZ super_vp;
    Vector3d refined_svp;
    int id;
    int keypose_inx;
    VIEWPOINT_STATE state;

    SuperViewPoint(){
        super_vp.x = 0;
        super_vp.y = 0;
        super_vp.z = 0;
        id = -1;
        state = GROUND;
    }
    void add(FrontierCluster& fc_p);
    void delet(FrontierCluster& fc_p);
    Vector3d getPoseForPlan();
    Vector3d getPoseActual();
};

inline void SuperViewPoint::add(FrontierCluster& fc_p){
    fc_list.push_back(fc_p);
    super_vp.x = (super_vp.x * (fc_list.size()-1) + fc_p.viewpoints[0].pos_(0))/fc_list.size();
    super_vp.y = (super_vp.y * (fc_list.size()-1) + fc_p.viewpoints[0].pos_(1))/fc_list.size();
    super_vp.z = (super_vp.z * (fc_list.size()-1) + fc_p.viewpoints[0].pos_(2))/fc_list.size();
    if(fc_p.viewpoints[0].state_ == AIR){
        state = AIR;
    }
}

inline Vector3d SuperViewPoint::getPoseForPlan(){
    if(state == AIR){
        return Vector3d(super_vp.x, super_vp.y, super_vp.z);
    }else{
        return Vector3d(super_vp.x, super_vp.y, super_vp.z + 1.0); //TODO magic number
    }
}

inline Vector3d SuperViewPoint::getPoseActual(){
    return Vector3d(super_vp.x, super_vp.y, super_vp.z);
}


typedef std::list<FrontierCluster> FrontierClusterList;
typedef std::list<SuperViewPoint> SuperViewPointList;

class FrontierClusterInfo{
public:
    ros::NodeHandle& nh;
    typedef std::shared_ptr<FrontierClusterInfo> Ptr;
    ros::Publisher frontier_cluster_pub, frontier_center_pub, frontier_oval_pub, svp_center_pub;
    ros::Subscriber drone_pos_sub;
    std::shared_ptr<visualization::Visualization> vis_ptr;

    // Input Info
    FreeSpaceFactory::Ptr free_space_fac_;
    std::shared_ptr<mapping::OccGridMap> grid_map_;
    std::shared_ptr<mapping::OccGridMap> grid_inf_map_;
    FreeSpaceInfo free_space_info;
    int temp_num = -1;
    bool reset_temp_num_at_begin= false;
    vector<SuperViewPoint> erased_svp_list;


    // Self Info
    FrontierClusterList frontier_cluster_list;
    SuperViewPointList svp_list;
    std::vector<int> id_pool;
    int id_cnt = 0;
    Vector3d drone_pose;
    Vector3d drone_vel;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_svp; // current frame
    // for viewpoint
    int id_vp_cnt = 0;
    std::vector<int> id_vp_pool;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr surfacepc_kdtree_;

    std::chrono::_V2::system_clock::time_point update_last_start_time;
    int car_id;


    // Parameter
    bool is_on_car;
    double car_odom_z;
    int key_pose_cnt = 0;
    int frontier_gen_frenq;
    double freespace_dis;
    int drone_num;
    int robot_id;
    double cluster_dv_max, cluster_dp_max, cluster_normal_thr;
    double dp_ratio_dv;
    int k_nearest;
    double sigmasq = 20.0;
    Vector3d last_key_pose;

    double fmesh_in_hull_dis_thr;

    // Viewpoint parameter
    double sensor_range;
    double ftr_dis_thr_from_sense_range;
    bool is_viewpoint;
    double candidate_rmax_, candidate_rmin_, candidate_dphi_;
    int candidate_rnum_, candidate_phinum_;
    double viewpoint_gd_cos_thr, viewpoint_gd_dis_thr;
    double svp_dis;
    double svp_arrive_dis;
    double svp_refine_r;
    double svp_s_thr;

    vector<Vector3d> vvv;
    vector<Vector3d> vvvv;
    vector<Vector3d> v5;
    vector<Vector3d> v6;



    // Main Function
    ros::Timer frontier_space_timer;
    void updateFrontierSpace();

    // Functions
    FrontierClusterInfo(ros::NodeHandle& nh);
    void frontierCluster(CellList& frontier_cell_set, FreeSpace& fs);
    void findCluster(CellList& frontier_cell_set_);
    void addCluster(FrontierCluster& fc);
    list<FrontierCluster>::iterator eraseCluster(list<FrontierCluster>::iterator iter);
    bool isArriandDeletePass();

    // Viewpoint
    void sampleViewpoints(FrontierCluster& fc);
    void sampleViewpointsInAir(FrontierCluster& fc);
    void sampleViewpointsOnGround(FrontierCluster& fc);
    double cosBetweenTwoVectors(Vector3d v1, Vector3d v2);
    void PCA(FrontierCluster& fc);
    bool isVisible(Vector3d p1, Vector3d p2, double check_res);
    bool checkViewPoint(Vector3d sample_p, FrontierCluster& fc);
    void addSuperPoint();
    list<SuperViewPoint>::iterator eraseSuperPoint(list<SuperViewPoint>::iterator it);
    void refineSuperPoint(SuperViewPoint& sp);
    bool checkSVP(SuperViewPoint& sp);
    
    
    // PoseGraph
    void addPoseEdge();
    // double calGraphDis(int inx1, int inx2);
    // double calGraphPath(int inx1, int inx2, vector<Vector3d>& path);
    // double getPathtoGoal(SuperViewPoint& svp, Vector3d& drone_p, vector<Vector3d>& path);
    // double getPathtoHome(Vector3d& drone_p, vector<Vector3d>& path);
    // void getNeighbor(int inx, vector<pair<int, double>>& neighbor);
    // double calHeris(int aim_inx, int inx);
    // double calConcretDis(SuperViewPoint& svp);
    // vector<Vector3d> pathRefine(vector<Vector3d>& path);

    // /spectralCluster
    void spectralCluster(FrontierCluster& c1, FrontierCluster& c2, CellList& frontier_cell_set, int k);
    void calSimilarMatrix(MatrixXd& W, MatrixXd& ED, const CellList& frontier_cell_set);  
    void calDegreeMatrix(const MatrixXd& W, MatrixXd& D);  
    void calLaplaceMatrix(const MatrixXd& W, const MatrixXd& D, MatrixXd& L);
    void getEigenLaplace(MatrixXd& L, MatrixXd& U, int k);
    void getEigenLaplaceSpectra(MatrixXd& L, MatrixXd& U, int k);
    void biKMeans(const MatrixXd& ED, const MatrixXd& Y, vector<size_t>& inx1, vector<size_t>& inx2);
    double calClusterDis(const Triangle t1, const Triangle t2);
    double calEulerDis(const Triangle t1, const Triangle t2);
    bool isKNearest(const MatrixXd& dis_matrix, const int iq, const int jq);

    // Ax functions
    bool isMeshInFreeSapce(FreeSpace& fs, Triangle& mesh);
    FrontierCluster getFC(int id);
    SuperViewPoint getSVP(int id);
    void odomCallBack(const nav_msgs::Odometry& msg);

    // Set functions
    void setMapServer(const std::shared_ptr<mapping::OccGridMap> &grid_map,
                      const std::shared_ptr<mapping::OccGridMap> &grid_inf_map);

    // Publish
    void publishFrontierClusterSurface(const ros::Time& rostime);
    void publishFrontierClusterCenterPCL(const ros::Time& rostime);
    void publishFrontierClusterOval(const ros::Time& rostime);
    void publishFrontierClusterSurface(const ros::Time& rostime, CellList& cell_list, int color);
    void publishSVPCenterPCL(const ros::Time& rostime);
    void pubViewPoints(const ros::Time& rostime);
    void pubPoseEdges(const ros::Time& rostime);
    void testMeshTable(FreeSpace& fs);
};

inline void FrontierClusterInfo::setMapServer(const std::shared_ptr<mapping::OccGridMap> &grid_map,
                                              const std::shared_ptr<mapping::OccGridMap> &grid_inf_map)
{
    grid_map_ = grid_map;
    grid_inf_map_ = grid_inf_map;
    free_space_fac_->setMap(grid_map, grid_inf_map);
}

// 得到该聚类的垂直于聚类法向量的两个面向量，显示为椭圆
inline void FrontierClusterInfo::PCA(FrontierCluster& fc){
    // cout<<"PCA"<<cell_list.size()<<endl;
    MatrixXd X;
    X.resize(3, fc.cell_list.size());
    for(size_t i = 0; i < fc.cell_list.size(); i++){
        X.col(i) = (fc.cell_list[i]._center - fc.center);
    }

	MatrixXd C = X * X.transpose();
	C = C / (X.cols());

	SelfAdjointEigenSolver<MatrixXd> eig(C);
	MatrixXd vec = eig.eigenvectors();
	MatrixXd val = eig.eigenvalues();
    MatrixXf::Index evalsMax;
    val.rowwise().sum().maxCoeff(&evalsMax);//得到最大特征值的位置
    Vector3d q;
    q << vec(0, evalsMax), vec(1, evalsMax), vec(2, evalsMax);//得到对应特征向量
    double valmax = val(evalsMax);
    double k = ((-1)*q.dot(fc.normal)/(fc.normal.dot(fc.normal)));
    fc.normal_nd = (q + k*fc.normal).normalized();

    MatrixXf::Index evalsMin;
    val.rowwise().sum().minCoeff(&evalsMin);//得到最大特征值的位置
    double valnd;
    for(MatrixXf::Index i = 0; i < 3; i++){
        if(i != evalsMax && i != evalsMin){
            valnd = val(i);
        }
    }
    fc.normal_nd = sqrt(valmax/valnd)*fc.normal_nd;

    fc.normal_rd = (fc.normal.cross(fc.normal_nd)).normalized();

    // cout<<"PCA done"<<endl;
}


#endif