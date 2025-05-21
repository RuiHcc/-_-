#ifndef _FREE_SPACE_H_
#define _FREE_SPACE_H_

// C++标准库
#include <vector>
#include <unordered_map>
// Eigen库
#include <Eigen/Eigen>
// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include "frontier_space/FreeSpaceMsg.h"
#include "frontier_space/FreeSpaceInfoMsg.h"
#include "frontier_space/FrontierClusterListMsg.h"
#include "frontier_space/FrontierClusterMsg.h"
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
// 私有库
#include "MeshTable.hpp"
#include "CorridorBuilder.h"
#include "PoseGraph.h"
#include <mapping/mapping.h>


using namespace Eigen;
using namespace std;
class FrontierClusterInfo;

// 前沿用三角形表示
struct Triangle {
    // 利用FASTLAB网格初始化
    Triangle(FastLab::Triangle& mesh, Vector3d& keypose, const double is_large_view_ang_thr){
        _a << mesh._a.x, mesh._a.y, mesh._a.z;
        _b << mesh._b.x, mesh._b.y, mesh._b.z;
        _c << mesh._c.x, mesh._c.y, mesh._c.z;
        _a += keypose;
        _b += keypose;
        _c += keypose;
        _center = (_a+_b+_c)/3.0;
        shrink(keypose);
        _center = (_a+_b+_c)/3.0;

        _normal = mesh._normal;

        _is_large_view_angle = isMeshLargeViewAng(keypose, is_large_view_ang_thr);
    }
    // 利用ROS带的多边形消息初始化
    Triangle(geometry_msgs::Polygon& mesh){
        _a << mesh.points[0].x, mesh.points[0].y, mesh.points[0].z;
        _b << mesh.points[1].x, mesh.points[1].y, mesh.points[1].z;
        _c << mesh.points[2].x, mesh.points[2].y, mesh.points[2].z;
        _center = (_a+_b+_c)/3.0;
        _normal << mesh.points[3].x, mesh.points[3].y, mesh.points[3].z;
    }
    // 缩放
    void shrink(Vector3d& keypose)
    {
            _a = _a + 0.2*((_center-_a).normalized());
            _b = _b + 0.2*((_center-_b).normalized());
            _c = _c + 0.2*((_center-_c).normalized());
    }
    // 大视角判定
    bool isMeshLargeViewAng(Vector3d& keypose, const double is_large_view_ang_thr)
    {
        // if the mesh's view angle is large -> frontier
        Eigen::Vector3d dir1 = (keypose - _center).normalized();
        Eigen::Vector3d dir2 = _normal;

        Eigen::Vector2d dir1_2d = dir1.head(2).normalized();
        Eigen::Vector2d dir2_2d = dir2.head(2).normalized();

        //TODO the x-y view angle and z view angle should have different threshold
        double angle = acos(dir1_2d.dot(dir2_2d));
        return (angle > is_large_view_ang_thr);
    }

    Vector3d _a;
    Vector3d _b;
    Vector3d _c;
    Vector3d _center;
    Vector3d _normal;
    bool _is_large_view_angle;
};
typedef std::vector<Triangle> CellList;

//! class FreeSpace ---------------------------------------------------------------------------
// The Info of FreeSpace
// You can query if a point in it
class FreeSpace{
public:
    FreeSpace();
    void get_convexhull(std::vector<Vector3d>& v, std::vector<size_t>& inxs);
    void create_convexhull(std::vector<Vector3d>& v, std::vector<size_t>& inxs);
    double is_in_freesapce(Vector3d p_query);
    void createMeshTable();

    // The view point corresponds to the freespace
    Vector3d key_pose;
    // The maxrange & barrier points to generate Convex
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_points;
    // ConvexHull
    std::vector<FastLab::Triangle> meshes;
    std::vector<bool> mesh_is_frontier;
    // Mesh Table for quick query if a point is in a ConvexHull
    FastLab::MeshTable mesh_table;
    // If the Mesh Table is generated
    bool is_convex_calculated = false;
};
//! ---------------------------------------------------------------------------


//! class FreeSpaceInfo ---------------------------------------------------------------------------
// Including FreeSpace Buffer & KeyPose Buffer
class FreeSpaceInfo{
public:
    FreeSpaceInfo(){
        posegraph.reset(new PoseGraph);
    }
    
    FreeSpaceInfo(const FreeSpaceInfo& fs_info)
    {
        free_space_list = fs_info.free_space_list;
        posegraph = fs_info.posegraph;
    }

    bool isPointInFreeSpace(Vector3d& searchPoint, double search_R);
    bool isPointsInFreeSpace(vector<Vector3d>& searchPoints, double search_R);

    vector<FreeSpace> free_space_list;
    PoseGraph::Ptr posegraph;
    int last_meet_inx = -1;
};
//! ---------------------------------------------------------------------------


//! class FreeSpaceFactory ---------------------------------------------------------------------------
// FreeSpaceFactory Functions:
// 1. generate FreeSpace in a specific pose (need set map_ptr)
// 2. pub freeSapce for communication
// 3. pub freeSapce for visulization
// 4. Listen Others' freespace and update buffer (need set free_space_info_others_ptr) 
// 5. Tailor self's frontiercluster using Others' freespace (need set fc_info_ptr)
// 6. Lable Frontier Surface
class FreeSpaceFactory{
public:
    FreeSpaceFactory(ros::NodeHandle& nh, std::shared_ptr<FrontierClusterInfo> fc_info_ptr);

    std::shared_ptr<mapping::OccGridMap> grid_map_;
    std::shared_ptr<mapping::OccGridMap> grid_inf_map_;
    Vector3d map_size;

    void setMap(const std::shared_ptr<mapping::OccGridMap> &grid_map,
                const std::shared_ptr<mapping::OccGridMap> &grid_inf_map)
    {
        this->grid_map_     = grid_map;
        this->grid_inf_map_ = grid_inf_map;
        map_size = grid_map_->global_map_size;
    }

    std::shared_ptr<FrontierClusterInfo> fc_info_ptr_;
    FreeSpace genFreeSpcePoints(Vector3d key_pose);
    void genConvexPointOnR(vector<Vector3d>& convex_points_vec, double set_R, int before_size, Vector3d key_pose);
    void extendPoints(vector<Vector3d>& point_vec, Vector3d p1, Vector3d p2, Vector3d keypose, double sensorrange);
    void deleteInvalidMeshes(FreeSpaceInfo& free_space_info_input, FreeSpace& fs, Vector3d& keypose);

    // void HostFrontierFix();
    void publishStarCvx( FreeSpace& fs,Vector3d key_pose,
                     ros::Time msg_time, std::string msg_frame_id );
    void publishStarCvxFrontier( FreeSpace &fs,Vector3d key_pose,
                     ros::Time msg_time, std::string msg_frame_id );
    void publishStarCvxAll( FreeSpaceInfo& freespace_info_in,Vector3d key_pose,
                        ros::Time msg_time, std::string msg_frame_id );

    void LableFrontierSurface(FreeSpace& fs, std::vector<Vector3d>& v, std::vector<size_t>& inxs, Vector3d& keypose,
                              CellList& frontier_cell_set);
    bool isFrontierEdge(Vector3d &ep1, Vector3d &ep2,Vector3d &keypose);
    // true = not occ
    bool checkMeshOcc(Triangle& tri); 
    bool checkMeshSize(Triangle& tri);

    typedef std::shared_ptr<FreeSpaceFactory> Ptr;
    ros::NodeHandle& nh;
    ros::Subscriber host_sub;
    ros::Publisher star_cvx_plane_pub_, star_cvx_edge_pub_, star_cvx_edge_all_pub_;
    ros::Publisher star_cvx_f_plane_pub_, star_cvx_f_edge_pub_;
    ros::Publisher free_space_pub;

    // mesh para
    double fmesh_scale_thr;
    double fmesh_area_thr;
    double fmesh_view_angle_thr;
    double frontier_margin;
    double fmesh_raycast_dis;

    // freespace para
    double sensor_range;
    double d_theta;
    double res;
    double res_z;
    int drone_id;
    int drone_num;
    double d_thr;
    double theta_thr;
    double frontier_roof, frontier_floor;
    double frontier_delta_roof, frontier_delta_floor;
    double lable_frontier_normal_thr;
    double ftr_dis_thr_from_sense_range;
};
//! ---------------------------------------------------------------------------



#endif