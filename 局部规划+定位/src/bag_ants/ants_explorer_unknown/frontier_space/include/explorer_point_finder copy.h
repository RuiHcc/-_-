#pragma once
#include <iostream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <random>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

#include "Cluster.h"

class PubTicToc{
 private:
    double last_pub_t_;
    double pub_dt_;
 public:
    PubTicToc(const double& pub_dt):pub_dt_(pub_dt)
    {
        tic();
    }

    typedef std::shared_ptr<PubTicToc> Ptr;

    inline void tic()
    {
        last_pub_t_ = ros::Time::now().toSec();
    }
    inline void tocPub(string& str)
    {
        if (ros::Time::now().toSec() - last_pub_t_ > pub_dt_)
            std::cout << str << std::endl;
    }
    inline bool canPub()
    {
        return (ros::Time::now().toSec() - last_pub_t_ > pub_dt_);
    }
};

enum ExplorationState {
    OFF,
    TOSTART,
    RAPID,
    CLASSIC,
    GOHOME,
    GOSAFEPOINT,
    APPROACH,
    CRUISE,
    FINISH
};


// enum COLOR_VIS {BLACK, PINK_CUBE, LIGHT_BLUE, GRAY, BLUE_CUBE, GREEN_CUBE};

class FrontierServer{
public:
    // 构造与析构
    FrontierServer(ros::NodeHandle& nh, FrontierClusterInfo::Ptr fc_info_ptr);
    ~FrontierServer(){};

    // 外部参数列表
    Vector3d explore_start_position_;
    double v_max; 
    Vector3d uwb_anchor_position_;
    double rapid_pub_interval;//ms
    double classic_pub_interval;//ms
    double arrive_goal_dis_thres_, arrive_local_goal_dis_thres_, arrive_cruise_goal_dis_thres_;
    double _init_x, _init_y, _init_z;
    bool is_on_car;
    double waypoints_[50][3];

    double pub_interval = rapid_pub_interval;

    // 内部变量列表
    Vector3d aim_pose, aim_vel;
    Vector3d home_position_;
    ros::Time last_pub_time;
    SuperViewPoint best_svp;
    bool force_pub_flag = false;
    int plan_fail_time = 0;
    std::vector<Eigen::Vector3d> safe_point_list;
    int safe_point_inx = 0;
    int First_frame_cnt = 0;
    double start_time;
    bool first_cal = true;
    std::vector<Eigen::Vector3d> wps_;
    int waypoint_num_, wpt_id_;
    int classic_fail_cnt_;
    // State
    ExplorationState m_currentState = ExplorationState::OFF;
    std::vector<string> state_str_;
    bool m_explorationToggled = false;
    bool trigger_ = false;

    // xulong shit
    bool need_finish = false;
    bool vel_ok = false;
    bool is_goal_uwb = false;

    // Variables
    vector<Vector3d> path_res;
    int path_inx;
    Vector3d local_aim;
    Vector3d local_vel;
    double dis2localaim;
    std::deque<Eigen::Vector3d> last_drone_pose_;
    vector<double> frontierInFOVScore;
    PubTicToc::Ptr pub_log_;
    bool is_wait_planner_succ_ = false;

    // 指针
    ros::NodeHandle& nh;
    typedef std::shared_ptr<FrontierServer> Ptr;
    FrontierClusterInfo::Ptr fc_info_ptr_;
    MultiPoseGraph::Ptr MPG;

    // ROS相关
    ros::Timer frontier_timer_;
    ros::Publisher m_PointListPub;
    ros::Publisher m_nextAimPub, m_replanPub;
    ros::Publisher m_vis_marker_pub;
    ros::Subscriber m_PlanSuccessSub, m_PlanRequireSub, m_PlanFailSub, trigger_sub_, finish_sub;

    void triggerCallback(const geometry_msgs::PoseStampedConstPtr msg);
    void planFailCallBack(const std_msgs::Int64 msg);
    void finishCallBack(const std_msgs::Int8 msg);

    // 功能函数
    void runonce(const ros::TimerEvent& /*event*/);

    void publishNextAim(const ros::Time& rostime, const Vector3d aim_next, const Vector3d aim_vel);

    double calVCost(Vector3d frontier_point, Vector3d dronep, Vector3d dronev);
    SuperViewPoint classicFronitierFind(Vector3d& res_aimpos, Vector3d& res_aimvel);
    Vector3d getLocalAim();
    Vector3d calResVel(Vector3d res_aimpos, Vector3d drone_p);

    // 工具函数
    inline void transitState(const ExplorationState& new_state);
    inline string stateStr(const ExplorationState state);

    double getPathtoGoal(SuperViewPoint& svp, Vector3d& drone_p, vector<Vector3d>& path);
    double getPathHome(Vector3d& home_p, Vector3d& drone_p, vector<Vector3d>& path);
    double calConcretDis(SuperViewPoint& svp);

    

};

inline string FrontierServer::stateStr(const ExplorationState state)
{
    return state_str_[(int)state];
}

inline void FrontierServer::transitState(const ExplorationState& new_state)
{
    ExplorationState cur_state = m_currentState;
    m_currentState = new_state;
    cout << "[Frtr_Server] from " + stateStr(cur_state) + " to " + stateStr(new_state)
        << endl;
}


inline double FrontierServer::calVCost(Vector3d frontier_point, Vector3d dronep, Vector3d dronev){
    double R = fc_info_ptr_->free_space_fac_->sensor_range;
    Vector3d vi = (frontier_point-dronep)*(v_max/R);
    Vector3d vdiff = vi - dronev;
    double ci = vdiff.norm() / v_max; //0~1

    Vector3d drone2aim = frontier_point - dronep;
    // double dis = drone2aim.norm();
    // ci = dis/R;


    // cout<<"ci: "<<ci<<endl;
    // cout<<"dis: "<<dis<<endl;
    // cout<<"frontier p: "<<frontier_point<<endl;
    // cout<<"drone p: "<<dronep<<endl;
    // cout<<"vi: "<<vi<<endl;
    // cout<<"dronev: "<<dronev<<endl;

    double ctheta = dronev.dot(drone2aim)/(dronev.norm()*drone2aim.norm());
    double pi;
    double pi_max = 3.0;
    // if(dis > R/3.0){
    //     pi = theta/(M_PI_2) * R/dis;
    // }else{
    //     pi = theta/(M_PI_2) * 3.0;
    // }
    // pi = pi/pi_max;//0~1

    // double score = -abs(ctheta);

    double score = ci;



    // return ci - vi.norm()*0.5;
    return score;
}
