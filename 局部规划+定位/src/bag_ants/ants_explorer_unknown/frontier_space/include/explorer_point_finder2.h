/*
 * @Author: Chenyang Zhang && 1727326672@qq.com
 * @Date: 2024-03-05 14:59:50
 * @LastEditors: Chenyang Zhang && 1727326672@qq.com
 * @LastEditTime: 2024-03-17 16:26:29
 * @FilePath: /bit_zcy_001/src/bag_ants/ants_explorer_unknown/frontier_space/include/explorer_point_finder2.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#pragma once

// 私有库
#include "Cluster.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

// namespace UnknownEnvironmentAnalysis{

enum PlatformType {
    UGV,
    UAV,
    USV
};

enum SensorType {
    LASER,
    CAMERA
};

enum RunningMode {
    WithMap,
    WithoutMap
};

enum SysState {
    OFF,
    ON,
    FINISH
};

class PointCloud2TargetPointServer{
public:
    // 构造与析构
    PointCloud2TargetPointServer(ros::NodeHandle& nh, FrontierClusterInfo::Ptr fc_info_ptr);
    ~PointCloud2TargetPointServer() {}

    /*外部参数列表**********************************************/ 
    bool testMode; // if == true, 为单元模块测试; if == false, 配合系统共同运行
    double mainModeRunFrequency;
    int platform_type_int, sensor_type_int, run_mode_int, sys_mode_int;
    PlatformType platform_type; // 平台类型，0 -> 无人车; 1 -> 无人机; 2 -> 无人船。（目前仅开发0）
    SensorType sensor_type; // 传感器类型，0 -> 激光; 1 -> 相机。（目前仅开发0）
    RunningMode run_mode;
    // SysState sys_mode;
    // 模式1相关参数

    // 模式2相关参数

    /*程序变量列表**********************************************/ 
    SysState sys_state_now;
    std::vector<string> state_str_;
    SuperViewPoint best_svp;
    Vector3d best_next_pose, best_next_vel;
    bool first_cal;
    int classic_fail_cnt_;

    /*自身及子类指针**********************************************/ 
    typedef std::shared_ptr<PointCloud2TargetPointServer> Ptr;
    FrontierClusterInfo::Ptr fc_info_ptr_;

    /*ROS通信组件函数 及订阅函数对应的回调函数**********************************************/ 
    ros::NodeHandle nh_;
    ros::Timer main_mode_timer_;
    ros::Publisher target_point_pub_, target_point_list_pub_;
    ros::Subscriber trigger_sub_, finish_sigual_sub_;

    void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void finishSigualCallback(const std_msgs::EmptyConstPtr& msg);

    /*功能函数**********************************************/ 
    void mainModeRunOnce(const ros::TimerEvent& e);
    SuperViewPoint bestSVPSelect(Vector3d& best_next_pose);
    void publishTargetPointList();
    void publishNextAim(const ros::Time& rostime, const Vector3d aim_next);

    /*工具函数**********************************************/ 
    inline void transitState(const SysState& new_state);

};


// };
