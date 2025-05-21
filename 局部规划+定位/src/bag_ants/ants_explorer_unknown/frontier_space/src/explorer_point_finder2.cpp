#include "explorer_point_finder2.h"

// namespace UnknownEnvironmentAnalysis{

PointCloud2TargetPointServer::PointCloud2TargetPointServer(ros::NodeHandle& nh, FrontierClusterInfo::Ptr fc_info_ptr) 
    : nh_(nh), fc_info_ptr_(fc_info_ptr) {
    // 外部参数读取
    nh_.param("unknown_environment_analysis/test_mode", testMode, false);
    nh_.param("unknown_environment_analysis/mainModeRunFrequency", mainModeRunFrequency, 0.01);
    nh_.param("unknown_environment_analysis/platform_type", platform_type_int, 0);
    platform_type = static_cast<PlatformType>(platform_type_int);
    nh_.param("unknown_environment_analysis/sensor_type", sensor_type_int, 0);
    sensor_type = static_cast<SensorType>(sensor_type_int);
    nh_.param("unknown_environment_analysis/run_mode", run_mode_int, 1);
    run_mode = static_cast<RunningMode>(run_mode_int);
    // nh_.param("unknown_environment_analysis/sys_mode", sys_mode_int, -1);
    // sys_mode = static_cast<SysState>(sys_mode_int);

    // 内部变量初始化
    sys_state_now = SysState::OFF;
    sys_state_now = SysState::ON;  // 调试用
    state_str_ = { "OFF", "ON", "FINISH" };
    // fc_info_ptr_.reset(new FrontierClusterInfo(nh_));
    first_cal = true;

    // ROS通信组件函数初始化
    // 1、模块运行定时器
    main_mode_timer_ = 
        nh_.createTimer(ros::Duration(mainModeRunFrequency), &PointCloud2TargetPointServer::mainModeRunOnce, this);

    // 2、目标点发布 or 目标列表发布
    target_point_pub_ = 
        nh_.advertise<geometry_msgs::PointStamped>("/target_point", 10);

    // 3、订阅
    trigger_sub_ = 
        nh_.subscribe("/unknown_environment_analysis/trigger", 10, &PointCloud2TargetPointServer::triggerCallback, this);
    finish_sigual_sub_ = 
        nh_.subscribe("/unknown_environment_analysis/finish_sigual", 10, &PointCloud2TargetPointServer::finishSigualCallback, this);
    // TODO: 服务器模式

}

/* 1、各个回调函数*************************************************************************************/
void PointCloud2TargetPointServer::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    ROS_INFO("PointCloud2TargetPointServer::triggerCallback()");
    if(sys_state_now != SysState::OFF) return;
    ROS_INFO("The system is triggered on!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    transitState(SysState::ON);
    // 记录起始时间
}

void PointCloud2TargetPointServer::finishSigualCallback(const std_msgs::EmptyConstPtr& msg) {
    ROS_INFO("PointCloud2TargetPointServer::finishSigualCallback()");
    transitState(SysState::FINISH);
}

/* 2、主功能函数*************************************************************************************/
void PointCloud2TargetPointServer::mainModeRunOnce(const ros::TimerEvent& e) {
    ROS_INFO("PointCloud2TargetPointServer::mainModeRunOnce() is on");

    if (sys_state_now == OFF)
    {
        ROS_INFO_THROTTLE(10.0, "\033[1;31mWait for Trigger!\033[0m");
        return;
    }

    if (sys_state_now == FINISH)
    {
        ROS_INFO_THROTTLE(10.0, "\033[1;31mFinish!\033[0m");
        return;
    }

    if (sys_state_now == ON)
    {
        cout<<"------------------------run once-----------------------------"<<endl;
        ROS_INFO_STREAM("[Ftr_Server] State: " << state_str_[(int)sys_state_now]);
    }
    if(run_mode == RunningMode::WithMap)
        fc_info_ptr_->updateFrontierSpace();
    else if(run_mode == RunningMode::WithoutMap) 
        fc_info_ptr_->updateFrontierSpace();
    else 
        ROS_ERROR("Unknown running mode!");

    cout<<"------------------------FrontierSpace updated-----------------------------"<<endl;
    

    // 选择目标点
    if(fc_info_ptr_->svp_list.size() < 1) {
        ROS_WARN("No SVP found!!!!!!!!!!!!!!!!!!!!!!!!!");
        return;
    }

    // return;

    if(testMode) {
        publishTargetPointList();
    } else {
        SuperViewPoint best_svp_last = best_svp;
        best_svp = bestSVPSelect(best_next_pose);
        // 选择后的处理：1、无最佳视点--一定的判断后转入OFF；2、发布目标点，但是还是之前的‘3、发布目标点，但是改变了。
        if(best_svp.id == -1){
            ROS_ERROR_STREAM("classFind Fail! cnt: " << classic_fail_cnt_ << "try again!");
            classic_fail_cnt_++;
        } else {
            bool fc_changed = true;
            double dis = (Vector3d(best_svp_last.super_vp.x, best_svp_last.super_vp.y, best_svp_last.super_vp.z) - 
                Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z)).norm();
            if(dis < 0.1) {
                fc_changed = false;
                ROS_ERROR_STREAM("lower than change dis");
            }
            if (!fc_changed) {
                publishNextAim(ros::Time::now(), local_aim);
                cout << "pub local aim: " << local_aim.transpose() << endl;
                last_pub_time = ros_time;
                break;
            } else is_wait_planner_succ_ = false;
        }
        classic_fail_cnt_ = 0;
    }
    
    ROS_INFO("PointCloud2TargetPointServer::mainModeRunOnce() is over");
}

/* 3、子功能函数*************************************************************************************/
SuperViewPoint PointCloud2TargetPointServer::bestSVPSelect(Vector3d& best_next_pose) {
    cout<<"------------------------bestSVPSelect-----------------------------"<<endl;
    // 获取当前位置
    Vector3d c_pos = fc_info_ptr_->drone_pose;
    Vector3d c_vel = fc_info_ptr_->drone_vel;

    Vector3d drone_p_cur(fc_info_ptr_->drone_pose(0),fc_info_ptr_->drone_pose(1),fc_info_ptr_->drone_pose(2));
    Vector3d drone_v_cur(c_vel(0),c_vel(1),c_vel(2));
    Vector3d v_norm = drone_v_cur.normalized();

    double min_score = 99999;
    Vector3d best_aim;
    SuperViewPoint best_sv;
    double score;

    bool is_far_mode = false;
    bool is_room_mode = false;

    auto isInRoom = [](double x, double y)->bool
    {
        return (y < -9.5 && x < 0.0);
    };

    Vector3d drone_aim_vel_last = Vector3d(aim_vel(0), aim_vel(1), aim_vel(2));
    if(fc_info_ptr_->svp_list.size() <= 0){
        ROS_ERROR_STREAM("svp_list empty!");
    }
    else if(fc_info_ptr_->svp_list.size() <= 7 &&
            !(isInRoom(c_pos.x(), c_pos.y())))
    {
        for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
        {
            Vector3d pos;
            pos << it->super_vp.x, it->super_vp.y, it->super_vp.z;
            if ((c_pos - pos).norm() < 15.0)
            {
                is_far_mode = false;
                break;
            }
        }
        ROS_ERROR_STREAM("far_mode!");
        is_far_mode = true;
    }
    else if (isInRoom(c_pos.x(), c_pos.y()))
    {
        for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
        {
            Vector3d pos;
            pos << it->super_vp.x, it->super_vp.y, it->super_vp.z;
            if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(pos)) continue;
            if (isInRoom(it->super_vp.x, it->super_vp.y))
            {
                is_room_mode = true;
                ROS_ERROR_STREAM("room_mode!");
                break;
            }
        }
    }

    for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
    {
        Vector3d pos;
        pos << it->super_vp.x, it->super_vp.y, it->super_vp.z;
        // if(pos(0)==0.0 && pos(1)==0.0 && pos(2)==0.0) continue;
        if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(pos)) continue;
        if (is_room_mode && !isInRoom(it->super_vp.x, it->super_vp.y)) continue;

        Vector3d drone2frontier = pos - fc_info_ptr_->drone_pose;
        // cout<<"SVP-keyposeInx: "<<it->keypose_inx<<" ,in all of "<<MPG->posegraphes[0].getSize()<<endl;
        // cout<<"[" << it->id << "] center: "<<pos.transpose()<<", drone2frontier: "<<drone2frontier.transpose()<<endl;
        // double cur_dis = drone2frontier.norm();
        double cur_dis = calConcretDis(*it);

        double d_theta = acos(c_vel.normalized().dot(drone2frontier.normalized())) * 180.0 / M_PI; //0~180
        double d_theta_from_last = acos(drone_aim_vel_last.normalized().dot(drone2frontier.normalized())) * 180.0 / M_PI; //0~180
        // cout<<"d_theta_from_last: "<<d_theta_from_last<<endl;

        // if(cur_dis < 3.0) continue; //TODO 

        if(cur_dis < fc_info_ptr_->free_space_fac_->sensor_range){
            // score = cur_dis;
            // score = cur_dis + (1.0/180.0) * d_theta / (cur_dis*cur_dis) + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
            // score = cur_dis + (1.0/30.0) * d_theta_from_last + (1.0/30.0) * d_theta; 
            // score = cur_dis + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
            score = cur_dis + (1.0/1) * d_theta_from_last / (80.0);

        }else{
            // score = cur_dis + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
            score = cur_dis + (1.0/1) * d_theta_from_last / (80.0);
            // score = cur_dis;// + (1.0/30.0) * d_theta_from_last;
        }

        if (is_far_mode)
        {
            double dis_2_home = (pos - home_position_).norm();
            score = 100.0 - dis_2_home;
        }

        // // near boundary
        // Eigen::Vector3d min_bound = fc_info_ptr_->grid_map_->getMinBound();
        // Eigen::Vector3d max_bound = fc_info_ptr_->grid_map_->getMaxBound();
        // if (abs(pos.x() - min_bound.x()) < 10.0 ||
        //     abs(pos.y() - min_bound.y()) < 10.0 ||
        //     abs(pos.x() - max_bound.x()) < 10.0 ||
        //     abs(pos.y() - max_bound.y()) < 10.0 )
        // {
        //     score -= 10.0;
        // }


        ROS_ASSERT(score > 0 && score < 1000);
        // cout<<"score: "<<score<<endl;
        // cout<<"dis | angle(ang_score) : "<< cur_dis << ", " << d_theta_from_last << "(" << d_theta_from_last / (36.0) << ")" <<endl;
        // cout<<endl;
        if(score < min_score){
            best_sv = *it; 
            min_score = score;
        }
    }


    if(min_score < 99999)
    {
        path_res.clear();
        getPathtoGoal(best_sv, drone_p_cur, path_res);
        fc_info_ptr_->vis_ptr->visualize_path(path_res, "graph_path");
        res_aimpos = best_sv.getPoseForPlan();

        res_aimvel = calResVel(res_aimpos, drone_p_cur);

        cout<<"vmax: "<<v_max<<endl;
        cout<<"res_aimpos: "<<res_aimpos.transpose()<<endl;
        cout<<"res_aimvel: "<<res_aimvel.transpose()<<endl;
        return best_sv;
    }
    best_sv.id = -1;
    return best_sv;
}

// void PointCloud2TargetPointServer::publishTargetPointList() {
//     common_msg::TargetPointList points_list;
//     geometry_msgs::Point point_cur;
//     points_list.targetPoints.clear();
//     for(list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++) {
//         point_cur.x = it->super_vp.x;
//         point_cur.y = it->super_vp.y;
//         point_cur.z = it->super_vp.z;
//         points_list.targetPoints.push_back(point_cur);
//     }

//     target_point_list_pub_.publish(points_list);
// }

/* 4、工具函数*************************************************************************************/
inline void PointCloud2TargetPointServer::transitState(const SysState& new_state)
{
    SysState cur_state = sys_state_now;
    sys_state_now = new_state;
    cout << "[PointCloud2TargetPointServer] from " 
            + state_str_[(int)cur_state] + " to " 
            + state_str_[(int)new_state]
    << endl;
}

// };