# include "explorer_point_finder.h"

FrontierServer::FrontierServer(ros::NodeHandle& nh, FrontierClusterInfo::Ptr fc_info_ptr):nh(nh), fc_info_ptr_(fc_info_ptr){
    nh.param("frontier_node/max_vel", v_max, -1.0);
    nh.param("frontier_node/classic_pub_interval", classic_pub_interval, -1.0);
    nh.param("frontier_node/rapid_pub_interval", rapid_pub_interval, -1.0);
    nh.param("frontier_node/arrive_goal_dis_thres", arrive_goal_dis_thres_, -1.0);
    nh.param("frontier_node/arrive_cruise_goal_dis_thres", arrive_cruise_goal_dis_thres_, -1.0);
    nh.param("frontier_node/arrive_local_goal_dis_thres", arrive_local_goal_dis_thres_, -1.0);
    nh.param("map/init_x", _init_x,  0.0);
    nh.param("map/init_y", _init_y,  0.0);
    nh.param("map/init_z", _init_z,  1.0);
    
    nh.param("fsm/explore_start_x", explore_start_position_[0], 0.0);
    nh.param("fsm/explore_start_y", explore_start_position_[1], 0.0);
    nh.param("fsm/explore_start_z", explore_start_position_[2], 0.0);

    nh.param("fsm/uwb_anchor_x", uwb_anchor_position_[0], 0.0);
    nh.param("fsm/uwb_anchor_y", uwb_anchor_position_[1], 0.0);
    nh.param("fsm/uwb_anchor_z", uwb_anchor_position_[2], 0.0);

    home_position_ << 0.0, 0.0, 1.0;
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    if (waypoint_num_ <= 0)
    {
        ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
    }
    wps_.resize(waypoint_num_ + 1);
    wps_[0] = home_position_;
    for (int i = 1; i < waypoint_num_ + 1; i++)
    {
        wps_[i](0) = waypoints_[i-1][0];
        wps_[i](1) = waypoints_[i-1][1];
        wps_[i](2) = waypoints_[i-1][2];
    }

    nh.param("is_on_car", is_on_car, false);
    m_nextAimPub = nh.advertise<nav_msgs::Odometry>("/next_aim", 1);
    m_replanPub  = nh.advertise<std_msgs::Int8>("/replan", 1);

    trigger_sub_ = nh.subscribe("trigger", 1, &FrontierServer::triggerCallback, this);
    // m_PlanSuccessSub = nh.subscribe("/frontier_node/planning/plan_success", 1, &FrontierServer::planSuccessCallBack, this);
    // m_PlanRequireSub = nh.subscribe("/frontier_node/planning/plan_require", 1, &FrontierServer::planRequireCallBack, this);
    m_PlanFailSub = nh.subscribe("/plan_fail_times", 1, &FrontierServer::planFailCallBack, this);
    finish_sub = nh.subscribe("/finish_signal", 1, &FrontierServer::finishCallBack, this);

    state_str_ = { "OFF", "TOSTART", "RAPID", "CLASSIC", "GOHOME", "GOSAFEPOINT", "APPROACH", "CRUISE", "FINISH" };
    m_currentState = ExplorationState::OFF;
    pub_log_.reset(new PubTicToc(1.5));



    safe_point_list.push_back(Vector3d(0,0,1));
    safe_point_list.push_back(Vector3d(-2,0,1));
    safe_point_list.push_back(Vector3d(2,0,1));

    MPG.reset(new MultiPoseGraph);
    MPG->main_inx = 0;

    last_pub_time = ros::Time::now();

    ros::Duration(1.0).sleep();

    frontier_timer_ = nh.createTimer(ros::Duration(0.05), &FrontierServer::runonce, this);
}

void FrontierServer::triggerCallback(const geometry_msgs::PoseStampedConstPtr msg) {
    ROS_ERROR_STREAM("triggerCallback");
    if (m_currentState != ExplorationState::OFF) return;
    trigger_ = true;
    ROS_WARN_STREAM("Triggered!!!!!!!!!!!!!!!!!!!!!!!!!!");
    // xulong add to start
    last_pub_time = ros::Time(ros::Time::now().toSec() - 10.0);
    transitState(TOSTART);
    // transitState(CLASSIC);
    start_time = ros::Time::now().toSec();
}

void FrontierServer::finishCallBack(const std_msgs::Int8 msg)
{
    ROS_ERROR_STREAM("Get Finish Signal!");
    need_finish = true;
    return;
}

void FrontierServer::planFailCallBack(const std_msgs::Int64 msg){
    plan_fail_time = msg.data;
    if (plan_fail_time < 1) is_wait_planner_succ_ = false;
}

// void FrontierServer::planSuccessCallBack(const std_msgs::Bool msg){
//     if(msg.data == false){
//         plan_fail_time++;
//     }else{
//         plan_fail_time = 0;
//     }
// }

// void FrontierServer::planRequireCallBack(const std_msgs::Bool msg){
//     if(msg.data == true){
//         force_pub_flag = true;
//     }
// }

//! Note! This timer is only for single robot exploration FSM.
//! For multi-robot exploration, please comment this timer!
void FrontierServer::runonce(const ros::TimerEvent& /*event*/)
{    
    if (m_currentState == FINISH)
    {
        ROS_INFO_THROTTLE(1.0, "\033[1;31mFinish!\033[0m");
        return;
    }
    
    if ((m_currentState != APPROACH && m_currentState != OFF && m_currentState != GOHOME && m_currentState != CRUISE) || 
        ((m_currentState == APPROACH || m_currentState == OFF || m_currentState == GOHOME || m_currentState == CRUISE) && pub_log_->canPub()) )
    {
        cout<<"------------------------run once-----------------------------"<<endl;
        ROS_INFO_STREAM("[Ftr_Server] State: " << stateStr(m_currentState));
    }
        

    // if(First_frame_cnt == 0){
    //     start_time = ros::Time::now().toSec();
    //     aim_pose = Vector3d(_init_x + 2.0, _init_y, 1);
    //     aim_vel = Vector3d(0.5, 0, 0);
    //     publishNextAim(ros::Time::now(), aim_pose, aim_vel);
    // }

    if (m_currentState != GOHOME && m_currentState != CRUISE && m_currentState != FINISH) fc_info_ptr_->updateFrontierSpace();
// cout<<fc_info_ptr_->free_space_fac_->grid_map_->recieve_cnt<< ";"<< First_frame_cnt <<endl;
    if(First_frame_cnt++ < 20 || fc_info_ptr_->free_space_fac_->grid_map_->recieve_cnt <= 10){
        aim_pose = Vector3d(0,0,1);
        cout<<"aim_pose01: "<<aim_pose.transpose()<<endl;
        return;
    }

    cout<<fc_info_ptr_->free_space_info.posegraph->getSize()<<endl;

    fc_info_ptr_->free_space_info.posegraph->deepCopy(MPG->posegraphes[0], fc_info_ptr_->free_space_info.posegraph->getSize()-fc_info_ptr_->temp_num);

    cout<<MPG->posegraphes[0].getSize()<<endl;

    double time1 = ros::Time::now().toSec();

    //! Result container
    Vector3d aim_p = Vector3d(aim_pose(0), aim_pose(1), aim_pose(2));
    double dis2goal;

    Vector3d drone_pos_ = fc_info_ptr_->drone_pose;
    vector<Vector3d> classic_aim;

    Vector3d safe_pose;

    frontierInFOVScore.clear();

    //! check aim occ
    // if(!map_server->isFree(aim_p)){
    //     force_pub_flag = true;
    // }

    ros::Time ros_time = ros::Time::now();

    // bool pub_next_aim_flag = ((ros_time.toSec() - last_pub_time.toSec())*1000.0 > pub_interval);
    // if(force_pub_flag){
    //     pub_next_aim_flag = true;
    // }
    // if(!pub_next_aim_flag)  return;


    // if(plan_fail_time > 8){
    //     transitState(GOSAFEPOINT);
    //     ROS_ERROR_STREAM("[Ftr_Server] PLAN FAIL Time: " << plan_fail_time << ", PLAN FAIL!!!!!!!!!!!!!!!");
    // }else if (force_pub_flag){
    //     transitState(CLASSIC);
    // }
    force_pub_flag = false;

    if(plan_fail_time >= 1)
    {
        ROS_ERROR_STREAM("[Ftr_Server] PLAN FAIL Time: " << plan_fail_time << ", PLAN FAIL!!!!!!!!!!!!!!!");
    }

    if(plan_fail_time >= 1 && !fc_info_ptr_->grid_map_->getInflateOccupancy(drone_pos_) && 
       m_currentState != CRUISE && m_currentState != GOHOME && m_currentState != TOSTART)
    {
        ROS_ERROR_STREAM("[Ftr_Server] PLAN FAIL Time: " << plan_fail_time << ", PLAN FAIL!!!!!!!!!!!!!!!");
        if (!is_wait_planner_succ_)
        {
            ROS_ERROR_STREAM("[Ftr_Server] Not wait_planner_succ!");
            for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
            {
                if (best_svp.id == it->id)
                {
                    fc_info_ptr_->svp_list.erase(it);
                    ROS_ERROR_STREAM("[Ftr_Server] erase it!");
                    is_wait_planner_succ_ = true;
                    break;
                }

            }
        }
        else
        {
            ROS_ERROR_STREAM("[Ftr_Server] wait_planner_succ!");
        }
        transitState(CLASSIC);
    }

    switch (m_currentState)
    {

    case FINISH:
    {
        ROS_INFO_THROTTLE(1.0, "\033[1;31mFinish!\033[0m");
        break;
    }

    case TOSTART:
    {
        ROS_INFO("\033[1;31mTOSTART Mode Active!\033[0m");  //红
        double dis2localaim = (explore_start_position_ - fc_info_ptr_->drone_pose).head(2).norm();
        if(dis2localaim < arrive_cruise_goal_dis_thres_)
        {
            ROS_INFO("\033[1;31mTOSTART Done! -  %f s\033[0m",(ros::Time::now().toSec()-start_time));  //红
            transitState(CLASSIC);
            break;
        }

        if ((ros_time - last_pub_time).toSec() > 10.0)
        {
            publishNextAim(ros_time, explore_start_position_, Vector3d::Zero());
            cout << "[TOSTART] pub start aim: " << explore_start_position_.transpose() << endl;
            last_pub_time = ros_time;
        }
        break;
    }

    case CLASSIC:
    {
        ROS_INFO("\033[1;31mClassic Mode Active!\033[0m");  //红

        // Classic Search
        vector<Vector3d> path_res_tmp = path_res;
        path_res.clear();
        SuperViewPoint best_svp_last = best_svp;
        best_svp = classicFronitierFind(aim_pose, aim_vel); // will fill the path_res

        // Plan go home
        // test cruise only
        // if(best_svp.id == -1 || true){
            
        // classical
        if(best_svp.id == -1){
            ROS_ERROR_STREAM("classFind Fail! cnt: " << classic_fail_cnt_);
            classic_fail_cnt_++;
            if (classic_fail_cnt_ < 5) break;
            transitState(GOHOME);
            path_res.clear();
            bool plan_home_success = false;
            if((fc_info_ptr_->drone_pose - home_position_).head(2).norm() > 0.1){
                aim_pose = home_position_;
                local_aim = aim_pose;
                if (!fc_info_ptr_->grid_map_->getInflateOccupancy(aim_pose)) 
                {
                    getPathHome(home_position_, fc_info_ptr_->drone_pose, path_res);
                    plan_home_success = true;
                    ROS_ERROR("plan_home_success!");
                }
            }
            if (!plan_home_success) break;
        }
        else
        {
            bool fc_changed = true;
            double dis = (Vector3d(best_svp_last.super_vp.x, best_svp_last.super_vp.y, best_svp_last.super_vp.z) - 
                Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z)).norm();
            if(dis < 0.1)
            {
                fc_changed = false;
                ROS_ERROR_STREAM("lower than change dis");
            }
            if (!fc_changed)
            {
                path_res = path_res_tmp;
                publishNextAim(ros_time, local_aim, local_vel);
                cout << "pub local aim: " << local_aim.transpose() << endl;
                last_pub_time = ros_time;
                transitState(APPROACH);
                break;
            }
            else is_wait_planner_succ_ = false;
        }


        cout<<"classic find: "<<aim_pose.transpose()<<endl;
        classic_fail_cnt_ = 0;

        // Choose local goal
        if(path_res.size() <= 2){   // directly aim to the svp
            // fc_info_ptr_->refineSuperPoint(best_svp);
            // aim_pose = best_svp.refined_svp;
            local_aim = aim_pose;
            aim_vel = calResVel(aim_pose, fc_info_ptr_->drone_pose);
            if (m_currentState == GOHOME) aim_vel = Vector3d(0, 0, 0);
            if (fc_info_ptr_->grid_map_->getInflateOccupancy(aim_pose))
            {
                ROS_INFO("\033[1;31m[Ftr_Server] Aim Occ, Replan!\033[0m");  //红
                ROS_INFO_STREAM("aim occ: " << aim_pose.transpose());
                transitState(CLASSIC);
                break;
            }
            publishNextAim(ros_time, aim_pose, aim_vel);
            cout << "pub aim: " << aim_pose.transpose() << endl;
            dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
            last_pub_time = ros_time;
        }else{
            path_inx = 1;
            local_aim = getLocalAim();
            local_vel = Vector3d::Zero();
            if (fc_info_ptr_->grid_map_->getInflateOccupancy(local_aim))
            {
                ROS_INFO("\033[1;31m[Ftr_Server] Aim Occ, Replan!\033[0m");  //红
                ROS_INFO_STREAM("aim occ: " << local_aim.transpose());
                transitState(CLASSIC);
                break;
            }
            publishNextAim(ros_time, local_aim, local_vel);
            cout << "pub local aim: " << local_aim.transpose() << endl;
            dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
            dis2localaim = (local_aim-fc_info_ptr_->drone_pose).norm();
            last_pub_time = ros_time;
        }
        if (m_currentState != GOHOME)
            transitState(APPROACH);
        break;
    }

    
    case GOHOME:
    {
        double dis_2_home = (fc_info_ptr_->drone_pose - home_position_).head(2).norm(); 
        // if (pub_log_->canPub()) 
            cout << "dis_2_home: "<< dis_2_home << endl;
        
        if(dis_2_home < arrive_cruise_goal_dis_thres_)
        {
            ROS_INFO("\033[1;31mExploration Done! -  %f s\033[0m",(ros::Time::now().toSec()-start_time));  //红
            transitState(CRUISE);
            std_msgs::Int8 msg;
            msg.data = 1;
            m_replanPub.publish(msg);
            path_inx = 0;
            break;
        }

        // if (pub_log_->canPub()) 
            ROS_INFO("\033[1;31mGo Home!\033[0m");  //红

        if(path_res.size() > 2){
            double dis2localaim = (local_aim - fc_info_ptr_->drone_pose).norm();
            if(dis2localaim < arrive_local_goal_dis_thres_){
                local_aim = getLocalAim();
                if(path_inx > path_res.size()){
                    ROS_ERROR_STREAM("path size out!");
                    break;
                }
                local_vel = Vector3d::Zero();
                publishNextAim(ros_time, local_aim, local_vel);
                cout << "pub local aim: " << local_aim.transpose() << endl;
                last_pub_time = ros_time;
            }
        }

        last_pub_time = ros_time;
        break;
    }


    case APPROACH:
    {
        dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
        if (pub_log_->canPub()) ROS_INFO("\033[1;33mApproach...\033[0m");  //黄
        double dis2goal2d = sqrt(pow(aim_pose(0)-fc_info_ptr_->drone_pose(0),2) + pow(aim_pose(1)-fc_info_ptr_->drone_pose(1),2));
        double dis2localaim = (local_aim - fc_info_ptr_->drone_pose).norm();

        // bool isMoving = true; 
        // last_drone_pose_.push_back(fc_info_ptr_->drone_pose);
        // if(last_drone_pose_.size() > 10) last_drone_pose_.pop_front();
        // if((fc_info_ptr_->drone_pose - last_drone_pose_.front()).norm() < 0.01){
        //     isMoving = false;
        // }

        if (pub_log_->canPub())
        {
            cout<<"Dis to Aim: "<<dis2goal2d<<endl;
            cout<<"Dis to LocalAim: "<<dis2localaim<<endl;
            cout << "path size: " << path_res.size() <<endl;
            cout << "path_inx: " << path_inx <<endl;
        } 

        // 目标fc是否变化
        bool fc_changed = false;
        //todo
        SuperViewPoint best_svp_now = fc_info_ptr_->getSVP(best_svp.id);
        if(best_svp_now.id == -1){
            fc_changed = true;
            // cout<<"changed no id"<<endl;
        }else{
            // cout<<"best_svp old: "<<Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z).transpose()<<endl;
            // cout<<"best_svp new: "<<Vector3d(best_svp_now.super_vp.x, best_svp_now.super_vp.y, best_svp_now.super_vp.z).transpose()<<endl;
            double dis = (Vector3d(best_svp_now.super_vp.x, best_svp_now.super_vp.y, best_svp_now.super_vp.z) - 
                Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z)).norm();
            if(dis > 1.0){
                fc_changed = true;
                // cout<<"changed dis"<<endl;
            }
        }

        // Replan Rules
        if (fc_changed){
            ROS_INFO("\033[1;31m[Ftr_Server] Ftr Change, Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }
        if ((ros_time.toSec() - last_pub_time.toSec()) > classic_pub_interval){
            ROS_INFO("\033[1;31m[Ftr_Server] Time to Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }

        if (dis2goal2d < arrive_goal_dis_thres_){ //* arrive classic goal
            ROS_INFO_STREAM("aim_pose: " << aim_pose.transpose() << ", drone_pose: " << fc_info_ptr_->drone_pose.transpose());
            ROS_INFO_STREAM("[Ftr_Server] dis2goal2d: " << dis2goal2d << ", arrive_dis_thres: " << arrive_goal_dis_thres_);
            ROS_INFO("\033[1;31mGet Classic Aim, Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }

        if (fc_info_ptr_->grid_map_->getInflateOccupancy(local_aim))
        {
            ROS_INFO("\033[1;31m[Ftr_Server] Aim Occ, Replan!\033[0m");  //红
            ROS_INFO_STREAM("aim occ: " << local_aim.transpose());
            transitState(CLASSIC);
            break;
        }

        if(path_res.size() > 2){
            double dis2localaim = (local_aim - fc_info_ptr_->drone_pose).norm();
            if(dis2localaim < arrive_local_goal_dis_thres_){
                local_aim = getLocalAim();
                if(path_inx == path_res.size()){
                    ROS_ERROR_STREAM("path size out!");
                }
                local_vel = Vector3d::Zero();

                publishNextAim(ros_time, local_aim, local_vel);
                cout << "pub local aim: " << local_aim.transpose() << endl;
                last_pub_time = ros_time;
            }
        }
        break;
    }

    case CRUISE:
    {
        if (pub_log_->canPub()) 
            ROS_INFO("\033[1;33mCRUISE...\033[0m");  //黄
        double dis2localaim;
        if (is_goal_uwb)
           dis2localaim = (uwb_anchor_position_ - fc_info_ptr_->drone_pose).head(2).norm();
        else
        {
            dis2localaim = (wps_[path_inx] - fc_info_ptr_->drone_pose).head(2).norm();
            if (pub_log_->canPub()) 
                ROS_INFO_STREAM("path_inx: " << path_inx << ", wps_: " << wps_[path_inx].transpose() << ", dis: " << dis2localaim);
        }
           
        // TODO: eliminate hard code for vel_ok
        bool vel_test_ok = false;
        if (fc_info_ptr_->drone_vel.head(2).norm() > 1.6)
        {
            vel_test_ok = true;
            std_msgs::Int8 msg;
            msg.data = 3;
            m_replanPub.publish(msg);
            vel_ok = true;
        }

        double time_diff = (ros_time - last_pub_time).toSec();
        double time_tol = 20.0;

        if(dis2localaim < arrive_cruise_goal_dis_thres_ || time_diff > time_tol)
        {
            path_inx++;
            if (time_diff > time_tol)
                path_inx--;

            if(need_finish && is_goal_uwb)
            {
                ROS_ERROR_STREAM("Finish!");
                transitState(FINISH);
                break;
            }

            if (need_finish && path_inx==2)
            {
                is_goal_uwb = true;
            }
            
            // if (path_inx == 2)
            // {
            //     std_msgs::Int8 msg;
            //     msg.data = 2;
            //     m_replanPub.publish(msg);
            // }
            // if (path_inx == 3)
            // {
            //     std_msgs::Int8 msg;
            //     msg.data = 3;
            //     m_replanPub.publish(msg);
            // }

            // xulong for cruise until 2km
            if(path_inx > waypoint_num_)
            {
                path_inx = 1;
                std_msgs::Int8 msg;
                msg.data = 3;
                m_replanPub.publish(msg);
                vel_ok = true;
            }
            
            if (!vel_ok)
            {
                std_msgs::Int8 msg;
                msg.data = 1;
                m_replanPub.publish(msg);
            }
            
            if (is_goal_uwb)
            {
                local_aim = uwb_anchor_position_;
            }
            else
            {
                local_aim = wps_[path_inx];
            }
            local_vel = Vector3d::Zero();
            publishNextAim(ros_time, local_aim, local_vel);
            cout << "[CRUISE] pub local aim: " << local_aim.transpose() << endl;
            last_pub_time = ros_time;
        }
        break;
    }


    }
    double time2 = ros::Time::now().toSec();
    if ((m_currentState != APPROACH && m_currentState != OFF && m_currentState != GOHOME && m_currentState != CRUISE) || 
        ((m_currentState == APPROACH || m_currentState == OFF || m_currentState == GOHOME || m_currentState == CRUISE) && pub_log_->canPub()) )
        ROS_INFO("\033[1;34m ALL Done - %f ms\033[0m", (time2-time1) * 1000.0);  //蓝
    if ((m_currentState == APPROACH || m_currentState == OFF || m_currentState == GOHOME || m_currentState == CRUISE) && pub_log_->canPub())
        pub_log_->tic();
}

Vector3d FrontierServer::getLocalAim(){
    for(int i = path_res.size()-1; i > path_inx; i--)
    {
        if(fc_info_ptr_->grid_map_->isInMap(path_res[i]) && 
            fc_info_ptr_->isVisible(fc_info_ptr_->drone_pose, path_res[i], fc_info_ptr_->grid_map_->getResolution()) > 0)
        {
            path_inx = i;
            return path_res[i];
        }
    }
    path_inx++;
    int idx = path_inx;
    if (path_inx >= path_res.size()) idx = (int)path_res.size() - 1;
    return path_res[idx];
}

SuperViewPoint FrontierServer::classicFronitierFind(Vector3d& res_aimpos, Vector3d& res_aimvel)
{
    Vector3d c_pos = fc_info_ptr_->drone_pose;
    Vector3d c_vel = fc_info_ptr_->drone_vel;
    if(first_cal){
        first_cal = false;
        aim_vel = Vector3d(1.0,0,0);
    }
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
// The result path contains the start & end points
double FrontierServer::getPathtoGoal(SuperViewPoint& svp, Vector3d& drone_p, vector<Vector3d>& path){
    // cout<<"In getPathtoGoal"<<endl;
    path.clear();
    Vector3d aim_svp_p = svp.getPoseForPlan();

    int aim_inx = svp.keypose_inx;
    int cur_inx = fc_info_ptr_->free_space_info.free_space_list.size() - 1 - fc_info_ptr_->temp_num;
    // If the svp is directly visible and within the sensor_range
    if(fc_info_ptr_->isVisible(drone_p, aim_svp_p, fc_info_ptr_->grid_map_->getResolution()) > 0 &&
         (drone_p-aim_svp_p).norm()<fc_info_ptr_->free_space_fac_->sensor_range)
    {
        path.push_back(aim_svp_p);
        path.push_back(drone_p);
        reverse(path.begin(), path.end());
        cout<<"Visible: "<<fc_info_ptr_->isVisible(drone_p, aim_svp_p, fc_info_ptr_->grid_map_->getResolution())<<", "<<drone_p.transpose()<<"->"<< aim_svp_p.transpose()<<endl;
        return (drone_p - aim_svp_p).norm();
    }
    // Else calculate from posegraph
    double dis =  MPG->calGraphPath(make_pair(0,cur_inx), make_pair(0,aim_inx), path);
    path.insert(path.begin(), drone_p);
    path.push_back(aim_svp_p);
    dis += (path[0]-path[1]).norm();
    dis += (path[path.size()-1]-path[path.size()-2]).norm();
    return dis;
}

double FrontierServer::getPathHome(Vector3d& home_p, Vector3d& drone_p, vector<Vector3d>& path)
{
    path.clear();

    int aim_inx = 0;
    int cur_inx = fc_info_ptr_->free_space_info.free_space_list.size() - 1 - fc_info_ptr_->temp_num;
    // If the svp is directly visible and within the sensor_range
    if(fc_info_ptr_->isVisible(drone_p, home_p, fc_info_ptr_->grid_map_->getResolution()) > 0 &&
         (drone_p-home_p).norm()<fc_info_ptr_->free_space_fac_->sensor_range)
    {
        path.push_back(home_p);
        path.push_back(drone_p);
        reverse(path.begin(), path.end());
        cout<<"Visible: "<<fc_info_ptr_->isVisible(drone_p, home_p, fc_info_ptr_->grid_map_->getResolution())<<", "<<drone_p.transpose()<<"->"<< home_p.transpose()<<endl;
        return (drone_p - home_p).norm();
    }
    // Else calculate from posegraph
    double dis =  MPG->calGraphPath(make_pair(0,cur_inx), make_pair(0,aim_inx), path);
    path.insert(path.begin(), drone_p);
    path.push_back(home_p);
    dis += (path[0]-path[1]).norm();
    dis += (path[path.size()-1]-path[path.size()-2]).norm();
    return dis;
}


// calculate the ditance toward an svp using posegraph
double FrontierServer::calConcretDis(SuperViewPoint& svp){
    Vector3d svp_p(svp.super_vp.x, svp.super_vp.y, svp.super_vp.z);
    // If directly visible, return stright line
    if(fc_info_ptr_->grid_map_->isInMap(svp_p) && fc_info_ptr_->isVisible(svp_p, fc_info_ptr_->drone_pose, 0.0) > 0)
    {
        return ((svp_p - fc_info_ptr_->drone_pose).norm());
    }
    else
    {
        vector<Vector3d> path;
        return getPathtoGoal(svp, fc_info_ptr_->drone_pose, path);
    }
}


Vector3d FrontierServer::calResVel(Vector3d res_aimpos, Vector3d drone_p){
    Vector3d res_aimvel= Vector3d::Zero();
    if((res_aimpos-drone_p).norm() > fc_info_ptr_->free_space_fac_->sensor_range){
        Vector3d diff = res_aimpos-drone_p;
        double max_axis_dis = max(max(abs(diff(0)),abs(diff(1))),abs(diff(2)));
        res_aimvel = diff*(v_max/max_axis_dis);
    }else{
        res_aimvel = (res_aimpos-drone_p)*(v_max/fc_info_ptr_->free_space_fac_->sensor_range);
    }
    return res_aimvel;
}

void FrontierServer::publishNextAim(const ros::Time& rostime, const Vector3d aim_pose, const Vector3d aim_vel){
    nav_msgs::Odometry odom;
    odom.header.stamp = rostime;
    odom.header.frame_id = "world";

    odom.pose.pose.position.x = aim_pose(0);
    odom.pose.pose.position.y = aim_pose(1);
    odom.pose.pose.position.z = aim_pose(2);
    // odom.pose.pose.position.z = 1e-3;


    odom.twist.twist.linear.x = aim_vel(0);
    odom.twist.twist.linear.y = aim_vel(1);
    odom.twist.twist.linear.z = aim_vel(2);
    // odom.twist.twist.linear.z = 0.0;

    if(is_on_car){
        odom.pose.pose.position.z = fc_info_ptr_->car_odom_z;
        odom.twist.twist.linear.z = 0.0;
    }


    if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(Vector3d(aim_pose(0),aim_pose(1),aim_pose(2)))){
        ROS_ERROR_STREAM("Frontier is OCC! "<<aim_pose.transpose());
        return;
    }


    m_nextAimPub.publish(odom);
}




