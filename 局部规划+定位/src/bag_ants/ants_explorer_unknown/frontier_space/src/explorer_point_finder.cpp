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

    home_position_ << 0.0, 0.0, 1.0;

    nh.param("is_on_car", is_on_car, false);

    // m_nextAimPub = nh.advertise<nav_msgs::Odometry>("/next_aim", 1);
    m_nextAimPub = nh.advertise<geometry_msgs::PointStamped>("/next_aim", 1);

    trigger_sub_ = nh.subscribe("trigger", 1, &FrontierServer::triggerCallback, this);

    finish_sub = nh.subscribe("/finish_signal", 1, &FrontierServer::finishCallBack, this);

    state_str_ = { "OFF", "TOSTART", "RAPID", "CLASSIC", "GOHOME", "GOSAFEPOINT", "APPROACH", "FINISH" };
    m_currentState = ExplorationState::OFF;
    pub_log_.reset(new PubTicToc(1.5));

    last_check_time = std::chrono::high_resolution_clock::now();
    last_check_pos = Vector3d(0,0,0);


    MPG.reset(new MultiPoseGraph);
    MPG->main_inx = 0;

    last_pub_time = ros::Time::now();

    ros::Duration(1.0).sleep();

    frontier_timer_ = nh.createTimer(ros::Duration(0.05), &FrontierServer::runonce, this);

    // 多车交互相关
    mcar_FreeSpaceAndFrontierInfo_pub = nh.advertise<msg_utils::FreeSpaceAndFrontierInfo>("/mcar_FreeSpaceAndFrontierInfo", 1);
    mcar_FreeSpaceAndFrontierInfo_sub = nh.subscribe("/mcar_FreeSpaceAndFrontierInfo", 1, &FrontierServer::mcar_FreeSpaceAndFrontierInfoCallback, this);
    comm2others_timer = nh.createTimer(ros::Duration(2.0), &FrontierServer::comm2others_once, this);
}

void FrontierServer::comm2others_once(const ros::TimerEvent& /*event*/) {
    // 定期发布自车的信息
    // return;

    // if(m_currentState == ExplorationState::)

    if(fc_info_ptr_->svp_list.size() <= 0 || fc_info_ptr_->free_space_info.free_space_list.size() <= 0 || fc_info_ptr_->free_space_info.posegraph->key_pose_list->size() <= 0) 
    return;

    msg_utils::FreeSpaceAndFrontierInfo cur_exp_state;
    msg_utils::FreeSpaceInfo cur_free_space_info;
    msg_utils::FrontierInfo cur_frontier_info;

    // 自由空间相关消息填充
    for( vector<FreeSpace>::iterator it2 = fc_info_ptr_->free_space_info.free_space_list.begin(); it2 != fc_info_ptr_->free_space_info.free_space_list.end(); it2++) {
        msg_utils::FreeSpace cur_free_space;
        for( int i = 0; i < it2->meshes.size(); i++ ) {
            geometry_msgs::Point cur_mesh;
            cur_mesh.x = it2->meshes[i]._a.x;
            cur_mesh.y = it2->meshes[i]._a.y;
            cur_mesh.z = it2->meshes[i]._a.z;
            cur_free_space.convex_points.push_back(cur_mesh);
        }
        cur_free_space_info.free_space_list.push_back(cur_free_space);
        // msg_utils::Posegra cur_posegra;
        // for( int i = 0; i < it2->key_pose.size(); i++ ) {
        //     //
        // }

    }
    // cout << "comm1"  << endl;
    msg_utils::Posegra cur_posegra;
    for(int jj = 0; jj < fc_info_ptr_->free_space_info.posegraph->key_pose_list->size(); jj++) {
        geometry_msgs::Point cur_pose;
        cur_pose.x = (*fc_info_ptr_->free_space_info.posegraph->key_pose_list)[jj].x;
        cur_pose.y = (*fc_info_ptr_->free_space_info.posegraph->key_pose_list)[jj].y;
        cur_pose.z = (*fc_info_ptr_->free_space_info.posegraph->key_pose_list)[jj].z;
        cur_posegra.key_pose_list.push_back(cur_pose);
    }

    cur_free_space_info.posegraph.key_pose_list = cur_posegra.key_pose_list;
    // cout << "comm2"  << endl;
    msg_utils::EdgeTable cur_edge_table;
    for( int i = 0; i < fc_info_ptr_->free_space_info.posegraph->pose_edge.size(); i++ ) {
        msg_utils::Edge cur_edge;
        // cur_edge.dis = fc_info_ptr_->free_space_info.posegraph.pose_edge[i]
        cur_edge.dis = fc_info_ptr_->free_space_info.posegraph->pose_edge[i].begin()->weight;
        cur_edge.key_pose_inx_to = fc_info_ptr_->free_space_info.posegraph->pose_edge[i].begin()->v_inx;
        cur_edge.key_pose_inx_from = i;

        cur_edge_table.data.push_back(cur_edge);
    }
    // cout << "comm3"  << endl;
    cur_free_space_info.posegraph.edges_in_PG = cur_edge_table;

    // 前沿相关消息填充
    for( list<SuperViewPoint>::iterator it = fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++) {
        // 填充超级视点
        msg_utils::SuperViewPoint cur_svp;

        cur_svp.robot_id = fc_info_ptr_->car_id;

        cur_svp.super_viewpoin_id = it->id;

        geometry_msgs::Point cur_svp_viewpoint;
        cur_svp_viewpoint.x = it->super_vp.x;
        cur_svp_viewpoint.y = it->super_vp.y;
        cur_svp_viewpoint.z = it->super_vp.z;
        cur_svp.super_viewpoint = cur_svp_viewpoint;
        // cout << "comm3.1"  << endl;

        for(int i = 0; i < it->fc_list.size(); i++) {
            // cout << "comm_a"  << endl;
            // 填充视点
            msg_utils::ViewPoint cur_viewpoint;
            cur_viewpoint.viewpoint_id = it->fc_list[i].id;
            geometry_msgs::Point frontier_cluster;
            frontier_cluster.x = it->fc_list[i].center(0);
            frontier_cluster.y = it->fc_list[i].center(1);
            frontier_cluster.z = it->fc_list[i].center(2);
            cur_viewpoint.frontier_cluster = frontier_cluster;
            // cout << "comm_b"  << endl;
            for(int j = 0; j < it->fc_list[i].cell_list.size(); j++) {
                // cout << "comm_c"  << endl;
                geometry_msgs::Polygon cur_cell;
                cur_cell.points.resize(3);
                cur_cell.points[0].x = it->fc_list[i].cell_list[j]._a(0);
                cur_cell.points[0].y = it->fc_list[i].cell_list[j]._a(1);
                cur_cell.points[0].z = it->fc_list[i].cell_list[j]._a(2);
                cur_cell.points[1].x = it->fc_list[i].cell_list[j]._b(0);
                cur_cell.points[1].y = it->fc_list[i].cell_list[j]._b(1);
                cur_cell.points[1].z = it->fc_list[i].cell_list[j]._b(2);
                cur_cell.points[2].x = it->fc_list[i].cell_list[j]._c(0);
                cur_cell.points[2].y = it->fc_list[i].cell_list[j]._b(1);
                cur_cell.points[2].z = it->fc_list[i].cell_list[j]._b(2);
                cur_viewpoint.cell_list.push_back(cur_cell);
                // cout << "comm_d"  << endl;
            }

            cur_svp.viewpoints.push_back(cur_viewpoint);
        }
        // cout << "comm3.2"  << endl;

        cur_frontier_info.super_viewpoints.push_back(cur_svp);
    }
    // cout << "comm4"  << endl;
    // cur_exp_state.robot_id = fc_info_ptr_->car_id;
    cur_exp_state.from_id = fc_info_ptr_->car_id;
    cur_exp_state.free_space_info = cur_free_space_info;
    cur_exp_state.frontier_info = cur_frontier_info;
    mcar_FreeSpaceAndFrontierInfo_pub.publish(cur_exp_state);
}

void FrontierServer::mcar_FreeSpaceAndFrontierInfoCallback(const msg_utils::FreeSpaceAndFrontierInfoConstPtr msg) {
    // 接受其他车的探索情况，首先是排除自己的
    if(msg->from_id == fc_info_ptr_->car_id) return;

    // 接受并合并其他车辆的探索信息
    FreeSpaceInfo fs_info_tmp;
    SuperViewPointList svp_list_tmp;

    /*******消息解析*********************************************************************/
    // 获取临时超级视点信息
    for(int i = 0; i < msg->frontier_info.super_viewpoints.size(); i++) { 
        SuperViewPoint svp_tmp;
        
        svp_tmp.id = msg->frontier_info.super_viewpoints[i].super_viewpoin_id;
        svp_tmp.super_vp.x = msg->frontier_info.super_viewpoints[i].super_viewpoint.x;
        svp_tmp.super_vp.y = msg->frontier_info.super_viewpoints[i].super_viewpoint.y;
        svp_tmp.super_vp.z = msg->frontier_info.super_viewpoints[i].super_viewpoint.z;

        for(int j = 0; j < msg->frontier_info.super_viewpoints[i].viewpoints.size(); j++) {
            FrontierCluster fc_tmp;
            
            for(int k = 0; k < msg->frontier_info.super_viewpoints[i].viewpoints[j].cell_list.size(); k++) {
                geometry_msgs::Polygon poly_tmp;
                poly_tmp = msg->frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k];
                Triangle tri_tmp(poly_tmp);
                fc_tmp.cell_list.push_back(tri_tmp);
            }
            svp_tmp.fc_list.push_back(fc_tmp);
        }

        svp_list_tmp.push_back(svp_tmp);
    }

    // 获取临时自由空间缓存信息
    for(int i = 0; i < msg->free_space_info.free_space_list.size(); i++) {
        // 自由空间多面体获取
        FreeSpace fs_tmp;

        fs_info_tmp.free_space_list.push_back(fs_tmp);

        // 自由空间路径拓扑图获取
    }



    /*******消息合并*********************************************************************/
    


}

void FrontierServer::triggerCallback(const geometry_msgs::PoseStampedConstPtr msg) {
    if (m_currentState != ExplorationState::OFF ) {
        // ROS_WARN_STREAM("wait for triggered!!");   
        return;
    }
    trigger_ = true;
    ROS_WARN_STREAM("start exploter");

    last_pub_time = ros::Time(ros::Time::now().toSec() - 10.0);
    transitState(TOSTART);

    start_time = ros::Time::now().toSec();
}

void FrontierServer::finishCallBack(const std_msgs::Int8 msg)
{
    ROS_ERROR_STREAM("Get Finish Signal!");
    need_finish = true;
    return;
}


//! Note! This timer is only for single robot exploration FSM.
//! For multi-robot exploration, please comment this timer!
void FrontierServer::runonce(const ros::TimerEvent& /*event*/)
{    
    if (m_currentState == FINISH)
    {
        ROS_INFO_THROTTLE(1.0, "\033[1;31mFinish!\033[0m");
        return;
    }
    
    // cout<<"------------------------run once-----------------------------"<<endl;
    // ROS_INFO_STREAM("[frontier_finder] State: " << stateStr(m_currentState));
        

    if(m_currentState != GOHOME && m_currentState != FINISH) 
        fc_info_ptr_->updateFrontierSpace();

    // return;

    if(First_frame_cnt++ < 20 || fc_info_ptr_->free_space_fac_->grid_map_->recieve_cnt <= 10) {
        aim_pose = Vector3d(0,0,1);
        cout<<"aim_pose01: "<<aim_pose.transpose()<<endl;
        return;
    }

    fc_info_ptr_->free_space_info.posegraph->deepCopy(MPG->posegraphes[0], fc_info_ptr_->free_space_info.posegraph->getSize()-fc_info_ptr_->temp_num);

    double time1 = ros::Time::now().toSec();

    //! Result container
    Vector3d aim_p = Vector3d(aim_pose(0), aim_pose(1), aim_pose(2));
    double dis2goal;

    Vector3d drone_pos_ = fc_info_ptr_->drone_pose;
    vector<Vector3d> classic_aim;

    Vector3d safe_pose;

    ros::Time ros_time = ros::Time::now();

    Vector3d cur_pos = fc_info_ptr_->drone_pose;

    if(m_currentState != FINISH && m_currentState != TOSTART && m_currentState != GOHOME && m_currentState != OFF) {

        if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_check_time).count() > 10000) {
            if(fc_info_ptr_->svp_list.size() <= 0) return;

            // if((last_check_pos - cur_pos).norm() < 0.6 && !fc_info_ptr_->isVisible(cur_pos, Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z), fc_info_ptr_->grid_map_->getResolution())) {
            if((last_check_pos - cur_pos).norm() < 0.3) {
                ROS_ERROR_STREAM("[frontier_finder] Not wait_planner_succ!");

                for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
                {
                    if (best_svp.id == it->id && (Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z) - aim_pose).norm() < 1.0)
                    {
                        fc_info_ptr_->svp_list.erase(it);
                        ROS_ERROR_STREAM("[frontier_finder] erase it!");
                        break;
                    }

                }
            }
            last_check_time = std::chrono::high_resolution_clock::now();
            last_check_pos = cur_pos;
        }
    }

    switch (m_currentState)
    {

    case FINISH: {
        ROS_INFO_THROTTLE(1.0, "\033[1;31mFinish!\033[0m");
        break;
    }

    case TOSTART: {
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

    case CLASSIC: {
        // if(fc_info_ptr_->svp_list.size() <= 0) {
        //     ROS_ERROR("no svp, stop and find");
        //     break;
        // }
        ROS_INFO("\033[1;31mClassic Mode Active!\033[0m");  //红
        vector<Vector3d> path_res_tmp = path_res;
        path_res.clear();
        SuperViewPoint best_svp_last = best_svp;
        best_svp = classicFronitierFind(aim_pose, aim_vel); // will fill the path_res
            
        if(best_svp.id == -1){
            ROS_ERROR_STREAM("classFind Fail! cnt: " << classic_fail_cnt_);
            classic_fail_cnt_++;
            if (classic_fail_cnt_ < 20) {
                break;
            } 
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
        } else {
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

        }

        cout<<"classic find: "<<aim_pose.transpose()<<endl;
        classic_fail_cnt_ = 0;

        if(path_res.size() <= 2) {   // directly aim to the svp

            local_aim = aim_pose;
            aim_vel = calResVel(aim_pose, fc_info_ptr_->drone_pose);
            if (m_currentState == GOHOME) aim_vel = Vector3d(0, 0, 0);
            if (fc_info_ptr_->grid_map_->getInflateOccupancy(aim_pose)) {
                ROS_INFO("\033[1;31m[frontier_finder] Aim Occ, Replan!\033[0m");  //红
                ROS_INFO_STREAM("aim occ: " << aim_pose.transpose());
                transitState(CLASSIC);
                break;
            }
            publishNextAim(ros_time, aim_pose, aim_vel);
            cout << "pub aim: " << aim_pose.transpose() << endl;
            dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
            last_pub_time = ros_time;
        } else {
            path_inx = 1;
            local_aim = getLocalAim();
            local_vel = Vector3d::Zero();
            if (fc_info_ptr_->grid_map_->getInflateOccupancy(local_aim)) {
                ROS_INFO("\033[1;31m[frontier_finder] Aim Occ, Replan!\033[0m");  //红
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

    
    case GOHOME: {
        double dis_2_home = (fc_info_ptr_->drone_pose - home_position_).head(2).norm(); 
        // if (pub_log_->canPub()) 
            // cout << "dis_2_home: "<< dis_2_home << endl;

        // if (pub_log_->canPub()) 
            // ROS_INFO("\033[1;31mGo Home!\033[0m");  //红

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


    case APPROACH: {
        dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
        if (pub_log_->canPub()) ROS_INFO("\033[1;33mApproach...\033[0m");  //黄
        double dis2goal2d = sqrt(pow(aim_pose(0)-fc_info_ptr_->drone_pose(0),2) + pow(aim_pose(1)-fc_info_ptr_->drone_pose(1),2));
        double dis2localaim = (local_aim - fc_info_ptr_->drone_pose).norm();

        if (pub_log_->canPub()) {
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
        } else {
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
            ROS_INFO("\033[1;31m[frontier_finder] Ftr Change, Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }
        if ((ros_time.toSec() - last_pub_time.toSec()) > classic_pub_interval){
            ROS_INFO("\033[1;31m[frontier_finder] Time to Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }

        if (dis2goal2d < arrive_goal_dis_thres_){ //* arrive classic goal
            ROS_INFO_STREAM("aim_pose: " << aim_pose.transpose() << ", drone_pose: " << fc_info_ptr_->drone_pose.transpose());
            ROS_INFO_STREAM("[frontier_finder] dis2goal2d: " << dis2goal2d << ", arrive_dis_thres: " << arrive_goal_dis_thres_);
            ROS_INFO("\033[1;31mGet Classic Aim, Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }

        if (fc_info_ptr_->grid_map_->getInflateOccupancy(local_aim))
        {
            ROS_INFO("\033[1;31m[frontier_finder] Aim Occ, Replan!\033[0m");  //红
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


    }
    
    double time2 = ros::Time::now().toSec();
    if ((m_currentState != APPROACH && m_currentState != OFF && m_currentState != GOHOME ) || 
        ((m_currentState == APPROACH || m_currentState == OFF || m_currentState == GOHOME) && pub_log_->canPub()) )
        ROS_INFO("\033[1;34m ALL Done - %f ms\033[0m", (time2-time1) * 1000.0);  //蓝
    if ((m_currentState == APPROACH || m_currentState == OFF || m_currentState == GOHOME) && pub_log_->canPub())
        pub_log_->tic();
}

Vector3d FrontierServer::getLocalAim(){
    for(int i = path_res.size()-1; i > path_inx; i--) {
        if(fc_info_ptr_->grid_map_->isInMap(path_res[i]) && 
            fc_info_ptr_->isVisible(fc_info_ptr_->drone_pose, path_res[i], fc_info_ptr_->grid_map_->getResolution()) > 0) {
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
            score = 100.0 - dis_2_home;  // BB
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
         (drone_p-home_p).norm()<fc_info_ptr_->free_space_fac_->sensor_range) {
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
    // nav_msgs::Odometry odom;
    // odom.header.stamp = rostime;
    // odom.header.frame_id = "world";

    // odom.pose.pose.position.x = aim_pose(0);
    // odom.pose.pose.position.y = aim_pose(1);
    // odom.pose.pose.position.z = aim_pose(2);
    // // odom.pose.pose.position.z = 1e-3;


    // odom.twist.twist.linear.x = aim_vel(0);
    // odom.twist.twist.linear.y = aim_vel(1);
    // odom.twist.twist.linear.z = aim_vel(2);
    // // odom.twist.twist.linear.z = 0.0;

    // if(is_on_car){
    //     odom.pose.pose.position.z = fc_info_ptr_->car_odom_z;
    //     odom.twist.twist.linear.z = 0.0;
    // }

    geometry_msgs::PointStamped odom;
    odom.header.stamp = rostime;
    odom.header.frame_id = "map";

    odom.point.x = aim_pose(0);
    odom.point.y = aim_pose(1);
    odom.point.z = aim_pose(2);


    if(is_on_car){
        odom.point.z = fc_info_ptr_->car_odom_z;
    }


    if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(Vector3d(aim_pose(0),aim_pose(1),aim_pose(2)))){
        ROS_ERROR_STREAM("Frontier is OCC! "<<aim_pose.transpose());
        return;
    }


    m_nextAimPub.publish(odom);
}




